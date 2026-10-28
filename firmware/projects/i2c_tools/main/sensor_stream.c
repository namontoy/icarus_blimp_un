/*
 * sensor_stream.c - samples every sensor on the bench at a fixed rate and
 * keeps the latest reading for the ground link, which sends it as
 * LINK_PKT_SENSORS (link_protocol.h, net_link.c).
 *
 * The drivers are the ones behind the console commands (cmd_imu.c, cmd_mag.c,
 * cmd_baro.c, cmd_tof.c): same attach, same configuration, same scaling, so
 * what the ground station plots is what 'imu -n 3' prints, and the bench
 * commands stay the regression test for the stream.
 *
 * Sharing the bus. The i2c_master driver serialises transactions, so this
 * task and a console command can both talk on the bus. The chips are the
 * problem, not the bus: every console command resets and reconfigures its
 * sensor, the VL53L1X port refuses a second attach at the same address, and
 * 'i2cconfig' cannot delete a bus that still has devices on it. So the rule
 * is a lease. A console command that touches a sensor calls
 * sensor_stream_pause(), which waits for the current cycle to end and
 * releases every chip the stream holds; the command runs; then
 * sensor_stream_resume(). The stream re-attaches on its next cycle,
 * reconfiguring from scratch, so whatever the command left in the chip does
 * not leak into the numbers. The ground station sees a gap of the command's
 * length plus the re-attach, which is the honest picture.
 *
 * Rates. The IMU, the magnetometer and the ToF are polled every cycle
 * (CONFIG_SENSOR_STREAM_HZ). The magnetometer and the ToF only yield when
 * their data-ready bit says a new measurement exists, so their values repeat
 * across packets at the chip's own rate (15 or 50 Hz and 10 Hz). The
 * barometer is one forced conversion every BARO_PERIOD_MS: a conversion
 * blocks this task for about 20 ms, so it is not run every cycle.
 *
 * Absent or failing sensors. Each sensor is attached on its own. One that is
 * missing is retried every ATTACH_RETRY_MS, quietly (the drivers probe the
 * address before logging anything), so a breakout plugged in later shows
 * up. One whose reads fail FAIL_LIMIT times in a row, or that stops
 * producing for STALE_MS, is released and retried the same way. A sensor's
 * valid bit in the packet is set only while its last good reading is
 * younger than STALE_MS, so an absent sensor is never plotted as zero.
 *
 * Two readers. The link takes the SENSORS payload (sensor_stream_latest);
 * the control task's read stage takes the same sample as the pipeline's raw
 * record (sensor_stream_latest_raw): the counts behind the floats, with the
 * time each source last produced a reading, so the task can tell a new
 * sample from a repeated one. Both copies are published together, under the
 * same spinlock, at the end of every cycle. This is the interim arrangement
 * until increment 2 of docs/control-loop-plan.md moves the drivers into the
 * control task and retires this file.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "cmd_baro.h"
#include "cmd_imu.h"
#include "cmd_mag.h"
#include "cmd_tof.h"
#include "link_protocol.h"
#include "sensor_stream.h"

static const char *TAG = "sensors";

#define STREAM_TASK_STACK   4096
#define STREAM_TASK_PRIO    3       /* above the consoles (2), below the link (4) */

/* One forced barometer conversion blocks about 20 ms at this preset (P x4,
   T x1 on a BMx280; oss 1 on a BMP180), so it runs on its own schedule. */
#define BARO_PERIOD_MS      250
#define BARO_OSS            1

/* The bench default for the rangefinder: budgets below 100 ms were measured
   reading about 5 % short (README section 8). 10 frames a second. */
#define TOF_BUDGET_MS       100
#define TOF_INTER_MS        100

#define ATTACH_RETRY_MS     2000
#define FAIL_LIMIT          3
#define STALE_MS            1000

typedef struct {
    const char *name;
    esp_err_t (*open)(void);
    void      (*close)(void);
    char      desc[64];         /* chip, address and settings, once attached */
    bool      attached;
    bool      released;         /* closed after failures, not merely absent  */
    int64_t   last_ok_us;
    int64_t   next_try_us;
    int       fails;            /* consecutive                               */
    uint32_t  samples;
    uint32_t  errors;
    uint32_t  skipped;          /* readings refused: magnetometer overflow   */
} chan_t;

enum { CH_IMU, CH_MAG, CH_BARO, CH_TOF, CH_COUNT };

static struct {
    SemaphoreHandle_t lease;
    TaskHandle_t      task;
    volatile bool     enabled;
    portMUX_TYPE      mux;          /* guards latest, raw_latest, scales and seq */
    link_sensors_t    latest;       /* payload only: header and CRC are zero */
    sensors_raw_t     raw;          /* the working copy, under the lease     */
    sensors_raw_t     raw_latest;   /* published with latest                 */
    sensor_scales_t   scales;
    uint32_t          seq;
    uint32_t          cycles;
    int64_t           next_baro_us;
    float             alt0_m;       /* first barometric altitude since boot  */
    bool              alt0_set;
    imu_dev_t        *imu;
    mag_dev_t        *mag;
    baro_dev_t       *baro;
    tof_dev_t        *tof;
    imu_reading_t     imu_r;
    mag_reading_t     mag_r;
    baro_reading_t    baro_r;
    tof_reading_t     tof_r;
} s = { .mux = portMUX_INITIALIZER_UNLOCKED };

static esp_err_t open_imu(void);
static esp_err_t open_mag(void);
static esp_err_t open_baro(void);
static esp_err_t open_tof(void);
static void close_imu(void);
static void close_mag(void);
static void close_baro(void);
static void close_tof(void);

static chan_t s_ch[CH_COUNT] = {
    [CH_IMU]  = { .name = "imu",  .open = open_imu,  .close = close_imu  },
    [CH_MAG]  = { .name = "mag",  .open = open_mag,  .close = close_mag  },
    [CH_BARO] = { .name = "baro", .open = open_baro, .close = close_baro },
    [CH_TOF]  = { .name = "tof",  .open = open_tof,  .close = close_tof  },
};

/* ---------------------------------------------------------------- drivers */

static esp_err_t open_imu(void)
{
    esp_err_t err = imu_attach(-1, 0, 0, &s.imu);
    if (err == ESP_OK) {
        int accel_g = 0, gyro_dps = 0;
        imu_ranges(s.imu, &accel_g, &gyro_dps);
        snprintf(s_ch[CH_IMU].desc, sizeof(s_ch[CH_IMU].desc),
                 "MPU-6050 @ 0x%02x   +/-%d g, +/-%d dps, DLPF %d Hz", imu_addr(s.imu), accel_g,
                 gyro_dps, imu_dlpf_hz(s.imu));
        portENTER_CRITICAL(&s.mux);
        s.scales.imu_accel_g  = accel_g;
        s.scales.imu_gyro_dps = gyro_dps;
        portEXIT_CRITICAL(&s.mux);
    }
    return err;
}

static void close_imu(void)
{
    imu_detach(s.imu);
    s.imu = NULL;
}

static esp_err_t open_mag(void)
{
    esp_err_t err = mag_attach(-1, &s.mag);
    if (err == ESP_OK) {
        mag_info_t i;
        mag_info(s.mag, &i);
        snprintf(s_ch[CH_MAG].desc, sizeof(s_ch[CH_MAG].desc),
                 "%s @ 0x%02x   +/-%.1f G, %.0f LSB/G", i.name, i.addr, i.range_gauss,
                 i.lsb_per_gauss);
        portENTER_CRITICAL(&s.mux);
        s.scales.mag_lsb_per_gauss = i.lsb_per_gauss;
        s.scales.mag_chip          = 1 + (int)i.chip;
        portEXIT_CRITICAL(&s.mux);
    }
    return err;
}

static void close_mag(void)
{
    mag_detach(s.mag);
    s.mag = NULL;
}

static esp_err_t open_baro(void)
{
    esp_err_t err = baro_attach(-1, BARO_OSS, &s.baro);
    if (err == ESP_OK) {
        baro_info_t i;
        baro_info(s.baro, &i);
        portENTER_CRITICAL(&s.mux);
        s.scales.baro_chip = 1 + (int)i.chip;
        portEXIT_CRITICAL(&s.mux);
        if (i.chip == BARO_CHIP_BMP180) {
            snprintf(s_ch[CH_BARO].desc, sizeof(s_ch[CH_BARO].desc),
                     "%s @ 0x%02x   oversampling %d, %d ms", i.name, i.addr, i.oss, i.conv_ms);
        } else {
            snprintf(s_ch[CH_BARO].desc, sizeof(s_ch[CH_BARO].desc),
                     "%s @ 0x%02x   P x%d, T x%d, %d ms", i.name, i.addr, i.osrs_p, i.osrs_t,
                     i.conv_ms);
        }
    }
    return err;
}

static void close_baro(void)
{
    baro_detach(s.baro);
    s.baro = NULL;
}

static esp_err_t open_tof(void)
{
    esp_err_t err = tof_attach(-1, TOF_MODE_LONG, TOF_BUDGET_MS, TOF_INTER_MS, &s.tof);
    if (err == ESP_OK) {
        tof_info_t i;
        tof_info(s.tof, &i);
        snprintf(s_ch[CH_TOF].desc, sizeof(s_ch[CH_TOF].desc),
                 "VL53L1X @ 0x%02x   long mode, budget %d ms, period %d ms", i.addr,
                 i.budget_ms, i.inter_ms);
    }
    return err;
}

static void close_tof(void)
{
    tof_detach(s.tof);
    s.tof = NULL;
}

/* ------------------------------------------------------------- bookkeeping */

static void release_all(void)
{
    for (int i = 0; i < CH_COUNT; i++) {
        if (s_ch[i].attached) {
            s_ch[i].close();
            s_ch[i].attached = false;
        }
    }
}

static void try_attach(int64_t now)
{
    for (int i = 0; i < CH_COUNT; i++) {
        chan_t *c = &s_ch[i];
        if (c->attached || now < c->next_try_us) {
            continue;
        }
        esp_err_t err = c->open();
        if (err == ESP_OK) {
            c->attached   = true;
            c->released   = false;
            c->fails      = 0;
            c->last_ok_us = now;
            ESP_LOGI(TAG, "%s: %s", c->name, c->desc);
        } else {
            /* ESP_ERR_NOT_FOUND is the quiet case: nothing at the address.
               Anything else was logged by the driver. */
            c->next_try_us = now + (int64_t)ATTACH_RETRY_MS * 1000;
        }
    }
}

static void note(chan_t *c, esp_err_t err, int64_t now)
{
    if (err == ESP_OK) {
        c->fails = 0;
        c->last_ok_us = now;
        c->samples++;
        return;
    }
    c->errors++;
    if (++c->fails < FAIL_LIMIT) {
        return;
    }
    ESP_LOGW(TAG, "%s: %d reads failed in a row (%s) - released, retrying every %d ms",
             c->name, c->fails, esp_err_to_name(err), ATTACH_RETRY_MS);
    c->close();
    c->attached    = false;
    c->released    = true;
    c->fails       = 0;
    c->next_try_us = now + (int64_t)ATTACH_RETRY_MS * 1000;
}

/* The chip answered but had nothing new. Its own rate explains a few cycles
   of that; a second of it means its measurement loop stalled, which note()
   turns into a release and a fresh attach. */
static void note_nothing_new(chan_t *c, int64_t now)
{
    if (now - c->last_ok_us > (int64_t)STALE_MS * 1000) {
        note(c, ESP_ERR_TIMEOUT, now);
    }
}

static bool fresh(const chan_t *c, int64_t now)
{
    return c->attached && (now - c->last_ok_us) < (int64_t)STALE_MS * 1000;
}

static void publish(int64_t now)
{
    link_sensors_t p;
    memset(&p, 0, sizeof(p));
    p.t_us = (uint32_t)now;

    if (fresh(&s_ch[CH_IMU], now)) {
        p.valid |= LINK_SENSORS_IMU;
        p.ax_g = s.imu_r.ax_g;
        p.ay_g = s.imu_r.ay_g;
        p.az_g = s.imu_r.az_g;
        p.gx_dps = s.imu_r.gx_dps;
        p.gy_dps = s.imu_r.gy_dps;
        p.gz_dps = s.imu_r.gz_dps;
        p.imu_temp_c = s.imu_r.temp_c;
    }
    if (fresh(&s_ch[CH_MAG], now)) {
        p.valid |= LINK_SENSORS_MAG;
        p.mx_gauss = s.mag_r.mx_gauss;
        p.my_gauss = s.mag_r.my_gauss;
        p.mz_gauss = s.mag_r.mz_gauss;
        p.heading_deg = s.mag_r.heading_deg;
    }
    if (fresh(&s_ch[CH_BARO], now)) {
        p.valid |= LINK_SENSORS_BARO;
        p.pressure_pa = s.baro_r.pressure_pa;
        p.baro_temp_c = s.baro_r.temp_c;
        p.baro_alt_m  = s.baro_r.altitude_m - s.alt0_m;
    }
    if (fresh(&s_ch[CH_TOF], now)) {
        p.valid |= LINK_SENSORS_TOF;
        p.tof_mm     = s.tof_r.distance_mm;
        p.tof_status = s.tof_r.status;
    }

    s.raw.t_us = (uint32_t)now;
    portENTER_CRITICAL(&s.mux);
    s.latest     = p;
    s.raw_latest = s.raw;
    s.seq++;
    portEXIT_CRITICAL(&s.mux);
}

/* ------------------------------------------------------------------ cycle */

static void cycle(int64_t now)
{
    try_attach(now);

    chan_t *c = &s_ch[CH_IMU];
    if (c->attached) {
        imu_reading_t r;
        esp_err_t err = imu_read(s.imu, &r);
        if (err == ESP_OK) {
            s.imu_r = r;
            memcpy(s.raw.acc, r.raw_acc, sizeof(s.raw.acc));
            memcpy(s.raw.gyro, r.raw_gyro, sizeof(s.raw.gyro));
            s.raw.imu_temp = r.raw_temp;
            s.raw.imu_t_us = (uint32_t)now;
        }
        note(c, err, now);
    }

    c = &s_ch[CH_MAG];
    if (c->attached) {
        mag_reading_t r;
        esp_err_t err = mag_read(s.mag, 0, &r);
        if (err == ESP_ERR_TIMEOUT) {
            note_nothing_new(c, now);
        } else if (err == ESP_OK && r.overflow) {
            /* Clipped, not a measurement: keep the last good one. If it goes
               on, the valid bit expires with it; re-attaching would not help
               against a magnet. */
            c->skipped++;
        } else {
            if (err == ESP_OK) {
                s.mag_r = r;
                memcpy(s.raw.mag, r.raw, sizeof(s.raw.mag));
                s.raw.mag_t_us = (uint32_t)now;
            }
            note(c, err, now);
        }
    }

    c = &s_ch[CH_TOF];
    if (c->attached) {
        tof_reading_t r;
        esp_err_t err = tof_read(s.tof, 0, &r);
        if (err == ESP_ERR_TIMEOUT) {
            note_nothing_new(c, now);
        } else {
            /* A frame with a non-zero status is still a frame: the packet
               carries the status and the page decides what to show. */
            if (err == ESP_OK) {
                s.tof_r = r;
                s.raw.tof_mm     = r.distance_mm;
                s.raw.tof_status = r.status;
                s.raw.tof_t_us   = (uint32_t)now;
            }
            note(c, err, now);
        }
    }

    c = &s_ch[CH_BARO];
    if (c->attached && now >= s.next_baro_us) {
        s.next_baro_us = now + (int64_t)BARO_PERIOD_MS * 1000;
        baro_reading_t r;
        esp_err_t err = baro_read(s.baro, &r);
        if (err == ESP_OK) {
            s.baro_r = r;
            if (!s.alt0_set) {
                s.alt0_m   = r.altitude_m;
                s.alt0_set = true;
            }
            /* Fixed point for the record: 0.1 Pa, 0.01 C, 0.01 %RH. */
            s.raw.baro_p_dpa  = (int32_t)lroundf(r.pressure_pa * 10.0f);
            s.raw.baro_t_cc   = (int32_t)lroundf(r.temp_c * 100.0f);
            s.raw.baro_h_cpct = r.has_humidity ? (uint16_t)lroundf(r.humidity_pct * 100.0f) : 0;
            s.raw.baro_t_us   = (uint32_t)now;
        }
        note(c, err, now);
    }

    s.cycles++;
    publish(now);
}

static void stream_task(void *arg)
{
    (void)arg;
    TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_SENSOR_STREAM_HZ);
    if (period < 1) {
        period = 1;
    }
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        if (!s.enabled) {
            vTaskDelay(pdMS_TO_TICKS(100));
            last = xTaskGetTickCount();
            continue;
        }
        xSemaphoreTake(s.lease, portMAX_DELAY);
        if (s.enabled) {
            cycle(esp_timer_get_time());
        }
        xSemaphoreGive(s.lease);

        /* After a long wait on the lease (a console command held it), or a
           slow attach, resync instead of bursting through the missed
           periods. */
        TickType_t now_t = xTaskGetTickCount();
        if ((TickType_t)(now_t - last) > period) {
            last = now_t;
        }
        vTaskDelayUntil(&last, period);
    }
}

/* ------------------------------------------------------------------- API */

esp_err_t sensor_stream_start(bool enabled)
{
    if (s.task != NULL) {
        s.enabled = enabled;
        return ESP_OK;
    }
    s.lease = xSemaphoreCreateMutex();
    if (s.lease == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s.enabled = enabled;
    if (xTaskCreate(stream_task, "sensors", STREAM_TASK_STACK, NULL, STREAM_TASK_PRIO,
                    &s.task) != pdPASS) {
        vSemaphoreDelete(s.lease);
        s.lease = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "sensor stream %s: %d Hz, barometer every %d ms",
             enabled ? "on" : "off until 'sensors --on'", CONFIG_SENSOR_STREAM_HZ,
             BARO_PERIOD_MS);
    return ESP_OK;
}

void sensor_stream_pause(void)
{
    if (s.lease == NULL) {
        return;
    }
    xSemaphoreTake(s.lease, portMAX_DELAY);
    release_all();
}

void sensor_stream_resume(void)
{
    if (s.lease == NULL) {
        return;
    }
    /* Re-attach on the next cycle, with no retry wait. */
    for (int i = 0; i < CH_COUNT; i++) {
        s_ch[i].next_try_us = 0;
    }
    xSemaphoreGive(s.lease);
}

bool sensor_stream_latest(link_sensors_t *out, uint32_t *seq)
{
    portENTER_CRITICAL(&s.mux);
    *out = s.latest;
    *seq = s.seq;
    portEXIT_CRITICAL(&s.mux);
    return *seq != 0;
}

bool sensor_stream_latest_raw(sensors_raw_t *out)
{
    portENTER_CRITICAL(&s.mux);
    *out = s.raw_latest;
    const uint32_t seq = s.seq;
    portEXIT_CRITICAL(&s.mux);
    return seq != 0;
}

void sensor_stream_scales(sensor_scales_t *out)
{
    portENTER_CRITICAL(&s.mux);
    *out = s.scales;
    portEXIT_CRITICAL(&s.mux);
}

/* --------------------------------------------------------------- console */

static void print_status(void)
{
    /* Snapshot under the lease so the lines agree with each other; the wait
       is at most one sampling cycle. */
    chan_t ch[CH_COUNT];
    imu_reading_t imu;
    mag_reading_t mag;
    baro_reading_t baro;
    tof_reading_t tof;
    float alt0;
    uint32_t cycles, seq;

    xSemaphoreTake(s.lease, portMAX_DELAY);
    memcpy(ch, s_ch, sizeof(ch));
    imu    = s.imu_r;
    mag    = s.mag_r;
    baro   = s.baro_r;
    tof    = s.tof_r;
    alt0   = s.alt0_m;
    cycles = s.cycles;
    seq    = s.seq;
    xSemaphoreGive(s.lease);
    const int64_t now = esp_timer_get_time();

    if (!s.enabled) {
        printf("sensor stream OFF - 'sensors --on' starts it\n");
    } else {
        printf("sensor stream ON   %d Hz, barometer every %d ms   cycles %" PRIu32
               "   packet seq %" PRIu32 "\n", CONFIG_SENSOR_STREAM_HZ, BARO_PERIOD_MS,
               cycles, seq);
    }

    for (int i = 0; i < CH_COUNT; i++) {
        const chan_t *c = &ch[i];
        if (!c->attached) {
            if (!s.enabled) {
                printf("  %-5s released (stream off)\n", c->name);
            } else if (c->released) {
                printf("  %-5s released after %d failed reads (%" PRIu32 " errors so far) - "
                       "retrying every %d ms\n", c->name, FAIL_LIMIT, c->errors,
                       ATTACH_RETRY_MS);
            } else {
                printf("  %-5s not found - probing again every %d ms\n", c->name,
                       ATTACH_RETRY_MS);
            }
            continue;
        }
        printf("  %-5s %s\n", c->name, c->desc);
        printf("        %s   %" PRIu32 " samples, %" PRIu32 " errors",
               fresh(c, now) ? "streaming" : "STALE", c->samples, c->errors);
        if (c->skipped) {
            printf(", %" PRIu32 " overflow", c->skipped);
        }
        printf("   last %lld ms ago\n", (long long)((now - c->last_ok_us) / 1000));

        switch (i) {
        case CH_IMU:
            printf("        acc %7.3f %7.3f %7.3f g   gyro %7.2f %7.2f %7.2f dps   %.1f C\n",
                   imu.ax_g, imu.ay_g, imu.az_g, imu.gx_dps, imu.gy_dps, imu.gz_dps,
                   imu.temp_c);
            break;
        case CH_MAG:
            printf("        field %7.3f %7.3f %7.3f G   |m| %.3f G   heading %.1f deg\n",
                   mag.mx_gauss, mag.my_gauss, mag.mz_gauss,
                   sqrtf(mag.mx_gauss * mag.mx_gauss + mag.my_gauss * mag.my_gauss +
                         mag.mz_gauss * mag.mz_gauss), mag.heading_deg);
            break;
        case CH_BARO:
            printf("        %.2f C   %.2f hPa   %.1f m standard atmosphere, %+.2f m since "
                   "the first reading\n", baro.temp_c, baro.pressure_pa / 100.0f,
                   baro.altitude_m, baro.altitude_m - alt0);
            break;
        case CH_TOF:
            printf("        %u mm   status %u (%s)   signal %u  ambient %u kcps/SPAD   "
                   "%u SPADs\n", tof.distance_mm, tof.status, tof_status_text(tof.status),
                   tof.signal_kcps_spad, tof.ambient_kcps_spad, tof.spads);
            break;
        }
    }
}

static struct {
    struct arg_lit *on;
    struct arg_lit *off;
    struct arg_end *end;
} sensors_args;

static int do_sensors_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&sensors_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, sensors_args.end, argv[0]);
        return 1;
    }
    if (s.task == NULL) {
        printf("sensor stream not started - see the boot log for why\n");
        return 1;
    }
    if (sensors_args.on->count && sensors_args.off->count) {
        ESP_LOGE(TAG, "--on and --off together make no sense");
        return 1;
    }
    if (sensors_args.off->count) {
        sensor_stream_pause();
        s.enabled = false;
        sensor_stream_resume();
        printf("sensor stream off: every sensor released\n");
        return 0;
    }
    if (sensors_args.on->count) {
        for (int i = 0; i < CH_COUNT; i++) {
            s_ch[i].next_try_us = 0;
        }
        s.enabled = true;
        printf("sensor stream on\n");
        return 0;
    }
    print_status();
    return 0;
}

void register_sensors(void)
{
    sensors_args.on  = arg_lit0(NULL, "on", "Start sampling (the default at boot)");
    sensors_args.off = arg_lit0(NULL, "off", "Stop sampling and release every sensor");
    sensors_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "sensors",
        .help = "The sensor stream behind the ground station: what it found, "
                "its latest readings, or --on/--off",
        .hint = NULL,
        .func = &do_sensors_cmd,
        .argtable = &sensors_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
