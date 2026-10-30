/*
 * capture.c - the control loop's log records, recorded in PSRAM for a bench
 * session and read out afterwards. Design in capture.h.
 *
 * Two parties touch the buffer and they never wait for each other:
 *
 *   - the control task on core 1 pushes one record per tick. It reserves a
 *     slot under the spinlock, copies outside it, then publishes the new
 *     count under the spinlock again, only if the capture id it reserved
 *     against is still the current one. A start or a clear in between bumps
 *     the id, and the late record is simply not counted.
 *   - the link task and the console on core 0 read records below `count`.
 *     Those are never rewritten while the id holds, so a read is a plain
 *     memcpy bracketed by two looks at the id.
 *
 * The buffer is allocated once at boot for the longest capture the Kconfig
 * allows, rather than per start: a failed allocation then shows up in the
 * boot log, not in the middle of a test, and the PSRAM heap never fragments.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "capture.h"
#include "ctl_task.h"
#include "link_protocol.h"
#include "sensor_stream.h"

static const char *TAG = "capture";

#define TICKS_PER_SECOND    (1000000u / CTL_PERIOD_US)
#define FALLBACK_SECONDS    10          /* internal RAM, when PSRAM is missing */

_Static_assert(sizeof(log_record_t) == LINK_LOG_RECORD_BYTES,
               "the link's chunk size assumes the record size in ctl_types.h");

static struct {
    portMUX_TYPE    mux;
    log_record_t   *buf;
    uint32_t        capacity;
    bool            in_psram;
    uint8_t         state;
    uint16_t        capture_id;
    uint32_t        count;
    uint32_t        wanted;
    uint32_t        t_start_us;
    uint32_t        dropped;
    sensor_scales_t scales;
} s = { .mux = portMUX_INITIALIZER_UNLOCKED, .state = LINK_LOG_IDLE };

/* ------------------------------------------------------------------- API */

esp_err_t capture_init(void)
{
    if (s.buf != NULL) {
        return ESP_OK;
    }
    uint32_t capacity = (uint32_t)CONFIG_CAPTURE_MAX_SECONDS * TICKS_PER_SECOND;
    log_record_t *buf = heap_caps_malloc(capacity * sizeof(log_record_t), MALLOC_CAP_SPIRAM);
    bool in_psram = buf != NULL;
    if (buf == NULL) {
        /* Not that much PSRAM free: take the largest block there is, in
           whole seconds, rather than give up the PSRAM altogether. */
        const unsigned asked_kb = (unsigned)(capacity * sizeof(log_record_t) / 1024);
        const size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
        const uint32_t fit = (uint32_t)(largest / sizeof(log_record_t)) / TICKS_PER_SECOND *
                             TICKS_PER_SECOND;
        if (fit >= TICKS_PER_SECOND) {
            buf = heap_caps_malloc(fit * sizeof(log_record_t), MALLOC_CAP_SPIRAM);
        }
        if (buf != NULL) {
            capacity = fit;
            in_psram = true;
            ESP_LOGW(TAG, "PSRAM holds %u s of records, not the %d s asked (%u KB)",
                     (unsigned)(capacity / TICKS_PER_SECOND), CONFIG_CAPTURE_MAX_SECONDS, asked_kb);
        } else {
            /* No PSRAM: keep a short capture possible so the module can
               still be exercised, and say so. */
            capacity = FALLBACK_SECONDS * TICKS_PER_SECOND;
            buf = heap_caps_malloc(capacity * sizeof(log_record_t), MALLOC_CAP_8BIT);
            if (buf == NULL) {
                ESP_LOGE(TAG, "no buffer: %u KB of PSRAM not available and no %u KB of internal "
                         "RAM either", asked_kb, (unsigned)(capacity * sizeof(log_record_t) / 1024));
                return ESP_ERR_NO_MEM;
            }
            ESP_LOGW(TAG, "PSRAM not available: %u s buffer in internal RAM instead of %d s",
                     (unsigned)FALLBACK_SECONDS, CONFIG_CAPTURE_MAX_SECONDS);
        }
    }
    s.buf      = buf;
    s.capacity = capacity;
    s.in_psram = in_psram;
    ESP_LOGI(TAG, "buffer %u KB in %s: up to %u s of %u-byte records at %u Hz",
             (unsigned)(capacity * sizeof(log_record_t) / 1024), in_psram ? "PSRAM" : "internal RAM",
             (unsigned)(capacity / TICKS_PER_SECOND), (unsigned)sizeof(log_record_t),
             (unsigned)TICKS_PER_SECOND);
    return ESP_OK;
}

esp_err_t capture_start(uint32_t seconds)
{
    if (s.buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (seconds == 0) {
        seconds = CAPTURE_DEFAULT_SECONDS;
    }
    uint32_t wanted = seconds * TICKS_PER_SECOND;
    if (wanted > s.capacity) {
        /* Refused, not shortened: a capture that ends before the time the
           operator asked for looks like a progress bug (it did, 2026-10-28). */
        return ESP_ERR_INVALID_SIZE;
    }
    /* The scales are read before the lock: sensor_stream_scales() takes its
       own spinlock, and the numbers describe the chips, not this capture. */
    sensor_scales_t sc;
    sensor_stream_scales(&sc);

    portENTER_CRITICAL(&s.mux);
    if (s.state == LINK_LOG_RUNNING) {
        portEXIT_CRITICAL(&s.mux);
        return ESP_ERR_INVALID_STATE;
    }
    s.capture_id++;
    s.count      = 0;
    s.wanted     = wanted;
    s.t_start_us = 0;
    s.dropped    = 0;
    s.scales     = sc;
    s.state      = LINK_LOG_RUNNING;
    portEXIT_CRITICAL(&s.mux);
    ESP_LOGI(TAG, "capture %u: %u s (%" PRIu32 " records) started", (unsigned)s.capture_id,
             (unsigned)(wanted / TICKS_PER_SECOND), wanted);
    return ESP_OK;
}

bool capture_stop(void)
{
    portENTER_CRITICAL(&s.mux);
    const bool was_running = s.state == LINK_LOG_RUNNING;
    if (was_running) {
        s.state = s.count ? LINK_LOG_DONE : LINK_LOG_IDLE;
    }
    const uint32_t count = s.count;
    portEXIT_CRITICAL(&s.mux);
    if (was_running) {
        ESP_LOGI(TAG, "capture %u stopped at %" PRIu32 " records", (unsigned)s.capture_id, count);
    }
    return was_running;
}

void capture_clear(void)
{
    portENTER_CRITICAL(&s.mux);
    s.capture_id++;
    s.count  = 0;
    s.wanted = 0;
    s.state  = LINK_LOG_IDLE;
    portEXIT_CRITICAL(&s.mux);
}

void capture_push(const log_record_t *r)
{
    /* Reserve. */
    portENTER_CRITICAL(&s.mux);
    if (s.state != LINK_LOG_RUNNING) {
        portEXIT_CRITICAL(&s.mux);
        return;
    }
    const uint32_t i  = s.count;
    const uint16_t id = s.capture_id;
    if (i >= s.wanted) {
        s.state = LINK_LOG_DONE;
        portEXIT_CRITICAL(&s.mux);
        return;
    }
    portEXIT_CRITICAL(&s.mux);

    /* Copy, with nothing held: PSRAM through the cache, a few microseconds. */
    s.buf[i] = *r;

    /* Publish, unless the capture changed under us. */
    portENTER_CRITICAL(&s.mux);
    if (s.capture_id == id && s.state == LINK_LOG_RUNNING && s.count == i) {
        if (i == 0) {
            s.t_start_us = r->t_us;
        }
        s.count = i + 1;
        if (s.count >= s.wanted) {
            s.state = LINK_LOG_DONE;
        }
    } else {
        s.dropped++;
    }
    portEXIT_CRITICAL(&s.mux);
}

void capture_info(capture_info_t *out)
{
    memset(out, 0, sizeof(*out));
    portENTER_CRITICAL(&s.mux);
    out->state             = s.state;
    out->capture_id        = s.capture_id;
    out->count             = s.count;
    out->wanted            = s.wanted;
    out->t_start_us        = s.t_start_us;
    out->dropped           = s.dropped;
    out->imu_accel_g       = (uint16_t)s.scales.imu_accel_g;
    out->imu_gyro_dps      = (uint16_t)s.scales.imu_gyro_dps;
    out->mag_lsb_per_gauss = s.scales.mag_lsb_per_gauss;
    out->mag_chip          = (uint8_t)s.scales.mag_chip;
    out->baro_chip         = (uint8_t)s.scales.baro_chip;
    portEXIT_CRITICAL(&s.mux);
    out->have_buffer = s.buf != NULL;
    out->in_psram    = s.in_psram;
    out->capacity    = s.capacity;
}

uint32_t capture_read(uint32_t first, uint32_t max, log_record_t *out, uint16_t *capture_id)
{
    portENTER_CRITICAL(&s.mux);
    const uint16_t id    = s.capture_id;
    const uint32_t count = s.count;
    portEXIT_CRITICAL(&s.mux);
    *capture_id = id;
    if (s.buf == NULL || first >= count) {
        return 0;
    }
    uint32_t n = count - first;
    if (n > max) {
        n = max;
    }
    memcpy(out, &s.buf[first], n * sizeof(log_record_t));

    portENTER_CRITICAL(&s.mux);
    const bool same = s.capture_id == id;
    portEXIT_CRITICAL(&s.mux);
    return same ? n : 0;
}

/* --------------------------------------------------------------- console */

static const char *state_name(uint8_t st)
{
    switch (st) {
    case LINK_LOG_RUNNING: return "RUNNING";
    case LINK_LOG_DONE:    return "DONE";
    default:               return "idle";
    }
}

static void print_status(void)
{
    capture_info_t i;
    capture_info(&i);
    if (!i.have_buffer) {
        printf("no capture buffer - see the boot log (PSRAM?)\n");
        return;
    }
    printf("buffer %" PRIu32 " records (%" PRIu32 " s) in %s   record %u B   period %u us\n",
           i.capacity, i.capacity / TICKS_PER_SECOND, i.in_psram ? "PSRAM" : "internal RAM",
           (unsigned)sizeof(log_record_t), (unsigned)CTL_PERIOD_US);
    if (i.state == LINK_LOG_IDLE) {
        printf("idle, nothing stored - 'capture --start' records %d s ('-t' for another length)\n",
               CAPTURE_DEFAULT_SECONDS);
        return;
    }
    printf("capture %u %s   %" PRIu32 " of %" PRIu32 " records (%.1f of %" PRIu32 " s)",
           (unsigned)i.capture_id, state_name(i.state), i.count, i.wanted,
           (float)i.count / TICKS_PER_SECOND, i.wanted / TICKS_PER_SECOND);
    if (i.dropped) {
        printf("   %" PRIu32 " dropped", i.dropped);
    }
    printf("\nscales: accel +/-%u g, gyro +/-%u dps, mag %.0f LSB/gauss (chip %u), baro chip %u\n",
           i.imu_accel_g, i.imu_gyro_dps, i.mag_lsb_per_gauss, i.mag_chip, i.baro_chip);
    printf("fetch it over the air: python software/blimp_link.py capture download\n");
}

static void print_head(int n)
{
    log_record_t r;
    uint16_t id;
    printf("%8s %6s %6s %6s %6s %6s %6s %6s %6s %6s %8s %7s %5s %4s %4s %6s %6s %6s %6s %5s\n",
           "t_us", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "p_dPa", "t_cC",
           "tof", "st", "frsh", "FL", "RL", "FR", "RR", "exec");
    for (uint32_t k = 0; k < (uint32_t)n; k++) {
        if (capture_read(k, 1, &r, &id) != 1) {
            break;
        }
        const uint32_t p = r.baro_p[0] | ((uint32_t)r.baro_p[1] << 8) | ((uint32_t)r.baro_p[2] << 16);
        int32_t t = (int32_t)(r.baro_t[0] | ((uint32_t)r.baro_t[1] << 8) | ((uint32_t)r.baro_t[2] << 16));
        if (t & 0x800000) {
            t -= 0x1000000;
        }
        printf("%8" PRIu32 " %6d %6d %6d %6d %6d %6d %6d %6d %6d %8" PRIu32 " %7" PRId32
               " %5u %4u 0x%02x %6d %6d %6d %6d %5u\n",
               r.t_us, r.acc[0], r.acc[1], r.acc[2], r.gyro[0], r.gyro[1], r.gyro[2],
               r.mag[0], r.mag[1], r.mag[2], p, t, r.tof_mm, r.tof_status, r.fresh,
               r.throttle[0], r.throttle[1], r.throttle[2], r.throttle[3], r.exec_us);
    }
}

static struct {
    struct arg_lit *start;
    struct arg_int *seconds;
    struct arg_lit *stop;
    struct arg_lit *clear;
    struct arg_int *head;
    struct arg_end *end;
} capture_args;

static int do_capture_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&capture_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, capture_args.end, argv[0]);
        return 1;
    }
    if (capture_args.start->count) {
        uint32_t seconds = capture_args.seconds->count ? (uint32_t)capture_args.seconds->ival[0] : 0;
        if (capture_args.seconds->count && capture_args.seconds->ival[0] <= 0) {
            ESP_LOGE(TAG, "-t takes a positive number of seconds");
            return 1;
        }
        esp_err_t err = capture_start(seconds);
        if (err == ESP_ERR_INVALID_STATE) {
            printf("a capture is running - 'capture --stop' first\n");
            return 1;
        }
        if (err == ESP_ERR_INVALID_SIZE) {
            printf("%" PRIu32 " s is longer than the buffer: up to %" PRIu32 " s\n", seconds,
                   s.capacity / TICKS_PER_SECOND);
            return 1;
        }
        if (err != ESP_OK) {
            printf("cannot start: %s\n", esp_err_to_name(err));
            return 1;
        }
        print_status();
        return 0;
    }
    if (capture_args.stop->count) {
        if (!capture_stop()) {
            printf("no capture running\n");
        }
        print_status();
        return 0;
    }
    if (capture_args.clear->count) {
        capture_clear();
        printf("cleared\n");
        return 0;
    }
    if (capture_args.head->count) {
        print_head(capture_args.head->ival[0]);
        return 0;
    }
    print_status();
    return 0;
}

void register_capture(void)
{
    capture_args.start   = arg_lit0(NULL, "start", "Start a new capture (discards the last one)");
    capture_args.seconds = arg_int0("t", "seconds", "<s>", "Length of the capture, default 60");
    capture_args.stop    = arg_lit0(NULL, "stop", "End the running capture; its records stay");
    capture_args.clear   = arg_lit0(NULL, "clear", "Discard the stored records");
    capture_args.head    = arg_int0(NULL, "head", "<n>", "Print the first n records, raw");
    capture_args.end     = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "capture",
        .help = "Record the control loop's records in PSRAM: status, --start [-t s], --stop, "
                "--clear, --head n",
        .hint = NULL,
        .func = &do_capture_cmd,
        .argtable = &capture_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
