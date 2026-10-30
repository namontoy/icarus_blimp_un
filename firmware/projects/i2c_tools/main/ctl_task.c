/*
 * ctl_task.c - the fixed-rate control task.
 *
 * Every CTL_PERIOD_US the task runs the pipeline of ctl_types.h, one stage
 * per function with typed input and output:
 *
 *   read_sensors() -> estimate() -> control() -> mixer() -> saturate()
 *                  -> pwm_out() -> log_tick()
 *
 * In this increment the middle stages are empty: the task exists to prove the
 * period, measure its own jitter and execution time, and carry the struct
 * layouts the later increments fill in (docs/control-loop-plan.md section 4).
 * Two stages already do their job in an interim form: read_sensors() peeks
 * the sensor stream's latest sample (the stream still owns the chips; the
 * loop only copies, the way it will peek the camera mailboxes) and samples
 * the battery ADC itself (cmd_battery.c, not on the I2C bus), and
 * log_tick() writes one record per tick into the capture (capture.c), with
 * the motor driver's readback standing in for pwm_out()'s own throttles.
 * Nothing here drives the I2C bus or the motors yet.
 *
 * Clock. A periodic esp_timer gives this task a notification; the task does
 * nothing but wait for it. The esp_timer's resolution is independent of the
 * FreeRTOS tick. It is dispatched from the esp_timer task, as in the ESP-FLY
 * shell (firmware/projects/espfly/main/main.c), so a tick can queue behind
 * Wi-Fi (priority 23 on core 0) for a while. ISR dispatch would avoid that,
 * and the code still uses it when CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
 * is on, but sdkconfig.defaults keeps it off: in ESP-IDF v5.5.5 an ISR alarm
 * landing next to a task alarm can leave the esp_timer task un-notified, and
 * Wi-Fi's own timers then stall (esp-idf issue #18808; on 2026-10-28 the
 * access point dropped the laptop every ~14 s until this was turned off).
 *
 * Placement. Core 1, priority 20: core 0 keeps Wi-Fi, lwIP, esp_timer, the
 * USB console and the main task; 20 is above every task of ours (link 4,
 * stream 3, consoles 2) and below the system tasks, which are pinned to
 * core 0 anyway. Once notified, this is the highest-priority runnable task
 * on its core, so it runs at once.
 *
 * Rules inside the tick: no printf or ESP_LOG, no allocation, no vTaskDelay,
 * no polling loop; later increments add I2C transactions bounded by a
 * timeout of a few milliseconds each and nothing else that can wait. The
 * printer is a separate low-priority task for exactly that reason.
 *
 * Reuse note (docs/reuse_log.md item 1): the fixed-rate loop with a tick
 * counter and "every Nth tick" dividers re-expresses the pattern of
 * esp-drone's stabilizer task (components/core/crazyflie/modules/src/
 * stabilizer.c and RATE_DO_EXECUTE in modules/interface/stabilizer_types.h,
 * commit 9d5a4f85). That loop runs at 1 kHz paced by the IMU's data-ready
 * interrupt, with dividers to run attitude, position and logging below the
 * IMU rate; here the loop runs at 50 Hz from a timer, because the MPU-6050
 * is polled over a shared bus, and the dividers (increment 2) spread the
 * barometer and the ToF over alternate ticks. No code was copied.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "argtable3/argtable3.h"
#include "esp_attr.h"
#include "esp_console.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "capture.h"
#include "cmd_battery.h"
#include "cmd_param.h"
#include "ctl_task.h"
#include "ctl_types.h"
#include "sensor_stream.h"

static const char *TAG = "ctl";

/* esp_psram_* only links with CONFIG_SPIRAM; a build without it (the
   2026-10-28 Wi-Fi bisect) then reports no PSRAM, as IGNORE_NOTFOUND would. */
static bool psram_ready(size_t *size)
{
#if CONFIG_SPIRAM
    if (esp_psram_is_initialized()) {
        *size = esp_psram_get_size();
        return true;
    }
#endif
    (void)size;
    return false;
}

#define CTL_TASK_CORE       1
#define CTL_TASK_PRIO       20
#define CTL_TASK_STACK      6144    /* the ULD and the drivers run here later */

#define CTL_STATS_PRIO      1       /* just above idle: a late line costs nothing */
#define CTL_STATS_STACK     3072
#define CTL_STATS_PERIOD_MS 1000

/* A notification this late means the timer is not running at all. */
#define CTL_TICK_WAIT_MS    100

static struct {
    TaskHandle_t       task;
    TaskHandle_t       printer;
    esp_timer_handle_t timer;
    portMUX_TYPE       mux;             /* guards stats and last_window        */
    ctl_stats_t        stats;           /* the window being filled             */
    ctl_stats_t        last_window;     /* the last completed one              */
    volatile bool      print;
    bool               isr_dispatch;

    /* Pipeline state, owned by the control task. Other tasks read copies
       (later increments), never these. */
    uint32_t      tick;                 /* counts periods; the rate dividers key off it */
    sensors_raw_t raw;
    state_est_t   est;
    setpoint_t    sp;
    ctrl_out_t    out;
    motor_cmd_t   cmd;

    /* The controller's parameters as of the last 'param' change, taken at
       the top of a tick (cmd_param.h); blimp_step() reads them from
       increment 3 on. */
    blimp_params_t params;
    uint32_t       params_gen;
} s = {
    .mux = portMUX_INITIALIZER_UNLOCKED,
    .params_gen = PARAM_GEN_NONE,
    /* Off unless asked for: once a second it drowns the console. */
#if CONFIG_CTL_STATS_AT_BOOT
    .print = true,
#endif
};

/* ---------------------------------------------------------------- stages */

/*
 * Increment 2: IMU and magnetometer every tick, barometer on even ticks,
 * ToF on odd ticks, camera mailboxes and battery ADC every tick.
 *
 * Until then: the sensor stream still owns the chips and samples them at
 * CONFIG_SENSOR_STREAM_HZ; this stage peeks its latest sample and marks a
 * source fresh when its timestamp moved since the last tick. So the records
 * the log stage writes already carry real counts, repeated between the
 * stream's samples, with the fresh bits saying which tick brought a new one.
 * Only the four sensor blocks are copied. The battery is already read here,
 * one oneshot conversion per tick as the schedule has it, because it is on
 * the ADC and owes nothing to the stream; its fresh bit is set whenever a
 * calibrated sample came in. The camera fields stay whatever their own
 * producer (none yet) leaves in them.
 */
static void read_sensors(uint32_t tick, sensors_raw_t *raw)
{
    (void)tick;
    raw->bat_fresh = battery_sample(&raw->bat_raw, &raw->bat_pin_mv, &raw->bat_mv);

    sensors_raw_t latest;
    if (!sensor_stream_latest_raw(&latest)) {
        raw->imu_fresh = raw->mag_fresh = raw->baro_fresh = raw->tof_fresh = false;
        return;
    }
    raw->imu_fresh  = latest.imu_t_us  != raw->imu_t_us;
    raw->mag_fresh  = latest.mag_t_us  != raw->mag_t_us;
    raw->baro_fresh = latest.baro_t_us != raw->baro_t_us;
    raw->tof_fresh  = latest.tof_t_us  != raw->tof_t_us;

    memcpy(raw->acc,  latest.acc,  sizeof(raw->acc));
    memcpy(raw->gyro, latest.gyro, sizeof(raw->gyro));
    raw->imu_temp = latest.imu_temp;
    raw->imu_t_us = latest.imu_t_us;
    memcpy(raw->mag, latest.mag, sizeof(raw->mag));
    raw->mag_t_us = latest.mag_t_us;
    raw->baro_p_dpa  = latest.baro_p_dpa;
    raw->baro_t_cc   = latest.baro_t_cc;
    raw->baro_h_cpct = latest.baro_h_cpct;
    raw->baro_t_us   = latest.baro_t_us;
    raw->tof_mm     = latest.tof_mm;
    raw->tof_status = latest.tof_status;
    raw->tof_t_us   = latest.tof_t_us;
}

/* Increment 3: counts to SI, mounting rotations, tilt for the range. */
static void estimate(const sensors_raw_t *raw, state_est_t *est)
{
    (void)raw;
    (void)est;
}

/* Increment 3: the mode switch. MANUAL bypasses the mixer; the autonomous
   modes output a zero wrench until blimp_step() fills them. */
static void control(const state_est_t *est, const setpoint_t *sp, ctrl_out_t *out)
{
    (void)est;
    (void)sp;
    (void)out;
}

/* Increment 3: wrench to per-motor thrust in N. */
static void mixer(const ctrl_out_t *out, motor_cmd_t *cmd)
{
    (void)out;
    (void)cmd;
}

/* Increment 3: thrust map, clamp, the Rule 13 cap, the clamp flags. */
static void saturate(motor_cmd_t *cmd)
{
    (void)cmd;
}

/* Increment 3: motors_set_all() and motors_failsafe_feed(), nothing else. */
static void pwm_out(const motor_cmd_t *cmd)
{
    (void)cmd;
}

static void put24(uint8_t out[3], uint32_t v)
{
    out[0] = (uint8_t)v;
    out[1] = (uint8_t)(v >> 8);
    out[2] = (uint8_t)(v >> 16);
}

/* -1..1 to the record's 16-bit fixed point. */
static int16_t q15(float v)
{
    if (!(v > -1.0f)) {
        return v < 0 ? -32767 : 0;      /* also NaN: nothing is better than a number */
    }
    return v > 1.0f ? 32767 : (int16_t)(v * 32767.0f);
}

/*
 * One log_record_t per tick into the capture (capture.c). It is built from
 * the pipeline's records, so when the later increments fill those in, this
 * stage does not change; until pwm_out() exists, the throttles are the motor
 * driver's readback of whatever the link or the console commanded, which is
 * the same number TELEMETRY reports. exec_us was measured before this stage
 * ran: the stage itself is a copy and costs a few microseconds.
 */
static void log_tick(const sensors_raw_t *raw, const setpoint_t *sp, const motor_cmd_t *cmd,
                     uint32_t exec_us, uint8_t fs_flags)
{
    (void)cmd;
    log_record_t r;
    memset(&r, 0, sizeof(r));
    r.t_us = raw->t_us;
    for (int i = 0; i < 3; i++) {
        r.acc[i]  = raw->acc[i];
        r.gyro[i] = raw->gyro[i];
        r.mag[i]  = raw->mag[i];
    }
    put24(r.baro_p, raw->baro_p_dpa < 0 ? 0u : (uint32_t)raw->baro_p_dpa);
    put24(r.baro_t, (uint32_t)raw->baro_t_cc);
    r.baro_h     = raw->baro_h_cpct;
    r.tof_mm     = raw->tof_mm;
    r.tof_status = raw->tof_status;
    r.fresh = (raw->imu_fresh  ? CTL_FRESH_IMU  : 0) | (raw->mag_fresh ? CTL_FRESH_MAG : 0) |
              (raw->baro_fresh ? CTL_FRESH_BARO : 0) | (raw->tof_fresh ? CTL_FRESH_TOF : 0) |
              (raw->cam_fresh[CTL_CAM_FORWARD] ? CTL_FRESH_CAM0 : 0) |
              (raw->cam_fresh[CTL_CAM_DOWN] ? CTL_FRESH_CAM1 : 0) |
              (raw->bat_fresh ? CTL_FRESH_BAT : 0);
    r.mode     = (uint8_t)sp->mode;
    r.fs_flags = fs_flags;
    for (int c = 0; c < CTL_CAM_COUNT; c++) {
        r.cam[c].seq   = (uint8_t)raw->cam[c].frame_seq;
        r.cam[c].flags = raw->cam[c].detected ? 1 : 0;
        r.cam[c].cx    = raw->cam[c].cx_px;
        r.cam[c].cy    = raw->cam[c].cy_px;
        r.cam[c].area  = raw->cam[c].area_px;
    }
    for (int m = 0; m < MOTOR_COUNT; m++) {
        r.setpoint[m] = q15(sp->cmd[m]);
        r.throttle[m] = q15(motor_get((motor_id_t)m));
    }
    r.bat_mv  = raw->bat_mv;
    r.exec_us = exec_us > 65535 ? 65535 : (uint16_t)exec_us;
    capture_push(&r);
}

/* ------------------------------------------------------------- the tick */

static void stats_add(uint32_t period_us, bool have_period, uint32_t exec_us,
                      uint32_t missed, bool overrun)
{
    portENTER_CRITICAL(&s.mux);
    ctl_stats_t *st = &s.stats;
    if (st->ticks == 0) {
        st->periods       = 0;
        st->period_min_us = UINT32_MAX;
        st->period_max_us = 0;
        st->period_sum_us = 0;
        st->exec_min_us   = UINT32_MAX;
        st->exec_max_us   = 0;
        st->exec_sum_us   = 0;
    }
    st->ticks++;
    st->total_ticks++;
    if (have_period) {
        st->periods++;
        if (period_us < st->period_min_us) st->period_min_us = period_us;
        if (period_us > st->period_max_us) st->period_max_us = period_us;
        st->period_sum_us += period_us;
    }
    if (exec_us < st->exec_min_us) st->exec_min_us = exec_us;
    if (exec_us > st->exec_max_us) st->exec_max_us = exec_us;
    st->exec_sum_us += exec_us;
    st->missed += missed;
    if (overrun) {
        st->overruns++;
    }
    portEXIT_CRITICAL(&s.mux);
}

/*
 * Runs in the esp_timer task, or in the timer interrupt on core 0 if ISR
 * dispatch is enabled. Either way it only hands the control task its
 * notification; the FreeRTOS port raises the cross-core interrupt itself
 * when the woken task lives on the other core. An ISR-dispatched callback
 * must not call portYIELD_FROM_ISR() (esp_timer's docs): it asks esp_timer
 * to yield once all its ISR timers have run.
 */
static void IRAM_ATTR tick_cb(void *arg)
{
    (void)arg;
    if (xPortInIsrContext()) {
#if CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(s.task, &woken);
        if (woken == pdTRUE) {
            esp_timer_isr_dispatch_need_yield();
        }
#endif
    } else {
        xTaskNotifyGive(s.task);
    }
}

static void ctl_task(void *arg)
{
    (void)arg;
    int64_t last_wake_us = 0;
    bool    have_period  = false;

    ESP_LOGI(TAG, "control task on core %d, priority %d, period %u us, %s dispatch",
             (int)xPortGetCoreID(), CTL_TASK_PRIO, (unsigned)CTL_PERIOD_US,
             s.isr_dispatch ? "ISR" : "task");

    for (;;) {
        /* The count is how many periods elapsed since the last take: 1 is
           the normal case, more means the previous tick ran long and ticks
           were skipped, 0 means the timer stopped. */
        const uint32_t n = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CTL_TICK_WAIT_MS));
        const int64_t t0 = esp_timer_get_time();
        if (n == 0) {
            portENTER_CRITICAL(&s.mux);
            s.stats.timer_silent++;
            portEXIT_CRITICAL(&s.mux);
            have_period = false;
            continue;
        }

        const uint32_t period_us = have_period ? (uint32_t)(t0 - last_wake_us) : 0;
        last_wake_us = t0;
        s.tick++;
        s.raw.t_us = (uint32_t)t0;

        (void)param_take(&s.params, &s.params_gen);
        read_sensors(s.tick, &s.raw);
        estimate(&s.raw, &s.est);
        control(&s.est, &s.sp, &s.out);
        mixer(&s.out, &s.cmd);
        saturate(&s.cmd);
        pwm_out(&s.cmd);

        const uint32_t exec_us = (uint32_t)(esp_timer_get_time() - t0);
        const bool overrun = exec_us > CTL_PERIOD_US;
        log_tick(&s.raw, &s.sp, &s.cmd, exec_us, overrun ? CTL_FS_OVERRUN : 0);

        stats_add(period_us, have_period, exec_us, n - 1, overrun);
        have_period = true;
    }
}

/* ---------------------------------------------------------------- printer */

static void take_window(ctl_stats_t *out)
{
    portENTER_CRITICAL(&s.mux);
    s.last_window = s.stats;
    s.stats.ticks = 0;              /* the next stats_add() re-initialises the window */
    *out = s.last_window;
    portEXIT_CRITICAL(&s.mux);
}

static void format_stats(const ctl_stats_t *st, char *buf, size_t len)
{
    if (st->ticks == 0) {
        snprintf(buf, len, "no ticks in the last window   missed %" PRIu32
                 "   overruns %" PRIu32 "   timer silent %" PRIu32,
                 st->missed, st->overruns, st->timer_silent);
        return;
    }
    const bool have_period = st->periods > 0;
    const uint32_t periods = have_period ? st->periods : 1;
    snprintf(buf, len,
             "%" PRIu32 " ticks   period %.2f/%.2f/%.2f ms   exec %" PRIu32 "/%" PRIu32
             "/%" PRIu32 " us (min/max/mean)   overruns %" PRIu32 "   missed %" PRIu32
             "   timer silent %" PRIu32,
             st->ticks,
             have_period ? st->period_min_us / 1000.0f : 0.0f,
             have_period ? st->period_max_us / 1000.0f : 0.0f,
             have_period ? (float)st->period_sum_us / periods / 1000.0f : 0.0f,
             st->exec_min_us, st->exec_max_us, (uint32_t)(st->exec_sum_us / st->ticks),
             st->overruns, st->missed, st->timer_silent);
}

/* Low priority on core 0, where the console lives: a line that arrives a
   few milliseconds late costs nothing, and the control task never prints. */
static void printer_task(void *arg)
{
    (void)arg;
    char line[200];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(CTL_STATS_PERIOD_MS));
        ctl_stats_t w;
        take_window(&w);
        if (s.print) {
            format_stats(&w, line, sizeof(line));
            ESP_LOGI(TAG, "%s", line);
        }
    }
}

/* ------------------------------------------------------------------- API */

void ctl_get_stats(ctl_stats_t *out)
{
    portENTER_CRITICAL(&s.mux);
    *out = s.last_window;
    /* The cumulative counters keep moving between windows; report the
       current ones so a 'ctl' typed right after a miss shows it. */
    out->total_ticks  = s.stats.total_ticks;
    out->overruns     = s.stats.overruns;
    out->missed       = s.stats.missed;
    out->timer_silent = s.stats.timer_silent;
    portEXIT_CRITICAL(&s.mux);
}

void ctl_set_stats_print(bool on)
{
    s.print = on;
}

esp_err_t ctl_task_start(void)
{
    if (s.task != NULL) {
        return ESP_OK;
    }

    /* Before the task exists: its first line reports the dispatch. */
#if CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
    s.isr_dispatch = true;
    ESP_LOGW(TAG, "esp_timer ISR dispatch is on: on ESP-IDF v5.5.x it can stall Wi-Fi's "
                  "timers (esp-idf issue #18808); see sdkconfig.defaults");
#else
    s.isr_dispatch = false;
#endif

    if (xTaskCreatePinnedToCore(ctl_task, "ctl", CTL_TASK_STACK, NULL, CTL_TASK_PRIO,
                                &s.task, CTL_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "could not create the control task");
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(printer_task, "ctl_stats", CTL_STATS_STACK, NULL,
                                CTL_STATS_PRIO, &s.printer, 0) != pdPASS) {
        ESP_LOGW(TAG, "could not create the statistics printer; 'ctl' still works");
    }

    esp_timer_create_args_t args = {
        .callback = tick_cb,
        .arg = NULL,
        .name = "ctl_tick",
#if CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
        .dispatch_method = ESP_TIMER_ISR,
#else
        .dispatch_method = ESP_TIMER_TASK,
#endif
    };
    esp_err_t err = esp_timer_create(&args, &s.timer);
    if (err == ESP_OK) {
        err = esp_timer_start_periodic(s.timer, CTL_PERIOD_US);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "control timer: %s", esp_err_to_name(err));
        return err;
    }

    /* The log ring (increment 4) will live in PSRAM; say how much there is
       before anything else claims it, so the buffer can be sized from a
       measured number rather than the datasheet's 2 MB. */
    size_t psram_size;
    if (psram_ready(&psram_size)) {
        ESP_LOGI(TAG, "PSRAM %u KB, %u KB free",
                 (unsigned)(psram_size / 1024),
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
    } else {
        ESP_LOGW(TAG, "no PSRAM: the flight log will have no buffer");
    }
    return ESP_OK;
}

/* --------------------------------------------------------------- console */

static struct {
    struct arg_str *stats;
    struct arg_end *end;
} ctl_args;

static int do_ctl_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&ctl_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, ctl_args.end, argv[0]);
        return 1;
    }
    if (s.task == NULL) {
        printf("control task not running - see the boot log for why\n");
        return 1;
    }
    if (ctl_args.stats->count) {
        const char *v = ctl_args.stats->sval[0];
        if (strcmp(v, "on") == 0) {
            s.print = true;
        } else if (strcmp(v, "off") == 0) {
            s.print = false;
        } else {
            ESP_LOGE(TAG, "--stats takes on or off");
            return 1;
        }
        printf("statistics line %s\n", s.print ? "on" : "off");
        return 0;
    }

    ctl_stats_t st;
    ctl_get_stats(&st);
    char line[200];
    format_stats(&st, line, sizeof(line));
    printf("control task on core %d, priority %d, period %u us, %s dispatch, "
           "%" PRIu32 " ticks since boot\n",
           CTL_TASK_CORE, CTL_TASK_PRIO, (unsigned)CTL_PERIOD_US,
           s.isr_dispatch ? "ISR" : "task", st.total_ticks);
    printf("last window: %s\n", line);
    printf("stages: read_sensors peeks the sensor stream (interim) and samples the battery; "
           "estimate, control, mixer, "
           "saturate, pwm_out empty; log feeds 'capture'\n");
    const uint32_t gen = s.params_gen;
    if (gen == PARAM_GEN_NONE) {
        printf("parameters: none taken yet\n");
    } else {
        printf("parameters: the loop holds the set after %" PRIu32 " of %" PRIu32 " 'param' changes "
               "(read by nothing until increment 3)\n", gen, param_changes());
    }
    size_t psram_size;
    if (psram_ready(&psram_size)) {
        printf("PSRAM %u KB, %u KB free (largest block %u KB)\n",
               (unsigned)(psram_size / 1024),
               (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
               (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024));
    } else {
        printf("PSRAM not initialised\n");
    }
    printf("statistics line every %d ms: %s\n", CTL_STATS_PERIOD_MS, s.print ? "on" : "off");
    return 0;
}

void register_ctl(void)
{
    ctl_args.stats = arg_str0(NULL, "stats", "<on|off>", "Print the timing line once a second");
    ctl_args.end   = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "ctl",
        .help = "The control loop: timing statistics, PSRAM; --stats on|off",
        .hint = NULL,
        .func = &do_ctl_cmd,
        .argtable = &ctl_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
