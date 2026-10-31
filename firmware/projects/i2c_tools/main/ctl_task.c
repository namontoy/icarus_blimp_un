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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "argtable3/argtable3.h"
#include "blimp/blimp.h"
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
#include "cam_link.h"
#include "ctl_task.h"
#include "ctl_types.h"
#include "net_link.h"
#include "sensor_stream.h"

static const char *TAG = "ctl";

/* A sample older than this is not handed to core/ as valid. The stream runs
   at 50 Hz, the ToF at about 10 Hz and the magnetometer with the IMU. */
#define CTL_IMU_STALE_US    100000u
#define CTL_MAG_STALE_US    250000u
#define CTL_TOF_STALE_US    250000u
#define CTL_CAM_STALE_US    300000u     /* about four of the camera's 80 ms frames */
#define CTL_G_MPS2          9.80665f

/* A sample stamped at `then` (0: never) is no older than max_us at `now`.
   Signed: the stream, the link and the camera stamp on their own tasks, so a
   sample can land after the tick took its time and be a few microseconds
   NEWER than it, which unsigned arithmetic would wrap to 71 minutes of age
   (core/'s link watchdog fell into failsafe that way, 2026-10-30). */
static bool fresh_at(uint32_t now, uint32_t then, uint32_t max_us)
{
    return then != 0u && (int32_t)(now - then) < (int32_t)max_us;
}

/* The chip axis (0-2) and sign along each body axis, from Kconfig's
   CTL_IMU_AXES and CTL_MAG_AXES. */
typedef struct {
    uint8_t idx[3];
    int8_t  sign[3];
} axis_map_t;

/* What 'hold' shows, copied out of the tick under the mux. */
typedef struct {
    ctl_mode_t        mode;
    bool              armed;
    bool              converging;
    float             still_s;          /* the estimator's stationary dwell    */
    float             yaw, yaw_rate;    /* rad, rad/s, core/'s estimate        */
    float             altitude;
    bool              altitude_valid;
    blimp_wrench_t    demand;           /* the errors the loops ran on         */
    ctrl_out_t        out;              /* the wrench the motors were given    */
    blimp_actuators_t act;
    float             throttle[MOTOR_COUNT];
    uint8_t           guid_state;
    uint32_t          link_age_ms;
    bool              link_seen;
    bool              follow_valid;     /* core/ has a sighting to steer on    */
    float             follow_yaw;       /* rad, where the target was last seen */
    bool              tgt_visible;      /* this tick's camera row, as handed in */
    float             tgt_bearing;      /* rad, core/'s sign                   */
} hold_snap_t;

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

    /* The controller, core/'s blimp_step(), run every tick in every mode so
       the estimator converges in the background; its output reaches the
       motors only in the bench hold (CLOSED_LOOP). */
    blimp_t           b;
    blimp_actuators_t act;
    blimp_hold_t      hold;             /* the setpoints in force               */
    sensor_scales_t   scales;
    axis_map_t        imu_axes, mag_axes;

    /* Requests from the 'hold' command, taken at the top of a tick under the
       mux; and the kill, set by the motor driver's stop hook from whatever
       task stopped the motors (cmd_motor.h), so a plain flag. */
    bool              want_hold;
    blimp_hold_t      hold_req;
    volatile bool     kill_req;
    hold_snap_t       snap;
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
 * calibrated sample came in. The forward camera's mailbox (cam_link.c) is
 * peeked the same way, fresh when its frame_seq moved; the downward camera
 * has no producer yet.
 */
static void read_sensors(uint32_t tick, sensors_raw_t *raw)
{
    (void)tick;
    raw->bat_fresh = battery_sample(&raw->bat_raw, &raw->bat_pin_mv, &raw->bat_mv);

    cam_result_t cam;
    if (cam_link_latest(&cam)) {
        raw->cam_fresh[CTL_CAM_FORWARD] = cam.frame_seq != raw->cam[CTL_CAM_FORWARD].frame_seq;
        raw->cam[CTL_CAM_FORWARD] = cam;
    } else {
        raw->cam_fresh[CTL_CAM_FORWARD] = false;
    }

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

/* "-Y +X +Z": the chip axis, with its sign, along body x, y, z. */
static bool parse_axes(const char *text, axis_map_t *out)
{
    bool seen[3] = { false, false, false };
    int n = 0;
    const char *c = text;
    while (*c) {
        while (*c == ' ' || *c == ',') {
            c++;
        }
        if (!*c) {
            break;
        }
        int8_t sign = 1;
        if (*c == '+' || *c == '-') {
            sign = (*c == '-') ? -1 : 1;
            c++;
        }
        const int idx = (*c == 'x' || *c == 'X') ? 0 : (*c == 'y' || *c == 'Y') ? 1
                      : (*c == 'z' || *c == 'Z') ? 2 : -1;
        if (idx < 0 || seen[idx] || n >= 3) {
            return false;
        }
        seen[idx] = true;
        out->idx[n] = (uint8_t)idx;
        out->sign[n] = sign;
        n++;
        c++;
    }
    return n == 3;
}

static void remap(const axis_map_t *m, const float chip[3], float body[3])
{
    for (int i = 0; i < 3; i++) {
        body[i] = (float)m->sign[i] * chip[m->idx[i]];
    }
}

/*
 * Counts to SI in core/'s body frame (x forward, y left, z up), through the
 * mounting maps of Kconfig, which is everything core/'s blimp_imu_sample_t
 * asks of the shell. The scales are the ones the stream attached the chips
 * at, re-read once a second (a console command can re-attach at others).
 *
 * The IMU sample keeps its own timestamp, not the tick's. The stream does not
 * deliver a new sample every tick (about 40 of 50 a second, 2026-10-30), and
 * core/'s estimator integrates the gyro over the gap between timestamps: a
 * repeated sample then has dt = 0 and is skipped cleanly, where marking it
 * invalid would integrate a zero rate for the tick and restart the estimator's
 * stillness count, which then never reaches the 60 s that arming needs.
 */
static void estimate(const sensors_raw_t *raw, state_est_t *est)
{
    if (s.scales.imu_gyro_dps == 0 || s.tick % 50u == 0u) {
        sensor_stream_scales(&s.scales);
    }
    const sensor_scales_t *sc = &s.scales;
    float chip[3];

    est->t_us = raw->imu_t_us;
    const bool imu_known = raw->imu_t_us != 0 && sc->imu_gyro_dps > 0 && sc->imu_accel_g > 0;
    const bool imu_ok = imu_known && fresh_at(raw->t_us, raw->imu_t_us, CTL_IMU_STALE_US);
    for (int i = 0; i < 3; i++) {
        chip[i] = (float)raw->gyro[i] * ((float)sc->imu_gyro_dps / 32768.0f) * (BLIMP_PI / 180.0f);
    }
    remap(&s.imu_axes, chip, est->gyro);
    for (int i = 0; i < 3; i++) {
        chip[i] = (float)raw->acc[i] * ((float)sc->imu_accel_g / 32768.0f) * CTL_G_MPS2;
    }
    remap(&s.imu_axes, chip, est->accel);
    est->gyro_valid = est->accel_valid = imu_ok;

    const float mag_lsb = sc->mag_lsb_per_gauss;
    for (int i = 0; i < 3; i++) {
        chip[i] = mag_lsb > 0.0f ? (float)raw->mag[i] / mag_lsb * 100.0f : 0.0f;     /* uT */
    }
    remap(&s.mag_axes, chip, est->mag);
    est->mag_valid = mag_lsb > 0.0f && fresh_at(raw->t_us, raw->mag_t_us, CTL_MAG_STALE_US);

    est->pressure_pa  = (float)raw->baro_p_dpa * 0.1f;
    est->air_temp_c   = (float)raw->baro_t_cc * 0.01f;
    est->humidity_pct = (float)raw->baro_h_cpct * 0.01f;
    est->baro_alt = est->pressure_pa > 0.0f
                  ? 44330.0f * (1.0f - powf(est->pressure_pa / 101325.0f, 0.190295f)) : 0.0f;
    /* Not handed to core/ yet. About one BME280 reading in thirteen comes back
       1-10 C cold with its pressure 150-1440 Pa low (2026-10-30, and in that
       morning's capture too): a jump of up to 120 m, which core/'s ceiling
       guard would answer with a failsafe in the middle of a test. The ToF is
       the altitude source until the barometer's reads are fixed. */
    est->baro_valid = false;

    /* Tilt from the accelerometer, for the range only: h = d cos(roll) cos(pitch). */
    const float ax = est->accel[0], ay = est->accel[1], az = est->accel[2];
    est->roll  = atan2f(ay, az);
    est->pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    est->range_alt = (float)raw->tof_mm * 0.001f * cosf(est->roll) * cosf(est->pitch);
    est->range_valid = imu_ok && raw->tof_status == 0 && raw->tof_mm > 0 &&
                       fresh_at(raw->t_us, raw->tof_t_us, CTL_TOF_STALE_US);

    est->imu_temp_c = (float)raw->imu_temp / 340.0f + 36.53f;      /* MPU-6050 datasheet */
    est->bat_v      = (float)raw->bat_mv * 0.001f;

    /* The forward camera's detection in core/'s convention: target_cam's az
       is + to the right of the image centre, core/'s bearing + to the left,
       so the mounting sign is -1 for an upright, unmirrored camera (Kconfig
       CAM_FWD_BEARING_SIGN, 'cam --sign'). The arrival time is new per row,
       which is how core/'s target follow tells a new frame from a repeat. */
    const cam_result_t *cam = &raw->cam[CTL_CAM_FORWARD];
    est->tgt_visible = cam->detected && fresh_at(raw->t_us, cam->t_us, CTL_CAM_STALE_US);
    est->tgt_bearing_rad   = (float)cam_link_bearing_sign() * cam->az_deg * (BLIMP_PI / 180.0f);
    est->tgt_elevation_rad = cam->el_deg * (BLIMP_PI / 180.0f);
    est->tgt_t_us          = cam->t_us;
}

/*
 * blimp_step(), every tick and in every mode. Only the bench hold arms it
 * (CLOSED_LOOP); in MANUAL it runs disarmed, so the estimator keeps
 * converging and the hold can start without a 60 s wait. Guidance never runs
 * on this vehicle yet (it needs the forward camera's detections), so the
 * hold's setpoints are always the source: hold.active is set in every mode,
 * and an axis not held outputs nothing. Every valid packet from the ground
 * refreshes core/'s link watchdog, which zeroes the hold when the ground has
 * been quiet for link_timeout_s.
 */
static void control(const state_est_t *est, const setpoint_t *sp, ctrl_out_t *out)
{
    blimp_input_t in;
    memset(&in, 0, sizeof(in));
    memcpy(in.imu.gyro,  est->gyro,  sizeof(in.imu.gyro));
    memcpy(in.imu.accel, est->accel, sizeof(in.imu.accel));
    memcpy(in.imu.mag,   est->mag,   sizeof(in.imu.mag));
    in.imu.baro_alt    = est->baro_alt;
    in.imu.range_alt   = est->range_alt;
    in.imu.t_us        = est->t_us;
    in.imu.gyro_valid  = est->gyro_valid;
    in.imu.accel_valid = est->accel_valid;
    in.imu.mag_valid   = est->mag_valid;
    in.imu.baro_valid  = est->baro_valid;
    in.imu.range_valid = est->range_valid;
    in.arm_request = sp->mode == CTL_MODE_CLOSED_LOOP;
    in.t_us        = s.raw.t_us;
    in.hold        = s.hold;
    in.hold.active = true;

    /* The forward camera's target, for the hold's follow ('hold follow').
       No range: target_cam's comes from the balloon's apparent size, not
       checked yet, and core/ turns a visible target WITH a range into an
       altitude fix (Rule 22: every balloon at 2.0 m) whenever the ToF has no
       reading; a balloon held up by hand on the bench is not at 2.0 m. */
    in.target.visible       = est->tgt_visible;
    in.target.bearing_rad   = est->tgt_bearing_rad;
    in.target.elevation_rad = est->tgt_elevation_rad;
    in.target.range_m       = NAN;
    in.target.confidence    = est->tgt_visible ? 1.0f : 0.0f;
    in.target.t_us          = est->tgt_t_us;

    uint32_t rx_us;
    if (net_link_last_rx(&rx_us)) {
        blimp_notify_link(&s.b, rx_us);
    }
    blimp_step(&s.b, &in, &s.act);

    blimp_wrench_t w;
    blimp_mixer_forward(&s.b.params, &s.act, &w);
    out->fx = w.fx;
    out->fy = 0.0f;
    out->fz = w.fz;
    out->tz = w.tz;
}

/*
 * core/'s mixer already ran inside blimp_step(), Rule 13 included; this
 * stage only puts its thrusts on the motors. core/ names them by position in
 * its frame, and which motor sits at each position is the motor map's
 * (cmd_motor.h), so a renamed motor still pushes from where it is.
 */
static void mixer(const ctrl_out_t *out, motor_cmd_t *cmd)
{
    (void)out;
    float by_pos[MOTOR_COUNT];
    by_pos[MOTOR_FL] = s.act.thrust_fl;
    by_pos[MOTOR_RL] = s.act.thrust_rl;
    by_pos[MOTOR_FR] = s.act.thrust_fr;
    by_pos[MOTOR_RR] = s.act.thrust_rr;
    for (int p = 0; p < MOTOR_COUNT; p++) {
        cmd->thrust_n[motor_at_position((motor_id_t)p)] = by_pos[p];
    }
    cmd->clamped_total = s.act.clamped_total;
}

/*
 * Thrust to throttle through core/'s own normalisation, motor_max_thrust_n
 * (params.c: "both the Rule 13 compliance basis and the PWM normalisation").
 * Linear, so against the measured F(u) = a u^2 + b u (docs/history/
 * pid-bench-2026-10-28) a part duty gives less thrust than core/ asked for:
 * the loop under-delivers, never over. A per-motor curve is the thrust map's
 * job (docs/pid-tuning-plan.md, P0). The driver then maps (0, 1] onto
 * [deadband, 1]. One-way, as core/'s mixer is: every thrust is >= 0.
 */
static void saturate(motor_cmd_t *cmd)
{
    const float tmax = s.b.params.motor_max_thrust_n;
    cmd->clamped_mask = 0;
    for (int m = 0; m < MOTOR_COUNT; m++) {
        float u = (tmax > 0.0f) ? cmd->thrust_n[m] / tmax : 0.0f;
        if (!(u > 0.0f)) {
            u = 0.0f;                   /* also NaN */
        } else if (u >= 1.0f) {
            u = 1.0f;
            cmd->clamped_mask |= (uint8_t)(1u << m);
        }
        cmd->throttle[m] = u;
    }
}

/*
 * The bench hold writes the motors here and nowhere else; the write feeds the
 * driver's failsafe, which then trips only if this task stops. A kill that
 * arrived during this tick is checked once more just before the write, so a
 * stop is overridden for at most the few microseconds between the two.
 * KILLED writes nothing: the stop that killed already coasted, and the
 * loop's claim keeps every other source out. MANUAL: the link and the console
 * drive the motors (net_link.c).
 */
static void pwm_out(const motor_cmd_t *cmd)
{
    if (s.sp.mode == CTL_MODE_CLOSED_LOOP && !s.kill_req) {
        (void)motors_loop_set_all(cmd->throttle);
    }
}

/* The stop hook (cmd_motor.h): any stop while the loop holds the motors. */
static void on_motor_stop(void)
{
    s.kill_req = true;
}

/* At the top of a tick: the kill, then what 'hold' asked for. */
static void take_requests(void)
{
    bool want;
    blimp_hold_t req;
    portENTER_CRITICAL(&s.mux);
    want = s.want_hold;
    req = s.hold_req;
    portEXIT_CRITICAL(&s.mux);

    if (s.kill_req) {
        s.kill_req = false;
        if (s.sp.mode == CTL_MODE_CLOSED_LOOP) {
            s.sp.mode = CTL_MODE_KILLED;    /* the claim stays: nothing else drives */
        }
    }
    if (!want) {
        if (s.sp.mode != CTL_MODE_MANUAL) {
            motors_loop_claim(false);       /* coasts, and gives the link the motors back */
            s.sp.mode = CTL_MODE_MANUAL;
        }
    } else if (s.sp.mode == CTL_MODE_MANUAL) {
        motors_loop_claim(true);
        s.sp.mode = CTL_MODE_CLOSED_LOOP;
    }
    s.hold = req;
}

static void snapshot(void)
{
    hold_snap_t n;
    n.mode           = s.sp.mode;
    n.armed          = s.b.armed;
    n.converging     = blimp_estimator_converging(&s.b.est);
    n.still_s        = s.b.est.stationary_s;
    n.yaw            = s.b.state.yaw;
    n.yaw_rate       = s.b.state.yaw_rate;
    n.altitude       = s.b.state.altitude;
    n.altitude_valid = s.b.state.altitude_valid;
    n.demand         = s.b.demand;
    n.out            = s.out;
    n.act            = s.act;
    memcpy(n.throttle, s.cmd.throttle, sizeof(n.throttle));
    n.guid_state     = (uint8_t)s.b.guid.state;
    uint32_t rx_us;
    n.link_seen   = net_link_last_rx(&rx_us);
    const int32_t link_age_us = (int32_t)(s.raw.t_us - rx_us);   /* newer than the tick: 0 */
    n.link_age_ms = link_age_us > 0 ? (uint32_t)link_age_us / 1000u : 0u;
    n.follow_valid = s.b.follow_valid;
    n.follow_yaw   = s.b.follow_yaw_rad;
    n.tgt_visible  = s.est.tgt_visible;
    n.tgt_bearing  = s.est.tgt_bearing_rad;
    portENTER_CRITICAL(&s.mux);
    s.snap = n;
    portEXIT_CRITICAL(&s.mux);
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

/* v / unit, rounded and saturated to int16; NaN logs 0. */
static int16_t fixed16(float v, float unit)
{
    const float q = v / unit;
    if (!(q == q)) {
        return 0;
    }
    if (q >= 32767.0f) {
        return 32767;
    }
    if (q <= -32767.0f) {
        return -32767;
    }
    return (int16_t)lrintf(q);
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
    if (sp->mode == CTL_MODE_MANUAL) {
        for (int m = 0; m < MOTOR_COUNT; m++) {
            r.setpoint[m] = q15(sp->cmd[m]);
        }
    } else {
        /* The bench hold (ctl_types.h, log_record_t): the wrench given to
           the motors, and the heading error the yaw loop ran on, which with
           the setpoint the tool recorded gives the estimated heading. */
        r.setpoint[0] = fixed16(s.out.fx, 1e-4f);           /* 0.1 mN       */
        r.setpoint[1] = fixed16(s.out.fz, 1e-4f);           /* 0.1 mN       */
        r.setpoint[2] = fixed16(s.out.tz, 1e-5f);           /* 10 uN m      */
        r.setpoint[3] = fixed16(s.b.demand.tz, 1e-3f);      /* mrad         */
    }
    for (int m = 0; m < MOTOR_COUNT; m++) {
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

        if (param_take(&s.params, &s.params_gen)) {
            s.b.params = s.params;          /* blimp_step() validates it every tick */
        }
        take_requests();
        read_sensors(s.tick, &s.raw);
        estimate(&s.raw, &s.est);
        control(&s.est, &s.sp, &s.out);
        mixer(&s.out, &s.cmd);
        saturate(&s.cmd);
        pwm_out(&s.cmd);

        const uint32_t exec_us = (uint32_t)(esp_timer_get_time() - t0);
        const bool overrun = exec_us > CTL_PERIOD_US;
        const uint8_t fs = (overrun ? CTL_FS_OVERRUN : 0) |
                           (s.sp.mode == CTL_MODE_KILLED ? CTL_FS_KILLED : 0) |
                           (s.cmd.clamped_total ? CTL_FS_CLAMP_TOTAL : 0);
        log_tick(&s.raw, &s.sp, &s.cmd, exec_us, fs);
        snapshot();

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

    /* The controller and the mounting maps, before the first tick. A map
       that does not parse falls back to the chip's own axes, loudly: the hold
       then turns the wrong way unless the chip happens to sit that way. */
    blimp_init(&s.b);
    s.sp.mode = CTL_MODE_MANUAL;
    if (!parse_axes(CONFIG_CTL_IMU_AXES, &s.imu_axes)) {
        ESP_LOGE(TAG, "CTL_IMU_AXES \"%s\" does not parse; using the chip's axes", CONFIG_CTL_IMU_AXES);
        (void)parse_axes("+X +Y +Z", &s.imu_axes);
    }
    if (!parse_axes(CONFIG_CTL_MAG_AXES, &s.mag_axes)) {
        ESP_LOGE(TAG, "CTL_MAG_AXES \"%s\" does not parse; using the chip's axes", CONFIG_CTL_MAG_AXES);
        (void)parse_axes("+X +Y +Z", &s.mag_axes);
    }
    motors_set_stop_hook(on_motor_stop);
    ESP_LOGI(TAG, "%s; IMU axes %s, magnetometer axes %s", blimp_version(),
             CONFIG_CTL_IMU_AXES, CONFIG_CTL_MAG_AXES);

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
           "estimate converts to SI in the body frame; control runs blimp_step() (%s) every tick, "
           "armed only in the bench hold ('hold'); mixer, saturate and pwm_out drive the motors "
           "only in the hold; log feeds 'capture'\n", blimp_version());
    const uint32_t gen = s.params_gen;
    if (gen == PARAM_GEN_NONE) {
        printf("parameters: none taken yet\n");
    } else {
        printf("parameters: the loop holds the set after %" PRIu32 " of %" PRIu32 " 'param' changes "
               "(blimp_step() reads them; they reach the motors in the bench hold)\n",
               gen, param_changes());
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

/* ------------------------------------------------------- 'hold' command */

/*
 * The bench hold (docs/control-loop-plan.md increment 3, the slice built
 * 2026-10-30 for docs/pid-tuning-plan.md C1-C4): core/'s own loops on a
 * heading, an altitude or a surge commanded here, gains from 'param'. Parsed
 * by hand, not argtable, for the same reason as 'param': a negative heading
 * would read as an option.
 *
 * Starting is refused unless the estimator has converged (core/ would not arm
 * anyway, and a hold that starts by itself a minute later is a surprise),
 * no motor is driven (the link's own timeout would otherwise coast them under
 * the loop and kill it), and a ground tool has sent a packet within 0.5 s:
 * core/'s link watchdog zeroes the hold after link_timeout_s of silence, so
 * the hold runs only while something on the ground keeps the link alive.
 */
#define HOLD_ALT_MIN_M      0.2f
#define HOLD_ALT_MAX_M      3.0f
#define HOLD_SURGE_MAX_GF   30.0f
#define HOLD_LINK_FRESH_MS  500u
#define HOLD_CAM_FRESH_MS   500u
#define HOLD_GF_N           0.00980665f
#define HOLD_RAD_DEG        (180.0f / BLIMP_PI)

static const char *mode_name(ctl_mode_t m)
{
    switch (m) {
    case CTL_MODE_MANUAL:      return "MANUAL";
    case CTL_MODE_OPEN_LOOP:   return "OPEN_LOOP";
    case CTL_MODE_CLOSED_LOOP: return "HOLD";
    case CTL_MODE_KILLED:      return "KILLED";
    default:                   return "?";
    }
}

static void print_hold(void)
{
    hold_snap_t n;
    blimp_hold_t h;
    portENTER_CRITICAL(&s.mux);
    n = s.snap;
    h = s.hold_req;
    portEXIT_CRITICAL(&s.mux);

    printf("mode %s", mode_name(n.mode));
    if (n.mode == CTL_MODE_CLOSED_LOOP) {
        printf(n.armed ? ", armed" : ", NOT armed: the estimator is converging, motors at zero");
    } else if (n.mode == CTL_MODE_KILLED) {
        printf(": a stop arrived; 'hold off' gives the motors back");
    }
    printf("\nestimator: %s (still %.0f of 60 s)\n", n.converging ? "converging" : "converged",
           (double)n.still_s);
    printf("heading: estimate %+.1f deg (0 = the nose at arming), rate %+.1f deg/s",
           (double)(n.yaw * HOLD_RAD_DEG), (double)(n.yaw_rate * HOLD_RAD_DEG));
    if (h.target_held) {
        if (n.follow_valid) {
            printf("; FOLLOWING the target, last seen at %+.1f deg, error %+.1f deg",
                   (double)(n.follow_yaw * HOLD_RAD_DEG), (double)(n.demand.tz * HOLD_RAD_DEG));
        } else {
            printf("; FOLLOWING, nothing seen yet: no yaw output until the first detection");
        }
    } else if (h.heading_held) {
        printf("; held at %+.1f deg, error %+.1f deg", (double)(h.heading_rad * HOLD_RAD_DEG),
               (double)(n.demand.tz * HOLD_RAD_DEG));
    } else {
        printf("; not held");
    }
    if (n.tgt_visible) {
        printf("\ncamera: target at bearing %+.1f deg (+ = to the left)", (double)(n.tgt_bearing * HOLD_RAD_DEG));
    } else {
        printf("\ncamera: no target ('cam' for the link)");
    }
    printf("\naltitude: estimate %.2f m (%s)", (double)n.altitude,
           n.altitude_valid ? "ToF" : "no source");
    if (h.altitude_held) {
        printf("; held at %.2f m", (double)h.altitude_m);
    } else {
        printf("; not held");
    }
    printf("\nsurge: %.1f gf\n", (double)(h.surge_n / HOLD_GF_N));
    printf("output: fx %.2f gf  fz %.2f gf  tz %.3f mN m;  throttle FL %.3f RL %.3f FR %.3f RR %.3f%s\n",
           (double)(n.out.fx / HOLD_GF_N), (double)(n.out.fz / HOLD_GF_N), (double)(n.out.tz * 1000.0f),
           (double)n.throttle[MOTOR_FL], (double)n.throttle[MOTOR_RL], (double)n.throttle[MOTOR_FR],
           (double)n.throttle[MOTOR_RR], n.act.clamped_yaw ? "  (yaw clamped)" : "");
    if (n.link_seen) {
        printf("ground link: last packet %" PRIu32 " ms ago; core/ zeroes the hold after %.1f s of silence\n",
               n.link_age_ms, (double)s.b.params.link_timeout_s);
    } else {
        printf("ground link: no packet yet; the hold needs a ground tool sending (blimp_link.py status)\n");
    }
    printf("gains: yaw.kp %.4g N m/rad, yaw.kd %.4g N m s/rad, alt.kp %.4g N/m, alt.kd %.4g N s/m "
           "('param yaw.' and 'param alt.' to change)\n",
           (double)s.params.yaw.kp, (double)s.params.yaw.kd, (double)s.params.alt.kp,
           (double)s.params.alt.kd);
}

static int hold_usage(void)
{
    printf("hold                     the state of the bench hold\n"
           "hold heading <deg>|off   hold a heading, + counter-clockwise from above, from the nose at arming\n"
           "hold altitude <m>|off    hold a height above the floor (ToF), %.1f to %.1f m\n"
           "hold surge <gf>|off      a constant forward push, 0 to %.0f gf\n"
           "hold follow [off]        turn toward the forward camera's target (overrides heading)\n"
           "hold off                 end it: the motors coast and go back to the link and the console\n"
           "hold kill                coast now and latch KILLED, like a STOP from the link\n",
           (double)HOLD_ALT_MIN_M, (double)HOLD_ALT_MAX_M, (double)HOLD_SURGE_MAX_GF);
    return 1;
}

static int do_hold_cmd(int argc, char **argv)
{
    if (s.task == NULL) {
        printf("control task not running - see the boot log for why\n");
        return 1;
    }
    if (argc == 1) {
        print_hold();
        return 0;
    }
    const char *what = argv[1];
    if (strcasecmp(what, "off") == 0 && argc == 2) {
        portENTER_CRITICAL(&s.mux);
        s.want_hold = false;
        s.hold_req = (blimp_hold_t){0};
        portEXIT_CRITICAL(&s.mux);
        printf("hold off: the motors coast and go back to the link and the console\n");
        return 0;
    }
    if (strcasecmp(what, "kill") == 0 && argc == 2) {
        motors_coast();         /* the stop hook latches KILLED if the loop holds them */
        printf("all motors coasting%s\n", motors_loop_owns() ? "; the hold is KILLED until 'hold off'" : "");
        return 0;
    }
    /* 'hold follow' takes no value ('hold follow off' ends it); the rest take one. */
    const bool follow = strcasecmp(what, "follow") == 0;
    if (follow ? (argc > 3) : (argc != 3)) {
        return hold_usage();
    }

    const char *val = argc == 3 ? argv[2] : "on";
    const bool off = strcasecmp(val, "off") == 0;
    if (follow && !off && strcasecmp(val, "on") != 0) {
        return hold_usage();
    }
    float v = 0.0f;
    if (!off && !follow) {
        char *end = NULL;
        v = strtof(val, &end);
        if (end == val || *end != '\0' || !isfinite(v)) {
            printf("'%s' is not a number\n", val);
            return 1;
        }
    }

    hold_snap_t n;
    blimp_hold_t req;
    bool want;
    portENTER_CRITICAL(&s.mux);
    n = s.snap;
    req = s.hold_req;
    want = s.want_hold;
    portEXIT_CRITICAL(&s.mux);

    if (n.mode == CTL_MODE_KILLED) {
        printf("refused: KILLED by a stop; 'hold off' first, then start again\n");
        return 1;
    }

    if (strcasecmp(what, "heading") == 0) {
        req.heading_held = !off;
        req.heading_rad  = v / HOLD_RAD_DEG;
    } else if (strcasecmp(what, "altitude") == 0) {
        if (!off && (v < HOLD_ALT_MIN_M || v > HOLD_ALT_MAX_M)) {
            printf("refused: altitude %.2f m is outside %.1f..%.1f m\n", (double)v,
                   (double)HOLD_ALT_MIN_M, (double)HOLD_ALT_MAX_M);
            return 1;
        }
        req.altitude_held = !off;
        req.altitude_m    = v;
    } else if (strcasecmp(what, "surge") == 0) {
        if (!off && (v < 0.0f || v > HOLD_SURGE_MAX_GF)) {
            printf("refused: surge %.1f gf is outside 0..%.0f gf\n", (double)v, (double)HOLD_SURGE_MAX_GF);
            return 1;
        }
        req.surge_n = off ? 0.0f : v * HOLD_GF_N;
    } else if (follow) {
        if (!off) {
            /* Following with no camera would hold nothing: say so now rather
               than leave a hold that never turns. */
            cam_link_status_t cs;
            cam_link_status(&cs);
            const int32_t age_us = (int32_t)((uint32_t)esp_timer_get_time() - cs.last_row_us);
            if (cs.last_row_us == 0u || age_us > (int32_t)(HOLD_CAM_FRESH_MS * 1000u)) {
                printf("refused: no row from the forward camera in the last %u ms; 'cam' shows the link\n",
                       (unsigned)HOLD_CAM_FRESH_MS);
                return 1;
            }
        }
        req.target_held = !off;
    } else {
        return hold_usage();
    }

    const bool starting = !want && !off;
    if (starting) {
        if (n.converging) {
            printf("refused: the estimator needs the gondola still for 60 s before it arms "
                   "(%.0f s so far)\n", (double)n.still_s);
            return 1;
        }
        for (int m = 0; m < MOTOR_COUNT; m++) {
            if (motor_get((motor_id_t)m) != 0.0f) {
                printf("refused: motor %s is driven (%+.3f); stop it first\n",
                       motor_name((motor_id_t)m), (double)motor_get((motor_id_t)m));
                return 1;
            }
        }
        if (!n.link_seen || n.link_age_ms > HOLD_LINK_FRESH_MS) {
            printf("refused: no packet from the ground in the last %u ms; the hold runs only while a "
                   "ground tool keeps the link alive (blimp_link.py status, the ground station)\n",
                   (unsigned)HOLD_LINK_FRESH_MS);
            return 1;
        }
    }

    portENTER_CRITICAL(&s.mux);
    s.hold_req = req;
    if (starting) {
        s.want_hold = true;
    }
    portEXIT_CRITICAL(&s.mux);

    if (starting) {
        printf("hold starting: armed at the next tick, heading 0 = where the nose points now\n");
    }
    printf("held: heading %s, altitude %s, surge %.1f gf, follow %s\n",
           req.heading_held ? "on" : "off", req.altitude_held ? "on" : "off",
           (double)(req.surge_n / HOLD_GF_N), req.target_held ? "on" : "off");
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

    const esp_console_cmd_t hold = {
        .command = "hold",
        .help = "The bench hold: core/'s loops on a commanded heading, altitude or surge, or "
                "turning toward the forward camera's target. 'hold' alone shows the state; "
                "'hold heading 30', 'hold follow', 'hold off', 'hold kill'",
        .hint = "[heading|altitude|surge <value>|off] | follow [off] | off | kill",
        .func = &do_hold_cmd,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&hold));
}
