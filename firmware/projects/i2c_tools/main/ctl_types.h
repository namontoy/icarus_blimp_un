/*
 * ctl_types.h - the typed records that flow through the control pipeline.
 *
 *   read sensors -> estimate -> control -> mixer -> saturate -> pwm_out -> log
 *   sensors_raw_t   state_est_t  ctrl_out_t  motor_cmd_t  (throttle)      log_record_t
 *                   setpoint_t
 *
 * These layouts are the contract between increments: later work fills the
 * stages in, it does not restructure them. Where a struct has a counterpart
 * in core/ (core/include/blimp/types.h) it mirrors that counterpart field for
 * field, so that the day the middle stages become one blimp_step() call the
 * firmware's job is a copy, not a conversion. core/ is not edited here and
 * its headers are not included yet: that happens when blimp_step() is wired
 * in, together with the core component.
 *
 * Frame and units follow core/: body x forward, y left, z up, yaw positive
 * counter-clockwise seen from above; SI (m, m/s^2, rad/s, N, N.m, s) after the
 * estimate stage, raw counts before it. Motors are FL, RL, FR, RR in
 * motor_id_t order (cmd_motor.h), which is also the order on the wire.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "cmd_motor.h"      /* motor_id_t, MOTOR_COUNT */

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------- modes */

/*
 * MANUAL      the link's MOTOR_SET goes straight to the motors, bypassing the
 *             mixer, so the bench behaves as it did before the loop existed.
 *             Link loss zeroes the motors.
 * OPEN_LOOP   a scripted wrench (not written yet): outputs zero until it is.
 * CLOSED_LOOP the autonomous controller (blimp_step() later): zero until then.
 * KILLED      motors off, latched until an explicit re-arm (MODE_SET or the
 *             'ctl' command). MOTOR_STOP over the link and the battery cutoff
 *             land here. Rule 1 forbids piloting once a run starts, so the
 *             autonomous modes do not stop on link loss; only this does.
 */
typedef enum {
    CTL_MODE_MANUAL      = 0,
    CTL_MODE_OPEN_LOOP   = 1,
    CTL_MODE_CLOSED_LOOP = 2,
    CTL_MODE_KILLED      = 3,
} ctl_mode_t;

/* Status and latch bits, one byte, as logged (log_record_t.fs_flags) and
   sent in the CONTROL packet. The latches stay set until re-armed. */
#define CTL_FS_LINK_LOST    (1u << 0)   /* MANUAL: no valid packet within CONFIG_NET_LINK_TIMEOUT_MS */
#define CTL_FS_CMD_STALE    (1u << 1)   /* MANUAL: last MOTOR_SET older than CONFIG_MOTOR_FAILSAFE_MS */
#define CTL_FS_KILLED       (1u << 2)   /* latched: MOTOR_STOP, 'ctl --kill' or a cutoff */
#define CTL_FS_BATT_LOW     (1u << 3)   /* latched: battery below the cutoff for the hold time */
#define CTL_FS_SUSPENDED    (1u << 4)   /* the bus is released to a console command */
#define CTL_FS_CLAMP_TOTAL  (1u << 5)   /* Rule 13 scaling engaged this tick */
#define CTL_FS_OVERRUN      (1u << 6)   /* this tick ran past its period */
#define CTL_FS_MOTOR_FS     (1u << 7)   /* the motor driver's own failsafe reports TRIPPED */

/* Which measurements were updated this tick (log_record_t.fresh). */
#define CTL_FRESH_IMU       (1u << 0)
#define CTL_FRESH_MAG       (1u << 1)
#define CTL_FRESH_BARO      (1u << 2)
#define CTL_FRESH_TOF       (1u << 3)
#define CTL_FRESH_CAM0      (1u << 4)
#define CTL_FRESH_CAM1      (1u << 5)
#define CTL_FRESH_BAT       (1u << 6)
#define CTL_FRESH_LINK      (1u << 7)   /* a valid packet arrived within the link timeout */

/* --------------------------------------------------------------- cameras */

/*
 * Two ESP32-CAMs are planned over UART (docs/inter-board-link-plan.md): a
 * forward camera that detects the balloon and a downward one for optical
 * flow. Neither is wired on this branch. The mailboxes exist now so that the
 * loop's contract does not change when they are: a future UART task writes
 * one cam_result_t per frame (12.5 fps, 80 ms) with xQueueOverwrite(), the
 * loop peeks it every tick and marks it fresh when frame_seq moved. Peeking
 * every tick rather than every fourth keeps the latency at one tick, not up
 * to 60 ms.
 */
#define CTL_CAM_COUNT 2
enum { CTL_CAM_FORWARD = 0, CTL_CAM_DOWN = 1 };

typedef struct {
    uint32_t frame_seq;     /* increments per processed frame; fresh when it changes */
    uint32_t t_us;          /* S3 clock at receipt (the CAM's own clock is not ours) */
    uint16_t cx_px, cy_px;  /* centroid of the detection, pixels, origin top-left */
    uint16_t area_px;       /* detection area, pixels                              */
    uint16_t width_px;      /* frame size, so a consumer can normalise             */
    uint16_t height_px;
    uint8_t  detected;      /* 1: a target was found in this frame                 */
} cam_result_t;

/* ---------------------------------------------------------- raw sensors */

/*
 * Everything the read stage produced, in counts, with one fresh flag and the
 * time of the last update per source. Mirrors blimp_imu_sample_t's per-field
 * validity idea (core/include/blimp/types.h) one stage earlier, before units.
 * A failed transaction keeps the last value and clears the flag: a stale
 * number with a cleared flag is honest, a zero would look like a reading.
 */
typedef struct {
    uint32_t t_us;                      /* this tick's time                          */

    /* MPU-6050, one 14-byte burst: chip axes, the ranges set at attach */
    int16_t  acc[3];
    int16_t  gyro[3];
    int16_t  imu_temp;
    uint32_t imu_t_us;
    bool     imu_fresh;

    /* HMC5883L or QMC5883L, chip axes (the driver already reorders the
       HMC5883L's X-Z-Y register order into X, Y, Z) */
    int16_t  mag[3];
    uint32_t mag_t_us;
    bool     mag_fresh;

    /* Barometer, compensated by the driver with the chip's own trimming
       values (Bosch's integer algorithm) and kept as fixed-point integers:
       0.1 Pa, 0.01 C, 0.01 %RH (BME280 only, else 0). Not counts, unlike the
       rest: a count means nothing without the trims, and the compensation
       is the chip's, not ours (log_record_t explains). */
    int32_t  baro_p_dpa;
    int32_t  baro_t_cc;
    uint16_t baro_h_cpct;
    uint32_t baro_t_us;
    bool     baro_fresh;

    /* VL53L1X: distance and ST's range status (0 = valid) */
    uint16_t tof_mm;
    uint8_t  tof_status;
    uint32_t tof_t_us;
    bool     tof_fresh;

    /* the camera mailboxes as last peeked */
    cam_result_t cam[CTL_CAM_COUNT];
    bool         cam_fresh[CTL_CAM_COUNT];

    /* battery divider on CONFIG_BATTERY_ADC_GPIO (GPIO1, ADC1_CH0, by
       default; cmd_battery.c): raw count, calibrated millivolts at the pin
       before the divider ratio, and the running average after it */
    uint16_t bat_raw;
    uint16_t bat_pin_mv;
    uint16_t bat_mv;
    bool     bat_fresh;
} sensors_raw_t;

/* --------------------------------------------------------- estimate out */

/*
 * The first block mirrors blimp_imu_sample_t (core/include/blimp/types.h)
 * field for field: SI, body frame, one valid flag per source. The mounting
 * rotation of each breakout is applied here, so everything downstream is in
 * the body frame. The second block carries what the SENSORS packet and the
 * console also want to show and core/ does not consume.
 */
typedef struct {
    /* -- blimp_imu_sample_t mirror -------------------------------------- */
    float    gyro[3];       /* rad/s, body                                   */
    float    accel[3];      /* m/s^2, body, gravity included                 */
    float    mag[3];        /* uT, body; direction is what matters           */
    float    baro_alt;      /* m, standard atmosphere from pressure          */
    float    range_alt;     /* m, ToF to the floor, tilt-corrected h = d cos(theta) */
    uint32_t t_us;
    bool     gyro_valid, accel_valid, mag_valid, baro_valid, range_valid;

    /* -- extras ----------------------------------------------------------- */
    float    roll, pitch;   /* rad, from the accelerometer, for the tilt correction;
                             * no filtering yet (estimate() is pass-through)   */
    float    pressure_pa;
    float    air_temp_c;    /* barometer die temperature                     */
    float    humidity_pct;  /* BME280 only, else 0                           */
    float    imu_temp_c;
    float    bat_v;         /* running average, after the divider            */
} state_est_t;

/* -------------------------------------------------------------- setpoint */

/* Where a standing command came from; the higher wins (the commander idea
   from esp-drone, re-expressed: see ctl_task.c). */
typedef enum {
    CTL_SRC_NONE    = 0,
    CTL_SRC_LINK    = 1,    /* MOTOR_SET over UDP; expires with the link       */
    CTL_SRC_CONSOLE = 2,    /* 'motor' typed at a console; deliberate, held   */
} ctl_src_t;

typedef struct {
    ctl_mode_t mode;                 /* the requested mode                    */
    float      cmd[MOTOR_COUNT];     /* MANUAL: per-motor command, -1..1, FL RL FR RR */
    ctl_src_t  cmd_src;
    uint32_t   cmd_t_us;             /* when cmd last arrived (0: never)       */
    uint32_t   link_rx_t_us;         /* when the last valid packet of any type arrived */
    bool       brake;                /* KILLED: hold the bridges low instead of coasting */
} setpoint_t;

/* ------------------------------------------------------------ control out */

/* Body wrench, the mirror of blimp_wrench_t: fy is not realisable on this
   airframe and is kept at 0 for layout parity. Roll is a static trim
   parameter, not a wrench component. */
typedef struct {
    float fx;   /* N,   surge, +forward          */
    float fy;   /* N,   lateral, always 0        */
    float fz;   /* N,   heave, +up               */
    float tz;   /* N.m, yaw, +counter-clockwise  */
} ctrl_out_t;

/* -------------------------------------------------------------- motor cmd */

/*
 * Per-motor thrust in N (the mirror of blimp_actuators_t's thrust_* fields,
 * in motor_id_t order FL, RL, FR, RR) as the mixer emits it, then what
 * saturate() derives: signed throttles through the thrust map, clamped to
 * [-1, 1], scaled so that sum |T_i| stays within the Rule 13 cap, with the
 * clamp flags that say which limit bound this tick. Log them; never absorb
 * them silently.
 */
typedef struct {
    float   thrust_n[MOTOR_COUNT];
    float   throttle[MOTOR_COUNT];
    bool    clamped_total;          /* the Rule 13 scaling engaged           */
    uint8_t clamped_mask;           /* bit m: motor m hit |throttle| = 1     */
} motor_cmd_t;

/* ------------------------------------------------------------- log record */

/*
 * One record per tick, 72 bytes, packed and little-endian, straight into the
 * PSRAM capture (capture.c) and out through the link. Counts, not units, so
 * the decoder applies the same scale factors as estimate() and a bug there
 * is visible in the log instead of baked into it. The one exception is the
 * barometer: its counts are useless without the chip's trimming values and
 * the compensation is Bosch's fixed algorithm, not ours, so the record keeps
 * the compensated values as integers at a resolution below the sensor's
 * noise (0.1 Pa is about 8 mm of altitude). The size is asserted below: the
 * capture and the decoder (software/blimp_log.py) both depend on it.
 *
 * 72 rather than the 64-byte target (decided 2026-10-27, the byte map in
 * docs/control-loop-plan.md D7): the eight extra bytes keep the camera
 * centroid and area at pixel resolution, which a range-from-apparent-size
 * calibration needs from flight logs, and keep the humidity. The cost
 * is about one minute of a ten-minute buffer. The barometer's pressure and
 * temperature take three bytes each. The setpoint is the four link commands
 * in MANUAL (x 32767); otherwise (the bench hold, 2026-10-30) the wrench the
 * motors were given, fx and fz in 0.1 mN and tz in 10 uN m, and the heading
 * error the yaw loop ran on in mrad. mode says which. (The plan's whole-mN
 * units were too coarse for the bench: the rope rig turns on 1 mN m.)
 */
typedef struct __attribute__((packed)) {
    uint32_t t_us;
    int16_t  acc[3];
    int16_t  gyro[3];
    int16_t  mag[3];
    uint8_t  baro_p[3];             /* pressure, 0.1 Pa, 24-bit unsigned LE (to 1677 hPa) */
    uint8_t  baro_t[3];             /* temperature, 0.01 C, 24-bit signed LE */
    uint16_t baro_h;                /* humidity, 0.01 %RH, BME280 only, else 0 */
    uint16_t tof_mm;
    uint8_t  tof_status;
    uint8_t  fresh;                 /* CTL_FRESH_*                           */
    uint8_t  mode;                  /* ctl_mode_t                            */
    uint8_t  fs_flags;              /* CTL_FS_*                              */
    struct __attribute__((packed)) {
        uint8_t  seq;               /* frame_seq & 0xFF                      */
        uint8_t  flags;             /* bit 0: detected                       */
        uint16_t cx, cy;            /* centroid, pixels                      */
        uint16_t area;              /* pixels, saturating at 65535           */
    } cam[CTL_CAM_COUNT];
    int16_t  setpoint[4];
    int16_t  throttle[MOTOR_COUNT]; /* x 32767                               */
    uint16_t bat_mv;
    uint16_t exec_us;               /* this tick's execution time            */
} log_record_t;

_Static_assert(sizeof(log_record_t) == 72, "log_record_t must stay 72 bytes");

#ifdef __cplusplus
}
#endif
