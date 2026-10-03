/* blimp/types.h - shared value types for the portable control core.
 *
 * UNITS ARE SI AND NON-NEGOTIABLE:
 *   length m . mass kg . force N . torque N.m . angle rad . time s
 * Grams-force appear ONLY in the rulebook and in params.h conversion macros.
 *
 * This header (and everything in core/) must compile with zero ESP-IDF,
 * FreeRTOS, MuJoCo or libc-IO dependencies. Freestanding C11 + <math.h> only.
 */
#ifndef BLIMP_TYPES_H
#define BLIMP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/* Body frame (z-up, matching the airship literature):
 *   +x forward (surge, nose direction)
 *   +y left
 *   +z up   (heave)
 *   yaw +ve counter-clockwise seen from above
 *
 * THE AIRFRAME (confirmed 2026-09-21 from the professor's PCB and photos):
 * four fixed brushed coreless motors, each driven by a low-side N-MOSFET
 * from one GPIO. That means UNIDIRECTIONAL thrust: no reverse, no tilt
 * servo, no lateral thruster.
 *   front pair (FL, FR) -> thrust along +x. Sum = surge, difference = yaw.
 *   rear pair  (RL, RR) -> thrust along +z. Sum = heave, UP ONLY.
 * Nothing on this vehicle produces -x, -z or +/-y force. Braking is drag
 * only. Descent is the small negative buoyancy the vehicle is deliberately
 * trimmed with (Rule 11 allows +/-10 gf). Yaw always comes with some surge,
 * because the only way to turn is to run one front motor harder than the
 * other and neither can go below zero. */

/* M_PI is NOT standard C11 - it is a POSIX/MSVC extension gated behind
 * _USE_MATH_DEFINES. core/ must build clean under -std=c11 on both the host
 * (clang/MSVC) and xtensa-gcc, so we carry our own. */
#define BLIMP_PI      3.14159265358979323846f
#define BLIMP_TWO_PI  6.28318530717958647692f
#define BLIMP_HALF_PI 1.57079632679489661923f

typedef struct { float x, y, z; } blimp_vec3_t;

/* Estimated vehicle state, produced by the estimator, consumed by control. */
typedef struct {
    float    roll, pitch, yaw;   /* rad, estimated attitude                 */
    float    gyro[3];            /* rad/s, body, bias-corrected             */
    float    accel[3];           /* m/s^2, body, gravity included           */
    float    altitude;           /* m, above arena floor                    */
    float    vz;                 /* m/s, +up                                */
    float    yaw_rate;           /* rad/s, convenience = gyro[2]            */
    uint32_t t_us;               /* monotonic timestamp, microseconds       */
    bool     attitude_valid;     /* estimator has converged                 */
    bool     altitude_valid;     /* a usable altitude source is present     */
} blimp_state_t;

/* Raw sensor sample handed to the estimator. Any field may be absent;
 * the *_valid flags say which are fresh this tick.
 *
 * ON THIS PCB only gyro/accel exist (MPU-6050). mag/baro/range stay invalid
 * until a breakout is added on the I2C header - see docs/10. */
typedef struct {
    float    gyro[3];            /* rad/s body      */
    float    accel[3];           /* m/s^2 body      */
    float    mag[3];             /* normalised or uT, body; direction only  */
    float    baro_alt;           /* m, absolute-ish */
    float    range_alt;          /* m, ToF to floor, already tilt-corrected */
    uint32_t t_us;
    bool     gyro_valid, accel_valid, mag_valid, baro_valid, range_valid;
} blimp_imu_sample_t;

/* Control demand in body frame.
 *
 * fy is NOT REALISABLE on this airframe (see the frame note above). The
 * field is kept so the struct layout is stable across the FFI and so a
 * future lateral thruster has a channel; the mixer ignores it and guidance
 * always emits 0. Do not build control logic that relies on fy. */
typedef struct {
    float fx;   /* N,   surge   (+forward); only >= 0 is achievable        */
    float fy;   /* N,   lateral (+left);    NOT achievable, always ignored  */
    float fz;   /* N,   heave   (+up);      only >= 0 is achievable        */
    float tz;   /* N*m, yaw     (+CCW)                                     */
} blimp_wrench_t;

/* What the mixer emits; what the actuator drivers consume.
 * Every thrust is >= 0 by construction (mixer.h). */
typedef struct {
    float thrust_fl;      /* N, front-left  motor, pushes +x               */
    float thrust_fr;      /* N, front-right motor, pushes +x               */
    float thrust_rl;      /* N, rear-left   motor, pushes +z               */
    float thrust_rr;      /* N, rear-right  motor, pushes +z               */
    bool  clamped_total;  /* Rule 13 total-thrust clamp engaged            */
    bool  clamped_yaw;    /* commanded yaw torque was not fully realised   */
    bool  clamped_heave;  /* heave demand clipped: negative (cannot push
                           * down) or above the heave budget               */
} blimp_actuators_t;

/* A course waypoint: a tethered red balloon to be physically touched. */
typedef struct {
    float x, y;      /* m, arena frame, origin at arena centre */
    float z;         /* m, balloon centre height (2.0 per Rule 22) */
} blimp_waypoint_t;

#endif /* BLIMP_TYPES_H */
