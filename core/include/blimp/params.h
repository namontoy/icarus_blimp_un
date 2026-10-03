/* blimp/params.h - every tunable in one struct.
 *
 * Rationale: one struct means live-tuning over telemetry, logging the exact
 * gains a flight ran with, and replaying a flight in sim bit-identically.
 * Nothing in core/ may hold a tunable as a file-scope constant.
 *
 * Starting values are derived from this vehicle's own physics (README
 * section 3, docs/05 section 3) - see params.c. They are a starting point
 * for bring-up, not a prediction. Re-tune per docs/07 section 7.
 */
#ifndef BLIMP_PARAMS_H
#define BLIMP_PARAMS_H

#include "blimp/types.h"

/* ---- Rulebook constants. Do not "tune" these. --------------------------- */
#define BLIMP_GF_TO_N(gf)            ((float)(gf) * 0.00980665f)
/* Rule 13: total thrust across ALL propulsion motors <= 100 gf. All four
 * motors on this vehicle are propulsion motors and count. */
#define BLIMP_RULE13_MAX_TOTAL_N     BLIMP_GF_TO_N(100.0f)
/* Rule 11: net vertical force must stay within +/-10 gf of neutral. */
#define BLIMP_RULE11_TRIM_TOL_N      BLIMP_GF_TO_N(10.0f)
/* Rule 22: waypoint balloon centre height. */
#define BLIMP_RULE22_WP_HEIGHT_M     2.0f

typedef struct {
    float kp, ki, kd;
    float i_min, i_max;      /* asymmetric integral clamp (anti-windup)     */
    float out_min, out_max;  /* output clamp                                */
    float d_lpf_hz;          /* derivative low-pass cutoff; <=0 disables    */
} blimp_pid_params_t;

typedef struct {
    /* -- geometry: MEASURE THESE ON THE REAL VEHICLE --------------------- */
    float front_half_span_m; /* l_f: lateral offset of each front motor from
                              * the yaw axis, m. Yaw torque = l_f*(T_fr-T_fl).
                              * PLACEHOLDER 0.10 - measure on the gondola.   */
    float cam_dz_m;          /* camera height relative to the ENVELOPE CENTRE
                              * (the point that should meet the balloon), m,
                              * negative = below. Used by guidance.c's vision
                              * altitude fallback when no altitude sensor is
                              * fitted. PLACEHOLDER -0.35 - measure.         */

    /* -- thrust budget (Rule 13) and per-motor limits -------------------- */
    float max_total_thrust_n;   /* <= BLIMP_RULE13_MAX_TOTAL_N, hard-checked */
    float motor_max_thrust_n;   /* per motor, MEASURED on a thrust stand.
                                 * The mixer never commands more than this on
                                 * any one motor; the PWM curve in motor.c
                                 * maps [0, this] onto [0, 100 %] duty.      */
    float heave_budget_n;       /* cap on the rear-pair SUM. Whatever the
                                 * rear pair uses comes out of the front
                                 * pair's share of the Rule 13 budget.      */
    float hover_thrust_n;       /* feed-forward for the rear pair: the small
                                 * negative buoyancy the vehicle is trimmed
                                 * with (Rule 11: within 10 gf). Set from the
                                 * lift measurement on the day.             */

    /* -- control loops ---------------------------------------------------- */
    blimp_pid_params_t alt;      /* altitude error (m)   -> fz (N), + hover ff */
    blimp_pid_params_t yaw;      /* heading error (rad)  -> tz (N*m); kd acts
                                  * on the GYRO yaw rate, not on the noisy
                                  * vision bearing (blimp.c)               */
    blimp_pid_params_t surge;    /* range error (m)      -> fx (N), >= 0       */
    float pitch_damp_kd;         /* pendulum swing damping, rate-only       */

    /* -- estimator -------------------------------------------------------- */
    float comp_alpha_attitude;   /* gyro weight, complementary filter       */
    float comp_alpha_alt;
    float mag_declination_rad;   /* Bogota approx -6.5 deg; set from survey */
    bool  use_mag;               /* false on this PCB (MPU-6050 has no mag) */
    float gyro_bias_tau_s;       /* bias estimator time constant            */

    /* -- guidance ---------------------------------------------------------- */
    float wp_touch_radius_m;     /* declare contact inside this range       */
    float wp_approach_speed_ms;  /* target closing speed on final approach  */
    float wp_align_tol_rad;      /* bearing error before committing forward */
    float touch_confirm_s;       /* dwell before accepting a touch          */
    float search_yaw_rate;       /* rad/s while scanning for the next target*/
    float miss_timeout_s;        /* give up an approach and re-acquire      */
    float prepare_settle_s;      /* untimed pre-first-touch settle, Rule 4  */

    /* -- failsafe ---------------------------------------------------------- */
    float link_timeout_s;        /* no ground packet -> failsafe            */
    float max_altitude_m;        /* ceiling guard                           */
} blimp_params_t;

/* Fills p with the documented starting values. Never fails. */
void blimp_params_defaults(blimp_params_t *p);

/* Returns false (and clamps in place) if any param violates a rulebook
 * constraint - notably the Rule 13 total-thrust budget. Call after any
 * live-tuning update; never let an out-of-spec param reach the mixer. */
bool blimp_params_validate(blimp_params_t *p);

#endif /* BLIMP_PARAMS_H */
