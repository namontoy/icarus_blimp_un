/* params.c - default tuning and rulebook validation.
 *
 * IMPORTANT ON THE DEFAULT GAINS
 *
 * docs/03 section 3.2 lists reference gains lifted from mochi-swarm-sim's
 * preferences.py (KPZ 0.65, KPYAW 1.0, ...). Those are in the reference
 * simulator's NORMALISED units, not SI, and were tuned on a tilting
 * bicopter. Transplanting them into a controller whose errors are metres
 * and radians and whose outputs are newtons and newton-metres would be
 * dimensionally meaningless.
 *
 * So the defaults below are derived instead from this vehicle's own physics
 * (README "Why small is fast", docs/05 section 3): place each loop at a chosen natural
 * frequency for the estimated effective inertia, and damp it critically.
 *
 *   altitude:  kp = m_eff * wn^2,  kd = 2 * zeta * wn * m_eff
 *   heading:   kp = I_z  * wn^2,   kd = 2 * zeta * wn * I_z
 *
 * with m_eff ~ 0.334 kg and I_z ~ 0.0083 kg*m^2 for the 0.78 m / 0.25 m^3
 * reference sphere at Bogota density, wn = 2 rad/s and zeta = 1.
 *
 * These are a STARTING POINT FOR BRING-UP, not a prediction. I_z in
 * particular has no measured analogue anywhere in the corpus - it is an
 * order-of-magnitude placeholder until system-ID experiment 3 produces a
 * real number. Re-tune per docs/07 section 7, one loop at a time.
 *
 * MOTOR AND GEOMETRY NUMBERS ARE PLACEHOLDERS. motor_max_thrust_n in
 * particular MUST come from a thrust-stand measurement before any flight:
 * it is both the Rule 13 compliance basis and motor.c's PWM normalisation.
 */
#include "blimp/params.h"

#include <math.h>

/* Reference-vehicle inertia estimates; see header comment. */
#define M_EFF_KG      0.334f
#define IZ_KGM2       0.0083f
#define WN_ALT_RADS   2.0f
#define WN_YAW_RADS   2.0f
#define ZETA          1.0f

/* Airframe placeholders - MEASURE ON THE REAL VEHICLE. */
#define FRONT_HALF_SPAN_M   0.10f   /* half the front motor crossbar        */
#define CAM_DZ_M           -0.35f   /* camera below the envelope centre     */
#define MOTOR_MAX_GF        35.0f   /* one 8.5x20 coreless + 65 mm prop,
                                     * order of magnitude only              */
#define HEAVE_BUDGET_GF     30.0f   /* rear pair cap, leaves >= 70 gf front;
                                     * climb accel ~0.7 m/s^2 - descent is
                                     * only the trim, so do not overshoot */
#define HOVER_TRIM_GF        6.0f   /* flown this much heavy (Rule 11 allows
                                     * 10); rear pair holds it up            */

void blimp_params_defaults(blimp_params_t *p)
{
    if (!p) {
        return;
    }

    /* ---- geometry: PLACEHOLDERS, measure on the real vehicle ---------- */
    p->front_half_span_m = FRONT_HALF_SPAN_M;
    p->cam_dz_m          = CAM_DZ_M;

    /* ---- thrust budget (Rule 13) and per-motor limits ------------------ */
    p->max_total_thrust_n = BLIMP_RULE13_MAX_TOTAL_N;
    p->motor_max_thrust_n = BLIMP_GF_TO_N(MOTOR_MAX_GF);
    p->heave_budget_n     = BLIMP_GF_TO_N(HEAVE_BUDGET_GF);
    p->hover_thrust_n     = BLIMP_GF_TO_N(HOVER_TRIM_GF);

    /* ---- altitude loop: metres -> newtons (plus hover feed-forward) ----
     * The output clamp here is nominal; blimp.c narrows it every tick to
     * the rear pair's real range around the feed-forward, so anti-windup
     * sees the actuator limit. The derivative acts on whatever altitude
     * source exists - on this PCB that is the vision-derived height error,
     * which is noisy, hence the low derivative cutoff. */
    p->alt.kp       = M_EFF_KG * WN_ALT_RADS * WN_ALT_RADS;      /* ~1.34  */
    p->alt.kd       = 2.0f * ZETA * WN_ALT_RADS * M_EFF_KG;      /* ~1.34  */
    p->alt.ki       = 0.0f;   /* start at zero - docs/03 section 3.3        */
    p->alt.i_min    = -0.06f; /* trim can be off either way                 */
    p->alt.i_max    =  0.15f;
    p->alt.out_min  = -BLIMP_RULE13_MAX_TOTAL_N;
    p->alt.out_max  =  BLIMP_RULE13_MAX_TOTAL_N;
    p->alt.d_lpf_hz = 2.0f;

    /* ---- heading loop: radians -> newton-metres -------------------------
     * kd is applied by blimp.c to the GYRO yaw rate (blimp_pid_step_d), not
     * to the derivative of the vision bearing, which is far too noisy to
     * differentiate. Max achievable yaw torque = motor_max * l_f. */
    p->yaw.kp       = IZ_KGM2 * WN_YAW_RADS * WN_YAW_RADS;       /* ~0.033 */
    p->yaw.kd       = 2.0f * ZETA * WN_YAW_RADS * IZ_KGM2;       /* ~0.033 */
    p->yaw.ki       = 0.0f;
    p->yaw.i_min    = -0.010f;
    p->yaw.i_max    =  0.010f;
    p->yaw.out_min  = -p->motor_max_thrust_n * p->front_half_span_m;
    p->yaw.out_max  =  p->motor_max_thrust_n * p->front_half_span_m;
    p->yaw.d_lpf_hz = 8.0f;

    /* ---- surge loop: range error (m) -> newtons, >= 0 -------------------
     * P only. There is no brake, so the arrival speed is set by how much
     * thrust we allow on the way in, not by a closing-speed loop; and the
     * vision range is too noisy to differentiate. out_max is the cruise
     * cap - raise it for faster legs, lower it for gentler touches. */
    p->surge.kp       = 0.30f;
    p->surge.kd       = 0.0f;
    p->surge.ki       = 0.0f;
    p->surge.i_min    = 0.0f;
    p->surge.i_max    = 0.0f;
    p->surge.out_min  = 0.0f;
    p->surge.out_max  = BLIMP_GF_TO_N(30.0f);
    p->surge.d_lpf_hz = 5.0f;

    /* Pendulum swing damping starts DISABLED: nothing on this airframe
     * actuates pitch directly, and an ill-tuned rate term excites the swing
     * mode it is meant to damp (docs/03 section 3.1). */
    p->pitch_damp_kd = 0.0f;

    /* ---- estimator ------------------------------------------------------ */
    p->comp_alpha_attitude = 0.98f;
    p->comp_alpha_alt      = 0.95f;
    /* Bogota magnetic declination is roughly -6.5 deg (west). VERIFY against
     * a current WMM/IGRF lookup for the venue before the event - only
     * matters once a magnetometer is actually fitted. */
    p->mag_declination_rad = -0.113f;
    p->use_mag             = false;  /* MPU-6050 only on this PCB           */
    p->gyro_bias_tau_s     = 20.0f;

    /* ---- guidance -------------------------------------------------------- */
    /* Contact happens when the two bodies touch, not when centres coincide:
     * waypoint balloon radius ~0.275 m (Rule 21, 50-60 cm inflated) plus our
     * own envelope radius ~0.39 m gives ~0.66 m centre-to-centre. Slightly
     * inside that, so the approach commits rather than stalling. */
    p->wp_touch_radius_m    = 0.60f;
    p->wp_approach_speed_ms = 0.40f;
    p->wp_align_tol_rad     = 0.17f;   /* ~10 deg                            */
    p->touch_confirm_s      = 0.30f;
    p->search_yaw_rate      = 0.50f;
    p->miss_timeout_s       = 8.0f;
    /* Rule 4 makes everything before the first touch untimed, so this is
     * deliberately generous. Rushing PREPARE buys nothing. */
    p->prepare_settle_s     = 5.0f;

    /* ---- failsafe --------------------------------------------------------- */
    p->link_timeout_s = 1.0f;
    p->max_altitude_m = 3.5f;
}

/* Clamp anything that would break a rule or produce undefined behaviour.
 * Returns false if a correction was needed. */
bool blimp_params_validate(blimp_params_t *p)
{
    bool ok = true;

    if (!p) {
        return false;
    }

    /* Rule 13 is absolute. A live-tuning packet must never raise this. */
    if (!isfinite(p->max_total_thrust_n) || p->max_total_thrust_n <= 0.0f) {
        p->max_total_thrust_n = BLIMP_RULE13_MAX_TOTAL_N;
        ok = false;
    }
    if (p->max_total_thrust_n > BLIMP_RULE13_MAX_TOTAL_N) {
        p->max_total_thrust_n = BLIMP_RULE13_MAX_TOTAL_N;
        ok = false;
    }

    /* Per-motor cap: positive, and no single motor may be allowed more than
     * the whole budget. */
    if (!isfinite(p->motor_max_thrust_n) || p->motor_max_thrust_n <= 0.0f) {
        p->motor_max_thrust_n = BLIMP_GF_TO_N(MOTOR_MAX_GF);
        ok = false;
    }
    if (p->motor_max_thrust_n > p->max_total_thrust_n) {
        p->motor_max_thrust_n = p->max_total_thrust_n;
        ok = false;
    }

    /* The rear pair must always leave usable authority for surge and yaw. */
    if (!isfinite(p->heave_budget_n) || p->heave_budget_n < 0.0f) {
        p->heave_budget_n = BLIMP_GF_TO_N(HEAVE_BUDGET_GF);
        ok = false;
    }
    if (p->heave_budget_n > p->max_total_thrust_n * 0.6f) {
        p->heave_budget_n = p->max_total_thrust_n * 0.6f;
        ok = false;
    }

    /* Hover feed-forward = the trim weight. Rule 11 caps the trim at 10 gf,
     * so a larger feed-forward implies an illegally heavy vehicle - clamp
     * it, and it may never exceed the heave budget either. */
    if (!isfinite(p->hover_thrust_n) || p->hover_thrust_n < 0.0f) {
        p->hover_thrust_n = 0.0f;
        ok = false;
    }
    if (p->hover_thrust_n > BLIMP_RULE11_TRIM_TOL_N) {
        p->hover_thrust_n = BLIMP_RULE11_TRIM_TOL_N;
        ok = false;
    }
    if (p->hover_thrust_n > p->heave_budget_n) {
        p->hover_thrust_n = p->heave_budget_n;
        ok = false;
    }

    if (!isfinite(p->front_half_span_m) || p->front_half_span_m <= 1e-4f) {
        p->front_half_span_m = FRONT_HALF_SPAN_M;
        ok = false;
    }
    if (!isfinite(p->cam_dz_m)) {
        p->cam_dz_m = CAM_DZ_M;
        ok = false;
    }

    if (!isfinite(p->wp_touch_radius_m) || p->wp_touch_radius_m <= 0.0f) {
        p->wp_touch_radius_m = 0.60f;
        ok = false;
    }
    if (!isfinite(p->link_timeout_s) || p->link_timeout_s <= 0.0f) {
        p->link_timeout_s = 1.0f;
        ok = false;
    }

    return ok;
}
