/* estimator.c - attitude, heading and altitude estimation. See blimp/estimator.h
 * for the contract this implements and docs/07 section 5 for the design.
 *
 * ALGORITHM SUMMARY
 *
 * Attitude (roll/pitch): a complementary filter. Gyro (bias-corrected) is
 * integrated through the full nonlinear ZYX Euler kinematics every tick; the
 * accelerometer's gravity vector pulls the result back each tick by
 * (1 - p->comp_alpha_attitude). Per docs/07 5.1 / the header comment, this
 * vehicle is near-neutrally-buoyant and weakly actuated, so specific force is
 * small and the accelerometer is a clean, low-noise gravity reference most of
 * the time - the opposite of a quadcopter, where accel is noisy and gyro is
 * trusted almost exclusively. We still blend rather than snap-to-accel so a
 * momentary real acceleration (e.g. a thrust transient) does not corrupt
 * attitude in one tick.
 *
 * Heading (yaw): gyro integration (same Euler kinematics) corrected by the
 * magnetometer, tilt-compensated using the roll/pitch just computed above,
 * with p->mag_declination_rad applied, when p->use_mag and the sample passes
 * the health gate below. This is the single highest-leverage change versus
 * every MPU6050-only reference design in the corpus (docs/07 5.2) - yaw drift
 * is THE problem on a vehicle with only a handful of gf of differential
 * thrust to fight it.
 *
 * Gyro bias: a slow low-pass on the RAW gyro reading, run only while the
 * vehicle looks stationary (low gyro magnitude AND accel magnitude near g),
 * time constant p->gyro_bias_tau_s, same alpha = dt/(dt+tau) shape as
 * pid.c's derivative filter. e->stationary_s accumulates while stationary and
 * resets the instant it isn't.
 *
 * Altitude: a second complementary filter. Vertical acceleration (rotated
 * into world frame and gravity-cancelled) is integrated for both altitude and
 * vertical rate; ToF range (precise, range-limited) or barometer (absolute,
 * slow, noisy), whichever is fresher/better this tick, corrects both the
 * position AND, via the finite difference against the PREVIOUS fused
 * altitude, the rate - see decision 4 below for why range is preferred over
 * baro when both are present.
 *
 * DESIGN DECISIONS NOT FULLY PINNED DOWN BY THE CONTRACT (reported here per
 * the house rule of implementing to the header as written and flagging
 * anything it leaves open - see mixer.c for the precedent of this comment
 * style):
 *
 * 1. blimp_estimator_converging() takes only `const blimp_estimator_t *e` -
 *    no params pointer - so it cannot scale its "has bias settled" threshold
 *    against the live-tunable p->gyro_bias_tau_s that update() actually uses
 *    to shape the bias filter. There is no field in blimp_estimator_t to
 *    cache a params-derived threshold either. Implemented here as a fixed
 *    constant, BLIMP_EST_CONVERGE_S, requiring e->stationary_s to exceed
 *    ~3 time constants of the DOCUMENTED DEFAULT tau (20 s, params.c) i.e.
 *    60 s of continuous stationary dwell, for ~95% settling. If
 *    gyro_bias_tau_s is retuned far from its default, this threshold will
 *    not automatically track it. Fixing this properly needs either a params
 *    argument on blimp_estimator_converging() or a cached
 *    settle-threshold/converged field on blimp_estimator_t - a header change,
 *    which is out of scope here per house rule 4.
 *
 * 2. Mag health ("reject samples whose magnitude deviates implausibly from a
 *    running norm, or that jump too fast", per the task and docs/07 5.2 item
 *    3): blimp_estimator_t has no field to persist a running magnitude
 *    baseline or the previous mag sample, and core/ bans global mutable state
 *    (hard rule 1; it would also be unsound across the multiple estimator
 *    instances blimp.h says the sim may hold). Implemented instead as two
 *    checks that need no extra history:
 *      (a) a degenerate-magnitude-only sanity check (non-finite, zero or
 *          near-zero magnitude) - deliberately NOT an absolute expected-value
 *          band, because types.h documents mag as "normalised or uT" and a
 *          single fixed band cannot be right for both unit conventions;
 *      (b) an innovation gate: the tilt-compensated heading this sample
 *          implies is compared against the GYRO-PROPAGATED yaw already
 *          computed this tick (already resident in e->q, no extra state
 *          needed) and rejected if the two disagree by more than
 *          BLIMP_EST_MAG_INNOVATION_MAX_RAD. This is standard AHRS practice
 *          (reject a correction whose innovation is implausible for one
 *          tick's worth of gyro drift) and is unit-agnostic, since it
 *          compares a derived ANGLE, not a raw magnitude. It directly serves
 *          "jump too fast" and, for a sample corrupted enough to fail
 *          plausibility, usually serves "deviates from norm" too.
 *      Because (b) compares against the CURRENT yaw estimate, it would
 *      otherwise also block ever acquiring a heading from a fresh or
 *      deliberately-wrong initial guess (exactly the scenario the roll/pitch
 *      convergence test exercises, just on the yaw axis too). So the gate is
 *      only enforced once e->mag_healthy is already true (an established fix
 *      exists to protect); while it is false, a sample that only fails the
 *      innovation check (not the magnitude check) is treated as acquisition
 *      and snapped to outright rather than rejected.
 *
 * 3. There is one comp_alpha_attitude field, not a separate heading-blend
 *    gain. It is reused for the yaw/mag blend as well as roll/pitch/accel.
 *
 * 4. When both baro and range are valid the same tick, range is used and
 *    baro is ignored for that tick, rather than blending both. mixer.h has a
 *    precedent for this kind of unstated-in-the-header call (see mixer.c's
 *    sign_safe note). Rationale: types.h documents range as "already
 *    tilt-corrected" and docs/07 5.1 calls it "precise to ~cm" against
 *    baro's "~+/-0.25 m real accuracy" - there is no configured relative
 *    trust weighting for the two in blimp_params_t to do a principled blend,
 *    and preferring the far more precise source outright is simpler and
 *    behaves correctly on its own for both the ramp-tracking and
 *    range-dropout test scenarios (the dropout case falls through to the
 *    baro-only path with no special-casing).
 */
#include "blimp/estimator.h"

#include "blimp/pid.h"   /* blimp_wrap_pi */

#include <math.h>

/* ---- Local constants -------------------------------------------------- */

#define BLIMP_EST_MAX_DT_S                 2.0f

/* Altitude fix bookkeeping - see estimator.h's note on validity windows. */
#define BLIMP_EST_ALT_STALE_S              2.5f   /* accel dead-reckoning is
                                                   * not trustworthy for long,
                                                   * least of all through a
                                                   * balloon collision      */
#define BLIMP_EST_ALT_REACQ_S              0.5f
#define BLIMP_EST_GRAVITY_MPS2             9.80665f

/* "Looks stationary" gate for gyro bias estimation. Must sit comfortably
 * ABOVE plausible gyro bias magnitudes (a MEMS rate gyro can cold-start
 * several deg/s off zero) or a bias at/above the threshold could never be
 * recognised as "stationary" in the first place and so could never be
 * estimated - and comfortably BELOW real in-flight rates (search_yaw_rate
 * defaults to 0.5 rad/s, params.c) so genuine rotation is never mistaken for
 * stillness. */
#define BLIMP_EST_STATIONARY_GYRO_RADS      0.2f    /* ~11.5 deg/s          */
#define BLIMP_EST_STATIONARY_ACCEL_TOL_MPS2 1.0f    /* band around 1 g      */

/* See decision 1 above. */
#define BLIMP_EST_CONVERGE_S                60.0f

/* See decision 2 above. */
#define BLIMP_EST_MAG_NORM_MIN               1e-6f
#define BLIMP_EST_MAG_INNOVATION_MAX_RAD     1.2f   /* ~69 deg              */

/* Fallbacks when params are missing/non-finite; mirror params.c's own
 * defaults, same pattern as mixer.c's sanitize_params(). */
#define BLIMP_EST_DEFAULT_TAU_S              20.0f
#define BLIMP_EST_DEFAULT_ALPHA_ATT          0.98f
#define BLIMP_EST_DEFAULT_ALPHA_ALT          0.95f

/* Gimbal-lock guard for the pitch=+/-90deg singularity in the Euler
 * kinematics, same idiom as mixer.c's BLIMP_MIXER_COS_TILT_EPS. */
#define BLIMP_EST_COS_PITCH_EPS              0.02f

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static float cos_safe(float c, float eps)
{
    if (c >= 0.0f) {
        return (c > eps) ? c : eps;
    }
    return (c < -eps) ? c : -eps;
}

static bool finite3(const float v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

/* ---- Quaternion <-> ZYX Euler (yaw-pitch-roll), standard right-handed
 * conventions, body frame per types.h (+x fwd, +y left, +z up). ------------
 *
 * R = Rz(yaw) * Ry(pitch) * Rx(roll), body -> reference frame. The formulas
 * below are the standard closed forms for that composition (matching the
 * quaternion convention w,x,y,z used throughout this file). */

static void quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];

    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r10 = 2.0f * (x * y + w * z);
    const float r20 = 2.0f * (x * z - w * y);
    const float r21 = 2.0f * (y * z + w * x);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);

    *yaw   = atan2f(r10, r00);
    *pitch = asinf(clampf(-r20, -1.0f, 1.0f));
    *roll  = atan2f(r21, r22);
}

static void euler_to_quat(float roll, float pitch, float yaw, float q[4])
{
    const float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f),   sy = sinf(yaw * 0.5f);

    const float w = cr * cp * cy + sr * sp * sy;
    const float x = sr * cp * cy - cr * sp * sy;
    const float y = cr * sp * cy + sr * cp * sy;
    const float z = cr * cp * sy - sr * sp * cy;

    const float n = sqrtf(w * w + x * x + y * y + z * z);
    if (!(n > 1e-9f) || !isfinite(n)) {
        /* Fell out of a valid rotation (e.g. NaN in, NaN out) - identity is
         * the only fail-safe choice available. */
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
        return;
    }
    q[0] = w / n; q[1] = x / n; q[2] = y / n; q[3] = z / n;
}

/* ---- Sanitised params, same pattern as mixer.c's sanitize_params(). ---- */

typedef struct {
    float comp_alpha_attitude;
    float comp_alpha_alt;
    float mag_declination_rad;
    bool  use_mag;
    float gyro_bias_tau_s;
} blimp_est_safe_params_t;

static blimp_est_safe_params_t sanitize_params(const blimp_params_t *p)
{
    blimp_est_safe_params_t sp;

    sp.comp_alpha_attitude =
        (p && isfinite(p->comp_alpha_attitude))
            ? clampf(p->comp_alpha_attitude, 0.0f, 1.0f)
            : BLIMP_EST_DEFAULT_ALPHA_ATT;

    sp.comp_alpha_alt =
        (p && isfinite(p->comp_alpha_alt))
            ? clampf(p->comp_alpha_alt, 0.0f, 1.0f)
            : BLIMP_EST_DEFAULT_ALPHA_ALT;

    sp.mag_declination_rad =
        (p && isfinite(p->mag_declination_rad)) ? p->mag_declination_rad : 0.0f;

    sp.use_mag = p ? p->use_mag : true;

    sp.gyro_bias_tau_s =
        (p && isfinite(p->gyro_bias_tau_s) && p->gyro_bias_tau_s > 1e-3f)
            ? p->gyro_bias_tau_s
            : BLIMP_EST_DEFAULT_TAU_S;

    return sp;
}

/* ---- Output snapshot, shared by every return path in update(). -------- */

static void write_state(const blimp_estimator_t *e, const blimp_imu_sample_t *s,
                        blimp_state_t *out)
{
    float roll, pitch, yaw;
    quat_to_euler(e->q, &roll, &pitch, &yaw);
    out->roll  = roll;
    out->pitch = pitch;
    out->yaw   = yaw;

    const bool gyro_ok = s->gyro_valid && finite3(s->gyro);
    out->gyro[0] = gyro_ok ? s->gyro[0] - e->gyro_bias[0] : 0.0f;
    out->gyro[1] = gyro_ok ? s->gyro[1] - e->gyro_bias[1] : 0.0f;
    out->gyro[2] = gyro_ok ? s->gyro[2] - e->gyro_bias[2] : 0.0f;

    const bool accel_ok = s->accel_valid && finite3(s->accel);
    out->accel[0] = accel_ok ? s->accel[0] : 0.0f;
    out->accel[1] = accel_ok ? s->accel[1] : 0.0f;
    out->accel[2] = accel_ok ? s->accel[2] : 0.0f;

    out->altitude = e->alt;
    out->vz       = e->vz;
    out->yaw_rate = out->gyro[2];
    out->t_us     = s->t_us;

    out->attitude_valid = e->initialised;
    /* A window after the last absolute fix, not a per-tick flag. */
    const float fix_age_s = (float)(uint32_t)(s->t_us - e->last_abs_us) * 1e-6f;
    out->altitude_valid = e->abs_ever && isfinite(fix_age_s) &&
                          fix_age_s <= BLIMP_EST_ALT_STALE_S;
}

void blimp_estimator_init(blimp_estimator_t *e)
{
    if (!e) {
        return;
    }
    e->gyro_bias[0] = e->gyro_bias[1] = e->gyro_bias[2] = 0.0f;
    e->q[0] = 1.0f; e->q[1] = 0.0f; e->q[2] = 0.0f; e->q[3] = 0.0f;
    e->alt          = 0.0f;
    e->vz           = 0.0f;
    e->baro_ref     = 0.0f;
    e->last_us      = 0;
    e->initialised  = false;
    e->mag_healthy  = false;
    e->stationary_s = 0.0f;
    e->last_abs_us  = 0u;
    e->abs_ever     = false;
}

void blimp_estimator_zero(blimp_estimator_t *e, const blimp_imu_sample_t *s)
{
    if (!e) {
        return;
    }

    e->baro_ref = (s && s->baro_valid && isfinite(s->baro_alt)) ? s->baro_alt : 0.0f;

    /* Latch current heading as the yaw origin; keep roll/pitch as-is. */
    float roll, pitch, yaw;
    quat_to_euler(e->q, &roll, &pitch, &yaw);
    euler_to_quat(roll, pitch, 0.0f, e->q);

    e->alt = (s && s->range_valid && isfinite(s->range_alt)) ? s->range_alt : 0.0f;
    e->vz  = 0.0f;

    /* Fresh dwell count from this reference point; gyro_bias and
     * mag_healthy deliberately survive a zero() - PREPARE re-runs between
     * loop attempts (guidance.h BLIMP_GUID_LOOP_DONE) should not throw away
     * an already-converged bias estimate. */
    e->stationary_s = 0.0f;
}

bool blimp_estimator_converging(const blimp_estimator_t *e)
{
    if (!e) {
        return true;   /* fail safe: no estimator state -> block arming */
    }
    if (!e->initialised) {
        return true;
    }
    return e->stationary_s < BLIMP_EST_CONVERGE_S;
}

void blimp_estimator_update(blimp_estimator_t *e, const blimp_params_t *p,
                            const blimp_imu_sample_t *s, blimp_state_t *out)
{
    if (out) {
        *out = (blimp_state_t){0};
    } else {
        return;
    }
    if (!e || !s) {
        return;
    }

    const blimp_est_safe_params_t sp = sanitize_params(p);

    if (!e->initialised) {
        e->last_us     = s->t_us;
        e->initialised = true;
        write_state(e, s, out);
        return;
    }

    /* ---- dt: reject backwards, zero or implausibly large gaps rather than
     * integrating garbage. last_us always advances so a rejected tick does
     * not poison every dt computed after it. */
    const bool ts_advanced = s->t_us > e->last_us;
    float dt = 0.0f;
    bool dt_ok = false;
    if (ts_advanced) {
        dt = (float)(s->t_us - e->last_us) * 1.0e-6f;
        dt_ok = isfinite(dt) && dt > 0.0f && dt <= BLIMP_EST_MAX_DT_S;
    }
    e->last_us = s->t_us;

    if (!dt_ok) {
        write_state(e, s, out);
        return;
    }

    /* ---- current attitude, as Euler ------------------------------------ */
    float roll, pitch, yaw;
    quat_to_euler(e->q, &roll, &pitch, &yaw);

    /* ---- sanitised sensor snapshot -------------------------------------- */
    const bool gyro_ok  = s->gyro_valid  && finite3(s->gyro);
    const bool accel_ok = s->accel_valid && finite3(s->accel);
    const float gx = gyro_ok ? s->gyro[0] : 0.0f;
    const float gy = gyro_ok ? s->gyro[1] : 0.0f;
    const float gz = gyro_ok ? s->gyro[2] : 0.0f;
    const float ax = accel_ok ? s->accel[0] : 0.0f;
    const float ay = accel_ok ? s->accel[1] : 0.0f;
    const float az = accel_ok ? s->accel[2] : 0.0f;

    /* ---- stationary detection + slow gyro-bias estimate ---------------- */
    if (gyro_ok && accel_ok) {
        const float gyro_mag  = sqrtf(gx * gx + gy * gy + gz * gz);
        const float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
        const bool stationary =
            gyro_mag < BLIMP_EST_STATIONARY_GYRO_RADS &&
            fabsf(accel_mag - BLIMP_EST_GRAVITY_MPS2) < BLIMP_EST_STATIONARY_ACCEL_TOL_MPS2;

        if (stationary) {
            e->stationary_s += dt;
            /* Same alpha=dt/(dt+tau) shape as pid.c's derivative filter. */
            const float alpha_bias = dt / (dt + sp.gyro_bias_tau_s);
            e->gyro_bias[0] += alpha_bias * (gx - e->gyro_bias[0]);
            e->gyro_bias[1] += alpha_bias * (gy - e->gyro_bias[1]);
            e->gyro_bias[2] += alpha_bias * (gz - e->gyro_bias[2]);
        } else {
            e->stationary_s = 0.0f;
        }
    } else {
        e->stationary_s = 0.0f;   /* can't confirm stillness without both */
    }

    const float bx = gyro_ok ? gx - e->gyro_bias[0] : 0.0f;
    const float by = gyro_ok ? gy - e->gyro_bias[1] : 0.0f;
    const float bz = gyro_ok ? gz - e->gyro_bias[2] : 0.0f;

    /* ---- gyro integration: full ZYX Euler kinematics -------------------- */
    const float sr    = sinf(roll),  cr = cosf(roll);
    const float sp_a  = sinf(pitch);
    const float cp_a  = cos_safe(cosf(pitch), BLIMP_EST_COS_PITCH_EPS);
    const float tan_p = sp_a / cp_a;

    const float roll_dot  = bx + sr * tan_p * by + cr * tan_p * bz;
    const float pitch_dot = cr * by - sr * bz;
    const float yaw_dot   = (sr * by + cr * bz) / cp_a;

    const float roll_g  = roll  + roll_dot  * dt;
    const float pitch_g = pitch + pitch_dot * dt;
    const float yaw_g   = yaw   + yaw_dot   * dt;

    /* ---- accelerometer correction (roll/pitch) -------------------------- */
    float roll_out = roll_g, pitch_out = pitch_g;
    if (accel_ok) {
        const float accel_mag2 = ax * ax + ay * ay + az * az;
        if (accel_mag2 > 1.0e-6f) {
            const float roll_acc  = atan2f(ay, az);
            const float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));
            const float keep = sp.comp_alpha_attitude;
            roll_out  = roll_g  + (1.0f - keep) * blimp_wrap_pi(roll_acc  - roll_g);
            pitch_out = pitch_g + (1.0f - keep) * blimp_wrap_pi(pitch_acc - pitch_g);
        }
    }
    roll_out  = blimp_wrap_pi(roll_out);
    pitch_out = blimp_wrap_pi(pitch_out);

    /* ---- magnetometer correction (heading) ------------------------------ */
    float yaw_out = blimp_wrap_pi(yaw_g);
    if (s->mag_valid) {
        if (!finite3(s->mag)) {
            e->mag_healthy = false;   /* corrupt sample, explicitly unhealthy */
        } else {
            const float mx = s->mag[0], my = s->mag[1], mz = s->mag[2];
            const float mag_norm = sqrtf(mx * mx + my * my + mz * mz);
            if (!(mag_norm > BLIMP_EST_MAG_NORM_MIN) || !isfinite(mag_norm)) {
                e->mag_healthy = false;   /* degenerate magnitude */
            } else {
                /* Tilt-compensate with the roll/pitch just computed above -
                 * this is what makes heading survive the gondola swinging;
                 * an uncompensated atan2(-my, mx) is only valid dead level. */
                const float srr = sinf(roll_out),  crr = cosf(roll_out);
                const float spp = sinf(pitch_out), cpp = cosf(pitch_out);
                const float xh = cpp * mx + spp * srr * my + spp * crr * mz;
                const float yh = crr * my - srr * mz;
                const float heading_mag = atan2f(-yh, xh);
                const float yaw_from_mag =
                    blimp_wrap_pi(heading_mag + sp.mag_declination_rad);
                const float innovation = blimp_wrap_pi(yaw_from_mag - yaw_g);

                if (fabsf(innovation) <= BLIMP_EST_MAG_INNOVATION_MAX_RAD) {
                    e->mag_healthy = true;
                    if (sp.use_mag) {
                        yaw_out = blimp_wrap_pi(yaw_g + (1.0f - sp.comp_alpha_attitude) * innovation);
                    }
                } else if (!e->mag_healthy) {
                    /* No established heading fix yet (e.g. just after
                     * init()/zero()): this is ACQUISITION, not a disturbance
                     * - the same gate that must reject a later in-flight
                     * disturbance would otherwise permanently block ever
                     * bootstrapping from a fresh/wrong initial guess. Snap
                     * outright instead of blending. */
                    e->mag_healthy = true;
                    if (sp.use_mag) {
                        yaw_out = yaw_from_mag;
                    }
                } else {
                    e->mag_healthy = false;   /* implausible jump - decision 2 */
                }
            }
        }
    }
    /* mag_valid false this tick: leave e->mag_healthy at its last value -
     * a momentary dropout is not itself evidence of a bad sensor. */

    euler_to_quat(roll_out, pitch_out, yaw_out, e->q);

    /* ---- altitude: complementary filter over accel + baro/range --------- */
    float a_world_z = 0.0f;
    if (accel_ok) {
        /* Rotate body accel into the reference frame and cancel gravity;
         * row 3 of R = Rz*Ry*Rx, see file header derivation. */
        a_world_z = -sinf(pitch_out) * ax
                  + cosf(pitch_out) * sinf(roll_out) * ay
                  + cosf(pitch_out) * cosf(roll_out) * az
                  - BLIMP_EST_GRAVITY_MPS2;
    }
    const float alt_pred = e->alt + e->vz * dt;
    const float vz_pred  = e->vz + a_world_z * dt;

    bool have_abs = false;
    float abs_alt = 0.0f;
    if (s->range_valid && isfinite(s->range_alt)) {
        abs_alt = s->range_alt;
        have_abs = true;
    } else if (s->baro_valid && isfinite(s->baro_alt) && isfinite(e->baro_ref)) {
        abs_alt = s->baro_alt - e->baro_ref;
        have_abs = true;
    }

    float alt_out, vz_out;
    if (have_abs) {
        const float fix_age_s = (float)(uint32_t)(s->t_us - e->last_abs_us) * 1e-6f;
        const bool reacquire = !e->abs_ever || !isfinite(fix_age_s) ||
                               fix_age_s > BLIMP_EST_ALT_REACQ_S;
        if (reacquire) {
            /* First fix, or first after a gap: snap the altitude and keep
             * the dead-reckoned rate. Differencing across the gap would
             * produce a vertical rate of many metres per second from one
             * sample, which the altitude loop's damping term would then
             * act on. */
            alt_out = abs_alt;
            vz_out  = vz_pred;
        } else {
            const float alt_before = e->alt;
            const float vz_meas    = (abs_alt - alt_before) / dt;
            const float keep       = sp.comp_alpha_alt;
            alt_out = keep * alt_pred + (1.0f - keep) * abs_alt;
            vz_out  = keep * vz_pred  + (1.0f - keep) * vz_meas;
        }
        e->last_abs_us = s->t_us;
        e->abs_ever    = true;
    } else {
        alt_out = alt_pred;
        vz_out  = vz_pred;
    }
    if (!isfinite(alt_out)) {
        alt_out = e->alt;
    }
    if (!isfinite(vz_out)) {
        vz_out = 0.0f;
    }
    e->alt = alt_out;
    e->vz  = vz_out;

    write_state(e, s, out);
}
