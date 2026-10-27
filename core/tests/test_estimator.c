/* test_estimator.c - attitude/heading/altitude estimation.
 *
 * Yaw drift is THE problem this estimator exists to solve (docs/07 section
 * 5.2): a bicopter has only a handful of gf of differential thrust and
 * cannot fight through accumulated heading error the way a quadcopter can.
 * So the headline tests here are not "does it run" but "does the
 * magnetometer actually bound the drift a mag-less design would suffer",
 * and "does tilt compensation actually matter" - both proven by contrast
 * against the naive/disabled alternative, not just asserted in isolation.
 */
#include "blimp/estimator.h"
#include "blimp/pid.h"
#include "test_util.h"

#include <math.h>
#include <string.h>

static const float G0 = 9.80665f;

/* Body-frame accelerometer reading for a given static roll/pitch, per the
 * estimator's own derivation (row 3 of R = Rz*Ry*Rx): a stationary
 * accelerometer reads +g "up" in the reference frame, transformed into body
 * frame. Used to construct test fixtures that are internally consistent
 * with the estimator's own math, not an independent re-derivation of it -
 * the independent checks are the ones against KNOWN ground truth values
 * (e.g. roll/pitch == 0, or the naive-vs-tilt-compensated heading contrast).
 */
static void accel_for_tilt(float roll, float pitch, float out[3])
{
    out[0] = -sinf(pitch) * G0;
    out[1] = cosf(pitch) * sinf(roll) * G0;
    out[2] = cosf(pitch) * cosf(roll) * G0;
}

/* Body-frame magnetometer reading for a given true heading/roll/pitch and a
 * purely-horizontal world field of unit magnitude (declination folded in
 * separately by the caller via true_yaw already including it, OR by passing
 * decl_rad and having the caller add it to the params - see callers). */
static void mag_for_pose(float roll, float pitch, float yaw, float out[3])
{
    const float cr = cosf(roll),  srr = sinf(roll);
    const float cp = cosf(pitch), spp = sinf(pitch);
    const float cy = cosf(yaw),   sy  = sinf(yaw);

    /* m_body = Bh * (row 0 of R)^T, R = Rz(yaw)Ry(pitch)Rx(roll):
     *   m_x = Bh * cy*cp
     *   m_y = Bh * (cy*sp*sr - sy*cr)
     *   m_z = Bh * (cy*sp*cr + sy*sr)             */
    out[0] = cy * cp;
    out[1] = cy * spp * srr - sy * cr;
    out[2] = cy * spp * cr  + sy * srr;
}

static blimp_imu_sample_t make_sample(uint32_t t_us)
{
    blimp_imu_sample_t s;
    memset(&s, 0, sizeof(s));
    s.t_us = t_us;
    return s;
}

int main(void)
{
    printf("test_estimator\n");

    blimp_params_t P;
    blimp_params_defaults(&P);
    /* The estimator's magnetometer path is exercised here so it stays
     * correct for the day a magnetometer is fitted. The PCB default
     * (params.c) is use_mag = false because the MPU-6050 has none. */
    P.use_mag = true;
    (void)blimp_params_validate(&P);

    /* ===================================================================
     * 1. Level, stationary: roll/pitch converge to ~0 from a deliberately
     *    wrong initial guess.
     * =================================================================== */
    TEST_GROUP("level stationary: roll/pitch converge from a bad initial guess");
    {
        static const float bad_rolls[]  = {-2.5f, -1.0f, -0.3f, 0.3f, 1.0f, 2.5f};
        static const float bad_pitches[] = {-1.2f, -0.4f, 0.0f, 0.4f, 1.2f};

        for (unsigned ri = 0; ri < sizeof(bad_rolls) / sizeof(bad_rolls[0]); ri++)
        for (unsigned pi = 0; pi < sizeof(bad_pitches) / sizeof(bad_pitches[0]); pi++) {
            blimp_estimator_t e;
            blimp_estimator_init(&e);
            /* Force a wrong initial attitude directly - a fresh init() gives
             * identity, so seed the quaternion by hand. */
            float q[4];
            const float roll0 = bad_rolls[ri], pitch0 = bad_pitches[pi];
            /* Reuse the same euler->quat convention the estimator uses by
             * constructing it the same way a normal update would: run one
             * tick with gyro giving exactly this attitude is unnecessarily
             * roundabout, so just build it with the identical formula used
             * inside estimator.c (duplicated here deliberately - the test
             * must not call into the module's internals). */
            {
                const float cr = cosf(roll0 * 0.5f),  sr = sinf(roll0 * 0.5f);
                const float cp = cosf(pitch0 * 0.5f), sp = sinf(pitch0 * 0.5f);
                q[0] = cr * cp; q[1] = sr * cp; q[2] = cr * sp; q[3] = -sr * sp;
                /* yaw=0 case of the general formula collapses to this. */
            }
            e.q[0] = q[0]; e.q[1] = q[1]; e.q[2] = q[2]; e.q[3] = q[3];

            blimp_state_t out;
            uint32_t t = 0;
            const uint32_t dt_us = 5000; /* 200 Hz */
            float acc[3];
            accel_for_tilt(0.0f, 0.0f, acc); /* TRUE pose is level */

            for (int i = 0; i < 4000; i++) { /* 20 s */
                blimp_imu_sample_t s = make_sample(t);
                s.gyro[0] = s.gyro[1] = s.gyro[2] = 0.0f;
                s.gyro_valid = true;
                s.accel[0] = acc[0]; s.accel[1] = acc[1]; s.accel[2] = acc[2];
                s.accel_valid = true;
                blimp_estimator_update(&e, &P, &s, &out);
                t += dt_us;
            }
            CHECK_FINITE(out.roll);
            CHECK_FINITE(out.pitch);
            CHECK_MSG(fabsf(out.roll) < 0.02f,
                      "roll did not converge from %.3f: got %.5f", (double)roll0, (double)out.roll);
            CHECK_MSG(fabsf(out.pitch) < 0.02f,
                      "pitch did not converge from %.3f: got %.5f", (double)pitch0, (double)out.pitch);
        }
    }

    /* ===================================================================
     * 2. Constant gyro bias: estimated and removed; heading drift is
     *    BOUNDED with mag enabled and UNBOUNDED without it.
     * =================================================================== */
    TEST_GROUP("gyro bias estimated and removed");
    {
        blimp_params_t p = P;
        p.gyro_bias_tau_s = 0.5f;   /* short tau: converge fast for the test */

        blimp_estimator_t e;
        blimp_estimator_init(&e);

        const float true_bias_z = 0.05f; /* rad/s, ~2.9 deg/s */
        uint32_t t = 0;
        const uint32_t dt_us = 10000; /* 100 Hz */
        blimp_state_t out;

        /* Long stationary calibration: many tau, should converge tightly. */
        for (int i = 0; i < 5000; i++) { /* 50 s >> tau */
            blimp_imu_sample_t s = make_sample(t);
            s.gyro[0] = 0.0f; s.gyro[1] = 0.0f; s.gyro[2] = true_bias_z;
            s.gyro_valid = true;
            s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = G0;
            s.accel_valid = true;
            blimp_estimator_update(&e, &p, &s, &out);
            t += dt_us;
        }
        CHECK_MSG(fabsf(e.gyro_bias[2] - true_bias_z) < 0.002f,
                  "gyro bias not estimated: got %.5f want ~%.5f",
                  (double)e.gyro_bias[2], (double)true_bias_z);
        /* Bias-corrected output should be near zero once converged. */
        CHECK_MSG(fabsf(out.gyro[2]) < 0.005f,
                  "bias not removed from output: gyro[2]=%.5f", (double)out.gyro[2]);
    }

    TEST_GROUP("heading drift: bounded with mag, unbounded without");
    {
        /* Two estimators, identical scripted inputs, differing only in
         * p.use_mag. Phase 1: brief stationary calibration that deliberately
         * does NOT fully converge (short vs. tau), leaving a known residual
         * bias. Phase 2: "in flight" (accel != g, so bias estimation
         * freezes) for a long duration with the SAME constant z-bias -
         * the residual integrates freely without mag, and is continuously
         * corrected with it. */
        blimp_params_t p_mag = P, p_nomag = P;
        p_mag.gyro_bias_tau_s = p_nomag.gyro_bias_tau_s = 4.0f;
        p_mag.use_mag = true;
        p_nomag.use_mag = false;
        p_mag.mag_declination_rad = p_nomag.mag_declination_rad = 0.0f;

        blimp_estimator_t e_mag, e_nomag;
        blimp_estimator_init(&e_mag);
        blimp_estimator_init(&e_nomag);

        const float true_bias_z = 0.05f;
        const uint32_t dt_us = 10000; /* 100 Hz */
        uint32_t t = 0;
        blimp_state_t out_mag, out_nomag;

        /* Phase 1: stationary, ~1 tau -> ~63% converged, clear residual left. */
        for (int i = 0; i < 400; i++) { /* 4 s == 1 tau */
            blimp_imu_sample_t s = make_sample(t);
            s.gyro[0] = 0.0f; s.gyro[1] = 0.0f; s.gyro[2] = true_bias_z;
            s.gyro_valid = true;
            s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = G0;
            s.accel_valid = true;
            blimp_estimator_update(&e_mag,   &p_mag,   &s, &out_mag);
            blimp_estimator_update(&e_nomag, &p_nomag, &s, &out_nomag);
            t += dt_us;
        }
        blimp_estimator_zero(&e_mag, NULL);
        blimp_estimator_zero(&e_nomag, NULL);
        /* zero() resets t=0 heading reference conceptually; keep t running. */

        /* Phase 2: "flying" - accel no longer reads pure g, so the
         * stationary gate is false and bias estimation freezes with its
         * phase-1 residual. Mag sample (when enabled) reflects the TRUE
         * (yaw=0) heading throughout - a real mag would, the bias error is
         * purely an estimator artifact, not a change in true heading. */
        for (int i = 0; i < 60000; i++) { /* 600 s of "flight" */
            blimp_imu_sample_t s = make_sample(t);
            s.gyro[0] = 0.0f; s.gyro[1] = 0.0f; s.gyro[2] = true_bias_z;
            s.gyro_valid = true;
            /* A vertical accel transient (thrust surge) large enough to
             * clear the stationary accel-magnitude tolerance, so bias
             * estimation freezes at its phase-1 residual for the rest of
             * this "flight" - WITHOUT implying any false roll/pitch tilt
             * (ax=ay=0 keeps the accel-derived roll/pitch exactly level, so
             * this doesn't also perturb the tilt-compensation this test is
             * not trying to exercise here). */
            s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = G0 + 2.0f;
            s.accel_valid = true;
            s.mag[0] = 1.0f; s.mag[1] = 0.0f; s.mag[2] = 0.0f; /* true heading 0 */
            s.mag_valid = true;
            blimp_estimator_update(&e_mag,   &p_mag,   &s, &out_mag);
            blimp_estimator_update(&e_nomag, &p_nomag, &s, &out_nomag);
            t += dt_us;
        }

        CHECK_FINITE(out_mag.yaw);
        CHECK_FINITE(out_nomag.yaw);
        CHECK_MSG(fabsf(out_mag.yaw) < 0.2f,
                  "mag-corrected heading drifted too far: %.4f rad", (double)out_mag.yaw);
        CHECK_MSG(fabsf(out_nomag.yaw) > 0.5f,
                  "gyro-only heading did NOT drift - test does not actually exercise the "
                  "failure mode it claims to (got %.4f rad)", (double)out_nomag.yaw);
        CHECK_MSG(fabsf(out_nomag.yaw) > fabsf(out_mag.yaw) * 2.0f,
                  "mag case did not clearly bound drift vs. no-mag case (mag=%.4f nomag=%.4f)",
                  (double)out_mag.yaw, (double)out_nomag.yaw);
    }

    /* ===================================================================
     * 3. Tilt compensation: same magnetic field, vehicle rolled and
     *    pitched, heading stays stable. Prove an uncompensated calc would
     *    fail this same scenario.
     * =================================================================== */
    TEST_GROUP("tilt compensation keeps heading stable under roll+pitch");
    {
        static const struct { float roll, pitch, yaw; } cases[] = {
            {0.5f, 0.3f, 0.4f},
            {0.9f, 0.8f, 1.0f},
            {1.0f, -0.9f, 0.5f},
            {-0.9f, 0.7f, 2.5f},
        };
        int naive_fail_count = 0;

        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            const float roll_t = cases[i].roll, pitch_t = cases[i].pitch, yaw_t = cases[i].yaw;

            blimp_params_t p = P;
            p.mag_declination_rad = 0.0f;
            blimp_estimator_t e;
            blimp_estimator_init(&e);
            blimp_state_t out;
            uint32_t t = 0;

            /* Phase A: establish a heading fix LEVEL, before any tilt. */
            float acc0[3], mag0[3];
            accel_for_tilt(0.0f, 0.0f, acc0);
            mag_for_pose(0.0f, 0.0f, yaw_t, mag0);
            for (int k = 0; k < 2000; k++) {
                blimp_imu_sample_t s = make_sample(t);
                s.gyro_valid = true;
                s.accel[0] = acc0[0]; s.accel[1] = acc0[1]; s.accel[2] = acc0[2];
                s.accel_valid = true;
                s.mag[0] = mag0[0]; s.mag[1] = mag0[1]; s.mag[2] = mag0[2];
                s.mag_valid = true;
                blimp_estimator_update(&e, &p, &s, &out);
                t += 5000;
            }
            CHECK_MSG(fabsf(blimp_wrap_pi(out.yaw - yaw_t)) < 0.05f,
                      "case %u: failed to establish level heading before tilting: got %.4f want %.4f",
                      i, (double)out.yaw, (double)yaw_t);
            const float yaw_established = out.yaw;

            /* The naive (non-tilt-compensated) heading calc a simpler
             * estimator would use on the TILTED reading: atan2(-my, mx),
             * ignoring roll/pitch entirely. */
            float acc_t[3], mag_t[3];
            accel_for_tilt(roll_t, pitch_t, acc_t);
            mag_for_pose(roll_t, pitch_t, yaw_t, mag_t);
            const float naive_heading = atan2f(-mag_t[1], mag_t[0]);
            const float naive_err = fabsf(blimp_wrap_pi(naive_heading - yaw_t));
            if (naive_err > 0.3f) {
                naive_fail_count++;
            }

            /* Phase B: roll and pitch the vehicle. TRUE heading is
             * UNCHANGED - the field is the same magnetic field, just read
             * through a different tilt. A tilt-compensated estimator's
             * heading must stay stable; an uncompensated one would jump by
             * naive_err computed above. */
            for (int k = 0; k < 4000; k++) {
                blimp_imu_sample_t s = make_sample(t);
                s.gyro_valid = true; /* zero rate: static pose, tilt is a step */
                s.accel[0] = acc_t[0]; s.accel[1] = acc_t[1]; s.accel[2] = acc_t[2];
                s.accel_valid = true;
                s.mag[0] = mag_t[0]; s.mag[1] = mag_t[1]; s.mag[2] = mag_t[2];
                s.mag_valid = true;
                blimp_estimator_update(&e, &p, &s, &out);
                t += 5000;
            }
            CHECK_FINITE(out.roll);
            CHECK_FINITE(out.pitch);
            CHECK_FINITE(out.yaw);
            CHECK_MSG(fabsf(out.roll - roll_t) < 0.05f,
                      "case %u: roll did not converge to the commanded tilt: got %.4f want %.4f",
                      i, (double)out.roll, (double)roll_t);
            CHECK_MSG(fabsf(out.pitch - pitch_t) < 0.05f,
                      "case %u: pitch did not converge to the commanded tilt: got %.4f want %.4f",
                      i, (double)out.pitch, (double)pitch_t);
            CHECK_MSG(fabsf(blimp_wrap_pi(out.yaw - yaw_established)) < 0.1f,
                      "case %u: heading was not stable across the tilt: before=%.4f after=%.4f",
                      i, (double)yaw_established, (double)out.yaw);
            CHECK_MSG(e.mag_healthy, "case %u: a clean, consistent mag sample was marked unhealthy", i);
        }
        CHECK_MSG(naive_fail_count > 0,
                  "no test case actually demonstrates tilt compensation mattering - "
                  "the naive uncompensated formula agreed with truth on every case");
    }

    /* ===================================================================
     * 4. Mag health: a large disturbance clears mag_healthy; output stays
     *    finite and sane.
     * =================================================================== */
    TEST_GROUP("mag health: large disturbance is rejected");
    {
        blimp_params_t p = P;
        p.mag_declination_rad = 0.0f;
        blimp_estimator_t e;
        blimp_estimator_init(&e);

        blimp_state_t out;
        uint32_t t = 0;
        /* Settle level, heading 0, with a clean consistent field. */
        for (int i = 0; i < 1000; i++) {
            blimp_imu_sample_t s = make_sample(t);
            s.gyro_valid = true;
            s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = G0;
            s.accel_valid = true;
            s.mag[0] = 1.0f; s.mag[1] = 0.0f; s.mag[2] = 0.0f;
            s.mag_valid = true;
            blimp_estimator_update(&e, &p, &s, &out);
            t += 5000;
        }
        CHECK(e.mag_healthy);

        /* Inject a wild disturbance: field rotated ~90+ degrees, a huge
         * magnitude spike, degenerate/corrupt samples (motors starting up
         * near the mag is the expected real-world cause). Each is tested
         * from an already-healthy, already-settled baseline: a mag reading
         * that fails the plausibility gate while NO fix is established yet
         * is legitimate acquisition (see estimator.c decision 2), not a
         * disturbance, so re-settling between injections is what actually
         * isolates "does a disturbance get rejected" from "does bootstrap
         * get to run" - both are exercised elsewhere in this file. */
        static const float disturbances[][3] = {
            {-50.0f, 80.0f, 30.0f},
            {0.0f, 1000.0f, 0.0f},        /* huge magnitude, ~90 deg off */
            {0.0f, 0.0f, 0.0f},          /* degenerate zero-magnitude sample */
            {(float)NAN, 1.0f, 0.0f},    /* corrupt sample */
            {(float)INFINITY, 0.0f, 0.0f},
        };
        for (unsigned i = 0; i < sizeof(disturbances) / sizeof(disturbances[0]); i++) {
            /* Re-settle to a known-healthy baseline before each injection. */
            for (int k = 0; k < 200; k++) {
                blimp_imu_sample_t s = make_sample(t);
                s.gyro_valid = true;
                s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = G0;
                s.accel_valid = true;
                s.mag[0] = 1.0f; s.mag[1] = 0.0f; s.mag[2] = 0.0f;
                s.mag_valid = true;
                blimp_estimator_update(&e, &p, &s, &out);
                t += 5000;
            }
            CHECK_MSG(e.mag_healthy, "failed to re-settle healthy before disturbance %u", i);
            const float yaw_before = out.yaw;

            blimp_imu_sample_t s = make_sample(t);
            s.gyro_valid = true;
            s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = G0;
            s.accel_valid = true;
            s.mag[0] = disturbances[i][0];
            s.mag[1] = disturbances[i][1];
            s.mag[2] = disturbances[i][2];
            s.mag_valid = true;
            blimp_estimator_update(&e, &p, &s, &out);
            t += 5000;

            CHECK_MSG(!e.mag_healthy, "disturbance %u did not clear mag_healthy", i);
            CHECK_FINITE(out.roll);
            CHECK_FINITE(out.pitch);
            CHECK_FINITE(out.yaw);
            CHECK_MSG(fabsf(out.roll) <= 3.15f && fabsf(out.pitch) <= 3.15f && fabsf(out.yaw) <= 3.15f,
                      "disturbance %u produced an out-of-range angle", i);
            /* Rejected sample must not have yanked heading around. */
            CHECK_MSG(fabsf(blimp_wrap_pi(out.yaw - yaw_before)) < 0.1f,
                      "disturbance %u corrupted heading despite being rejected: %.4f vs %.4f",
                      i, (double)out.yaw, (double)yaw_before);
        }

        /* Recovery: a good sample afterwards should mark healthy again. */
        blimp_imu_sample_t s = make_sample(t);
        s.gyro_valid = true;
        s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = G0;
        s.accel_valid = true;
        s.mag[0] = 1.0f; s.mag[1] = 0.0f; s.mag[2] = 0.0f;
        s.mag_valid = true;
        blimp_estimator_update(&e, &p, &s, &out);
        CHECK_MSG(e.mag_healthy, "did not recover mag_healthy after a good sample");
    }

    /* ===================================================================
     * 5. Altitude tracks a ramp; degrades gracefully when ToF drops out.
     * =================================================================== */
    TEST_GROUP("altitude tracks a ramp, degrades gracefully without ToF");
    {
        blimp_params_t p = P;
        blimp_estimator_t e;
        blimp_estimator_init(&e);

        const float vz_true = 0.1f; /* m/s */
        const float alt0 = 0.5f;
        uint32_t t = 0;
        const uint32_t dt_us = 10000;
        const float dt = 1.0e-2f;
        blimp_state_t out;

        blimp_imu_sample_t zero_s = make_sample(0);
        zero_s.baro_valid = true; zero_s.baro_alt = 1000.0f; /* arbitrary station pressure-alt */
        blimp_estimator_zero(&e, &zero_s);

        float true_alt = alt0;
        for (int i = 0; i < 3000; i++) { /* 30 s, range available */
            blimp_imu_sample_t s = make_sample(t);
            s.gyro_valid = true;
            s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = G0; /* level, no net accel: pure ramp */
            s.accel_valid = true;
            s.range_valid = true;
            s.range_alt = true_alt;
            s.baro_valid = true;
            s.baro_alt = zero_s.baro_alt + true_alt;
            blimp_estimator_update(&e, &p, &s, &out);
            t += dt_us;
            true_alt += vz_true * dt;
        }
        CHECK_FINITE(out.altitude);
        CHECK_FINITE(out.vz);
        CHECK(out.altitude_valid);
        CHECK_MSG(fabsf(out.altitude - true_alt) < 0.05f,
                  "altitude did not track ramp (range phase): got %.4f want %.4f",
                  (double)out.altitude, (double)true_alt);
        CHECK_MSG(fabsf(out.vz - vz_true) < 0.05f,
                  "vertical rate did not converge to ramp rate: got %.4f want %.4f",
                  (double)out.vz, (double)vz_true);

        /* ToF drops out; only baro remains. */
        for (int i = 0; i < 3000; i++) { /* another 30 s, baro only */
            blimp_imu_sample_t s = make_sample(t);
            s.gyro_valid = true;
            s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = G0;
            s.accel_valid = true;
            s.range_valid = false;
            s.baro_valid = true;
            s.baro_alt = zero_s.baro_alt + true_alt;
            blimp_estimator_update(&e, &p, &s, &out);
            t += dt_us;
            true_alt += vz_true * dt;
        }
        CHECK_FINITE(out.altitude);
        CHECK_FINITE(out.vz);
        CHECK(out.altitude_valid);   /* baro alone still counts as "a usable source" */
        CHECK_MSG(fabsf(out.altitude - true_alt) < 0.1f,
                  "altitude did not track ramp on baro alone: got %.4f want %.4f",
                  (double)out.altitude, (double)true_alt);

        /* Both sources gone: altitude_valid must clear, output still finite,
         * dead reckoning continues rather than snapping to garbage. */
        blimp_imu_sample_t s = make_sample(t);
        s.gyro_valid = true;
        s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = G0;
        s.accel_valid = true;
        s.range_valid = false;
        s.baro_valid = false;
        const float alt_before_dropout = out.altitude;
        blimp_estimator_update(&e, &p, &s, &out);
        /* Validity is a window (estimator.h): one dropout tick keeps the
         * last fix usable and dead-reckons from it. */
        CHECK(out.altitude_valid);
        CHECK_FINITE(out.altitude);
        CHECK_NEAR(out.altitude, alt_before_dropout + out.vz * dt, 1e-3);
        /* ... but a long dropout must eventually clear it. */
        for (int i = 0; i < 400; i++) {   /* 4 s > BLIMP_EST_ALT_STALE_S */
            t += (uint32_t)(dt * 1e6f);
            s.t_us = t;
            blimp_estimator_update(&e, &p, &s, &out);
        }
        CHECK(!out.altitude_valid);
        CHECK_FINITE(out.altitude);
    }

    /* ===================================================================
     * 6. Sweeps with NaN/Inf/missing fields: output always finite.
     * =================================================================== */
    TEST_GROUP("NaN/Inf/missing-field sweep never produces non-finite output");
    {
        const float nan_v = (float)NAN, inf_v = (float)INFINITY;
        /* Each field gets one of: normal value, all-NaN, all-Inf, an
         * absurdly huge-but-finite value (a runaway sensor/bus glitch, not
         * strictly NaN/Inf but just as capable of blowing up an unguarded
         * calculation), or marked invalid (value irrelevant). */
        typedef enum { FLD_OK, FLD_NAN, FLD_INF, FLD_HUGE, FLD_INVALID } fld_t;
        static const fld_t opts[] = {FLD_OK, FLD_NAN, FLD_INF, FLD_HUGE, FLD_INVALID};
        const int no = (int)(sizeof(opts) / sizeof(opts[0]));
        const float huge_v = 1.0e30f;

        blimp_estimator_t e;
        blimp_estimator_init(&e);
        blimp_state_t out;
        uint32_t t = 0;
        long combos = 0;
        int nonfinite = 0;

        for (int gi = 0; gi < no; gi++)
        for (int ai = 0; ai < no; ai++)
        for (int mi = 0; mi < no; mi++)
        for (int bi = 0; bi < no; bi++)
        for (int rii = 0; rii < no; rii++) {
            blimp_imu_sample_t s = make_sample(t);
            t += 5000;

            switch (opts[gi]) {
                case FLD_OK: s.gyro[0]=0.01f; s.gyro[1]=-0.02f; s.gyro[2]=0.03f; s.gyro_valid=true; break;
                case FLD_NAN: s.gyro[0]=s.gyro[1]=s.gyro[2]=nan_v; s.gyro_valid=true; break;
                case FLD_INF: s.gyro[0]=s.gyro[1]=s.gyro[2]=inf_v; s.gyro_valid=true; break;
                case FLD_HUGE: s.gyro[0]=s.gyro[1]=s.gyro[2]=huge_v; s.gyro_valid=true; break;
                case FLD_INVALID: s.gyro_valid=false; break;
            }
            switch (opts[ai]) {
                case FLD_OK: s.accel[0]=0.1f; s.accel[1]=-0.1f; s.accel[2]=G0; s.accel_valid=true; break;
                case FLD_NAN: s.accel[0]=s.accel[1]=s.accel[2]=nan_v; s.accel_valid=true; break;
                case FLD_INF: s.accel[0]=s.accel[1]=s.accel[2]=inf_v; s.accel_valid=true; break;
                case FLD_HUGE: s.accel[0]=s.accel[1]=s.accel[2]=huge_v; s.accel_valid=true; break;
                case FLD_INVALID: s.accel_valid=false; break;
            }
            switch (opts[mi]) {
                case FLD_OK: s.mag[0]=1.0f; s.mag[1]=0.2f; s.mag[2]=-0.1f; s.mag_valid=true; break;
                case FLD_NAN: s.mag[0]=s.mag[1]=s.mag[2]=nan_v; s.mag_valid=true; break;
                case FLD_INF: s.mag[0]=s.mag[1]=s.mag[2]=inf_v; s.mag_valid=true; break;
                case FLD_HUGE: s.mag[0]=s.mag[1]=s.mag[2]=huge_v; s.mag_valid=true; break;
                case FLD_INVALID: s.mag_valid=false; break;
            }
            switch (opts[bi]) {
                case FLD_OK: s.baro_alt=100.0f; s.baro_valid=true; break;
                case FLD_NAN: s.baro_alt=nan_v; s.baro_valid=true; break;
                case FLD_INF: s.baro_alt=inf_v; s.baro_valid=true; break;
                case FLD_HUGE: s.baro_alt=huge_v; s.baro_valid=true; break;
                case FLD_INVALID: s.baro_valid=false; break;
            }
            switch (opts[rii]) {
                case FLD_OK: s.range_alt=1.5f; s.range_valid=true; break;
                case FLD_NAN: s.range_alt=nan_v; s.range_valid=true; break;
                case FLD_INF: s.range_alt=inf_v; s.range_valid=true; break;
                case FLD_HUGE: s.range_alt=huge_v; s.range_valid=true; break;
                case FLD_INVALID: s.range_valid=false; break;
            }

            blimp_estimator_update(&e, &P, &s, &out);
            combos++;

            int bad = 0;
            if (!isfinite(out.roll)) bad++;
            if (!isfinite(out.pitch)) bad++;
            if (!isfinite(out.yaw)) bad++;
            if (!isfinite(out.gyro[0]) || !isfinite(out.gyro[1]) || !isfinite(out.gyro[2])) bad++;
            if (!isfinite(out.accel[0]) || !isfinite(out.accel[1]) || !isfinite(out.accel[2])) bad++;
            if (!isfinite(out.altitude)) bad++;
            if (!isfinite(out.vz)) bad++;
            if (!isfinite(out.yaw_rate)) bad++;
            if (!isfinite(e.gyro_bias[0]) || !isfinite(e.gyro_bias[1]) || !isfinite(e.gyro_bias[2])) bad++;
            if (!isfinite(e.q[0]) || !isfinite(e.q[1]) || !isfinite(e.q[2]) || !isfinite(e.q[3])) bad++;
            if (!isfinite(e.alt) || !isfinite(e.vz) || !isfinite(e.stationary_s)) bad++;
            if (bad) {
                nonfinite++;
            }
        }
        printf("    swept %ld field combinations\n", combos);
        CHECK_MSG(nonfinite == 0, "%d/%ld combinations produced non-finite state", nonfinite, combos);
        CHECK(combos > 3000);
    }

    /* ===================================================================
     * 7. dt sweep: backwards, zero, huge - must skip rather than corrupt.
     * =================================================================== */
    TEST_GROUP("absurd dt is rejected, not integrated");
    {
        blimp_estimator_t e;
        blimp_estimator_init(&e);
        blimp_state_t out;

        blimp_imu_sample_t s0 = make_sample(1000000);
        s0.gyro_valid = true; s0.accel_valid = true; s0.accel[2] = G0;
        blimp_estimator_update(&e, &P, &s0, &out); /* primes last_us */
        blimp_estimator_update(&e, &P, &s0, &out); /* dt from a real prior tick */
        CHECK_FINITE(out.roll);

        const float roll_before = out.roll;
        const float pitch_before = out.pitch;

        static const uint32_t bad_ts[] = {1000000, 999999, 0};
        for (unsigned i = 0; i < sizeof(bad_ts) / sizeof(bad_ts[0]); i++) {
            blimp_imu_sample_t s = s0;
            s.t_us = bad_ts[i];
            s.gyro[0] = 5.0f; /* if this integrated, it would move attitude a lot */
            blimp_estimator_update(&e, &P, &s, &out);
            CHECK_FINITE(out.roll);
            CHECK_FINITE(out.pitch);
            CHECK_NEAR(out.roll, roll_before, 1e-6);
            CHECK_NEAR(out.pitch, pitch_before, 1e-6);
        }

        /* Implausibly large forward jump: also must not be integrated as if
         * real (a huge single-tick rotation would be nonsense). */
        blimp_imu_sample_t huge = s0;
        huge.t_us = s0.t_us + 100000000u; /* 100 s jump */
        huge.gyro[0] = 5.0f;
        blimp_estimator_update(&e, &P, &huge, &out);
        CHECK_FINITE(out.roll);
        CHECK_NEAR(out.roll, roll_before, 1e-6);
    }

    /* ===================================================================
     * 8. NULL safety.
     * =================================================================== */
    TEST_GROUP("null safety");
    {
        blimp_estimator_init(NULL);
        blimp_estimator_zero(NULL, NULL);
        CHECK(blimp_estimator_converging(NULL));

        blimp_estimator_t e;
        blimp_estimator_init(&e);
        blimp_imu_sample_t s = make_sample(1);
        s.gyro_valid = true; s.accel_valid = true; s.accel[2] = G0;

        blimp_estimator_zero(&e, NULL); /* null sample */
        blimp_estimator_update(NULL, &P, &s, NULL);
        blimp_estimator_update(&e, NULL, &s, NULL);   /* null params, null out */
        blimp_estimator_update(&e, &P, NULL, NULL);   /* null sample, null out */

        blimp_state_t out;
        memset(&out, 0xAB, sizeof(out));   /* poison it first */
        blimp_estimator_update(NULL, &P, &s, &out);
        CHECK_NEAR(out.roll, 0.0f, 1e-9);
        CHECK_NEAR(out.altitude, 0.0f, 1e-9);
        CHECK(!out.attitude_valid);
        CHECK(!out.altitude_valid);

        blimp_estimator_update(&e, NULL, NULL, &out); /* both null: still safe */
        CHECK_FINITE(out.roll);
    }

    /* ===================================================================
     * 9. Determinism.
     * =================================================================== */
    TEST_GROUP("deterministic");
    {
        blimp_estimator_t e1, e2;
        blimp_estimator_init(&e1);
        blimp_estimator_init(&e2);
        blimp_state_t o1, o2;

        for (int i = 0; i < 50; i++) {
            blimp_imu_sample_t s = make_sample((uint32_t)(i * 5000));
            s.gyro[0] = 0.01f; s.gyro[1] = -0.02f; s.gyro[2] = 0.005f; s.gyro_valid = true;
            s.accel[0] = 0.05f; s.accel[1] = 0.0f; s.accel[2] = G0; s.accel_valid = true;
            s.mag[0] = 1.0f; s.mag[1] = 0.1f; s.mag[2] = 0.0f; s.mag_valid = true;
            s.baro_valid = true; s.baro_alt = 100.0f + (float)i * 0.01f;
            blimp_estimator_update(&e1, &P, &s, &o1);
            blimp_estimator_update(&e2, &P, &s, &o2);
        }
        CHECK(o1.roll == o2.roll);
        CHECK(o1.pitch == o2.pitch);
        CHECK(o1.yaw == o2.yaw);
        CHECK(o1.altitude == o2.altitude);
        CHECK(o1.vz == o2.vz);
    }

    /* ===================================================================
     * 10. blimp_estimator_converging: true until initialised/settled.
     * =================================================================== */
    TEST_GROUP("converging blocks until stationary dwell settles");
    {
        blimp_estimator_t e;
        blimp_estimator_init(&e);
        CHECK(blimp_estimator_converging(&e)); /* never run: still converging */

        blimp_state_t out;
        uint32_t t = 0;
        blimp_imu_sample_t s = make_sample(t);
        s.gyro_valid = true; s.accel_valid = true; s.accel[2] = G0;
        blimp_estimator_update(&e, &P, &s, &out); /* first tick: priming only */
        CHECK(blimp_estimator_converging(&e));

        /* Run stationary well past the converge threshold. */
        for (int i = 0; i < 8000; i++) { /* 80 s at 100 Hz */
            t += 10000;
            blimp_imu_sample_t ss = make_sample(t);
            ss.gyro_valid = true; ss.accel_valid = true; ss.accel[2] = G0;
            blimp_estimator_update(&e, &P, &ss, &out);
        }
        CHECK_MSG(!blimp_estimator_converging(&e),
                  "still converging after 80 s of stationary dwell (stationary_s=%.2f)",
                  (double)e.stationary_s);

        /* Motion resets the dwell clock and re-blocks arming. */
        blimp_imu_sample_t moving = make_sample(t + 10000);
        moving.gyro_valid = true; moving.gyro[0] = 5.0f;
        moving.accel_valid = true; moving.accel[2] = G0;
        blimp_estimator_update(&e, &P, &moving, &out);
        CHECK(blimp_estimator_converging(&e));
    }

    TEST_REPORT();
}
