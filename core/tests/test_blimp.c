/* test_blimp.c - end-to-end tests for blimp_step(), the whole flight
 * controller (blimp/blimp.h): estimator -> arming -> failsafe -> guidance ->
 * PID -> mixer, exactly as one real control tick runs it.
 *
 * The headline property, exactly as in test_mixer.c: this is the last stop
 * before the actuators, so the Rule 13 budget and "nothing is ever NaN"
 * must hold across a long, adversarial, deterministic sweep - not just a
 * handful of nominal examples. Beyond that sweep, this file also exercises
 * the specific safety-relevant BEHAVIOURS the task calls out by name:
 * arming blocked while the estimator converges, failsafe engaging on link
 * timeout and on ceiling breach, and guidance making real progress when fed
 * fully cooperative observations.
 *
 * A note on time scale: several groups below use large (~1 s) per-tick dt
 * values to reach a 60+ second gyro-bias convergence window in a handful of
 * ticks rather than the ~12000 a real 5 ms control period would need. This
 * is safe here specifically because the vehicle in those sequences is never
 * actually rotating or accelerating (a perfect stationary IMU sample), so
 * nothing about the result depends on integration step size - and it stays
 * within both estimator.c's and blimp.c's own accepted dt range
 * (<= 2.0 s). The adversarial sweep at the bottom uses a normal ~5 ms
 * period (among other deliberately-abnormal ones).
 */
#include "blimp/blimp.h"
#include "test_util.h"

/* ---- shared fixtures ---------------------------------------------------- */

/* A perfectly still IMU sample: zero body rates, accelerometer reading
 * exactly 1 g on the body +z axis (types.h: "accel: m/s^2, body, gravity
 * included"; a level, motionless vehicle reads +g on +z). Feeding this
 * every tick keeps the estimator's own "stationary" gyro-bias gate
 * satisfied indefinitely, which every arming-related test below relies on. */
static blimp_imu_sample_t stationary_imu(uint32_t t_us)
{
    blimp_imu_sample_t s = {0};
    s.t_us        = t_us;
    s.gyro[0]     = 0.0f;
    s.gyro[1]     = 0.0f;
    s.gyro[2]     = 0.0f;
    s.accel[0]    = 0.0f;
    s.accel[1]    = 0.0f;
    s.accel[2]    = 9.80665f;
    s.gyro_valid  = true;
    s.accel_valid = true;
    return s;
}

/* Every invariant blimp_mixer_apply() promises (mixer.h), re-checked here
 * at the very end of the whole controller stack rather than in isolation -
 * this is the actual last stop before real motors. Returns violation count
 * so a caller can accumulate across many ticks and report just the total. */
static int actuator_violations(const blimp_actuators_t *a)
{
    if (!isfinite(a->thrust_fl) || !isfinite(a->thrust_fr) ||
        !isfinite(a->thrust_rl) || !isfinite(a->thrust_rr)) {
        return 1;
    }
    int bad = 0;
    if (a->thrust_fl < 0.0f || a->thrust_fr < 0.0f ||
        a->thrust_rl < 0.0f || a->thrust_rr < 0.0f) {
        bad++;
    }
    const float total = a->thrust_fl + a->thrust_fr + a->thrust_rl + a->thrust_rr;
    if (total > BLIMP_RULE13_MAX_TOTAL_N + 1e-6f) {
        bad++;
    }
    return bad;
}

/* "Nothing is ever NaN" applies to the estimated state too, not only the
 * actuator output - a NaN that hasn't reached the mixer yet is still a
 * real bug waiting to happen the next tick. */
static int state_violations(const blimp_state_t *s)
{
    int bad = 0;
    if (!isfinite(s->roll))     bad++;
    if (!isfinite(s->pitch))    bad++;
    if (!isfinite(s->yaw))      bad++;
    if (!isfinite(s->altitude)) bad++;
    if (!isfinite(s->vz))       bad++;
    if (!isfinite(s->yaw_rate)) bad++;
    for (int i = 0; i < 3; i++) {
        if (!isfinite(s->gyro[i]))  bad++;
        if (!isfinite(s->accel[i])) bad++;
    }
    return bad;
}

/* Fast-forwards a freshly-initialised b through the arming gate using a
 * cooperative, perfectly-stationary sequence (1 s/tick - see file header),
 * keeping the ground link fresh throughout so link staleness cannot be what
 * blocks it. Returns the t_us of the tick that completed arming, so the
 * caller can keep advancing the same clock. Fails the test outright (does
 * not return a "maybe") if arming never completes in a generous budget -
 * that is a real bug in this file's own model of convergence, or in the
 * arming path, not an expected outcome. */
static uint32_t force_arm(blimp_t *b)
{
    blimp_init(b);
    uint32_t t_us = 0;
    for (int i = 0; i < 200; i++) {
        t_us += 1000000u;   /* 1 s */
        blimp_input_t in = {0};
        in.imu         = stationary_imu(t_us);
        in.arm_request = true;
        in.t_us        = t_us;
        blimp_notify_link(b, t_us);
        blimp_actuators_t out;
        blimp_step(b, &in, &out);
        if (b->armed) {
            return t_us;
        }
    }
    CHECK_MSG(false,
             "vehicle never armed within budget (stationary_s=%.3f)",
             (double)b->est.stationary_s);
    return t_us;
}

int main(void)
{
    printf("test_blimp\n");

    /* ==================== null safety (hard rule 7) ==================== */
    TEST_GROUP("null safety");
    {
        blimp_init(NULL);                 /* must not crash */
        blimp_notify_link(NULL, 123u);     /* must not crash */
        CHECK(blimp_version() != NULL);

        blimp_step(NULL, NULL, NULL);      /* must not crash */

        blimp_actuators_t out;
        blimp_step(NULL, NULL, &out);      /* out must still come back safe */
        CHECK(actuator_violations(&out) == 0);
        CHECK_NEAR(out.thrust_fl, 0.0f, 1e-9);
        CHECK_NEAR(out.thrust_fr, 0.0f, 1e-9);

        blimp_t b;
        blimp_init(&b);
        blimp_step(&b, NULL, &out);        /* in==NULL: still safe */
        CHECK(actuator_violations(&out) == 0);

        blimp_step(&b, NULL, NULL);        /* both null-ish: no crash */

        blimp_input_t in = {0};
        in.imu = stationary_imu(5000u);
        in.t_us = 5000u;
        blimp_step(&b, &in, NULL);          /* out==NULL: no crash */
    }

    /* ==================== fresh init ==================================== */
    TEST_GROUP("blimp_init leaves a disarmed, zeroed, safe controller");
    {
        blimp_t b;
        blimp_init(&b);
        CHECK(!b.armed);
        CHECK(b.last_cmd_us == 0u);
        CHECK(b.last_step_us == 0u);
        CHECK_NEAR(b.demand.fx, 0.0f, 1e-9);
        CHECK_NEAR(b.demand.fy, 0.0f, 1e-9);
        CHECK_NEAR(b.demand.fz, 0.0f, 1e-9);
        CHECK_NEAR(b.demand.tz, 0.0f, 1e-9);
        CHECK(actuator_violations(&b.act) == 0);
    }

    /* ==================== disarmed behaviour ============================= */
    TEST_GROUP("disarmed: safe zero hold, guidance reported IDLE");
    {
        blimp_t b;
        blimp_init(&b);
        uint32_t t_us = 0;
        int violations = 0;
        for (int i = 0; i < 20; i++) {
            t_us += 5000u;   /* ordinary 5 ms / 200 Hz period */
            blimp_input_t in = {0};
            in.imu         = stationary_imu(t_us);
            in.arm_request = false;
            in.t_us        = t_us;
            blimp_actuators_t out;
            blimp_step(&b, &in, &out);

            CHECK(!b.armed);
            CHECK(b.guid.state == BLIMP_GUID_IDLE);
            CHECK_NEAR(out.thrust_fl, 0.0f, 1e-9);
            CHECK_NEAR(out.thrust_fr, 0.0f, 1e-9);
            CHECK_NEAR(out.thrust_rl, 0.0f, 1e-9);
            violations += actuator_violations(&out);
        }
        CHECK(violations == 0);
    }

    /* ==================== arming gate ==================================== */
    TEST_GROUP("arming is blocked while the estimator converges");
    {
        blimp_t b;
        blimp_init(&b);
        uint32_t t_us = 0;
        bool ever_armed_early = false;
        int  first_armed_tick = -1;

        for (int i = 1; i <= 90; i++) {
            t_us += 1000000u;   /* 1 s/tick, see file header */
            blimp_input_t in = {0};
            in.imu         = stationary_imu(t_us);
            in.arm_request = true;
            in.t_us        = t_us;
            blimp_notify_link(&b, t_us);
            blimp_actuators_t out;
            blimp_step(&b, &in, &out);

            if (i <= 55 && b.armed) {
                ever_armed_early = true;
            }
            if (first_armed_tick < 0 && b.armed) {
                first_armed_tick = i;
            }
        }
        CHECK_MSG(!ever_armed_early,
                 "armed before the estimator plausibly converged (tick<=55)");
        CHECK_MSG(first_armed_tick > 0,
                 "vehicle never armed at all across 90 s of stationary dwell");
    }

    TEST_GROUP("arming is a latch: re-converging mid-flight does not disarm");
    {
        blimp_t b;
        uint32_t t_us = force_arm(&b);
        CHECK(b.armed);

        /* A real rotation resets e.stationary_s to 0 (estimator.c), making
         * blimp_estimator_converging() true again - decision 3's exact
         * documented scenario. armed must stay latched regardless. */
        for (int i = 0; i < 5; i++) {
            t_us += 5000u;
            blimp_input_t in = {0};
            in.imu          = stationary_imu(t_us);
            in.imu.gyro[2]  = 3.0f;   /* well above the stationary threshold */
            in.arm_request  = true;
            in.t_us         = t_us;
            blimp_notify_link(&b, t_us);
            blimp_actuators_t out;
            blimp_step(&b, &in, &out);
            CHECK(b.armed);
        }
        CHECK_MSG(blimp_estimator_converging(&b.est),
                 "test premise failed: motion did not make the estimator "
                 "re-report 'converging'");
    }

    TEST_GROUP("explicit disarm always works");
    {
        blimp_t b;
        uint32_t t_us = force_arm(&b);
        CHECK(b.armed);

        blimp_input_t in = {0};
        in.imu         = stationary_imu(t_us + 5000u);
        in.arm_request = false;
        in.t_us        = t_us + 5000u;
        blimp_notify_link(&b, in.t_us);
        blimp_actuators_t out;
        blimp_step(&b, &in, &out);
        CHECK(!b.armed);
        CHECK(b.guid.state == BLIMP_GUID_IDLE);
    }

    /* ==================== failsafe: link timeout ========================= */
    TEST_GROUP("failsafe: ground-link staleness");
    {
        blimp_t b;
        uint32_t t_us = force_arm(&b);
        CHECK(b.armed);
        CHECK(b.guid.state != BLIMP_GUID_FAILSAFE);

        /* No further blimp_notify_link() calls: jump well past the default
         * link_timeout_s (1.0 s). */
        t_us += 5000000u;   /* +5 s in one tick */
        blimp_input_t in = {0};
        in.imu         = stationary_imu(t_us);
        in.arm_request = true;
        in.t_us        = t_us;
        blimp_actuators_t out;
        blimp_step(&b, &in, &out);

        CHECK(b.guid.state == BLIMP_GUID_FAILSAFE);
        CHECK(b.armed);   /* failsafe holds; it does not disarm */
        CHECK_NEAR(out.thrust_fl, 0.0f, 1e-9);
        CHECK_NEAR(out.thrust_fr, 0.0f, 1e-9);
        CHECK_NEAR(out.thrust_rl, 0.0f, 1e-9);
        CHECK(actuator_violations(&out) == 0);
    }

    /* ==================== failsafe: ceiling breach ======================= */
    TEST_GROUP("failsafe: ceiling breach");
    {
        blimp_t b;
        uint32_t t_us = force_arm(&b);
        CHECK(b.armed);

        bool saw_failsafe = false;
        int violations = 0;
        for (int i = 0; i < 150 && !saw_failsafe; i++) {
            t_us += 1000000u;   /* 1 s/tick */
            blimp_input_t in = {0};
            in.imu           = stationary_imu(t_us);
            in.imu.baro_valid = true;
            in.imu.baro_alt   = 10.0f;   /* well above default max_altitude_m=3.5 */
            in.arm_request   = true;
            in.t_us          = t_us;
            blimp_notify_link(&b, t_us);   /* keep link fresh: isolate this check */
            blimp_actuators_t out;
            blimp_step(&b, &in, &out);
            violations += actuator_violations(&out);
            if (b.guid.state == BLIMP_GUID_FAILSAFE) {
                saw_failsafe = true;
            }
        }
        CHECK_MSG(saw_failsafe,
                 "ceiling breach never engaged failsafe (final altitude=%.3f, "
                 "max=%.3f)", (double)b.state.altitude,
                 (double)b.params.max_altitude_m);
        CHECK(violations == 0);
    }

    /* ==================== guidance progress ============================== */
    TEST_GROUP("guidance progresses under cooperative observations");
    {
        blimp_t b;
        uint32_t t_us = force_arm(&b);
        CHECK(b.armed);
        const int initial_wp = b.guid.wp_index;

        bool left_prepare_or_idle = false;
        bool wp_or_loop_advanced  = false;
        bool has_elapsed_time     = false;
        int  violations           = 0;

        for (int i = 0; i < 4000; i++) {
            t_us += 100000u;   /* 0.1 s/tick */
            blimp_input_t in = {0};
            in.imu                  = stationary_imu(t_us);
            in.arm_request          = true;
            in.contact_detected     = true;   /* maximally cooperative touch */
            in.target.visible       = true;
            in.target.confidence    = 1.0f;
            in.target.bearing_rad   = 0.0f;    /* perfectly aligned */
            in.target.elevation_rad = 0.0f;
            in.target.range_m       = 0.30f;   /* inside default touch radius */
            in.target.t_us          = t_us;
            in.t_us                 = t_us;
            blimp_notify_link(&b, t_us);

            blimp_actuators_t out;
            blimp_step(&b, &in, &out);

            violations += actuator_violations(&out);
            violations += state_violations(&b.state);
            CHECK(b.guid.state >= BLIMP_GUID_IDLE &&
                 b.guid.state <= BLIMP_GUID_FAILSAFE);

            if (b.guid.state != BLIMP_GUID_IDLE &&
                b.guid.state != BLIMP_GUID_PREPARE &&
                b.guid.state != BLIMP_GUID_FAILSAFE) {
                left_prepare_or_idle = true;
            }
            if (b.guid.wp_index != initial_wp || b.guid.loop_index > 0) {
                wp_or_loop_advanced = true;
            }
            if (blimp_guidance_elapsed_s(&b.guid, t_us) > 0.0f) {
                has_elapsed_time = true;
            }
        }

        CHECK(violations == 0);
        CHECK_MSG(left_prepare_or_idle,
                 "guidance never left IDLE/PREPARE/FAILSAFE under 400 s of "
                 "fully cooperative input (final state=%s)",
                 blimp_guidance_state_name(b.guid.state));
        CHECK_MSG(wp_or_loop_advanced || has_elapsed_time,
                 "no observable guidance progress: wp_index/loop_index "
                 "unchanged and elapsed_s stayed 0 throughout");
    }

    /* ==================== corrupted params are clamped, every tick ======== */
    TEST_GROUP("corrupted params never reach the mixer unclamped");
    {
        blimp_t b;
        blimp_init(&b);
        b.params.max_total_thrust_n  = BLIMP_GF_TO_N(100000.0f);
        b.params.heave_budget_n = (float)NAN;

        blimp_input_t in = {0};
        in.imu         = stationary_imu(5000u);
        in.arm_request = false;
        in.t_us        = 5000u;
        blimp_actuators_t out;
        blimp_step(&b, &in, &out);

        CHECK(actuator_violations(&out) == 0);
        CHECK(b.params.max_total_thrust_n <= BLIMP_RULE13_MAX_TOTAL_N + 1e-9f);
    }

    /* ==================== THE DISQUALIFICATION TEST ======================
     * A long, deterministic, adversarial sweep across the whole controller
     * stack: varied/degenerate gyro and accel readings (including NaN/Inf),
     * varied tick-to-tick dt (normal, tiny, absurdly large, stuck-at-zero),
     * arm requests toggling on and off, contact toggling, a mix of visible
     * and invisible/degenerate target observations, and an intermittently
     * fed/starved ground link. No single dimension is "realistic" on its
     * own; the point is that every combination must still fail safe. */
    TEST_GROUP("Rule 13 / finiteness hold across a long adversarial sweep");
    {
        blimp_t b;
        blimp_init(&b);
        uint32_t t_us = 0;
        long samples = 0;
        int violations = 0;
        int combo_index = 0;

        static const float gyros[] = {
            0.0f, 0.5f, 50.0f, (float)NAN, (float)INFINITY
        };
        static const float accels[] = {
            9.80665f, 0.0f, 500.0f, (float)NAN
        };
        static const uint32_t deltas[] = { 5000u, 1u, 3000000u, 0u };
        static const float bearings[] = { 0.0f, 4.0f, (float)NAN };
        static const float ranges[]   = { 0.1f, 50.0f, (float)NAN };

        for (unsigned gi = 0; gi < sizeof(gyros) / sizeof(gyros[0]); gi++)
        for (unsigned ai = 0; ai < sizeof(accels) / sizeof(accels[0]); ai++)
        for (unsigned di = 0; di < sizeof(deltas) / sizeof(deltas[0]); di++)
        for (int arm = 0; arm < 2; arm++)
        for (int contact = 0; contact < 2; contact++)
        for (unsigned bi = 0; bi < sizeof(bearings) / sizeof(bearings[0]); bi++)
        for (unsigned ri = 0; ri < sizeof(ranges) / sizeof(ranges[0]); ri++) {
            t_us += deltas[di];

            blimp_input_t in = {0};
            in.imu.t_us        = t_us;
            in.imu.gyro[0]     = gyros[gi];
            in.imu.gyro[1]     = 0.0f;
            in.imu.gyro[2]     = gyros[gi];
            in.imu.accel[0]    = 0.0f;
            in.imu.accel[1]    = 0.0f;
            in.imu.accel[2]    = accels[ai];
            in.imu.gyro_valid  = true;
            in.imu.accel_valid = true;
            in.imu.mag_valid   = false;
            in.imu.baro_valid  = (combo_index % 5 == 0);
            in.imu.baro_alt    = accels[ai];
            in.imu.range_valid = (combo_index % 7 == 0);
            in.imu.range_alt   = bearings[bi];

            in.arm_request       = (arm != 0);
            in.contact_detected  = (contact != 0);
            in.target.visible    = ((combo_index % 3) != 0);
            in.target.confidence = 1.0f;
            in.target.bearing_rad   = bearings[bi];
            in.target.elevation_rad = 0.0f;
            in.target.range_m       = ranges[ri];
            in.target.t_us          = t_us;
            in.t_us                 = t_us;

            if (combo_index % 2 == 0) {
                blimp_notify_link(&b, t_us);
            }

            blimp_actuators_t out;
            blimp_step(&b, &in, &out);

            violations += actuator_violations(&out);
            violations += state_violations(&b.state);
            if (!isfinite(b.demand.fx) || !isfinite(b.demand.fy) ||
                !isfinite(b.demand.fz) || !isfinite(b.demand.tz)) {
                violations++;
            }

            samples++;
            combo_index++;
        }

        printf("    swept %ld ticks\n", samples);
        CHECK_MSG(samples > 2000, "only %ld combinations swept", samples);
        CHECK_MSG(violations == 0,
                 "%d invariant violations across %ld swept ticks",
                 violations, samples);
    }

    /* ==================== the bench hold (blimp.h, blimp_hold_t) ========== */
    TEST_GROUP("bench hold: heading only, no heave, the right motor turns CCW");
    {
        /* 20 ms ticks after arming; the link kept fresh. The vehicle is
         * still, so the yaw estimate stays at the arm-time origin (0). */
        blimp_t b;
        uint32_t t_us = force_arm(&b);
        blimp_actuators_t out = {0};
        blimp_hold_t h = {0};
        h.active = true;
        h.heading_held = true;

        h.heading_rad = 0.5f;               /* turn left: CCW, +tz */
        for (int i = 0; i < 3; i++) {
            t_us += 20000u;
            blimp_input_t in = {0};
            in.imu = stationary_imu(t_us);
            in.arm_request = true;
            in.t_us = t_us;
            in.hold = h;
            blimp_notify_link(&b, t_us);
            blimp_step(&b, &in, &out);
        }
        CHECK(b.armed);
        CHECK(b.demand.tz > 0.49f && b.demand.tz < 0.51f);
        CHECK(out.thrust_fr > out.thrust_fl);           /* mixer.h: right harder = CCW */
        CHECK_NEAR(out.thrust_rl, 0.0f, 1e-9);          /* altitude not held: */
        CHECK_NEAR(out.thrust_rr, 0.0f, 1e-9);          /* no hover feed-forward */
        CHECK(actuator_violations(&out) == 0);

        h.heading_rad = -0.5f;              /* turn right: CW, -tz */
        t_us += 20000u;
        blimp_input_t in = {0};
        in.imu = stationary_imu(t_us);
        in.arm_request = true;
        in.t_us = t_us;
        in.hold = h;
        blimp_notify_link(&b, t_us);
        blimp_step(&b, &in, &out);
        CHECK(out.thrust_fl > out.thrust_fr);
        CHECK(actuator_violations(&out) == 0);

        h.heading_rad = 0.0f;               /* on the setpoint, at rest */
        for (int i = 0; i < 3; i++) {
            t_us += 20000u;
            in.imu = stationary_imu(t_us);
            in.t_us = t_us;
            in.hold = h;
            blimp_notify_link(&b, t_us);
            blimp_step(&b, &in, &out);
        }
        CHECK_NEAR(out.thrust_fl, 0.0f, 1e-6);
        CHECK_NEAR(out.thrust_fr, 0.0f, 1e-6);

        /* A setpoint across the +-pi seam takes the short way round:
         * -3.0 rad from 0 is a CCW turn of 3.28 rad the long way, so the
         * wrapped error is -3.0 (CW), not +3.28. */
        h.heading_rad = -3.0f;
        t_us += 20000u;
        in.imu = stationary_imu(t_us);
        in.t_us = t_us;
        in.hold = h;
        blimp_notify_link(&b, t_us);
        blimp_step(&b, &in, &out);
        CHECK(b.demand.tz < -2.9f && b.demand.tz > -3.1f);

        /* Non-finite setpoint: the axis outputs nothing. */
        h.heading_rad = NAN;
        t_us += 20000u;
        in.imu = stationary_imu(t_us);
        in.t_us = t_us;
        in.hold = h;
        blimp_notify_link(&b, t_us);
        blimp_step(&b, &in, &out);
        CHECK_NEAR(out.thrust_fl, 0.0f, 1e-9);
        CHECK_NEAR(out.thrust_fr, 0.0f, 1e-9);
        CHECK(actuator_violations(&out) == 0);
    }

    TEST_GROUP("bench hold: altitude, surge, and what is not held");
    {
        blimp_t b;
        uint32_t t_us = force_arm(&b);
        blimp_actuators_t out = {0};
        blimp_input_t in = {0};
        blimp_hold_t h = {0};
        h.active = true;

        /* Altitude held but no altitude source: nothing on the rear pair. */
        h.altitude_held = true;
        h.altitude_m = 1.5f;
        t_us += 20000u;
        in.imu = stationary_imu(t_us);
        in.arm_request = true;
        in.t_us = t_us;
        in.hold = h;
        blimp_notify_link(&b, t_us);
        blimp_step(&b, &in, &out);
        CHECK(!b.state.altitude_valid);
        CHECK_NEAR(out.thrust_rl + out.thrust_rr, 0.0f, 1e-9);

        /* With the ToF at 1.0 m: below the setpoint, the pair climbs, with
         * the hover feed-forward in it; the front pair stays off. */
        for (int i = 0; i < 25; i++) {
            t_us += 20000u;
            in.imu = stationary_imu(t_us);
            in.imu.range_alt = 1.0f;
            in.imu.range_valid = true;
            in.t_us = t_us;
            in.hold = h;
            blimp_notify_link(&b, t_us);
            blimp_step(&b, &in, &out);
        }
        CHECK(b.state.altitude_valid);
        CHECK(out.thrust_rl + out.thrust_rr > b.params.hover_thrust_n);
        CHECK_NEAR(out.thrust_fl + out.thrust_fr, 0.0f, 1e-9);
        CHECK(actuator_violations(&out) == 0);

        /* Above the setpoint: the pair backs off below the feed-forward,
         * never below zero. */
        h.altitude_m = 0.5f;
        for (int i = 0; i < 5; i++) {
            t_us += 20000u;
            in.imu = stationary_imu(t_us);
            in.imu.range_alt = 1.0f;
            in.imu.range_valid = true;
            in.t_us = t_us;
            in.hold = h;
            blimp_notify_link(&b, t_us);
            blimp_step(&b, &in, &out);
        }
        CHECK(out.thrust_rl + out.thrust_rr < b.params.hover_thrust_n);
        CHECK(actuator_violations(&out) == 0);

        /* Surge alone: an even push on the front pair, capped at the
         * cruise cap however large the request. */
        h.altitude_held = false;
        h.surge_n = 0.05f;
        t_us += 20000u;
        in.imu = stationary_imu(t_us);
        in.t_us = t_us;
        in.hold = h;
        blimp_notify_link(&b, t_us);
        blimp_step(&b, &in, &out);
        CHECK_NEAR(out.thrust_fl + out.thrust_fr, 0.05f, 1e-5);
        CHECK_NEAR(out.thrust_fl, out.thrust_fr, 1e-6);
        CHECK_NEAR(out.thrust_rl + out.thrust_rr, 0.0f, 1e-9);
        h.surge_n = 1.0e6f;
        t_us += 20000u;
        in.imu = stationary_imu(t_us);
        in.t_us = t_us;
        in.hold = h;
        blimp_notify_link(&b, t_us);
        blimp_step(&b, &in, &out);
        CHECK(out.thrust_fl + out.thrust_fr <= b.params.surge.out_max + 1e-5f);
        CHECK(actuator_violations(&out) == 0);

        /* Active but nothing held: all zero, guidance not consulted. */
        h = (blimp_hold_t){0};
        h.active = true;
        t_us += 20000u;
        in.imu = stationary_imu(t_us);
        in.t_us = t_us;
        in.hold = h;
        blimp_notify_link(&b, t_us);
        blimp_step(&b, &in, &out);
        CHECK_NEAR(out.thrust_fl + out.thrust_fr + out.thrust_rl + out.thrust_rr, 0.0f, 1e-9);
    }

    TEST_GROUP("bench hold: failsafes and disarm still win");
    {
        blimp_t b;
        uint32_t t_us = force_arm(&b);
        blimp_actuators_t out = {0};
        blimp_hold_t h = {0};
        h.active = true;
        h.heading_held = true;
        h.heading_rad = 1.0f;
        h.surge_n = 0.05f;

        /* Link silent past link_timeout_s: zero. */
        t_us += 5000000u;
        blimp_input_t in = {0};
        in.imu = stationary_imu(t_us);
        in.arm_request = true;
        in.t_us = t_us;
        in.hold = h;
        blimp_step(&b, &in, &out);
        CHECK(b.guid.state == BLIMP_GUID_FAILSAFE);
        CHECK_NEAR(out.thrust_fl + out.thrust_fr + out.thrust_rl + out.thrust_rr, 0.0f, 1e-9);

        /* Link back: the hold resumes (guidance stays parked, not stepped). */
        t_us += 20000u;
        in.imu = stationary_imu(t_us);
        in.t_us = t_us;
        blimp_notify_link(&b, t_us);
        blimp_step(&b, &in, &out);
        CHECK(out.thrust_fr > out.thrust_fl);

        /* Disarm request: zero, whatever the hold says. */
        t_us += 20000u;
        in.imu = stationary_imu(t_us);
        in.arm_request = false;
        in.t_us = t_us;
        blimp_notify_link(&b, t_us);
        blimp_step(&b, &in, &out);
        CHECK(!b.armed);
        CHECK_NEAR(out.thrust_fl + out.thrust_fr + out.thrust_rl + out.thrust_rr, 0.0f, 1e-9);
    }

    TEST_REPORT();
}
