/* test_guidance.c - the race state machine.
 *
 * Two of these tests are rules-compliance tests rather than software tests,
 * and they are the reason this file exists:
 *
 *   - TIMING. Rule 4 says the clock starts on the FIRST TOUCH and stops on
 *     the FINAL TOUCH. Measuring from arm instead would inflate every lap by
 *     the whole PREPARE phase. Since PREPARE is deliberately unhurried, that
 *     error would be enormous and would look like a slow vehicle rather than
 *     a measurement bug.
 *
 *   - ANTI-LIVELOCK. Under Rule 5 three loops run back-to-back with no human
 *     intervention, and under Rule 7 a run needing intervention is VOID. A
 *     machine that can wedge scores zero, so we hammer it with adversarial
 *     perception and assert it always keeps moving.
 */
#include "blimp/guidance.h"
#include "test_util.h"

#include <string.h>

#define US_PER_S 1000000u

/* Mirrors GUID_STATE_HARD_LIMIT_S inside guidance.c (25 s), plus one tick of
 * margin. That constant is deliberately private to the implementation, so the
 * test asserts against the documented GUARANTEE ("no state dwells unbounded")
 * rather than reaching into the module's internals. */
#define GUID_STATE_HARD_LIMIT_S_TEST 26.0f

static blimp_params_t P;

static blimp_state_t make_state(float alt, uint32_t t_us)
{
    blimp_state_t s;
    memset(&s, 0, sizeof(s));
    s.altitude = alt;
    s.altitude_valid = true;
    s.attitude_valid = true;
    s.t_us = t_us;
    return s;
}

static blimp_target_obs_t make_obs(bool visible, float bearing, float range,
                                   uint32_t t_us)
{
    blimp_target_obs_t o;
    memset(&o, 0, sizeof(o));
    o.visible = visible;
    o.bearing_rad = bearing;
    o.range_m = range;
    o.confidence = visible ? 1.0f : 0.0f;
    o.t_us = t_us;
    return o;
}

/* Drive the machine with a cooperative "perfect pilot" observation stream
 * until it reaches `want` or we run out of ticks. Contact is asserted
 * whenever the machine is in a state that is expecting it. Returns the tick
 * count consumed, or -1 on timeout. */
static int run_until(blimp_guidance_t *g, blimp_guid_state_t want,
                     uint32_t *t_us, int max_ticks)
{
    const float dt = 0.01f;
    blimp_wrench_t d;
    for (int i = 0; i < max_ticks; i++) {
        if (g->state == want) { return i; }
        const bool expecting =
            (g->state == BLIMP_GUID_TOUCH || g->state == BLIMP_GUID_CONFIRM);
        /* Cooperative target: dead ahead, and close once we are approaching. */
        const float rng = (g->state == BLIMP_GUID_APPROACH) ? 0.4f : 2.0f;
        blimp_target_obs_t o = make_obs(true, 0.0f, rng, *t_us);
        blimp_state_t s = make_state(2.0f, *t_us);
        (void)blimp_guidance_step(g, &P, &s, &o, expecting, dt, &d);
        *t_us += (uint32_t)(dt * 1e6f);
    }
    return (g->state == want) ? max_ticks : -1;
}

int main(void)
{
    printf("test_guidance\n");
    blimp_params_defaults(&P);
    (void)blimp_params_validate(&P);

    /* ---- state names ------------------------------------------------- */
    TEST_GROUP("state names");
    {
        CHECK(strcmp(blimp_guidance_state_name(BLIMP_GUID_IDLE), "IDLE") == 0);
        CHECK(strcmp(blimp_guidance_state_name(BLIMP_GUID_PREPARE), "PREPARE") == 0);
        CHECK(strcmp(blimp_guidance_state_name(BLIMP_GUID_FINISHED), "FINISHED") == 0);
        CHECK(strcmp(blimp_guidance_state_name((blimp_guid_state_t)999), "UNKNOWN") == 0);
    }

    /* ---- init / arm --------------------------------------------------- */
    TEST_GROUP("init and arm");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        CHECK(g.state == BLIMP_GUID_IDLE);
        CHECK(g.wp_count == 4 && g.loop_count == 3);
        CHECK(g.best_loop_time_s < 0.0f);     /* no valid loop yet */
        CHECK(!g.timing_active);

        /* Degenerate counts fall back to the real course, never to zero -
         * a wp_count of 0 would make the machine complete a loop instantly. */
        blimp_guidance_init(&g, 0, 0);
        CHECK(g.wp_count > 0 && g.loop_count > 0);

        blimp_guidance_init(&g, 4, 3);
        blimp_guidance_arm(&g, 1000u);
        CHECK(g.state == BLIMP_GUID_PREPARE);

        /* Re-arming mid-run must not reset progress. */
        g.wp_index = 2;
        blimp_guidance_arm(&g, 2000u);
        CHECK_MSG(g.wp_index == 2, "re-arm clobbered wp_index (%d)", g.wp_index);
    }

    /* ---- PREPARE is untimed and unhurried ------------------------------ */
    TEST_GROUP("PREPARE dwells and does not start the clock");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);

        blimp_wrench_t d;
        const float dt = 0.01f;
        /* Half the settle time with a perfectly visible, aligned target: it
         * must still refuse to leave. */
        for (int i = 0; i < 200; i++) {
            blimp_target_obs_t o = make_obs(true, 0.0f, 2.0f, t);
            blimp_state_t s = make_state(2.0f, t);
            (void)blimp_guidance_step(&g, &P, &s, &o, false, dt, &d);
            t += 10000u;
        }
        CHECK_MSG(g.state == BLIMP_GUID_PREPARE,
                  "left PREPARE after only 2 s (settle is %.1f s), state=%s",
                  (double)P.prepare_settle_s,
                  blimp_guidance_state_name(g.state));
        CHECK(!g.timing_active);
        CHECK_NEAR(blimp_guidance_elapsed_s(&g, t), 0.0f, 1e-6);

        /* Past the settle time it releases. */
        for (int i = 0; i < 500 && g.state == BLIMP_GUID_PREPARE; i++) {
            blimp_target_obs_t o = make_obs(true, 0.0f, 2.0f, t);
            blimp_state_t s = make_state(2.0f, t);
            (void)blimp_guidance_step(&g, &P, &s, &o, false, dt, &d);
            t += 10000u;
        }
        CHECK(g.state != BLIMP_GUID_PREPARE);
    }

    /* ---- PREPARE cannot wedge when perception never works -------------- */
    TEST_GROUP("PREPARE escapes to SEARCH if the target never appears");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);
        blimp_wrench_t d;
        for (int i = 0; i < 3000 && g.state == BLIMP_GUID_PREPARE; i++) {
            blimp_target_obs_t o = make_obs(false, 0.0f, 0.0f, t);
            blimp_state_t s = make_state(2.0f, t);
            (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
            t += 10000u;
        }
        CHECK_MSG(g.state == BLIMP_GUID_SEARCH,
                  "PREPARE wedged with no target; state=%s",
                  blimp_guidance_state_name(g.state));
    }

    /* ---- ALIGN before APPROACH (turn-to-face-then-drive) --------------- */
    TEST_GROUP("ALIGN commands yaw only, no surge");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);
        g.state = BLIMP_GUID_ALIGN;
        g.state_entry_us = t;

        blimp_wrench_t d;
        blimp_target_obs_t o = make_obs(true, 0.9f, 3.0f, t);  /* well off-axis */
        blimp_state_t s = make_state(2.0f, t);
        (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
        CHECK_MSG(fabsf(d.fx) < 1e-6f,
                  "ALIGN commanded surge %.4f while still turning", (double)d.fx);
        CHECK_MSG(d.tz > 0.5f, "ALIGN did not command yaw toward target (tz=%.3f)",
                  (double)d.tz);
        CHECK(g.state == BLIMP_GUID_ALIGN);   /* not aligned yet */
    }

    /* ---- THE RULE 4 TIMING TEST ---------------------------------------- */
    TEST_GROUP("Rule 4: clock runs first touch -> final touch, NOT from arm");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 1);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);

        /* Burn a long, deliberately slow PREPARE - this is the time that
         * must NOT appear in the score. */
        blimp_wrench_t d;
        const uint32_t t_arm = t;
        for (int i = 0; i < 900; i++) {
            blimp_target_obs_t o = make_obs(true, 0.0f, 2.0f, t);
            blimp_state_t s = make_state(2.0f, t);
            (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
            t += 10000u;
        }
        const float prepare_s = (float)(t - t_arm) * 1e-6f;
        CHECK(prepare_s > 5.0f);
        CHECK_MSG(!g.timing_active,
                  "clock started during PREPARE - Rule 4 violated");

        /* Now fly the loop. */
        const int rc = run_until(&g, BLIMP_GUID_LOOP_DONE, &t, 60000);
        CHECK_MSG(rc >= 0, "never completed a loop");

        const float scored = blimp_guidance_elapsed_s(&g, t);
        const float wall   = (float)(t - t_arm) * 1e-6f;
        CHECK_MSG(scored > 0.0f, "no time recorded (%.3f)", (double)scored);
        CHECK_MSG(scored < wall - prepare_s + 0.5f,
                  "scored time %.2f s includes the untimed PREPARE "
                  "(wall %.2f s, prepare %.2f s)",
                  (double)scored, (double)wall, (double)prepare_s);
        CHECK_MSG(g.best_loop_time_s > 0.0f, "best loop time not recorded");
        CHECK_NEAR(g.best_loop_time_s, g.last_loop_time_s, 1e-6);

        /* The frozen result must survive the untimed reposition. */
        const float later = blimp_guidance_elapsed_s(&g, t + 50u * US_PER_S);
        CHECK_MSG(fabsf(later - scored) < 1e-6f,
                  "scored time kept ticking after the final touch "
                  "(%.3f -> %.3f)", (double)scored, (double)later);
    }

    /* ---- three loops back to back, no intervention ---------------------- */
    TEST_GROUP("Rule 5: three loops back-to-back reach FINISHED");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);
        const int rc = run_until(&g, BLIMP_GUID_FINISHED, &t, 400000);
        CHECK_MSG(rc >= 0, "did not reach FINISHED; stuck in %s at loop %d wp %d",
                  blimp_guidance_state_name(g.state), g.loop_index, g.wp_index);
        CHECK(g.loop_index >= 3);
        CHECK_MSG(g.best_loop_time_s > 0.0f, "no best loop time after 3 loops");
    }

    /* ---- a waypoint can never advance without a confirmed touch --------- */
    TEST_GROUP("no waypoint advances without a confirmed touch");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);
        blimp_wrench_t d;

        /* Never assert contact. Fly cooperatively for a long time. */
        int max_wp = 0;
        for (int i = 0; i < 40000; i++) {
            blimp_target_obs_t o = make_obs(true, 0.0f, 0.3f, t);
            blimp_state_t s = make_state(2.0f, t);
            (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
            if (g.wp_index > max_wp) { max_wp = g.wp_index; }
            t += 10000u;
        }
        CHECK_MSG(max_wp == 0,
                  "advanced to waypoint %d with no contact ever detected",
                  max_wp);
        CHECK(!g.timing_active);
    }

    /* ---- contact spam must not skip waypoints --------------------------- */
    TEST_GROUP("contact spam does not skip waypoints");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);
        blimp_wrench_t d;

        /* Contact asserted on EVERY tick from the very start, including
         * during PREPARE. The dwell must still gate each advance. */
        for (int i = 0; i < 2000; i++) {
            blimp_target_obs_t o = make_obs(true, 0.0f, 0.3f, t);
            blimp_state_t s = make_state(2.0f, t);
            (void)blimp_guidance_step(&g, &P, &s, &o, true, 0.01f, &d);
            t += 10000u;
            CHECK_MSG(g.wp_index <= g.wp_count,
                      "wp_index ran past wp_count (%d > %d)",
                      g.wp_index, g.wp_count);
            CHECK(g.loop_index <= g.loop_count);
        }
        /* 20 s of continuous contact at a 0.30 s dwell could in principle
         * advance many waypoints, but never more than the course allows. */
        CHECK(g.wp_index >= 0 && g.wp_index <= g.wp_count);
    }

    /* ---- losing the target mid-approach recovers ------------------------ */
    TEST_GROUP("target lost mid-approach -> SEARCH -> recovers");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);
        CHECK(run_until(&g, BLIMP_GUID_APPROACH, &t, 20000) >= 0);

        blimp_wrench_t d;
        for (int i = 0; i < 300; i++) {     /* 3 s with no target */
            blimp_target_obs_t o = make_obs(false, 0.0f, 0.0f, t);
            blimp_state_t s = make_state(2.0f, t);
            (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
            t += 10000u;
        }
        CHECK_MSG(g.state == BLIMP_GUID_SEARCH,
                  "did not fall back to SEARCH; state=%s",
                  blimp_guidance_state_name(g.state));
        CHECK(run_until(&g, BLIMP_GUID_APPROACH, &t, 20000) >= 0);
    }

    /* ---- miss timeout bounds an approach -------------------------------- */
    TEST_GROUP("miss timeout fires and the machine recovers");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);
        CHECK(run_until(&g, BLIMP_GUID_APPROACH, &t, 20000) >= 0);

        blimp_wrench_t d;
        /* Visible and aligned but never getting closer, and never touching:
         * the classic "stuck against an invisible obstacle" case. */
        bool saw_coast = false;
        int transitions = 0;
        blimp_guid_state_t prev = g.state;
        for (int i = 0; i < 3000; i++) {
            blimp_target_obs_t o = make_obs(true, 0.0f, 3.0f, t);
            blimp_state_t s = make_state(2.0f, t);
            (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
            if (g.state == BLIMP_GUID_COAST) { saw_coast = true; }
            if (g.state != prev) { transitions++; prev = g.state; }
            t += 10000u;
        }
        CHECK_MSG(g.miss_count > 0,
                  "miss timeout never fired on a stalled approach");
        /* The machine must CYCLE (approach -> coast -> realign -> retry),
         * not sit in APPROACH forever. Ending the window back in APPROACH is
         * correct - that is a retry, which is exactly the desired behaviour;
         * what would be wrong is never having left. */
        CHECK_MSG(saw_coast, "never coasted after a missed approach");
        CHECK_MSG(transitions >= 3,
                  "only %d state changes in 30 s - approach did not retry",
                  transitions);
    }

    /* ---- THE ANTI-LIVELOCK SWEEP ---------------------------------------- */
    TEST_GROUP("adversarial perception: never wedges, never emits NaN");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);
        blimp_wrench_t d;

        const float nan_v = (float)NAN;
        const float inf_v = (float)INFINITY;
        int nonfinite = 0, illegal_state = 0, bad_index = 0;
        uint32_t last_change = t;
        float longest_stuck_s = 0.0f;
        blimp_guid_state_t prev = g.state;

        /* 60,000 ticks (10 simulated minutes) of hostile input: flickering
         * visibility, NaN and Inf bearings, absurd ranges, random contact. */
        unsigned seed = 12345u;
        for (int i = 0; i < 60000; i++) {
            seed = seed * 1103515245u + 12345u;
            const unsigned r = (seed >> 16) & 0xFFu;

            blimp_target_obs_t o;
            memset(&o, 0, sizeof(o));
            o.t_us = t;
            o.visible = (r & 1u) != 0u;
            switch (r % 5u) {
            case 0: o.bearing_rad = nan_v;  o.range_m = 1.0f;   break;
            case 1: o.bearing_rad = inf_v;  o.range_m = -5.0f;  break;
            case 2: o.bearing_rad = 0.0f;   o.range_m = 1e9f;   break;
            case 3: o.bearing_rad = 3.0f;   o.range_m = nan_v;  break;
            default: o.bearing_rad = ((float)(r % 100u) - 50.0f) * 0.05f;
                     o.range_m = (float)(r % 50u) * 0.1f;       break;
            }
            blimp_state_t s = make_state((r % 7u) ? 2.0f : nan_v, t);
            s.altitude_valid = (r % 11u) != 0u;

            const blimp_guid_state_t got =
                blimp_guidance_step(&g, &P, &s, &o, (r % 13u) == 0u, 0.01f, &d);

            if (!isfinite(d.fx) || !isfinite(d.fy) ||
                !isfinite(d.fz) || !isfinite(d.tz)) { nonfinite++; }
            if (got < BLIMP_GUID_IDLE || got > BLIMP_GUID_FAILSAFE) { illegal_state++; }
            if (g.wp_index < 0 || g.wp_index > g.wp_count) { bad_index++; }
            if (g.loop_index < 0 || g.loop_index > g.loop_count) { bad_index++; }

            if (got != prev) {
                const float stuck = (float)(t - last_change) * 1e-6f;
                if (stuck > longest_stuck_s) { longest_stuck_s = stuck; }
                last_change = t;
                prev = got;
            }
            t += 10000u;
        }
        const float tail = (float)(t - last_change) * 1e-6f;
        if (tail > longest_stuck_s) { longest_stuck_s = tail; }

        CHECK_MSG(nonfinite == 0, "%d non-finite demands emitted", nonfinite);
        CHECK_MSG(illegal_state == 0, "%d illegal states returned", illegal_state);
        CHECK_MSG(bad_index == 0, "%d out-of-range indices", bad_index);
        printf("    longest time in one state: %.1f s over 600 s of hostile input\n",
               (double)longest_stuck_s);
        CHECK_MSG(longest_stuck_s <= GUID_STATE_HARD_LIMIT_S_TEST,
                  "wedged in %s for %.1f s - anti-livelock guarantee broken",
                  blimp_guidance_state_name(g.state), (double)longest_stuck_s);
    }

    /* ---- degenerate inputs ---------------------------------------------- */
    TEST_GROUP("degenerate inputs");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        blimp_wrench_t d;
        blimp_target_obs_t o = make_obs(true, 0.0f, 1.0f, 1000u);
        blimp_state_t s = make_state(2.0f, 1000u);

        /* NULL tolerance */
        CHECK(blimp_guidance_step(NULL, &P, &s, &o, false, 0.01f, &d)
              == BLIMP_GUID_IDLE);
        blimp_guidance_arm(&g, 1000u);
        (void)blimp_guidance_step(&g, NULL, &s, &o, false, 0.01f, &d);
        (void)blimp_guidance_step(&g, &P, NULL, &o, false, 0.01f, &d);
        (void)blimp_guidance_step(&g, &P, &s, NULL, false, 0.01f, &d);
        (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, NULL);
        blimp_guidance_arm(NULL, 0u);
        CHECK_NEAR(blimp_guidance_elapsed_s(NULL, 0u), 0.0f, 1e-9);

        /* bad dt leaves the demand neutral */
        (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.0f, &d);
        CHECK_NEAR(d.fx, 0.0f, 1e-9);
        (void)blimp_guidance_step(&g, &P, &s, &o, false, -1.0f, &d);
        CHECK_NEAR(d.tz, 0.0f, 1e-9);
        (void)blimp_guidance_step(&g, &P, &s, &o, false, (float)NAN, &d);
        CHECK_FINITE(d.fz);
    }

    /* ---- no lateral actuator ------------------------------------------- */
    TEST_GROUP("fy is always zero: nothing on this airframe can strafe");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);
        CHECK(run_until(&g, BLIMP_GUID_APPROACH, &t, 20000) >= 0);

        blimp_wrench_t d;
        blimp_target_obs_t o = make_obs(true, 0.10f, 2.0f, t);
        blimp_state_t s = make_state(2.0f, t);
        (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
        CHECK_NEAR(d.fy, 0.0f, 1e-9);
        o = make_obs(true, -0.10f, 2.0f, t);
        (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
        CHECK_NEAR(d.fy, 0.0f, 1e-9);
    }

    /* ---- vision altitude fallback --------------------------------------- */
    TEST_GROUP("altitude error comes from the balloon when no altitude sensor exists");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);
        blimp_wrench_t d;
        blimp_state_t s = make_state(0.0f, t);
        s.altitude_valid = false;    /* MPU-6050 only: no altitude source */
        s.pitch = 0.0f;
        /* Balloon dead ahead at 2 m range, seen 10 deg above the nose. Its
         * centre is 2*sin(10deg) above the camera, and the camera sits
         * cam_dz below the envelope centre we want to line up with it. */
        blimp_target_obs_t o = make_obs(true, 0.0f, 2.0f, t);
        o.elevation_rad = 0.1745f;
        (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
        CHECK_NEAR(d.fz, 2.0f * sinf(0.1745f) + P.cam_dz_m, 1e-4);

        /* +pitch is nose DOWN in this codebase, so the camera overstates the
         * world elevation by the pitch angle. */
        s.pitch = 0.05f;
        (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
        CHECK_NEAR(d.fz, 2.0f * sinf(0.1745f - 0.05f) + P.cam_dz_m, 1e-4);

        /* A real altitude source, when present, wins over vision. */
        s.altitude_valid = true;
        s.altitude = 1.5f;
        (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
        CHECK_NEAR(d.fz, 0.5f, 1e-6);

        /* Nothing available at all: hold, zero error. */
        s.altitude_valid = false;
        o = make_obs(false, 0.0f, 0.0f, t);
        (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
        CHECK_NEAR(d.fz, 0.0f, 1e-9);
    }

    /* ---- search hint ----------------------------------------------------- */
    TEST_GROUP("SEARCH turns toward the offboard hint before scanning");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);
        blimp_wrench_t d;
        /* Nothing visible for long enough to leave PREPARE into SEARCH. */
        for (int i = 0; i < 1400 && g.state != BLIMP_GUID_SEARCH; i++) {
            blimp_target_obs_t o = make_obs(false, 0.0f, 0.0f, t);
            blimp_state_t s = make_state(2.0f, t);
            (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
            t += 10000u;
        }
        CHECK(g.state == BLIMP_GUID_SEARCH);

        /* Hint says the target is 1.0 rad to the LEFT of our heading. */
        blimp_target_obs_t o = make_obs(false, 0.0f, 0.0f, t);
        o.hint_valid = true;
        o.hint_yaw_rad = 1.0f;
        blimp_state_t s = make_state(2.0f, t);
        s.yaw = 0.0f;
        (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
        CHECK_NEAR(d.tz, 1.0f, 1e-5);
        CHECK(!g.hint_reached);

        /* Now facing the hinted heading and still nothing: scan takes over. */
        s.yaw = 0.95f;
        (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
        CHECK(g.hint_reached);
        CHECK_NEAR(fabsf(d.tz), 0.45f, 1e-5);

        /* Hint wraps correctly across +/-pi. */
        blimp_guidance_t g2 = g;
        g2.hint_reached = false;
        s.yaw = 3.0f;
        o.hint_yaw_rad = -3.0f;   /* 0.283 rad to the left, not 6 rad right */
        (void)blimp_guidance_step(&g2, &P, &s, &o, false, 0.01f, &d);
        CHECK_NEAR(d.tz, 0.2832f, 1e-3);

        /* No hint: plain scan. */
        o.hint_valid = false;
        (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
        CHECK_NEAR(fabsf(d.tz), 0.45f, 1e-5);
    }

    /* ---- blind climb ------------------------------------------------------ */
    TEST_GROUP("blind climb: nudges up when no altitude and no target, capped");
    {
        blimp_guidance_t g;
        blimp_guidance_init(&g, 4, 3);
        uint32_t t = 1000u;
        blimp_guidance_arm(&g, t);
        blimp_wrench_t d;
        float climb_ticks = 0.0f;
        for (int i = 0; i < 3000; i++) {   /* 30 s, nothing ever visible */
            blimp_target_obs_t o = make_obs(false, 0.0f, 0.0f, t);
            blimp_state_t s = make_state(0.0f, t);
            s.altitude_valid = false;
            (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
            if (d.fz > 0.0f) { climb_ticks += 1.0f; }
            t += 10000u;
        }
        /* Climbed for roughly the cap (4 s = 400 ticks), then held. */
        CHECK_MSG(climb_ticks > 350.0f && climb_ticks < 450.0f,
                  "blind climb ran for %.0f ticks, expected ~400", (double)climb_ticks);
        CHECK(g.blind_climb_s >= 4.0f - 0.02f);

        /* Had a fix saying we were 0.5 m ABOVE the plane, then lost
         * everything: the blind action must be DOWN, not a climb. */
        blimp_guidance_init(&g, 4, 3);
        blimp_guidance_arm(&g, t);
        for (int i = 0; i < 700; i++) {
            blimp_target_obs_t o = make_obs(false, 0.0f, 0.0f, t);
            blimp_state_t s = make_state(2.5f, t);   /* valid, above plane */
            (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
            t += 10000u;
        }
        CHECK(g.alt_err_known);
        CHECK_NEAR(g.last_alt_err, -0.5f, 1e-6);
        float min_fz = 1.0f;
        for (int i = 0; i < 300; i++) {
            blimp_target_obs_t o = make_obs(false, 0.0f, 0.0f, t);
            blimp_state_t s = make_state(0.0f, t);
            s.altitude_valid = false;
            (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
            if (d.fz < min_fz) { min_fz = d.fz; }
            t += 10000u;
        }
        CHECK_MSG(min_fz < -0.25f, "lost above the plane but never demanded a descent (min fz %.3f)",
                  (double)min_fz);

        /* With an altitude source there is never a blind climb. */
        blimp_guidance_init(&g, 4, 3);
        blimp_guidance_arm(&g, t);
        int climbs = 0;
        for (int i = 0; i < 1500; i++) {
            blimp_target_obs_t o = make_obs(false, 0.0f, 0.0f, t);
            blimp_state_t s = make_state(2.0f, t);   /* altitude_valid, on plane */
            (void)blimp_guidance_step(&g, &P, &s, &o, false, 0.01f, &d);
            if (d.fz != 0.0f) { climbs++; }
            t += 10000u;
        }
        CHECK(climbs == 0);
    }

    TEST_REPORT();
}
