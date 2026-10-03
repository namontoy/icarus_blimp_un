/* test_pid.c - unit tests for the PID block. */
#include "blimp/pid.h"
#include "test_util.h"

static blimp_pid_params_t basic_params(void)
{
    blimp_pid_params_t p = {0};
    p.kp = 1.0f;
    p.ki = 0.0f;
    p.kd = 0.0f;
    p.i_min = -1.0f;
    p.i_max = 1.0f;
    p.out_min = -10.0f;
    p.out_max = 10.0f;
    p.d_lpf_hz = 0.0f;   /* unfiltered for deterministic arithmetic */
    return p;
}

int main(void)
{
    printf("test_pid\n");

    /* ---- wrap_pi ----------------------------------------------------- */
    TEST_GROUP("wrap_pi");
    CHECK_NEAR(blimp_wrap_pi(0.0f), 0.0f, 1e-6);
    CHECK_NEAR(blimp_wrap_pi(3.0f), 3.0f, 1e-6);
    /* just past +pi must come out just above -pi, not near +pi */
    CHECK_NEAR(blimp_wrap_pi(3.3f), 3.3f - 2.0f * BLIMP_PI, 1e-5);
    CHECK_NEAR(blimp_wrap_pi(-3.3f), -3.3f + 2.0f * BLIMP_PI, 1e-5);
    /* many turns out of range still wraps (fmodf path) */
    CHECK_NEAR(blimp_wrap_pi(100.0f * 2.0f * BLIMP_PI + 0.5f), 0.5f, 1e-3);
    CHECK(blimp_wrap_pi(1.0f / 0.0f - 1.0f / 0.0f) == 0.0f);  /* NaN -> 0 */
    for (int i = -50; i <= 50; i++) {
        const float w = blimp_wrap_pi((float)i * 0.7f);
        CHECK(w > -(BLIMP_PI + 1e-6f) && w <= (BLIMP_PI + 1e-6f));
    }

    /* ---- proportional ------------------------------------------------ */
    TEST_GROUP("proportional");
    {
        blimp_pid_t s;
        blimp_pid_reset(&s);
        blimp_pid_params_t p = basic_params();
        CHECK_NEAR(blimp_pid_step(&s, &p, 2.0f, 0.01f), 2.0f, 1e-6);
        CHECK_NEAR(blimp_pid_step(&s, &p, -3.0f, 0.01f), -3.0f, 1e-6);
    }

    /* ---- output clamping --------------------------------------------- */
    TEST_GROUP("output clamp");
    {
        blimp_pid_t s;
        blimp_pid_reset(&s);
        blimp_pid_params_t p = basic_params();
        CHECK_NEAR(blimp_pid_step(&s, &p, 50.0f, 0.01f), 10.0f, 1e-6);
        CHECK(s.saturated);
        CHECK_NEAR(blimp_pid_step(&s, &p, -50.0f, 0.01f), -10.0f, 1e-6);
        CHECK(s.saturated);
        blimp_pid_reset(&s);
        (void)blimp_pid_step(&s, &p, 1.0f, 0.01f);
        CHECK(!s.saturated);
    }

    /* ---- no derivative kick on first sample -------------------------- */
    TEST_GROUP("no derivative kick");
    {
        blimp_pid_t s;
        blimp_pid_reset(&s);
        blimp_pid_params_t p = basic_params();
        p.kp = 0.0f;
        p.kd = 1.0f;
        /* A large first error must NOT produce a huge derivative spike. */
        CHECK_NEAR(blimp_pid_step(&s, &p, 100.0f, 0.01f), 0.0f, 1e-6);
        /* Second tick with unchanged error is still zero derivative. */
        CHECK_NEAR(blimp_pid_step(&s, &p, 100.0f, 0.01f), 0.0f, 1e-6);
        /* Now a real change: d = (110-100)/0.01 = 1000, clamped to out_max. */
        CHECK_NEAR(blimp_pid_step(&s, &p, 110.0f, 0.01f), 10.0f, 1e-6);
    }

    /* ---- integral accumulation and clamp ------------------------------ */
    TEST_GROUP("integral clamp");
    {
        blimp_pid_t s;
        blimp_pid_reset(&s);
        blimp_pid_params_t p = basic_params();
        p.kp = 0.0f;
        p.ki = 1.0f;
        p.i_max = 0.05f;
        p.i_min = 0.0f;
        for (int i = 0; i < 1000; i++) {
            (void)blimp_pid_step(&s, &p, 1.0f, 0.01f);
        }
        CHECK_MSG(s.integral <= 0.05f + 1e-6f,
                  "integral %.6f exceeded i_max", (double)s.integral);
        CHECK(s.integral >= 0.0f);
    }

    /* ---- conditional integration: no windup while saturated ----------- */
    TEST_GROUP("anti-windup");
    {
        blimp_pid_t s;
        blimp_pid_reset(&s);
        blimp_pid_params_t p = basic_params();
        p.kp = 100.0f;      /* guarantees saturation */
        p.ki = 10.0f;
        p.i_max = 1000.0f;  /* clamp is deliberately not the limiting factor */
        p.i_min = -1000.0f;
        for (int i = 0; i < 500; i++) {
            (void)blimp_pid_step(&s, &p, 5.0f, 0.01f);
        }
        /* With conditional integration the integral must stay small; without
         * it this would run away to ~250. */
        CHECK_MSG(s.integral < 10.0f,
                  "integral wound up to %.3f while saturated",
                  (double)s.integral);

        /* The integral must be able to UNWIND once the error reverses - a
         * guard that freezes it permanently is as broken as one that lets it
         * run away. */
        const float wound = s.integral;
        for (int i = 0; i < 20; i++) {
            (void)blimp_pid_step(&s, &p, -1.0f, 0.01f);
        }
        CHECK_MSG(s.integral < wound,
                  "integral frozen at %.4f, did not unwind", (double)wound);

        /* And with the error genuinely reversed the output must follow. */
        const float out = blimp_pid_step(&s, &p, -1.0f, 0.01f);
        CHECK_MSG(out < 0.0f, "did not recover from saturation, out=%.4f",
                  (double)out);
    }

    /* ---- ki == 0 never carries an integral ---------------------------- */
    TEST_GROUP("ki=0 carries nothing");
    {
        blimp_pid_t s;
        blimp_pid_reset(&s);
        blimp_pid_params_t p = basic_params();
        p.ki = 0.0f;
        for (int i = 0; i < 100; i++) {
            (void)blimp_pid_step(&s, &p, 3.0f, 0.01f);
        }
        CHECK_NEAR(s.integral, 0.0f, 1e-9);
    }

    /* ---- degenerate inputs -------------------------------------------- */
    TEST_GROUP("degenerate inputs");
    {
        blimp_pid_t s;
        blimp_pid_reset(&s);
        blimp_pid_params_t p = basic_params();
        CHECK_NEAR(blimp_pid_step(&s, &p, 1.0f, 0.0f), 0.0f, 1e-9);
        CHECK_NEAR(blimp_pid_step(&s, &p, 1.0f, -0.01f), 0.0f, 1e-9);
        CHECK_NEAR(blimp_pid_step(NULL, &p, 1.0f, 0.01f), 0.0f, 1e-9);
        CHECK_NEAR(blimp_pid_step(&s, NULL, 1.0f, 0.01f), 0.0f, 1e-9);
        const float nan_v = (float)NAN;
        CHECK_NEAR(blimp_pid_step(&s, &p, nan_v, 0.01f), 0.0f, 1e-9);
        /* state must survive all of the above intact */
        CHECK_FINITE(s.integral);
        CHECK_FINITE(s.d_state);
    }

    /* ---- derivative filter is stable ---------------------------------- */
    TEST_GROUP("derivative filter stability");
    {
        blimp_pid_t s;
        blimp_pid_reset(&s);
        blimp_pid_params_t p = basic_params();
        p.kp = 0.0f;
        p.kd = 1.0f;
        p.d_lpf_hz = 5.0f;
        /* alternating noise must not blow up the filter state */
        for (int i = 0; i < 2000; i++) {
            const float d = (i % 2) ? 50.0f : -50.0f;
            const float o = blimp_pid_step_d(&s, &p, 0.0f, d, 0.005f);
            CHECK_FINITE(o);
        }
        CHECK_FINITE(s.d_state);
        CHECK(fabsf(s.d_state) <= 50.0f + 1e-3f);
    }

    TEST_REPORT();
}
