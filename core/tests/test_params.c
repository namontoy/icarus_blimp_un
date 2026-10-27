/* test_params.c - defaults sanity and rulebook-compliance validation.
 *
 * The point of these tests is not to pin down gain values (they will change
 * during bring-up) but to guarantee that no code path - including a malicious
 * or corrupt live-tuning packet - can produce params that break a rule.
 */
#include "blimp/params.h"
#include "test_util.h"

int main(void)
{
    printf("test_params\n");

    TEST_GROUP("defaults are sane and rule-legal");
    {
        blimp_params_t p;
        blimp_params_defaults(&p);

        CHECK(blimp_params_validate(&p));   /* defaults need no correction */

        CHECK(p.max_total_thrust_n <= BLIMP_RULE13_MAX_TOTAL_N + 1e-9f);
        CHECK(p.max_total_thrust_n > 0.0f);
        CHECK(p.motor_max_thrust_n > 0.0f);
        CHECK(p.motor_max_thrust_n <= p.max_total_thrust_n);
        CHECK(p.heave_budget_n > 0.0f);
        CHECK(p.heave_budget_n < p.max_total_thrust_n);
        CHECK(p.hover_thrust_n >= 0.0f);
        CHECK(p.hover_thrust_n <= BLIMP_RULE11_TRIM_TOL_N);
        CHECK(p.front_half_span_m > 0.0f);
        CHECK(!p.use_mag);   /* MPU-6050 only on this PCB */

        /* Integral gains start at zero - docs/03 section 3.3. If someone
         * changes this, it should be a deliberate, reviewed decision. */
        CHECK_NEAR(p.alt.ki, 0.0f, 1e-12);
        CHECK_NEAR(p.yaw.ki, 0.0f, 1e-12);
        CHECK_NEAR(p.surge.ki, 0.0f, 1e-12);

        /* Every gain finite and non-negative. */
        CHECK_FINITE(p.alt.kp);  CHECK(p.alt.kp >= 0.0f);
        CHECK_FINITE(p.alt.kd);  CHECK(p.alt.kd >= 0.0f);
        CHECK_FINITE(p.yaw.kp);  CHECK(p.yaw.kp >= 0.0f);
        CHECK_FINITE(p.yaw.kd);  CHECK(p.yaw.kd >= 0.0f);

        /* Output clamps must be ordered and within the thrust budget. */
        CHECK(p.alt.out_min < p.alt.out_max);
        CHECK(p.yaw.out_min < p.yaw.out_max);
        CHECK(p.surge.out_min < p.surge.out_max);

        /* Rule 22: the waypoint height the guidance layer targets. */
        CHECK_NEAR(BLIMP_RULE22_WP_HEIGHT_M, 2.0f, 1e-9);

        /* Touch radius must be big enough to actually reach a balloon:
         * our envelope radius plus the balloon radius is roughly 0.66 m. */
        CHECK(p.wp_touch_radius_m > 0.2f && p.wp_touch_radius_m < 1.2f);
    }

    TEST_GROUP("gf conversion");
    {
        CHECK_NEAR(BLIMP_GF_TO_N(100.0f), 0.980665f, 1e-6);
        CHECK_NEAR(BLIMP_GF_TO_N(10.0f), 0.0980665f, 1e-7);
        CHECK_NEAR(BLIMP_RULE11_TRIM_TOL_N, 0.0980665f, 1e-7);
    }

    TEST_GROUP("validate rejects over-budget thrust (Rule 13)");
    {
        blimp_params_t p;
        blimp_params_defaults(&p);
        p.max_total_thrust_n = BLIMP_GF_TO_N(250.0f);   /* cheating */
        CHECK(!blimp_params_validate(&p));
        CHECK_MSG(p.max_total_thrust_n <= BLIMP_RULE13_MAX_TOTAL_N + 1e-9f,
                  "Rule 13 breach survived validation: %.6f N",
                  (double)p.max_total_thrust_n);
    }

    TEST_GROUP("validate repairs degenerate values");
    {
        blimp_params_t p;
        blimp_params_defaults(&p);
        p.front_half_span_m = 0.0f;
        p.motor_max_thrust_n = (float)NAN;
        p.heave_budget_n = -5.0f;
        p.hover_thrust_n = -1.0f;
        p.cam_dz_m = (float)NAN;
        p.wp_touch_radius_m = -1.0f;
        p.link_timeout_s = 0.0f;

        CHECK(!blimp_params_validate(&p));
        CHECK(p.front_half_span_m > 0.0f);
        CHECK(p.motor_max_thrust_n > 0.0f);
        CHECK(p.heave_budget_n >= 0.0f);
        CHECK(p.hover_thrust_n >= 0.0f);
        CHECK(isfinite(p.cam_dz_m));
        CHECK(p.wp_touch_radius_m > 0.0f);
        CHECK(p.link_timeout_s > 0.0f);

        /* Repaired params must now pass cleanly - validate is idempotent. */
        CHECK(blimp_params_validate(&p));
    }

    TEST_GROUP("heave budget cannot starve the front pair");
    {
        blimp_params_t p;
        blimp_params_defaults(&p);
        p.heave_budget_n = p.max_total_thrust_n;   /* all of it */
        CHECK(!blimp_params_validate(&p));
        CHECK(p.heave_budget_n <= p.max_total_thrust_n * 0.6f + 1e-9f);
    }

    TEST_GROUP("hover feed-forward is bounded by the Rule 11 trim tolerance");
    {
        blimp_params_t p;
        blimp_params_defaults(&p);
        p.hover_thrust_n = 1.0f;   /* ~100 gf heavy: illegal trim */
        CHECK(!blimp_params_validate(&p));
        CHECK(p.hover_thrust_n <= BLIMP_RULE11_TRIM_TOL_N + 1e-9f);
        CHECK(p.hover_thrust_n <= p.heave_budget_n + 1e-9f);
    }

    TEST_GROUP("one motor can never be allowed more than the whole budget");
    {
        blimp_params_t p;
        blimp_params_defaults(&p);
        p.motor_max_thrust_n = 5.0f;
        CHECK(!blimp_params_validate(&p));
        CHECK(p.motor_max_thrust_n <= p.max_total_thrust_n + 1e-9f);
    }

    TEST_GROUP("null safety");
    {
        blimp_params_defaults(NULL);          /* must not crash */
        CHECK(!blimp_params_validate(NULL));
    }

    TEST_REPORT();
}
