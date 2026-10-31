/* test_motor_ramp.c - the motor driver's soft start (main/motor_ramp.c), on
 * the computer. The team's setting of 2026-10-31 throughout unless a group
 * says otherwise: deadband 10 %, ceiling 40 %, 200 ms from rest to a full
 * command, so a rise of 1.5 duty per second. Run by tools/fw_host_test.py,
 * which tools/verify.py calls.
 */
#include <math.h>
#include <stdio.h>

#include "motor_ramp.h"
#include "test_util.h"

#define MS(x) ((int64_t)(x) * 1000)
#define TOL   1e-5f

int main(void)
{
    printf("test_motor_ramp\n");
    const motor_ramp_cfg_t team = { 0.200f, 0.10f, 0.40f };
    motor_ramp_t r;

    TEST_GROUP("a start from rest begins at the deadband and reaches a full command after the ramp time");
    {
        motor_ramp_init(&r, MS(0));
        CHECK_NEAR(motor_ramp_set(&r, 0.40f, MS(0), &team), 0.10f, TOL);
        CHECK(motor_ramp_busy(&r));
        CHECK_NEAR(motor_ramp_step(&r, MS(100), &team), 0.25f, TOL);
        CHECK_NEAR(motor_ramp_step(&r, MS(200), &team), 0.40f, TOL);
        CHECK_NEAR(motor_ramp_step(&r, MS(300), &team), 0.40f, TOL);
        CHECK(!motor_ramp_busy(&r));
    }

    TEST_GROUP("a smaller command stops climbing at itself");
    {
        motor_ramp_init(&r, MS(0));
        CHECK_NEAR(motor_ramp_set(&r, 0.20f, MS(0), &team), 0.10f, TOL);
        CHECK_NEAR(motor_ramp_step(&r, MS(50), &team), 0.175f, TOL);
        CHECK_NEAR(motor_ramp_step(&r, MS(100), &team), 0.20f, TOL);
        CHECK(!motor_ramp_busy(&r));
    }

    TEST_GROUP("a decrease and a stop are immediate");
    {
        motor_ramp_init(&r, MS(0));
        motor_ramp_set(&r, 0.40f, MS(0), &team);
        motor_ramp_step(&r, MS(250), &team);
        CHECK_NEAR(motor_ramp_set(&r, 0.25f, MS(260), &team), 0.25f, TOL);
        CHECK_NEAR(motor_ramp_set(&r, 0.0f, MS(270), &team), 0.0f, TOL);
        CHECK(!r.driving && !motor_ramp_busy(&r));
    }

    TEST_GROUP("a rise on a running motor is limited too, a small one barely waits");
    {
        motor_ramp_init(&r, MS(0));
        motor_ramp_set(&r, 0.20f, MS(0), &team);
        motor_ramp_step(&r, MS(100), &team);                      /* at 0.20 */
        CHECK_NEAR(motor_ramp_set(&r, 0.21f, MS(120), &team), 0.21f, TOL);   /* 20 ms allows 0.03 */
        CHECK_NEAR(motor_ramp_set(&r, 0.40f, MS(140), &team), 0.24f, TOL);   /* 20 ms more: +0.03 */
        CHECK_NEAR(motor_ramp_step(&r, MS(240), &team), 0.39f, TOL);
    }

    TEST_GROUP("restarting the way it was turning starts at once, from the deadband");
    {
        motor_ramp_init(&r, MS(0));
        motor_ramp_set(&r, 0.40f, MS(0), &team);
        motor_ramp_step(&r, MS(300), &team);
        motor_ramp_set(&r, 0.0f, MS(1000), &team);
        CHECK_NEAR(motor_ramp_set(&r, 0.40f, MS(1010), &team), 0.10f, TOL);
    }

    TEST_GROUP("a reversal while driven coasts for the ramp time, then ramps the other way");
    {
        motor_ramp_init(&r, MS(0));
        motor_ramp_set(&r, 0.40f, MS(0), &team);
        motor_ramp_step(&r, MS(300), &team);
        CHECK_NEAR(motor_ramp_set(&r, -0.30f, MS(1000), &team), 0.0f, TOL);
        CHECK(motor_ramp_busy(&r));                               /* waiting to start */
        CHECK_NEAR(motor_ramp_step(&r, MS(1100), &team), 0.0f, TOL);
        CHECK_NEAR(motor_ramp_step(&r, MS(1199), &team), 0.0f, TOL);
        CHECK_NEAR(motor_ramp_step(&r, MS(1200), &team), -0.10f, TOL);
        CHECK_NEAR(motor_ramp_step(&r, MS(1300), &team), -0.25f, TOL);
        CHECK_NEAR(motor_ramp_step(&r, MS(1400), &team), -0.30f, TOL);
        CHECK(!motor_ramp_busy(&r));
    }

    TEST_GROUP("a stop and then the other way also waits, counted from the stop");
    {
        motor_ramp_init(&r, MS(0));
        motor_ramp_set(&r, 0.40f, MS(0), &team);
        motor_ramp_step(&r, MS(300), &team);
        motor_ramp_set(&r, 0.0f, MS(2000), &team);
        CHECK_NEAR(motor_ramp_set(&r, -0.40f, MS(2050), &team), 0.0f, TOL);
        CHECK_NEAR(motor_ramp_step(&r, MS(2190), &team), 0.0f, TOL);
        CHECK_NEAR(motor_ramp_step(&r, MS(2200), &team), -0.10f, TOL);
    }

    TEST_GROUP("a coast written elsewhere (stop, brake, failsafe) counts as a stop");
    {
        motor_ramp_init(&r, MS(0));
        motor_ramp_set(&r, 0.40f, MS(0), &team);
        motor_ramp_step(&r, MS(300), &team);
        motor_ramp_stop(&r, MS(500));
        CHECK(!r.driving && !motor_ramp_busy(&r) && r.applied == 0.0f);
        CHECK_NEAR(motor_ramp_set(&r, -0.40f, MS(600), &team), 0.0f, TOL);   /* reversal waits */
        CHECK_NEAR(motor_ramp_step(&r, MS(700), &team), -0.10f, TOL);
    }

    TEST_GROUP("never driven: either way starts at once");
    {
        motor_ramp_init(&r, MS(0));
        CHECK_NEAR(motor_ramp_set(&r, -0.40f, MS(10), &team), -0.10f, TOL);
    }

    TEST_GROUP("deadband 0 and no ceiling (the Kconfig defaults): a start climbs from 0");
    {
        const motor_ramp_cfg_t plain = { 0.200f, 0.0f, 1.0f };
        motor_ramp_init(&r, MS(0));
        CHECK_NEAR(motor_ramp_set(&r, 0.50f, MS(0), &plain), 0.0f, TOL);
        CHECK(r.driving && motor_ramp_busy(&r));
        CHECK_NEAR(motor_ramp_step(&r, MS(50), &plain), 0.25f, TOL);
        CHECK_NEAR(motor_ramp_step(&r, MS(100), &plain), 0.50f, TOL);
    }

    TEST_GROUP("ramp off: a command lands as given, reversals included");
    {
        const motor_ramp_cfg_t off = { 0.0f, 0.10f, 0.40f };
        motor_ramp_init(&r, MS(0));
        CHECK_NEAR(motor_ramp_set(&r, 0.40f, MS(0), &off), 0.40f, TOL);
        CHECK_NEAR(motor_ramp_set(&r, -0.40f, MS(10), &off), -0.40f, TOL);
        CHECK(!motor_ramp_busy(&r));
    }

    TEST_GROUP("odd inputs: NaN is a stop, a clock that goes back moves nothing, a long gap lands");
    {
        motor_ramp_init(&r, MS(0));
        motor_ramp_set(&r, 0.40f, MS(0), &team);
        CHECK_NEAR(motor_ramp_set(&r, NAN, MS(10), &team), 0.0f, TOL);
        CHECK(!r.driving);
        motor_ramp_init(&r, MS(1000));
        motor_ramp_set(&r, 0.40f, MS(1000), &team);
        CHECK_NEAR(motor_ramp_step(&r, MS(900), &team), 0.10f, TOL);       /* back in time */
        CHECK_NEAR(motor_ramp_step(&r, MS(60000), &team), 0.40f, TOL);     /* a minute later */
    }

    TEST_REPORT();
}
