/* test_param_table.c - the 'param' command's table, on the computer.
 *
 * main/param_table.c has no ESP-IDF dependency, so it is compiled here
 * against core/src/params.c with tools/build.py's flags and checked for what
 * the console relies on: every field of blimp_params_t is reachable by name,
 * every default is accepted, a refused value changes nothing, and whatever a
 * set leaves behind already passes blimp_params_validate() (Rule 13, Rule 11).
 * Run by tools/fw_host_test.py, which tools/verify.py calls.
 */
#include <string.h>

#include "blimp/params.h"
#include "param_table.h"
#include "test_util.h"

static size_t entry_size(const param_entry_t *e)
{
    return e->type == PARAM_BOOL ? sizeof(bool) : sizeof(float);
}

static bool same(const blimp_params_t *a, const blimp_params_t *b)
{
    for (size_t i = 0; i < param_count(); i++) {
        const param_entry_t *e = param_at(i);
        if (param_get(a, e) != param_get(b, e)) {
            return false;
        }
    }
    return true;
}

int main(void)
{
    printf("test_param_table\n");

    TEST_GROUP("the table covers blimp_params_t, once per field");
    {
        unsigned char owner[sizeof(blimp_params_t)];
        memset(owner, 0, sizeof(owner));
        bool overlap = false;
        for (size_t i = 0; i < param_count(); i++) {
            const param_entry_t *e = param_at(i);
            CHECK_MSG(e->offset + entry_size(e) <= sizeof(blimp_params_t), "%s past the end", e->name);
            for (size_t b = e->offset; b < e->offset + entry_size(e) && b < sizeof(owner); b++) {
                overlap |= owner[b] != 0;
                owner[b] = 1;
            }
            CHECK_MSG(param_find(e->name) == e, "%s not found by name, or named twice", e->name);
            CHECK_MSG(e->min < e->max, "%s: empty range", e->name);
        }
        CHECK(!overlap);
        /* A field with no entry leaves a hole of at least one float; the only
           holes allowed are the padding after a bool. */
        size_t run = 0, longest = 0;
        for (size_t b = 0; b < sizeof(owner); b++) {
            run = owner[b] ? 0 : run + 1;
            longest = run > longest ? run : longest;
        }
        CHECK_MSG(longest < sizeof(float),
                  "%u bytes of blimp_params_t have no entry: a field was added to params.h "
                  "without one in param_table.c", (unsigned)longest);
        CHECK(param_find("alt.kq") == NULL);
        CHECK(param_find("alt") == NULL);
        CHECK(param_find(NULL) == NULL);
        CHECK(param_at(param_count()) == NULL);
    }

    TEST_GROUP("every default sits inside its range and sets back cleanly");
    {
        blimp_params_t d;
        blimp_params_defaults(&d);
        for (size_t i = 0; i < param_count(); i++) {
            const param_entry_t *e = param_at(i);
            const float v = param_get(&d, e);
            CHECK_MSG(v >= e->min && v <= e->max, "%s default %g outside [%g, %g]",
                      e->name, (double)v, (double)e->min, (double)e->max);
            blimp_params_t p = d;
            CHECK_MSG(param_set(&p, e, v) == PARAM_OK, "%s: its own default was not accepted", e->name);
            CHECK(same(&p, &d));
        }
    }

    TEST_GROUP("parsing");
    {
        const param_entry_t *kp = param_find("alt.kp");
        const param_entry_t *mag = param_find("use_mag");
        float v = -1.0f;
        CHECK(param_parse(kp, "0.4", &v) == PARAM_OK);
        CHECK_NEAR(v, 0.4, 1e-7);
        CHECK(param_parse(kp, "-0.06", &v) == PARAM_OK);
        CHECK_NEAR(v, -0.06, 1e-7);
        CHECK(param_parse(kp, "1e-3", &v) == PARAM_OK);
        CHECK_NEAR(v, 1e-3, 1e-9);
        CHECK(param_parse(kp, "abc", &v) == PARAM_ERR_NUMBER);
        CHECK(param_parse(kp, "0.4x", &v) == PARAM_ERR_NUMBER);
        CHECK(param_parse(kp, "0,4", &v) == PARAM_ERR_NUMBER);
        CHECK(param_parse(kp, "", &v) == PARAM_ERR_NUMBER);
        CHECK(param_parse(kp, NULL, &v) == PARAM_ERR_NUMBER);
        CHECK(param_parse(mag, "on", &v) == PARAM_OK && v == 1.0f);
        CHECK(param_parse(mag, "false", &v) == PARAM_OK && v == 0.0f);
        CHECK(param_parse(mag, "1", &v) == PARAM_OK && v == 1.0f);
        CHECK(param_parse(mag, "2", &v) == PARAM_ERR_NUMBER);
        CHECK(param_parse(mag, "yes", &v) == PARAM_ERR_NUMBER);
    }

    TEST_GROUP("a set stores the value; a refused one changes nothing");
    {
        blimp_params_t d, p;
        blimp_params_defaults(&d);
        p = d;
        const param_entry_t *kp = param_find("alt.kp");
        CHECK(param_set(&p, kp, 0.4f) == PARAM_OK);
        CHECK_NEAR(p.alt.kp, 0.4, 1e-7);
        CHECK_NEAR(p.alt.kd, d.alt.kd, 0.0);           /* nothing else moved */

        blimp_params_t before = p;
        CHECK(param_set(&p, kp, 40.0f) == PARAM_ERR_RANGE);     /* a typo for 0.40 */
        CHECK(param_set(&p, kp, -0.4f) == PARAM_ERR_RANGE);     /* positive feedback */
        CHECK(param_set(&p, kp, (float)NAN) == PARAM_ERR_RANGE);
        CHECK(param_set(&p, kp, (float)INFINITY) == PARAM_ERR_RANGE);
        CHECK(param_set(&p, param_find("max_total_thrust_n"), BLIMP_RULE13_MAX_TOTAL_N * 1.01f)
              == PARAM_ERR_RANGE);                              /* Rule 13 */
        CHECK(memcmp(&p, &before, sizeof(p)) == 0);

        CHECK(param_set(&p, param_find("use_mag"), 1.0f) == PARAM_OK);
        CHECK(p.use_mag);
        CHECK(param_get(&p, param_find("use_mag")) == 1.0f);
    }

    TEST_GROUP("PID clamps keep their order");
    {
        blimp_params_t p;
        blimp_params_defaults(&p);
        blimp_params_t before = p;
        CHECK(param_set(&p, param_find("alt.i_min"), 0.2f) == PARAM_ERR_ORDER);   /* i_max is 0.15 */
        CHECK(param_set(&p, param_find("yaw.out_max"), -0.1f) == PARAM_ERR_ORDER);
        CHECK(memcmp(&p, &before, sizeof(p)) == 0);
        /* Moving the pair: widen first, then the other end. */
        CHECK(param_set(&p, param_find("alt.i_max"), 0.3f) == PARAM_OK);
        CHECK(param_set(&p, param_find("alt.i_min"), 0.2f) == PARAM_OK);
    }

    TEST_GROUP("the validator's clamps reach related fields");
    {
        blimp_params_t p;
        blimp_params_defaults(&p);
        /* The heave budget below the hover feed-forward drags the feed-forward down. */
        CHECK(param_set(&p, param_find("heave_budget_n"), 0.03f) == PARAM_CLAMPED);
        CHECK_NEAR(p.heave_budget_n, 0.03, 1e-7);
        CHECK(p.hover_thrust_n <= p.heave_budget_n);

        blimp_params_defaults(&p);
        /* A total below one motor's cap pulls the motor's cap down with it. */
        CHECK(param_set(&p, param_find("max_total_thrust_n"), 0.2f) == PARAM_CLAMPED);
        CHECK(p.motor_max_thrust_n <= p.max_total_thrust_n);
    }

    TEST_GROUP("whatever a set leaves already passes blimp_params_validate()");
    {
        for (size_t i = 0; i < param_count(); i++) {
            const param_entry_t *e = param_at(i);
            const float tries[] = {e->min, e->max, 0.5f * (e->min + e->max)};
            for (size_t k = 0; k < sizeof(tries) / sizeof(tries[0]); k++) {
                blimp_params_t p;
                blimp_params_defaults(&p);
                const param_result_t r = param_set(&p, e, tries[k]);
                if (r != PARAM_OK && r != PARAM_CLAMPED) {
                    continue;               /* refused: p untouched, checked above */
                }
                blimp_params_t again = p;
                CHECK_MSG(blimp_params_validate(&again), "%s = %g left params the validator "
                          "still changes", e->name, (double)tries[k]);
                CHECK(p.max_total_thrust_n <= BLIMP_RULE13_MAX_TOTAL_N);
                CHECK(p.hover_thrust_n <= BLIMP_RULE11_TRIM_TOL_N);
            }
        }
    }

    TEST_REPORT();
}
