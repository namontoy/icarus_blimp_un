/* test_mixer.c - control allocation for four motors (the front pair
 * reversible since core 0.4.0), and above all the Rule 13 invariant.
 *
 * The mixer is the last code between the controller and the motors. If any
 * reachable input can drive the total commanded thrust above 100 gf, we are
 * disqualified. So the headline test here is not a handful of examples: it is
 * a sweep over tens of thousands of demands, including demands far outside
 * anything physically achievable, asserting the budget holds on every one -
 * one-way, and again with the front pair reversible, where a motor pushing
 * backwards counts against the budget by its magnitude.
 */
#include "blimp/mixer.h"
#include "test_util.h"

static blimp_params_t P;

/* Total thrust in the Rule 13 sense: every motor's magnitude, reverse
 * included. One-way, every thrust is >= 0 and this is the plain sum. */
static float total_commanded(const blimp_actuators_t *a)
{
    return fabsf(a->thrust_fl) + fabsf(a->thrust_fr) +
           fabsf(a->thrust_rl) + fabsf(a->thrust_rr);
}

/* The lowest a front motor may go for these params (mixer.h): minus the
 * reverse ratio, sanitised into [0, 1], times the per-motor cap. -0.0f
 * one-way, so the check below is the exact ">= 0" it always was. */
static float front_floor(const blimp_params_t *p)
{
    float r = (isfinite(p->front_reverse_ratio) && p->front_reverse_ratio > 0.0f)
                  ? p->front_reverse_ratio : 0.0f;
    if (r > 1.0f) {
        r = 1.0f;
    }
    return -(r * p->motor_max_thrust_n);
}

/* Every invariant blimp_mixer_apply() promises in mixer.h, checked together.
 * Returns the number of violations so callers can report the offending
 * input rather than just a count. */
static int check_invariants(const blimp_params_t *p,
                            const blimp_actuators_t *a)
{
    int bad = 0;
    const float thr[4] = {a->thrust_fl, a->thrust_fr, a->thrust_rl, a->thrust_rr};
    for (int i = 0; i < 4; i++) {
        if (!isfinite(thr[i])) {
            bad++;
            continue;
        }
        /* thr[0..1] the front pair, thr[2..3] the rear: up only. */
        if (thr[i] < ((i < 2) ? front_floor(p) : 0.0f)) bad++;
        if (thr[i] > p->motor_max_thrust_n + 1e-6f) bad++;
    }
    if (!(total_commanded(a) <= p->max_total_thrust_n + 1e-6f)) bad++;
    if (!(a->thrust_rl + a->thrust_rr <= p->heave_budget_n + 1e-6f)) bad++;
    return bad;
}

/* One sweep of the Rule 13 test over params p; returns the violations. */
static int rule13_sweep(const blimp_params_t *p, const char *name, long *samples_out)
{
    /* Magnitudes from trivial to absurd: the achievable envelope is
     * ~1 N, so sweeping to 1e7 covers anything a diverging controller,
     * a corrupt telemetry packet or an integrator blow-up could emit. */
    static const float mags[] = {
        0.0f, 1e-9f, 1e-4f, 0.01f, 0.1f, 0.5f, 0.981f, 1.5f,
        10.0f, 1e3f, 1e5f, 1e7f, -0.0f
    };
    const int nm = (int)(sizeof(mags) / sizeof(mags[0]));
    int violations = 0, worst_reported = 0;
    long samples = 0;

    for (int a = 0; a < nm; a++) {
        for (int b = 0; b < nm; b++) {
            for (int c = 0; c < nm; c++) {
                for (int d = 0; d < nm; d++) {
                    for (int sgn = 0; sgn < 4; sgn++) {
                        blimp_wrench_t w;
                        w.fx = (sgn & 1) ? -mags[a] : mags[a];
                        w.fy = (sgn & 2) ? -mags[b] : mags[b];
                        w.fz = (sgn & 1) ? mags[c] : -mags[c];
                        w.tz = (sgn & 2) ? mags[d] : -mags[d];

                        blimp_actuators_t act;
                        blimp_mixer_apply(p, &w, &act);
                        samples++;

                        const int bad = check_invariants(p, &act);
                        if (bad) {
                            violations++;
                            if (worst_reported < 5) {
                                worst_reported++;
                                printf("    VIOLATION (%s) fx=%g fy=%g fz=%g "
                                       "tz=%g -> fl=%g fr=%g rl=%g rr=%g "
                                       "total=%g (cap %g)\n", name,
                                       (double)w.fx, (double)w.fy,
                                       (double)w.fz, (double)w.tz,
                                       (double)act.thrust_fl,
                                       (double)act.thrust_fr,
                                       (double)act.thrust_rl,
                                       (double)act.thrust_rr,
                                       (double)total_commanded(&act),
                                       (double)p->max_total_thrust_n);
                            }
                        }
                    }
                }
            }
        }
    }
    printf("    swept %ld demands (%s)\n", samples, name);
    *samples_out = samples;
    return violations;
}

int main(void)
{
    printf("test_mixer\n");
    blimp_params_defaults(&P);
    (void)blimp_params_validate(&P);

    /* ================= THE DISQUALIFICATION TEST ===================== */
    TEST_GROUP("Rule 13: swept demands can never exceed the thrust budget");
    {
        /* One-way (the default, front_reverse_ratio 0), as it always was;
         * then reversible at the bench's 0.74 on the default motors; then
         * fully reversible on motors big enough that the 100 gf total, not
         * the per-motor caps, is what binds: 2 x 50 gf front plus a 60 gf
         * heave budget is 160 gf of possible demand against 100. */
        CHECK(P.front_reverse_ratio == 0.0f);
        blimp_params_t rev = P;
        rev.front_reverse_ratio = 0.74f;
        CHECK(blimp_params_validate(&rev));
        blimp_params_t big = P;
        big.motor_max_thrust_n  = BLIMP_GF_TO_N(50.0f);
        big.heave_budget_n      = BLIMP_GF_TO_N(60.0f);
        big.front_reverse_ratio = 1.0f;
        CHECK(blimp_params_validate(&big));

        const struct { const blimp_params_t *p; const char *name; } sets[] = {
            {&P, "one-way"}, {&rev, "reverse 0.74"}, {&big, "reverse 1.0, budget binding"},
        };
        for (unsigned i = 0; i < sizeof(sets) / sizeof(sets[0]); i++) {
            long samples = 0;
            const int violations = rule13_sweep(sets[i].p, sets[i].name, &samples);
            CHECK_MSG(violations == 0, "%d invariant violations across %ld swept demands (%s)",
                      violations, samples, sets[i].name);
            CHECK(samples > 40000);
        }

        /* The binding case must really bind, or it proves nothing: a full
         * yaw, a full brake and a full heave together ask for more than
         * 100 gf, and get exactly 100. */
        blimp_wrench_t all = {-1e3f, 0.0f, 1e3f, 1e3f};
        blimp_actuators_t act;
        blimp_mixer_apply(&big, &all, &act);
        CHECK(act.clamped_total);
        CHECK_NEAR(total_commanded(&act), big.max_total_thrust_n, 1e-5);
    }

    /* ---- non-finite inputs must not escape ---------------------------- */
    TEST_GROUP("NaN and Inf demands are contained");
    {
        const float nan_v = (float)NAN;
        const float inf_v = (float)INFINITY;
        const float weird[] = {nan_v, inf_v, -inf_v, 0.0f, 1.0f, -1.0f};
        blimp_params_t rev = P;
        rev.front_reverse_ratio = 0.74f;
        int violations = 0;
        for (int a = 0; a < 6; a++)
        for (int b = 0; b < 6; b++)
        for (int c = 0; c < 6; c++)
        for (int d = 0; d < 6; d++) {
            blimp_wrench_t w = {weird[a], weird[b], weird[c], weird[d]};
            blimp_actuators_t act;
            blimp_mixer_apply(&P, &w, &act);
            violations += check_invariants(&P, &act);
            blimp_mixer_apply(&rev, &w, &act);
            violations += check_invariants(&rev, &act);
        }
        CHECK_MSG(violations == 0, "%d violations from non-finite demands",
                  violations);
    }

    /* ---- corrupt params must not open a hole -------------------------- */
    TEST_GROUP("adversarial params cannot breach the budget");
    {
        blimp_params_t bad;
        int violations = 0;
        const float nan_v = (float)NAN;

        /* An unvalidated params struct claiming a 10x budget. The mixer
         * sanitises internally, so it must still honour the cap it was
         * GIVEN; enforcing the 100 gf ceiling is params_validate's job, and
         * blimp.c must call it. */
        blimp_params_defaults(&bad);
        bad.max_total_thrust_n = BLIMP_GF_TO_N(1000.0f);
        bad.motor_max_thrust_n = BLIMP_GF_TO_N(1000.0f);
        bad.heave_budget_n     = BLIMP_GF_TO_N(1000.0f);
        blimp_wrench_t w = {5.0f, 5.0f, 5.0f, 5.0f};
        blimp_actuators_t act;
        blimp_mixer_apply(&bad, &w, &act);
        CHECK(total_commanded(&act) <= bad.max_total_thrust_n + 1e-6f);
        CHECK_MSG(!blimp_params_validate(&bad),
                  "params_validate failed to reject a 1000 gf budget");
        CHECK(bad.max_total_thrust_n <= BLIMP_RULE13_MAX_TOTAL_N + 1e-9f);
        CHECK(bad.motor_max_thrust_n <= BLIMP_RULE13_MAX_TOTAL_N + 1e-9f);
        CHECK(bad.heave_budget_n <= BLIMP_RULE13_MAX_TOTAL_N + 1e-9f);

        /* NaN / zero params: the mixer must fall back to sane, rule-legal
         * values rather than emit NaN or exceed the Rule 13 cap. */
        blimp_params_defaults(&bad);
        bad.front_half_span_m  = nan_v;
        bad.motor_max_thrust_n = nan_v;
        bad.heave_budget_n     = nan_v;
        bad.max_total_thrust_n = nan_v;
        for (int i = 0; i < 200; i++) {
            blimp_wrench_t ww = {(float)i - 100.0f, (float)i * 0.5f,
                                 (float)(100 - i), (float)i * 0.01f};
            blimp_actuators_t aa;
            blimp_mixer_apply(&bad, &ww, &aa);
            if (!isfinite(aa.thrust_fl) || !isfinite(aa.thrust_fr) ||
                !isfinite(aa.thrust_rl) || !isfinite(aa.thrust_rr) ||
                aa.thrust_fl < 0.0f || aa.thrust_fr < 0.0f ||
                aa.thrust_rl < 0.0f || aa.thrust_rr < 0.0f ||
                total_commanded(&aa) > BLIMP_RULE13_MAX_TOTAL_N + 1e-6f) {
                violations++;
            }
        }
        CHECK_MSG(violations == 0,
                  "%d violations with NaN params", violations);

        /* A corrupt ratio, straight to the mixer without the validator:
         * non-finite or negative is one-way, and anything above 1 is 1, so
         * no motor is ever asked for more reverse than forward thrust. */
        const float ratios[] = {nan_v, (float)INFINITY, -(float)INFINITY, -5.0f, 1000.0f};
        violations = 0;
        for (unsigned k = 0; k < sizeof(ratios) / sizeof(ratios[0]); k++) {
            blimp_params_defaults(&bad);
            bad.front_reverse_ratio = ratios[k];
            for (int i = 0; i < 200; i++) {
                blimp_wrench_t ww = {(float)i - 100.0f, 0.0f,
                                     (float)(100 - i) * 0.01f, (float)(i - 100) * 0.01f};
                blimp_actuators_t aa;
                blimp_mixer_apply(&bad, &ww, &aa);
                const float lo = (isfinite(ratios[k]) && ratios[k] > 1.0f)
                                     ? -bad.motor_max_thrust_n : 0.0f;
                if (aa.thrust_fl < lo || aa.thrust_fr < lo ||
                    aa.thrust_rl < 0.0f || aa.thrust_rr < 0.0f ||
                    total_commanded(&aa) > BLIMP_RULE13_MAX_TOTAL_N + 1e-6f) {
                    violations++;
                }
            }
        }
        CHECK_MSG(violations == 0, "%d violations with a corrupt reverse ratio", violations);
    }

    /* ---- NULL safety --------------------------------------------------- */
    TEST_GROUP("null safety");
    {
        blimp_actuators_t act;
        blimp_wrench_t w = {0.1f, 0.0f, 0.1f, 0.0f};
        blimp_mixer_apply(NULL, &w, &act);          /* null params */
        CHECK(isfinite(act.thrust_fl) && act.thrust_fl >= 0.0f);
        blimp_mixer_apply(&P, NULL, &act);          /* null demand -> safe */
        CHECK_NEAR(total_commanded(&act), 0.0f, 1e-9);
        blimp_mixer_apply(&P, &w, NULL);            /* must not crash */
        blimp_wrench_t r;
        blimp_mixer_forward(&P, NULL, &r);
        CHECK_NEAR(r.fx, 0.0f, 1e-9);
        blimp_mixer_forward(&P, &act, NULL);
    }

    /* ---- forward is a true inverse inside the envelope ---------------- */
    TEST_GROUP("apply then forward round-trips when unsaturated");
    {
        /* Stay inside the budget and above the yaw floor (fx >= |tz|/l_f)
         * so nothing clamps. */
        const float fxs[] = {0.10f, 0.20f, 0.30f};
        const float fzs[] = {0.0f, 0.05f, 0.20f};
        const float tzs[] = {-0.008f, -0.002f, 0.0f, 0.002f, 0.008f};
        int checked = 0;
        for (unsigned i = 0; i < sizeof(fxs)/sizeof(fxs[0]); i++)
        for (unsigned j = 0; j < sizeof(fzs)/sizeof(fzs[0]); j++)
        for (unsigned k = 0; k < sizeof(tzs)/sizeof(tzs[0]); k++) {
            blimp_wrench_t w = {fxs[i], 0.0f, fzs[j], tzs[k]};
            blimp_actuators_t act;
            blimp_mixer_apply(&P, &w, &act);
            CHECK_MSG(!act.clamped_total && !act.clamped_yaw && !act.clamped_heave,
                      "unexpected clamp for fx=%g fz=%g tz=%g",
                      (double)w.fx, (double)w.fz, (double)w.tz);
            blimp_wrench_t r;
            blimp_mixer_forward(&P, &act, &r);
            CHECK_NEAR(r.fx, w.fx, 2e-3);
            CHECK_NEAR(r.fz, w.fz, 2e-3);
            CHECK_NEAR(r.tz, w.tz, 2e-4);
            checked++;
        }
        CHECK_MSG(checked > 20, "only %d unsaturated round-trips exercised",
                  checked);
    }

    /* ---- sign conventions --------------------------------------------- */
    TEST_GROUP("right motor harder turns the nose left (+tz)");
    {
        blimp_wrench_t w = {0.2f, 0.0f, 0.0f, 0.005f};
        blimp_actuators_t act;
        blimp_mixer_apply(&P, &w, &act);
        CHECK_MSG(act.thrust_fr > act.thrust_fl,
                  "+tz should load the RIGHT motor: fl=%g fr=%g",
                  (double)act.thrust_fl, (double)act.thrust_fr);
        CHECK_NEAR(act.thrust_fr - act.thrust_fl, 0.005f / P.front_half_span_m, 1e-5);
        CHECK_NEAR(act.thrust_rl, act.thrust_rr, 1e-9);   /* rear pair equal */
        CHECK_NEAR(act.thrust_rl + act.thrust_rr, 0.0f, 1e-9);
    }

    /* ---- yaw is realised exactly even with zero surge (priority 3) --- */
    TEST_GROUP("yaw floor: a pure yaw demand becomes a tight arc, not nothing");
    {
        blimp_wrench_t w = {0.0f, 0.0f, 0.0f, 0.02f};
        blimp_actuators_t act;
        blimp_mixer_apply(&P, &w, &act);
        blimp_wrench_t r;
        blimp_mixer_forward(&P, &act, &r);
        CHECK_NEAR(r.tz, 0.02f, 1e-5);                 /* delivered exactly */
        CHECK_NEAR(act.thrust_fl, 0.0f, 1e-9);         /* one motor off ... */
        CHECK(act.thrust_fr > 0.0f);                   /* ... one motor on  */
        CHECK(!act.clamped_yaw);
        CHECK(!act.clamped_total);                     /* nothing was asked for and refused */
        CHECK_MSG(r.fx > 0.0f, "no surge accompanied the yaw floor");
    }

    /* ---- the front deadband is a parameter (2026-10-30) ---------------- */
    TEST_GROUP("front_deadband_n: the default silences under 0.5 gf, a small one does not");
    {
        /* The rope rig's case: 1 gf motors (30 % duty), l_f 0.12 m, and a
           heading loop asking for 0.26 gf of difference. */
        blimp_params_t q;
        blimp_params_defaults(&q);
        q.motor_max_thrust_n = BLIMP_GF_TO_N(1.0f);
        q.front_half_span_m  = 0.12f;
        CHECK(blimp_params_validate(&q));               /* 0.5 gf is half of 1 gf: allowed */
        const blimp_wrench_t w = {0.0f, 0.0f, 0.0f, 0.12f * BLIMP_GF_TO_N(0.26f)};
        blimp_actuators_t act;
        blimp_mixer_apply(&q, &w, &act);
        CHECK_NEAR(act.thrust_fl + act.thrust_fr, 0.0f, 1e-9);     /* silenced */

        q.front_deadband_n = BLIMP_GF_TO_N(0.05f);
        CHECK(blimp_params_validate(&q));
        blimp_mixer_apply(&q, &w, &act);
        CHECK_NEAR(act.thrust_fr - act.thrust_fl, BLIMP_GF_TO_N(0.26f), 1e-6);
        CHECK(check_invariants(&q, &act) == 0);

        /* A surge under the deadband is zero too, and above it passes. */
        const blimp_wrench_t s_small = {BLIMP_GF_TO_N(0.04f), 0.0f, 0.0f, 0.0f};
        const blimp_wrench_t s_big   = {BLIMP_GF_TO_N(0.40f), 0.0f, 0.0f, 0.0f};
        blimp_mixer_apply(&q, &s_small, &act);
        CHECK_NEAR(act.thrust_fl + act.thrust_fr, 0.0f, 1e-9);
        blimp_mixer_apply(&q, &s_big, &act);
        CHECK_NEAR(act.thrust_fl + act.thrust_fr, BLIMP_GF_TO_N(0.40f), 1e-6);

        /* Above half a motor's worth it is clamped there by the validator
           and by the mixer itself (a corrupted value must not silence the
           front pair); NaN or negative falls back to 0.5 gf. */
        q.front_deadband_n = 1.0f;
        CHECK(!blimp_params_validate(&q));
        CHECK_NEAR(q.front_deadband_n, 0.5f * q.motor_max_thrust_n, 1e-9);
        q.front_deadband_n = NAN;
        CHECK(!blimp_params_validate(&q));
        CHECK_NEAR(q.front_deadband_n, BLIMP_GF_TO_N(0.5f), 1e-9);
        q.front_deadband_n = -1.0f;
        CHECK(!blimp_params_validate(&q));
        CHECK_NEAR(q.front_deadband_n, BLIMP_GF_TO_N(0.5f), 1e-9);

        blimp_params_t raw = q;                          /* unvalidated, straight to the mixer */
        raw.front_deadband_n = 1.0f;
        const blimp_wrench_t w_big = {0.0f, 0.0f, 0.0f, 0.12f * BLIMP_GF_TO_N(0.8f)};
        blimp_mixer_apply(&raw, &w_big, &act);
        CHECK(act.thrust_fr > 0.0f);                     /* 0.8 gf > half of 1 gf: passes */
        CHECK(check_invariants(&raw, &act) == 0);
    }

    /* ---- yaw is protected when surge saturates ------------------------- */
    TEST_GROUP("yaw survives a surge clamp");
    {
        blimp_wrench_t w = {100.0f, 0.0f, 0.0f, 0.02f};
        blimp_actuators_t act;
        blimp_mixer_apply(&P, &w, &act);
        blimp_wrench_t r;
        blimp_mixer_forward(&P, &act, &r);
        CHECK(act.clamped_total);
        CHECK(!act.clamped_yaw);
        CHECK_NEAR(r.tz, 0.02f, 1e-5);
        CHECK(act.thrust_fr <= P.motor_max_thrust_n + 1e-6f);
    }

    /* ---- yaw clamp is honest and bounded by one motor ------------------ */
    TEST_GROUP("yaw beyond one motor's worth is clamped and flagged");
    {
        blimp_wrench_t w = {0.0f, 0.0f, 0.0f, 100.0f};
        blimp_actuators_t act;
        blimp_mixer_apply(&P, &w, &act);
        CHECK(act.clamped_yaw);
        CHECK_NEAR(act.thrust_fr, P.motor_max_thrust_n, 1e-6);
        CHECK_NEAR(act.thrust_fl, 0.0f, 1e-9);
        blimp_wrench_t neg = {0.0f, 0.0f, 0.0f, -100.0f};
        blimp_mixer_apply(&P, &neg, &act);
        CHECK_NEAR(act.thrust_fl, P.motor_max_thrust_n, 1e-6);
        CHECK_NEAR(act.thrust_fr, 0.0f, 1e-9);
    }

    /* ---- heave: up only, budgeted, equal split, first in line ---------- */
    TEST_GROUP("heave is up-only, budgeted, and takes priority over surge");
    {
        blimp_actuators_t act;

        /* Downward demand: nothing can be done, and it must be reported. */
        blimp_wrench_t down = {0.1f, 0.0f, -0.3f, 0.0f};
        blimp_mixer_apply(&P, &down, &act);
        CHECK_NEAR(act.thrust_rl + act.thrust_rr, 0.0f, 1e-9);
        CHECK(act.clamped_heave);
        CHECK(!act.clamped_total);   /* -fz is not a budget problem */

        /* Modest heave: delivered, split equally, no flags. */
        blimp_wrench_t up = {0.1f, 0.0f, 0.2f, 0.0f};
        blimp_mixer_apply(&P, &up, &act);
        CHECK_NEAR(act.thrust_rl + act.thrust_rr, 0.2f, 1e-6);
        CHECK_NEAR(act.thrust_rl, act.thrust_rr, 1e-9);
        CHECK(!act.clamped_heave);

        /* Huge heave: clipped to the heave budget, flagged both ways. */
        blimp_wrench_t big = {0.0f, 0.0f, 100.0f, 0.0f};
        blimp_mixer_apply(&P, &big, &act);
        CHECK_NEAR(act.thrust_rl + act.thrust_rr, P.heave_budget_n, 1e-6);
        CHECK(act.clamped_heave);
        CHECK(act.clamped_total);

        /* Heave at budget AND huge surge: the front pair gets exactly what
         * the rear pair left of the Rule 13 total. */
        blimp_wrench_t both = {100.0f, 0.0f, 100.0f, 0.0f};
        blimp_mixer_apply(&P, &both, &act);
        CHECK_NEAR(act.thrust_rl + act.thrust_rr, P.heave_budget_n, 1e-6);
        const float front = act.thrust_fl + act.thrust_fr;
        const float front_expect = fminf(P.max_total_thrust_n - P.heave_budget_n,
                                         2.0f * P.motor_max_thrust_n);
        CHECK_NEAR(front, front_expect, 1e-6);
        CHECK(total_commanded(&act) <= P.max_total_thrust_n + 1e-6f);
    }

    /* ---- one-way, surge cannot go negative ----------------------------- */
    TEST_GROUP("one-way, a negative surge is silently zero (no reverse thrust)");
    {
        blimp_wrench_t w = {-0.5f, 0.0f, 0.0f, 0.0f};
        blimp_actuators_t act;
        blimp_mixer_apply(&P, &w, &act);
        CHECK_NEAR(total_commanded(&act), 0.0f, 1e-9);
        CHECK(!act.clamped_total);
    }

    /* ==== the front pair reversible (core 0.4.0) ======================== */
    blimp_params_t R = P;
    R.front_reverse_ratio = 0.74f;          /* the lab-supply motor (AGENTS.md) */
    (void)blimp_params_validate(&R);
    const float M     = R.motor_max_thrust_n;
    const float rev_m = 0.74f * M;          /* the most one motor pushes back   */
    const float lf    = R.front_half_span_m;

    TEST_GROUP("reverse: a pure yaw demand is a couple, no net push");
    {
        const float tzs[] = {0.004f, -0.004f, 0.02f, -0.02f};
        for (unsigned i = 0; i < sizeof(tzs) / sizeof(tzs[0]); i++) {
            blimp_wrench_t w = {0.0f, 0.0f, 0.0f, tzs[i]};
            blimp_actuators_t act;
            blimp_mixer_apply(&R, &w, &act);
            blimp_wrench_t r;
            blimp_mixer_forward(&R, &act, &r);
            CHECK_NEAR(act.thrust_fl, -act.thrust_fr, 1e-7);
            CHECK_NEAR(r.fx, 0.0f, 1e-7);
            CHECK_NEAR(r.tz, tzs[i], 1e-6);
            CHECK(!act.clamped_total && !act.clamped_yaw);
            CHECK(check_invariants(&R, &act) == 0);
        }
        /* +tz still loads the right motor forward (the sign convention). */
        blimp_wrench_t w = {0.0f, 0.0f, 0.0f, 0.02f};
        blimp_actuators_t act;
        blimp_mixer_apply(&R, &w, &act);
        CHECK(act.thrust_fr > 0.0f && act.thrust_fl < 0.0f);
    }

    TEST_GROUP("reverse: yaw beyond the reversed motor's reach adds only the surge it must");
    {
        /* |d| = 0.55 N is more than the 2 x 0.254 N a couple can make: the
         * reversed motor sits at its limit and the pair pushes forward by
         * |d| - 2 rev_m, where one-way it would push by all of |d|. */
        const float d = 0.55f;
        CHECK(d > 2.0f * rev_m && d < M + rev_m);
        blimp_wrench_t w = {0.0f, 0.0f, 0.0f, d * lf};
        blimp_actuators_t act;
        blimp_mixer_apply(&R, &w, &act);
        blimp_wrench_t r;
        blimp_mixer_forward(&R, &act, &r);
        CHECK_NEAR(act.thrust_fl, -rev_m, 1e-6);
        CHECK_NEAR(r.tz, d * lf, 1e-6);
        CHECK_NEAR(r.fx, d - 2.0f * rev_m, 1e-6);
        CHECK(!act.clamped_yaw && !act.clamped_total);
        blimp_actuators_t one;
        blimp_mixer_apply(&P, &w, &one);
        blimp_wrench_t r1;
        blimp_mixer_forward(&P, &one, &r1);
        CHECK(r.fx < r1.fx);
    }

    TEST_GROUP("reverse: the most yaw is one motor forward and one back");
    {
        blimp_wrench_t w = {0.0f, 0.0f, 0.0f, 100.0f};
        blimp_actuators_t act;
        blimp_mixer_apply(&R, &w, &act);
        CHECK(act.clamped_yaw);
        CHECK_NEAR(act.thrust_fr, M, 1e-6);
        CHECK_NEAR(act.thrust_fl, -rev_m, 1e-6);
        blimp_wrench_t neg = {0.0f, 0.0f, 0.0f, -100.0f};
        blimp_mixer_apply(&R, &neg, &act);
        CHECK_NEAR(act.thrust_fl, M, 1e-6);
        CHECK_NEAR(act.thrust_fr, -rev_m, 1e-6);
        CHECK(check_invariants(&R, &act) == 0);
    }

    TEST_GROUP("reverse: a negative surge brakes on both motors, and is capped");
    {
        blimp_wrench_t w = {-0.1f, 0.0f, 0.0f, 0.0f};
        blimp_actuators_t act;
        blimp_mixer_apply(&R, &w, &act);
        CHECK_NEAR(act.thrust_fl, -0.05f, 1e-7);
        CHECK_NEAR(act.thrust_fr, -0.05f, 1e-7);
        CHECK(!act.clamped_total);

        /* More than both motors can push back: each at its limit, flagged. */
        blimp_wrench_t big = {-100.0f, 0.0f, 0.0f, 0.0f};
        blimp_mixer_apply(&R, &big, &act);
        CHECK_NEAR(act.thrust_fl, -rev_m, 1e-6);
        CHECK_NEAR(act.thrust_fr, -rev_m, 1e-6);
        CHECK(act.clamped_total);

        /* Under the deadband a brake is zero, as a push is. */
        blimp_wrench_t tiny = {-0.5f * R.front_deadband_n, 0.0f, 0.0f, 0.0f};
        blimp_mixer_apply(&R, &tiny, &act);
        CHECK_NEAR(fabsf(act.thrust_fl) + fabsf(act.thrust_fr), 0.0f, 1e-9);
    }

    TEST_GROUP("reverse: yaw before brake");
    {
        /* A full brake with a turn: the turn is exact and the brake gets
         * what the reversed limits leave, so it is flagged. */
        blimp_wrench_t w = {-100.0f, 0.0f, 0.0f, 0.02f};
        blimp_actuators_t act;
        blimp_mixer_apply(&R, &w, &act);
        blimp_wrench_t r;
        blimp_mixer_forward(&R, &act, &r);
        CHECK_NEAR(r.tz, 0.02f, 1e-6);
        CHECK(!act.clamped_yaw);
        CHECK(act.clamped_total);
        CHECK(r.fx < 0.0f);
        CHECK_NEAR(act.thrust_fl, -rev_m, 1e-6);
        CHECK(check_invariants(&R, &act) == 0);
    }

    TEST_GROUP("reverse: the rear pair stays up only");
    {
        blimp_wrench_t down = {0.0f, 0.0f, -0.3f, 0.0f};
        blimp_actuators_t act;
        blimp_mixer_apply(&R, &down, &act);
        CHECK_NEAR(act.thrust_rl + act.thrust_rr, 0.0f, 1e-9);
        CHECK(act.clamped_heave);
    }

    TEST_GROUP("reverse: apply then forward round-trips when unsaturated");
    {
        const float fxs[] = {-0.10f, -0.03f, 0.03f, 0.10f};
        const float fzs[] = {0.0f, 0.05f};
        const float tzs[] = {-0.008f, -0.002f, 0.0f, 0.002f, 0.008f};
        int checked = 0;
        for (unsigned i = 0; i < sizeof(fxs)/sizeof(fxs[0]); i++)
        for (unsigned j = 0; j < sizeof(fzs)/sizeof(fzs[0]); j++)
        for (unsigned k = 0; k < sizeof(tzs)/sizeof(tzs[0]); k++) {
            blimp_wrench_t w = {fxs[i], 0.0f, fzs[j], tzs[k]};
            blimp_actuators_t act;
            blimp_mixer_apply(&R, &w, &act);
            CHECK(!act.clamped_total && !act.clamped_yaw && !act.clamped_heave);
            blimp_wrench_t r;
            blimp_mixer_forward(&R, &act, &r);
            CHECK_NEAR(r.fx, w.fx, 1e-6);
            CHECK_NEAR(r.fz, w.fz, 1e-6);
            CHECK_NEAR(r.tz, w.tz, 1e-6);
            checked++;
        }
        CHECK(checked == 40);
    }

    TEST_GROUP("reverse: the ratio is validated into [0, 1]");
    {
        blimp_params_t q = P;
        q.front_reverse_ratio = 0.74f;
        CHECK(blimp_params_validate(&q));
        CHECK_NEAR(q.front_reverse_ratio, 0.74f, 1e-9);
        q.front_reverse_ratio = 1.5f;
        CHECK(!blimp_params_validate(&q));
        CHECK_NEAR(q.front_reverse_ratio, 1.0f, 1e-9);
        q.front_reverse_ratio = -0.2f;
        CHECK(!blimp_params_validate(&q));
        CHECK_NEAR(q.front_reverse_ratio, 0.0f, 1e-9);
        q.front_reverse_ratio = NAN;
        CHECK(!blimp_params_validate(&q));
        CHECK_NEAR(q.front_reverse_ratio, 0.0f, 1e-9);
    }

    /* ---- fy is ignored -------------------------------------------------- */
    TEST_GROUP("fy has no effect on any motor");
    {
        blimp_wrench_t a = {0.2f, 0.0f,   0.1f, 0.004f};
        blimp_wrench_t b = {0.2f, 100.0f, 0.1f, 0.004f};
        blimp_wrench_t c = {0.2f, -100.0f, 0.1f, 0.004f};
        blimp_actuators_t aa, ab, ac;
        blimp_mixer_apply(&P, &a, &aa);
        blimp_mixer_apply(&P, &b, &ab);
        blimp_mixer_apply(&P, &c, &ac);
        CHECK(aa.thrust_fl == ab.thrust_fl && aa.thrust_fl == ac.thrust_fl);
        CHECK(aa.thrust_fr == ab.thrust_fr && aa.thrust_fr == ac.thrust_fr);
        CHECK(aa.thrust_rl == ab.thrust_rl && aa.thrust_rl == ac.thrust_rl);
        CHECK(aa.thrust_rr == ab.thrust_rr && aa.thrust_rr == ac.thrust_rr);
        blimp_wrench_t r;
        blimp_mixer_forward(&P, &ab, &r);
        CHECK_NEAR(r.fy, 0.0f, 1e-9);
    }

    /* ---- clamp flags are honest --------------------------------------- */
    TEST_GROUP("clamp flags report truthfully");
    {
        blimp_actuators_t act;
        blimp_wrench_t easy = {0.1f, 0.0f, 0.05f, 0.001f};
        blimp_mixer_apply(&P, &easy, &act);
        CHECK(!act.clamped_total);
        CHECK(!act.clamped_yaw);
        CHECK(!act.clamped_heave);

        blimp_wrench_t huge = {1000.0f, 0.0f, 1000.0f, 1000.0f};
        blimp_mixer_apply(&P, &huge, &act);
        CHECK(act.clamped_total);
        CHECK(act.clamped_yaw);
        CHECK(act.clamped_heave);
        CHECK(total_commanded(&act) <= P.max_total_thrust_n + 1e-6f);
    }

    /* ---- determinism ---------------------------------------------------- */
    TEST_GROUP("deterministic");
    {
        blimp_wrench_t w = {0.3f, 0.02f, 0.1f, 0.01f};
        blimp_actuators_t a1, a2;
        blimp_mixer_apply(&P, &w, &a1);
        for (int i = 0; i < 100; i++) {
            blimp_mixer_apply(&P, &w, &a2);
            CHECK(a1.thrust_fl == a2.thrust_fl);
            CHECK(a1.thrust_fr == a2.thrust_fr);
            CHECK(a1.thrust_rl == a2.thrust_rl);
            CHECK(a1.thrust_rr == a2.thrust_rr);
        }
    }

    TEST_REPORT();
}
