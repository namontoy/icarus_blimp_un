/* mixer.c - control allocation for four fixed, unidirectional motors.
 * See blimp/mixer.h for the contract this implements.
 *
 * GEOMETRY (types.h): front pair pushes +x at lateral offsets +/-l_f, rear
 * pair pushes +z. The front pair is a 2-DOF sum/difference allocation; the
 * rear pair is a 1-DOF sum with an equal split.
 *
 * WHY THE PRIORITY ORDER IS HEAVE > YAW > SURGE:
 *   - Heave keeps the vehicle on the 2 m waypoint plane (Rule 22). A balloon
 *     is ~0.55 m across; leave the plane by half that and the approach is a
 *     miss, and a miss voids the whole run under Rule 7.
 *   - Yaw keeps the nose on the balloon. Same argument.
 *   - Surge only changes WHEN we arrive, never WHETHER.
 *
 * EVERYTHING IS CLAMPED, NOTHING IS SCALED. The previous (bicopter) mixer
 * scaled both rotors by a common factor to keep the yaw RATIO; on a
 * unidirectional pair that would shrink the yaw DIFFERENCE, which is the
 * quantity that actually turns the vehicle. Here the difference is
 * protected first and the common-mode (surge) absorbs every clamp.
 */
#include "blimp/mixer.h"

#include <math.h>

/* Demand components are clamped to this magnitude (N or N*m) right after
 * NaN/Inf sanitisation - ~1e6x the entire budget, so nothing finite-but-
 * huge can overflow an intermediate downstream. Defence in depth for the
 * module that sits last before the motors. */
#define BLIMP_MIXER_DEMAND_ABS_MAX 1.0e6f

/* Deadband on the front pair, in newtons: params' front_deadband_n since
 * 2026-10-30 (a fixed 0.5 gf before). Without it a yaw demand of a few
 * micro-newton-metres from the heading loop's noise floor would, through the
 * yaw-floor rule, keep one front motor creeping and walk the vehicle forward
 * while it is supposed to be holding station (observed in simulation: 0.5 m
 * of drift during PREPARE). As a constant it was wrong for the real motors:
 * 0.5 gf is half of what one gives at the team's 30 % duty cap, and on the
 * rope rig it silenced every correction the bench hold asked for. */
#define BLIMP_MIXER_FALLBACK_DEADBAND_N BLIMP_GF_TO_N(0.5f)

/* Fallbacks if a params field is NaN/absent. Mirror params.c's defaults. */
#define BLIMP_MIXER_FALLBACK_L_F        0.10f
#define BLIMP_MIXER_FALLBACK_MOTOR_MAX  BLIMP_GF_TO_N(35.0f)
#define BLIMP_MIXER_FALLBACK_HEAVE_FRAC 0.40f

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

static float sanitize_demand_component(float v)
{
    if (!isfinite(v)) {
        return 0.0f;
    }
    return clampf(v, -BLIMP_MIXER_DEMAND_ABS_MAX, BLIMP_MIXER_DEMAND_ABS_MAX);
}

/* Sanitised copies of the params fields this module reads, so one bad or
 * adversarial field cannot propagate into Inf/NaN output. Never mutates *p:
 * the mixer is a pure function and p is const. */
typedef struct {
    float l_f;          /* > 0                                  */
    float max_total;    /* > 0                                  */
    float motor_max;    /* > 0                                  */
    float heave_budget; /* 0 <= heave_budget <= max_total       */
    float deadband;     /* 0 <= deadband <= motor_max / 2       */
} blimp_mixer_safe_params_t;

static blimp_mixer_safe_params_t sanitize_params(const blimp_params_t *p)
{
    blimp_mixer_safe_params_t s;

    s.l_f = (p && isfinite(p->front_half_span_m) && p->front_half_span_m > 1e-4f)
                ? p->front_half_span_m : BLIMP_MIXER_FALLBACK_L_F;

    s.max_total = (p && isfinite(p->max_total_thrust_n) &&
                   p->max_total_thrust_n > 0.0f)
                      ? p->max_total_thrust_n
                      : BLIMP_RULE13_MAX_TOTAL_N;

    s.motor_max = (p && isfinite(p->motor_max_thrust_n) &&
                   p->motor_max_thrust_n > 0.0f)
                      ? p->motor_max_thrust_n
                      : BLIMP_MIXER_FALLBACK_MOTOR_MAX;

    s.heave_budget = (p && isfinite(p->heave_budget_n) && p->heave_budget_n >= 0.0f)
                         ? p->heave_budget_n
                         : BLIMP_MIXER_FALLBACK_HEAVE_FRAC * s.max_total;
    if (s.heave_budget > s.max_total) {
        s.heave_budget = s.max_total;
    }

    s.deadband = (p && isfinite(p->front_deadband_n) && p->front_deadband_n >= 0.0f)
                     ? p->front_deadband_n : BLIMP_MIXER_FALLBACK_DEADBAND_N;
    if (s.deadband > 0.5f * s.motor_max) {
        s.deadband = 0.5f * s.motor_max;
    }
    return s;
}

void blimp_mixer_apply(const blimp_params_t *p,
                       const blimp_wrench_t *demand,
                       blimp_actuators_t *out)
{
    if (!out) {
        return;
    }

    /* Fail safe by default: every early return below leaves *out at an
     * all-zero, trivially rule-legal command. On this vehicle all-zero is
     * "motors off, sink gently on the trim weight" - the correct failsafe. */
    out->thrust_fl     = 0.0f;
    out->thrust_fr     = 0.0f;
    out->thrust_rl     = 0.0f;
    out->thrust_rr     = 0.0f;
    out->clamped_total = false;
    out->clamped_yaw   = false;
    out->clamped_heave = false;

    if (!demand) {
        return;
    }

    const blimp_mixer_safe_params_t sp = sanitize_params(p);

    const float fx = sanitize_demand_component(demand->fx);
    const float fz = sanitize_demand_component(demand->fz);
    const float tz = sanitize_demand_component(demand->tz);
    /* demand->fy deliberately unread: not realisable (types.h). */

    /* ---- Priority 1+2: heave first, rear pair, equal split -------------
     * Negative heave cannot be produced (motors only push up) - clip to 0
     * and say so. Over-budget heave is clipped to the smaller of the heave
     * budget, twice the per-motor cap, and the whole Rule 13 budget. */
    float heave = fz;
    bool heave_clipped = false;
    bool heave_over_cap = false;
    if (heave < 0.0f) {
        heave = 0.0f;
        heave_clipped = true;
    }
    float heave_cap = fminf(sp.heave_budget, 2.0f * sp.motor_max);
    heave_cap = fminf(heave_cap, sp.max_total);
    if (heave > heave_cap) {
        heave = heave_cap;
        heave_clipped  = true;
        heave_over_cap = true;
    }
    const float t_rl = 0.5f * heave;
    const float t_rr = 0.5f * heave;

    /* ---- Priority 3: front pair - yaw exact, surge takes what is left ---
     * Required difference d = T_fr - T_fl = tz / l_f. Realising |d| with
     * both motors >= 0 needs the higher motor at >= |d|, so |d| is bounded
     * by the per-motor cap and by whatever Rule 13 budget the rear pair
     * left. Within that, the pair sum s must satisfy
     *     |d| <= s <= min(front_budget, 2*motor_max - |d|)
     * (lower bound: lower motor >= 0; upper: higher motor <= motor_max and
     * the pair within budget). Surge is clamped into that interval. */
    float front_budget = sp.max_total - heave;
    if (front_budget < 0.0f) {
        front_budget = 0.0f;
    }

    float d = clampf(tz / sp.l_f, -BLIMP_MIXER_DEMAND_ABS_MAX,
                     BLIMP_MIXER_DEMAND_ABS_MAX);
    if (fabsf(d) < sp.deadband) {
        d = 0.0f;
    }
    bool yaw_clipped = false;
    const float d_max = fminf(sp.motor_max, front_budget);
    if (fabsf(d) > d_max) {
        d = (d > 0.0f) ? d_max : -d_max;
        yaw_clipped = true;
    }

    float surge = (fx > 0.0f) ? fx : 0.0f;   /* no reverse thrust exists */
    if (surge < sp.deadband) {
        surge = 0.0f;
    }
    bool surge_clipped = false;
    const float s_min = fabsf(d);
    const float s_max = fminf(front_budget, 2.0f * sp.motor_max - fabsf(d));
    if (surge > s_max) {
        surge = s_max;
        surge_clipped = true;
    }
    if (surge < s_min) {
        /* Yaw floor: the minimum surge that realises d exactly. Not a
         * clamp on anything the caller asked for - it is the price of a
         * unidirectional pair - so it does not raise clamped_total. */
        surge = s_min;
    }

    float t_fl = 0.5f * (surge - d);
    float t_fr = 0.5f * (surge + d);
    /* Belt-and-braces against float rounding; the interval algebra above
     * guarantees 0 <= t <= motor_max analytically. */
    t_fl = clampf(t_fl, 0.0f, sp.motor_max);
    t_fr = clampf(t_fr, 0.0f, sp.motor_max);

    out->thrust_fl     = t_fl;
    out->thrust_fr     = t_fr;
    out->thrust_rl     = t_rl;
    out->thrust_rr     = t_rr;
    out->clamped_heave = heave_clipped;
    out->clamped_yaw   = yaw_clipped;
    out->clamped_total = surge_clipped || heave_over_cap;
}

void blimp_mixer_forward(const blimp_params_t *p,
                         const blimp_actuators_t *act,
                         blimp_wrench_t *realised)
{
    if (!realised) {
        return;
    }

    realised->fx = 0.0f;
    realised->fy = 0.0f;
    realised->fz = 0.0f;
    realised->tz = 0.0f;

    if (!act) {
        return;
    }

    const blimp_mixer_safe_params_t sp = sanitize_params(p);

    const float fl = isfinite(act->thrust_fl) ? act->thrust_fl : 0.0f;
    const float fr = isfinite(act->thrust_fr) ? act->thrust_fr : 0.0f;
    const float rl = isfinite(act->thrust_rl) ? act->thrust_rl : 0.0f;
    const float rr = isfinite(act->thrust_rr) ? act->thrust_rr : 0.0f;

    realised->fx = fl + fr;
    realised->fz = rl + rr;
    realised->tz = sp.l_f * (fr - fl);
}
