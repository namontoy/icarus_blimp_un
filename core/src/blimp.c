/* blimp.c - the top-level flight controller. See blimp/blimp.h for the
 * contract this implements. blimp_step() is called at the control rate by
 * BOTH the ESP32 firmware's control_task (firmware/main/main.c) and the
 * MuJoCo sim (via blimp_ffi.c) - this file is the ONLY place the estimator,
 * guidance, PID loops and mixer are wired together, so anything tuned in
 * sim transfers to the vehicle unchanged (blimp.h's own stated design goal).
 *
 * TICK ORDER (matches the task brief and is repeated at each step below):
 *   1. dt from timestamps, absurd dt rejected
 *   2. estimator: in->imu -> b->state
 *   3. arming: blocked while blimp_estimator_converging()
 *   4. failsafe: link staleness, ceiling breach, estimator invalid
 *   5. guidance: b->state (+ in->target, contact) -> b->demand
 *   6. PID loops: b->demand (guidance's setpoints) -> a final wrench
 *      (5-6 with in->hold.active: the bench hold's commanded setpoints
 *      through the same loops, guidance not stepped; blimp.h)
 *   7. mixer: final wrench -> *out
 *   8. *out is written on EVERY path, including every early return.
 *
 * WHY FAILSAFE IS "MOTORS OFF" (blimp.h's note on blimp_notify_link):
 * this vehicle is a near-neutrally-buoyant airship (Rule 11 forces mass to
 * equal displaced air within +/-10 gf) and is deliberately trimmed a few gf
 * HEAVY, with the rear pair carrying that trim in normal flight. Cutting
 * every motor therefore does not make it fall - it sinks gently to the
 * floor at the trim weight's terminal velocity, which is the safest thing
 * a vehicle with no perception link can do. A multirotor's "controlled
 * descent" posture is unnecessary and a blind "hold altitude" would wander
 * at 2 m among the balloons. See BLIMP_SAFE_WRENCH below - the hover
 * feed-forward is deliberately NOT added on the failsafe path.
 *
 * DESIGN DECISIONS NOT FULLY PINNED DOWN BY THE CONTRACT (reported here per
 * the house rule of implementing to the header as written and flagging
 * anything it leaves open - see mixer.c/estimator.c for the precedent of
 * this comment style):
 *
 * 1. THE GUIDANCE -> PID CONVENTION (the task's own explicit warning: "do
 *    not double-apply control"). blimp_guidance_step() returns a
 *    blimp_wrench_t, and blimp_wrench_t's own doc (types.h) says its fields
 *    are already N/N*m. But blimp_t (blimp.h) ALSO carries three persistent
 *    PID loop states - pid_alt, pid_yaw, pid_surge - matching exactly
 *    params.c's own already-built, already-tested comments:
 *        "altitude loop: metres -> newtons"       (comment on p->alt)
 *        "heading loop: radians -> newton-metres" (comment on p->yaw)
 *        "surge loop: range error (m) -> newtons" (comment on p->surge)
 *    Three PID states for three converted channels, and NO pid_lat field, is
 *    only consistent with one reading: guidance packs pre-PID ERROR values
 *    into demand.fx (surge/range error, metres), demand.fz (altitude error,
 *    metres) and demand.tz (heading error, radians) - reusing blimp_wrench_t
 *    as a convenient typed carrier, NOT as already-final forces/torques for
 *    those three fields - and this file's job is to run them through
 *    pid_surge/pid_alt/pid_yaw to get the real fx/fz/tz. demand.fy is
 *    pinned to 0: the confirmed airframe has no lateral actuator at all
 *    (types.h), and the mixer ignores the field.
 *
 *    A GENUINE, CONSIDERED ALTERNATIVE READING EXISTS, and is why this
 *    decision gets this much space: docs/07 4.1's ASCII loop diagram labels
 *    guidance's outputs "heading setpoint" and "F_x setpoint" (not
 *    "error"), and docs/03 3.1 item 4 says guidance "converts ... into
 *    heading and forward-thrust (Fx) setpoints" - reading both literally,
 *    Fx would already be a final force (computed directly by guidance's own
 *    range-to-thrust law, docs/07 6.1's pseudocode) with pid_surge unused,
 *    and fz/tz would be absolute targets requiring THIS file to compute the
 *    error against b->state itself, not consume b->demand as the error
 *    outright. Weighed against that: docs/07's OWN section 4.2 loop TABLE
 *    (more precise than its ASCII sketch) lists the altitude loop's input
 *    as literally "height error" and the heading loop's as "yaw error" -
 *    agreeing with params.c, not the diagram - and that same table has NO
 *    row at all for a surge/Fx loop, meaning docs/07's architecture doesn't
 *    even acknowledge pid_surge exists. Given hard rule 4 ("implement to
 *    the contract as written"; the header/params.c pair, not a design doc,
 *    is the contract) and that params.c's comment is real, in-repo, already
 *    load-bearing code - not a possibly-stale design sketch - the
 *    error-based reading above is what this file implements. It is still
 *    an INFERENCE, not something guidance.h states outright, because
 *    guidance.c (the module that actually documents its own demand
 *    convention, per the task) did not exist in this tree at the time this
 *    file was written/verified - see the concerns list in this module's
 *    report for the current status. If guidance.c's own comments say
 *    otherwise, THIS FILE'S STEP 6 IS WHAT NEEDS TO CHANGE, not the PID
 *    module or the mixer.
 * 2. blimp_estimator_zero() (estimator.h: "Call during the untimed PREPARE
 *    phase, never mid-run") is called from HERE, at the exact tick arming
 *    transitions from false->true - since blimp_guidance_arm() is what
 *    enters PREPARE (guidance.h), and blimp.c is the only place both the
 *    estimator and the arm transition are visible together. Neither
 *    blimp.h nor estimator.h says outright who calls blimp_estimator_zero();
 *    this is the only place in the whole module graph positioned to do it
 *    at the right moment.
 * 3. Arming is a LATCH, not a continuous gate. blimp_estimator_converging()
 *    is re-derived from e->stationary_s, which RESETS TO ZERO the instant
 *    the vehicle is no longer "stationary" (estimator.c) - i.e. it goes
 *    back to "converging" on literally the first tick of real flight after
 *    a good arm. Gating armed on every tick would therefore auto-disarm a
 *    flying vehicle the moment it starts moving, which is absurd and unsafe.
 *    So blimp_estimator_converging() gates only the false->true ARMING
 *    TRANSITION (task step 3's literal wording: "block [arming] while
 *    converging"); once armed, only an explicit arm_request=false disarms.
 * 4. SAFE HOLD is the literal all-zero wrench (fx=fy=fz=tz=0), not an
 *    active rate-damping controller. docs/08-firmware-setup.md section 7
 *    spells out the concrete failsafe command: "no forward thrust, no yaw
 *    torque, a neutral ... heave command" - i.e. zero, not a servoed hold.
 *    blimp_t has no spare PID state to dedicate to hold-damping (its three
 *    PID fields are already claimed by decision 1 above, and blimp.h is a
 *    shared header this task may not modify), so reusing pid_alt/pid_yaw
 *    for a differently-shaped calculation during hold would leave them with
 *    stale prev_error on the tick control resumes - the exact "derivative
 *    kick" pid.h's own header warns about. Instead, ALL THREE PIDs are
 *    reset (blimp_pid_reset) on every hold tick, so the first tick back in
 *    normal control gets pid.c's documented "no derivative kick on first
 *    sample after reset" behaviour for free, and hold itself is trivially
 *    provably safe (mixer.c(NULL demand path) proves *out is all-zero,
 *    Rule-13-legal, for the zero wrench regardless of params).
 * 5. blimp_params_validate() is called unconditionally at the top of every
 *    tick. Nothing in blimp.h says this outright, but test_mixer.c's own
 *    "adversarial params" group says it in so many words: "Enforcing the
 *    100 gf ceiling is params_validate's job, and blimp.c must call it."
 *    Calling it every tick (not just "after a live-tuning update", which
 *    blimp.c has no event hook for) is the only way to make that guarantee
 *    hold no matter how b->params was last written (direct struct access,
 *    or blimp_ffi.c's set_params).
 * 6. "Estimator invalid" (failsafe trigger) is read as
 *    !state.attitude_valid (estimator hasn't produced a real sample yet -
 *    true only before the very first estimator_update() call, since
 *    attitude_valid latches e->initialised, which never resets) OR any
 *    state field failing isfinite(). The header doesn't define "invalid"
 *    more precisely than that; the isfinite() half is defence in depth,
 *    not something estimator.c is expected to ever actually produce.
 * 7. "Ceiling breach" only fires when state.altitude_valid is true this
 *    tick - an unknown altitude cannot be judged against a ceiling, and the
 *    estimator degrading gracefully to "no source" (estimator.h) should not
 *    itself be treated as a ceiling breach.
 */
#include "blimp/blimp.h"

#include <math.h>
/* NULL. MSVC's <math.h> happens to define it, glibc's does not; a Linux build
 * failed on that, so name the header that actually owns it. */
#include <stddef.h>

/* Anything from here to the next control tick this large is not a real
 * gap - a missed/late tick, a paused debugger, or (on the very first ever
 * call) the sentinel described below. Matches BLIMP_EST_MAX_DT_S's role and
 * value in estimator.c, for the same reason: at a 200 Hz-class control
 * rate, 2 s is already several hundred missed ticks. */
#define BLIMP_CTRL_MAX_DT_S   2.0f

/* How far a link packet's time may run ahead of the tick's own t_us and
 * still count as fresh (step 4). The race it covers is a few milliseconds
 * (a packet arriving during the tick); 100 ms is generous for that and far
 * below link_timeout_s, so a clock that is really wrong still trips the
 * watchdog. */
#define BLIMP_LINK_AHEAD_MAX_US   100000u

/* The literal contract: on any early return, *out must be the trivially
 * safe, Rule-13-legal, all-zero command. See decision 4 above for why this
 * is a plain zero wrench rather than an active hold controller. */
static const blimp_wrench_t BLIMP_SAFE_WRENCH = { 0.0f, 0.0f, 0.0f, 0.0f };

void blimp_init(blimp_t *b)
{
    if (!b) {
        return;
    }

    blimp_params_defaults(&b->params);
    (void)blimp_params_validate(&b->params);

    blimp_estimator_init(&b->est);
    /* Rule 2/5: 4 red balloons, up to 3 loop attempts per course - the only
     * numbers the rulebook fixes independent of any per-run configuration.
     * A real run's actual per-run waypoint ORDER (Rule 2) is not a count and
     * is not something blimp_init() has any argument to receive; nothing in
     * blimp.h's blimp_init(blimp_t*) signature carries it. */
    blimp_guidance_init(&b->guid, 4, 3);

    blimp_pid_reset(&b->pid_alt);
    blimp_pid_reset(&b->pid_yaw);
    blimp_pid_reset(&b->pid_surge);

    b->state       = (blimp_state_t){0};
    b->demand      = (blimp_wrench_t){0};
    b->act         = (blimp_actuators_t){0};
    b->last_cmd_us = 0;
    b->last_step_us = 0;
    b->armed       = false;
}

void blimp_notify_link(blimp_t *b, uint32_t t_us)
{
    if (!b) {
        return;
    }
    b->last_cmd_us = t_us;
}

const char *blimp_version(void)
{
    /* Logged with every flight so a log can be matched to the code that
     * produced it (blimp.h). A plain literal, not a build timestamp: hard
     * rule 6 bans time-of-day/non-determinism, and this must be identical
     * for identical source. Bump the number when this file's behaviour
     * changes in a way worth distinguishing in a flight log. */
    return "blimp-core-0.2.1";   /* 0.2.1: a link packet newer than the tick is
                                  * fresh; the arming tick steers on the new
                                  * origin. 0.2.0: the bench hold. */
}

/* Resets every control-loop PID. Used whenever the next tick's error signal
 * is not a continuation of the previous one (arming, disarming, failsafe
 * entry/exit, a rejected dt) - see decision 4 above. */
static void reset_control_loops(blimp_t *b)
{
    blimp_pid_reset(&b->pid_alt);
    blimp_pid_reset(&b->pid_yaw);
    blimp_pid_reset(&b->pid_surge);
}

/* Altitude: PID on the height error PLUS the hover feed-forward - the rear
 * pair has to carry the trim weight before it can do anything else. The
 * PID's output clamp is narrowed every tick to the rear pair's REAL range
 * around the feed-forward, [-hover, heave_budget - hover], so conditional
 * integration sees the actuator limit instead of the nominal one in params.
 * Done on a local copy: params stay the single source of truth and are never
 * mutated here. Shared by guidance's run and the bench hold. */
static float altitude_loop(blimp_t *b, float err_m, float dt)
{
    blimp_pid_params_t alt_p = b->params.alt;
    const float hover    = b->params.hover_thrust_n;
    const float heave_hi = b->params.heave_budget_n - hover;
    if (alt_p.out_min < -hover)    { alt_p.out_min = -hover; }
    if (alt_p.out_max > heave_hi)  { alt_p.out_max = heave_hi; }
    if (alt_p.out_max < alt_p.out_min) { alt_p.out_max = alt_p.out_min; }
    /* Damping from the estimator's fused vertical rate rather than from
     * differentiating the (vision-noisy) altitude error; only while the
     * altitude is actually known, otherwise the accel-only dead-reckoned vz
     * drifts and would be fought for no reason. */
    const uint32_t fix_age_us = b->state.t_us - b->est.last_abs_us;
    const bool fix_fresh = b->est.abs_ever && fix_age_us < 500000u;
    const float d_alt = (b->state.altitude_valid && fix_fresh)
                            ? -b->state.vz : 0.0f;
    return blimp_pid_step_d(&b->pid_alt, &alt_p, err_m, d_alt, dt) + hover;
}

/* Heading: P(+I) on the heading error, with the derivative term taken from
 * the GYRO yaw rate rather than by differentiating the error (pid.h's
 * blimp_pid_step_d exists for exactly this). The error shrinks as the nose
 * swings toward the setpoint, so d(error)/dt = -yaw_rate. */
static float heading_loop(blimp_t *b, float err_rad, float dt)
{
    return blimp_pid_step_d(&b->pid_yaw, &b->params.yaw, blimp_wrap_pi(err_rad),
                            -b->state.yaw_rate, dt);
}

/* The bench hold (blimp.h): the run's own loops on commanded setpoints.
 * Guidance is not stepped, so its state stays where arming or a failsafe
 * left it; b->demand carries the errors the loops ran on (decision 1's
 * convention: fz the height error, tz the heading error), fx the commanded
 * surge itself, since no range error exists here. */
static void hold_wrench(blimp_t *b, const blimp_hold_t *h, float dt,
                        blimp_wrench_t *w)
{
    *w = BLIMP_SAFE_WRENCH;
    b->demand = BLIMP_SAFE_WRENCH;

    /* Surge is open loop here: a constant push, never reverse, never above
     * the run's own cruise cap. */
    blimp_pid_reset(&b->pid_surge);
    if (isfinite(h->surge_n) && h->surge_n > 0.0f) {
        const float cap = b->params.surge.out_max;
        w->fx = (isfinite(cap) && h->surge_n > cap) ? cap : h->surge_n;
        if (!(w->fx >= 0.0f)) {
            w->fx = 0.0f;
        }
        b->demand.fx = w->fx;
    }

    if (h->altitude_held && isfinite(h->altitude_m) && b->state.altitude_valid) {
        b->demand.fz = h->altitude_m - b->state.altitude;
        w->fz = altitude_loop(b, b->demand.fz, dt);
    } else {
        blimp_pid_reset(&b->pid_alt);
    }

    if (h->heading_held && isfinite(h->heading_rad)) {
        b->demand.tz = blimp_wrap_pi(h->heading_rad - b->state.yaw);
        w->tz = heading_loop(b, b->demand.tz, dt);
    } else {
        blimp_pid_reset(&b->pid_yaw);
    }
}

/* Every early-return path (b non-NULL) funnels through here so *out is
 * ALWAYS written, via blimp_mixer_apply()'s own documented "every early
 * return leaves *out at an all-zero...command" guarantee (mixer.c). Also
 * keeps b->demand and b->act (blimp.h's own persisted-for-telemetry fields,
 * read directly by firmware/main/main.c and by blimp_ffi.c's accessors) in
 * sync with what was actually commanded, the same way the normal path does -
 * without this, b->act would silently stay all-zero forever regardless of
 * what *out carried. */
static void safe_hold(blimp_t *b, blimp_actuators_t *out)
{
    b->demand = BLIMP_SAFE_WRENCH;
    blimp_mixer_apply(&b->params, &BLIMP_SAFE_WRENCH, &b->act);
    if (out) {
        *out = b->act;
    }
}

void blimp_step(blimp_t *b, const blimp_input_t *in, blimp_actuators_t *out)
{
    /* ---- NULL safety (hard rule 7) -------------------------------------
     * Neither b nor in can be dereferenced without both present. Still
     * write a safe *out (step 8) using whatever IS available. */
    if (!b || !in) {
        blimp_mixer_apply(b ? &b->params : NULL, &BLIMP_SAFE_WRENCH, out);
        return;
    }

    /* Decision 5: never let a corrupted/unvalidated params struct (however
     * it got that way) reach the mixer this tick. */
    (void)blimp_params_validate(&b->params);

    /* ---- 1. dt from timestamps ------------------------------------------
     * b->last_step_us == 0 is blimp_init()'s own sentinel for "no previous
     * tick" - treated as a hard reject rather than computing dt against
     * epoch zero (which would silently compute "time since boot", not
     * "time since last tick", on the very first call). Beyond that, only
     * a strictly-advancing, finite, <= BLIMP_CTRL_MAX_DT_S gap is accepted -
     * same shape as estimator.c's own dt handling, so the two don't drift
     * apart in behaviour for the same class of bad input. last_step_us
     * always advances (even on a rejected tick) so one bad sample cannot
     * poison every dt computed after it. */
    const bool first_tick   = (b->last_step_us == 0u);
    const bool ts_advanced  = in->t_us > b->last_step_us;
    float dt    = 0.0f;
    bool  dt_ok = false;
    if (!first_tick && ts_advanced) {
        dt = (float)(in->t_us - b->last_step_us) * 1.0e-6f;
        dt_ok = isfinite(dt) && dt > 0.0f && dt <= BLIMP_CTRL_MAX_DT_S;
    }
    b->last_step_us = in->t_us;

    /* ---- 2. estimator: in->imu -> b->state ------------------------------
     * Always runs, even on a rejected top-level dt: the estimator derives
     * its OWN dt from in->imu.t_us independently (estimator.h) and has its
     * own, separately-guarded reject-and-skip path. Running it unconditionally
     * is also the only way b->state is ever produced at all on tick 1.
     *
     * VISION ALTITUDE FIX. The confirmed PCB has no altitude sensor. When a
     * balloon is in view, its elevation and range against its known 2.0 m
     * centre height (Rule 22) give our altitude directly (same geometry as
     * guidance.c's altitude_error(), using the previous tick's pitch). It
     * is injected here as an absolute range-to-floor fix so the estimator
     * fuses, filters and REMEMBERS it (estimator.h's validity window) and
     * keeps a vertical rate between sightings. A real ToF/baro sample, if
     * ever fitted, takes precedence and this block does nothing. */
    blimp_imu_sample_t imu = in->imu;
    if (!imu.range_valid && !imu.baro_valid && in->target.alt_fix_valid &&
        isfinite(in->target.alt_fix_m)) {
        /* Offboard fix from any balloon in view (guidance.h) - the better
         * source, since it does not need the TARGET to be visible. */
        imu.range_alt   = in->target.alt_fix_m;
        imu.range_valid = true;
    } else if (!imu.range_valid && !imu.baro_valid && in->target.visible &&
        b->state.attitude_valid &&
        isfinite(in->target.range_m) && isfinite(in->target.elevation_rad) &&
        isfinite(b->state.pitch) && isfinite(b->params.cam_dz_m)) {
        float r = in->target.range_m;
        if (r < 0.0f)  { r = 0.0f; }
        if (r > 20.0f) { r = 20.0f; }
        const float el_world = in->target.elevation_rad - b->state.pitch;
        const float alt_vis  = BLIMP_RULE22_WP_HEIGHT_M -
                               (r * sinf(el_world) + b->params.cam_dz_m);
        if (isfinite(alt_vis)) {
            imu.range_alt   = alt_vis;
            imu.range_valid = true;
        }
    }
    blimp_estimator_update(&b->est, &b->params, &imu, &b->state);

    /* ---- 3. arming: latched, blocked (only at the transition) while the
     * estimator is converging (decision 3 above). ------------------------- */
    if (!b->armed) {
        if (in->arm_request && !blimp_estimator_converging(&b->est)) {
            /* Decision 2: zero the barometric reference / latch the yaw
             * origin at the instant PREPARE begins, using THIS tick's
             * sample - not a stale one from many ticks ago. */
            blimp_estimator_zero(&b->est, &in->imu);
            /* b->state came out of update() above, before the zero: refresh
             * it, or this tick steers on the old origin (one tick at the
             * yaw torque cap on the S3's bench hold, 2026-10-30). */
            blimp_estimator_output(&b->est, &imu, &b->state);
            blimp_guidance_arm(&b->guid, in->t_us);
            b->armed = true;
            reset_control_loops(b);
        }
    } else if (!in->arm_request) {
        b->armed = false;
    }

    if (!b->armed) {
        /* Disarmed: guidance's own IDLE state literally means this -
         * guidance.h documents BLIMP_GUID_IDLE as "disarmed". Set it
         * directly rather than calling guidance_step() at all - there is
         * no public "disarm" entry point in guidance.h, and calling
         * guidance_step() with an armed=false vehicle is not a scenario
         * the header describes. */
        b->guid.state = BLIMP_GUID_IDLE;
        reset_control_loops(b);
        safe_hold(b, out);
        return;
    }

    /* ---- 4. failsafe checks --------------------------------------------- */
    /* Unsigned, so the clock's wrap is harmless. A shell that stamps packets
     * on another task can hand over a packet NEWER than this tick's t_us (it
     * landed between the tick taking its time and calling notify_link), and
     * unsigned that difference wrapped to ~71 minutes: the S3's bench hold
     * fell into failsafe for one tick in about 150, every PID reset
     * (2026-10-30). A packet up to BLIMP_LINK_AHEAD_MAX_US ahead of the tick
     * is fresh; anything further ahead is not a race but a broken clock, and
     * stays stale. */
    uint32_t link_age_us = in->t_us - b->last_cmd_us;
    if (b->last_cmd_us - in->t_us <= BLIMP_LINK_AHEAD_MAX_US) {
        link_age_us = 0u;
    }
    const float link_age_s = (float)link_age_us * 1.0e-6f;
    const bool link_stale = !isfinite(link_age_s) ||
                            link_age_s > b->params.link_timeout_s;

    /* Ceiling guard with hysteresis: enter above max_altitude_m, and stay
     * in failsafe (motors off, sinking) until 0.5 m below it, so the vehicle
     * does not chatter in and out of failsafe right at the limit. */
    const bool in_failsafe = (b->guid.state == BLIMP_GUID_FAILSAFE);
    const float ceiling = in_failsafe ? b->params.max_altitude_m - 0.5f
                                      : b->params.max_altitude_m;
    const bool ceiling_breach = b->state.altitude_valid &&
                                isfinite(b->state.altitude) &&
                                b->state.altitude > ceiling;

    const bool estimator_invalid =
        !b->state.attitude_valid ||
        !isfinite(b->state.roll)  || !isfinite(b->state.pitch) ||
        !isfinite(b->state.yaw)   || !isfinite(b->state.altitude) ||
        !isfinite(b->state.vz)    || !isfinite(b->state.yaw_rate);

    if (link_stale || ceiling_breach || estimator_invalid) {
        if (!in_failsafe) {
            b->guid.state = BLIMP_GUID_FAILSAFE;
            b->guid.state_entry_us = in->t_us;
        }
        reset_control_loops(b);
        safe_hold(b, out);
        return;
    }
    /* Not (or no longer) in a failsafe condition. If guidance is still
     * parked in FAILSAFE, its own FAILSAFE case resumes the run - Rule 7
     * forbids human intervention, so recovery has to be autonomous. */

    /* A rejected top-level dt: not a "fault" in the guidance/link/ceiling
     * sense (task step 1 lists it separately from step 4's failsafe
     * checks), so guid.state is left untouched - just skip this one tick's
     * guidance/PID advancement rather than feed a meaningless dt into
     * either. */
    if (!dt_ok) {
        reset_control_loops(b);
        safe_hold(b, out);
        return;
    }

    blimp_wrench_t final_wrench;

    if (in->hold.active) {
        /* ---- 5-6 (bench hold): commanded setpoints through the same loops
         * (blimp.h's blimp_hold_t) instead of guidance's. */
        hold_wrench(b, &in->hold, dt, &final_wrench);
    } else {
        /* ---- 5. guidance: b->state (+ target, contact) -> b->demand ----- */
        (void)blimp_guidance_step(&b->guid, &b->params, &b->state, &in->target,
                                  in->contact_detected, dt, &b->demand);

        /* ---- 6. PID loops: b->demand's setpoints -> a final wrench ------
         * See decision 1 above for why fx/fz/tz are treated as pre-PID
         * errors and fy is passed straight through as an already-final
         * force. */

        /* Surge: P on the range error. The mixer discards anything negative
         * (no reverse thrust), and the PID's own out_min is 0 so its
         * anti-windup bookkeeping agrees with that. */
        final_wrench.fx = blimp_pid_step(&b->pid_surge, &b->params.surge,
                                         b->demand.fx, dt);

        /* No lateral actuator exists (types.h). */
        final_wrench.fy = 0.0f;

        final_wrench.fz = altitude_loop(b, b->demand.fz, dt);

        /* Heading on the vision bearing error (guidance's demand.tz). */
        final_wrench.tz = heading_loop(b, b->demand.tz, dt);
    }

    /* ---- 7. mixer: final wrench -> *out ----------------------------------
     * ---- 8. *out is written here, on the only path that isn't an early
     * return - blimp_mixer_apply() always writes *out on its own. Written
     * into b->act first (same reasoning as safe_hold()) so it and *out
     * never disagree, then copied to *out if the caller wants it too. */
    blimp_mixer_apply(&b->params, &final_wrench, &b->act);
    if (out) {
        *out = b->act;
    }
}
