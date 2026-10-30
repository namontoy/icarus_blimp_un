/* guidance.c - the autonomy state machine for IAR-LARC waypoint racing.
 *
 * See guidance.h for the contract and for the two rules this is built around.
 *
 * THE DEMAND CONVENTION (restated here because it is load-bearing and
 * genuinely counter-intuitive - blimp.c depends on it exactly):
 *
 *     demand.fx = RANGE error    (metres)  -> blimp.c: pid_surge -> newtons
 *     demand.fz = ALTITUDE error (metres)  -> blimp.c: pid_alt   -> newtons
 *     demand.tz = HEADING error  (radians) -> blimp.c: pid_yaw   -> N*m
 *     demand.fy = always 0 (no lateral actuator on this airframe)
 *
 * The first three reuse blimp_wrench_t as a typed carrier for pre-PID errors
 * rather than as forces.
 *
 * WHAT THE AIRFRAME CANNOT DO, AND HOW EACH STATE COPES (types.h):
 *   - no lateral force  -> turn-to-face-then-drive; cross-track error is
 *                          corrected by re-aiming (ALIGN), never by strafing.
 *   - no reverse thrust -> after a touch the vehicle COASTs with motors off
 *                          and lets drag bleed the speed before turning,
 *                          instead of backing away.
 *   - no yaw in place   -> a yaw demand with zero surge is realised by the
 *                          mixer as the minimum surge that keeps both front
 *                          motors non-negative, so ALIGN and SEARCH are
 *                          tight arcs, not pivots. The states are written
 *                          knowing the vehicle creeps forward while turning.
 *   - no push-down      -> altitude corrections downward are slow (the
 *                          vehicle sinks on its trim weight only), so the
 *                          altitude error is fed continuously in every
 *                          state and never allowed to build up.
 *
 * ALTITUDE WITHOUT AN ALTITUDE SENSOR: the confirmed PCB has only an
 * MPU-6050. When the estimator reports no altitude source, the height error
 * is derived from the target balloon (elevation angle, range, and its known
 * 2.0 m centre height under Rule 22) - see altitude_error(). This is
 * available exactly when it matters: whenever a balloon is in view.
 *
 * ANTI-LIVELOCK IS A HARD REQUIREMENT, NOT A NICETY. Under Rule 7 a run where
 * "the vehicle requires human intervention" is void, and under Rule 5 the
 * three loops run back-to-back with no intervention at all. A state machine
 * that can wedge therefore does not merely perform badly, it scores zero. So
 * EVERY state here has a bounded dwell and a defined escape, including the
 * ones that "obviously" cannot stick.
 */
#include "blimp/guidance.h"

#include <math.h>

/* ---- Local tuning that is genuinely internal to the state machine --------
 * Anything a field engineer would want to change lives in blimp_params_t.
 * These are structural constants of the algorithm itself. */

/* How long the target may be missing before APPROACH/ALIGN give up and scan. */
#define GUID_LOST_GRACE_S        0.60f

/* Heading error commanded while yaw-scanning in SEARCH. Expressed as an
 * error (per the demand convention) rather than a rate, because the yaw PID
 * downstream consumes errors. Large enough to produce steady rotation,
 * small enough not to slam the torque clamp. */
#define GUID_SEARCH_ERR_RAD      0.45f

/* Motors-off dwell after a touch (or a missed approach). With no reverse
 * thrust this is the only way to shed speed before turning; long enough for
 * drag to take the edge off, short enough not to waste timed seconds. */
#define GUID_COAST_S             1.00f

/* Terminal commit: once inside the touch radius we stop steering on range and
 * just push gently through, because the balloon is bigger than our position
 * uncertainty and hunting at contact range is how touches get missed. */
#define GUID_TOUCH_PUSH_M        0.25f
#define GUID_TOUCH_TIMEOUT_S     3.00f

/* Range error is clamped before it reaches the surge PID: a 6 m error on a
 * vehicle with 100 gf of thrust just saturates the loop for seconds. */
#define GUID_MAX_RANGE_ERR_M     1.50f

/* Altitude error is clamped the same way, and tighter: climbing is strong
 * (the rear pair) but descending is weak (trim weight only), so a large
 * climb demand overshoots the plane and takes seconds to come back. With
 * the altitude loop's rate damping this bounds the climb rate. */
#define GUID_MAX_ALT_ERR_M       0.50f

/* Backstop dwell for every state. Nothing may sit longer than this without
 * an escape; see the anti-livelock note in the file header. */
#define GUID_STATE_HARD_LIMIT_S  25.0f

/* SEARCH with a hint: turn to the hinted heading first; once within this
 * error of it and still seeing nothing, fall back to the blind scan. */
#define GUID_HINT_REACHED_RAD    0.20f

/* BLIND CLIMB (no altitude sensor AND no balloon in view - the confirmed
 * PCB's situation whenever a target is out of the camera's field of view).
 * A small positive height error nudges the vehicle upward so a balloon at
 * the 2 m plane comes into the vertical field of view; once one is seen the
 * vision altitude error takes over. The nudge is deliberately tiny: through
 * the altitude gain it is only ~2 gf of net lift (~0.4 m/s terminal), and it
 * is capped per lost episode because without an altitude source nothing
 * else stops a climb - the ceiling guard needs altitude to fire. After the
 * cap the vehicle holds on its hover feed-forward alone. Starts only after a
 * short grace so a one-frame vision dropout does not trigger it. */
#define GUID_BLIND_CLIMB_ERR_M     0.015f
#define GUID_BLIND_CLIMB_AFTER_S   1.00f
#define GUID_BLIND_CLIMB_MAX_S     4.00f

/* PREPARE additionally requires the target to be roughly acquired before it
 * will release, but must not wait forever if perception never cooperates. */
#define GUID_PREPARE_ACQUIRE_TIMEOUT_S 12.0f

static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static float safe_f(float v)
{
    return isfinite(v) ? v : 0.0f;
}

/* Microsecond delta as seconds, robust to wrap and to a backwards clock.
 * uint32 microseconds wraps every ~71.6 minutes, which is far longer than a
 * match but well within a bench session, so it must not misbehave. */
static float elapsed_since(uint32_t now_us, uint32_t then_us)
{
    const uint32_t d = now_us - then_us;   /* defined unsigned wraparound */
    return (float)d * 1e-6f;
}

static void enter(blimp_guidance_t *g, blimp_guid_state_t s, uint32_t t_us)
{
    g->state = s;
    g->state_entry_us = t_us;
    g->confirm_timer_s = 0.0f;
    g->hint_reached = false;
}

/* Heading error toward the offboard search hint, or 0 if there is none. */
static float hint_error(const blimp_state_t *st, const blimp_target_obs_t *obs)
{
    if (!st || !obs || !obs->hint_valid) { return 0.0f; }
    if (!isfinite(obs->hint_yaw_rad) || !isfinite(st->yaw)) { return 0.0f; }
    float e = obs->hint_yaw_rad - st->yaw;
    while (e >  BLIMP_PI) { e -= BLIMP_TWO_PI; }
    while (e < -BLIMP_PI) { e += BLIMP_TWO_PI; }
    return e;
}

const char *blimp_guidance_state_name(blimp_guid_state_t s)
{
    switch (s) {
    case BLIMP_GUID_IDLE:      return "IDLE";
    case BLIMP_GUID_PREPARE:   return "PREPARE";
    case BLIMP_GUID_SEARCH:    return "SEARCH";
    case BLIMP_GUID_ALIGN:     return "ALIGN";
    case BLIMP_GUID_APPROACH:  return "APPROACH";
    case BLIMP_GUID_TOUCH:     return "TOUCH";
    case BLIMP_GUID_CONFIRM:   return "CONFIRM";
    case BLIMP_GUID_COAST:     return "COAST";
    case BLIMP_GUID_LOOP_DONE: return "LOOP_DONE";
    case BLIMP_GUID_FINISHED:  return "FINISHED";
    case BLIMP_GUID_FAILSAFE:  return "FAILSAFE";
    default:                   return "UNKNOWN";
    }
}

void blimp_guidance_init(blimp_guidance_t *g, int wp_count, int loop_count)
{
    if (!g) { return; }
    g->state            = BLIMP_GUID_IDLE;
    g->wp_index         = 0;
    g->wp_count         = (wp_count > 0) ? wp_count : 4;
    g->loop_index       = 0;
    g->loop_count       = (loop_count > 0) ? loop_count : 3;
    g->state_entry_us   = 0u;
    g->run_start_us     = 0u;
    g->last_seen_us     = 0u;
    g->confirm_timer_s  = 0.0f;
    g->search_dir       = 1.0f;
    g->miss_count       = 0;
    g->timing_active    = false;
    g->last_loop_time_s = 0.0f;
    g->best_loop_time_s = -1.0f;     /* negative = no valid loop yet */
    g->hint_reached     = false;
    g->blind_climb_s    = 0.0f;
    g->alt_err_known    = false;
    g->last_alt_err     = 0.0f;
}

void blimp_guidance_arm(blimp_guidance_t *g, uint32_t t_us)
{
    if (!g) { return; }
    /* Idempotent: re-arming mid-run must not restart the loop counter or
     * discard a time already recorded. */
    if (g->state != BLIMP_GUID_IDLE && g->state != BLIMP_GUID_FAILSAFE) {
        return;
    }
    g->wp_index        = 0;
    g->confirm_timer_s = 0.0f;
    g->miss_count      = 0;
    g->timing_active   = false;
    g->last_seen_us    = t_us;
    g->hint_reached    = false;
    g->blind_climb_s   = 0.0f;
    g->alt_err_known   = false;
    g->last_alt_err    = 0.0f;
    enter(g, BLIMP_GUID_PREPARE, t_us);
}

float blimp_guidance_elapsed_s(const blimp_guidance_t *g, uint32_t now_us)
{
    if (!g) { return 0.0f; }
    /* Frozen result once a loop has completed, so the score survives the
     * untimed reposition into the next loop. */
    if (!g->timing_active) {
        return (g->last_loop_time_s > 0.0f) ? g->last_loop_time_s : 0.0f;
    }
    return elapsed_since(now_us, g->run_start_us);
}

/* Altitude error toward the waypoint plane, in metres, positive = climb.
 *
 * Rule 22 puts every balloon centre at 2.0 m, so that is the cruise altitude
 * for the whole run. Two sources, in order of preference:
 *
 *   1. The estimator's fused altitude, when it has a real source (baro/ToF).
 *   2. The target balloon itself. The camera sees it at elevation `el` above
 *      the nose axis, at range `r`. Correct for the gondola's pitch (this
 *      codebase's +pitch is nose DOWN, so the world elevation is el - pitch),
 *      and the balloon centre sits r*sin(el_world) above the camera. The
 *      camera is cam_dz below the envelope centre (negative), so the balloon
 *      sits r*sin(el_world) + cam_dz above the point we want to hit. That is
 *      exactly the height error.
 *
 * Returns 0 when neither is available: hold whatever we have. */
static float altitude_error(const blimp_params_t *p,
                            const blimp_state_t *st,
                            const blimp_target_obs_t *obs,
                            bool visible)
{
    if (st && st->altitude_valid && isfinite(st->altitude)) {
        return clampf(BLIMP_RULE22_WP_HEIGHT_M - st->altitude,
                      -GUID_MAX_ALT_ERR_M, GUID_MAX_ALT_ERR_M);
    }
    if (visible && obs && p) {
        const float r        = clampf(safe_f(obs->range_m), 0.0f, 20.0f);
        const float pitch    = st ? safe_f(st->pitch) : 0.0f;
        const float el_world = safe_f(obs->elevation_rad) - pitch;
        const float cam_dz   = safe_f(p->cam_dz_m);
        const float err      = r * sinf(el_world) + cam_dz;
        return clampf(err, -GUID_MAX_ALT_ERR_M, GUID_MAX_ALT_ERR_M);
    }
    return 0.0f;
}

blimp_guid_state_t blimp_guidance_step(blimp_guidance_t *g,
                                       const blimp_params_t *p,
                                       const blimp_state_t *st,
                                       const blimp_target_obs_t *obs,
                                       bool contact_detected,
                                       float dt,
                                       blimp_wrench_t *demand)
{
    /* Default to a neutral demand on every path, including every early
     * return. blimp.c adds the hover feed-forward on top, so a zero demand
     * here means "hold, do not translate", which is the safe output. */
    if (demand) {
        demand->fx = 0.0f;
        demand->fy = 0.0f;
        demand->fz = 0.0f;
        demand->tz = 0.0f;
    }
    if (!g) { return BLIMP_GUID_IDLE; }
    if (!p) { return g->state; }
    if (!isfinite(dt) || dt <= 0.0f) { return g->state; }

    const uint32_t now = (obs && obs->t_us) ? obs->t_us
                       : (st ? st->t_us : g->state_entry_us);

    const bool visible = obs && obs->visible &&
                         isfinite(obs->bearing_rad) && isfinite(obs->range_m);
    if (visible) { g->last_seen_us = now; }

    const float bearing   = visible ? safe_f(obs->bearing_rad) : 0.0f;
    const float range     = visible ? clampf(safe_f(obs->range_m), 0.0f, 20.0f)
                                    : 0.0f;
    const float lost_s    = elapsed_since(now, g->last_seen_us);
    const float in_state  = elapsed_since(now, g->state_entry_us);
    const float alt_err   = altitude_error(p, st, obs, visible);
    const float touch_r   = (p->wp_touch_radius_m > 0.0f)
                                ? p->wp_touch_radius_m : 0.60f;
    const float align_tol = (p->wp_align_tol_rad > 0.0f)
                                ? p->wp_align_tol_rad : 0.17f;

    /* Altitude is held in every active state - the vehicle should never be
     * allowed to wander off the 2 m waypoint plane while it thinks. */
    if (demand) { demand->fz = alt_err; }

    /* Altitude with no fix at all (see GUID_BLIND_CLIMB_*). The estimator
     * remembers a fix for a few seconds; only once that window has expired
     * AND no balloon is in view does this run, and it is budgeted per lost
     * episode because nothing else bounds it:
     *   - never had a fix since arming (sitting on the floor at start):
     *     nudge upward so a balloon at the 2 m plane comes into view;
     *   - had one and it said we were ABOVE the plane: re-apply it (come
     *     down). A stale error saying "climb" is NOT re-applied: climbing
     *     on stale information is how a vehicle with no altitude sensor
     *     ends up on the ceiling, and a balloon below us comes back into
     *     view on its own as we drift while one above us does not. */
    const bool alt_known = st && st->altitude_valid;
    if (alt_known) {
        g->blind_climb_s = 0.0f;
        g->alt_err_known = true;
        g->last_alt_err  = alt_err;
    }
    const bool blind = !alt_known && !visible &&
                       lost_s > GUID_BLIND_CLIMB_AFTER_S &&
                       g->blind_climb_s < GUID_BLIND_CLIMB_MAX_S;
    const bool active_state = (g->state == BLIMP_GUID_PREPARE &&
                               in_state >= p->prepare_settle_s) ||
                              g->state == BLIMP_GUID_SEARCH ||
                              g->state == BLIMP_GUID_ALIGN ||
                              g->state == BLIMP_GUID_APPROACH ||
                              g->state == BLIMP_GUID_COAST;
    if (blind && active_state) {
        g->blind_climb_s += dt;
        if (demand) {
            if (!g->alt_err_known) {
                demand->fz = GUID_BLIND_CLIMB_ERR_M;
            } else if (g->last_alt_err < 0.0f) {
                demand->fz = clampf(g->last_alt_err, -0.30f, 0.0f);
            } else {
                demand->fz = 0.0f;
            }
        }
    }

    switch (g->state) {

    case BLIMP_GUID_IDLE:
    case BLIMP_GUID_FINISHED:
        /* Terminal / inactive: hold altitude, command nothing else. */
        if (demand) { demand->fz = alt_err; }
        break;

    case BLIMP_GUID_FAILSAFE:
        /* blimp.c parks the machine here and does NOT step it while the
         * failsafe condition (link loss, ceiling, estimator fault) holds;
         * being stepped at all means the condition has cleared. Resume the
         * run where it left off (wp_index is intact): Rule 7 forbids human
         * intervention, so recovery must be autonomous. A short settle
         * first so a marginal condition does not flap. */
        if (demand) { demand->fz = alt_err; }
        if (in_state >= 1.0f) {
            enter(g, visible ? BLIMP_GUID_ALIGN : BLIMP_GUID_SEARCH, now);
        }
        break;

    case BLIMP_GUID_PREPARE:
        /* UNTIMED (Rule 4). Settle, let the estimator converge, acquire and
         * roughly align on the first balloon. Time spent here is free, so the
         * only reason to leave is that we are genuinely ready. */
        if (demand) {
            demand->tz = visible ? bearing : 0.0f;
            demand->fx = 0.0f;
        }
        if (in_state >= p->prepare_settle_s) {
            if (visible && fabsf(bearing) < align_tol * 2.0f) {
                enter(g, BLIMP_GUID_ALIGN, now);
            } else if (in_state >= GUID_PREPARE_ACQUIRE_TIMEOUT_S) {
                /* Perception never cooperated. Go hunt rather than sit here:
                 * PREPARE being untimed is not a licence to wedge. */
                enter(g, BLIMP_GUID_SEARCH, now);
            }
        }
        break;

    case BLIMP_GUID_SEARCH: {
        /* Target not visible. Every turn costs surge on this airframe (no
         * yaw without thrust), so do not scan blindly if we know where to
         * look: turn to the offboard hint first (bounded rotation), and
         * only scan once we are there and still see nothing. With no hint,
         * scan; alternate direction each time we re-enter after a failure
         * so the arcs cancel instead of spiralling into a wall. */
        const float h_err = hint_error(st, obs);
        const bool  have_hint = obs && obs->hint_valid && st;
        if (have_hint && !g->hint_reached && fabsf(h_err) <= GUID_HINT_REACHED_RAD) {
            g->hint_reached = true;
        }
        if (demand) {
            demand->tz = (have_hint && !g->hint_reached)
                             ? h_err
                             : g->search_dir * GUID_SEARCH_ERR_RAD;
            demand->fx = 0.0f;
        }
        if (contact_detected) {
            /* Bumped into it without seeing it (it can sit above the
             * camera's field of view on the way up). A touch is a touch. */
            enter(g, BLIMP_GUID_CONFIRM, now);
        } else if (visible) {
            enter(g, BLIMP_GUID_ALIGN, now);
        } else if (in_state >= GUID_STATE_HARD_LIMIT_S) {
            g->search_dir = -g->search_dir;
            enter(g, BLIMP_GUID_SEARCH, now);
        }
        break;
    }

    case BLIMP_GUID_ALIGN:
        /* Rotate to face. Surge is deliberately zero: the mixer will add the
         * minimum needed to realise the yaw, and no more, so this is the
         * tightest arc the airframe can fly. Committing forward while still
         * far off-axis carves a wide arc and arrives off-axis, which is the
         * cross-track error we cannot strafe out of. */
        if (demand) {
            demand->tz = bearing;
            demand->fx = 0.0f;
        }
        if (contact_detected) {
            enter(g, BLIMP_GUID_CONFIRM, now);
        } else if (!visible && lost_s > GUID_LOST_GRACE_S) {
            enter(g, BLIMP_GUID_SEARCH, now);
        } else if (visible && fabsf(bearing) <= align_tol) {
            enter(g, BLIMP_GUID_APPROACH, now);
        } else if (in_state >= GUID_STATE_HARD_LIMIT_S) {
            g->search_dir = -g->search_dir;
            enter(g, BLIMP_GUID_SEARCH, now);
        }
        break;

    case BLIMP_GUID_APPROACH:
        /* Drive at the target and keep correcting heading. Cross-track
         * error shows up as bearing error and is corrected by steering. */
        if (demand) {
            demand->tz = visible ? bearing : 0.0f;
            demand->fx = visible
                ? clampf(range - touch_r, -GUID_MAX_RANGE_ERR_M,
                          GUID_MAX_RANGE_ERR_M)
                : 0.0f;
        }
        if (contact_detected) {
            /* Contact can legitimately arrive before the range estimate says
             * we are close - the balloon is large and range is crude. */
            enter(g, BLIMP_GUID_CONFIRM, now);
        } else if (visible && range <= touch_r) {
            enter(g, BLIMP_GUID_TOUCH, now);
        } else if (!visible && lost_s > GUID_LOST_GRACE_S) {
            enter(g, BLIMP_GUID_SEARCH, now);
        } else if (visible && fabsf(bearing) > align_tol * 3.0f) {
            enter(g, BLIMP_GUID_ALIGN, now);   /* drifted badly off-axis */
        } else if (in_state >= p->miss_timeout_s) {
            g->miss_count++;
            enter(g, BLIMP_GUID_COAST, now);
        }
        break;

    case BLIMP_GUID_TOUCH:
        /* Terminal commit. Stop steering on range and push gently through;
         * hunting at contact range is how touches get missed. */
        if (demand) {
            demand->tz = visible ? bearing * 0.5f : 0.0f;
            demand->fx = GUID_TOUCH_PUSH_M;
        }
        if (contact_detected) {
            enter(g, BLIMP_GUID_CONFIRM, now);
        } else if (in_state >= GUID_TOUCH_TIMEOUT_S) {
            g->miss_count++;
            enter(g, BLIMP_GUID_COAST, now);
        }
        break;

    case BLIMP_GUID_CONFIRM:
        /* Dwell before accepting the touch, so a single spurious contact
         * cannot advance a waypoint and void the run under Rule 7. */
        if (demand) {
            demand->fx = 0.0f;
            demand->tz = 0.0f;
        }
        if (contact_detected) {
            g->confirm_timer_s += dt;
        } else {
            /* Contact lost before the dwell completed: not a touch. */
            g->confirm_timer_s = 0.0f;
            enter(g, BLIMP_GUID_TOUCH, now);
            break;
        }

        if (g->confirm_timer_s >= p->touch_confirm_s) {
            /* ---- A WAYPOINT IS HEREBY SCORED ------------------------------
             * Rule 4: the clock starts on the FIRST touch of the loop. */
            if (g->wp_index == 0 && !g->timing_active) {
                g->run_start_us  = now;
                g->timing_active = true;
            }

            g->miss_count = 0;
            g->wp_index++;

            if (g->wp_index >= g->wp_count) {
                /* Rule 4: the clock stops on the FINAL waypoint. Freeze the
                 * result before anything untimed happens. */
                if (g->timing_active) {
                    g->last_loop_time_s = elapsed_since(now, g->run_start_us);
                    if (g->best_loop_time_s < 0.0f ||
                        g->last_loop_time_s < g->best_loop_time_s) {
                        g->best_loop_time_s = g->last_loop_time_s;
                    }
                    g->timing_active = false;
                }
                enter(g, BLIMP_GUID_LOOP_DONE, now);
            } else {
                enter(g, BLIMP_GUID_COAST, now);
            }
        }
        break;

    case BLIMP_GUID_COAST:
        /* Motors idle, heading held. Nothing on this airframe pushes
         * backwards, so the only way to stop pressing into the balloon we
         * just touched (Rule 16: do not disturb the tether) is to let drag
         * and the balloon's own push-back take the speed off before we
         * start the next turn. */
        if (demand) {
            demand->fx = 0.0f;
            demand->tz = 0.0f;
        }
        if (in_state >= GUID_COAST_S) {
            enter(g, visible ? BLIMP_GUID_ALIGN : BLIMP_GUID_SEARCH, now);
        }
        break;

    case BLIMP_GUID_LOOP_DONE:
        /* UNTIMED (Rule 5): the next loop's clock does not start until its
         * own first touch, so repositioning back to waypoint 1 is free. */
        g->loop_index++;
        g->wp_index = 0;
        if (g->loop_index >= g->loop_count) {
            enter(g, BLIMP_GUID_FINISHED, now);
        } else {
            enter(g, BLIMP_GUID_PREPARE, now);
        }
        break;

    default:
        /* Corrupt state value: recover deterministically rather than
         * executing undefined behaviour. */
        enter(g, BLIMP_GUID_FAILSAFE, now);
        break;
    }

    /* Final scrub: guidance must never hand a non-finite demand downstream,
     * whatever perception fed us. fy is pinned to 0 unconditionally. */
    if (demand) {
        demand->fx = safe_f(demand->fx);
        demand->fy = 0.0f;
        demand->fz = safe_f(demand->fz);
        demand->tz = safe_f(demand->tz);
    }
    return g->state;
}
