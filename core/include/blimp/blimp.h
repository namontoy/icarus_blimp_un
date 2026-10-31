/* blimp/blimp.h - umbrella header and the single top-level control entry point.
 *
 * THE POINT OF THIS FILE: blimp_step() is the entire flight controller. The
 * ESP32 firmware task calls it, and the MuJoCo simulation calls the SAME
 * compiled code through a DLL. Anything tuned in simulation therefore
 * transfers to the vehicle exactly, with no reimplementation drift between a
 * "sim controller" and a "real controller".
 *
 * Constraints on everything under core/:
 *   - freestanding C11 plus <math.h>; no ESP-IDF, FreeRTOS, MuJoCo or stdio
 *   - no dynamic allocation, no global mutable state, no blocking
 *   - deterministic: same inputs, same outputs, every time (replayable logs)
 *   - every tunable lives in blimp_params_t, never as a file-scope constant
 */
#ifndef BLIMP_H
#define BLIMP_H

#include "blimp/types.h"
#include "blimp/params.h"
#include "blimp/pid.h"
#include "blimp/mixer.h"
#include "blimp/estimator.h"
#include "blimp/guidance.h"

/* Everything the controller remembers between ticks. One instance per
 * vehicle; the sim may hold several. */
typedef struct {
    blimp_params_t    params;
    blimp_estimator_t est;
    blimp_guidance_t  guid;
    blimp_pid_t       pid_alt;
    blimp_pid_t       pid_yaw;
    blimp_pid_t       pid_surge;
    blimp_state_t     state;
    blimp_wrench_t    demand;
    blimp_actuators_t act;
    uint32_t          last_cmd_us;   /* last ground packet, for link timeout */
    uint32_t          last_step_us;
    bool              armed;
    /* The bench hold's target follow (blimp_hold_t.target_held): the
     * heading where the target was last seen, set on each new observation
     * (a new target.t_us) and held between them and after it is lost. */
    bool              follow_valid;
    float             follow_yaw_rad;
    uint32_t          follow_obs_us;
} blimp_t;

/* Bench hold: setpoints commanded from the ground in place of guidance, for
 * the closed-loop tests of docs/pid-tuning-plan.md (C1-C4), which step a
 * heading or an altitude that guidance would otherwise choose itself. The
 * loops, gains, limits, failsafes and mixer are the run's own; only the
 * source of the setpoints changes, so a gain that passes here is the gain
 * that flies.
 *
 * An axis that is not held outputs nothing, the hover feed-forward included:
 * a heading test with the gondola on a tether must not run the heave pair.
 * Its loop is reset, so taking it up later starts without a derivative kick.
 * A held altitude with no altitude source this tick also outputs nothing,
 * which lets the vehicle sink on its trim (the failsafe posture, blimp.c).
 *
 * target_held turns the heading toward the target in blimp_input_t.target
 * instead (the balloon-follow test on the rope rig): each NEW observation
 * (target.visible with a target.t_us not seen before) sets the heading to
 * the current yaw plus its bearing, and the yaw loop holds that heading until
 * the next one. A fixed heading between frames, not the frame's stale
 * bearing, so a turn under way is not over-driven while the camera catches
 * up; after the target is lost, the heading where it was last seen. Until
 * the first observation the yaw axis outputs nothing. It overrides
 * heading_held.
 *
 * All-zero (active = false, what `blimp_input_t in = {0}` gives) is a
 * normal run, unchanged. */
typedef struct {
    bool  active;          /* these setpoints replace guidance this tick     */
    bool  heading_held;    /* yaw loop on heading_rad; else tz = 0           */
    bool  altitude_held;   /* altitude loop on altitude_m; else fz = 0       */
    bool  target_held;     /* yaw loop toward in->target (above); core 0.3.0,
                            * in what was the padding byte                   */
    float heading_rad;     /* rad, +CCW from above, relative to the heading
                            * latched at arming (blimp_estimator_zero)       */
    float altitude_m;      /* m above the floor, as the estimator reports it */
    float surge_n;         /* N, a constant forward thrust, clipped to
                            * [0, surge.out_max]; 0 for the C tests          */
} blimp_hold_t;

/* Inputs to one control tick. */
typedef struct {
    blimp_imu_sample_t  imu;
    blimp_target_obs_t  target;
    bool                contact_detected;
    bool                arm_request;
    uint32_t            t_us;
    blimp_hold_t        hold;
} blimp_input_t;

/* Initialises everything, loads default params. Call once before any step. */
void blimp_init(blimp_t *b);

/* One full control tick: estimate, guide, control, mix.
 *
 * Writes the actuator command to *out. Never blocks, never allocates, and
 * always writes a safe (all-zero thrust) command on any internal fault, so a
 * caller that ignores the return value still fails safe. */
void blimp_step(blimp_t *b, const blimp_input_t *in, blimp_actuators_t *out);

/* Refreshes the ground-link watchdog. Call on every valid ground packet;
 * blimp_step() drops to failsafe if this goes stale for link_timeout_s.
 * NOTE: on this vehicle failsafe IS "all motors off" - and that is safe,
 * because a near-neutrally-buoyant airship trimmed a few gf heavy does not
 * fall, it sinks gently to the floor. Rule-legal and recoverable. */
void blimp_notify_link(blimp_t *b, uint32_t t_us);

/* Version stamp, logged with every flight so a log can be matched to the
 * code that produced it. */
const char *blimp_version(void);

#endif /* BLIMP_H */
