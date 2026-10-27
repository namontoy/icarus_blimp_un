/* blimp/guidance.h - the autonomy state machine for IAR-LARC waypoint racing.
 *
 * THE GAME (rulebook rules 2-7, 18-22):
 *   Touch four tethered red balloons in a specified order. Two courses
 *   (rectangle perimeter, figure-eight), up to three loops each, best valid
 *   time per course, lowest total wins. A missed waypoint or any human
 *   intervention VOIDS the entire run (Rule 7).
 *
 * TWO RULES THIS STATE MACHINE IS BUILT AROUND:
 *
 *   Rule 4 - "The timer starts when the airship touches the first waypoint
 *   and stops when the final waypoint is successfully touched." Everything
 *   before the first touch is UNTIMED. So PREPARE is deliberately unhurried:
 *   settle trim, converge gyro bias, zero the yaw reference, acquire the
 *   first balloon, align. Rushing it buys nothing and risks the run.
 *
 *   Rule 5 - up to three loops run back-to-back with no intervention. Each
 *   loop is timed from its own first touch, so the repositioning between
 *   loops is ALSO untimed. Re-enter PREPARE between loops.
 *
 * THE AIRFRAME SHAPES THE LAW (types.h): four fixed unidirectional motors.
 * No lateral force, no reverse, no push-down. So the guidance law is
 * turn-to-face-then-drive, cross-track error is corrected by re-aiming, and
 * there is no "back off" after a touch - the vehicle COASTs on drag instead.
 * ALIGN cannot pivot in place either: the mixer realises a yaw demand with
 * the minimum surge that keeps both front motors non-negative, so ALIGN is a
 * tight arc, not a spin. Every state here is written with that in mind.
 *
 * ALTITUDE WITHOUT AN ALTITUDE SENSOR: the confirmed PCB carries only an
 * MPU-6050. When the estimator has no altitude source, guidance derives the
 * height error from the target balloon itself - its elevation angle, range
 * and known 2.0 m centre height (Rule 22) - so the vehicle can still hold
 * the waypoint plane whenever a balloon is in view. See altitude_error() in
 * guidance.c and params.cam_dz_m.
 */
#ifndef BLIMP_GUIDANCE_H
#define BLIMP_GUIDANCE_H

#include "blimp/types.h"
#include "blimp/params.h"

typedef enum {
    BLIMP_GUID_IDLE = 0,   /* disarmed                                      */
    BLIMP_GUID_PREPARE,    /* UNTIMED: settle, converge, acquire, align     */
    BLIMP_GUID_SEARCH,     /* target not visible: yaw-scan to reacquire     */
    BLIMP_GUID_ALIGN,      /* target visible, rotating to face it           */
    BLIMP_GUID_APPROACH,   /* driving at the target                         */
    BLIMP_GUID_TOUCH,      /* terminal commit, expecting contact            */
    BLIMP_GUID_CONFIRM,    /* dwelling to confirm contact before advancing  */
    BLIMP_GUID_COAST,      /* motors idle after a touch; drag bleeds speed  */
    BLIMP_GUID_LOOP_DONE,  /* loop complete; untimed reposition to first wp */
    BLIMP_GUID_FINISHED,   /* all loops done                                */
    BLIMP_GUID_FAILSAFE    /* link loss, ceiling breach or estimator fault  */
} blimp_guid_state_t;

/* What perception hands to guidance each tick. Deliberately bearing+range,
 * not a fused (x, y) position - see estimator.h. */
typedef struct {
    float bearing_rad;   /* +CCW from nose to the target                    */
    float elevation_rad; /* + above the nose                                */
    float range_m;       /* estimated, may be crude (apparent size)         */
    float confidence;    /* 0..1                                            */
    float hint_yaw_rad;  /* SEARCH HINT: the absolute heading, in the SAME
                          * yaw frame the estimator reports (state.yaw,
                          * zeroed at arm), at which the current target is
                          * expected. Computed OFFBOARD (Rule 1 allows it)
                          * from the fixed course geometry, the announced
                          * order and the vehicle's telemetry; an absolute
                          * heading rather than a relative bearing so it
                          * does not go stale between packets while the
                          * vehicle turns. Used only when !visible.        */
    float alt_fix_m;     /* ALTITUDE FIX, metres above the floor, computed
                          * OFFBOARD from ANY balloon in the camera image -
                          * every waypoint balloon sits at 2.0 m (Rule 22),
                          * so each one is an altitude reference, not only
                          * the current target. blimp.c fuses this exactly
                          * like a ToF/baro sample when no such sensor is
                          * fitted. Prefer the nearest balloon (best
                          * angular precision). */
    bool  visible;
    bool  hint_valid;    /* false -> SEARCH falls back to a blind scan      */
    bool  alt_fix_valid; /* false -> no balloon usable for altitude         */
    uint32_t t_us;
} blimp_target_obs_t;

typedef struct {
    blimp_guid_state_t state;
    int      wp_index;          /* index into the run's waypoint order      */
    int      wp_count;
    int      loop_index;
    int      loop_count;
    uint32_t state_entry_us;
    uint32_t run_start_us;      /* set on the FIRST touch, per Rule 4       */
    uint32_t last_seen_us;
    float    confirm_timer_s;
    float    search_dir;        /* +1/-1, which way to scan                 */
    int      miss_count;        /* consecutive failed approaches on this wp */
    bool     timing_active;
    float    last_loop_time_s;  /* completed loop's time, frozen at finish   */
    float    best_loop_time_s;  /* best valid loop so far; <0 = none yet.
                                 * Rule 5 scores the BEST of up to three
                                 * loops, so the machine must retain it -
                                 * elapsed_s() alone cannot, since it is
                                 * relative to a run_start that gets reused. */
    bool     hint_reached;      /* SEARCH: turned to the hinted heading and
                                 * still saw nothing -> now scanning        */
    float    blind_climb_s;     /* seconds of blind altitude action used in
                                 * this lost episode (no altitude, no target)*/
    bool     alt_err_known;     /* an altitude fix has been seen since arm */
    float    last_alt_err;      /* the altitude error at the last valid fix */
} blimp_guidance_t;

void blimp_guidance_init(blimp_guidance_t *g, int wp_count, int loop_count);

/* Arms the machine into PREPARE. Safe to call repeatedly. */
void blimp_guidance_arm(blimp_guidance_t *g, uint32_t t_us);

/* One guidance tick. Produces the wrench demand for the control loops.
 *
 * contact_detected: true when the touch detector fires this tick. See
 * docs/05 section 4 - primary is vision proximity plus an IMU deceleration
 * signature; a timeout fallback exists so the vehicle never stalls forever
 * at one balloon.
 *
 * THE DEMAND CONVENTION (blimp.c depends on this exactly):
 *   demand.fx = RANGE error, metres   -> blimp.c runs it through pid_surge
 *   demand.fz = ALTITUDE error, metres-> blimp.c runs it through pid_alt
 *   demand.tz = HEADING error, radians-> blimp.c runs it through pid_yaw
 *   demand.fy = always 0 - no lateral actuator exists on this airframe.
 * The first three reuse blimp_wrench_t as a typed carrier for pre-PID errors
 * rather than as forces. That is deliberate but genuinely confusing, so it is
 * stated here as well as in guidance.c and blimp.c. Do not "fix" one side
 * without the other or control will be applied twice.
 *
 * Returns the state after the tick. */
blimp_guid_state_t blimp_guidance_step(blimp_guidance_t *g,
                                       const blimp_params_t *p,
                                       const blimp_state_t *st,
                                       const blimp_target_obs_t *obs,
                                       bool contact_detected,
                                       float dt,
                                       blimp_wrench_t *demand);

/* Elapsed time of the current loop as the judges would score it: measured
 * from the first touch (Rule 4). Returns 0 before timing starts. */
float blimp_guidance_elapsed_s(const blimp_guidance_t *g, uint32_t now_us);

const char *blimp_guidance_state_name(blimp_guid_state_t s);

#endif /* BLIMP_GUIDANCE_H */
