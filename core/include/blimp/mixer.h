/* blimp/mixer.h - control allocation for four fixed, unidirectional motors.
 *
 * Maps a body wrench (fx, fz, tz) onto four non-negative motor thrusts:
 *
 *   front pair, both pushing +x:   T_fl + T_fr = fx
 *                                  l_f * (T_fr - T_fl) = tz
 *   rear pair, both pushing +z:    T_rl + T_rr = fz,  T_rl = T_rr
 *
 * (Right motor harder -> nose swings left -> +tz, CCW from above.)
 *
 * fy is ignored: nothing on this airframe can produce it (types.h).
 *
 * WHAT CANNOT BE DONE, and what the mixer does instead:
 *   - fx < 0 (brake/reverse): clipped to 0. Drag is the only brake.
 *   - fz < 0 (push down):     clipped to 0, clamped_heave set. The vehicle
 *                             descends on its own negative trim.
 *   - yaw with zero surge:    impossible - realising tz needs one motor
 *                             above zero. The mixer raises the front-pair
 *                             sum to the MINIMUM that realises tz exactly,
 *                             so a pure yaw demand becomes a tight arc.
 *
 * Saturation priority, applied in this order:
 *   1. Every motor >= 0.
 *   2. Heave first: the rear pair takes min(fz, heave_budget, 2*motor_max).
 *      Drifting off the 2 m waypoint plane is worse than arriving late.
 *   3. Yaw over surge on the front pair: the differential is realised
 *      exactly whenever the per-motor cap and the remaining Rule 13 budget
 *      allow it; surge takes what is left.
 *   4. The Rule 13 total (all four motors) is absolute and checked last.
 */
#ifndef BLIMP_MIXER_H
#define BLIMP_MIXER_H

#include "blimp/types.h"
#include "blimp/params.h"

/* Pure function: no state, no allocation. Safe to call from any context.
 *
 * Guarantees on return, for ANY finite or non-finite input and ANY params:
 *   every thrust_* >= 0 and <= p->motor_max_thrust_n (sanitised)
 *   thrust_fl + thrust_fr + thrust_rl + thrust_rr
 *       <= p->max_total_thrust_n (sanitised) + 1e-6f
 *   thrust_rl + thrust_rr <= p->heave_budget_n (sanitised) + 1e-6f
 *   every field finite (no NaN/Inf)
 *
 * The clamped_* flags report which limits bound this tick. Log them; do not
 * silently absorb them - unexplained residual yaw in flight usually traces
 * back to an unlogged yaw clamp. */
void blimp_mixer_apply(const blimp_params_t *p,
                       const blimp_wrench_t *demand,
                       blimp_actuators_t *out);

/* Inverse, for telemetry and sim cross-checks: what wrench do these actuator
 * commands actually produce? fy is always reported as 0. */
void blimp_mixer_forward(const blimp_params_t *p,
                         const blimp_actuators_t *act,
                         blimp_wrench_t *realised);

#endif /* BLIMP_MIXER_H */
