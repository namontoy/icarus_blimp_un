/* blimp/pid.h - PID with clamped integral and filtered derivative.
 *
 * Why clamp rather than back-calculate: the reference tuning runs both
 * integral gains at literally 0.0 (docs/03 section 3.3). With a 100 gf total
 * budget on a near-neutrally-buoyant vehicle, one windup event can consume an
 * entire axis of authority for many seconds. Start at ki = 0; add tightly
 * clamped integral only if bring-up shows a trim-resistant steady-state error.
 */
#ifndef BLIMP_PID_H
#define BLIMP_PID_H

#include "blimp/types.h"
#include "blimp/params.h"

typedef struct {
    float integral;
    float prev_error;
    float d_state;       /* filtered derivative state */
    bool  initialised;
    bool  saturated;     /* output hit a clamp on the last tick        */
    int   sat_dir;       /* +1 clamped at out_max, -1 at out_min, else 0.
                          * Conditional integration needs the DIRECTION of
                          * the saturation, not merely that it occurred. */
} blimp_pid_t;

void blimp_pid_reset(blimp_pid_t *s);

/* Standard form. dt in seconds, must be > 0; a non-positive or non-finite dt
 * returns 0 and leaves state untouched.
 *
 * Conditional integration: the integral does not accumulate while the output
 * is saturated in the same direction as the error. On the first call after a
 * reset the derivative term is forced to zero (no derivative kick). */
float blimp_pid_step(blimp_pid_t *s, const blimp_pid_params_t *p,
                     float error, float dt);

/* As above but with an externally supplied derivative - e.g. the gyro rate
 * for a heading loop. Avoids differentiating a stepped setpoint, which is the
 * usual source of derivative kick when guidance switches waypoints. */
float blimp_pid_step_d(blimp_pid_t *s, const blimp_pid_params_t *p,
                       float error, float derivative, float dt);

/* Wraps an angular error to (-pi, pi]. Use for EVERY heading error - a
 * heading loop that does not wrap will take the long way round at the
 * +/-180 degree boundary. */
float blimp_wrap_pi(float angle_rad);

#endif /* BLIMP_PID_H */
