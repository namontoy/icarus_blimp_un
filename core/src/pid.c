/* pid.c - PID with conditional integration and filtered derivative.
 *
 * Deliberately small and boring. The interesting decisions are documented in
 * pid.h: integral gains start at zero, and anti-windup is conditional
 * integration plus a hard clamp rather than back-calculation, because on a
 * 100 gf budget a single windup event can eat an entire axis of authority
 * for several seconds.
 */
#include "blimp/pid.h"

#include <math.h>

void blimp_pid_reset(blimp_pid_t *s)
{
    if (!s) {
        return;
    }
    s->integral    = 0.0f;
    s->prev_error  = 0.0f;
    s->d_state     = 0.0f;
    s->initialised = false;
    s->saturated   = false;
    s->sat_dir     = 0;
}

float blimp_wrap_pi(float angle_rad)
{
    if (!isfinite(angle_rad)) {
        return 0.0f;
    }
    /* fmodf keeps this O(1) regardless of how far out of range the input is,
     * which matters if a heading reference is ever fed in unwrapped. */
    float a = fmodf(angle_rad + BLIMP_PI, 2.0f * BLIMP_PI);
    if (a < 0.0f) {
        a += 2.0f * BLIMP_PI;
    }
    return a - BLIMP_PI;
}

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

float blimp_pid_step_d(blimp_pid_t *s, const blimp_pid_params_t *p,
                       float error, float derivative, float dt)
{
    if (!s || !p) {
        return 0.0f;
    }
    if (!isfinite(dt) || dt <= 0.0f || !isfinite(error)) {
        return 0.0f;
    }
    if (!isfinite(derivative)) {
        derivative = 0.0f;
    }

    /* Low-pass the derivative. A first-order RC filter with cutoff fc has
     * alpha = dt / (dt + 1/(2*pi*fc)). d_lpf_hz <= 0 disables filtering. */
    float d_term = derivative;
    if (p->d_lpf_hz > 0.0f) {
        const float rc    = 1.0f / (2.0f * BLIMP_PI * p->d_lpf_hz);
        const float alpha = dt / (dt + rc);
        if (!s->initialised) {
            s->d_state = derivative;
        } else {
            s->d_state += alpha * (derivative - s->d_state);
        }
        d_term = s->d_state;
    }

    const float p_term = p->kp * error;

    /* Conditional integration: do not wind FURTHER INTO a saturation we are
     * already sitting in, but always allow the integral to unwind back out of
     * it. The test is the sign of the error against the DIRECTION the output
     * is clamped, using the previous tick's saturation state. */
    float i_candidate = s->integral;
    if (p->ki != 0.0f) {
        const bool pushing_further =
            (s->sat_dir > 0 && error > 0.0f) ||
            (s->sat_dir < 0 && error < 0.0f);
        if (!pushing_further) {
            i_candidate += p->ki * error * dt;
            i_candidate = clampf(i_candidate, p->i_min, p->i_max);
        }
    } else {
        i_candidate = 0.0f;   /* ki disabled: never carry stale integral */
    }

    const float raw = p_term + i_candidate + p->kd * d_term;
    const float out = clampf(raw, p->out_min, p->out_max);

    s->saturated   = (raw != out);
    s->sat_dir     = (raw > out) ? 1 : ((raw < out) ? -1 : 0);
    s->integral    = i_candidate;
    s->prev_error  = error;
    s->initialised = true;

    return out;
}

float blimp_pid_step(blimp_pid_t *s, const blimp_pid_params_t *p,
                     float error, float dt)
{
    if (!s || !p) {
        return 0.0f;
    }
    if (!isfinite(dt) || dt <= 0.0f || !isfinite(error)) {
        return 0.0f;
    }

    /* No derivative kick on the first sample after a reset. */
    const float derivative =
        s->initialised ? (error - s->prev_error) / dt : 0.0f;

    return blimp_pid_step_d(s, p, error, derivative, dt);
}
