/*
 * motor_ramp.c - the soft start's rules (motor_ramp.h). Portable: no ESP-IDF,
 * no allocation, no globals; cmd_motor.c holds one motor_ramp_t per output
 * and calls this under its own lock.
 */
#include "motor_ramp.h"

#include <math.h>

static int8_t sign_of(float v)
{
    return (v > 0.0f) ? 1 : ((v < 0.0f) ? -1 : 0);
}

void motor_ramp_init(motor_ramp_t *r, int64_t now_us)
{
    r->target = 0.0f;
    r->applied = 0.0f;
    r->driving = false;
    r->dir = 0;
    r->stopped_us = now_us;
    r->last_us = now_us;
}

void motor_ramp_stop(motor_ramp_t *r, int64_t now_us)
{
    if (r->driving) {
        r->stopped_us = now_us;
    }
    r->driving = false;
    r->applied = 0.0f;
    r->target = 0.0f;
    r->last_us = now_us;
}

float motor_ramp_set(motor_ramp_t *r, float target, int64_t now_us, const motor_ramp_cfg_t *c)
{
    r->target = isfinite(target) ? target : 0.0f;
    return motor_ramp_step(r, now_us, c);
}

float motor_ramp_step(motor_ramp_t *r, int64_t now_us, const motor_ramp_cfg_t *c)
{
    float dt = (float)(now_us - r->last_us) * 1e-6f;
    if (!(dt > 0.0f)) {
        dt = 0.0f;          /* a clock that stood still or went back: no time passed */
    }
    r->last_us = now_us;

    const float target = r->target;
    const int8_t s = sign_of(target);
    if (s == 0) {
        /* A stop is immediate, whatever the ramp. */
        if (r->driving) {
            r->stopped_us = now_us;
        }
        r->driving = false;
        r->applied = 0.0f;
        return 0.0f;
    }

    const float rate = (c->ramp_s > 0.0f) ? (c->ceiling - c->deadband) / c->ramp_s : 0.0f;
    if (!(rate > 0.0f)) {
        /* Ramp off (or no span to ramp over): the command lands as given, as
           before the soft start existed. */
        r->driving = true;
        r->dir = s;
        r->applied = target;
        return target;
    }

    if (r->driving && r->dir != s) {
        /* Asked the other way while still driven: coast first, and count the
           coast from now. */
        r->driving = false;
        r->applied = 0.0f;
        r->stopped_us = now_us;
        return 0.0f;
    }

    const float mag = fabsf(target);
    if (!r->driving) {
        if (r->dir != 0 && r->dir != s && now_us - r->stopped_us < (int64_t)(c->ramp_s * 1e6f)) {
            return 0.0f;    /* still coasting down before turning the other way */
        }
        /* A start: from the deadband (or the target, if lower), at once. */
        r->driving = true;
        r->dir = s;
        r->applied = (float)s * ((c->deadband < mag) ? c->deadband : mag);
        return r->applied;
    }

    /* Driving the same way: down at once, up no faster than the rate. */
    const float now_mag = fabsf(r->applied);
    if (mag <= now_mag) {
        r->applied = target;
        return target;
    }
    float next = now_mag + rate * dt;
    if (next > mag) {
        next = mag;
    }
    r->applied = (float)s * next;
    return r->applied;
}

bool motor_ramp_busy(const motor_ramp_t *r)
{
    return r->driving ? (r->applied != r->target) : (r->target != 0.0f);
}
