/*
 * param_table.c - the table behind the 'param' console command; see
 * param_table.h.
 *
 * The ranges catch a typo, a slipped sign or a wrong unit, not a bad tune:
 * each is several times wider than anything this vehicle's physics makes
 * useful, so tuning never runs into one. What does bound the vehicle is
 * blimp_params_validate() in core/, which param_set() runs after every
 * write, exactly as blimp_step() runs it every tick (Rule 13's 100 gf, Rule
 * 11's 10 gf of trim). Widen a range here, with the reason, if a real tune
 * needs it.
 *
 * Gains are never negative: a negative gain is positive feedback, and the
 * loops' signs are fixed in core/ (blimp.c), not here.
 *
 * The scale of each loop, for reading the ranges:
 *   altitude: metres to newtons. The rear pair can push at most 60 % of the
 *     100 gf budget (0.59 N, blimp_params_validate()), so a kp of 20 N/m
 *     saturates it at 3 cm of error. The default is 1.34 (params.c).
 *   yaw: radians to newton-metres. The largest torque is one motor's thrust
 *     on the front half-span, about 0.34 N x 0.20 m = 0.07 N m; a kp of
 *     2 N m/rad saturates it within 2 degrees. The default is 0.033.
 *   surge: metres of range to newtons, one-way. The default kp is 0.30.
 *
 * d_lpf_hz stops at 25 Hz, the Nyquist frequency of the 50 Hz loop
 * (CTL_PERIOD_US); 0 switches the derivative filter off (pid.h).
 */
#include "param_table.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define RULE13_N    BLIMP_RULE13_MAX_TOTAL_N
#define RULE11_N    BLIMP_RULE11_TRIM_TOL_N
#define HEAVE_CAP_N (BLIMP_RULE13_MAX_TOTAL_N * 0.6f)   /* blimp_params_validate()'s */
#define TORQUE_N_M  0.5f        /* 100 gf on the longest half-span below      */
#define NYQUIST_HZ  25.0f

#define F(field, lo, hi, unit_) \
    { #field, offsetof(blimp_params_t, field), PARAM_FLOAT, (lo), (hi), (unit_) }
#define B(field) \
    { #field, offsetof(blimp_params_t, field), PARAM_BOOL, 0.0f, 1.0f, "0 or 1" }

/* In the struct's order (core/include/blimp/params.h), so 'param' lists them
   the way the header reads. */
static const param_entry_t TABLE[] = {
    /* geometry */
    F(front_half_span_m,     0.01f,     0.50f,      "m"),
    F(cam_dz_m,             -1.50f,     1.50f,      "m"),

    /* thrust budget: the upper ends are the validator's own limits */
    F(max_total_thrust_n,    0.01f,     RULE13_N,   "N"),
    F(motor_max_thrust_n,    0.01f,     RULE13_N,   "N"),
    F(heave_budget_n,        0.0f,      HEAVE_CAP_N, "N"),
    F(hover_thrust_n,        0.0f,      RULE11_N,   "N"),

    /* altitude loop: m -> N */
    F(alt.kp,                0.0f,      20.0f,      "N/m"),
    F(alt.ki,                0.0f,      10.0f,      "N/(m s)"),
    F(alt.kd,                0.0f,      20.0f,      "N s/m"),
    F(alt.i_min,            -RULE13_N,  RULE13_N,   "N"),
    F(alt.i_max,            -RULE13_N,  RULE13_N,   "N"),
    F(alt.out_min,          -RULE13_N,  RULE13_N,   "N"),
    F(alt.out_max,          -RULE13_N,  RULE13_N,   "N"),
    F(alt.d_lpf_hz,          0.0f,      NYQUIST_HZ, "Hz"),

    /* yaw loop: rad -> N m (kd acts on the gyro rate, blimp.c) */
    F(yaw.kp,                0.0f,      2.0f,       "N m/rad"),
    F(yaw.ki,                0.0f,      2.0f,       "N m/(rad s)"),
    F(yaw.kd,                0.0f,      2.0f,       "N m s/rad"),
    F(yaw.i_min,            -TORQUE_N_M, TORQUE_N_M, "N m"),
    F(yaw.i_max,            -TORQUE_N_M, TORQUE_N_M, "N m"),
    F(yaw.out_min,          -TORQUE_N_M, TORQUE_N_M, "N m"),
    F(yaw.out_max,          -TORQUE_N_M, TORQUE_N_M, "N m"),
    F(yaw.d_lpf_hz,          0.0f,      NYQUIST_HZ, "Hz"),

    /* surge loop: m of range -> N */
    F(surge.kp,              0.0f,      10.0f,      "N/m"),
    F(surge.ki,              0.0f,      10.0f,      "N/(m s)"),
    F(surge.kd,              0.0f,      10.0f,      "N s/m"),
    F(surge.i_min,          -RULE13_N,  RULE13_N,   "N"),
    F(surge.i_max,          -RULE13_N,  RULE13_N,   "N"),
    F(surge.out_min,        -RULE13_N,  RULE13_N,   "N"),
    F(surge.out_max,        -RULE13_N,  RULE13_N,   "N"),
    F(surge.d_lpf_hz,        0.0f,      NYQUIST_HZ, "Hz"),

    F(pitch_damp_kd,         0.0f,      1.0f,       "N m s/rad"),

    /* estimator */
    F(comp_alpha_attitude,   0.0f,      1.0f,       "1"),
    F(comp_alpha_alt,        0.0f,      1.0f,       "1"),
    F(mag_declination_rad,  -3.1416f,   3.1416f,    "rad"),
    B(use_mag),
    F(gyro_bias_tau_s,       0.5f,      600.0f,     "s"),

    /* guidance */
    F(wp_touch_radius_m,     0.05f,     2.0f,       "m"),
    F(wp_approach_speed_ms,  0.0f,      2.0f,       "m/s"),
    F(wp_align_tol_rad,      0.0f,      1.6f,       "rad"),
    F(touch_confirm_s,       0.0f,      5.0f,       "s"),
    F(search_yaw_rate,      -2.0f,      2.0f,       "rad/s"),
    F(miss_timeout_s,        0.5f,      120.0f,     "s"),
    F(prepare_settle_s,      0.0f,      120.0f,     "s"),

    /* failsafe */
    F(link_timeout_s,        0.1f,      10.0f,      "s"),
    F(max_altitude_m,        0.5f,      6.0f,       "m"),
};

#define TABLE_LEN (sizeof(TABLE) / sizeof(TABLE[0]))

size_t param_count(void)
{
    return TABLE_LEN;
}

const param_entry_t *param_at(size_t i)
{
    return i < TABLE_LEN ? &TABLE[i] : NULL;
}

const param_entry_t *param_find(const char *name)
{
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < TABLE_LEN; i++) {
        if (strcmp(TABLE[i].name, name) == 0) {
            return &TABLE[i];
        }
    }
    return NULL;
}

/* The field itself. The object at that offset is a float or a bool, so
   reading it through a pointer of its own type is well defined. */
static float *float_at(blimp_params_t *p, const param_entry_t *e)
{
    return (float *)(void *)((char *)p + e->offset);
}

static bool *bool_at(blimp_params_t *p, const param_entry_t *e)
{
    return (bool *)(void *)((char *)p + e->offset);
}

float param_get(const blimp_params_t *p, const param_entry_t *e)
{
    const void *field = (const char *)p + e->offset;
    if (e->type == PARAM_BOOL) {
        return *(const bool *)field ? 1.0f : 0.0f;
    }
    return *(const float *)field;
}

param_result_t param_parse(const param_entry_t *e, const char *text, float *value)
{
    if (text == NULL || *text == '\0') {
        return PARAM_ERR_NUMBER;
    }
    if (e->type == PARAM_BOOL) {
        if (strcmp(text, "1") == 0 || strcmp(text, "true") == 0 || strcmp(text, "on") == 0) {
            *value = 1.0f;
            return PARAM_OK;
        }
        if (strcmp(text, "0") == 0 || strcmp(text, "false") == 0 || strcmp(text, "off") == 0) {
            *value = 0.0f;
            return PARAM_OK;
        }
        return PARAM_ERR_NUMBER;
    }
    char *end = NULL;
    const float v = strtof(text, &end);
    if (end == text || *end != '\0') {
        return PARAM_ERR_NUMBER;
    }
    *value = v;
    return PARAM_OK;
}

static bool pid_ordered(const blimp_pid_params_t *g)
{
    return g->i_min <= g->i_max && g->out_min <= g->out_max;
}

param_result_t param_set(blimp_params_t *p, const param_entry_t *e, float value)
{
    /* !(a <= b) rather than a > b, so that a NaN fails too. */
    if (!isfinite(value) || !(value >= e->min) || !(value <= e->max)) {
        return PARAM_ERR_RANGE;
    }

    blimp_params_t t = *p;
    if (e->type == PARAM_BOOL) {
        *bool_at(&t, e) = value >= 0.5f;
    } else {
        *float_at(&t, e) = value;
    }

    /* The PID clamps are not blimp_params_validate()'s business today; an
       inverted pair would make blimp_pid_step() clamp to the wrong end. To
       move a pair past each other, set the one that widens it first. */
    if (!pid_ordered(&t.alt) || !pid_ordered(&t.yaw) || !pid_ordered(&t.surge)) {
        return PARAM_ERR_ORDER;
    }

    const bool clean = blimp_params_validate(&t);
    *p = t;
    return clean ? PARAM_OK : PARAM_CLAMPED;
}

const char *param_result_str(param_result_t r)
{
    switch (r) {
    case PARAM_OK:         return "set";
    case PARAM_CLAMPED:    return "set, then clamped by blimp_params_validate()";
    case PARAM_ERR_NUMBER: return "not a number";
    case PARAM_ERR_RANGE:  return "outside the accepted range";
    case PARAM_ERR_ORDER:  return "would put a PID clamp's minimum above its maximum";
    }
    return "?";
}
