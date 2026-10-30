/*
 * param_table.h - every field of core/'s blimp_params_t by name, with the
 * range the console accepts for it (docs/control-loop-plan.md D10: one flat
 * table {name, type, offset, min, max} over the same struct).
 *
 * Portable C with no ESP-IDF dependency, like optflow_cam's flow_match.c, so
 * tools/fw_host_test.py compiles it on the computer against core/src/params.c
 * and checks it: every field of the struct has exactly one entry, every
 * default sits inside its range, and a set never leaves a value that
 * blimp_params_validate() would not accept. cmd_param.c owns the live copy,
 * the lock and the console command; this file only knows the struct.
 *
 * Names are the C field paths ("alt.kp", "hover_thrust_n"), the same as the
 * simulator's ctypes mirror (sim/blimp_ffi.py), so a gain found in a sweep
 * is typed on the console under the name it had there.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "blimp/params.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PARAM_FLOAT,
    PARAM_BOOL,                 /* stored as bool, read and written as 0 or 1 */
} param_type_t;

typedef struct {
    const char  *name;          /* the C field path                            */
    size_t       offset;        /* offsetof(blimp_params_t, <name>)            */
    param_type_t type;
    float        min, max;      /* inclusive: what the console accepts         */
    const char  *unit;
} param_entry_t;

typedef enum {
    PARAM_OK = 0,               /* stored as given                             */
    PARAM_CLAMPED,              /* stored, but blimp_params_validate() changed
                                   this field or another one                   */
    PARAM_ERR_NUMBER,           /* not a number (a bool takes 0/1/true/false/on/off) */
    PARAM_ERR_RANGE,            /* not finite, or outside [min, max]           */
    PARAM_ERR_ORDER,            /* would leave a PID block's i_min above its
                                   i_max, or out_min above out_max             */
} param_result_t;

size_t param_count(void);
const param_entry_t *param_at(size_t i);

/** The entry with exactly this name, or NULL. */
const param_entry_t *param_find(const char *name);

/** The field's value; a bool reads as 0.0f or 1.0f. */
float param_get(const blimp_params_t *p, const param_entry_t *e);

/** Text to a value for this entry. Checks the syntax only, not the range. */
param_result_t param_parse(const param_entry_t *e, const char *text, float *value);

/**
 * Store one value. The range and the PID clamp order are checked first and
 * an error leaves *p untouched; then blimp_params_validate() runs over the
 * whole struct, as blimp_step() does every tick, and PARAM_CLAMPED says it
 * changed something (this field or a related one, e.g. hover_thrust_n when
 * heave_budget_n drops below it). *p is valid on every return.
 */
param_result_t param_set(blimp_params_t *p, const param_entry_t *e, float value);

/** One short phrase per result, for the console. */
const char *param_result_str(param_result_t r);

#ifdef __cplusplus
}
#endif
