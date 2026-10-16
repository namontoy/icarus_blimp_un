/*
 * tc_pipeline - the detector as a stack of stages, one entry point.
 *
 *   stage 0   HSV colour test + blobs (blob_detect), always available
 *   stage 1   learned colour table in place of the HSV test        (A)
 *   stage 2   1 + a classifier on each blob's features            (B)
 *   stage 3   2 + a small int8 CNN on crops of the top blobs       (C)
 *
 * Why one function for all of it: the camera calls it, and so does the PC
 * viewer through the same source compiled as a shared library. If the stages
 * were wired together in target_main.c, the viewer would have to repeat that
 * wiring in Python and could quietly disagree with the camera about what a
 * stage does. Here there is only one copy.
 *
 * Fallback is by prefix. A stage needs every model below it, so the stage that
 * actually runs is min(requested, highest N whose models 1..N are all present).
 * A model that failed its checks at load is passed as NULL, so it is simply
 * absent here: no per-frame validation, and no way to run a stage whose model
 * is broken. The effective stage goes out in every CSV row, so a fallback is
 * always visible downstream rather than silent.
 *
 * Portable like blob_detect: no ESP-IDF, no heap. Time comes from a clock the
 * caller passes in (or NULL for no timing).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "blob_detect.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TC_STAGE_MAX    3

/* Validated models, one per learned stage; NULL = absent. Each becomes a
   concrete type in the phase that introduces its stage. */
typedef struct {
    const void *lut;    /* stage 1: colour table (Phase A2)                   */
    const void *clf;    /* stage 2: blob classifier (Phase B2)                */
    const void *cnn;    /* stage 3: int8 CNN (Phase C2)                       */
} tc_models_t;

typedef struct {
    blob_result_t r;            /* blobs that survived every stage that ran   */
    int     stage;              /* effective stage, 0..TC_STAGE_MAX           */
    float   conf[BLOB_MAX_REPORT];  /* per reported blob, 0..1; -1 = not
                                   computed (stage < 2) or empty slot         */
    int32_t t_px_us;            /* per-pixel colour test (HSV or table)       */
    int32_t t_b_us;             /* blob features + classifier                 */
    int32_t t_c_us;             /* crops + CNN                                */
    int32_t t_total_us;         /* the whole call                             */
} tc_result_t;

/* Highest stage the given models allow, 0..TC_STAGE_MAX. */
int tc_stage_available(const tc_models_t *m);

/* The stage that will run for a request: min(request, available). A request
   outside 0..TC_STAGE_MAX is clamped into it. */
int tc_stage_effective(int stage_req, const tc_models_t *m);

/*
 * Run the stages up to tc_stage_effective(stage_req, m) on one frame.
 * `work` is blob_detect's workspace (blob_workspace_bytes(cfg) bytes, 4-byte
 * aligned). `now_us` may be NULL, and then every time field is 0.
 * Returns out->r.found.
 */
int tc_pipeline_run(const blob_cfg_t *cfg, int stage_req, const tc_models_t *m,
                    const uint8_t *frame, void *work, blob_clock_fn now_us,
                    tc_result_t *out);

/* Standard CRC-32 (zlib.crc32, reflected, poly 0xEDB88320, init and xorout
   0xFFFFFFFF; "123456789" -> 0xCBF43926). The camera's link uses the ROM
   routine, which a PC does not have; model records are checked on both, so
   they get this portable one. Bitwise and slow, which is fine: it runs when a
   model is loaded, never per frame. */
uint32_t tc_crc32(const void *buf, size_t len);

#ifdef __cplusplus
}
#endif
