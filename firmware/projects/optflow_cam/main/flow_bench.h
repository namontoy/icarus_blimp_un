/*
 * Frame-size benchmark for flow_match: how long one match takes, and how
 * precise it is, at each candidate frame size and matcher setting.
 *
 * Portable C like flow_match.c, so the same table runs on the host
 * (host/size_bench.c) and on the camera board (CONFIG_OPTFLOW_BENCH), where
 * the times are the ones that matter. The host's times only rank the rows; the
 * board's decide whether a row keeps up with an 80 ms frame period.
 *
 * Every candidate is the same floor seen at the same height through the same
 * lens: the synthetic texture is defined in floor millimetres, not in pixels,
 * so a bigger frame sees the same patterns with more pixels across them. Each
 * trial shifts the floor by a known sub-pixel amount, adds sensor noise to
 * both frames, and scores the matcher's answer in millimetres - the unit the
 * blimp cares about - so the precision gained by a bigger frame is measured,
 * not assumed.
 */
#pragma once

#include <stdint.h>
#include "flow_match.h"

typedef struct {
    const char *name;   /* frame size, as esp32-camera calls it            */
    int width, height;
    int block, search, grid;
} flow_bench_case_t;

/* The rows benchmarked, in order. Ends with a zero width. */
extern const flow_bench_case_t flow_bench_cases[];

typedef struct {
    float height_mm;    /* camera above the floor                          */
    float hfov_deg;     /* lens horizontal field of view                   */
    float period_ms;    /* frame period: sets the velocity range           */
    float speed_mm_s;   /* floor speed of the precision trials             */
    float noise;        /* sensor noise, counts (1 sigma, roughly)         */
    int   trials;       /* frame pairs per case                            */
} flow_bench_scene_t;

typedef struct {
    float mm_per_px;
    float range_mm_s;   /* (search - 1) px per frame, in floor mm/s        */
    float patch_mm;     /* floor covered by one block's edge               */
    float mops;         /* absolute differences per match, millions        */
    float ms_min, ms_mean, ms_max;
    float err_rms_mm;   /* per-frame displacement error, mm, both axes     */
    float err_rms_px;
    float err_mean_mm;  /* mean error along the motion, mm: the scale bias  */
    float valid_mean;   /* blocks matched per trial; flat ones cost no SAD  */
    int   n_ok;         /* trials that returned a measurement              */
} flow_bench_result_t;

/* Fill prev and cur (width*height bytes each) for one trial: the floor
   shifted by (sx_mm, sy_mm) between them. seed selects the floor patch and
   the noise. */
void flow_bench_frames(uint8_t *prev, uint8_t *cur, int width, int height,
                       float mm_per_px, float sx_mm, float sy_mm,
                       float noise, uint32_t seed);

/* The matcher settings and the floor scale of a case. */
void  flow_bench_cfg(const flow_bench_case_t *c, flow_cfg_t *cfg);
float flow_bench_mm_per_px(const flow_bench_case_t *c, const flow_bench_scene_t *scene);

/* Run one case. prev and cur must each hold width*height bytes; now_us is
   the platform's microsecond clock. Returns 0 if the case is invalid for
   flow_match (e.g. search above FLOW_MAX_SEARCH). */
int flow_bench_run(const flow_bench_case_t *c, const flow_bench_scene_t *scene,
                   uint8_t *prev, uint8_t *cur, int64_t (*now_us)(void),
                   flow_bench_result_t *out);
