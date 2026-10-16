/*
 * tc_pipeline - see tc_pipeline.h for what and why.
 *
 * Phase 0: only stage 0 exists. The learned stages are absent from every
 * tc_models_t, so every request falls back to 0 and the rows say so.
 */
#include "tc_pipeline.h"

int tc_stage_available(const tc_models_t *m)
{
    if (m == NULL || m->lut == NULL) return 0;
    if (m->clf == NULL) return 1;
    if (m->cnn == NULL) return 2;
    return 3;
}

int tc_stage_effective(int stage_req, const tc_models_t *m)
{
    if (stage_req < 0) stage_req = 0;
    if (stage_req > TC_STAGE_MAX) stage_req = TC_STAGE_MAX;
    int avail = tc_stage_available(m);
    return stage_req < avail ? stage_req : avail;
}

int tc_pipeline_run(const blob_cfg_t *cfg, int stage_req, const tc_models_t *m,
                    const uint8_t *frame, void *work, blob_clock_fn now_us,
                    tc_result_t *out)
{
    int64_t t0 = now_us ? now_us() : 0;

    out->stage = tc_stage_effective(stage_req, m);
    out->t_b_us = 0;
    out->t_c_us = 0;
    for (int k = 0; k < BLOB_MAX_REPORT; k++) out->conf[k] = -1.0f;

    blob_detect_timed(cfg, frame, work, &out->r, now_us, &out->t_px_us);

    out->t_total_us = now_us ? (int32_t)(now_us() - t0) : 0;
    return out->r.found;
}

uint32_t tc_crc32(const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        c ^= p[i];
        for (int b = 0; b < 8; b++)
            c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return c ^ 0xFFFFFFFFu;
}
