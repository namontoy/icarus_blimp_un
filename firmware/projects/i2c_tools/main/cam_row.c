/*
 * cam_row.c - target_cam's CSV row (cam_row.h).
 *
 * Strict on purpose: exactly 25 numbers (v1) or 35 ending in 2 (v2), each
 * kept column inside a plausible range. The UART is shared with the
 * camera's boot ROM, its log and, when its PC viewer asks, raw image frames;
 * a line that half-parses is far more likely one of those than a row, and a
 * row read wrong would turn the blimp. A format added later is rejected
 * (cam_link.c counts it) until this file learns it.
 */
#include "cam_row.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CAM_ROW_FIELDS_MAX  40
#define CAM_ROW_V1_FIELDS   25
#define CAM_ROW_V2_FIELDS   35

bool cam_row_parse(const char *line, cam_row_t *out)
{
    if (!line || !out) {
        return false;
    }
    float f[CAM_ROW_FIELDS_MAX];
    int n = 0;
    const char *p = line;
    for (;;) {
        if (n == CAM_ROW_FIELDS_MAX) {
            return false;
        }
        char *end = NULL;
        const float v = strtof(p, &end);
        if (end == p || !isfinite(v)) {
            return false;
        }
        f[n++] = v;
        if (*end == ',') {
            p = end + 1;
            continue;
        }
        if (*end == '\0') {
            break;
        }
        return false;
    }

    int version;
    if (n == CAM_ROW_V1_FIELDS) {
        version = 1;
    } else if (n == CAM_ROW_V2_FIELDS && f[CAM_ROW_V2_FIELDS - 1] == 2.0f) {
        version = 2;
    } else {
        return false;
    }

    /* The camera's frame is 160 x 120 today; the bounds leave room for a
       larger one and refuse garbage. */
    const float t_ms = f[0], ok = f[1], cx = f[2], cy = f[3], az = f[4], el = f[5], area = f[6];
    if (t_ms < 0.0f || (ok != 0.0f && ok != 1.0f) ||
        cx < 0.0f || cx > 2000.0f || cy < 0.0f || cy > 2000.0f ||
        fabsf(az) > 90.0f || fabsf(el) > 90.0f || area < 0.0f || area > 1.0e6f) {
        return false;
    }
    /* The columns the UDP link relays. The bounds are chosen so that anything
       target_cam's printf can print passes (an ok = 0 row prints range -1,
       blob slots 0,0,-1 and conf -1) and a shifted or corrupted column does
       not. */
    if (f[7] < 0.0f || f[7] > 2000.0f || f[8] < 0.0f || f[8] > 2000.0f ||    /* w, h     */
        f[9] < 0.0f || f[9] > 2.0f ||                                       /* fill     */
        (f[11] != 0.0f && f[11] != 1.0f) ||                                 /* clipped  */
        f[12] < 0.0f || f[12] > 10000.0f ||                                 /* blobs    */
        f[13] < 0.0f || f[13] > 1.0e6f ||                                   /* matched  */
        f[10] < -1.0f || f[10] > 1.0e7f ||                                  /* range    */
        f[23] < 0.0f || f[23] > 1.0e6f || f[24] < 0.0f || f[24] > 1.0e8f) { /* dt, proc */
        return false;
    }
    for (int k = 0; k < 3; k++) {
        const float baz = f[14 + 3 * k], bel = f[15 + 3 * k], brg = f[16 + 3 * k];
        if (fabsf(baz) > 90.0f || fabsf(bel) > 90.0f || brg < -1.0f || brg > 1.0e7f) {
            return false;
        }
    }
    if (version == 2) {
        /* 25 stage, 26 conf, 27-29 b2..b4 conf, 30-32 timings (not kept), 33 loop_us */
        if (f[25] < 0.0f || f[25] > 15.0f || f[26] < -1.0f || f[26] > 1.0f ||
            f[27] < -1.0f || f[27] > 1.0f || f[28] < -1.0f || f[28] > 1.0f ||
            f[29] < -1.0f || f[29] > 1.0f || f[33] < 0.0f || f[33] > 1.0e8f) {
            return false;
        }
    }

    memset(out, 0, sizeof(*out));
    out->version  = version;
    /* The camera's clock is printed as an integer (%lld) and a float is exact
       only to 2^24 ms = 4.6 h, so read the first column again as an integer. */
    char *tend = NULL;
    const unsigned long long t_exact = strtoull(line, &tend, 10);
    out->cam_t_ms = (tend && *tend == ',') ? (uint32_t)t_exact : (uint32_t)fmodf(t_ms, 4294967296.0f);
    out->detected = ok == 1.0f;
    out->cx_px    = cx;
    out->cy_px    = cy;
    out->az_deg   = az;
    out->el_deg   = el;
    out->area_px  = area;
    out->range_mm = f[10];
    out->w_px       = (uint16_t)f[7];
    out->h_px       = (uint16_t)f[8];
    out->fill       = f[9];
    out->clipped    = f[11] == 1.0f;
    out->blobs      = (uint16_t)f[12];
    out->matched_px = (uint32_t)f[13];
    for (int k = 0; k < 3; k++) {
        out->b_az_deg[k]    = f[14 + 3 * k];
        out->b_el_deg[k]    = f[15 + 3 * k];
        out->b_range_mm[k]  = f[16 + 3 * k];
        out->b_conf[k]      = -1.0f;
    }
    out->dt_ms   = f[23];
    out->proc_us = (uint32_t)f[24];
    out->conf    = -1.0f;
    if (version == 2) {
        out->stage   = (uint8_t)f[25];
        out->conf    = f[26];
        out->b_conf[0] = f[27];
        out->b_conf[1] = f[28];
        out->b_conf[2] = f[29];
        out->loop_us = (uint32_t)f[33];
    }
    return true;
}
