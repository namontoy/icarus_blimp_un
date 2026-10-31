/*
 * cam_row.c - target_cam's CSV row (cam_row.h).
 *
 * Strict on purpose: exactly 25 numbers (v1) or 35 ending in 2 (v2), each
 * read column inside a plausible range. The UART is shared with the
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
    memset(out, 0, sizeof(*out));
    out->version  = version;
    out->cam_t_ms = (uint32_t)fmodf(t_ms, 4294967296.0f);
    out->detected = ok == 1.0f;
    out->cx_px    = cx;
    out->cy_px    = cy;
    out->az_deg   = az;
    out->el_deg   = el;
    out->area_px  = area;
    out->range_mm = f[10];
    return true;
}
