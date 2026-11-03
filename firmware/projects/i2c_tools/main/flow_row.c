/*
 * flow_row.c - optflow_cam's CSV row (flow_row.h).
 *
 * Strict for the same reason as cam_row.c: the UART also carries the
 * camera's boot ROM and log, and a line that half-parses is more likely one of
 * those than a row. Three checks come from the matcher itself
 * (firmware/projects/optflow_cam/main/flow_match.c): every block it tries is
 * counted exactly once (valid + flat + edge + ambig = total), a refused pair
 * prints dx = dy = 0, and no shift exceeds the search radius (at most
 * FLOW_MAX_SEARCH = 10 px).
 */
#include "flow_row.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define FLOW_ROW_FIELDS     16
#define FLOW_ROW_SHIFT_MAX  16.0f   /* px; the matcher's radius is at most 10 */
#define FLOW_ROW_BLOCKS_MAX 64      /* FLOW_MAX_BLOCKS                         */

static bool is_count(float v, float max)
{
    return v >= 0.0f && v <= max && v == floorf(v);
}

bool flow_row_parse(const char *line, flow_row_t *out)
{
    if (!line || !out) {
        return false;
    }
    float f[FLOW_ROW_FIELDS];
    int n = 0;
    const char *p = line;
    for (;;) {
        if (n == FLOW_ROW_FIELDS) {
            return false;               /* a longer line is not this row */
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
    if (n != FLOW_ROW_FIELDS) {
        return false;
    }

    const float t_ms = f[0], ok = f[1], dx = f[2], dy = f[3], spread = f[8];
    const float valid = f[9], total = f[10], flat = f[11], edge = f[12], ambig = f[13];
    const float dt_ms = f[14], proc_us = f[15];
    if (t_ms < 0.0f || (ok != 0.0f && ok != 1.0f) ||
        fabsf(dx) > FLOW_ROW_SHIFT_MAX || fabsf(dy) > FLOW_ROW_SHIFT_MAX ||
        spread < 0.0f || spread > FLOW_ROW_SHIFT_MAX ||
        !is_count(total, FLOW_ROW_BLOCKS_MAX) || total < 1.0f ||
        !is_count(valid, total) || !is_count(flat, total) ||
        !is_count(edge, total) || !is_count(ambig, total) ||
        valid + flat + edge + ambig != total ||
        dt_ms < 0.0f || dt_ms > 10000.0f || proc_us < 0.0f || proc_us > 1.0e7f) {
        return false;
    }
    /* The matcher measures only with three blocks or more, and a refused
       pair keeps the zeroed shift. */
    if ((ok == 1.0f && valid < 3.0f) || (ok == 0.0f && (dx != 0.0f || dy != 0.0f))) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    /* The clock is printed as an integer (%lld); a float is exact only to
       2^24 ms, so read it again as one (cam_row.c does the same). */
    char *tend = NULL;
    const unsigned long long t_exact = strtoull(line, &tend, 10);
    out->cam_t_ms  = (tend && *tend == ',') ? (uint32_t)t_exact : (uint32_t)fmodf(t_ms, 4294967296.0f);
    out->ok        = ok == 1.0f;
    out->dx_px     = dx;
    out->dy_px     = dy;
    out->spread_px = spread;
    out->valid     = (uint8_t)valid;
    out->total     = (uint8_t)total;
    out->flat      = (uint8_t)flat;
    out->edge      = (uint8_t)edge;
    out->ambig     = (uint8_t)ambig;
    out->dt_ms     = dt_ms;
    out->proc_us   = (uint32_t)proc_us;
    return true;
}
