/*
 * Sparse block-matching optical flow. See flow_match.h for the rationale.
 *
 * Algorithm, per frame pair:
 *   for each block on a grid_x by grid_y grid:
 *     1. reject the block if its variance is too low (featureless floor gives
 *        an ambiguous match that looks confident)
 *     2. compute the full SAD surface over +/-search pixels
 *     3. find the minimum; reject it if it sits on the search boundary, since
 *        the true minimum is then outside the window and the value we have is
 *        not a minimum at all
 *     4. refine to sub-pixel by fitting a parabola through the minimum and its
 *        two neighbours on each axis
 *   take the median of the surviving vectors, and their median absolute
 *   deviation as a confidence measure.
 *
 * The SAD surface is computed WITHOUT an early-exit shortcut. Early exit is a
 * large speedup, but it returns a clamped value for every position that is not
 * the running best - and step 4 needs the TRUE SAD at the minimum's neighbours.
 * Clamped neighbours produce a silently wrong sub-pixel offset, which is far
 * worse than the cycles saved. Measured cost is affordable (see README).
 */
#include <math.h>
#include "flow_match.h"

/* ------------------------------------------------------------------ helpers */

static uint32_t block_variance(const uint8_t *img, int stride,
                               int x, int y, int block)
{
    uint32_t sum = 0, sumsq = 0;
    for (int r = 0; r < block; r++) {
        const uint8_t *p = img + (size_t)(y + r) * stride + x;
        for (int c = 0; c < block; c++) {
            uint32_t v = p[c];
            sum   += v;
            sumsq += v * v;
        }
    }
    /* var = (n*sumsq - sum^2) / n^2. For a 16x16 block of 8-bit pixels
       n*sumsq reaches 4.26e9, which only just fits in 32 bits, so go through
       64 rather than rely on that margin. */
    uint32_t n = (uint32_t)block * (uint32_t)block;
    uint64_t num = (uint64_t)n * (uint64_t)sumsq - (uint64_t)sum * (uint64_t)sum;
    return (uint32_t)(num / ((uint64_t)n * (uint64_t)n));
}

static uint32_t sad_block(const uint8_t *prev, const uint8_t *cur, int stride,
                          int px, int py, int cx, int cy, int block)
{
    uint32_t sum = 0;
    for (int r = 0; r < block; r++) {
        const uint8_t *a = prev + (size_t)(py + r) * stride + px;
        const uint8_t *b = cur  + (size_t)(cy + r) * stride + cx;
        for (int c = 0; c < block; c++) {
            int d = (int)a[c] - (int)b[c];
            sum += (uint32_t)(d < 0 ? -d : d);
        }
    }
    return sum;
}

/*
 * Sub-pixel refinement from the SAD minimum s0 and its two neighbours sm, sp.
 * Returns 0 when the three samples do not describe a minimum at all - flat or
 * saddle-shaped - because the offset would then be noise amplified by a
 * near-zero denominator. On success *delta is in pixels, positive toward +1.
 *
 * Which estimator is used matters more than anything else in this file. A
 * hovering airframe moves a small fraction of a pixel per frame, so the
 * integer part of the match is almost always zero and the integrated distance
 * is a sum of these refinements alone. A biased estimator is therefore
 * indistinguishable from a wrong lens FOV, except that it also varies with
 * speed. See host/bias_sweep for the measured transfer curves.
 */

/* Vertex of the parabola through (-1, sm), (0, s0), (+1, sp).
   Biased: a SAD surface near its minimum is closer to a V than to a parabola,
   and fitting a parabola to a V pulls the estimate toward the integer offset.
   For an ideal V with true offset d this returns d / (2 * (1 - d)), which is
   0.068 for d = 0.12 - a gain of 0.57, and the measured gain is 0.50. */
static int subpixel_parabola(uint32_t sm, uint32_t s0, uint32_t sp,
                             float *delta)
{
    int32_t denom = (int32_t)sm - 2 * (int32_t)s0 + (int32_t)sp;
    if (denom <= 0) {
        return 0;
    }
    *delta = 0.5f * (float)((int32_t)sm - (int32_t)sp) / (float)denom;
    return 1;
}

/* The same parabola, fitted to the logarithms of the three samples. This is
   the standard estimator for a correlation PEAK. A SAD minimum is the opposite
   case: s0 tends toward zero as the match gets better, log s0 diverges
   downward, the denominator grows without bound and the estimate collapses to
   zero. The +1 keeps the logarithm finite for an exact match. It is offered
   because it is the conventional answer, and measured so the comparison is on
   record rather than argued. */
static int subpixel_gaussian(uint32_t sm, uint32_t s0, uint32_t sp,
                             float *delta)
{
    float lm = logf((float)sm + 1.0f);
    float l0 = logf((float)s0 + 1.0f);
    float lp = logf((float)sp + 1.0f);
    float denom = lm - 2.0f * l0 + lp;
    if (!(denom > 0.0f)) {
        return 0;
    }
    *delta = 0.5f * (lm - lp) / denom;
    return 1;
}

/* Two straight lines: one through the minimum and whichever neighbour is
   higher, the other mirrored through the minimum. Their intersection is the
   estimate. For a cost surface of the form A*|k - d| this is EXACT at every d,
   which is the point: absolute differences produce exactly that shape, so the
   V is the correct model and the parabola is the approximation.

   delta = (sm - sp) / (2 * (max(sm, sp) - s0)). */
static int subpixel_vfit(uint32_t sm, uint32_t s0, uint32_t sp, float *delta)
{
    uint32_t hi = (sm > sp) ? sm : sp;
    if (hi <= s0) {
        return 0;
    }
    *delta = 0.5f * (float)((int32_t)sm - (int32_t)sp)
                  / (float)(hi - s0);
    return 1;
}

static int subpixel(int mode, uint32_t sm, uint32_t s0, uint32_t sp,
                    float *delta)
{
    switch (mode) {
    case FLOW_SUBPIX_GAUSSIAN: return subpixel_gaussian(sm, s0, sp, delta);
    case FLOW_SUBPIX_VFIT:     return subpixel_vfit(sm, s0, sp, delta);
    default:                   return subpixel_parabola(sm, s0, sp, delta);
    }
}

static void sort_floats(float *v, int n)
{
    /* n <= FLOW_MAX_BLOCKS (64); insertion sort is the right call at this size
       and keeps the file dependency-free. */
    for (int i = 1; i < n; i++) {
        float k = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > k) {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = k;
    }
}

static float median_sorted(const float *v, int n)
{
    return (n & 1) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

static float absf(float a) { return a < 0.0f ? -a : a; }

/* ---------------------------------------------------------------- public API */

int flow_cfg_valid(const flow_cfg_t *cfg)
{
    if (cfg == 0) {
        return 0;
    }
    if (cfg->block < 4 || cfg->search < 1 || cfg->search > FLOW_MAX_SEARCH) {
        return 0;
    }
    if (cfg->grid_x < 1 || cfg->grid_y < 1) {
        return 0;
    }
    if (cfg->grid_x * cfg->grid_y > FLOW_MAX_BLOCKS) {
        return 0;
    }
    if (cfg->subpix < FLOW_SUBPIX_PARABOLA || cfg->subpix > FLOW_SUBPIX_VFIT) {
        return 0;
    }
    /* A block must fit with a full search margin on every side. */
    if (cfg->width  < cfg->block + 2 * cfg->search ||
        cfg->height < cfg->block + 2 * cfg->search) {
        return 0;
    }
    return 1;
}

int flow_match(const flow_cfg_t *cfg, const uint8_t *prev, const uint8_t *cur,
               flow_result_t *out)
{
    out->dx = out->dy = 0.0f;
    out->n_valid = out->n_flat = out->n_edge = out->n_ambig = 0;
    out->spread = 0.0f;
    out->n_total = 0;

    if (!flow_cfg_valid(cfg) || prev == 0 || cur == 0) {
        return 0;
    }

    const int stride = cfg->width;
    const int s      = cfg->search;
    const int b      = cfg->block;
    const int span   = 2 * s + 1;

    /* Legal top-left corners for a block that keeps a full search margin. */
    const int x0 = s, x1 = cfg->width  - b - s;
    const int y0 = s, y1 = cfg->height - b - s;

    float vx[FLOW_MAX_BLOCKS], vy[FLOW_MAX_BLOCKS];
    uint32_t sad[(2 * FLOW_MAX_SEARCH + 1) * (2 * FLOW_MAX_SEARCH + 1)];
    int n = 0;

    for (int gy = 0; gy < cfg->grid_y; gy++) {
        int by = (cfg->grid_y == 1) ? (y0 + y1) / 2
                                    : y0 + (y1 - y0) * gy / (cfg->grid_y - 1);
        for (int gx = 0; gx < cfg->grid_x; gx++) {
            int bx = (cfg->grid_x == 1) ? (x0 + x1) / 2
                                        : x0 + (x1 - x0) * gx / (cfg->grid_x - 1);
            out->n_total++;

            if (block_variance(prev, stride, bx, by, b) < cfg->min_var) {
                out->n_flat++;
                continue;
            }

            uint32_t best = 0xFFFFFFFFu;
            int best_ox = 0, best_oy = 0;
            for (int oy = -s; oy <= s; oy++) {
                for (int ox = -s; ox <= s; ox++) {
                    uint32_t v = sad_block(prev, cur, stride, bx, by,
                                           bx + ox, by + oy, b);
                    sad[(oy + s) * span + (ox + s)] = v;
                    if (v < best) {
                        best = v;
                        best_ox = ox;
                        best_oy = oy;
                    }
                }
            }

            /* A minimum on the boundary means the real match is outside the
               search window. Reporting it would peg the flow at +/-search and
               quietly understate a fast motion - exactly the failure that makes
               an estimator diverge while looking healthy. */
            if (best_ox == -s || best_ox == s || best_oy == -s || best_oy == s) {
                out->n_edge++;
                continue;
            }

            int   ix = best_ox + s, iy = best_oy + s;
            float ddx, ddy;
            if (!subpixel(cfg->subpix,
                          sad[iy * span + (ix - 1)],
                          sad[iy * span + ix],
                          sad[iy * span + (ix + 1)], &ddx) ||
                !subpixel(cfg->subpix,
                          sad[(iy - 1) * span + ix],
                          sad[iy * span + ix],
                          sad[(iy + 1) * span + ix], &ddy)) {
                out->n_ambig++;
                continue;
            }

            vx[n] = (float)best_ox + ddx;
            vy[n] = (float)best_oy + ddy;
            n++;
        }
    }

    out->n_valid = n;
    /* Two blocks cannot outvote anything, so a median over fewer than three is
       not robust and we would rather report nothing than something fragile. */
    if (n < 3) {
        return 0;
    }

    sort_floats(vx, n);
    sort_floats(vy, n);
    out->dx = median_sorted(vx, n);
    out->dy = median_sorted(vy, n);

    /* Median absolute deviation, as a single scalar over both axes. This is the
       number that tells you whether the blocks agreed; n_valid only tells you
       how many had enough texture to try. */
    float dev[FLOW_MAX_BLOCKS];
    for (int i = 0; i < n; i++) {
        dev[i] = absf(vx[i] - out->dx);
    }
    sort_floats(dev, n);
    float madx = median_sorted(dev, n);
    for (int i = 0; i < n; i++) {
        dev[i] = absf(vy[i] - out->dy);
    }
    sort_floats(dev, n);
    float mady = median_sorted(dev, n);
    out->spread = 0.5f * (madx + mady);

    return 1;
}
