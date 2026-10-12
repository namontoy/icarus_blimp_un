/*
 * Host-side verification of flow_match, with no camera and no ESP32.
 *
 * The trick that makes this possible: flow_match.c is portable C with no
 * ESP-IDF dependency, so the exact code that will run on the camera compiles
 * here too. We synthesise a textured image, shift it by a KNOWN amount with
 * bilinear interpolation, and check that the matcher recovers that shift.
 *
 * Texture choice matters. Pure sinusoids are periodic, and a periodic image has
 * genuinely ambiguous matches - the matcher would be blamed for an error that
 * is really in the test. Two octaves of bilinearly-upsampled value noise gives
 * an aperiodic but band-limited field, which is both distinctive enough to
 * match and smooth enough that bilinear sub-pixel shifting is meaningful.
 *
 *   cc -O2 -Wall -Wextra -o test_flow_match test_flow_match.c ../main/flow_match.c -lm
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../main/flow_match.h"

#define W 160
#define H 120

/* Deterministic LCG, so a failure is always reproducible. */
static uint32_t rng_state = 12345u;
static float rnd(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return (float)((rng_state >> 8) & 0xFFFFFF) / (float)0xFFFFFF;
}

/* One octave of value noise: a random lattice, bilinearly interpolated. */
static void add_octave(float *img, int w, int h, int cell, float amp)
{
    int gw = w / cell + 2, gh = h / cell + 2;
    float *g = malloc((size_t)gw * gh * sizeof(float));
    for (int i = 0; i < gw * gh; i++) {
        g[i] = rnd();
    }
    for (int y = 0; y < h; y++) {
        float fy = (float)y / cell;
        int   iy = (int)fy;
        float ty = fy - iy;
        for (int x = 0; x < w; x++) {
            float fx = (float)x / cell;
            int   ix = (int)fx;
            float tx = fx - ix;
            float a = g[iy * gw + ix],       b = g[iy * gw + ix + 1];
            float c = g[(iy + 1) * gw + ix], d = g[(iy + 1) * gw + ix + 1];
            float top = a + (b - a) * tx, bot = c + (d - c) * tx;
            img[y * w + x] += amp * (top + (bot - top) * ty);
        }
    }
    free(g);
}

static void make_texture(uint8_t *dst, int w, int h)
{
    float *f = calloc((size_t)w * h, sizeof(float));
    add_octave(f, w, h, 11, 1.00f);
    add_octave(f, w, h, 5,  0.45f);
    float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < w * h; i++) {
        if (f[i] < lo) lo = f[i];
        if (f[i] > hi) hi = f[i];
    }
    for (int i = 0; i < w * h; i++) {
        float v = 20.0f + 215.0f * (f[i] - lo) / (hi - lo);
        dst[i] = (uint8_t)(v + 0.5f);
    }
    free(f);
}

static float sample_bilinear(const uint8_t *img, int w, int h, float x, float y)
{
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x > w - 2) x = (float)(w - 2);
    if (y > h - 2) y = (float)(h - 2);
    int ix = (int)x, iy = (int)y;
    float tx = x - ix, ty = y - iy;
    float a = img[iy * w + ix],       b = img[iy * w + ix + 1];
    float c = img[(iy + 1) * w + ix], d = img[(iy + 1) * w + ix + 1];
    float top = a + (b - a) * tx, bot = c + (d - c) * tx;
    return top + (bot - top) * ty;
}

/* cur(x,y) = prev(x - sx, y - sy): content moves by (+sx, +sy). */
static void shift_image(const uint8_t *src, uint8_t *dst, int w, int h,
                        float sx, float sy)
{
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float v = sample_bilinear(src, w, h, (float)x - sx, (float)y - sy);
            dst[y * w + x] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v + 0.5f));
        }
    }
}

static const flow_cfg_t CFG = {
    .width = W, .height = H, .block = 16, .search = 7,
    .grid_x = 5, .grid_y = 5, .min_var = 50,
};

static int failures = 0;
static float worst_err = 0.0f;
static double err_sum = 0.0;
static int err_n = 0;

static void expect_shift(const uint8_t *base, float sx, float sy, float tol)
{
    uint8_t *cur = malloc(W * H);
    shift_image(base, cur, W, H, sx, sy);

    flow_result_t r;
    int ok = flow_match(&CFG, base, cur, &r);

    float ex = ok ? r.dx - sx : NAN;
    float ey = ok ? r.dy - sy : NAN;
    float e  = ok ? fmaxf(fabsf(ex), fabsf(ey)) : NAN;
    int pass = ok && e <= tol;

    printf("  shift (%+6.2f,%+6.2f) -> ", sx, sy);
    if (!ok) {
        printf("NO MEASUREMENT (valid %d/%d flat %d edge %d ambig %d)   %s\n",
               r.n_valid, r.n_total, r.n_flat, r.n_edge, r.n_ambig,
               "FAIL");
    } else {
        printf("(%+6.2f,%+6.2f) err (%+5.2f,%+5.2f) spread %4.2f  %2d/%2d  %s\n",
               r.dx, r.dy, ex, ey, r.spread, r.n_valid, r.n_total,
               pass ? "ok" : "FAIL");
        if (e > worst_err) worst_err = e;
        err_sum += fabs(ex) + fabs(ey);
        err_n += 2;
    }
    if (!pass) failures++;
    free(cur);
}

int main(void)
{
    uint8_t *base = malloc(W * H);
    make_texture(base, W, H);

    printf("flow_match host verification  (%dx%d, block %d, search +/-%d, grid %dx%d)\n\n",
           W, H, CFG.block, CFG.search, CFG.grid_x, CFG.grid_y);

    printf("1. integer shifts (tolerance 0.25 px)\n");
    float ints[][2] = { {0,0}, {1,0}, {0,1}, {-1,-1}, {-2,3}, {4,-4}, {5,5}, {-6,2} };
    for (size_t i = 0; i < sizeof(ints)/sizeof(ints[0]); i++) {
        expect_shift(base, ints[i][0], ints[i][1], 0.25f);
    }

    printf("\n2. sub-pixel shifts (tolerance 0.25 px)\n");
    float fracs[][2] = { {0.5f,0}, {0,-0.5f}, {0.25f,-0.75f}, {1.5f,2.5f},
                         {-3.3f,1.7f}, {2.1f,-0.4f}, {-0.2f,-4.8f} };
    for (size_t i = 0; i < sizeof(fracs)/sizeof(fracs[0]); i++) {
        expect_shift(base, fracs[i][0], fracs[i][1], 0.25f);
    }

    printf("\n3. featureless frame must yield NO MEASUREMENT, not zero motion\n");
    {
        uint8_t *flat = malloc(W * H);
        memset(flat, 128, W * H);
        flow_result_t r;
        int ok = flow_match(&CFG, flat, flat, &r);
        int pass = (ok == 0) && (r.n_flat == r.n_total);
        printf("  flat grey -> ok=%d valid %d/%d flat %d   %s\n",
               ok, r.n_valid, r.n_total, r.n_flat, pass ? "ok" : "FAIL");
        if (!pass) failures++;
        free(flat);
    }

    printf("\n4. shift beyond the search radius must be REJECTED, not clamped\n");
    for (int sx = 8; sx <= 12; sx += 2) {
        uint8_t *cur = malloc(W * H);
        shift_image(base, cur, W, H, (float)sx, 0.0f);
        flow_result_t r;
        int ok = flow_match(&CFG, base, cur, &r);
        /* The honest outcomes are "no measurement", or a measurement whose
           error is obvious. What must NOT happen is a confident report pegged
           at the search limit. */
        int pegged = ok && fabsf(r.dx) >= (float)CFG.search - 0.5f && r.spread < 0.5f;
        printf("  shift (%+3d, 0) -> ok=%d dx=%+6.2f spread %4.2f edge %d/%d   %s\n",
               sx, ok, ok ? r.dx : 0.0f, ok ? r.spread : 0.0f,
               r.n_edge, r.n_total, pegged ? "FAIL (silently pegged)" : "ok");
        if (pegged) failures++;
        free(cur);
    }

    printf("\n-----------------------------------------------------------\n");
    if (err_n) {
        printf("mean |error| %.3f px over %d axes, worst %.3f px\n",
               err_sum / err_n, err_n, worst_err);
    }
    printf("%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    free(base);
    return failures ? 1 : 0;
}
