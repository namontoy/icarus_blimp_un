/*
 * Frame-size benchmark for flow_match. See flow_bench.h.
 */
#include <math.h>
#include "flow_bench.h"

/*
 * The candidates. Every row is sized for one design point: a camera 2 m above
 * the floor, an 80 ms frame period, and a velocity range of at least about
 * 0.6 m/s (twice the ~0.3 m/s the blimp coasts at, docs/flight-tests.md) at
 * the bench's 58 degree lens. The search radius therefore grows with the
 * frame width (usable range is search - 1 px per frame), and it is the search
 * radius, squared, that the match time follows. Rows that cannot reach that
 * range inside FLOW_MAX_SEARCH say so in the range column.
 *
 * QQVGA 16/7/5 is the AI-Thinker's configuration today, measured on that
 * board at 49-52 ms (ESP32, 240 MHz): it is the row that calibrates the rest.
 */
const flow_bench_case_t flow_bench_cases[] = {
    { "QQVGA", 160, 120, 16,  7, 5 },
    { "HQVGA", 240, 176, 16,  7, 5 },
    { "QVGA",  320, 240, 16,  8, 5 },
    { "QVGA",  320, 240, 16,  8, 4 },
    { "QVGA",  320, 240, 24,  8, 4 },
    { "CIF",   400, 296, 16, 10, 4 },
    { "HVGA",  480, 320, 16, 10, 4 },
    { "VGA",   640, 480, 16, 10, 4 },
    { 0, 0, 0, 0, 0, 0 },
};

/* ------------------------------------------------------------- the floor */

static uint32_t hash3(int32_t x, int32_t y, uint32_t salt)
{
    uint32_t h = (uint32_t)x * 0x8da6b343u ^ (uint32_t)y * 0xd8163841u ^ salt * 0xcb1ab31fu;
    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return h;
}

static float lattice(int32_t x, int32_t y, uint32_t salt)
{
    return (float)(hash3(x, y, salt) >> 8) * (1.0f / 16777216.0f);
}

/* One octave of value noise with cells cell_mm across, at floor point (X, Y). */
static float octave(float X, float Y, float cell_mm, uint32_t salt)
{
    float fx = X / cell_mm, fy = Y / cell_mm;
    float gx = floorf(fx), gy = floorf(fy);
    float tx = fx - gx, ty = fy - gy;
    int32_t ix = (int32_t)gx, iy = (int32_t)gy;
    float a = lattice(ix, iy, salt),     b = lattice(ix + 1, iy, salt);
    float c = lattice(ix, iy + 1, salt), d = lattice(ix + 1, iy + 1, salt);
    float top = a + (b - a) * tx, bot = c + (d - c) * tx;
    return top + (bot - top) * ty;
}

/*
 * Floor brightness at (X, Y) mm, 0..1. Four octaves from 150 mm down to
 * 12 mm: the coarse ones are what QQVGA (13 mm/px at 2 m) can see, the 12 mm
 * one is texture only a finer pixel resolves. A real floor has detail at
 * every scale; this is the cheapest stand-in that lets a bigger frame find
 * something its extra pixels can use.
 */
static float floor_at(float X, float Y, uint32_t seed)
{
    return 0.40f * octave(X, Y, 150.0f, seed)
         + 0.25f * octave(X, Y,  70.0f, seed + 1u)
         + 0.20f * octave(X, Y,  30.0f, seed + 2u)
         + 0.15f * octave(X, Y,  12.0f, seed + 3u);
}

static uint32_t s_rng;

static float noise_sample(float sigma)
{
    /* Sum of four uniforms: close enough to Gaussian for sensor noise, and
       needs no logf. Variance of the sum is 4/12, hence the sqrt(3). */
    float s = 0.0f;
    for (int i = 0; i < 4; i++) {
        s_rng = s_rng * 1664525u + 1013904223u;
        s += (float)(s_rng >> 8) * (1.0f / 16777216.0f);
    }
    return (s - 2.0f) * 1.7320508f * sigma;
}

/* A pixel integrates the floor over its footprint, as a sensor does; 2x2
   samples per pixel keep the 12 mm octave from aliasing at coarse frames. */
static uint8_t pixel(float X, float Y, float mmpx, float noise, uint32_t seed)
{
    float q = 0.25f * mmpx;
    float v = 0.25f * (floor_at(X - q, Y - q, seed) + floor_at(X + q, Y - q, seed)
                     + floor_at(X - q, Y + q, seed) + floor_at(X + q, Y + q, seed));
    float c = 30.0f + 200.0f * v + noise_sample(noise);
    if (c < 0.0f) c = 0.0f;
    if (c > 255.0f) c = 255.0f;
    return (uint8_t)(c + 0.5f);
}

void flow_bench_frames(uint8_t *prev, uint8_t *cur, int width, int height,
                       float mm_per_px, float sx_mm, float sy_mm,
                       float noise, uint32_t seed)
{
    /* Each seed looks at a different patch of floor. */
    float ox = (float)(seed % 97u) * 211.0f, oy = (float)(seed % 89u) * 173.0f;
    s_rng = seed * 2654435761u + 1u;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float X = ox + ((float)x + 0.5f) * mm_per_px;
            float Y = oy + ((float)y + 0.5f) * mm_per_px;
            prev[y * width + x] = pixel(X, Y, mm_per_px, noise, 7u);
            /* cur(x) shows the floor point that was at x - s: content moved +s */
            cur[y * width + x] = pixel(X - sx_mm, Y - sy_mm, mm_per_px, noise, 7u);
        }
    }
}

void flow_bench_cfg(const flow_bench_case_t *c, flow_cfg_t *cfg)
{
    cfg->width   = c->width;
    cfg->height  = c->height;
    cfg->block   = c->block;
    cfg->search  = c->search;
    cfg->grid_x  = c->grid;
    cfg->grid_y  = c->grid;
    cfg->min_var = 50;
    cfg->subpix  = FLOW_SUBPIX_VFIT;
}

float flow_bench_mm_per_px(const flow_bench_case_t *c, const flow_bench_scene_t *scene)
{
    return 2.0f * scene->height_mm * tanf(scene->hfov_deg * 3.14159265f / 360.0f)
         / (float)c->width;
}

int flow_bench_run(const flow_bench_case_t *c, const flow_bench_scene_t *scene,
                   uint8_t *prev, uint8_t *cur, int64_t (*now_us)(void),
                   flow_bench_result_t *out)
{
    const float pi = 3.14159265f;
    flow_cfg_t cfg;
    flow_bench_cfg(c, &cfg);
    float mmpx = flow_bench_mm_per_px(c, scene);

    out->mm_per_px  = mmpx;
    out->range_mm_s = (float)(c->search - 1) * mmpx * 1000.0f / scene->period_ms;
    out->patch_mm   = (float)c->block * mmpx;
    out->mops       = (float)(c->grid * c->grid) * (float)((2 * c->search + 1) * (2 * c->search + 1))
                    * (float)(c->block * c->block) * 1e-6f;
    out->ms_min = 1e9f;
    out->ms_mean = out->ms_max = 0.0f;
    out->err_rms_mm = out->err_rms_px = out->err_mean_mm = 0.0f;
    out->valid_mean = 0.0f;
    out->n_ok = 0;

    if (!flow_cfg_valid(&cfg)) {
        return 0;
    }

    float step_mm = scene->speed_mm_s * scene->period_ms * 1e-3f;
    double se_mm = 0.0, along_mm = 0.0;
    float ux = 0.0f, uy = 0.0f;
    for (int t = 0; t < scene->trials; t++) {
        /* Directions spread round the circle, so neither axis is favoured. */
        float ang = 2.0f * pi * ((float)t + 0.37f) / (float)scene->trials;
        ux = cosf(ang);
        uy = sinf(ang);
        float sx = step_mm * ux, sy = step_mm * uy;
        flow_bench_frames(prev, cur, c->width, c->height, mmpx, sx, sy,
                          scene->noise, 1000u + (uint32_t)t);

        flow_result_t r;
        int64_t t0 = now_us();
        int ok = flow_match(&cfg, prev, cur, &r);
        float ms = (float)(now_us() - t0) * 1e-3f;

        if (ms < out->ms_min) out->ms_min = ms;
        if (ms > out->ms_max) out->ms_max = ms;
        out->ms_mean += ms;
        out->valid_mean += (float)r.n_valid;
        if (ok) {
            float ex = r.dx * mmpx - sx, ey = r.dy * mmpx - sy;
            se_mm += (double)(ex * ex + ey * ey);
            along_mm += (double)(ex * ux + ey * uy);
            out->n_ok++;
        }
    }
    out->ms_mean /= (float)scene->trials;
    out->valid_mean /= (float)scene->trials;
    if (out->n_ok > 0) {
        out->err_rms_mm = (float)sqrt(se_mm / (2.0 * out->n_ok));
        out->err_rms_px = out->err_rms_mm / mmpx;
        out->err_mean_mm = (float)(along_mm / out->n_ok);
    }
    return 1;
}
