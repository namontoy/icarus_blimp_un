/*
 * Sub-pixel bias ("pixel locking") measurement for flow_match, on the host.
 *
 * Why this exists. On the bench the camera integrates position by summing
 * per-frame displacements, and every slide test so far has read 51-87% of the
 * true distance - always short, never long. The recorded runs show why that is
 * hard to see: during a 300 mm slide at 1.5 m the mean displacement is only
 * 0.121 px per frame, so the integer part of the match is almost always zero
 * and the ENTIRE measurement comes out of the sub-pixel refinement. If the
 * parabolic fit in flow_match systematically pulls small shifts toward zero -
 * the classic pixel-locking error - the integrated distance is short by that
 * same fraction, and the deficit grows with how slowly you move.
 *
 * This program measures that curve directly, with no camera and no ESP32.
 *
 * Fairness of the synthetic frames. The existing test_flow_match shifts a
 * uint8 image with bilinear interpolation, which smooths the shifted frame but
 * not the reference. That asymmetry is itself a bias source and would
 * contaminate exactly the number we are after. Here the texture is instead a
 * CONTINUOUS function - bilinearly interpolated value noise, evaluated
 * analytically at any real offset - so both frames are produced by one
 * identical evaluation of the same function and differ only in offset and in
 * 8-bit quantisation, which a real sensor also applies.
 *
 *   make bias_sweep && ./bias_sweep
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../main/flow_match.h"

#define W 160
#define H 120

static flow_cfg_t CFG = {
    .width = W, .height = H, .block = 16, .search = 7,
    .grid_x = 5, .grid_y = 5, .min_var = 50,
    .subpix = FLOW_SUBPIX_PARABOLA,
};

static const char *subpix_name(int m)
{
    switch (m) {
    case FLOW_SUBPIX_GAUSSIAN: return "gaussian";
    case FLOW_SUBPIX_VFIT:     return "vfit";
    default:                   return "parabola";
    }
}

/* ------------------------------------------------------- continuous texture */

/* A value-noise octave: a random lattice sampled by bilinear interpolation.
   Keeping the lattice rather than a rendered image is what lets us evaluate
   the texture at an arbitrary real offset without resampling a raster. */
typedef struct {
    float *g;           /* lattice values */
    int    gw, gh;      /* lattice dimensions */
    float  cell;        /* lattice spacing in output pixels */
    float  amp;
} octave_t;

#define MAX_OCT 4
typedef struct {
    octave_t oct[MAX_OCT];
    int      n;
    float    lo, hi;    /* range, for mapping to 8 bits */
} texture_t;

static uint32_t rng_state = 12345u;
static float rnd(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return (float)((rng_state >> 8) & 0xFFFFFF) / (float)0xFFFFFF;
}

static float oct_eval(const octave_t *o, float x, float y)
{
    /* Wrap into the lattice so an offset frame never samples past the edge. */
    float fx = x / o->cell, fy = y / o->cell;
    int   ix = (int)floorf(fx), iy = (int)floorf(fy);
    float tx = fx - ix,         ty = fy - iy;
    int   gw = o->gw, gh = o->gh;
    int   x0 = ((ix % (gw - 1)) + (gw - 1)) % (gw - 1);
    int   y0 = ((iy % (gh - 1)) + (gh - 1)) % (gh - 1);
    float a = o->g[y0 * gw + x0],           b = o->g[y0 * gw + x0 + 1];
    float c = o->g[(y0 + 1) * gw + x0],     d = o->g[(y0 + 1) * gw + x0 + 1];
    float top = a + (b - a) * tx, bot = c + (d - c) * tx;
    return o->amp * (top + (bot - top) * ty);
}

static float tex_eval(const texture_t *t, float x, float y)
{
    float v = 0.0f;
    for (int i = 0; i < t->n; i++) {
        v += oct_eval(&t->oct[i], x, y);
    }
    return v;
}

static void tex_make(texture_t *t, const float *cells, const float *amps, int n)
{
    t->n = n;
    for (int i = 0; i < n; i++) {
        octave_t *o = &t->oct[i];
        o->cell = cells[i];
        o->amp  = amps[i];
        o->gw = (int)(W / cells[i]) + 3;
        o->gh = (int)(H / cells[i]) + 3;
        o->g  = malloc((size_t)o->gw * o->gh * sizeof(float));
        for (int k = 0; k < o->gw * o->gh; k++) {
            o->g[k] = rnd();
        }
    }
    /* Calibrate the 8-bit mapping on a dense sample of the field. */
    t->lo = 1e9f; t->hi = -1e9f;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float v = tex_eval(t, (float)x, (float)y);
            if (v < t->lo) t->lo = v;
            if (v > t->hi) t->hi = v;
        }
    }
}

static void tex_free(texture_t *t)
{
    for (int i = 0; i < t->n; i++) {
        free(t->oct[i].g);
    }
}

/* Render the texture into an 8-bit frame, as seen with the origin displaced by
   (sx, sy). Content therefore appears to move by (+sx, +sy), matching the sign
   convention of flow_match and of test_flow_match's shift_image.
   ramp adds a linear brightness gradient across x, in counts across the frame,
   which is how a lamp to one side shows up. */
static void render(const texture_t *t, uint8_t *dst, float sx, float sy,
                   float ramp)
{
    float scale = 215.0f / (t->hi - t->lo);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float v = tex_eval(t, (float)x - sx, (float)y - sy);
            v = 20.0f + scale * (v - t->lo);
            v += ramp * ((float)x / (float)(W - 1) - 0.5f);
            dst[y * W + x] = (uint8_t)(v < 0.0f ? 0.0f
                                     : (v > 255.0f ? 255.0f : v + 0.5f));
        }
    }
}

/* ------------------------------------------------------------------- part A */

/* Gain of a single match, averaged over independent texture realisations.
   The absolute phase of the pair relative to the pixel grid is varied too:
   locking depends on where the shift sits between integers, and a flying
   airframe samples every phase, so the figure that matters is the average. */
static void part_a(int n_tex, int n_phase, const float *cells,
                   const float *amps, int n_oct, const char *label)
{
    static const float steps[] = {
        0.02f, 0.05f, 0.08f, 0.10f, 0.12f, 0.15f, 0.20f, 0.25f, 0.30f,
        0.40f, 0.50f, 0.70f, 1.00f, 1.50f, 2.00f, 3.00f,
    };
    const int ns = (int)(sizeof(steps) / sizeof(steps[0]));

    printf("A. single-step gain, texture \"%s\"  (%d textures x %d phases)\n",
           label, n_tex, n_phase);
    printf("   %8s %10s %8s %9s %7s\n",
           "true_px", "measured", "gain", "bias_px", "valid");

    uint8_t *prev = malloc(W * H), *cur = malloc(W * H);

    for (int i = 0; i < ns; i++) {
        double sum = 0.0, sumval = 0.0;
        int n = 0;
        rng_state = 20250101u;          /* same textures for every step */
        for (int k = 0; k < n_tex; k++) {
            texture_t t;
            tex_make(&t, cells, amps, n_oct);
            for (int p = 0; p < n_phase; p++) {
                /* Place the pair at an arbitrary absolute position so the
                   shift is not always measured from a lattice-aligned start. */
                float base = (float)p * 0.137f * 7.0f;
                render(&t, prev, base, base * 0.6f, 0.0f);
                render(&t, cur, base + steps[i], base * 0.6f, 0.0f);
                flow_result_t r;
                if (flow_match(&CFG, prev, cur, &r)) {
                    sum += r.dx;
                    sumval += r.n_valid;
                    n++;
                }
            }
            tex_free(&t);
        }
        if (n == 0) {
            printf("   %8.2f %10s\n", steps[i], "no match");
            continue;
        }
        double meas = sum / n;
        printf("   %8.2f %10.4f %8.3f %+9.4f %7.1f\n",
               steps[i], meas, meas / steps[i], meas - steps[i], sumval / n);
    }
    printf("\n");
    free(prev);
    free(cur);
}

/* ------------------------------------------------------------------- part B */

/* The bench test, reproduced: integrate many consecutive small displacements
   and compare the total against the truth. This is the number that the slide
   runs actually measure, and unlike part A it accumulates whatever phase
   dependence exists rather than averaging it away. */
static void part_b(int n_tex, const float *cells, const float *amps, int n_oct,
                   const char *label)
{
    static const float steps[] = {
        0.05f, 0.08f, 0.10f, 0.12f, 0.15f, 0.20f, 0.30f, 0.50f, 1.00f,
    };
    const int ns = (int)(sizeof(steps) / sizeof(steps[0]));
    const int nframes = 200;

    printf("B. integrated over %d consecutive frames, texture \"%s\"  (%d textures)\n",
           nframes, label, n_tex);
    printf("   %8s %10s %11s %11s %8s %6s\n",
           "step_px", "mm/frame", "true_total", "meas_total", "recovered", "lost");

    uint8_t *prev = malloc(W * H), *cur = malloc(W * H);

    for (int i = 0; i < ns; i++) {
        double tot = 0.0;
        int reps = 0;
        rng_state = 20250101u;
        for (int k = 0; k < n_tex; k++) {
            texture_t t;
            tex_make(&t, cells, amps, n_oct);
            double sum = 0.0;
            render(&t, prev, 0.0f, 0.0f, 0.0f);
            for (int f = 1; f <= nframes; f++) {
                render(&t, cur, steps[i] * (float)f, 0.0f, 0.0f);
                flow_result_t r;
                if (flow_match(&CFG, prev, cur, &r)) {
                    sum += r.dx;
                }
                memcpy(prev, cur, W * H);
            }
            tot += sum;
            reps++;
            tex_free(&t);
        }
        double meas = tot / reps;
        double truth = (double)steps[i] * nframes;
        printf("   %8.2f %10.2f %11.2f %11.2f %8.1f%% %6.1f%%\n",
               steps[i], steps[i] * 9.5385, truth, meas,
               100.0 * meas / truth, 100.0 * (1.0 - meas / truth));
    }
    printf("\n");
    free(prev);
    free(cur);
}

/* ------------------------------------------------------------------- part C */

/* Zero real motion, but the illumination changes between the two frames. The
   lamp-off experiment on the bench cut the at-rest drift by 20x, so the
   matcher is known to be sensitive to this; here we can put a number on it.
   A plain SAD has no brightness normalisation, so a gradient that grows or
   shrinks between frames moves the SAD minimum along the gradient. */
static void part_c(int n_tex, const float *cells, const float *amps, int n_oct)
{
    static const float ramps[] = { 0.0f, 1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 40.0f };
    const int nr = (int)(sizeof(ramps) / sizeof(ramps[0]));

    printf("C. zero motion, brightness gradient changing between frames\n");
    printf("   a gradient of this many counts across the frame appears in frame 2\n");
    printf("   %10s %12s %12s %8s\n",
           "ramp", "measured_dx", "mm/s at 12.5", "valid");

    uint8_t *prev = malloc(W * H), *cur = malloc(W * H);

    for (int i = 0; i < nr; i++) {
        double sum = 0.0, sumval = 0.0;
        int n = 0;
        rng_state = 20250101u;
        for (int k = 0; k < n_tex; k++) {
            texture_t t;
            tex_make(&t, cells, amps, n_oct);
            render(&t, prev, 0.0f, 0.0f, 0.0f);
            render(&t, cur, 0.0f, 0.0f, ramps[i]);
            flow_result_t r;
            if (flow_match(&CFG, prev, cur, &r)) {
                sum += r.dx;
                sumval += r.n_valid;
                n++;
            }
            tex_free(&t);
        }
        if (n == 0) {
            printf("   %10.1f %12s\n", ramps[i], "no match");
            continue;
        }
        double meas = sum / n;
        /* 9.5385 mm/px at 1.5 m, 12.5 frames per second. */
        printf("   %10.1f %12.4f %12.2f %8.1f\n",
               ramps[i], meas, meas * 9.5385 * 12.5, sumval / n);
    }
    printf("\n");
    free(prev);
    free(cur);
}

/* ------------------------------------------------------------ dense curve */

/* Prints "true measured" pairs on a fine grid, for inverting recorded bench
   traces: given a measured per-frame dx, what displacement produced it? The
   grid is fine near zero because that is where the bench data lives and where
   the curve is steepest. */
static void dump_curve(int n_tex, int n_phase, const float *cells,
                       const float *amps, int n_oct)
{
    uint8_t *prev = malloc(W * H), *cur = malloc(W * H);
    printf("# true_px measured_px   (block %d search %d grid %dx%d)\n",
           CFG.block, CFG.search, CFG.grid_x, CFG.grid_y);
    for (int i = 0; i <= 200; i++) {
        /* 0 to 3 px, quadratically spaced so the dense end is near zero. */
        float tr = 3.0f * ((float)i / 200.0f) * ((float)i / 200.0f);
        double sum = 0.0;
        int n = 0;
        rng_state = 20250101u;
        for (int k = 0; k < n_tex; k++) {
            texture_t t;
            tex_make(&t, cells, amps, n_oct);
            for (int p = 0; p < n_phase; p++) {
                float base = (float)p * 0.137f * 7.0f;
                render(&t, prev, base, base * 0.6f, 0.0f);
                render(&t, cur, base + tr, base * 0.6f, 0.0f);
                flow_result_t r;
                if (flow_match(&CFG, prev, cur, &r)) {
                    sum += r.dx;
                    n++;
                }
            }
            tex_free(&t);
        }
        if (n) {
            printf("%.6f %.6f\n", tr, sum / n);
        }
    }
    free(prev);
    free(cur);
}

/* --------------------------------------------------------------------- main */

int main(int argc, char **argv)
{
    /* "smooth" is the texture the existing suite uses: two octaves, coarsest
       feature 11 px. "fine" adds a third octave near the resolution limit,
       which is closer to the printed symbols the camera is actually looking
       at and is where locking is expected to be worst. */
    static const float smooth_cells[] = { 11.0f, 5.0f };
    static const float smooth_amps[]  = { 1.00f, 0.45f };
    static const float fine_cells[]   = { 11.0f, 5.0f, 2.2f };
    static const float fine_amps[]    = { 1.00f, 0.45f, 0.30f };

    static const int modes[] = {
        FLOW_SUBPIX_PARABOLA, FLOW_SUBPIX_GAUSSIAN, FLOW_SUBPIX_VFIT,
    };
    const int nm = (int)(sizeof(modes) / sizeof(modes[0]));

    if (argc > 1 && strcmp(argv[1], "curve") == 0) {
        /* Dense transfer curve for the estimator named by argv[2], used to
           invert recorded bench traces. Defaults to the parabola, which is
           what the recorded runs were taken with. */
        CFG.subpix = FLOW_SUBPIX_PARABOLA;
        if (argc > 2) {
            for (int i = 0; i < nm; i++) {
                if (strcmp(argv[2], subpix_name(modes[i])) == 0) {
                    CFG.subpix = modes[i];
                }
            }
        }
        printf("# estimator %s\n", subpix_name(CFG.subpix));
        dump_curve(8, 4, fine_cells, fine_amps, 3);
        return 0;
    }

    printf("sub-pixel bias sweep  (%dx%d, block %d, search +/-%d, grid %dx%d,"
           " min_var %u)\n", W, H, CFG.block, CFG.search,
           CFG.grid_x, CFG.grid_y, CFG.min_var);
    printf("reference: the 300 mm bench slide averaged 0.121 px per frame\n\n");

    for (int i = 0; i < nm; i++) {
        CFG.subpix = modes[i];
        printf("===================================================="
               " estimator: %s\n\n", subpix_name(modes[i]));
        part_a(8, 4, smooth_cells, smooth_amps, 2, "smooth");
        part_a(8, 4, fine_cells, fine_amps, 3, "fine");
        part_b(4, fine_cells, fine_amps, 3, "fine");
        part_c(16, fine_cells, fine_amps, 3);
    }

    return 0;
}
