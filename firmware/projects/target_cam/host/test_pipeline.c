/*
 * Host-side verification of tc_pipeline: the stage plumbing, not the learning.
 *
 * Each learned stage is tested with its own data in the phase that adds it.
 * What is checked here holds for every phase:
 *
 *   - the portable CRC-32 gives zlib's published values, because the PC
 *     training scripts write model records with zlib.crc32 and the camera
 *     checks them with tc_crc32;
 *   - the effective stage follows the prefix rule (a stage needs every model
 *     below it), so a missing or rejected model always falls back;
 *   - the pipeline at stage 0 is blob_detect, bit for bit, so wrapping the
 *     detector changed nothing the CSV reports.
 */
#include <stdio.h>
#include <string.h>

#include "../main/tc_pipeline.h"

#define W 160
#define H 120

static uint8_t  frame[W * H * 2];
static uint32_t work_buf[(W * H) / 2 + 64];     /* > 28 kB, 4-byte aligned */
static int failures;

static uint32_t rng = 777u;
static int rnd(int n)
{
    rng = rng * 1664525u + 1013904223u;
    return (int)((rng >> 8) % (uint32_t)n);
}

static void check(int cond, const char *what)
{
    if (!cond) {
        printf("    FAIL: %s\n", what);
        failures++;
    }
}

static void put(int x, int y, int r, int g, int b)
{
    uint16_t px = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
    uint8_t *p = frame + ((size_t)y * W + (size_t)x) * 2;
    p[0] = (uint8_t)(px >> 8);                      /* big-endian, as the camera */
    p[1] = (uint8_t)px;
}

/* Grey noise with n red discs of random size and place. */
static void scene(int n)
{
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int v = 60 + rnd(140);
            put(x, y, v, v, v + 4);
        }
    for (int k = 0; k < n; k++) {
        float cx = (float)(10 + rnd(140)), cy = (float)(10 + rnd(100));
        float rr = (float)(3 + rnd(14));
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                float dx = (float)x + 0.5f - cx, dy = (float)y + 0.5f - cy;
                if (dx * dx + dy * dy <= rr * rr)
                    put(x, y, 200 + rnd(40), 20 + rnd(20), 20 + rnd(20));
            }
    }
}

static blob_cfg_t red_cfg(void)
{
    blob_cfg_t c = {
        .width = W, .height = H, .cell = 2, .big_endian = 1,
        .hue = 0, .hue_tol = 15, .sat_min = 120, .val_min = 50, .min_area = 30,
    };
    return c;
}

/* A clock that advances 1 us per call, so every timed span is positive and
   the spans can be checked against each other. */
static int64_t s_ticks;
static int64_t fake_clock(void) { return ++s_ticks; }

/* ------------------------------------------------------------------ */

static void test_crc(void)
{
    printf("1. tc_crc32 against zlib's published values\n");
    const char *q = "The quick brown fox jumps over the lazy dog";
    uint32_t a = tc_crc32("123456789", 9);
    uint32_t b = tc_crc32(q, strlen(q));
    uint32_t e = tc_crc32("", 0);
    printf("   \"123456789\" %08x, fox %08x, empty %08x\n",
           (unsigned)a, (unsigned)b, (unsigned)e);
    check(a == 0xCBF43926u, "check value is not CBF43926");
    check(b == 0x414FA339u, "fox sentence is not 414FA339");
    check(e == 0u, "empty buffer is not 0");
}

static void test_stage_rule(void)
{
    printf("2. effective stage = min(request, highest complete prefix)\n");
    static const int dummy;                         /* any non-NULL pointer */
    const void *P = &dummy;
    struct { tc_models_t m; int avail; const char *name; } cases[] = {
        { { NULL, NULL, NULL }, 0, "no models"                         },
        { { P,    NULL, NULL }, 1, "table only"                        },
        { { P,    P,    NULL }, 2, "table + classifier"                },
        { { P,    P,    P    }, 3, "all three"                         },
        { { NULL, P,    P    }, 0, "classifier + CNN, no table"        },
        { { P,    NULL, P    }, 1, "table + CNN, no classifier"        },
    };
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        int got = tc_stage_available(&cases[i].m);
        printf("   %-28s available %d\n", cases[i].name, got);
        check(got == cases[i].avail, "wrong available stage");
        for (int req = -2; req <= TC_STAGE_MAX + 2; req++) {
            int want = req < 0 ? 0 : req > TC_STAGE_MAX ? TC_STAGE_MAX : req;
            if (want > cases[i].avail) want = cases[i].avail;
            check(tc_stage_effective(req, &cases[i].m) == want,
                  "wrong effective stage");
        }
    }
    check(tc_stage_effective(3, NULL) == 0, "NULL models must mean stage 0");
}

static void test_stage0_is_blob_detect(void)
{
    printf("3. pipeline at stage 0 == blob_detect, bit for bit (40 scenes)\n");
    blob_cfg_t c = red_cfg();
    tc_models_t none = { NULL, NULL, NULL };
    int same = 0, found = 0;
    for (int i = 0; i < 40; i++) {
        scene(i % 6);
        blob_result_t ref;
        blob_detect(&c, frame, work_buf, &ref);
        for (int req = 0; req <= TC_STAGE_MAX; req++) {
            tc_result_t t;
            memset(&t, 0xA5, sizeof t);             /* catch unset fields */
            int ok = tc_pipeline_run(&c, req, &none, frame, work_buf, NULL, &t);
            int eq = memcmp(&t.r, &ref, sizeof ref) == 0 && ok == ref.found;
            same += eq;
            check(eq, "pipeline result differs from blob_detect");
            check(t.stage == 0, "stage must fall back to 0 with no models");
            for (int k = 0; k < BLOB_MAX_REPORT; k++)
                check(t.conf[k] == -1.0f, "conf must be -1 below stage 2");
            check(t.t_px_us == 0 && t.t_b_us == 0 && t.t_c_us == 0 &&
                  t.t_total_us == 0, "times must be 0 with no clock");
        }
        found += ref.found;
    }
    printf("   %d/160 identical, %d of 40 scenes had a target\n", same, found);
}

static void test_timing(void)
{
    printf("4. stage times with a clock\n");
    blob_cfg_t c = red_cfg();
    tc_models_t none = { NULL, NULL, NULL };
    scene(3);
    tc_result_t t;
    s_ticks = 0;
    tc_pipeline_run(&c, 0, &none, frame, work_buf, fake_clock, &t);
    printf("   ticks: px %d, total %d, clock calls %lld\n",
           (int)t.t_px_us, (int)t.t_total_us, (long long)s_ticks);
    check(t.t_px_us > 0, "pixel pass not timed");
    check(t.t_total_us > t.t_px_us, "total must include the pixel pass");
    check(t.t_b_us == 0 && t.t_c_us == 0, "stages that did not run must read 0");

    blob_result_t a, b;
    int32_t tp;
    blob_detect(&c, frame, work_buf, &a);
    blob_detect_timed(&c, frame, work_buf, &b, fake_clock, &tp);
    check(memcmp(&a, &b, sizeof a) == 0, "timing changed the detection");
}

int main(void)
{
    test_crc();
    test_stage_rule();
    test_stage0_is_blob_detect();
    test_timing();
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
