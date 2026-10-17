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
 *
 * Stage 1, the colour table (Phase A2):
 *
 *   - a table built from the HSV rule makes stage 1 report exactly what
 *     stage 0 does, so the table path changes nothing but the pixel test;
 *   - a model record is refused for each kind of damage, with the right
 *     reason, and a refused table leaves the caller's copy untouched.
 *
 * Blob features (Phase B1):
 *
 *   - asking for them changes nothing the detector reports, fills exactly
 *     the reported slots with blob_features()'s answer, and times them.
 *     What each feature measures is test_blob_feat.c's business.
 *
 * Whether a TRAINED table is any good is a question about data, answered on
 * the labelled frames by host/train/train_lut.py and eval_frames.py, not here.
 */
#include <stdio.h>
#include <string.h>

#include "../main/tc_pipeline.h"
#include "../main/color_lut.h"

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
            int ok = tc_pipeline_run(&c, req, 0, &none, frame, work_buf, NULL, &t);
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
    tc_pipeline_run(&c, 0, 0, &none, frame, work_buf, fake_clock, &t);
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

/* Phase B1: features are an add-on to stages 0 and 1, asked for by flag. */
static void test_features_flag(void)
{
    printf("7. TC_RUN_FEATURES: features for every reported blob, detection unchanged\n");
    blob_cfg_t c = red_cfg();
    tc_models_t none = { NULL, NULL, NULL };
    int same = 0, nblobs = 0;
    for (int i = 0; i < 30; i++) {
        scene(i % 5);
        tc_result_t a, b;
        memset(&a, 0xA5, sizeof a);
        memset(&b, 0x5A, sizeof b);
        tc_pipeline_run(&c, 0, 0, &none, frame, work_buf, NULL, &a);
        tc_pipeline_run(&c, 0, TC_RUN_FEATURES, &none, frame, work_buf, NULL, &b);
        int eq = memcmp(&a.r, &b.r, sizeof a.r) == 0 && a.stage == b.stage;
        same += eq;
        check(eq, "computing features changed the detection");
        check(a.n_feat == 0 && b.n_feat == b.r.n_report, "n_feat wrong");
        for (int k = 0; k < BLOB_MAX_REPORT; k++) {
            blob_feat_t ref;
            memset(&ref, 0, sizeof ref);
            if (k < b.r.n_report) blob_features(&c, frame, &b.r.b[k], &ref);
            check(memcmp(&ref, &b.feat[k], sizeof ref) == 0,
                  "pipeline features differ from blob_features (or an empty slot is not 0)");
            check(b.conf[k] == -1.0f, "features alone must not produce a confidence");
        }
        nblobs += b.r.n_report;
    }
    printf("   %d/30 detections unchanged, %d blobs with features\n", same, nblobs);

    scene(4);
    tc_result_t t;
    s_ticks = 0;
    tc_pipeline_run(&c, 0, TC_RUN_FEATURES, &none, frame, work_buf, fake_clock, &t);
    printf("   ticks: px %d, b %d, total %d\n", (int)t.t_px_us, (int)t.t_b_us,
           (int)t.t_total_us);
    check(t.r.n_report > 0, "scene(4) should have blobs");
    check(t.t_b_us > 0, "features not timed");
    check(t.t_total_us > t.t_px_us + t.t_b_us, "total must include both spans");
}

static uint8_t lut[COLOR_LUT_BYTES];
static uint8_t mask_a[W * H], mask_b[W * H];

static void test_hsv_table_is_stage0(void)
{
    printf("5. stage 1 with the HSV rule as a table == stage 0, bit for bit\n");
    blob_cfg_t cfgs[4];
    cfgs[0] = red_cfg();                                    /* tight red    */
    cfgs[1] = red_cfg();                                    /* loose, wraps */
    cfgs[1].hue = 350; cfgs[1].hue_tol = 40; cfgs[1].sat_min = 60; cfgs[1].val_min = 20;
    cfgs[2] = red_cfg();                                    /* 1x1 cells    */
    cfgs[2].cell = 1;
    cfgs[3] = red_cfg();                                    /* other colour */
    cfgs[3].hue = 220; cfgs[3].hue_tol = 30; cfgs[3].sat_min = 90; cfgs[3].val_min = 40;
    static uint32_t big_work[(W * H * 6) / 4 + 64];         /* cell 1: 113 kB */
    int same = 0, runs = 0, found = 0;
    for (int ci = 0; ci < 4; ci++) {
        blob_cfg_t c = cfgs[ci];
        color_lut_from_hsv(&c, lut);
        tc_models_t m = { lut, NULL, NULL };
        if (ci == 0)
            printf("   red preset as a table: %d of 65536 colours\n", color_lut_count(lut));
        for (int i = 0; i < 25; i++) {
            scene(i % 6);
            tc_result_t t0, t1;
            tc_pipeline_run(&c, 0, 0, &m, frame, big_work, NULL, &t0);
            tc_pipeline_run(&c, 1, 0, &m, frame, big_work, NULL, &t1);
            int eq = memcmp(&t0.r, &t1.r, sizeof t0.r) == 0;
            int na = tc_pipeline_mask(&c, 0, &m, frame, mask_a);
            int nb = tc_pipeline_mask(&c, 1, &m, frame, mask_b);
            eq = eq && na == nb && memcmp(mask_a, mask_b, sizeof mask_a) == 0;
            check(eq, "HSV table differs from the HSV test");
            check(t0.stage == 0 && t1.stage == 1, "wrong effective stage");
            same += eq;
            runs++;
            found += t0.r.found;
        }
    }
    printf("   %d/%d identical (results and masks), %d had a target\n", same, runs, found);

    /* And the table is really what decides at stage 1: an empty one sees
       nothing where stage 0 sees the discs. */
    blob_cfg_t c = red_cfg();
    memset(lut, 0, sizeof lut);
    tc_models_t m = { lut, NULL, NULL };
    scene(3);
    tc_result_t t0, t1;
    tc_pipeline_run(&c, 0, 0, &m, frame, work_buf, NULL, &t0);
    tc_pipeline_run(&c, 1, 0, &m, frame, work_buf, NULL, &t1);
    printf("   empty table: stage 0 matched %d px, stage 1 matched %d\n",
           t0.r.matched, t1.r.matched);
    check(t0.r.found && !t1.r.found && t1.r.matched == 0, "stage 1 ignored the table");
}

/* A record as the training script writes it (tc_pipeline.h). */
static uint8_t rec[TC_MODEL_HDR_BYTES + COLOR_LUT_BYTES];

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static void make_record(void)
{
    memcpy(rec, COLOR_LUT_MAGIC, 4);
    rec[4] = COLOR_LUT_VERSION; rec[5] = 0;
    rec[6] = TC_MODEL_HDR_BYTES; rec[7] = 0;
    wr32(rec + 8, COLOR_LUT_BYTES);
    wr32(rec + 12, 1792100000u);
    for (int i = 0; i < COLOR_LUT_BYTES; i++)
        rec[TC_MODEL_HDR_BYTES + i] = (uint8_t)rnd(256);
    uint32_t crc = tc_crc32_update(tc_crc32(rec, 16), rec + TC_MODEL_HDR_BYTES,
                                   COLOR_LUT_BYTES);
    wr32(rec + 16, crc);
}

static void test_records(void)
{
    printf("6. model records: accepted intact, refused with the reason when damaged\n");
    check(tc_crc32_update(tc_crc32("1234", 4), "56789", 5) == 0xCBF43926u,
          "CRC does not continue across buffers");

    make_record();
    uint32_t id = 0;
    memset(lut, 0, sizeof lut);
    tc_model_err_t e = color_lut_load(rec, sizeof rec, lut, &id);
    printf("   intact          %s, train id %u\n", tc_model_err_str(e), (unsigned)id);
    check(e == TC_MODEL_OK && id == 1792100000u, "intact record refused");
    check(memcmp(lut, rec + TC_MODEL_HDR_BYTES, COLOR_LUT_BYTES) == 0, "table not copied");

    struct { int off; uint8_t xor_; size_t len; tc_model_err_t want; const char *what; } dmg[] = {
        { 0,     0x01, sizeof rec,     TC_MODEL_MAGIC,   "magic"            },
        { 4,     0x02, sizeof rec,     TC_MODEL_VERSION, "version"          },
        { 6,     0x04, sizeof rec,     TC_MODEL_VERSION, "header size"      },
        { 9,     0x01, sizeof rec,     TC_MODEL_SIZE,    "payload size"     },
        { 12,    0x80, sizeof rec,     TC_MODEL_CRC,     "train id bit"     },
        { 16,    0x01, sizeof rec,     TC_MODEL_CRC,     "stored CRC"       },
        { 20,    0x01, sizeof rec,     TC_MODEL_CRC,     "first table bit"  },
        { 8211,  0x80, sizeof rec,     TC_MODEL_CRC,     "last table bit"   },
        { -1,    0,    sizeof rec - 1, TC_MODEL_SHORT,   "one byte short"   },
        { -1,    0,    10,             TC_MODEL_SHORT,   "header cut"       },
    };
    for (size_t i = 0; i < sizeof dmg / sizeof dmg[0]; i++) {
        make_record();
        if (dmg[i].off >= 0) rec[dmg[i].off] ^= dmg[i].xor_;
        memset(lut, 0x5A, sizeof lut);
        id = 7;
        e = color_lut_load(rec, dmg[i].len, lut, &id);
        printf("   %-15s %s\n", dmg[i].what, tc_model_err_str(e));
        check(e == dmg[i].want, "wrong reason");
        int untouched = id == 7;
        for (int k = 0; k < COLOR_LUT_BYTES; k++) untouched &= lut[k] == 0x5A;
        check(untouched, "a refused record changed the caller's table");
    }
    check(color_lut_load(NULL, 0, lut, &id) == TC_MODEL_SHORT, "NULL record accepted");
}

int main(void)
{
    test_crc();
    test_stage_rule();
    test_stage0_is_blob_detect();
    test_timing();
    test_hsv_table_is_stage0();
    test_records();
    test_features_flag();
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
