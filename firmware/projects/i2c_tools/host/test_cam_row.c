/* test_cam_row.c - target_cam's CSV row parser (main/cam_row.c), on the
 * computer.
 *
 * The rows are made here with the printf format strings of target_cam's own
 * row (firmware/projects/target_cam/main/target_main.c, namontoy's, the
 * v2 row; v1 is the same without its last ten columns), so this checks the
 * format the camera really prints, not one written from its description. And
 * the lines a bare UART also carries, which must never read as a row: the
 * camera's '#' lines, its boot ROM's log, half rows, a version not known.
 * Run by tools/fw_host_test.py, which tools/verify.py calls.
 */
#include <stdio.h>
#include <string.h>

#include "cam_row.h"
#include "test_util.h"

/* One row as target_cam prints it: the 25 v1 columns, and with v2 the ten
 * more of the learned detector. Absent extra blobs print as 0,0,-1. */
static void make_row(char *buf, size_t size, int version, long long t_ms, int ok, float cx, float cy,
                     float az, float el, int area, float range_mm)
{
    int n = snprintf(buf, size, "%lld,%d,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%.2f,%.0f,%d,%d,%d",
                     t_ms, ok, (double)cx, (double)cy, (double)az, (double)el, area, ok ? 12 : 0,
                     ok ? 11 : 0, ok ? 0.91 : 0.0, (double)range_mm, 0, ok, ok ? 130 : 0);
    for (int k = 1; k < 4; k++) {
        n += snprintf(buf + n, size - (size_t)n, ",%.2f,%.2f,%.0f", 0.0, 0.0, -1.0);
    }
    n += snprintf(buf + n, size - (size_t)n, ",%.1f,%lld", 80.0, 12345LL);
    if (version == 2) {
        snprintf(buf + n, size - (size_t)n, ",%d,%.3g,%.3g,%.3g,%.3g,%ld,%ld,%ld,%lld,%d", 1,
                 ok ? 0.93 : -1.0, -1.0, -1.0, 1e-05, 1200L, 300L, 0L, 45000LL, 2);
    }
}

/* strcat without MSVC's deprecation, which -Werror turns into an error. */
static void append(char *buf, size_t size, const char *tail)
{
    const size_t n = strlen(buf);
    snprintf(buf + n, size - n, "%s", tail);
}

int main(void)
{
    printf("test_cam_row\n");
    char line[512];
    cam_row_t r;

    TEST_GROUP("a v2 row with a detection, as target_cam prints it");
    {
        make_row(line, sizeof(line), 2, 123456, 1, 95.25f, 50.10f, -12.34f, 4.56f, 120, 1500.0f);
        CHECK(cam_row_parse(line, &r));
        CHECK(r.version == 2);
        CHECK(r.detected);
        CHECK(r.cam_t_ms == 123456u);
        CHECK_NEAR(r.cx_px, 95.25f, 1e-3);
        CHECK_NEAR(r.cy_px, 50.10f, 1e-3);
        CHECK_NEAR(r.az_deg, -12.34f, 1e-3);
        CHECK_NEAR(r.el_deg, 4.56f, 1e-3);
        CHECK_NEAR(r.area_px, 120.0f, 1e-3);
        CHECK_NEAR(r.range_mm, 1500.0f, 1e-3);
    }

    TEST_GROUP("a v2 row with nothing detected");
    {
        make_row(line, sizeof(line), 2, 123536, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0, -1.0f);
        CHECK(cam_row_parse(line, &r));
        CHECK(!r.detected);
        CHECK_NEAR(r.range_mm, -1.0f, 1e-6);
    }

    TEST_GROUP("a v1 row (the first detector's 25 columns)");
    {
        make_row(line, sizeof(line), 1, 99, 1, 120.0f, 60.0f, 20.5f, -3.0f, 300, -1.0f);
        CHECK(cam_row_parse(line, &r));
        CHECK(r.version == 1);
        CHECK(r.detected);
        CHECK_NEAR(r.az_deg, 20.5f, 1e-3);
    }

    TEST_GROUP("what else the UART carries is never a row");
    {
        CHECK(!cam_row_parse("", &r));
        CHECK(!cam_row_parse("# columns      t_ms,ok,cx_px,cy_px,az_deg,el_deg,area_px", &r));
        CHECK(!cam_row_parse("# streaming off", &r));
        CHECK(!cam_row_parse("I (312) cam_hal: cam init ok", &r));
        CHECK(!cam_row_parse("rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)", &r));
        CHECK(!cam_row_parse("ets Jun  8 2016 00:22:57", &r));
        CHECK(!cam_row_parse(NULL, &r));

        /* A row cut short, as a dropped byte or a reset would leave it. */
        make_row(line, sizeof(line), 2, 1000, 1, 80.0f, 60.0f, 0.5f, 0.0f, 50, -1.0f);
        line[strlen(line) / 2] = '\0';
        CHECK(!cam_row_parse(line, &r));

        /* A corrupted digit. */
        make_row(line, sizeof(line), 2, 1000, 1, 80.0f, 60.0f, 0.5f, 0.0f, 50, -1.0f);
        char *third = strchr(strchr(line, ',') + 1, ',') + 1;
        third[1] = 'x';
        CHECK(!cam_row_parse(line, &r));

        /* Two rows run together (a lost newline). */
        char two[1024];
        make_row(line, sizeof(line), 1, 1000, 1, 80.0f, 60.0f, 0.5f, 0.0f, 50, -1.0f);
        snprintf(two, sizeof(two), "%s%s", line, line);
        CHECK(!cam_row_parse(two, &r));

        /* A trailing comma, an empty column. */
        make_row(line, sizeof(line), 1, 1000, 1, 80.0f, 60.0f, 0.5f, 0.0f, 50, -1.0f);
        append(line, sizeof(line), ",");
        CHECK(!cam_row_parse(line, &r));
    }

    TEST_GROUP("out of range, or a version not known: refused");
    {
        make_row(line, sizeof(line), 2, 1000, 2, 80.0f, 60.0f, 0.5f, 0.0f, 50, -1.0f);      /* ok = 2 */
        CHECK(!cam_row_parse(line, &r));
        make_row(line, sizeof(line), 2, 1000, 1, 80.0f, 60.0f, 120.0f, 0.0f, 50, -1.0f);    /* az */
        CHECK(!cam_row_parse(line, &r));
        make_row(line, sizeof(line), 2, 1000, 1, -5.0f, 60.0f, 0.5f, 0.0f, 50, -1.0f);      /* cx */
        CHECK(!cam_row_parse(line, &r));
        make_row(line, sizeof(line), 2, 1000, 1, 80.0f, 60.0f, 0.5f, 0.0f, 50, -1.0f);
        line[strlen(line) - 1] = '3';                                                   /* csv_v 3 */
        CHECK(!cam_row_parse(line, &r));
        make_row(line, sizeof(line), 2, 1000, 1, 80.0f, 60.0f, 0.5f, 0.0f, 50, -1.0f);
        append(line, sizeof(line), ",7");                                               /* 36 columns */
        CHECK(!cam_row_parse(line, &r));
        CHECK(!cam_row_parse("1000,1,80,60,nan,0,50,12,11,0.9,-1,0,1,130,0,0,-1,0,0,-1,0,0,-1,80.0,12345", &r));
    }

    TEST_GROUP("every kept column round-trips, v2");
    {
        make_row(line, sizeof(line), 2, 123456, 1, 95.25f, 50.10f, -12.34f, 4.56f, 120, 1500.0f);
        CHECK(cam_row_parse(line, &r));
        CHECK(r.w_px == 12 && r.h_px == 11);
        CHECK_NEAR(r.fill, 0.91f, 1e-3);
        CHECK(!r.clipped);
        CHECK(r.blobs == 1);
        CHECK(r.matched_px == 130u);
        for (int k = 0; k < 3; k++) {
            CHECK_NEAR(r.b_az_deg[k], 0.0f, 1e-6);
            CHECK_NEAR(r.b_el_deg[k], 0.0f, 1e-6);
            CHECK_NEAR(r.b_range_mm[k], -1.0f, 1e-6);
        }
        CHECK_NEAR(r.dt_ms, 80.0f, 1e-3);
        CHECK(r.proc_us == 12345u);
        CHECK(r.stage == 1);
        CHECK_NEAR(r.conf, 0.93f, 1e-3);
        CHECK_NEAR(r.b_conf[0], -1.0f, 1e-6);
        CHECK_NEAR(r.b_conf[1], -1.0f, 1e-6);
        CHECK_NEAR(r.b_conf[2], 1e-05f, 1e-7);   /* the fourth %.3g of make_row */
        CHECK(r.loop_us == 45000u);
    }

    TEST_GROUP("a v1 row: the v2 columns take their defaults");
    {
        make_row(line, sizeof(line), 1, 99, 1, 120.0f, 60.0f, 20.5f, -3.0f, 300, 800.0f);
        CHECK(cam_row_parse(line, &r));
        CHECK(r.w_px == 12 && r.h_px == 11);
        CHECK(r.matched_px == 130u);
        CHECK_NEAR(r.dt_ms, 80.0f, 1e-3);
        CHECK(r.proc_us == 12345u);
        CHECK(r.stage == 0);
        CHECK_NEAR(r.conf, -1.0f, 1e-6);
        for (int k = 0; k < 3; k++) {
            CHECK_NEAR(r.b_conf[k], -1.0f, 1e-6);
        }
        CHECK(r.loop_us == 0u);
    }

    TEST_GROUP("a real ok = 0 row, as the learned detector prints it");
    {
        const char *row0 = "73920,0,0.00,0.00,0.00,0.00,0,0,0,0.00,-1,0,0,4,0.00,0.00,-1,0.00,0.00,-1,"
                           "0.00,0.00,-1,80.0,9816,1,-1,-1,-1,-1,1180,310,0,43000,2";
        CHECK(cam_row_parse(row0, &r));
        CHECK(!r.detected);
        CHECK(r.blobs == 0 && r.w_px == 0 && r.h_px == 0 && !r.clipped);
        CHECK(r.matched_px == 4u);
        CHECK_NEAR(r.range_mm, -1.0f, 1e-6);
        CHECK_NEAR(r.conf, -1.0f, 1e-6);
        CHECK(r.loop_us == 43000u);
    }

    TEST_GROUP("the camera's clock is exact beyond 2^24 ms");
    {
        make_row(line, sizeof(line), 2, 16777217LL, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0, -1.0f);
        CHECK(cam_row_parse(line, &r));
        CHECK(r.cam_t_ms == 16777217u);        /* 2^24 + 1: a float cannot hold it */
        make_row(line, sizeof(line), 2, 4000000001LL, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0, -1.0f);
        CHECK(cam_row_parse(line, &r));
        CHECK(r.cam_t_ms == 4000000001u);
    }

    TEST_GROUP("each new range check refuses");
    {
        /* Replace column `col` of a valid v2 row with `value`. */
        static const struct { int col; const char *value; const char *what; } bad[] = {
            {7, "2001", "w_px"}, {8, "-1", "h_px"}, {9, "2.5", "fill"}, {11, "2", "clipped"},
            {11, "0.5", "clipped fractional"}, {12, "10001", "blobs"}, {13, "1000001", "matched_px"},
            {10, "-2", "range below -1"}, {10, "20000000", "range"},
            {14, "91", "b2 az"}, {15, "-91", "b2 el"}, {16, "-2", "b2 range"},
            {17, "91", "b3 az"}, {19, "20000000", "b3 range"}, {20, "-91", "b4 az"}, {22, "-3", "b4 range"},
            {23, "-1", "dt_ms"}, {23, "2000000", "dt_ms"}, {24, "-1", "proc_us"}, {24, "200000000", "proc_us"},
            {25, "16", "stage"}, {25, "-1", "stage"}, {26, "1.5", "conf"}, {26, "-2", "conf"},
            {27, "2", "b2 conf"}, {28, "-1.5", "b3 conf"}, {29, "9", "b4 conf"},
            {33, "-1", "loop_us"}, {33, "200000000", "loop_us"},
        };
        for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
            make_row(line, sizeof(line), 2, 1000, 1, 80.0f, 60.0f, 0.5f, 0.0f, 50, 900.0f);
            char out[512];
            size_t o = 0;
            int col = 0;
            for (const char *p = line;; p++) {
                if (col == bad[i].col) {
                    o += (size_t)snprintf(out + o, sizeof(out) - o, "%s", bad[i].value);
                    while (*p && *p != ',') {
                        p++;
                    }
                    col++;
                    if (!*p) {
                        break;
                    }
                    out[o++] = ',';
                    continue;
                }
                if (!*p) {
                    break;
                }
                out[o++] = *p;
                if (*p == ',') {
                    col++;
                }
            }
            out[o] = '\0';
            CHECK_MSG(!cam_row_parse(out, &r), "accepted bad %s: %s", bad[i].what, out);
        }
        /* The untouched row still parses, so the loop above rejects by the column, not by the harness. */
        make_row(line, sizeof(line), 2, 1000, 1, 80.0f, 60.0f, 0.5f, 0.0f, 50, 900.0f);
        CHECK(cam_row_parse(line, &r));
    }

    TEST_REPORT();
}
