/* test_cam_row.c - target_cam's CSV row parser (main/cam_row.c), on the
 * computer.
 *
 * The rows are made here with the printf format strings of target_cam's own
 * row (target_main.c on namontoy's target-cam-learned-detector branch, the
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

    TEST_REPORT();
}
