/* test_flow_row.c - optflow_cam's CSV row parser (main/flow_row.c), on the
 * computer.
 *
 * The rows are made with the printf format of optflow_cam's own row
 * (firmware/projects/optflow_cam/main/optflow_main.c), so this checks the
 * format the camera really prints. Since 2026-11-02 one UART can carry either
 * camera firmware, so the two parsers must refuse each other's rows: a flow
 * row read as a balloon would turn the blimp. Run by tools/fw_host_test.py,
 * which tools/verify.py calls.
 */
#include <stdio.h>
#include <string.h>

#include "cam_row.h"
#include "flow_row.h"
#include "test_util.h"

/* One row as optflow_cam prints it. */
static void make_row(char *buf, size_t size, long long t_ms, int ok, float dx, float dy, float spread,
                     int valid, int total, int flat, int edge, int ambig)
{
    snprintf(buf, size, "%lld,%d,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.3f,%d,%d,%d,%d,%d,%.1f,%lld",
             t_ms, ok, (double)dx, (double)dy, (double)(-dx * 6.9f / 0.08f), (double)(-dy * 6.9f / 0.08f),
             12.5, -3.25, (double)spread, valid, total, flat, edge, ambig, 80.0, 51234LL);
}

int main(void)
{
    printf("test_flow_row\n");
    char line[256];
    flow_row_t r;
    cam_row_t c;

    TEST_GROUP("a measured row, as optflow_cam prints it");
    {
        make_row(line, sizeof(line), 5391506, 1, -0.123f, 0.456f, 0.044f, 11, 25, 14, 0, 0);
        CHECK(flow_row_parse(line, &r));
        CHECK(r.ok);
        CHECK(r.cam_t_ms == 5391506u);
        CHECK_NEAR(r.dx_px, -0.123f, 1e-4);
        CHECK_NEAR(r.dy_px, 0.456f, 1e-4);
        CHECK_NEAR(r.spread_px, 0.044f, 1e-4);
        CHECK(r.valid == 11 && r.total == 25 && r.flat == 14 && r.edge == 0 && r.ambig == 0);
        CHECK_NEAR(r.dt_ms, 80.0f, 1e-3);
        CHECK(r.proc_us == 51234u);
        CHECK(!cam_row_parse(line, &c));    /* never a balloon */
    }

    TEST_GROUP("a refused pair: ok 0, the shift zeroed, the reasons counted");
    {
        make_row(line, sizeof(line), 1000, 0, 0.0f, 0.0f, 0.0f, 2, 25, 23, 0, 0);
        CHECK(flow_row_parse(line, &r));
        CHECK(!r.ok);
        CHECK(r.dx_px == 0.0f && r.dy_px == 0.0f);
        CHECK(r.valid == 2 && r.flat == 23);
        CHECK(!cam_row_parse(line, &c));
    }

    TEST_GROUP("a clock past 2^24 ms keeps its last digit");
    {
        make_row(line, sizeof(line), 16777217LL, 1, 0.5f, 0.0f, 0.01f, 25, 25, 0, 0, 0);
        CHECK(flow_row_parse(line, &r));
        CHECK(r.cam_t_ms == 16777217u);
    }

    TEST_GROUP("lines that are not a flow row");
    {
        CHECK(!flow_row_parse("", &r));
        CHECK(!flow_row_parse("# ESP32-CAM downward optical flow", &r));
        CHECK(!flow_row_parse("ets Jul 29 2019 12:21:46", &r));
        CHECK(!flow_row_parse("I (312) optflow: flash LED on GPIO4 driven low", &r));
        make_row(line, sizeof(line), 1000, 1, 0.1f, 0.1f, 0.01f, 25, 25, 0, 0, 0);
        line[strlen(line) / 2] = '\0';
        CHECK(!flow_row_parse(line, &r));              /* half a row */
        make_row(line, sizeof(line), 1000, 1, 0.1f, 0.1f, 0.01f, 25, 25, 0, 0, 0);
        snprintf(line + strlen(line), sizeof(line) - strlen(line), ",7");
        CHECK(!flow_row_parse(line, &r));              /* a column too many */
        /* target_cam's rows: 25 and 35 columns, never flow */
        CHECK(!flow_row_parse("123456,1,95.25,50.10,-12.34,4.56,120,12,11,0.91,1500,0,1,130,"
                              "0.00,0.00,-1,0.00,0.00,-1,0.00,0.00,-1,80.0,12345", &r));
    }

    TEST_GROUP("rows the matcher cannot print");
    {
        make_row(line, sizeof(line), 1000, 1, 17.0f, 0.0f, 0.01f, 25, 25, 0, 0, 0);
        CHECK(!flow_row_parse(line, &r));              /* past any search radius */
        make_row(line, sizeof(line), 1000, 0, 0.3f, 0.0f, 0.0f, 2, 25, 23, 0, 0);
        CHECK(!flow_row_parse(line, &r));              /* refused, yet a shift */
        make_row(line, sizeof(line), 1000, 1, 0.3f, 0.0f, 0.01f, 2, 25, 23, 0, 0);
        CHECK(!flow_row_parse(line, &r));              /* measured from two blocks */
        make_row(line, sizeof(line), 1000, 1, 0.3f, 0.0f, 0.01f, 11, 25, 13, 0, 0);
        CHECK(!flow_row_parse(line, &r));              /* the blocks do not add up */
        make_row(line, sizeof(line), 1000, 2, 0.3f, 0.0f, 0.01f, 25, 25, 0, 0, 0);
        CHECK(!flow_row_parse(line, &r));              /* ok is 0 or 1 */
        make_row(line, sizeof(line), 1000, 1, 0.3f, 0.0f, -0.01f, 25, 25, 0, 0, 0);
        CHECK(!flow_row_parse(line, &r));              /* a negative spread */
        make_row(line, sizeof(line), 1000, 1, 0.3f, 0.0f, 0.01f, 25, 0, 0, 0, 0);
        CHECK(!flow_row_parse(line, &r));              /* no blocks tried */
    }

    TEST_REPORT();
}
