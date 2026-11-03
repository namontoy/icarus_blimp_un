/*
 * Frame-size benchmark on the host: the same table the board runs with
 * CONFIG_OPTFLOW_BENCH (main/flow_bench.c), timed on this PC.
 *
 * The PC is far faster than the camera's core, so its times are only useful
 * as ratios. The "est. ESP32" column scales each row by the one measurement
 * we have from a board: the QQVGA row's configuration took 49-52 ms on the
 * AI-Thinker at 240 MHz (sdkconfig.defaults), taken here as 50.5 ms. The
 * ESP32-S3 runs the same Xtensa instruction set at the same 240 MHz, so the
 * estimate is a reasonable first guess for it too - and only a guess: run the
 * board benchmark before trusting a row near the budget.
 *
 *   make -C firmware/projects/optflow_cam/host bench
 */
#define _POSIX_C_SOURCE 199309L   /* clock_gettime under -std=c11 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "../main/flow_bench.h"

#define REF_ESP32_MS 50.5f   /* QQVGA 16/7/5 on the AI-Thinker, measured */

static int64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

int main(int argc, char **argv)
{
    flow_bench_scene_t scene = {
        .height_mm = 2000.0f, .hfov_deg = 58.0f, .period_ms = 80.0f,
        .speed_mm_s = 300.0f, .noise = 2.0f, .trials = 24,
    };
    if (argc > 1) scene.speed_mm_s = strtof(argv[1], NULL);
    if (argc > 2) scene.noise = strtof(argv[2], NULL);

    int maxw = 0, maxh = 0;
    for (const flow_bench_case_t *c = flow_bench_cases; c->width; c++) {
        if (c->width > maxw) maxw = c->width;
        if (c->height > maxh) maxh = c->height;
    }
    uint8_t *prev = malloc((size_t)maxw * (size_t)maxh);
    uint8_t *cur  = malloc((size_t)maxw * (size_t)maxh);
    if (!prev || !cur) return 1;

    printf("# height %.0f mm, h-FOV %.0f deg, period %.0f ms, floor speed %.0f mm/s, "
           "noise %.1f counts, %d trials per row\n",
           scene.height_mm, scene.hfov_deg, scene.period_ms, scene.speed_mm_s,
           scene.noise, scene.trials);
    printf("%-6s %7s %3s %3s %4s %6s %8s %7s %6s %8s %9s %7s %7s %7s %6s %5s\n",
           "frame", "size", "blk", "srch", "grid", "mm/px", "range", "patch",
           "Mops", "host ms", "est.ESP32", "rms mm", "rms px", "bias mm", "valid", "ok");

    float ref_host_ms = 0.0f;
    for (const flow_bench_case_t *c = flow_bench_cases; c->width; c++) {
        flow_bench_result_t r;
        if (!flow_bench_run(c, &scene, prev, cur, now_us, &r)) {
            printf("%-6s %3dx%-3d %3d %3d %3dx%d   invalid for flow_match\n",
                   c->name, c->width, c->height, c->block, c->search, c->grid, c->grid);
            continue;
        }
        /* Fastest of the trials: the host's scheduler only ever adds time. */
        if (ref_host_ms == 0.0f) ref_host_ms = r.ms_min;
        printf("%-6s %3dx%-3d %3d %3d %3dx%d %6.2f %5.0fmm/s %5.0fmm %6.2f %8.3f %7.1fms %7.3f %7.4f %+7.3f %6.1f %2d/%d\n",
               c->name, c->width, c->height, c->block, c->search, c->grid, c->grid,
               r.mm_per_px, r.range_mm_s, r.patch_mm, r.mops, r.ms_min,
               REF_ESP32_MS * r.ms_min / ref_host_ms,
               r.err_rms_mm, r.err_rms_px, r.err_mean_mm, r.valid_mean,
               r.n_ok, scene.trials);
    }
    free(prev);
    free(cur);
    return 0;
}
