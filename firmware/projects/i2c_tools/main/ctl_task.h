/*
 * ctl_task.h - the fixed-rate control task and the 'ctl' console command.
 *
 * One task on core 1 runs the pipeline of ctl_types.h at CTL_PERIOD_US and
 * keeps timing statistics; a low-priority task prints them once a second.
 * The stages are empty in this increment: nothing here touches the I2C bus
 * or the motors yet. Design and increments: docs/control-loop-plan.md.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 20 ms, 50 Hz. A compile-time constant, not a tunable: the rate dividers of
 * the sensor schedule, the IMU's digital low-pass setting (which must sit
 * below the 25 Hz Nyquist rate) and the log cadence all assume it, so a
 * runtime change would silently break three things.
 */
#define CTL_PERIOD_US   20000u

/*
 * Timing statistics. The window fields are reset by the printer each second;
 * the cumulative counters are not. All times from esp_timer_get_time().
 */
typedef struct {
    /* window */
    uint32_t ticks;             /* ticks in this window                       */
    uint32_t periods;           /* wake-to-wake intervals measured: ticks minus the
                                 * first one after a start or a silent wait    */
    uint32_t period_min_us;     /* wake to wake                               */
    uint32_t period_max_us;
    uint64_t period_sum_us;
    uint32_t exec_min_us;       /* wake to the end of the stages              */
    uint32_t exec_max_us;
    uint64_t exec_sum_us;
    /* cumulative */
    uint32_t total_ticks;
    uint32_t overruns;          /* exec > CTL_PERIOD_US                       */
    uint32_t missed;            /* notifications that arrived while a tick was still running: skipped ticks */
    uint32_t timer_silent;      /* waits that saw no notification for CTL_TICK_WAIT_MS */
} ctl_stats_t;

/**
 * Create the timer, the control task on core 1 and the statistics printer.
 * Call after motors_init() and the I2C bus exist (later increments use
 * both). Not fatal when it fails: the console and the link work without the
 * loop, which is how the failure would be diagnosed.
 */
esp_err_t ctl_task_start(void);

/** The last completed one-second window plus the cumulative counters. */
void ctl_get_stats(ctl_stats_t *out);

/** Print the statistics line once a second on the USB console (default on). */
void ctl_set_stats_print(bool on);

/** Register the 'ctl' command with the console. */
void register_ctl(void);

#ifdef __cplusplus
}
#endif
