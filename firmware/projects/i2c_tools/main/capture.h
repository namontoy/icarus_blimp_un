/*
 * capture.h - a fixed-length recording of the control loop's log records,
 * kept in PSRAM and read back over the link or the console.
 *
 * The control task pushes one log_record_t per tick (ctl_task.c, the log
 * stage); this module stores them in a buffer allocated once at boot for
 * CONFIG_CAPTURE_MAX_SECONDS of ticks. A capture is linear, not a ring: it
 * starts empty, fills for the requested number of seconds (or until stopped)
 * and then holds its records until the next start or a clear. Records below
 * `count` never change while the capture id holds, which is what lets the
 * link read them out while the loop keeps running, with no lock on the data
 * path (docs/control-loop-plan.md D7, D8).
 *
 * Why RAM and not flash: the S3-Zero's 4 MB flash holds the app and little
 * else, a flash write stalls the core the loop runs on, and the bench
 * sessions this serves are a minute long. 72 bytes at 50 Hz is 3.6 kB/s:
 * two minutes is 432 kB of the 2 MB PSRAM.
 *
 * Reading it out: LINK_PKT_LOG_CMD over UDP (net_link.c; the host side is
 * software/blimp_link.py 'capture' and the ground station's Bench page), or
 * 'capture --head N' on the console for a sanity check. Opening the USB port
 * resets the chip, so a capture is fetched over the air, never over USB.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "ctl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 'capture --start' and LOG_CMD START with seconds = 0 record this long. */
#define CAPTURE_DEFAULT_SECONDS 60

typedef struct {
    uint8_t  state;             /* LINK_LOG_IDLE, LINK_LOG_RUNNING, LINK_LOG_DONE (link_protocol.h) */
    bool     have_buffer;       /* false: the allocation failed at boot          */
    bool     in_psram;          /* false: the smaller internal-RAM fallback      */
    uint16_t capture_id;        /* increments per start; chunks carry it         */
    uint32_t count;             /* records stored                                */
    uint32_t capacity;          /* records the buffer holds                      */
    uint32_t wanted;            /* records this capture stops at                 */
    uint32_t t_start_us;        /* the first record's clock                      */
    uint32_t dropped;           /* pushes lost to a slow reader, diagnostics     */
    /* The scales the decoder needs, snapshotted at start (link_log_info_t). */
    uint16_t imu_accel_g;
    uint16_t imu_gyro_dps;
    float    mag_lsb_per_gauss;
    uint8_t  mag_chip;
    uint8_t  baro_chip;
} capture_info_t;

/**
 * Allocate the buffer: CONFIG_CAPTURE_MAX_SECONDS of records in PSRAM, or a
 * ten-second fallback in internal RAM when there is none. Call before the
 * control task starts. Failure is logged and leaves 'capture' reporting
 * "no buffer"; nothing else depends on it.
 */
esp_err_t capture_init(void);

/**
 * Start a new capture of `seconds` (0: CAPTURE_DEFAULT_SECONDS). Discards
 * whatever the buffer held. ESP_ERR_INVALID_STATE while one is running,
 * ESP_ERR_NO_MEM without a buffer, ESP_ERR_INVALID_SIZE when the length does
 * not fit the buffer: refused rather than shortened, so a 300 s test asked
 * of a 120 s buffer fails at the start instead of ending early (2026-10-28).
 */
esp_err_t capture_start(uint32_t seconds);

/** End the running capture early; its records stay. Returns true if one was running. */
bool capture_stop(void);

/** Back to idle, nothing stored. A running capture is stopped first. */
void capture_clear(void);

/**
 * Store one record. Control task only, every tick: it copies 72 bytes and
 * touches nothing else, so it costs a few microseconds. Ignored unless a
 * capture is running.
 */
void capture_push(const log_record_t *r);

void capture_info(capture_info_t *out);

/**
 * Copy up to `max` records starting at `first` into `out`. Returns how many
 * were copied: 0 when `first` is at or past `count`, or when a new capture
 * started during the copy (the records would be from two captures).
 * `*capture_id` gets the id the records belong to.
 */
uint32_t capture_read(uint32_t first, uint32_t max, log_record_t *out, uint16_t *capture_id);

/** Register the 'capture' command with the console. */
void register_capture(void);

#ifdef __cplusplus
}
#endif
