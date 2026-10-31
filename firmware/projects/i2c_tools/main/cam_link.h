/*
 * cam_link.h - the forward ESP32-CAM's detections over UART1, and the 'cam'
 * console command.
 *
 * The camera runs namontoy's target_cam (branches target-cam-*), which prints
 * one CSV row per processed frame, 12.5 a second, on its UART0 at 115200:
 *
 *   t_ms,ok,cx_px,cy_px,az_deg,el_deg,area_px,w_px,h_px,fill,range_mm,...
 *
 * 25 columns in its CSV v1, 35 in v2 (the learned detector's, which ends in
 * the version number). Only the first eleven are read here, and they are
 * the same in both. Lines starting with '#' are its banner and replies; the
 * boot ROM's log and anything else that is not a row are counted and
 * dropped. The wiring (the Cableado, docs/inter-board-link-plan.md): CAM U0T
 * (GPIO1) -> 470 ohm -> S3 GPIO6, S3 GPIO2 -> 470 ohm -> CAM U0R (GPIO3), and
 * the grounds joined.
 *
 * A reader task (core 0, below the link) assembles lines and keeps the latest
 * row as a cam_result_t under a spinlock; the control task peeks it every
 * tick (ctl_types.h, the camera mailboxes). frame_seq counts rows, t_us is
 * the S3's clock when the row's newline arrived: the CAM's own t_ms is its
 * boot clock, not ours.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "ctl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     running;           /* the UART and the task are up                  */
    int      rx_gpio, tx_gpio;  /* tx -1: receive only                           */
    int      baud;
    uint32_t bytes;             /* received since boot                           */
    uint32_t rows;              /* parsed rows                                   */
    uint32_t rows_detected;     /* of which ok = 1                               */
    uint32_t info_lines;        /* lines starting with '#'                       */
    uint32_t rejected;          /* lines that were neither                       */
    uint32_t overflows;         /* lines longer than the buffer, dropped         */
    uint32_t rows_last_s;       /* rows in the last whole second                 */
    uint32_t last_row_us;       /* S3 clock at the last row, 0: none yet         */
    int      csv_version;       /* 1 or 2, from the column count                 */
    bool     streaming_off;     /* the camera said "# streaming off" last        */
} cam_link_status_t;

/**
 * Install UART1 on CONFIG_CAM_FWD_UART_RX_GPIO / _TX_GPIO and start the
 * reader. Call once, before ctl_task_start(). Not fatal when it fails: the
 * loop then sees no rows, and 'cam' says why.
 */
esp_err_t cam_link_start(void);

/** The latest row; false until one has arrived. Any task, no waiting. */
bool cam_link_latest(cam_result_t *out);

/** core/'s bearing = this x the camera's az: CONFIG_CAM_FWD_BEARING_SIGN,
    or what 'cam --sign' set since boot. */
int cam_link_bearing_sign(void);

void cam_link_status(cam_link_status_t *out);

/** The 'cam' console command. */
void register_cam(void);

#ifdef __cplusplus
}
#endif
