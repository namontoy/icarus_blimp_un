/*
 * cam_link.h - the forward ESP32-CAM's detections over UART1, and the 'cam'
 * console command.
 *
 * The camera runs namontoy's target_cam (firmware/projects/target_cam/), which
 * prints one CSV row per processed frame, 12.5 a second, on its UART0 at
 * 115200 for the PC and copies every data row to its UART1 (TX IO14, same
 * baud) for this board:
 *
 *   t_ms,ok,cx_px,cy_px,az_deg,el_deg,area_px,w_px,h_px,fill,range_mm,...
 *
 * 25 columns in its CSV v1, 35 in v2 (the learned detector's, which ends in
 * the version number). The control loop reads the first eleven (they are the
 * same in both) through cam_link_latest(); every column the UDP link relays
 * (cam_row.h, LINK_PKT_CAMERA) goes through cam_link_latest_row(). Lines starting with '#' are its banner and replies; the
 * boot ROM's log and anything else that is not a row are counted and
 * dropped. The wiring (the Cableado, docs/inter-board-link-plan.md): CAM IO14
 * -> 470 ohm -> S3 GPIO6, S3 GPIO2 -> 470 ohm -> CAM IO13, and the grounds
 * joined. target_cam does not read its IO13 yet and sends no '#' lines on
 * UART1, so on that wiring 'cam --tx' reaches nothing; it is kept for the
 * camera's UART0 pads (U0T GPIO1, U0R GPIO3), which the 2026-10-30 build was
 * first wired to.
 *
 * The same wires can carry optflow_cam's rows instead (the module flashed
 * with the optical-flow firmware and pointed down, 2026-11-02): 16 columns,
 * parsed by flow_row.c into their own mailbox (cam_link_latest_flow()), which
 * the log stage records; 'cam' shows them and 'cam --zero' restarts the sum
 * it prints.
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
#include "cam_row.h"
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
    /* optflow_cam's rows (flow_row.h); the counts above are target_cam's only,
       so 'hold follow' never mistakes a flow row for the forward camera */
    uint32_t flow_rows;         /* parsed flow rows                              */
    uint32_t flow_rows_ok;      /* of which ok = 1                               */
    uint32_t flow_rows_last_s;  /* in the last whole second                      */
    uint32_t flow_ok_last_s;
    uint32_t flow_last_row_us;  /* S3 clock at the last flow row, 0: none yet    */
} cam_link_status_t;

/**
 * Install UART1 on CONFIG_CAM_FWD_UART_RX_GPIO / _TX_GPIO and start the
 * reader. Call once, before ctl_task_start(). Not fatal when it fails: the
 * loop then sees no rows, and 'cam' says why.
 */
esp_err_t cam_link_start(void);

/** The latest row; false until one has arrived. Any task, no waiting. */
bool cam_link_latest(cam_result_t *out);

/**
 * The same row with every column kept, for net_link.c's CAMERA packet. Its
 * frame_seq and t_us are the ones cam_link_latest() reports for that row (one
 * parse, one stamp), so the ground station's rows line up with the control
 * loop's. False until a row has arrived.
 */
typedef struct {
    uint32_t  frame_seq;    /* rows parsed since boot                        */
    uint32_t  t_us;         /* S3 clock when the row's newline arrived       */
    cam_row_t row;
} cam_link_row_t;

bool cam_link_latest_row(cam_link_row_t *out);

/**
 * The latest optflow_cam row (2026-11-02: the forward camera's module run
 * pointed down, for the surface tests). Its own mailbox, never the forward
 * camera's; false until a flow row has arrived. Any task, no waiting.
 */
bool cam_link_latest_flow(cam_flow_t *out);

/** core/'s bearing = this x the camera's az: CONFIG_CAM_FWD_BEARING_SIGN,
    or what 'cam --sign' set since boot. */
int cam_link_bearing_sign(void);

void cam_link_status(cam_link_status_t *out);

/** The 'cam' console command. */
void register_cam(void);

#ifdef __cplusplus
}
#endif
