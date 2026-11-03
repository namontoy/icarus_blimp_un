/*
 * cam_link.h - the two ESP32-CAMs over UART: the forward camera's detections
 * on UART1, the downward camera's optical flow on UART2 (since 2026-11-02),
 * and the 'cam' console command.
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
 * The downward camera runs optflow_cam (firmware/projects/optflow_cam/,
 * CONFIG_OPTFLOW_S3_UART), which copies every data row to its IO14 the same
 * way: 16 columns, parsed by flow_row.c into their own mailbox
 * (cam_link_latest_flow()), which the log stage records; 'cam' shows them and
 * 'cam --zero' restarts the sum it prints. Its wiring (the link plan's
 * CAM-DOWN, on UART2, which has no IO-MUX pins and so is always routed
 * through the GPIO matrix): CAM IO14 -> 470 ohm -> S3 GPIO18, S3 GPIO17 ->
 * 470 ohm -> CAM IO13, the grounds joined. optflow_cam does not read IO13,
 * so GPIO17 is optional wiring; 'cam --tx-down' writes to it.
 *
 * On 2026-11-02 the flow was first run with the forward module flashed with
 * optflow_cam on UART1's wires. Both ports therefore offer every line to both
 * parsers: the column count decides the mailbox, not the port, so a swapped
 * pair of cables still works and a flow row still never reads as a balloon.
 *
 * One reader task per port (core 0, below the link) assembles lines and keeps
 * the latest row of each kind under one spinlock; the control task peeks them
 * every tick (ctl_types.h, the camera mailboxes). frame_seq counts rows, t_us is
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

/* The two ports, in cam_link_status_t.port[]. */
enum { CAM_PORT_FWD = 0, CAM_PORT_DOWN = 1, CAM_PORTS = 2 };

typedef struct {
    bool     running;           /* the UART and its reader task are up           */
    int      rx_gpio, tx_gpio;  /* tx -1: receive only                           */
    int      baud;
    uint32_t bytes;             /* received since boot                           */
    uint32_t rows;              /* target_cam rows that came on this port        */
    uint32_t flow_rows;         /* optflow_cam rows that came on this port       */
    uint32_t info_lines;        /* lines starting with '#'                       */
    uint32_t rejected;          /* lines that were neither                       */
    uint32_t overflows;         /* lines longer than the buffer, dropped         */
    bool     streaming_off;     /* this camera said "# streaming off" last       */
} cam_port_status_t;

/* running ... streaming_off are the FORWARD port's (UART1), as they were
   before the second camera (net_link.c relays them as the forward camera's
   health); the row and flow counts are by kind, whichever port carried them;
   port[] has each port's own. */
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
    cam_port_status_t port[CAM_PORTS];
} cam_link_status_t;

/**
 * Install UART1 on CONFIG_CAM_FWD_UART_RX_GPIO / _TX_GPIO and UART2 on
 * CONFIG_CAM_DOWN_UART_RX_GPIO / _TX_GPIO, and start a reader on each. Call
 * once, before ctl_task_start(). Not fatal when it fails: a port that fails
 * stays down alone, the loop sees no rows from it, and 'cam' says so. Returns
 * the first error.
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
 * The latest optflow_cam row, from either port (the downward camera on UART2;
 * on 2026-11-02 the forward module run pointed down on UART1). Its own
 * mailbox, never the forward camera's; false until a flow row has arrived.
 * Any task, no waiting.
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
