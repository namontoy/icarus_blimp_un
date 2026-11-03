/*
 * flow_row.h - one CSV row of optflow_cam (the downward optical-flow firmware,
 * firmware/projects/optflow_cam/), parsed. Portable C with no ESP-IDF, so
 * tools/fw_host_test.py checks it on the host
 * (firmware/projects/i2c_tools/host/test_flow_row.c); cam_link.c offers it the
 * lines on either camera port (UART1, UART2) that are not target_cam's rows.
 *
 * The row, as optflow_cam prints it per frame pair:
 *
 *   t_ms,ok,dx_px,dy_px,vx_mm_s,vy_mm_s,x_mm,y_mm,spread_px,
 *   valid,total,flat,edge,ambig,dt_ms,proc_us                       (16)
 *
 * dx, dy are the image's shift from the previous frame, image x to the right
 * and y down: the median of the blocks that matched. ok = 0 means the matcher
 * refused (fewer than three blocks matched), which is no information, not
 * zero motion; it then prints dx = dy = 0. vx, vy, x and y come from the
 * camera's own height setting and are not kept: on the S3 the ToF gives the
 * height, and the ground tools scale the pixels with it.
 *
 * The downward camera has its own UART (UART2, RX GPIO18), but the flow was
 * first run with the forward module flashed with optflow_cam on target_cam's
 * wires, and either port still takes either firmware's rows; 16 columns
 * against target_cam's 25 or 35 tell them apart, and each parser rejects the
 * other's row.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t cam_t_ms;      /* the camera's own boot clock                     */
    bool     ok;            /* 1: a measurement                                */
    float    dx_px, dy_px;  /* image shift from the previous frame; 0 when !ok */
    float    spread_px;     /* median absolute deviation of the block vectors  */
    uint8_t  valid;         /* blocks that matched                             */
    uint8_t  total;         /* blocks tried: valid + flat + edge + ambig       */
    uint8_t  flat;          /* refused: too little texture                     */
    uint8_t  edge;          /* refused: the match sat on the search boundary   */
    uint8_t  ambig;         /* refused: no clean minimum                       */
    float    dt_ms;         /* the camera: between the two frames              */
    uint32_t proc_us;       /* the camera: matching time                       */
} flow_row_t;

/**
 * Parse one line, without its line ending. True only for a whole row with
 * every column in range and the block counts adding up; anything else (a
 * target_cam row, a '#' line, the boot ROM's log, a half row) is false and
 * leaves *out unspecified.
 */
bool flow_row_parse(const char *line, flow_row_t *out);
