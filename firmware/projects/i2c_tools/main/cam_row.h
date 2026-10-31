/*
 * cam_row.h - one CSV row of namontoy's target_cam (the forward ESP32-CAM's
 * firmware, branches target-cam-*), parsed. Portable C with no ESP-IDF, so
 * tools/fw_host_test.py checks it on the host
 * (firmware/projects/i2c_tools/host/test_cam_row.c); cam_link.c feeds it the
 * lines that arrive on UART1.
 *
 * The row, as target_cam prints it per frame:
 *
 *   t_ms,ok,cx_px,cy_px,az_deg,el_deg,area_px,w_px,h_px,fill,range_mm,
 *   clipped,blobs,matched_px,b2_az,b2_el,b2_range_mm,b3_az,b3_el,
 *   b3_range_mm,b4_az,b4_el,b4_range_mm,dt_ms,proc_us            (CSV v1: 25)
 *   ...,stage,conf,b2_conf,b3_conf,b4_conf,t_px_us,t_b_us,t_c_us,
 *   loop_us,csv_v                                     (v2: 35, the last is 2)
 *
 * w_px and h_px are the detection's bounding box, not the frame. az is
 * atan((cx - W/2) / f): + to the RIGHT of the image centre; el + above it.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int      version;       /* 1 or 2                                     */
    uint32_t cam_t_ms;      /* the camera's own boot clock                */
    bool     detected;      /* ok = 1                                     */
    float    cx_px, cy_px;  /* centroid, origin top-left                  */
    float    az_deg, el_deg;
    float    area_px;
    float    range_mm;      /* from the apparent size; -1 when unknown    */
} cam_row_t;

/**
 * Parse one line, without its line ending. True only for a whole row of a
 * known version with every read column in range; anything else (a '#'
 * line, the boot ROM's log, a half row, an image frame's bytes) is false and
 * leaves *out unspecified.
 */
bool cam_row_parse(const char *line, cam_row_t *out);
