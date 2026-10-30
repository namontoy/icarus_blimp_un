/*
 * 'imu' console command and the MPU-6050 driver behind it.
 *
 * The driver is three functions, attach / read / detach. The console command
 * is a printing loop on top of them, and so is the sensor stream
 * (sensor_stream.c) that feeds the ground station: same configuration, same
 * scaling, so what the page plots is what 'imu -n 3' prints.
 *
 * Bus sharing: each attach adds one device to the shared bus
 * (cmd_i2ctools.h) and configures the chip from scratch; detach removes it.
 * A caller that runs while the stream holds the chip must take the lease
 * first, see sensor_stream_pause() in sensor_stream.h.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register the 'imu' command with the console. */
void register_imu(void);

typedef struct imu_dev imu_dev_t;

typedef struct {
    float ax_g, ay_g, az_g;         /* accelerometer, body axes               */
    float gx_dps, gy_dps, gz_dps;   /* gyroscope                              */
    float temp_c;                   /* die temperature, datasheet conversion  */
    int16_t raw_acc[3];             /* the counts the floats came from, x y z: */
    int16_t raw_gyro[3];            /* what the flight log stores, scaled by   */
    int16_t raw_temp;               /* the ranges imu_ranges() reports         */
} imu_reading_t;

/**
 * Find the MPU-6050, wake it and set its ranges.
 *
 * @param addr      7-bit address, or -1 for the default 0x68
 * @param accel_fs  AFS_SEL, 0..3 = +/-2, 4, 8, 16 g
 * @param gyro_fs   FS_SEL, 0..3 = +/-250, 500, 1000, 2000 dps
 * @return ESP_ERR_NOT_FOUND, quietly, if nothing answers at the address (the
 *         caller decides whether that deserves a log line); other errors are
 *         logged here.
 */
esp_err_t imu_attach(int addr, int accel_fs, int gyro_fs, imu_dev_t **out);

/** Ranges the chip reported back after attach, for a printed header. */
void imu_ranges(const imu_dev_t *d, int *accel_g, int *gyro_dps);

/** Bandwidth of the chip's digital low-pass filter as attached, Hz (21). */
int imu_dlpf_hz(const imu_dev_t *d);

int imu_addr(const imu_dev_t *d);

/** One coherent sample of accelerometer, temperature and gyroscope. */
esp_err_t imu_read(imu_dev_t *d, imu_reading_t *r);

/** Remove the device from the bus and free the handle. NULL is fine. */
void imu_detach(imu_dev_t *d);

#ifdef __cplusplus
}
#endif
