/*
 * 'mag' console command and the magnetometer driver behind it: an HMC5883L
 * (0x1E) or a QMC5883L (0x0D), found automatically.
 *
 * The driver is attach / read / detach; the console command and the sensor
 * stream (sensor_stream.c) are both printing loops on top of it. A caller
 * that runs while the stream holds the chip takes the lease first, see
 * sensor_stream_pause() in sensor_stream.h.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register the 'mag' command with the console. */
void register_mag(void);

typedef struct mag_dev mag_dev_t;

typedef enum {
    MAG_CHIP_HMC5883L,
    MAG_CHIP_QMC5883L,
} mag_chip_t;

typedef struct {
    float mx_gauss, my_gauss, mz_gauss;   /* 1 gauss = 100 uT               */
    float heading_deg;                    /* atan2(y, x), 0..360, chip flat
                                             with its X axis forward        */
    bool  overflow;                       /* field beyond the range: the
                                             numbers are clipped, not
                                             measurements                   */
} mag_reading_t;

typedef struct {
    mag_chip_t  chip;
    const char *name;           /* "HMC5883L" or "QMC5883L"                 */
    int         addr;
    float       range_gauss;
    float       lsb_per_gauss;
} mag_info_t;

/**
 * Find a magnetometer, reset or reconfigure it and start continuous
 * measurement. addr < 0 probes 0x1E and then 0x0D.
 *
 * @return ESP_ERR_NOT_FOUND, quietly, when no address answers; ESP_FAIL if
 *         the device could not be added to the bus; configuration errors
 *         are logged here.
 */
esp_err_t mag_attach(int addr, mag_dev_t **out);

void mag_info(const mag_dev_t *d, mag_info_t *info);

/**
 * The newest measurement. Waits up to timeout_ms for the chip's data-ready
 * bit (0 checks once), so a frozen sensor is a reported error and not a
 * plausible-looking repeated row.
 *
 * @return ESP_ERR_TIMEOUT when no new sample arrived in time (logged only
 *         when timeout_ms > 0), bus errors otherwise.
 */
esp_err_t mag_read(mag_dev_t *d, int timeout_ms, mag_reading_t *r);

/** Remove the device from the bus and free the handle. NULL is fine. */
void mag_detach(mag_dev_t *d);

#ifdef __cplusplus
}
#endif
