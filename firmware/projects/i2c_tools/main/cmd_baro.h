/*
 * 'baro' console command and the barometer driver behind it: a BMP085/BMP180,
 * BMP280 or BME280, found automatically at 0x77 or 0x76.
 *
 * The driver is attach / read / detach; the console command and the sensor
 * stream (sensor_stream.c) are both loops on top of it. A caller that runs
 * while the stream holds the chip takes the lease first, see
 * sensor_stream_pause() in sensor_stream.h.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register the 'baro' command with the console. */
void register_baro(void);

typedef struct baro_dev baro_dev_t;

typedef enum {
    BARO_CHIP_BMP180,
    BARO_CHIP_BMP280,
    BARO_CHIP_BME280,
} baro_chip_t;

typedef struct {
    float temp_c;
    float pressure_pa;
    float altitude_m;       /* standard atmosphere, from 1013.25 hPa: only as
                               good as that assumption, and absolute        */
    float humidity_pct;     /* BME280 only, see has_humidity                 */
    bool  has_humidity;
} baro_reading_t;

typedef struct {
    baro_chip_t chip;
    const char *name;       /* "BMP085/180", "BMP280" or "BME280"            */
    int         addr;
    int         oss;        /* the 0..3 preset it was attached with          */
    int         conv_ms;    /* conversion time of one sample                 */
    int         osrs_t, osrs_p, osrs_h;   /* BMx280 oversampling multipliers
                                             (x1..x16, 0 = skipped); all 0 on
                                             a BMP180                        */
} baro_info_t;

/**
 * Find a barometer, read its calibration and configure it. addr < 0 tries
 * 0x77 and then 0x76. oss is 0 (fast) to 3 (best): the BMP180's own
 * oversampling setting, or the datasheet's low-power to ultra-high
 * resolution presets on a BMx280.
 *
 * @return ESP_ERR_NOT_FOUND, quietly, when no address answers;
 *         ESP_ERR_NOT_SUPPORTED when something answers with an unknown chip
 *         ID (logged); ESP_FAIL if the device could not be added to the bus.
 */
esp_err_t baro_attach(int addr, int oss, baro_dev_t **out);

void baro_info(const baro_dev_t *d, baro_info_t *info);

/** One sample: a forced conversion, blocking for info.conv_ms. */
esp_err_t baro_read(baro_dev_t *d, baro_reading_t *r);

/** Remove the device from the bus and free the handle. NULL is fine. */
void baro_detach(baro_dev_t *d);

/**
 * Read air temperature in degrees Celsius from whichever of the supported
 * barometers is on the bus (0x77 is tried first, then 0x76).
 *
 * Adds and removes its own device on the shared bus, so it is safe to call
 * from another command. Used by 'ping' to correct the speed of sound.
 */
esp_err_t baro_read_temperature_c(float *temp_c);

#ifdef __cplusplus
}
#endif
