/*
 * 'baro' console command: decoded barometer readings from a BMP085/BMP180,
 * BMP280 or BME280, found automatically at 0x77 or 0x76.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register the 'baro' command with the console. */
void register_baro(void);

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
