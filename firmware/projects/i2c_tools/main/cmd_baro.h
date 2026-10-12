/*
 * 'baro' console command: decoded BMP085/BMP180 readings.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register the 'baro' command with the console. */
void register_baro(void);

/**
 * Read air temperature in degrees Celsius from the BMP180.
 *
 * Adds and removes its own device on the shared bus, so it is safe to call
 * from another command. Used by 'ping' to correct the speed of sound.
 */
esp_err_t baro_read_temperature_c(float *temp_c);

#ifdef __cplusplus
}
#endif
