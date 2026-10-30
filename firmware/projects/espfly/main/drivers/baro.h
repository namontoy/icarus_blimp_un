/* baro.h - barometric altitude driver interface.
 *
 * Cite Barometer390.cpp's update() pattern if consulting BlimpSwarm for
 * reference (docs/07 section 5.3 - NOT the older dead BarometerOld class
 * some secondary sources mis-cite). GPL-3.0 - reimplement independently,
 * do not copy (docs/03 section 8's licensing note).
 */
#ifndef BLIMP_DRIVER_BARO_H
#define BLIMP_DRIVER_BARO_H

#include "esp_err.h"

#include "blimp/types.h"

/* Probes BLIMP_BOARD_BARO_I2C_ADDR on the already-installed shared I2C bus.
 * Always returns ESP_OK; see imu.h's init() comment for why. */
esp_err_t blimp_baro_init(void);

/* Fills sample->baro_alt and sample->baro_valid only.
 *
 * HONEST STUB: returns ESP_ERR_NOT_SUPPORTED, baro_valid = false
 * unconditionally, until BMP390's (or whatever part C.7 confirms) register
 * map and pressure->altitude conversion (barometric formula, needs a
 * reference pressure - docs/02 section 1.1 flags Bogota's own pressure as
 * itself uncertain, 74.2-76.7 kPa) are implemented. Real accuracy will only
 * be roughly +/-0.25 m even once implemented (docs/03 section 1.2) - this is
 * why altitude fuses baro with a ToF range in the design (estimator.h), not
 * baro alone; note this skeleton has no tof driver yet (board_config.h). */
esp_err_t blimp_baro_read(blimp_imu_sample_t *sample);

#endif /* BLIMP_DRIVER_BARO_H */
