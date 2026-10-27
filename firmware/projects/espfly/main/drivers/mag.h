/* mag.h - magnetometer driver interface.
 *
 * docs/07 section 5.2: adding a magnetometer at all is the single highest-
 * leverage change over every reference design in the corpus (they are all
 * MPU6050-class with no magnetometer, so yaw is pure unbounded gyro
 * integration). Whether one is even POPULATED on the professor's board is
 * still an open question (docs/02 section 6 C.8) - this driver must degrade
 * gracefully (mag_valid = false) if not, exactly as blimp_params_t.use_mag
 * and the estimator's mag_healthy flag (estimator.h) already assume.
 */
#ifndef BLIMP_DRIVER_MAG_H
#define BLIMP_DRIVER_MAG_H

#include "esp_err.h"

#include "blimp/types.h"

/* Probes BLIMP_BOARD_MAG_I2C_ADDR. Always returns ESP_OK even if nothing
 * ACKs - a magnetometer being absent is a real, expected possibility here
 * (see the header comment above), not a fault to halt boot over. */
esp_err_t blimp_mag_init(void);

/* Fills sample->mag[3] and sample->mag_valid only.
 *
 * HONEST STUB: returns ESP_ERR_NOT_SUPPORTED, mag_valid = false
 * unconditionally, until a specific part (LIS3MDL is the docs/03 section 1.2
 * recommendation if one must be added) and its register map are confirmed
 * and implemented. Hard/soft-iron calibration must happen on the FULLY
 * ASSEMBLED gondola (docs/07 section 5.2 item 2, section 7 step 4) - motor
 * current and battery leads distort the field - so calibration constants
 * belong in blimp_params_t (a live-tunable, per-vehicle value), never
 * hard-coded here. */
esp_err_t blimp_mag_read(blimp_imu_sample_t *sample);

#endif /* BLIMP_DRIVER_MAG_H */
