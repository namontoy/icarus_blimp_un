/* imu.h - gyro + accelerometer driver interface.
 *
 * WHICH PART is unresolved (board_config.h's I2C section explains why in
 * detail - docs/02 section 6 C.7 vs docs/07 section 5.1's BNO085 option).
 * This interface is written to be honest about that: init() only probes the
 * bus address and reports presence; read() is a stub until a specific part
 * is confirmed and its register map is implemented.
 *
 * Consumes/produces core's own types (blimp/types.h) directly - no firmware-
 * private intermediate struct - per the task contract that drivers must
 * produce/consume blimp_imu_sample_t.
 */
#ifndef BLIMP_DRIVER_IMU_H
#define BLIMP_DRIVER_IMU_H

#include "esp_err.h"

#include "blimp/types.h"

/* Installs nothing on the I2C bus itself (main.c's board_i2c_init() owns
 * that); probes BLIMP_BOARD_IMU_I2C_ADDR and logs whether a device ACKed.
 * Always returns ESP_OK so a missing/not-yet-populated sensor does not block
 * boot - blimp_imu_read() reports its own failure per call via the sample's
 * *_valid flags and its own return code. */
esp_err_t blimp_imu_init(void);

/* Fills sample->gyro[3], sample->accel[3] and sets sample->gyro_valid /
 * sample->accel_valid. Does NOT touch sample->t_us, sample->mag*, sample->
 * baro*, sample->range* - imu.c/baro.c/mag.c each own only their own slice
 * of a shared blimp_imu_sample_t that main.c assembles once per control
 * tick (see main.c's control_task).
 *
 * NULL-safe: returns ESP_ERR_INVALID_ARG and touches nothing if sample is
 * NULL, matching core's own NULL-safety convention (pid.c) even though this
 * file is firmware, not core.
 *
 * Currently an HONEST STUB: returns ESP_ERR_NOT_SUPPORTED and sets both
 * *_valid flags false unconditionally. See imu.c's header comment for
 * exactly what remains to implement once the real part is known. */
esp_err_t blimp_imu_read(blimp_imu_sample_t *sample);

#endif /* BLIMP_DRIVER_IMU_H */
