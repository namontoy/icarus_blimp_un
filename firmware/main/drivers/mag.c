/* mag.c - magnetometer driver.
 *
 * See imu.c's header comment for the I2C driver-choice note - applies here
 * identically. A NACK at BLIMP_BOARD_MAG_I2C_ADDR is a genuinely expected,
 * non-fatal outcome (docs/02 section 6 C.8 - a magnetometer may not be
 * populated on the professor's board at all yet), so this file logs it at
 * INFO, not WARN, unlike imu.c/baro.c.
 */
#include "mag.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "board_config.h"

static const char *TAG = "mag";
static bool s_present;

/* See imu.c's identical helper for why this is duplicated rather than
 * shared. */
static bool i2c_probe_addr(uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return false;
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((addr << 1) | I2C_MASTER_WRITE), true);
    i2c_master_stop(cmd);
    const esp_err_t err =
        i2c_master_cmd_begin((i2c_port_t)BLIMP_BOARD_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}

esp_err_t blimp_mag_init(void)
{
    s_present = i2c_probe_addr((uint8_t)BLIMP_BOARD_MAG_I2C_ADDR);
    if (s_present) {
        ESP_LOGI(TAG, "device ACKed at 0x%02X (LIS3MDL assumed, NOT "
                       "confirmed)",
                 (unsigned)BLIMP_BOARD_MAG_I2C_ADDR);
    } else {
        ESP_LOGI(TAG, "no device ACKed at 0x%02X - no magnetometer "
                       "populated yet, or wrong address (docs/02 section 6 "
                       "C.8 is still open). Falling back to gyro-only yaw "
                       "(blimp_params_t.use_mag handles this in core).",
                 (unsigned)BLIMP_BOARD_MAG_I2C_ADDR);
    }
    return ESP_OK;
}

esp_err_t blimp_mag_read(blimp_imu_sample_t *sample)
{
    if (!sample) {
        return ESP_ERR_INVALID_ARG;
    }
    sample->mag_valid = false;

    /* TODO once a magnetometer is confirmed present and its part number
     * known:
     *   - i2c_master_write_read_device() the raw XYZ registers, scale to
     *     the part's documented gauss/microtesla-per-LSB.
     *   - Apply hard/soft-iron calibration constants (NOT hard-coded here -
     *     they belong in blimp_params_t as a live-tunable, per-vehicle
     *     value, since they must be re-derived on the fully assembled
     *     gondola and again on-site at the venue - docs/07 section 5.2
     *     items 2 and 4).
     *   - This driver reports raw-but-calibrated field vector only; the
     *     runtime health check (reject an implausible magnitude - docs/07
     *     section 5.2 item 3) lives in the ESTIMATOR (estimator.h's
     *     mag_healthy flag), not here, so mag.c stays a thin sensor driver. */
    (void)s_present;
    return ESP_ERR_NOT_SUPPORTED;
}
