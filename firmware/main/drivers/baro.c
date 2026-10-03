/* baro.c - barometric altitude driver.
 *
 * See imu.c's header comment for the I2C driver-choice note (legacy
 * `driver/i2c.h`, not the newer v5.2+ `driver/i2c_master.h`) - applies here
 * identically.
 */
#include "baro.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "board_config.h"

static const char *TAG = "baro";
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

esp_err_t blimp_baro_init(void)
{
    s_present = i2c_probe_addr((uint8_t)BLIMP_BOARD_BARO_I2C_ADDR);
    if (s_present) {
        ESP_LOGI(TAG, "device ACKed at 0x%02X (BMP390 assumed, NOT "
                       "confirmed - BMP280/BME280/DPS310 share this address "
                       "pair, see board_config.h)",
                 (unsigned)BLIMP_BOARD_BARO_I2C_ADDR);
    } else {
        ESP_LOGW(TAG, "no device ACKed at 0x%02X - barometer missing, "
                       "wrong address, or bus not wired yet",
                 (unsigned)BLIMP_BOARD_BARO_I2C_ADDR);
    }
    return ESP_OK;
}

esp_err_t blimp_baro_read(blimp_imu_sample_t *sample)
{
    if (!sample) {
        return ESP_ERR_INVALID_ARG;
    }
    sample->baro_valid = false;

    /* TODO once docs/02 section 6 C.7 confirms BMP390 (or whichever part is
     * actually populated):
     *   - i2c_master_write_read_device() the raw pressure/temperature
     *     registers (BMP390's own compensation scheme needs its factory
     *     calibration words, read once at init and cached - do NOT re-read
     *     them every tick).
     *   - Apply the barometric formula against a REFERENCE pressure zeroed
     *     at arm time (blimp_estimator_zero(), estimator.h) - NOT a fixed
     *     sea-level constant, since Bogota's own local pressure is itself
     *     uncertain by ~2.5 kPa (docs/02 section 1.1) and altitude here only
     *     ever needs to be relative to the arena floor at PREPARE time, not
     *     absolute above sea level. */
    (void)s_present;
    return ESP_ERR_NOT_SUPPORTED;
}
