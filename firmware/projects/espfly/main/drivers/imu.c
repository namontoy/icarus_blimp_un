/* imu.c - InvenSense MPU-6050 gyro + accelerometer driver.
 *
 * PART CONFIRMED: MPU-6050 (user-confirmed, 2026-09-21). This replaces the
 * earlier honest stub, which deliberately refused to guess a register map
 * while the part was unknown.
 *
 * WHAT THE MPU-6050 DOES NOT HAVE: a magnetometer. That is not a detail on
 * this vehicle. Yaw is therefore pure gyro integration, which drifts without
 * bound, and a blimp cannot fight through an accumulated heading error the
 * way a quadcopter can - it has a handful of grams-force of differential
 * thrust, not continuous full authority. docs/07 section 5.2 ranks "add a
 * magnetometer" as the single highest-leverage change available. Until one
 * is fitted, set params.use_mag = false and expect heading to degrade over a
 * long run; the estimator handles the mag-absent case correctly (it simply
 * has nothing bounding yaw).
 *
 * ORIENTATION: the chip's axes are almost certainly not the vehicle's axes.
 * Core uses x-forward, y-left, z-up (blimp/types.h). The remap lives in
 * board_config.h as BLIMP_BOARD_IMU_AXIS_* and MUST be verified by hand in
 * bring-up step 3 (docs/07 section 7): rotate the assembled gondola about
 * each axis and confirm the sign of every gyro and accel channel. Getting a
 * sign wrong here produces a controller that diverges confidently.
 *
 * I2C API: uses the legacy driver/i2c.h master API, matching baro.c and
 * mag.c. ESP-IDF v5.3 still ships it. Do not migrate one driver to
 * driver/i2c_master.h in isolation - the bus is shared and main.c's
 * board_i2c_init() owns installation.
 */
#include "imu.h"

#include <math.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_config.h"

static const char *TAG = "imu";

/* ---- MPU-6050 register map (datasheet rev 4.2, register map rev 4.2) ---- */
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75

#define MPU6050_WHO_AM_I_VALUE    0x68

/* PWR_MGMT_1: clear SLEEP, and select the gyro X PLL as the clock source.
 * The datasheet explicitly recommends a gyro PLL over the internal 8 MHz
 * oscillator for stability - this matters here because yaw is integrated
 * open-loop with nothing to correct it. */
#define MPU6050_CLKSEL_PLL_XGYRO  0x01

/* CONFIG: DLPF_CFG = 3 -> 44 Hz accel / 42 Hz gyro, 1 kHz gyro output rate.
 * Chosen deliberately low: a blimp's own dynamics are well under 1 Hz, and
 * the dominant high-frequency content is prop wash and gondola vibration,
 * which is noise we want nothing to do with. */
#define MPU6050_DLPF_CFG          0x03
#define MPU6050_GYRO_OUTPUT_HZ    1000.0f

/* Full-scale ranges. Both deliberately narrow rather than maximal, because
 * resolution is worth more than headroom on a vehicle this slow:
 *   gyro  +/-500 dps   - blimp yaw rates are a few tens of dps at most
 *   accel +/-4 g       - near-neutrally-buoyant, weakly actuated */
#define MPU6050_GYRO_FS_SEL       1u     /* +/-500 dps  */
#define MPU6050_ACCEL_FS_SEL      1u     /* +/-4 g      */
#define MPU6050_GYRO_LSB_PER_DPS  65.5f
#define MPU6050_ACCEL_LSB_PER_G   8192.0f

#define DEG_TO_RAD                0.017453292519943295f
#define STANDARD_G                9.80665f

#define I2C_TIMEOUT_TICKS         pdMS_TO_TICKS(50)

static bool s_present;

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = ESP_OK;
    err |= i2c_master_start(cmd);
    err |= i2c_master_write_byte(
        cmd, (uint8_t)((BLIMP_BOARD_IMU_I2C_ADDR << 1) | I2C_MASTER_WRITE), true);
    err |= i2c_master_write_byte(cmd, reg, true);
    err |= i2c_master_write_byte(cmd, val, true);
    err |= i2c_master_stop(cmd);
    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(BLIMP_BOARD_I2C_PORT, cmd, I2C_TIMEOUT_TICKS);
    }
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
    if (!buf || len == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = ESP_OK;
    err |= i2c_master_start(cmd);
    err |= i2c_master_write_byte(
        cmd, (uint8_t)((BLIMP_BOARD_IMU_I2C_ADDR << 1) | I2C_MASTER_WRITE), true);
    err |= i2c_master_write_byte(cmd, reg, true);
    err |= i2c_master_start(cmd);          /* repeated start */
    err |= i2c_master_write_byte(
        cmd, (uint8_t)((BLIMP_BOARD_IMU_I2C_ADDR << 1) | I2C_MASTER_READ), true);
    if (len > 1u) {
        err |= i2c_master_read(cmd, buf, len - 1u, I2C_MASTER_ACK);
    }
    err |= i2c_master_read_byte(cmd, buf + len - 1u, I2C_MASTER_NACK);
    err |= i2c_master_stop(cmd);
    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(BLIMP_BOARD_I2C_PORT, cmd, I2C_TIMEOUT_TICKS);
    }
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t blimp_imu_init(void)
{
    s_present = false;

    uint8_t who = 0u;
    esp_err_t err = reg_read(MPU6050_REG_WHO_AM_I, &who, 1u);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no ACK at 0x%02X (%s) - IMU absent or bus misconfigured",
                 BLIMP_BOARD_IMU_I2C_ADDR, esp_err_to_name(err));
        return ESP_OK;          /* never block boot on a missing sensor */
    }
    /* WHO_AM_I returns bits 6:1 of the address, so the low bit reads back as
     * 0 regardless of AD0 strapping - mask it before comparing. Clones often
     * report 0x68 here too; a mismatch is still worth shouting about. */
    if ((who & 0xFEu) != MPU6050_WHO_AM_I_VALUE) {
        ESP_LOGW(TAG, "WHO_AM_I = 0x%02X, expected 0x%02X - wrong part or a clone",
                 who, MPU6050_WHO_AM_I_VALUE);
    }

    /* Wake from sleep and select the gyro PLL clock. The device boots asleep;
     * forgetting this yields an all-zeros read that looks like a dead bus. */
    err = reg_write(MPU6050_REG_PWR_MGMT_1, MPU6050_CLKSEL_PLL_XGYRO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wake failed: %s", esp_err_to_name(err));
        return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(50));      /* PLL settle */

    err |= reg_write(MPU6050_REG_CONFIG, MPU6050_DLPF_CFG);
    err |= reg_write(MPU6050_REG_GYRO_CONFIG,
                     (uint8_t)(MPU6050_GYRO_FS_SEL << 3));
    err |= reg_write(MPU6050_REG_ACCEL_CONFIG,
                     (uint8_t)(MPU6050_ACCEL_FS_SEL << 3));

    /* Sample rate = gyro output rate / (1 + SMPLRT_DIV). With DLPF enabled
     * the gyro output rate is 1 kHz, so divide down to the control rate. */
    const float target_hz = (float)BLIMP_BOARD_IMU_SAMPLE_HZ;
    uint32_t div = 0u;
    if (target_hz > 0.0f && target_hz < MPU6050_GYRO_OUTPUT_HZ) {
        div = (uint32_t)lrintf(MPU6050_GYRO_OUTPUT_HZ / target_hz) - 1u;
        if (div > 255u) { div = 255u; }
    }
    err |= reg_write(MPU6050_REG_SMPLRT_DIV, (uint8_t)div);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "configuration failed: %s", esp_err_to_name(err));
        return ESP_OK;
    }

    s_present = true;
    ESP_LOGI(TAG, "MPU-6050 ready @0x%02X: +/-500dps, +/-4g, DLPF=%d, %.0f Hz",
             BLIMP_BOARD_IMU_I2C_ADDR, MPU6050_DLPF_CFG,
             (double)(MPU6050_GYRO_OUTPUT_HZ / (float)(div + 1u)));
    ESP_LOGW(TAG, "no magnetometer on this part - yaw is unbounded gyro "
                  "integration; see docs/07 section 5.2");
    return ESP_OK;
}

esp_err_t blimp_imu_read(blimp_imu_sample_t *sample)
{
    if (!sample) {
        return ESP_ERR_INVALID_ARG;
    }
    sample->gyro_valid = false;
    sample->accel_valid = false;

    if (!s_present) {
        return ESP_ERR_INVALID_STATE;
    }

    /* One burst read: ACCEL_XOUT_H .. GYRO_ZOUT_L is 14 contiguous bytes
     * (accel 6, temp 2, gyro 6). A burst guarantees all six axes come from
     * the same sample - reading them in separate transactions can straddle
     * an internal update and produce a physically impossible combination. */
    uint8_t raw[14];
    const esp_err_t err = reg_read(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    const int16_t ax = (int16_t)((uint16_t)raw[0]  << 8 | raw[1]);
    const int16_t ay = (int16_t)((uint16_t)raw[2]  << 8 | raw[3]);
    const int16_t az = (int16_t)((uint16_t)raw[4]  << 8 | raw[5]);
    /* raw[6..7] = temperature, unused */
    const int16_t gx = (int16_t)((uint16_t)raw[8]  << 8 | raw[9]);
    const int16_t gy = (int16_t)((uint16_t)raw[10] << 8 | raw[11]);
    const int16_t gz = (int16_t)((uint16_t)raw[12] << 8 | raw[13]);

    /* All-zero across every channel is the classic "still asleep" or
     * "SDA stuck low" signature, not a real reading. */
    if ((ax | ay | az | gx | gy | gz) == 0) {
        ESP_LOGW(TAG, "all-zero sample - device asleep or bus fault");
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Chip frame -> SI. */
    const float g_chip[3] = {
        (float)gx / MPU6050_GYRO_LSB_PER_DPS * DEG_TO_RAD,
        (float)gy / MPU6050_GYRO_LSB_PER_DPS * DEG_TO_RAD,
        (float)gz / MPU6050_GYRO_LSB_PER_DPS * DEG_TO_RAD,
    };
    const float a_chip[3] = {
        (float)ax / MPU6050_ACCEL_LSB_PER_G * STANDARD_G,
        (float)ay / MPU6050_ACCEL_LSB_PER_G * STANDARD_G,
        (float)az / MPU6050_ACCEL_LSB_PER_G * STANDARD_G,
    };

    /* Chip frame -> body frame (x-fwd, y-left, z-up). The mapping is a
     * board_config.h concern because it depends on how the PCB is physically
     * mounted in the gondola, which is not known here. VERIFY BY HAND in
     * bring-up step 3 before closing any loop. */
    const int   idx[3]  = {BLIMP_BOARD_IMU_AXIS_X, BLIMP_BOARD_IMU_AXIS_Y,
                           BLIMP_BOARD_IMU_AXIS_Z};
    const float sign[3] = {BLIMP_BOARD_IMU_SIGN_X, BLIMP_BOARD_IMU_SIGN_Y,
                           BLIMP_BOARD_IMU_SIGN_Z};
    for (int i = 0; i < 3; i++) {
        sample->gyro[i]  = sign[i] * g_chip[idx[i]];
        sample->accel[i] = sign[i] * a_chip[idx[i]];
    }

    sample->gyro_valid = true;
    sample->accel_valid = true;
    return ESP_OK;
}
