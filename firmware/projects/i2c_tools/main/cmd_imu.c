/*
 * 'imu' console command: wake the MPU-6050 and print decoded readings.
 *
 * Shares the bus that 'i2cconfig' set up, so it follows whatever SDA/SCL the
 * rest of the tool is using.
 */
#include <stdio.h>
#include <string.h>
#include "argtable3/argtable3.h"
#include "driver/i2c_master.h"
#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cmd_i2ctools.h"
#include "cmd_imu.h"

static const char *TAG = "cmd_imu";

#define IMU_TIMEOUT_MS      50
#define IMU_SCL_SPEED_HZ    (100 * 1000)

#define MPU6050_ADDR_DEFAULT    0x68
#define MPU6050_REG_SMPLRT_DIV  0x19
#define MPU6050_REG_CONFIG      0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_WHO_AM_I    0x75

#define MPU6050_WHO_AM_I_VALUE  0x68

/* Datasheet sensitivities, indexed by the FS_SEL / AFS_SEL field. */
static const float accel_lsb_per_g[4]   = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const int   accel_range_g[4]     = { 2, 4, 8, 16 };
static const float gyro_lsb_per_dps[4]  = { 131.0f, 65.5f, 32.8f, 16.4f };
static const int   gyro_range_dps[4]    = { 250, 500, 1000, 2000 };

static esp_err_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, IMU_TIMEOUT_MS);
}

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t out[2] = { reg, val };
    return i2c_master_transmit(dev, out, sizeof(out), IMU_TIMEOUT_MS);
}

static struct {
    struct arg_int *chip_address;
    struct arg_int *count;
    struct arg_int *interval;
    struct arg_int *accel_fs;
    struct arg_int *gyro_fs;
    struct arg_end *end;
} imu_args;

static int do_imu_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&imu_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, imu_args.end, argv[0]);
        return 1;
    }

    int chip_addr = imu_args.chip_address->count ? imu_args.chip_address->ival[0]
                                                 : MPU6050_ADDR_DEFAULT;
    int count     = imu_args.count->count    ? imu_args.count->ival[0]    : 10;
    int interval  = imu_args.interval->count ? imu_args.interval->ival[0] : 200;
    int want_afs  = imu_args.accel_fs->count ? imu_args.accel_fs->ival[0] : 0;
    int want_fs   = imu_args.gyro_fs->count  ? imu_args.gyro_fs->ival[0]  : 0;

    if (want_afs < 0 || want_afs > 3 || want_fs < 0 || want_fs > 3) {
        ESP_LOGE(TAG, "full-scale selectors must be 0-3");
        return 1;
    }

    i2c_device_config_t dev_conf = {
        .scl_speed_hz = IMU_SCL_SPEED_HZ,
        .device_address = chip_addr,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(tool_bus_handle, &dev_conf, &dev) != ESP_OK) {
        ESP_LOGE(TAG, "could not add device 0x%02x to the bus", chip_addr);
        return 1;
    }

    int ret = 1;
    uint8_t who = 0;
    if (reg_read(dev, MPU6050_REG_WHO_AM_I, &who, 1) != ESP_OK) {
        ESP_LOGE(TAG, "no response from 0x%02x - check wiring and 'i2cdetect'", chip_addr);
        goto cleanup;
    }
    if (who != MPU6050_WHO_AM_I_VALUE) {
        /* Not fatal: MPU-6500/9250 and clones answer differently but share the
           register map closely enough that the readings below still decode. */
        ESP_LOGW(TAG, "WHO_AM_I is 0x%02x, expected 0x%02x for an MPU-6050", who,
                 MPU6050_WHO_AM_I_VALUE);
    }

    /* The MPU-6050 powers up asleep; clearing PWR_MGMT_1 wakes it and selects
       the internal 8 MHz oscillator. */
    if (reg_write(dev, MPU6050_REG_PWR_MGMT_1, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "failed to wake the sensor");
        goto cleanup;
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // let the oscillator and filters settle

    /* Set the ranges explicitly instead of inheriting whatever the chip holds.
       An ESP32 reset does not power-cycle this sensor, so its registers
       survive a reflash - and have been observed changing across one. A
       command whose scaling depends on found state is a command that can
       silently report numbers off by 4x. */
    if (reg_write(dev, MPU6050_REG_GYRO_CONFIG, (uint8_t)(want_fs << 3)) != ESP_OK ||
        reg_write(dev, MPU6050_REG_ACCEL_CONFIG, (uint8_t)(want_afs << 3)) != ESP_OK) {
        ESP_LOGE(TAG, "failed to set the range configuration");
        goto cleanup;
    }

    /* Read them back and scale from what the chip actually reports, so a write
       that did not take shows up as wrong-looking ranges in the header rather
       than as quietly mis-scaled readings. */
    uint8_t cfg[2] = {0};
    if (reg_read(dev, MPU6050_REG_GYRO_CONFIG, cfg, 2) != ESP_OK) {
        ESP_LOGE(TAG, "failed to read the range configuration");
        goto cleanup;
    }
    int fs_sel  = (cfg[0] >> 3) & 0x03;
    int afs_sel = (cfg[1] >> 3) & 0x03;

    if (fs_sel != want_fs || afs_sel != want_afs) {
        ESP_LOGW(TAG, "range readback mismatch: asked for afs=%d fs=%d, chip reports "
                      "afs=%d fs=%d", want_afs, want_fs, afs_sel, fs_sel);
    }

    printf("MPU-6050 @ 0x%02x   accel +/-%dg   gyro +/-%d dps\n",
           chip_addr, accel_range_g[afs_sel], gyro_range_dps[fs_sel]);
    printf("%9s %9s %9s %9s %9s %9s %8s\n",
           "ax[g]", "ay[g]", "az[g]", "gx[dps]", "gy[dps]", "gz[dps]", "T[C]");

    for (int i = 0; i < count; i++) {
        /* Accel, temperature and gyro are contiguous from 0x3B, so one
           transaction gives a coherent sample of all three. */
        uint8_t raw[14];
        if (reg_read(dev, MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw)) != ESP_OK) {
            ESP_LOGE(TAG, "read failed at sample %d", i);
            goto cleanup;
        }

        int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
        int16_t ay = (int16_t)((raw[2]  << 8) | raw[3]);
        int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
        int16_t t  = (int16_t)((raw[6]  << 8) | raw[7]);
        int16_t gx = (int16_t)((raw[8]  << 8) | raw[9]);
        int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
        int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

        printf("%9.3f %9.3f %9.3f %9.2f %9.2f %9.2f %8.1f\n",
               ax / accel_lsb_per_g[afs_sel],
               ay / accel_lsb_per_g[afs_sel],
               az / accel_lsb_per_g[afs_sel],
               gx / gyro_lsb_per_dps[fs_sel],
               gy / gyro_lsb_per_dps[fs_sel],
               gz / gyro_lsb_per_dps[fs_sel],
               t / 340.0f + 36.53f);   // datasheet conversion
        fflush(stdout);

        if (i + 1 < count) {
            vTaskDelay(pdMS_TO_TICKS(interval));
        }
    }
    ret = 0;

cleanup:
    i2c_master_bus_rm_device(dev);
    return ret;
}

void register_imu(void)
{
    imu_args.chip_address = arg_int0("c", "chip", "<addr>", "I2C address (default 0x68)");
    imu_args.count        = arg_int0("n", "count", "<n>", "Samples to print (default 10)");
    imu_args.interval     = arg_int0("i", "interval", "<ms>", "Delay between samples (default 200)");
    imu_args.accel_fs     = arg_int0("a", "accel", "<0-3>", "Accel range: 0=2g 1=4g 2=8g 3=16g (default 0)");
    imu_args.gyro_fs      = arg_int0("g", "gyro", "<0-3>", "Gyro range: 0=250 1=500 2=1000 3=2000 dps (default 0)");
    imu_args.end          = arg_end(2);

    const esp_console_cmd_t imu_cmd = {
        .command = "imu",
        .help = "Read the MPU-6050 and print decoded accel/gyro/temperature",
        .hint = NULL,
        .func = &do_imu_cmd,
        .argtable = &imu_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&imu_cmd));
}
