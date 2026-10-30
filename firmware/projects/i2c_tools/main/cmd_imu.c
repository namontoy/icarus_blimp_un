/*
 * 'imu' console command and the MPU-6050 driver behind it.
 *
 * Shares the bus that 'i2cconfig' set up, so it follows whatever SDA/SCL the
 * rest of the tool is using. imu_attach() / imu_read() / imu_detach() are
 * the driver; the command below and the sensor stream (sensor_stream.c) are
 * its two callers, which is how the bench stays the regression test for the
 * numbers the ground station plots.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "argtable3/argtable3.h"
#include "driver/i2c_master.h"
#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cmd_i2ctools.h"
#include "cmd_imu.h"
#include "sensor_stream.h"

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

/*
 * CONFIG.DLPF_CFG = 4: 21 Hz bandwidth on the accelerometer, 20 Hz on the
 * gyro, 8.5 ms of group delay, the widest setting under the 25 Hz Nyquist
 * rate of the 50 Hz control loop. Found unset on 2026-10-28: the power-on
 * value 0 leaves 260 Hz / 256 Hz, and since the chip keeps its registers
 * across ESP32 resets, whatever a previous run left. Read at 20-50 Hz, an
 * unfiltered sample folds the motors' vibration (hundreds of hertz) into
 * the band where attitude lives. With the filter on the gyro's internal
 * rate is 1 kHz; SMPLRT_DIV 0 keeps the registers at that rate.
 */
#define MPU6050_DLPF_CFG        4
#define MPU6050_DLPF_HZ         21

/* Datasheet sensitivities, indexed by the FS_SEL / AFS_SEL field. */
static const float accel_lsb_per_g[4]   = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const int   accel_range_g[4]     = { 2, 4, 8, 16 };
static const float gyro_lsb_per_dps[4]  = { 131.0f, 65.5f, 32.8f, 16.4f };
static const int   gyro_range_dps[4]    = { 250, 500, 1000, 2000 };

struct imu_dev {
    i2c_master_dev_handle_t dev;
    int addr;
    int afs_sel;    /* what the chip reports back: the scaling follows these, */
    int fs_sel;     /* not what was asked for                                  */
};

static esp_err_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, IMU_TIMEOUT_MS);
}

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t out[2] = { reg, val };
    return i2c_master_transmit(dev, out, sizeof(out), IMU_TIMEOUT_MS);
}

/* ---------------------------------------------------------------- driver */

/*
 * Wake the chip and set the ranges explicitly instead of inheriting whatever
 * the chip holds. An ESP32 reset does not power-cycle this sensor, so its
 * registers survive a reflash - and have been observed changing across one.
 * A driver whose scaling depends on found state is a driver that can
 * silently report numbers off by 4x. The ranges are read back and the
 * scaling uses what the chip reports, so a write that did not take shows up
 * as wrong-looking ranges in the header rather than as quietly mis-scaled
 * readings.
 */
static esp_err_t imu_configure(imu_dev_t *d, int want_afs, int want_fs)
{
    uint8_t who = 0;
    if (reg_read(d->dev, MPU6050_REG_WHO_AM_I, &who, 1) != ESP_OK) {
        ESP_LOGE(TAG, "no response from 0x%02x - check wiring and 'i2cdetect'", d->addr);
        return ESP_ERR_NOT_FOUND;
    }
    if (who != MPU6050_WHO_AM_I_VALUE) {
        /* Not fatal: MPU-6500/9250 and clones answer differently but share the
           register map closely enough that the readings below still decode. */
        ESP_LOGW(TAG, "WHO_AM_I is 0x%02x, expected 0x%02x for an MPU-6050", who,
                 MPU6050_WHO_AM_I_VALUE);
    }

    /* The MPU-6050 powers up asleep; clearing PWR_MGMT_1 wakes it and selects
       the internal 8 MHz oscillator. */
    if (reg_write(d->dev, MPU6050_REG_PWR_MGMT_1, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "failed to wake the sensor");
        return ESP_ERR_INVALID_RESPONSE;
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // let the oscillator and filters settle

    if (reg_write(d->dev, MPU6050_REG_GYRO_CONFIG, (uint8_t)(want_fs << 3)) != ESP_OK ||
        reg_write(d->dev, MPU6050_REG_ACCEL_CONFIG, (uint8_t)(want_afs << 3)) != ESP_OK) {
        ESP_LOGE(TAG, "failed to set the range configuration");
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t cfg[2] = {0};
    if (reg_read(d->dev, MPU6050_REG_GYRO_CONFIG, cfg, 2) != ESP_OK) {
        ESP_LOGE(TAG, "failed to read the range configuration");
        return ESP_ERR_INVALID_RESPONSE;
    }
    d->fs_sel  = (cfg[0] >> 3) & 0x03;
    d->afs_sel = (cfg[1] >> 3) & 0x03;

    if (d->fs_sel != want_fs || d->afs_sel != want_afs) {
        ESP_LOGW(TAG, "range readback mismatch: asked for afs=%d fs=%d, chip reports "
                      "afs=%d fs=%d", want_afs, want_fs, d->afs_sel, d->fs_sel);
    }

    /* The low-pass filter and the sample-rate divider (see MPU6050_DLPF_CFG),
       written every attach for the same reason as the ranges, and read back. */
    if (reg_write(d->dev, MPU6050_REG_SMPLRT_DIV, 0x00) != ESP_OK ||
        reg_write(d->dev, MPU6050_REG_CONFIG, MPU6050_DLPF_CFG) != ESP_OK) {
        ESP_LOGE(TAG, "failed to set the low-pass filter");
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint8_t dlpf = 0xFF;
    if (reg_read(d->dev, MPU6050_REG_CONFIG, &dlpf, 1) != ESP_OK || (dlpf & 0x07) != MPU6050_DLPF_CFG) {
        ESP_LOGW(TAG, "DLPF readback: CONFIG reads 0x%02x, asked for DLPF_CFG %d", dlpf,
                 MPU6050_DLPF_CFG);
    }
    return ESP_OK;
}

int imu_dlpf_hz(const imu_dev_t *d)
{
    (void)d;
    return MPU6050_DLPF_HZ;
}

esp_err_t imu_attach(int addr, int accel_fs, int gyro_fs, imu_dev_t **out)
{
    if (out == NULL || accel_fs < 0 || accel_fs > 3 || gyro_fs < 0 || gyro_fs > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;
    if (addr < 0) {
        addr = MPU6050_ADDR_DEFAULT;
    }
    /* Probe before talking: a register read of an empty address makes the
       driver log a NACK error for what is, for the stream's retries, an
       expected answer. */
    if (tool_bus_handle == NULL ||
        i2c_master_probe(tool_bus_handle, addr, IMU_TIMEOUT_MS) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    imu_dev_t *d = calloc(1, sizeof(*d));
    if (d == NULL) {
        return ESP_ERR_NO_MEM;
    }
    d->addr = addr;
    i2c_device_config_t dev_conf = {
        .scl_speed_hz = IMU_SCL_SPEED_HZ,
        .device_address = addr,
    };
    if (i2c_master_bus_add_device(tool_bus_handle, &dev_conf, &d->dev) != ESP_OK) {
        ESP_LOGE(TAG, "could not add device 0x%02x to the bus", addr);
        free(d);
        return ESP_FAIL;
    }
    esp_err_t err = imu_configure(d, accel_fs, gyro_fs);
    if (err != ESP_OK) {
        imu_detach(d);
        return err;
    }
    *out = d;
    return ESP_OK;
}

void imu_ranges(const imu_dev_t *d, int *accel_g, int *gyro_dps)
{
    *accel_g  = accel_range_g[d->afs_sel];
    *gyro_dps = gyro_range_dps[d->fs_sel];
}

int imu_addr(const imu_dev_t *d)
{
    return d->addr;
}

esp_err_t imu_read(imu_dev_t *d, imu_reading_t *r)
{
    if (d == NULL || r == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Accel, temperature and gyro are contiguous from 0x3B, so one
       transaction gives a coherent sample of all three. */
    uint8_t raw[14];
    esp_err_t err = reg_read(d->dev, MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
    int16_t t  = (int16_t)((raw[6]  << 8) | raw[7]);
    int16_t gx = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

    r->ax_g   = ax / accel_lsb_per_g[d->afs_sel];
    r->ay_g   = ay / accel_lsb_per_g[d->afs_sel];
    r->az_g   = az / accel_lsb_per_g[d->afs_sel];
    r->gx_dps = gx / gyro_lsb_per_dps[d->fs_sel];
    r->gy_dps = gy / gyro_lsb_per_dps[d->fs_sel];
    r->gz_dps = gz / gyro_lsb_per_dps[d->fs_sel];
    r->temp_c = t / 340.0f + 36.53f;   // datasheet conversion
    r->raw_acc[0] = ax;  r->raw_acc[1] = ay;  r->raw_acc[2] = az;
    r->raw_gyro[0] = gx; r->raw_gyro[1] = gy; r->raw_gyro[2] = gz;
    r->raw_temp = t;
    return ESP_OK;
}

void imu_detach(imu_dev_t *d)
{
    if (d == NULL) {
        return;
    }
    if (d->dev != NULL) {
        i2c_master_bus_rm_device(d->dev);
    }
    free(d);
}

/* --------------------------------------------------------------- console */

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

    /* The stream holds this chip while it runs; borrow it for the command. */
    sensor_stream_pause();

    int ret = 1;
    imu_dev_t *d = NULL;
    esp_err_t err = imu_attach(chip_addr, want_afs, want_fs, &d);
    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "no response from 0x%02x - check wiring and 'i2cdetect'", chip_addr);
        goto cleanup;
    }
    if (err != ESP_OK) {
        goto cleanup;   /* imu_attach already said why */
    }

    int accel_g = 0, gyro_dps = 0;
    imu_ranges(d, &accel_g, &gyro_dps);
    printf("MPU-6050 @ 0x%02x   accel +/-%dg   gyro +/-%d dps\n", chip_addr, accel_g, gyro_dps);
    printf("%9s %9s %9s %9s %9s %9s %8s\n",
           "ax[g]", "ay[g]", "az[g]", "gx[dps]", "gy[dps]", "gz[dps]", "T[C]");

    for (int i = 0; i < count; i++) {
        imu_reading_t r;
        if (imu_read(d, &r) != ESP_OK) {
            ESP_LOGE(TAG, "read failed at sample %d", i);
            goto cleanup;
        }
        printf("%9.3f %9.3f %9.3f %9.2f %9.2f %9.2f %8.1f\n",
               r.ax_g, r.ay_g, r.az_g, r.gx_dps, r.gy_dps, r.gz_dps, r.temp_c);
        fflush(stdout);

        if (i + 1 < count) {
            vTaskDelay(pdMS_TO_TICKS(interval));
        }
    }
    ret = 0;

cleanup:
    imu_detach(d);
    sensor_stream_resume();
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
