/*
 * 'baro' console command: decoded BMP085/BMP180 temperature and pressure.
 *
 * Unlike the MPU-6050 and HMC5883L, this chip hands out raw ADC counts that
 * mean nothing until they are run through 11 calibration coefficients stored
 * in its EEPROM. The compensation below is Bosch's fixed-point algorithm from
 * the datasheet, kept in integer math deliberately so it matches the reference
 * implementation bit for bit.
 */
#include <stdio.h>
#include <math.h>
#include "argtable3/argtable3.h"
#include "driver/i2c_master.h"
#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cmd_i2ctools.h"
#include "cmd_baro.h"

static const char *TAG = "cmd_baro";

#define BARO_TIMEOUT_MS     50
#define BARO_SCL_SPEED_HZ   (100 * 1000)

#define BMP180_ADDR_DEFAULT 0x77
#define BMP180_REG_CALIB    0xAA   // 22 bytes of calibration EEPROM
#define BMP180_REG_CHIP_ID  0xD0
#define BMP180_REG_CTRL     0xF4
#define BMP180_REG_RESULT   0xF6

#define BMP180_CHIP_ID      0x55
#define BMP180_CMD_TEMP     0x2E
#define BMP180_CMD_PRESSURE 0x34

/* Standard sea-level pressure, for the altitude estimate. */
#define SEA_LEVEL_PA        101325.0f

/* Conversion time per oversampling setting, from the datasheet. */
static const int baro_conv_ms[4] = { 5, 8, 14, 26 };

struct bmp180_calib {
    int16_t  ac1, ac2, ac3;
    uint16_t ac4, ac5, ac6;
    int16_t  b1, b2, mb, mc, md;
};

static esp_err_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, BARO_TIMEOUT_MS);
}

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t out[2] = { reg, val };
    return i2c_master_transmit(dev, out, sizeof(out), BARO_TIMEOUT_MS);
}

/* vTaskDelay rounds down to whole ticks and the first tick can land almost
   immediately, so at the 10 ms tick of this build a naive pdMS_TO_TICKS(26)
   would wait only 20 ms - short of the conversion time. Round up, then add a
   tick, so the delay is always at least the requested one. */
static void delay_at_least_ms(int ms)
{
    TickType_t ticks = (ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
    vTaskDelay(ticks + 1);
}

static esp_err_t read_calibration(i2c_master_dev_handle_t dev, struct bmp180_calib *c)
{
    uint8_t raw[22];
    esp_err_t err = reg_read(dev, BMP180_REG_CALIB, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    /* The datasheet guarantees no coefficient is 0x0000 or 0xFFFF, so either
       value means we are reading a chip that is not there or not ready. */
    for (int i = 0; i < 11; i++) {
        uint16_t word = (uint16_t)((raw[i * 2] << 8) | raw[i * 2 + 1]);
        if (word == 0x0000 || word == 0xFFFF) {
            ESP_LOGE(TAG, "calibration word %d is 0x%04x - invalid EEPROM", i, word);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    c->ac1 = (int16_t)((raw[0]  << 8) | raw[1]);
    c->ac2 = (int16_t)((raw[2]  << 8) | raw[3]);
    c->ac3 = (int16_t)((raw[4]  << 8) | raw[5]);
    c->ac4 = (uint16_t)((raw[6]  << 8) | raw[7]);
    c->ac5 = (uint16_t)((raw[8]  << 8) | raw[9]);
    c->ac6 = (uint16_t)((raw[10] << 8) | raw[11]);
    c->b1  = (int16_t)((raw[12] << 8) | raw[13]);
    c->b2  = (int16_t)((raw[14] << 8) | raw[15]);
    c->mb  = (int16_t)((raw[16] << 8) | raw[17]);
    c->mc  = (int16_t)((raw[18] << 8) | raw[19]);
    c->md  = (int16_t)((raw[20] << 8) | raw[21]);
    return ESP_OK;
}

static esp_err_t read_raw_temp(i2c_master_dev_handle_t dev, int32_t *ut)
{
    esp_err_t err = reg_write(dev, BMP180_REG_CTRL, BMP180_CMD_TEMP);
    if (err != ESP_OK) {
        return err;
    }
    delay_at_least_ms(baro_conv_ms[0]);

    uint8_t raw[2];
    err = reg_read(dev, BMP180_REG_RESULT, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }
    *ut = (raw[0] << 8) | raw[1];
    return ESP_OK;
}

static esp_err_t read_raw_pressure(i2c_master_dev_handle_t dev, int oss, int32_t *up)
{
    esp_err_t err = reg_write(dev, BMP180_REG_CTRL, BMP180_CMD_PRESSURE + (oss << 6));
    if (err != ESP_OK) {
        return err;
    }
    delay_at_least_ms(baro_conv_ms[oss]);

    uint8_t raw[3];
    err = reg_read(dev, BMP180_REG_RESULT, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }
    *up = ((raw[0] << 16) | (raw[1] << 8) | raw[2]) >> (8 - oss);
    return ESP_OK;
}

/* Bosch's compensation, datasheet section 3.5. B5 is shared between the
   temperature and pressure results, so temperature must be read first. */
static void compensate(const struct bmp180_calib *c, int32_t ut, int32_t up, int oss,
                       float *temp_c, int32_t *press_pa)
{
    int32_t x1 = ((ut - (int32_t)c->ac6) * (int32_t)c->ac5) >> 15;
    int32_t x2 = ((int32_t)c->mc << 11) / (x1 + (int32_t)c->md);
    int32_t b5 = x1 + x2;
    *temp_c = ((b5 + 8) >> 4) / 10.0f;   // datasheet yields 0.1 C units

    int32_t b6 = b5 - 4000;
    x1 = ((int32_t)c->b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = ((int32_t)c->ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = ((((int32_t)c->ac1 * 4 + x3) << oss) + 2) >> 2;

    x1 = ((int32_t)c->ac3 * b6) >> 13;
    x2 = ((int32_t)c->b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = ((uint32_t)c->ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)up - b3) * (uint32_t)(50000 >> oss);

    int32_t p = (b7 < 0x80000000U) ? (int32_t)((b7 << 1) / b4)
                                   : (int32_t)((b7 / b4) << 1);

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    *press_pa = p + ((x1 + x2 + 3791) >> 4);
}

static struct {
    struct arg_int *chip_address;
    struct arg_int *count;
    struct arg_int *interval;
    struct arg_int *oss;
    struct arg_end *end;
} baro_args;

static int do_baro_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&baro_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, baro_args.end, argv[0]);
        return 1;
    }

    int chip_addr = baro_args.chip_address->count ? baro_args.chip_address->ival[0]
                                                  : BMP180_ADDR_DEFAULT;
    int count     = baro_args.count->count    ? baro_args.count->ival[0]    : 10;
    int interval  = baro_args.interval->count ? baro_args.interval->ival[0] : 500;
    int oss       = baro_args.oss->count      ? baro_args.oss->ival[0]      : 3;

    if (oss < 0 || oss > 3) {
        ESP_LOGE(TAG, "oversampling must be 0-3");
        return 1;
    }

    i2c_device_config_t dev_conf = {
        .scl_speed_hz = BARO_SCL_SPEED_HZ,
        .device_address = chip_addr,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(tool_bus_handle, &dev_conf, &dev) != ESP_OK) {
        ESP_LOGE(TAG, "could not add device 0x%02x to the bus", chip_addr);
        return 1;
    }

    int ret = 1;
    uint8_t chip_id = 0;
    if (reg_read(dev, BMP180_REG_CHIP_ID, &chip_id, 1) != ESP_OK) {
        ESP_LOGE(TAG, "no response from 0x%02x - check wiring and 'i2cdetect'", chip_addr);
        goto cleanup;
    }
    if (chip_id != BMP180_CHIP_ID) {
        ESP_LOGW(TAG, "chip ID is 0x%02x, expected 0x%02x", chip_id, BMP180_CHIP_ID);
    }

    struct bmp180_calib calib;
    if (read_calibration(dev, &calib) != ESP_OK) {
        goto cleanup;
    }

    printf("BMP085/180 @ 0x%02x   oversampling %d (%d ms)   sea level %.2f hPa\n",
           chip_addr, oss, baro_conv_ms[oss], SEA_LEVEL_PA / 100.0f);
    printf("%8s %11s %10s\n", "T[C]", "P[hPa]", "alt[m]");

    for (int i = 0; i < count; i++) {
        int32_t ut = 0, up = 0;
        if (read_raw_temp(dev, &ut) != ESP_OK ||
            read_raw_pressure(dev, oss, &up) != ESP_OK) {
            ESP_LOGE(TAG, "read failed at sample %d", i);
            goto cleanup;
        }

        float temp_c;
        int32_t press_pa;
        compensate(&calib, ut, up, oss, &temp_c, &press_pa);

        /* Barometric formula; only as good as the assumed sea-level pressure. */
        float altitude = 44330.0f * (1.0f - powf(press_pa / SEA_LEVEL_PA, 1.0f / 5.255f));

        printf("%8.1f %11.2f %10.1f\n", temp_c, press_pa / 100.0f, altitude);
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

void register_baro(void)
{
    baro_args.chip_address = arg_int0("c", "chip", "<addr>", "I2C address (default 0x77)");
    baro_args.count        = arg_int0("n", "count", "<n>", "Samples to print (default 10)");
    baro_args.interval     = arg_int0("i", "interval", "<ms>", "Delay between samples (default 500)");
    baro_args.oss          = arg_int0("o", "oss", "<0-3>", "Oversampling, 0=fast 3=best (default 3)");
    baro_args.end          = arg_end(2);

    const esp_console_cmd_t baro_cmd = {
        .command = "baro",
        .help = "Read the BMP085/BMP180 and print temperature, pressure and altitude",
        .hint = NULL,
        .func = &do_baro_cmd,
        .argtable = &baro_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&baro_cmd));
}

/* Air temperature alone, for the 'ping' speed-of-sound correction. The
   barometer is already on the bus, so exposing this is cheaper than making
   the caller carry its own temperature source - and more honest than letting
   an ultrasonic range be computed from an assumed 20 C.

   Only B5 of Bosch's compensation matters here, so those three lines are
   repeated rather than calling compensate() with a zero pressure reading,
   which would underflow its unsigned B7 term. */
esp_err_t baro_read_temperature_c(float *temp_c)
{
    if (temp_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_conf = {
        .scl_speed_hz = BARO_SCL_SPEED_HZ,
        .device_address = BMP180_ADDR_DEFAULT,
    };
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = i2c_master_bus_add_device(tool_bus_handle, &dev_conf, &dev);
    if (err != ESP_OK) {
        return err;
    }

    struct bmp180_calib calib;
    int32_t ut = 0;
    err = read_calibration(dev, &calib);
    if (err == ESP_OK) {
        err = read_raw_temp(dev, &ut);
    }
    if (err == ESP_OK) {
        int32_t x1 = ((ut - (int32_t)calib.ac6) * (int32_t)calib.ac5) >> 15;
        int32_t x2 = ((int32_t)calib.mc << 11) / (x1 + (int32_t)calib.md);
        *temp_c = (((x1 + x2) + 8) >> 4) / 10.0f;
    }

    i2c_master_bus_rm_device(dev);
    return err;
}
