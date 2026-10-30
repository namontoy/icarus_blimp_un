/*
 * 'baro' console command and the Bosch barometer driver behind it.
 *
 * Three chips are supported, because the bench has had all of them: the
 * BMP085/BMP180 on the GY-88 carrier, and, since that carrier burned out on
 * 2026-10-22, a stand-alone breakout sold as a BMP280 that identifies itself
 * as a BME280. All three answer chip ID register 0xD0 with a different value
 * (0x55, 0x58, 0x60), so the driver probes 0x77 and then 0x76, reads that
 * one register and picks the chip from the answer - never from the
 * address, which a BMx280 shares with the BMP180 when its SDO pin is high.
 *
 * None of these chips reports physical units. They hand out raw ADC counts
 * that mean nothing until they go through calibration coefficients stored in
 * the chip, so each driver carries Bosch's fixed-point compensation from its
 * datasheet, kept in integer math deliberately so it matches the reference
 * implementation bit for bit.
 *
 * baro_attach() / baro_read() / baro_detach() are the driver; the command
 * below and the sensor stream (sensor_stream.c) are its callers.
 */
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "argtable3/argtable3.h"
#include "driver/i2c_master.h"
#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cmd_i2ctools.h"
#include "cmd_baro.h"
#include "sensor_stream.h"

static const char *TAG = "cmd_baro";

#define BARO_TIMEOUT_MS     50
#define BARO_SCL_SPEED_HZ   (100 * 1000)

/* Where to look when no address is given. 0x77 is the BMP180's only address
   and a BMx280's with SDO high; 0x76 is a BMx280 with SDO low, which is how
   the current breakout is strapped. */
#define BARO_ADDR_PRIMARY   0x77
#define BARO_ADDR_SECONDARY 0x76

/* The same register on all three chips, which is what makes probing cheap. */
#define BARO_REG_CHIP_ID    0xD0

#define BMP180_REG_CALIB    0xAA   // 22 bytes of calibration EEPROM
#define BMP180_REG_CTRL     0xF4
#define BMP180_REG_RESULT   0xF6

#define BMP180_CHIP_ID      0x55
#define BMP180_CMD_TEMP     0x2E
#define BMP180_CMD_PRESSURE 0x34

#define BMP280_CHIP_ID      0x58
#define BME280_CHIP_ID      0x60

#define BMX280_REG_CALIB_TP     0x88   // 26 bytes: T1-T3, P1-P9, one reserved, H1
#define BMX280_REG_CALIB_H      0xE1   // 7 bytes: H2-H6, BME280 only
#define BMX280_REG_RESET        0xE0
#define BMX280_REG_CTRL_HUM     0xF2   // BME280 only
#define BMX280_REG_STATUS       0xF3
#define BMX280_REG_CTRL_MEAS    0xF4
#define BMX280_REG_CONFIG       0xF5
#define BMX280_REG_DATA         0xF7   // press[3] temp[3] (+ hum[2] on the BME280)

#define BMX280_RESET_WORD       0xB6
#define BMX280_STATUS_MEASURING 0x08
#define BMX280_STATUS_IM_UPDATE 0x01
#define BMX280_MODE_FORCED      0x01
/* What a data register holds when its measurement was skipped or never ran. */
#define BMX280_SKIPPED_20BIT    0x80000
#define BMX280_SKIPPED_16BIT    0x8000
/* Longest forced conversion is about 46 ms (P x16, T x2, H x1); this only has
   to catch a chip that never finishes. */
#define BMX280_DONE_TIMEOUT_MS  200

/* Standard sea-level pressure, for the altitude estimate. */
#define SEA_LEVEL_PA        101325.0f

/* Conversion time per oversampling setting, from the datasheet. */
static const int baro_conv_ms[4] = { 5, 8, 14, 26 };

struct bmp180_calib {
    int16_t  ac1, ac2, ac3;
    uint16_t ac4, ac5, ac6;
    int16_t  b1, b2, mb, mc, md;
};

struct bmx280_calib {
    uint16_t t1;
    int16_t  t2, t3;
    uint16_t p1;
    int16_t  p2, p3, p4, p5, p6, p7, p8, p9;
    uint8_t  h1, h3;
    int16_t  h2, h4, h5;
    int8_t   h6;
};

/* osrs register codes: 0 skips the measurement, 1..5 are x1, x2, x4, x8, x16. */
struct bmx280_osrs {
    uint8_t t, p, h;
};

/* '-o 0..3' on a BMx280, mapped onto the datasheet's recommended presets
   (BMP280 table 15: low power, standard, high and ultra-high resolution), so
   the option means "more oversampling, less noise, slower" on every chip.
   Humidity stays at x1: it changes slowly and nothing here needs it fine. */
static const struct bmx280_osrs bmx280_presets[4] = {
    { .t = 1, .p = 2, .h = 1 },
    { .t = 1, .p = 3, .h = 1 },
    { .t = 1, .p = 4, .h = 1 },
    { .t = 2, .p = 5, .h = 1 },
};

struct baro_dev {
    i2c_master_dev_handle_t dev;
    int                 addr;
    baro_chip_t         chip;
    int                 oss;
    struct bmp180_calib b180;   /* BMP180 */
    struct bmx280_calib b280;   /* BMP280 and BME280 */
    struct bmx280_osrs  o;      /* BMP280 and BME280 */
};

static const char *baro_chip_name(baro_chip_t chip)
{
    switch (chip) {
    case BARO_CHIP_BMP180: return "BMP085/180";
    case BARO_CHIP_BMP280: return "BMP280";
    case BARO_CHIP_BME280: return "BME280";
    }
    return "?";
}

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

/* ------------------------------------------------------------- detection */

static bool chip_from_id(uint8_t id, baro_chip_t *chip)
{
    switch (id) {
    case BMP180_CHIP_ID: *chip = BARO_CHIP_BMP180; return true;
    case BMP280_CHIP_ID: *chip = BARO_CHIP_BMP280; return true;
    case BME280_CHIP_ID: *chip = BARO_CHIP_BME280; return true;
    default:             return false;
    }
}

/*
 * Find a barometer and add it to the bus. With want_addr < 0, try 0x77 and
 * then 0x76, and take the first address whose chip ID is one we know. Each
 * address is probed before it is talked to: i2c_master_probe() is the quiet
 * way to ask, where a register read of an empty address makes the driver log
 * a NACK error for what is, here, an expected answer.
 */
static esp_err_t baro_open(int want_addr, i2c_master_dev_handle_t *dev, int *addr,
                           baro_chip_t *chip)
{
    const int candidates[2] = { BARO_ADDR_PRIMARY, BARO_ADDR_SECONDARY };
    const int n = (want_addr >= 0) ? 1 : 2;
    esp_err_t result = ESP_ERR_NOT_FOUND;

    if (tool_bus_handle == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    for (int i = 0; i < n; i++) {
        int a = (want_addr >= 0) ? want_addr : candidates[i];
        if (i2c_master_probe(tool_bus_handle, a, BARO_TIMEOUT_MS) != ESP_OK) {
            continue;
        }
        i2c_device_config_t dev_conf = {
            .scl_speed_hz = BARO_SCL_SPEED_HZ,
            .device_address = a,
        };
        if (i2c_master_bus_add_device(tool_bus_handle, &dev_conf, dev) != ESP_OK) {
            return ESP_FAIL;
        }
        uint8_t id = 0;
        if (reg_read(*dev, BARO_REG_CHIP_ID, &id, 1) == ESP_OK && chip_from_id(id, chip)) {
            *addr = a;
            return ESP_OK;
        }
        ESP_LOGW(TAG, "0x%02x answers, but its chip ID reads 0x%02x - not a BMP180 (0x55), "
                      "BMP280 (0x58) or BME280 (0x60)", a, id);
        i2c_master_bus_rm_device(*dev);
        *dev = NULL;
        result = ESP_ERR_NOT_SUPPORTED;
    }
    return result;
}

/* ----------------------------------------------------------------- BMP180 */

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

/* Barometric formula; only as good as the assumed sea-level pressure. */
static float altitude_m(float press_pa)
{
    return 44330.0f * (1.0f - powf(press_pa / SEA_LEVEL_PA, 1.0f / 5.255f));
}

/* --------------------------------------------------------- BMP280/BME280 */

static int osrs_mult(uint8_t code)
{
    return code ? (1 << (code - 1)) : 0;
}

/* Worst-case forced conversion time, BME280 datasheet appendix B (the BMP280
   has the same formula without the humidity term). Kept in microseconds so
   the half-millisecond terms are not lost before the final rounding up. */
static int bmx280_conv_ms(const struct bmx280_osrs *o)
{
    int us = 1250 + 2300 * osrs_mult(o->t);
    if (o->p) {
        us += 2300 * osrs_mult(o->p) + 575;
    }
    if (o->h) {
        us += 2300 * osrs_mult(o->h) + 575;
    }
    return (us + 999) / 1000;
}

static esp_err_t bmx280_read_calibration(i2c_master_dev_handle_t dev, baro_chip_t chip,
                                         struct bmx280_calib *c)
{
    uint8_t r[26];
    esp_err_t err = reg_read(dev, BMX280_REG_CALIB_TP, r, sizeof(r));
    if (err != ESP_OK) {
        return err;
    }
    /* Little-endian, unlike the BMP180. */
    c->t1 = (uint16_t)(r[0] | (r[1] << 8));
    c->t2 = (int16_t)(r[2] | (r[3] << 8));
    c->t3 = (int16_t)(r[4] | (r[5] << 8));
    c->p1 = (uint16_t)(r[6] | (r[7] << 8));
    c->p2 = (int16_t)(r[8] | (r[9] << 8));
    c->p3 = (int16_t)(r[10] | (r[11] << 8));
    c->p4 = (int16_t)(r[12] | (r[13] << 8));
    c->p5 = (int16_t)(r[14] | (r[15] << 8));
    c->p6 = (int16_t)(r[16] | (r[17] << 8));
    c->p7 = (int16_t)(r[18] | (r[19] << 8));
    c->p8 = (int16_t)(r[20] | (r[21] << 8));
    c->p9 = (int16_t)(r[22] | (r[23] << 8));
    c->h1 = r[25];

    /* Unlike the BMP180's, these words are signed and small negatives such
       as 0xFFF9 are legitimate, so a blanket 0x0000/0xFFFF test would reject
       good chips. T1 and P1 are the two that can be checked: both are
       unsigned scale factors, P1 is a divisor in the pressure formula, and
       neither is ever 0 or all ones on a chip that answered properly. */
    if (c->t1 == 0 || c->t1 == 0xFFFF || c->p1 == 0 || c->p1 == 0xFFFF) {
        ESP_LOGE(TAG, "calibration reads T1=0x%04x P1=0x%04x - invalid NVM", c->t1, c->p1);
        return ESP_ERR_INVALID_RESPONSE;
    }

    c->h2 = 0;
    c->h3 = 0;
    c->h4 = 0;
    c->h5 = 0;
    c->h6 = 0;
    if (chip == BARO_CHIP_BME280) {
        uint8_t h[7];
        err = reg_read(dev, BMX280_REG_CALIB_H, h, sizeof(h));
        if (err != ESP_OK) {
            return err;
        }
        /* H4 and H5 are 12-bit values sharing the nibbles of 0xE5. */
        c->h2 = (int16_t)(h[0] | (h[1] << 8));
        c->h3 = h[2];
        c->h4 = (int16_t)(((int16_t)(int8_t)h[3] * 16) | (h[4] & 0x0F));
        c->h5 = (int16_t)(((int16_t)(int8_t)h[5] * 16) | (h[4] >> 4));
        c->h6 = (int8_t)h[6];
    }
    return ESP_OK;
}

/*
 * Bring the chip to a known state: soft reset, wait for it to reload its
 * calibration from NVM, then write and read back the oversampling and filter
 * settings. A reset is cheap and means nothing a previous run (or 'i2cset')
 * left behind - normal mode, an IIR filter - can leak into the numbers. The
 * chip is left asleep; each sample is one forced conversion, the same
 * request-wait-read rhythm as the BMP180.
 */
static esp_err_t bmx280_setup(i2c_master_dev_handle_t dev, baro_chip_t chip,
                              const struct bmx280_osrs *o, struct bmx280_calib *calib)
{
    esp_err_t err = reg_write(dev, BMX280_REG_RESET, BMX280_RESET_WORD);
    if (err != ESP_OK) {
        return err;
    }
    delay_at_least_ms(2);   // start-up time after reset, datasheet table 1

    uint8_t status = BMX280_STATUS_IM_UPDATE;
    for (int waited = 0; status & BMX280_STATUS_IM_UPDATE; waited += portTICK_PERIOD_MS) {
        if (reg_read(dev, BMX280_REG_STATUS, &status, 1) != ESP_OK) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (waited > BMX280_DONE_TIMEOUT_MS) {
            ESP_LOGE(TAG, "chip still copying its NVM after reset (status 0x%02x)", status);
            return ESP_ERR_TIMEOUT;
        }
        if (status & BMX280_STATUS_IM_UPDATE) {
            vTaskDelay(1);
        }
    }

    err = bmx280_read_calibration(dev, chip, calib);
    if (err != ESP_OK) {
        return err;
    }

    /* ctrl_hum only takes effect at the next ctrl_meas write, so it goes
       first. config (IIR filter off, standby irrelevant in forced mode) is
       only accepted while the chip sleeps, which it does after the reset. */
    const uint8_t ctrl_hum  = o->h & 0x07;
    const uint8_t config    = 0x00;
    const uint8_t ctrl_meas = (uint8_t)((o->t << 5) | (o->p << 2));   // mode 00: sleep
    if ((chip == BARO_CHIP_BME280 && reg_write(dev, BMX280_REG_CTRL_HUM, ctrl_hum) != ESP_OK) ||
        reg_write(dev, BMX280_REG_CONFIG, config) != ESP_OK ||
        reg_write(dev, BMX280_REG_CTRL_MEAS, ctrl_meas) != ESP_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t back[4] = { 0 };   // ctrl_hum, status, ctrl_meas, config
    if (reg_read(dev, BMX280_REG_CTRL_HUM, back, sizeof(back)) != ESP_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if ((chip == BARO_CHIP_BME280 && (back[0] & 0x07) != ctrl_hum) ||
        back[2] != ctrl_meas || (back[3] & 0xFD) != config) {
        ESP_LOGE(TAG, "settings did not stick: ctrl_hum 0x%02x ctrl_meas 0x%02x config 0x%02x",
                 back[0], back[2], back[3]);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

/* One forced conversion. Waits the worst-case time first, then polls the
   'measuring' bit, so a slow or stuck chip is an error and not a stale
   sample read back as if it were new. */
static esp_err_t bmx280_measure(i2c_master_dev_handle_t dev, baro_chip_t chip,
                                const struct bmx280_osrs *o,
                                int32_t *adc_t, int32_t *adc_p, int32_t *adc_h)
{
    const uint8_t ctrl_meas = (uint8_t)((o->t << 5) | (o->p << 2) | BMX280_MODE_FORCED);
    esp_err_t err = reg_write(dev, BMX280_REG_CTRL_MEAS, ctrl_meas);
    if (err != ESP_OK) {
        return err;
    }
    delay_at_least_ms(bmx280_conv_ms(o));

    uint8_t status = BMX280_STATUS_MEASURING;
    for (int waited = 0; status & BMX280_STATUS_MEASURING; waited += portTICK_PERIOD_MS) {
        if (reg_read(dev, BMX280_REG_STATUS, &status, 1) != ESP_OK) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (waited > BMX280_DONE_TIMEOUT_MS) {
            ESP_LOGE(TAG, "conversion never finished (status 0x%02x)", status);
            return ESP_ERR_TIMEOUT;
        }
        if (status & BMX280_STATUS_MEASURING) {
            vTaskDelay(1);
        }
    }

    /* One burst, so pressure, temperature and humidity are from the same
       conversion: the chip shadows the data registers during a read. */
    uint8_t r[8] = { 0 };
    const size_t len = (chip == BARO_CHIP_BME280) ? 8 : 6;
    err = reg_read(dev, BMX280_REG_DATA, r, len);
    if (err != ESP_OK) {
        return err;
    }
    *adc_p = (int32_t)(((uint32_t)r[0] << 12) | ((uint32_t)r[1] << 4) | (r[2] >> 4));
    *adc_t = (int32_t)(((uint32_t)r[3] << 12) | ((uint32_t)r[4] << 4) | (r[5] >> 4));
    *adc_h = (chip == BARO_CHIP_BME280) ? (int32_t)((r[6] << 8) | r[7]) : BMX280_SKIPPED_16BIT;
    return ESP_OK;
}

/* Bosch's compensation, BMP280 datasheet section 8.2 (32-bit temperature,
   64-bit pressure) and BME280 section 8.2 (humidity). t_fine carries the
   temperature into the other two, so temperature is always measured. */
static int32_t bmx280_t_fine(const struct bmx280_calib *c, int32_t adc_t)
{
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)c->t1 << 1))) * ((int32_t)c->t2)) >> 11;
    int32_t var2 = (((((adc_t >> 4) - ((int32_t)c->t1)) *
                      ((adc_t >> 4) - ((int32_t)c->t1))) >> 12) * ((int32_t)c->t3)) >> 14;
    return var1 + var2;
}

static float bmx280_temp_c(int32_t t_fine)
{
    return ((t_fine * 5 + 128) >> 8) / 100.0f;   // datasheet yields 0.01 C units
}

/* Returns false instead of a number when the formula would divide by zero. */
static bool bmx280_press_pa(const struct bmx280_calib *c, int32_t t_fine, int32_t adc_p,
                            float *press_pa)
{
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)c->p6;
    var2 = var2 + ((var1 * (int64_t)c->p5) << 17);
    var2 = var2 + (((int64_t)c->p4) << 35);
    var1 = ((var1 * var1 * (int64_t)c->p3) >> 8) + ((var1 * (int64_t)c->p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)c->p1) >> 33;
    if (var1 == 0) {
        return false;
    }
    int64_t p = 1048576 - adc_p;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)c->p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)c->p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)c->p7) << 4);
    *press_pa = (uint32_t)p / 256.0f;   // datasheet yields Q24.8 Pa
    return true;
}

static float bmx280_humidity_pct(const struct bmx280_calib *c, int32_t t_fine, int32_t adc_h)
{
    int32_t v = t_fine - ((int32_t)76800);
    v = (((((adc_h << 14) - (((int32_t)c->h4) << 20) - (((int32_t)c->h5) * v)) +
           ((int32_t)16384)) >> 15) *
         (((((((v * ((int32_t)c->h6)) >> 10) * (((v * ((int32_t)c->h3)) >> 11) +
               ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)c->h2) + 8192) >> 14));
    v = (v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)c->h1)) >> 4));
    v = (v < 0) ? 0 : v;
    v = (v > 419430400) ? 419430400 : v;
    return (uint32_t)(v >> 12) / 1024.0f;   // datasheet yields Q22.10 %RH
}

/* ---------------------------------------------------------------- driver */

esp_err_t baro_attach(int addr, int oss, baro_dev_t **out)
{
    if (out == NULL || oss < 0 || oss > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    i2c_master_dev_handle_t dev = NULL;
    int found = 0;
    baro_chip_t chip = BARO_CHIP_BMP180;
    esp_err_t err = baro_open(addr, &dev, &found, &chip);
    if (err != ESP_OK) {
        return err;
    }
    baro_dev_t *d = calloc(1, sizeof(*d));
    if (d == NULL) {
        i2c_master_bus_rm_device(dev);
        return ESP_ERR_NO_MEM;
    }
    d->dev  = dev;
    d->addr = found;
    d->chip = chip;
    d->oss  = oss;

    if (chip == BARO_CHIP_BMP180) {
        err = read_calibration(dev, &d->b180);
    } else {
        d->o = bmx280_presets[oss];
        if (chip != BARO_CHIP_BME280) {
            d->o.h = 0;
        }
        err = bmx280_setup(dev, chip, &d->o, &d->b280);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s setup failed", baro_chip_name(chip));
        }
    }
    if (err != ESP_OK) {
        baro_detach(d);
        return err;
    }
    *out = d;
    return ESP_OK;
}

void baro_info(const baro_dev_t *d, baro_info_t *info)
{
    info->chip = d->chip;
    info->name = baro_chip_name(d->chip);
    info->addr = d->addr;
    info->oss  = d->oss;
    if (d->chip == BARO_CHIP_BMP180) {
        info->conv_ms = baro_conv_ms[d->oss];
        info->osrs_t = info->osrs_p = info->osrs_h = 0;
    } else {
        info->conv_ms = bmx280_conv_ms(&d->o);
        info->osrs_t  = osrs_mult(d->o.t);
        info->osrs_p  = osrs_mult(d->o.p);
        info->osrs_h  = osrs_mult(d->o.h);
    }
}

esp_err_t baro_read(baro_dev_t *d, baro_reading_t *r)
{
    if (d == NULL || r == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    r->humidity_pct = 0.0f;
    r->has_humidity = false;

    if (d->chip == BARO_CHIP_BMP180) {
        int32_t ut = 0, up = 0;
        esp_err_t err = read_raw_temp(d->dev, &ut);
        if (err == ESP_OK) {
            err = read_raw_pressure(d->dev, d->oss, &up);
        }
        if (err != ESP_OK) {
            return err;
        }
        int32_t press_pa = 0;
        compensate(&d->b180, ut, up, d->oss, &r->temp_c, &press_pa);
        r->pressure_pa = (float)press_pa;
    } else {
        int32_t adc_t = 0, adc_p = 0, adc_h = 0;
        esp_err_t err = bmx280_measure(d->dev, d->chip, &d->o, &adc_t, &adc_p, &adc_h);
        if (err != ESP_OK) {
            return err;
        }
        if (adc_t == BMX280_SKIPPED_20BIT || adc_p == BMX280_SKIPPED_20BIT) {
            ESP_LOGE(TAG, "sample came back empty (T 0x%05" PRIx32 ", P 0x%05" PRIx32
                          ") - the conversion did not run", adc_t, adc_p);
            return ESP_ERR_INVALID_RESPONSE;
        }
        int32_t t_fine = bmx280_t_fine(&d->b280, adc_t);
        r->temp_c = bmx280_temp_c(t_fine);
        if (!bmx280_press_pa(&d->b280, t_fine, adc_p, &r->pressure_pa)) {
            ESP_LOGE(TAG, "pressure compensation divides by zero - bad calibration");
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (d->chip == BARO_CHIP_BME280 && adc_h != BMX280_SKIPPED_16BIT) {
            r->humidity_pct = bmx280_humidity_pct(&d->b280, t_fine, adc_h);
            r->has_humidity = true;
        }
    }
    r->altitude_m = altitude_m(r->pressure_pa);
    return ESP_OK;
}

void baro_detach(baro_dev_t *d)
{
    if (d == NULL) {
        return;
    }
    if (d->dev != NULL) {
        i2c_master_bus_rm_device(d->dev);
    }
    free(d);
}

/* Air temperature alone, for the 'ping' speed-of-sound correction. One
   attach, one sample at the fastest preset, detach: about 20 ms on the bus,
   which is nothing next to an ultrasonic ping. */
esp_err_t baro_read_temperature_c(float *temp_c)
{
    if (temp_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    baro_dev_t *d = NULL;
    esp_err_t err = baro_attach(-1, 0, &d);
    if (err != ESP_OK) {
        return err;
    }
    baro_reading_t r;
    err = baro_read(d, &r);
    if (err == ESP_OK) {
        *temp_c = r.temp_c;
    }
    baro_detach(d);
    return err;
}

/* ---------------------------------------------------------------- console */

static int run_baro(baro_dev_t *d, int count, int interval)
{
    baro_info_t info;
    baro_info(d, &info);
    const bool bmp180 = (info.chip == BARO_CHIP_BMP180);
    const bool humid  = (info.chip == BARO_CHIP_BME280);

    if (bmp180) {
        printf("BMP085/180 @ 0x%02x   oversampling %d (%d ms)   sea level %.2f hPa\n",
               info.addr, info.oss, info.conv_ms, SEA_LEVEL_PA / 100.0f);
    } else {
        printf("%s @ 0x%02x   oversampling %d (P x%d, T x%d", info.name, info.addr,
               info.oss, info.osrs_p, info.osrs_t);
        if (humid) {
            printf(", H x%d", info.osrs_h);
        }
        printf(", %d ms)   sea level %.2f hPa\n", info.conv_ms, SEA_LEVEL_PA / 100.0f);
    }
    if (humid) {
        printf("%8s %11s %10s %7s\n", "T[C]", "P[hPa]", "alt[m]", "RH[%]");
    } else {
        printf("%8s %11s %10s\n", "T[C]", "P[hPa]", "alt[m]");
    }

    for (int i = 0; i < count; i++) {
        baro_reading_t r;
        if (baro_read(d, &r) != ESP_OK) {
            ESP_LOGE(TAG, "read failed at sample %d", i);
            return 1;
        }
        if (bmp180) {
            printf("%8.1f %11.2f %10.1f\n", r.temp_c, r.pressure_pa / 100.0f, r.altitude_m);
        } else {
            printf("%8.2f %11.2f %10.1f", r.temp_c, r.pressure_pa / 100.0f, r.altitude_m);
            if (humid) {
                if (r.has_humidity) {
                    printf(" %7.1f", r.humidity_pct);
                } else {
                    printf(" %7s", "-");
                }
            }
            printf("\n");
        }
        fflush(stdout);

        if (i + 1 < count) {
            vTaskDelay(pdMS_TO_TICKS(interval));
        }
    }
    return 0;
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

    int want_addr = baro_args.chip_address->count ? baro_args.chip_address->ival[0] : -1;
    int count     = baro_args.count->count    ? baro_args.count->ival[0]    : 10;
    int interval  = baro_args.interval->count ? baro_args.interval->ival[0] : 500;
    int oss       = baro_args.oss->count      ? baro_args.oss->ival[0]      : 3;

    if (oss < 0 || oss > 3) {
        ESP_LOGE(TAG, "oversampling must be 0-3");
        return 1;
    }

    /* The stream holds this chip while it runs; borrow it for the command. */
    sensor_stream_pause();

    int ret = 1;
    baro_dev_t *d = NULL;
    esp_err_t err = baro_attach(want_addr, oss, &d);
    if (err == ESP_OK) {
        ret = run_baro(d, count, interval);
    } else if (err == ESP_ERR_NOT_FOUND) {
        if (want_addr >= 0) {
            ESP_LOGE(TAG, "no response from 0x%02x - check wiring and 'i2cdetect'", want_addr);
        } else {
            ESP_LOGE(TAG, "no barometer at 0x77 or 0x76 - check wiring and 'i2cdetect'");
        }
    } else if (err == ESP_FAIL) {
        ESP_LOGE(TAG, "could not add the device to the bus");
    }
    baro_detach(d);
    sensor_stream_resume();
    return ret;
}

void register_baro(void)
{
    baro_args.chip_address = arg_int0("c", "chip", "<addr>", "I2C address (default: 0x77, then 0x76)");
    baro_args.count        = arg_int0("n", "count", "<n>", "Samples to print (default 10)");
    baro_args.interval     = arg_int0("i", "interval", "<ms>", "Delay between samples (default 500)");
    baro_args.oss          = arg_int0("o", "oss", "<0-3>", "Oversampling, 0=fast 3=best (default 3)");
    baro_args.end          = arg_end(2);

    const esp_console_cmd_t baro_cmd = {
        .command = "baro",
        .help = "Read the barometer (BMP180, BMP280 or BME280, found automatically) "
                "and print temperature, pressure, altitude and, on a BME280, humidity",
        .hint = NULL,
        .func = &do_baro_cmd,
        .argtable = &baro_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&baro_cmd));
}
