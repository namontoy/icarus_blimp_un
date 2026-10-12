/*
 * 'mag' console command: decoded HMC5883L magnetometer readings.
 *
 * Shares the bus that 'i2cconfig' set up, like the rest of the tool.
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
#include "cmd_mag.h"

static const char *TAG = "cmd_mag";

#define MAG_TIMEOUT_MS      50
#define MAG_SCL_SPEED_HZ    (100 * 1000)

#define HMC5883L_ADDR_DEFAULT   0x1E
#define HMC5883L_REG_CONFIG_A   0x00
#define HMC5883L_REG_CONFIG_B   0x01
#define HMC5883L_REG_MODE       0x02
#define HMC5883L_REG_DATA_X_H   0x03
#define HMC5883L_REG_STATUS     0x09
#define HMC5883L_REG_ID_A       0x0A

#define HMC5883L_MODE_CONTINUOUS 0x00
/* 8 samples averaged, 15 Hz output, normal measurement bias. */
#define HMC5883L_CONFIG_A_DEFAULT 0x70
#define HMC5883L_STATUS_RDY       0x01
/* 15 Hz means a sample every ~67 ms; well inside this. */
#define HMC5883L_DRDY_TIMEOUT_MS  500
/* Returned on any axis whose field exceeded the selected gain. */
#define HMC5883L_OVERFLOW       (-4096)

/* Datasheet gain table, indexed by the GN field of CONFIG_B. */
static const float mag_lsb_per_gauss[8] = {
    1370.0f, 1090.0f, 820.0f, 660.0f, 440.0f, 390.0f, 330.0f, 230.0f
};
static const float mag_range_gauss[8] = {
    0.88f, 1.3f, 1.9f, 2.5f, 4.0f, 4.7f, 5.6f, 8.1f
};

static esp_err_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, MAG_TIMEOUT_MS);
}

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t out[2] = { reg, val };
    return i2c_master_transmit(dev, out, sizeof(out), MAG_TIMEOUT_MS);
}

static struct {
    struct arg_int *chip_address;
    struct arg_int *count;
    struct arg_int *interval;
    struct arg_end *end;
} mag_args;

static int do_mag_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&mag_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, mag_args.end, argv[0]);
        return 1;
    }

    int chip_addr = mag_args.chip_address->count ? mag_args.chip_address->ival[0]
                                                 : HMC5883L_ADDR_DEFAULT;
    int count     = mag_args.count->count    ? mag_args.count->ival[0]    : 10;
    int interval  = mag_args.interval->count ? mag_args.interval->ival[0] : 200;

    i2c_device_config_t dev_conf = {
        .scl_speed_hz = MAG_SCL_SPEED_HZ,
        .device_address = chip_addr,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(tool_bus_handle, &dev_conf, &dev) != ESP_OK) {
        ESP_LOGE(TAG, "could not add device 0x%02x to the bus", chip_addr);
        return 1;
    }

    int ret = 1;
    /* The three ID registers spell "H43" on a genuine HMC5883L. */
    uint8_t id[3] = {0};
    if (reg_read(dev, HMC5883L_REG_ID_A, id, sizeof(id)) != ESP_OK) {
        ESP_LOGE(TAG, "no response from 0x%02x - check wiring and 'i2cdetect'", chip_addr);
        goto cleanup;
    }
    if (id[0] != 'H' || id[1] != '4' || id[2] != '3') {
        ESP_LOGW(TAG, "ID registers read %02x %02x %02x, expected 'H43' (48 34 33)",
                 id[0], id[1], id[2]);
    }

    /* Set the output rate explicitly rather than inheriting whatever the chip
       happens to hold. An ESP32 reset does not power-cycle this sensor, so its
       config survives a reflash and the first run after a cold power-up starts
       from different state than every run after it. */
    if (reg_write(dev, HMC5883L_REG_CONFIG_A, HMC5883L_CONFIG_A_DEFAULT) != ESP_OK) {
        ESP_LOGE(TAG, "failed to set the output rate");
        goto cleanup;
    }
    /* Continuous mode keeps fresh samples coming without re-triggering each
       read. The power-on default is single-measurement, which takes one
       reading and then idles - leaving the data registers static. */
    if (reg_write(dev, HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS) != ESP_OK) {
        ESP_LOGE(TAG, "failed to select continuous mode");
        goto cleanup;
    }
    uint8_t mode_check = 0xFF;
    if (reg_read(dev, HMC5883L_REG_MODE, &mode_check, 1) != ESP_OK ||
        (mode_check & 0x03) != HMC5883L_MODE_CONTINUOUS) {
        ESP_LOGE(TAG, "mode register reads 0x%02x, not continuous - aborting rather "
                      "than printing a frozen sample repeatedly", mode_check);
        goto cleanup;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // first conversion after a mode change

    /* Read the gain back rather than assuming the reset default, so scaling
       stays correct if it was changed with 'i2cset'. */
    uint8_t conf_b = 0;
    if (reg_read(dev, HMC5883L_REG_CONFIG_B, &conf_b, 1) != ESP_OK) {
        ESP_LOGE(TAG, "failed to read the gain configuration");
        goto cleanup;
    }
    int gn = (conf_b >> 5) & 0x07;

    printf("HMC5883L @ 0x%02x   range +/-%.2f Ga   %.0f LSB/Ga\n",
           chip_addr, mag_range_gauss[gn], mag_lsb_per_gauss[gn]);
    printf("%9s %9s %9s %9s %9s\n", "mx[uT]", "my[uT]", "mz[uT]", "|m|[uT]", "hdg[deg]");

    for (int i = 0; i < count; i++) {
        /* Wait for DRDY instead of trusting the interval. Reading the data
           registers clears this bit, so it only comes back when the chip has
           actually taken a new measurement - which turns a frozen sensor into
           a reported error rather than a plausible-looking repeated row. */
        uint8_t status = 0;
        int waited_ms = 0;
        while (1) {
            if (reg_read(dev, HMC5883L_REG_STATUS, &status, 1) != ESP_OK) {
                ESP_LOGE(TAG, "status read failed at sample %d", i);
                goto cleanup;
            }
            if (status & HMC5883L_STATUS_RDY) {
                break;
            }
            if (waited_ms >= HMC5883L_DRDY_TIMEOUT_MS) {
                ESP_LOGE(TAG, "no new sample after %d ms (status 0x%02x) - the sensor "
                              "is not measuring", HMC5883L_DRDY_TIMEOUT_MS, status);
                goto cleanup;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            waited_ms += 10;
        }

        /* Data registers are X, then Z, then Y - not X, Y, Z. */
        uint8_t raw[6];
        if (reg_read(dev, HMC5883L_REG_DATA_X_H, raw, sizeof(raw)) != ESP_OK) {
            ESP_LOGE(TAG, "read failed at sample %d", i);
            goto cleanup;
        }

        int16_t mx = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t mz = (int16_t)((raw[2] << 8) | raw[3]);
        int16_t my = (int16_t)((raw[4] << 8) | raw[5]);

        if (mx == HMC5883L_OVERFLOW || my == HMC5883L_OVERFLOW || mz == HMC5883L_OVERFLOW) {
            printf("%9s %9s %9s %9s %9s  (overflow - raise the gain range)\n",
                   "-", "-", "-", "-", "-");
        } else {
            /* 1 Gauss = 100 uT; Earth's field is roughly 25-65 uT. */
            float fx = mx * 100.0f / mag_lsb_per_gauss[gn];
            float fy = my * 100.0f / mag_lsb_per_gauss[gn];
            float fz = mz * 100.0f / mag_lsb_per_gauss[gn];
            float heading = atan2f(fy, fx) * 180.0f / (float)M_PI;
            if (heading < 0) {
                heading += 360.0f;
            }
            printf("%9.2f %9.2f %9.2f %9.2f %9.1f\n",
                   fx, fy, fz, sqrtf(fx * fx + fy * fy + fz * fz), heading);
        }
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

void register_mag(void)
{
    mag_args.chip_address = arg_int0("c", "chip", "<addr>", "I2C address (default 0x1E)");
    mag_args.count        = arg_int0("n", "count", "<n>", "Samples to print (default 10)");
    mag_args.interval     = arg_int0("i", "interval", "<ms>", "Delay between samples (default 200)");
    mag_args.end          = arg_end(2);

    const esp_console_cmd_t mag_cmd = {
        .command = "mag",
        .help = "Read the HMC5883L and print field strength and heading",
        .hint = NULL,
        .func = &do_mag_cmd,
        .argtable = &mag_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&mag_cmd));
}
