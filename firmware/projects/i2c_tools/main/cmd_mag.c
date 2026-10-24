/*
 * 'mag' console command and the magnetometer driver behind it.
 *
 * Two chips are supported. The HMC5883L (Honeywell) is the one on the GY-88
 * carrier. The QMC5883L (QST) is what most stand-alone "HMC5883L" breakouts
 * actually carry: after Honeywell discontinued the part, boards kept the name
 * and the footprint but changed the chip, and with it the address (0x0D, not
 * 0x1E), the register map, the byte order and the scale. The one that
 * replaced the burned GY-88 on 2026-10-22 is such a board.
 *
 * With no address given the driver probes 0x1E and then 0x0D, and picks the
 * chip from its identification registers ("H43" on the HMC5883L, 0xFF in
 * register 0x0D on the QMC5883L), not from the address alone.
 *
 * Shares the bus that 'i2cconfig' set up, like the rest of the tool.
 * mag_attach() / mag_read() / mag_detach() are the driver; the command below
 * and the sensor stream (sensor_stream.c) are its two callers.
 */
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
#include "cmd_mag.h"
#include "sensor_stream.h"

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
/* Returned on any axis whose field exceeded the selected gain. */
#define HMC5883L_OVERFLOW       (-4096)

#define QMC5883L_ADDR_DEFAULT   0x0D
#define QMC5883L_REG_DATA_X_L   0x00   // X, Y, Z, each LSB first
#define QMC5883L_REG_STATUS     0x06
#define QMC5883L_REG_CONTROL_1  0x09
#define QMC5883L_REG_CONTROL_2  0x0A
#define QMC5883L_REG_SET_RESET  0x0B
#define QMC5883L_REG_CHIP_ID    0x0D

#define QMC5883L_CHIP_ID        0xFF
#define QMC5883L_STATUS_DRDY    0x01
#define QMC5883L_STATUS_OVL     0x02
#define QMC5883L_SOFT_RESET     0x80
/* The datasheet asks for 0x01 here and does not explain it further. */
#define QMC5883L_SET_RESET_PERIOD 0x01
/*
 * Control 1: OSR 512 (00), range +/-8 G (01), 50 Hz (01), continuous (01).
 *
 * +/-8 G rather than +/-2 G: the chip's noise (about 2 mG) is several times
 * the 8 G step (0.33 mG), so the finer range buys no real resolution, while
 * the motor magnets a few centimetres away can push the local field well
 * past 2 G and turn every sample into an overflow. OSR 512 is the
 * quietest setting; 50 Hz is still four times faster than 'mag' prints.
 */
#define QMC5883L_CONTROL_1_DEFAULT 0x15
#define QMC5883L_LSB_PER_GAUSS  3000.0f
#define QMC5883L_RANGE_GAUSS    8.0f

/* The HMC5883L at 15 Hz means a sample every ~67 ms, the QMC5883L at 50 Hz
   every 20 ms; both well inside this. */
#define MAG_DRDY_TIMEOUT_MS     500

/* Datasheet gain table, indexed by the GN field of CONFIG_B. */
static const float mag_lsb_per_gauss[8] = {
    1370.0f, 1090.0f, 820.0f, 660.0f, 440.0f, 390.0f, 330.0f, 230.0f
};
static const float mag_range_gauss[8] = {
    0.88f, 1.3f, 1.9f, 2.5f, 4.0f, 4.7f, 5.6f, 8.1f
};

struct mag_dev {
    i2c_master_dev_handle_t dev;
    int        addr;
    mag_chip_t chip;
    int        gn;      /* HMC5883L gain field, read back after configuring */
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

/* ------------------------------------------------------------- detection */

static bool is_hmc5883l(i2c_master_dev_handle_t dev)
{
    /* The three ID registers spell "H43" on a genuine HMC5883L. */
    uint8_t id[3] = {0};
    return reg_read(dev, HMC5883L_REG_ID_A, id, sizeof(id)) == ESP_OK &&
           id[0] == 'H' && id[1] == '4' && id[2] == '3';
}

static bool is_qmc5883l(i2c_master_dev_handle_t dev)
{
    uint8_t id = 0;
    return reg_read(dev, QMC5883L_REG_CHIP_ID, &id, 1) == ESP_OK && id == QMC5883L_CHIP_ID;
}

/*
 * Find a magnetometer and add it to the bus. Each address is probed before
 * it is talked to: i2c_master_probe() asks quietly, where a register read of
 * an empty address makes the driver log a NACK error for what is, here, an
 * expected answer.
 *
 * With an explicit address the chip is identified by its ID registers. If
 * neither ID matches - clones exist of both - the address decides, with a
 * warning, which is how the HMC5883L code behaved before there was a choice.
 */
static esp_err_t mag_open(int want_addr, i2c_master_dev_handle_t *dev, int *addr,
                          mag_chip_t *chip)
{
    const int candidates[2] = { HMC5883L_ADDR_DEFAULT, QMC5883L_ADDR_DEFAULT };
    const int n = (want_addr >= 0) ? 1 : 2;

    if (tool_bus_handle == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    for (int i = 0; i < n; i++) {
        int a = (want_addr >= 0) ? want_addr : candidates[i];
        if (i2c_master_probe(tool_bus_handle, a, MAG_TIMEOUT_MS) != ESP_OK) {
            continue;
        }
        i2c_device_config_t dev_conf = {
            .scl_speed_hz = MAG_SCL_SPEED_HZ,
            .device_address = a,
        };
        if (i2c_master_bus_add_device(tool_bus_handle, &dev_conf, dev) != ESP_OK) {
            return ESP_FAIL;
        }
        *addr = a;
        if (is_hmc5883l(*dev)) {
            *chip = MAG_CHIP_HMC5883L;
        } else if (is_qmc5883l(*dev)) {
            *chip = MAG_CHIP_QMC5883L;
        } else {
            *chip = (a == QMC5883L_ADDR_DEFAULT) ? MAG_CHIP_QMC5883L : MAG_CHIP_HMC5883L;
            ESP_LOGW(TAG, "0x%02x matches neither ID ('H43' at 0x0A, 0xFF at 0x0D); "
                          "treating it as a %s by its address", a,
                     *chip == MAG_CHIP_QMC5883L ? "QMC5883L" : "HMC5883L");
        }
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

/* --------------------------------------------------------------- HMC5883L */

static esp_err_t hmc5883l_configure(mag_dev_t *d)
{
    /* Set the output rate explicitly rather than inheriting whatever the chip
       happens to hold. An ESP32 reset does not power-cycle this sensor, so its
       config survives a reflash and the first run after a cold power-up starts
       from different state than every run after it. */
    if (reg_write(d->dev, HMC5883L_REG_CONFIG_A, HMC5883L_CONFIG_A_DEFAULT) != ESP_OK) {
        ESP_LOGE(TAG, "failed to set the output rate");
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* Continuous mode keeps fresh samples coming without re-triggering each
       read. The power-on default is single-measurement, which takes one
       reading and then idles - leaving the data registers static. */
    if (reg_write(d->dev, HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS) != ESP_OK) {
        ESP_LOGE(TAG, "failed to select continuous mode");
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint8_t mode_check = 0xFF;
    if (reg_read(d->dev, HMC5883L_REG_MODE, &mode_check, 1) != ESP_OK ||
        (mode_check & 0x03) != HMC5883L_MODE_CONTINUOUS) {
        ESP_LOGE(TAG, "mode register reads 0x%02x, not continuous - aborting rather "
                      "than printing a frozen sample repeatedly", mode_check);
        return ESP_ERR_INVALID_STATE;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // first conversion after a mode change

    /* Read the gain back rather than assuming the reset default, so scaling
       stays correct if it was changed with 'i2cset'. */
    uint8_t conf_b = 0;
    if (reg_read(d->dev, HMC5883L_REG_CONFIG_B, &conf_b, 1) != ESP_OK) {
        ESP_LOGE(TAG, "failed to read the gain configuration");
        return ESP_ERR_INVALID_RESPONSE;
    }
    d->gn = (conf_b >> 5) & 0x07;
    return ESP_OK;
}

/* --------------------------------------------------------------- QMC5883L */

static esp_err_t qmc5883l_configure(mag_dev_t *d)
{
    /* Soft reset first, for the same reason the HMC5883L path writes its
       config every run: an ESP32 reset does not reach this chip, so whatever
       an earlier run or 'i2cset' left in it would otherwise carry over. */
    if (reg_write(d->dev, QMC5883L_REG_CONTROL_2, QMC5883L_SOFT_RESET) != ESP_OK) {
        ESP_LOGE(TAG, "soft reset failed");
        return ESP_ERR_INVALID_RESPONSE;
    }
    vTaskDelay(2);   // at least one full 10 ms tick; power-on time is 350 us

    if (reg_write(d->dev, QMC5883L_REG_SET_RESET, QMC5883L_SET_RESET_PERIOD) != ESP_OK ||
        reg_write(d->dev, QMC5883L_REG_CONTROL_1, QMC5883L_CONTROL_1_DEFAULT) != ESP_OK) {
        ESP_LOGE(TAG, "failed to write the configuration");
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* Range and mode decide the scale and whether samples keep coming, so
       read both back and refuse to report numbers scaled by a guess. */
    uint8_t ctrl1 = 0, setreset = 0;
    if (reg_read(d->dev, QMC5883L_REG_CONTROL_1, &ctrl1, 1) != ESP_OK ||
        reg_read(d->dev, QMC5883L_REG_SET_RESET, &setreset, 1) != ESP_OK ||
        ctrl1 != QMC5883L_CONTROL_1_DEFAULT || setreset != QMC5883L_SET_RESET_PERIOD) {
        ESP_LOGE(TAG, "configuration did not stick (control 1 reads 0x%02x, want 0x%02x; "
                      "set/reset 0x%02x) - aborting", ctrl1, QMC5883L_CONTROL_1_DEFAULT, setreset);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

/* ---------------------------------------------------------------- driver */

esp_err_t mag_attach(int addr, mag_dev_t **out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    i2c_master_dev_handle_t dev = NULL;
    int found = 0;
    mag_chip_t chip = MAG_CHIP_HMC5883L;
    esp_err_t err = mag_open(addr, &dev, &found, &chip);
    if (err != ESP_OK) {
        return err;
    }
    mag_dev_t *d = calloc(1, sizeof(*d));
    if (d == NULL) {
        i2c_master_bus_rm_device(dev);
        return ESP_ERR_NO_MEM;
    }
    d->dev  = dev;
    d->addr = found;
    d->chip = chip;
    err = (chip == MAG_CHIP_HMC5883L) ? hmc5883l_configure(d) : qmc5883l_configure(d);
    if (err != ESP_OK) {
        mag_detach(d);
        return err;
    }
    *out = d;
    return ESP_OK;
}

void mag_info(const mag_dev_t *d, mag_info_t *info)
{
    info->chip = d->chip;
    info->addr = d->addr;
    if (d->chip == MAG_CHIP_HMC5883L) {
        info->name          = "HMC5883L";
        info->range_gauss   = mag_range_gauss[d->gn];
        info->lsb_per_gauss = mag_lsb_per_gauss[d->gn];
    } else {
        info->name          = "QMC5883L";
        info->range_gauss   = QMC5883L_RANGE_GAUSS;
        info->lsb_per_gauss = QMC5883L_LSB_PER_GAUSS;
    }
}

esp_err_t mag_read(mag_dev_t *d, int timeout_ms, mag_reading_t *r)
{
    if (d == NULL || r == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const bool hmc = (d->chip == MAG_CHIP_HMC5883L);
    const uint8_t status_reg = hmc ? HMC5883L_REG_STATUS : QMC5883L_REG_STATUS;
    const uint8_t ready_bit  = hmc ? HMC5883L_STATUS_RDY : QMC5883L_STATUS_DRDY;

    /* Wait for DRDY instead of trusting an interval. Reading the data
       registers clears this bit, so it only comes back when the chip has
       actually taken a new measurement - which turns a frozen sensor into a
       reported error rather than a plausible-looking repeated row. */
    uint8_t status = 0;
    int waited_ms = 0;
    while (1) {
        esp_err_t err = reg_read(d->dev, status_reg, &status, 1);
        if (err != ESP_OK) {
            return err;
        }
        if (status & ready_bit) {
            break;
        }
        if (waited_ms >= timeout_ms) {
            if (timeout_ms > 0) {
                ESP_LOGE(TAG, "no new sample after %d ms (status 0x%02x) - the sensor "
                              "is not measuring", timeout_ms, status);
            }
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        waited_ms += 10;
    }

    uint8_t raw[6];
    int16_t mx, my, mz;
    float lsb_per_gauss;
    if (hmc) {
        /* Data registers are X, then Z, then Y - not X, Y, Z. */
        esp_err_t err = reg_read(d->dev, HMC5883L_REG_DATA_X_H, raw, sizeof(raw));
        if (err != ESP_OK) {
            return err;
        }
        mx = (int16_t)((raw[0] << 8) | raw[1]);
        mz = (int16_t)((raw[2] << 8) | raw[3]);
        my = (int16_t)((raw[4] << 8) | raw[5]);
        r->overflow = (mx == HMC5883L_OVERFLOW || my == HMC5883L_OVERFLOW ||
                       mz == HMC5883L_OVERFLOW);
        lsb_per_gauss = mag_lsb_per_gauss[d->gn];
    } else {
        /* X, Y, Z in order, each little-endian - the opposite of the HMC5883L
           on both counts. */
        esp_err_t err = reg_read(d->dev, QMC5883L_REG_DATA_X_L, raw, sizeof(raw));
        if (err != ESP_OK) {
            return err;
        }
        mx = (int16_t)(raw[0] | (raw[1] << 8));
        my = (int16_t)(raw[2] | (raw[3] << 8));
        mz = (int16_t)(raw[4] | (raw[5] << 8));
        /* The QMC5883L has no magic overflow value; it raises OVL instead,
           and then the numbers are clipped, not measurements. */
        r->overflow = (status & QMC5883L_STATUS_OVL) != 0;
        lsb_per_gauss = QMC5883L_LSB_PER_GAUSS;
    }

    r->mx_gauss = mx / lsb_per_gauss;
    r->my_gauss = my / lsb_per_gauss;
    r->mz_gauss = mz / lsb_per_gauss;
    float heading = atan2f(r->my_gauss, r->mx_gauss) * 180.0f / (float)M_PI;
    if (heading < 0) {
        heading += 360.0f;
    }
    r->heading_deg = heading;
    return ESP_OK;
}

void mag_detach(mag_dev_t *d)
{
    if (d == NULL) {
        return;
    }
    if (d->dev != NULL) {
        i2c_master_bus_rm_device(d->dev);
    }
    free(d);
}

/* ---------------------------------------------------------------- console */

/* One table row in the common format. Units: uT; 1 Gauss = 100 uT, and
   Earth's field is roughly 25-65 uT. The heading assumes the chip lies flat,
   with its own X axis forward. */
static void print_field(const mag_reading_t *r)
{
    float fx = r->mx_gauss * 100.0f, fy = r->my_gauss * 100.0f, fz = r->mz_gauss * 100.0f;
    printf("%9.2f %9.2f %9.2f %9.2f %9.1f\n",
           fx, fy, fz, sqrtf(fx * fx + fy * fy + fz * fz), r->heading_deg);
}

static int run_mag(mag_dev_t *d, int count, int interval)
{
    mag_info_t info;
    mag_info(d, &info);
    if (info.chip == MAG_CHIP_HMC5883L) {
        printf("HMC5883L @ 0x%02x   range +/-%.2f Ga   %.0f LSB/Ga\n",
               info.addr, info.range_gauss, info.lsb_per_gauss);
    } else {
        printf("QMC5883L @ 0x%02x   range +/-%.0f G   %.0f LSB/G   50 Hz, OSR 512\n",
               info.addr, info.range_gauss, info.lsb_per_gauss);
    }
    printf("%9s %9s %9s %9s %9s\n", "mx[uT]", "my[uT]", "mz[uT]", "|m|[uT]", "hdg[deg]");

    for (int i = 0; i < count; i++) {
        mag_reading_t r;
        esp_err_t err = mag_read(d, MAG_DRDY_TIMEOUT_MS, &r);
        if (err == ESP_ERR_TIMEOUT) {
            return 1;   /* mag_read already said why */
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "read failed at sample %d", i);
            return 1;
        }

        if (r.overflow) {
            printf("%9s %9s %9s %9s %9s  (overflow - %s)\n", "-", "-", "-", "-", "-",
                   info.chip == MAG_CHIP_HMC5883L ? "raise the gain range"
                                                  : "field beyond +/-8 G");
        } else {
            print_field(&r);
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
    struct arg_end *end;
} mag_args;

static int do_mag_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&mag_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, mag_args.end, argv[0]);
        return 1;
    }

    int want_addr = mag_args.chip_address->count ? mag_args.chip_address->ival[0] : -1;
    int count     = mag_args.count->count    ? mag_args.count->ival[0]    : 10;
    int interval  = mag_args.interval->count ? mag_args.interval->ival[0] : 200;

    /* The stream holds this chip while it runs; borrow it for the command. */
    sensor_stream_pause();

    int ret = 1;
    mag_dev_t *d = NULL;
    esp_err_t err = mag_attach(want_addr, &d);
    if (err == ESP_OK) {
        ret = run_mag(d, count, interval);
    } else if (err == ESP_FAIL) {
        ESP_LOGE(TAG, "could not add the device to the bus");
    } else if (err == ESP_ERR_NOT_FOUND && want_addr >= 0) {
        ESP_LOGE(TAG, "no response from 0x%02x - check wiring and 'i2cdetect'", want_addr);
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "no magnetometer at 0x1E or 0x0D - check wiring and 'i2cdetect'");
    }
    mag_detach(d);
    sensor_stream_resume();
    return ret;
}

void register_mag(void)
{
    mag_args.chip_address = arg_int0("c", "chip", "<addr>", "I2C address (default: 0x1E, then 0x0D)");
    mag_args.count        = arg_int0("n", "count", "<n>", "Samples to print (default 10)");
    mag_args.interval     = arg_int0("i", "interval", "<ms>", "Delay between samples (default 200)");
    mag_args.end          = arg_end(2);

    const esp_console_cmd_t mag_cmd = {
        .command = "mag",
        .help = "Read the magnetometer (HMC5883L or QMC5883L, found automatically) "
                "and print field strength and heading",
        .hint = NULL,
        .func = &do_mag_cmd,
        .argtable = &mag_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&mag_cmd));
}
