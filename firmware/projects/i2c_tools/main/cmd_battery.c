/*
 * cmd_battery.c - the motor battery's voltage on CONFIG_BATTERY_ADC_GPIO.
 *
 * One oneshot conversion per control tick, at 12 dB attenuation (about
 * 0-3.1 V at the pin on the S3) and the chip's default 12-bit width, turned
 * into millivolts by the curve-fitting scheme from the eFuse calibration,
 * then scaled by the divider ratio and averaged over the last
 * CONFIG_BATTERY_AVG_TICKS samples.
 *
 * Why average here when the divider already carries 100 nF: the capacitor
 * against the divider's 50 k source (tau 5 ms) smooths the motor PWM ripple
 * and gives the ADC's sample capacitor a stiff source, but the cell itself
 * sags with each change of thrust, and the cutoff (docs/control-loop-plan.md
 * D6, increment 3) is meant to act on the resting level, not on a step. The
 * average is a boxcar so its delay is plain (half the window, 250 ms at the
 * default 25 ticks); the cutoff adds its own 2 s hold on top.
 *
 * Not verified on the bench: whether the divider is wired on the current
 * bench (plan section 7), and how close the curve-fitting result comes to a
 * multimeter at 3.0-4.2 V. The ratio is the nominal 2.0 until then.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "cmd_battery.h"

static const char *TAG = "battery";

#define BATTERY_ATTEN   ADC_ATTEN_DB_12
#define BATTERY_WIDTH   ADC_BITWIDTH_DEFAULT

static struct {
    adc_oneshot_unit_handle_t unit;
    adc_cali_handle_t         cali;     /* NULL: no calibration, raw counts only */
    adc_channel_t             chan;
    bool                      ready;

    /* boxcar average, owned by the control task */
    uint16_t ring[CONFIG_BATTERY_AVG_TICKS];
    uint32_t sum;
    uint8_t  head;
    uint8_t  n;

    portMUX_TYPE      mux;              /* guards `latest`                        */
    battery_reading_t latest;
} s = {
    .mux = portMUX_INITIALIZER_UNLOCKED,
};

esp_err_t battery_init(void)
{
    if (s.ready) {
        return ESP_OK;
    }
    adc_unit_t unit;
    esp_err_t err = adc_oneshot_io_to_channel(CONFIG_BATTERY_ADC_GPIO, &unit, &s.chan);
    if (err != ESP_OK || unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "GPIO%d is not an ADC1 pin", CONFIG_BATTERY_ADC_GPIO);
        return ESP_ERR_INVALID_ARG;
    }

    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    err = adc_oneshot_new_unit(&unit_cfg, &s.unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 unit: %s", esp_err_to_name(err));
        return err;
    }
    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = BATTERY_ATTEN,
        .bitwidth = BATTERY_WIDTH,
    };
    err = adc_oneshot_config_channel(s.unit, s.chan, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 channel %d: %s", (int)s.chan, esp_err_to_name(err));
        adc_oneshot_del_unit(s.unit);
        s.unit = NULL;
        return err;
    }

    /* Without the eFuse calibration the raw count is still worth having on
       the console, but no millivolts are made up from a nominal slope: the
       S3's uncalibrated ADC is off by up to a couple of hundred mV, which is
       half the span between a full and a flat cell. */
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    const adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = s.chan,
        .atten    = BATTERY_ATTEN,
        .bitwidth = BATTERY_WIDTH,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s.cali);
#else
    err = ESP_ERR_NOT_SUPPORTED;
#endif
    if (err != ESP_OK) {
        s.cali = NULL;
        ESP_LOGW(TAG, "no ADC calibration (%s): raw counts only, no voltage",
                 esp_err_to_name(err));
    }

    s.ready = true;
    ESP_LOGI(TAG, "GPIO%d (ADC1_CH%d), 12 dB, divider x%d.%03d, average of %d ticks%s",
             CONFIG_BATTERY_ADC_GPIO, (int)s.chan,
             CONFIG_BATTERY_DIVIDER_PERMILLE / 1000, CONFIG_BATTERY_DIVIDER_PERMILLE % 1000,
             CONFIG_BATTERY_AVG_TICKS, s.cali ? "" : ", uncalibrated");
    return ESP_OK;
}

bool battery_sample(uint16_t *raw, uint16_t *pin_mv, uint16_t *avg_mv)
{
    if (!s.ready) {
        return false;
    }
    int counts = 0;
    int mv = 0;
    const bool read_ok = adc_oneshot_read(s.unit, s.chan, &counts) == ESP_OK;
    const bool good = read_ok && s.cali != NULL &&
                      adc_cali_raw_to_voltage(s.cali, counts, &mv) == ESP_OK && mv >= 0;
    if (!good) {
        portENTER_CRITICAL(&s.mux);
        s.latest.failures += s.cali != NULL || !read_ok;   /* uncalibrated is not a failure */
        if (read_ok) {
            s.latest.raw = (uint16_t)counts;
        }
        portEXIT_CRITICAL(&s.mux);
        return false;
    }

    /* 3.1 V at the pin times a ratio of at most 10 stays inside 16 bits. */
    const uint16_t bat = (uint16_t)((uint32_t)mv * CONFIG_BATTERY_DIVIDER_PERMILLE / 1000u);
    if (s.n == CONFIG_BATTERY_AVG_TICKS) {
        s.sum -= s.ring[s.head];
    } else {
        s.n++;
    }
    s.ring[s.head] = bat;
    s.sum += bat;
    s.head = (uint8_t)((s.head + 1) % CONFIG_BATTERY_AVG_TICKS);
    const uint16_t avg = (uint16_t)((s.sum + s.n / 2) / s.n);

    *raw    = (uint16_t)counts;
    *pin_mv = (uint16_t)mv;
    *avg_mv = avg;

    portENTER_CRITICAL(&s.mux);
    s.latest.valid  = true;
    s.latest.raw    = (uint16_t)counts;
    s.latest.pin_mv = (uint16_t)mv;
    s.latest.bat_mv = bat;
    s.latest.avg_mv = avg;
    s.latest.avg_n  = s.n;
    s.latest.samples++;
    s.latest.t_us   = esp_timer_get_time();
    portEXIT_CRITICAL(&s.mux);
    return true;
}

void battery_latest(battery_reading_t *out)
{
    portENTER_CRITICAL(&s.mux);
    *out = s.latest;
    portEXIT_CRITICAL(&s.mux);
}

/* --------------------------------------------------------------- console */

static int do_battery_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!s.ready) {
        printf("battery monitor not running - see the boot log for why\n");
        return 1;
    }
    battery_reading_t r;
    battery_latest(&r);
    printf("GPIO%d (ADC1_CH%d), 12 dB, divider x%d.%03d (nominal, not yet checked "
           "against a meter)\n",
           CONFIG_BATTERY_ADC_GPIO, (int)s.chan,
           CONFIG_BATTERY_DIVIDER_PERMILLE / 1000, CONFIG_BATTERY_DIVIDER_PERMILLE % 1000);
    if (!r.valid) {
        printf("no calibrated sample yet (%s; raw %u, %" PRIu32 " failures); the control "
               "task samples once per tick\n",
               s.cali ? "calibrated" : "no eFuse calibration", r.raw, r.failures);
        return 1;
    }
    const uint32_t age_ms = (uint32_t)((esp_timer_get_time() - r.t_us) / 1000);
    printf("battery %u.%03u V (average of %u)   last %u.%03u V   pin %u mV   raw %u\n",
           r.avg_mv / 1000, r.avg_mv % 1000, r.avg_n,
           r.bat_mv / 1000, r.bat_mv % 1000, r.pin_mv, r.raw);
    printf("%" PRIu32 " samples, %" PRIu32 " failures, last %" PRIu32 " ms ago\n",
           r.samples, r.failures, age_ms);
    /* With no divider the pin floats and can read anything from zero to a
       few hundred mV (reasoned, not measured); no 1S cell in use gets near
       1 V, so below that nothing sensible is connected. */
    if (r.avg_mv < 1000) {
        printf("below 1 V: the divider is probably not wired to GPIO%d\n",
               CONFIG_BATTERY_ADC_GPIO);
    }
    return 0;
}

void register_battery(void)
{
    const esp_console_cmd_t cmd = {
        .command = "battery",
        .help = "Motor battery voltage on the divider (sampled by the control task)",
        .hint = NULL,
        .func = &do_battery_cmd,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
