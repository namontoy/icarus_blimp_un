/*
 * 'tof' console command: VL53L1X time-of-flight rangefinder, through ST's ULD.
 *
 * Shares the bus that 'i2cconfig' set up, like the rest of the tool. The ULD
 * itself is in components/vl53l1x_uld, with a platform layer that binds it to
 * that bus instead of creating one of its own.
 *
 * Scope note: this sensor cannot hold the 2 m hover band - under normal room
 * ambient its long-mode reach collapses to roughly 1.1-1.7 m, which is why
 * the PING))) does altitude. It earns its place at close range: obstacle and
 * wall proximity, and ground proximity on landing, where a 2 degree-ish spot
 * and millimetre resolution beat a wide ultrasonic cone. The ambient column
 * below is the number that predicts its reach, so a run tells you directly
 * whether the light in the room leaves it any range to work with.
 *
 * Wiring - 3.3 V part, so it goes straight onto the existing bus:
 *
 *     VL53L1X VIN  -> 3V3        VL53L1X SDA -> GPIO4 (shared)
 *     VL53L1X GND  -> GND        VL53L1X SCL -> GPIO5 (shared)
 *     VL53L1X XSHUT-> optional, see -x
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "esp_console.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "VL53L1X_api.h"
#include "vl53l1x_port.h"
#include "cmd_i2ctools.h"
#include "cmd_tof.h"

static const char *TAG = "cmd_tof";

/* VL53L1X_IDENTIFICATION__MODEL_ID. A VL53L0X reads 0xEEAA here instead. */
#define TOF_MODEL_ID            0xEACC

/* FIRMWARE__SYSTEM_STATUS is non-zero once the sensor has booted. ST's header
   documents this as "1:booted", but that is not literal - the part on this
   bench reports 3 - and ST's own reference waits for any non-zero value. */
#define TOF_BOOT_TIMEOUT_MS     200

#define TOF_MODE_SHORT          1
#define TOF_MODE_LONG           2

/* Bounds the static sample buffer, and the REPL cannot interrupt a running
   command, so an unbounded count would wedge the console. */
#define TOF_MAX_SAMPLES         200

/* Above this, the datasheet's range tables show reach falling off sharply:
   long mode drops to about 1.5 m at 50 kcps/SPAD and 0.7 m at 200. */
#define TOF_AMBIENT_WARN_KCPS   50

/* The only timing budgets the ULD accepts; 15 ms is short mode only. */
static const uint16_t tof_budgets[] = { 15, 20, 33, 50, 100, 200, 500 };

/* ULD range status, after VL53L1X_GetResult() has mapped the raw register
   through ST's status_rtn table. */
static const char *tof_status_text(uint8_t status)
{
    switch (status) {
    case 0:   return "valid";
    case 1:   return "sigma fail";
    case 2:   return "signal fail";
    case 3:   return "below min range";
    case 4:   return "out of bounds";
    case 5:   return "hardware fail";
    case 6:   return "no update";
    case 7:   return "wraparound";
    case 8:   return "processing fail";
    case 9:   return "xtalk fail";
    case 10:  return "sync error";
    case 11:  return "merged pulse";
    case 12:  return "signal too low";
    case 13:  return "other";
    case 255: return "no measurement";
    default:  return "unknown";
    }
}

static bool tof_budget_valid(int budget, int mode)
{
    for (size_t i = 0; i < sizeof(tof_budgets) / sizeof(tof_budgets[0]); i++) {
        if (budget == tof_budgets[i]) {
            /* The 15 ms budget is not long enough for a long-mode ranging
               cycle; the ULD rejects the combination. */
            return !(budget == 15 && mode == TOF_MODE_LONG);
        }
    }
    return false;
}

static int cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static struct {
    struct arg_int *chip_address;
    struct arg_int *count;
    struct arg_int *mode;
    struct arg_int *budget;
    struct arg_int *inter;
    struct arg_int *xshut;
    struct arg_end *end;
} tof_args;

static float s_dist_mm[TOF_MAX_SAMPLES];

static int do_tof_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tof_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, tof_args.end, argv[0]);
        return 1;
    }

    int chip_addr = tof_args.chip_address->count ? tof_args.chip_address->ival[0]
                                                 : VL53L1X_ADDR_DEFAULT;
    int count  = tof_args.count->count  ? tof_args.count->ival[0]  : 10;
    int mode   = tof_args.mode->count   ? tof_args.mode->ival[0]   : TOF_MODE_LONG;
    int budget = tof_args.budget->count ? tof_args.budget->ival[0] : 100;
    int inter  = tof_args.inter->count  ? tof_args.inter->ival[0]  : budget;

    if (count < 1 || count > TOF_MAX_SAMPLES) {
        ESP_LOGE(TAG, "count must be 1..%d", TOF_MAX_SAMPLES);
        return 1;
    }
    if (mode != TOF_MODE_SHORT && mode != TOF_MODE_LONG) {
        ESP_LOGE(TAG, "mode must be 1 (short, to ~1.3 m) or 2 (long, to ~4 m)");
        return 1;
    }
    if (!tof_budget_valid(budget, mode)) {
        ESP_LOGE(TAG, "budget must be one of 20, 33, 50, 100, 200, 500 ms "
                      "(15 ms is accepted in short mode only)");
        return 1;
    }
    if (inter < budget) {
        /* The sensor needs at least the budget between measurements; asking
           for less is quietly rejected and leaves ranging stalled. */
        ESP_LOGE(TAG, "inter-measurement period (%d ms) must be >= the timing "
                      "budget (%d ms)", inter, budget);
        return 1;
    }

    /* Many breakouts pull XSHUT up and need nothing. Where it is pulled down,
       or tied to a GPIO, the sensor will not answer until it is driven high -
       which otherwise looks exactly like missing wiring. */
    if (tof_args.xshut->count) {
        int xshut = tof_args.xshut->ival[0];
        if (!GPIO_IS_VALID_OUTPUT_GPIO(xshut)) {
            ESP_LOGE(TAG, "GPIO %d cannot drive an output", xshut);
            return 1;
        }
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << xshut,
            .mode = GPIO_MODE_OUTPUT,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&io) != ESP_OK) {
            ESP_LOGE(TAG, "could not configure XSHUT on GPIO %d", xshut);
            return 1;
        }
        gpio_set_level(xshut, 1);
        vTaskDelay(pdMS_TO_TICKS(20));   // let the sensor come out of reset
    }

    esp_err_t err = vl53l1x_port_attach(tool_bus_handle, (uint16_t)chip_addr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not attach 0x%02x: %s", chip_addr, esp_err_to_name(err));
        return 1;
    }
    uint16_t dev = (uint16_t)chip_addr;   // the ULD's device token is the address

    int ret = 1;
    bool ranging = false;

    uint16_t model_id = 0;
    if (VL53L1X_GetSensorId(dev, &model_id) != 0) {
        ESP_LOGE(TAG, "no response from 0x%02x - check wiring, 'i2cdetect', and "
                      "XSHUT (see -x)", chip_addr);
        goto cleanup;
    }
    if (model_id != TOF_MODEL_ID) {
        ESP_LOGW(TAG, "model ID reads 0x%04x, expected 0x%04x for a VL53L1X",
                 model_id, TOF_MODEL_ID);
    }

    /* ST's sequence: do not configure the sensor until it says it has booted.
       Bounded, so a wedged sensor reports rather than hangs the console. */
    uint8_t boot_state = 0;
    int waited_ms = 0;
    while (1) {
        if (VL53L1X_BootState(dev, &boot_state) != 0) {
            ESP_LOGE(TAG, "boot state read failed");
            goto cleanup;
        }
        if (boot_state != 0) {
            break;
        }
        if (waited_ms >= TOF_BOOT_TIMEOUT_MS) {
            ESP_LOGE(TAG, "still not booted after %d ms (state %u)",
                     TOF_BOOT_TIMEOUT_MS, boot_state);
            goto cleanup;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        waited_ms += 10;
    }

    /* SensorInit writes the whole 91-byte default configuration and runs one
       throwaway ranging cycle, so it is slow but it leaves the chip in a known
       state - which matters here for the same reason it did on the GY-88: a
       reset of the ESP32 does not power-cycle the sensor, so whatever the last
       run left behind would otherwise still be in effect. */
    if (VL53L1X_SensorInit(dev) != 0) {
        ESP_LOGE(TAG, "sensor init failed");
        goto cleanup;
    }
    if (VL53L1X_SetDistanceMode(dev, (uint16_t)mode) != 0 ||
        VL53L1X_SetTimingBudgetInMs(dev, (uint16_t)budget) != 0 ||
        VL53L1X_SetInterMeasurementInMs(dev, (uint16_t)inter) != 0) {
        ESP_LOGE(TAG, "could not apply mode/timing configuration");
        goto cleanup;
    }

    /* Read the configuration back rather than trusting the writes, the same
       rule the imu and mag commands follow: distances scaled from assumed
       state are distances that can be silently wrong. */
    uint16_t rb_mode = 0, rb_budget = 0;
    if (VL53L1X_GetDistanceMode(dev, &rb_mode) != 0 ||
        VL53L1X_GetTimingBudgetInMs(dev, &rb_budget) != 0) {
        ESP_LOGE(TAG, "configuration readback failed");
        goto cleanup;
    }
    if (rb_mode != mode || rb_budget != budget) {
        ESP_LOGW(TAG, "readback mismatch: asked for mode=%d budget=%d, chip "
                      "reports mode=%u budget=%u", mode, budget, rb_mode, rb_budget);
    }

    if (VL53L1X_StartRanging(dev) != 0) {
        ESP_LOGE(TAG, "could not start ranging");
        goto cleanup;
    }
    ranging = true;

    printf("VL53L1X @ 0x%02x   model 0x%04x   %s mode   budget %d ms   period %d ms\n",
           chip_addr, model_id, (mode == TOF_MODE_SHORT) ? "short" : "long", budget, inter);
    printf("%9s %17s %12s %12s %7s\n",
           "dist[mm]", "status", "sig[kcps/sp]", "amb[kcps/sp]", "spads");

    int valid = 0, bad = 0, dropped = 0;
    long amb_sum = 0;

    for (int i = 0; i < count; i++) {
        /* Wait for the sensor to publish a frame instead of trusting the
           period. Measuring on a schedule that has drifted from the sensor's
           own cadence reprints the previous frame as if it were new. */
        uint8_t ready = 0;
        int poll_ms = 0;
        const int poll_limit = inter + budget + 100;
        while (1) {
            if (VL53L1X_CheckForDataReady(dev, &ready) != 0) {
                ESP_LOGE(TAG, "data-ready check failed at sample %d", i);
                goto cleanup;
            }
            if (ready) {
                break;
            }
            if (poll_ms >= poll_limit) {
                ESP_LOGE(TAG, "no frame after %d ms - ranging has stalled", poll_limit);
                goto cleanup;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            poll_ms += 10;
        }

        /* One 17-byte read for a coherent frame, then the two per-SPAD rates.
           All three come from the same frame because the result registers hold
           still until the interrupt is cleared below. */
        VL53L1X_Result_t result;
        uint16_t sig_per_spad = 0, amb_per_spad = 0;
        if (VL53L1X_GetResult(dev, &result) != 0 ||
            VL53L1X_GetSignalPerSpad(dev, &sig_per_spad) != 0 ||
            VL53L1X_GetAmbientPerSpad(dev, &amb_per_spad) != 0) {
            ESP_LOGE(TAG, "result read failed at sample %d", i);
            goto cleanup;
        }
        if (VL53L1X_ClearInterrupt(dev) != 0) {
            ESP_LOGE(TAG, "could not clear the interrupt at sample %d", i);
            goto cleanup;
        }

        amb_sum += amb_per_spad;
        if (result.Status == 0) {
            printf("%9u %17s %12u %12u %7u\n", result.Distance,
                   tof_status_text(result.Status), sig_per_spad, amb_per_spad,
                   result.NumSPADs);
            s_dist_mm[valid++] = (float)result.Distance;
        } else {
            /* A non-zero status means the sensor is telling us this range is
               not trustworthy, so it is not printed as a distance. */
            printf("%9s %17s %12u %12u %7u\n", "-",
                   tof_status_text(result.Status), sig_per_spad, amb_per_spad,
                   result.NumSPADs);
            bad++;
        }
        fflush(stdout);
    }
    dropped = bad;

    if (valid > 0) {
        float sum = 0.0f;
        for (int i = 0; i < valid; i++) {
            sum += s_dist_mm[i];
        }
        float mean = sum / valid;

        float var = 0.0f;
        for (int i = 0; i < valid; i++) {
            float d = s_dist_mm[i] - mean;
            var += d * d;
        }
        float sd = (valid > 1) ? sqrtf(var / (valid - 1)) : 0.0f;

        qsort(s_dist_mm, valid, sizeof(s_dist_mm[0]), cmp_float);
        float median = (valid % 2) ? s_dist_mm[valid / 2]
                                   : (s_dist_mm[valid / 2 - 1] + s_dist_mm[valid / 2]) / 2.0f;

        printf("\n%d/%d valid   min %.0f  median %.0f  mean %.1f  max %.0f  sd %.1f mm\n",
               valid, count, s_dist_mm[0], median, mean, s_dist_mm[valid - 1], sd);
    }
    if (dropped) {
        printf("%d sample(s) reported an unusable range\n", dropped);
    }

    /* Ambient is what decides how far this sensor can see, so say so when it
       is high rather than leaving a short reading looking like a short room. */
    long amb_mean = amb_sum / count;
    if (amb_mean > TOF_AMBIENT_WARN_KCPS) {
        printf("ambient averages %ld kcps/SPAD: at that level the datasheet puts "
               "long-mode reach near 1.5 m or below, so treat anything further "
               "as out of range rather than absent\n", amb_mean);
    }
    ret = 0;

cleanup:
    if (ranging) {
        VL53L1X_StopRanging(dev);
    }
    vl53l1x_port_detach(dev);
    return ret;
}

void register_tof(void)
{
    tof_args.chip_address = arg_int0("c", "chip", "<addr>", "I2C address (default 0x29)");
    tof_args.count        = arg_int0("n", "count", "<n>", "Samples to print (default 10)");
    tof_args.mode         = arg_int0("d", "mode", "<1|2>", "1 short (~1.3 m), 2 long (~4 m, default)");
    tof_args.budget       = arg_int0("b", "budget", "<ms>", "Timing budget 20/33/50/100/200/500 (default 100)");
    tof_args.inter        = arg_int0("i", "interval", "<ms>", "Inter-measurement period (default = budget)");
    tof_args.xshut        = arg_int0("x", "xshut", "<gpio>", "Drive XSHUT high on this GPIO first");
    tof_args.end          = arg_end(2);

    const esp_console_cmd_t tof_cmd = {
        .command = "tof",
        .help = "Read the VL53L1X time-of-flight rangefinder",
        .hint = NULL,
        .func = &do_tof_cmd,
        .argtable = &tof_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&tof_cmd));
}
