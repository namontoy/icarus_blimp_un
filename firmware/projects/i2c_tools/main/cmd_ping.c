/*
 * 'ping' console command: Parallax PING))) 28015 ultrasonic rangefinder.
 *
 * PARKED, not built. Dropped from main/CMakeLists.txt and from the console
 * while the motor wiring took GPIO 7-14; the sensor needs a pin assigned
 * again before it comes back. To restore it: add "cmd_ping.c" to the SRCS
 * list, and the include, register_ping() call and banner line to
 * i2ctools_example_main.c. The code itself is unchanged and was working.
 *
 * Measures the echo pulse on the sensor's single bidirectional SIG line and
 * converts it to distance, correcting the speed of sound with air temperature
 * read from the BMP180 already sitting on the I2C bus.
 *
 * Wiring - the ESP32-S3 is not 5 V tolerant and PING))) drives SIG at 5 V:
 *
 *     PING))) 5V   ---------------------------- 5V rail
 *     PING))) GND  ---------------------------- GND
 *     PING))) SIG  --- 1.0k ---+--------------- GPIO (default 38)
 *                              |
 *                             1.8k
 *                              |
 *                             GND
 *
 * The divider lands a 5 V echo at 3.2 V on the pin and holds the line low
 * while neither side drives it. The trigger still reaches full 3.3 V because
 * SIG is a high-impedance input at that moment, and the datasheet accepts it:
 * "can communicate with 5 V TTL or 3.3 V CMOS microcontrollers".
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cmd_baro.h"
#include "cmd_ping.h"

static const char *TAG = "cmd_ping";

/* GPIO38, a back pad: the pin docs/inter-board-link-plan.md gives the
   PING))). It was 6 until 2026-10-30, when 6 became the forward camera's
   UART receive pin (cam_link.c, the Cableado's assignment); 4/5 are the I2C
   bus, 7-14 the motors, 21 the RGB LED, 19/20 USB. */
#define PING_GPIO_DEFAULT       38

/* Datasheet timings (28015 v1.5). */
#define PING_TRIGGER_US         5       // 2 us min, 5 us typical
#define PING_HOLDOFF_US         750
#define PING_BURST_US           200
#define PING_ECHO_MIN_US        115
#define PING_ECHO_MAX_US        18500

/* Holdoff + burst + longest echo is 19.45 ms. pdMS_TO_TICKS rounds down and
   the first tick of a delay can land almost immediately, so at this build's
   10 ms tick a 20 ms timeout could expire after only 10 ms of real waiting -
   aborting a measurement that was still in flight. 40 ms gives at least three
   whole ticks past the worst case. */
#define PING_WAIT_MS            40

/* Used only when the barometer cannot be reached; 20 C puts the speed of
   sound at 343.5 m/s. */
#define PING_FALLBACK_TEMP_C    20.0f

/* The hover band this airframe is being built for. Printed per sample so a
   bench run answers "would the controller see itself in range" directly. */
#define PING_BAND_LOW_CM        185.0f
#define PING_BAND_HIGH_CM       225.0f

/* Bounds the static sample buffer below, and the REPL cannot interrupt a
   running command, so an unbounded count would wedge the console. */
#define PING_MAX_SAMPLES        200

/* Edge timestamps, filled by the ISR. The first edge seen after arming is the
   rising one and the second is the falling one, so the handler never has to
   read the pin level - which would race with the edge that woke it. */
static struct {
    volatile int64_t t_rise;
    volatile int64_t t_fall;
    volatile bool    armed;
    volatile bool    have_rise;
    TaskHandle_t     waiter;
} s_cap;

static bool      s_isr_installed;
static gpio_num_t s_isr_pin = GPIO_NUM_NC;

static void IRAM_ATTR ping_isr(void *arg)
{
    (void)arg;
    if (!s_cap.armed) {
        return;             // our own trigger pulse, not the sensor's echo
    }
    int64_t now = esp_timer_get_time();

    if (!s_cap.have_rise) {
        s_cap.t_rise = now;
        s_cap.have_rise = true;
        return;
    }
    s_cap.t_fall = now;
    s_cap.armed = false;

    BaseType_t hp_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_cap.waiter, &hp_woken);
    if (hp_woken) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t ping_attach(gpio_num_t pin)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        /* The external 1.8k already holds the line low. This weak pull-down
           only matters if that resistor is missing, where it keeps the pin
           from floating between measurements. */
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }

    if (!s_isr_installed) {
        err = gpio_install_isr_service(0);
        /* Another component may already own the service; that is fine. */
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return err;
        }
        s_isr_installed = true;
    }
    if (s_isr_pin != pin) {
        if (s_isr_pin != GPIO_NUM_NC) {
            gpio_isr_handler_remove(s_isr_pin);
        }
        err = gpio_isr_handler_add(pin, ping_isr, NULL);
        if (err != ESP_OK) {
            return err;
        }
        s_isr_pin = pin;
    }
    return ESP_OK;
}

/*
 * One measurement. Returns the round-trip echo width in microseconds.
 *
 * ESP_ERR_NOT_FOUND - SIG never rose: the sensor is not answering at all.
 * ESP_ERR_TIMEOUT   - SIG rose but never fell: no echo came back in range.
 * Telling those apart matters, because the first is a wiring fault and the
 * second is a legitimate "nothing within 3 m".
 */
static esp_err_t ping_measure_us(gpio_num_t pin, int *echo_us)
{
    s_cap.waiter = xTaskGetCurrentTaskHandle();
    s_cap.have_rise = false;
    s_cap.armed = false;

    /* Trigger. The ISR stays disarmed across it so our own two edges are not
       mistaken for the echo; the line was already low, so switching back to
       an input produces no edge of its own. */
    gpio_set_level(pin, 0);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(PING_TRIGGER_US);
    gpio_set_level(pin, 0);
    gpio_set_direction(pin, GPIO_MODE_INPUT);

    xTaskNotifyStateClear(NULL);
    s_cap.armed = true;

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PING_WAIT_MS)) == 0) {
        s_cap.armed = false;
        return s_cap.have_rise ? ESP_ERR_TIMEOUT : ESP_ERR_NOT_FOUND;
    }

    int64_t width = s_cap.t_fall - s_cap.t_rise;
    if (width < PING_ECHO_MIN_US) {
        /* Below the datasheet minimum this is a glitch on the line, not a
           2 cm target. Reporting it as a distance would invent a reading. */
        return ESP_ERR_INVALID_RESPONSE;
    }
    *echo_us = (int)width;
    return ESP_OK;
}

static int cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static struct {
    struct arg_int *pin;
    struct arg_int *count;
    struct arg_int *interval;
    struct arg_dbl *temperature;
    struct arg_end *end;
} ping_args;

static float s_dist_cm[PING_MAX_SAMPLES];

static int do_ping_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&ping_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, ping_args.end, argv[0]);
        return 1;
    }

    int pin_num  = ping_args.pin->count      ? ping_args.pin->ival[0]      : PING_GPIO_DEFAULT;
    int count    = ping_args.count->count    ? ping_args.count->ival[0]    : 10;
    int interval = ping_args.interval->count ? ping_args.interval->ival[0] : 100;

    if (!GPIO_IS_VALID_GPIO(pin_num)) {
        ESP_LOGE(TAG, "GPIO %d is not usable on this chip", pin_num);
        return 1;
    }
    if (count < 1 || count > PING_MAX_SAMPLES) {
        ESP_LOGE(TAG, "count must be 1..%d", PING_MAX_SAMPLES);
        return 1;
    }
    if (interval < 10) {
        /* The sensor itself only needs 200 us, but reverberation from the
           previous burst has to die away or it shows up as a short echo. */
        ESP_LOGE(TAG, "interval must be at least 10 ms");
        return 1;
    }

    /* Air temperature changes the speed of sound by 0.6 m/s per degree: about
       0.17 %/C, or 3.5 cm at 2 m over a 10 C error. That is a sixth of the
       40 cm hover band, so it is measured rather than assumed. */
    float temp_c;
    const char *temp_src;
    if (ping_args.temperature->count) {
        temp_c = (float)ping_args.temperature->dval[0];
        temp_src = "given";
    } else if (baro_read_temperature_c(&temp_c) == ESP_OK) {
        temp_src = "BMP180";
    } else {
        temp_c = PING_FALLBACK_TEMP_C;
        temp_src = "assumed - barometer unreachable";
        ESP_LOGW(TAG, "could not read the BMP180; falling back to %.1f C", temp_c);
    }
    float c_air = 331.5f + 0.6f * temp_c;       // m/s, datasheet page 3

    gpio_num_t pin = (gpio_num_t)pin_num;
    esp_err_t err = ping_attach(pin);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not set up GPIO %d: %s", pin_num, esp_err_to_name(err));
        return 1;
    }

    printf("PING))) on GPIO %d   air %.1f C (%s)   c = %.1f m/s\n",
           pin_num, temp_c, temp_src, c_air);
    printf("%10s %10s %8s\n", "echo[us]", "dist[cm]", "band");

    int valid = 0, no_echo = 0, no_reply = 0, glitch = 0;
    for (int i = 0; i < count; i++) {
        int echo_us = 0;
        err = ping_measure_us(pin, &echo_us);

        if (err == ESP_OK && echo_us > PING_ECHO_MAX_US) {
            /* The sensor gives up at its 18.5 ms ceiling, which is the same
               signal as "nothing reflected": over 3 m, too shallow an angle,
               or a surface that swallowed the burst. */
            printf("%10d %10s %8s  (over 3 m / no reflection)\n", echo_us, "-", "-");
            no_echo++;
        } else if (err == ESP_OK) {
            /* Half the round trip, metres to centimetres. */
            float cm = (echo_us / 2.0f) * c_air * 1e-4f;
            const char *band = (cm < PING_BAND_LOW_CM)  ? "LOW"
                             : (cm > PING_BAND_HIGH_CM) ? "HIGH" : "OK";
            printf("%10d %10.1f %8s\n", echo_us, cm, band);
            s_dist_cm[valid++] = cm;
        } else if (err == ESP_ERR_TIMEOUT) {
            printf("%10s %10s %8s  (echo never ended)\n", "-", "-", "-");
            no_echo++;
        } else if (err == ESP_ERR_INVALID_RESPONSE) {
            printf("%10s %10s %8s  (pulse under %d us - line glitch)\n",
                   "-", "-", "-", PING_ECHO_MIN_US);
            glitch++;
        } else {
            printf("%10s %10s %8s  (no response on SIG)\n", "-", "-", "-");
            no_reply++;
        }
        fflush(stdout);

        if (i + 1 < count) {
            vTaskDelay(pdMS_TO_TICKS(interval));
        }
    }

    if (no_reply == count) {
        ESP_LOGE(TAG, "SIG never went high. Check the 5 V supply, the ground "
                      "shared with the board, and that SIG reaches GPIO %d.", pin_num);
        return 1;
    }

    if (valid > 0) {
        float sum = 0.0f;
        for (int i = 0; i < valid; i++) {
            sum += s_dist_cm[i];
        }
        float mean = sum / valid;

        float var = 0.0f;
        for (int i = 0; i < valid; i++) {
            float d = s_dist_cm[i] - mean;
            var += d * d;
        }
        /* Sample standard deviation; a single reading has no spread to report. */
        float sd = (valid > 1) ? sqrtf(var / (valid - 1)) : 0.0f;

        qsort(s_dist_cm, valid, sizeof(s_dist_cm[0]), cmp_float);
        float median = (valid % 2) ? s_dist_cm[valid / 2]
                                   : (s_dist_cm[valid / 2 - 1] + s_dist_cm[valid / 2]) / 2.0f;

        printf("\n%d/%d valid   min %.1f  median %.1f  mean %.1f  max %.1f  sd %.1f cm\n",
               valid, count, s_dist_cm[0], median, mean, s_dist_cm[valid - 1], sd);
    }
    if (no_echo || no_reply || glitch) {
        printf("dropped: %d out of range, %d no response, %d glitch\n",
               no_echo, no_reply, glitch);
    }
    return 0;
}

void register_ping(void)
{
    ping_args.pin         = arg_int0("p", "pin", "<gpio>", "SIG GPIO (default 6)");
    ping_args.count       = arg_int0("n", "count", "<n>", "Samples to take (default 10)");
    ping_args.interval    = arg_int0("i", "interval", "<ms>", "Delay between samples (default 100)");
    ping_args.temperature = arg_dbl0("t", "temp", "<C>", "Air temperature; default reads the BMP180");
    ping_args.end         = arg_end(2);

    const esp_console_cmd_t ping_cmd = {
        .command = "ping",
        .help = "Read the PING))) 28015 ultrasonic rangefinder",
        .hint = NULL,
        .func = &do_ping_cmd,
        .argtable = &ping_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ping_cmd));
}
