/*
 * Background RGB status LED for the i2c-tools console.
 *
 * Carried over from the blink example, with one deliberate difference: this
 * version never logs periodically. The console shares its USB Serial/JTAG
 * stream with the log, so a recurring ESP_LOGI would overwrite whatever the
 * user is typing at the prompt.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "led_blink.h"

static const char *TAG = "status-led";

/* One full revolution of the hue wheel every CONFIG_EXAMPLE_BLINK_PERIOD ms. */
#define LED_REFRESH_PERIOD_MS 20
#define LED_SATURATION        255
#define LED_BRIGHTNESS        16

static led_strip_handle_t s_led_strip;

static void led_blink_task(void *arg)
{
    while (1) {
        /* Derive the hue from the clock rather than accumulating it, so the
           sweep keeps a steady rate no matter how the refresh period divides
           the cycle. */
        uint32_t elapsed_ms = pdTICKS_TO_MS(xTaskGetTickCount()) % CONFIG_EXAMPLE_BLINK_PERIOD;
        uint16_t hue = (uint16_t)((elapsed_ms * 360) / CONFIG_EXAMPLE_BLINK_PERIOD);

        led_strip_set_pixel_hsv(s_led_strip, 0, hue, LED_SATURATION, LED_BRIGHTNESS);
        led_strip_refresh(s_led_strip);

        vTaskDelay(LED_REFRESH_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

void led_blink_start(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_EXAMPLE_BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LED init failed (%s); continuing without it", esp_err_to_name(err));
        return;
    }
    led_strip_clear(s_led_strip);

    if (xTaskCreate(led_blink_task, "status_led", 3072, NULL, 2, NULL) != pdPASS) {
        ESP_LOGW(TAG, "could not start LED task; continuing without it");
        return;
    }
    ESP_LOGI(TAG, "Status LED sweeping on GPIO%d", CONFIG_EXAMPLE_BLINK_GPIO);
}
