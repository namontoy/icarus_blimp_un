/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

#ifdef CONFIG_BLINK_LED_STRIP

/* The on-board LED is a single RGB pixel, so instead of toggling it on and off
   we walk the hue wheel: one full revolution every CONFIG_BLINK_PERIOD ms. */
#define LED_REFRESH_PERIOD_MS 20
#define LED_SATURATION        255
#define LED_BRIGHTNESS        16

static led_strip_handle_t led_strip;

static void update_led(void)
{
    /* Derive the hue from the clock rather than accumulating it, so the sweep
       keeps a steady rate no matter how the refresh period divides the cycle. */
    uint32_t elapsed_ms = pdTICKS_TO_MS(xTaskGetTickCount()) % CONFIG_BLINK_PERIOD;
    uint16_t hue = (uint16_t)((elapsed_ms * 360) / CONFIG_BLINK_PERIOD);

    ESP_ERROR_CHECK(led_strip_set_pixel_hsv(led_strip, 0, hue, LED_SATURATION, LED_BRIGHTNESS));
    /* Refresh the strip to send data */
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));

    /* One line per revolution; logging every refresh would swamp the console. */
    static uint16_t s_prev_hue = 0;
    if (hue < s_prev_hue) {
        ESP_LOGI(TAG, "Hue sweep running (%u ms per revolution)", (unsigned)CONFIG_BLINK_PERIOD);
    }
    s_prev_hue = hue;
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to sweep the hue of the addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

/* A plain GPIO LED has no color to sweep, so it just blinks. */
#define LED_REFRESH_PERIOD_MS CONFIG_BLINK_PERIOD

static uint8_t s_led_state = 0;

static void update_led(void)
{
    ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
    /* Toggle the LED state */
    s_led_state = !s_led_state;
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();

    while (1) {
        update_led();
        vTaskDelay(LED_REFRESH_PERIOD_MS / portTICK_PERIOD_MS);
    }
}
