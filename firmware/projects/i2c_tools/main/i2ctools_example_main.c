/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cmd_system.h"
#include "cmd_i2ctools.h"
#include "cmd_imu.h"
#include "cmd_mag.h"
#include "cmd_baro.h"
#include "cmd_tof.h"
#include "cmd_motor.h"
#include "driver/i2c_master.h"
#include "led_blink.h"
#include "net_wifi.h"
#include "net_link.h"
#include "net_console.h"
#include "sensor_stream.h"

static const char *TAG = "i2c-tools";

static gpio_num_t i2c_gpio_sda = CONFIG_EXAMPLE_I2C_MASTER_SDA;
static gpio_num_t i2c_gpio_scl = CONFIG_EXAMPLE_I2C_MASTER_SCL;

static i2c_port_t i2c_port = I2C_NUM_0;

#if CONFIG_EXAMPLE_STORE_HISTORY

#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"

static void initialize_filesystem(void)
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = true
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}
#endif // CONFIG_EXAMPLE_STORE_HISTORY

/*
 * 'exit' / 'quit' only explain how to leave. The REPL runs on the chip, and
 * the program holding the serial port open is idf.py monitor on the laptop:
 * nothing the chip prints can make it close. People type 'exit' by habit, so
 * the answer belongs where they look. The console over UDP runs these same
 * commands in its own task, and there the monitor's keys mean nothing, so the
 * hint follows whichever console asked.
 */
static int do_exit_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (strcmp(pcTaskGetName(NULL), "net_console") == 0) {
        printf("This console runs on the blimp and has nothing to close here.\n"
               "blimp_link.py's interactive console leaves on 'exit' or Ctrl+C.\n");
    } else {
        printf("This console runs on the blimp; it can't close the monitor on your laptop.\n"
               "To leave idf.py monitor press Ctrl+X (or Ctrl+T, then x).\n"
               "That also frees the serial port. Ctrl+C is passed to the chip, not the monitor.\n");
    }
    return 0;
}

static void register_exit(void)
{
    const char *names[] = { "exit", "quit" };
    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        const esp_console_cmd_t cmd = {
            .command = names[i],
            .help = "How to leave this console (the monitor exits with Ctrl+X)",
            .hint = NULL,
            .func = &do_exit_cmd,
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    }
}

void app_main(void)
{
    /*
     * Motors first, before the console, the filesystem and the I2C bus.
     *
     * Nothing here needs the motor driver this early - the eight DRV8833
     * input pins do. From chip reset they are high-impedance inputs, and with
     * VM already live off the step-up that leaves both bridges in a state
     * nobody commanded. Registering the command is not enough: it only adds
     * a console entry, and the pins would stay floating until someone typed
     * 'motor' for the first time. Initialising here bounds that window to
     * the few milliseconds of early boot.
     *
     * Not ESP_ERROR_CHECK: a motor driver that fails to come up must not cost
     * us the console, which is how we would find out why.
     */
    esp_err_t motor_err = motors_init();
    if (motor_err != ESP_OK) {
        ESP_LOGE(TAG, "motor driver failed to start: %s - thrusters unavailable",
                 esp_err_to_name(motor_err));
    }

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

#if CONFIG_EXAMPLE_STORE_HISTORY
    initialize_filesystem();
    repl_config.history_save_path = HISTORY_PATH;
#endif
    repl_config.prompt = "i2c-tools>";

    // install console REPL environment
#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#endif

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .scl_io_num = i2c_gpio_scl,
        .sda_io_num = i2c_gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &tool_bus_handle));

    register_i2ctools();
    register_imu();
    register_mag();
    register_baro();
    register_tof();
    register_motor();     /* already initialised above */
    register_wifi();
    register_link();
    register_sensors();
    register_exit();

    /*
     * The wireless link comes up after every command is registered: the
     * console over UDP runs the same registry as the USB REPL, so the first
     * datagram can already be 'help'. Nothing here is fatal. A bench
     * instrument without a radio is still a bench instrument, and the USB
     * console is how we would find out what went wrong.
     */
    esp_err_t net_err = net_wifi_init();
    if (net_err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi unavailable: %s - serial console only", esp_err_to_name(net_err));
    } else {
        if (net_link_start() != ESP_OK) {
            ESP_LOGE(TAG, "UDP link unavailable");
        }
        if (net_console_start() != ESP_OK) {
            ESP_LOGE(TAG, "console over UDP unavailable");
        }
    }

    /*
     * The sensor stream feeds the link's SENSORS packet, so it starts after
     * the link. It samples through the same drivers as the console commands
     * and hands the chips over whenever one of them runs (sensor_stream.h).
     * CONFIG_SENSOR_STREAM_AT_BOOT=n leaves the chips untouched until
     * 'sensors --on'. Not fatal either: the console commands still work.
     */
#if CONFIG_SENSOR_STREAM_AT_BOOT
    const bool stream_at_boot = true;
#else
    const bool stream_at_boot = false;   /* a disabled Kconfig bool is undefined, not 0 */
#endif
    if (sensor_stream_start(stream_at_boot) != ESP_OK) {
        ESP_LOGE(TAG, "sensor stream unavailable - console sensor commands only");
    }

    /* Status LED runs in its own task, so the REPL below stays responsive. */
    led_blink_start();

    printf("\n ==============================================================\n");
    printf(" |             Steps to Use i2c-tools                         |\n");
    printf(" |                                                            |\n");
    printf(" |  1. Try 'help', check all supported commands               |\n");
    printf(" |  2. Try 'i2cconfig' to configure your I2C bus              |\n");
    printf(" |  3. Try 'i2cdetect' to scan devices on the bus             |\n");
    printf(" |  4. Try 'i2cget' to get the content of specific register   |\n");
    printf(" |  5. Try 'i2cset' to set the value of specific register     |\n");
    printf(" |  6. Try 'i2cdump' to dump all the register (Experiment)    |\n");
    printf(" |  7. Try 'imu' for decoded MPU-6050 readings                |\n");
    printf(" |  8. Try 'mag' for HMC5883L field and heading               |\n");
    printf(" |  9. Try 'baro' for BMP180 pressure and altitude            |\n");
    printf(" | 10. Try 'tof' for VL53L1X time-of-flight range             |\n");
    printf(" | 11. Try 'motor' to drive the DRV8833 thrusters             |\n");
    printf(" | 12. Try 'wifi' and 'link' for the Wi-Fi ground link        |\n");
    printf(" |     (software/blimp_link.py runs all of this over the air) |\n");
    printf(" | 13. Try 'sensors' for the stream the ground station plots  |\n");
    printf(" | 14. Type 'exit' or 'quit' to see how to leave the monitor  |\n");
    printf(" |                                                            |\n");
    printf(" ==============================================================\n\n");

    // start console REPL
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
