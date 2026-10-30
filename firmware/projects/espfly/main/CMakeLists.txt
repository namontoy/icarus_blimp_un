# firmware/main/CMakeLists.txt - the "main" ESP-IDF component.
#
# Not one of the numbered deliverables in the task this tree was built
# under - it is added because every ESP-IDF component, including "main",
# needs one of these or the project simply does not configure. Without it,
# board_config.h / main.c / drivers/ / net/ would not be reachable at all.
#
# INCLUDE_DIRS lists "." / "drivers" / "net" so:
#   - main.c can #include "board_config.h", "drivers/imu.h", "net/espnow_link.h"
#   - files inside drivers/ and net/ can #include "board_config.h" (in ".")
#     and each other's own headers by bare name (same directory, resolves
#     without needing INCLUDE_DIRS at all, but listed anyway for clarity)
idf_component_register(
    SRCS
        "main.c"
        "drivers/imu.c"
        "drivers/baro.c"
        "drivers/mag.c"
        "drivers/motor.c"
        "net/espnow_link.c"
    INCLUDE_DIRS
        "."
        "drivers"
        "net"
    REQUIRES
        core        # the portable control core (../../core, registered via
                    # firmware/CMakeLists.txt's EXTRA_COMPONENT_DIRS)
        driver      # legacy I2C (driver/i2c.h), LEDC (driver/ledc.h - motor.c's
                    # four PWM channels into the MOSFET gates) and GPIO
                    # constants - all bundled under the "driver" component.
                    # No MCPWM any more: this airframe has no ESC/servo, so
                    # the newer driver/mcpwm_prelude.h dropped out with servo.c.
        esp_adc     # ADC oneshot (esp_adc/adc_oneshot.h) - a separate
                    # component from "driver" as of ESP-IDF v5
        esp_wifi    # esp_wifi.h AND esp_now.h - ESP-NOW is part of the
                    # esp_wifi component, not a separate "esp_now" component
        esp_event   # esp_event_loop_create_default(), needed by the WiFi
                    # init sequence esp_now bring-up depends on
        esp_netif   # esp_netif_init(), same reason
        esp_timer   # esp_timer.h - drives the fixed control period
        nvs_flash   # nvs_flash_init() - required by WiFi calibration data
    # NOT listed, and NOT needed: "freertos", "esp_common", "esp_system",
    # "log", "esp_hw_support" and a handful of others are part of ESP-IDF's
    # automatic "common" component requirements, linked into every
    # component without an explicit REQUIRES entry (documented in the
    # ESP-IDF v5.x build-system reference).
)
