# tools/lens_test/esp32cam/main/CMakeLists.txt - the "main" component.
#
# The camera driver (espressif/esp32-camera) comes from the IDF component
# registry through main/idf_component.yml; the component manager adds it to this
# component's requirements itself, so it is not listed here.
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi          # soft AP / station
        esp_event         # default event loop for Wi-Fi and IP events
        esp_netif         # esp_netif_init(), IP info for the log line
        esp_http_server   # the two HTTP servers (control on :80, stream on :81)
        esp_timer         # uptime in /info
        nvs_flash         # Wi-Fi calibration storage
        driver            # GPIO for the flash LED, LEDC types in camera_config_t
)
