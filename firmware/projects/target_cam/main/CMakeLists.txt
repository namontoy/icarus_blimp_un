idf_component_register(SRCS "target_main.c"
                            "blob_detect.c"
                            "link.c"
                            "settings.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_timer nvs_flash
                       INCLUDE_DIRS ".")
