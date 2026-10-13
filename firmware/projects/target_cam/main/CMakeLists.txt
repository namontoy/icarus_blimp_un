idf_component_register(SRCS "target_main.c"
                            "blob_detect.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_timer
                       INCLUDE_DIRS ".")
