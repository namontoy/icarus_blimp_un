idf_component_register(SRCS "optflow_main.c"
                            "flow_match.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_timer
                       INCLUDE_DIRS ".")
