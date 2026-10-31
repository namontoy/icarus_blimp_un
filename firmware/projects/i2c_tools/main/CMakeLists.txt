idf_component_register(SRCS "i2ctools_example_main.c"
                            "cmd_i2ctools.c"
                            "led_blink.c"
                            "cmd_imu.c"
                            "cmd_mag.c"
                            "cmd_baro.c"
                            "cmd_motor.c"
                            "cmd_tof.c"
                            "net_wifi.c"
                            "net_link.c"
                            "net_console.c"
                            "sensor_stream.c"
                            "ctl_task.c"
                            "capture.c"
                            "cmd_battery.c"
                            "param_table.c"
                            "cmd_param.c"
                            "motor_map.c"
                            "cam_link.c"
                            "cam_row.c"
                       PRIV_REQUIRES fatfs esp_driver_i2c esp_driver_gpio esp_driver_ledc esp_driver_uart esp_timer vl53l1x_uld esp_adc
                                     console esp_wifi esp_netif esp_event nvs_flash lwip esp_psram esp_app_format
                                     core
                       INCLUDE_DIRS ".")
