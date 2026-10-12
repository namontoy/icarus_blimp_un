idf_component_register(SRCS "i2ctools_example_main.c"
                            "cmd_i2ctools.c"
                            "led_blink.c"
                            "cmd_imu.c"
                            "cmd_mag.c"
                            "cmd_baro.c"
                            "cmd_motor.c"
                            "cmd_tof.c"
                       PRIV_REQUIRES fatfs esp_driver_i2c esp_driver_gpio esp_driver_ledc esp_timer vl53l1x_uld
                       INCLUDE_DIRS ".")
