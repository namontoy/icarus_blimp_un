# ST's VL53L1X ULD core plus a platform layer for the esp-idf i2c_master API.
idf_component_register(SRCS "st/VL53L1X_api.c"
                            "st/VL53L1X_calibration.c"
                            "port/vl53l1x_port.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c)
