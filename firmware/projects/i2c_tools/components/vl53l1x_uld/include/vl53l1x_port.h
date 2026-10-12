/*
 * Platform layer binding ST's VL53L1X ULD to an esp-idf i2c_master bus.
 *
 * ST's ULD calls the VL53L1_* functions declared in vl53l1_platform.h and
 * identifies a sensor by an opaque uint16_t. This port uses the sensor's
 * 7-bit I2C address as that token, and keeps a small table mapping it to the
 * i2c_master device handle.
 *
 * The bus is supplied by the caller rather than created here, so the sensor
 * can share a bus with other devices that are already on it.
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Default 7-bit address. Not persistent: it reverts on every power cycle. */
#define VL53L1X_ADDR_DEFAULT 0x29

/**
 * Add the sensor to an existing bus and make it reachable by the ULD.
 *
 * @param bus       bus the caller owns and keeps alive until detach
 * @param addr_7bit sensor address; also the @c dev token to pass to the ULD
 * @return ESP_ERR_NO_MEM if the device table is full, ESP_ERR_INVALID_STATE
 *         if this address is already attached.
 */
esp_err_t vl53l1x_port_attach(i2c_master_bus_handle_t bus, uint16_t addr_7bit);

/** Release the device. Safe to call with an address that is not attached. */
void vl53l1x_port_detach(uint16_t addr_7bit);

#ifdef __cplusplus
}
#endif
