/*
 * sensor_stream.h - samples every sensor on the bench at a fixed rate and
 * keeps the latest reading for the ground link, which sends it as
 * LINK_PKT_SENSORS. Also the 'sensors' console command.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "link_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Create the lease and the sampling task. With enabled false the task idles
 * and touches no chip until 'sensors --on'. Needs the I2C bus; does not need
 * the link, which simply has nothing to send until a sample exists.
 */
esp_err_t sensor_stream_start(bool enabled);

/**
 * Borrow the sensors. Waits for the current sampling cycle to finish, then
 * releases every chip the stream holds (devices removed from the bus, the
 * ToF address detached) and keeps the stream out until sensor_stream_resume().
 * Every console command that touches a sensor, and 'i2cconfig', brackets its
 * work with these two. Safe to call before the stream was started.
 */
void sensor_stream_pause(void);

/** Give the sensors back. The stream re-attaches and reconfigures them on
    its next cycle, so nothing the command left behind leaks into the data. */
void sensor_stream_resume(void);

/**
 * The newest sample as a SENSORS payload: every field but the header and the
 * CRC, which are the link's business. seq moves once per sample, so a caller
 * that remembers the last value it sent knows whether this one is new.
 *
 * @return false until the first sample exists.
 */
bool sensor_stream_latest(link_sensors_t *out, uint32_t *seq);

/** Register the 'sensors' command with the console. */
void register_sensors(void);

#ifdef __cplusplus
}
#endif
