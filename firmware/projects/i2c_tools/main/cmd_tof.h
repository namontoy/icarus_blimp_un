/*
 * 'tof' console command and the VL53L1X time-of-flight driver behind it.
 *
 * The driver is attach / read / detach over ST's ULD; the console command
 * and the sensor stream (sensor_stream.c) are both loops on top of it. The
 * ULD port refuses a second attach at the same address, so a caller that
 * runs while the stream holds the sensor takes the lease first, see
 * sensor_stream_pause() in sensor_stream.h.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register the 'tof' command with the console. */
void register_tof(void);

typedef struct tof_dev tof_dev_t;

#define TOF_MODE_SHORT  1   /* to about 1.3 m, immune to ambient light      */
#define TOF_MODE_LONG   2   /* to about 4 m in the dark, less under ambient */

typedef struct {
    uint16_t distance_mm;
    uint8_t  status;            /* ULD range status, 0 = valid; text from
                                   tof_status_text()                        */
    uint16_t signal_kcps_spad;  /* return signal rate per SPAD              */
    uint16_t ambient_kcps_spad; /* ambient rate per SPAD: what decides the
                                   reach, see the .c file                    */
    uint16_t spads;             /* SPADs enabled for the measurement        */
} tof_reading_t;

typedef struct {
    int      addr;
    uint16_t model_id;          /* 0xEACC on a VL53L1X                      */
    int      mode;              /* TOF_MODE_*                               */
    int      budget_ms;
    int      inter_ms;
} tof_info_t;

/**
 * Attach the sensor to the shared bus, wait for its boot, initialise it,
 * apply mode, timing budget and inter-measurement period (read back), and
 * start continuous ranging. addr < 0 means the default 0x29. The budget must
 * pass tof_budget_valid() and inter_ms must be at least the budget.
 *
 * @return ESP_ERR_NOT_FOUND, quietly, if nothing answers at the address;
 *         ESP_ERR_INVALID_STATE if the address is already attached (the
 *         stream holds it, or a command did not detach); other errors are
 *         logged here.
 */
esp_err_t tof_attach(int addr, int mode, int budget_ms, int inter_ms, tof_dev_t **out);

void tof_info(const tof_dev_t *d, tof_info_t *info);

/**
 * The newest frame. Waits up to timeout_ms for data-ready (0 checks once);
 * reading on a schedule that drifted from the sensor's own cadence would
 * hand back the previous frame as if it were new.
 *
 * @return ESP_ERR_TIMEOUT when no frame arrived in time (not logged),
 *         ESP_ERR_INVALID_RESPONSE on a failed read (logged).
 */
esp_err_t tof_read(tof_dev_t *d, int timeout_ms, tof_reading_t *r);

/** Stop ranging, release the address and free the handle. NULL is fine. */
void tof_detach(tof_dev_t *d);

/** True for the budgets the ULD accepts; 15 ms is short mode only. */
bool tof_budget_valid(int budget_ms, int mode);

/** ST's range status as text. */
const char *tof_status_text(uint8_t status);

#ifdef __cplusplus
}
#endif
