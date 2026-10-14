/*
 * settings - the hand-calibrated detector settings, kept in NVS.
 *
 * Calibration is done at the bench with the PC viewer, but the camera flies
 * with nobody at a PC. So whatever was tuned has to survive a power cycle, and
 * the boot banner has to say whether the board is running saved settings or
 * the Kconfig defaults - otherwise a flight on forgotten defaults looks exactly
 * like a flight on a careful calibration.
 *
 * The record is a versioned, fixed-size struct. A record of another version or
 * size is ignored (defaults are used, and the banner says so) rather than
 * reinterpreted. The one exception is the v1 record, which settings_load()
 * migrates field by field: see there.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int16_t hue;            /* blob_cfg_t fields                              */
    int16_t hue_tol;
    int16_t sat_min;
    int16_t val_min;
    int32_t min_area;
    int32_t target_mm;      /* 0 = range off                                  */
    float   hfov_deg;
    int16_t period_ms;      /* frame period; v1 stored an integer fps here    */
    int8_t  awb;            /* 1 auto, 0 frozen                               */
    int8_t  big_endian;
    int8_t  preset;         /* blob_preset_t, or -1 for hand-tuned            */
} tc_settings_t;

/* Initialise NVS. A partition that is full or from a newer IDF is erased:
   losing a calibration is better than a camera that will not boot. */
esp_err_t settings_init(void);

/* true and *out filled if a valid saved record exists. *migrated is set when
   it was a v1 record, whose frame rate was replaced by the build default. */
bool settings_load(tc_settings_t *out, bool *migrated);

esp_err_t settings_save(const tc_settings_t *s);

/* Remove the saved record, so the next boot uses the Kconfig defaults. */
esp_err_t settings_erase(void);
