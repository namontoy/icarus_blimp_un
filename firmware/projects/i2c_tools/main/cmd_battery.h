/*
 * cmd_battery.h - the motor battery's voltage, through the divider on
 * CONFIG_BATTERY_ADC_GPIO (GPIO1, ADC1_CH0, by default), and the 'battery'
 * console command.
 *
 * The divider is the wiring plan's 100 k + 100 k with 100 nF to GND
 * (docs/s3-wroom-cam-wiring-plan.md), so a full 4.2 V cell puts 2.1 V on the
 * pin, inside the ADC's 12 dB range. The control task samples it once per
 * tick in its read stage (ctl_task.c), the schedule of
 * docs/control-loop-plan.md section 3; this module owns the ADC unit and
 * the running average, and keeps a copy of the latest reading for the
 * console. Only the control task calls battery_sample(), so the ADC has one
 * user and needs no lock of its own.
 *
 * ADC1 and not ADC2: on the ESP32-S3 the Wi-Fi driver takes ADC2, and a
 * oneshot read of it fails while the radio is up.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     valid;             /* at least one calibrated sample since boot      */
    uint16_t raw;               /* last ADC count, 12 bits                        */
    uint16_t pin_mv;            /* last calibrated voltage at the pin             */
    uint16_t bat_mv;            /* last sample after the divider ratio            */
    uint16_t avg_mv;            /* running average after the divider ratio        */
    uint8_t  avg_n;             /* samples in the average, up to CONFIG_BATTERY_AVG_TICKS */
    uint32_t samples;           /* calibrated samples since boot                  */
    uint32_t failures;          /* reads or conversions that failed               */
    int64_t  t_us;              /* esp_timer time of the last good sample         */
} battery_reading_t;

/**
 * Create the ADC1 unit, configure the channel at 12 dB and the calibration
 * scheme. Call once, before ctl_task_start(). Not fatal when it fails:
 * battery_sample() then returns false and the records carry 0 mV.
 */
esp_err_t battery_init(void);

/**
 * One conversion, for the control task's read stage: no logging, no
 * allocation, nothing that waits (the oneshot driver takes the unit with a
 * try-lock and holds a spinlock for the conversion, tens of microseconds).
 * Writes the raw count, the pin voltage and the running average after the
 * divider; returns true when this call brought a new calibrated sample and
 * leaves the outputs as they were otherwise.
 */
bool battery_sample(uint16_t *raw, uint16_t *pin_mv, uint16_t *avg_mv);

/** A copy of the latest reading; `valid` is false until the first good sample. */
void battery_latest(battery_reading_t *out);

/** The 'battery' console command. */
void register_battery(void);

#ifdef __cplusplus
}
#endif
