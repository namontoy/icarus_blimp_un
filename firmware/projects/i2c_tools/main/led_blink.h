/*
 * Background RGB status LED for the i2c-tools console.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the status LED sweeping hues in its own task.
 *
 * Safe to call before the console REPL starts. On failure the LED is simply
 * left dark and the caller continues: a dead status LED must never take the
 * I2C console down with it.
 */
void led_blink_start(void);

#ifdef __cplusplus
}
#endif
