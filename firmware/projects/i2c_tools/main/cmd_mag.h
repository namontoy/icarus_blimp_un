/*
 * 'mag' console command: decoded magnetometer readings from an HMC5883L
 * (0x1E) or a QMC5883L (0x0D), found automatically.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Register the 'mag' command with the console. */
void register_mag(void);

#ifdef __cplusplus
}
#endif
