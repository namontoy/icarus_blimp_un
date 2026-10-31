/*
 * motor_map.h - the motor map kept in NVS, and the 'motormap' command
 * (motor_map.c; the map itself is cmd_motor.h's).
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Apply the map saved in NVS, if there is one; after motors_init(). */
void motor_map_load(void);

/** Register the 'motormap' command with the console. */
void register_motormap(void);

#ifdef __cplusplus
}
#endif
