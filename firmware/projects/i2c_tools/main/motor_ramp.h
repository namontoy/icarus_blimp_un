/*
 * motor_ramp.h - the motor driver's soft start: how fast one output's duty
 * may rise. Portable C with no ESP-IDF, so tools/fw_host_test.py checks it on
 * the host (firmware/projects/i2c_tools/host/test_motor_ramp.c); cmd_motor.c
 * steps it on every command and on a 5 ms timer between commands.
 *
 * Why (2026-10-31, docs/history/envelope-2026-10-31.md): a motor at rest has
 * no back-EMF, so at a given duty it draws several times its running current.
 * The team's bench numbers (big propeller, 1.83 / 2.59 / 3.71 V at 0.78 /
 * 1.25 / 2.02 A) fit a winding of about 0.85 ohm, so a start from rest at the
 * 40 % ceiling asks roughly 1.8 A of a driver channel the team keeps under
 * 1.2 A, where the running current is about 0.7 A. That day the rear pair,
 * which draws a little more than the front, stopped starting above 28-34 %
 * duty: no current at all, not even with a push, while below that both
 * started and, once turning, both ran at 40 %. So a start begins at the
 * deadband and the duty climbs to the command over the ramp time, longer
 * than the ~0.1 s a motor takes to spin up, so the back-EMF keeps pace.
 *
 * The rules, per output:
 * - A stop or a decrease is immediate: the ramp may slow a start, never a
 *   stop, and a lower ceiling bites at once.
 * - A rise, from rest or while running, is limited to (ceiling - deadband)
 *   per ramp time, so a full command is reached from rest after the ramp
 *   time. A small correction on a running motor is a small step and barely
 *   waits.
 * - Driving the other way than the motor last turned waits until it has
 *   coasted for the ramp time since it was last driven: reversing a spinning
 *   motor is the largest current of all, its back-EMF adding to the supply.
 *   A restart the same way as it was turning starts at once.
 *
 * Duties are signed fractions of the PWM period, after the deadband and
 * ceiling mapping and after the motor map's sign: what goes on the pins.
 * Times are microseconds of a monotonic clock (esp_timer_get_time()).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float ramp_s;       /* rest to a full command; <= 0 switches the ramp off */
    float deadband;     /* duty fraction a start begins at, 0..1              */
    float ceiling;      /* duty fraction of a full command, above deadband    */
} motor_ramp_cfg_t;

typedef struct {
    float   target;     /* signed duty asked for                              */
    float   applied;    /* signed duty on the pins now                        */
    bool    driving;    /* false: coasting                                    */
    int8_t  dir;        /* +1 or -1, the way it was last driven; 0: never     */
    int64_t stopped_us; /* when it last stopped being driven                  */
    int64_t last_us;    /* the previous update                                */
} motor_ramp_t;

/* At rest, never driven. */
void motor_ramp_init(motor_ramp_t *r, int64_t now_us);

/* A new target; returns the duty for the pins now. A non-finite target is a
   stop. */
float motor_ramp_set(motor_ramp_t *r, float target, int64_t now_us, const motor_ramp_cfg_t *c);

/* Moves the applied duty toward the target for the time since the last
   update; returns the duty for the pins now. */
float motor_ramp_step(motor_ramp_t *r, int64_t now_us, const motor_ramp_cfg_t *c);

/* Something wrote the pins another way (coast, brake, the failsafe): at rest
   now, target 0, and a reversal still waits for the coast. */
void motor_ramp_stop(motor_ramp_t *r, int64_t now_us);

/* True while a later step would change the pins: climbing toward the target,
   or coasting before a reversal. */
bool motor_ramp_busy(const motor_ramp_t *r);
