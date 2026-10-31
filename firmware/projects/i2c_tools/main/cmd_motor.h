/*
 * 'motor' console command and thrust API: two DRV8833 dual H-bridges.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Four thrusters in a quad layout, listed in the order they are wired:
 *
 *     MOTOR_FL   driver #1, IN1/IN2   front left
 *     MOTOR_RL   driver #1, IN3/IN4   rear left
 *     MOTOR_FR   driver #2, IN1/IN2   front right
 *     MOTOR_RR   driver #2, IN3/IN4   rear right
 *
 * Note that the split falls along the airframe: driver #1 carries the whole
 * left side and driver #2 the whole right. That is tidy to wire and it has a
 * consequence worth knowing before it happens - a single driver going into
 * thermal shutdown or fault does not cost one thruster, it costs one side,
 * which is a yaw and roll upset rather than a thrust loss. Anything that
 * eventually monitors nFAULT should treat a fault as "lose the side".
 *
 * This is also the index order of the array taken by motors_set_all().
 */
typedef enum {
    MOTOR_FL = 0,
    MOTOR_RL,
    MOTOR_FR,
    MOTOR_RR,
    MOTOR_COUNT
} motor_id_t;

/*
 * How the H-bridge spends the part of the PWM period when it is not driving.
 *
 * SLOW  - recirculates the winding current through both low-side FETs
 *         (the DRV8833's "brake" state). Current stays continuous, so thrust
 *         tracks duty roughly linearly and the motor is quieter. Default.
 * FAST  - lets the bridge go high-impedance and the current decay to zero
 *         (the "coast" state). The prop freewheels between pulses, which
 *         loses low-end linearity but dissipates less in the driver.
 *
 * On a blimp the low end is where the controller lives, so SLOW is the one
 * that matters. FAST is here to be measured against it, not as a default.
 */
typedef enum {
    MOTOR_DECAY_SLOW = 0,
    MOTOR_DECAY_FAST
} motor_decay_t;

/**
 * Claim the eight GPIOs, start the PWM timer, and leave every motor coasting.
 *
 * Safe to call more than once. Must be called before any motor_* call; the
 * console command calls it for you on first use.
 */
esp_err_t motors_init(void);

/**
 * Set one motor's signed thrust command, -1.0 to +1.0. Exactly 0.0 coasts.
 *
 * Positive drives IN1 of that pair high times the map's sign (below), which
 * is a direction in the wiring, not a direction on the airframe - whether it
 * pushes the nose forward depends on how the motor leads and the prop are
 * fitted. Establish the sign per thruster on the bench and set it in the map
 * ('motormap'), rather than by negating commands in the controller. On
 * 2026-10-30 a positive command pushed forward on FL and FR and up on RL
 * and RR, so every default sign is +1.
 *
 * The magnitude is mapped through the break-away deadband (see
 * motors_set_deadband). With the default deadband of 0 the magnitude is the
 * duty itself; with a measured one, 0.01 produces the smallest duty that
 * actually turns the prop rather than a duty that only heats the winding.
 *
 * Rejects a non-finite throttle with ESP_ERR_INVALID_ARG rather than passing
 * a NaN through to the duty arithmetic - a NaN out of a controller must not
 * be able to latch an H-bridge at an arbitrary duty.
 */
esp_err_t motor_set(motor_id_t motor, float throttle);

/** Set all four at once, in motor_id_t order: FL, RL, FR, RR. */
esp_err_t motors_set_all(const float throttle[MOTOR_COUNT]);

/** Last value accepted by motor_set, before deadband mapping. */
float motor_get(motor_id_t motor);

/** All bridges high-impedance. The props freewheel. */
void motors_coast(void);

/** All bridges shorted low. The props are held against back-EMF. */
void motors_brake(void);

/** Pick the decay mode. Takes effect on the next motor_set. */
esp_err_t motors_set_decay(motor_decay_t mode);

/**
 * Smallest duty, 0.0 to 0.9, that is assumed to break the prop away from
 * rest. Thrust commands are mapped onto [deadband, 1.0].
 *
 * This is a per-motor physical property. The default is 0 (command = duty)
 * until it has been measured - measure it with 'motor --sweep' and set it
 * from the result.
 */
esp_err_t motors_set_deadband(float deadband);
float motors_get_deadband(void);

/**
 * Duty ceiling, 0.1 to 1.0 and above the deadband: commands are mapped onto
 * [deadband, ceiling], so a command of 1.0 gives this duty and nothing gives
 * more (Kconfig MOTOR_MAX_DUTY_PERCENT, 100 by default; the team's limit is
 * 30 %). With a ceiling below 1, core/'s motor_max_thrust_n must be the
 * thrust at the ceiling, not at full duty.
 */
esp_err_t motors_set_max_duty(float max_duty);
float motors_get_max_duty(void);

/**
 * Coast every motor if motors_failsafe_feed() has not been called for
 * timeout_ms. 0 disables it.
 *
 * This exists because the failure that actually hurts is not a bad thrust
 * command, it is a controller that stops sending them while the last one
 * stays latched in the PWM hardware and the airship keeps accelerating.
 */
esp_err_t motors_failsafe_set_timeout_ms(int timeout_ms);
void motors_failsafe_feed(void);

/** True if the failsafe has fired and not yet been cleared by a new command. */
bool motors_failsafe_tripped(void);

/*
 * What the failsafe is doing right now, as one value for status lines and
 * telemetry. OFF: disabled by a 0 timeout. IDLE: nothing driven, nothing to
 * guard. ARMED: a motor is driven and the clock is running. TRIPPED: it
 * fired and no command has come since.
 */
typedef enum {
    MOTOR_FAILSAFE_OFF = 0,
    MOTOR_FAILSAFE_IDLE,
    MOTOR_FAILSAFE_ARMED,
    MOTOR_FAILSAFE_TRIPPED
} motor_failsafe_state_t;

motor_failsafe_state_t motors_failsafe_state(void);

/** "FL", "RL", "FR", "RR" for logs and console output. */
const char *motor_name(motor_id_t motor);

/** Which driver and input pair this motor sits on, e.g. "#1 IN1/IN2". */
const char *motor_where(motor_id_t motor);

/*
 * The motor map (2026-10-30), so that renaming or moving a motor is a console
 * line and not a rebuild. The four driver outputs are the four pin pairs of
 * Kconfig's MOTOR_*_IN*_GPIO, numbered 1-4 in the order FL, RL, FR, RR of
 * those settings. Each output carries three things:
 *
 *   name      FL/RL/FR/RR: what MOTOR_SET, motor_set(), the console and the
 *             ground use. A name is only a label.
 *   position  the same four labels in core/'s frame, seen from behind the
 *             blimp (FL is the surge motor on the blimp's own left): what the
 *             control loop drives, through motor_at_position(). Kept apart
 *             from the name because the loop must push with the motor that
 *             is physically there, whatever the team calls it: a mirrored
 *             name turns the heading loop into positive feedback.
 *   sign      +1 or -1: which command direction pushes the way core/ expects
 *             (forward for the surge pair, up for the heave pair). Applied
 *             to every command, so a flipped propeller or reversed leads is
 *             corrected here instead of by rewiring.
 *
 * Names and positions are each a permutation of the four labels. Defaults
 * come from Kconfig (MOTOR_POSITIONS, MOTOR_SIGNS); motor_map.c keeps a
 * changed map in NVS and the 'motormap' command edits it.
 */
typedef struct {
    uint8_t name[MOTOR_COUNT];      /* per output: a motor_id_t                 */
    uint8_t position[MOTOR_COUNT];  /* per output: a motor_id_t, core/'s frame */
    int8_t  sign[MOTOR_COUNT];      /* per output: +1 or -1                     */
} motor_map_t;

/** The map as Kconfig gives it. */
void motors_default_map(motor_map_t *out);

/** The map in use. */
void motors_get_map(motor_map_t *out);

/**
 * Replace the map. ESP_ERR_INVALID_ARG unless names and positions are
 * permutations and every sign is +1 or -1; ESP_ERR_INVALID_STATE while any
 * motor is driven or the control loop owns the motors.
 */
esp_err_t motors_set_map(const motor_map_t *map);

/** The name of the motor at a position in core/'s frame. */
motor_id_t motor_at_position(motor_id_t position);

/** The output, 0-3, that a name drives. */
int motor_output_of(motor_id_t motor);

/*
 * The control loop's claim on the motors (ctl_task.c, the bench hold). While
 * it holds them, motor_set() and motors_set_all() return
 * ESP_ERR_INVALID_STATE, so neither the link's MOTOR_SET nor the console can
 * fight the loop, and only motors_loop_set_all() writes the bridges.
 * motors_coast() and motors_brake() always work: a stop never waits for an
 * owner. Releasing the claim leaves the motors coasting.
 */
void motors_loop_claim(bool on);
bool motors_loop_owns(void);
esp_err_t motors_loop_set_all(const float throttle[MOTOR_COUNT]);

/*
 * Called by motors_coast() and motors_brake() before they write the pins,
 * whenever the loop holds the motors: a stop from anyone (the link's
 * MOTOR_STOP, 'motor -s') is then a kill the loop latches, not a pause it
 * overrides at its next tick. The loop registers it; called from whichever
 * task stopped the motors, so it must only set a flag.
 */
void motors_set_stop_hook(void (*hook)(void));

/** Register the 'motor' command with the console. */
void register_motor(void);

#ifdef __cplusplus
}
#endif
