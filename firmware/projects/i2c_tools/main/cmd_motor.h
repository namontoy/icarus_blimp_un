/*
 * 'motor' console command and thrust API: two DRV8833 dual H-bridges.
 */
#pragma once

#include <stdbool.h>
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
 * Positive drives IN1 of that pair high, which is a direction in the wiring,
 * not a direction on the airframe - whether it pushes the nose forward
 * depends on how the motor leads and the prop are fitted. Establish the sign
 * per thruster on the bench, and correct it by swapping that motor's two
 * GPIOs in Kconfig rather than by negating commands in the controller.
 *
 * The magnitude is mapped through the break-away deadband (see
 * motors_set_deadband), so 0.01 produces the smallest duty that actually
 * turns the prop rather than a duty that only heats the winding.
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
 * This is a per-motor physical property and the default is only a starting
 * guess - measure it with 'motor --sweep' and set it from the result.
 */
esp_err_t motors_set_deadband(float deadband);
float motors_get_deadband(void);

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

/** "FL", "RL", "FR", "RR" for logs and console output. */
const char *motor_name(motor_id_t motor);

/** Which driver and input pair this motor sits on, e.g. "#1 IN1/IN2". */
const char *motor_where(motor_id_t motor);

/** Register the 'motor' command with the console. */
void register_motor(void);

#ifdef __cplusplus
}
#endif
