/* motor.h - rotor thrust driver interface: newtons in, PWM duty out.
 *
 * Four fixed, UNIDIRECTIONAL brushed coreless motors, each a low-side
 * SI2300 N-MOSFET gate driven by one LEDC PWM channel (board_config.h).
 * Duty 0..100% maps to thrust 0..motor_max_thrust_n; there is no reverse.
 *
 * Consumes blimp_actuators_t (core/include/blimp/types.h) directly - the
 * mixer's own output type, unmodified - so main.c's control loop is a
 * straight `blimp_step(...); blimp_motor_apply(&s_blimp.params, &out);` with
 * no translation layer in between.
 *
 * The thrust->duty CALIBRATION CURVE (motor.c) is a deliberate, clearly-
 * marked PLACEHOLDER. It must be replaced by a real fit from a thrust-stand
 * sweep before it means anything - flying on it unmeasured risks exceeding
 * Rule 13's 100 gf total-thrust cap or leaving most of the commanded range
 * unreachable.
 */
#ifndef BLIMP_DRIVER_MOTOR_H
#define BLIMP_DRIVER_MOTOR_H

#include "esp_err.h"

#include "blimp/params.h"
#include "blimp/types.h"

/* Configures LEDC timer 0 (board_config.h: 20 kHz, 10-bit) and all four
 * motor channels (FL/FR/RL/RR), each starting at duty 0 - i.e. commanded
 * zero thrust, which is also the safe power-up state. */
esp_err_t blimp_motor_init(void);

/* Writes all four motor channels from one mixer output. p is needed only to
 * read p->motor_max_thrust_n, the per-motor normalisation for the
 * thrust->duty calibration curve - see motor.c. NULL-safe: returns
 * ESP_ERR_INVALID_ARG and writes nothing to hardware if either pointer is
 * NULL (a NULL act must never silently command a stale or undefined duty).
 * Never commands more than 100% duty on any channel. */
esp_err_t blimp_motor_apply(const blimp_params_t *p, const blimp_actuators_t *act);

/* Commands all four channels to 0% duty (zero thrust) immediately. Call from
 * the failsafe path and from blimp_motor_init(). */
esp_err_t blimp_motor_all_stop(void);

/* Directly commands one motor channel's PWM duty, bypassing the mixer and
 * the thrust->duty curve entirely. motor_index: 0=FL, 1=FR, 2=RL, 3=RR.
 * duty_0_to_1 is clamped to [0,1]; NaN is sanitised to 0 (never leaves a
 * channel at an undefined duty). Used by blimp_motor_identify_forever() and
 * available for bench testing. */
esp_err_t blimp_motor_set_duty_raw(int motor_index, float duty_0_to_1);

/* MOTOR IDENTIFICATION MODE - bring-up only, never for flight.
 *
 * Loops forever. Logs a loud warning that propellers must be removed or the
 * vehicle physically restrained, then for each motor in turn (FL, FR, RL,
 * RR) logs which GPIO is about to move, runs that motor alone at 20% duty
 * for 1500 ms, then holds it off for 1000 ms before moving to the next -
 * so a person watching the vehicle can write down which physical motor
 * corresponds to which board_config.h position. Never returns. Gated behind
 * BLIMP_BOOT_MOTOR_ID_MODE in board_config.h; app_main() calls this right
 * after blimp_motor_init() and before creating any flight task when that
 * flag is set. */
void blimp_motor_identify_forever(void);

#endif /* BLIMP_DRIVER_MOTOR_H */
