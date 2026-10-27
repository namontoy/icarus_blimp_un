/* board_config.h - THE SINGLE FILE SOMEONE EDITS WHEN THE PCB REVISION CHANGES.
 *
 * ================================================================
 *  HARDWARE CONFIRMED (2026-09-21): Seeed XIAO ESP32S3 module stacked on the
 *  Seeed x Max Imagination "ESP-FLY" flight-controller PCB. Bare MPU-6050 on
 *  I2C (no magnetometer, no barometer, no ToF). Four SI2300 N-channel MOSFETs
 *  doing LOW-SIDE drive of four brushed coreless motors, one GPIO per gate,
 *  10k gate pull-down + 1N4148 flyback each. NO ESCs, NO servos. Motors are
 *  UNIDIRECTIONAL: duty 0..100% = thrust 0..max, never reverse.
 *
 *  PIN SOURCE: Seeed-Projects/Co-Create_ESP-FLY, Firmware/esp-drone/sdkconfig
 *  (the ESP-FLY reference firmware's own board config). Every GPIO number
 *  below that is annotated "ESP-FLY sdkconfig" was read directly from that
 *  file, not measured or guessed. Everything annotated PLACEHOLDER or
 *  PROBABLE was not in that file and still needs bench verification.
 * ================================================================
 */
#ifndef BLIMP_BOARD_CONFIG_H
#define BLIMP_BOARD_CONFIG_H

/* -------------------------------------------------------------------------
 * Shared sensor I2C bus - IMU only on this PCB
 * -------------------------------------------------------------------------
 * imu.c assumes this bus has ALREADY been installed (i2c_driver_install) by
 * main.c's board_i2c_init() before blimp_imu_init() runs.
 *
 * SDA=GPIO5 (XIAO pin D4), SCL=GPIO6 (XIAO pin D5), per ESP-FLY sdkconfig
 * (CONFIG_I2C0_PIN_SDA=5, CONFIG_I2C0_PIN_SCL=6). The ESP-FLY PCB carries its
 * own 10k pull-ups on both lines.
 */
#define BLIMP_BOARD_I2C_PORT           0        /* -> I2C_NUM_0 */
#define BLIMP_BOARD_I2C_SDA_GPIO       5        /* ESP-FLY sdkconfig: CONFIG_I2C0_PIN_SDA (XIAO D4) */
#define BLIMP_BOARD_I2C_SCL_GPIO       6        /* ESP-FLY sdkconfig: CONFIG_I2C0_PIN_SCL (XIAO D5) */
#define BLIMP_BOARD_I2C_CLK_HZ         400000   /* 400 kHz fast mode; drop to
                                                  * 100 kHz if the bus proves
                                                  * long/noisy near the motor
                                                  * wiring */

/* MPU-6050. AD0 tied low on this PCB -> address 0x68 (0x69 if ever strapped
 * high). imu.c is DONE and already assumes 0x68 - confirmed, not a guess. */
#define BLIMP_BOARD_IMU_I2C_ADDR       0x68

/* NO MAGNETOMETER on this part or this PCB. Yaw is unbounded gyro
 * integration - the estimator and guidance already handle that
 * (blimp_params_t.use_mag = false). */

/* IMU output rate. Matches main.c's BLIMP_CONTROL_PERIOD_US (200 Hz / 5000 us)
 * so the control loop gets exactly one fresh sample per tick. */
#define BLIMP_BOARD_IMU_SAMPLE_HZ      200

/* CHIP-FRAME -> BODY-FRAME REMAP.
 *
 * Core's body frame is x-FORWARD, y-LEFT, z-UP (blimp/types.h). The MPU-6050
 * die has its own axes, and which chip axis lands on which vehicle axis
 * depends entirely on how the PCB is physically mounted in the gondola -
 * which is not yet known.
 *
 * AXIS_<b> = index of the CHIP axis that feeds body axis <b> (0=cx,1=cy,2=cz)
 * SIGN_<b> = +1.0f or -1.0f applied after selection.
 *
 * The identity mapping below assumes the chip's +X points forward, +Y left
 * and +Z up. THAT IS A GUESS.
 *
 * VERIFY BY HAND, bring-up step 3 (docs/07 section 7), before closing any
 * loop: hold the assembled gondola level and confirm accel reads ~(0,0,+9.81);
 * then rotate nose-up, nose-left and roll-right in turn and confirm each gyro
 * channel moves on the expected axis with the expected sign. A sign error
 * here does not look like a sign error in flight - it looks like a
 * confidently diverging controller. */
#define BLIMP_BOARD_IMU_AXIS_X         0
#define BLIMP_BOARD_IMU_AXIS_Y         1
#define BLIMP_BOARD_IMU_AXIS_Z         2
#define BLIMP_BOARD_IMU_SIGN_X         (+1.0f)
#define BLIMP_BOARD_IMU_SIGN_Y         (+1.0f)
#define BLIMP_BOARD_IMU_SIGN_Z         (+1.0f)

/* MPU-6050's own INT pin. ESP-FLY sdkconfig: CONFIG_MPU_PIN_INT=8 (XIAO D9).
 * NOT USED by this firmware (imu.c polls over I2C on the fixed control tick
 * instead of using data-ready interrupts) - defined here only so the GPIO is
 * documented as spoken-for and nothing else claims it by accident. */
#define BLIMP_BOARD_MPU_INT_GPIO       8

/* -------------------------------------------------------------------------
 * Sensors NOT populated on this PCB
 * -------------------------------------------------------------------------
 * No magnetometer, no barometer, no ToF range finder. The HAS_* flags below
 * gate every init()/read() call for these in main.c so an unpopulated sensor
 * is never even probed. The *_I2C_ADDR defines are kept (unused while their
 * HAS_* flag is 0) purely as a reference for the day a breakout board is
 * added on the same I2C header - flip the flag to 1 and wire up the driver's
 * register map at that point. */
#define BLIMP_BOARD_HAS_MAG            0
#define BLIMP_BOARD_HAS_BARO           0
#define BLIMP_BOARD_HAS_TOF            0

#define BLIMP_BOARD_MAG_I2C_ADDR       0x1E     /* PLACEHOLDER: LIS3MDL default
                                                  * (SDO/SA1 high); 0x1C if low.
                                                  * Not populated today. */
#define BLIMP_BOARD_BARO_I2C_ADDR      0x77     /* PLACEHOLDER: BMP390 default
                                                  * (SDO high); 0x76 if low.
                                                  * Not populated today. */
#define BLIMP_BOARD_TOF_I2C_ADDR       0x29     /* PLACEHOLDER: VL53L1X default.
                                                  * Not populated today. */

/* -------------------------------------------------------------------------
 * Battery voltage monitor
 * -------------------------------------------------------------------------
 * The ESP-FLY schematic shows a 10k/10k divider (R8/R9) feeding an ADC pin,
 * but the ESP-FLY reference sdkconfig does not name which GPIO - it is not
 * one of the digital pins that sdkconfig assigns to anything else. GPIO2
 * (XIAO pin D1, ADC1_CH1) is the only analog-capable pin on the XIAO left
 * unclaimed by I2C/motors/LEDs/MPU-INT above, so it is the PROBABLE battery
 * sense pin. VERIFY WITH A MULTIMETER ON D1 before trusting a logged voltage.
 */
#define BLIMP_BOARD_BATT_ADC_CHANNEL   1        /* -> ADC_CHANNEL_1 on ADC_UNIT_1,
                                                  * i.e. GPIO2 on ESP32-S3 */
#define BLIMP_BOARD_BATT_ADC_GPIO      2        /* PROBABLE - verify with a
                                                  * multimeter on D1 */
#define BLIMP_BOARD_BATT_DIVIDER_RATIO 2.0f      /* PROBABLE: matches the
                                                  * schematic's 10k/10k
                                                  * (equal-value) R8/R9
                                                  * divider - Vbat_true =
                                                  * Vadc_measured * 2.0. Not
                                                  * yet bench-confirmed
                                                  * against the real board. */
#define BLIMP_BOARD_BATT_LOW_V         3.3f      /* 1S LiPo low-voltage
                                                  * failsafe threshold,
                                                  * ENGINEERING GUESS pending
                                                  * the actual cell chosen */

/* -------------------------------------------------------------------------
 * Status LEDs
 * -------------------------------------------------------------------------
 * ESP-FLY sdkconfig: CONFIG_LED_PIN_RED=43 (XIAO D6), CONFIG_LED_PIN_GREEN=9
 * (XIAO D10). GPIO43/44 are also ESP32-S3's default UART0 TX/RX pins - this
 * firmware does not use UART0 for the console (see sdkconfig's
 * CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG - the XIAO's USB-C goes to the native
 * USB Serial/JTAG peripheral instead, which has no pin conflict with these
 * two), so GPIO43 is free for the red LED. No driver file exists yet for
 * these; drive them with plain gpio_set_level() wherever a status indication
 * is needed. */
#define BLIMP_BOARD_LED_RED_GPIO       43       /* ESP-FLY sdkconfig: CONFIG_LED_PIN_RED */
#define BLIMP_BOARD_LED_GREEN_GPIO     9        /* ESP-FLY sdkconfig: CONFIG_LED_PIN_GREEN */

/* -------------------------------------------------------------------------
 * Actuation: four fixed brushed motors, low-side MOSFET + PWM (docs/02
 * section 6 D.10-11 is now answered: MOSFET+PWM, matching the ESP-FLY
 * reference exactly - no ESC, no servo, no lateral thruster on this
 * airframe. See blimp/types.h's frame note: front pair pushes +x (surge/yaw),
 * rear pair pushes +z (heave, up only).
 *
 * GPIOs are the ESP-FLY sdkconfig's own four motor pins (CONFIG_MOTOR01_PIN..
 * CONFIG_MOTOR04_PIN). Position assignment (which physical corner is
 * "MOTOR01" etc.) is THIS PROJECT'S OWN CHOICE, not something the ESP-FLY
 * firmware defines - verify it for real with blimp_motor_identify_forever()
 * (BLIMP_BOOT_MOTOR_ID_MODE below) before trusting yaw direction in flight.
 *
 * GPIO3 (RL) is an ESP32-S3 strapping pin (JTAG voltage/boot selection). The
 * board's 10k gate pull-down holds the MOSFET off during reset/boot, so this
 * is safe, but it is called out here because a future revision moving RL
 * elsewhere would remove one small footgun. GPIO1 (RR) is ADC1_CH0 if ADC
 * were ever needed on it - it is not, here.
 */
#define BLIMP_MOTOR_FL_GPIO            7        /* ESP-FLY sdkconfig: CONFIG_MOTOR01_PIN (XIAO D8) */
#define BLIMP_MOTOR_FR_GPIO            4        /* ESP-FLY sdkconfig: CONFIG_MOTOR02_PIN (XIAO D3) */
#define BLIMP_MOTOR_RL_GPIO            3        /* ESP-FLY sdkconfig: CONFIG_MOTOR03_PIN (XIAO D2) -
                                                  * strapping pin (JTAG select); safe behind the
                                                  * board's 10k gate pull-down, see comment above */
#define BLIMP_MOTOR_RR_GPIO            1        /* ESP-FLY sdkconfig: CONFIG_MOTOR04_PIN (XIAO D0) */

/* LEDC PWM carrier for all four motor gates. 20 kHz is comfortably above
 * audible range (unlike a hobby-servo-style 50 Hz) and well within what a
 * SI2300-class MOSFET switches cleanly with a 10k gate pull-down; 10-bit
 * resolution (1024 steps) at 20 kHz is achievable from LEDC_AUTO_CLK's
 * available source clocks on ESP32-S3. */
#define BLIMP_LEDC_MOTOR_TIMER         0        /* -> LEDC_TIMER_0, shared by all 4 channels */
#define BLIMP_LEDC_MOTOR_FREQ_HZ       20000
#define BLIMP_LEDC_MOTOR_RES_BITS      10       /* -> LEDC_TIMER_10_BIT */
#define BLIMP_LEDC_MOTOR_CH_FL         0        /* -> LEDC_CHANNEL_0 */
#define BLIMP_LEDC_MOTOR_CH_FR         1        /* -> LEDC_CHANNEL_1 */
#define BLIMP_LEDC_MOTOR_CH_RL         2        /* -> LEDC_CHANNEL_2 */
#define BLIMP_LEDC_MOTOR_CH_RR         3        /* -> LEDC_CHANNEL_3 */

/* A coreless brushed motor typically does not turn at all below some small
 * duty fraction - the driving voltage is too low to overcome static
 * friction/cogging. PLACEHOLDER - measure the real stall/start duty on a
 * bench (spin-up sweep with a tachometer or by ear) and replace this number;
 * see motor.c's thrust->duty curve for how it is used. */
#define BLIMP_MOTOR_DUTY_DEADBAND      0.08f

/* When 1, app_main() calls blimp_motor_identify_forever() right after
 * blimp_motor_init() and never creates the flight tasks - a bring-up-only
 * mode for working out which physical motor is wired to which GPIO/position.
 * MUST be 0 for any normal flight build. */
#define BLIMP_BOOT_MOTOR_ID_MODE       0

/* -------------------------------------------------------------------------
 * ESP-NOW ground link
 * -------------------------------------------------------------------------
 */
#define BLIMP_ESPNOW_WIFI_CHANNEL       1        /* PLACEHOLDER. Only matters
                                                  * while running ESP-NOW
                                                  * standalone (no STA
                                                  * association) - this
                                                  * skeleton's assumption.
                                                  * Survey the venue's 2.4 GHz
                                                  * congestion before fixing a
                                                  * final channel. */

/* Broadcast address: every peer accepts this. Fine for bring-up (no pairing
 * step needed) but means the vehicle will act on ANY nearby sender using the
 * same protocol - replace with the actual ground-station MAC address before
 * flying anywhere other than an isolated bench, and consider enabling
 * esp_now_peer_info_t.encrypt with a shared LMK once a specific peer is
 * pinned (not implemented in this skeleton - PLACEHOLDER, unencrypted). */
#define BLIMP_ESPNOW_PEER_MAC          { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }

#endif /* BLIMP_BOARD_CONFIG_H */
