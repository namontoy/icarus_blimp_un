/* blimp/estimator.h - attitude, heading and altitude estimation.
 *
 * Design notes (docs/03 section 4, docs/07 section 5):
 *
 * - Roll/pitch are EASY on a blimp. The vehicle is near-neutrally buoyant and
 *   weakly actuated, so specific force is small and the accelerometer is a
 *   clean gravity reference most of the time - the opposite of a quadcopter.
 *
 * - Yaw is THE problem. Gyro integration drifts without bound, and unlike a
 *   quadcopter the vehicle cannot fight through an accumulated heading error
 *   with brute authority: it has a handful of gf of differential thrust. Every
 *   reference design in the corpus is MPU6050-based with no magnetometer,
 *   which is tolerable over a 5-minute quad flight and is not tolerable here.
 *   Adding a magnetometer is the single highest-leverage deviation from the
 *   reference designs.
 *
 * - Altitude fuses barometer (absolute, noisy, slow) with a downward ToF
 *   range (centimetre-accurate, range-limited) and vertical acceleration.
 *
 * No position estimate is produced. Horizontal guidance runs on bearing and
 * range from perception directly - see guidance.h. That is both what the
 * published indoor-blimp work does and the only thing compatible with Rule 15
 * without building real onboard SLAM.
 */
#ifndef BLIMP_ESTIMATOR_H
#define BLIMP_ESTIMATOR_H

#include "blimp/types.h"
#include "blimp/params.h"

typedef struct {
    float gyro_bias[3];      /* rad/s, slowly estimated while stationary   */
    float q[4];              /* attitude quaternion, w x y z               */
    float alt;               /* m, fused                                   */
    float vz;                /* m/s                                        */
    float baro_ref;          /* m, zeroed at arm time                      */
    uint32_t last_us;
    bool  initialised;
    bool  mag_healthy;       /* cleared on implausible magnitude/jumps     */
    float stationary_s;      /* seconds the vehicle has looked motionless  */
    uint32_t last_abs_us;    /* timestamp of the last absolute altitude fix */
    bool  abs_ever;          /* any absolute altitude fix since init        */
} blimp_estimator_t;

void blimp_estimator_init(blimp_estimator_t *e);

/* Zeroes the barometric reference and latches the current heading as the yaw
 * origin. Call during the untimed PREPARE phase, never mid-run. */
void blimp_estimator_zero(blimp_estimator_t *e, const blimp_imu_sample_t *s);

/* Consumes one sensor sample and updates out. Tolerates missing fields via
 * the sample's *_valid flags. dt is derived from sample timestamps; a
 * backwards or absurd dt is rejected and the tick skipped. */
void blimp_estimator_update(blimp_estimator_t *e, const blimp_params_t *p,
                            const blimp_imu_sample_t *s, blimp_state_t *out);

/* ALTITUDE VALIDITY IS A WINDOW, NOT A PER-TICK FLAG. state.altitude_valid
 * stays true for BLIMP_EST_ALT_STALE_S after the last absolute fix (ToF,
 * baro, or the vision-derived fix blimp.c injects as range_alt when a
 * balloon is in view), while altitude/vz dead-reckon on the accelerometer.
 * On the confirmed PCB (no altitude sensor) the balloon drops out of the
 * camera's field of view for seconds at a time; without this window the
 * altitude loop would lose its reference every time that happened. A fix
 * arriving after a gap longer than BLIMP_EST_ALT_REACQ_S snaps the altitude
 * instead of differencing it into an absurd vertical rate. */

/* True while gyro bias is still converging - block arming on this. */
bool blimp_estimator_converging(const blimp_estimator_t *e);

#endif /* BLIMP_ESTIMATOR_H */
