# Task: add a fixed-rate control loop to the existing blimp firmware (empty stages first)

## Where
- This repo, project `firmware/projects/i2c_tools`, branch `s3-wireless-link`. Read `CLAUDE.md`
  first and build, flash and monitor exactly as it describes for this machine.
- Persistent config goes in `sdkconfig.defaults` / `sdkconfig.defaults.esp32s3`, never in `sdkconfig`
  (gitignored).
- Do not touch `firmware/projects/optflow_cam`. In `software/` only add the decoder script and the
  new commands in `blimp_link.py`.

## Context
- Vehicle: an autonomous blimp (helium envelope + gondola) for IAR-LARC 2026. Rule 1: no manual or
  remote piloting once a run has started. Rule 13: total thrust of all motors <= 100 gf. It is trimmed
  a few gf heavy, so motors off = slow sink to the floor; that is the failsafe.
- Board: Waveshare ESP32-S3-Zero (ESP32-S3FH4R2: 4 MB flash, 2 MB quad PSRAM; PSRAM is NOT enabled in
  the current sdkconfig). Console over USB-Serial-JTAG; opening the USB port resets the chip.
- Existing link: Wi-Fi AP + UDP (`net_wifi.c`, `net_link.c`, formats in `link_protocol.h`): heartbeat,
  MOTOR_SET (signed per-motor commands, FL RL FR RR), MOTOR_STOP, TELEMETRY at 20 Hz, SENSORS; link
  timeout `CONFIG_NET_LINK_TIMEOUT_MS` = 500 ms. Ground tools: `software/blimp_link.py` (stdlib only)
  and `software/ground_station/`.
- The current firmware works: every sensor reads from its console command and from `sensor_stream.c`
  (a 20 Hz task with a pause/resume lease the console commands take), and `cmd_motor.c` drives the
  motors forward and backward. Do NOT break any of that.
- Sensors on one I2C bus (SDA GPIO4, SCL GPIO5), all through the `i2c_master` driver:
  MPU-6050 (accel + gyro), a magnetometer breakout (HMC5883L or QMC5883L; the driver detects it),
  BME280 (P, T, H; `cmd_baro.c` also handles a BMP180), VL53L1X through ST's ULD in
  `components/vl53l1x_uld`.
- Cameras: NOT on this branch. The ESP32-CAM link is still a plan (`docs/inter-board-link-plan.md`) and
  the detectors live on the `target-cam-*` branches. In this task the camera input is a stub: define
  `cam_result_t` (per camera: frame sequence number, timestamp, detected flag, centroid x/y, area) fed
  by nothing yet. Planned rate: 12.5 fps (80 ms) per camera, two cameras.
- Battery voltage: a divider into GPIO1 (ADC1). There is no ADC code in the firmware yet; adding it is
  part of this task.
- Actuators: four reversible brushed motors on two DRV8833s, driver `cmd_motor.c`:
  `motor_id_t` FL, RL, FR, RR (that order in `motors_set_all()`), signed throttle -1..+1, 0 coasts,
  break-away deadband already applied inside `motor_set()` (`CONFIG_MOTOR_DEADBAND_PERMILLE`), slow
  decay by default, and a motor failsafe (`CONFIG_MOTOR_FAILSAFE_MS`, fed by `motors_failsafe_feed()`).
  Use these four names everywhere; never M1..M4.
  - Heave pair RL, RR: props horizontal, on the rear bar at +/-5.5 cm. Sum = heave. Their difference
    is a static roll trim at most: a 5.5 cm lever arm cannot fight the pendulum stability of the
    gondola, so roll is not a controlled axis.
  - Surge pair FL, FR: props edge-on, on the same bar at +/-20 cm. Sum = surge, difference = yaw.
  - Thrust measured so far: 14.7-16.8 gf per motor on a lab supply, reverse about 74 % of forward;
    expect less through the DRV8833 from a sagging 1S cell. Four motors stay under Rule 13, but the
    cap is enforced anyway so that a motor change cannot break it silently.
- Pins: `Kconfig.projbuild` and the existing code are the source of truth.

## Relation to the flight controller in `core/`
The flight controller already exists in this repo as portable C in `core/` (brought over from the
DroneUN repo): `blimp_step()` = estimate -> guide -> control -> mix, with a Rule 13 test and a MuJoCo
loop that runs the same compiled code. Its previous RTOS shell, written for the ESP-FLY board, is kept
for reference at `firmware/projects/espfly/main/main.c` and is not built. This task builds the
equivalent shell for the S3-Zero, so the stage boundaries and types must match core's, and later
`blimp_step()` can replace the middle stages with one call instead of being re-implemented:
- Frame: body x forward, y left, z up, yaw positive counter-clockwise seen from above. SI units
  (m, m/s^2, rad/s, N, N.m, s) after the estimate stage; raw counts before it.
- `sensors_raw_t` mirrors `blimp_imu_sample_t` (per-field valid/fresh flags); `ctrl_out_t` mirrors
  `blimp_wrench_t` (fx, fy = 0, fz, tz in N and N.m); `motor_cmd_t` mirrors `blimp_actuators_t`
  (thrust per motor in N plus clamp flags).
Read `core/include/blimp/types.h`, `core/include/blimp/blimp.h` and the ESP-FLY shell before
proposing the structs. Do not edit `core/` in this task. (Core's mixer is still unidirectional and
its motor thrust is a placeholder; neither is this task's problem.)

## Goal
Restructure the firmware into a fixed-rate pipeline:
read sensors -> estimate -> control -> mixer -> saturate -> PWM -> log.
Every stage is a separate function with typed input/output structs. In this task the estimate and
control stages are pass-through/empty. Later work only fills them in, never restructures, so the
struct layouts must be right the first time (hence the section above).

## Requirements
1. Timing
   - One control task pinned to core 1 (Wi-Fi, lwIP and the main task run on core 0), priority above
     `net_link` and `sensor_stream`, period 20 ms (50 Hz).
   - This build has `CONFIG_FREERTOS_HZ=100` (10 ms tick). Either raise it to 1000 in
     `sdkconfig.defaults` or drive the task from a periodic `esp_timer` that notifies it (the ESP-FLY
     shell does the latter and explains why). Say which you chose and why.
   - Measure period and execution time every tick with `esp_timer_get_time()`: min/max/mean and an
     overrun counter. A low-priority task prints them once per second; the loop itself never calls
     printf/ESP_LOG, never allocates, never waits beyond one bounded I2C transaction.
   - "No blocking" means: every I2C transaction has a timeout of a few ms; a failed read sets
     fresh = false and keeps the last value; no retries, no `vTaskDelay`, no polling loops inside
     the tick. Audit every existing `*_read()` and the ULD calls for waits (the barometer driver
     waits for conversions with `vTaskDelay`; the ULD's `VL53L1_WaitMs` rounds up to whole ticks) and
     split or replace what blocks: barometer in continuous (normal) mode and read the latest
     registers; ToF polled with CheckForDataReady -> GetDistance -> ClearInterrupt.
2. Multi-rate sensor schedule (base tick = 20 ms)
   - The control task becomes the ONLY I2C user. Retire `sensor_stream.c`'s task; the SENSORS packet
     and the console sensor commands read a copy of the latest `sensors_raw_t` (1-deep queue or a
     copy under a short critical section), they do not touch the bus. A console command that needs the
     bus (`i2cconfig`, calibration) must stop the control task explicitly, and that state must show
     in telemetry.
   - IMU + magnetometer: every tick. Configure the MPU-6050 DLPF below 25 Hz (Nyquist at 50 Hz);
     `imu_attach()` sets only the full-scale ranges today, so add the filter setting. Motor vibration
     otherwise aliases into the loop.
   - BME280 and VL53L1X: every 2 ticks. The BME280 delivers P, T and H in one 8-byte burst, so read
     all three together; there is no separate humidity schedule. ToF: set the timing budget and
     inter-measurement period to the 40 ms cadence, check data-ready, reuse the last value otherwise.
   - Cameras: a `cam_result_t` mailbox per camera (`xQueueOverwrite` / `xQueuePeek`, or a seqlock) that
     a future UART task fills. The loop checks it every tick and sets fresh when the sequence number
     changed. Do not copy on a 4-tick cadence: it only adds up to 60 ms of latency.
   - Battery: every tick, ADC oneshot with a running average; raw millivolts in the log.
   - Every measurement carries a fresh flag and the timestamp of its last update.
3. Data structs (suggested names)
   - `sensors_raw_t`, `state_est_t`, `setpoint_t`, `ctrl_out_t`, `motor_cmd_t`, `cam_result_t`,
     `log_record_t`, and one `params_t` holding every tunable with its default in one file (no
     file-scope magic numbers).
   - `ctrl_out_t` holds the body wrench: fx (surge, N), fz (heave, N), tz (yaw moment, N.m). Roll is a
     static trim parameter, not a wrench component.
4. Stages
   - `estimate()`: raw counts -> SI units, including the mounting rotation of each breakout so the
     output is already in the body frame above. No filtering yet.
   - `control()`: a mode variable MANUAL / OPEN_LOOP / CLOSED_LOOP / KILLED.
     - MANUAL passes `link_motor_set_t.cmd[4]` (already per motor, FL RL FR RR) straight to
       `motor_cmd_t`, bypassing the mixer, so MANUAL behaves exactly as today.
     - OPEN_LOOP and CLOSED_LOOP are stubs that output a zero wrench (motors off), not MANUAL: an
       autonomous mode must never depend on the link.
   - `mixer()`: wrench -> per-motor thrust in N:
     FL = fx/2 - tz/(2 l_surge), FR = fx/2 + tz/(2 l_surge), RL = fz/2 + roll_trim, RR = fz/2 - roll_trim,
     with `l_surge` = 0.20 m a parameter. Expected sign: right motor harder -> nose swings left -> +tz.
     Verify every sign on the bench and document the result in the code.
   - `saturate()`: thrust -> signed throttle through a thrust map (placeholder: linear with
     `motor_max_thrust_n` forward and 0.74 x that in reverse, marked TODO-IDENTIFY until a duty sweep
     through the DRV8833 replaces it), clamp to [-1, 1], then enforce the total-thrust cap
     sum|T_i| <= `max_total_thrust_n` (100 gf, Rule 13) by scaling, and set clamp flags. The deadband
     is already handled inside `motor_set()`; do not add a second one.
   - `pwm_out()`: `motors_set_all()` + `motors_failsafe_feed()`, nothing else.
5. Failsafes (all latched until an explicit re-arm; all visible in telemetry and in the log)
   - Link watchdog: reuse the existing link timeout and motor failsafe rather than adding a third
     timer. In MANUAL, link loss -> motors to zero. In OPEN_LOOP/CLOSED_LOOP the run is autonomous by
     rule, so link loss must NOT stop the motors; only kill and the battery cutoff do.
   - Kill: `LINK_PKT_MOTOR_STOP` already exists; make it latch mode = KILLED, and add a console
     equivalent.
   - Battery cutoff: configurable threshold with hysteresis and a hold time (below threshold for
     2 s continuously), tuned with the motors loaded, because sag under load false-triggers a bare
     threshold. Latched, so it cannot chatter when the voltage recovers after the motors stop.
6. Logging
   - One packed `log_record_t` per tick: t_us, raw IMU/mag/baro/ToF as int16/int32 counts, the camera
     stubs, fresh flags, mode, failsafe state, the setpoint (the 4 link commands or the wrench), the
     4 throttles as int16, battery mV, exec time us. Target <= 64 bytes. File header: magic, format
     version, record size, firmware version string.
   - Ring buffer in PSRAM: enable `CONFIG_SPIRAM` (quad mode, 2 MB on this module) in
     `sdkconfig.defaults.esp32s3`, allocate once at init with `heap_caps_malloc(MALLOC_CAP_SPIRAM)`,
     never in the loop. Budget: 64 B x 50 Hz = 3.2 kB/s, about 10 minutes in 2 MB. Report the free
     PSRAM with Wi-Fi up before sizing the buffer.
   - Commands start / stop / dump as new link packet types plus console commands. Dump over the
     existing UDP link with a chunk index and a re-request from the ground tool: UDP loses packets,
     and USB is not an option because opening the port resets the chip and the buffer lives in RAM.
     Until the dump works, the ground station's own SENSORS log is the fallback.
   - `software/`: a decoder from dump to CSV (stdlib only, like `blimp_link.py`) and a JSON metadata
     template per test (date, firmware version, battery voltage, trim, notes).

## Working method (mandatory)
- First read and summarise the existing firmware: tasks and their cores/priorities, drivers and
  which calls block, the link protocol, pins. Propose a plan. Do not edit anything until I approve it.
- Work in small increments: each increment is one buildable, flashable change set. After each one,
  stop, tell me how to verify it on hardware, and wait for my result. I do the physical steps.
- For every design decision, explain the reasoning briefly. This is an academic project:
  understanding matters as much as the result.
- Suggested increments:
  1. Struct definitions (checked against `core/`), task skeleton with timing statistics and empty
     stages. Nothing drives the motors yet.
  2. Sensor reads moved into the multi-rate schedule; `sensor_stream.c` retired; ADC added.
  3. Mixer, saturation, PWM, modes, failsafes.
  4. Logging, dump, Python decoder.

## Acceptance test
- A 60 s log with the blimp at rest shows a period of 20 ms +/- 1 ms, measured from the log timestamps
  and matching the printed statistics, zero overruns, and a reported execution time.
- In MANUAL mode the motors respond to commands exactly as in the current firmware; the TELEMETRY and
  SENSORS packets are still live; every console sensor command still works, or its retirement is
  written down.
- The decoder reproduces the record count and timestamps of a dump.
