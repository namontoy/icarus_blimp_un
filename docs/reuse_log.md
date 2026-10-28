# Reuse log: what the blimp firmware takes from esp-drone

**Date started:** 2026-10-27
**Task:** prompt 2, kept locally in `docs/prompts/` (gitignored, not in the repo); the rules it
sets are the ones applied below
**Target:** `firmware/projects/i2c_tools/` (the S3-Zero control loop, `docs/control-loop-plan.md`)

## Source

| | |
|---|---|
| Repository | https://github.com/espressif/esp-drone (Espressif's port of Bitcraze's crazyflie-firmware) |
| Commit read | `9d5a4f85199ea056f40472b2248e10dcb6988b5c` (`master`, "Merge pull request #137 ... idf-6.1-p4-s31-refactor", 2026-10-26) |
| Local copy | `external/esp-drone/`, shallow clone, read-only, gitignored (`.gitignore`: `external/esp-drone/`) |
| Licence | GPL-3.0 (`external/esp-drone/LICENSE`; every file under `components/core/crazyflie/` carries the Bitcraze GPL-3.0 header) |

## Licence rule applied

Code copied verbatim would carry GPL-3.0 into this firmware and keep its original header
(`docs/03-software-architecture.md` section 2, `docs/04-risk-and-troubleshooting.md` risk 18).
A pattern re-expressed in our own code does not. Every item below says which it is. As of this
log, **nothing is copied verbatim**: no file in `firmware/projects/i2c_tools/` carries a
Bitcraze or GPL header.

## Items

| # | Pattern | Origin in esp-drone (`components/core/crazyflie/`) | Quadrotor assumption it carried | What we do | Status |
|---|---|---|---|---|---|
| 1 | Fixed-rate loop with rate dividers for slower sensors | `modules/src/stabilizer.c` (`stabilizerTask`: `vTaskDelayUntil` at `RATE_MAIN_LOOP`, a `tick` counter passed to every stage); `RATE_DO_EXECUTE(RATE_HZ, TICK)` in `modules/interface/stabilizer_types.h` | 1 kHz main loop paced by the IMU's data-ready interrupt (`sensorsWaitDataReady()`); the dividers (500/250/100 Hz) exist to run attitude, position and logging below the IMU rate | Re-expressed: `ctl_task.c` keeps a `tick` counter and runs the loop at 50 Hz from an `esp_timer`, not from the IMU, because the MPU-6050 is polled over a shared bus; the dividers become "every 2 ticks" tests for the barometer and the ToF | **re-expressed**, increment 1 (counter) and 2 (dividers) |
| 2 | PID: P on the error, optional second-order low-pass on D, symmetric integral clamp (`iLimit`, 0 = off), output clamp (`outputLimit`, 0 = off) | `modules/src/pid.c` (`pidUpdate`), `modules/interface/pid.h` | Differentiates the error (`error - prevError`), so a setpoint step kicks the output; the gains in `pid.h` are attitude-rate gains for a 30 g quadrotor | Dropped: the loop's control stage is a placeholder until it calls `blimp_step()`, whose PID is `core/`'s `blimp_pid_t` (`core/src/pid.c`: conditional integration, D supplied externally or filtered, no derivative kick on the first sample). Prompt 2 rule 1 drops this item in that case, and rule 6 forbids an unused second PID | **dropped** (decision recorded, no code) |
| 3 | Commander: setpoint sources with a priority, a standing setpoint that expires (`COMMANDER_WDT_TIMEOUT_STABILIZE`, then `..._SHUTDOWN`), stored in one-deep queues (`xQueueOverwrite`/`xQueuePeek`) | `modules/src/commander.c` | The expiry first levels the quadrotor (roll = pitch = 0, keep thrust), then cuts thrust: a vehicle that falls when thrust stops needs a two-stage timeout | Semantics only: a standing MANUAL command expires straight to zero (the blimp sinks gently when motors stop; there is no "level first" stage), the kill latch outranks every source, and the two existing timers (`CONFIG_NET_LINK_TIMEOUT_MS`, `CONFIG_MOTOR_FAILSAFE_MS`) provide the expiry; the one-deep mailbox idea is used for the link command and the camera results | **re-expressed**, increment 3 |
| 4 | Log and param registries: variables registered by name, readable and tunable at runtime, served as a table of contents over CRTP | `modules/src/param.c` (`PARAM_GROUP_START`/`PARAM_ADD` linker-section macros, `_param_start`/`_param_end`, TOC channel with CRC), `modules/src/log.c` (log blocks created by the client, each with a period and a timer) | None in the mechanism itself; the machinery (linker sections, TOC paging, CRTP ports and channels) belongs to the Crazyflie client protocol | Re-expressed as one flat table `{name, type, pointer, min, max}` in `ctl_params.c` and two packet pairs on our link: `PARAM_GET`/`PARAM_SET` with a `PARAM_VALUE` reply, and `LOG_BLOCK_REQ` (a list of table indices and a period) with `LOG_BLOCK_DATA`; no macros, no TOC, no CRTP | **re-expressed**, increment 5 |

## Not imported, and why

| What | Origin | Reason |
|---|---|---|
| Roll/pitch rate to attitude cascade | `modules/src/controller_pid.c`, `attitude_pid_controller.c` | the blimp is passively stable in roll and pitch; only heave, yaw and surge are actuated |
| Quadrotor mixer | `modules/src/power_distribution_stock.c` (`thrust +/- roll +/- pitch +/- yaw` per motor) | our four motors are two orthogonal pairs (surge/yaw, heave/roll trim); the allocation is `core/`'s mixer |
| Kalman process models with thrust along body z | `modules/src/kalman_core.c`, `estimator_kalman.c` | buoyancy and added mass dominate; thrust is not along one body axis |
| Position and flow-deck controller | `modules/src/position_controller_pid.c`, flow deck code | guidance is `core/`'s bearing-and-range state machine |
| Free-fall and tumble detection, arming and self-test, `motorsConv16ToBits` | `modules/src/sitaw.c`, `motors.c` | a blimp neither free-falls nor tumbles; the DRV8833 driver has its own duty mapping (`cmd_motor.c`) |
| Any sensor or bus driver | `hal/src/sensors_mpu6050_hm5883L_ms5611.c`, `i2cdev` | esp-drone targets ESP-IDF v4.4/v5.0 and the legacy I2C driver; ours is `i2c_master` on v5.5.5 |
| 500 Hz-1 kHz loop rates | `stabilizer_types.h` (`RATE_MAIN_LOOP`) | the loop runs at 50 Hz with a 20 ms I2C budget |
| CRTP over UDP / ESP-NOW | `components/core/crazyflie/hal/src/wifi_esp32.c` and friends | our link is `link_protocol.h` over the S3's own access point |
| Log blocks streamed over CRTP | `modules/src/log.c`, `modules/interface/log.h` (variables registered by name, grouped into blocks the ground station subscribes to at a rate) | the capture built on 2026-10-27 (`capture.c`, `LOG_CMD`/`LOG_INFO`/`LOG_CHUNK`) is a different device: the whole record of every tick kept in PSRAM and fetched afterwards, written from this project's needs without reading esp-drone's logging; the registry idea itself stays item 4, for increment 5 |

## Gains, cut-offs and noise constants

None are copied from esp-drone (prompt 2 rule 3). Where a tunable exists in `core/src/params.c`
it is the starting point and says so; everything else starts at zero or at a conservative
placeholder marked `TODO-IDENTIFY`.

## Traceability comments

Every module that re-expresses a pattern starts with a comment naming the esp-drone file, the
commit above, the quadrotor assumption it contained and how it was removed. Current list:

| File | Pattern |
|---|---|
| `firmware/projects/i2c_tools/main/ctl_task.c` | item 1 (fixed-rate loop, rate dividers) |
