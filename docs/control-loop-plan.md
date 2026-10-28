# Control loop plan: a fixed-rate pipeline for the S3-Zero firmware

**Date:** 2026-10-27
**Branch:** `s3-wireless-link`
**Task prompts:** two, kept locally in `docs/prompts/` (gitignored since 2026-10-27, not in the
repo): prompt 1, the loop, and prompt 2, what to take from esp-drone. What they bind this work
to is restated here: the increments and their bench checks (sections 4 and 6), the `core/` and
`sdkconfig` rules, and the reuse rules in `docs/reuse_log.md`.
**Status:** plan, with increment 1 built and not yet run on hardware. Each increment stops
for a bench check; the checks are in section 6. "Measured" and "reasoned" are kept apart.
**Revisions, 2026-10-27, after review:** D1 keeps the 100 Hz tick; D3 the sensor commands
never stop the loop; D5 the stop latches in every mode and the GUI arms (option A); D7 the
72-byte record (option B). GitHub's PyQt6 ground station was merged (`50a2aec`) and the plan
re-read against it: the `CONTROL` packet moves to increment 2, the ground-station work joins
increment 3, and a camera relay packet is reserved in D9.
**Revisions, 2026-10-27, later the same day:** the capture, which is increment 4's buffer and
dump, was built ahead of increments 2 and 3 as its own module (`capture.c`, the `capture`
command, `LOG_CMD`/`LOG_INFO`/`LOG_CHUNK`, `blimp_link.py capture`, `blimp_log.py`, the ground
station's Bench page), fed by the log stage from an interim peek of the sensor stream. D7 is
amended (the barometer fields are compensated fixed point, the buffer is a linear capture), D8
now describes the protocol as built, and the bench check of increment 1 gained a capture step.

## 1. What the firmware does today (read 2026-10-27)

### 1.1 Tasks

| Task | Core | Priority | Period | Owner | Role |
|---|---|---|---|---|---|
| `main` (app_main, then the USB REPL) | 0 | 1 | - | ESP-IDF | USB console; `esp_console_run()` has one static line buffer |
| `net_console` | any | 2 | - | `net_console.c` | the same console over UDP 7201 |
| `status_led` | any | 2 | 20 ms | `led_blink.c` | WS2812 on GPIO21 through RMT |
| `sensors` | any | 3 | 50 ms (`CONFIG_SENSOR_STREAM_HZ` 20) | `sensor_stream.c` | samples IMU, mag and ToF every cycle, the barometer every 250 ms; the pause/resume lease |
| `net_link` | any | 4 | 10 ms recv poll, telemetry every 50 ms | `net_link.c` | UDP 7200 packets, link timeout, TELEMETRY and SENSORS out |
| `tcpip_thread` (lwIP) | any | 18 | - | ESP-IDF | |
| `esp_timer` | 0 | 22 | - | ESP-IDF | runs the motor failsafe tick (50 ms, task dispatch) |
| `wifi` | 0 | 23 | - | ESP-IDF | |

`CONFIG_FREERTOS_HZ=100` (10 ms tick), `-Og` with assertions, 160 MHz, task watchdog on both
idle tasks (5 s), no PSRAM. Everything but Wi-Fi, esp_timer and `main` floats between cores.

### 1.2 Drivers, and which calls block

| Driver | Per read on the bus | Waits inside the read | Waits at attach | Timeout per transaction |
|---|---|---|---|---|
| `imu_read()` (MPU-6050) | one 14-byte burst | none | `vTaskDelay(50)` after wake | 50 ms |
| `mag_read()` (HMC/QMC5883L) | status byte, then 6 bytes | `vTaskDelay(10)` loop up to `timeout_ms`; `timeout_ms = 0` checks once | 2 ticks after reset (QMC), 100 ms (HMC) | 50 ms |
| `baro_read()` (BME280/BMP280/BMP180) | ctrl write, then an 8-byte burst | **forced conversion: `delay_at_least_ms(conv)` plus a `vTaskDelay(1)` status loop, 10-30 ms blocked** | reset + NVM copy wait | 50 ms |
| `tof_read()` (VL53L1X, ST ULD) | `CheckForDataReady` (2 reads), `GetResult` (17 bytes), signal and ambient (2 words), `ClearInterrupt` | `vTaskDelay(10)` loop up to `timeout_ms`; 0 checks once | `SensorInit` polls boot with `VL53L1_WaitMs(1)` (spins below one tick, else `vTaskDelay(ticks+1)`) | **100 ms** (`PORT_TIMEOUT_MS`) |
| `motors_set_all()` | - | a mutex and eight `ledc_set_duty` | - | - |

So the only reads that block by design are the barometer's forced conversion and any
`timeout_ms > 0` wait in `mag_read()`/`tof_read()`; the stream already calls both with
`timeout_ms = 0`. The ULD's `VL53L1_WaitMs` is reached only through attach-time calls
(`SensorInit`, `StopRanging`), never through the data-ready, result and clear sequence.

### 1.3 Link protocol (`link_protocol.h`, mirrored by `software/blimp_link.py`)

Header `magic 0xB1A7, version 1, type, seq, reserved` (8 B), payload, CRC-32 (zlib) over the
packet with the CRC zeroed. Host to blimp: `HEARTBEAT 0x01`, `MOTOR_SET 0x02` (`f32 cmd[4]`
FL RL FR RR), `MOTOR_STOP 0x03` (`u8 brake`). Blimp to host: `TELEMETRY 0x81` (52 B: motor
readback, failsafe, link flags, counters, heap) and `SENSORS 0x82` (76 B, SI units, valid bits).
`net_link.c` applies `MOTOR_SET` straight to `motors_set_all()`, coasts the motors itself on
link loss (500 ms without any packet) when they were last commanded over the air, and sends the
stream's latest sample once per telemetry tick. The ground station (`software/ground_station/`)
and `mock_blimp.py` speak exactly these five packets; the ground station has been the PyQt6
window since 2026-10-27 (`blimp_bridge.py`, its side of the link, did not change).

### 1.4 Pins

| Function | GPIO | Source of truth |
|---|---|---|
| I2C SDA / SCL | 4 / 5 | `Kconfig.projbuild`, `app_main` |
| motor FL IN1/IN2 | 13 / 12 | `Kconfig.projbuild`, "Motor driver" menu |
| motor RL IN1/IN2 | 11 / 10 | |
| motor FR IN1/IN2 | 9 / 8 | |
| motor RR IN1/IN2 | 7 / 14 | |
| nSLEEP | strapped high (`-1`) | |
| status LED (WS2812, RMT) | 21 | |
| battery divider | 1 (ADC1_CH0) | `docs/s3-wroom-cam-wiring-plan.md`: 100 k + 100 k, 100 nF to GND; **no firmware yet** |
| console | native USB (opening the port resets the chip) | `sdkconfig.defaults.esp32s3` |

### 1.5 What `core/` expects of the shell

`blimp_step()` (`core/include/blimp/blimp.h`) takes `blimp_input_t { blimp_imu_sample_t imu;
blimp_target_obs_t target; contact_detected; arm_request; t_us }` and writes a
`blimp_actuators_t`. `blimp_imu_sample_t` is SI in the body frame (x forward, y left, z up)
with one `*_valid` flag per sensor; `range_alt` is already tilt-corrected; `baro_alt` is
absolute-ish metres; `mag` is "normalised or uT, direction only". `blimp_wrench_t` is `fx, fy,
fz, tz`; `blimp_actuators_t` is `thrust_fl, thrust_fr, thrust_rl, thrust_rr` in N plus three
clamp flags. The ESP-FLY shell (`firmware/projects/espfly/main/main.c`) drives its loop from
a periodic `esp_timer` and a semaphore, keeps `blimp_t` owned by one task, and hands data
between tasks only by copy under a mutex or through a one-deep queue.

## 2. Design decisions

Each decision names its reason. The alternatives were weighed, not skipped.

**D1. Clock: a periodic `esp_timer` dispatched from the timer interrupt notifies the task;
the FreeRTOS tick stays at its 100 Hz default.** The esp_timer is the period source because
its resolution does not depend on the tick, and dispatched from the interrupt
(`CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD=y`) it does not queue behind the esp_timer
task, which Wi-Fi (priority 23 on core 0) can hold off for milliseconds. The ESP-FLY shell
used task dispatch; this is the same pattern with less jitter, and the callback is one
`vTaskNotifyGiveFromISR()`. A 1 kHz tick was proposed for everything that still calls
`vTaskDelay()` (the attach sequences, the ULD's `WaitMs`, the barometer's conversion waits,
the console commands all round up to whole ticks) and declined on 2026-10-27: the loop does
not depend on the tick, those waits sit outside it or leave it in increment 2, and every
tick-count assumption in the existing code would have had to be re-checked.

**D2. One control task on core 1, priority 20, 20 ms period.** Core 0 keeps Wi-Fi, esp_timer,
the USB console and `main`; lwIP floats but at priority 18 cannot preempt the loop. Priority
20 is above every task of ours (link 4, stream 3, consoles 2) and below the system tasks,
which are pinned to core 0 anyway. The loop is the highest-priority task on core 1, so once
notified it runs at once.

**D3. The control task becomes the only I2C user, and the sensor commands never stop it.**
`sensor_stream.c` is retired in increment 2. `imu`, `mag`, `baro`, `tof` and `sensors` print
the loop's latest copy of `sensors_raw_t` (one copy under a short critical section) and never
touch the bus, so a glance at the numbers interrupts neither the loop nor the stream to the
ground station (decided 2026-10-27). Their chip-specific options (address, ranges,
oversampling, timing budget, distance mode) stop being per-command settings: the ones that
matter in flight become fields of `params_t`, applied by the loop at its next re-attach, and
the rest are refused with a message while the loop runs. The only things that still take the
bus are `i2cconfig` (it recreates the bus) and a future calibration command, through an
explicit `ctl_suspend()` (motors to zero, every chip released, the state visible in the
status packet and the log) and `ctl_resume()`.

**D4. Structs mirror `core/`.** `sensors_raw_t` carries counts plus a fresh flag and a
timestamp per sensor; `estimate()` turns it into exactly the fields of `blimp_imu_sample_t`
(SI, body frame) inside `state_est_t`, so the later call to `blimp_step()` is a copy, not a
conversion. `ctrl_out_t` is `fx, fy, fz, tz` like `blimp_wrench_t` (fy kept at 0 for layout
parity); `motor_cmd_t` is four thrusts in N in `motor_id_t` order (FL, RL, FR, RR, the
driver's and the wire's order) plus clamp flags and the throttles that `saturate()` derives.
`core/` is not edited, and its headers are not included yet: that happens when `blimp_step()`
is wired in, together with the `core` component.

**D5. Modes and latches.** `MANUAL` passes the latest link `cmd[4]` to the motors, bypassing
the mixer; `OPEN_LOOP` and `CLOSED_LOOP` output a zero wrench until filled in; `KILLED`
outputs zero and stays there until an explicit re-arm. `MOTOR_STOP` becomes the kill (it
latches `KILLED`; its `brake` byte is honoured while killed); a new `MODE_SET` packet and the
`ctl` console command are the re-arm. Consequence for the tools: `blimp_link.py motor` arms
MANUAL first and ends a run with zero commands rather than a stop, so a bench run does not
leave the blimp killed; Ctrl+C and `blimp_link.py stop` do kill. Checked against the PyQt6
ground station (merged 2026-10-27, `software/ground_station/app.py`, `panels.py`): it sends
`MOTOR_STOP` from the STOP button, from the deadman when its setpoint refresh stops, when
"Motors enabled" is switched off and on window close, and enabling the motors again only
restarts the `MOTOR_SET` stream; nothing arms. **Decided 2026-10-27: the latch applies in
every mode** (one packet, one meaning, every stop from the ground sticks until a human
re-arms), and the ground station changes with it, in increment 3: it sends `MODE_SET`
(mode `MANUAL`, arm) when "Motors enabled" goes on, so toggling that checkbox is the re-arm
gesture; it decodes the `CONTROL` packet and shows the mode and the latches in its header
pills; its deadman keeps sending `MOTOR_STOP`, so a frozen GUI kills and needs a deliberate
re-enable; and `mock_blimp.py` accepts `MODE_SET` and models the latch, so the demo mode
behaves like the firmware. The alternative kept on record, a latch only in the autonomous
modes, was rejected once the GUI was free to change.

**D6. Failsafes reuse the two existing timers.** In `MANUAL` the loop zeroes the motors when
no valid packet arrived within `CONFIG_NET_LINK_TIMEOUT_MS` (link loss) or when the last
`MOTOR_SET` is older than `CONFIG_MOTOR_FAILSAFE_MS` (a host that only heartbeats). Both are
computed from timestamps `net_link.c` publishes; `net_link.c` itself stops touching the motor
driver. In the autonomous modes link loss changes nothing, by Rule 1. The motor driver's own
failsafe keeps running underneath and becomes the backstop for a control task that stops
calling `pwm_out()`: the loop feeds it every tick, so it trips only if the loop dies, which is
the failure it was written for. The battery cutoff is a third latch with hysteresis and a
2 s hold, tuned later with the motors loaded.

**D7. Logging.** One packed 72-byte `log_record_t` per tick (layout in `ctl_types.h`, size
asserted at compile time) into a ring buffer allocated once in PSRAM with
`heap_caps_malloc(MALLOC_CAP_SPIRAM)`. PSRAM is enabled from increment 1 so the free size with
Wi-Fi up is measured before the buffer is sized. The prompt's 64-byte target was weighed
against a 72-byte layout on a byte map (https://claude.ai/artifact/M6mTm6JoWDDLVuhY1ahTzo)
and the 72-byte one was chosen on 2026-10-27: the eight extra bytes keep the camera centroid
and area at pixel resolution, which a range-from-apparent-size calibration needs from flight
logs (the 64-byte view rounded the area to steps of about 300 px at QVGA), and keep the
humidity count. The cost is about one minute of a ten-minute buffer (3.6 kB/s against
3.2 kB/s). The barometer's pressure and temperature take three bytes each.

**D7, amended 2026-10-27 when the capture was built.** Two departures from the text above.
The barometer fields are not counts: a BMx280 count means nothing without the chip's trimming
values, and the compensation is Bosch's fixed integer algorithm, not ours, so the record keeps
the compensated values as fixed point, pressure in 0.1 Pa (24-bit unsigned, to 1677 hPa),
temperature in 0.01 degC (24-bit signed) and humidity in 0.01 %RH, a resolution below the
sensor's noise (0.1 Pa is about 8 mm of altitude). The IMU and the magnetometer stay counts,
and their scales travel in the capture header. And the buffer is a linear capture of a
requested length, not a ring: a bench session is a minute long, the question it answers is
"what happened during this test", and a linear buffer lets the link read records while the
loop keeps writing (records below the count never change). `capture_init()` allocates
`CONFIG_CAPTURE_MAX_SECONDS` of records (120 s, 432 kB) once at boot in PSRAM, with a 10 s
fallback in internal RAM when there is none. A ring with a pre-trigger can be added behind the
same API if a flight log asks for one.

**D8. Dump over UDP with re-request (built 2026-10-27).** One host-to-blimp packet,
`LOG_CMD` (20 B): `INFO`; `START` with the length in seconds (0 for the firmware's 60 s
default, clamped to the buffer); `STOP`; `CLEAR`; and `GET` with a first record index and a
count of chunks, up to 8. The blimp answers the first four with `LOG_INFO` (64 B: state,
result, record size and format version, capture id, period, count, capacity, the length
asked for, the first record's clock, the IMU ranges, the magnetometer's LSB per gauss, the
chip ids, the firmware version string) and `GET` with that many `LOG_CHUNK`s (1172 B: capture
id, first index, count, 16 records) back to back, or a `LOG_INFO` with a range error at the
end of the data. The host asks for windows of chunks and asks again for the ones that did not
arrive; records below the count never change while the capture id holds, so the capture need
not be stopped, and a chunk with another id means a restart and the download aborts. 3000
records are 188 chunks. The host writes a `.bin` (a 4-byte magic, the 52-byte `LOG_INFO`
payload, the records) and a `.json` beside it with the header, the operator's notes and the
summary; `software/blimp_log.py` decodes the `.bin` to units, a summary or a CSV. UDP loses
packets and opening USB resets the chip, so this is the only path to a buffer that lives in
RAM.

**D9. New packet types; the existing ones are untouched.** `TELEMETRY` and `SENSORS` keep
their layouts so the ground station and `mock_blimp.py` keep working. The loop's state travels
in a new `CONTROL` packet (mode, latches, bus owner, timing statistics, battery). Host to
blimp: `MODE_SET`, `LOG_CMD`, `PARAM_GET`, `PARAM_SET`, `LOG_BLOCK_REQ`; blimp to host:
`CONTROL`, `LOG_INFO`, `LOG_CHUNK`, `PARAM_VALUE`, `LOG_BLOCK_DATA` (the three `LOG_*` types were built with the capture on 2026-10-27).
`LINK_PROTO_VERSION` stays 1: an old receiver rejects an unknown type by type, and nothing it already knows moved.
`CONTROL` arrives in increment 2, with its mode and latch fields at zero until increment 3,
because the suspended-bus state of D3 has to be visible in telemetry from the moment the loop
owns the bus. One more type is reserved and not built here: the ground station's plan
(`docs/ground-station-plan.md`, row 4 of its status table, merged 2026-10-27) expects the S3
to relay the forward camera's detections and the downward camera's flow in a link packet. Its
source will be the `cam_result_t` mailbox; the packet belongs to the camera-to-S3 link work
(`docs/inter-board-link-plan.md`), not to this task.

**D10. Every tunable in one `params_t`, in one file, with its default.** Introduced in
increment 2 with the first tunables (mounting rotations, DLPF setting, ToF timing, battery
divider and averaging); increment 3 adds the mixer geometry, thrust map, caps and cutoff.
Prompt 2's flat table `{name, type, pointer, min, max}` indexes the same struct.

**D11. The period is a compile-time constant, not a parameter.** The rate dividers, the DLPF
choice and the log cadence all assume 20 ms, so a runtime change would silently break them.
`CTL_PERIOD_US` lives in `ctl_task.h` with that reason.

## 3. Multi-rate schedule (base tick 20 ms)

| Tick | IMU + mag | Barometer | ToF | Cameras | Battery |
|---|---|---|---|---|---|
| even | read | 8-byte burst (normal mode: the latest conversion, no wait) | - | peek both mailboxes | ADC |
| odd | read | - | data-ready, then result and clear | peek both mailboxes | ADC |

The barometer and the ToF alternate rather than share a tick, so the longest tick is IMU +
mag + ToF: about 7 ms of bus time at 100 kHz, reasoned from byte counts and measured in
increment 2. Every measurement carries a fresh flag and the time of its last update; a failed
transaction keeps the last value and clears the flag. Cameras are peeked every tick and set
fresh when the frame sequence number moved.

## 4. Increments

1. **Skeleton** (built 2026-10-27): `ctl_types.h` (every struct, checked against `core/`),
   `ctl_task.c` (timer, task on core 1, timing statistics, empty stages, the statistics
   printer, the `ctl` command), `sdkconfig.defaults` (ISR dispatch; the tick stays at 100 Hz),
   `sdkconfig.defaults.esp32s3` (quad PSRAM). Nothing touches the bus or the motors. esp-drone
   cloned read-only and `docs/reuse_log.md` started.
2. **Sensors in the loop**: the schedule above, DLPF, barometer in normal mode, ToF polled,
   battery ADC, camera mailboxes; `sensor_stream.c` retired, and with it the interim
   `sensor_stream_latest_raw()` peek that `read_sensors()` does today; SENSORS packet and the console
   sensor commands from the loop's copy; `ctl_suspend()`/`ctl_resume()`; `params_t`; the
   `CONTROL` packet (timing statistics, battery, bus state; mode and latches zero until
   increment 3) and `blimp_link.py status` showing it.
3. **Control path**: `estimate()` (counts to SI with mounting rotations), modes, mixer,
   `saturate()` with the Rule 13 cap, `pwm_out()`, link and battery latches, kill, the
   `MODE_SET` packet and the mode and latch fields of `CONTROL`, `blimp_link.py arm`/`mode`
   and the changed `motor`/`stop`, the `motor` console command routed through the loop, and
   the ground-station side of D5 (arm on enable, mode and latch pills, the "sensor stream
   paused" notice reworded for D3, `mock_blimp.py` modelling the latch).
4. **Logging** (built 2026-10-27, ahead of 2 and 3): `capture.c` (the buffer in PSRAM, the
   `capture` command), `LOG_CMD`/`LOG_INFO`/`LOG_CHUNK` in `net_link.c`, the chunked download
   with re-request in `blimp_link.py capture`, `software/blimp_log.py` (decode, summary, CSV),
   the `.json` beside every capture, and the ground station's Bench page (`bench.py`: capture
   control and a trace viewer). The log stage already writes every record; what the records
   carry grows with the stages: the loop's own throttles with `pwm_out()` (today the motor
   driver's readback), the battery with the ADC, the cameras with their mailboxes.
5. **Reuse items from esp-drone**: the parameter table with `PARAM_GET`/`PARAM_SET`, the log
   block request (`LOG_BLOCK_REQ`/`LOG_BLOCK_DATA`), `reuse_log.md` completed.

## 5. What the esp-drone prompt adds, and what it does not

Read at commit `9d5a4f85199ea056f40472b2248e10dcb6988b5c` (`master`, 2026-10-26), GPL-3.0,
cloned read-only into `external/esp-drone/` (gitignored).

| Item | Where in esp-drone (`components/core/crazyflie/`) | Decision |
|---|---|---|
| Fixed-rate loop with rate dividers | `modules/src/stabilizer.c`; `RATE_DO_EXECUTE` in `modules/interface/stabilizer_types.h` | re-expressed in `ctl_task.c` (increment 1 the tick counter, increment 2 the dividers) |
| PID with filtered D, symmetric integral and output clamps | `modules/src/pid.c` | **dropped**: the loop will call `blimp_step()`, whose PID is `core/`'s `blimp_pid_t` (prompt 2 rule 1 says so for this case); no increment needs a second PID, and an unused one is dead code (rule 6) |
| Commander: setpoint priority and timeout | `modules/src/commander.c` | semantics only (a source priority kill > console > link, a standing setpoint that expires); no code (increment 3) |
| Param and log registries served over the link | `modules/src/param.c`, `modules/src/log.c` | re-expressed as one flat table and two packet pairs (increment 5); no linker sections, no TOC |
| Quad mixer, attitude cascade, Kalman models, `sitaw.c`, arming and self-test, 8-bit PWM, sensor and bus drivers, CRTP, 500 Hz-1 kHz rates | `power_distribution_stock.c`, `controller_pid.c`, `kalman_core.c`, `sitaw.c`, `hal/src/sensors_*.c`, ... | not imported, for the reasons in the prompt |

Nothing is copied verbatim, so no file in this firmware carries the GPL-3.0 header.
`docs/reuse_log.md` keeps the same table with the status per increment and the quadrotor
assumption each pattern carried.

## 6. Bench checks per increment

Props off for every step. USB console open in a second window.

**Increment 1.** Flash. The boot log shows the control task starting on core 1 and the PSRAM
size and free figure. Once per second a `ctl:` line prints the period (min/max/mean), the
execution time and the counters; the period should read 20.00 ms with min and max within
0.1 ms of it, zero missed ticks, zero overruns. `ctl` prints the same on demand; `ctl --stats
off` silences the line. Then the regression: `sensors`, `imu -n 3`, `motor -m FL -t 0.15`
and `python software/blimp_link.py status` behave exactly as before, and the `ctl:` line keeps
reading 20.00 ms while `blimp_link.py status` runs (Wi-Fi traffic on core 0 must not move it).
Record the free PSRAM with the access point up and a station associated; that number sizes
the log buffer in increment 4.

**Capture (built 2026-10-27, checked together with increment 1).** `capture --start -t 10` on
the console, wait, `capture` shows `DONE 500 of 500`; `capture --head 3` prints records whose
IMU counts move between samples and whose `fresh` byte changes; `python
software/blimp_link.py capture download` saves `captures/capture-<date>.bin` and prints a
summary with 500 records, a step of 20.00 ms with a standard deviation under 0.1 ms, IMU and
magnetometer fresh at the stream's 20 Hz, barometer at 4 Hz, ToF at 10 Hz, and an execution
time of a few hundred microseconds; the Bench page's Download produces the same file and its
traces show the accelerometer at rest near (0, 0, 1) g. Against `mock_blimp.py` all of this
passes (2026-10-27); on hardware it has not run.

**Increment 2.** `ctl` shows every sensor attached and fresh; `sensors` matches `imu -n 3`,
and neither interrupts the stream; the ground station panels fill (PyQt6 installed per its
README); the printed execution time stays well under 20 ms with all four chips on the bus;
`baro -o 3` (a chip option) is refused with a message that names the parameter;
`i2cconfig --sda 4 --scl 5` suspends the loop, which `ctl` and the `CONTROL` packet show
(`blimp_link.py status`), and the loop resumes with the same numbers afterwards.

**Increment 3.** `blimp_link.py motor 0.15 0 0 0`: FL turns, telemetry readback as before;
Wi-Fi off during a run: motors zero within 500 ms and `CONTROL` shows `link_lost`;
`--rate 0.5`: motors zero 1 s after each packet. `blimp_link.py stop` latches `KILLED`;
`arm` clears it. Sign tests: nose up gives a positive pitch rate, left side up a positive
roll rate, turning the gondola counter-clockwise a positive yaw rate, lifting it an increasing
altitude; each mixer sign is checked by commanding one wrench component at a time and
watching which motor leads.

**Increment 4.** What remains of it is checked when the stages that feed the record exist: a
capture taken during `blimp_link.py motor 0.15 0 0 0` shows FL's throttle at 0.15 beside its
setpoint (increment 3), and the battery column follows a lab supply (increment 2).

## 7. Open questions for the bench

- Battery divider: the wiring plan says 100 k + 100 k on GPIO1 (ratio 2.0, so a 4.2 V cell
  reads 2.1 V, inside the 12 dB attenuation range). Is it wired on the current bench?
- Mounting rotation of each breakout: the defaults are identity (chip axes = body axes) until
  the sign tests of increment 3 say otherwise.
- The console `motor --hold`: under the loop the standing command is kept alive by the loop
  itself, so it holds until `motor -s` or a kill; today the 1 s failsafe ends it. Acceptable?
- ToF timing at the 40 ms cadence: the ULD accepts budgets of 15, 20, 33, 50 and 100 ms, and
  budgets below 100 ms were measured about 5 % short on the bench. Budget and period are
  parameters; the first runs decide between 33 ms (a frame every other slot) and 20 ms.
