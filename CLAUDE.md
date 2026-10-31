@AGENTS.md

# CLAUDE.md

AGENTS.md above is the project rulebook, shared with every coding agent; put project rules there,
not here. This file holds what is specific to working on this repo in Claude Code.

- Skills run as `/core-change`, `/sim-sweep` and `/flash-firmware`; the last runs only when a
  person types it. Their files in `.claude/skills/` are the originals for the Codex copies, so
  after editing one, run `python tools/sync_skills.py`.
- Personal notes go in `CLAUDE.local.md` (gitignored), not in this file.

## What this is

An autonomous blimp for IAR-LARC 2026; the race, the rules and the vehicle are in AGENTS.md. On
2026-10-27 the DroneUN repo (flight controller, simulator, design docs) was merged into this one,
so the repo now has a host side and two chips:

| Path | Chip / host | Role |
|---|---|---|
| `core/` | host and ESP32 | The flight controller as portable C11: `blimp_step()`. Unit-tested on the host, compiled into the simulator, and later into the S3 firmware. Control logic lives only here. |
| `sim/`, `tools/` | host (verified on Windows and Linux) | MuJoCo closed loop around the compiled core, plus the build, verify and sweep scripts. `python tools/setup_env.py` once, then `python tools/verify.py` after any change. |
| `firmware/projects/i2c_tools/` | ESP32-S3 (Waveshare ESP32-S3-Zero) | The flight-controller-to-be: IMU, magnetometer and barometer (stand-alone breakouts since the GY-88 burned; see below), altimeter, four DRV8833 thrusters, and since the `s3-wireless-link` work its own Wi-Fi access point carrying the console and a UDP packet link (`docs/wireless-link.md`). The name comes from the Espressif `i2c_tools` example it grew out of, and the console prompt is still `i2c-tools>`. |
| `firmware/projects/optflow_cam/` | ESP32-CAM (AI-Thinker, original ESP32) | Downward optical flow. It prints CSV over UART to a laptop. |
| `firmware/projects/espfly/` | XIAO ESP32-S3 on the ESP-FLY board | DroneUN's previous flight shell for the previous board: an `esp_timer`-driven control task around `blimp_step()`, an ESP-NOW link and `net/protocol.h`. Not built here; kept as the reference for the S3-Zero control task until that supersedes it (ESP-IDF v5.3, `tools\idf_build.bat`). |
| `firmware/projects/blink/` | — | Stock ESP-IDF example, kept as a toolchain smoke test. |
| `software/` | host | `blimp_link.py` is the ground tool for this repo's firmware (standard library only; `mock_blimp.py` fakes the firmware on localhost, capture included; `blimp_log.py` decodes captures; `thrust_map.py`, `step_test.py` and `step_response.py` are the bench instruments of `docs/pid-tuning-plan.md`, and `hold_test.py` runs its closed-loop heading steps on the firmware's bench hold). The esp-drone bench tools that were here until 2026-10-30 (`drone_gui.py`, `drone_dynamics.py`, `motor_diag.py`, `SETUP.md`) spoke CRTP over UDP to a different quadcopter's firmware, never to this repo's; the ground station replaced them. They are in git history (`git show dab8786:software/drone_gui.py`) and `docs/esp-drone-tools-report.md` describes them. |

The two chips are not linked yet. `docs/inter-board-link-plan.md` (a draft) proposes UART links. `docs/sensor-fusion-plan.md` is a proposal (`blimp_ins`) that is **not implemented**. `hardware/` and `external/` are empty placeholders (`external/` is where a read-only esp-drone clone goes, gitignored); `scripts/` holds `idf.bat`. The top-level `README.md` is the project front page, from DroneUN.

## Building firmware (ESP-IDF v5.5.5)

ESP-IDF was installed with EIM, so the activation script sits at a non-standard path:

```bash
source /home/talos/.espressif/tools/activate_idf_v5.5.5.sh
```

After activation `idf.py` is a shell function, so `timeout`, `xargs` and similar can't exec it. From scripts or tool calls, invoke it directly:

```bash
IDF_PY="/home/talos/.espressif/tools/python/v5.5.5/venv/bin/python /home/talos/.espressif/v5.5.5/esp-idf/tools/idf.py"
$IDF_PY -C firmware/projects/i2c_tools set-target esp32s3   # once per fresh build dir
$IDF_PY -C firmware/projects/i2c_tools build
$IDF_PY -C firmware/projects/i2c_tools -p /dev/ttyACM0 flash
$IDF_PY -C firmware/projects/optflow_cam set-target esp32
$IDF_PY -C firmware/projects/optflow_cam -p /dev/ttyUSB0 flash
```

- `sdkconfig`, `build/`, `managed_components/` and `dependencies.lock` are gitignored. Persistent config changes go in `sdkconfig.defaults` / `sdkconfig.defaults.<target>` or `Kconfig.projbuild`, never in `sdkconfig`.
- `idf.py monitor` fully buffers its output when not on a TTY. To capture logs non-interactively, read the port with pyserial.
- **ESP32-S3-Zero:** it uses native USB with no UART bridge, so `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` is required (it is set in `sdkconfig.defaults.esp32s3`). It flashes without the BOOT button. **Opening `/dev/ttyACM0` resets the chip**, so wait about 3 s and discard the boot log before you send console commands. The `rst:0x15 (USB_UART_CHIP_RESET)` at the top of such a log is that reset, not a crash.
- **ESP32-CAM:** it flashes through an external programmer and has a GPIO0 bootloader trap. Read `docs/esp32-cam-optical-flow-report.md` §7 before flashing it.
- **Windows:** `scripts\idf.bat i2c_tools build` (or any `idf.py` arguments after the project name). It expects ESP-IDF v5.5.5 at `%USERPROFILE%\esp\esp-idf-v5.5.5` and Python 3.11 at its per-user default, and works around `export.bat` bailing out silently under Git Bash's `MSYSTEM` variable; see `docs/wireless-link.md` §10.

## Tests

The controller and the simulator are checked with `python tools/verify.py` (about 20 s: the `core/` unit tests including the Rule 13 thrust sweep, the DLL build, the S3 firmware's `param` table compiled and checked on the host (`tools/fw_host_test.py`), the skill sync, and a simulator smoke test); `--full` adds both courses, and `--gui` the ground station's offscreen tests (`software/ground_station/tests/`, about 1 min; run it after any change to the bridge, the station or the pages that drive motors). Run it after any change under `core/`, `sim/` or `tools/`. It needs clang on `PATH` and the `.venv` from `python tools/setup_env.py`. On Linux, run `setup_env.py` with a Python 3.12 interpreter (the `.venv` inherits its version); the verify and sweep scripts then run with any Python 3 and pick up `.venv` themselves. A Linux run gives different per-seed lap times from a Windows run (AGENTS.md, Commands), so compare sweeps, not single seeds, across the two.

The other automated firmware test is the optical-flow matcher, run on the host with no hardware:

```bash
make -C firmware/projects/optflow_cam/host test    # verification suite (test_flow_match.c)
make -C firmware/projects/optflow_cam/host sweep   # sub-pixel bias sweep (bias_sweep.c)
```

Everything else is verified on the bench through the S3 console (`help` lists commands; each sensor/motor has its own command). The user does the physical steps, so ask for a physical action, then stop and wait for confirmation before you run anything that depends on it.

## Python tools (`software/`)

Any Python 3.9 or newer: everything in `software/` is standard library except the ground station, which needs PyQt6 (`pip install -r software/requirements.txt`; the repo's `.venv` has it). With conda, set `PYTHONNOUSERSITE=1` **before** any `pip install`: user-site packages broke a conda env here once (2026-10-13). Checks that need no hardware:

```bash
python software/blimp_link.py selftest     # the packet codec
python software/thrust_map.py --selftest   # step_response.py has one too; step_test.py a dry run
python tools/verify.py --gui               # the ground station, offscreen
```

`optflow_cam/host/flow_monitor.py` is a separate serial viewer/logger and needs `pyserial`.

`software/ground_station/` is the GUI: one native PyQt6 process (`pip install -r software/ground_station/requirements.txt`) whose `station.py` bridges `blimp_link.py`, this computer's Wi-Fi and a demo mode to the window through Qt signals. It talks only to the S3, over Wi-Fi: the cameras are wired to the S3, and their data has no link packet yet, so the camera panel runs on demo data only; the widgets are custom-painted with QPainter (no pyqtgraph). `python -m software.ground_station --demo` runs it with no hardware; design in `docs/ground-station-plan.md` (signal table in §4) and `docs/ground-station-native-plan.md`. The window has three pages under the header: Flight, Control (`control.py`, 2026-10-28: flying by hand with surge, yaw and heave from the keyboard or a gamepad, mixed on the ground into the four duties as the firmware's MANUAL mode expects; PC and Steam Deck input modes, `--mode`; `gamepad.py` reads XInput on Windows and the kernel's event devices on Linux; on a Steam Deck the station runs from Gaming mode as a non-Steam game through `steamdeck.sh`, because in Desktop mode Steam Input never feeds its virtual pad, checked 2026-10-29; the "Go autonomous" button waits for `MODE_SET`) and Bench (`bench.py`, 2026-10-27: the capture and a trace viewer, written to `captures/`); the console dock serves all three. One source drives the motors at a time: arming the Control page switches the Flight page's "Motors enabled" off and the other way round. The bridge's thread owns the standing motor command (`blimp_bridge.py`, 2026-10-28): the pages send changes, the station's tick pings it every 100 ms, and 3 s without a ping releases the motors and latches until the window STOPs; a long repaint can no longer cut the motors (`docs/motor-setpoint-ownership.md`). Since 2026-10-29 (`docs/link-robustness-review.md`) a change goes out at once and the repeat holds 20 Hz on Windows too, a STOP is never overtaken, a Wi-Fi drop zeroes the motors and resumes them ramped if the link returns within 3 s (a longer drop or a blimp reboot releases and latches), B on the pad latches, `--race` makes the window a monitor whose one motor command is STOP, and the header's Drive pill shows which holds. The station emits every SENSORS packet to the window (`sen`, from the receive callback) and the link state at 10 Hz (`tel`). The window's identity is the university's palette with McLaren-style type (Titillium Web and IBM Plex Mono, bundled in `fonts/`); keep new UI inside `theme.py`'s tokens. Any script that builds the window must end with `app.teardown(station, win)` before the interpreter exits, or PyQt's exit cleanup segfaults (see its docstring).

`software/blimp_link.py` and `software/mock_blimp.py` need only the standard library (Python 3.8+), on any OS. `python software/blimp_link.py selftest` checks the packet codec; `python software/mock_blimp.py` in one terminal and `python software/blimp_link.py --host 127.0.0.1 status` in another exercises the link with no hardware.

## S3 firmware architecture (`i2c_tools`)

- `app_main` (`i2ctools_example_main.c`) creates the one I2C master bus (`tool_bus_handle`, SDA=4, SCL=5) and starts an `esp_console` REPL. Every peripheral is a `cmd_<name>.c` module that exposes `register_<name>()` plus a small C API for other modules to call, e.g. `baro_read_temperature_c()`. To add a module you need to touch four places: `main/CMakeLists.txt` SRCS, the include, the `register_*()` call and the banner line in `app_main`.
- `motors_init()` runs **first**, before the console. The DRV8833 inputs float from reset while VM is already live, so the driver must claim the pins immediately. A motor init failure is logged, not fatal, so the console survives.
- The motor driver uses **all eight LEDC channels**, one per DRV8833 input, which gives glitch-free reversal. Nothing else may use LEDC. The status LED is a WS2812 on GPIO21, driven by RMT through `led_strip`.
- **Motor failsafe** (`CONFIG_MOTOR_FAILSAFE_MS`, default 1000): it is armed only while some motor has a non-zero command. If no thrust command arrives within the window, every motor coasts. Any future controller must keep calling the thrust API (which feeds the failsafe) or the motors will stop.
- **Wireless link** (`net_wifi.c`, `net_link.c`, `net_console.c`, formats in `link_protocol.h`; design and bring-up checklist in `docs/wireless-link.md`). The S3 is a Wi-Fi access point (`ICARUS-<mac>`, WPA2, settings under "Wireless link" in menuconfig). UDP 7200 carries fixed-size packets: `MOTOR_SET` is a standing command only while the host repeats it, and `net_link.c`'s own timeout (500 ms) coasts motors that were commanded over the air, independently of the motor failsafe above. Since 2026-10-29 (`docs/link-robustness-review.md`) that timeout counts motor packets only (a heartbeat or another station's packets no longer keep a command alive), a `MOTOR_SET` not newer than the last from the same sender is dropped (`rx_stale` in `link`), `MOTOR_STOP` is obeyed from anyone, `motors_set_all()` leaves a bridge alone when its command has not changed, and the UDP console refuses `motor --failsafe` (`net_console_is_caller()`). UDP 7201 is the same console as USB, one line per datagram, replies closed by a 0x04 byte; it captures output by rebinding its task's per-task `stdout`, so logs from other tasks still go to USB. Run one console at a time: `esp_console_run()` has a static line buffer. `wifi` and `link` are the status commands.
- **Sensor stream** (`sensor_stream.c`, the `sensors` command): samples the IMU, magnetometer and VL53L1X at 50 Hz (20 until 2026-10-28) and the barometer at 4 Hz through the same attach/read/detach drivers as the console commands (`cmd_imu.h`, `cmd_mag.h`, `cmd_baro.h`, `cmd_tof.h`), and `net_link.c` sends each sample as `LINK_PKT_SENSORS` for the ground station (`software/ground_station/`, whose "Join blimp Wi-Fi" button has `wifi.py` join the access point: `netsh wlan` by profile name on Windows, because Windows 11 withholds scans from consoles without location services; `nmcli` on Linux). The chips are shared by turns: `imu`, `mag`, `baro`, `tof` and `i2cconfig` call `sensor_stream_pause()` (the stream releases every chip) and `sensor_stream_resume()` (it re-attaches and reconfigures). Any new code that touches a sensor must do the same, because every console command resets its chip, the ToF port allows one attach per address, and a bus with devices on it cannot be deleted.
- **Control task** (`ctl_task.c`, `ctl_types.h`, the `ctl` command): one task on core 1, priority 20, woken every 20 ms by an `esp_timer` with task dispatch (the FreeRTOS tick stays at 100 Hz). **Keep `CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD` off on ESP-IDF v5.5.x**: an ISR-dispatched timer stalls Wi-Fi's task-dispatched ones there (esp-idf issue #18808, fixed only on master), and the access point then drops its station every ~14 s (found 2026-10-28; `docs/wifi-drop-report.md`. On Linux, `tools/wifi_soak.py` soaks the link and counts drops: run it with the ESP-IDF Python, which has pyserial, after any change that can touch the radio; a check shorter than 5 min proves nothing, because the first drop came 60-160 s after boot). It runs read sensors, estimate, control, mixer, saturate, pwm_out, log as separate functions over the typed records in `ctl_types.h`, and measures its period and execution time in one-second windows (`ctl` shows them; `ctl --stats on`, or `CONFIG_CTL_STATS_AT_BOOT`, prints them every second; off by default because the line buries the console). The read stage peeks the sensor stream's latest sample (`sensor_stream_latest_raw()`; the stream still owns the chips) and samples the battery divider on GPIO1 itself (`cmd_battery.c`, the `battery` command: one ADC1 oneshot conversion per tick, eFuse-calibrated, ratio and averaging in the "Battery monitor" menu; added 2026-10-29, read 3.77 V on 2026-10-30 but not checked against a meter, and nothing acts on it until the low-battery latch), and the log stage writes one 72-byte record per tick into the capture. **Since 2026-10-30 (branch `control-inc3`, increment 3's bench slice; built and host-tested, not yet run on the board): the bench hold.** `estimate()` converts counts to SI in `core/`'s body frame through the mounting maps `CONFIG_CTL_IMU_AXES` ("-Y +X +Z", measured on the rope rig) and `CONFIG_CTL_MAG_AXES`, keeping the IMU sample's own timestamp (a repeated sample then has dt = 0; marking it invalid would restart the estimator's 60 s stillness count); the barometer is not handed to `core/` (a read glitch, docs/10). `control()` runs `blimp_step()` every tick, armed only in the hold; the `hold` command (`hold heading 30`, `hold off`, `hold kill`; parsed by hand like `param`) sets `core/`'s `blimp_hold_t` setpoints, and starting is refused until the estimator has converged, while a motor turns, or without a ground packet in the last 0.5 s (core/'s link watchdog zeroes the hold after 1 s of silence, so a ground tool must keep the link alive: `software/hold_test.py` does). In the hold the loop claims the motors (`motors_loop_claim()`): `MOTOR_SET` and the console's `motor` are refused (counted in `link`), and any stop (`MOTOR_STOP`, `motor -s`, `hold kill`) latches KILLED through the driver's stop hook until `hold off`. Outside the hold `net_link.c` owns the motors as before. Thrust goes to throttle as T / `motor_max_thrust_n`, then the driver's deadband. **The motor map** (`cmd_motor.h`, `motor_map.c`, the `motormap` command, kept in NVS): each driver output has a name (what the wire and the console use) and a position in `core/`'s frame (what the loop drives) and a sign; on 2026-10-30 the team's names turned out mirrored against `core/`'s frame (the output named FL is on the blimp's right), the team kept them, and `CONFIG_MOTOR_POSITIONS` is "FR RR FL RL". The plan, the decisions (bus ownership moves to this task, `MOTOR_STOP` becomes a latched kill, new packet types) and the bench checks per increment are in `docs/control-loop-plan.md`; what is taken from esp-drone, and under which licence status, is in `docs/reuse_log.md`. PSRAM (`sdkconfig.defaults.esp32s3`) holds the capture buffer (`capture.c`, `CONFIG_CAPTURE_MAX_SECONDS` = 400 s; a longer request is refused, never shortened): `capture --start`, or `LOG_CMD` over the link, records the loop's records; `python software/blimp_link.py capture download` fetches them over the air (never over USB, which resets the chip) and `software/blimp_log.py` decodes them; the ground station's Bench page does the same with traces. The barometer fields of a record are compensated fixed point, the rest counts (plan D7, amended). Changing `sdkconfig.defaults*` does not touch an existing `sdkconfig`: delete it and run `set-target esp32s3` again.
- **Controller parameters** (`param_table.c`, `cmd_param.c`, the `param` command; 2026-10-30, increment 5's table pulled forward): the project compiles `core/` as an ESP-IDF component (`EXTRA_COMPONENT_DIRS`), and `param` lists, sets and resets every field of `blimp_params_t` by its C path (`param alt.kp 0.4`), over USB or the UDP console. `param_table.c` is portable C with an accepted range per field, tested on the host by `tools/fw_host_test.py`; a set is range- and order-checked, then `blimp_params_validate()` runs over the whole set. The live copy is RAM only (a reboot restores `params.c`, where kept gains are committed) and the control task takes it at the top of a tick through `param_take()` under a spinlock, and `blimp_step()` reads it (on the motors only in the bench hold). The ground station's Bench page and `step_test.py` record the listing in each capture's `.json`, and race mode refuses a change. `software/param_check.py` runs its bench checks over the air (console only, so the ground station may stay open) and restores every value. argtable is not used for `param`: it would read a negative value as an option.

### Altitude sensing

- **Primary altimeter: VL53L1X time-of-flight**, through the ST ULD driver in `components/vl53l1x_uld/` and the `tof` command. Its datasheet suggests its long-mode range may fall short of 2.25 m under room ambient IR. That is an **open question, to be settled by experimental measurement** at hover height, so don't treat it as decided either way. On the bench (dark room, about 1.57 m), timing budgets below 100 ms read about 5 % short.
- **Backup: Parallax PING))) 28015 ultrasonic**, in `cmd_ping.c`. It is **parked and not yet tested in this role**: it isn't in SRCS because the motors took GPIO 7–14, and its header comment explains how to restore it. It needs temperature compensation (air temperature from the barometer, via `baro_read_temperature_c()`) and a 5 V SIG divider.
- Either sensor measures slant range, so correct it with tilt: `h = d·cos θ`.

### Hardware facts that shape code

- **Sensor set (2026-10-22):** the GY-88 burned. The bench now carries stand-alone breakouts, and two of them are not the chips they were sold as:

  | Role | On the GY-88 | Stand-alone breakout now on the bus |
  |---|---|---|
  | IMU | MPU-6050 `0x68` | MPU-6050 `0x68` (boots asleep; write `PWR_MGMT_1`=0) |
  | Magnetometer | HMC5883L `0x1E`, ID "H43" at `0x0A`, directly on the bus (no MPU bypass) | **QMC5883L** `0x0D` (sold as "HMC5883L"), `0xFF` at reg `0x0D`, X/Y/Z little-endian |
  | Barometer | BMP180 `0x77`, chip ID `0x55` | **BME280** `0x76` (sold as "BMP280"), chip ID `0x60`; a real BMP280 would be `0x58` |

  `mag` and `baro` probe both addresses and pick the driver by ID register, not by address. The MPU-6050's low-pass filter is written at attach (21 Hz, `MPU6050_DLPF_CFG`) since 2026-10-28; the configuration of every chip and every rate between the chips and the window is in `docs/sensor-configuration-report.md`. **Keep both sets supported**: a new GY-88 may be bought. The GY-88 and BMP280 paths compile but have not run since the swap. The QMC5883L reads ‖m‖ ≈ 14.5 µT against the HMC5883L's 26.5 µT; its scale is unverified (`firmware/projects/i2c_tools/README.md` §14).
- **The sensors keep their register state across ESP32 resets and reflashes** (found on the GY-88; assume the same of the breakouts, since an ESP32 reset doesn't power-cycle them either). Drivers must write ranges/modes explicitly at init and read them back. Never derive scale factors from whatever the chip currently holds.
- The code uses the new `driver/i2c_master.h` API. `esp-idf-lib`'s drivers for these chips use the legacy `driver/i2c.h` API, so don't mix the two on one bus.
- Motors and the sensors shared one 5 V step-up (through the GY-88's AMS1117; how the replacement breakouts are fed has not been recorded). If the IMU drops out exactly when thrust is applied, suspect a brownout before you suspect code (see the header comment in `cmd_motor.c`).

## Optical flow architecture (`optflow_cam`)

- `main/flow_match.{c,h}` is the SAD block matcher. It is **portable C with no ESP-IDF dependency**, and that is what makes host testing possible. Keep it that way. `optflow_main.c` holds the camera setup, frame pacing, CSV output and single-letter serial commands (`h<mm>`, `f<fps>`, `z`, `s`, `?`).
- The matcher is designed to **fail honestly**: a match on the search boundary is rejected, never clamped. `ok=0` means "no measurement", not "zero motion". `spread_px` is the confidence figure. Preserve these properties.
- The flow is contaminated by rotation (about 35 mm/s per °/s), and the CAM has no gyro, so it reports raw `dx_px`/`dy_px` for de-rotation elsewhere.
- **`optflow_cam/README.md` is partly stale.** `docs/esp32-cam-optical-flow-report.md` supersedes it (§9 lists the disproved figures), and the current `sdkconfig.defaults` (240 MHz CPU, 20 MHz XCLK) differs from the README's thermal table. The committed sub-pixel estimator is `FLOW_SUBPIX_VFIT`. Measurement CSVs live in `optflow_cam/data/`.

## Docs conventions

`docs/` holds long-form technical reports. They are dated, they keep "measured" separate from "reasoned", and they list claims that couldn't be verified. Raw session reports go in `docs/history/`. Reference PDFs in `docs/reference/` are gitignored, and their citations are in `docs/reference/README.md`. The numbered `docs/0x-*.md` files and `docs/10-known-issues.md` came from DroneUN; AGENTS.md says which of them describe the current vehicle. The rulebook and the MochiSwarm paper go in `reference/` (see `reference/README.md`), also gitignored. Code comments follow the same style: prose explaining *why*, including the hardware reasoning. Match that density when editing.
