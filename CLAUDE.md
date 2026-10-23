# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An indoor autonomous blimp that has to hover at 2 m (band 1.85–2.25 m). It is a two-chip system:

| Path | Chip | Role |
|---|---|---|
| `firmware/projects/i2c_tools/` | ESP32-S3 (Waveshare ESP32-S3-Zero) | The flight-controller-to-be: IMU, magnetometer and barometer (stand-alone breakouts since the GY-88 burned; see below), altimeter, four DRV8833 thrusters, and since the `s3-wireless-link` work its own Wi-Fi access point carrying the console and a UDP packet link (`docs/wireless-link.md`). The name comes from the Espressif `i2c_tools` example it grew out of, and the console prompt is still `i2c-tools>`. |
| `firmware/projects/optflow_cam/` | ESP32-CAM (AI-Thinker, original ESP32) | Downward optical flow. It prints CSV over UART to a laptop. |
| `firmware/projects/blink/` | — | Stock ESP-IDF example, kept as a toolchain smoke test. |
| `software/` | host | `blimp_link.py` is the ground tool for this repo's firmware (standard library only; `mock_blimp.py` fakes the firmware on localhost). The other Python tools were imported from a **separate quadcopter project (esp-drone)**: they speak CRTP-over-UDP to esp-drone firmware, **not** to anything in this repo. See `docs/esp-drone-tools-report.md` §8 for what would need to change to use them here. |

The two chips are not linked yet. `docs/inter-board-link-plan.md` (untracked draft) proposes UART links. `docs/sensor-fusion-plan.md` is a proposal (`blimp_ins`) that is **not implemented**. `hardware/`, `scripts/`, `simulation/`, `tests/` and `external/` are empty placeholders, and the top-level `README.md` is empty.

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
- **ESP32-S3-Zero:** it uses native USB with no UART bridge, so `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` is required (it is set in `sdkconfig.defaults.esp32s3`). It flashes without the BOOT button. **Opening `/dev/ttyACM0` resets the chip**, so wait about 3 s and discard the boot log before you send console commands.
- **ESP32-CAM:** it flashes through an external programmer and has a GPIO0 bootloader trap. Read `docs/esp32-cam-optical-flow-report.md` §7 before flashing it.
- **Windows:** `scripts\idf.bat i2c_tools build` (or any `idf.py` arguments after the project name). It expects ESP-IDF v5.5.5 at `%USERPROFILE%\esp\esp-idf-v5.5.5` and Python 3.11 at its per-user default, and works around `export.bat` bailing out silently under Git Bash's `MSYSTEM` variable; see `docs/wireless-link.md` §10.

## Tests

The `pytest_*.py` files under `firmware/projects/` are Espressif's stock example tests for other hardware (a CCS811 and an LED). They are not this project's tests.

The real automated test is the optical-flow matcher, run on the host with no hardware:

```bash
make -C firmware/projects/optflow_cam/host test    # verification suite (test_flow_match.c)
make -C firmware/projects/optflow_cam/host sweep   # sub-pixel bias sweep (bias_sweep.c)
```

Everything else is verified on the bench through the S3 console (`help` lists commands; each sensor/motor has its own command). The user does the physical steps, so ask for a physical action, then stop and wait for confirmation before you run anything that depends on it.

## Python tools (`software/`)

Use the conda env described in `software/SETUP.md`: `blimp-tools`, Python 3.10, with `PYTHONNOUSERSITE=1` set **before** any `pip install`, because user-site packages have already broken this env once. Checks that need no hardware:

```bash
python software/drone_dynamics.py      # self-check
python software/motor_diag.py --help
QT_QPA_PLATFORM=offscreen python -c "import sys; sys.path.insert(0, 'software'); import drone_gui"
```

The GUI labels are in Spanish. `optflow_cam/host/flow_monitor.py` is a separate serial viewer/logger and needs `pyserial`.

`software/blimp_link.py` and `software/mock_blimp.py` need only the standard library (Python 3.8+), on any OS. `python software/blimp_link.py selftest` checks the packet codec; `python software/mock_blimp.py` in one terminal and `python software/blimp_link.py --host 127.0.0.1 status` in another exercises the link with no hardware.

## S3 firmware architecture (`i2c_tools`)

- `app_main` (`i2ctools_example_main.c`) creates the one I2C master bus (`tool_bus_handle`, SDA=4, SCL=5) and starts an `esp_console` REPL. Every peripheral is a `cmd_<name>.c` module that exposes `register_<name>()` plus a small C API for other modules to call, e.g. `baro_read_temperature_c()`. To add a module you need to touch four places: `main/CMakeLists.txt` SRCS, the include, the `register_*()` call and the banner line in `app_main`.
- `motors_init()` runs **first**, before the console. The DRV8833 inputs float from reset while VM is already live, so the driver must claim the pins immediately. A motor init failure is logged, not fatal, so the console survives.
- The motor driver uses **all eight LEDC channels**, one per DRV8833 input, which gives glitch-free reversal. Nothing else may use LEDC. The status LED is a WS2812 on GPIO21, driven by RMT through `led_strip`.
- **Motor failsafe** (`CONFIG_MOTOR_FAILSAFE_MS`, default 1000): it is armed only while some motor has a non-zero command. If no thrust command arrives within the window, every motor coasts. Any future controller must keep calling the thrust API (which feeds the failsafe) or the motors will stop.
- **Wireless link** (`net_wifi.c`, `net_link.c`, `net_console.c`, formats in `link_protocol.h`; design and bring-up checklist in `docs/wireless-link.md`). The S3 is a Wi-Fi access point (`ICARUS-<mac>`, WPA2, settings under "Wireless link" in menuconfig). UDP 7200 carries fixed-size packets: `MOTOR_SET` is a standing command only while the host repeats it, and `net_link.c`'s own timeout (500 ms, any packet) coasts motors that were commanded over the air, independently of the motor failsafe above. UDP 7201 is the same console as USB, one line per datagram, replies closed by a 0x04 byte; it captures output by rebinding its task's per-task `stdout`, so logs from other tasks still go to USB. Run one console at a time: `esp_console_run()` has a static line buffer. `wifi` and `link` are the status commands.

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

  `mag` and `baro` probe both addresses and pick the driver by ID register, not by address. **Keep both sets supported**: a new GY-88 may be bought. The GY-88 and BMP280 paths compile but have not run since the swap. The QMC5883L reads ‖m‖ ≈ 14.5 µT against the HMC5883L's 26.5 µT; its scale is unverified (`firmware/projects/i2c_tools/README.md` §14).
- **The sensors keep their register state across ESP32 resets and reflashes** (found on the GY-88; assume the same of the breakouts, since an ESP32 reset doesn't power-cycle them either). Drivers must write ranges/modes explicitly at init and read them back. Never derive scale factors from whatever the chip currently holds.
- The code uses the new `driver/i2c_master.h` API. `esp-idf-lib`'s drivers for these chips use the legacy `driver/i2c.h` API, so don't mix the two on one bus.
- Motors and the sensors shared one 5 V step-up (through the GY-88's AMS1117; how the replacement breakouts are fed has not been recorded). If the IMU drops out exactly when thrust is applied, suspect a brownout before you suspect code (see the header comment in `cmd_motor.c`).

## Optical flow architecture (`optflow_cam`)

- `main/flow_match.{c,h}` is the SAD block matcher. It is **portable C with no ESP-IDF dependency**, and that is what makes host testing possible. Keep it that way. `optflow_main.c` holds the camera setup, frame pacing, CSV output and single-letter serial commands (`h<mm>`, `f<fps>`, `z`, `s`, `?`).
- The matcher is designed to **fail honestly**: a match on the search boundary is rejected, never clamped. `ok=0` means "no measurement", not "zero motion". `spread_px` is the confidence figure. Preserve these properties.
- The flow is contaminated by rotation (about 35 mm/s per °/s), and the CAM has no gyro, so it reports raw `dx_px`/`dy_px` for de-rotation elsewhere.
- **`optflow_cam/README.md` is partly stale.** `docs/esp32-cam-optical-flow-report.md` supersedes it (§9 lists the disproved figures), and the current `sdkconfig.defaults` (240 MHz CPU, 20 MHz XCLK) differs from the README's thermal table. The committed sub-pixel estimator is `FLOW_SUBPIX_VFIT`. Measurement CSVs live in `optflow_cam/data/`.

## Docs conventions

`docs/` holds long-form technical reports. They are dated, they keep "measured" separate from "reasoned", and they list claims that couldn't be verified. Raw session reports go in `docs/history/`. Reference PDFs in `docs/reference/` are gitignored, and their citations are in `docs/reference/README.md`. Code comments follow the same style: prose explaining *why*, including the hardware reasoning. Match that density when editing.
