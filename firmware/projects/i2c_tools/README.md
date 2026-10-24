# `i2c_tools`: the blimp's ESP32-S3 firmware

Firmware for the **Waveshare ESP32-S3-Zero**, the board that will become the
blimp's flight controller. Right now it is a **bench instrument, not flight
firmware**. It gives you a console with one command per sensor and per
thruster, and a Wi-Fi link that carries the same console and a small UDP packet
protocol. There is no control loop and no sensor fusion yet
(`docs/sensor-fusion-plan.md` is a proposal and is not implemented).

The name and the `i2c-tools>` prompt come from the Espressif `i2c_tools`
example this project grew out of. The five stock I2C commands are still here.
Everything else was added for the blimp.

Written 2026-10-22 against branch `s3-wireless-link`. To build and flash it, see
**[`BUILD_AND_FLASH.md`](BUILD_AND_FLASH.md)**.

---

## Contents

1. [Status: what has been measured](#1-status-what-has-been-measured)
2. [Hardware and wiring](#2-hardware-and-wiring)
3. [Building and flashing (summary)](#3-building-and-flashing-summary)
4. [What happens at boot](#4-what-happens-at-boot)
5. [Command reference](#5-command-reference)
6. [Motor driver](#6-motor-driver)
7. [Wireless link](#7-wireless-link)
8. [Altitude sensing](#8-altitude-sensing)
9. [Configuration (menuconfig)](#9-configuration-menuconfig)
10. [C API for other modules](#10-c-api-for-other-modules)
11. [Adding a module](#11-adding-a-module)
12. [Design rules](#12-design-rules)
13. [File map](#13-file-map)
14. [Known issues and caveats](#14-known-issues-and-caveats)
15. [Related documents](#15-related-documents)

---

## 1. Status: what has been measured

Following the convention in `docs/`, "measured" means it ran on the bench and
the numbers were recorded. "Built" means it compiles and the reasoning is
written down, but it has not run on hardware.

| Part | State | Evidence |
|---|---|---|
| I2C bus, `i2cdetect` | **Measured** 2026-10-11, 2026-10-22 | With the GY-88 a settled bus showed exactly `1e 29 68 77` (`docs/sensor-bringup-report.md`). With the stand-alone breakouts that replaced it: `0d 29 68 76` |
| MPU-6050 (`imu`) | **Measured** 2026-10-11 | ‖a‖ = 0.933 g at rest (uncalibrated), gyro X bias −2.66 °/s |
| HMC5883L (`mag`), GY-88 | **Measured** 2026-10-11 | Field about 26.5 µT. Not calibrated (no hard/soft-iron correction). The GY-88 burned on 2026-10-22; the code path is kept, unchanged, for a replacement board, and has not run since |
| QMC5883L (`mag`), stand-alone breakout | **Measured** 2026-10-22, 7 samples | Detected at `0x0D`, chip ID `0xFF`. ‖m‖ ≈ 14.5 µT, about half the HMC5883L's figure above. Uncalibrated, and only one orientation was tried, so this is not yet a check of the scale (§14) |
| BMP180 (`baro`), GY-88 | **Measured** 2026-10-11 | Noise 0.09 hPa, about 0.75 m of altitude. Too coarse to hold a 0.4 m band alone. Burned with the GY-88; code kept, not run since |
| BME280 (`baro`), stand-alone breakout | **Measured** 2026-10-22, 10 samples per setting | Sold as a BMP280; the chip ID (`0x60`) says BME280. 853.24 hPa, 27.0 °C, 58 % RH. Pressure sd 0.008 hPa (about 0.09 m) at `-o 3`, 0.026 hPa (about 0.27 m) at `-o 0`. Ten samples on a still bench, not a noise characterisation |
| BMP280 (`baro`) | **Built, not run on hardware** | Same driver as the BME280 minus humidity; no BMP280 has been on the bus |
| VL53L1X (`tof`) | **Measured** 2026-10-11, at 1.57 m only | 1566 mm, sd 1.5 mm at a 100 ms budget, dark room. Budgets below 100 ms read about 5 % short. **Range at 2 m not measured yet** (§8) |
| Motors (`motor`) | **Measured** 2026-10-12 | 2 s run at 10 %, then the failsafe coasted it. No spurious errors |
| Per-motor break-away duty | Not measured | The 12 % default is a guess. Measure it with `--sweep` (§6) |
| Wi-Fi AP, UDP link | **Measured** 2026-10-24 | The laptop joined `ICARUS-…` and the ground station page (`software/ground_station/`) received telemetry. The rest of the checklist in `docs/wireless-link.md` §8 (console over UDP, link-loss and failsafe tests, current, range) is still open |
| Sensor stream (`sensors`, the `SENSORS` packet) | **Built, not run on hardware** | Written 2026-10-24 on the four drivers above. The ground station decodes the same packet from `software/mock_blimp.py` |
| PING))) ultrasonic | **Parked, not tested** | Not in the build (§8) |

## 2. Hardware and wiring

### Board

Waveshare ESP32-S3-Zero (ESP32-S3FH4R2: 4 MB flash, 2 MB PSRAM). Its USB-C goes
straight to the chip's own USB Serial/JTAG port, so:

- the console runs over USB (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`, in
  `sdkconfig.defaults.esp32s3`). It shows up as `/dev/ttyACM0` on Linux, or a
  `COMx` port on Windows, with USB ID `303a:1001`;
- flashing needs no BOOT button;
- **opening the port resets the chip.** If you drive the console from a script,
  wait about 3 s after opening the port and throw away the boot log before you
  send a command.

The on-board WS2812 RGB LED is on **GPIO21**, driven by RMT through the
`espressif/led_strip` component. It runs a slow hue sweep to show the firmware
is alive.

### I2C bus

```
ESP32-S3-Zero
  GPIO4 ── SDA ──┬── MPU-6050 breakout (0x68)
                 ├── QMC5883L breakout (0x0D)
                 ├── BME280 breakout (0x76)
  GPIO5 ── SCL ──┴── VL53L1X breakout (0x29)
  3V3   ── VCC
  GND   ── GND
```

This is the bench wiring since 2026-10-22, when the GY-88 burned and three
separate breakouts replaced it. With a GY-88 the first three become one board
at `0x68`, `0x1E` and `0x77`. The firmware handles either set without a
rebuild (see the table below).

There is one bus, created once in `app_main` and shared by every module through
`tool_bus_handle`. It runs at 100 kHz with the internal pull-ups on (plus
whatever the breakouts carry) and `glitch_ignore_cnt = 7`. The code uses the new
`driver/i2c_master.h` API. **Don't add drivers written for the legacy
`driver/i2c.h` API** (for example `esp-idf-lib`'s drivers for these same chips):
the two APIs can't share a bus.

| Device | Address | Notes |
|---|---|---|
| MPU-6050 (accel + gyro + temperature) | `0x68` | Boots asleep. The driver writes `PWR_MGMT_1 = 0` |
| HMC5883L (magnetometer, on the GY-88) | `0x1E` | Directly on the bus, not behind the MPU's auxiliary bus. Identified by `H43` at register `0x0A` |
| QMC5883L (magnetometer, stand-alone) | `0x0D` | What most "HMC5883L" breakouts carry today: a different chip with its own register map. Identified by `0xFF` at register `0x0D` |
| BMP180 (pressure + temperature, on the GY-88) | `0x77` | Chip ID `0x55` at register `0xD0` |
| BMP280 / BME280 (stand-alone) | `0x76` or `0x77` | Chip ID `0x58` (BMP280) or `0x60` (BME280, adds humidity) at `0xD0`. The address depends on the SDO pin; breakouts usually tie it low (`0x76`) |
| VL53L1X (time-of-flight) | `0x29` | XSHUT is optional (`tof -x <gpio>`) |

**The sensors keep their register state across ESP32 resets and reflashes**,
because the reset button doesn't reset the sensor boards (this was found on the
GY-88 and applies equally to the breakouts). Every driver therefore
writes its ranges and modes at every run and reads them back (§12).

### Motors: two DRV8833, four thrusters

| Thruster | IN1 | IN2 | Driver |
|---|---|---|---|
| FL (front left) | GPIO13 | GPIO12 | #1, IN1/IN2 |
| RL (rear left) | GPIO11 | GPIO10 | #1, IN3/IN4 |
| FR (front right) | GPIO9 | GPIO8 | #2, IN1/IN2 |
| RR (rear right) | GPIO7 | GPIO14 | #2, IN3/IN4 |

These are the Kconfig defaults (§9). nSLEEP is assumed to be tied high in
hardware (`CONFIG_MOTOR_NSLEEP_GPIO = -1`). Each driver runs one side of the
blimp, so a driver fault costs a whole side.

**If a thruster spins backwards, swap its two GPIO numbers in menuconfig**
instead of rewiring or adding sign flips in code.

**Power warning.** The motors and the sensors' AMS1117 regulator share one 5 V
step-up. If the IMU drops off the bus exactly when thrust starts, suspect a
brownout before you suspect the code. The header comment of `main/cmd_motor.c`
explains this and recommends pull-down resistors on the IN lines, so the
motors stay off while the pins float during reset.

### Pins in use

| GPIO | Use |
|---|---|
| 4, 5 | I2C SDA, SCL |
| 7–14 | Motor driver inputs (all eight LEDC channels) |
| 21 | WS2812 status LED (RMT) |
| 6 | Reserved for the parked PING))) sensor (not built in) |

## 3. Building and flashing (summary)

The full guide for Linux and Windows, with troubleshooting, is
[`BUILD_AND_FLASH.md`](BUILD_AND_FLASH.md). In short, from the repo root:

```bash
source $HOME/.espressif/tools/activate_idf_v5.5.5.sh      # activate ESP-IDF v5.5.5, in every new terminal
idf.py -C firmware/projects/i2c_tools set-target esp32s3   # once per fresh build dir
idf.py -C firmware/projects/i2c_tools build
idf.py -C firmware/projects/i2c_tools -p /dev/ttyACM0 flash monitor
```

On Windows: `scripts\idf.bat i2c_tools build` (see `docs/wireless-link.md` §10).

- `sdkconfig`, `build/`, `managed_components/` and `dependencies.lock` are
  gitignored. **Persistent configuration changes go in `sdkconfig.defaults`,
  `sdkconfig.defaults.esp32s3` or `main/Kconfig.projbuild`, never in
  `sdkconfig`.**
- `sdkconfig.defaults` sets 2 MB flash and the custom partition table, and
  `sdkconfig.defaults.esp32s3` raises flash to 4 MB and sets the USB console,
  LED pin and I2C pins.
- The partition table (`partitions_example.csv`): `nvs` at 0x9000 (24 KB, used
  by the Wi-Fi driver), `phy_init`, a 2 MB `factory` app at 0x10000, and a 528 KB
  FAT `storage` partition that holds the console history.
- A clean build on 2026-10-22 gave a 905 KB image, which leaves 56 % of the app
  partition free.

## 4. What happens at boot

`app_main` (`main/i2ctools_example_main.c`) runs these steps in this order:

1. **`motors_init()` comes first, before anything else.** The DRV8833 inputs
   float from reset while the motor supply is already live, so a floating pin
   could spin a thruster. The driver claims all eight pins immediately and sets
   every motor to coast. If it fails, the error is logged and boot continues, so
   you keep the console to find out why.
2. The console REPL is created. Command history is stored on the FAT `storage`
   partition mounted at `/data` (`CONFIG_EXAMPLE_STORE_HISTORY`).
3. The I2C master bus is created (SDA 4, SCL 5).
4. The commands are registered: the stock I2C tools, `imu`, `mag`, `baro`,
   `tof`, `motor`, `wifi`, `link`, `sensors`, `exit`/`quit`.
5. **Wireless**: `net_wifi_init()` brings up the access point, then
   `net_link_start()` and `net_console_start()` open the UDP ports. Each step
   is non-fatal. A Wi-Fi failure leaves you with the USB console.
6. **Sensor stream**: `sensor_stream_start()` creates the sampling task,
   which attaches whatever sensors answer and feeds the link's `SENSORS`
   packet (`CONFIG_SENSOR_STREAM_AT_BOOT`). Also non-fatal.
7. The LED task starts. The banner is printed, and the REPL starts on USB.

`cmd_system.h` is included, but `register_system()` is not called, so the stock
`free`, `restart`, `version` etc. commands are not available.

## 5. Command reference

Type `help` at the console for the built-in list with every option. The tables
below add the defaults and what each command actually does.

### Stock I2C tools (from the Espressif example)

| Command | What it does |
|---|---|
| `i2cconfig [--port] [--freq] [--sda] [--scl]` | Reconfigure the bus. The defaults already match this board, so you rarely need it |
| `i2cdetect` | Scan every address. Expect `1e 29 68 77` |
| `i2cget -c <chip> -r <reg> [-l <len>]` | Read registers |
| `i2cset -c <chip> -r <reg> <data>...` | Write registers |
| `i2cdump -c <chip> [-s <size>]` | Dump a device's registers |

**The first scan after a reset can show phantom addresses.** Run `i2cdetect` a
second time before you believe an unexpected entry
(`docs/sensor-bringup-report.md`).

### `imu`: MPU-6050

```
imu [-c <addr>] [-n <samples>] [-i <ms>] [-a <0-3>] [-g <0-3>]
```

| Option | Meaning | Default |
|---|---|---|
| `-c` | I2C address | `0x68` |
| `-n` | Number of samples | 10 |
| `-i` | Interval between samples, ms | 200 |
| `-a` | Accelerometer range: 0/1/2/3 = ±2/4/8/16 g | 0 |
| `-g` | Gyro range: 0/1/2/3 = ±250/500/1000/2000 °/s | 0 |

It wakes the chip, writes both ranges, reads them back and scales from what it
read back. Each sample is one 14-byte burst read, so the accelerometer, gyro and
temperature values come from the same instant. Columns: `ax ay az` [g],
`gx gy gz` [°/s], `T` [°C]. A WHO_AM_I mismatch only prints a warning, because
clones report other values.

### `mag`: HMC5883L or QMC5883L

```
mag [-c <addr>] [-n <samples>] [-i <ms>]
```

Defaults: 10 samples, 200 ms. With no `-c` it probes `0x1E`, then `0x0D`, and
picks the driver from the identification registers, not from the address. If
an address answers but neither ID matches, the address decides and a warning is
logged. Both drivers print the same table: X, Y, Z in µT, ‖m‖ and a heading
that assumes the chip lies flat with its X axis forward.

**HMC5883L.** It checks the identification bytes (`H43`), writes CONFIG_A (8-sample average, 15 Hz), confirms continuous mode
took effect, and waits for the data-ready bit (500 ms timeout) for each sample.
The gain is read back from CONFIG_B, not assumed. It prints X, Y, Z in µT plus a
heading. An axis that saturates (the chip reports −4096) prints `-` and is not
reported as a number.

**QMC5883L.** A soft reset first (it has the same "survives an ESP32 reset"
problem), then SET/RESET period `0x01` and control 1 = `0x15`: oversampling
512, range ±8 G, 50 Hz, continuous. Both registers are read back, and the run
aborts if they didn't stick. Range ±8 G is fixed on purpose: the chip's noise
is several times the ±8 G step, so ±2 G would add no real resolution, and the
motor magnets nearby could saturate it. Scale is 3000 LSB/G (datasheet). It
waits for DRDY per sample. The data are X, Y, Z little-endian (the HMC5883L is
X, Z, Y big-endian). The overflow flag (OVL) prints `-`.

**The output of both is uncalibrated**, so treat the heading as indicative
only.

### `baro`: BMP180, BMP280 or BME280

```
baro [-c <addr>] [-n <samples>] [-i <ms>] [-o <0-3>]
```

Defaults: 10 samples, 500 ms, oversampling 3 (highest). With no `-c` it probes
`0x77`, then `0x76`, and picks the driver from the chip ID at `0xD0`. An
unknown ID is logged and skipped. Every driver rejects calibration words that
are all-zero or all-one (the sign of a bad read) and runs Bosch's integer
compensation.

- **BMP180** (`0x55`): unchanged from before. `-o` is the chip's own OSS
  setting.
- **BMP280 / BME280** (`0x58` / `0x60`): soft reset, wait for the calibration
  copy to finish, read calibration, write the configuration in sleep mode and
  read it back. Each sample is a forced-mode conversion: the driver waits the
  datasheet's maximum conversion time, polls the measuring bit, then reads all
  results in one burst so they belong to the same conversion. `-o` maps to
  (pressure, temperature) oversampling ×2/×1, ×4/×1, ×8/×1, ×16/×2. The IIR
  filter is off, so consecutive samples are independent. Humidity (BME280
  only) is ×1 and is printed as a fourth column. A result the chip marks as
  skipped prints `-` or an error, not a number.

The header line names the chip, address, oversampling and conversion time. It
prints temperature, pressure, and altitude computed against the standard 1013.25 hPa sea-level
pressure. That altitude is useful for *differences*, not as an absolute
height.

### `tof`: VL53L1X time-of-flight

```
tof [-c <addr>] [-n <samples>] [-d 1|2] [-b <ms>] [-i <ms>] [-x <gpio>]
```

| Option | Meaning | Default |
|---|---|---|
| `-c` | I2C address | `0x29` |
| `-n` | Samples (max 200) | 10 |
| `-d` | Distance mode: 1 = short (about 1.3 m), 2 = long | 2 |
| `-b` | Timing budget, ms: 20, 33, 50, 100, 200, 500 (15 only in short mode) | 100 |
| `-i` | Inter-measurement period, ms (must be ≥ the budget) | = budget |
| `-x` | XSHUT GPIO, if wired | none |

It checks model ID `0xEACC`, waits for the sensor's boot, runs `SensorInit`,
writes mode and budget, reads both back, then waits for data-ready on every
sample. Each line shows the distance, the range status **as text** (an invalid
reading is labelled, not hidden), signal rate, ambient rate and SPAD count.
The run ends with statistics. If ambient light is above 50 kcps/SPAD it warns,
because ambient IR is what limits long-range performance (§8).

### `motor`: thrusters

```
motor -m <FL|RL|FR|RR|0-3> -t <-1..1> [-d <ms>] [--hold]
motor --sweep -m <id> [-t <negative for reverse>] [-d <ms per step>]
motor -s | -b
motor [--decay slow|fast] [--deadband <percent>] [--failsafe <ms>]
motor [--status]
```

| Option | Meaning |
|---|---|
| `-m` | Which thruster: `FL`, `RL`, `FR`, `RR`, or 0–3 in that order |
| `-t` | Signed thrust, −1.0 to +1.0. 0 coasts |
| `-d` | Run time before an automatic coast. Default 1500 ms, max 30000 |
| `--hold` | Leave the motor running after the command returns. The failsafe still coasts it after 1 s unless you disable the failsafe with `--failsafe 0` |
| `-s` | Coast all four (outputs off, the prop freewheels) |
| `-b` | Brake all four (both outputs low, the prop is held) |
| `--sweep` | Ramp one motor from 5 % to 100 % duty in 20 steps, with the deadband switched off, so you can see where it starts turning. Each step lasts `-d` ms |
| `--decay` | PWM decay mode. Default `slow` |
| `--deadband` | Break-away duty in percent, 0–90 |
| `--failsafe` | Failsafe timeout in ms. 0 disables it |
| `--status` | Pins, PWM frequency and resolution, decay, deadband, failsafe state, standing commands. A bare `motor` prints the same |

Settings options can go on the same line as a run, for example
`motor --deadband 15 -m FL -t 0.2`. The console can't interrupt a running
command, so a timed run blocks the prompt until it ends.

### `wifi`: access point

```
wifi              status: SSID, channel, security, address, ports, stations with RSSI
wifi --start      bring the access point up
wifi --stop       radio off (and the UDP console goes with it)
```

### `link`: UDP packet link

`link` prints the UDP port, telemetry rate and timeout, the last host that sent
a valid packet and whether it is alive or STALE, how many packets were received,
rejected and sent, and whether the motors are currently being driven over the
link.

### `sensors`: the sensor stream

```
sensors            what the stream found, its latest readings, sample and error counts
sensors --off      stop sampling and release every sensor
sensors --on       start again
```

`sensor_stream.c` samples the IMU, the magnetometer and the VL53L1X 20 times a
second and the barometer 4 times a second, through the same drivers as the
four commands above, and the link sends each sample as a `SENSORS` packet (§7)
for the ground station. It starts at boot (`SENSOR_STREAM_AT_BOOT`). A sensor
that is missing is probed again every 2 s, quietly, so a breakout plugged in
later shows up; one whose reads fail three times in a row, or that stops
producing for a second, is released and re-attached the same way. The
barometer's altitude in the packet is relative to the first reading since
boot; `sensors` prints both that and the absolute one.

**`imu`, `mag`, `baro`, `tof` and `i2cconfig` pause the stream** for the length
of their run: each command resets and reconfigures its chip, the ToF port
allows one attach per address, and a bus with devices on it cannot be
deleted. The stream re-attaches, reconfiguring from scratch, when the command
returns. On the ground station that shows as a "paused" notice for the length
of the command.

### `exit` / `quit`: how to leave

These don't close anything. They print how to leave. The console runs on the
chip, and the program holding the serial port is `idf.py monitor` on the
laptop, so the chip can't close it. Over USB the hint is **Ctrl+X** (or Ctrl+T
followed by `x`), the exit key set by this folder's `esp-idf-monitor.cfg`
(see `BUILD_AND_FLASH.md`). Ctrl+C is passed to the chip, not the monitor. Over
the UDP console, the hint says that `blimp_link.py` leaves on `exit` or Ctrl+C.
`blimp_link.py` handles that on the laptop, so the blimp normally never sees
the command.

## 6. Motor driver

`main/cmd_motor.c`. The header comment there is the long version of this
section.

- **One LEDC channel per DRV8833 input, all eight of them.** That is what makes
  a reversal glitch-free: both inputs of a motor change on their own channels
  without reconfiguring a pin. As a result **nothing else in this firmware may
  use LEDC**. The status LED uses RMT for this reason.
- **PWM** runs at 20 kHz, above hearing, which matters because the envelope
  amplifies sound. At 20 kHz LEDC gives 11 bits (2048 steps).
- **Decay mode.** In *slow* decay (the default) the off part of each PWM period
  brakes the motor. That gives a more linear speed-to-duty curve and is the
  usual choice. In *fast* decay the off part coasts.
- **Deadband.** A small brushed motor doesn't turn below some break-away duty.
  Commands are mapped onto `[deadband, 1.0]`, so a small command gives a small
  thrust instead of nothing. The default of 12 % is a starting guess. **Measure
  each motor with `motor --sweep -m FL` (and RL, FR, RR)**, then set the value
  with `--deadband` at runtime or `CONFIG_MOTOR_DEADBAND_PERMILLE` permanently.
- **Failsafe** (`CONFIG_MOTOR_FAILSAFE_MS`, default 1000 ms). The PWM peripheral
  holds its last duty forever, so a controller that stops running would leave
  the blimp under thrust. The failsafe is armed only while some motor has a
  non-zero command. If no thrust command arrives within the window, every motor
  coasts. Its states are `off` (disabled), `idle` (no motor driven), `armed` and
  `tripped`. **Any future controller must keep calling the thrust API**
  (`motor_set`/`motors_set_all`, or `motors_failsafe_feed`), or the motors stop.
- Non-finite commands (NaN, infinity) are rejected, not clamped.

## 7. Wireless link

Files: `net_wifi.c`, `net_link.c`, `net_console.c`, and `link_protocol.h` for
the packet formats. The full design, the reasoning behind it and the bring-up
checklist are in **`docs/wireless-link.md`**. The access point and the packet
link first ran on hardware on 2026-10-24 (the laptop joined and the ground
station received telemetry); the rest of that checklist is still open (§1).

### Access point

The S3 is its own Wi-Fi access point, so no router is needed and the laptop's
built-in Wi-Fi is the whole ground station.

- SSID `ICARUS-XXXXXX`, where the suffix is the last three bytes of the board's
  MAC, so two blimps in one room can be told apart.
- WPA2, default password `icarusblimp`. An empty password makes an open network,
  which is for the bench only: the password is the only thing standing between a
  stray phone and the motors.
- Channel 1, at most 2 stations. The blimp is `192.168.4.1`, and the first
  laptop normally gets `192.168.4.2`.
- The configuration is rebuilt from Kconfig at every boot and is not stored in
  NVS. If NVS is from an incompatible version it is erased automatically,
  because nothing else in this firmware stores data there.
- Power: the radio draws about 100 mA listening and up to about 300 mA in
  transmit bursts, from the same 5 V step-up as the motors and sensors.

### UDP 7200: packet link

Fixed-size little-endian packets. Each one has an 8-byte header (magic
`0xB1A7`, version 1, type, sequence number) and ends with a zlib-style CRC-32.
A packet with the wrong size, magic, version or CRC is counted and dropped.

| Type | Code | Size | Direction | Content |
|---|---|---|---|---|
| HEARTBEAT | `0x01` | 12 B | host → blimp | Keeps the link alive |
| MOTOR_SET | `0x02` | 28 B | host → blimp | `float cmd[4]`, in FL, RL, FR, RR order, −1..1 |
| MOTOR_STOP | `0x03` | 16 B | host → blimp | Coast, or brake if the brake byte is 1 |
| TELEMETRY | `0x81` | 52 B | blimp → host | Clock, the four motor commands the driver holds, failsafe state, link state, station count, packet counters, free heap |
| SENSORS | `0x82` | 76 B | blimp → host | Clock, accelerometer and gyroscope (g, °/s), IMU temperature, magnetometer (gauss) and heading, pressure, barometer temperature, altitude relative to the first reading, ToF range (mm) and status, and a `valid` bitmask (IMU 1, MAG 2, BARO 4, TOF 8) |

- **`MOTOR_SET` is a standing command only while the host keeps repeating it.**
  `net_link.c` has its own timeout (500 ms, reset by any valid packet). When it
  expires, it coasts the motors that were commanded over the air. This is
  separate from the motor failsafe in §6, and it never touches motors commanded
  from the console.
- The blimp sends telemetry (20 Hz) to the last host that sent a valid packet.
  It keeps sending for ten times the timeout after the host goes quiet, then
  stops.
- One `SENSORS` packet follows each telemetry packet while the sensor stream
  has a new sample (`sensors`, §5); none while the stream is paused or off,
  so the ground station sees a gap rather than a frozen number.

### UDP 7201: console over the air

The same console as USB: send one command line per datagram. The reply comes
back in datagrams of up to 1400 bytes, and a single `0x04` byte marks the end.
Output is captured by redirecting the console task's own `stdout`, so **log
messages from other tasks still go to USB**, not to the UDP reply.

**Use one console at a time.** `esp_console_run()` uses a static line buffer,
so typing on USB while a UDP command runs can corrupt both.

### Ground tool

`software/blimp_link.py` needs only the Python standard library (3.8+), on any
OS. Join the `ICARUS-…` network first, then:

```bash
python software/blimp_link.py selftest              # packet codec check, no blimp needed
python software/blimp_link.py status                # telemetry, 5 s by default
python software/blimp_link.py console               # interactive console over UDP
python software/blimp_link.py console "tof -n 5"    # one command
python software/blimp_link.py motor 0.2 0 0 0 --seconds 2   # FL RL FR RR
python software/blimp_link.py stop [--brake]
```

`motor` limits commands to ±0.3 unless you pass `--limit`. To try all of this
without hardware, run `python software/mock_blimp.py` in one terminal and add
`--host 127.0.0.1` to the commands above.

## 8. Altitude sensing

The blimp has to hover at 2 m, inside a band from 1.85 m to 2.25 m.

- **Primary: VL53L1X time-of-flight** (`tof`, driver in
  `components/vl53l1x_uld/`). Its datasheet suggests long mode may fall short of
  2.25 m when the room has strong ambient IR. **Whether it reaches the hover
  height is an open question, to be settled by measurement at 2 m in the real
  room**, so don't treat it as decided either way. What has been measured: 1566 mm with sd 1.5 mm
  at a 100 ms budget in a dark room. Budgets below 100 ms read about 5 % short.
- **Backup: Parallax PING))) 28015 ultrasonic** (`main/cmd_ping.c`). **It is
  parked**: it isn't in `SRCS`, because the motors took GPIO 7–14. It has not
  been tested in this role. It needs temperature compensation (air temperature
  from `baro_read_temperature_c()`) and a 1.0 k / 1.8 k divider on SIG, since the
  sensor is 5 V. To restore it, add it back in the four places listed in §11.
  The header comment of `cmd_ping.c` gives the details.
- **The barometer is not an altimeter at this scale.** The BMP180's noise of
  about 0.75 m was wider than half the band. The BME280 that replaced it read
  about 0.09 m sd over ten samples (§1), which is promising, but barometric
  height also drifts with the weather and with doors and air conditioning, so
  at best it can help hold a height that another sensor sets.
- Both rangefinders measure slant range. Correct for tilt with **h = d · cos θ**,
  using the IMU's attitude.

> The header comment in `cmd_tof.c` and the 2026-10-11 bring-up report say
> the ToF can't reach 2 m and that the PING))) is the altimeter. That was the
> conclusion before the 2026-10-13 decision recorded in the top-level
> `CLAUDE.md`. Follow this section and `CLAUDE.md`, not those older texts.

## 9. Configuration (menuconfig)

`idf.py -C firmware/projects/i2c_tools menuconfig`. These settings live in
`main/Kconfig.projbuild`. To make a change permanent for everyone, change the
default there or put it in `sdkconfig.defaults*`.

| Menu | Setting | Default |
|---|---|---|
| Example Configuration | `EXAMPLE_STORE_HISTORY` (console history on FAT) | y |
| | `EXAMPLE_I2C_MASTER_SCL` / `_SDA` | 5 / 4 |
| | `EXAMPLE_BLINK_GPIO` / `EXAMPLE_BLINK_PERIOD` | 21 / 3000 ms |
| Motor driver (DRV8833) | `MOTOR_{FL,RL,FR,RR}_IN{1,2}_GPIO` | see the §2 table |
| | `MOTOR_NSLEEP_GPIO` | −1 (tied high in hardware) |
| | `MOTOR_PWM_FREQ_HZ` | 20000 |
| | `MOTOR_DEADBAND_PERMILLE` | 120 (12 %) |
| | `MOTOR_FAILSAFE_MS` | 1000 (0 disables) |
| Wireless link (Wi-Fi access point + UDP) | `NET_WIFI_AT_BOOT` | y (otherwise use `wifi --start`) |
| | `NET_WIFI_SSID_PREFIX` | `ICARUS` |
| | `NET_WIFI_PASSWORD` | `icarusblimp` (empty means open, otherwise 8–63 characters) |
| | `NET_WIFI_CHANNEL` / `NET_WIFI_MAX_STA` | 1 / 2 |
| | `NET_LINK_PORT` / `NET_CONSOLE_PORT` | 7200 / 7201 |
| | `NET_LINK_TIMEOUT_MS` | 500 |
| | `NET_LINK_TELEMETRY_HZ` | 20 |
| Sensor stream | `SENSOR_STREAM_AT_BOOT` | y (otherwise `sensors --on`) |
| | `SENSOR_STREAM_HZ` | 20 |

## 10. C API for other modules

Each `cmd_<name>.c` registers a console command **and** exposes a small C API
in its header, so a future controller can call the hardware directly
without parsing console output.

| Header | Functions |
|---|---|
| `cmd_motor.h` | `motors_init`, `motor_set`, `motors_set_all`, `motor_get`, `motors_coast`, `motors_brake`, `motors_set_decay`, `motors_set_deadband`, `motors_failsafe_set_timeout_ms`, `motors_failsafe_feed`, `motors_failsafe_tripped`, `motors_failsafe_state`, `motor_name`, `motor_where` |
| `cmd_imu.h` | `imu_attach`, `imu_read`, `imu_ranges`, `imu_addr`, `imu_detach` |
| `cmd_mag.h` | `mag_attach`, `mag_read`, `mag_info`, `mag_detach` |
| `cmd_baro.h` | `baro_attach`, `baro_read`, `baro_info`, `baro_detach`; `baro_read_temperature_c` (for speed-of-sound compensation; compiled, but its only caller `cmd_ping.c` is parked) |
| `cmd_tof.h` | `tof_attach`, `tof_read`, `tof_info`, `tof_detach`, `tof_budget_valid`, `tof_status_text` |
| `sensor_stream.h` | `sensor_stream_start`, `sensor_stream_pause`, `sensor_stream_resume`, `sensor_stream_latest` |
| `net_wifi.h` | `net_wifi_init`, `net_wifi_start`, `net_wifi_stop`, `net_wifi_is_up`, `net_wifi_station_count`, `net_wifi_ssid`, `net_wifi_ip_str` |
| `net_link.h`, `net_console.h` | Start functions and link status |
| `components/vl53l1x_uld` | ST's ULD API, plus `vl53l1x_port_attach(bus, addr)` / `vl53l1x_port_detach` to bind it to the shared bus |

Every sensor driver has the same shape: `xxx_attach()` finds the chip, adds it
to the bus and configures it from scratch (ranges written and read back),
`xxx_read()` returns one decoded sample, `xxx_detach()` removes it. The
console command and the sensor stream are both loops over that API, which is
why the bench commands remain the regression test for what the ground
station plots. A caller that wants a chip while the stream is running
brackets its work with `sensor_stream_pause()` / `sensor_stream_resume()`.

The shared bus handle is `tool_bus_handle` (declared in `cmd_i2ctools.h`).
Drivers add their device to that bus. They never create or delete the bus
themselves, and they treat a NULL handle as "no bus".

## 11. Adding a module

A new peripheral is a new `main/cmd_<name>.c` / `.h` pair with a
`register_<name>()` function. To include it in the build, change these four
places:

1. `main/CMakeLists.txt`: add the file to `SRCS`. Add any new IDF component to
   `PRIV_REQUIRES` as well. The list is explicit, so a missing component shows
   up only as a bare `No such file or directory` on a header.
2. `main/i2ctools_example_main.c`: add the `#include`.
3. The same file: add the `register_<name>()` call.
4. The same file: add a banner line, so `help` isn't the only place the command
   is mentioned.

## 12. Design rules

These came out of the bring-up work (`docs/sensor-bringup-report.md`). New
code should follow them too.

- **Never derive a scale factor from whatever the chip currently holds.**
  Write the configuration, read it back, and scale from what you read back. The
  GY-88 survives ESP32 resets with whatever an earlier run left in it.
- **Never invent a reading.** An invalid, saturated or timed-out sample is
  reported as such (`-`, a status string, an error), never as a plausible
  number.
- **Drivers don't own the bus.** They attach to `tool_bus_handle`.
- **Respect the 10 ms FreeRTOS tick.** Delays shorter than a tick round to
  zero or one tick, so short waits poll a status bit instead.
- **Failures are not fatal.** A missing sensor or a Wi-Fi error is logged, and
  the console stays up so you can investigate.
- **Vendored code is checked, not trusted.** See
  `components/vl53l1x_uld/README.md` for what was compared against ST's
  original and the two header edits that were made.

## 13. File map

```
i2c_tools/
├── README.md                 this file
├── BUILD_AND_FLASH.md        build and flash guide, Linux and Windows
├── esp-idf-monitor.cfg       makes Ctrl+X the exit key of idf.py monitor
├── CMakeLists.txt            project file (MINIMAL_BUILD)
├── partitions_example.csv    nvs, phy_init, 2 MB factory app, FAT storage
├── sdkconfig.defaults        common defaults (custom partitions, 2 MB flash)
├── sdkconfig.defaults.esp32s3  S3-Zero: 4 MB, USB Serial/JTAG console, LED 21, I2C 4/5
├── components/
│   └── vl53l1x_uld/          ST VL53L1X ULD 3.5.5 + port onto the shared bus
└── main/
    ├── i2ctools_example_main.c   app_main: boot order, registration, banner
    ├── Kconfig.projbuild         menuconfig settings (§9)
    ├── idf_component.yml         led_strip + example console helpers
    ├── cmd_i2ctools.c/.h         stock i2cconfig/detect/get/set/dump, shared bus handle
    ├── cmd_imu.c/.h              MPU-6050 driver + `imu`
    ├── cmd_mag.c/.h              HMC5883L or QMC5883L driver, auto-detected, + `mag`
    ├── cmd_baro.c/.h             BMP180, BMP280 or BME280 driver, auto-detected, + `baro`
    ├── cmd_tof.c/.h              VL53L1X driver + `tof`
    ├── sensor_stream.c/.h        samples every sensor for the SENSORS packet + `sensors`
    ├── cmd_motor.c/.h            DRV8833 ×2, failsafe
    ├── cmd_ping.c/.h             PING))) ultrasonic, PARKED (not in SRCS)
    ├── led_blink.c/.h            WS2812 hue sweep
    ├── net_wifi.c/.h             access point + `wifi` command
    ├── net_link.c/.h             UDP packet link + `link` command
    ├── net_console.c/.h          console over UDP
    └── link_protocol.h           packet layouts, shared with blimp_link.py
```

The `pytest_*.py` file in this folder is Espressif's stock example test, written
for different hardware (a CCS811). It is not a test of this firmware.

## 14. Known issues and caveats

- **No calibration anywhere.** Accelerometer, gyro and magnetometer values are
  raw and scaled, not calibrated. The gyro bias alone is several °/s.
- **QMC5883L field strength unverified.** It read ‖m‖ ≈ 14.5 µT on
  2026-10-22, against about 26.5 µT from the HMC5883L in the same lab. A local
  hard-iron offset or nearby steel can do that to an uncalibrated sensor, and
  so could a clone whose scale differs from the QST datasheet. Turning the
  board slowly through a full circle, flat and then on edge, and checking that
  the readings trace a sphere (and what radius) would separate the two.
- **Phantom I2C addresses** on the first scan after a reset (§5).
- **Brownout** when the motors start: the motors and sensors share one 5 V rail
  (§2).
- **The bench wiring is not flight wiring.** It relies on the internal pull-ups
  and on short jumper leads. The bring-up report explains what has to change for
  flight.
- **One console at a time** (USB or UDP), and UDP replies don't include logs
  from other tasks (§7).
- **The sensor stream and the console share the chips by turns.** `imu`,
  `mag`, `baro`, `tof` and `i2cconfig` pause the stream, which releases every
  sensor, and it re-attaches when they return; the ground station's sensor
  panels go stale for the length of the command. `i2cget`/`i2cset` on a
  sensor register while the stream runs do not pause it, so what they change
  is what the stream reads until its next re-attach.
- **Stale text to correct later:**
  - `cmd_tof.c`'s header comment about the altimeter decision (§8);
  - the `MOTOR_DEADBAND_PERMILLE` help in `Kconfig.projbuild`, which says
    `motor -m 1A --sweep`. The current syntax is `motor -m FL --sweep`.
- A `build/` folder copied from another checkout is tied to its original path,
  and the build fails with "configured for project …". Run `fullclean` (see
  `BUILD_AND_FLASH.md`).

## 15. Related documents

| Document | What's in it |
|---|---|
| [`BUILD_AND_FLASH.md`](BUILD_AND_FLASH.md) | Installing ESP-IDF, building, flashing, troubleshooting |
| `docs/sensor-bringup-report.md` | Sensor bring-up, 2026-10-11: wiring, measurements, every problem found |
| `docs/wireless-link.md` | Wireless design, packet protocol, bring-up checklist, Windows build |
| `docs/sensor-fusion-plan.md` | Proposed `blimp_ins` fusion (not implemented) |
| `docs/inter-board-link-plan.md` | Proposed UART link to the ESP32-CAM optical-flow board (draft) |
| `components/vl53l1x_uld/README.md` | Where the ToF driver came from and what was changed |
| Top-level `CLAUDE.md` | Project overview and the current altimeter decision |
