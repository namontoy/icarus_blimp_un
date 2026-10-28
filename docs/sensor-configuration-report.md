# Sensors, actuators and the data path: configuration and rates as of 2026-10-28

**Target:** Waveshare ESP32-S3-Zero, `firmware/projects/i2c_tools/`, with the ground station
`software/ground_station/` over the blimp's own access point
**Branch:** `s3-wireless-link`, after f393bf9 (Nerio's Wi-Fi fix and deadband change) plus the
changes of this report
**Date:** 2026-10-28
**Status:** the configuration below is read from the code; the rates marked "measured" come
from the bench records of 2026-10-27/28 in `docs/control-loop-plan.md` section 6 and
`docs/wifi-drop-report.md`. The four changes made today (section 5) are built and pass the
host tests; none has run on hardware yet. An interactive version of sections 3 and 4 is the
page "Icarus Signal Path", https://claude.ai/artifact/3ZfKTD5EJriVvfrytv1Vgg (private to the
team; a view of this report, not a second source).

---

## 1. Summary

The sensors are configured explicitly at every attach, as the hardware rules ask, with one
omission: the MPU-6050's digital low-pass filter was never written, so it stayed at the
power-on 260 Hz while the firmware sampled it at 20 Hz. That is the one misconfiguration found.
The other inconsistencies a reader of the traces would see are not in the chips but in the
rates between layers: the stream polled at 20 Hz while the loop records at 50 Hz, so two or
three records in a row carried the same IMU sample; the window took the newest sensor packet
at each of its 10 Hz ticks, so half of the packets never reached the plots; and the capture
shortened any request longer than its buffer without saying so, which read as a progress bar
that filled early and as a "120 s limit". All four are changed today (section 5). What remains
open is calibration, not configuration: the QMC5883L's scale, the mounting orientation of the
breakouts, and the ToF's reach, listed in section 6.

## 2. The elements and how each is configured

Every driver writes its chip's configuration at attach and reads it back, because the chips
keep their registers across ESP32 resets (CLAUDE.md, "Hardware facts"). All four sensors share
one I2C bus, SDA 4, SCL 5, internal pull-ups, each device at 100 kHz.

| Element | Chip, address | Configuration written at attach | Where |
|---|---|---|---|
| IMU | MPU-6050 at 0x68 | `PWR_MGMT_1` = 0 (wake, internal 8 MHz clock); accelerometer ±2 g (16384 LSB/g); gyro ±250 °/s (131 LSB/(°/s)); **since today** `CONFIG.DLPF_CFG` = 4 (21 Hz accel, 20 Hz gyro, 8.5 ms group delay) and `SMPLRT_DIV` = 0 (1 kHz register rate). Ranges and filter read back; a mismatch is logged. | `cmd_imu.c` `imu_configure()` |
| Magnetometer on the bench | QMC5883L at 0x0D (sold as HMC5883L) | soft reset; SET/RESET period 1; `CONTROL_1` = 0x15: OSR 512, range ±8 G (3000 LSB/G), ODR 50 Hz, continuous. Samples taken only when DRDY says one is new; OVL marks an overflow and the sample is dropped. | `cmd_mag.c` `qmc5883l_configure()` |
| Magnetometer on a GY-88 | HMC5883L at 0x1E | `CONFIG_A` = 0x70: 8 samples averaged, 15 Hz, normal bias; `MODE` continuous, read back. `CONFIG_B` (gain) is **read, not written**: the driver uses whatever gain the chip holds (power-on 1090 LSB/G, ±1.3 G). Not on the bench since the GY-88 burned. | `cmd_mag.c` `hmc5883l_configure()` |
| Barometer on the bench | BME280 at 0x76 (sold as BMP280), chip ID 0x60 | reset; calibration read from NVM; `ctrl_hum` x1; `config` = 0 (IIR filter off); `ctrl_meas` pressure x4, temperature x1, **forced mode**: one conversion per read, about 16 ms, the chip sleeps between. Compensation is Bosch's integer algorithm on the chip's trims. | `cmd_baro.c` `bmx280_configure()`, preset `-o 1` |
| Barometer alternatives | BMP280 (ID 0x58), BMP180 at 0x77 | same code paths, not run since the swap | `cmd_baro.c` |
| Rangefinder | VL53L1X at 0x29 | ST ULD `SensorInit`; distance mode **long**; timing budget 100 ms; inter-measurement 100 ms; continuous ranging. Each read waits for data-ready, takes the result, signal and ambient rates, clears the interrupt. | `cmd_tof.c` `tof_attach()`; the stream's `TOF_BUDGET_MS`/`TOF_INTER_MS` |
| Motors | 2 × DRV8833, 4 brushed motors FL RL FR RR | LEDC, 20 kHz, 11-bit duty (2048 steps), slow decay (drive/brake), **deadband 0 since 2026-10-28** (a command is its duty; it was an unmeasured 12 %); a zero command coasts. Failsafe: coast all after `CONFIG_MOTOR_FAILSAFE_MS` = 1000 ms without a thrust command, checked by a 50 ms esp_timer, armed only while a motor is driven. | `cmd_motor.c` |
| Battery | divider on GPIO1 (ADC1_CH0), 100 k + 100 k planned | **not read yet**: the ADC comes with increment 2 of the control loop. The record's `bat_mv` is 0. | `docs/control-loop-plan.md` |
| Cameras | 2 × ESP32-CAM | **not linked to the S3**: no packet carries them; the camera fields of the record are 0 and the GUI's camera panel runs on demo data. | `docs/inter-board-link-plan.md` |
| Radio | S3 as access point, WPA2, channel 1 | UDP 7200 packets, UDP 7201 console; link timeout 500 ms; telemetry 20 Hz; esp_timer task dispatch (ISR dispatch stalls Wi-Fi's timers on ESP-IDF v5.5.x, `docs/wifi-drop-report.md`) | `net_wifi.c`, `net_link.c` |

## 3. Rates, layer by layer

How often each layer produces a value, and what a consumer sees. "Repeat" means the consumer
sees the same sample again because its own rate is higher than the producer's.

| Source | Inside the chip | Read by the firmware (`sensor_stream.c`) | Control-loop record (`ctl_task.c` → `capture.c`) | SENSORS packet (`net_link.c`) | Window (`station.py` → `panels.py`) |
|---|---|---|---|---|---|
| IMU accel, gyro | 1 kHz register rate behind a 21 Hz filter (today; 260 Hz before) | **50 Hz** (today; 20 Hz before): one 14-byte burst per cycle, every read is a new sample | 50 Hz, `fresh.IMU` set on every tick that saw a new sample (before today: 2-3 records per sample) | at most 20/s: the newest sample at each telemetry tick | every packet, in arrival order (today; before: the newest at each 10 Hz tick) |
| Magnetometer (QMC5883L) | 50 Hz ODR | polled 50 Hz, a sample only when DRDY: about one per cycle; overflow samples dropped | 50 Hz with `fresh.MAG` per new sample | as above | as above |
| Magnetometer (HMC5883L, GY-88 only) | 15 Hz | polled 50 Hz, new every third or fourth cycle | `fresh.MAG` about 15/s | as above | as above |
| Barometer (BME280) | one forced conversion per read, about 16 ms | **4 Hz** (every 250 ms; the conversion blocks the stream task for 16 ms) | `fresh.BARO` 4/s, pressure 0.1 Pa, temperature 0.01 °C, humidity 0.01 % | as above, the last reading repeated | as above |
| Rangefinder (VL53L1X) | 100 ms budget, 100 ms period: 10 Hz | polled 50 Hz, new every fifth cycle | `fresh.TOF` 10/s | as above, with ST's status byte | as above |
| Motors (readback) | PWM 20 kHz | the driver holds the last command | 50 Hz, the driver's readback ×32767 | TELEMETRY 20 Hz `motor_cmd[4]` | 10 Hz tick |
| Loop timing | | | `exec_us` per record, overrun flag | | `ctl` on the console |

Measured on the bench (2026-10-27/28, `docs/control-loop-plan.md` §6): the loop's period 19.98 to
20.02 ms with 0 missed ticks over 41,700 ticks with the ground station connected; a 60 s capture
over the air, 3000 records, step 19.70/20.00/20.30 ms with σ 0.016 ms and 0 gaps; execution
4-40 µs per tick with the middle stages empty. The stream at 50 Hz has **not** been measured
yet: its cycle has to fit the 20 ms period with the barometer's 16 ms conversion inside every
thirteenth cycle (section 7).

Two things about the SENSORS packet are worth knowing when reading the live plots:

- The stream and the link tick both run at their own rates, unsynchronised, so the interval
  between packets jitters by up to one period, and when the stream is faster than the link
  only the newest sample travels. The packet's `t_us` is the sample time on the blimp; the
  panels place samples by arrival time on the laptop, not by `t_us`.
- A sensor's `valid` bit stays set for 1 s after its last good reading (`STALE_MS`), so a chip
  that stops answering shows its last value, as valid, for up to a second before the panel
  shows it stale.

The capture does not have either problem: every tick is a record, with the blimp's clock and
the fresh bits, which is why a capture is the thing to analyse and the live plots are the
thing to glance at.

## 4. The data path

### Readings, blimp to window

1. **Sensor stream task** (core 0, priority 3, 50 Hz): reads the IMU, the magnetometer (if
   DRDY) and the ToF (if data-ready) every cycle, the barometer every 250 ms; publishes the
   SENSORS payload (floats, units) and the raw record (counts) under one spinlock.
2. **Control task** (core 1, priority 20, 50 Hz from an esp_timer): peeks the raw record,
   marks each source fresh if its timestamp moved, and writes one 72-byte `log_record_t` into
   the capture buffer when a capture is running. 4-40 µs.
3. **Link task** (core 0, priority 4): every 50 ms sends TELEMETRY (52 B) and, if the stream
   has a newer sample than the last one sent, SENSORS (76 B) to the last host that sent a valid
   packet. Also answers LOG_CMD with LOG_INFO or LOG_CHUNK.
4. **Wi-Fi**: UDP, no acknowledgement; a lost packet is one missing sample.
5. **`blimp_link.py` receive thread** (laptop): validates magic, version, size and CRC-32;
   decodes; calls the bridge's packet callback. Heartbeats go out at 10 Hz so the blimp keeps
   answering.
6. **Station** (`station.py`): the callback emits the `sen` signal per packet (today; the
   10 Hz tick used to sample the newest) and the `tel` signal every 100 ms with the link state;
   Qt queues both onto the GUI thread.
7. **Panels**: the sensors panel appends to 10 s deques and repaints at 30 fps; roll and pitch
   come from the accelerometer assuming the chip's axes are the body's; the compass shows the
   packet's heading (atan2 of the magnetometer's y and x, chip flat, X forward).

### Commands, window to blimp and back

1. A slider moves; the motors panel sends the setpoint to the station **10 times a second**
   while "Motors enabled" is on (`MOTOR_SEND_MS`).
2. The bridge repeats MOTOR_SET (28 B) at **20 Hz** while the setpoint is younger than the
   **0.5 s deadman**; if the window stops refreshing it, the bridge sends MOTOR_STOP itself.
3. `net_link.c` applies the four commands to the driver and feeds the motor failsafe. Its own
   **500 ms link timeout** coasts motors commanded over the air if no packet of any kind
   arrives; the driver's **1000 ms failsafe** coasts them if no thrust command arrives while
   a motor is driven.
4. The driver maps the command to duty (1:1 since the deadband is 0) at 20 kHz.
5. TELEMETRY carries the driver's readback at 20 Hz; the panel's bars draw the readback, never
   the slider, so a command that did not arrive is visible.

### The capture, for a minute of data

Start (console or LOG_CMD) → the control task writes a record per tick into PSRAM (400 s
fit; a longer request is refused, not shortened) → LOG_INFO every 0.5 s drives the progress
bar as records ÷ requested, so at half the time the bar is at half → Download: the host asks
for windows of 8 chunks of 16 records, asks again for the ones that did not arrive, writes the
`.bin` and a `.json`, decodes and plots. 3000 records are 188 chunks, about two seconds.

## 5. Inconsistencies found, and what changed today

| # | What a reader sees | Cause | Change (2026-10-28) | Verified |
|---|---|---|---|---|
| 1 | Noisy accelerometer and gyro at rest, worse with motors running | The MPU-6050's low-pass filter was never configured: 260 Hz bandwidth, sampled at 20-50 Hz, so vibration folds into the band of interest | `cmd_imu.c`: `DLPF_CFG` 4 (21 Hz) and `SMPLRT_DIV` 0 written and read back at every attach; the stream's attach line now says "DLPF 21 Hz" | builds; not run on hardware. Expect quieter rest readings and a smaller difference between the capture and the console's `imu -n` |
| 2 | Steps in the capture's IMU traces: the same value two or three records in a row | The stream sampled at 20 Hz while the loop records at 50 Hz | `CONFIG_SENSOR_STREAM_HZ` default 50 | builds; **not run at this rate**; check `sensors` shows 0 errors and the capture's fresh rate for the IMU reads about 50/s |
| 3 | The live plots show fewer points than the stream sends, at uneven spacing | The station took the newest packet at each 10 Hz tick, dropping half of a 20 Hz stream | `station.py`: the `sen` signal is emitted per packet from the receive callback | offscreen test: every packet of the mock reaches the window |
| 4 | "Captures cannot be longer than 120 s", and the progress bar fills before the time asked for | `CONFIG_CAPTURE_MAX_SECONDS` was 120 and `capture_start()` shortened longer requests silently; the window's length box allowed up to 600 | buffer default 400 s (1.4 MB of the 1594 kB free); the firmware refuses a longer request (`LOG_INFO` result 5) and takes the largest free PSRAM block if the default does not fit; the window reads `max_seconds` from the blimp when the Bench page opens and bounds the box; the bar is records ÷ requested, so it tracks the requested time | mock and offscreen test: a 500 s request is refused, the box reads "up to 400 s" |
| 5 | Motors far faster than the sliders said | A 12 % deadband mapped the smallest command to 12.3 % duty | deadband default 0 (Nerio, 6a91878) | reported by the operator |

Also found, not changed: the HMC5883L path inherits the chip's gain instead of writing it
(only matters if a GY-88 returns); the SENSORS packet timing notes of section 3; the I2C bus
runs every device at 100 kHz, which costs about 1.6 ms per IMU burst and will matter when the
loop owns the bus (400 kHz is supported by all four chips).

## 6. Open questions, calibration rather than configuration

- **QMC5883L scale.** It reads a field magnitude of about 14.5 µT where the HMC5883L on the
  GY-88 read 26.5 µT and Bogotá's field is about 30 µT. The driver's 3000 LSB/G for the ±8 G
  range is the datasheet's figure, so either the part is not what its ID says or the local
  field at the bench is distorted. A capture while the gondola is turned through every
  orientation gives the data for an ellipsoid fit, which settles both the scale and the offset.
- **Mounting orientation.** Roll, pitch and heading assume chip axes = body axes (x forward,
  y left, z up). The sign tests of increment 3 (nose up gives a positive pitch rate, and so on)
  decide the rotation; until then the attitude numbers in the window are chip-frame numbers.
- **Barometer altitude.** `baro_alt_m` is relative to the first reading since boot, taken a
  few seconds after power-up; the standard-atmosphere altitude (`baro_alt_std_m`, Nerio's
  addition) is absolute and only as good as 1013.25 hPa. Neither is a height above the floor.
- **ToF reach.** Long mode reads about 5 % short at budgets below 100 ms (bench, dark room,
  1.57 m); whether it reaches 2.25 m under room ambient light is still to be measured.

## 7. What to check on the bench with this build

1. Boot log: `sensors` attaches the IMU with "DLPF 21 Hz"; `capture` reports a 400 s buffer in
   PSRAM (or says what it got instead).
2. `sensors` after a minute: 50 Hz, 0 errors on every chip; if the barometer's cycle makes the
   stream miss cycles, `sensors` shows it as errors or stale readings and `CONFIG_SENSOR_STREAM_HZ`
   goes back to 20 until increment 2 puts the barometer in normal mode.
3. `imu -n 10` at rest against a 10 s capture at rest: both should read the same mean and a
   smaller spread than before the filter.
4. Bench page: with the link up, the length box reads "up to 400 s"; a 120 s capture shows
   50 % at 60 s and 100 % at 120 s; a request above 400 s is refused with a message.
5. The live accelerometer plot now moves at the packet rate, up to 20 points per second.

## 8. Sources

`firmware/projects/i2c_tools/main/cmd_imu.c`, `cmd_mag.c`, `cmd_baro.c`, `cmd_tof.c`,
`cmd_motor.c`, `sensor_stream.c`, `ctl_task.c`, `capture.c`, `net_link.c`, `Kconfig.projbuild`,
`sdkconfig.defaults*`; `software/blimp_link.py`, `software/ground_station/station.py`,
`blimp_bridge.py`, `panels.py`, `bench.py`; `docs/control-loop-plan.md` §6 and
`docs/wifi-drop-report.md` for the measured figures; the MPU-6050 register map (CONFIG,
SMPLRT_DIV), the QMC5883L and BME280 datasheets for the register meanings.
