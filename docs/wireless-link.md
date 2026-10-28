# Wireless link: the S3 as a Wi-Fi access point, UDP for commands, telemetry and the console

**Date:** 2026-10-21
**Branch:** `s3-wireless-link`
**Status:** builds on ESP-IDF v5.5.5; **not yet run on hardware**. Section 9 lists what the
first bench session has to prove.

## 1. Why

The bench firmware in `firmware/projects/i2c_tools/` is driven over a USB cable. A blimp
cannot fly with a cable, and even on the bench the cable ties the airframe to the desk. This
adds a radio link that carries three things: the existing console (every registered command,
unchanged), a small binary packet protocol for a ground station to drive the motors and read
telemetry at a fixed rate, and the safety timers that make a lost link harmless.

## 2. Decision: one link, Wi-Fi access point on the S3

The S3 is its own access point. The laptop joins it; nothing else is involved.

| | Wi-Fi AP on the S3, UDP | ESP-NOW | BLE |
|---|---|---|---|
| Hardware on the laptop | none | a second ESP32 on USB plus bridge firmware | none |
| Latency, quiet channel | ~2–10 ms | ~1–6 ms | 10–30 ms |
| Recovery after a dropout | re-associate and DHCP, 1–5 s | instant, every frame is independent | 1–3 s |
| Radio current on the S3 | listening ~100 mA, bursts ~300 mA | about the same | lower |
| Crowded hall | shares the band, MAC retries | shares the band, MAC retries | shares the band |
| Tooling | sockets, any OS | two firmwares and a serial framing | awkward from Python |

The deciding fact is hardware: the lab has one S3-Zero with a working USB port and the XIAO is
not available, so there is no board to make an ESP-NOW dongle from. ESP-NOW's real advantage
is instant recovery, and the motor failsafe already covers that case. The packets are
transport-agnostic (magic, version, sequence, CRC-32, fixed size, under 250 bytes), so moving
them onto ESP-NOW later is a change to `net_link.c` only.

What matters more than the radio: keep the link non-critical. Guidance and control belong on
the S3; the ground link is for arm, stop, telemetry and tuning. Then a congested hall degrades
the log, not the run. The DroneUN flight firmware's command packet currently assumes the laptop
closes the loop; that assumption is the one to revisit, not the radio.

Latency and current figures above are reasoned from datasheets and the DroneUN notes, not
measured on this board.

## 3. What was built

New files in `firmware/projects/i2c_tools/main/`:

| File | Role |
|---|---|
| `link_protocol.h` | Wire format: packed structs, sizes asserted, CRC-32 (zlib variant). No ESP-IDF dependency |
| `net_wifi.c/.h` | NVS, netif, Wi-Fi driver, the access point; `wifi` command (`--start`, `--stop`, status with stations and RSSI) |
| `net_link.c/.h` | UDP packet task: commands in, telemetry out at a fixed rate, link timeout; `link` command |
| `net_console.c/.h` | The console over UDP: one command line per datagram, output back as datagrams, closed by a 0x04 byte |
| `sensor_stream.c/.h` | Added 2026-10-24: samples every sensor at a fixed rate for the `SENSORS` packet; `sensors` command; the pause/resume lease that console sensor commands take |

Changed: `cmd_motor.h/.c` gain `motors_failsafe_state()` (off / idle / armed / tripped) for
telemetry; `app_main` registers the two commands and starts the link after every other
command is registered; `main/CMakeLists.txt`; `Kconfig.projbuild` gets a "Wireless link" menu;
`partitions_example.csv` gives the app 2 MB (section 9). For the sensor stream, `cmd_imu.c`,
`cmd_mag.c`, `cmd_baro.c` and `cmd_tof.c` were split into an attach/read/detach driver and a
console command that loops over it, with the same configuration, scaling and messages as
before, and `i2cconfig` pauses the stream while it recreates the bus.

Host side, standard library only: `software/blimp_link.py` (the ground tool) and
`software/mock_blimp.py` (the firmware's link behaviour on localhost, for development without
hardware). `scripts/idf.bat` builds on Windows (section 10).

### Settings (`idf.py menuconfig` → "Wireless link")

| Setting | Default | Note |
|---|---|---|
| `NET_WIFI_AT_BOOT` | y | Off: radio stays down until `wifi --start` on USB |
| `NET_WIFI_SSID_PREFIX` | `ICARUS` | SSID is `ICARUS-` plus the last three MAC bytes, e.g. `ICARUS-A1B2C3` |
| `NET_WIFI_PASSWORD` | `icarusblimp` | WPA2, 8–63 characters; empty = open (bench only) |
| `NET_WIFI_CHANNEL` | 1 | Fixed; survey the venue before competition day |
| `NET_WIFI_MAX_STA` | 2 | |
| `NET_LINK_PORT` | 7200 | packets |
| `NET_CONSOLE_PORT` | 7201 | text console |
| `NET_LINK_TIMEOUT_MS` | 500 | section 5 |
| `NET_LINK_TELEMETRY_HZ` | 20 | |

The access point's address is ESP-IDF's default, `192.168.4.1`; the laptop gets `192.168.4.2`
from the S3's DHCP server. The configuration is rebuilt from Kconfig at every boot and never
stored in NVS, so a reflash with a new key is the whole story.

## 4. Protocol

`link_protocol.h` is the source of truth; `blimp_link.py` mirrors it with `struct` and asserts
the same sizes. Little-endian, one packet per datagram, every packet:

```
link_header_t { u16 magic = 0xB1A7; u8 version = 1; u8 type; u16 seq; u16 reserved }  (8 B)
payload
u32 crc32   over the whole struct with this field zeroed; zlib/PNG CRC-32
```

| Type | Direction | Payload | Size |
|---|---|---|---|
| `0x01 HEARTBEAT` | host → blimp | none | 12 B |
| `0x02 MOTOR_SET` | host → blimp | `f32 cmd[4]`, FL RL FR RR (`motor_id_t` order), −1..+1, 0 coasts | 28 B |
| `0x03 MOTOR_STOP` | host → blimp | `u8 brake` | 16 B |
| `0x81 TELEMETRY` | blimp → host | `u32 t_us`, `f32 motor_cmd[4]`, `u8 failsafe`, `u8 link_alive`, `u8 link_motors`, `u8 stations`, `u32 rx_ok`, `u32 rx_bad`, `u32 free_heap`, `u16 link_age_ms` | 52 B |
| `0x82 SENSORS` | blimp → host | `u32 t_us`, `f32 acc[3]` g, `f32 gyro[3]` dps, `f32 imu_temp_c`, `f32 mag[3]` gauss, `f32 heading_deg`, `f32 pressure_pa`, `f32 baro_temp_c`, `f32 baro_alt_m` (relative to the first reading), `u16 tof_mm`, `u8 tof_status`, `u8 valid` (IMU 1, MAG 2, BARO 4, TOF 8) | 76 B |
| `0x04 LOG_CMD` | host → blimp | `u8 cmd` (0 info, 1 start, 2 stop, 3 clear, 4 get), `u8 count` (get: chunks, up to 8), `u16 seconds` (start), `u32 first` (get: record index) | 20 B |
| `0x83 LOG_INFO` | blimp → host | `u8 state` (0 idle, 1 running, 2 done), `u8 result` (0 ok, 1 no buffer, 2 busy, 3 range, 4 bad command), `u8 record_size` (72), `u8 format` (1), `u16 capture_id`, `u16 period_us`, `u32 count`, `u32 capacity`, `u32 wanted`, `u32 t_start_us`, `u16 imu_accel_g`, `u16 imu_gyro_dps`, `f32 mag_lsb_per_gauss`, `u8 mag_chip`, `u8 baro_chip`, `u16 max_seconds`, `char fw[16]` | 64 B |
| `0x84 LOG_CHUNK` | blimp → host | `u16 capture_id`, `u8 count` (records present), `u8 reserved`, `u32 first`, `u8 data[16 × 72]` (one `log_record_t` of `ctl_types.h` each, zero past `count`) | 1172 B |

Anything with the wrong size for its type, another magic or version, or a bad CRC is counted in
`rx_bad` and ignored. There are no acknowledgements: the telemetry's `motor_cmd` readback is
the acknowledgement of a `MOTOR_SET`, the same idea esp-drone used with its parameter echo.

The three `LOG_*` types (2026-10-27) carry the capture: the control loop's records, kept in
PSRAM on the blimp (`capture.c`, `docs/control-loop-plan.md` D7 and D8). `LOG_CMD` info, start,
stop and clear are each answered by one `LOG_INFO`; `get` by up to eight `LOG_CHUNK`s back to
back, or a `LOG_INFO` with result 3 at the end of the data. The host asks again for chunks that
did not arrive. `LOG_CHUNK` is the one packet larger than `LINK_MAX_PACKET_BYTES`; it only ever
travels blimp to host, so the firmware's receive buffer is unchanged.

The blimp replies to whoever sent the last valid packet of any type, so the first heartbeat
starts the telemetry and a reconnect is just sending again. Telemetry continues for ten
timeouts (5 s) after the host goes quiet, so a host that paused sees the motors coasted and
`link_alive = 0`, then stops until the next packet.

## 5. Safety contract

A `MOTOR_SET` is a standing command **only while it keeps arriving**. Two timers guard the
standing state, independently:

- **Link timeout** (`NET_LINK_TIMEOUT_MS`, 500 ms): if the motors were last commanded over the
  air and no valid packet of any kind arrives in the window, every motor coasts and the event
  is logged once on USB. Motors set from the USB console are not touched: a `motor --hold`
  typed at the desk is deliberate.
- **Motor failsafe** (`MOTOR_FAILSAFE_MS`, 1000 ms, unchanged): coasts if no thrust command of
  any origin arrives while a motor is driven. It catches a host that keeps sending heartbeats
  but has stopped sending motor commands, which the link timeout alone would miss.

So `blimp_link.py motor` streams `MOTOR_SET` at 20 Hz and sends `MOTOR_STOP` three times at the
end or on Ctrl+C. Its `--limit` (0.3 by default) refuses larger commands unless raised on
purpose. The firmware has no arming step: the WPA2 key is what stands between a stray phone and
the motors. **Remove the propellers** for every bench session.

## 6. The console over UDP

One datagram = one command line, run through `esp_console_run()`, the same registry as the USB
REPL. The output comes back as datagrams of up to 1400 bytes and a final datagram containing
only `0x04` (End of Transmission). Output is flushed when the buffer fills, on a newline once
20 ms have passed since the last flush, and at the end, so `tof -n 200` streams line by line
while `help` arrives in a few datagrams.

How it works: under ESP-IDF's FreeRTOS, newlib's `stdout` is the calling task's own stream.
The console task points its `stdout` and `stderr` at a `funopen()` stream that buffers into a
datagram. Two consequences:

- A log line from another task during a command (the failsafe tripping, a station leaving)
  still goes to USB, not to the UDP console. Telemetry carries the failsafe state for that
  reason.
- **One console at a time.** `esp_console_run()` splits lines in one static buffer and each
  command has one argtable, so a line from USB and one over UDP at the same instant would
  corrupt each other. Nothing enforces it; on a bench with one operator it does not need to be.

## 7. Host tool

```
python software/blimp_link.py selftest                     # codec round-trips, no hardware
python software/blimp_link.py status                       # telemetry for 5 s
python software/blimp_link.py console                      # interactive console over the air
python software/blimp_link.py console "motor --status"     # one command
python software/blimp_link.py motor 0.2 0 0 0 --seconds 2  # FL RL FR RR, props off
python software/blimp_link.py stop [--brake]
python software/blimp_link.py capture start -t 60               # record the loop's records on the blimp
python software/blimp_link.py capture download [--csv out.csv]  # fetch them: captures/capture-<date>.bin
python software/blimp_log.py captures/capture-<date>.bin        # summary, --csv, --head
```

Python 3.8+ standard library, no conda environment needed. Join the blimp's SSID from the OS
first; on Windows that is the Wi-Fi menu or `netsh wlan connect name=ICARUS-xxxxxx` once a
profile exists. The laptop loses internet while on it, as with any single Wi-Fi adapter.

Without hardware, `python software/mock_blimp.py` serves the same protocol on 127.0.0.1 with the
same timers; add `--host 127.0.0.1` to the commands above. This is how the tool was tested so
far (section 9).

## 8. Bring-up checklist

Props off. USB console open in a second window so the logs are visible.

1. Flash. Boot log should show `net_wifi: access point 'ICARUS-xxxxxx' configured`, then
   `access point ... up on channel 1, address 192.168.4.1`, `net_link: UDP link on port 7200`,
   `net_console: console over UDP on port 7201`. `wifi` on USB prints the same.
2. Join the SSID from the laptop. USB log: `station xx:xx:xx:xx:xx:xx joined`. `wifi` shows
   the station and its RSSI.
3. `python software/blimp_link.py status` prints telemetry lines at 5 per second with
   `failsafe idle`, `link alive`, `rx` counting up. `link` on USB shows the peer.
4. `python software/blimp_link.py console "motor --status"` returns the same text USB does.
   Then `console` interactive: `help`, `imu -n 3`, `tof -n 3`. Compare with USB.
5. `motor 0.15 0 0 0 --seconds 3`: FL turns, telemetry readback shows `FL +0.150`, `failsafe
   armed`, `link alive (motors)`. Stops cleanly at the end; readback returns to 0, `idle`.
6. Link-loss test: start `motor 0.15 0 0 0 --seconds 30`, then turn the laptop's Wi-Fi off.
   Within about half a second the USB log shows `net_link: link lost ... all motors coasting`
   and the motor stops. The driver's failsafe must **not** also fire (it would log
   `E failsafe:` 1 s later): coasting disarms it.
7. Failsafe test: `motor` with `--rate 0.5` (one packet every 2 s). The driver's failsafe
   trips 1 s after each packet (`E failsafe:` on USB, `failsafe TRIPPED` in telemetry), and
   the next packet re-arms it. Proves the two timers are independent.
8. Sensor rail under Wi-Fi: run `imu -n 50 -i 100` over UDP while the laptop keeps the link
   busy (`status`). Any I2C error or reset is a brownout suspect, per the power note in
   `cmd_motor.c` and `net_wifi.c`.
9. Current: logic-rail current with the access point idle, with a station associated and
   telemetry running, and during a `motor` run. Record the figures here.
10. Range: walk the laptop to the far end of the room with `status` running; note where
    `rx_bad` or gaps appear.
11. Sensor stream: `sensors` on USB lists what it found (expect the MPU-6050, the QMC5883L,
    the BME280 and the VL53L1X). With the ground station page open, the four sensor blocks
    fill within a second of the link coming up. Type `imu -n 3` in the page's console: the
    panels show the orange "paused" notice while the command runs, then resume with the same
    numbers the command printed. `link` on USB counts the `SENSORS` packets separately.

## 9. What is proven and what is not

| Claim | Status |
|---|---|
| Firmware builds on ESP-IDF v5.5.5, no warnings | ✅ built 2026-10-21 |
| Packet codec round-trips, rejects truncation, bit flips, wrong magic and version | ✅ `blimp_link.py selftest` |
| Host tool drives the protocol end to end, including both timers | ✅ against `mock_blimp.py` on localhost |
| CRC-32 implementation in C matches zlib | reasoned: same algorithm; the check value in the header is the test to run on target |
| Access point comes up, laptop associates, DHCP works | ✅ 2026-10-24: the laptop joined and the ground station page received telemetry |
| `SENSORS` packet carries the real readings (`sensor_stream.c`) | ❓ built 2026-10-24, not run on hardware; the page decodes the same layout from the mock |
| Console sensor commands and the stream share the chips by turns (`sensor_stream_pause()`) | reasoned from the `i2c_master` and ULD port sources; ❓ not run |
| Per-task `stdout` redirection captures command output | reasoned from the newlib/FreeRTOS sources; ❓ not run |
| `ESP_LOGx` printed inside a command reaches the UDP console | ❓ depends on the log sink being `vprintf` on the calling task; verify with `imu -c 0x69` (deliberate wrong address) |
| Link timeout and motor failsafe behave as in section 5 | ✅ in the mock; ❓ on hardware |
| No brownout of the sensor rail with the radio on | ❓ **never tested**; item 8 above |
| Image fits its partition | ✅ measured 2026-10-21: 929 KB (`-Og` debug build) in the factory partition, which `partitions_example.csv` grows from 1 MB (11 % free) to 2 MB (56 % free); the FAT `storage` partition moves up and reformats once |
| 2.4 GHz behaviour at the venue | ❓ unknown; channel is a Kconfig setting for that reason |
| Capture packets (`LOG_CMD`, `LOG_INFO`, `LOG_CHUNK`) and the chunked download with re-request | ✅ against `mock_blimp.py` 2026-10-27 (`blimp_link.py capture`, `blimp_log.py`, the Bench page offscreen); ❓ on hardware, where PSRAM is also unproven |

Built since the first cut: the sensor stream (`sensor_stream.c` sampling through the
attach/read/detach drivers lifted out of `cmd_imu.c`, `cmd_mag.c`, `cmd_baro.c` and `cmd_tof.c`,
sent as the `SENSORS` packet, 2026-10-24) and the ground station page
(`software/ground_station/`, design in `docs/ground-station-plan.md`, 2026-10-23). Still
planned: an arm step if the key alone turns out to be too little.

## 10. Building on Windows

The firmware was written against ESP-IDF v5.5.5, the same version as the Linux setup in
`CLAUDE.md`. On Windows, with ESP-IDF cloned to `%USERPROFILE%\esp\esp-idf-v5.5.5` and
Python 3.11 at its default per-user path:

```
scripts\idf.bat i2c_tools build
scripts\idf.bat i2c_tools -p COM5 flash monitor
```

The script clears three traps: ESP-IDF's `export.bat` bails out silently when the `MSYSTEM`
variable is set (Git Bash sets it and cmd.exe inherits it) or when `PROMPT` is undefined, and
the Windows Store Python 3.12 on the default PATH breaks ESP-IDF's virtualenv lookup. Set
`IDF_PATH` and `PY311` first if your paths differ. Note for v5.3.x: the `led_strip` component
resolves to 3.1.0, which calls an RMT function that version lacks; v5.5.5 has it.
