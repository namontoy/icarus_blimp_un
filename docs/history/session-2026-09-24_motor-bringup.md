# Session Report: ESP-Drone Motor Bring-up over Wi-Fi

- **Repository:** `~/github/esp-drone` (ESP-IDF port of Crazyflie firmware 2021.01), host client `~/github/crazyflie-clients-python` (cfclient + cflib 0.1.26 in `.venv`)
- **Session dates:** 2026-09-24 to 2026-09-25 (report written 2026-10-13; the file name `oct01` was requested)
- **Hardware under test:** ESP32-S3 board built as `TARGET_ESP32_S2_DRONE_V1_2`, MPU6050 IMU, one brushed motor connected (on M3)

## 0. Scope and Coverage

| Program in `test_scripts/` | Covered in this session? | Notes |
|---|---|---|
| `motor_diag.py` | **Yes.** Written in this session, then moved into the repo and extended with `--duty` | The only program this report describes in detail |
| `drone_gui.py` | **No** | First appears in commit `8138c33` (file dated 2026-09-29), after this session. This session never read or discussed it, so this report does not describe it. |
| `drone_dynamics.py` | **No** | Same as `drone_gui.py`: added later in `8138c33`, never read here |

So sections 1–5 cover `motor_diag.py` and the stock CRTP-over-UDP link it uses (shared with cfclient). Any custom protocol, GUI logic or dynamics model inside `drone_gui.py` / `drone_dynamics.py` is **not** covered. See their own `test_scripts/README.md` (also added after this session).

---

## 1. Wi-Fi Protocol (Host ↔ MCU) as Used by `motor_diag.py`

This is the stock ESP-Drone link: Crazyflie **CRTP** packets carried in **UDP** datagrams. `motor_diag.py` does not define a protocol of its own; it uses cflib's UDP driver.

### 1.1 Network Setup

| Item | Value | Source |
|---|---|---|
| Wi-Fi mode | ESP32 runs a **SoftAP**; the PC joins it as a station | `drivers/general/wifi/wifi_esp32.c` |
| SSID | `<CONFIG_WIFI_BASE_SSID>_<12 hex digits of MAC>` | `wifi_esp32.c:280` |
| Password | Set in menuconfig (`ESPDrone Config`). **Deliberately not reproduced here.** | `main/Kconfig.projbuild` |
| Drone IP / gateway | `192.168.43.42` | `wifi_esp32.c:308-310` |
| Transport | UDP, server bound on the drone at **port 2390** | `wifi_esp32.c:29` |
| Client URI | `udp://192.168.43.42:2390` | `motor_diag.py`, cflib `udpdriver.py` |
| Reply addressing | Drone sends every reply to the **source address of the last datagram it received** (`source_addr`), so only one client at a time works in practice | `wifi_esp32.c:186` |

Practical consequence found in the session: the PC cannot be on the drone's AP and on the internet at once (single Wi-Fi interface). Tests had to be run offline and the output saved to a file (`test_scripts/diag_out.txt`).

### 1.2 Datagram Framing

```
UDP payload = [ CRTP header (1 B) | CRTP data (0..30 B) | checksum (1 B) ]
checksum    = 8-bit sum (mod 256) of all preceding bytes (header + data)
```

- Receive buffer is 64 B, but packets longer than `WIFI_RX_TX_PACKET_SIZE` (32 B: 1 + 30 + 1) are dropped.
- Datagrams with a bad checksum are dropped silently (only a debug print).
- CRTP header: `(port & 0x0F) << 4 | (channel & 0x0F)` (`crtp.h:35`). Bits 2–3 are the link bits used by real Crazyflie radios; not examined in this session.
- `CRTP_MAX_DATA_SIZE = 30`.

### 1.3 CRTP Ports Relevant to This Work

| Port | Value | Used by `motor_diag.py`? |
|---|---|---|
| `CRTP_PORT_PARAM` | 0x02 | Yes: read and write parameters |
| `CRTP_PORT_SETPOINT` | 0x03 | No (cfclient Flight tab uses it) |
| `CRTP_PORT_LOG` | 0x05 | Yes: log blocks through cflib `LogConfig` |
| `CRTP_PORT_SETPOINT_GENERIC` | 0x07 | No |
| `CRTP_PORT_PLATFORM` | 0x0D | Indirectly: cflib asks for the protocol version at connect |
| `CRTP_PORT_LINK` | 0x0F | Indirectly: link-level (cflib) |

### 1.4 Parameter Sub-protocol (Port 0x02)

The firmware reports `PROTOCOL_VERSION 4` (`components/config/include/config.h:46`). cflib switches to the **V2** (16-bit ID) parameter format when the version is ≥ 4, so V2 is what runs on this link.

Channels: `TOC=0`, `READ=1`, `WRITE=2`, `MISC=3` (`param.c:61-64`).

| Operation | Host → MCU (data bytes) | MCU → Host |
|---|---|---|
| TOC info (V2) | ch 0: `[0x03]` (`CMD_GET_INFO_V2`) | `[0x03, count u16 LE, crc32 u32 LE]`. This also sets `useV2 = true` on the firmware side. |
| TOC item (V2) | ch 0: `[0x02, id u16 LE]` (`CMD_GET_ITEM_V2`) | type byte + group name + parameter name (NUL-terminated) |
| Read (V2) | ch 1: `[id u16 LE]` | `[id u16 LE, status u8 (0 = OK), value…]` |
| Write (V2) | ch 2: `[id u16 LE, value (little-endian, size of the type)]` | **Echo of the same packet.** The echo is the acknowledgement, and cflib updates its value cache from it. |
| Set by name | ch 3: `[0x00, "group\0", "name\0", type, value…]` | Same packet with the type byte replaced by an error code |
| MISC commands 1–6 | ch 3: `VALUE_UPDATED`, `GET_EXTENDED_TYPE`, `PERSISTENT_STORE/GET_STATE/CLEAR`, `GET_DEFAULT_VALUE` | **Not implemented. Silently ignored, no reply** (`param.c:182`). See section 4.1. |

Parameters used in this session:

| Parameter | Type | Meaning |
|---|---|---|
| `motorPowerSet.enable` | uint8 | 1 = bypass the mixer and drive motors from `m1..m4` |
| `motorPowerSet.m1..m4` | uint16 | Raw thrust per motor, 0–65535 |
| `stabilizer.stop` | uint8 | Emergency stop (latched; see section 4) |
| `system.selftestPassed` | int8, read-only | 1 = boot self-test passed and the flight loop was started |
| `system.forceArm` | int8 | Forces `systemIsArmed()` to true |
| `imu_tests.mpu6050` | uint8, read-only | MPU6050 self-test result |
| `health.startPropTest` | uint8 | Re-runs the propeller test (drives each motor at **full** power briefly; not used) |

### 1.5 Log Sub-protocol (Port 0x05)

Used through cflib `LogConfig` + `SyncLogger`. The byte-level format of log block packets was **not examined** in this session. Variables used:

| Log variable | Type | Meaning |
|---|---|---|
| `sys.armed`, `sys.canfly` | int8 | Arming state |
| `acc.z` | float | Accelerometer Z in g (+1.0 = level and upright) |
| `stabilizer.roll` | float | Estimated roll in degrees |
| `stabilizer.rtStab` | float | Measured stabilizer loop rate in Hz (a by-function log variable) |
| `pwm.m1_pwm..m4_pwm` | uint32 | Value last passed to `motorsSetRatio()`, i.e. what actually went to the PWM driver |
| `motor.m1..m4` | uint32 | **Mixer** output. It does **not** reflect the `motorPowerSet` override, so it is misleading for this test. |

### 1.6 Rates

| Item | Rate |
|---|---|
| `motor_diag.py` log blocks | 100 ms period (10 Hz), 5 samples per block |
| Stabilizer loop (where `motorPowerSet` is applied) | 1000 Hz (`rtStab` measured ≈ 999–1000 Hz) |
| Motor PWM | LEDC, **15 kHz**, **8-bit** resolution, low-speed mode, timer 0, channels 4–7 |
| Parameter writes | No fixed rate. cflib sends them one at a time and waits for each acknowledgement. |

---

## 2. `motor_diag.py`: How It Works and How to Run It

**Location:** `test_scripts/motor_diag.py`

### 2.1 What It Does

1. Connects to `udp://192.168.43.42:2390` with cflib, caching the parameter and log tables in `test_scripts/cfcache/`.
2. Prints the parameters that decide whether motors can run: self-test, IMU test, `stabilizer.stop`, `forceArm`, and all `motorPowerSet.*`.
3. Reads 5 log samples at 10 Hz: `sys.armed`, `sys.canfly`, `acc.z`, `stabilizer.roll`, `stabilizer.rtStab`. This shows whether the flight loop is running and whether the board is upright.
4. With `--spin N`: writes `motorPowerSet.mN`, then `motorPowerSet.enable=1`. After 0.5 s it logs `pwm.mN_pwm` (5 samples) to confirm the value reached the driver. After 3 s in total it writes `mN=0` and `enable=0`.

Without `--spin` the script is read-only.

### 2.2 Command Line

```
motor_diag.py [--spin {1,2,3,4}] [--value VALUE | --duty DUTY]
  --spin N    motor to test (M1..M4). Omit for a read-only check.
  --value V   raw motorPowerSet value 0-65535 (default 10000)
  --duty P    duty cycle in percent 0-100; converted as round(P/100*65535)
```

`--value` and `--duty` cannot be used together, and out-of-range values are rejected. The spin banner prints the duty the firmware actually applies, `(value >> 8)/256`, for example `--duty 25` → `64/255 ≈ 25.0%`.

### 2.3 How to Run It

```bash
# 1. Remove the propeller. 2. Join the drone's AP. 3. Close cfclient (only one client works at a time).
cd ~/github/esp-drone
~/github/crazyflie-clients-python/.venv/bin/python test_scripts/motor_diag.py               # read-only
~/github/crazyflie-clients-python/.venv/bin/python test_scripts/motor_diag.py --spin 3 --duty 25
```

What healthy output looks like (verified in the session):
- `selftestPassed=1`, `mpu6050=1`, `stop=0`, `armed=1`
- `acc.z ≈ +1.0`, `roll ≈ 0`, `rtStab ≈ 1000`
- During a spin, `pwm.mN_pwm` equals the requested value

### 2.4 Dependencies

- Python 3.10 (the cfclient virtualenv)
- **cflib 0.1.26**, from `~/github/crazyflie-clients-python/.venv`. It uses `cflib.crtp`, `Crazyflie`, `SyncCrazyflie`, `SyncLogger` and `LogConfig`.
- Standard library only otherwise (`argparse`, `os`, `time`)
- Firmware with `power_distribution_stock.c` (for `motorPowerSet`) and `motors.c` (for the `pwm` log group) compiled in. Both are in the stock build.

---

## 3. Design Decisions and Why

| Decision | Reason |
|---|---|
| Drive the motor through the existing `motorPowerSet` parameter override instead of new firmware code | Already in the firmware (`power_distribution_stock.c:135`), so no rebuild was needed. It goes through the real path `powerDistribution()` → `motorsSetRatio()` → LEDC, so the same safety gates apply (self-test, arming, `stabilizer.stop`, tumble detection). The user had asked for no project modifications at the time. |
| Use a cflib script instead of the cfclient Parameters tab | The Parameters tab locks up on this firmware (section 4.1). Direct `param.set_value()` calls never send the MISC request that causes the lockup. |
| Read-only by default; spinning needs an explicit `--spin` | Safety: running the diagnostic should never move a motor by accident. |
| Confirm with `pwm.mN_pwm`, not `motor.mN` | `motor.mN` is the mixer output and ignores the override. `pwm.*` is set inside `motorsSetRatio()`, so it shows what was really written. |
| Log `acc.z`, `stabilizer.roll`, `rtStab` alongside the parameters | Shows separately whether the flight loop runs (`rtStab`, roll changing) and whether the board is upright (`acc.z`). That is what exposed the tumble-detection cutoff. |
| Run fixed 3-second spins, then automatically write 0 | Limits how long the motor runs and how hot it gets without a propeller. |
| `--duty` in percent, plus printing the real 8-bit duty | Users think in duty percent. The firmware keeps only the top 8 bits of the 16-bit value, so printing `bits/255` shows the true quantization (about 0.39% per step, maximum 99.6%). |
| Cache folder next to the script (`test_scripts/cfcache`) | The first copy lived in the session scratchpad, which was later wiped. A folder next to the script survives and avoids writing into the cfclient checkout. |

---

## 4. Approaches Tried and Discarded; Known Issues

### 4.1 Discarded: cfclient Parameters Tab (GUI) for Motor Control

- **Symptom:** pressing *Set* changed nothing, not even the value shown in the GUI.
- **Wrong first hypothesis (retracted):** the table not redrawing (`ParamTab.py:86` emits `dataChanged` with invalid indexes). That redraw issue does exist, but it was not the cause.
- **Root cause:** selecting a row makes cfclient call `get_default_value()` (`ParamTab.py:411`), which sends MISC command 6 (`GET_DEFAULT_VALUE`). The firmware ignores MISC commands other than 0, so no reply ever comes. cflib's `_ParamUpdater` sends requests one at a time through a queue and holds a lock until the matching reply arrives, so the queue stalls for the rest of the connection. Every later *Set* waits in the queue forever. "Default value: -" in the GUI is the visible sign of this.
- **Fix options identified (not done):** (a) firmware: reply with `ENOENT` to MISC commands 2–6 (cflib already handles `ENOENT`); (b) use a cflib version that predates default and persistent parameters, or Espressif's forked cfclient. The exact matching version was not determined.

### 4.2 Discarded: Scratchpad Location for the Script

The first version lived in the session's temporary folder and was lost when it was cleared. It was recreated in `test_scripts/` at the user's request.

### 4.3 Diagnosed Blocker: Tumble Detection Cut the Motor

- **First run:** the write reached the drone, but `pwm.mN_pwm` read 0 and `stabilizer.stop` changed from 0 to 1.
- **Data:** `acc.z ≈ −0.85 g`, `roll ≈ 147°`, so the IMU read the board as upside down.
- **Mechanism:** `sitaw.c:118-127` treats a total motor PWM above 1000 as flying. It then checks `acc.z < −0.5` (`SITAW_TU_ACC_THRESHOLD`) on 30 consecutive samples, about 30 ms. Once triggered, `stabilizerSetEmergencyStop()` runs and `powerStop()` zeroes all motors. The trigger stays **latched** (`trigger.released` is only re-checked while motors spin), so `stabilizer.stop` cannot be cleared without a reboot.
- **Fix used in the session:** reboot and place the board upright. After that, `acc.z ≈ +1.0`, and both M3 and M4 showed `pwm = 10000`.

### 4.4 Known Issues (Still Open as of the End of the Session)

1. **No firmware timeout on `motorPowerSet`.** The override bypasses the commander's link-loss watchdog. If the script dies mid-spin (Ctrl+C, Wi-Fi drop, exception), the motor **keeps running** at the last value until reboot, reconnection, or battery removal.
2. **`motor_diag.py` has no `try/finally` stop.** Same consequence as item 1 on Ctrl+C or an exception. Adding a guaranteed stop and an optional `--ramp` was offered but **not implemented** in this session.
3. **Values below 256 give 0% duty** (8-bit quantization).
4. **Misleading firmware comment:** `system.c` says the self-test can be forced from cfclient, but `system.selftestPassed` is read-only, so it cannot.
5. **Single-client UDP:** replies go to the last sender, so cfclient and the script must not run at the same time.
6. **Invalid local edit noticed** in `crtp_localization_service.c` (used the undefined names `values` and `setpoint`). It only built because the file is not in `SRCS`. Commit `6d7f435` ("crtp: fix quaternion decompress in extPosePackedHandler"), made after this session, appears to address it. This session did not verify that.

---

## 5. Ideas Worth Reusing in Another ESP32 Project

This session did **no sensor-fusion work**. The estimator was only read (`acc.z`, `stabilizer.roll`) as a sanity check of orientation. The reusable ideas below are about drivers, protocols and safety.

**Link and protocol**
- **UDP + 1-byte additive checksum + small fixed-size packets** (≤ 32 B) is enough for a single-client bench link on a SoftAP. Add a sequence number if you need loss detection; this framing has none.
- **The echo is the acknowledgement:** having the MCU echo each parameter write lets the host confirm delivery and update its cache in one step.
- **Always answer unknown commands** with an explicit error (`ENOENT`/`EINVAL`) instead of ignoring them. A host that waits for each reply will otherwise stall, which is exactly the cfclient failure in 4.1.
- **Reply to the last sender's address:** simple, but it limits you to one client. Keep a client table if you need more than one.

**Motor driver (brushed, LEDC)**
- LEDC at 15 kHz (above hearing range) with 8-bit resolution on 4 channels sharing one timer is a solid brushed-motor setup.
- Keep a **16-bit thrust interface** and shift down to the hardware resolution in one place (`motorsSetRatio`). Higher layers then stay independent of the PWM resolution.
- Record the **value actually written** in a loggable variable (like `motor_ratios[]`) next to the mixer output, so you can tell control problems from driver problems.
- A short **boot self-test pulse per motor** (20% for 50 ms in `motorsTest()`) is a cheap wiring check.

**Safety architecture**
- Gate actuator output behind independent checks evaluated **every loop**: self-test passed, calibrated, armed, not emergency-stopped.
- **Make tumble detection depend on motor output** ("only judge orientation while motors are commanding thrust") and **latch** it. This worked as designed here, but it also blocks inverted bench tests. Expose a documented bench-mode switch rather than editing the header.
- **Any manual override needs its own watchdog** (for example, auto-disable when no write arrives within N ms). This is the main gap identified in 4.4.
- Start the control loop only after **gyro bias calibration** completes, which requires the board to be still.

---

## 6. Work in This Session Not About the Three Programs

**Firmware changes:** none. The user asked for no project modifications during the investigation. The only file this session created in the repo is `test_scripts/motor_diag.py` (and its `cfcache/` folder created at run time).

**Firmware analysis and explanations given**
- Traced the motor command path: `stabilizerTask` → `controller` → `powerDistribution()` / `powerStop()` → `motorsSetRatio()` → LEDC.
- Explained the boot gating: `systemTask` self-tests → `systemStart()` → `systemWaitStart()` in the stabilizer → gyro calibration → "Ready to fly." → loop.
- Explained arming: armed by default because `START_DISARMED` is undefined; `forceArm`; and the cfclient *Arm* button relying on a newer "supervisor" protocol this firmware lacks.
- Reviewed the uncommitted local diffs present at the start: buzzer-less `soundTest()`, VL53L1X and PMW3901 disabled, quaternion-decompress edits, LED changes.

**Hardware findings**
- Motor pin map with this `sdkconfig` (no pin swap for `TARGET_ESP32_S2_DRONE_V1_2`): **M1 = GPIO7, M2 = GPIO4, M3 = GPIO3, M4 = GPIO1**.
- The single connected motor is on **M3 (GPIO3)**. Confirmed by the user, and `--spin 3` works.
- IMU orientation: upright board gives `acc.z ≈ +1.0`, `roll ≈ 0` (normal mounting). The earlier −0.85 g reading came from how the board was placed.
- Stabilizer loop measured at about 1000 Hz. Self-test and MPU6050 test both pass.
- Safety guidance given for no-load runs at high duty: brush and bushing wear and vibration rather than overheating, ramp up gradually, limit run time, clamp the motor, prefer a current-limited bench supply, know the kill switches.

**Docs:** none written in this session.

**Repository changes observed after this session (not made by it, listed for context only):** `4b983f9` (commits the local firmware edits), `8138c33` (adds `drone_gui.py`, `drone_dynamics.py`, `motor_diag.py`, `CLAUDE.md`, `.gitignore`), `6d7f435` (quaternion fix in localization service), `f31edd1` (sdkconfig snapshots), `0e075f3` (`test_scripts/README.md`), `560e17b` (**disables tumble detection** `SITAW_TU_ENABLED`). Note on the last one: with tumble detection off, an inverted or crashed drone will keep its motors running. Treat that as a bench-only setting.
