# ESP-Drone bench tools: merged technical report

Merged from two Claude Code sessions on the second PC (Linux Mint, `~/github/esp-drone`, fork `github.com/namontoy/esp-drone`). It documents the three Python programs in `test_scripts/` (`drone_gui.py`, `drone_dynamics.py`, `motor_diag.py`), the Wi-Fi protocol they use, and what was learned along the way. The Wi-Fi passphrase is deliberately not reproduced anywhere in this document.

---

## 0. Provenance and reading guide

| Tag | Session | Work date | Contribution |
|---|---|---|---|
| **[B]** | Report file `docs/history/session-2026-09-24_motor-bringup.md` | **2026-09-24 to 09-25** (saved at first as report-session-oct01.md, a misleading name; the work is the *earlier* of the two) | Motor bring-up over Wi-Fi. Wrote `motor_diag.py`. Diagnosed the tumble-detection cutoff and the cfclient Parameters-tab stall. |
| **[A]** | Report file `docs/history/session-2026-09-29_gui-and-dynamics.md` | **2026-09-29** | Wrote `drone_gui.py` and `drone_dynamics.py`. Ran `motor_diag.py` read-only. Fixed the reconnect hang. Diagnosed the stalled stabilizer loop. |

Tags in this document: **[A]**, **[B]**, or **[A+B]** when both sessions agree. Where the sessions differ, **[A] (later) wins**, except where only [B] examined the point.

**Coverage of the three programs**

| Program | [A] | [B] |
|---|---|---|
| `drone_gui.py` | Written, debugged, tested offscreen and partly on hardware | Never saw it |
| `drone_dynamics.py` | Written, self-checked | Never saw it |
| `motor_diag.py` | Read and ran it twice, read-only | Written, extended with `--duty`, run on hardware |

**Differences reconciled**

- **CRTP header bits 3–2.** [B] said they were not examined. [A] found that cflib sets them to `11`, the firmware's `CRTP_HEADER()` leaves them 0, and the firmware ignores them. [A] is used.
- **Log packet byte format.** [B] did not examine it. [A] documents it (section 2.6).
- **Two different "motors dead" causes.** They do not conflict. [B] hit the **latched tumble-detection stop** (board inverted). [A] hit the **stalled stabilizer loop** (gyro never calibrated because the drone was moved at power-up). Both are in section 5.
- **Repo state after the sessions.** [B] lists commits `4b983f9`, `8138c33`, `6d7f435`, `f31edd1`, `0e075f3`, `560e17b` as made after its session. Commit `560e17b` (*sitaw: disable tumble detection*) was made on 2026-10-12 while migrating this work. Neither report says who made the other five or in which session. The `test_scripts/` files match what [A] left.

**Not covered by either session:** the state estimators (complementary / `sensfusion6`, Kalman), and hardware verification of several GUI features (section 5.6).

---

## 1. Hardware and setup

- **Board:** ESP32-S3, built as `TARGET_ESP32_S2_DRONE_V1_2`, MPU-6050 IMU. [B]
- **Firmware:** ESP-IDF port of Crazyflie 2021.01. The host client is cfclient plus cflib 0.1.26 in `~/github/crazyflie-clients-python/.venv`. [A+B]
- **Motors:** brushed, driven by LEDC PWM. Only one motor was connected during [B]'s tests: **M3**. [B]
- **Motor pin map** (this `sdkconfig`, no pin swap for this target): **M1 = GPIO7, M2 = GPIO4, M3 = GPIO3, M4 = GPIO1**. [B]
- **Single Wi-Fi interface:** the PC cannot be on the drone's AP and on the internet at once. [B] ran tests offline and saved output to a file. [B]
- **Safety:** remove propellers for all bench tests. The drone must be powered up **lying still** (section 5.2).

---

## 2. Wi-Fi protocol between the host and the MCU

The link is stock ESP-Drone: **Crazyflie CRTP packets carried in UDP datagrams**. The tools define no protocol of their own and use cflib's UDP driver. [A+B]

### 2.1 Network setup [A+B]

| Item | Value | Source in firmware |
|---|---|---|
| Role of the ESP32 | Soft access point (`WIFI_MODE_AP`); the PC joins as a station | `drivers/general/wifi/wifi_esp32.c`, `wifiInit()` |
| SSID | `CONFIG_WIFI_BASE_SSID` + `_` + 12 hex digits of the AP MAC, i.e. `ESP-DRONE_<MAC>` | `wifiInit()` |
| Security | WPA/WPA2-PSK, or open if the passphrase is empty. Set in menuconfig (`ESPDrone Config`, `CONFIG_WIFI_PASSWORD`). | `main/Kconfig.projbuild` |
| Channel / max stations | 6 / 3 | `sdkconfig` [A] |
| Drone IP | `192.168.43.42/24`, static; the gateway is the same address | `wifiInit()` |
| Host IP | Leased by the drone's DHCP server (`192.168.43.43` observed) [A] | |
| ESP-NOW | Also started on the same radio, for a joystick [A] | `espnow_ctrl_responder_*` |

`drone_gui.py` keeps a copy of the passphrase in its `WIFI_PASSWORD` constant, which must match the firmware setting.

### 2.2 Transport and ports

- **UDP over IPv4. No handshake and no retransmission at transport level.** [A+B]
- **Drone listens on UDP 2390.** The host (cflib `UdpDriver`) **binds a fixed local port 2399** and `connect()`s to `192.168.43.42:2390`. [A] A fixed port means only one host program can hold the link at a time. Client URI: `udp://192.168.43.42:2390`. [B]
- **Reply addressing:** the drone replies to the address and port of the **last valid datagram received**. It sends nothing until it has received at least one valid packet. [A+B]
- **Firmware queues:** one RX task and one TX task, each with a 16-entry queue. `wifilink.c` feeds a second 16-entry CRTP queue and `crtp.c` dispatches by CRTP port. Datagrams longer than 32 bytes (`WIFI_RX_TX_PACKET_SIZE`) are dropped; the receive buffer is 64 bytes. [A+B]

### 2.3 Packet format (one CRTP packet per UDP datagram) [A+B]

```
UDP payload:  [ header (1) ][ CRTP data (0..30) ][ checksum (1) ]      total <= 32 bytes
checksum   =  (sum of all preceding bytes) mod 256
```

- **Header byte:** bits 7–4 = port, bits 1–0 = channel, bits 3–2 reserved. cflib sets the reserved bits to `11` (for example port 2 / channel 2 goes out as `0x2E`); the firmware's `CRTP_HEADER()` leaves them 0 (log data from the drone starts with `0x52`); the firmware ignores them. [A] `CRTP_MAX_DATA_SIZE = 30`.
- **Checksum:** both sides append and verify it, and strip it before parsing. The firmware drops bad packets (debug print `udp packet cksum unmatched`); cflib returns `None`.
- There is **no sequence number**, so loss cannot be detected at this level.

### 2.4 Special datagrams [A]

| Bytes | Meaning |
|---|---|
| `FF 01 01 01` | Sent by cflib on connect (link-port packet with valid checksum). The first packet that "opens" the drone's reply path. |
| `FF 01 02 02` | Sent by cflib on close. |
| `FF FF` | **Keep-alive**: empty CRTP packet on port 15 / channel 3 plus checksum, every **100 ms**. Sent by the cflib in the cfclient venv, a fork whose history includes "esp-drone: add support". |
| `'n' 'o' 'w' lx ly ry rx` (7 bytes) | ESP-NOW joystick frames, injected by the firmware into the same RX queue. Never sent by the host tools. |

### 2.5 Link-alive rule [A]

`wifilinkIsConnected()` is true while a packet arrived within the last **1000 ms**. When it goes false, `log.c` **deletes all log blocks and flushes the CRTP TX queue**. That is why the keep-alive exists: without it, logging stops after 1 s of host silence.

### 2.6 CRTP services used by the tools

| Port | Channel | Use | Notes |
|---|---|---|---|
| 2 PARAM | 0 TOC | TOC download at connect (cached in `test_scripts/cfcache/`) | Crazyflie TOC v2 |
| 2 PARAM | 1 READ | `param.get_value()` | `[id u16 LE]` → `[id u16 LE, status u8 (0 = OK), value…]` [B] |
| 2 PARAM | 2 WRITE | Motor commands | `[id u16 LE][value in the param's type]`. The firmware writes it and **echoes the packet back; the echo is the acknowledgement** and cflib updates its cache from it. [A+B] |
| 2 PARAM | 3 MISC | cflib/cfclient extras | See 2.7: mostly unanswered |
| 5 LOG | 0 TOC | Log TOC download (cached) | Standard |
| 5 LOG | 1 CONTROL | Create and start blocks | `CREATE_BLOCK_V2`: block id + list of `{type u8, var id u16}`; `START_BLOCK`: block id + period in **10 ms units** [A] |
| 5 LOG | 2 DATA | Telemetry from the drone | `block id (1)` + `timestamp ms (3, LE)` + values in declaration order, **≤ 26 bytes of values** (`LOG_MAX_LEN`) [A] |
| 13 PLATFORM | – | cflib asks for the protocol version at connect | |
| 4 MEM, 15 LINK | – | Enumerated or used by cflib at connect | |
| 3 SETPOINT | – | A zero setpoint sent once by cflib's `close_link()` [A] | Not otherwise used by the tools |

**Example:** `motorPowerSet.m1 = round(0.20 × 65535) = 13107 = 0x3333`, on the wire `2E <id lo> <id hi> 33 33 <checksum>`. The ident comes from the firmware's param TOC. [A]

### 2.7 Parameter sub-protocol details [B]

- The firmware reports `PROTOCOL_VERSION 4` (`components/config/include/config.h:46`), so cflib uses the **V2** (16-bit ID) parameter format. Channels are `TOC=0, READ=1, WRITE=2, MISC=3` (`param.c:61-64`).
- TOC info (V2): ch 0 `[0x03]` → `[0x03, count u16, crc32 u32]`, which also sets `useV2` on the firmware side. TOC item: `[0x02, id u16]` → type byte + group + name.
- Set by name: ch 3 `[0x00, "group\0", "name\0", type, value…]`; the reply has the type byte replaced by an error code.
- **MISC commands 1–6** (`VALUE_UPDATED`, `GET_EXTENDED_TYPE`, `PERSISTENT_STORE/GET_STATE/CLEAR`, `GET_DEFAULT_VALUE`) are **not implemented and silently ignored, with no reply** (`param.c:182`). This causes the cfclient lockup in 5.4.

### 2.8 Parameters and log variables used

| Kind | Name | Type | Meaning |
|---|---|---|---|
| param RW | `motorPowerSet.enable` | u8 | 1 bypasses the mixer; motors follow `m1..m4` (`power_distribution_stock.c:135`) |
| param RW | `motorPowerSet.m1..m4` | u16 | 0–65535. The driver keeps only the **top 8 bits** (`motorsConv16ToBits`) → 8-bit PWM, ~0.39 % steps, max 99.6 %, values < 256 give 0 % |
| param RO | `system.selftestPassed`, `imu_tests.mpu6050` | i8/u8 | Boot self-test results |
| param RW | `stabilizer.stop` | u8 | Emergency stop, **latched** (5.3) |
| param RW | `system.forceArm` | i8 | Forces `systemIsArmed()` true |
| param RW | `health.startPropTest` | u8 | Re-runs the propeller test at **full** power. Not used. [B] |
| log | `acc.x/y/z` (g), `gyro.x/y/z` (°/s) | float | IMU. `acc.z ≈ +1.0` when level and upright |
| log | `stabilizer.roll/pitch/yaw` | float | Degrees |
| log | `stabilizer.rtStab` | float | Stabilizer loop rate in Hz, ≈ 1000 when running. **0 = loop stalled** |
| log | `sys.armed`, `sys.canfly` | i8 | Arming and power state |
| log | `pwm.m1_pwm..m4_pwm` | u32 | The 16-bit ratio last passed to `motorsSetRatio()`, i.e. what really went to the driver. **Use this to confirm a command.** |
| log | `motor.m1..m4` | u32 | **Mixer** output. Ignores the `motorPowerSet` override, so it is **misleading** for bench tests. [B] |
| log | `pm.vbat` (V), `pm.batteryLevel` (0–90 %, 10 % steps), `pm.state` (0 battery, 1 charging, 2 charged, 3 low power, 4 shutdown) | | Battery. [A] |

### 2.9 Rates

| Stream | Direction | Rate |
|---|---|---|
| Keep-alive `FF FF` | host → drone | 10 Hz [A] |
| Log block `imu` (acc + gyro, 24 B) | drone → host | 20 Hz [A] |
| Log block `att` (roll/pitch/yaw, armed, canfly, rtStab; 18 B) | drone → host | 20 Hz [A] |
| Log block `pwm` (4 × u32, 16 B) | drone → host | 10 Hz [A] |
| Log block `pm` (vbat, level, state; 6 B) | drone → host | 2 Hz [A] |
| `motor_diag.py` log | drone → host | 10 Hz, 5 samples per block [A+B] |
| Motor param writes (GUI) | host → drone | ≤ 20 Hz per motor, on change only [A] |
| Stabilizer loop | – | 1000 Hz, driven by the MPU-6050 data-ready interrupt [A+B] |
| Motor PWM | – | LEDC, **15 kHz, 8-bit**, low-speed mode, timer 0, channels 4–7 [B] |

GUI steady-state traffic is about 52 log datagrams/s from the drone plus 10 keep-alives/s. A connect takes about 0.5 s once the TOCs are cached. [A]

---

## 3. The programs

### 3.0 Common requirements

- **Python environment:** the cfclient virtualenv, which provides cflib 0.1.26 (the esp-drone-aware fork), **PyQt6 6.11** and **pyqtgraph 0.14**. The system Python 3.10 has none of these and no tkinter. [A+B]
- **Run with:** `~/github/crazyflie-clients-python/.venv/bin/python test_scripts/<script>.py`
- **Before running:** join the drone's AP (the GUI does this itself), **close cfclient** (UDP port 2399 is shared), remove propellers, power the drone up lying still.
- `test_scripts/README.md` is the user-facing guide.

### 3.1 `drone_gui.py` (≈ 855 lines) [A]

A PyQt6 window that joins the drone's Wi-Fi, opens the CRTP link, shows telemetry and drives motors. The UI text is in Spanish. Four classes:

- **`WifiWorker`:** joins Wi-Fi with NetworkManager (`nmcli`) on a daemon thread, reporting through Qt signals tagged with an **attempt id**. Connect sequence: find the Wi-Fi interface; if already on an `ESP-DRONE_*` network use it, else rescan, pick the strongest and `nmcli dev wifi connect`; set `connection.autoconnect no` on the profile it created; poll for a `192.168.43.x` lease for up to 10 s. Disconnect only drops a network the GUI joined itself.
- **`DroneLink`:** wraps cflib's asynchronous `Crazyflie` API. On `fully_connected` it sets `ready = True`, reads the selftest params and starts the four log blocks (2.9). Log callbacks only copy data into a lock-protected `deque` (200 samples = 10 s) and stamp `last_rx`. `set_motor`, `set_enable` and `stop_all` are no-ops until `ready`. `close()` takes about 1.5 s: detach `cf`, zero motors, sleep 0.2 s, `close_link()` (which sleeps 1 s).
- **`DroneView`:** top view drawn with `QPainter`, front up. M1 rear-left (CW), M2 front-left (CCW), M3 front-right (CW), M4 rear-right (CCW). Shows the **applied** duty per motor (`(pwm >> 8) / 256`), the requested %, a rotation arrow coloured by duty, and at the centre the open-loop XY arrow (blue) and yaw arc (purple) from `drone_dynamics.py`.
- **`MainWindow`:** connection bar (button, status, selftest info, battery bar, "remove propellers" warning); red banner when `rtStab` stays below 100 Hz for > 2.5 s after connecting; motors panel (enable checkbox, mode chooser *Individual* sliders M1–M4 or *Todos* master slider, max-limit spinbox default 30 %, 8-bit readback, **STOP** also on Space/Esc); the top view; an MPU-6050 panel with numbers, `|acc|`, flags, stabilizer Hz and two 10 s plots.

**State machine:** `Desconectado → Buscando Wi-Fi → Asociando → Abriendo CRTP → Conectado`, returning to `Desconectado` through `teardown()` on button, error, watchdog or close.

- **Teardown** is non-blocking: bump the attempt id, disable controls, reset UI, then run `link.close()` → `wifi.drop()` on a worker thread. `closeEvent` does the same synchronously.
- **Watchdogs** (needed because cflib's UDP driver never fires its link-error callback): no log data for 2 s → `Error: sin datos del drone`; CRTP setup unfinished 15 s after Wi-Fi is up → `Error: el drone no responde (CRTP)`.
- **Motor path:** slider → `pending[m] = round(pct/100 × 65535)`; every 50 ms only changed values are sent. Changing mode zeroes all sliders, sends 0 to all four motors and keeps the user's enable choice. STOP zeroes sliders, clears enable and calls `stop_all()`.

It adds its own folder to `sys.path` to import `drone_dynamics`, so it runs from any directory.

### 3.2 `drone_dynamics.py` (≈ 109 lines, pure Python, no dependencies) [A]

An open-loop X-quad "tendency" model.

- `MOTOR_LAYOUT`: motor → (x, y, rotation), x+ right, y+ rear. Single source of truth, also imported by the GUI.
- `thrust(d) = (clamp(d, 0, 100)/100) ** 2` (`THRUST_EXPONENT = 2`; thrust ∝ rpm², rpm ≈ ∝ duty).
- `planar_tendency(duties)` → `(dx, dy, magnitude)` from `−Σ pᵢ·Tᵢ` normalised by 2. Missing motors count as 0 %.
- `yaw_tendency(duties)` = `((T1 + T3) − (T2 + T4)) / 2`, in −1..+1, + meaning CCW.
- `max_tendency(limit)` = `thrust(limit)`, used for arrow scaling. `describe(dx, dy)` gives a Spanish label for one of 8 sectors, or `equilibrado`.
- **Self-check:** `python test_scripts/drone_dynamics.py` prints `drone_dynamics self-check OK` after asserts on equal duties, left pair, rear pair, M1 alone, both CW, a CCW motor alone, M3 alone and `max_tendency`.

### 3.3 `motor_diag.py` (≈ 98 lines) [A+B]

Written in [B], extended with `--duty`, later moved into `test_scripts/`. Uses cflib's **synchronous** API (`SyncCrazyflie`, `SyncLogger`). **Read-only by default.**

1. Connects to `udp://192.168.43.42:2390`, caching TOCs in `test_scripts/cfcache/`.
2. Prints the params that decide whether motors can run: selftest, MPU test, `stabilizer.stop`, `forceArm`, `motorPowerSet.*`.
3. Reads 5 log samples at 10 Hz: `sys.armed`, `sys.canfly`, `acc.z`, `stabilizer.roll`, `stabilizer.rtStab`. This separates "loop running" from "board upright".
4. With `--spin N`: writes `motorPowerSet.mN`, then `enable = 1`; after 0.5 s logs 5 samples of `pwm.mN_pwm` to confirm the value reached the driver; after 3 s total writes `mN = 0` and `enable = 0`.

```
motor_diag.py [--spin {1,2,3,4}] [--value 0-65535 | --duty 0-100]     (default --value 10000)
```

`--value` and `--duty` are mutually exclusive and range-checked. `--duty` converts as `round(P/100 × 65535)`, and the spin banner prints the duty the firmware really applies, e.g. `--duty 25` → `64/255 ≈ 25.0 %`.

```bash
PY=~/github/crazyflie-clients-python/.venv/bin/python
$PY test_scripts/motor_diag.py                      # read-only check
$PY test_scripts/motor_diag.py --spin 3 --duty 25   # M3 at 25 % for 3 s, no propeller
```

**Healthy output** (verified in [B]): `selftestPassed=1`, `mpu6050=1`, `stop=0`, `armed=1`, `acc.z ≈ +1.0`, `roll ≈ 0`, `rtStab ≈ 1000`; during a spin `pwm.mN_pwm` equals the requested value. Dependencies: cflib 0.1.26 plus the standard library (`argparse`, `os`, `time`), and firmware with `power_distribution_stock.c` and `motors.c`, both in the stock build. It does **not** use the GUI's UDP socket patch (4).

---

## 4. Design decisions and why

| Decision | Why |
|---|---|
| **Drive motors through the existing `motorPowerSet` override** [B] | Already in the firmware, no rebuild. It follows the real path `powerDistribution()` → `motorsSetRatio()` → LEDC, so the safety gates still apply (self-test, arming, `stabilizer.stop`, tumble detection). |
| **cflib scripts / direct `param.set_value()` instead of the cfclient Parameters tab** [A+B] | The tab locks up on this firmware (5.4). Direct writes never send the MISC request that causes it. |
| **Read-only by default; spinning needs `--spin`; fixed 3 s spins that end by writing 0** [B] | A diagnostic must never move a motor by accident, and a no-load motor should not run long. |
| **Confirm with `pwm.mN_pwm`, not `motor.mN`** [A+B] | `pwm.*` is set inside `motorsSetRatio()` and shows what was written; `motor.*` ignores the override. |
| **Log `acc.z`, `roll` and `rtStab` next to the params** [B] | Separates "loop running" from "board upright"; this exposed the tumble cutoff. |
| **`--duty` in percent and print `bits/255`** [B] | Users think in duty %; the driver keeps only 8 bits. |
| **Cache folder `test_scripts/cfcache`** [B] | The first copy lived in a scratchpad that was wiped. |
| **PyQt6 + pyqtgraph in the cfclient venv** [A] | Already installed with the patched cflib; no tkinter on the system Python; pyqtgraph handles 20 Hz plots cheaply. |
| **cflib async API + Qt signals** [A] | cflib calls back from its own threads; widgets may only be touched from the GUI thread. |
| **One button does Wi-Fi + CRTP; only drop a network the GUI joined; autoconnect off** [A] | NetworkManager returns to the user's usual network afterwards. |
| **Coalesced writes (50 ms, send on change)** [A] | Slider drags fire dozens of events per second; this caps traffic and guarantees the last value is sent. |
| **Four log blocks, each ≤ 26 B, grouped by how fast values change** [A] | CRTP log packets carry at most 26 value bytes. |
| **Monkey-patch `UdpDriver.connect` to add a 0.2 s socket timeout** [A] | Fixes the reconnect hang (5.1) without editing the venv. |
| **`ready` flag gating param writes** [A] | `Param.set_value()` waits up to **60 s** for the TOC when called early. |
| **Teardown on a worker thread: zero motors → close CRTP → drop Wi-Fi** [A] | The stop and close packets must leave before the network goes away. |
| **Attempt id on Wi-Fi signals** [A] | A cancelled "Buscando Wi-Fi" thread used to finish and connect anyway. |
| **GUI watchdogs; display `rtStab` and a red banner** [A] | The UDP driver never reports link failure, and a stalled loop looks like a GUI bug. |
| **View and tendency use the *applied* duty (pwm log), not the slider** [A] | Shows what the firmware does (0 while disabled, 8-bit rounding). |
| **Mode switch zeroes every motor** [A] | No motor jumps to a value left over from the other mode. |
| **Physics in a separate pure module; quadratic thrust** [A] | Testable without Qt and reusable; thrust ∝ rpm² and rpm ≈ ∝ duty for a brushed motor. |
| **Arrow length relative to the slider limit, captions absolute** [A] | At the default 30 % limit the absolute maximum is 9 % of full scale and arrows hid under the body. |

---

## 5. Problems: fixed, diagnosed, discarded, open

### 5.1 Reconnect hang (fixed) [A]

- **Symptom:** connect → disconnect → connect froze the GUI.
- **Cause chain:** cflib's UDP driver binds fixed port 2399 and blocks in `recvfrom()` with no timeout. On Linux, `close()` from another thread does not wake it, so the port stays bound while the GUI has already dropped Wi-Fi (reproduced locally: reader thread stays alive, rebind fails with `EADDRINUSE`). The failed second `open_link` then called `stop_all()`, which made 5 `set_value()` calls that each waited up to 60 s for a TOC that never came.
- **Firmware ruled out:** it accepts a reconnecting client (`source_addr` updates per packet) and resets logging after 1 s of silence.
- **Fixes:** the socket-timeout patch, the `ready` gate, worker-thread teardown, watchdogs, the attempt id. Verified with a dummy UDP server and then on hardware.

### 5.2 "Connected but all zeros, motors dead" (not a software bug) [A]

- The link was fine (about 20 IMU packets/s) but `stabilizer.rtStab = 0.0`, also seen from `motor_diag.py`.
- **Cause:** `stabilizerTask` waits in `while (!sensorsAreCalibrated())` for gyro calibration. That loop is the only place sensor data is copied out and `motorPowerSet` is applied. The drone had **not been lying still at power-up**.
- Calibration never times out; it re-tests the last 1024 gyro samples until the variance is low, so the drone recovers by itself once left still. **Being still matters, not being level.**
- **No remote recalibrate or reboot command exists.** Two options were proposed and **not implemented**: a `system.reboot` param calling `esp_restart()`, or a recalibrate param (which also needs `stabilizerTask` to stop the motors and wait for calibration again).

### 5.3 Tumble detection cut the motor (diagnosed) [B]

- **Symptom:** the write reached the drone, but `pwm.mN_pwm` read 0 and `stabilizer.stop` went 0 → 1, with `acc.z ≈ −0.85 g`, `roll ≈ 147°` (board inverted).
- **Mechanism:** `sitaw.c:118-127` treats total motor PWM above 1000 as flying, then checks `acc.z < −0.5` (`SITAW_TU_ACC_THRESHOLD`) on 30 consecutive samples (about 30 ms). It then calls `stabilizerSetEmergencyStop()` and `powerStop()` zeroes all motors. The trigger is **latched**, so `stabilizer.stop` cannot be cleared without a reboot.
- **Fix used:** reboot and place the board upright; then `acc.z ≈ +1.0` and M3 and M4 both showed `pwm = 10000`.
- **Current repo state:** commit `560e17b` (2026-10-12) comments out `SITAW_TU_ENABLED` in `sitaw.h`. **Treat that as a bench-only setting:** with tumble detection off, an inverted or crashed drone keeps its motors running.

### 5.4 cfclient Parameters tab locks up (discarded approach) [B]

- **Symptom:** pressing *Set* changed nothing, not even the displayed value.
- **Wrong first hypothesis (retracted):** the table not redrawing (`ParamTab.py:86` emits `dataChanged` with invalid indexes). That bug exists but is not the cause.
- **Root cause:** selecting a row makes cfclient call `get_default_value()` (`ParamTab.py:411`), which sends MISC command 6. The firmware ignores it, so no reply ever comes, and cflib's `_ParamUpdater` holds its lock waiting for one. Every later *Set* waits forever. "Default value: -" is the visible sign.
- **Options identified, not done:** (a) firmware replies `ENOENT` to MISC commands 2–6 (cflib already handles `ENOENT`); (b) use an older cflib or Espressif's forked cfclient (exact version not determined). [A] later found that the venv's cflib is an esp-drone-aware fork, but it still exhibits the stall.

### 5.5 Other discarded or changed approaches

- **tkinter:** rejected (not installed, no fast plotting). [A]
- **Separate Wi-Fi disconnect thread:** replaced by one ordered teardown worker. [A]
- **Absolute arrow scale:** replaced by limit-relative scaling. [A]
- **First rebind test "failed":** the harness had no listener on 2390; the code was fine. [A]
- **Script in a session scratchpad:** lost when cleared; recreated in `test_scripts/`. [B]

### 5.6 Open issues

1. **No firmware watchdog on `motorPowerSet`.** The override bypasses the commander's link-loss watchdog. If the Wi-Fi drops or the script dies while enabled, **the motor keeps running at its last value** until reboot or battery removal. Suggested fix: clear the enable flag when no CRTP packet has arrived for N ms (`power_distribution_stock.c`). [A+B]
2. **`motor_diag.py` has no `try/finally` stop**, so Ctrl+C or an exception mid-spin leaves the motor running. A guaranteed stop and an optional `--ramp` were offered but not implemented. [B]
3. **One host client at a time** (fixed local port 2399, replies go to the last sender). [A+B]
4. **8-bit command resolution** (about 0.4 % steps; values < 256 give 0 %). [A+B]
5. **Battery level** comes from a fixed 10 %-step voltage table tuned for one cell type (`bat671723HS25C`, `pm_esplane.c`) and sags under load. Checked on hardware once: 3.91 V, 50 %. [A]
6. **Not yet verified on hardware** [A]: ALL-slider mode; top-view orientation against the real motor wiring (suggested test: M3 alone should light up front-right); the tendency arrows.
7. **Dynamics model is open-loop:** in flight the stabilizer counteracts it, and thrust ∝ duty² ignores voltage sag, motor and propeller differences, and ground effect. [A]
8. **`nmcli` connection without root** relies on polkit allowing it in a local desktop session; untested elsewhere. [A]
9. **Misleading firmware comment:** `system.c` says self-test can be forced from cfclient, but `system.selftestPassed` is read-only. [B]
10. **Local edit in `crtp_localization_service.c`** used undefined names (`values`, `setpoint`) and only built because the file is not in `SRCS`. Commit `6d7f435` appears to fix it; not verified. [B]
11. **`system.forceArm`** does nothing when the drone is already armed (armed by default because `START_DISARMED` is undefined), and the cfclient *Arm* button relies on a newer "supervisor" protocol this firmware lacks. [A+B]

---

## 6. Ideas worth reusing in another ESP32 project

**Sensor driver (MPU-6050, 1 kHz)** [A]

- **Interrupt-driven sampling:** the data-ready pin fires a GPIO ISR that gives a semaphore; a task does **one I²C burst read** (14 bytes) and gives a second semaphore that paces the 1 kHz control loop. No polling, and the loop is locked to the sensor's clock.
- **Gyro bias by variance gate:** ring buffer of the last 1024 raw samples; accept the mean as bias only when all three axis variances are below a threshold, at least 1 s after boot. It never fails, it waits. Add an LED or sound cue on success.
- **Accelerometer scale from gravity:** average `|a|` over 200 samples after the bias is found and divide every reading by it, so rest reads 1 g.
- **Configuration:** gyro ±2000 °/s; on-chip DLPF set per board (42 Hz on a vibration-prone board, else 98 or 256 Hz); second-order software low-pass at 80 Hz (gyro) and 30 Hz (accel). Axis remapping at decode time handles a rotated mount.
- **Health metric:** count loop iterations per second (`rtStab`) and put it in telemetry. It instantly separates "link dead" from "loop stalled".

**Wi-Fi link** [A+B]

- Soft AP with a static IP and a DHCP server: the host always targets one known address, no discovery.
- One message per UDP datagram plus a 1-byte additive checksum is cheap and enough for a single-client bench link. Add a sequence number if you need loss detection.
- **The echo is the acknowledgement:** echoing each parameter write lets the host confirm and update its cache in one step.
- **Always answer unknown commands with an explicit error** (`ENOENT`/`EINVAL`). A host that waits for each reply otherwise stalls, which is exactly the cfclient failure in 5.4.
- Reply to the last sender's address: simple and reconnect-friendly, but one client only; keep a client table for more.
- **Activity timeout plus host keep-alive (10 Hz):** the device resets session state (log subscriptions, TX queue) when the host disappears. **Pair it with an actuator failsafe**, which this firmware lacks for raw motor commands.
- Multiplex a second input source (the ESP-NOW joystick) into the same RX queue by recognising a distinctive packet shape.

**Telemetry and parameters** [A]

- Name-addressed params and log variables with downloadable, CRC-cached tables of contents (connect drops to about 0.5 s when warm).
- Subscription log blocks: the host declares variables and a period; the device packs them into fixed-layout packets with a block id and a 24-bit ms timestamp. Group by rate of change and keep each block within the payload limit.

**Host GUI rules that transfer directly** [A]

- Never block the UI thread on network calls; gate commands on a "fully connected" flag.
- Coalesce high-rate UI input into a fixed-rate, send-on-change command stream.
- Tear down in a fixed order: zero the actuators, close the link, then drop the network.
- Tag async results with an attempt id; keep your own watchdogs when the transport does not report failures.
- Always give UDP receive sockets a timeout (or wake them explicitly) before closing them from another thread.

**Motor driver and safety** [B]

- LEDC at 15 kHz with 8-bit resolution on 4 channels sharing one timer is a solid brushed-motor setup.
- Keep a **16-bit thrust interface** and shift down to the hardware resolution in one place (`motorsSetRatio`).
- Log the **value actually written** next to the mixer output, so you can tell control problems from driver problems.
- A short boot self-test pulse per motor (20 % for 50 ms in `motorsTest()`) is a cheap wiring check.
- Gate actuator output behind independent checks evaluated every loop: self-test passed, calibrated, armed, not emergency-stopped. Start the control loop only after gyro calibration.
- Tumble detection that depends on motor output and latches worked as designed, but blocks inverted bench tests. Expose a documented bench-mode switch rather than editing a header.
- **Any manual override needs its own watchdog** (auto-disable when no write arrives within N ms).
- Battery estimate: a voltage-table lookup to coarse levels plus a power-state enum; tune it to the actual cell and expect sag.
- `drone_dynamics.py` is reusable for any quad visualiser: swap `MOTOR_LAYOUT` and `THRUST_EXPONENT`.

**Safety guidance for no-load runs at high duty** [B]: expect brush and bushing wear and vibration rather than overheating; ramp up gradually, limit run time, clamp the motor, prefer a current-limited bench supply, and know the kill switches.

---

## 7. Other work from the Sep 24–25 session [B]

- **No firmware changes** (the user asked for none). The only repo file created was `test_scripts/motor_diag.py`.
- **Traced the motor command path:** `stabilizerTask` → controller → `powerDistribution()` / `powerStop()` → `motorsSetRatio()` → LEDC.
- **Boot gating:** `systemTask` self-tests → `systemStart()` → `systemWaitStart()` in the stabilizer → gyro calibration → "Ready to fly." → loop.
- **Reviewed the uncommitted local diffs present at the start:** buzzer-less `soundTest()`, VL53L1X and PMW3901 disabled, quaternion-decompress edits, LED changes.
- **Hardware findings:** motor pin map and M3 confirmation (section 1); upright board gives `acc.z ≈ +1.0`; the earlier −0.85 g reading came from how the board was placed; loop ≈ 1000 Hz; selftest and MPU-6050 test both pass.

---

## 8. Using this in the blimp project

These notes are not from the sessions; they are a reading aid.

- **Quad-specific, needs changes for a blimp:** the four-motor layout and X-quad model in `drone_dynamics.py`, the `motorPowerSet.*` params, the `ESP-DRONE_*` SSID filter, the fixed address `192.168.43.42`, and the `192.168.43.x` lease check in the GUI.
- **Protocol dependency:** `drone_gui.py` and `motor_diag.py` speak CRTP-over-UDP to the esp-drone firmware. They work with the blimp only if the blimp firmware implements the same protocol, or the host side is adapted to whatever protocol the blimp uses.
- **Directly reusable:** the GUI skeleton (Wi-Fi worker, attempt id, teardown order, watchdogs, coalesced commands), the telemetry-block pattern, and the sensor-driver and safety ideas in section 6.
- **Known gap that may already be closed in the blimp repo:** open issue 5.6-1 (no watchdog on manual motor commands). The blimp commit titles about arming the motor failsafe suggest it has its own; confirm in the code.
