# Technical report: ESP-Drone bench tools (`test_scripts/`)

Session of 2026-09-29. Repository: `esp-drone` (port of the Crazyflie 2021.01 firmware to ESP32-S3), private copy at `github.com/namontoy/esp-drone`.

This report covers three Python programs in `test_scripts/`:

| Program | Origin | Coverage in this session |
|---|---|---|
| `drone_gui.py` | Written in this session | Full: design, implementation, two rounds of debugging, offscreen tests, partial hardware tests |
| `drone_dynamics.py` | Written in this session | Full: model, implementation, self-check, rendered test cases |
| `motor_diag.py` | Already in the repo (dated 2026-09-25); **not written in this session** | Read, run read-only against the drone twice, used to diagnose. Its design reasons are known only from its own docstring and comments. |

The scripts in the repo match what the session left. The only later commit (`560e17b`, a firmware change to tumble detection) does not touch `test_scripts/`.

Secrets such as the Wi-Fi passphrase are deliberately left out. The firmware takes it from `CONFIG_WIFI_PASSWORD` in `sdkconfig`. `drone_gui.py` holds a copy in its `WIFI_PASSWORD` constant, which must match.

---

## 1. Wi-Fi protocol between the host and the MCU

### 1.1 Network setup

| Item | Value | Source |
|---|---|---|
| Role of the ESP32-S3 | Soft access point (`WIFI_MODE_AP`) | `components/drivers/general/wifi/wifi_esp32.c`, `wifiInit()` |
| SSID | `CONFIG_WIFI_BASE_SSID` + `_` + AP MAC in hex, i.e. `ESP-DRONE_<MAC>` | `wifiInit()` |
| Security | WPA/WPA2-PSK, or open if the configured passphrase is empty | `wifiInit()` |
| Channel | 6 (`CONFIG_WIFI_CHANNEL`) | `sdkconfig` |
| Max stations | 3 (`CONFIG_WIFI_MAX_STA_CONN`) | `sdkconfig` |
| Drone IP | `192.168.43.42/24`, static. The gateway is also `.42`. | `wifiInit()` (`esp_netif_set_ip_info`) |
| Host IP | Leased by the drone's DHCP server; `192.168.43.43` was observed | `ss -uanp` during the session |
| ESP-NOW | Also started on the same radio, for a joystick | `espnow_ctrl_responder_*` |

The host joins the AP with NetworkManager (`nmcli`), described in section 2.1.

### 1.2 Transport and ports

- **UDP over IPv4. There is no handshake or retransmission at transport level.**
  - The drone listens on **UDP 2390** (`UDP_SERVER_PORT`).
  - The host side (cflib's `UdpDriver`) **binds a fixed local port, 2399**, and `connect()`s the socket to `192.168.43.42:2390`.
  - A fixed port means only one host program can hold the link at a time. Section 4 explains why this mattered.
- **Reply addressing (firmware):** the drone sends replies to the address and port of the **last valid datagram received** (`source_addr`, updated by `recvfrom`). It sends nothing until it has received at least one valid packet (`isUDPConnected`).
- **Tasks and queues (firmware):**
  - One RX task and one TX task, each with a FreeRTOS queue of 16 entries (`udpDataRx`, `udpDataTx`).
  - Above that, `wifilink.c` feeds a second queue of 16 CRTP packets. `crtp.c` then dispatches by CRTP port.
  - Datagrams longer than 32 bytes (`WIFI_RX_TX_PACKET_SIZE`) are dropped.

### 1.3 Packet format (one CRTP packet per UDP datagram)

```
UDP payload:  [ header (1) ][ CRTP data (0..30) ][ checksum (1) ]      total ≤ 32 bytes
checksum   =  (sum of all preceding bytes) mod 256
```

- **CRTP header byte:**
  - Bits 7–4 are the **port**, bits 1–0 the **channel**, and bits 3–2 are reserved.
  - cflib sets the reserved bits to `11` for bootloader compatibility. For example, port 2 / channel 2 goes out as `0x2E`.
  - The firmware's own `CRTP_HEADER(port, ch)` leaves them at 0. For example, the log data header from the drone is `0x52`.
  - The firmware reads port and channel through bitfields and ignores the reserved bits.
- **Checksum:**
  - Both sides append it and both verify it.
  - The firmware drops packets with a bad checksum and logs `udp packet cksum unmatched`.
  - cflib returns `None` for a bad packet.
  - The checksum is not part of the CRTP packet: each side strips it before parsing.
- **Special datagrams:**

  | Bytes | Meaning |
  |---|---|
  | `FF 01 01 01` | Sent by cflib on connect: a CRTP link-port packet with a valid checksum. It serves as the first packet that "opens" the drone's reply path. |
  | `FF 01 02 02` | Sent by cflib on close. |
  | `FF FF` | Keep-alive: an empty CRTP packet on port 15 / channel 3 plus its checksum. Sent every **100 ms** by the cflib installed in the cfclient venv (a fork whose history includes "esp-drone: add support"). |
  | `'n' 'o' 'w' lx ly ry rx` (7 bytes) | ESP-NOW joystick frames, injected by the firmware into the same RX queue and converted to a setpoint. Never sent by the host tools. |

- **Link-alive rule (firmware):**
  - `wifilinkIsConnected()` is true while a packet arrived within the last **1000 ms** (`WIFI_ACTIVITY_TIMEOUT_MS`).
  - When it goes false, `log.c` **deletes all log blocks and flushes the CRTP TX queue**.
  - That is why the keep-alive exists: without it, logging would stop after 1 s of host silence.

### 1.4 CRTP services used by the tools

| CRTP port | Channel | Use by the tools | Payload |
|---|---|---|---|
| 2 PARAM | 0 TOC | TOC download during connect (cached in `test_scripts/cfcache/`) | standard Crazyflie TOC v2 |
| 2 PARAM | 1 READ | `param.get_value()` (selftest flags) | ident (u16 LE) → value |
| 2 PARAM | 2 WRITE | motor commands | `ident (u16 LE)` + value in the parameter's type (e.g. `u16 LE` for `motorPowerSet.mN`, `u8` for `enable`). The firmware writes the value and echoes the packet back. |
| 5 LOG | 0 TOC | log TOC download (cached) | standard |
| 5 LOG | 1 CONTROL | create and start blocks | `CREATE_BLOCK_V2`: block id + list of `{type u8, var id u16}`; `START_BLOCK`: block id + period in **10 ms units** |
| 5 LOG | 2 DATA | telemetry from the drone | `block id (1)` + `timestamp ms (3, LE)` + values packed in declaration order, **≤ 26 bytes of values** (`LOG_MAX_LEN`) |
| 13 PLATFORM | – | platform info, fetched by cflib at connect | standard |
| 4 MEM | – | memory enumeration, done by cflib at connect | standard |
| 3 SETPOINT | – | a zero setpoint, sent once by cflib's `close_link()` | standard |

**Example:** setting M1 to 20 % duty means `motorPowerSet.m1 = round(0.20 × 65535) = 13107 = 0x3333`. On the wire that is:

```
2E  <id lo> <id hi>  33 33  <checksum>
```

The ident (`id lo`, `id hi`) is assigned by the firmware's param TOC.

### 1.5 Parameters and log variables used

| Kind | Name | Type | Meaning |
|---|---|---|---|
| param (RW) | `motorPowerSet.enable` | u8 | 1 makes the power distribution use `m1..m4` instead of the controller output (`power_distribution_stock.c`) |
| param (RW) | `motorPowerSet.m1..m4` | u16 | 0–65535 motor command. The motor driver keeps the **top 8 bits** (`motorsConv16ToBits`), giving 8-bit LEDC PWM at 15 kHz |
| param (RO) | `system.selftestPassed`, `imu_tests.mpu6050` | i8/u8 | boot self-test results |
| param (RW) | `stabilizer.stop`, `system.forceArm` | u8/i8 | read by `motor_diag.py` only |
| log | `acc.x/y/z` | float | g |
| log | `gyro.x/y/z` | float | °/s |
| log | `stabilizer.roll/pitch/yaw` | float | ° |
| log | `stabilizer.rtStab` | float | stabilizer loop rate (Hz, updated every second); ≈1000 when running |
| log | `sys.armed`, `sys.canfly` | i8 | arming and power state |
| log | `pwm.m1_pwm..m4_pwm` | u32 | the 16-bit ratio the driver last applied |
| log | `pm.vbat` | float | battery voltage (V) |
| log | `pm.batteryLevel` | u8 | 0–90 % in 10 % steps, from a voltage table |
| log | `pm.state` | i8 | 0 battery, 1 charging, 2 charged, 3 low power, 4 shutdown |

### 1.6 Update rates

| Stream | Direction | Rate | Notes |
|---|---|---|---|
| Keep-alive `FF FF` | host → drone | 10 Hz | cflib UDP driver thread |
| Log block `imu` (acc + gyro, 24 B) | drone → host | 20 Hz (50 ms) | GUI |
| Log block `att` (roll/pitch/yaw, armed, canfly, rtStab; 18 B) | drone → host | 20 Hz | GUI |
| Log block `pwm` (4 × u32, 16 B) | drone → host | 10 Hz (100 ms) | GUI |
| Log block `pm` (vbat, level, state; 6 B) | drone → host | 2 Hz (500 ms) | GUI |
| Motor param writes | host → drone | ≤ 20 Hz **per motor**, only on change | GUI coalescing timer (50 ms) |
| GUI refresh (labels, plots, top view) | – | 20 Hz | Qt timer, 50 ms |
| `motor_diag.py` log | drone → host | 10 Hz, 5 samples | one block |
| Firmware stabilizer loop | – | 1000 Hz | driven by the MPU-6050 data-ready interrupt |

Steady-state traffic from the GUI is about 52 log datagrams/s from the drone, plus 10 keep-alives/s and occasional parameter writes to it. A connect takes about 0.5 s once the TOCs are cached (measured).

---

## 2. The programs: how they work and how to run them

### 2.0 Common dependencies

- **Python environment:** the existing cfclient virtualenv, which provides:
  - `cflib` 0.1.26, a fork with the esp-drone UDP support: checksum, keep-alive and port 2390;
  - **PyQt6** 6.11;
  - **pyqtgraph** 0.14.

  The system Python (3.10) has none of these, nor tkinter.
- **Run with:** `~/github/crazyflie-clients-python/.venv/bin/python test_scripts/<script>.py`
- **Other requirements:**
  - Linux with **NetworkManager** (`nmcli`), for the GUI.
  - Close cfclient first, because local UDP port 2399 is shared.
  - The drone must be **powered up lying still**. Otherwise the stabilizer never starts: see 4.2.
- A user-facing README is in `test_scripts/README.md`.

### 2.1 `drone_gui.py` (≈ 855 lines)

**Structure.** One file, with four main classes.

- **`WifiWorker(QObject)`: Wi-Fi via `nmcli`**
  - Runs on a daemon thread and reports through Qt signals tagged with an **attempt id**.
  - **Connect sequence:**
    1. Find the Wi-Fi interface.
    2. If it is already associated with an `ESP-DRONE_*` network, use that network.
    3. Otherwise rescan, pick the strongest `ESP-DRONE_*` network, and run `nmcli dev wifi connect <ssid> password <WIFI_PASSWORD> ifname <iface>`.
    4. Set `connection.autoconnect no` on the profile it created.
    5. Poll `IP4.ADDRESS` for up to 10 s until a `192.168.43.x` lease appears.
  - **Disconnect:** `nmcli connection down id <ssid>`, only if the GUI made the connection itself.
- **`DroneLink(QObject)`: wraps cflib's asynchronous `Crazyflie` API**
  - On `fully_connected` (param values downloaded), it sets `ready = True`, reads the two selftest params on a thread, and starts the four log blocks (section 1.6).
  - Log callbacks run on cflib's thread. They only copy data into a lock-protected `deque` (200 samples = 10 s) or dicts, and stamp `last_rx`.
  - `set_motor`, `set_enable` and `stop_all` are **no-ops until `ready`** and catch exceptions.
  - `close()` blocks for about 1.5 s:
    1. it detaches `cf` first, so no other thread uses a half-closed link;
    2. it zeroes the motors if `ready`;
    3. it sleeps 0.2 s;
    4. it calls `close_link()`, which sleeps 1 s inside the UDP driver.
- **`DroneView(QWidget)`: top view drawn with `QPainter`**
  - Front is up. M1 is rear-left (CW), M2 front-left (CCW), M3 front-right (CW), M4 rear-right (CCW).
  - Per motor: the **applied** duty (from `pwm.mN_pwm`, `(raw >> 8) / 256`), the requested %, and a 270° rotation arrow coloured by duty (grey at 0, then green, orange, red).
  - At the centre: the open-loop XY arrow (blue) and the yaw arc (purple) from `drone_dynamics.py`, with a caption.
- **`MainWindow(QMainWindow)`: layout, state machine, 50 ms tick**
  - **Connection bar:** button, status, selftest info, a battery bar coloured by level and state, and a "remove propellers" warning.
  - **Red banner:** shown when `rtStab` stays below 100 Hz for more than 2.5 s after connecting.
  - **Motors panel:**
    - an enable checkbox (`motorPowerSet.enable`);
    - a **mode chooser**: Individual sliders M1–M4, or **Todos**, a master **ALL** slider, with the inactive set disabled;
    - a **max-limit** spinbox (default 30 %) that caps every slider;
    - an 8-bit readback under each slider;
    - **STOP**, also on Space and Esc.
  - **Top-view panel.**
  - **MPU-6050 panel:** numbers, `|acc|`, armed/canfly, stabilizer Hz, and two 10 s plots (pyqtgraph).

**Connection state machine:**

```
Desconectado ─Conectar→ Buscando Wi-Fi → Asociando → Abriendo CRTP → Conectado
      ↑                                                                  │
      └──────────── teardown (button / error / watchdog / close) ←───────┘
```

- **Teardown (`teardown()`):**
  - It is non-blocking. It bumps the attempt id, disables the controls, resets the motor UI, battery, banner and view, then runs `link.close()` → `wifi.drop()` on a worker thread.
  - When the worker finishes, it emits `torn_down`, which re-enables the button.
  - `closeEvent` does the same thing synchronously, since blocking on exit is acceptable.
- **Watchdogs in `tick()`:**
  - no log data for **2 s** while linked → `Error: sin datos del drone`;
  - CRTP setup not finished **15 s** after Wi-Fi is up → `Error: el drone no responde (CRTP)`.
- **Motor command path:**
  - slider `valueChanged` → `pending[m] = round(pct/100·65535)`;
  - each 50 ms tick sends `set_value` only for motors whose value changed (`sent[m]`).
  - Mode switch:
    1. resets all sliders to 0;
    2. queues 0 for all four motors;
    3. invalidates `sent`, so the zeros go out;
    4. keeps the user's enable choice.
  - STOP: sliders to 0, enable off, `stop_all()`.

**Run.**
```bash
~/github/crazyflie-clients-python/.venv/bin/python test_scripts/drone_gui.py
```
It works from any directory, because it puts its own folder on `sys.path` to import `drone_dynamics`.

### 2.2 `drone_dynamics.py` (≈ 109 lines, pure Python, no dependencies)

- **`MOTOR_LAYOUT`:** motor → (x, y, rotation), with x+ right and y+ rear. This is the single source of truth, imported by the GUI.
- **`thrust(d)`:** `(clamp(d, 0, 100) / 100) ** THRUST_EXPONENT`, with exponent 2.
- **`planar_tendency(duties)`:** returns `(dx, dy, magnitude)`.
  - The vector is `−Σ pᵢ·Tᵢ`, normalised by 2, the maximum imbalance.
  - Missing motors count as 0 %.
  - The direction is a unit vector in screen axes, so the GUI draws it directly.
- **`yaw_tendency(duties)`:** `((T1 + T3) − (T2 + T4)) / 2` in −1..+1, where + means the body tends to rotate CCW.
- **`max_tendency(limit)`:** the largest magnitude reachable when no motor exceeds `limit`. It equals `thrust(limit)` and is used by the GUI for arrow scaling.
- **`describe(dx, dy)`:** a Spanish label for one of 8 sectors (`adelante-derecha`, …), or `equilibrado`.

**Run the self-check:**
```bash
~/github/crazyflie-clients-python/.venv/bin/python test_scripts/drone_dynamics.py
# prints: drone_dynamics self-check OK
```

The asserts cover: equal duties; left pair only (→ right, magnitude 1); rear pair only (→ forward); M1 alone (→ forward-right, √2/2); both CW (→ yaw +1, no XY); CCW motor alone (→ yaw < 0); M3 alone (→ rear-left, yaw > 0); and `max_tendency` consistency.

### 2.3 `motor_diag.py` (≈ 98 lines; not written in this session)

**What it does:** it uses cflib's **synchronous** API (`SyncCrazyflie`, `SyncLogger`).
1. It prints nine params: selftest, MPU test, `stabilizer.stop`, `forceArm`, and `motorPowerSet.*`.
2. It prints five log samples at 100 ms: `sys.armed`, `sys.canfly`, `acc.z`, `stabilizer.roll`, `stabilizer.rtStab`.
3. With `--spin N`, it sets `motorPowerSet.mN` and `enable`, prints five `pwm.mN_pwm` samples, waits for a total of 3 s, then zeroes both.

**Options:** `--spin {1..4}`, and either `--value` (raw 0–65535, default 10000) or `--duty` (0–100 %).

**Run:**
```bash
PY=~/github/crazyflie-clients-python/.venv/bin/python
$PY test_scripts/motor_diag.py                      # read-only
$PY test_scripts/motor_diag.py --spin 3 --duty 25   # spin M3 at 25 % for 3 s (no propellers)
```

**Use in this session:** it was run twice in read-only mode. Its `stabilizer.rtStab = 0.0` output (against about 995 Hz in an earlier log) is what showed the stabilizer loop was stalled (section 4.2). It does not use the GUI's UDP socket patch, which doesn't matter for a single short run.

---

## 3. Design decisions and why

| Decision | Why |
|---|---|
| **PyQt6 + pyqtgraph in the cfclient venv** | Already installed and already holding the patched cflib. The system Python has no tkinter. pyqtgraph handles 20 Hz plots cheaply. |
| **cflib async API + Qt signals** | cflib calls back from its own threads. Qt widgets must only be touched from the GUI thread, so callbacks only store data or emit signals. |
| **One button does Wi-Fi + CRTP** | The user chose it. The GUI only drops a Wi-Fi network it joined itself and disables autoconnect on the profile, so NetworkManager returns to the user's usual network. |
| **Direct `motorPowerSet` param writes** (same as `motor_diag.py`) | Bypasses the controller for per-motor bench tests. cflib's direct writes avoid the cfclient Parameters tab, which (according to `motor_diag.py`'s docstring) stalls on `MISC_GET_DEFAULT_VALUE` requests that this firmware never answers. |
| **Coalesced param writes (50 ms, send-on-change)** | Dragging a slider fires dozens of events per second. Coalescing caps traffic at 20 writes/s per motor and guarantees the last value is sent. |
| **Four log blocks, each ≤ 26 B** | A CRTP log packet carries at most 26 bytes of values. acc+gyro (24 B) fill one block; attitude/flags/rtStab (18 B), pwm (16 B) and power (6 B) need others. Rates are chosen by how fast each value changes. |
| **Monkey-patch `UdpDriver.connect` to add a 0.2 s socket timeout** | Fixes the reconnect hang (4.1) without editing the venv. The driver's `receive_packet()` already treats `OSError` (the parent of `socket.timeout`) as "no packet". |
| **`ready` flag gating all param writes** | cflib's `Param.set_value()` waits up to **60 s** for the TOC if called early. Five such calls froze the GUI for minutes. |
| **Teardown on a worker thread, in order: zero motors → close CRTP → drop Wi-Fi** | `close()` takes about 1.5 s of sleeps. The order ensures the stop and close packets leave before the network goes away. |
| **Attempt id on Wi-Fi signals** | Cancelling during "Buscando Wi-Fi" previously let the stale thread finish and connect anyway. Results from an old attempt are now ignored. |
| **Watchdogs in the GUI** (2 s RX silence, 15 s setup) | The UDP driver never calls cflib's link-error callback, so `connection_lost` never fires. |
| **Display `stabilizer.rtStab` + red banner** | A live link can carry all-zero data when the stabilizer loop is stalled. Without this indicator, that looks like a GUI bug. |
| **Top view and tendency use the *applied* duty (pwm log), not the slider** | Shows what the firmware actually does: for example 0 while enable is off, and 8-bit rounding. |
| **Mode switch zeros every motor** | No motor may jump to a value left over from the other mode. |
| **Physics in a separate pure module** | The user asked for it. It also makes the model testable without Qt and reusable. |
| **Quadratic thrust model** | The user chose it, on the grounds that propeller thrust ∝ rpm² and a brushed motor's rpm is roughly ∝ duty. |
| **Arrow length relative to the slider limit; captions absolute** | At the default 30 % limit, the absolute maximum is only 9 % of full scale, and arrows were hidden under the body. |

---

## 4. Approaches tried and discarded; known issues

### 4.1 Reconnect hang (fixed)

- **Symptom:** connect → disconnect → connect froze the GUI.
- **Root cause,** a chain:
  1. cflib's UDP driver binds the fixed port 2399 and blocks in `recvfrom()` with no timeout.
  2. On Linux, `close()` from another thread **does not wake** a blocked `recvfrom`, so the kernel keeps the socket and the port bound. The GUI brought Wi-Fi down right after closing, so no further datagram arrived to unblock it.
     - This was reproduced locally: the reader thread stays alive, and a new `bind` fails with `EADDRINUSE`.
     - It sometimes worked by chance, when a final log packet arrived just after the close.
  3. The second `open_link` failed. The failure handler called `stop_all()`, which made 5 × `set_value()` calls, each waiting up to 60 s for a TOC that would never come.
- **Firmware ruled out:** it accepts a reconnecting client, because `source_addr` updates on each packet, and it resets logging after 1 s of silence.
- **Fixes:**
  - the socket-timeout patch;
  - the `ready` gate;
  - teardown on a worker thread;
  - GUI watchdogs;
  - (later) the attempt id.
- **Verified:** with a dummy UDP server, rebind succeeds with the patch and fails with `Address already in use` without it. The user then confirmed that connect and disconnect work on hardware.

### 4.2 "No sensor data, motors dead" (not a software bug)

- **Symptom:** after the reconnect fix, the GUI showed `Conectado`, but every value read exactly 0 and the motors did not respond.
- **Investigation:**
  - The link was fine: about 20 IMU packets/s, both with and without the patch.
  - `motor_diag.py`, which does not use any of the new code, showed `stabilizer.rtStab = 0.0`.
  - The firmware binary had not changed since 2026-09-23.
- **Cause:** `stabilizerTask` waits for the gyro calibration, `while (!sensorsAreCalibrated())`, before its loop starts. That loop is the only place that copies sensor data out and applies `motorPowerSet`. The user found that the drone had **not been lying still at power-up**.
- **Discarded hypothesis:** that the GUI change broke data display.
- **Firmware review** (no code changed):
  - Calibration never times out: it keeps re-testing the last 1024 gyro samples until the variance is below threshold. So the drone should recover by itself once it is left still.
  - Being still matters, not being level: the level check exists only in an unused manufacturing test.
- **Remote recalibrate / re-arm command:** none exists. `system.forceArm` doesn't help, since the drone is already armed; neither does `health.startPropTest`, and the platform and link services have no reboot command.
- **Two options proposed, neither implemented:**
  1. a `system.reboot` param calling `esp_restart()`;
  2. a true recalibrate param, which would also need `stabilizerTask` changed to stop the motors and wait for calibration again.

### 4.3 Other things tried and changed

- **tkinter:** rejected. It isn't installed, and has no fast plotting.
- **Separate Wi-Fi disconnect thread** (`WifiWorker.disconnect`): replaced by a single ordered teardown worker.
- **Absolute arrow scale:** replaced by limit-relative scaling (section 3).
- **Test harness false alarm:** the first rebind test "failed" because the harness had no listener on 2390, so `close()` raised `ConnectionRefused`. Repeated with a dummy UDP server; the code itself was fine.

### 4.4 Known issues and limits

1. **No firmware watchdog on `motorPowerSet`.** If the Wi-Fi drops while enabled, the motors keep their last command until the drone is reset. The GUI cannot prevent this. Suggested firmware fix: clear `motorSetEnable` when no CRTP packet has arrived for N ms (`power_distribution_stock.c`).
2. **One host client at a time,** because of the fixed local port 2399 in cflib's UDP driver.
3. **Command resolution:** the PWM is 8-bit, so commands move in steps of about 0.4 %.
4. **Battery level** comes from a fixed voltage table (10 % steps) tuned for one cell type (`bat671723HS25C` in `pm_esplane.c`), and the voltage sags under motor load.
5. **`nmcli` permissions:** connecting as a normal user relies on polkit allowing it in a local desktop session. This was not tested in other setups.
6. **Not yet verified on hardware:**
   - ALL-slider mode;
   - the top-view orientation against the real motor wiring (suggested test: M3 alone should light up front-right);
   - the tendency arrows on hardware.

   These were verified offscreen only: rendered images and a fake link. Battery read-out *was* checked on hardware (3.91 V, 50 %, "En batería").
7. **The dynamics model is open-loop.** In flight the stabilizer counteracts the tendency it shows. Thrust ∝ duty² ignores voltage sag, motor and propeller differences, and ground effect.

---

## 5. Ideas worth reusing in another ESP32 project

These come from firmware code read during the session (the Crazyflie port) and from the host tools.

The session **did not examine** the estimators in detail: complementary/`sensfusion6`, Kalman. There are no findings on them here.

### 5.1 IMU driver (`hal/src/sensors_mpu6050_hm5883L_ms5611.c`)

- **Interrupt-driven 1 kHz sampling:**
  - The MPU-6050 data-ready pin fires a GPIO ISR (rising edge) that gives a binary semaphore.
  - A sensor task does **one I²C burst read** of 14 bytes (accel + temperature + gyro, plus mag/baro when present) and processes it.
  - It then gives a second semaphore that paces the 1 kHz control loop.
  - Result: no polling, and the control loop is locked to the sensor's sample clock.
- **Gyro bias by variance gate:**
  - Keep a ring buffer of the last 1024 raw samples.
  - On each sample, compute per-axis variance and mean.
  - Accept the mean as bias only when all three variances are below a threshold, and at least 1 s after boot.
  - It never fails, it just waits, so it tolerates handling at power-up. Add an LED or sound cue on success, as this firmware does.
- **Accelerometer scale from gravity:** average `|a|` over 200 samples right after the gyro bias is found, then divide every reading by it, so the result reads 1 g at rest.
- **Configuration:**
  - gyro ±2000 °/s;
  - on-chip DLPF set per board (42 Hz on the vibration-prone board; otherwise 98 Hz or 256 Hz);
  - second-order software low-pass filters (`lpf2p`) at 80 Hz for the gyro and 30 Hz for the accelerometer, run at 1 kHz.
- **Axis remapping** at decode time handles a rotated IMU mount (x/y swapped, x negated).
- **Health metric:** a rate supervisor counts loop iterations per second (`rtStab`). Exposing it in telemetry instantly separates "link dead" from "loop stalled".

### 5.2 Wi-Fi link design (firmware + host)

- **Soft AP with a static IP and a DHCP server:** the host always targets one known address, with no discovery.
- **Thin UDP framing:** one message per datagram, plus a 1-byte additive checksum. It is cheap and easy to implement on both sides.
- **Reply to the last sender's address:** supports reconnects and changing host ports for free.
- **Activity timeout plus a host keep-alive (10 Hz):** the device resets session state (log subscriptions, TX queue) when the host disappears. Worth pairing with an **actuator failsafe**, which this firmware lacks for raw motor commands.
- **Multiplexing a second input source** (ESP-NOW joystick) into the same RX queue, by recognising a distinctive packet shape.

### 5.3 Telemetry and parameter patterns (host side)

- **Name-addressed parameters and log variables with downloadable tables of contents**, cached on the host by CRC. Connects drop to about 0.5 s once the cache is warm.
- **Subscription log blocks:**
  - The host declares a list of variables and a period (10 ms units).
  - The device packs them into fixed-layout packets with a block id and a 24-bit ms timestamp.
  - Group variables by how fast they change, and keep each block within the payload limit.
- **GUI rules that transfer directly:**
  - Never block the UI thread on network calls.
  - Gate commands on a "fully connected" flag.
  - Coalesce high-rate UI inputs into a fixed-rate, send-on-change command stream.
  - Run teardown in a fixed order: zero the actuators, close the link, then drop the network.
  - Tag async results with an attempt id.
  - Keep your own watchdogs when the transport doesn't report failures.
- **Socket hygiene:** always give UDP receive sockets a timeout, or wake them explicitly, before closing them from another thread.

### 5.4 Small reusable pieces

- **`drone_dynamics.py`:** an X-quad open-loop tendency model, usable as is for any quad visualiser. Swap `MOTOR_LAYOUT` and `THRUST_EXPONENT` for other frames.
- **Battery estimate:** a voltage-table lookup to coarse charge levels plus a power-state enum. Simple, but tune the table to the actual cell and expect sag under load.
