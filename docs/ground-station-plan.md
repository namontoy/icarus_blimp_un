# Ground station: design and implementation plan

**Date:** 2026-10-23
**Branch:** `s3-wireless-link`
**Status:** plan agreed in conversation; implementation in `software/ground_station/`, see
section 8 for what exists and what is pending. **2026-10-27:** the browser stack of section 2
was replaced by a native PyQt6 window (`docs/ground-station-native-plan.md`); sections 2, 4, 8
and 9 are updated to match, and the design brief and safety chain still apply. Also
2026-10-27: the cameras talk only to the S3, never to this computer, so the serial camera
source is gone and section 7 is rewritten.

## 1. What it has to do

One screen for the bench and, later, the arena. Three jobs, in the order the team asked for
them:

1. **See the sensors.** IMU (accelerometer, gyroscope, die temperature), magnetometer and
   heading, barometer (pressure, temperature, pressure altitude) and the VL53L1X range, live,
   with short history plots.
2. **Drive the motors.** Four thrusters individually or as the front pair (surge, yaw) and
   the rear pair (heave), with a thrust limit, a stop that is one keystroke away, and the
   readback from the blimp so a command that did not arrive is visible.
3. **Do what the camera viewer does with images.** Show the ESP32-CAM's frames with the
   detector's boxes, the target verdict and the blob list, grab one frame or stream, and show
   the link's health. (Amended 2026-10-27, section 7: the detections are what reaches the
   station, through the S3; frames only as occasional calibration grabs, not yet implemented.) Colour calibration, frame labelling and the classifier dataset stay in
   `tcview` on the `target-cam` branches: those are tooling for the detector, not flight.

Plus the console, since every bench command already works over the air, and an event log for
the things a person must not miss (link lost, failsafe tripped, a station leaving).

## 2. Stack

**Superseded 2026-10-27.** The ground station is now one PyQt6 process with custom-painted
widgets (no web server, no browser, no pywebview); the reasons and the file-by-file plan are in
`docs/ground-station-native-plan.md`. The browser version below is in the git history up to
commit `57eebe9`. The original comparison is kept because it records why the browser was chosen
first:

| Option | For | Against |
|---|---|---|
| PyQt6, like `drone_gui.py` and `tcview` | one process; the team knows it; pyqtgraph | a branded, racing-style look means custom-painted widgets for everything; Qt install per machine; one screen only |
| **Browser UI, Python back end (chosen)** | CSS, SVG and web fonts give the look for free; runs in any browser, phone or tablet included, so a second person can watch; no build step, anyone can edit the HTML; the back end reuses `blimp_link.py` and the serial parser | two layers joined by a WebSocket; the browser tab must stay open for commands |
| Tauri, Electron, Flutter, C# | native windows | a toolchain the team does not use, for no gain here |

The back end is **Tornado**: one Python process serves the static page and the WebSocket on one
port, and it is already installed on the development machine. It bridges three things: the
UDP packet link and console from `blimp_link.py`, the camera's serial protocol, and a demo
mode. The front end is **plain HTML, CSS and JavaScript**, no framework and no bundler. Charts
are drawn on `<canvas>`; the blimp diagram and gauges are inline SVG.

```
browser (any)  <-- WebSocket, JSON -->  python -m ground_station  <-- UDP 7200/7201 -->  S3 firmware
                                                 |
                                                 +-- pyserial (tcview protocol) --> ESP32-CAM
```

Since 2026-10-27 the WebSocket is gone: the window and the back end share one process, and
`station.py`'s Qt signals carry what the messages carried (section 4).

```
PyQt6 window (app.py, panels.py)  <-- Qt signals -->  Station (station.py)  <-- Wi-Fi, UDP 7200/7201 -->  S3 firmware  <-- UART -->  ESP32-CAMs
```

The S3 is the only device the station talks to (section 7).

## 3. Design brief

**Palette: the university's.** The Universidad Nacional's identity guide builds its palette
from the two colours of the coat of arms: an institutional green and an alternate red
([identidad.unal.edu.co, B1](https://identidad.unal.edu.co/guia-identidad-visual/b-directrices-y-especificaciones/b1-elementos-de-identidad-visual/)).
The values used here come from the university library's identity page, which is the only UNAL
page that publishes them in hex: green `#94B43B` (148, 180, 59) and red `#DC313A`
([bibliotecas.unal.edu.co](https://bibliotecas.unal.edu.co/identidad/manual-de-imagen)). The
guide's own palette download is a PDF behind a certificate the fetch tool rejected; if the
Pantone references matter for print, read it there.

**Letters and details: the F1 team.** McLaren's visual language is papaya orange on
anthracite, a wide geometric sans in capitals, angled cuts, and telemetry readouts in big
tabular numerals. The university's typeface, Ancízar, is not freely licensed, so the display
face is **Titillium Web**, the typeface Formula 1 itself used on its site for years, with
**IBM Plex Mono** for data and the console. Papaya `#FF8000` appears only as the detail
colour: the STOP button, the live indicator, the active state of a control, one stripe in the
header. Everything else is green, dark green-tinted neutrals and white.

| Token | Value | Use |
|---|---|---|
| `--bg` | `#0C0F0D` | page, green-biased near-black |
| `--panel` / `--raised` | `#141A16` / `#1C2420` | panels, controls |
| `--border` | `#26302A` | panel edges |
| `--fg` / `--muted` / `--faint` | `#ECEFEA` / `#A9B3A6` / `#6F7A71` | text levels |
| `--un-green` | `#94B43B` | identity, OK state, primary buttons |
| `--un-red` | `#DC313A` | critical state |
| `--amber` | `#F2C14E` | warning state |
| `--papaya` | `#FF8000` | McLaren detail: STOP, live, active |
| series x / y / z | `#789E24` / `#2E9ED0` / `#D96D00` | plot lines; validated with the dataviz palette checker on the panel surface for colour-vision deficiency and contrast (the brighter identity colours fail that check as lines, so plots use these darker steps) |

Motifs: panels with one chamfered corner; uppercase, letter-spaced labels; numerals in tabular
figures; a thin papaya stripe at the top of the header; thrust shown as segmented bars like a
timing tower. Dark only: a ground station is a pit-wall display, and the venue is a hall.

**Layout** (1920 × 1080 reference; columns stack below 1100 px):

```
+------------------------------------------------------------------------------+
| ICARUS  Universidad Nacional de Colombia     link · failsafe · stations  STOP |
+-------------------+-----------------------------------+----------------------+
| MOTORS            | CAMERA                            | SENSORS              |
|  top view, 4 bars |  160x120 frame x4, boxes, verdict |  IMU numbers + plots |
|  pair controls    |  grab / live / connect, FOV plot  |  heading dial        |
|  limit, enable    |  blob table, link health          |  ToF gauge, baro     |
+-------------------+-----------------------------------+----------------------+
| ===== drag handle =====                                                      |
| [CONSOLE] [EVENTS 0 crit · 1 warn]                                        ▾ |
|  the selected tab: console input, output, quick commands / the event log     |
+------------------------------------------------------------------------------+
```

The three panels take all the height under the header; the dock below them holds the
console and the event log as tabs (2026-10-24). Its handle drags the height, ▾ collapses it to
the tab bar, and the browser remembers height, tab and collapsed state. The IMU plots and the
camera frame are the flexible elements, so at 1080p no panel needs a scrollbar; a short window
gets smaller gauges and the motor hint disappears.

## 4. Signals between the station and the window

`station.Station` is a `QObject` that owns the blimp bridge, the camera, the Wi-Fi control and
the console relay. The bridges call back from their own threads; Qt queues a signal emitted off
the GUI thread onto it, so every slot runs on the GUI thread. Station to window:

| Signal | Content | Rate |
|---|---|---|
| `tel(dict)` | link state (`alive`, `age_s`, `rx`, `bad`, `streaming`) and the telemetry packet decoded (`motor`, `failsafe`, `link_alive`, counters) | 10 Hz |
| `sen(dict)` | the sensors packet decoded (section 5), one signal per packet from the receive callback (until 2026-10-28 the newest at each tick, which dropped half of a 20 Hz stream) | up to 20 Hz |
| `cam_frame(QImage, dict)` | one frame, decoded on the camera thread, with `{seq, t_ms, w, h, row}` (the CSV row of the same capture) | per frame |
| `cam_row(dict)` | a CSV row without a frame (flight data) | per row |
| `cam_status(dict)` | camera source: `source`, `connected`, `live`, frames ok/bad | on every change or command |
| `con_text(str)` / `con_end()` | console output chunks and the end of a reply | as it arrives |
| `ev(dict)` | `{ts, level, text}` for the event log (named `ev` because `event` would shadow `QObject.event`) | as they happen |
| `wifi(dict)` | this computer's Wi-Fi toward the blimp: `backend`, `joined`, `ip`, `ssid`, `busy`, `phase`, `error`, `candidates` | at start, on change (checked every 2 s), after a join or leave |
| `cap(dict)` | the capture on the blimp (Bench page, 2026-10-27): `phase` (status, download, saved, error), the last `LOG_INFO` decoded as `info`, `progress`, `file`, `error` | after every command, and every 0.5 s while one records |
| `cap_data(dict)` | a capture downloaded and decoded: `header`, `records`, `rows` (units), `summary`, `file` | per download |

Window to station, as plain method calls: `set_motors([fl, rl, fr, rr])`, `stop(brake)`,
`console(line)`, `camera_command(action, **kw)` with `action` grab or live (the demo camera
only, for now), `wifi_action('connect' | 'disconnect')`, `capture(action, seconds, notes)` with
`action` start, stop, clear, status or download (run on the station's capture thread),
`pad_state()` (the gamepad's latest snapshot from `gamepad.py`'s reader thread, polled by the
Control page at its own 20 Hz tick rather than signalled, since a stick is a continuous input;
2026-10-28), and `close()`. The demo flag and the blimp
address that the old `hello` message carried are attributes of the station (`demo`,
`blimp_host`), and so are the deadman and stream rate (`deadman_s`, `stream_hz`).

The header's **Join blimp Wi-Fi** button has the station join the `ICARUS-…` access point
(`wifi.py`: `netsh wlan` on Windows by profile name, since Windows 11 withholds scans from
consoles without location services; `nmcli` on Linux as in `software/drone_gui.py`). Added
2026-10-24 and kept unchanged in the native version.

## 5. The sensor packet

The firmware streams telemetry (motors, failsafe, link health) and, since 2026-10-24, its
sensors. `LINK_PKT_SENSORS` (0x82, 76 bytes) in `link_protocol.h` and `blimp_link.py` carries
the accelerometer and gyroscope in g and deg/s, IMU temperature, magnetometer in gauss and
heading, pressure, barometer temperature and altitude relative to the first reading since
boot, ToF range and status, and a `valid` bitmask so an absent sensor is never plotted as
zero. On the blimp, `sensor_stream.c` samples through the same attach/read/detach drivers as
the console commands (`imu`, `mag`, `baro`, `tof`; 20 Hz, the barometer at 4 Hz) and
`net_link.c` sends one packet per new sample after each telemetry packet. A console sensor
command borrows the chips for its run and the stream pauses, which the page shows as an
orange notice after 1.5 s without a packet. `mock_blimp.py` sends the same packet for work
without hardware. The stream has not run on hardware yet; `sensors` on the console reports
what it found.

## 6. Safety, end to end

- A thrust command is a standing command only while it keeps arriving. The browser sends its
  motor setpoint at least 5 times a second while motors are enabled; the back end repeats it to
  the blimp at 20 Hz but **stops within 500 ms of the last browser update**; the firmware
  coasts 500 ms after the last packet (link timeout) or 1 s after the last thrust command
  (motor failsafe). A closed tab, a frozen browser, a dead laptop and a dropped Wi-Fi all end
  in coasting motors within about a second.
- STOP (button, Space, Escape) sends `MOTOR_STOP` three times and disables the motor controls.
- The thrust limit defaults to 30 % and clamps every slider; raising it is a deliberate act.
- Motors are off until the operator enables them; enabling never moves a motor by itself.
- The readback bars draw what the blimp reports, never the slider value.

## 7. Camera

**Rewritten 2026-10-27.** The cameras are wired to the S3 and never to this computer
(`docs/inter-board-link-plan.md`). Each ESP32-CAM extracts what it needs from its own pictures
and sends the S3 only that: the forward camera its detections (target verdict, bearing,
elevation, range, blobs, in target_cam's CSV columns), the downward camera its optical flow. The
station talks only to the S3, over Wi-Fi, so camera data will reach it as a packet on the UDP
link, which does not exist yet. Until then the camera panel runs on the demo source in `--demo`
and is empty otherwise.

The inter-board plan also expects occasional single-frame grabs for calibration, relayed by the
S3; not implemented on either chip. The panel keeps its picture view, Grab frame and Live for
that, and the station's `cam_frame` signal carries a frame as a `QImage`, so a future source
only has to deliver PNG bytes (or anything `QImage.fromData` reads).

Earlier versions of this section, and of the code, read the ESP32-CAM directly over a USB
serial port with the target_cam viewer's parser (`--cam PORT`, pyserial). That was removed on
2026-10-27; it is in the git history.

## 8. Phases

| Phase | Content | Status |
|---|---|---|
| 1 | Sensor packet in the protocol, mock sends it | done 2026-10-23 |
| 2 | Back end: Tornado server, blimp bridge with the deadman, demo source, console relay | done 2026-10-23 |
| 3 | Front end: layout, identity, motors, sensors, camera, console, events, keyboard | done 2026-10-23; first run against the blimp 2026-10-24 (link and motors live, sensors empty) |
| 4 | Camera data through the S3: a link packet for the forward camera's detections (and the downward camera's flow), then a station source for it; calibration frame grabs after that | after the camera-to-S3 link (`docs/inter-board-link-plan.md`); the serial-port version was dropped 2026-10-27 |
| 5 | Firmware: sensor task and `LINK_PKT_SENSORS` | done 2026-10-24 (`sensor_stream.c`), not yet run on hardware |
| 6 | Bench: run against the real blimp with the stream, record what the page gets wrong | next |
| 7 | Logging to CSV, parameter tuning from the window, Spanish labels if wanted | the capture and its traces on the Bench page done 2026-10-27 (`bench.py`; in demo and against `mock_blimp.py`, not yet against the blimp); parameters and labels later |
| 8 | Native PyQt6 window in place of the Tornado server and the page (`docs/ground-station-native-plan.md`) | done 2026-10-27 in demo and against `mock_blimp.py`; not yet run against the blimp |
| 9 | Control page: flying by hand (surge, yaw and heave from the keyboard or a gamepad, mixed on the ground into the four duties, as the firmware's MANUAL mode expects, D5 of the control-loop plan), PC and Steam Deck input modes, the autonomous hand-off button | done 2026-10-28 in demo (`control.py`, `gamepad.py`); the hand-off waits for `MODE_SET` (control-loop plan increment 3); not yet run on a Steam Deck or against the blimp |

## 9. Running it

```
python -m software.ground_station --demo            # no hardware: mock blimp and mock camera in-process
python -m software.ground_station                   # the real blimp at 192.168.4.1, join its Wi-Fi first
```

It opens as its own window; the full-screen button in the header or F11 toggles full screen,
and the window size and the dock height are remembered between runs (QSettings). Dependencies
are pinned in `software/ground_station/requirements.txt` (PyQt6 only); the
install is the same `pip install -r` line on every OS, and how the Wi-Fi join differs between
Windows and Linux is in `software/ground_station/README.md` under "Installing".
