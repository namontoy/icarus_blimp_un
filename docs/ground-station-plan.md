# Ground station: design and implementation plan

**Date:** 2026-10-23
**Branch:** `s3-wireless-link`
**Status:** plan agreed in conversation; implementation in `software/ground_station/`, see
section 8 for what exists and what is pending.

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
   the link's health. Colour calibration, frame labelling and the classifier dataset stay in
   `tcview` on the `target-cam` branches: those are tooling for the detector, not flight.

Plus the console, since every bench command already works over the air, and an event log for
the things a person must not miss (link lost, failsafe tripped, a station leaving).

## 2. Stack: browser front end, Python back end

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
| CONSOLE  input, output, quick commands                 | EVENTS               |
+------------------------------------------------------------------------------+
```

## 4. Messages between browser and back end

JSON, one object per WebSocket message. Server to browser:

| `t` | Content | Rate |
|---|---|---|
| `hello` | demo flag, blimp address, camera port, protocol limits | once |
| `tel` | the telemetry packet decoded (`motor`, `failsafe`, `link_alive`, counters) plus `age_s` | 10 Hz |
| `sen` | the sensors packet decoded (section 5) | 10 Hz |
| `cam` | one frame as a PNG data URI with its width, height, sequence and the CSV row of the same capture | per frame |
| `camrow` | a CSV row without a frame (flight data) | per row |
| `camstat` | camera link: connected, port, baud, rows/s, frames ok/bad | 2 Hz |
| `con` / `con_end` | console output chunks and the end of a reply | as it arrives |
| `ev` | `{level, text}` for the event log | as they happen |

Browser to server: `motor {cmd:[fl,rl,fr,rr]}`, `stop {brake}`, `con {line}`,
`cam {action: connect|disconnect|grab|live|baud, ...}`.

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

The ESP32-CAM talks the protocol in `firmware/projects/target_cam/host/viewer/tcview/protocol.py`
over its own USB serial port: CSV rows (target verdict, bearing, elevation, range, blobs,
timings) at 115200 baud and `#!FRAME` packets of raw RGB565 160 × 120 pixels when asked (`g` for
one, `v1`/`v0` for live), with a baud handshake up to 460800. The back end reuses that parser
when the `target_cam` project is present on the branch (it is not on `master` yet) and the
pyserial reader is its own; frames are converted to PNG for the browser. Without the camera
project the camera panel runs on the demo source. When the inter-board link lands, frames and
rows will come through the S3 over UDP instead, and only the back end's source changes.

## 8. Phases

| Phase | Content | Status |
|---|---|---|
| 1 | Sensor packet in the protocol, mock sends it | done 2026-10-23 |
| 2 | Back end: Tornado server, blimp bridge with the deadman, demo source, console relay | done 2026-10-23 |
| 3 | Front end: layout, identity, motors, sensors, camera, console, events, keyboard | done 2026-10-23; first run against the blimp 2026-10-24 (link and motors live, sensors empty) |
| 4 | Camera over serial with the tcview parser | after the `target-cam` branch merges |
| 5 | Firmware: sensor task and `LINK_PKT_SENSORS` | done 2026-10-24 (`sensor_stream.c`), not yet run on hardware |
| 6 | Bench: run against the real blimp with the stream, record what the page gets wrong | next |
| 7 | Logging to CSV, parameter tuning from the page, Spanish labels if wanted | later |

## 9. Running it

```
python -m software.ground_station --demo            # no hardware: mock blimp and mock camera in-process
python -m software.ground_station                   # the real blimp at 192.168.4.1, join its Wi-Fi first
python -m software.ground_station --cam COM5        # plus the camera on a serial port
```

It prints the address to open, `http://127.0.0.1:8765/`, and opens it in the default browser
unless `--no-browser`. Tornado is the one dependency beyond the standard library
(`pip install tornado`); pyserial is needed for the camera.
