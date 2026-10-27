# Ground station: from browser page to native PyQt6 app

**Date:** 2026-10-27
**Branch:** `s3-wireless-link`
**Status:** plan approved in conversation; implemented 2026-10-27 (verification in §8: demo,
deadman and `mock_blimp.py` runs pass offscreen, and the user ran the demo on screen; the bench
run with the S3 is pending). Amended the same day: `--cam PORT`, `SerialCamera` and pyserial are removed,
because the cameras talk only to the S3 (`docs/ground-station-plan.md` §7); the `camera.py` and `app.py` rows
of §2 and the camera buttons of §5 below describe the first implementation. It supersedes the stack
choice in `docs/ground-station-plan.md` §2. That document's design brief (§3), safety chain (§6)
and camera notes (§7) still apply.

## 1. Why

`software/ground_station/` is today a Tornado back end that serves an HTML/CSS/JS page over a
WebSocket, optionally inside a pywebview window. The goal is one plain Python GUI process, with
no web server, browser or web engine, that looks and behaves like the current page (palette,
three-panel layout, dock, safety chain) and keeps the Wi-Fi join.

**The Wi-Fi connect from `drone_gui.py` is already in the ground station.** `wifi.py` is that
nmcli flow (scan for the strongest SSID with the prefix, connect with the key, turn autoconnect
off), plus a Windows `netsh` back end. The header already has a "Join blimp Wi-Fi" button. The
native app reuses `wifi.py` unchanged, so drone_gui's behaviour carries over and Windows keeps
working.

Decisions:

- **Replace** the web version. Git history keeps it.
- **Bundle** the Titillium Web and IBM Plex Mono TTFs (OFL licence).
- Toolkit: **PyQt6**, already pinned at 6.11.0 in `software/requirements.txt` and the one
  `drone_gui.py` uses. Plots, gauges and the top view are custom `QPainter` widgets, not
  pyqtgraph, so they match the page's look exactly and add no dependency.
- **No firmware change.** The app talks to the same UDP 7200/7201 link through the same
  `blimp_link.py`.

## 2. What stays, what goes

| File | Fate |
|---|---|
| `blimp_bridge.py` | **kept as is**: BlimpLink and console, 20 Hz stream, 0.5 s deadman, `DemoBlimp`. Only the wording "from the page" becomes "from the GUI". |
| `camera.py` | **kept**: `DemoCamera`, `SerialCamera`, `make_camera`. Frames still arrive as PNG bytes; Qt decodes them with `QImage.fromData`. Docstring updated. |
| `wifi.py` | **kept unchanged** |
| `server.py`, `window.py`, `web/` | **deleted** |
| `__main__.py` | imports `main` from `app.py` instead of `server.py` |
| new `app.py` | argparse (same options as now: `--demo --blimp --cam --wifi-ssid --wifi-password --wifi-prefix`; dropped: `--port --bind --window --browser --no-browser`), QApplication, fonts, stylesheet, main window, `F11` |
| new `station.py` | `Station(QObject)`: what `server.Hub` does, with Qt signals in place of WebSocket messages |
| new `theme.py` | the colour tokens from `app.css` under the same names, the QSS sheet, font loading |
| new `widgets.py` | the custom-painted pieces (section 5) |
| new `panels.py` | `MotorsPanel`, `CameraPanel`, `SensorsPanel`, `Dock` |
| new `fonts/` | TitilliumWeb 400/600/700/900, IBMPlexMono 400/500/600, plus `OFL.txt` for each family, from the google/fonts repo |

## 3. Station (replaces Hub and the WebSocket)

`Station(QObject)` owns `BlimpBridge` (or `DemoBlimp` in `--demo`), the camera source, `Wifi`
and the console worker thread. It is ported nearly line for line from `server.Hub`:

- Signals: `tel(dict)`, `sen(dict)`, `cam_frame(QImage, dict)`, `cam_row(dict)`,
  `cam_status(dict)`, `con_text(str)`, `con_end()`, `event(dict)`, `wifi(dict)`. They are emitted
  from the bridge, camera and Wi-Fi threads. Qt queues cross-thread signals onto the GUI thread,
  which takes over the job of `loop.add_callback`.
- A `QTimer` at 100 ms does what `Hub.tick` does: `blimp.snapshot()` → `tel`, a new
  `sensors_rx` → `sen`, and every 20 ticks a Wi-Fi status check.
- Methods the panels call: `set_motors(cmd)`, `stop(brake)`, `console(line)`,
  `camera_command(action, **kw)`, `wifi_action('connect'|'disconnect')` (on its own thread,
  as now), `close()`.
- The camera's PNG becomes a `QImage` on the camera thread, so the GUI thread only paints it.

## 4. Safety chain

It is unchanged in substance from `docs/ground-station-plan.md` §6.

- While "Motors enabled" is on, a GUI-thread `QTimer` (100 ms) calls `station.set_motors(...)`.
  If the GUI thread freezes, the refresh stops, the bridge's deadman releases within 0.5 s and the
  firmware coasts. This is the same chain the browser had.
- STOP: the papaya button, `Esc` always, and `Space` unless the console input has focus (the
  page's rule), through application-wide `QShortcut`s. It sends `MOTOR_STOP` three times,
  zeroes the sliders and disables the motors.
- On window close: stop if enabled, then `station.close()`. This replaces the page's
  `beforeunload` and the server's "last client left → release".
- The thrust limit defaults to 30 %. The readback bars draw the telemetry, never the slider.

## 5. UI: a one-to-one map of the page

The window is a vertical `QSplitter`: the header on top, then the three-column grid (motors
340 px | camera stretch | sensors 400 px, 12 px gaps), with the dock below it. The splitter
handle is the dock's drag handle. Size, tab and collapsed state persist in `QSettings`, which
replaces `localStorage`. The minimum window width is about 1100 px; the page's phone/tablet
stacking layout is dropped. If the window height is below 860 px, the motor hint hides and the
gauges shrink, as the page's `max-height` media query does.

- **Header**: the ICARUS wordmark with a green I, the subtitle with a papaya dot, and the 3 px
  papaya stripe with its angled end. Then the Wi-Fi button and its note, the pills (WI-FI, LINK,
  FAILSAFE, STA, CAM, DEMO DATA), the ⛶ full-screen button and the chamfered STOP button.
  `Pill` is a painted widget: a 3 px state-coloured left border and a cut corner. The states
  are ok, warn, crit, live, idle and demo.
- **Panel**: a painted frame with the 14 px chamfered top-right corner and the 2 px green top
  line, an uppercase title and a head note.
- **Motors**: `TopView` (the envelope ellipse and gondola, front arrows that scale with
  readback, rear rings that grow), four tower rows (name, slider −100…100 with double-click to
  zero, readback value, 20-segment bar, green, or blue when reversed). Controls: a `Switch`
  (painted toggle) for "Motors enabled", the limit spinbox,
  "Zero all", and the hint. Since 2026-10-27 each slider row has − and + step buttons at its ends
  (one slider unit, 1 % of the limit, auto-repeat), and the "Move pairs together" switch, which
  links FL with FR and RL with RR, sits in its own row above the sliders and starts off: it had
  been on by default among the controls below, where it went unnoticed and made one slider look
  like it moved two motors.
- **Camera**: `CameraView` (the 160×120 frame scaled without smoothing and letterboxed; the
  green box, papaya crosshair and range label from the row; rotate 180°; the "no frame yet"
  text and the frame stamp). Buttons: Connect/Disconnect, Grab frame, Live (papaya when on),
  Rotate 180°, baud combo and Set. The side column has the verdict tile
  (TARGET / NO TARGET / NO DATA), four readouts, `FovPlot`, the blob table (`QTableWidget`
  styled like the page) and the link health line.
- **Sensors**: the missing and paused notices (amber left border). The IMU grid has two
  `Sparkline`s (10 s history, auto or symmetric range, grid, "−10 s/now" labels, the last-point
  dot and a hover crosshair with a tooltip), and these two are the flexible parts. Then attitude
  (roll and pitch from the accelerometer, IMU temperature), `Compass` (fixed dial, red/grey
  needle, heading text), `TofGauge` (0–3 m, hover band 1.85–2.25 m shaded, fill, papaya line,
  status text), barometer readouts and the magnetometer row.
- **Dock**: the tab bar (CONSOLE, then EVENTS with a "n crit · n warn" badge and a papaya dot
  when unread), the context note and the ▾/▴ collapse button. The console has a mono output, an
  `i2c-tools>` prompt, an input with ↑/↓ history, Send, and the quick-command chips; one
  command runs at a time, with the 320 s safety reset. The events list has time, a coloured
  level and the text; crit rows get a red tint; 200 entries at most.

The logic in `app.js` is ported as is: `onTel`, `onSen`, `onCamRow`, `renderCameraSide`,
`onWifi` (button text and classes become the Qt property `state` plus a QSS re-polish),
staleness (no telemetry for 1.5 s → STALE, sensors paused for 1.5 s, camera rows 2 s), the
roll/pitch formulas and the ToF status names. The JS `DemoFeed` is not ported: `--demo` already
runs `DemoBlimp` and `DemoCamera` in-process.

**Styling.** `theme.py` holds the tokens (`BG #0C0F0D`, `PANEL #141A16`, `RAISED #1C2420`,
`RAISED_2 #243029`, `BORDER #26302A`, `FG/MUTED/FAINT`, `UN_GREEN #94B43B`, `UN_RED #DC313A`,
`AMBER #F2C14E`, `PAPAYA #FF8000`, series `#789E24/#2E9ED0/#D96D00`, `GRID #1E2622`). One QSS
sheet built from them covers buttons, `.primary`, the toggle pressed state, the disabled state,
inputs, the combo, the slider, the table, the scrollbars and the tabs. QSS cannot draw the page's
pentagon slider thumb, so the slider gets a flat light thumb. The painted widgets read the same
constants. The fonts load with `QFontDatabase.addApplicationFont`; if a file is missing, the app
falls back to Segoe UI / Consolas or DejaVu.

## 6. Docs and packaging

- `requirements.txt` (ground station): `PyQt6==6.11.0`, `pyserial==3.5`; drop tornado and
  pywebview. The comment in `software/requirements.txt` gets updated.
- `software/ground_station/README.md`: install becomes `pip install -r …` on every OS, with no
  apt WebKit, no WebView2 and no conda caveat. Usage text changes "page" to "window"; the Files
  table is updated.
- `docs/ground-station-plan.md`: §2 records the switch to PyQt6, pointing here; §4 becomes the
  Station signal table; §8 gets a phase row; §9 updates the run lines.
- `CLAUDE.md`: in the "Python tools" paragraph, replace the Tornado/HTML/pywebview description
  and "keep new UI inside `web/app.css`'s tokens" with "inside `theme.py`'s tokens"; the
  sensor-stream paragraph's pywebview mention is removed.

## 7. Order of work

1. `theme.py` + fonts + `app.py` skeleton: the header and empty panels, `--demo` starts.
2. `station.py` (port of Hub) and the event log/console dock, which proves the signal plumbing.
3. Motors panel and the safety timer/STOP.
4. Sensors panel with Sparkline, Compass, TofGauge.
5. Camera panel with CameraView, FovPlot, blob table.
6. Wi-Fi button and pill, full screen, QSettings persistence, close handling.
7. Delete `server.py`, `window.py` and `web/`; update requirements, README, plan doc and
   CLAUDE.md.

## 8. Verification

Everything runs in the `blimp-tools` conda env, with `PYTHONNOUSERSITE=1`. PyQt6 is expected
to be there already, since `drone_gui.py` needs it; that has not been checked yet.

1. Import smoke test with no display:
   `QT_QPA_PLATFORM=offscreen python -c "import sys; sys.path.insert(0,'software'); import ground_station.app"`.
2. Offscreen demo run: a scratch script builds the window with `--demo`, spins the event loop
   for about 5 s, enables motors, moves a slider, toggles camera Live, sends `help` on the
   console, then saves `window.grab()` to a PNG. That screenshot gets checked against the page's
   layout and colours.
3. Deadman check in the same script: enable motors with a non-zero slider, stop the 100 ms
   refresh timer, and confirm the event "setpoint … went stale … motors released" plus readback
   back to 0 (the mock blimp reports it).
4. Over a real UDP link with no hardware: `python software/mock_blimp.py` in one terminal and
   `python -m software.ground_station --blimp 127.0.0.1` in another; telemetry, sensors and the
   console come through the real sockets.
5. `python -m software.ground_station --demo` on screen, compared by eye against the old
   page, which is in git history.
6. On the bench with the S3 (physical steps by the user, one at a time): Join blimp Wi-Fi, LINK
   pill live, sensors streaming, console `wifi`, motors with the propellers off, STOP with
   Space/Esc. The Windows laptop is the other target. `wifi.py`'s netsh path is unchanged, so
   only the window needs checking there.
