# Ground station

One browser page for the blimp: motors, sensors, the camera and the console, served by a small
Python back end that talks to the S3 firmware over its Wi-Fi access point and to the ESP32-CAM
over serial. Design, message formats and the safety chain: `docs/ground-station-plan.md`.

```
pip install tornado pyserial                   # once; pyserial only for the camera

python -m software.ground_station --demo       # no hardware: mock blimp + demo camera, from the repo root
python -m software.ground_station              # the real blimp (join its Wi-Fi first), no camera
python -m software.ground_station --cam COM5   # plus the camera on a serial port (/dev/ttyUSB0 on Linux)
python -m software.ground_station --bind 0.0.0.0   # also lets a tablet on the same Wi-Fi open the page
```

It prints `http://127.0.0.1:8765/` and opens it. The page also runs with no back end at all
(open `web/index.html` from disk, or add `?demo` to the address): it then shows demo data and
says so with an orange DEMO DATA pill. That copy of the page never reaches a blimp.

## Using it

- **STOP**: the orange button, Space or Esc. Coasts all four motors and switches the motor
  controls off. Always available, even while the console is busy.
- **Motors**: tick *Motors enabled*, then move a slider; *Move pairs together* links FL with
  FR and RL with RR. The limit (30 % by default) clamps every slider. The segmented bars and the
  arrows on the top view are the blimp's readback, not the slider. The page repeats the
  setpoint 10 times a second; the back end stops repeating it 0.5 s after the page goes quiet,
  and the firmware coasts on its own timers after that. Remove the propellers on the bench.
- **Sensors**: live numbers, 10 s plots of the accelerometer and gyroscope (hover for values),
  roll and pitch from the accelerometer, the magnetic heading, the range below with the
  1.85–2.25 m hover band, and the barometer. The firmware streams them from boot (`sensors` in
  the console shows what it found). A console sensor command (`imu`, `mag`, `baro`, `tof`)
  borrows the chips while it runs, so the panel shows a "paused" notice after 1.5 s without a
  packet and resumes when the command returns. The barometer's altitude is relative to its
  first reading since the blimp booted.
- **Camera**: Connect, Grab frame, Live, Rotate 180° (display only), and the baud rate. The
  frame is the camera's 160 × 120 pixels at four times with the detector's box, the verdict
  tile, the blob table and the field-of-view plot come from the camera's own CSV rows.
- **Console** and **Events** share the dock under the three panels, as two tabs. Drag the
  handle on the dock's top edge to set its height (double-click resets it), and the ▾ button
  collapses it to the tab bar. Height, tab and collapsed state are remembered by the browser.
  The three panels above take whatever height is left, so at 1080p nothing needs a scrollbar;
  on a short screen, collapse the dock or let a panel scroll.
- **Console**: any firmware command; Enter sends, ↑/↓ recall history, the chips are the usual
  ones. One command runs at a time, on the air and on the blimp.
- **Events**: link up and down, failsafe trips, stale setpoints, camera link changes. The tab
  shows the crit and warn counts, and a papaya dot while there is something unread.

## Files

| File | What |
|---|---|
| `server.py` | Tornado: static page, WebSocket, the 10 Hz push, the console worker |
| `blimp_bridge.py` | `blimp_link.BlimpLink` and `BlimpConsole` with the setpoint deadman; `DemoBlimp` runs `mock_blimp.py` in-process |
| `camera.py` | `SerialCamera` (the target_cam viewer's parser over pyserial), `DemoCamera`, PNG and RGB565 helpers |
| `web/index.html`, `web/app.css`, `web/app.js` | the page: no framework, no build step |

The camera parser is `tcview/protocol.py` from `firmware/projects/target_cam/host/viewer`, which
is on the `target-cam` branches; until that merges, `--cam COM5` reports the parser as missing
and `--cam demo` is the option.
