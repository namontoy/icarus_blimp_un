# Ground station

One browser page for the blimp: motors, sensors, the camera and the console, served by a small
Python back end that talks to the S3 firmware over its Wi-Fi access point and to the ESP32-CAM
over serial. Design, message formats and the safety chain: `docs/ground-station-plan.md`.

```
pip install -r software/ground_station/requirements.txt   # once; see "Installing" for your platform

python -m software.ground_station --demo       # no hardware: mock blimp + demo camera, from the repo root
python -m software.ground_station              # the real blimp, no camera; "Join blimp Wi-Fi" is in the header
python -m software.ground_station --cam COM5   # plus the camera on a serial port (/dev/ttyUSB0 on Linux)
python -m software.ground_station --browser    # the system browser instead of the window
python -m software.ground_station --bind 0.0.0.0   # also lets a tablet on the same Wi-Fi open the page
```

With pywebview installed it opens as its own window (WebView2 on Windows, WebKitGTK on
Linux): no tabs or address bar, and ⛶ or F11 goes full screen. Without it, or with
`--browser`, it prints `http://127.0.0.1:8765/` and opens the browser. The page also runs
with no back end at all (open `web/index.html` from disk, or add `?demo` to the address): it
then shows demo data and says so with an orange DEMO DATA pill. That copy never reaches a
blimp.

## Installing

Python 3.9 or newer (3.12 on the Windows laptop; the 3.10 conda environment of
`software/SETUP.md` also works) and the repo cloned with the `s3-wireless-link` branch
checked out. Everything the ground station needs is pinned in
`software/ground_station/requirements.txt`; the esp-drone tools' `software/requirements.txt`
includes it, so one `pip install -r` of either file covers it. Install into the Python you will
run with: inside a virtual environment, `python` is that environment's interpreter, and if the
browser opens instead of a window the terminal names the interpreter that lacks pywebview.

### Windows 10 and 11

```
cd C:\path\to\icarus_blimp_un
python -m pip install -r software\ground_station\requirements.txt
python -m software.ground_station
```

- The window is Microsoft's WebView2, which Windows 11 ships. On Windows 10, if no window
  appears, install the "WebView2 Evergreen runtime" from Microsoft once.
- The Wi-Fi button needs no administrator rights. Windows 11 withholds the list of networks
  from console programs unless location services are on, so the button joins by name: the
  profile Windows kept from a manual join, or `--wifi-ssid ICARUS-XXXXXX` (the suffix is in the
  blimp's boot log and its `wifi` command). The profile the page creates is manual-connect,
  so Windows never jumps to the blimp by itself.
- Serial ports are `COM5` and the like (Device Manager, Ports): `--cam COM5`.

### Linux (Debian 12, Ubuntu 22.04 or newer)

```
sudo apt install python3-gi python3-gi-cairo gir1.2-gtk-3.0 gir1.2-webkit2-4.1
cd ~/icarus_blimp_un
python3 -m pip install -r software/ground_station/requirements.txt
python3 -m software.ground_station
```

- The window is WebKitGTK through PyGObject; the apt line provides both (older releases call
  the last package `gir1.2-webkit2-4.0`). Without it the page opens in the browser, which also
  works. The Qt alternative needs no apt access: `pip install "pywebview[qt6]"`.
- A conda environment does not see the system PyGObject, so the `blimp-tools` environment of
  `SETUP.md` gets the browser, or the Qt alternative installed into it.
- The Wi-Fi button needs NetworkManager (`nmcli`) managing the Wi-Fi interface, which a
  desktop install has. It scans for the strongest `ICARUS-…`, joins with the key, then turns
  autoconnect off so the laptop does not keep going back to the blimp. A local session is
  normally allowed to do this without a password.
- Serial ports are `/dev/ttyUSB0` or `/dev/ttyACM0`, and your user must be in `dialout`
  (`sudo usermod -aG dialout $USER`, then log in again).

### macOS

`pip install -r` works and the window is WebKit (pyobjc comes with pywebview), but the Wi-Fi
button is disabled: join `ICARUS-…` from the menu bar. Not tried.

### Windows and Linux side by side

| | Windows | Linux |
|---|---|---|
| Window | WebView2, in Windows 11 | WebKitGTK via `python3-gi` + `gir1.2-webkit2-4.1`, or `pywebview[qt6]` |
| Without it | the default browser | the default browser |
| Wi-Fi join | `netsh wlan connect` by profile name; a profile is created from the key if none exists | `nmcli dev wifi connect`, the strongest `ICARUS-…` in range |
| Finding the SSID | stored profile or `--wifi-ssid`; a scan only with location services on | a scan |
| Leaving | `netsh wlan disconnect` | `nmcli connection down`; autoconnect was turned off |
| Rights | none | NetworkManager's rules; a local desktop session normally passes |
| Camera port | `COM5` | `/dev/ttyUSB0`, user in `dialout` |
| Full screen | ⛶ or F11 | ⛶ or F11 |
| Tested | 2026-10-24, this laptop | not yet; written from `drone_gui.py`'s nmcli flow |

## Joining the blimp's Wi-Fi from the page

**Join blimp Wi-Fi** in the header makes this computer join the `ICARUS-…` access point and
the WI-FI pill shows its address on that network (192.168.4.x); the button then reads
**Leave**. The join is done by the back end (`wifi.py`): `netsh wlan` on Windows, `nmcli` on
Linux as `software/drone_gui.py` does it, and the pill is judged from the computer's own
address toward the blimp, so a network joined by hand shows as joined too.

- **Windows 11** no longer lets a console list networks or report the current SSID unless
  location services are on for console apps, so the back end never relies on a scan. It joins
  by profile name: a profile left by a manual join, or `--wifi-ssid ICARUS-XXXXXX` (the suffix
  is in the blimp's boot log and in its `wifi` command). With neither, the page says so. The
  profile it creates is set to manual connection, so Windows never jumps to the blimp on its
  own; `--wifi-password` must match the key in the firmware's menuconfig.
- **Linux**: scans for the strongest `ICARUS-…`, connects, then turns autoconnect off.
- Anything else: the button is disabled and the page asks you to join by hand.
- Leaving the blimp's network means leaving the internet with it, on a laptop with one
  radio; the back end does not reconnect the previous network.

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
| `wifi.py` | joins the blimp's access point: `netsh wlan` on Windows, `nmcli` on Linux; `python wifi.py status` from the command line |
| `window.py` | the page in its own window through pywebview; full screen on request |
| `web/index.html`, `web/app.css`, `web/app.js` | the page: no framework, no build step |

The camera parser is `tcview/protocol.py` from `firmware/projects/target_cam/host/viewer`, which
is on the `target-cam` branches; until that merges, `--cam COM5` reports the parser as missing
and `--cam demo` is the option.
