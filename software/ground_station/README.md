# Ground station

One window for the blimp in a single PyQt6 process: a Flight page (motors, sensors, the
camera), a Control page (flying by hand from the keyboard or a gamepad, with a PC and a Steam
Deck input mode; `control.py`, 2026-10-28), a Bench page (captures of the control loop's
records, with their traces; `bench.py`, 2026-10-27) and the console under all three. It talks to the S3 firmware over its Wi-Fi access
point and to nothing else. The
cameras are wired to the S3, which will relay what they extract (detections, optical flow);
the link has no packet for that yet, so the camera panel runs on demo data only. Design: `docs/ground-station-native-plan.md`; signals and the safety chain:
`docs/ground-station-plan.md`.

```
pip install -r software/ground_station/requirements.txt   # once, on any OS

python -m software.ground_station --demo       # no hardware: mock blimp + demo camera, from the repo root
python -m software.ground_station              # the real blimp; "Join blimp Wi-Fi" is in the header
```

The full-screen button in the header (four corner brackets) or F11 toggles full screen. In
`--demo` the blimp is `mock_blimp.py` on localhost and the camera is drawn, and the header says
so with an orange DEMO DATA pill. Ctrl-C in the terminal closes the window, which stops the
motors first. Window size, dock height, tab and collapsed state are kept by Qt's QSettings
(`~/.config/Icarus/GroundStation.conf` on Linux, the registry on Windows).

## Installing

Python 3.9 or newer (3.12 on the Windows laptop; the 3.10 conda environment of
`software/SETUP.md` also works) and the repo cloned with the `s3-wireless-link` branch
checked out. Everything the ground station needs is pinned in
`software/ground_station/requirements.txt`; the esp-drone tools' `software/requirements.txt`
includes it, so one `pip install -r` of either file covers it. Install into the Python you will
run with: inside a virtual environment, `python` is that environment's interpreter. The PyQt6
wheel carries Qt itself, so there is nothing to install from the system on any OS, and the
fonts (Titillium Web, IBM Plex Mono, both OFL) ship in `fonts/`.

### Windows 10 and 11

```
cd C:\path\to\icarus_blimp_un
python -m pip install -r software\ground_station\requirements.txt
python -m software.ground_station
```

- The Wi-Fi button needs no administrator rights. Windows 11 withholds the list of networks
  from console programs unless location services are on, so the button joins by name: the
  profile Windows kept from a manual join, or `--wifi-ssid ICARUS-XXXXXX` (the suffix is in the
  blimp's boot log and its `wifi` command). The profile the station creates is manual-connect,
  so Windows never jumps to the blimp by itself.

### Linux (Debian 12, Ubuntu 22.04 or newer)

```
cd ~/icarus_blimp_un
python3 -m pip install -r software/ground_station/requirements.txt
python3 -m software.ground_station
```

- The `blimp-tools` conda environment of `SETUP.md` already has PyQt6, which is all it needs.
- The Wi-Fi button needs NetworkManager (`nmcli`) managing the Wi-Fi interface, which a
  desktop install has. It scans for the strongest `ICARUS-…`, joins with the key, then turns
  autoconnect off so the laptop does not keep going back to the blimp. A local session is
  normally allowed to do this without a password.

### macOS

`pip install -r` works the same way, but the Wi-Fi button is disabled: join `ICARUS-…` from
the menu bar. Not tried.

### Steam Deck (SteamOS)

The Deck is a Linux computer with a gamepad built in, so the Linux lines above apply: in
Desktop mode, from a terminal, install into a virtual environment (SteamOS's root filesystem is
read-only) and run `python3 -m software.ground_station --mode deck`. The Wi-Fi button goes
through NetworkManager, which SteamOS runs. The Deck's controls reach the window through
`gamepad.py`, which reads the kernel's event devices and takes the first one that has a
gamepad's buttons; the `hid-steam` driver presents them that way and stops their keyboard-and-
mouse emulation while the device is held open. None of this has run on a Deck yet
(2026-10-28): the event parsing is checked with synthetic events, the Windows side with XInput.
`python software/ground_station/gamepad.py` prints what the reader sees and is the first thing
to run there; permission denied on `/dev/input` means the user needs the `input` group or a
udev rule. Launching the station from Gaming mode as a non-Steam game is another matter: Steam
Input would then present a virtual Xbox pad, which the same reader takes, but the window has
not been tried under gamescope.

### Windows and Linux side by side

| | Windows | Linux |
|---|---|---|
| Window | PyQt6 from pip | PyQt6 from pip |
| Wi-Fi join | `netsh wlan connect` by profile name; a profile is created from the key if none exists | `nmcli dev wifi connect`, the strongest `ICARUS-…` in range |
| Finding the SSID | stored profile or `--wifi-ssid`; a scan only with location services on | a scan |
| Leaving | `netsh wlan disconnect` | `nmcli connection down`; autoconnect was turned off |
| Rights | none | NetworkManager's rules; a local desktop session normally passes |
| Full screen | header button or F11 | header button or F11 |
| Wi-Fi join tested | 2026-10-24, this laptop (web version; `wifi.py` is unchanged) | not yet; written from `drone_gui.py`'s nmcli flow |

## Joining the blimp's Wi-Fi from the window

**Join blimp Wi-Fi** in the header makes this computer join the `ICARUS-…` access point and
the WI-FI pill shows its address on that network (192.168.4.x); the button then reads
**Leave**. The join is done by the station (`wifi.py`, on its own thread): `netsh wlan` on Windows, `nmcli` on
Linux as `software/drone_gui.py` does it, and the pill is judged from the computer's own
address toward the blimp, so a network joined by hand shows as joined too.

- **Windows 11** no longer lets a console list networks or report the current SSID unless
  location services are on for console apps, so the station never relies on a scan. It joins
  by profile name: a profile left by a manual join, or `--wifi-ssid ICARUS-XXXXXX` (the suffix
  is in the blimp's boot log and in its `wifi` command). With neither, the window says so. The
  profile it creates is set to manual connection, so Windows never jumps to the blimp on its
  own; `--wifi-password` must match the key in the firmware's menuconfig.
- **Linux**: scans for the strongest `ICARUS-…`, connects, then turns autoconnect off.
- Anything else: the button is disabled and the window asks you to join by hand.
- Leaving the blimp's network means leaving the internet with it, on a laptop with one
  radio; the station does not reconnect the previous network.

## Looking at it without hardware

`--demo` runs the mock blimp and the demo camera in-process, so every page carries live-looking
data on any computer. `--geometry WxH` opens the window at a size instead of the remembered
one, and `--page` on a page, so the Steam Deck's view can be checked on a laptop:

```
python -m software.ground_station --demo                                          # as on a laptop
python -m software.ground_station --demo --mode deck --geometry 1280x800 --page control
python -m software.ground_station.screens                                         # PNGs of every page, both sets
```

`screens.py` renders the window offscreen (no display needed) at 1680 × 1000 with PC input and
at 1280 × 800 with Steam Deck input, three pages each, into `build/screens/`; the pixels are
what Qt draws with the bundled fonts, so they are honest pictures for a report. It uses a
throwaway settings file, so it never changes the remembered window.

## Using it

- **STOP**: the orange button, Space or Esc. Coasts all four motors and switches the motor
  controls off. Always available, even while the console is busy. Esc works from any field;
  Space is left to the console line and the limit box while one has the focus.
- **Motors**: tick *Motors enabled*, then move a slider or step it with its − and + buttons (1 % of
  the limit per click; hold to repeat). Each slider drives its own motor unless *Move pairs
  together* (above the sliders, off at start) is on: then FL moves FR and RL moves RR. The limit (30 % by default) clamps every slider. The segmented bars and the
  arrows on the top view are the blimp's readback, not the slider. The window repeats the
  setpoint 10 times a second; the bridge stops repeating it 0.5 s after the window goes quiet,
  and the firmware coasts on its own timers after that. Closing the window stops the motors. Remove the propellers on the bench.
- **Sensors**: live numbers, 10 s plots of the accelerometer and gyroscope (hover for values; every
  packet the blimp sends is plotted, up to 20 a second),
  roll and pitch from the accelerometer, the magnetic heading, the range below with the
  1.85–2.25 m hover band, the barometer and the magnetometer. The firmware streams them from boot (`sensors` in
  the console shows what it found). A console sensor command (`imu`, `mag`, `baro`, `tof`)
  borrows the chips while it runs, so the panel shows a "paused" notice after 1.5 s without a
  packet and resumes when the command returns. The barometer's altitude is relative to its
  first reading since the blimp booted.
- **Camera**: the verdict tile, bearing, elevation, range, confidence, the blob table and the
  field-of-view plot come from the forward camera's detection rows (target_cam's CSV columns).
  Only `--demo` fills them today; the real rows will come through the S3 once the link has a
  packet for them. The picture view with Grab frame, Live and Rotate 180° (display only) is
  kept for the occasional calibration frame the S3 may relay later (`docs/inter-board-link-plan.md`);
  outside the demo the two buttons are disabled.
- **Control** (the second tab under the header): fly by hand. Pick the input, *PC* (the
  keyboard, plus a gamepad if one is plugged in) or *Steam Deck* (the Deck's own controls; the
  hints leave the screen to fit 1280 × 800), and the stick layout (mode 2: left stick heave and
  yaw, right stick surge; mode 1 swaps surge and heave). Switch *Manual control* on, or press A
  on the pad: arming needs telemetry and the sticks at rest, and it switches the Flight page's
  *Motors enabled* off (one source drives at a time). Then W/S is surge, A/D or ←/→ yaw, ↑/↓
  heave (half a stick; hold Shift for the whole), or the sticks. The page mixes surge, yaw and
  heave into the four motors the way `core/`'s mixer is laid out (front pair sum and
  difference, rear pair heave; open loop, per motor, which is what the firmware's MANUAL mode
  takes) and sends them 20 times a second while armed, ramped over 0.4 s so nothing jumps; the
  limit (30 % by default; D-pad ↑/↓ on the pad) scales everything. Let go and the command ramps
  to zero. STOP (button, Space, Esc, B on the pad) disarms and coasts; so does a gamepad that
  disappears while driving, and the Motors panel's deadman chain runs underneath. *Go
  autonomous* is the hand-off to `blimp_step()` on the blimp; it stays disabled until the
  firmware has its mode switch (`MODE_SET`, `docs/control-loop-plan.md` increment 3). `--mode
  pc|deck` sets the input from the command line; the input, the layout and the limit are
  remembered.
- **Bench** (the third tab under the header): the blimp's capture. Set a length (the box is
  bounded by the blimp's own buffer, 400 s by default, once the page has asked it) and press
  *Start*; the blimp records one 72-byte record per 20 ms of its control loop into PSRAM (IMU,
  magnetometer and barometer, the range, the motor readback, the loop's execution time, which
  sensors were fresh). *Download* fetches it over the link in chunks (the ones that are lost
  are asked for again), saves `captures/capture-<date>.bin` with a `.json` beside it (the
  header, your notes, the summary figures) and shows the traces: pick a group and a time
  window, hover for the values at a sample; *Export CSV…* writes every record in units.
  `python software/blimp_log.py <file>` reads the same files. In `--demo` the mock records
  its toy airframe, so the page can be tried without a blimp.
- **Console** and **Events** share the dock under the three panels, as two tabs. Drag the
  handle on the dock's top edge to set its height (double-click resets it), and the ▾ button
  collapses it to the tab bar. Height, tab and collapsed state are remembered between runs.
  The three panels above take whatever height is left, so at 1080p nothing needs a scrollbar;
  on a short screen, collapse the dock or let a panel scroll.
- **Console**: any firmware command; Enter sends, ↑/↓ recall history, the chips are the usual
  ones. One command runs at a time, on the air and on the blimp.
- **Events**: link up and down, failsafe trips, stale setpoints, camera link changes. The tab
  shows the crit and warn counts, and a papaya dot while there is something unread.

## Files

| File | What |
|---|---|
| `app.py` | `main()`, the options, the header, the window: panels, dock splitter, keys, settings |
| `station.py` | the back end in the GUI process: bridges, camera, Wi-Fi and console as Qt signals |
| `panels.py` | the Motors, Camera and Sensors panels and the Console/Events dock |
| `control.py` | the Control page: input modes, arming, the stick-to-motor mix, the keys, the pad's buttons |
| `gamepad.py` | the gamepad reader: XInput on Windows, the kernel's event devices on Linux and the Steam Deck; `python gamepad.py` prints what it sees |
| `bench.py` | the Bench page: the capture panel and the trace viewer |
| `screens.py` | renders every page to PNG offscreen, PC and Steam Deck sets, for a look without hardware |
| `widgets.py` | the painted pieces: top view, compass, ToF gauge, plots (the sparklines and the capture's trace plot), camera view, pills, STOP, the stick pads |
| `theme.py` | the palette's tokens, fonts and the Qt style sheet; new UI keeps to these |
| `fonts/` | Titillium Web and IBM Plex Mono with their OFL licences |
| `blimp_bridge.py` | `blimp_link.BlimpLink` and `BlimpConsole` with the setpoint deadman; `DemoBlimp` runs `mock_blimp.py` in-process |
| `camera.py` | `DemoCamera` (rows and frames) and the empty source used until the S3 relays camera data |
| `wifi.py` | joins the blimp's access point: `netsh wlan` on Windows, `nmcli` on Linux; `python wifi.py status` from the command line |

The web version (Tornado, an HTML/CSS/JS page, pywebview) it replaced is in git history up to
commit 57eebe9. Both versions also had `--cam PORT`, a reader for an ESP32-CAM plugged into this
computer over USB serial (pyserial and the target_cam viewer's parser); it was removed on
2026-10-27 because the cameras talk only to the S3, and it is in the same history.
