# Lens test: telling camera modules apart on an ESP32-CAM

A bench kit for sorting the pile of OV2640 (and other 24-pin) camera modules. The
marking on a module's flex names the flex, not the lens: the same marking is sold
with 68° and 120° lenses, with and without an infrared filter. So each module is
plugged into one ESP32-CAM and measured, and the result is filed as a **lens card**
that the physical module gets labelled with.

Two parts:

| Part | What it does |
|---|---|
| `esp32cam/` | ESP-IDF firmware for the AI-Thinker ESP32-CAM. Probes whatever sensor is plugged in and serves stills, a live stream and the sensor controls over Wi-Fi. |
| `lens_test.py` | PC tool. Live view with numbers on screen, hotkeys that measure field of view, sharpness, vignetting, distortion and IR response, and a `table` command that summarises every card. |

Nothing here is flight code; the controller lives in `core/` and never depends on
this directory.

## One-time setup

Hardware: an AI-Thinker ESP32-CAM with its ESP32-CAM-MB USB adapter (or any
3.3 V USB-serial adapter wired to U0R/U0T with IO0 pulled to GND while flashing),
a USB cable that can carry 500 mA, and the modules to sort.

1. Build and flash the firmware (needs the ESP-IDF v5.3 install described in
   `docs/08-firmware-setup.md`; the first build downloads the camera driver, so
   it needs internet once):

   ```
   tools\lens_test\build.bat                  # build
   tools\lens_test\build.bat "-p COM3 flash"  # flash; substitute the port
   ```

   The board must be in download mode first, because the USB-C adapters on the
   bench have no auto-reset wiring (the flash config therefore uses
   `--before no_reset`): hold **IO0**, press and release **RST**, release IO0.
   After flashing, press **RST** once to start the new firmware. "Invalid head
   of packet" from esptool means the board was not in download mode. The
   original ESP32 has no software way into download mode, so this is needed on
   every flash.

2. Install OpenCV into the project venv (once):

   ```
   .venv\Scripts\python -m pip install -r tools\lens_test\requirements.txt
   ```

3. Optional: `tools\lens_test\build.bat menuconfig`, menu "Lens test", to make
   the board join your router instead of creating its own network.

## Running

Two ways to reach the board. **Serial** (recommended): the same USB cable used
for flashing, `--port COM3`; the laptop keeps its own Wi-Fi and internet. The
firmware prints its boot log at 115200 baud and then switches UART0 to 460800
for the tool (the "Lens test" menu sets the rate; `idf.py monitor -b 460800`
still shows the log). The generic USB-C adapter on the bench cannot do 921600. **Wi-Fi**: join the network **lens-test**, password
**lenstest1**, and use `--host 192.168.4.1` (the default). `http://192.168.4.1/`
also shows the live stream in a browser or on a phone, handy for aiming.

1. Power the board, plug in a module (power off while swapping: the sensor is
   not hot-pluggable).
2. Check the sensor was recognised:

   ```
   .venv\Scripts\python tools\lens_test\lens_test.py --port COM3 info
   ```

   `camera_ok: false` means nothing answered on the camera bus: power off and
   reseat the flex with the gold contacts toward the board and the latch closed.

3. Start the live view with the label you will write on the module:

   ```
   .venv\Scripts\python tools\lens_test\lens_test.py --port COM3 live --label A
   ```

   The window shows the stream with the sharpness, corner/centre brightness and
   peak brightness computed live, the regions they are computed over, and the
   measurements already on the card. Hotkeys are pressed with the **video window**
   focused; anything the tool asks (a label, a distance) is typed in the
   **terminal** the script was started from, so keep both visible.

Hotkeys:

| Key | Does |
|---|---|
| `a` | one shot: field of view, distortion, vignetting and sharpness from one full-size still (three clicks on the tape) |
| `n` | start a new card (asks for the label); `t` adds a free-text note |
| `s` | save a full-size still to the card folder |
| `f` | field of view: one full-size still, two clicks, two numbers typed |
| `d` | distortion: one still, three clicks along a straight edge |
| `v` | record vignetting from the live stream (white wall) |
| `k` | record sharpness from the live stream (asks the distance) |
| `i` | record the IR peak from the live stream (TV remote) |
| `l` | toggle the sensor's lens correction (LENC) |
| `e` | lock or unlock auto-exposure and gain |
| `c` | toggle the sensor's colour-bar test pattern |
| `h`, `q` | help, quit (the card is saved after every change) |

## Measuring protocol

**Fast path, about a minute per module, the board never moves.** Put the board on a
marked spot facing the wall so the tape runs along the *bottom* edge of the
picture with blank wall above it, big marks readable. Per module: `a` (Enter
accepts the remembered distance, click the tape at its left end, middle and
right end, Enter accepts the remembered marks difference), `i` (keep the remote
off until the band says to press it, then hold a button 20 cm from the lens;
the lights can stay on), `q`. The IR test compares frames with the remote off
and on at a locked exposure, so a bright room does not matter.

The detailed path below measures the same things one at a time, with the
vignetting on a clean white wall and the sharpness on a text sheet; use it for
the modules that make the shortlist.

### Detailed path, about five minutes per module

Set the bench up once: a plain white wall at a measured distance (1.00 m is
convenient), a tape measure stuck horizontally across it, a printed text sheet
you can hold at the wall, a TV remote, and a door frame or table edge in reach.

1. `n`: label the card, then `t`: note the flex marking and the housing
   (for example "DCX-OV2640-V2, metal housing").
2. **Field of view** (`f`). Board square to the wall, lens at the measured
   distance. Type the distance, click the tape near the left and right edges of
   the picture, type the difference between the two tape readings. The tool
   reports the horizontal and vertical fields under two lens models: pinhole
   (right for the standard lens) and equidistant (right for fisheyes). The
   closer the clicks are to the edges, the less the model matters.
3. **Distortion** (`d`). Aim so a straight edge runs close to one border. Click
   one end, the middle, the other end. Barrel distortion is positive.
4. **Vignetting** (`v`). Fill the frame with the white wall, press `v`, press
   `l` to toggle LENC, press `v` again. Both states are kept.
5. **Sharpness** (`k`). Text sheet in the centre at 1 m, press `k`; repeat at
   3 m and 5 m if the module is a candidate. Only compare modules on the same
   sheet at the same distance and stream size.
6. **IR response** (`i`). Dim the room, press `e` to lock exposure, hold the
   remote 20 cm from the lens with a button pressed, press `i`. A peak of 250 or
   more means no IR-cut filter: a "night" module, which turns red balloons pink
   in daylight.
7. `s` for a reference still, then power off, swap modules, reset, `n`.

When all modules are done:

```
.venv\Scripts\python tools\lens_test\lens_test.py table --markdown
```

## Reading the numbers

| Lens class | Horizontal field | Bow of a straight edge | Corner/centre |
|---|---|---|---|
| standard ("66°/68°") | about 52° | under 1 % | above 0.7 |
| wide ("120°") | about 105° | a few % | roughly 0.4 to 0.7 |
| fisheye ("160°") | 130° and more (equidistant) | large, corners may be black | low or zero in the corners |

The class column in `table` applies these thresholds. Field of view and the
pixel focal length `f_px` go straight into the perception calibration; the
simulator's camera is set by `FOV_HALF_ANGLE_RAD` and `FOV_VERT_HALF_ANGLE_RAD`
in `sim/sim_main.py` (half of the measured fields).

## Files

```
build/lens_test/<label>/card.json     the lens card (JSON, one per module)
build/lens_test/<label>/fov.jpg       the still the field of view was clicked on, with the clicks
build/lens_test/<label>/distortion.jpg
build/lens_test/<label>/still_N.jpg
```

`build/` is gitignored. Paste the `table --markdown` output into
`docs/10-known-issues.md` when the choice is made, with the distance and
stream size used.

## Firmware protocol

Over serial (UART0 at the configured rate), one text command per line:

| Command | Reply |
|---|---|
| `ping` | `PONG` |
| `quiet` | `OK`; silences the board's log output (the tool sends it first) |
| `info`, `status` | `JSON {...}` |
| `set lenc 0` | `OK` or `ERR <why>`; same variables as `/set` below |
| `cap MAX` | `FRM <bytes> <w> <h>`, then the JPEG bytes, then `END` |

Over HTTP:

| URL | Returns |
|---|---|
| `/` | browser page: stream, LENC and AE buttons, full-size still link |
| `/info` | JSON: sensor name, PID/VER/MIDH/MIDL, max size, hardware JPEG or not |
| `/status` | JSON: every sensor setting |
| `/set?var=lenc&val=0` | any esp32-camera setter: `framesize`, `quality`, `brightness`, `contrast`, `saturation`, `awb`, `aec`, `agc`, `aec_value`, `agc_gain`, `gainceiling`, `lenc`, `hmirror`, `vflip`, `colorbar`, ... |
| `/capture?size=MAX` | one JPEG; `size` is `MAX`, a name (`VGA`, `UXGA`) or `WxH` |
| `:81/stream?size=VGA` | MJPEG stream (its own server, so the others keep answering) |

Any sensor the esp32-camera driver knows is accepted (OV2640, OV3660, OV5640,
OV7670, OV7725, GC-series, ...). Sensors without a JPEG encoder are served
through a software conversion, slowly.

## Troubleshooting

- **The board resets with "Brownout detector was triggered"**: the USB supply
  sags when Wi-Fi transmits. Use a shorter, thicker cable or a powered hub.
- **"Detected camera not supported" / `camera_ok: false`**: the module is not
  seated, is in backwards, or is a sensor the driver does not know. Reseat it
  with power off.
- **Full-size stills fail or time out** (large sensors, low quality numbers):
  the JPEG outgrew its buffer and the frame was dropped. Ask for less data with a
  larger quality number, which is worse quality in esp32-camera's inverted
  scale, for example `/set?var=quality&val=12`.
- **The live view stalls after a still**: the board switches the sensor to full
  size and back; a second or two of freeze is normal. Press `q` and restart if
  it does not recover.
- **Numbers jump between frames**: lock exposure (`e`) and keep the light steady.
