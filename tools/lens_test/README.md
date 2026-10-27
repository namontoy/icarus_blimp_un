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
| `a` | measure: one full-size still; printed markers are found by themselves, handwritten marks are clicked left to right; field of view, distortion verdict, vignetting and sharpness all from that still |
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

**Fast path, about a minute per module, the board never moves.** Big marks on
the wall every 25 cm: either the printed ArUco pages from `lens_test.py markers`
(taped above the tape measure, then nothing is clicked at all) or big handwritten
numbers on the tape's 0, 25, 50 ... cm marks. Board on a marked spot, facing the
wall, blank wall above the marks. Per module: `a`; the first time, type the
lens-to-wall distance (remembered from then on); with handwritten marks click
every big mark you can see from left to right and press Enter; read the one-line
result; `q`. The field of view is fitted to all the marks (pinhole and
fisheye models), the straight-line residual is the distortion verdict, the blank
wall above gives the vignetting and the marks' print the sharpness. `i` with a
TV remote adds the IR-filter answer when wanted (lights can stay on).

