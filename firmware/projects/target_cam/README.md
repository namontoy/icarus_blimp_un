# target_cam — forward-looking colour-target detection on an ESP32-CAM

The blimp's nose camera. Its job in the IAR-LARC 2026 race is to find the
**red mylar waypoint balloons** (22", 50–60 cm, centre 2 m above the floor, four
of them on a 4 m × 2 m rectangle) so the airframe can steer from one to the
next. It reports up to **four blobs** of one colour, largest (usually nearest)
first, and streams them as CSV on UART0: bearing left/right and up/down,
apparent size, and range from the known 550 mm balloon size.

Four colour presets are built in, for bench testing with whatever balloons
are to hand: **red** (the competition colour, default), **blue**, **green**
and **orange**.

It is the sibling of [`optflow_cam`](../optflow_cam/README.md) and is built the
same way. The detector is portable C, verified on the host before any hardware.
The firmware never starts a radio, paces its frames, reports its own failures,
and keeps that project's board handling: a boot report before the camera powers
up, the GPIO0 reflash window, and the flash LED driven low.

> **Hardware status: first bench run 2026-10-13; not yet calibrated.**
> - **Measured on the bench (AI-Thinker ESP32-CAM on an ESP32-CAM-MB):** it
>   boots cleanly, holds a steady **10.0 FPS** (`dt_ms` ≈ 100), and detection
>   costs **≈ 8.2 ms** per frame. The RGB565 byte order is **big-endian**,
>   as the source said.
> - **Since 2026-10-14 the default pace is 12.5 FPS (80 ms), measured the same
>   day:** 751 rows in 60 s (12.505 rows/s), `dt_ms` 80.0 on every row, no
>   missing rows, detection 7.3–8.2 ms. Under `f30` the loop waits on the
>   sensor and runs at the same 12.505 FPS, so this is the sensor's own rate in
>   RGB565 and every frame is used. The 10 FPS figure above is from the old
>   default. On the camera's clock the period averages 79.97 ms, not 80.00;
>   why the 80 ms wait and the sensor agree that closely is not established.
> - **The red preset did not detect the real balloon at 1.5 m.** The centre
>   patch read hue 304 ± 109, sat 78, val 188: a wide hue spread and low
>   saturation, probably from the mylar mirroring the room. Loose thresholds
>   (`H3 T45 S35 V40`) found a 17×17 px blob. `H0 T30 S50 V40` also matched
>   wood clutter in the lower left of the view. Calibrating against the real
>   image is the next step; that is what the [PC viewer](host/viewer/README.md)
>   is for.
> - **Built but not yet run on hardware:** the viewer protocol (frames, baud
>   switching, `#!` lines) and settings in flash. Their PC side is tested
>   against a simulated camera only.
> - The firmware **builds clean for `esp32` with ESP-IDF v5.5.5 and
>   esp32-camera 2.1.8** (zero warnings in this project), and
>   `esp_wifi_init` / `esp_bt_controller_init` are absent from the ELF.

---

## Why colour blobs, not a neural network

| Option | Verdict |
|---|---|
| **Colour blob (this)** | A few ms per QQVGA frame, deterministic, and every failure can be explained with the `m` mask dump. Works on any target you can paint. |
| ESP-DL person/face models | Needs PSRAM and runs at ~1–3 FPS on the original ESP32, near full CPU, so the board runs hot. And it only knows the classes it was trained on. |
| Stream JPEG to a laptop, run YOLO | The most capable option. But WiFi is the largest heat source on this board (see optflow_cam), it adds latency, and the airframe depends on a link. |

Hue is the key choice. A red balloon in shadow is *darker*, not *less red*. So
testing hue, with floors on saturation and brightness, survives lighting
changes that would break a plain RGB threshold.

## Pipeline

```
OV2640 ──RGB565 QQVGA──> frame buffer (DRAM)
                             │
   per pixel: hue within ±tol, sat ≥ S, val ≥ V      (no divide until both floors pass)
                             │
   pool into 2x2 cells: a cell is set when ≥ half its pixels match   → speckle gone
                             │
   connected cells, 4-connectivity, union-find (memory bounded for ANY input)
                             │
   4 largest blobs → exact centroid and box from their matching PIXELS
                     (own cells, plus edge cells that touch them)
                             │
   az/el from pinhole model, range = 550 mm · f / larger box side
                             │
                    CSV line on UART0 @ 115200
```

### The edge-pixel fix (found by the host suite)

The first version measured the centroid only from the blob's own cells. Along
a curved edge many 2×2 cells hold just one matching pixel, so they fall below
the majority threshold. Dropping them lost **3% of the area** and biased
sub-pixel centroids by up to **0.07 px**. The final pass now also takes the
matching pixels from below-threshold cells that touch the blob. On the same
frames, centroid error is **0.000 px** and area is exact. Speckle away from the
blob is still excluded.

## Colour presets

| Preset | Hue window | Sat ≥ | Val ≥ | Notes |
|---|---|---|---|---|
| **red** (default) | 348°–12° | 110 | 50 | Competition waypoints. Pink walls and skin fall below the saturation floor. |
| blue | 190°–250° | 90 | 40 | |
| green | 90°–160° | 80 | 40 | |
| orange | 15°–35° | 120 | 60 | **Wood and brown furniture also pass.** Test orange against a plain background. |

Red and orange are only 25° apart, so their windows are narrow with a gap
between them. Host test 14 checks that each preset finds only its own balloon.
Select a preset at build time (`menuconfig` → Target colour preset) or at run
time with `p red`, `p blue`, `p green` or `p orange`. `c` (calibrate on the
centre of the view) replaces the preset with measured thresholds.

## Geometry (QQVGA, stock 65° diagonal lens → 54° h-FOV)

f = 80 / tan(27°) = **157 px**, so **0.365° per pixel** at the centre.

For a 550 mm waypoint balloon, from the host suite's shaded mylar model:

| Range | Radius | Detected | Bearing bias (toward the lit side) | Box range reads |
|---|---|---|---|---|
| 1 m | 43 px | yes | 5.3 px (1.9°) | 1.08 m |
| 2 m | 22 px | yes | 2.7 px (1.0°) | 2.11 m |
| 4 m | 11 px | yes | 1.4 px (0.5°) | 4.32 m |
| 6 m | 7 px | yes | 0.9 px | 6.64 m |
| 10 m | 4 px | yes | 0.5 px | 10.79 m |
| 12 m | 4 px | yes, 36 px area | 0.5 px | 12.34 m |

The arena is 10 m × 5 m, so any balloon in it is inside detection range.

**Range is taken from the box's larger side, not the area.** On mylar, the
white specular highlight and the dark far side both fail the colour test. The
area therefore undercounts the balloon, and range read 10–13% long. The box
spans the highlight hole and reads 5–11% long. The remaining bias is
systematic, so calibrate `d<mm>` against a real balloon at a measured distance.
The bearing bias is about 12% of the radius toward the light; it shrinks as the
balloon gets closer to the centre of view. **Clipped targets report
`range_mm = -1`**, because a target cut off by the frame edge looks smaller,
which would make the range read long.

## Building and flashing

Same toolchain as optflow_cam:

```bash
source $HOME/.espressif/tools/activate_idf_v5.5.5.sh
idf.py -C firmware/projects/target_cam set-target esp32
idf.py -C firmware/projects/target_cam build
```

**Flashing through the ESP32-CAM-MB** (the bench programmer, `/dev/ttyACM0`)
is what worked on 2026-10-13. `idf.py flash` and auto-reset did not:

1. Hold the board's **BOOT** button, unplug and replug the USB cable, and keep
   holding BOOT.
2. From `target_cam/build`:
   ```bash
   $HOME/.espressif/tools/python/v5.5.5/venv/bin/python -m esptool --chip esp32 \
       -p /dev/ttyACM0 -b 115200 --before no_reset --after no_reset write_flash @flash_args
   ```
   Stay at 115200: the bridge wedged once after a 460800 flash.
3. Release BOOT and replug to run the new firmware.

The 5-second reflash window after boot is there because GPIO0 is XCLK on the
AI-Thinker board. Once the camera starts, esptool can no longer force download
mode. Power the board from 5 V.

Opening the port with DTR or RTS asserted resets the board, and terminal
programs and pyserial do that by default. The viewer lowers both lines before
it opens the port, so it does not reset the board. `target_monitor.py` does
not lower them, so it resets the board on connect (harmless; you see the boot
log).

**Settings in flash.** A boot starts from the Kconfig defaults and then loads
any settings saved with `!save` (NVS namespace `targetcam`). The banner's
`settings` line says which. `!reset` erases them. A reflash does not erase them
unless you erase the whole flash. A record saved by the 10 FPS firmware (record
v1) is still loaded, calibration included, but its frame rate is replaced by the
build default of 80 ms, and the banner says so; `!save` rewrites it as v2.

For a camera mounted upside down, set `TARGETCAM_VFLIP` and `TARGETCAM_HMIRROR`
in `menuconfig`. That keeps +az as "right of the nose" and +el as "above it".

## PC viewer: see the image and calibrate by hand

[`host/viewer/`](host/viewer/README.md) (`python -m tcview --port
/dev/ttyACM0`) has two screens:

- **Image & calibration:** the camera's raw pixels, a mask overlay, sliders,
  pixel and region colour readouts, and save-to-flash.
- **Flight data:** only the CSV the flight board will receive, as a dashboard.

Its overlay is computed by this project's own `blob_detect.c`. The setup, the
two screens and the `#!` protocol are described there. The checklist below
predates it and still works from a plain terminal, but the viewer replaces
steps 2–4 with a live image.

## First power-up checklist

1. Run `host/target_monitor.py --port /dev/ttyACM0`. Check that rows arrive and
   that `dt_ms` is ~80 ms (12.5 FPS target).
2. Hold the target so it fills the centre of the view, then send **`c`**. The
   firmware samples the central 16×16 patch and sets hue, tolerance and floors
   from it.
   - It **refuses** if saturation is below 70: the target is too dull to tell
     apart from walls.
   - It **warns** if the hue spread is over 20°: the patch is not one colour.
3. Send **`m`** to get the mask as ASCII art (`#` matches, `+` partial, `X`
   centroid). Check that the target is solid and the background is empty.
4. If **nothing** matches even right after a good `c`, send **`o`** to flip the
   RGB565 byte order. The source says big-endian. This check confirms it.
5. Move the target across the view and check `az_deg` against a protractor or
   a measured offset at a known distance. That calibrates `TARGETCAM_HFOV_DEG`,
   which scales every angle and every range.
6. Try **`w0`** (freeze AWB) with the target filling much of the frame. Auto
   white balance tries to neutralise a dominant colour, which pulls the target
   toward grey. Whether freezing helps on this airframe is a measurement still
   to be made.

## Runtime commands

Commands are case-sensitive: `S` is saturation, `s` is the stream.

| Command | Effect |
|---|---|
| `p<name>` | Colour preset: `p red`, `p blue`, `p green`, `p orange` (or `p0`–`p3`, or the first letter) |
| `c` | Calibrate colour from the centre 16×16 patch |
| `H<deg>` | Target hue: 0 red, 30 orange, 60 yellow, 120 green, 210 blue, 300 magenta |
| `T<deg>` | Hue tolerance ±, 1–90 |
| `S<0-255>` | Minimum saturation |
| `V<0-255>` | Minimum brightness |
| `a<px>` | Smallest blob reported |
| `d<mm>` | Real target diameter for range (default 550); `d0` turns range off |
| `f<fps>` | Frame rate, 1–30, decimals allowed (`f12.5`). Rounded to a whole-ms period, and the reply gives the real one: `f15` is 67 ms (14.93 FPS). The sensor gives 12.5 at XCLK 20 MHz, so faster gains nothing. |
| `w<0\|1>` | AWB frozen / auto |
| `F<deg>` | Horizontal FOV, 10–150 (decimals allowed), for angles and range |
| `o` | Flip RGB565 byte order |
| `m` | Dump the detection mask once |
| `s` | Toggle the CSV stream |
| `?` | Banner and current settings |
| `k` | Print the `#!CFG` line (the live settings, machine-readable) |
| `g` | Send one raw RGB565 frame (`#!FRAME`…`#!END`, see the viewer README) |
| `v<0\|1>` | Send a frame every loop / stop. **Timing is not flight-representative while on.** |
| `B<baud>` | Change speed: 115200, 230400, 460800, 921600, 1500000, 2000000. Reverts to 115200 if no valid command arrives at the new speed within 3 s. Every boot starts at 115200. |
| `!save` | Write the live settings to flash. The camera boots with them. |
| `!load` | Reload the settings saved in flash |
| `!reset` | Erase the saved settings and go back to the Kconfig defaults |

Every command line is answered with `#!ACK <cmd>` or `#!NAK <cmd> -- <reason>`.
A setting change is followed by `#!CFG`. The commands that touch flash are
whole words, so one garbled byte (on the bench, `m` once arrived as `W`) can
never write to flash.

## Output format

```
t_ms,ok,cx_px,cy_px,az_deg,el_deg,area_px,w_px,h_px,fill,range_mm,clipped,blobs,matched_px,b2_az,b2_el,b2_range_mm,b3_az,b3_el,b3_range_mm,b4_az,b4_el,b4_range_mm,dt_ms,proc_us
```

| Field | Meaning |
|---|---|
| `ok` | **0 means no target, not a target dead ahead.** All position fields are 0 and must be ignored. |
| `cx_px` … `clipped` | The **largest** blob, usually the nearest balloon. |
| `cx_px`, `cy_px` | Centroid. Pixel *i* spans [*i*, *i*+1), so the frame centre is exactly (80, 60). |
| `az_deg`, `el_deg` | Bearing from the optical axis. +az is right, +el is up. **Body frame**: no IMU here, so roll and pitch tilt these. |
| `area_px` | Matching pixels in the blob. |
| `w_px`, `h_px`, `fill` | Bounding box and area/box. A full disc gives ~0.79; mylar reads lower because of the highlight hole. Much lower means an irregular or partly hidden target. |
| `range_mm` | `target_mm · f / max(w, h)`. **−1** when off (`d0`) or when clipped. |
| `clipped` | The box touches the frame edge, so part of the target is out of view. |
| `blobs` | Blobs at or above `min_area` in the frame. Can exceed 4. |
| `matched_px` | Matching pixels in the whole frame. Rising with no blob means scattered colour clutter: tighten `S` or `T`. |
| `bN_az`, `bN_el`, `bN_range_mm` | Blobs 2–4, in decreasing size. Absent ones read `0,0,-1`. **Choosing which one is the next waypoint is the navigator's job.** The largest is the nearest, which is not always the next in the course sequence. |
| `dt_ms` | Measured interval between captures. |
| `proc_us` | Detector cost. |

## Verification

```bash
cd firmware/projects/target_cam/host && make test
```

This builds with AddressSanitizer and UBSan and runs synthetic RGB565 frames
(big-endian, like the camera). The background is grey value noise with colour
noise, not a blank frame. Result:

```
 1. red disc at 6 sub-pixel positions       centroid error 0.000 px, area exact
 2. red wrapping through 0/360              hue 350 and 12 both accepted
 3. no target (grey + a green disc)         found=0, 0 pixels matched
 4. two red discs                           larger first, runner-up second
 5. desaturated pink, near-black red        rejected
 6. 300 isolated red pixels                 classified, but pooled away: found=0
 7. byte order swapped                      not detected; correct order detected
 8. disc off the right edge                 clipped=1
 9. full-frame checkerboard, cells 1/2/4    9600/2400/600 blobs, no overrun
10. calibration on 5 colours                hue within 1 deg, spread <= 3 deg
11. config validation                       bad cell/hue/tol/area rejected
12. red mylar balloon 1-12 m, red preset,   found at every range; skin, wood,
    with skin, wood and a pink wall         pink wall never detected; centroid
                                            within 13.4% of r; box range +5..11%
13. four red balloons at 2.5/4.5/6.5/9 m    all four reported, nearest first
14. each preset vs. 4 coloured balloons     each finds only its own balloon
15. blob_mask vs. blob_pixel_matches        0 disagreements, both byte orders
PASSED (0 failures)
```

Test 9 matters most for robustness. A checkerboard is the worst case for
provisional labels. The label arrays are sized for it, so no input can overrun
them, and ASan confirms it.

Tests 1–11 use hard-edged discs. Tests 12–14 use a shaded sphere: lit from
the upper left, a dark far side at a quarter of the brightness, and a white
specular highlight. This approximates a mylar balloon, but it is a model.
**Real mylar also mirrors the room, and how the OV2640 renders that red is
unmeasured.** The first bench test should be a real waypoint balloon at
measured distances (see the checklist).

## Files

| File | Purpose |
|---|---|
| `main/blob_detect.h/.c` | The detector. Portable C, no ESP-IDF and no heap; the caller provides a workspace (28 KB at 2×2 cells). |
| `main/target_main.c` | Camera setup, pacing, calibration, mask dump, bearing/range, CSV, commands. |
| `main/link.h/.c` | Raw frame transport (`#!FRAME`, CRC-32) and run-time baud changes with the 3 s safety revert. |
| `main/settings.h/.c` | Versioned settings record in NVS (`!save` / `!load` / `!reset`). |
| `main/camera_pins.h` | Pin maps: AI-Thinker, ESP-EYE, XIAO S3 (shared with optflow_cam). |
| `main/Kconfig.projbuild` | Board, frame period, XCLK, FOV, flips, default colour, area, target size. |
| `sdkconfig.defaults` | 240 MHz, 1 kHz tick, PSRAM optional, brownout on. |
| `host/test_blob_detect.c` | The verification suite above. |
| `host/target_monitor.py` | Live viewer with an azimuth gauge, and a CSV logger (needs `pyserial`). |
| `host/viewer/` | `tcview`: image, hand calibration and flight-data screens (PyQt6, own conda env). |
| `host/Makefile` | `make test` runs the suite; `make lib` builds `libblob_detect.so` for the viewer. |

## Known gaps

1. **Not calibrated on a real balloon.** The red preset missed one at 1.5 m
   (see the status note). The viewer protocol and settings in flash have not
   run on hardware yet.
2. **FOV is assumed** (54°). It scales every angle and range. Calibrate it.
3. **No attitude compensation.** Bearings are in the camera frame. The board
   that owns the IMU must rotate them before using them as world bearings.
4. **AWB versus a large target** is unmeasured (step 6 of the checklist).
5. **One target colour at a time**, up to four blobs. Tracking a particular
   balloon across frames, and knowing which waypoint is next, is the
   navigator's job on the S3.
6. **Two cameras, two UARTs.** optflow_cam and target_cam each stream on their
   own UART0. The flight board needs two serial inputs, or the two streams
   must be merged.
