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

> **Hardware status: untested.** Nothing in this project has run on a camera
> yet.
> - **Verified:** the detector passes its host suite (below). The firmware
>   **builds clean for `esp32` with ESP-IDF v5.5.5 and esp32-camera 2.1.8**:
>   zero warnings in this project, a 318 KB binary with 69% of the app partition
>   free, and `esp_wifi_init` / `esp_bt_controller_init` absent from the ELF.
> - **Read from the source, not measured:** the RGB565 byte order (high byte
>   first, taken from the driver's own `to_bmp.c`), the frame rate, colour
>   fidelity under real lighting, and the effect of AWB.

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

Same toolchain and wiring as optflow_cam:

```bash
source /home/talos/.espressif/tools/activate_idf_v5.5.5.sh
idf.py -C firmware/projects/target_cam set-target esp32
idf.py -C firmware/projects/target_cam build
idf.py -C firmware/projects/target_cam -p /dev/ttyUSB0 flash
```

The 5-second reflash window after boot is there because GPIO0 is XCLK on the
AI-Thinker board. Once the camera starts, esptool can no longer force download
mode. Power the board from 5 V.

For a camera mounted upside down, set `TARGETCAM_VFLIP` and `TARGETCAM_HMIRROR`
in `menuconfig`. That keeps +az as "right of the nose" and +el as "above it".

## First power-up checklist

1. Run `host/target_monitor.py --port /dev/ttyUSB0`. Check that rows arrive and
   that `dt_ms` is ~100 ms (10 FPS target).
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
| `f<fps>` | Frame rate, 1–30 (the sensor gives ~12.5 at XCLK 20 MHz) |
| `w<0\|1>` | AWB frozen / auto |
| `o` | Flip RGB565 byte order |
| `m` | Dump the detection mask once |
| `s` | Toggle the CSV stream |
| `?` | Banner and current settings |

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
| `main/camera_pins.h` | Pin maps: AI-Thinker, ESP-EYE, XIAO S3 (shared with optflow_cam). |
| `main/Kconfig.projbuild` | Board, FPS, XCLK, FOV, flips, default colour, area, target size. |
| `sdkconfig.defaults` | 240 MHz, 1 kHz tick, PSRAM optional, brownout on. |
| `host/test_blob_detect.c` | The verification suite above. |
| `host/target_monitor.py` | Live viewer with an azimuth gauge, and a CSV logger (needs `pyserial`). |

## Known gaps

1. **Nothing has run on hardware.** First steps are the checklist above.
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
