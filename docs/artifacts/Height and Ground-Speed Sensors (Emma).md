DroneUN · IAR-LARC 2026Sensor study11 October 2026

# Height and ground-speed sensors

Three questions came up about the blimp's downward-looking sensors. This page answers them with the datasheets, about 530 closed-loop runs of our MuJoCo simulator, a synthetic optical-flow bench, and independent re-checks of each result. It ends with bench tests that settle what the simulations cannot.

- **Q1**The VL53L1X range figures come from a completely dark room. Will it work for altitude, or should we use an ultrasonic sensor?
- **Q2**How do we measure ground speed or distance travelled? Can a downward ESP32-CAM do optical flow, with what settings and what minimum frame rate?
- **Q3**Should the images go to a PC for processing, with the result sent back?

00

## Answers

Q1 · Altitude

### Keep the VL53L1X, and test it on a real floor

*!*Works if no sunlight reaches the floor

The datasheet's 3.6 m range is measured in the dark, so the concern is fair. LED and fluorescent lamps give off almost no light at the sensor's 940 nm, and under them the sensor ranges almost as in the dark. Sunlight on the floor is what hurts: range falls to 1.66 m over a white floor and 1.14 m over a dark one. In flight the sensor is 1.53 to 2.29 m above the floor.

Ultrasonic ignores sunlight but adds 8 to 9 g, a 5 V echo line and noise from our own motors. Raw ultrasonic readings completed 4 of 30 simulated loops. Filtered readings completed 23 of 30, and 30 of 30 once the driver holds its last reading (section 01). Test the two side by side, and keep the ultrasonic as a backup.

Q2 · Ground speed

### Worth adding, for the search heading and Rule 7

*✓*Optical flow; motor-command dead reckoning fails

When the target balloon is out of view, the controller turns toward a search heading. Without a speed sensor that heading is 15 to 35° wrong. With optical flow it is 6 to 9° wrong, as good as the simulator's current stand-in, which uses the true position.

Lap times do not change. Flow is also the only option here that can keep the blimp inside the 10 × 5 m area (Rule 7). The simulated blimp already leaves the area in most runs.

Q2 · ESP32-CAM

### Possible, but not proven: measure its frame rate first

*!*Needs ≥ 15 fps, gyro correction and a textured floor

The camera needs grayscale 96 × 96 frames, a fixed 8.33 ms exposure, gyro de-rotation, the ToF height, and at least 15 fps (10 fps is borderline). The catch is that the stock camera driver runs raw grayscale at about 12.5 fps on the ESP32-CAM's chip. We worked that out from the driver source; nobody has measured it.

No published project shows ESP32-CAM optical flow working. It also needs a second board, about 10 g. The test firmware below measures all of this in an afternoon.

Q3 · PC processing

### Allowed, but compute on the camera board

*✓*Onboard flow, 13 bytes per frame over UART

Rule 1 allows off-board computation; Rule 15 only requires the sensing to be on board. The search heading tolerates up to about 0.6 s of delay.

Gyro correction, however, needs each frame matched to the gyro within a few milliseconds. WiFi outages also freeze the estimate, and streaming costs battery. Use the PC to look at frames during practice, not in the control loop.

**The biggest unknown is the venue, not the sensors.** The floor material and the daylight in the UMNG hall decide both the ToF question and the optical-flow question, and nothing about them is published. The event runs 2–6 November 2026, three weeks from now. Ask the organisers this week (section 06).

01

## Altitude: VL53L1X or ultrasonic

The VL53L1X fires a 940 nm infrared laser pulse and times the return. Background light at that wavelength, reflected off the floor patch it looks at, competes with the return. ST therefore gives the maximum range at three light levels, measured in counts per SPAD (single-photon detector): dark, 50 kcps/SPAD ("a sunny day behind a window") and 200 kcps/SPAD (the same with direct sun on the sensor). Ordinary office lighting is about 5 kcps/SPAD, close to the dark column.

What matters is how much 940 nm light reaches the floor under the blimp. White LEDs and fluorescent tubes emit almost none; sunlight and halogen or incandescent lamps emit a lot. The floor's reflectance at 940 nm matters too: some black rubber sports floors absorb infrared even when they look grey.

VL53L1X maximum range, long mode, 100 ms timing budget

Typical values from ST datasheet DS12385. The shaded band is where the floor actually is in flight.

White floor (88 %) Mid-grey floor (54 %) Dark-grey floor (17 %) Sensor height in flight, 1.53–2.29 m

**Read it this way:** in the dark every floor reaches the band, though the dark-grey floor only just. In window light the white and mid-grey floors barely reach its bottom edge and the dark one falls short. In direct sun nothing does. The values are typical ones; ST guarantees only 260 / 220 / 80 cm in the dark for the three floors. Shorter timing budgets than 100 ms reduce range further.

Data table

| Light level | White 88 % | Grey 54 % | Dark grey 17 % |
| --- | --- | --- | --- |
| Dark (office light ≈ 5 kcps/SPAD is close to this) | 360 cm | 340 cm | 170 cm |
| 50 kcps/SPAD, sunny day behind a window | 166 cm | 154 cm | 114 cm |
| 200 kcps/SPAD, direct sun on the sensor | 73 cm | 69 cm | 68 cm |
| Guaranteed minimum, dark | 260 cm | 220 cm | 80 cm |

**Check the chip before anything else.** Boards sold as "ToF400C" or silkscreened "VL53L0/1X" sometimes carry the older VL53L0X. That chip reaches about 1.2 m by default and 2 m at best in the dark, which is not enough here. One register read tells them apart: `0x010F` reads `0xEACC` on a VL53L1X, and `0xC0` reads `0xEE` on a VL53L0X. Test T1 does this.

### What the simulator says

We modelled each sensor at the gondola's underside, 0.45 m below the envelope centre, tilting with the gondola. Each model ran on both courses with noise seeds 0–4, 220 s per run, through the real flight controller in `core/`. The chart counts completed loops out of 30 (3 loops × 5 seeds × 2 courses). Thrust is the repo's 35 gf placeholder. Five models were repeated at the measured 16.8 gf, with the same pattern (table below).

Loops completed out of 30, by altitude sensor model

Hover a row for failsafe trips and what the model assumes.

**Two things stand out.** A ToF that loses the floor (window light or a dark floor) does no better than having no altitude sensor at all. And how the driver reports readings matters as much as which sensor it is: compare the "holds last reading" rows with the "flagged only when new" rows. That second point is a controller problem, explained next.

| Sensor model | What it assumes | Loops /30 35 gf | Loops /30 16.8 gf | Failsafe trips (10 runs) |
| --- | --- | --- | --- | --- |
| Ideal ToF | Repo sim model: every 10 ms tick, σ 1 cm | 30 | 30 | 0 |
| VL53L1X, driver holds last reading | Long mode, 33 ms (30 Hz), σ max(5 mm, 0.5 %), 0.5 % invalid; last good reading reused between samples | 30 | – | 0 |
| VL53L1X, flagged only when new | Same readings, but marked valid only on the tick a sample arrives | 26 | 27 | 1 |
| VL53L1X, window light | Valid below 1.2 m, fading to none at 1.6 m; σ 2.5 cm | 19 | 20 | 51 |
| VL53L1X, dark floor | Valid below 1.8 m, fading to none at 2.4 m; σ 1.5 cm; 3 % invalid | 19 | – | 44 |
| HC-SR04 filtered, holds last reading | 60 ms cycle, σ 1 cm, 5 % bad echoes, median-of-3 plus 0.25 m jump gate | 30 | – | 0 |
| HC-SR04 filtered, flagged only when new | Same filter, valid only on sample ticks | 23 | 23 | 1 |
| HC-SR04 raw | 5 % bad echoes (short false echoes or 4 m timeouts), no filter | 4 | 7 | 387 |
| BMP180 barometer (GY-88) | σ 0.34 m (best datasheet mode at Bogotá air density), 25 Hz | 0 | – | 534 |
| No altitude sensor | Altitude only from balloon geometry in the camera view | 19 | – | 39 |

### A controller problem the study found

The controller runs every 10 ms, but the ToF delivers a new reading every 33 ms and the HC-SR04 every 60 ms. On ticks with no fresh reading, `core/src/blimp.c:270-290` substitutes the rough altitude it computes from balloon geometry in the camera view. Then `core/src/estimator.c:509-536` turns the jump between the two into a vertical speed by dividing by one 10 ms tick. Each switch becomes a vertical-speed spike, and the heave loop chases it.

1. **Fix A (driver, no core change):** the ToF driver, which is not written yet (`firmware/main/main.c` leaves `range_valid` unset), keeps the last good reading valid between samples for up to about 100 ms.
2. **Fix B (core):** use the camera-based altitude only when no real sensor reading has arrived for 0.5 s. A scratch copy of `core/` with this change completed 15 of 15 loops on course 2, against 12 of 15 with the stock core.
3. **Outliers:** the core has no outlier gate, so one bad ultrasonic echo causes a 3–10 m/s vertical-speed spike. Filter in the driver (median-of-3 plus jump gate) or gate in the estimator.
4. **Barometer:** the same divide-by-one-tick step multiplies noise by about 15.6. A BMP180 (σ ≈ 0.34 m here) therefore makes 5 m/s of vertical-speed noise, which is why every barometer variant completed 0 loops. A sweep of baro noise gave 9, 4, 1 and 0 of 9 loops at σ 0.01, 0.04, 0.08 and 0.15 m.

None of this is in the repo yet. Each fix is a small, well-scoped change for the `core-change` checklist. Fix A or B should land before the first flight with a real ToF.

### Side by side

| Option | Mass | Range at our height | Rate | Sunlight | Dark floor | Our motors | Wiring |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **VL53L1X** (owned) | ≈ 1–2 g\* | Fine in LED light; fails above \~1.1–1.7 m in daylight | up to 30 Hz (50 Hz short mode) | Range collapses | Marginal (170 cm typical, 80 cm guaranteed) | No effect | I²C, 3.3 V, already planned |
| **HC-SR04** (owned) | 8–10 g | 2–400 cm; beam \~30° wide | ≈ 16 Hz (60 ms cycle) | Immune | Immune (soft carpet absorbs) | Prop noise and vibration cause false echoes; needs foam mount, test T3 | 5 V supply, echo through 1k/2k divider |
| TF-Luna (buy) | \< 5 g | 0.2–8 m; only 0.2–2.5 m on 10 % floors | up to 250 Hz | 70 klux immunity | OK to 2.5 m | No effect | 5 V, 70 mA, UART or I²C |
| BMP180 (on GY-88) | 0 g | Relative only, σ 0.25–0.5 m | tens of Hz | Immune | Immune | Prop wash, doors and HVAC shift pressure (≈ 11 cm per Pa here) | Already there; unusable until the estimator filters it |

\* Board mass not verified; weigh it. 8–10 g is about 5 % of the 0.78 m envelope's gross lift (≈ 190 g at Bogotá air density). Each extra 12 g needs about 1.5 cm more envelope diameter.

### Recommendation

- Keep the **VL53L1X as the primary altitude sensor**: long mode, 50 ms budget (20 Hz) to start, full 16 × 16 ROI. A narrower ROI cuts range: with a 15° ROI, dark-floor range falls to 45 cm. Use only readings with range status 0.
- Mount it at the gondola's lowest point, pointing straight down, with no cover window and clear of the props.
- Write the driver to hold the last good reading (Fix A). Add Fix B to `core/` when convenient.
- Bench-test the **HC-SR04 as a backup** (T3). If the venue floor turns out to be sunlit, fly it filtered, or order a TF-Luna now as insurance.
- Do not use the barometer for altitude in the current estimator.

02

## Ground speed: is it worth it?

Today's controller has no position estimate. It steers on the camera's bearing to the target balloon. When the balloon is out of view, it turns toward a *search heading*, and something has to compute that heading. The simulator cheats: it uses the true position plus a 10° error. On the real blimp, the ground station or the S3 must work it out from an estimated position, and keeping an estimated position needs ground speed.

Two more reasons. First, Rule 7 voids a run if the blimp leaves the competition area. The simulated blimp already crosses the 10 × 5 m boundary while flying in 8 of 10 still-air runs, by up to 1.5 m, and in every run with a draft. Second, indoor drafts are real: a 0.05 m/s draft moved a stopped blimp 1.2 m in 40 s in the simulator.

We replaced the simulator's search heading with realistic estimates under drafts of 0, 0.05 and 0.10 m/s, using a realistic camera field of view (55 × 42° instead of 90 × 70°). That gave 182 runs; an independent re-run of the key configurations matched them.

Search-heading error while the target is out of view (median, degrees)

0.05 m/s draft, 55 × 42° camera, 5 seeds per course. Lower is better.

Course 1 (rectangle) Course 2 (figure eight)

**Flow brings the heading error down to the simulator's cheat level; heading quality matters as much as speed.** Flow with an uncorrected gyro heading is 18–22° off. The calibrated version re-aligns the heading at each balloon touch, using the known balloon spacing, and is 6–7° off. Motor-command dead reckoning (no sensor, thrust and drag model with ±20 % model error) is worse than doing nothing.

| Mode (0.05 m/s draft) | Loops /15 course 1 · 2 | Best lap, median s course 1 · 2 | Runs with \< 3 loops | Worst distance from centre |
| --- | --- | --- | --- | --- |
| Simulator stand-in (true position) | 15 · 15 | 26.5 · 23.0 | 0 / 10 | 5.1 m |
| No speed sensor, gyro heading | 15 · 12 | 26.2 · 43.6 | 2 / 10 | 10.0 m |
| No speed sensor, perfect heading | 15 · 12 | 24.2 · 24.9 | 1 / 10 | 15.5 m |
| Motor-command dead reckoning | 13 · 10 | 29.5 · 30.8 | 5 / 10 | 12.9 m |
| Flow, gyro heading not re-aligned | 15 · 13 | 23.9 · 24.4 | 2 / 10 | 12.7 m |
| Flow, PMW3901-like (100 Hz, 10 ms) | 13 · 15 | 24.0 · 23.7 | 1 / 10 | 14.4 m |
| Flow, ESP32-CAM on board (20 Hz, 60 ms) | 15 · 15 | 23.8 · 23.3 | 0 / 10 | 4.3 m |
| Flow, ESP32-CAM through a PC (15 Hz, ≈ 150 ms, outages) | 15 · 12 | 26.8 · 24.0 | 1 / 10 | 14.2 m |

5 seeds × 2 courses per row. At 0.10 m/s (3 seeds) motor-command dead reckoning strayed up to 25.6 m. Two extra audit runs of the on-board ESP32-CAM mode at 0.10 m/s completed 3 and 1 loops; the second wandered 21.6 m away.

**What holds and what does not.** The heading-error improvement is solid. So is the finding that motor-command dead reckoning fails. With only 3–5 seeds per configuration, the flow modes do *not* complete measurably more loops than the no-sensor modes. Losses are rare, seed-driven events in which the blimp wanders off for 200 s or more, and they hit every mode except the stand-in.

Lap times do not change, so the value of flow is in finding the next balloon and staying in bounds, not in speed. The search heading tolerates up to about 0.6 s of delay and update rates down to 2 Hz. It degrades beyond about 1 s of delay or below 1 Hz.

Staying inside the area needs one more piece that does not exist yet: guidance that steers back when the position estimate nears the boundary. Flow is the input that makes it possible.

### Options for ground speed and position

| Option | What it gives | Mass | Status for us | Notes |
| --- | --- | --- | --- | --- |
| **Second ESP32-CAM, facing down** | Flow → ground velocity, with ToF height and gyro | ≈ 10 g + wiring | Have it | Unproven; frame rate is the risk (section 03). Board ≈ 0.5–0.8 W, not measured |
| PMW3901 breakout | Flow chip with its own optics, ≈ 121 fps, 42° view | ≈ 1–3 g | Buy (import) | The standard choice (Crazyflie Flow deck). Needs ≥ 60 lux and a textured floor. ≈ US$25–40 |
| MicoAir MTF-01 | PMW3901-class flow plus an 8 m rangefinder, UART at 100 Hz | 4.5 g | Buy (import) | Would also cross-check the ToF. 5 V, ≈ 100 mA. ≈ US$33–43. MTF-01P is 8 g |
| XIAO ESP32S3 Sense | Same OV2640 camera, but the driver runs raw frames ≈ 2× faster (≈ 25 fps) | ≈ 7 g | Buy | Fallback if the ESP32-CAM tops out near 12.5 fps. Mass from a retailer listing |
| Balloon fix with the forward camera | Absolute position from bearing and size of known balloons | 0 g | Planned camera | Only while balloons are in view. ≈ 5–7 % range error at 5 m, untested on mylar |
| Motor-command dead reckoning | Velocity from thrust, drag and added mass | 0 g | Ruled out | 45–100° heading error in the sim; drafts dominate |
| Integrating the accelerometer | Velocity from MPU-6050 | 0 g | Ruled out | A 1° tilt error leaks 0.17 m/s² of gravity, several times the blimp's real accelerations |
| UWB anchors, beacons, floor markers | Position from installed hardware | – | Ruled out | Needs equipment placed in the arena; Rule 15 and the rulebook do not allow for it |
| Magnetometer (GY-88) | Heading only | 0 g | Have it | Helps the heading half of the problem, which the sim shows matters as much as speed. Must sit away from motors |

03

## Optical flow on a downward ESP32-CAM

**What the camera has to cope with** (from the simulator, at the sensor point under the gondola): ground speed up to 1.1 m/s, height 1.53–2.29 m, yaw rate up to 91°/s and swing rate up to 31°/s (95th percentiles), and tilt up to 10°. The swing and yaw make roughly as much image motion as forward travel, which leads to three hard requirements.

- **Gyro de-rotation is mandatory.** Without it the velocity error is 0.3–0.6 m/s, larger than the speed itself. Warping the image by the gyro rotation brings it to 0.025–0.05 m/s.
- **The gyro and the frames must share a clock.** Each millisecond of offset adds about 0.002 m/s of error, and 3 ms of frame-to-frame jitter adds 0.02 m/s. Stamp every frame and use MPU-6050 samples from the same instant.
- **Correct for the lever arm.** The camera hangs 0.45 m below the envelope centre, so swinging moves it sideways. Subtract the gyro rate × 0.45 m, or the envelope-velocity error grows to 0.15–0.2 m/s.

Velocity error vs frame rate, 96 × 96 grayscale, Lucas–Kanade with gyro warp

Synthetic floors rendered with an OV2640 model (rolling shutter, noise, 8.33 ms exposure) and replayed simulator motion. Hover for values.

Wood-like floor (17.7 % contrast) Speckled floor (5.5 %) Faint mottling + court lines (1.7 %) 0.05 m/s target

**Below 10 fps the estimate falls apart; above 15 fps more frames barely help.** At 5 fps only 41–61 % of frames give a usable estimate (hollow markers). At 10 fps one test set still dipped to 92–94 % valid, so 15 fps is the safe target. The shaded line marks the ≈ 12.5 fps we expect from the stock driver. Above 15 fps the error floor, about 0.03 m/s, comes mostly from the rolling shutter, not the frame rate.

Data table

| Floor | 5 fps | 10 fps | 15 fps | 20 fps | 30 fps |
| --- | --- | --- | --- | --- | --- |
| Wood-like | 0.017 (61 % valid) | 0.025 | 0.030 | 0.030 | 0.025 |
| Speckled | 0.137 (41 % valid) | 0.025 | 0.029 | 0.029 | 0.024 |
| Faint mottling + lines | 0.215 (43 % valid) | 0.030 | 0.038 | 0.041 | 0.035 |

RMS velocity error of the camera, m/s, 10 Hz output. Phase correlation gives nearly the same numbers. PX4Flow-style block matching (SAD) needs 15–20 fps and is less accurate on the wood-like floor.

### The frame-rate catch

Published ESP32-CAM benchmarks show about 51 fps at 96 × 96, but that is JPEG. For raw grayscale, Espressif's driver (`esp32-camera`, `sensors/ov2640.c`, `set_window`) slows the sensor clock on the original ESP32 chip: the clock divider is 4 and the doubler is off. That gives about **12.5 fps**, between "borderline" and "safe". ESP32-S3 boards get the doubler, so about 25 fps. We derived this from register values and nobody has measured it, so test T4 does. There are three ways up if it holds:

1. Press `u` in the test firmware: it halves the divider for about 25 fps, but the ESP32 may drop or corrupt frames at that pixel rate.
2. Capture small JPEG frames (fast) and decode them on the chip. The JPEG blocks can bias flow toward zero, so use high quality.
3. Use an ESP32-S3 camera board such as the XIAO ESP32S3 Sense for the downward camera.

Compute is not the bottleneck: 64 × 64 block matching costs about 6 ms per frame on the ESP32, and 96 × 96 Lucas–Kanade with the warp about 7 ms on an S3. These are cycle-count estimates, not measurements.

### Configuration

| Setting | Value | Why |
| --- | --- | --- |
| Placement | A second ESP32-CAM under the gondola, lens straight down | The first one stays forward for the balloons |
| Pixel format, size | `PIXFORMAT_GRAYSCALE`, `FRAMESIZE_96X96` | Flow needs only brightness. 96 × 96 beat 160 × 120 and 64 × 64 on the bench |
| Clock, radio | XCLK 20 MHz, WiFi off | WiFi on the camera board costs current and drops raw frames |
| Frame buffers | Internal RAM, `fb_count` 2 (1 if frames tear), `CAMERA_GRAB_LATEST` | Always the newest frame. PSRAM writes are slow for raw formats |
| Exposure | Manual, 8.33 ms (or 16.7 ms); auto-exposure off; manual gain for a mean grey of 90–150 | Colombian mains is 60 Hz, so lamps flicker at 120 Hz. Other exposures made the flow fail on the bench |
| Banding filter | 60 Hz on | Same flicker reason; test T8 checks it |
| Focus | Turn the lens until the floor is sharp at ≈ 1.6 m | Stock lenses are often focused close. Check with a frame dump |
| Algorithm | Lucas–Kanade (global shift, 1–2 pyramid levels) or phase correlation, seeded with the gyro-predicted shift | Works from 10–15 fps. Block matching needs 15–20 fps and is in the test firmware only for the bench |
| De-rotation | Gyro warp per frame, MPU-6050 samples within ≈ 5 ms of the frame; subtract the lever arm | Rotation is as large as the travel signal |
| Scale | v = flow × height / focal length; height from the ToF; focal length calibrated on a tape (T6) | A 5 cm height error is a 3 % speed error. Barometer height would make it 20 % |
| Output | 13-byte packet per frame over UART to the S3: dx, dy, quality, frame interval | Fits the camera-to-S3 UART already in the wiring plan |

**Floor texture.** On synthetic floors the camera works from about 1.7 % contrast, a faint mottle with painted lines. It even works at 0.5 % in good light, but not in dim light, and a flat floor gives nothing. A glossy floor that mirrors the ceiling lamps was not modelled, and it is a real risk: the reflections move with the blimp, not with the floor. A PMW3901 matched the camera on textured floors in this bench and did worse on the faint one. That result comes from a proxy of the chip, not the chip itself.

04

## On a PC, or on the board?

|  | Images to a PC, result sent back | Flow computed on the ESP32-CAM |
| --- | --- | --- |
| Rules | Allowed: Rule 1 permits off-board computation; the camera is on board (Rule 15) | Allowed |
| Data rate | 96 × 96 raw at 20 fps is 184 kB/s (1.5 Mbit/s) up, plus the gyro stream | 13 bytes per frame over UART (390 B/s at 30 fps) |
| Delay | ≈ 100–250 ms typical over WiFi, with outages under congestion. The search heading tolerated ≈ 0.6 s in the sim | Tens of ms |
| Gyro timing | Hard: each frame must carry an on-board timestamp and the PC must align it with the gyro to within a few ms | Easy: camera and S3 stamp on the same UART link |
| Robustness | A WiFi dropout freezes the position estimate. Other leagues share the 2.4 GHz band at the venue | No radio involved |
| Power | Streaming draws ≈ 180–310 mA at 5 V on the camera board (low-confidence figures) | Lower; WiFi off |
| Sim result | Heading error 7–9°; 1 of 10 runs incomplete | Heading error 9°; 0 of 10 runs incomplete (1 of 2 in an extra 0.10 m/s check) |

**Verdict:** compute on the board. The PC is still handy during practice: stream a few frames to see the venue floor's texture and focus, and log telemetry.

05

## Bench tests

Ordered so that the cheapest tests that could change a decision come first. All use parts you have: the ToF board, the ultrasonic sensor, the ESP32-CAMs and an ESP32-S3. The sketches are in section 07. At the venue, repeat T2 and T5 during the dedicated practice time (Rule 23).

T1

### Which chip is on the ToF board?

5 min · tof_check.ino

Do

Wire the board to the S3 (I²C), flash, open the serial monitor at 921600.

Pass

`reg 0x010F = 0xEACC` (VL53L1X). If it prints `0xC0 = 0xEE`, the board is a VL53L0X: replace it.

T2

### ToF range, floor and light

1–2 h · tof_check.ino

Do

Sensor on a pole pointing down at 1.6, 2.0 and 2.3 m. Floors: the lab floor, black cloth, white paper, a glossy tile. Light: room LEDs, a halogen or incandescent lamp, and a floor patch in sunlight by a window. The sketch sweeps short, medium and long modes and budgets of 33–200 ms, 300 readings each.

Record

The `S,` lines: mean, σ, % invalid, rate, and `ambient_mcps`, the sensor's own measure of 940 nm light.

Pass

Long mode at 50 ms, 2.3 m: under 2 % invalid and σ under 2 cm. Compare `ambient_mcps` in sunlight with the lab: at the venue, the same reading tells you which case you are in.

T3

### Ultrasonic with the motors running

1 h · ultrasonic_check.ino

Do

Mount the HC-SR04 at its real distance from the motors, props on, on foam tape. 5 V supply, echo through a 1 kΩ / 2 kΩ divider. Run `go` with the motor at 0, 40 and 70 % duty, at 1.6 and 2.3 m. Weigh the module.

Pass

Under 1 % outliers (readings more than 5 cm from the local median) with the motors running. Compare σ with the ToF from T2.

T4

### ESP32-CAM real frame rate

30 min · esp32cam_flow.ino

Do

Flash, log 60 s of CSV at 96 × 96 and at 160 × 120 (key `r`), then with `u` (faster clock). Watch the serial log for `FB-OVF` or `NO-EOI` errors.

Pass

Median `frame_interval_us` ≤ 66 ms (≥ 15 fps), steady, with no errors. If it sits at ≈ 80 ms (12.5 fps) and `u` corrupts frames, plan on an ESP32-S3 camera board.

T5

### Floor texture and focus

30 min · esp32cam_flow.ino

Do

Camera at 1.6 m over each floor sample. Set manual exposure (`e`, `+`/`-`) for a mean of 90–150, focus the lens, then dump a frame (`d`) and look at it.

Pass

`quality` above 100 and `contrast` at least 8 grey levels, with the floor sharp in the dump.

T6

### Flow distance on a tape

1 h · esp32cam_flow.ino + ToF

Do

Carry the camera at a fixed 1.6 m (pole or cart) along a 4 m tape, five passes each at about 0.3 and 0.8 m/s. Sum dx and convert with height / focal length. Fit the focal length from the first passes.

Pass

After calibration, passes agree within 5 % of 4 m, and quality stays above the T5 level.

T7

### Swing test for gyro de-rotation

1–2 h · camera + MPU-6050

Do

Hang camera and IMU 0.45 m below a pivot, like the gondola, and swing and twist without moving the pivot. Log flow and gyro with shared timestamps. Find the time offset that best cancels them.

Pass

Residual apparent speed under 0.05 m/s after de-rotation and lever-arm correction.

T8

### Flicker

15 min · esp32cam_flow.ino

Do

Under the lab lights, key `k` at several manual exposures, banding filter on and off (`f`).

Pass

Brightness peak-to-peak under 5 % at the chosen exposure.

06

## Ask the organisers now

IAR-LARC 2026 is a first edition, held at the UMNG Calle 100 campus (Cra 11 #101-80, Bogotá) on 2–6 November 2026. Nothing public describes the hall. The IAR site lists Diego S. D'Antonio (Oakland University) and David Saldaña (Lehigh University) as contacts.

1. Which building and hall? Photos of the floor, please.
2. Floor material, colour and finish: wood, synthetic sports floor, polished concrete, tiles? Glossy or matte? Painted lines?
3. Lighting: LED, fluorescent, metal-halide? Windows or skylights, and does sunlight reach the floor during the competition hours?
4. Ceiling height, air conditioning, doors that stay open.
5. How is the 10 × 5 m competition area marked, and may teams put anything on the floor?
6. WiFi: is 2.4 GHz crowded, may teams bring a router, and is a ground-station laptop allowed at the arena edge?
7. When is our dedicated practice time (Rule 23), and does the event start on the 2nd or the 4th?

07

## Test sketches

Arduino sketches for the arduino-esp32 core 3.x (the camera firmware needs 2.0.x or newer). All were syntax-checked against the real `esp32-camera` headers or API stubs, but none was compiled with the Arduino core, and nothing has run on hardware yet. An equivalent ESP-IDF v5.3 version of the camera firmware builds cleanly for the ESP32. Pins are `#define`s at the top; the defaults follow the team's wiring plan, with the bench-only sensors on the S3-Zero's free back pads (GPIO17/18, 38/39).

tof_check.inoT1, T2 · ESP32-S3 + VL53L1X · library "VL53L1X" by Pololu

```
// tof_check: identify the ToF chip by register, then sweep distance mode x timing budget x ROI.
// Board: Waveshare ESP32-S3-Zero, arduino-esp32 core 3.x, "USB CDC On Boot: Enabled".
// Library: "VL53L1X" by Pololu. Serial 921600. Output lines (CSV, see TEST_PLAN.md):
//   D,mode,budget_ms,roi,idx,t_ms,range_mm,status,peak_mcps,ambient_mcps   one per reading
//   S,mode,budget_ms,roi,n,mean_mm,std_mm,pct_invalid,rate_hz,ambient_mcps,signal_mcps   one per setting
// ambient_mcps is the sensor's own reading of 940 nm background light: compare it between the lab
// and the venue to see how much brighter (in infrared) the venue is. It is not the datasheet's kcps/SPAD.
#include <Wire.h>
#include <VL53L1X.h>

// Pins are ASSUMPTIONS from the team's wiring plan; edit to match the real wiring.
#define PIN_SDA   4    // I2C data  (any GPIO works: ESP32-S3 routes I2C through the GPIO matrix)
#define PIN_SCL   5    // I2C clock
#define PIN_XSHUT 16   // ToF shutdown, active low
#define PIN_INT   15   // ToF data-ready (unused here; polled)
#define N_SAMPLES 300
#define SWEEP_ROI 1    // 1: also run 8x8 ROI after 16x16 (doubles the run time)

static const uint8_t ADDR = 0x29;
VL53L1X tof;

static bool rd(uint16_t reg, uint8_t nreg_bytes, uint16_t &val) {
  Wire.beginTransmission(ADDR);
  if (nreg_bytes == 2) Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t n = (nreg_bytes == 2) ? 2 : 1;   // VL53L1X ID is 16 bit, VL53L0X ID is 8 bit
  if (Wire.requestFrom((int)ADDR, (int)n) != n) return false;
  val = Wire.read();
  if (n == 2) val = (val << 8) | Wire.read();
  return true;
}

// Returns true only for a VL53L1X. Expected IDs: VL53L1X 0xEACC at 0x010F (Pololu init()),
// VL53L0X 0xEE at 0xC0 (Pololu init() compares 0xEE; the 0xC0 address is from ST's API).
static bool identify() {
  uint16_t v;
  if (rd(0x010F, 2, v)) {
    Serial.printf("# reg 0x010F = 0x%04X -> %s\n", v, v == 0xEACC ? "VL53L1X (expected 0xEACC)" : "NOT a VL53L1X");
    if (v == 0xEACC) return true;
  } else Serial.println("# reg 0x010F: no ACK/read failed");
  if (rd(0xC0, 1, v)) Serial.printf("# reg 0xC0 = 0x%02X -> %s\n", v, v == 0xEE ? "VL53L0X (expected 0xEE)" : "unknown");
  else Serial.println("# reg 0xC0: no ACK/read failed");
  return false;
}

static void run_setting(VL53L1X::DistanceMode m, const char *mname, uint16_t budget_ms, int roi) {
  tof.stopContinuous();
  tof.setDistanceMode(m);
  if (!tof.setMeasurementTimingBudget((uint32_t)budget_ms * 1000UL)) {
    Serial.printf("# skip %s %u ms: budget rejected\n", mname, budget_ms); return;
  }
  tof.setROISize(roi, roi);
  tof.startContinuous(budget_ms);            // inter-measurement period = budget
  for (int i = 0; i < 3; i++) tof.read(true); // discard first readings after a mode change
  double sum = 0, sum2 = 0, amb = 0, sig = 0; int nvalid = 0, to = 0;
  uint32_t t0 = millis(), t1 = t0;
  for (int i = 0; i < N_SAMPLES; i++) {
    tof.read(true);
    t1 = millis();
    if (tof.timeoutOccurred()) to++;
    uint8_t st = (uint8_t)tof.ranging_data.range_status;  // 0 = RangeValid
    uint16_t r = tof.ranging_data.range_mm;
    Serial.printf("D,%s,%u,%d,%d,%lu,%u,%u,%.3f,%.3f\n", mname, budget_ms, roi, i, t1, r, st,
                  tof.ranging_data.peak_signal_count_rate_MCPS, tof.ranging_data.ambient_count_rate_MCPS);
    amb += tof.ranging_data.ambient_count_rate_MCPS;
    if (st == 0) { nvalid++; sum += r; sum2 += (double)r * r; sig += tof.ranging_data.peak_signal_count_rate_MCPS; }
  }
  double mean = nvalid ? sum / nvalid : 0;
  double sd = nvalid > 1 ? sqrt(max(0.0, (sum2 - nvalid * mean * mean) / (nvalid - 1))) : 0;
  Serial.printf("S,%s,%u,%d,%d,%.1f,%.2f,%.1f,%.2f,%.3f,%.3f  (timeouts %d)\n", mname, budget_ms, roi, N_SAMPLES,
                mean, sd, 100.0 * (N_SAMPLES - nvalid) / N_SAMPLES, 1000.0 * N_SAMPLES / (t1 - t0),
                amb / N_SAMPLES, nvalid ? sig / nvalid : 0.0, to);
}

void setup() {
  Serial.begin(921600);
  delay(1500);
  pinMode(PIN_INT, INPUT);
  pinMode(PIN_XSHUT, OUTPUT); digitalWrite(PIN_XSHUT, LOW); delay(10);
  pinMode(PIN_XSHUT, INPUT);  // release: the breakout pulls XSHUT up to its own rail
  delay(10);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  if (!identify()) { Serial.println("# STOP: not a VL53L1X, or wiring problem"); while (1) delay(1000); }
  tof.setBus(&Wire); tof.setTimeout(500);
  if (!tof.init()) { Serial.println("# STOP: Pololu init() failed"); while (1) delay(1000); }
  Serial.println("# aim the sensor at the target; starting in 10 s (send any char to start now)");
  for (int i = 0; i < 100 && !Serial.available(); i++) delay(100);
  while (Serial.available()) Serial.read();

  static const VL53L1X::DistanceMode modes[] = {VL53L1X::Short, VL53L1X::Medium, VL53L1X::Long};
  static const char *names[] = {"short", "medium", "long"};
  static const uint16_t budgets[] = {20, 33, 50, 100, 200};
  for (int roi = 16; roi >= (SWEEP_ROI ? 8 : 16); roi -= 8)
    for (int m = 0; m < 3; m++)
      for (int b = 0; b < 5; b++) {
        if (m > 0 && budgets[b] < 33) continue;   // ST minimum is 20 ms short-mode only (from memory; verify in UM2356)
        run_setting(modes[m], names[m], budgets[b], roi);
      }
  Serial.println("# done");
}
void loop() {}
```

ultrasonic_check.inoT3 · ESP32-S3 + HC-SR04 or US-100 + one DRV8833 channel

```
// ultrasonic_check: HC-SR04 (trig/echo) or US-100 (UART mode) at several ping periods,
// optionally with one DRV8833 channel spinning a motor to test acoustic/electrical noise.
// Serial 921600, line commands:  h | u  (select sensor)   m<0-100> (motor duty %, e.g. m40; m0 stops)
//                                go (3 periods x N samples)   Output: U,sensor,period_ms,duty,idx,t_ms,dist_mm,valid
#include <Arduino.h>

// Pins: the team's wiring plan uses 1 (battery), 2/6 (camera UART), 4/5 (I2C), 7-14 (motors), 15/16 (ToF).
// GPIO17/18 and 38-42 are the S3-Zero's unused back pads, clear of GPIO0 boot, 3/45/46 strapping,
// 19/20 USB, 21 RGB LED and 43/44 UART0. On a separate bench board any free GPIO will do.
#define PIN_TRIG  17   // HC-SR04 trigger (3.3 V logic is enough to trigger)
#define PIN_ECHO  18   // HC-SR04 echo, 5 V: MUST go through 1k (series) + 2k (to GND) divider -> ~3.3 V
#define PIN_US_RX 38   // US-100 TX -> here (Serial1)
#define PIN_US_TX 39   // US-100 RX <- here
#define PIN_M_IN1 13   // DRV8833 #1 IN1: PWM
#define PIN_M_IN2 12   // DRV8833 #1 IN2: held low (forward only). nSLEEP must be tied high.
#define N_SAMPLES 300
#define TEMP_C    20.0 // speed of sound depends on temperature, not altitude: 331.3+0.606*T m/s

static char sensor = 'h';
static int duty = 0;
static uint32_t last_cmd_ms = 0;

static void set_motor(int pct) {            // 1 s soft ramp to avoid a current spike on the supply
  pct = constrain(pct, 0, 100);
  int target = pct * 255 / 100, cur = duty * 255 / 100;
  for (int i = 0; i <= 20; i++) { ledcWrite(PIN_M_IN1, cur + (target - cur) * i / 20); delay(50); }
  duty = pct; last_cmd_ms = millis();
}

static int read_hcsr04() {                  // mm, or -1
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(12); digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 25000);   // 25 ms ~ 4.3 m
  if (us == 0) return -1;
  int mm = (int)(us * (331.3 + 0.606 * TEMP_C) / 2000.0);
  return (mm < 20 || mm > 4000) ? -1 : mm;
}

static int read_us100() {                   // UART mode (jumper on): send 0x55, get high/low byte of mm
  while (Serial1.available()) Serial1.read();
  Serial1.write(0x55);
  uint32_t t = millis();
  while (Serial1.available() < 2) if (millis() - t > 50) return -1;
  int mm = (Serial1.read() << 8) | Serial1.read();
  return (mm == 0 || mm > 4500) ? -1 : mm;
}

static void run_all() {
  static const int periods[] = {30, 60, 100};
  for (int p = 0; p < 3; p++) {
    double s = 0, s2 = 0; int nv = 0;
    for (int i = 0; i < N_SAMPLES; i++) {
      uint32_t t0 = millis();
      int mm = (sensor == 'h') ? read_hcsr04() : read_us100();
      Serial.printf("U,%s,%d,%d,%d,%lu,%d,%d\n", sensor == 'h' ? "hcsr04" : "us100", periods[p], duty, i, t0, mm, mm > 0);
      if (mm > 0) { nv++; s += mm; s2 += (double)mm * mm; }
      while (millis() - t0 < (uint32_t)periods[p]) {}
    }
    double m = nv ? s / nv : 0, sd = nv > 1 ? sqrt(max(0.0, (s2 - nv * m * m) / (nv - 1))) : 0;
    Serial.printf("S,%c,%d,duty%d,n%d,mean_mm %.1f,std_mm %.2f,invalid_pct %.1f\n", sensor, periods[p], duty, N_SAMPLES, m, sd,
                  100.0 * (N_SAMPLES - nv) / N_SAMPLES);
  }
  Serial.println("# done");
}

void setup() {
  Serial.begin(921600);
  pinMode(PIN_TRIG, OUTPUT); digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_M_IN2, OUTPUT); digitalWrite(PIN_M_IN2, LOW);
  ledcAttach(PIN_M_IN1, 20000, 8);          // arduino-esp32 3.x API; 2.x used ledcSetup+ledcAttachPin
  ledcWrite(PIN_M_IN1, 0);
  Serial1.begin(9600, SERIAL_8N1, PIN_US_RX, PIN_US_TX);
  delay(1500);
  Serial.println("# commands: h | u | m<duty%> | go");
}

void loop() {
  if (duty && millis() - last_cmd_ms > 120000) { set_motor(0); Serial.println("# motor watchdog: off"); }
  if (!Serial.available()) return;
  String c = Serial.readStringUntil('\n'); c.trim();
  last_cmd_ms = millis();
  if (c == "h" || c == "u") { sensor = c[0]; Serial.printf("# sensor %c\n", sensor); }
  else if (c.startsWith("m")) { set_motor(c.substring(1).toInt()); Serial.printf("# duty %d%%\n", duty); }
  else if (c == "go") run_all();
}
```

esp32cam_flow.inoT4–T8 · board "AI Thinker ESP32-CAM" · put flow.h and flow.c in the same folder

```
/*
 * esp32cam_flow.ino - optical-flow bench firmware for the AI-Thinker ESP32-CAM (OV2640).
 *
 * Purpose: measure on the real board (1) the frame rate the camera path really delivers in
 * grayscale at 96x96 / QQVGA, (2) the compute time of the block-matching flow, (3) how flow
 * quality depends on light, exposure and floor texture. It is a test tool, not flight code.
 *
 * Needs arduino-esp32 2.0.x or newer (camera_config_t.fb_location / .grab_mode).
 * Board: "AI Thinker ESP32-CAM", PSRAM enabled is fine but not used by this sketch.
 * flow.c / flow.h sit next to this file (copies of ../../host/flow.*, see sync_flow.py).
 *
 * Serial (USB-serial bridge, SERIAL_BAUD below):
 *   CSV, one line per frame:  t_us,frame_interval_us,dx,dy,quality,compute_us,mean,contrast
 *   dx,dy in pixels per frame (content motion, +x right, +y down after the flip options).
 *   Lines starting with '#' are messages (settings echo, frame dump, flicker report).
 *   Line rate: a CSV line is <= 64 bytes. At 30 fps that is 1.9 kB/s = 19.2 kbit/s on the wire
 *   (10 bits per byte), 17 % of 115200 baud. 921600 keeps a 64-byte line to 0.7 ms, so a
 *   print never eats the 33 ms frame slot, and a base64 frame dump (9216 B -> 12.3 kB for
 *   96x96, 25.6 kB for QQVGA) takes 0.13 s / 0.28 s instead of 1.1 s / 2.4 s.
 * Serial2 (UART2, SPARE_TX_PIN / SPARE_RX_PIN): 13-byte binary packet per frame for the main MCU.
 *
 * Commands (single letter, send without newline or with it):
 *   ? help/settings   p pause CSV   d dump one frame (base64)   x defaults
 *   r resolution 96x96 <-> QQVGA    w window 64 <-> 96   s row_step 1 <-> 2   c search 4 <-> 2
 *   e exposure auto <-> manual      + / - manual exposure (aec_value) +-25 (0..1200)
 *   a gain auto <-> manual          , / . manual gain -1 / +1 (0..30)
 *   g gain ceiling 2x..128x         n aec2 (DSP CTRL0 bit6) toggle   u sensor clock divider 4 <-> 2 (experimental)
 *   f 60 Hz banding filter on/off   v vflip   h hmirror   l flash LED (IO4) on/off
 *   k measure brightness flicker over 60 frames
 *   b binary packets on Serial2 on/off
 */
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "flow.h"

// ---------------------------------------------------------------- build-time options
#define SERIAL_BAUD        921600
#define FB_COUNT           2       // esp32-camera README recommends >1 buffer only for JPEG; with raw formats it runs
                                   // (continuous mode, CAMERA_GRAB_LATEST keeps 1 queued) but is untested here: if frames
                                   // tear or fb_get times out, set 1 and compare frame_interval_us
#define SPARE_TX_PIN       14      // UART2 TX to the main MCU (IO14: SD-card CLK, free if no SD)
#define SPARE_RX_PIN       13      // UART2 RX from the main MCU (IO13: SD-card DAT3, free if no SD)
#define SPARE_BAUD         115200
#define FLASH_LED_PIN      4       // white LED, also SD DAT1; keep off for flow (it changes exposure)

// CAMERA_MODEL_AI_THINKER pin map
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ---------------------------------------------------------------- runtime settings
struct Settings {
  framesize_t size;     // FRAMESIZE_96X96 or FRAMESIZE_QQVGA
  int  win;             // flow window 64 or 96
  int  row_step;        // 1 or 2
  int  search;          // 4 or 2
  bool autoExp;         // exposure control (AEC) on
  int  aec;             // manual exposure, OV2640 lines 0..1200
  bool autoGain;        // AGC on
  int  gain;            // manual gain 0..30
  int  ceiling;         // 0..6 -> 2x..128x
  bool aec2;            // driver set_aec2: OV2640 DSP CTRL0 bit6 (inverted); effect on frame time untested
  bool fast;            // experimental: sensor CLKRC divider 2 instead of the driver's 4 in CIF mode
  bool band60;          // banding filter for 60 Hz mains
  bool vflip;
  bool hmirror;
  bool led;
  bool csv;
  bool bin;
};
static Settings S;

static void defaults() {
  S.size = FRAMESIZE_QQVGA; S.win = 64; S.row_step = 1; S.search = 4;
  S.autoExp = true; S.aec = 200; S.autoGain = true; S.gain = 4; S.ceiling = 2;
  S.aec2 = false;      // not proven to be a night mode (see Settings); the 'n' key compares on/off
  S.fast = false;
  S.band60 = true;     // Colombia mains 60 Hz -> lights flicker at 120 Hz
  S.vflip = false; S.hmirror = false; S.led = false; S.csv = true; S.bin = false;
}

static flow_t   gFlow;           // 18 kB of window buffers, in internal RAM (static .bss)
static bool     gDump = false, gFlicker = false;
static int      gFlickN = 0;
static float    gFlickSum = 0, gFlickMin = 255, gFlickMax = 0;
static uint8_t  gSeq = 0;
static int64_t  gLastT = 0;
static const char* kCeil[] = {"2x", "4x", "8x", "16x", "32x", "64x", "128x"};

static int frame_w() { return S.size == FRAMESIZE_96X96 ? 96 : 160; }
static int frame_h() { return S.size == FRAMESIZE_96X96 ? 96 : 120; }

// ---------------------------------------------------------------- camera
static void apply_sensor() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
  s->set_exposure_ctrl(s, S.autoExp);
  if (!S.autoExp) s->set_aec_value(s, S.aec);
  s->set_gain_ctrl(s, S.autoGain);
  if (!S.autoGain) s->set_agc_gain(s, S.gain);
  s->set_gainceiling(s, (gainceiling_t)S.ceiling);
  s->set_aec2(s, S.aec2);
  s->set_lenc(s, 0);          // lens shading correction off: it adds a radial gain, not wanted
  s->set_whitebal(s, 0);      // grayscale: no point, and one less thing that adapts mid-test
  s->set_vflip(s, S.vflip);
  s->set_hmirror(s, S.hmirror);
  // OV2640 banding filter has no driver call. Bank-1 registers via set_reg (address bit 8 = bank):
  //   COM8 0x13 bit5 = banding filter enable;  COM3 0x0C bit2 = 0 selects 60 Hz, 1 selects 50 Hz.
  // From the datasheet; the flicker command 'k' is the proof that it does something.
  s->set_reg(s, 0x113, 0x20, S.band60 ? 0x20 : 0x00);
  s->set_reg(s, 0x10C, 0x04, 0x00);
  // Frame rate: for non-JPEG CIF-class sizes (QQVGA, 96x96) esp32-camera 2.1.8 ov2640.c set_window() writes
  // CLKRC = clk_div 3 (sysclk/4) on the ESP32; its own 30 fps@24 MHz table uses 1 (sysclk/2). So expect about
  // 12.5 fps at 20 MHz XCLK until measured. 'u' tries divider 2 (double the PCLK; may corrupt frames).
  s->set_reg(s, 0x111, 0x3F, S.fast ? 1 : 3);
}

static bool camera_start() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;           // 20 MHz is the OV2640 maximum
  c.pixel_format = PIXFORMAT_GRAYSCALE;
  c.frame_size = S.size;
  c.jpeg_quality = 12;                 // unused for grayscale, the driver wants a value
  c.fb_count = FB_COUNT;
  // QQVGA gray = 19.2 kB per buffer, two = 38.4 kB: fits internal RAM, and DRAM avoids the slow
  // PSRAM write path the esp32-camera README warns about. Falls back to PSRAM if DRAM is short.
  c.fb_location = CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;    // always the newest frame; stale ones are dropped
  esp_err_t e = esp_camera_init(&c);
  if (e != ESP_OK) {
    c.fb_location = CAMERA_FB_IN_PSRAM;
    e = esp_camera_init(&c);
    if (e == ESP_OK) Serial.println("# frame buffers in PSRAM (DRAM allocation failed)");
  }
  if (e != ESP_OK) { Serial.printf("# camera init failed 0x%x\n", (unsigned)e); return false; }
  apply_sensor();
  flow_params_t p; flow_default_params(&p);
  p.win = S.win; p.row_step = S.row_step; p.search_px = S.search;
  if (S.win > frame_w() || S.win > frame_h()) p.win = (frame_w() < frame_h() ? frame_w() : frame_h());
  if (flow_init(&gFlow, &p) != 0) { Serial.println("# flow_init failed"); return false; }
  gLastT = 0;
  return true;
}

static bool camera_restart() { esp_camera_deinit(); return camera_start(); }

// ---------------------------------------------------------------- output
static void print_settings() {
  Serial.printf("# size=%s win=%d row_step=%d search=%d exp=%s aec=%d gain=%s agc=%d ceiling=%s aec2=%d "
                "band60=%d vflip=%d hmirror=%d led=%d bin=%d fast=%d heap=%u\n",
                S.size == FRAMESIZE_96X96 ? "96x96" : "QQVGA", S.win, S.row_step, S.search,
                S.autoExp ? "auto" : "manual", S.aec, S.autoGain ? "auto" : "manual", S.gain,
                kCeil[S.ceiling], S.aec2, S.band60, S.vflip, S.hmirror, S.led, S.bin, S.fast,
                (unsigned)ESP.getFreeHeap());
}

static void dump_frame(const camera_fb_t* fb) {
  static const char* b64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  Serial.printf("#FRAME %u %u %u gray8 base64\n", (unsigned)fb->width, (unsigned)fb->height, (unsigned)fb->len);
  char line[77];
  size_t n = 0;
  for (size_t i = 0; i < fb->len; i += 3) {
    uint32_t v = fb->buf[i] << 16;
    if (i + 1 < fb->len) v |= fb->buf[i + 1] << 8;
    if (i + 2 < fb->len) v |= fb->buf[i + 2];
    line[n++] = b64[(v >> 18) & 63]; line[n++] = b64[(v >> 12) & 63];
    line[n++] = (i + 1 < fb->len) ? b64[(v >> 6) & 63] : '=';
    line[n++] = (i + 2 < fb->len) ? b64[v & 63] : '=';
    if (n >= 76) { line[n] = 0; Serial.println(line); n = 0; }
  }
  if (n) { line[n] = 0; Serial.println(line); }
  Serial.println("#END");
}

static uint8_t crc8(const uint8_t* d, int n) {   // poly 0x07, init 0
  uint8_t c = 0;
  while (n--) { c ^= *d++; for (int k = 0; k < 8; k++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1); }
  return c;
}

// 13 bytes, little endian: A5 5A | seq | flags | interval_us u16 | dx i16 | dy i16 | quality | valid | crc8
// dx, dy in 1/1000 px per frame; flags bit0 = ok; interval_us saturates at 65535.
static void send_packet(const flow_result_t& r, int64_t interval_us) {
  uint8_t p[13];
  int16_t dx = (int16_t)lrintf(r.dx_px * 1000.0f), dy = (int16_t)lrintf(r.dy_px * 1000.0f);
  uint16_t iv = interval_us > 65535 ? 65535 : (uint16_t)interval_us;
  p[0] = 0xA5; p[1] = 0x5A; p[2] = gSeq++; p[3] = r.ok ? 1 : 0;
  p[4] = iv & 255; p[5] = iv >> 8;
  p[6] = (uint16_t)dx & 255; p[7] = (uint16_t)dx >> 8;
  p[8] = (uint16_t)dy & 255; p[9] = (uint16_t)dy >> 8;
  p[10] = r.quality; p[11] = r.valid_blocks;
  p[12] = crc8(p + 2, 10);
  Serial2.write(p, sizeof p);
}

// ---------------------------------------------------------------- commands
static void handle(char ch) {
  sensor_t* s = esp_camera_sensor_get();
  bool restart = false, resensor = false;
  switch (ch) {
    case '?': print_settings(); return;
    case 'p': S.csv = !S.csv; break;
    case 'd': gDump = true; return;
    case 'x': defaults(); restart = true; break;
    case 'r': S.size = (S.size == FRAMESIZE_96X96) ? FRAMESIZE_QQVGA : FRAMESIZE_96X96;
              if (S.size == FRAMESIZE_96X96 && S.win > 96) S.win = 96;
              restart = true; break;     // new frame size needs new buffers: deinit + init
    case 'w': S.win = (S.win == 64) ? 96 : 64; restart = true; break;
    case 's': S.row_step = (S.row_step == 1) ? 2 : 1; restart = true; break;
    case 'c': S.search = (S.search == 4) ? 2 : 4; restart = true; break;
    case 'e': S.autoExp = !S.autoExp; resensor = true; break;
    case '+': S.aec = min(1200, S.aec + 25); S.autoExp = false; resensor = true; break;
    case '-': S.aec = max(0, S.aec - 25); S.autoExp = false; resensor = true; break;
    case 'a': S.autoGain = !S.autoGain; resensor = true; break;
    case '.': S.gain = min(30, S.gain + 1); S.autoGain = false; resensor = true; break;
    case ',': S.gain = max(0, S.gain - 1); S.autoGain = false; resensor = true; break;
    case 'g': S.ceiling = (S.ceiling + 1) % 7; resensor = true; break;
    case 'n': S.aec2 = !S.aec2; resensor = true; break;
    case 'u': S.fast = !S.fast; restart = true; break;   // re-init so the driver's PLL set-up is rewritten first
    case 'f': S.band60 = !S.band60; resensor = true; break;
    case 'v': S.vflip = !S.vflip; resensor = true; break;
    case 'h': S.hmirror = !S.hmirror; resensor = true; break;
    case 'l': S.led = !S.led; digitalWrite(FLASH_LED_PIN, S.led ? HIGH : LOW); break;
    case 'b': S.bin = !S.bin; break;
    case 'k': gFlicker = true; gFlickN = 0; gFlickSum = 0; gFlickMin = 255; gFlickMax = 0; return;
    default: return;
  }
  if (restart) { camera_restart(); }
  else if (resensor && s) { apply_sensor(); flow_reset(&gFlow); }
  print_settings();
}

// ---------------------------------------------------------------- main
void setup() {
  Serial.setRxBufferSize(256);       // must come before begin(), later calls are ignored
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(SPARE_BAUD, SERIAL_8N1, SPARE_RX_PIN, SPARE_TX_PIN);
  pinMode(FLASH_LED_PIN, OUTPUT); digitalWrite(FLASH_LED_PIN, LOW);
  delay(300);
  defaults();
  Serial.println();
  Serial.println("# esp32cam_flow: t_us,frame_interval_us,dx,dy,quality,compute_us,mean,contrast");
  if (!camera_start()) { while (true) delay(1000); }
  print_settings();
}

void loop() {
  while (Serial.available()) { char ch = (char)Serial.read(); if (ch > ' ') handle(ch); }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { delay(5); return; }
  // fb->timestamp is stamped by esp32-camera in cam_start_frame(), when the driver arms the capture of that
  // frame (right after the previous one finished), not at exposure or frame end. Its interval follows the sensor
  // cadence while the camera task keeps up, but the absolute time is about one frame older than the exposure.
  int64_t t = (int64_t)fb->timestamp.tv_sec * 1000000LL + fb->timestamp.tv_usec;
  int64_t interval = gLastT ? (t - gLastT) : 0;
  gLastT = t;

  flow_result_t r;
  int64_t c0 = esp_timer_get_time();
  int rc = flow_update(&gFlow, fb->buf, fb->width, fb->height, &r);
  uint32_t compute_us = (uint32_t)(esp_timer_get_time() - c0);

  if (rc == 0) {
    if (S.csv) {
      char buf[96];
      int n = snprintf(buf, sizeof buf, "%lld,%lld,%.3f,%.3f,%u,%lu,%u,%u\n", (long long)t, (long long)interval,
                       r.dx_px, r.dy_px, r.quality, (unsigned long)compute_us, r.mean, r.contrast);
      Serial.write((const uint8_t*)buf, n);
    }
    if (S.bin) send_packet(r, interval);
    if (gFlicker) {
      gFlickSum += r.mean; gFlickMin = min(gFlickMin, (float)r.mean); gFlickMax = max(gFlickMax, (float)r.mean);
      if (++gFlickN >= 60) {
        float m = gFlickSum / gFlickN;
        Serial.printf("#FLICKER mean=%.1f min=%.0f max=%.0f p2p=%.1f%%\n", m, gFlickMin, gFlickMax,
                      m > 0 ? 100.0f * (gFlickMax - gFlickMin) / m : 0.0f);
        gFlicker = false;
      }
    }
  }
  if (gDump) { dump_frame(fb); gDump = false; }
  esp_camera_fb_return(fb);
}
```

flow.hportable C11, no allocation

```
/* flow.h - block-matching optical flow for a downward camera (PX4Flow style).
 *
 * Portable C11 and nothing else: no allocation, no globals, no stdio, so the same
 * file builds on the host (clang), in the Arduino sketch and in ESP-IDF.
 *
 * Convention: dx_px/dy_px are the displacement of the IMAGE CONTENT between the
 * previous and the current frame, in pixels per frame, +x to the right and +y down
 * in the image. A camera moving forward over the floor makes the content move
 * the opposite way; the sign is fixed in the sketch (flip options), not here.
 * Ground speed [m/s] = flow_px_per_s * height_m / focal_px (pinhole, camera
 * looking straight down; tilt and rotation must be removed with the gyro first).
 */
#ifndef FLOW_H
#define FLOW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {   /* the Arduino sketch is C++ but compiles flow.c as C: without this the link fails */
#endif

#define FLOW_MAX_WIN      96   /* largest analysis window, px per side */
#define FLOW_BLOCK         8   /* block side, px */
#define FLOW_MAX_SEARCH    6   /* largest +-search the margins allow, px */
#define FLOW_MAX_BLOCKS  121   /* worst case is win 96 with search 1 or 2: ((96 - 2*(S+2)) / 8)^2 = 11^2; search 4 gives 10^2 */

typedef struct {
    int win;              /* analysis window side: 64 or 96 (centred crop) */
    int search_px;        /* +-integer search range, 1..FLOW_MAX_SEARCH (PX4Flow default 4..6) */
    int row_step;         /* 1 = every row of the block, 2 = every second row (halves cost) */
    int min_texture;      /* block needs max-min >= this (0..255); a plain floor gives no flow */
    int max_sad_per_px;   /* best match must average <= this abs diff per pixel, else mismatch */
    int contrast_ref;     /* frame contrast (std dev, grey levels) that earns full quality */
    int min_valid_blocks; /* fewer valid blocks than this and ok = 0 */
} flow_params_t;

typedef struct {
    float    dx_px;         /* sub-pixel content shift, px/frame */
    float    dy_px;
    uint8_t  quality;       /* 0..255: inlier share of all blocks, scaled by contrast */
    uint8_t  valid_blocks;  /* blocks passing texture + SAD checks */
    uint8_t  inlier_blocks; /* valid blocks agreeing with the median */
    uint8_t  ok;            /* 1 if enough valid blocks and a previous frame existed */
    uint8_t  mean;          /* window mean grey level (exposure check) */
    uint8_t  contrast;      /* window std dev, grey levels */
    uint32_t sad_ops;       /* integer-pel |a-b| evaluations executed this frame (early exit included) */
    uint32_t half_ops;      /* half-pel / fit |a-b| evaluations (each reads 1-4 pixels of cur) */
} flow_result_t;

typedef struct {
    flow_params_t p;
    uint8_t  buf_a[FLOW_MAX_WIN * FLOW_MAX_WIN];
    uint8_t  buf_b[FLOW_MAX_WIN * FLOW_MAX_WIN];
    uint8_t *prev;
    uint8_t *cur;
    int      have_prev;
} flow_t;

void flow_default_params(flow_params_t *p);
/* Returns 0 on success, -1 if the parameters do not fit the buffers. */
int  flow_init(flow_t *f, const flow_params_t *p);
void flow_reset(flow_t *f);  /* drop the previous frame (after a camera mode change) */
/* Crop the centred win x win window out of a w x h 8-bit frame, compare it with the
 * previous window and fill *out. Returns 0 on success, -1 if the frame is smaller
 * than the window. The first call after init/reset only stores the frame (ok = 0). */
int  flow_update(flow_t *f, const uint8_t *frame, int w, int h, flow_result_t *out);

#ifdef __cplusplus
}
#endif

#endif
```

flow.cblock matching with half-pixel refinement, for the bench

```
/* flow.c - integer SAD block matching with half-pixel refinement. See flow.h.
 *
 * Why this shape (PX4Flow, Honegger et al., ICRA 2013): a grid of small blocks, each
 * matched by SAD over a small search window, then a robust vote across blocks. One bad
 * block (a shadow, a glare patch) cannot move the median, and SAD on bytes is the
 * cheapest metric on a core with no SIMD.
 */
#include "flow.h"

#define HP_MAX  (2 * FLOW_MAX_SEARCH + 1)   /* half-pel range of one axis: -HP_MAX..HP_MAX */
#define HIST_N  (2 * HP_MAX + 1)

void flow_default_params(flow_params_t *p)
{
    p->win = 64;
    p->search_px = 4;
    p->row_step = 1;
    p->min_texture = 16;
    p->max_sad_per_px = 14;
    p->contrast_ref = 20;
    p->min_valid_blocks = 6;
}

int flow_init(flow_t *f, const flow_params_t *p)
{
    if (p->win < 32 || p->win > FLOW_MAX_WIN) return -1;
    if (p->search_px < 1 || p->search_px > FLOW_MAX_SEARCH) return -1;
    if (p->row_step < 1 || p->row_step > 2) return -1;
    {   /* the per-block arrays in flow_update are FLOW_MAX_BLOCKS long: refuse what would overflow them */
        int nb = (p->win - 2 * (p->search_px + 2)) / FLOW_BLOCK;
        if (nb < 1 || nb * nb > FLOW_MAX_BLOCKS) return -1;
    }
    f->p = *p;
    f->prev = f->buf_a;
    f->cur = f->buf_b;
    f->have_prev = 0;
    return 0;
}

void flow_reset(flow_t *f) { f->have_prev = 0; }

static inline int iabs(int v) { return v < 0 ? -v : v; }

/* SAD of the 8x8 block at prev(bx,by) against cur displaced by whole pixels (ox,oy).
 * Stops as soon as the running sum reaches 'best': a losing candidate is rejected
 * after a few rows, which is where most of the saving comes from. */
static int sad_int(const uint8_t *prev, const uint8_t *cur, int win, int bx, int by,
                   int ox, int oy, int row_step, int best, uint32_t *ops)
{
    int sum = 0;
    for (int r = 0; r < FLOW_BLOCK; r += row_step) {
        const uint8_t *a = prev + (by + r) * win + bx;
        const uint8_t *b = cur + (by + r + oy) * win + bx + ox;
        for (int c = 0; c < FLOW_BLOCK; c++) sum += iabs((int)a[c] - (int)b[c]);
        *ops += FLOW_BLOCK;
        if (sum >= best) return sum;
    }
    return sum;
}

/* Pixel of cur at half-pel position (x2/2, y2/2): average of the 1, 2 or 4 pixels
 * around it. Bilinear at exactly half-pel needs no multiplies. */
static inline int cur_half(const uint8_t *cur, int win, int x2, int y2)
{
    int x = x2 >> 1, y = y2 >> 1;
    const uint8_t *p = cur + y * win + x;
    int fx = x2 & 1, fy = y2 & 1;
    if (!fx && !fy) return p[0];
    if (fx && !fy) return (p[0] + p[1] + 1) >> 1;
    if (!fx && fy) return (p[0] + p[win] + 1) >> 1;
    return (p[0] + p[1] + p[win] + p[win + 1] + 2) >> 2;
}

static int sad_half(const uint8_t *prev, const uint8_t *cur, int win, int bx, int by,
                    int ox2, int oy2, int row_step, uint32_t *ops)
{
    int sum = 0;
    for (int r = 0; r < FLOW_BLOCK; r += row_step) {
        const uint8_t *a = prev + (by + r) * win + bx;
        for (int c = 0; c < FLOW_BLOCK; c++)
            sum += iabs((int)a[c] - cur_half(cur, win, 2 * (bx + c) + ox2, 2 * (by + r) + oy2));
        *ops += FLOW_BLOCK;
    }
    return sum;
}

/* Value at which the cumulative histogram reaches half of n: the median, no sorting. */
static int hist_median(const uint8_t *h, int n)
{
    int acc = 0;
    for (int i = 0; i < HIST_N; i++) {
        acc += h[i];
        if (2 * acc >= n) return i - HP_MAX;
    }
    return 0;
}

int flow_update(flow_t *f, const uint8_t *frame, int w, int h, flow_result_t *out)
{
    const flow_params_t *p = &f->p;
    const int win = p->win;
    const int S = p->search_px;
    const int M = S + 2;  /* margin: integer search, +1 px for the half-pel taps, +1 px for the sub-half-pel fit */
    if (w < win || h < win) return -1;

    /* swap and crop; the brightness statistics come out of the same pass */
    uint8_t *t = f->prev; f->prev = f->cur; f->cur = t;
    const int x0 = (w - win) / 2, y0 = (h - win) / 2;
    uint32_t sum = 0, sumsq = 0;
    for (int y = 0; y < win; y++) {
        const uint8_t *src = frame + (y0 + y) * w + x0;
        uint8_t *dst = f->cur + y * win;
        for (int x = 0; x < win; x++) { uint8_t v = src[x]; dst[x] = v; sum += v; sumsq += (uint32_t)v * v; }
    }
    const uint32_t npx = (uint32_t)win * win;
    const uint32_t mean = sum / npx;
    const int32_t var = (int32_t)(sumsq / npx) - (int32_t)(mean * mean);
    int sd = 0;                       /* integer sqrt, at most ~128 steps, only once per frame */
    while (sd * sd < var) sd++;

    out->dx_px = out->dy_px = 0.0f;
    out->quality = 0; out->valid_blocks = 0; out->inlier_blocks = 0; out->ok = 0;
    out->mean = (uint8_t)mean; out->contrast = (uint8_t)(sd > 255 ? 255 : sd);
    out->sad_ops = 0; out->half_ops = 0;

    if (!f->have_prev) { f->have_prev = 1; return 0; }

    const int nb = (win - 2 * M) / FLOW_BLOCK;         /* blocks per side */
    int8_t  bu[FLOW_MAX_BLOCKS], bv[FLOW_MAX_BLOCKS];   /* block vectors, half-pel units (vote) */
    int16_t fu[FLOW_MAX_BLOCKS], fv[FLOW_MAX_BLOCKS];   /* same, in 1/16 px with the fit (average) */
    uint8_t hu[HIST_N] = {0}, hv[HIST_N] = {0};
    int nvalid = 0;
    uint32_t ops = 0, hops = 0;

    for (int j = 0; j < nb; j++) {
        for (int i = 0; i < nb; i++) {
            const int bx = M + i * FLOW_BLOCK, by = M + j * FLOW_BLOCK;

            /* texture gate on the reference block: a flat block matches everywhere */
            int lo = 255, hi = 0;
            for (int r = 0; r < FLOW_BLOCK; r++)
                for (int c = 0; c < FLOW_BLOCK; c++) {
                    int v = f->prev[(by + r) * win + bx + c];
                    if (v < lo) lo = v;
                    if (v > hi) hi = v;
                }
            if (hi - lo < p->min_texture) continue;

            /* integer search; zero motion first so the early exit has a low bar at once */
            int best = sad_int(f->prev, f->cur, win, bx, by, 0, 0, p->row_step, 1 << 30, &ops);
            int bu0 = 0, bv0 = 0;
            for (int oy = -S; oy <= S; oy++)
                for (int ox = -S; ox <= S; ox++) {
                    if (!ox && !oy) continue;
                    int s = sad_int(f->prev, f->cur, win, bx, by, ox, oy, p->row_step, best, &ops);
                    if (s < best) { best = s; bu0 = ox; bv0 = oy; }
                }

            /* half-pel refinement: the 8 half-pel neighbours of the integer winner */
            int hbest = best, hu0 = 2 * bu0, hv0 = 2 * bv0;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    if (!dx && !dy) continue;
                    int s = sad_half(f->prev, f->cur, win, bx, by, 2 * bu0 + dx, 2 * bv0 + dy,
                                     p->row_step, &hops);
                    if (s < hbest) { hbest = s; hu0 = 2 * bu0 + dx; hv0 = 2 * bv0 + dy; }
                }

            /* mismatch gate, normalised by the pixels actually summed */
            int npix = FLOW_BLOCK * ((FLOW_BLOCK + p->row_step - 1) / p->row_step);
            if (hbest > p->max_sad_per_px * npix) continue;

            /* sub-half-pel: SAD falls roughly linearly toward the true position (it is a sum of
             * |differences|, not squares), so a symmetric V fit through the winner and its two
             * half-pel neighbours is closer than a parabola. offset in half-pel steps, |d| <= 0.5 */
            int sxm = sad_half(f->prev, f->cur, win, bx, by, hu0 - 1, hv0, p->row_step, &hops);
            int sxp = sad_half(f->prev, f->cur, win, bx, by, hu0 + 1, hv0, p->row_step, &hops);
            int sym = sad_half(f->prev, f->cur, win, bx, by, hu0, hv0 - 1, p->row_step, &hops);
            int syp = sad_half(f->prev, f->cur, win, bx, by, hu0, hv0 + 1, p->row_step, &hops);
            int ox16 = 0, oy16 = 0;   /* offsets in 1/16 px: one half-pel step = 8 units */
            {
                int mxx = sxm > sxp ? sxm : sxp, dnx = mxx - hbest;
                int myy = sym > syp ? sym : syp, dny = myy - hbest;
                if (dnx > 0) ox16 = (4 * (sxm - sxp)) / dnx;   /* = 8 * 0.5*(sxm-sxp)/dnx */
                if (dny > 0) oy16 = (4 * (sym - syp)) / dny;
                if (ox16 > 4) ox16 = 4;
                if (ox16 < -4) ox16 = -4;
                if (oy16 > 4) oy16 = 4;
                if (oy16 < -4) oy16 = -4;
            }
            bu[nvalid] = (int8_t)hu0; bv[nvalid] = (int8_t)hv0;
            fu[nvalid] = (int16_t)(8 * hu0 + ox16); fv[nvalid] = (int16_t)(8 * hv0 + oy16);
            nvalid++;
            hu[hu0 + HP_MAX]++; hv[hv0 + HP_MAX]++;
        }
    }
    out->sad_ops = ops; out->half_ops = hops;
    out->valid_blocks = (uint8_t)nvalid;
    if (nvalid == 0) return 0;

    /* robust vote: per-axis median, then average the blocks within 1 half-pel of it.
     * Averaging the per-block fits gives resolution finer than half a pixel. */
    const int mu = hist_median(hu, nvalid), mv = hist_median(hv, nvalid);
    int su = 0, sv = 0, nin = 0;
    for (int k = 0; k < nvalid; k++)
        if (iabs(bu[k] - mu) <= 1 && iabs(bv[k] - mv) <= 1) { su += fu[k]; sv += fv[k]; nin++; }
    out->inlier_blocks = (uint8_t)nin;
    if (nin == 0) return 0;
    out->dx_px = (float)su / (16.0f * (float)nin);
    out->dy_px = (float)sv / (16.0f * (float)nin);

    /* quality = agreement x contrast. Agreement is inliers over ALL blocks, so a frame
     * where most blocks were rejected for lack of texture scores low even if the rest agree. */
    int q = (255 * nin) / (nb * nb);
    int c = sd < p->contrast_ref ? sd : p->contrast_ref;
    q = (q * c) / p->contrast_ref;
    out->quality = (uint8_t)q;
    out->ok = (uint8_t)(nvalid >= p->min_valid_blocks);
    return 0;
}
```

08

## Method, limits and sources

### How the numbers were made

- **Simulator:** the repo's MuJoCo closed loop running the real `core/` controller through `build/libblimp.dll`. Courses 1 and 2, noise seeds 0–4. Altitude runs used 220 s; ground-speed runs used 300 s, because drafts slow the laps. The first 60 s (gyro-bias convergence) are excluded. Sensor and ground-station models lived in scratch harnesses; nothing in the repo was changed. Each experiment was re-run independently on several configurations by a second agent.
- **Optical-flow bench:** procedural floors, a pinhole OV2640 model (66° diagonal lens, 33 ms rolling-shutter readout, shot and read noise, 120 Hz flicker), and gyro noise and bias. Motion is replayed from the simulator plus parametric sweeps. There are 464 configurations, and three algorithms written the way they would run on a microcontroller.
- **Research:** six topic reports (ToF, ultrasonic, flow sensors, ESP32-CAM, alternatives, venue), each fact-checked against primary sources by a separate agent.

### What these results do not tell you

- Every flight number is a simulation of an unmeasured vehicle, with only 3–5 seeds per configuration, so differences of one or two loops are noise.
- The floors in the flow bench are synthetic. The real floor, its gloss and its 940 nm reflectance are unknown.
- The ESP32-CAM's 12.5 fps and all microcontroller timings are derived, not measured.
- The calibrated-heading flow modes re-align the heading at each touch using the known balloon spacing. That is an optimistic model of what a ground-station filter would do.
- Rule 7 is judged by our harness; the repo simulator does not check it. Every mode, including the stand-in, leaves the area in some runs.
- Fix B was tested only on course 2, at 35 gf, in a scratch copy of `core/`.
- Why the repo's own barometer model also fails (0 / 30) is not fully explained.
- Masses and currents for the ESP32-CAM, HC-SR04 and ToF board are seller figures. Weigh the real parts.

### Sources

- ST, [VL53L1X datasheet DS12385](https://www.st.com/resource/en/datasheet/vl53l1x.pdf): range tables, ambient definitions, model ID
- ST, [VL53L0X datasheet](https://www.st.com/resource/en/datasheet/vl53l0x.pdf): range limits, ID register
- Pololu, [vl53l1x-arduino](https://github.com/pololu/vl53l1x-arduino): driver used in tof_check
- ST community, [VL53L1X in sunlight](https://community.st.com/imaging-sensors-49/whether-vl53l1x-tof-sensor-working-in-sunlight-72244)
- Arduino forum, [TOF400F board with a VL53L0X](https://forum.arduino.cc/t/tof400f-sensor-help/889052)
- Benewake, [TF-Luna datasheet](https://files.waveshare.com/upload/c/c3/SJ-GU-TF-Luna_A05_Datasheet.pdf)
- Bosch, [BMP180 datasheet](https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf)
- ArduPilot, [sonar mounting and motor noise](https://ardupilot.org/copter/docs/sonar.html); MaxBotix, [troubleshooting guide](https://maxbotix.com/pages/maxsonar-troubleshooting-guide)
- Pimoroni, [PMW3901 breakout](https://shop.pimoroni.com/products/pmw3901-optical-flow-sensor-breakout); Bitcraze, [Flow deck v2](https://www.bitcraze.io/products/flow-deck-v2/)
- Bitcraze, [crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) (`mm_flow.c`): flow measurement model
- MicoAir, [MTF-01](https://micoair.com/?p=2191)
- PX4, [optical flow guide](https://docs.px4.io/main/en/sensor/optical_flow.html)
- Espressif, [esp32-camera](https://github.com/espressif/esp32-camera): README, `sensors/ov2640.c`
- arXiv [2505.24081](https://arxiv.org/abs/2505.24081): ESP32-CAM frame rate by frame size (JPEG over WiFi)
- R. Lut, [hx-esp32-cam-fpv](https://github.com/RomanLut/hx-esp32-cam-fpv): WiFi video latency
- Lehigh, [MochiSwarm](https://arxiv.org/abs/2503.03077) (arXiv 2503.03077): the baseline blimp's sensors
- Seguin et al., ICCA 2020, DOI 10.1109/ICCA51439.2020.9264514: GT-MAB camera localisation
- [IAR-LARC 2026 site](https://sites.google.com/oakland.edu/iar-larc/home); UMNG [LARC 2026 page](https://mecatronica.umng.edu.co/index.html/larc.html)
- IAR-LARC 2026 rulebook, Rules 1, 7, 13, 15, 22, 23 (in the repo's `reference/`)

Prepared for the DroneUN team, 11 October 2026. Simulation settings: seeds 0–4, both courses, 0.78 m envelope, 35 gf placeholder thrust unless stated.