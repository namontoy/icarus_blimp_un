# Known issues and design debt

Live list of what is wrong, unresolved, or deliberately deferred in the code.
Everything here was found by building and running, not by inspection.

**2026-10-27, repository merge.** This file and the numbered design docs came from the DroneUN
repo, merged into icarus_blimp_un on that date. The firmware they describe (`droneun_firmware.bin`,
the ESP-FLY board) now lives in `firmware/projects/espfly/` as the previous board's shell. The live
firmware is `firmware/projects/i2c_tools/` on the ESP32-S3-Zero (two DRV8833, reversible motors,
stand-alone MPU-6050 / QMC5883L / BME280 / VL53L1X breakouts, ESP32-CAM); its fixed-rate control
task, which will call `blimp_step()`, is the next step (`docs/control-loop-plan.md`). `core/`, `sim/` and
`tools/verify.py` are unchanged by the merge. Section 2's
"unmeasured vehicle" caveat still holds: one motor was thrust-tested at 14.7-16.8 gf on a lab
supply, and `core/src/params.c` still carries the 35 gf placeholder.

**2026-10-28, Wi-Fi drops fixed.** With the control task's first build the S3's access point
dropped the laptop every ~14 s. The cause was the task's esp_timer running with ISR dispatch,
which on ESP-IDF v5.5.x can stall every task-dispatched timer, Wi-Fi's included (esp-idf issue
#18808, fixed on master only). The timer is task-dispatched now; keep
`CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD` off until a release carries the fix. Report,
bisect and follow-ups: `docs/wifi-drop-report.md`; rerun `tools/wifi_soak.py` after any change
that can touch the radio.

Status at the current code (closed-loop rows: `run_course.py --max-time 220`, seed 0,
camera 58 x 45 degrees):

```
host unit tests   11,276 checks, 0 failures                    PASS
libblimp.dll                                                   BUILDS
droneun_firmware.bin  770 KB, 26% free, 0 warnings             BUILDS AND LINKS (esp32s3)
MuJoCo scene, two-body vehicle, real course geometry           ALL CHECKS PASS
closed loop, course 1, NO altitude sensor (the real PCB)       3/3 loops, best 19.0 s
closed loop, course 2, NO altitude sensor                      1/3 loops, best 47.9 s
closed loop, course 1, downward ToF fitted                     3/3 loops, best 23.0 s
closed loop, course 2, downward ToF fitted                     3/3 loops, best 23.5 s
```

Seed 0 is one draw of the sensor noise. Across seeds 0-4 (same settings):

```
                     NO altitude sensor                downward ToF
course 1              6/15 loops, best 19.0-78.7 s     15/15 loops, best 21.6-24.0 s
course 2              8/15 loops, best 25.9-113.4 s    15/15 loops, best 23.0-24.2 s
failsafe tripped     in 10 of 10 runs                  in 0 of 10 runs
```

Measured 2026-10-22 after the simulated camera was narrowed from 90 x 70 to the
lens the team actually owns (58 x 45 degrees, section 2). With the old camera the
sensorless rows were 12/15 (best 20.0-48.3 s) and 7/15 (33.7-56.9 s); the ToF rows
did not move. With the ToF sensor the vehicle completes both courses in simulation;
on the confirmed hardware alone it completes about half the loops.
Every number above is a simulation of an unmeasured vehicle; see section 2.
The default `--max-time` (150 s) includes the 60 s gyro-bias convergence, so it
usually ends a run before the third loop - use 220 s to compare against this table.

The tables above were measured on Windows 11. The same sweep on Ubuntu 22.04
(2026-10-27, clang 14, glibc, MuJoCo 3.13.0, same settings) gives different runs
per seed, because the two maths libraries round differently and the closed loop
amplifies the difference; runs on one platform stay deterministic per seed:

```
                     NO altitude sensor                downward ToF
course 1              9/15 loops, best 19.0-34.6 s     15/15 loops, best 21.8-28.1 s
course 2              5/15 loops, best 27.9-105.3 s    15/15 loops, best 21.9-24.2 s
failsafe tripped     in 10 of 10 runs                  in 0 of 10 runs
```

The conclusions hold on both platforms: with the ToF every loop completes and the
failsafe never trips; without it about half the loops complete and every run
trips the failsafe. Compare a change against the sweep of the platform it ran on.

---

## 1. What changed and why (the airframe is not a bicopter)

The professor's PCB is the Seeed x Max Imagination **ESP-FLY** flight controller:
XIAO ESP32S3, bare MPU-6050, four SI2300 low-side MOSFETs driving four brushed
coreless motors. That means:

- **unidirectional thrust** on every motor - no reverse, no braking, no tilt
  servo, no lateral thruster;
- **no altitude sensor, no magnetometer** - the only sensors are a gyro and an
  accelerometer, plus whatever the camera sees.

The mixer, guidance, estimator input path, scene generator, sim harness, motor
driver and board config were all rewritten around that. The old tilting-bicopter
model (mixer, servo driver, MCPWM/ESC motor driver, `tilt`/`thrust_lat` fields,
lateral-trim budget) is gone. The layout chosen: front pair pushes forward
(surge + differential yaw), rear pair pushes up (heave), the vehicle trimmed a
few gf **heavy** so the rear pair holds it and "motors off" sinks gently to the
floor - which is now also the failsafe.

---

## 2. What the simulation numbers do NOT tell you

- **Motor thrust is a placeholder.** `motor_max_thrust_n` (35 gf per motor) and
  motor.c's thrust-to-duty curve are guesses. Measure both on a thrust stand
  before any flight ([docs/07 section 8](07-control-design.md)). Rule 13
  compliance rests on that number.
- **Perception is synthetic.** The sim's camera has a 58 x 45 degree field of
  view, 2 degrees of bearing noise and 8 % range noise. That field is the widest
  lens among the team's 14 OV2640 modules, measured with `tools/lens_test/` on
  2026-10-22 (table at the end of this section): three standard lenses of 44, 54
  and 57-59 degrees horizontal, no wide or fisheye lens among them. Until that
  measurement the sim assumed 90 x 70 degrees; narrowing it to the real lens
  halved the loops the sensorless board completes on course 1 (12/15 to 6/15)
  and left the ToF rows unchanged. Every "balloon not in view" problem below is
  therefore a field-of-view problem too. `tools/sim_sweep.py --fov-deg 54x42`
  simulates another lens without touching the code; the comparison of the
  team's three lens types is at the end of this section. The detector is
  planned to run on 240 x 180 frames. The sim has no pixels, so that size
  changes none of its numbers; at 58 degrees it means 4.2 mrad per pixel and a
  0.55 m balloon of 130/range px (22 px at 6 m), against which the simulated
  2-degree bearing noise is about 5 px, on the safe side.
- **The search hint and altitude fix are stand-ins.** In the sim they are true
  values plus noise. On the real system the ground station must compute them
  from the course geometry, the announced order and telemetry (hint), and from
  every red balloon in the image (altitude fix). Neither exists yet - see 4.
- **Pendulum and yaw inertia are estimates.** The two-body scene puts 170 g in
  the gondola 0.42 m below the envelope centre; the yaw inertia is 0.004 kg m^2.
  Weigh the gondola and re-run.

---

### Measured camera modules (2026-10-22)

All 14 modules are OV2640 (PID 0x26). Lens-to-wall 0.90 m, 1600 x 1200 stills,
four handwritten marks 25 cm apart on a tape measure clicked by hand, pinhole fit
over all marks (`lens_test.py --port COM3 live`, key `a`). Sharpness is the
Laplacian variance on the tape print at 0.9 m, comparable only within this table.
"Rotated" means the picture comes out upside down relative to the flex, so that
module type needs the driver's flip and mirror settings. Lens cards and stills are
in `build/lens_test/<letter>/` on the bench laptop (not committed).

| Module | HFOV x VFOV (deg) | Line fit (px) | Sharpness 0.9 m | Picture | Group |
|---|---|---|---|---|---|
| A (DCX-OV2640-V2, metal housing) | 44.5 x 34.1 | 0.0 | 235 | upright | narrow |
| C | 53.6 x 41.5 | 0.9 | 169 | upright | 54 deg |
| F | 53.1 x 41.1 | 1.2 | 114 | upright | 54 deg |
| H | 53.5 x 41.4 | 1.7 | 469 | upright | 54 deg |
| L | 54.6 x 42.3 | 2.1 | 226 | upright | 54 deg |
| M | 54.0 x 41.9 | 1.4 | 114 | upright | 54 deg |
| B | 58.7 x 45.8 | 0.9 | 899 | rotated | 58 deg |
| D | 56.8 x 44.2 | 1.6 | 195 | rotated | 58 deg |
| E | 56.8 x 44.1 | 2.8 | 80 | rotated | 58 deg |
| G | 57.3 x 44.5 | 4.8 | 159 | rotated | 58 deg (IR test: filter present) |
| I | 56.5 x 43.9 | 0.6 | 395 | rotated | 58 deg |
| J | 57.0 x 44.3 | 4.8 | 106 | rotated | 58 deg |
| K | 56.8 x 44.2 | 1.5 | 337 | rotated | 58 deg |
| N | 59.3 x 46.2 | 0.8 | 565 | rotated | 58 deg |

The groups are module types, not individual lenses: every rotated module is in the
58-degree group and every upright compact one in the 54-degree group. Sharpness
varies tenfold inside a group (focus differences), so the pick is B or N for the
widest sharp view, or H in the 54-degree group. Not measured yet: sharpness at
3 m, mass, and the IR filter on all but G.

### Lens comparison in simulation (2026-10-22)

Same sweep as the status tables (seeds 0-4, 220 s, 0.78 m envelope), with the
camera set to each lens type through `tools/sim_sweep.py --fov-deg`. The 90 x 70
row is the sim's old assumption, kept as a stand-in for a wide-angle module the
team does not own; it reproduces the previously recorded numbers exactly, so the
runs are deterministic per seed and the rows are directly comparable. Loops are
completed out of 15; times are the best-lap range.

| Camera (deg) | Modules | Course 1, no sensor | Course 1, ToF | Course 2, no sensor | Course 2, ToF |
|---|---|---|---|---|---|
| 44 x 34 | A | 6/15, 19.3-94.1 s | 13/15, 20.6-30.8 s | 2/15, 82.6-141.4 s | 11/15, 23.2-33.7 s |
| 54 x 42 | C F H L M | 6/15, 17.5-118.1 s | 14/15, 22.0-30.7 s | 7/15, 24.9-69.2 s | 13/15, 22.0-36.3 s |
| 58 x 45 | B D E G I J K N | 6/15, 19.0-78.7 s | 15/15, 21.6-24.0 s | 8/15, 25.9-113.4 s | 15/15, 23.0-24.2 s |
| 90 x 70 | none (wide lens, not owned) | 12/15, 20.0-48.3 s | 15/15, 22.4-27.0 s | 7/15, 33.7-56.9 s | 15/15, 22.3-27.1 s |

With the ToF, the 58-degree type is the narrowest lens that completes every
loop; the 54-degree type drops 3 of 30 and module A 6 of 30, each a lost
sighting followed by a long search (the missing loops had 80-120 s of budget
left, so they are not the clock running out), and the wide lens buys nothing
more. Without an altitude sensor every lens trips the failsafe in every run,
the wide one included, so no lens replaces the sensor. Fit B or N, and the ToF.

## 3. THE hardware decision: add a downward ToF sensor

The single most valuable change available. With no altitude sensor the vehicle
holds the 2 m waypoint plane only while a balloon is in the camera's view; the
moment it turns away, altitude has no reference and any vertical velocity it
picked up (a bounce off a balloon, an overshoot) simply persists. The any-balloon
altitude fix (section 4) makes this workable - it is what took vision-only from
1/4 to 3/3 on course 1 - but it is still one lost sighting away from a runaway.

A VL53L1X time-of-flight breakout (~2 g, ~USD 3, I2C, 4 m range) on the same bus
as the MPU-6050 removes the problem entirely: the estimator already fuses it,
the firmware already has `BLIMP_BOARD_HAS_TOF` and a reserved address, and the
sim's `--altitude-source tof` shows the result: across five noise seeds every
loop completed and the failsafe never tripped, against 14 of 30 loops and a
failsafe in every run without it (status table above). A BMP390 barometer is the weaker alternative
(+/-0.2 m noise against a 0.55 m balloon).

---

## 4. Ground station work that does not exist yet

The core expects, per command packet (`firmware/main/net/protocol.h`, v3):

| Field | What the ground station must compute |
|---|---|
| `bearing_rad`, `elevation_rad`, `range_m`, `visible` | the current target balloon in the 240 x 180 camera frame (red blob, apparent size for range) |
| `hint_yaw_rad`, `hint_valid` | expected absolute heading of the current target, from the fixed 4 x 2 m rectangle, the announced order, the last touched balloon and telemetry yaw |
| `alt_fix_m`, `alt_fix_valid` | vehicle altitude from the NEAREST balloon in view: every balloon sits at 2.0 m, so its elevation and range give our height. This is what keeps altitude alive between target sightings |

Nothing of this is written. It is the next major piece of work, and it needs the
real camera's field of view and the real camera stream.

---

## 5. Contract notes that bit during this rewrite

| Where | Note |
|---|---|
| `guidance.h` / `blimp.c` | `contact_detected` is a LEVEL, not an edge: CONFIRM dwells on it for `touch_confirm_s`. The real touch detector must latch its output (the sim latches for 0.35 s). Feeding it edges made the core re-touch balloon 1 seventy-five times. |
| `estimator.h` | `altitude_valid` is a 2.5 s window after the last absolute fix, not a per-tick flag; a fix after a gap longer than 0.5 s snaps altitude instead of differencing it into an absurd vertical rate. |
| `blimp.c` | Altitude damping uses the estimator's fused vertical rate, and only within 0.5 s of a fix - accel-only dead reckoning through a balloon collision was observed to be wrong by 0.7 m in 3 s. |
| `guidance.c` | A stale altitude error is re-applied only DOWNWARD. Climbing on stale information is how a vehicle with no altitude sensor reaches the ceiling (observed: 7 m). |
| `mixer.c` | 0.5 gf deadbands on yaw and surge. Without them the heading loop's noise floor kept one front motor creeping and walked the vehicle 0.5 m forward during PREPARE. |
| `blimp.c` | FAILSAFE has hysteresis (0.5 m below the ceiling to clear) and guidance resumes the run autonomously once clear - Rule 7 forbids human intervention. |
| `build_scene.py` | MuJoCo's default angular drag coefficient (1.5) on a sphere is ~200x the physical value; it made every scan cost 3 gf of continuous differential thrust. Set to 0.05. Contacts softened (solref 0.06 s) - the default made a latex envelope meeting a mylar balloon launch the vehicle upward at 2 m/s. |
| `sim/blimp_ffi.py` | The layout table is regenerated from a compiler probe on every header change; the import-time check compares against the DLL. |

---

## 6. Remaining smaller items

- `board_config.h`: the battery divider pin (GPIO2) and ratio (2.0) are inferred
  from the schematic, not from the ESP-FLY config - verify with a multimeter on
  D1. The IMU axis remap is still the identity guess - verify by hand.
- The motor-to-GPIO assignment (FL=7, FR=4, RL=3, RR=1) is this project's
  choice. Run the motor identification mode (`BLIMP_BOOT_MOTOR_ID_MODE`) with
  the props off and write down what actually spins.
- Without an altitude sensor, course 2 is slower than course 1 (best laps
  25.9-113.4 s vs 19.0-78.7 s) and neither is reliable (8 and 6 loops of 15).
  The figure-eight's diagonal legs are longer and its turns sharper, and every
  turn costs surge on a unidirectional pair - but with the ToF both courses run
  at the same pace (22-24 s), so most of the gap looks like altitude handling
  rather than geometry. Tuning target, not a bug.
- Startup still needs 60 s of stillness for the gyro bias to converge
  (`BLIMP_EST_CONVERGE_S`). Untimed under Rule 4, but plan the pre-run routine
  around it.
- The tether model is a 4-link capsule chain, not a rope; balloon rebound after
  a touch is approximate.
- The scene's camera site (`camera_dz` = -0.35 m, 5 cm ahead of the envelope
  centre) sits 4 cm above the gondola's top face, inside the envelope's lower
  surface; the real camera will mount on the gondola front a few centimetres
  lower. Once the gondola is built, measure the camera height and put it in both
  `params.c` (`cam_dz_m`, which the altitude fix uses) and `build_scene.py`.
- The sim calls a balloon visible only while its CENTRE is inside the field of
  view. With the camera 0.35 m below the 2 m balloon plane and a 45-degree
  vertical field, a balloon centre enters the frame from 0.90 m ahead of the
  envelope centre, while contact is at 0.67 m, so the sim reports the target
  lost for the last 0.2 m of every approach. A real detector still sees the
  balloon's lower half, so this is the conservative side; if the final approach
  is ever tuned on it, gate on the blob's extent instead of its centre.
