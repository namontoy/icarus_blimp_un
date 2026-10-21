# Known issues and design debt

Live list of what is wrong, unresolved, or deliberately deferred in the code.
Everything here was found by building and running, not by inspection.

Status at the current code (closed-loop rows: `run_course.py --max-time 220`, seed 0):

```
host unit tests   11,276 checks, 0 failures                    PASS
libblimp.dll                                                   BUILDS
droneun_firmware.bin  770 KB, 26% free, 0 warnings             BUILDS AND LINKS (esp32s3)
MuJoCo scene, two-body vehicle, real course geometry           ALL CHECKS PASS
closed loop, course 1, NO altitude sensor (the real PCB)       3/3 loops, best 21.6 s
closed loop, course 2, NO altitude sensor                      2/3 loops, best 54.1 s
closed loop, course 1, downward ToF fitted                     3/3 loops, best 22.4 s
closed loop, course 2, downward ToF fitted                     3/3 loops, best 23.6 s
```

Seed 0 is one draw of the sensor noise. Across seeds 0-4 (same settings):

```
                     NO altitude sensor                downward ToF
course 1             12/15 loops, best 20.0-48.3 s     15/15 loops, best 22.4-27.0 s
course 2              7/15 loops, best 33.7-56.9 s     15/15 loops, best 22.3-27.1 s
failsafe tripped     in 10 of 10 runs                  in 0 of 10 runs
```

The vehicle completes both courses in simulation on the confirmed hardware.
Every number above is a simulation of an unmeasured vehicle; see section 2.
The default `--max-time` (150 s) includes the 60 s gyro-bias convergence, so it
usually ends a run before the third loop - use 220 s to compare against this table.

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
- **Perception is synthetic.** The sim's camera has a 90 x 70 degree field of
  view, 2 degrees of bearing noise and 8 % range noise. The XIAO ESP32S3 Sense
  camera is narrower (~50 x 40 degrees). A narrower field of view makes every
  "balloon not in view" problem below worse. The team's OV2640 modules come
  with unknown lenses (the flex marking does not say); `tools/lens_test/`
  measures each one, and the measured field belongs in `sim/sim_main.py`'s
  `FOV_HALF_ANGLE_RAD` constants before the lens is chosen.
- **The search hint and altitude fix are stand-ins.** In the sim they are true
  values plus noise. On the real system the ground station must compute them
  from the course geometry, the announced order and telemetry (hint), and from
  every red balloon in the image (altitude fix). Neither exists yet - see 4.
- **Pendulum and yaw inertia are estimates.** The two-body scene puts 170 g in
  the gondola 0.42 m below the envelope centre; the yaw inertia is 0.004 kg m^2.
  Weigh the gondola and re-run.

---

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
loop completed and the failsafe never tripped, against 19 of 30 loops and a
failsafe in every run without it (status table above). A BMP390 barometer is the weaker alternative
(+/-0.2 m noise against a 0.55 m balloon).

---

## 4. Ground station work that does not exist yet

The core expects, per command packet (`firmware/main/net/protocol.h`, v3):

| Field | What the ground station must compute |
|---|---|
| `bearing_rad`, `elevation_rad`, `range_m`, `visible` | the current target balloon in the camera image (red blob, apparent size for range) |
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
- Without an altitude sensor, course 2 is much slower than course 1 (best laps
  33.7-56.9 s vs 20.0-48.3 s). The figure-eight's diagonal legs are longer and
  its turns sharper, and every turn costs surge on a unidirectional pair - but
  with the ToF both courses run at the same pace (22-27 s), so most of the gap
  looks like altitude handling rather than geometry. Tuning target, not a bug.
- Startup still needs 60 s of stillness for the gyro bias to converge
  (`BLIMP_EST_CONVERGE_S`). Untimed under Rule 4, but plan the pre-run routine
  around it.
- The tether model is a 4-link capsule chain, not a rope; balloon rebound after
  a touch is approximate.
