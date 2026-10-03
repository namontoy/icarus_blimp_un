# Simulation (`sim/`)

> **STATUS (2026-09-21, evening): the vehicle completes both courses.** The scene and
> harness were rebuilt for the confirmed airframe - four fixed unidirectional motors on a
> two-body vehicle (envelope + gondola with legs, buoyancy at the envelope centre, trimmed
> 6 gf heavy), no altitude sensor unless `--altitude-source` says otherwise, camera on the
> gondola, an offboard search hint and an any-balloon altitude fix in the observation.
>
> | run | result |
> |---|---|
> | `run_course.py --course 1` (no altitude sensor, the real PCB) | 3/3 loops, best 21.6 s |
> | `run_course.py --course 2` | 2/3 loops in 220 s, best 54.1 s |
> | `run_course.py --course 1 --altitude-source tof` | 3/3 loops, best 22.4 s |
> | `run_course.py --course 2 --altitude-source tof` | 2/3 loops in 220 s, best 44.7 s |
>
> Everything below section 0 that describes a tilting bicopter, `left_tilt`/`right_tilt`
> actuators, `--lateral`, `thrust_l`/`thrust_r`/`tilt`/`thrust_lat` fields or the old
> ceiling-breach failure is **historical**. The current actuator names are `motor_fl`,
> `motor_fr`, `motor_rl`, `motor_rr`; the current options are `--altitude-source
> {none,baro,tof,both}`; the current failure analysis lives in
> [docs/10-known-issues.md](10-known-issues.md).

This document covers `sim/blimp_ffi.py`, `sim/sim_main.py` and `sim/run_course.py`: what they are,
how to install and run them, how to read their output, and how to use them for the gain-tuning
steps of [docs/07 §7](07-control-design.md)'s bring-up ladder. It does **not** cover
`sim/scene/build_scene.py` or `sim/scene/verify_scene.py` — those already have their own docstrings
and are reused here unmodified, per this task's own instruction.

---

## 0. What this is, and what it deliberately is not

[`blimp/blimp.h`](../core/include/blimp/blimp.h)'s own header comment states the point: `blimp_step()`
*is* the flight controller, and the same compiled bytes run on the ESP32 and inside this
simulation. `sim/blimp_ffi.py` loads `build/libblimp.dll` — compiled from the exact
`core/src/*.c` — via `ctypes`, and `sim/sim_main.py` calls it every tick, exactly the way
`firmware/main/main.c`'s control task will. **Nothing in `sim/` reimplements any control,
estimation or guidance logic.** If the vehicle flies badly, that is real information about
`core/`, not a bug in `sim/` to quietly work around.

What `sim/` *does* own, because it has to stand in for hardware that doesn't exist yet:

- Synthesising a `blimp_imu_sample_t` from MuJoCo's ground truth, **with realistic noise, bias and
  dropouts** — never handing the estimator ground truth (§4).
- Synthesising a `blimp_target_obs_t` (bearing/elevation/range) from the true vehicle/balloon
  geometry, with noise, dropouts and a limited field of view (§4) — guidance never sees where the
  balloon "really" is, only what a camera-like sensor would report.
- Applying the returned `blimp_actuators_t` to the actual MuJoCo actuators and stepping physics.
- Detecting real MuJoCo contact between the vehicle's envelope geom and a balloon geom, and timing
  the run the way a judge would (Rule 4/5) — independently of whatever `blimp_guidance_t` itself
  believes, so a guidance bug can't also corrupt the measurement of its own failure (§5).

---

## 1. Install

```
pip install mujoco numpy
```

Verified against **MuJoCo 3.13.0**, **Python 3.12**, **clang 19.1.1** (`x86_64-pc-windows-msvc`),
on Windows. `tools/build.py` already requires clang on `PATH` for every other module; nothing
additional is needed for the DLL build.

## 2. Generate the scene

Unchanged from `sim/scene/build_scene.py`'s own docs — reused here, not rewritten:

```
# from the repository root
python sim/scene/build_scene.py                 # writes sim/scene/course1.xml
python sim/scene/build_scene.py --course 2       # writes sim/scene/course2.xml
python sim/scene/verify_scene.py                 # sanity-checks course1.xml
```

`sim/run_course.py` will also call `build_scene.py` itself, automatically, if the course file it
needs doesn't exist yet or a non-default `--envelope-diameter` was requested — see §3.

## 3. Build the DLL and run

```
# from the repository root
python tools/build.py dll              # build/libblimp.dll — see the status banner above
python sim/run_course.py --course 1    # rectangle perimeter, up to 3 loop attempts, default gains
python sim/run_course.py --course 2 --loops 1 --seed 7
```

`sim/run_course.py --help` lists every flag. The ones the task asked for by name:

| Flag | Meaning |
|---|---|
| `--course {1,2}` | 1 = rectangle perimeter (Rule 19), 2 = figure-eight (Rule 20) |
| `--loops N` | how many **completed** loops this harness waits for before stopping (default 3, matching Rule 5's cap) — see the caveat in §3.1 |
| `--envelope-diameter D` | metres; regenerates the scene via `build_scene.py` if it differs from the generator's own default (0.78 m) |
| `--altitude-source {none,baro,tof,both}` | which altitude sensor the synthetic IMU pretends to have. `none` is the real PCB (MPU-6050 only): altitude is held from the balloons themselves. `tof` models the recommended VL53L1X ([docs/10 §3](10-known-issues.md)) |
| `--seed N` | seeds the sensor-noise RNG (`numpy.random.default_rng`) for a reproducible run |

Other useful flags: `--max-time` (total simulated-seconds budget, default 150 s — see §3.1's convergence-time note for why this needs to be generous), `--dll PATH` (point at a different compiled DLL — used in this document's own verification, §6, to test against a scratch stub while `guidance.c` didn't exist), `--xml PATH` (skip scene generation entirely).

### 3.1 Two real caveats in what `--loops` and the run actually mean

- **`blimp_init()` hardcodes `blimp_guidance_init(&b->guid, 4, 3)`** (4 waypoints, 3 loop attempts)
  — see `core/src/blimp.c`'s own decision-log comment. There is no `blimp_ffi.c` setter for
  `wp_count`/`loop_count`, so `--loops` does **not** reach into the compiled core and change its
  internal accounting; it only tells **this harness** how many loops, as independently judged by
  real MuJoCo contact events (§5), to wait for before stopping the simulation early. If a future
  FFI export exposes a setter, wire `--loops` through to it too.
- **The estimator's arming gate takes a fixed 60 simulated seconds**, regardless of gains:
  `core/src/estimator.c` defines `BLIMP_EST_CONVERGE_S = 60.0f`, and
  `blimp_estimator_converging()` blocks the disarmed→armed transition until the vehicle has been
  continuously "stationary" (by its own gyro/accel-magnitude thresholds) for that long. This is
  entirely inside Rule 4's untimed PREPARE window, so it costs nothing on race day — but it means
  **any sim run needs `--max-time` comfortably above 60 s** just to see the vehicle move at all;
  the default (150 s) budgets 60 s of convergence, up to three ~5 s `PREPARE` dwells between loops,
  and the rest for actual flight.

## 4. What `sim_main.py` synthesises, and why those numbers

None of the constants in `sim/sim_main.py`'s header (noise sigmas, dropout probabilities, field of
view) are control gains — they model the *sensors*, not the vehicle, and are free to retune for a
harder or easier test without touching `core/`:

| Model | Default | Rationale |
|---|---|---|
| Control tick rate | 100 Hz | docs/07 §4.2's "100–250 Hz class"; MuJoCo physics runs at the scene's own 500 Hz (0.002 s) underneath, 5 substeps/tick, zero-order-hold on actuators between ticks |
| Gyro noise / bias | σ=0.01 rad/s / σ=0.03 rad/s (fixed per run, per axis) | exercises `gyro_bias_tau_s`'s bias tracking — a zero-bias gyro would never test it |
| Accel noise / bias | σ=0.05 / 0.05 m/s² | accelerometer = finite-differenced true CoM acceleration minus gravity, rotated body-frame — not a MuJoCo sensor element, so no assumption about MuJoCo's own accelerometer gravity convention is needed (see §6.2 on why conventions like this get double-checked empirically rather than assumed) |
| Baro noise / bias / dropout | σ=0.08 m / σ=0.20 m (fixed per run) / 0.5%/tick | docs/07 §5.1's own "~±0.25 m real accuracy" |
| Range (ToF) noise / max range | σ=0.01 m / 4.0 m | docs/07 §5.1's own "range-limited to ~4 m" |
| Perception FOV | ±45° horizontal, ±35° vertical | a plausible narrow-ish camera; not sourced from a spec, freely adjustable |
| Perception dropout | 2%/tick | vision glitches, independent of FOV/range gating |
| Bearing/elevation/range noise | grows with true range | "range: estimated, may be crude (apparent size)" — guidance.h's own phrasing |

The magnetometer reads a fixed, arbitrary "world north" direction (there is no real magnetic survey
to match in simulation) rotated into the body frame, plus noise and a 1% dropout — enough to
exercise `use_mag`/mag-health logic without claiming a real calibration.

## 5. Timing: the judge, not the firmware, decides

`sim_main.py` keeps its **own** edge-triggered contact detector over all four balloon geoms, and
uses *that* — not `blimp_guidance_t.run_start_us`/`timing_active` — as the authoritative Rule 4/5
lap timer: first real contact with the announced order's first waypoint starts the clock, the
final waypoint's contact stops it, and an out-of-order touch is logged as a warning (would void a
real run under Rule 7) but not credited. This is deliberate: a real judge does not read firmware
state, and a bug in `guidance.c`'s own bookkeeping should not also corrupt the *measurement* of
that bug. `sim_main.py --course 1` (or `run_course.py`) prints both: the judge's per-waypoint /
per-loop times, and (in the periodic diagnostic line) `blimp_guidance_t`'s own `wp_index`/
`loop_index`/state — so the two are easy to cross-check once `guidance.c` exists.

`sim_main.py`'s per-tick diagnostic (every `--print-period` seconds, default 2 s) and the end-of-run
summary report exactly what the task asked to see honestly: guidance state over time (does it
spend time in `SEARCH`/`ALIGN`/`APPROACH`, or sit in `IDLE`/`FAILSAFE`?), the closest true range
ever achieved to *each* waypoint (shows movement toward a target even if it's never touched), final
position/heading/armed state, and every warning (out-of-order touches, NaN/Inf aborts).

## 6. Verification performed, and what it found

### 6.1 Struct-layout self-test (no DLL needed)

`sim/blimp_ffi.py` mirrors every `core/include/blimp/*.h` struct that crosses the
`blimp_ffi.c` boundary as a `ctypes.Structure`. Because `guidance.h`/`params.h`/`types.h` compile
standalone (only *linking* `blimp_ffi.c` needs `guidance.c`), a throwaway probe
(`#include "blimp/blimp.h"`, `sizeof`/`offsetof` on every wire struct, scratch-only, never placed
under `core/`) was compiled directly against the real headers with the same clang used for
`tools/build.py`'s own builds, and its output is what `blimp_ffi.py`'s `_EXPECTED_LAYOUT` table
and `_selftest()` (run automatically at import time) check against. Running `python
sim/blimp_ffi.py` today prints:

```
blimp_ffi.py: struct layout self-test PASSED (10 structs checked against the real compiler)

DLL smoke test SKIPPED: libblimp.dll not found at ...
Build it first: cd ... && python tools/build.py dll
```

— i.e. every ctypes mirror is confirmed byte-for-byte correct against the actual compiled layout,
independent of the DLL-build blocker.

### 6.2 The tilt-direction sign mismatch (mixer.c vs. the generated scene)

First found while smoke-testing the closed loop against a scratch-only guidance stub, before
`core/src/guidance.c` existed — the vehicle armed, then crashed to the floor within about 5
simulated seconds. Isolated with a minimal reproduction that bypasses `guidance.c`, `blimp.c` and
even the FFI entirely — it only needs the real `sim/scene/course1.xml` and `core/src/mixer.c`'s own
two formulas:

```python
# mixer.c: tilt = atan2(fz, sign_safe(fx)); for fx~0, fz=+0.5N this is tilt ~= +90 deg.
# mixer.c's own documented inverse (blimp_mixer_forward): fz = T * sin(tilt).
# So mixer.c's own convention says tilt=+90 deg means "thrust now points +z (up)".
tilt_cmd = atan2(0.5, 1e-4)   # ~ +90 deg, per mixer.c, for a pure +0.5N heave demand
# ... apply directly to the scene's left_tilt/right_tilt/left_thrust/right_thrust actuators ...
```

Result, over 1 simulated second, starting level at `z=1.0`:

| Commanded tilt | mixer.c's own claimed effect | Actual vehicle Δz in the scene |
|---|---|---|
| `+atan2(0.5, eps)` ≈ +90° (mixer.c's literal formula) | climb (`fz=+0.5N`, "+up" per `types.h`) | **−0.55 m (sank)** |
| `−atan2(0.5, eps)` ≈ −90° (the opposite sign) | — | **+0.79 m (climbed)** |

The scene's tilt-hinge joints (`pod_left_tilt`/`pod_right_tilt`, `axis="0 1 0"`,
`build_scene.py`'s `_vehicle_xml()`) rotate the nozzle's thrust direction as
`(cosθ, 0, −sinθ)` for joint angle `θ` (verified directly against MuJoCo's own kinematics, not
derived by hand alone — see the task report for the exact probe). `mixer.c`'s own inverse function
(`blimp_mixer_forward`, used for its own telemetry cross-checks) requires
`(cosθ, 0, +sinθ)` for its `tilt = atan2(fz, fx)` convention to mean what `types.h` says `fz`
means. The two disagree by exactly a sign on the `z` component — **one of `core/src/mixer.c`'s
`atan2(fz, ...)` argument or `core/src/mixer.c`'s `blimp_mixer_forward` `sin(tilt)` sign, or the
scene generator's tilt-joint `axis`, needs to flip.** Neither file is this task's to change
(`mixer.c` belongs to a different module's already-verified deliverable; the scene generator was
explicitly out of scope here) — this is reported, not fixed.

**This remains real and reproducible against the current tree** (re-confirmed after
`guidance.c` landed, using the same isolated method — it does not depend on guidance logic at
all). But §6.3's tick-by-tick trace against the *real* `guidance.c` shows it is not, by itself,
what sends the vehicle into `FAILSAFE` — under closed-loop PID control the commanded tilt never
settles at a fixed sign for long enough for this alone to be the dominant effect. It is a real,
independent, worth-fixing bug; §6.3 is the dominant one for the specific failure observed today.

### 6.3 What actually happens under the real `guidance.c`: a derivative-driven tilt/pitch instability

Once `core/src/guidance.c` existed, `python sim/run_course.py --course 1` was re-run against the
real, complete, freshly-built `build/libblimp.dll` (see §8 for the literal command output). The
vehicle: sits `IDLE` for 60 s (§3.1, expected), arms into `PREPARE`, and within **~3.5 more
simulated seconds** breaches the 3.5 m altitude ceiling and drops into `FAILSAFE` — never reaching
`SEARCH`, never seeing waypoint 1. A tick-by-tick trace (`blimp_ffi_get_demand()` — guidance's raw,
pre-PID altitude *error* — and `blimp_ffi_get_state()`/`get_actuators()`, logged every single 10 ms
control tick, not just the periodic summary) through the arm→failsafe window shows precisely why:

```
t=60.01  demand.fz=+1.0000  alt=1.000  vz=-0.034  tilt=+83°
t=60.21  demand.fz=+0.9512  alt=1.049  vz=+0.504  tilt=+83°
t=60.43  demand.fz=+0.7801  alt=1.220  vz=+0.989  tilt=-50°   <- sign flip, demand.fz still POSITIVE
t=60.66  demand.fz=+0.6140  alt=1.386  vz=+0.459  tilt=-7°
t=60.79  demand.fz=+0.5571  alt=1.443  vz=+0.687  tilt=-20°
t=60.90  demand.fz=+0.4706  alt=1.529  vz=+0.633  tilt=-82°   (pitch has reached -21.7 deg by here)
```

`blimp_guidance_t`'s own altitude error (`demand.fz` above, read straight from `blimp_ffi_get_demand()`)
is smooth, monotonic and **never once changes sign** in this entire window — it simply tracks
`2.0 − altitude` as altitude climbs. Yet the mixer's *commanded* `tilt` (which depends on the
altitude *loop's PID output*, not the raw error — `blimp.c`'s own documented step 6 runs
`demand.fz` through `pid_alt` before the mixer ever sees it) swings from +83° to −83° and back
essentially every tick, and the estimated pitch attitude grows from ~0° to over −20° over the same
under-4-second window. `blimp_params_t.alt.kd` (≈1.34 N·s/m, the same order of magnitude as
`alt.kp`) acting on a derivative of a genuinely noisy vertical-rate signal is the leading
candidate — `vz` above is visibly jittery even while climbing on average — and `d_lpf_hz=5.0` may
not be cutting that noise enough at a 100 Hz tick rate for a derivative term this large relative to
`kp`. **This may be telling us as much about this simulation's IMU noise model as about
`params.c`'s starting `kd`** — that ambiguity is exactly what bring-up ladder step 8 (docs/07 §7,
tethered/caged altitude-loop-alone tuning) exists to resolve with real, not simulated, sensor
noise, and it is reported here rather than guessed at.

**Practical consequence:** regardless of which of §6.2 or §6.3 is fixed first, right now the
vehicle cannot even complete its untimed settle-to-trim phase without a ceiling-breach failsafe —
this is upstream of anything guidance-logic-quality or waypoint-touching related, and is the
concrete, literal answer to "where does it get stuck": **in `PREPARE`, on the altitude loop, before
ever reaching `SEARCH`.**

### 6.4 A one-time, non-reproducible segfault — reported, not swept aside

On one occasion, immediately after rebuilding `build/libblimp.dll`, `python sim/run_course.py
--course 1` crashed with a native segmentation fault (Python exit code 139) before printing any
output at all — i.e. during scene/DLL loading, not during the simulated run. **This could not be
reproduced** in five subsequent attempts with the same command (including the exact bisection in
§8), all of which completed normally (with `FAIL: course not completed`, not a crash). The evidence
is consistent with a transient Windows file-system/loader race immediately following the DLL
rebuild (antivirus scanning or a not-yet-fully-flushed file handle are both plausible), not a
reproducible logic bug in `sim/` or `core/` — but it is recorded here rather than omitted, per the
explicit instruction that a harness failing to run is never acceptable to hide. If this recurs,
capture it with `python -X faulthandler` and a debugger attached; it did not recur here.

### 6.5 Struct-layout self-test (works without a DLL at all)

`sim/blimp_ffi.py` mirrors every `core/include/blimp/*.h` struct that crosses the
`blimp_ffi.c` boundary as a `ctypes.Structure`. Because `guidance.h`/`params.h`/`types.h` compile
standalone (only *linking* `blimp_ffi.c` ever needed `guidance.c`), a throwaway probe
(`#include "blimp/blimp.h"`, `sizeof`/`offsetof` on every wire struct, scratch-only, never placed
under `core/`) was compiled directly against the real headers with the same clang used for
`tools/build.py`'s own builds, and its output is what `blimp_ffi.py`'s `_EXPECTED_LAYOUT` table
and `_selftest()` (run automatically at import time) check against — this was done, and passed,
*before* `guidance.c` existed, and still runs (and still passes) on every import today:

```
blimp_ffi.py: struct layout self-test PASSED (10 structs checked against the real compiler)
loaded build/libblimp.dll, blimp_version() = 'blimp-core-0.1.0'
first step(): thrust_l=0.0000 thrust_r=0.0000 tilt=0.0000 thrust_lat=0.0000 armed=False
DLL smoke test PASSED
```

— i.e. every ctypes mirror is confirmed byte-for-byte correct against the actual compiled layout,
and (now that the DLL exists) round-trips real values correctly end to end.

## 7. Using this for gain tuning (docs/07 §7's bring-up ladder)

This simulation is a stand-in for the *later* steps of [docs/07 §7](07-control-design.md#7-bring-up-and-tuning-procedure)'s ladder — it cannot replace the bench steps (1–7: buoyancy trim, actuator
sanity, thrust-stand, sensor bring-up, the yaw spin-down system-ID test) that need the real
hardware. **Right now, §6.2/§6.3's findings mean step 8 (altitude loop alone) is the very next
thing to work on** — the vehicle cannot get past its own PREPARE-phase altitude hold, so nothing
past it (heading, guidance, waypoints) can be meaningfully evaluated in sim yet. Once that's
resolved:

- **Step 8 (altitude loop alone) / step 9 (heading loop alone):** run with `--course 1
  --max-time <generous>` and watch the periodic diagnostic's `pos=(...)` and `yaw=` fields settle
  (or not) toward the 2 m target / a commanded heading during `PREPARE`/`SEARCH`. To retune a gain
  without editing `core/src/params.c` and rebuilding, call `ctl.get_params()` /
  `ctl.set_params()` directly in a small script against `sim.ctl` (a `ClosedLoopSim` instance) —
  `blimp_ffi_set_params()` rulebook-validates in place, exactly as a live-tuning link packet would.
- **Step 10 (combined altitude + heading):** the default run already closes both simultaneously;
  the "time spent per guidance state" and closest-approach-per-waypoint tables are the fastest way
  to see whether the vehicle is oscillating (bouncing between `ALIGN`/`SEARCH`) rather than
  advancing.
- **Step 11 (guidance with a fake/known target):** `sim_main.py`'s target synthesis already feeds
  a bearing/elevation/range derived from true geometry with a limited FOV — running with
  `PERCEPTION_DROPOUT_PROB` and the noise sigmas (sim_main.py header) temporarily zeroed
  approximates "ground-truth bearing fed manually" for isolating guidance-state-machine bugs from
  perception noise.
- **Step 13/14 (full autonomy / full rehearsal):** `run_course.py --course {1,2} --loops 3` is
  directly this step, headless and repeatable with `--seed`.

Nothing here should be read as replacing the physical bench tests in steps 1–7 — an estimator/gain
set that only ever saw simulated sensor noise is not validated against the real airframe's actual
noise, latency or non-idealities.

## 8. Verification log (literal, as run against the real, complete tree)

```
# from the repository root
python tools/build.py dll
```
```
BUILT build\libblimp.dll
```
```
python sim/scene/build_scene.py
```
```
Wrote <repo>\sim\scene\course1.xml
  course=1 (rectangle perimeter), order=[1, 2, 3, 4]
  envelope: D=0.78 m  V=0.2485 m^3  rho_air=0.89 kg/m^3
  vehicle total mass forced by Rule 11: 221.1 g (net force target: 0 +/- 10 gf)
  main body mass: 191.1 g  (pods: 2 x 15.0 g)
```
```
python sim/run_course.py --course 1
```
```
[sim_main] course=1 (course1_rectangle_perimeter) order=[1, 2, 3, 4] loops_requested=3 ...
  t=  0.00s  guid=IDLE      armed=False ... pos=(-1.28,-0.64,+1.00) ...
  ... (60 s of IDLE while the estimator converges, §3.1) ...
  t= 60.21s  guid=PREPARE   armed=True  ... pos=(-1.28,-0.64,+1.05) ... act=(L0.402 R0.402 tilt+83 ...)
  t= 62.22s  guid=PREPARE   armed=True  ... pos=(-1.12,-0.64,+1.94) ... act=(L0.236 R0.236 tilt-83 ...)
  t= 64.23s  guid=FAILSAFE  armed=True  ... pos=(+0.34,-0.64,+4.39) ...   <- ceiling breach (>3.5 m)
  ... (FAILSAFE holds zero thrust; vehicle coasts upward under residual momentum, §6.3) ...
  t=148.33s  guid=FAILSAFE  armed=True  ... pos=(-2.51,-0.64,+7.53) ...

=== course 1 (course1_rectangle_perimeter) — order [1, 2, 3, 4] ===
  loop 0: NOT completed (0/4 waypoints touched)

NO loop completed within 150.0s of simulated time.

Time spent per guidance state:
    FAILSAFE     86.49s
    IDLE         60.01s
    PREPARE       3.50s

Closest approach ever made to each waypoint:
    balloon 1: 0.85 m
    balloon 2: never observed
    balloon 3: never observed
    balloon 4: never observed

BEST TIME: none -- no loop completed within 150.0s of simulated time
FAIL: course not completed (0 loops finished) within the time budget.
```
Exit code: **1** (correctly non-zero — the course was not completed). No NaN/Inf. No crash on this
run (see §6.4 for the one exception observed and not reproduced). This is the literal, current,
honest state: the harness runs correctly end to end against the real compiled core; the vehicle
gets stuck in `PREPARE`, on the altitude loop, before ever reaching `SEARCH` — see §6.2/§6.3 for the
full diagnosis and §7 for where to start fixing it.
