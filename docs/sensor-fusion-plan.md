# Plan: `blimp_ins` — a Fused Navigation Unit for the Blimp

**Status: PROPOSAL — awaiting approval. Nothing here is implemented.**
**Date:** 2026-10-11
**Companion document:** [sensor-bringup-report.md](sensor-bringup-report.md)

The goal: turn the five bench drivers into one component that continuously
publishes the blimp's best estimate of **where it is and how it is oriented**,
at a rate and quality good enough to close a position control loop around.

The user's phrase for this was *"more than an IMU"*. The correct term is an
**AHRS/INS** — Attitude and Heading Reference System with altitude. Hence the
proposed component name `blimp_ins`.

---

## 1. Observability: what this sensor set can and cannot deliver

**This section is the most important part of the plan.** No filter can estimate
a quantity that no sensor observes. Before choosing algorithms, here is what is
actually recoverable:

| State | Observable? | From | Expected quality |
|---|---|---|---|
| **Roll, pitch** | ✅ Yes | gyro + accel (gravity vector) | ~0.5–1° after calibration |
| **Yaw / heading** | ⚠️ Conditionally | gyro + magnetometer | ~2–5° *if* mag is calibrated and motors don't swamp it (§9.4 of the report) |
| **Angular rates** | ✅ Yes | gyro, bias-corrected | ~0.2 °/s |
| **Altitude** | ✅ Yes | PING))) + baro + accel | ~2–5 cm, well inside the 40 cm hover band |
| **Vertical velocity** | ✅ Yes | derived from the above | ~5 cm/s |
| **Horizontal position** | ❌ **No** | — | **unbounded drift** |
| **Horizontal velocity** | ❌ **No** | — | **unbounded drift** |

### 1.1 Why horizontal position is not merely "hard"

Dead reckoning from the accelerometer does not work, and this is arithmetic, not
pessimism. Position error from an uncorrected accelerometer bias `b`:

```
error(t) = ½ · b · t²
```

| Accel bias | Error @ 1 s | @ 10 s | @ 60 s |
|---|---|---|---|
| 70 mg (**current, uncalibrated**) | 0.34 m | 34 m | 1 236 m |
| 10 mg (good calibration) | 0.05 m | 4.9 m | 177 m |
| 1 mg (excellent, temp-stabilised) | 0.005 m | 0.49 m | 17.7 m |

Even a **1 mg** bias — far better than an MPU-6050 will hold across the
temperature ramp measured in §9.5 — drifts half a metre in ten seconds. A blimp
flies minutes-long missions. There is no gain tuning, no smarter filter, and no
amount of magnetometer or barometer data that fixes this: those sensors observe
*orientation* and *altitude*, so they constrain neither horizontal axis.

**Conclusion: precise horizontal position requires a new sensor.** The plan
below is structured so that everything *except* horizontal position can be built
and validated now, with a clean insertion point for whichever sensor is chosen.

### 1.2 Horizontal position sensor options (decision needed — see §9)

| Option | Gives | Accuracy | Mass | Cost | Needs |
|---|---|---|---|---|---|
| **Optical flow** (PMW3901 / PAA5100) | body-frame **velocity** | ~5 cm/s | ~1 g | low | textured floor + a height estimate (PING))) supplies it) |
| **UWB anchors** (DW3000) | absolute **position** | ~10 cm | ~3 g/tag | medium | 3–4 fixed anchors in the room |
| **Downward camera + ArUco markers** | absolute **pose** | ~2 cm | ~5 g | low-med | markers on floor/ceiling, lighting, real compute |
| **Side-facing rangefinders** (2–3 more ToF/ultrasonic) | distance to walls | ~2 cm | ~3 g ea. | low | a known, convex, empty room |

**Recommendation: optical flow (PMW3901).** It is the lightest, cheapest and
infrastructure-free option; it pairs naturally with the PING))) height we already
need; and velocity (rather than position) is exactly what damps a blimp's slow
drift. It leaves position subject to slow random-walk, which is acceptable for
station-keeping but not for absolute waypoints — if absolute waypoints are
required, add UWB or markers on top.

### 1.3 Why a blimp is an unusually *favourable* platform for attitude

Worth stating explicitly, because it shapes the gain choices:
- **Specific force is tiny.** The airframe accelerates slowly, so the
  accelerometer is a near-pure gravity reference almost all the time. On a
  quadrotor the accel is mostly thrust and manoeuvre; here it is mostly gravity.
  Accelerometer updates can therefore be trusted with **small R** (high weight).
- **Dynamics are slow.** Time constants of seconds. A **50 Hz** control loop is
  generous; a 200 Hz estimator is comfortable.
- **But**: large surface area means **air currents dominate horizontal motion**,
  and that disturbance is unobservable without a position/velocity sensor. This
  reinforces §1.1.

---

## 2. Library selection (researched, with specifics)

### 2.1 Chosen: `espressif/esp-dsp` — ESP-DSP

| Property | Value |
|---|---|
| Latest version | **1.8.2** |
| Dependency | `idf.py add-dependency "espressif/esp-dsp^1.8.2"` |
| License | **Apache-2.0** |
| Supported chips | ESP32, **ESP32-S3**, ESP32-P4 — S3 gets hand-written Xtensa assembly |
| Provides | matrix (`dspm`), dot product, FFT, **IIR/FIR**, vector math, **Kalman/EKF** |

It is Espressif's *official* DSP library, so it tracks ESP-IDF and is optimised
for exactly our silicon.

**The EKF module.** Header `modules/kalman/ekf/include/ekf.h`.

Base class `ekf` — *"a base class for Extended Kalman Filter. It contains main
matrix operations and define the processing flow."*

| Method | Role |
|---|---|
| `ekf(int x, int w)` | constructor, allocates the matrices |
| `Init()` | pure virtual, call before first use |
| `Process(float *u, float dt)` | **main step** — `u` is the gyro input, `dt` the real elapsed time |
| `RungeKutta(dspm::Mat &x, float *u, float dt)` | state integration |
| `StateXdot(dspm::Mat &x, float *u)` | state derivative |
| `LinearizeFG(dspm::Mat &x, float *u)` | computes F and G |
| `CovariancePrediction(float dt)` | propagates P |
| `Update(dspm::Mat &H, float *measured, float *expected, float *R)` | measurement update |

**Derived class `ekf_imu13states` — and this is a remarkably good fit for the
GY-88.** Its 13 states:

| States | Meaning |
|---|---|
| `X[0..3]` | **attitude quaternion** |
| `X[4..6]` | **gyroscope bias error** (rad/s) |
| `X[7..9]` | **magnetometer amplitude vector** (soft-iron / scale) |
| `X[10..12]` | **magnetometer offset vector** (hard-iron) |

Reference magnetometer model: `magn_ampl × rotation_matrix' + magn_offset`.

Update methods:

| Method | Use |
|---|---|
| `UpdateRefMeasurement(float *accel, float *magn, float R[6])` | **primary method after calibration** — updates attitude and gyro bias only |
| `UpdateRefMeasurementMagn(float *accel, float *magn, float R[6])` | **calibration phase** — updates the *full* state including the magnetometer parameters |
| `UpdateRefMeasurement(float *accel, float *magn, float *attitude, float R[10])` | with a direct attitude quaternion, for init or constant-state |

Accelerometer input is in **g-units**. `R` is the measurement-noise diagonal —
lower means "trust the measurement more".

**Why this matters for us specifically:** two of the three worst problems in the
bring-up report are solved *by the filter itself*, online, continuously:
- §9.2, the **−2.66 °/s gyro X bias** → states `X[4..6]`, and because it is
  estimated continuously it tracks the temperature drift of §9.5 that a one-shot
  boot calibration would miss.
- §9.3, the **uncalibrated magnetometer** → states `X[7..12]` are precisely the
  hard-iron offset and soft-iron amplitude. Run `UpdateRefMeasurementMagn` during
  a figure-eight calibration manoeuvre, then switch to `UpdateRefMeasurement`.

**Two caveats, stated plainly:**
1. **The EKF module is C++**, not C — it uses classes and `dspm::Mat`. (The core
   DSP kernels are ANSI C plus assembly; the Kalman layer on top is C++.) So
   `blimp_ins` must compile as C++, or expose a C API over a C++ core. Proposed:
   **C++ internals, C header** — the rest of the firmware stays C.
2. **`ekf_imu13states` is attitude-only.** There are no position or velocity
   states. Altitude therefore needs a *second*, separate filter (§4.2). This is
   not a flaw; it is the right decomposition, since attitude and altitude have
   almost no coupling on a blimp.

**Bonus use of the same library:** `dsps_biquad_f32` / IIR gives us a cheap
low-pass or notch for motor-vibration rejection on the accelerometer, and FFT
lets us *measure* the vibration spectrum to place that notch properly.

### 2.2 Considered and kept as fallback: `espp/filters`

| Property | Value |
|---|---|
| Latest version | 1.3.7 |
| License | MIT |
| Language | C++ |
| Classes | `MadgwickFilter`, `ComplementaryFilter`, `KalmanFilter`, `BiquadFilter`, `ButterworthFilter`, `LowpassFilter`, `SimpleLowpassFilter`, `SosFilter`, `TransferFunction` |

`MadgwickFilter` implements Madgwick's orientation filter and supports both
6-axis and 9-axis IMUs.

**Why not primary:** Madgwick and Mahony are excellent, cheap and robust, but
they do **not** estimate gyro bias as a state (Mahony's integral term partially
compensates, Madgwick's does not) and neither estimates magnetometer hard/soft
iron. Given a measured −2.66 °/s bias and a wholly uncalibrated magnetometer,
the 13-state EKF's online estimation is worth its cost.

**Kept because:** it is the escape hatch. If the EKF turns out too expensive
(§8.1) or numerically troublesome, Madgwick drops in at a fraction of the cost.
Phase 5 therefore builds *both* behind one interface and compares them on real
data — that comparison is itself a useful result.

### 2.3 Rejected
- **Writing an EKF from scratch** — ESP-DSP's is Apache-2.0, S3-optimised, and
  already structured exactly for a 9-DOF IMU. No reason.
- **`esp-idf-lib` (UncleRus) sensor drivers** — our drivers are written, tested,
  and follow conventions this library does not (readback-after-write,
  distinguishable failure modes). It also had a legacy-API catch on this IDF
  version. No reason to swap now.

---

## 3. Target architecture

```
                   ┌──────────────────────────────────────┐
 L6  Interface     │ console cmds (bench) · telemetry · log│
                   └───────────────┬──────────────────────┘
                   ┌───────────────▼──────────────────────┐
 L5  State API     │ blimp_ins_get(&state)  — one struct,  │
                   │ one mutex/seqlock, timestamped        │
                   └───────────────┬──────────────────────┘
                   ┌───────────────▼──────────────────────┐
 L4  Fusion        │ attitude EKF (13 st.) · altitude KF  │
                   │ (4 st.) · tilt correction · gating    │
                   └───────────────┬──────────────────────┘
                   ┌───────────────▼──────────────────────┐
 L3  Calibration   │ apply offsets/scales · NVS store      │
                   └───────────────┬──────────────────────┘
                   ┌───────────────▼──────────────────────┐
 L2  Sampling      │ ins_task @200 Hz · slow_task @20 Hz   │
                   │ · ping_task · timestamps from esp_timer│
                   └───────────────┬──────────────────────┘
                   ┌───────────────▼──────────────────────┐
 L1  Drivers       │ mpu6050 · hmc5883l · bmp180 · ping ·  │
                   │ vl53l1x  — no printf, no bus ownership │
                   └───────────────┬──────────────────────┘
                   ┌───────────────▼──────────────────────┐
 L0  Resources     │ one i2c_master_bus · GPIO assignments │
                   └──────────────────────────────────────┘
```

### 3.1 The published state struct (sketch)

```c
typedef struct {
    int64_t  t_us;              /* esp_timer_get_time() at estimate validity   */

    /* attitude */
    float    q[4];              /* body→earth quaternion, EKF X[0..3]          */
    float    roll, pitch, yaw;  /* rad, derived; yaw valid only if mag_ok      */
    float    rate[3];           /* rad/s, bias-corrected gyro                  */
    float    gyro_bias[3];      /* rad/s, EKF X[4..6] — exposed for monitoring */

    /* altitude */
    float    alt_m;             /* above the surface under the airframe        */
    float    vz_ms;             /* +up                                         */
    float    alt_var;           /* estimator variance, for the controller      */

    /* horizontal — placeholder until §1.2 is decided */
    float    pos_xy[2];
    float    vel_xy[2];

    /* raw-ish, for logging and debugging */
    float    temp_c;
    float    press_hpa;
    float    mag_ut[3];

    /* health — the controller MUST check these */
    struct {
        bool imu_ok, mag_ok, baro_ok, ping_ok, tof_ok;
        bool attitude_valid, heading_valid, alt_valid;
        uint32_t imu_errors, i2c_errors, ping_timeouts, tof_bad_status;
        uint8_t  ping_reject_streak;
    } health;
} blimp_ins_state_t;
```

Design notes:
- **Health flags are not optional.** A controller that cannot tell a fresh
  estimate from a stale one will fly into a wall. `heading_valid` in particular
  must go false when the magnetometer is disturbed (§7).
- `alt_var` is published so the controller can soften its gains when the
  altimeter is being rejected.
- One writer (the fusion task), many readers → a seqlock or a short mutex.

---

## 4. Filter designs

### 4.1 Attitude: ESP-DSP `ekf_imu13states`

| Aspect | Choice |
|---|---|
| Prediction | `Process(gyro_rad_s, dt)` at **200 Hz**, `dt` **measured** from `esp_timer`, never assumed |
| Update | `UpdateRefMeasurement(accel_g, mag, R[6])` at **15 Hz** (magnetometer-limited) |
| Calibration mode | `UpdateRefMeasurementMagn(...)` while the operator performs a figure-eight |
| Initialisation | from a rest average: accel → roll/pitch, mag → yaw, seeded via the attitude-input overload |
| `R` for accel | **small** — a blimp's specific force is tiny (§1.3), so gravity is trustworthy |
| `R` for mag | **larger**, and inflated further when a disturbance is detected (§7) |

Must-dos:
- **Reject accelerometer updates when ‖a‖ deviates from 1 g** by more than a
  threshold — that means the airframe is accelerating and the vector is not
  gravity. Cheap, and prevents the classic manoeuvre-induced tilt error.
- Likewise **reject magnetometer updates when ‖m‖ strays** from its calibrated
  norm (motor current, ferrous structure).
- Renormalise the quaternion every step; watch for covariance loss of symmetry
  or positive-definiteness.

### 4.2 Altitude: a hand-written 4-state linear Kalman filter

`ekf_imu13states` has no altitude states, so this is a separate, small filter.
It uses `dspm::Mat` from the same library, so there is only one linear-algebra
dependency.

**States** `x = [ h, v_z, b_az, b_baro ]`

| State | Meaning | Why |
|---|---|---|
| `h` | altitude above the surface (m) | the controlled quantity |
| `v_z` | vertical velocity (m/s, +up) | damping term for the controller |
| `b_az` | vertical accelerometer bias (m/s²) | §9.1 — must be estimated, not assumed |
| `b_baro` | barometer altitude offset (m) | §9.6 — makes the baro's *absolute* error harmless while keeping its low noise and high rate |

**Prediction** at 50 Hz from the earth-frame vertical accelerometer (gravity
removed using the attitude from §4.1).

**Measurement updates**

| Source | Measures | Rate | Gating |
|---|---|---|---|
| **PING)))** | `h` (primary absolute) | 20 Hz | tilt-correct `h = d·cos θ`; reject on driver error; **innovation gate** (see below); reject if tilt > ~25° |
| **BMP180** | `h + b_baro` | 10 Hz | reject on sensor error |
| **VL53L1X** | `h` | 10 Hz | **only below ~1.3 m**, only when `ambient < 50 kcps/SPAD`, only `status == valid`, tilt-corrected |

**The innovation gate is essential, not a refinement.** The PING)))'s beam is
~20–30° wide, roughly **a metre across at 2 m** (§9.9). Furniture, a person
walking under the blimp, or a multipath return will produce a confident, wrong
reading. Rule: reject any sample whose innovation exceeds `3σ`; count
consecutive rejections; if the streak exceeds a limit, either the world changed
(a table really did appear) or the sensor failed — so widen the gate gradually
and **lower `alt_valid`** so the controller knows.

Also worth implementing: **prefer the smallest plausible return.** A blimp over
a floor with an obstacle gets a *shorter* echo from the obstacle, never a longer
one, so short outliers are real geometry and long outliers are multipath.

### 4.3 Horizontal: deferred until §1.2 is decided
Interface stub only. When optical flow is chosen, it becomes a 4-state
`[x, y, vx, vy]` filter driven by body-frame velocity rotated into the earth
frame (which needs attitude from §4.1 *and* height from §4.2 to scale the flow —
a nice illustration of why altitude must be solid first).

---

## 5. Rate and timing budget

| Item | Rate | Set by |
|---|---|---|
| MPU-6050 accel + gyro | **200 Hz** | DLPF + FIFO, INT-driven |
| HMC5883L | **15 Hz** (75 Hz available) | `CONFIG_A` ODR; 15 Hz with 8× averaging is the better noise trade for a slow airframe |
| BMP180 | **10 Hz** | 26 ms conversion at oss = 3 |
| PING))) | **20 Hz** | ~20 ms worst-case measurement cycle |
| VL53L1X | **10 Hz** | ≥100 ms budget, forced by the −5.5 % bias below that (§9.8) |
| Attitude EKF | 200 Hz predict / 15 Hz update | |
| Altitude KF | 50 Hz predict / per-sensor update | |
| Control loop | **50 Hz** | generous for blimp dynamics |

### 5.1 Required `sdkconfig` changes

| Setting | Now | Proposed | Why |
|---|---|---|---|
| `CONFIG_FREERTOS_HZ` | 100 | **1000** | a 10 ms tick cannot schedule a 200 Hz loop, and it is the root of every delay workaround in the report (§7.4) |
| I²C clock | 100 kHz | **400 kHz** | the sample budget at 200 Hz needs it; verify signal integrity with the current pull-ups first |
| `COMPILER_OPTIMIZATION` | `-Og` + asserts | **`-O2`** for flight builds | keep the debug build for the bench |
| MPU-6050 INT pin | unused | **assign a GPIO** | interrupt-driven FIFO reads instead of polling |

> Raising `FREERTOS_HZ` to 1000 will change the behaviour of the delay
> workarounds in `cmd_ping.c` and `vl53l1x_port.c`. Both are written as
> ceiling-division-plus-one and will still be *correct*, just less wasteful —
> but both comments must be revisited so they don't mislead later.

---

## 6. Phased delivery

Each phase has an acceptance test. Nothing proceeds on an unverified phase.

### Phase 0 — Commit the bench *(hours)*
Nothing in `firmware/` is committed and there are no commits on the branch
(§12 of the report). Commit the bench, the vendored ULD as its own auditable
commit, the pyserial helpers into `scripts/`, and both of these documents.
**Accept:** `git log` shows the work; a clean checkout builds.

### Phase 1 — Verify the PING))) ⚠️ *blocking* *(hours)*
It is the **primary altimeter** and **nothing about it is verified** (§9.9).
Build the 1.0 k/1.8 k divider, connect, and measure.
**Accept:** echo width matches a tape measure within 2 % from 0.3 m to 3 m;
GPIO6 never exceeds 3.3 V (scope or meter); ≥50 consecutive samples with no
driver error; characterise behaviour over the actual flight-surface floor.
**If this fails, the altimeter strategy must be reconsidered before anything
else is built on it.**

### Phase 2 — Lift drivers out of the console *(1–2 days)*
Create `components/blimp_sensors/`. Each driver becomes
`xxx_init(bus, cfg, &handle)` / `xxx_read(handle, &data)`: no `printf`, no bus
ownership, no per-call reconfiguration, handles held open. The existing console
commands are rewritten as **thin views over the same API** — so the bench keeps
working and becomes the regression test for the refactor.
**Accept:** every command of §5 of the report produces readings matching
Appendix B within noise.

### Phase 3 — Calibration *(2–3 days)*
New `cal` console command plus an NVS-backed store.
- **Gyro bias:** average N samples at rest (also a sanity check on the EKF's
  own estimate).
- **Accelerometer:** 6-position scale + offset solve. **This is the fix for
  §9.1 and it is mandatory** — the EKF's accel update assumes ‖a‖ = 1 g.
- **Magnetometer:** ellipsoid fit, *or* let the EKF estimate it
  (`UpdateRefMeasurementMagn`) and store the converged values as the seed.
- **IMU↔mag axis alignment** (§9.14): determine the relative rotation by
  rotating the board about each axis and comparing. Do not assume identity.
**Accept:** after calibration, ‖a‖ = 1.000 ± 0.005 g in all six orientations;
‖m‖ constant within ±3 % through a full rotation; resting gyro < 0.05 °/s.

### Phase 4 — Timestamped sampling layer *(2 days)*
Tasks at the rates of §5, every sample stamped with `esp_timer_get_time()`,
latest-value structs behind a seqlock, MPU-6050 FIFO + INT. The PING))) keeps
its own task (its measurement blocks for up to 20 ms — unacceptable inline, and
the existing ISR + task-notify design is already right for this).
**Accept:** each rate holds within ±1 % over 10 minutes; jitter histogram
logged; zero dropped samples; bus error counters at zero.

### Phase 5 — Attitude estimation *(3–4 days)*
Add `espressif/esp-dsp`. Implement `ekf_imu13states` behind a C API, **and**
a Madgwick path behind the same interface for comparison.
**Accept:** on a turntable or with a protractor, static attitude within 1° at
0/±15/±30/±45°; heading within 5° at eight compass points; **gyro bias state
converges to the measured −2.66 °/s on X**; attitude drifts < 1° over 10 minutes
at rest; `Process()` + update execution time measured and reported (§8.1);
EKF-vs-Madgwick comparison documented.

### Phase 6 — Altitude estimation *(3–4 days)*
The 4-state filter of §4.2, with tilt correction and innovation gating.
**Accept:** tracks a hand-moved sensor against a tape measure within 5 cm from
0.5 m to 3 m; **rejects a book deliberately pushed into the beam** without the
estimate jumping; survives covering the PING))) entirely (falls back to
baro + accel, `alt_valid` drops, no divergence); `b_baro` converges and absorbs
the 1 440 m pressure-altitude offset.

### Phase 7 — Motor interference characterisation ⚠️ *(1–2 days)*
**This must happen before heading is trusted in flight** (§9.4, the largest
unknown in the report). Run the actual motors at the actual mounting distance
and measure the magnetometer deflection versus throttle; also check the
accelerometer vibration spectrum (ESP-DSP's FFT is right there).
**Accept:** a documented mag-error-vs-throttle curve; a decision on whether to
(a) accept it, (b) relocate the magnetometer on a boom, (c) compensate as a
function of commanded current, or (d) abandon magnetic heading and hold yaw on
the gyro plus whatever the Phase 8 sensor provides. Plus a notch/low-pass filter
placed from the measured vibration spectrum.

### Phase 8 — Horizontal position *(scope depends on §1.2)*
Integrate the chosen sensor; extend the estimator; **re-do Phase 7 for it**.

### Phase 9 — State API, telemetry, logging *(2 days)*
Publish `blimp_ins_state_t`; add an `ins` console command showing the live fused
state; binary logging to flash or streamed over USB for offline analysis.
**Accept:** a replayable log; a `state` command whose attitude tracks the board
in real time.

**Rough total to a trustworthy attitude + altitude estimate: ~3 weeks**
(Phases 0–7). Horizontal position is additional and gated on the §1.2 decision.

---

## 7. Cross-cutting requirements

**Carry the bench's conventions forward** (§7 of the report) — they are the
reason the bench is trustworthy:
- never scale from found state: write config, read it back, warn on mismatch;
- every failure mode distinguishable; **nothing fabricates a plausible number**;
- measure instead of assuming (live temperature for the speed of sound);
- drivers never own the bus.

**New requirements the flight firmware adds:**
- **Fail loud, fail safe.** Every estimate carries validity. A sensor that stops
  responding lowers a flag rather than silently freezing a value — the frozen
  value is what crashes aircraft.
- **I²C robustness** (§9.11): retry with backoff, error counters in the health
  struct, bus reset on hang, and an actual measurement of rise time at 400 kHz.
- **Explain the `i2cdetect` phantom ACKs** (§9.12) before the firmware
  enumerates the bus at boot, or do not enumerate at boot.
- **Bounded, pre-allocated memory.** No allocation in the fusion loop; watch
  `dspm::Mat`'s allocation behaviour specifically.
- **Everything logged is replayable.** The fusion code must be runnable offline
  against a recorded log, or tuning becomes guesswork.

---

## 8. Risks

### 8.1 ⚠️ EKF compute cost is unmeasured
13 states means 13×13 covariance work — `CovariancePrediction` is O(n³), on the
order of a few thousand floats per step. At 200 Hz on a 240 MHz S3 with a
single-precision FPU this is *probably* fine, but it is **not known**.
**Mitigation:** benchmark `Process()` and `UpdateRefMeasurement()` with
`esp_timer` in the very first Phase 5 commit, before building anything on top.
If it is too slow: run covariance prediction at a lower rate than state
prediction, drop the prediction rate to 100 Hz (ample for a blimp), or fall back
to the Madgwick path that Phase 5 builds alongside it for exactly this reason.

### 8.2 ⚠️ The magnetometer may be unusable in flight
§9.4. If Phase 7 shows motors swamping a 26 µT field, heading has no absolute
reference and will drift on the gyro alone. **Mitigation:** Phase 7 happens
*before* the control loop depends on heading; options (b)/(c)/(d) in Phase 7.

### 8.3 ⚠️ C++ in the build
The EKF module is C++. **Mitigation:** confine it to `blimp_ins`, expose a C
header, no exceptions, no RTTI, no allocation in the loop. Verify flash and RAM
cost early — there is 4 MB of flash, so this is a RAM and discipline question,
not a space one.

### 8.4 PING))) may not survive the real environment
Soft floors absorb 40 kHz; the wide beam sees clutter; propeller noise may couple
acoustically into a 40 kHz receiver. Phase 1 and Phase 7 both probe this.
**Mitigation:** the VL53L1X is already in place as an independent close-range
cross-check, and the innovation gate of §4.2 is designed for exactly this.

### 8.5 Raising `FREERTOS_HZ` changes existing timing code
§5.1. **Mitigation:** both affected comments are revisited in Phase 2, and
Appendix B of the report gives the regression references.

### 8.6 Attitude and altitude are coupled through tilt correction
A bad attitude corrupts `h = d·cos θ`, which corrupts altitude, which (once
optical flow arrives) corrupts horizontal velocity scaling. **Mitigation:**
build and validate strictly in order — attitude (5), then altitude (6), then
horizontal (8) — and reject rangefinder updates beyond ~25° of tilt.

---

## 9. Decisions needed before Phase 1

1. **Horizontal position sensor** (§1.2) — recommendation: **PMW3901 optical
   flow**. This gates Phase 8 and the overall estimator shape.
2. **Lift budget in grams** — the PING))) alone is 9 g.
3. **Flight arena**: fixed known room, or arbitrary space? (Decides whether UWB
   or markers are even available.)
4. **Floor surface** — carpet degrades the primary altimeter.
5. **Motor type and mounting distance** from the GY-88 (§9.4).
6. **Is 5 V available in flight**, or does the PING))) need a boost converter?
7. **Approve raising `CONFIG_FREERTOS_HZ` to 1000** and I²C to 400 kHz.
8. **Absolute waypoints, or station-keeping only?** Optical flow alone gives the
   latter well and the former poorly.

---

## Sources

- [espressif/esp-dsp (GitHub)](https://github.com/espressif/esp-dsp)
- [ESP-DSP API reference — EKF classes](https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/esp-dsp-apis.html)
- [ESP-DSP on the ESP Component Registry (v1.8.2)](https://components.espressif.com/components/espressif/esp-dsp)
- [ESP-DSP Kalman filter example](https://components.espressif.com/components/espressif/esp-dsp/versions/1.5.1/examples/apps/kalman_filter?language=en)
- [`ekf_imu13states.h` header source](https://raw.githubusercontent.com/espressif/esp-dsp/master/modules/kalman/ekf_imu13states/include/ekf_imu13states.h)
- [espp/filters on the ESP Component Registry](https://components.espressif.com/components/espp/filters)
- [ESP-DSP library user guide](https://documentation.espressif.com/esp-dsp/en/latest/esp32/esp-dsp-library.html)
