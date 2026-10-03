# Control & Estimation Design

This is the authoritative source for the *math* of the vehicle: the plant model, the mixer
formulas, the loop structure and starting gains, the estimator design, the guidance law, the
bring-up ladder, the system-ID plan, and exactly what is inherited from MochiSwarm/BlimpSwarm at
file level. [docs/03-software-architecture.md](03-software-architecture.md) owns the RTOS/task
breakdown, comms, and repo layout; [docs/05-race-strategy.md](05-race-strategy.md) owns the
rules-driven strategy conclusions (sphere vs streamlined, untimed phases, rule exploits). This
document reuses their numbers rather than re-deriving them, and is explicit every time it does.

Built from three independently researched and independently fact-checked source packages: a
control-theory literature review (rulebook + MochiSwarm paper + GT-MAB PhD thesis + SBlimp paper +
RL-blimp paper + the real firmware and simulator repos), a MochiSwarm-paper deep dive, and a
BlimpSwarm-firmware deep dive. Each package's own verification pass found errors in its first
draft — wrong citations, stale numbers, one internal contradiction with this project's own physics.
**Where a verification pass corrected something, this document uses the corrected version and says
so.** Section 10 collects every place the corpus disagreed with itself into one table.

> ### Reference vehicle used for every worked number below
> **36-inch (0.914 m) sphere, V = 0.400 m³, at Bogotá air density.** This is the project's own
> currently-recommended baseline ([README §3.3](../README.md), [docs/02 §1.3](02-hardware.md)) —
> not a number borrowed from a reference paper. Every formula in §1–§2 is parametric in `V` and
> `ρ_air`: if the envelope size changes once the weight budget closes, re-run them; don't just
> patch the answer.

### Notation

| Symbol | Meaning |
|---|---|
| `F_x, F_z, T_z` | Commanded surge force, heave force, yaw torque (body frame) |
| `ρ_air` | Local air density |
| `V, D` | Envelope volume, equivalent sphere diameter |
| `m` | Rigid vehicle mass (forced by neutral buoyancy, Rule 11) |
| `m_A` | Added mass |
| `m_eff` | Effective inertia for translation = `m + m_A` |
| `I_z` | Yaw moment of inertia (rigid + added) |
| `C_d, A` | Drag coefficient, frontal area |
| `k_drag` | Quadratic drag coefficient = `½ ρ_air C_d A` |
| `l` | Motor-pod half-span (arm length) from the yaw axis |
| `θ` | Shared rotor tilt angle (mixer) |
| gf | grams-force; 100 gf = 0.981 N throughout, per Rule 13 |

---

## 1. The plant

### 1.1 What kind of system this is

Standard rigid-body-plus-added-mass Newton-Euler model, evaluated at the center of buoyancy (CB),
exactly as used in GT-MAB's thesis (Eq. 5.26–5.39) and MochiSwarm's paper (Eq. 2–5):

```
M_CB · v̇  +  C_CB(v)·v  +  D_CB(v)·v  +  g_CB(η)  =  τ

M_CB = M_RB + M_A         M_A = diag(m_Ax, m_Ay, m_Az, I_Ax, I_Ay, I_Az)   (diagonal — symmetric envelope)
g_CB(η) ≈ r_z,cg · m·g · sin(θ_pitch)     (pendulum restoring torque, CG below CB)
```

Three effects dominate, in order of how much they change the design versus a naive rigid-body
treatment:

1. **Added mass.** An accelerating body drags surrounding air with it. For an idealized sphere
   the coefficient is `k = 0.5` (classical potential-flow result — Lamb's closed-form ellipsoid
   integrals, reproduced in NACA TR-184), so `m_A = 0.5·ρ_air·V`. At this vehicle's scale this is
   **not a second-order correction** — §1.2 shows it roughly doubles effective inertia versus a
   rigid-body-only estimate. A real (non-spherical) miniature blimp's *measured* added mass runs
   37–44% of rigid mass (GT-MAB thesis, tow-test, 21 trials/axis, 98–99% fit) — lower than a
   sphere's 50%, because their airframe isn't a perfect sphere. Treat `k=0.5` as an
   upper-bound estimate for our own sphere until the tow-test in §8 measures it directly.
2. **Quadratic (not linear) drag.** A Reynolds-number check at this vehicle's scale and likely
   racing speeds (`D≈0.9 m`, `v≈0.3–2 m/s`, `ν_air≈1.5×10⁻⁵ m²/s`) gives `Re ≈ 10⁴–10⁵` —
   solidly in the subcritical bluff-body regime where drag scales with `v²` and `C_d` is roughly
   constant (~0.47 for a sphere). See §1.3: most of the published miniature-blimp literature
   instead fits a **linear** drag model, and this is a real, flagged disagreement.
3. **Pendulum restoring torque, lightly damped.** CG below CB gives passive roll/pitch stability
   "for free," but the same mechanism creates a slow swing mode that a naive attitude-PID can
   *excite* rather than damp if it isn't designed around this from the start (the reason §4's
   architecture has no active pitch/roll position loop, only rate damping). The only measured
   analogue in the corpus (GT-MAB, a different vehicle) found a swing mode damped to only ~1.9%
   of critical — i.e., essentially undamped without active help. **This is a reference number for
   a different airframe, not ours** — expect something in the same qualitative regime (slow,
   underdamped) and confirm with our own pendulum-release test (§8, Experiment 2).

**Structural under-actuation, confirmed independently in three sources** (MochiSwarm's own
allocation equation, the Stuttgart/MPI RL-blimp paper, and GT-MAB's FCS1.0 field data): a 2-motor
tilting bicopter can command `F_x`, `F_z`, `T_z` and **cannot** directly generate sway (`F_y`) or
independent roll/pitch torque. This is not a limitation to design around later — it is why the
control architecture in §4 has no position-hold loop on the horizontal plane and why the guidance
law in §6 is a continuous pursuit law, not a "turn-to-face-then-drive" state machine (the latter is
exactly what cost GT-MAB's non-holonomic generation 2–3 s of settling time per 90° turn and
waypoint overshoot — §2.1).

### 1.2 Numeric parameter table

Every row not directly citing Rule 13 or Rule 21 is an **estimate**, flagged with how it was
derived and how uncertain it is.

| Parameter | Value | Basis | Flag |
|---|---:|---|---|
| `ρ_air` (Bogotá, ~2,640 m) | **0.89 kg/m³** | Ideal-gas law + barometric relation at Bogotá's real average temperature (not the colder ISA-lapse-rate prediction, since Bogotá is equatorial). Two independent derivations in the source corpus landed at 0.888 and 0.901 kg/m³; this project's own docs/02 §1.1 sizing table uses 0.901 with a stated pressure uncertainty of 74.2–76.7 kPa (no authoritative Colombian station reading was found). | **ESTIMATED**, ~1.5% spread across independent derivations — immaterial to any design decision, material to nothing except the third decimal place |
| Envelope volume `V` | 0.400 m³ (36″ sphere) | Project's own recommendation, not a rulebook or reference-platform number | ENGINEERING CHOICE |
| Diameter `D` | 0.914 m | Geometry | derived |
| Vehicle mass `m` (forced) | **356 g** | `m = ρ_air · V` — Rule 11 forces this; it is **not a free input**. See §1.3/§10 for why this matters. | **ESTIMATED**, moves with the film-density unknown in docs/02 §1.2 |
| Added mass `m_A` (sphere, k=0.5) | **178 g** | `m_A = 0.5·ρ_air·V` | **ESTIMATED**, theory upper bound for a perfect sphere — likely somewhat lower on the real gondola+envelope (§1.1); measure, don't assume |
| Effective inertia `m_eff` | **534 g** | `m + m_A` | derived |
| Yaw inertia `I_z` | **~4×10⁻⁴–2×10⁻³ kg·m²** | Rough estimate: two motor pods of ~15 g each at `l≈0.12 m` from the yaw axis, `I_z ≈ 2·m_pod·l²`. A sphere's own rotational added inertia is classically ~0 (Munk/NACA TR-184 — only translational added mass is large for a sphere), so `I_z` is dominated entirely by off-axis rigid hardware, not the envelope. | **HIGHLY UNCERTAIN, order-of-magnitude only.** Not measured anywhere in the corpus for any comparable vehicle. This is the single most important gap §8's spin-down test closes. |
| Max total thrust `F_max` | 100 gf = 0.981 N | Rule 13 | **SOURCED, hard constraint** |
| Per-side thrust (2-motor split) | 50 gf = 0.49 N | Rule 13's own worked example | **SOURCED** |
| Motor-pod half-span `l` | **0.12 m (placeholder)** | Real firmware default 0.15 m, simulator default 0.10 m — **these two "same" reference implementations disagree by 50%** (§10). Neither is ours. | **ESTIMATE, TBD from CAD/build** |
| Frontal area `A` | 0.656 m² | `πr²` | derived |
| Drag coefficient `C_d` | 0.47 | Sphere, subcritical regime, `Re≈6×10⁴` at 1 m/s | MEDIUM — see §1.3, the literature's usual linear-drag assumption disagrees |
| `k_drag` | 0.138 kg/m | `0.5·ρ_air·C_d·A` | derived |
| Restoring torque / pendulum damping | reference only: `θ̈=-20.43 sinθ - 0.168θ̇ + 27.99·f_xt`, ζ≈1.9% | GT-MAB thesis, a **different** vehicle, pitch axis only | **ILLUSTRATIVE, not numerically applicable to our vehicle** — measure our own (§8, Exp. 2) |
| Thruster response time constant | reference: 28 ms (GT-MAB), our own TBD | Different motor, order-of-magnitude anchor only | reference only |

### 1.3 Where the model disagrees with itself: drag regime

The published miniature-blimp literature (GT-MAB, MochiSwarm, SBlimp) models damping as **linear**
in velocity, citing low-Reynolds-number arguments. But their own reported operating speeds
(0.05–0.3 m/s) are much slower than what this vehicle's thrust budget likely permits, and a
Reynolds-number check at *our* likely speeds places us in the **subcritical/quadratic-drag**
regime for a bluff body, not the Stokes regime that formally justifies linear drag.

**Recommendation:** don't adopt the literature's linear-damping assumption by default. Fit **both**
a linear and a quadratic (or combined) drag model against the tow-test data in §8 (Experiment 1)
and let the NRMSE decide — exactly as GT-MAB's own protocol does, just without assuming its
conclusion transfers to our speed range.

---

## 2. Race performance envelope

### 2.1 Worked numbers (reference vehicle: 36″ / 0.400 m³, Bogotá)

| Quantity | Value | How | Flag |
|---|---:|---|---|
| Zero-speed max acceleration `a₀ = F_max/m_eff` | **1.84 m/s²** | `0.981/0.534` | **ESTIMATED** — matches the figure already published in [README §3.3](../README.md) exactly |
| Physics-limited top speed `v_max = √(F_max/k_drag)` | **2.67 m/s** | `√(0.981/0.138)` | **ESTIMATED — and not the relevant number.** See below. |
| Stopping distance from `v_max`, full reverse thrust, `d=v_max²/(2a₀)` | **1.94 m** | `2.67²/(2×1.84)` | **ESTIMATED, and a real design warning:** that's essentially the entire 2 m short leg of Course 1. Sprinting to `v_max` on a 2–4.5 m leg means you cannot stop in time. |
| Yaw torque authority (naive, torque/inertia only) | **~60–300 rad/s²** angular accel. | `τ_max/I_z`, `τ_max ≈ ΔF·l ≈ 0.981×0.12 = 0.118 N·m` against `I_z` from §1.2's range | **ESTIMATE, upper bound.** Ignores rotational drag from the pods/arms sweeping air (not captured by a sphere's near-zero added rotational inertia — the arms are the non-spherical, drag-dominant part). Real achievable rate is far below this. |
| Realistic yaw rate anchor | **~35–45°/s average**, settling a 90° turn in 2–3 s | GT-MAB FCS1.0 (non-holonomic, bicopter-class), measured, **different vehicle** | Reference anchor, not our number — design target: beat this via damping tuning (§7 step 9), validate with the spin-down test (§8 Exp. 3) |
| Search-phase yaw-rate schedule | 140°/s → 24°/s over ~8 s | `mochi-swarm-sim`'s tuned spiral-search constants — what this vehicle class's designers judged usable | Reference-tuned starting point, not measured for us |
| 4 m leg, accelerate-then-brake to arrive at ~0.4 m/s | **2.85 s** | Already computed in [docs/05 §3.1](05-race-strategy.md) | cite, not re-derive |
| 2 m leg, same strategy | **1.91 s** | ditto | cite |
| 4.47 m diagonal (figure-eight), same strategy | **3.05 s** | ditto | cite |

**Top speed, properly stated:** the drag-limited physics ceiling is 2.67 m/s, but per docs/05's own
finding, **the race is inertia-limited, not drag-limited** — the legs are too short to approach
`v_max`, and the 1.94 m stopping distance above shows exactly why trying to would be a mistake. The
practical cruise/arrival speed this document's lap-time numbers assume is **~0.4 m/s**, chosen so
the vehicle can stop precisely on a 50–60 cm target, not because the motors can't do more.

### 2.2 Realistic lap-time targets, both courses

Reusing [docs/05 §3.4](05-race-strategy.md)'s table for the 0.400 m³ reference vehicle (full
envelope-size sensitivity table lives there):

| Settle time / waypoint | Course 1 (rectangle) | Course 2 (figure-eight) | **Total** |
|---|---:|---:|---:|
| 1 s | 9.6 s | 10.9 s | **20.6 s** |
| 2 s | 11.6 s | 12.9 s | **24.6 s** |

These are **physics floors**: perfect control, no air movement, instant detection. A realistic
first working vehicle will be **2–4× slower**. Treat ~20–25 s as the theoretical limit and
**anything under 60 s as a competitive result** until proven otherwise (docs/05's own framing).

### 2.3 What actually limits lap time

At 1 s of settling per waypoint, ~2 s of a ~9.6 s Course 1 run is spent not moving — **a fifth to a
third of the lap**, and the part most likely to blow out on a real vehicle (re-acquisition,
alignment, contact confirmation). Compounding this: the reference simulator's own default
"charge"/touch-confirmation behavior holds committed thrust for **up to 5.0 s per waypoint**
before advancing (§9, §10) — four waypoints of that, left untouched, is **up to 20 s of pure dwell
per lap**, larger than the entire straight-line transit budget. **Terminal-approach and
waypoint-transition logic (§6) is worth at least as much engineering effort as the vehicle's
physical design**, and is far cheaper to iterate on.

---

## 3. Actuation and mixer

> **Rewritten 2026-09-21 for the confirmed airframe.** The original section described a
> shared-tilt differential bicopter; the professor's ESP-FLY board drives **four fixed brushed
> motors through low-side MOSFETs**, which is a different machine. What follows is what
> `core/src/mixer.c` implements today. The bicopter material survives only in git-less history
> and in [docs/10 section 1](10-known-issues.md).

### 3.1 The airframe and its hard limits

Body frame: +x forward, +y left, +z up, yaw +CCW from above.

| Motor | Position | Pushes | Contributes |
|---|---|---|---|
| FL | front, +y (l_f left of the yaw axis) | +x | surge, −yaw |
| FR | front, −y | +x | surge, +yaw |
| RL, RR | rear | +z | heave (up only) |

Every motor is **unidirectional** (0 to `motor_max_thrust_n`). Consequences that shape every
loop above the mixer:

- **No braking.** Drag is the only decelerator (e-fold distance ~3 m for the 0.78 m sphere),
  so the arrival speed at a balloon is set by how much surge is allowed on the way in
  (`surge.out_max`, 30 gf default), not by a closing-speed loop.
- **No push-down.** The vehicle is trimmed a few gf **heavy** (`hover_thrust_n`, Rule 11 allows
  10 gf); the rear pair carries that in flight and "motors off" is a gentle sink to the floor —
  which is also the failsafe. Descent is therefore slow (trim weight only) while climb is fast,
  so the altitude error is clamped to ±0.5 m and damped on the fused vertical rate.
- **No yaw without surge.** Turning needs one front motor above the other and neither can go
  below zero, so a pure yaw demand is realised with the *minimum* surge that keeps both motors
  non-negative. ALIGN and SEARCH are tight arcs, not pivots. The surge cost of a turn is
  `Δv = 2·I_z·ω/(l_f·M)`; with realistic inertia it is small (≈0.4 m/s per 90° at 1.5 rad/s).
- **No lateral force.** `fy` in the wrench is ignored; cross-track error is corrected by
  re-aiming.

### 3.2 Allocation

```
heave  = clamp(F_z, 0, min(heave_budget, 2·motor_max, F_total))   # rear pair, equal split
T_rl = T_rr = heave / 2

front_budget = F_total − heave                                       # Rule 13 remainder
d      = T_z / l_f                                                   # required T_fr − T_fl
d      = clamp(d, −min(motor_max, front_budget), +min(motor_max, front_budget))
s      = clamp(max(F_x, 0), |d|, min(front_budget, 2·motor_max − |d|))   # pair sum
T_fl = (s − d) / 2
T_fr = (s + d) / 2
```

Deadbands of 0.5 gf on `d` and `s` stop the heading loop's noise floor from creeping the
vehicle forward through the yaw-floor rule.

### 3.3 Saturation priority

Applied in this order, every tick, all clamps and no scaling (scaling both front motors would
shrink the yaw *difference*, which is what turns the vehicle):

1. **Every motor ≥ 0 and ≤ `motor_max_thrust_n`.**
2. **Heave first.** Leaving the 2 m waypoint plane by half a balloon is a miss, and a miss voids
   the run (Rule 7).
3. **Yaw over surge.** The differential is realised exactly whenever the per-motor cap and the
   remaining budget allow; surge absorbs every clamp.
4. **Rule 13 total (all four motors) ≤ 100 gf, checked last.** `test_mixer.c` sweeps 114,244
   demands including NaN, Inf and corrupt params against this invariant.

`clamped_total`, `clamped_yaw`, `clamped_heave` report which limit bound this tick; they are in
the telemetry packet and must be logged, not absorbed.

### 3.4 Motor drive

`firmware/main/drivers/motor.c`: LEDC PWM, 20 kHz, 10-bit, one channel per MOSFET gate.
Thrust→duty is a **placeholder** inverse-square law with an 8 % deadband; it must be replaced
by a thrust-stand lookup table (section 8). `blimp_motor_identify_forever()` spins each motor
alone at 20 % so the FL/FR/RL/RR-to-GPIO assignment can be confirmed with the props off.

---

## 4. Control architecture

### 4.1 Loop diagram

```
                         ┌─────────────────────────────┐
   Perception (bearing β,│                             │
   range/size, conf.)    │        GUIDANCE LOOP         │  10–20 Hz, perception-limited
   ───────────────────▶  │  (state machine, §6)        │
                         │  → heading setpoint          │
                         │  → F_x setpoint              │
                         └───────────┬─────────┬────────┘
                                     │         │
                          heading err│         │F_x, target height (2.0 m)
                                     ▼         ▼
                       ┌─────────────────┐ ┌─────────────────┐ ┌───────────────────┐
                       │  HEADING LOOP   │ │ ALTITUDE LOOP   │ │ ATTITUDE-DAMPING   │
                       │  (cascaded P→PD)│ │  (PD, height)   │ │ (rate-only, pitch/ │
                       │  → T_z          │ │  → F_z          │ │ roll, no pos. hold)│
                       └────────┬────────┘ └────────┬────────┘ └─────────┬──────────┘
                                │                    │                    │ (small trim,
                                │  200 Hz class       │  200 Hz class      │  diagnostic
                                ▼                    ▼                    │  until swing
                       ┌─────────────────────────────────────┐            │  is observed)
                       │              MIXER  (§3)             │◀───────────┘
                       │   θ, T_left, T_right, saturation      │
                       └───────────────────┬───────────────────┘
                                           ▼
                                    ACTUATORS (servo, 2× motor)
                                           ▲
                       ┌───────────────────┴───────────────────┐
                       │              ESTIMATOR (§5)             │
                       │ gyro+accel+mag → heading/attitude       │
                       │ baro+ToF+accel → altitude, vert. rate   │
                       └─────────────────────────────────────────┘
                                           ▲
                       ┌───────────────────┴───────────────────┐
                       │           FAILSAFE MONITOR              │  10–20 Hz
                       │ link age, battery V, attitude sanity    │
                       │ → can force mixer output to zero        │
                       └─────────────────────────────────────────┘
```

### 4.2 Loop table

| Loop | Input | Output | Rate | Starting gains | Notes |
|---|---|---|---|---|---|
| Estimator (attitude/heading) | gyro+accel+mag | quaternion/yaw | **200 Hz class** | — | See §4.2 rate caveat below |
| Estimator (altitude) | baro+ToF+accel | height, vert. rate | 50–100 Hz | — | |
| Altitude loop | height error | `F_z` | matches estimator | `KPZ=0.65, KDZ=0.45, KIZ=0.0` (sim) *or* `kpz=0.5–0.7, kdz=0.5` (real firmware, two different defaults across `FullBicopter`/`Differential`) | **Sources disagree on the exact value (§10)** — start from whichever structurally matches the chosen mixer topology (`Differential.cpp`'s defaults), cross-check against the simulator's empirically-closed-loop-tuned set |
| Heading loop, 1st order | yaw error | desired yaw rate | matches estimator | `KPYAW=1.0, KIYAW=0.0` | Pure PD in every reference — see §4.3 |
| Heading loop, 2nd order | rate error | `T_z` | matches estimator | `KPPYAW=0.02, KDYAW=0.02, KDDYAW=0.015` | |
| Attitude-damping (pitch/roll) | rate only | small corrective torque (mostly diagnostic until swing observed) | matches estimator | rate-derivative gain only, tuned against the measured pendulum period (§8 Exp. 2) | No position/P term — see §1.1, §4.3 |
| Guidance | bearing, range/size | heading + `F_x` setpoints | **10–20 Hz**, perception-limited | — | Owns the state machine, §6 |
| Mixer | `F_x, F_z, T_z` | `θ, T_left, T_right` | same tick as control loops | — | §3 |
| Failsafe monitor | link age, battery V, attitude sanity | override mixer to zero | 10–20 Hz | — | §4.4 |

**On control-loop rate:** the corpus gives **three different numbers** for "the" control rate
across its own sources — the MochiSwarm paper's own claimed 200 Hz, the real shipped firmware
example's `TIME_STEP_MICRO=4000 µs` (250 Hz), and the simulator's control callback firing once per
MuJoCo physics step at its 0.01 s timestep (100 Hz). **Recommendation:** treat 200 Hz as a
*validated, achievable target class* for this MCU doing real sensor fusion + control + PWM (it has
genuinely been demonstrated on this exact chip family), but **measure our own achieved rate on our
own PCB and sensor stack** during bring-up (§7) rather than assuming any of the three numbers
transfers as-is.

### 4.3 Anti-windup

Every reference implementation's own tuned starting point is **pure PD — both integral gains
(`KIZ`, `KIYAW`) are literally 0.0.** Follow that:

- **Start with zero integral gain on both loops.** Use physical trim (ballast to the ±10 gf
  buoyancy window, required anyway by Rule 11) to remove most of the steady-state altitude error
  an integral term would otherwise fight.
- **For heading, the guidance loop's continuous bearing correction is an implicit outer-loop
  integrator** once a target is in view — a persistent heading bias shows up as a continuously
  self-correcting bearing error, not open-loop drift.
- **Only add integral action if bring-up reveals a genuine, trim-resistant steady-state error**
  (e.g., asymmetric drag or a thrust-line offset). When added, use clamp-based conditional
  integration matching the reference tuning (`[0, 0.12]` for Z, `±0.30` for yaw) rather than an
  unbounded integrator — the total thrust budget is 100 gf; a single windup event on one axis can
  consume the whole authority budget. Back-calculation anti-windup is the fallback if clamping
  alone proves insufficient.

### 4.4 Failsafe

A blimp sized near-neutral buoyancy (±10 gf, Rule 11) **fails safe by default**: zero thrust means
drift-and-slowly-rise, not fall. Design the failsafe to exploit this rather than porting a
multirotor's "controlled descent" posture:

- Link-loss timeout on a heartbeat pattern (≈1 Hz nominal, considered lost after 4–5 missed
  beats).
- On trigger: zero mixer output, hold last-known heading passively (no active correction), log the
  event, keep telemetry alive if the radio still works.
- Pre-flight buoyancy/actuator sanity gates (§7 steps 1–2) should **block ARM** outright if
  buoyancy is outside ±10 gf — don't rely on the failsafe to catch a mis-trimmed vehicle mid-run.

---

## 5. Estimation

### 5.1 Sensor fusion design

- **Attitude/heading:** gyro (fast, drift-prone) + accelerometer (gravity reference, low
  bandwidth) + **magnetometer** (absolute yaw reference), fused via Madgwick/Mahony at the
  estimator rate. **Two implementation options, genuinely worth weighing against each other:**
  - **(A) Hand-rolled complementary/Madgwick filter** on the MCU from discrete IMU + magnetometer
    parts (LIS3MDL or an integrated 9-DoF part) — full control, more firmware to write and tune.
  - **(B) Offload to the sensor's own on-chip fusion**, following the real reference hardware's
    actual choice: the BNO085 IMU's "Gyro Integrated Rotation Vector" — a fused,
    magnetometer-included AHRS output computed on the sensor itself, at a high update rate,
    consumed directly as a quaternion. Materially less firmware to write and debug; worth
    evaluating against whatever IMU is already on the professor's PCB (docs/02 §6, Q7–Q8) before
    committing to (A).
- **Altitude:** barometer (absolute, noisy, ~±0.25 m real accuracy) + ToF rangefinder (precise to
  ~cm, range-limited to ~4 m, needs a plausible target — fine given the arena's fixed geometry) +
  accelerometer double-integration (high bandwidth, drifts without correction), combined in a
  complementary or light EKF filter.
- **Position/guidance:** deliberately **not** a full 3D dead-reckoning/EKF position estimate.
  Guidance runs on **bearing + range/proximity from perception directly**, matching every
  published indoor-blimp design's own choice to separate altitude control from horizontal control
  rather than fuse into one 6-DoF filter — and it's the only approach compatible with Rule 15
  (sensing onboard) without building genuine SLAM, which is out of scope for the timeline.

### 5.2 The yaw-drift problem: ranked plan

**Root cause:** every MPU6050-class reference design in the corpus has no magnetometer — pure
gyro integration for yaw. Tolerable on a quadcopter's few-minute flight; costly here, where flights
are longer and yaw authority is weak/slow (a handful of gf of differential thrust, not a quad's
continuous full authority to "fight through" an accumulated error).

**Ranked plan, with fallbacks:**

1. **Add a magnetometer and fuse it into the heading estimator.** Single highest-leverage change
   versus every reference design examined — bounds long-run drift to roughly 1–2° under good
   conditions instead of unbounded gyro-integration drift. *(Confirm this bound empirically — §7
   step 5's soak test — don't assume the datasheet-adjacent figure transfers to this build.)*
2. **Mitigate motor-current-induced magnetometer distortion.** Physically separate the
   magnetometer from motor wiring/battery leads/ferrous hardware; calibrate hard/soft-iron **on
   the fully assembled gondola**, not a bare breakout, so calibration captures the real installed
   magnetic environment. For a cheap first cut, fit a static compensation
   `B_measured = B_true + k·I` from a ground throttle sweep and subtract it in real time — this is
   the simpler baseline the correctly-cited Silic & Mohseni paper (IEEE Sensors Journal, 2019;
   **not** "Silic & Frew" as some secondary summaries have it) itself compares its own more
   sophisticated method against; their actual contribution is an **online adaptive estimator**
   with a bias term proportional to the *square* of throttle command (to track rapidly changing
   throttle), which is the better target if there's engineering time for it.
3. **Runtime magnetometer health check.** Reject or down-weight a mag reading in the fusion filter
   if its total field magnitude deviates too far from the calibrated expected magnitude — a cheap
   guard against a transient EMI spike corrupting one sample.
4. **Re-calibrate on-site at the actual venue**, not just the home lab — indoor arenas (structural
   steel, rebar, building wiring) carry local magnetic anomalies a home-lab calibration won't
   capture. Fall back to short-term pure-gyro integration if the mag is flagged bad; accept that
   drift resumes until a good reading returns rather than letting one bad sample corrupt the whole
   fused estimate.
5. **Use vision as a secondary yaw anchor during terminal approach.** Once a waypoint balloon is
   locked, the guidance loop closes directly on visual bearing error, reducing sensitivity to
   absolute-heading quality exactly during the phase (final approach) where an error is most
   costly. The search/reacquisition phase is the most drift-vulnerable; final approach is
   comparatively self-correcting.
6. **Validate empirically before ever flight-testing** (§7 step 5): a genuine ≥10-minute
   stationary/slow-rotation soak on the assembled hardware, fused heading vs. known ground truth,
   to get a real measured drift rate for *this* build.

### 5.3 A caution on citing BlimpSwarm's estimator code

If the team goes to the real firmware source for reference: `SensorSuite.h` wires in
`Barometer390.cpp`'s `Barometer` class, **not** the older `Barometer.cpp`/`BarometerOld` class that
is the one most commonly quoted (including in an earlier pass of this project's own research). Both
files are real and both compute vertical velocity as a raw two-sample finite difference with no
extra smoothing — the design lesson (this is noisy, worth improving) holds either way — but cite
`Barometer390.cpp`'s `update()` if precision matters, not the dead file.

---

## 6. Guidance

### 6.1 State machine and pseudocode

Built directly from the confirmed rules: solo, fully autonomous, 4 identical **same-color** red
balloons per course, visiting order announced per-run (Rule 2 — must be runtime-loaded, never
hardcoded), up to 3 loop attempts per course, invalid on wrong order/incomplete touch/human
intervention (Rule 7). Because every waypoint is the same color, the reference simulator's
color-based gate-selection scenario cannot be used unmodified — target selection here is by
expected position/order from known course geometry, not color.

```
function GUIDANCE_TICK(state, waypoint_i, perception, now):

    det = perception.latest()   # bearing β, range/apparent-size ρ, confidence c, age

    switch state.phase:

      case SEARCH:
        if det.valid and det.confidence > C_MIN and matches_expected_window(waypoint_i):
            state.phase = TRACK
        else:
            yaw_rate_cmd = spiral_search_schedule(state.time_in_phase)   # 140°/s → 24°/s over ~8s,
            fx_cmd = 0                                                   #  reference-tuned start (§2.1)
        if state.time_in_phase > SEARCH_TIMEOUT:
            state.phase = MISS(waypoint_i)          # never found it — see §6.3

      case TRACK:
        if not det.valid:
            state.lost_frames += 1
            if state.lost_frames > LOST_FRAMES_MAX:
                state.phase = SEARCH; continue
        else:
            state.lost_frames = 0
        Tz_cmd = HEADING_LOOP(det.β)
        Fx_cmd = clamp(K_APPROACH · range_to_thrust(det.ρ), 0, FX_MAX)
        if aligned(det.β, ALIGN_TOL) and close_enough(det.ρ, APPROACH_RANGE):
            state.phase = APPROACH
            state.approach_enter_time = now

      case APPROACH:
        Tz_cmd = HEADING_LOOP(det.β)        # keep correcting bearing all the way in
        Fx_cmd = APPROACH_THRUST             # fixed, modest committed thrust — tuned DOWN hard,
                                              #  not the reference impl's 5 s hold (§10)
        contact = CONTACT_DETECTOR()         # §6.2
        if contact.confirmed:
            state.phase = TOUCHED(waypoint_i)
        else if now - state.approach_enter_time > APPROACH_TIMEOUT:
            state.phase = MISS(waypoint_i)   # §6.3
        else if not det.valid and state.lost_frames > LOST_FRAMES_MAX:
            state.phase = SEARCH             # lost the target mid-approach — don't ram forward blind

      case TOUCHED(i):
        zero_thrust_briefly()                # kill Fx immediately — don't keep charging into an
        log_touch(i, now)                    #  already-touched, possibly still-swinging balloon
        if i == last_waypoint_in_sequence:
            state.phase = COURSE_COMPLETE
        else:
            waypoint_i = next_waypoint(sequence, i)
            state.phase = SEARCH

      case MISS(i):
        state.miss_count[i] += 1
        log_miss(i, now, reason = "timeout" or "search_exhausted")
        if state.miss_count[i] <= MAX_RETRIES_PER_WAYPOINT:
            back_off_small_distance()
            state.phase = SEARCH             # retry the SAME waypoint — order must be preserved (Rule 7)
        else:
            state.phase = RUN_FAULT          # exhausted retries — see §6.3

    [any phase] on link-loss / battery-critical / attitude-fault / arena-boundary → FAILSAFE (§4.4)
```

### 6.2 Terminal condition — "touch the balloon"

`CONTACT_DETECTOR()` is the load-bearing function in this whole state machine. Two layered checks,
best first:

1. **Primary — IMU deceleration spike.** A real physical bump produces a distinctive sudden
   negative acceleration along the body-`x` axis, distinguishable from steady thrust-driven
   acceleration, arriving within a short expected window after `APPROACH_THRUST` was committed. No
   added mass, no protrusions, no Rule 17 risk.
2. **Fallback — vision proximity + short dwell.** Apparent balloon width/height fraction crosses a
   threshold, sustained for a short confirm window (2 frames / ~0.1 s, matching the reference
   implementation's own confirm gate — that part is fine to keep).

**What this document explicitly does *not* keep from the reference implementation:** its default
behavior, once a touch is judged plausible, is to hold a fixed thrust for up to **5.0 seconds**
before advancing regardless of any further confirmation (§9, §10). **Replace this with a
closed-loop transition** — advance the instant contact is confirmed by either check above. That
5 s × 4 waypoints = up to 20 s of pure dwell per lap was independently flagged as likely the single
largest avoidable time sink in the whole race (§2.3).

### 6.3 What to do on a miss

Exactly the `MISS(i)` branch above, and it is deliberate that it does **not** advance to the next
waypoint on failure:

- **Never skip or reorder waypoints.** Rule 7 invalidates a run on wrong order *or* incomplete
  touch — silently moving on after a failed attempt produces an invalid run just as surely as
  giving up entirely, so there is no firmware shortcut here.
- **Bounded retries on the same waypoint** (`MAX_RETRIES_PER_WAYPOINT`, e.g. 3): back off a small,
  fixed distance and re-enter `SEARCH`, so a bad first attempt doesn't mechanically repeat.
- **After retries are exhausted, this is a run fault, not a recoverable state.** Log it and surface
  it via telemetry. Do not have the firmware quietly abandon the waypoint. The correct place to
  recover is the **untimed reset between the up-to-3 loop attempts** ([docs/05 §1.1](05-race-strategy.md))
  — burn this loop attempt cleanly and try again on the next one, rather than producing a run that
  will be invalidated anyway.

---

## 7. Bring-up and tuning procedure

Numbered, bench-to-flight, nothing untested ever flies. Extends
[docs/03 §3.4](03-software-architecture.md)'s ladder with explicit pass criteria and inserts the
yaw spin-down system-ID step *before* heading-loop tuning, so gains aren't tuned against a guessed
`I_z` (§1.2) when a bench test is available first.

| # | Step | Setup | Pass criterion |
|---|---|---|---|
| 1 | Buoyancy trim check | Fully assembled vehicle, 0.1 g bench scale | Net force within ±10 gf (Rule 11) at intended battery charge state. **Firmware blocks ARM outside this band.** |
| 2 | Actuator sanity, raw passthrough | Raw-joystick passthrough firmware, no controller, props off | Every servo/motor responds in the expected direction within 1 s; no reversed channel; measured mechanical tilt range recorded for §3.3's clamp |
| 3 | Motor spin-direction + thrust-stand | One motor at a time; props off then on; bench thrust-stand (§8 Exp. 4) | No abnormal noise/heat props-off; thrust reads positive for a forward command; fitted thrust curve covers the full commanded range with no discontinuities |
| 4 | Sensor bring-up | Rotate assembled gondola by hand through known angles; hard/soft-iron mag cal on the **fully assembled** gondola | Gyro/accel signs match the defined body frame on all 3 axes; baro/ToF track hand-held height changes correctly; post-cal mag magnitude within a few % of expected at several headings |
| 5 | Estimator validation, open-loop | Log fused heading/altitude vs. known ground truth by hand/tether; ≥10 min stationary soak | Heading error within a few degrees of ground truth through the moving test; a **measured** stationary drift rate now exists, replacing the guessed 1–2°/session figure (§5.2) |
| 6 | Attitude-damping only, tethered | Rate-damping term only (zero position gain), tethered | Hand-induced swing visibly decays over successive periods; no active hold engaged |
| 7 | **Yaw spin-down system-ID** | Bench spin-down test, §8 Exp. 3 | `I_z` and yaw damping fitted at ≥90% NRMSE (matching GT-MAB's own fit quality on an analogous test) — this number now replaces §1.2's order-of-magnitude estimate for every subsequent gain |
| 8 | Altitude loop alone, tethered/caged | Heading fixed; tune `KPZ` then `KDZ` from §4.2 starting values | Holds commanded height with no sustained oscillation; settle time measured and recorded |
| 9 | Heading loop alone, tethered/caged | Altitude fixed; tune `KPYAW`/`KDYAW`, then 2nd-order terms, against the **measured** `I_z` from step 7 | Commanded 90° step settles without overshoot beyond a defined tolerance; measured settle time compared against the 2–3 s empirical anchor (§2.1) — target beating it |
| 10 | Combined altitude + heading, free flight, enclosed area | Both loops closed simultaneously | Station-keeps within a small, recorded radius; no cross-coupling instability |
| 11 | Guidance with a fake/known target | Ground-truth or tape-mark bearing fed manually, no live vision | State machine transitions `SEARCH→TRACK→APPROACH→TOUCHED` correctly and repeatably, including at least one deliberately-triggered `MISS→retry` cycle |
| 12 | Enable real perception, single waypoint | Live camera + a real balloon, test-space lighting | Reliable tracking + contact confirmation against a real balloon, ≥5 repeats, ≥80% success (tighten once measured) |
| 13 | Full autonomy, one full course | Space matching the real arena footprint, real waypoint sequence | Completes all 4 touches in the announced order, unattended, at least once |
| 14 | Full rehearsal, both courses, timed | All 3 loop attempts per course | Consistent completion across repeats; time logged and compared against §2.2's target table |
| 15 | Add clamped integral terms — only if needed | Only if a real, trim-resistant steady-state error remains after 8–10 | New integral term does not reintroduce windup when an axis is deliberately saturated (e.g., against a fan disturbance); gains logged in `decision_log.md` |

---

## 8. System identification

Adapted from GT-MAB's published, already-peer-reviewed protocol (tow/release + pendulum-release +
thrust-stand), substituting a phone camera + timer + grid backdrop for their OptiTrack mocap rig.

**Experiment 1 — surge/heave tow-release test.** Release the assembled vehicle with a small
calibrated excess weight (1–2 gf) purely along the surge axis, then repeat purely along the heave
axis. *Record:* excess weight used, a phone-camera video against a ruled/grid backdrop at a known
frame rate, frame-by-frame position vs. time. *Fit:* both a linear model
(`m·v̇ + D·v = F`) and a quadratic model (`m·v̇ + k·v|v| = F`) by least-squares against the logged
trajectory; compare NRMSE; pick the better fit (or a combined model) rather than assuming either
transfers from the literature (§1.3). This directly measures `m_eff` and the real drag
coefficient(s), replacing §1.2's theoretical `k=0.5` sphere estimate and drag numbers.

**Experiment 2 — pendulum-release test.** Displace the gondola to a fixed initial pitch/roll angle
and release. *Record:* initial angle, a phone-camera video against a protractor-marked backdrop,
the oscillation's period and decay envelope. *Fit:* `θ̈ = -ω_n²·sinθ - 2ζω_n·θ̇` (or its
small-angle linearization) to extract the restoring-torque coefficient and damping ratio. This
replaces §1.1/§1.2's GT-MAB reference numbers (a different vehicle) with our own.

**Experiment 3 — yaw spin-down test.** Spin the fully assembled gondola about the vertical axis
(hand-spun, or a brief motor-driven pulse) to a known initial angular rate, then release/cut
thrust. *Record:* the decay via onboard gyro logging (the IMU already exists for this) or an
external phone-camera + fiducial marker. *Fit:* `I_z·θ̈ = -D_yaw·θ̇` (with a quadratic term if the
linear fit is poor, same logic as Experiment 1) to extract `I_z` and yaw damping. This is the
number §1.2 flags as **not measured anywhere in the corpus for any comparable vehicle** — this
experiment closes that gap and directly feeds bring-up step 7 and the heading-loop gains (§4.2).

**Experiment 4 — static thrust stand.** Mount each motor/prop assembly on a simple pivot +
kitchen-scale lever rig. Sweep PWM/duty (or voltage) across the full commanded range. *Record:*
thrust in gf at each commanded point. *Fit:* thrust-vs-command curve — do not assume linearity;
verify it. Needed for both the mixer's saturation math (§3.3) and for confirming the real per-motor
and total-thrust margin against Rule 13's 100 gf cap. **Do this test at approximately Bogotá
elevation/air density if at all possible** — static thrust is somewhat density-dependent, and every
number sourced in this corpus was measured at sea level or near it.

**Experiment 5 — magnetometer distortion calibration (secondary, feeds §5.2 item 2).**
Ground-run the motors through a throttle sweep with the vehicle stationary, logging magnetometer
output vs. commanded throttle/measured current. *Record:* the raw mag-vs-throttle table. *Fit:* the
linear `B = B_true + k·I` model as a cheap first cut, or the quadratic-in-throttle online-adaptive
model (Silic & Mohseni, §5.2) if there's time to implement it.

---

## 9. What we take from MochiSwarm/BlimpSwarm

File-level, with licensing. `BlimpSwarm` and `SwarmBase` are **GPL-3.0** — literal code copied into
our firmware carries conveyance obligations if the vehicle/source is handed to competition judges
(docs/03's own licensing note; not legal advice, worth a conversation with the professor).
`mochi-swarm-sim` (both `TongshuWu1`'s fork and its `vincentzouras` parent) carries **no LICENSE
file** — `license: null` on the GitHub API for both — meaning all-rights-reserved by default; ask
the author (Tongshu Wu, confirmed Lehigh SwarmsLab contributor) before redistributing any modified
fork publicly, including in the Team Description Paper.

| File | What we take | License | Caveat |
|---|---|---|---|
| `BlimpSwarm/src/robot/Differential.cpp` | The single-shared-tilt mixer *structure* (§3.2) and the cascaded PD(I) loop structure with clamp-based anti-windup, as the skeleton to reimplement in our own ESP-IDF C | GPL-3.0 | Reimplement independently from the published structure/equations; keep the literal file in `third_party_reference/` only |
| `BlimpSwarm/src/robot/FullBicopter.cpp` | The fully-general two-independent-tilt inverse-mixer formulas (§3.4), as a documented alternative if independent dual-tilt servos are added later | GPL-3.0 | Same |
| `BlimpSwarm/src/act/BLMotor.h/.cpp` | The *method* — fit a linear PWM→thrust curve from a static thrust-stand sweep (§8 Exp. 4) — **not** the specific numeric constants (Lehigh's own 1102/13500 kV motor; ours will differ) | GPL-3.0 | Also confirms brushless + ESC (BLHeli_S arm/calibrate) is the real, current motor technology — resolves the rulebook Appendix A "coreless brushed DC" text as a stale carryover from Lehigh's own earlier (mid-2023) generation, since superseded. **Go with brushless.** |
| `BlimpSwarm/src/sense/BNO85.cpp` | The design pattern of offloading AHRS fusion to the IMU's own on-chip fused output, as Option B in §5.1 | GPL-3.0 | Worth evaluating against whatever IMU is already on the professor's PCB before committing to a hand-rolled filter |
| `BlimpSwarm/src/sense/Barometer390.cpp` | The general pattern (baro + finite-difference velocity) and the caution that this is a known weak point worth improving | GPL-3.0 | Cite **this** file, not the older dead `Barometer.cpp`/`BarometerOld` class commonly (mis)quoted elsewhere (§5.3) |
| `mochi-swarm-sim/src/preferences.py`, `src/robot/differential.py` | Starting gain ranges (§4.2) and mixer structure, cross-checked against the real firmware; the simulator itself as an algorithm-in-the-loop dev tool (docs/03 §7) | **No license** | Ask before public redistribution of a modified fork |
| `mochi-swarm-sim/src/state/auto_gate_sequence_state.py`, `spiral_search.py`, `auto_preferences.py` | The search→track→charge→advance skeleton and the spiral-search yaw-rate schedule (140°/s→24°/s, §2.1) as a starting search pattern | **No license** | **Do not** take the `AUTO_CHARGE` default dwell timing as-is (§6.2, §10) — it's sized for a pickup-delivery demo, not a race |
| `mochi-swarm-sim/models/mochi_BlimpRace_balloon_course*.xml` | The tethered-buoyant-balloon physics primitive (ball-jointed multi-link rope + `fluidshape='ellipsoid'` body) — a real, physically-reusable piece | **No license** | **Correction (§10):** these files do **not** model the actual IAR-LARC course despite an earlier assumption in this project's own docs/03 §7.1 — build our own scene at the correct scale/waypoint count |
| MochiSwarm paper (arXiv 2503.03077) | The symbolic Newton-Euler + PD + visual-servo control-law structure (§1.1, §4), and published hardware anchor numbers as cross-checks only | Academic preprint | Not our vehicle's numbers — ours differs in size/mass/altitude |

---

## 10. Where the corpus disagreed with itself

Every disagreement found across the three source packages, with this document's resolution.

| # | Disagreement | Sources | Resolution used here |
|---|---|---|---|
| 1 | Brushed vs. brushless motors | Rulebook Appendix A ("coreless brushed DC motors x2") vs. the actively-referenced firmware/docs BOM (brushless 1102/13500 kV + BLHeli_S ESCs) | **Brushless.** The rulebook text is a stale carryover from Lehigh's earlier (mid-2023) brushed-DC generation, superseded before the platform the rulebook itself cites as its baseline (§9) |
| 2 | Control loop rate | Paper claims 200 Hz; shipped firmware example hardcodes 250 Hz (`TIME_STEP_MICRO=4000 µs`); simulator's control callback runs at 100 Hz (MuJoCo timestep) | Treat 200 Hz as a validated target **class** for this MCU; **measure our own** achieved rate rather than trusting any one of the three (§4.2) |
| 3 | Actuator PWM rate | Paper: "all actuators receive 50 Hz PWM"; real firmware hardcodes the two ESC channels to 55 Hz and 58 Hz (unequal, and neither is 50 Hz) | Don't assume symmetric or 50 Hz channel timing; verify per our own ESC/servo choice during bring-up |
| 4 | Barometer estimator source file | `Barometer.cpp`/`BarometerOld` commonly quoted vs. `Barometer390.cpp`, the class actually wired into `SensorSuite` | Cite `Barometer390.cpp` (§5.3); the design lesson is unchanged either way |
| 5 | GT-MAB waypoint-switching-radius figure | Commonly repeated as qualitative "Poor" (non-holonomic) vs. 0.2 m (holonomic); the thesis's own Table 6.4 actually gives a second, numeric row: 0.4 m vs. 0.2 m | Cite **0.4 m** as the sourced, quantified non-holonomic figure (§2.1) |
| 6 | Simulator race-course files vs. the real arena | This project's own docs/03 §7.1 assumed `mochi_BlimpRace_balloon_course*.xml` models the actual 10 m×5 m / 4-waypoint IAR-LARC course; independently verified in this corpus to actually be a ~150–200 m, 9-color slalom demo scene | **Corrects docs/03.** Do not rely on the shipped course files as "the" official model; build our own scene at correct scale, reusing only the tethered-balloon physics primitive (§9) |
| 7 | Reference arithmetic's input mass vs. this project's own physics | The control-theory research package's own worked "top speed / leg time" arithmetic assumed a free-standing 200 g rigid mass at a 0.40 m³ envelope; this project's own README §3.1 establishes that neutral buoyancy (Rule 11) **forces** `m=ρ_air·V`, not a freely chosen input — at 0.40 m³ and Bogotá density that's ~356 g, not 200 g | §2's worked numbers use **356 g**, consistent with this project's own established constraint, not the corpus's uncorrected 200 g |
| 8 | Magnetometer-compensation citation | Repeatedly cited (including in this project's own docs/03) as "Silic & Frew," a simple static-linear model | The paper is **Silic & Mohseni**, IEEE Sensors Journal 2019; its real contribution is a **quadratic-in-throttle, online-adaptive** estimator, with the simple static-linear fit only as its own baseline comparison (§5.2) |
| 9 | "Blimps only self-limit for safety" generalization | GT-MAB explicitly states its 0.3 m/s cruise cap is a deliberate safety choice; SBlimp's own paper attributes *its* low cruise speed to actual rotor thrust saturation, a real actuator limit | The generalization is overstated for SBlimp specifically. Doesn't change this vehicle's own thrust-rich conclusion (§2), which rests on its own arithmetic, not on this analogy — just don't over-cite the safety framing |
| 10 | Sphere added-mass theory vs. measured added mass on a real (non-spherical) airframe | Idealized sphere `k=0.5` vs. GT-MAB's measured 37–44% of rigid mass on their own (non-spherical) vehicle | Not a strict contradiction (different geometries), but `m_A=178 g` (§1.2) should be read as a theory **upper bound** until Experiment 1 (§8) measures the real number on our own build |
| 11 | Motor-pod arm length `l` | Real firmware default `lx=0.15 m`; simulator default `LX=0.10 m` — the two "same" reference implementations disagree by 50% | Neither is ours. Measure from CAD/assembly; §1.2/§3.2 use 0.12 m only as an explicitly flagged placeholder |

---

## Flagged uncertainties

- `I_z` (yaw moment of inertia) has no measured analogue anywhere in the corpus for any comparable
  vehicle — everything derived from it (§2.1's yaw-rate estimate, §4.2's heading gains) is an
  order-of-magnitude placeholder until §8 Experiment 3 and §7 step 7 produce a real number.
- The added-mass coefficient (`k=0.5`, sphere theory) and the drag coefficient (`C_d=0.47`,
  subcritical sphere) are both theoretical estimates for an idealized envelope shape, not measured
  on this build — §8 Experiment 1 is the item that resolves both, and §1.3 flags a genuine open
  question about which drag model (linear vs. quadratic) actually fits at this vehicle's real
  operating speeds.
- Every PID starting gain in §4.2 is taken from reference tuning on a **different specific
  airframe** (different mass, buoyancy, thrust envelope, and in the yaw case, an arm length that
  the two reference sources don't even agree on with each other) — a starting point for bring-up,
  not a prediction of what this build will converge on.
- The `l=0.12 m` arm length used in every worked mixer/yaw number in this document is a placeholder
  pending CAD — the two reference implementations it's bracketed between (0.10 m and 0.15 m)
  disagree by 50%, so treat any number derived from it as similarly uncertain until measured.
- Whether a physical contact/bump sensor for waypoint-touch confirmation (§6.2's primary check) is
  mechanically feasible depends on gondola/bumper design not yet finalized (docs/05 §4) — the
  IMU-deceleration-spike fallback assumes a rigid enough gondola to produce a detectable signature,
  which should be confirmed on the bench alongside §7 step 3.
- GPL-3.0 conveyance obligations (§9) are presented as a real consideration based on GPLv3's own
  text, not as legal advice — worth an actual conversation with the professor/university before
  assuming how it applies to a competition submission.
