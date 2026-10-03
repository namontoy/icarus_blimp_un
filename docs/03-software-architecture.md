
# SOFTWARE & CONTROL

Target platform: custom ESP32-S3 PCB (already fabricated). Target event: IAR-LARC 2026 ("International Airship Racing at LARC"), Nov 2-6 2026, Universidad Militar Nueva Granada, Bogota — an inaugural, autonomous lighter-than-air waypoint race, co-organized by Lehigh University's SwarmsLab (David Saldana) and Oakland University (Diego D'Antonio), the same lab behind MochiSwarm/BlimpSwarm.

**Confirmed game rules** (extracted directly from `IAR_LARC2026.pdf`, dated 2026-03-27): two courses over the same 4m x 2m waypoint rectangle inside a 10m x 5m arena — Course 1 is the rectangle perimeter, Course 2 is a figure-eight through it. Four red mylar balloons (22 in nominal, ~50-60 cm inflated), tethered at 2.0 m to their centers, must be touched in a sequence **"specified prior to each run"** (Rule 2) — i.e. the visiting order is runtime-configurable, not fixed. Up to 3 loop attempts per course, best time per course, sum of the two = final score. Fully autonomous once started (Rule 1: manual/RC control forbidden after start), but **onboard or offboard computation is explicitly permitted** — only *sensing* must be onboard (Rule 15). Vehicle limits: helium volume <= ~1 m^3 (50 in balloon equivalent, Rule 10), neutral buoyancy within +/-10 gf (Rule 11), total thrust across all motors <=100 gf with servos excluded from that budget (Rule 13), and a bounding box of 1m x 1m x 1.5m (Rule 14) — note the rulebook contradicts itself here, since Rule 12 separately states "no explicit size limitations," an unresolved discrepancy that should be raised with the organizers (dantonio@oakland.edu / saldana@lehigh.edu) rather than assumed. There are **no opponent vehicles and no hoops** in this ruleset — that is a different competition (GMU's "Defend the Republic"), and should not be designed for here.

The rulebook's own Appendix A/B name **MochiSwarm/BlimpSwarm** (Lehigh, arXiv:2503.03077, `github.com/LehighBlimpGroup/BlimpSwarm`) as the official baseline design and **`github.com/TongshuWu1/mochi-swarm-sim`** as the official simulator — i.e. two of the professor's own reference links are literally the competition's reference implementation, not incidental inspiration. This section is written on that basis.

---

## 1. Firmware architecture (ESP32-S3, FreeRTOS)

### 1.1 Task/RTOS breakdown

Dual-core Xtensa LX7 @ 240 MHz. Pin the hard-real-time sensing/estimation/control/mixer chain to one core (mirroring esp-drone's practice of a dedicated high-priority `stabilizerTask`), and put WiFi/comms, perception ingestion, logging, param service and OTA on the other core. This directly addresses two documented failure modes in the corpus: (a) ESP32-S3 WiFi TX bursts to 285-355 mA that can brown out a small 1S rail if bulk capacitance is inadequate, and (b) task-priority contention where a control loop shares a core with the WiFi stack.

| Task | Core | Rate | Priority (rel.) | Does |
|---|---|---|---|---|
| IMU sampling / raw read | 0 | 500 Hz-1 kHz raw, decimated | Highest | Reads accel+gyro off ICM-42688-P/BMI270 via IRQ or polled DMA |
| Attitude/heading estimator | 0 | 200 Hz | High | Madgwick/Mahony fusion of gyro+accel+mag -> quaternion/yaw |
| Altitude estimator | 0 | 50-100 Hz | High | Baro + ToF + accel complementary/EKF fusion -> height, vertical rate |
| Control loops (altitude, heading, attitude-damping) | 0 | 100-200 Hz (matches estimator) | High, just below estimator | PID/PD loops -> commanded Fz, Fx, Tz |
| Mixer | 0 | Same tick as control | High | atan2/hypot bicopter allocation -> servo angle + per-side thrust |
| Actuator output (PWM) | 0 | Servo 50 Hz, motor PWM carrier ~15 kHz (duty updated at control rate) | High | MCPWM/LEDC writes |
| Guidance (outer loop) | 0 or 1 | 10-20 Hz (perception-limited) | Medium | Converts bearing/range from perception into heading+thrust setpoints |
| Autonomy state machine | 1 | 10-20 Hz | Medium | search/track/charge/advance, course sequencing (Sec. 5) |
| Perception ingestion (parses off-board/companion detections) | 1 | As received (event-driven) | Medium | Decodes bearing/range/confidence packets |
| Comms (CRTP-style framing, WiFi/ESP-NOW RX+TX) | 1 | Event-driven, telemetry down at 10-50 Hz | Medium-low | Command uplink, telemetry/log downlink |
| Failsafe monitor | 1 (reads shared state) | 10-20 Hz | High (pre-emptive) | Watches link age, battery V, buoyancy/attitude sanity; forces safe state |
| Param/log service | 1 | Event-driven / 10 Hz log stream | Low | Live gain tuning without reflash; ring-buffer logging |
| OTA | 1 | Idle-priority, ground-only | Lowest | Only active pre/post-flight, never mid-run |

Rationale for the 200 Hz estimator/control rate: this is the exact rate MochiSwarm's own XIAO ESP32-S3 firmware runs its flight-control loop at (confirmed directly in arXiv:2503.03077's text), so it is a validated, achievable rate on this MCU class doing real sensor fusion + control + PWM, not just an aspirational number.

### 1.2 Sensor pipeline

- **IMU**: upgrade from the reference designs' bare MPU6050 (no magnetometer, used by ESP-FLY/CircuitDigest/esp-drone's stock sensor stack) to ICM-42688-P or BMI270 (lower noise, I2C up to 1 MHz / SPI up to 24-25 MHz).
- **Magnetometer**: LIS3MDL (discrete) or an integrated 9-DoF part (LSM9DS1) — **not present** in any of the reference builds in the corpus, and its absence is the direct cause of the yaw-drift problem (Sec. 4).
- **Barometer**: BMP390 (noise floor ~0.085 Pa, but real absolute accuracy only ~+/-0.25 m — too coarse alone).
- **Range**: VL53L1X (4 m, 50 Hz) or VL53L5CX (8x8 zone, 60 Hz, 4 m) — the same part esp-drone's own position-hold extension board uses — for altitude-to-floor and/or wall/obstacle proximity.
- **Battery monitor**: resistive divider into ADC with a 100 nF filter cap, two-point calibrated (ESP32-S3 ADC has known nonlinearity around 2.5-3 V).

### 1.3 Comms

- Primary link: WiFi (SoftAP or STA) carrying a CRTP-style port:channel framed protocol (see Sec. 2) for command/telemetry/log/param traffic, following the same pattern esp-drone/Crazyflie use.
- **ESP32-S3 has only a 2.4 GHz radio (WiFi b/g/n + BLE5) — there is no 5 GHz option on this chip.** In a hall with many teams' 2.4 GHz devices, ESP-NOW does **not** grant immunity from that congestion (it shares the same band/channels, it just skips AP-association overhead); it only helps the *added-overhead* part of the latency budget, not RF contention itself.
- **Coexistence gotcha to design around from day one**: in `WIFI_MODE_APSTA`, once the radio's STA side associates to an AP, ESP-NOW is forced onto that AP's channel — you cannot run ESP-NOW on a different channel than the WiFi link. If both a WiFi video/telemetry link and an ESP-NOW control link are used simultaneously, they must be coordinated onto one channel, or a channel-hopping helper (the `QuickESPNow`-style pattern) is needed.
- If venue congestion turns out to be severe, a dedicated non-2.4GHz-WiFi radio (ExpressLRS on 900 MHz) is a viable fallback specifically for the safety-critical command/failsafe channel, decoupled entirely from whatever the video/telemetry link is doing.
- Ground-station tooling: reuse the CRTP port/channel model (4-bit port, 2-bit channel, <=30 B payload, priority-ordered ports) as the wire framing regardless of transport, since it is proven, simple, and matches what a cfclient-style PC tool expects.

### 1.4 Failsafe

Unlike a quadcopter, a blimp sized to near-neutral buoyancy (+/-10 gf per the rules) **fails safe by default**: on link loss, sensor fault, or low battery, the correct action is to **zero all commanded thrust and let buoyancy take over** — the vehicle drifts and slowly rises rather than falling. This is a materially different, and generally safer, posture than a multirotor's "attempt a controlled descent" failsafe, and the firmware should be designed to exploit it rather than reflexively porting a multirotor failsafe checklist:
- Link-loss timeout modeled on MAVLink's heartbeat convention (1 Hz nominal, link considered lost after ~4-5 missed heartbeats) — directly reusable as a *design pattern* even though MAVLink itself isn't the wire protocol here.
- On trigger: cut mixer output to zero, hold last-known heading passively (no active correction), log the event, and keep telemetry alive if the radio is still functional so recovery/retrieval is possible.
- Pre-flight buoyancy/actuator sanity gates (Sec. 3, bring-up steps 1-2) should block "ARM" if buoyancy is outside the +/-10 gf window, rather than relying on the failsafe to catch a mis-trimmed vehicle mid-run.

### 1.5 Logging, parameter system, OTA

- **Param/log/console**: adopt esp-drone's architecture pattern (a small parameter table + log-variable table addressable by ID over the comms link) so gains (Sec. 3) can be tuned live during bring-up without reflashing — this was repeatedly flagged across the corpus as a high-value piece of infrastructure to not rebuild from scratch.
- **OTA**: ESP-IDF's `app_update` component (`ota_0`/`ota_1` + `ota-data` partitions) with rollback-safety enabled (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`, and the new image must call `esp_ota_mark_app_valid_cancel_rollback()` after self-test or the bootloader reverts automatically). This matters concretely here because the PCB will be sealed inside a gondola under a helium envelope — every USB-serial reflash risks a helium leak from reopening the envelope, so OTA should be the default update path, gated to ground-only (never mid-run).

---

## 2. Verdict on esp-drone: cannibalize, do not fork, do not write fully fresh

**Verdict: cannibalize infrastructure from esp-drone, cannibalize the control-law starting point from BlimpSwarm/MochiSwarm, and integrate both into a new ESP-IDF project we own.** Do not `git fork` espressif/esp-drone and build on top of it; do not start from a bare ESP-IDF template with zero borrowed code either.

**Why not a straight fork:**
- esp-drone's flight core is a direct port of Bitcraze's Crazyflie firmware — architecturally a **fast, thrust-dominant rigid-body quadcopter stack**. Its motor mixer (`power_distribution_stock.c`) hardcodes a 4-motor quad-X/+ allocation; its PID cascade (`controller_pid.c`) is tuned to continuously fight gravity with sub-second response; its crash-safety heuristics (`sitaw.c`, free-fall/tumble detection) assume a vehicle that can genuinely free-fall or flip. None of this transfers to a passively-stable, thrust-starved (100 gf total budget, ~0 net weight) bicopter blimp — it would all have to be ripped out anyway, so "forking" buys mostly code we must delete.
- No prior art exists combining esp-drone/Crazyflie-lineage firmware with a lighter-than-air airframe (checked directly; absence of evidence, not proof, but consistent across multiple independent search passes) — we would be the first to validate this combination, for no clear benefit over starting clean.
- esp-drone is GPL-3.0 (inherited from Crazyflie). A literal fork commits the whole flight-control tree to that license's conveyance obligations. BlimpSwarm is *also* GPL-3.0, so licensing is not actually a differentiator between the two donors — but it means **any code copied verbatim from either repo carries GPL-3.0 obligations for the resulting firmware if the vehicle is conveyed/handed to competition judges** (a real, non-hypothetical trigger under GPLv3's "User Product" provisions) — worth a five-minute conversation with the professor before assuming any part of the codebase can stay closed, regardless of which donor we pull from.

**Why not write 100% from scratch either:** esp-drone hands us, for free, infrastructure that would otherwise be real engineering time: a working ESP-IDF project skeleton with FreeRTOS task/priority scaffolding, a CRTP-over-UDP (and ESP-NOW) comms stack, and — most valuable — a **pluggable estimator/controller/mixer architecture** (enum + function-pointer tables in `estimator.c`/`controller.c`/`power_distribution.h`) built specifically so a new backend can be added *alongside* the stock ones without gutting `stabilizer.c`. `NBR_OF_MOTORS` is referenced at only a small, bounded set of call sites (driver, self-test, `sitaw.c`), so retargeting the actuator count is a scoped change, not a rewrite of the plumbing around it.

**What we actually take, concretely:**

| From | Take | Do NOT take |
|---|---|---|
| esp-drone | ESP-IDF/FreeRTOS project skeleton and task-priority pattern; CRTP-over-UDP + ESP-NOW comms stack; param/log/console live-tuning framework; the *pattern* of pluggable estimator/controller/mixer backends (function-pointer tables) | `power_distribution_stock.c` (quad mixer); `controller_pid.c` cascade as the control law; `sitaw.c` crash heuristics; the assumption of continuous full-weight-fighting thrust |
| BlimpSwarm/MochiSwarm (`LehighBlimpGroup/BlimpSwarm`, `Differential.cpp`) | The **validated bicopter differential-thrust mixer math** (`servo_angle = atan2(Fz, Fx)`, `thrust_mag = hypot/sqrt(Fx^2+Fz^2)`, split across the two rotors by commanded yaw torque, output smoothing `alpha=0.95`) as the starting point for our mixer backend; the ESP-NOW-based ground-link concept; the general idea of a pluggable "robot class" per actuation layout | The literal Arduino codebase (our PCB differs from XIAO ESP32-S3 pin-for-pin; we are building natively on ESP-IDF, not Arduino, for finer RTOS/task control) |
| Written fresh | Estimator (mag-augmented, Sec. 4); control law tuned for near-neutral buoyancy + pendulum-damping instead of active roll/pitch (Sec. 3); autonomy state machine for the actual IAR-LARC waypoint sequence (Sec. 5); perception integration (Sec. 6) | — |

This is a middle path deliberately: it captures esp-drone's genuinely reusable *infrastructure* (telemetry/tuning, which is real, unglamorous engineering effort to rebuild) while rejecting its *flight-control-law* assumptions (which are simply wrong for this airframe), and it grounds the actual control law in the one codebase (BlimpSwarm) that has already been flown as a bicopter blimp rather than inventing a mixer from zero.

---

## 3. Control law

### 3.1 Loops and what they command

Given a bicopter/differential-thrust actuation layout (2x servo+motor stacks, matching both the rulebook's Appendix A baseline and BlimpSwarm's proven hardware) and a passively-stable envelope (center of buoyancy above center of gravity gives a pendulum restoring torque on roll/pitch "for free"), the control architecture deliberately does **not** mirror a quadcopter's full 3-axis attitude cascade:

1. **Attitude-damping loop (pitch/roll)** — rate-feedback (derivative) only, no P term, no full attitude hold. Its job is to *damp* the gondola's natural pendulum oscillation (the same restoring torque that gives passive stability also creates a slow swing mode that active control can excite if it fights rather than damps it — this is literally the subject of published indoor-blimp research, e.g. GT-MAB's swing-oscillation work). Commands: none directly to the mixer beyond a small corrective torque term if the airframe allows it; on a 2-actuator bicopter this loop is mostly diagnostic/logging until/unless oscillation is observed in bring-up.
2. **Altitude loop** — commands net vertical thrust trim (`Fz`) to hold or change height. PID.
3. **Heading loop** — commands yaw torque (`Tz`), driving the differential/vectored thrust split. PID + a second-order rate/acceleration term.
4. **Guidance (outer) loop** — converts perception bearing-error and range/distance-to-target into heading and forward-thrust (`Fx`) setpoints for loops 2-3; also owns the dwell/confirm gating before declaring a waypoint "touched" (Sec. 5).
5. **Mixer** — `servo_angle = atan2(Fz, Fx)`, `thrust_mag = hypot(Fz, Fx)` (or `sqrt(Fx^2+Fz^2)`), split thrust between left/right rotors by the commanded yaw torque `Tz`, clip thrust to the driver's duty range and servo angle to its mechanical/electrical limit.

### 3.2 Starting gain ranges

The most directly reusable, already-tuned reference numbers come from `mochi-swarm-sim`'s `src/preferences.py`, whose mixer structurally matches the real Lehigh firmware's `Differential.cpp` (independently cross-checked: same `atan2`-angle / magnitude-split-by-yaw-torque architecture). Use these as **starting** values, not final gains — they were tuned for a different specific airframe (MochiSwarm's own mass/buoyancy/thrust envelope) and must be re-tuned against the actual gondola built here:

| Loop | Gain | Starting value | Notes |
|---|---|---|---|
| Altitude (Z) | KPZ | 0.65 | |
| | KDZ | 0.45 | |
| | KIZ | **0.0** | Reference implementation runs pure PD on altitude — see anti-windup note below |
| | Integral clamp | [0, 0.12] | Asymmetric — one-sided |
| Heading (yaw), 1st order | KPYAW | 1.0 | |
| | KDYAW | 0.02 | |
| | KIYAW | **0.0** | Also pure PD in the reference |
| Heading (yaw), 2nd order (rate/accel augmentation) | KPPYAW | 0.02 | |
| | KDDYAW | 0.015 | |
| | Yaw integral clamp | +/-0.30 | |
| | Yaw output clip | +/-0.26 | |
| Force/servo limits (mixer saturation) | FX | +/-0.35 | Normalized units in the reference sim |
| | FZ | [-1.4, 1.4] | |
| | TZ | +/-0.08 | |
| | Servo angle | +/-1.45 rad | |
| | Max thrust | 0.72 (normalized) | Real airframe must instead respect the **100 gf total-thrust rule cap** |

### 3.3 Anti-windup

The reference tuning's own choice is instructive: **both integral gains (KIZ, KIYAW) are literally 0.0** — i.e., the proven starting point is pure PD control, not PID. Recommendation:
- **Start with zero integral gain on both loops**, exactly as the reference implementation does. Substitute physical trim for altitude integral action: precise ballast/buoyancy trimming (required anyway by Rule 11's +/-10 gf tolerance) removes most of the steady-state altitude error an integral term would otherwise be fighting.
- For heading, accept that the **guidance loop's continuous bearing correction acts as an implicit outer-loop integrator** once a target is in view — a persistent heading bias will simply show up as a persistent, continuously-corrected bearing error rather than an open-loop drift.
- **Only add integral action later, deliberately, if bring-up reveals a real steady-state error that trim/geometry cannot remove** (e.g. an asymmetric drag or thrust-line offset) — and when added, use a tightly clamped/conditional-integration form (matching the reference's clamp-based anti-windup: `[0, 0.12]` for Z, `+/-0.30` for yaw) rather than an unbounded integral, because the thrust budget is tiny (100 gf total) and near-neutral buoyancy means a single windup event can consume the whole authority budget on one axis. Back-calculation anti-windup is a more robust alternative if clamp-only integration proves insufficient in practice.

### 3.4 Bring-up/tuning order

1. **Buoyancy trim check** — with all electronics installed, verify net vertical force is within the rule's +/-10 gf window on a bench scale, *before* any control code runs. Block "ARM" in firmware on this check.
2. **Actuator sanity (raw/manual passthrough)** — flash a raw-joystick-passthrough mode (mirroring BlimpSwarm's own `Bicopter1raw` approach) with no controller in the loop; verify servo angle range/direction and motor thrust direction by hand/joystick before any autonomy code touches the actuators.
3. **Sensor bring-up** — verify IMU sign conventions by rotating the assembled gondola by hand and checking gyro/accel signs match the expected body frame; run hard/soft-iron magnetometer calibration **on the fully assembled gondola** (not a bare breakout, since motor/battery wiring will distort the field); verify barometer/ToF give plausible readings.
4. **Estimator validation (open-loop)** — log fused heading and altitude while moving the vehicle by hand or on a tether against known ground truth (known headings, known heights, and a long stationary soak to measure real drift rate) before closing any loop.
5. **Attitude-damping only, tethered** — enable only the pitch/roll rate-damping term (zero position gain), tethered flight, confirm the pendulum swing is damped, not excited.
6. **Altitude loop alone, tethered/caged** — heading in manual/fixed, tune KPZ then KDZ from the reference starting values; verify smooth hold with no oscillation.
7. **Heading loop alone, tethered/caged** — altitude in manual/fixed thrust, tune KPYAW/KDYAW then the second-order KPPYAW/KDDYAW terms; verify no overshoot/oscillation.
8. **Combined altitude + heading, free flight, enclosed area** — verify station-keeping with both loops closed simultaneously.
9. **Guidance loop with a fake/ground-truth target** (e.g. tape mark, or a manually-fed bearing) — before enabling real vision, verify approach behavior and the dwell/confirm-before-declaring-touched gate (Sec. 5) so a vision bug can't be confused with a guidance bug.
10. **Enable real perception** for a single waypoint.
11. **Full autonomy state machine**, one full course, in a space matching the real arena footprint.
12. **Full rehearsal** — both courses, all 3 loop attempts, with timing, to validate repeatability under the actual scoring format.
13. **Only now, if needed** — introduce small, tightly-clamped integral terms if a real, trim-resistant steady-state error remains; re-verify no windup occurs when an axis saturates (e.g., station-keeping against a fan-induced disturbance).

---

## 4. Estimator and the yaw-drift problem

### 4.1 What fuses what

- **Attitude/heading**: gyro (fast, drift-prone) + accelerometer (gravity-vector reference for roll/pitch, low-bandwidth/noisy) + **magnetometer** (absolute yaw reference) via a Madgwick or Mahony complementary filter, run at the estimator rate (200 Hz, matching the control loop).
- **Altitude**: barometer (absolute but noisy, ~+/-0.25 m real accuracy, low bandwidth) + ToF rangefinder (precise to ~cm but range-limited to ~4 m and needs a plausible reflective target — usable given the arena's fixed geometry) + accelerometer double-integration (high bandwidth, drifts without correction), combined in a complementary or light EKF filter — the same pattern esp-drone's own `estimator_complementary.c` uses, just with a ToF term added as MochiSwarm-adjacent designs already do via extension boards.
- **Position/guidance**: deliberately **not** a full 3D dead-reckoning/EKF position estimate. Consistent with the published indoor-blimp literature's own pattern of separating altitude control from horizontal control rather than fusing into one 6-DoF filter (blimp dynamics are slow/high-drag, so this simplification is standard, not a shortcut), and consistent with MochiSwarm's own description of a "hybrid feedback-feedforward visual servoing state machine" rather than SLAM: horizontal guidance runs on **bearing + range/proximity from perception** directly, not a fused absolute (x,y) position. This is also the only approach compatible with Rule 15 (no external localization) without building genuine onboard SLAM, which is out of scope for the timeline.

### 4.2 The yaw-drift problem: concrete plan

**Root cause**: every reference design in the corpus (ESP-FLY, CircuitDigest, esp-drone's stock sensor stack, all MPU6050-based) has **no magnetometer** — pure gyro integration for yaw. On a quadcopter's 5-7 minute flight this is tolerable; on a blimp, whose entire value proposition is long endurance, the same drift compounds over a much longer flight, and the vehicle also has weak/slow yaw authority (a handful of gf of differential thrust vs. a quad's continuous full-authority control), so it cannot "fight through" an accumulated heading error the way a quad might.

**Plan**:
1. **Add a magnetometer** (LIS3MDL discrete, or an integrated 9-DoF part) and fuse it into the heading estimator via Madgwick/Mahony — this is the single highest-leverage change versus every reference design examined, bounding long-run drift to roughly 1-2 degrees under good conditions instead of unbounded gyro-integration drift.
2. **Mitigate magnetic interference from the vehicle itself**: physically separate the magnetometer from motor wiring/battery leads/ferrous gondola hardware; perform hard/soft-iron calibration **on the fully assembled gondola**, not a bare breakout, so the calibration captures the real installed magnetic environment (bring-up step 3).
3. **Runtime magnetometer health check**: reject or down-weight the mag reading in the fusion filter if the total field magnitude deviates too far from the calibrated expected magnitude — a cheap, effective guard against transient EMI (e.g. a motor current spike) corrupting a single sample.
4. **Re-calibrate on-site at the actual venue** before competition runs, not just at the home lab — indoor arenas (structural steel, rebar, building electrical wiring) can carry local magnetic anomalies that a home-lab calibration won't capture. Design the estimator to gracefully fall back to short-term pure-gyro integration if the mag is flagged bad, accepting that drift resumes until a good reading returns, rather than letting a single bad sample corrupt the whole fused estimate.
5. **Use vision as a secondary yaw anchor during terminal approach**: once a waypoint balloon is locked in the tracker, the guidance loop closes directly on visual bearing error, which reduces sensitivity to absolute-heading-estimate quality exactly during the phase (final approach) where an error would otherwise be most costly (missing the touch). The search/reacquisition phase is the most drift-vulnerable; final approach is comparatively self-correcting.
6. **Validate empirically in bring-up step 4**: a genuine 10+ minute stationary or slow-rotation soak test on the real assembled hardware, comparing fused heading to known ground truth, to quantify actual drift rate before ever flight-testing — do not assume the datasheet-level "1-2 degree" figure transfers without measuring it on this specific build.

---

## 5. Autonomy state machine (competition game)

Built directly from the confirmed IAR-LARC rules (Sec. intro), **not** an assumed/guessed game: solo, fully-autonomous waypoint racing, 4 identical red balloons per course, order specified per-run, up to 3 loop attempts per course, invalid on wrong order/human intervention/leaving the arena/safety violation.

**Key design implication the rules force**: because all 4 waypoints are the **same color** (red), the reference simulator's own built-in scenario (`auto_gate_sequence_state.py` / `spiral_search.py` in `mochi-swarm-sim`, which distinguishes gates by *color*) cannot be used unmodified — that scenario's multi-color gate race is a generic template, not this specific ruleset. The adaptation needed is well-scoped: keep the proven search -> track -> charge -> advance skeleton, but replace **color-based target selection** with **expected-position/order-based selection**, since the visiting sequence and the fixed 4m x 2m waypoint geometry are both known in advance of each run.

### 5.1 State machine

```
INIT/PREFLIGHT
  -> checks buoyancy (+/-10gf), actuator sanity, sensor health
  -> loads the run's waypoint order (Rule 2: specified before each run -> must be a
     runtime-configurable parameter, never hardcoded)
     |
     v
ARMED -> ASCEND_TO_RACE_ALTITUDE (target ~2.0m, matching waypoint tether height)
     |
     v
SEARCH(i)              <- initial heading seeded toward waypoint i's known compass
  |   ^                    bearing from the known start position/course geometry,
  |   |                    falling back to spiral_search-style reacquisition sweep
  |   | LOST_TIMEOUT
  v   |
TRACK(i)  --- perception gives bearing+range to a red blob consistent with
  |            waypoint i's expected position window
  v
APPROACH/CONFIRM(i)  <- guidance closes bearing to ~0, ramps forward thrust,
  |                       gated by (a) sustained visual lock/dwell timer,
  |                       (b) minimum-range-then-hold heuristic or a physical
  |                       contact/bump sensor if fitted (more reliable than
  |                       vision-only "we think we touched it")
  v
ADVANCE(i+1) -> back to SEARCH(i+1) until all 4 waypoints in this run's order are done
  |
  v
COURSE_COMPLETE(loop_count++)
  -> if loop_count < 3 and course not yet best-timed: RESET_TO_START, loop again
  -> else: HOLD / controlled end-of-run state

[Any state] -- link-loss / battery-critical / attitude-fault / arena-boundary --> FAILSAFE
FAILSAFE: zero thrust, rely on buoyancy (Sec. 1.4), log, keep telemetry alive if possible
```

- **Waypoint order is a parameter, not a constant**: since Rule 2 states the order is specified before each run, the firmware must load it at run-start (via the param system, Sec. 1.5) rather than compiling it in — this is a concrete, actionable firmware requirement, not a nice-to-have.
- **Same-color disambiguation**: because Course 1 (rectangle) and Course 2 (figure-eight) traverse the *same* 4 physical waypoints in different orders, "which balloon is next" is resolved by expected bearing/position from known course geometry plus dead-reckoning progress, with the vision system confirming "a red blob exists near where we expect the next waypoint," not by color.
- **Touch confirmation**: prefer a genuine physical contact/bump sensor over vision-inferred "close enough" if the mechanical design allows one cheaply — it directly avoids the single riskiest failure mode under Rule 7 (invalid run on wrong order/incomplete touch), and vision-based range estimation on a single monocular color-blob detector is the least reliable part of the whole pipeline.

---

## 6. Perception plan: onboard vs. off-board, and latency budget

### 6.1 Decision

**Recommend off-board (ground-station laptop) vision as the primary path**, with a bare-minimum onboard HSV-blob fallback mode as a WiFi-independence safety net, and a companion-vision-MCU (Nicla-Vision-style) as a fallback if off-board proves unworkable at the venue.

**Reasoning:**
- Rule 1 explicitly permits offboard computation; Rule 15 only requires *sensing* (i.e., the camera) to be onboard. An off-board laptop running classical CV on a WiFi video feed from an onboard camera is rules-legal.
- The competition's own reference platform (MochiSwarm) does **not** run its color detector on the ESP32 at all — it offloads to a companion coprocessor (Nicla Vision, STM32H747) specifically because, in the paper's own words, "image processing induces a high computation overhead for microprocessors," running detection at only 10 Hz on that dedicated vision chip (30 Hz on an OpenMV Cam H7). This is strong, direct evidence bounding what's realistic to expect if vision shares the ESP32-S3 with a safety-critical 200 Hz control loop.
- The target is a **single, uniform color** (red) with **no opponents and no other shapes** to discriminate — this is a simple HSV-threshold + contour-analysis problem, not an ML/object-detection problem, and runs comfortably in single-digit milliseconds on an ordinary laptop, freeing us from needing an onboard NPU-class solution (ESP-DL/ESP-WHO's ~12-27 fps ceiling on ESP32-S3, or the fragile, unfinished state of on-device AprilTag/ArUco ports) at all.
- Off-board frees the ESP32-S3's flight-control core entirely for the control loop, avoiding the exact core-contention risk esp-drone's own task-priority design was built to prevent.
- Risk: 2.4 GHz-only radio in a shared, congested arena. Mitigation: keep the safety-critical command/failsafe link on the lowest-latency, lowest-overhead path available (ESP-NOW, or a dedicated non-WiFi radio if venue testing shows severe congestion) and treat occasional video-frame loss as tolerable, since a dropped frame just triggers the state machine's existing SEARCH/LOST_TIMEOUT reacquisition path, not a safety event.

### 6.2 Latency budget

The vehicle's own mechanical time constant is the controlling number here: a 100 gf-thrust-capped, near-neutral-buoyancy, high-added-mass airframe responds on a **seconds** scale (this is a physics inference from the rules' own thrust/buoyancy numbers plus MochiSwarm's documented dynamics model, not a directly measured figure for this exact vehicle — flagged as such). Perception-to-command latency is judged against that budget, not against quadcopter-scale millisecond requirements.

| Stage | Component | Typical latency | Confidence |
|---|---|---|---|
| Capture | OV2640 JPEG @ QVGA (320x240) | ~23 ms (1/44 fps, measured by community benchmark) | Medium — measured, not vendor-spec |
| Uplink (video) | WiFi, best-case raw-802.11 injection pipeline, 640x360@30-40fps | 90-110 ms glass-to-glass (measured, `hx-esp32-cam-fpv`) | Medium — this is a best-case bound using packet injection, not a plain TCP/UDP socket; a standard socket+decode pipeline will likely be somewhat higher (estimated 100-250 ms) — this estimate is an inference, not directly measured |
| Off-board processing | HSV threshold + contour, laptop CPU | <10 ms (order-of-magnitude estimate for classical CV at this resolution; not benchmarked in the corpus) | Low-medium — reasonable estimate, not sourced |
| Downlink (command) | ESP-NOW, 12-128 B payload | 5.6-5.8 ms (measured, Electric UI benchmark) | High |
| Downlink (command), alt. | WiFi-TCP, small payload | ~6 ms (measured) | High |
| **Total perception round-trip** | capture -> uplink -> process -> downlink | **~150-350 ms estimated** | Low-medium (sum of measured stage estimates; end-to-end not directly measured for this exact pipeline) |
| Vehicle mechanical response | thrust-to-motion time constant | Seconds-scale (physics inference from 100 gf thrust cap + near-neutral buoyancy + high added mass) | Medium — reasoned, not measured on this specific build |

**Conclusion**: even the pessimistic end of the round-trip estimate (~350 ms) is a small fraction of the vehicle's multi-second mechanical response time — there is comfortable margin for an off-board architecture, unlike on a fast quadcopter where every millisecond matters. This margin should still be **validated empirically** early (bring-up step 9-10) rather than assumed, since several of the inputs above are estimates rather than measurements on this exact hardware/venue.

### 6.3 On-site calibration

Do not invent a color-calibration procedure from scratch — the Lehigh team's own workflow (`LehighBlimpGroup/balltraining`, `Blob-detection-and-Tracking`) is directly portable: capture the camera's live gain triplet under current lighting, fill a grid overlay with the target color for 30 s-2 min to collect >=1500 LAB-space samples, fit a 2D Gaussian per color, and bake the resulting constants into the detector. Port this as an off-board Python tool (`ground_station/vision/calibrate_color.py`) run at the venue before competition, since lighting will differ from the home lab.

---

## 7. Simulation plan

### 7.1 What we simulate, and in what

`mochi-swarm-sim` (`github.com/TongshuWu1/mochi-swarm-sim`) — MuJoCo + Python — is **directly usable and should be adopted as the primary pre-flight testbed**, for concrete, verified reasons:
- It is the rulebook's own cited official simulator (Appendix B).
- Its model files are not generic — `models/mochi_BlimpRace_balloon_course.xml` and a `_figure8.xml` variant are literal MuJoCo models of Course 1 (rectangle) and Course 2 (figure-eight), including the waypoint balloons modeled as buoyant spheres on damped 3-link tethers — i.e. the actual physical waypoint mechanism, not an abstraction.
- Its bicopter mixer (`src/robot/differential.py`) has been independently cross-checked and structurally matches the real BlimpSwarm firmware's `Differential.cpp` (same `atan2`-angle / magnitude-split-by-yaw-torque architecture), so tuning done in sim is tuning the same control structure we intend to fly, not a different one.
- It already includes a simulated camera (configured at HQVGA/10 fps RGB — matching the real Nicla Vision's actual detection rate) and OpenCV in its dependency stack, so the perception pipeline (Sec. 6) can be prototyped in sim before any hardware exists.

**What to simulate specifically**: full rigid-body dynamics with buoyancy, added mass, and drag; the bicopter thrust-vectoring mixer; the tethered-balloon waypoint physics (reusable as-is since it's the same physical object); the simulated color camera; and the full autonomy state machine (Sec. 5), adapted from the repo's `auto_gate_sequence_state.py`/`spiral_search.py` skeleton with color-based target selection swapped for position/order-based selection as described above.

### 7.2 How sim connects to the real firmware

`mochi-swarm-sim`'s controller code is **Python**, not the C we will write for the ESP-IDF firmware — so out of the box it is an *algorithm/design validation* tool, not a code-reuse or hardware-in-the-loop tool. Recommended three-phase connection:

1. **Algorithm-in-sim (SITL)** — prototype and tune the control law, mixer, estimator logic, and state machine entirely in Python inside `mochi-swarm-sim` first. Iteration here is near-instant (no flashing, instant reset, full ground-truth state for debugging) compared to hardware. Treat the validated Python behavior as the **spec** the firmware must match.
2. **Port to firmware** — reimplement the validated control law/mixer/state-machine as C in the ESP-IDF project, preserving the same structure and equations (same `atan2`/`hypot` mixer, same clamp-based anti-windup, same state-transition table) so behavior should match by construction, not by coincidence. Write host-side unit tests for the pure-math pieces (mixer, controller functions — no hardware dependency) and cross-check their outputs against the Python sim's outputs for identical inputs, as a regression gate (`tests/cross_check/`).
3. **Hardware-in-the-loop bridge (stretch goal, not in the repo today)** — a thin UDP/serial bridge so the *actual compiled firmware* sends motor/servo commands to the MuJoCo sim in place of real actuators, and receives simulated IMU/camera/ToF data in place of real sensors. This would validate the real firmware's timing and behavior (including simulated WiFi packet loss, for comms-resilience testing) without hardware or balloon risk, but it does not exist in the current repo and would need to be built — budget it as optional, later-phase work, not a prerequisite for first flight.

**Caveats**: the repo has only 31 commits, 0 stars/forks, is itself forked from a third party (`vincentzouras/mochi-swarm-sim`), and carries **no license file** (defaults to full copyright reservation) — fine for internal team use, but ask before any public redistribution of a modified fork (e.g. in a competition submission or public writeup).

---

## 8. Repository layout

```
blimp/
  firmware/                        ESP-IDF project (ESP32-S3), the actual flight code
    CMakeLists.txt
    partitions.csv                 ota_0 / ota_1 / ota-data layout
    sdkconfig.defaults
    main/
      app_main.c                   task creation, init sequence
      config/
        pins.h                     board-specific GPIO map for the professor's PCB
        params_defaults.h
      drivers/                     HAL: IMU, mag, baro, ToF, motor/servo PWM, battery ADC
        imu_icm42688.c / .h
        mag_lis3mdl.c / .h
        baro_bmp390.c / .h
        tof_vl53l1x.c / .h
        actuators_mcpwm.c / .h
        battery_adc.c / .h
      estimator/
        attitude_madgwick.c / .h   gyro+accel+mag fusion
        altitude_fusion.c / .h     baro+ToF+accel complementary/EKF fusion
      control/
        loop_altitude.c / .h
        loop_heading.c / .h
        loop_attitude_damp.c / .h  pitch/roll pendulum-damping only, no active hold
        guidance.c / .h            bearing/range -> heading + thrust setpoints
        mixer_bicopter.c / .h      atan2/hypot differential mixer
      autonomy/
        state_machine.c / .h       search / track / approach / advance, course logic
        course_config.c / .h       runtime-loaded waypoint order (Rule 2)
      comms/
        crtp.c / .h                port:channel framing (donor: esp-drone pattern)
        link_espnow.c / .h
        link_wifi_udp.c / .h
        ground_protocol.h          shared wire format with ground_station/
      params/                      live-tunable parameter + log system (donor: esp-drone pattern)
        param.c / .h
        log.c / .h
      safety/
        failsafe.c / .h            link-loss -> zero-thrust / drift
        preflight_checks.c / .h    buoyancy / actuator sanity gates before ARM
      ota/
        ota_update.c / .h

  ground_station/                  off-board vision + operator console (Python)
    vision/
      color_detector.py            HSV/contour red-blob tracker
      calibrate_color.py           on-site calibration tool (ported from balltraining workflow)
      camera_stream_receiver.py
    link/
      espnow_bridge.py
      udp_telemetry.py
    console/
      param_tuner_ui.py            cfclient-style live gain-tuning UI
    requirements.txt

  sim/
    mochi-swarm-sim/               vendored fork (git submodule or subtree) - ask before redistributing
    adaptations/
      waypoint_state_machine.py    our order/position-based variant of auto_gate_sequence_state
      course_configs/
        course1_rectangle.xml
        course2_figure8.xml
    notebooks/                     gain-tuning / analysis notebooks

  hardware/
    pcb/                           existing ESP32-S3 board: schematics / gerbers
    gondola_cad/                   frame / foam-core / 3D-print files
    bom.md

  tools/
    mag_calibration/                hard/soft-iron cal routine + on-site recal script
    imu_bench_check/
    flash_scripts/                  idf.py wrappers, COM-port helpers (Windows dev machines)

  tests/
    unit/                          host-side tests for mixer/controller, pure math, no hardware
      test_mixer_bicopter.c
      test_guidance.c
    cross_check/                   compare firmware C outputs vs. sim Python outputs
      compare_mixer_outputs.py

  docs/
    dossier/                       this project dossier, per-section markdown
    rulebook/                      IAR_LARC2026.pdf + extracted notes
    bringup_procedure.md
    decision_log.md                esp-drone verdict, perception on/off-board decision, etc.

  third_party_reference/           READ-ONLY reference code, never compiled into firmware/
    esp-drone/                     donor for infrastructure patterns (GPL-3.0 - keep license/notice)
    BlimpSwarm/                    donor for mixer math (GPL-3.0 - keep license/notice)
```

**Licensing note**: since both donor codebases (esp-drone, BlimpSwarm) are GPL-3.0, any code copied verbatim (not just pattern-inspired) into `firmware/` carries that obligation forward for the resulting firmware if it is conveyed to competition judges — keep `third_party_reference/` clearly separated and uncompiled, and log in `docs/decision_log.md` exactly which files in `firmware/` are verbatim-derived vs. independently written, so the licensing question can be answered precisely rather than guessed at later.


---

## Flagged uncertainties

- The IAR-LARC rulebook itself contradicts on vehicle size (Rule 12 says no size limit, Rule 14 gives 1m x 1m x 1.5m) — this dossier follows the explicit 1x1x1.5m figure as the safe design target, but it is not resolved in the source and should be confirmed in writing with the organizers.
- The perception latency budget table's uplink/processing estimates below the ESP-NOW/WiFi-TCP rows are inferences/estimates (flagged individually in the table), not measurements on this exact camera+link+laptop combination — should be validated empirically during bring-up step 9-10, not treated as sourced fact.
- PID starting gains are taken from mochi-swarm-sim's reference tuning for MochiSwarm's own airframe (mass, buoyancy, thrust envelope); they are a starting point, not a prediction of the actual gains this build will converge on.
- Whether a physical contact/bump sensor for waypoint-touch confirmation is mechanically feasible depends on gondola/bumper design not yet finalized — the plan recommends it over vision-only range inference but this needs a hardware decision outside this section's scope.
- GPL-3.0 conveyance obligations are presented as a real consideration based on GPLv3's own text and the fact both donor repos are GPL-3.0, but this is not legal advice — worth an actual conversation with the professor/university before assuming how it applies to a competition submission.
- The bounded/whole-team on-device vision fallback mode's actual achievable frame rate on the professor's specific ESP32-S3 PCB (memory variant, PSRAM presence) is unconfirmed — the ESP-DL/ESP-WHO figures cited are from other ESP32-S3 boards, not this one.