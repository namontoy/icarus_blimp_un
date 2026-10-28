# PID tuning: what to measure, how, and the tests that produce the gains

**Date:** 2026-10-28
**Branch:** `pid-tuning`, from `s3-wireless-link`
**Status:** plan, with its instruments written (2026-10-28). Measured so far: the rear pair's
thrust on a scale (P0, `docs/history/pid-bench-2026-10-28/`); nothing on the vehicle yet. `software/thrust_map.py` (the thrust stand, P0), `software/step_test.py` (the step
schedules of Y1, Y2, H1 and H2, with the capture and the fits) and `software/step_response.py`
(the fits, in `core/`'s units) are checked against synthetic responses and against
`mock_blimp.py` over UDP; the bench sessions are what comes next. It extends docs/07 §7 (the
bring-up ladder) and §8 (the identification experiments) with the instruments this repo now has
(the 50 Hz capture, the link's motor command, the Control page) and with the tuning rules to
turn a measured response into gains. Where this document and docs/07 differ, this one is later.

## 1. What is being tuned

`core/` runs three loops over `blimp_pid_step` (`core/src/blimp.c`, `core/src/params.c`), all in
SI, all with the integral gain at zero by decision (`core/include/blimp/pid.h`):

| Loop | Error | Output | Derivative on | Default gains | Limits |
|---|---|---|---|---|---|
| altitude | height, m (ToF, else barometer) | `fz`, N, plus the hover feed-forward | the estimator's fused vertical rate | kp 1.34 N/m, kd 1.34 N·s/m | heave budget 30 gf, error clamped ±0.5 m |
| heading | yaw, rad, wrapped | `tz`, N·m | the gyro's yaw rate, low-passed at 8 Hz | kp 0.033 N·m/rad, kd 0.033 N·m·s/rad | ± motor_max × l_f |
| surge | range to target, m | `fx`, N, ≥ 0 | nothing | kp 0.30 N/m | 0 … 30 gf |

The defaults are not tuned values: they place each loop at ω_n = 2 rad/s, ζ = 1 on an estimated
effective mass (0.334 kg, the sphere plus added mass) and an estimated yaw inertia (0.0083 kg·m²,
an order of magnitude), with `kp = m ω_n²`, `kd = 2 ζ ω_n m` and the same for I_z. So the gains are
a function of two plant numbers and a bandwidth choice, and tuning is first of all measuring those
numbers. Two more things sit in series with every gain:

- **The thrust map.** The loops output newtons; the mixer turns them into duties through
  `motor_max_thrust_n`, a 35 gf placeholder against a measured 14.7–16.8 gf (one motor, lab
  supply; reverse about 74 %). Every loop gain is therefore in error by that ratio today. The
  thrust stand (docs/07 §8, experiment 4) comes before any closed-loop test.
- **Delays.** The loop ticks at 50 Hz (20 ms); the IMU's low-pass is 21 Hz (about 8 ms); the
  VL53L1X's timing budget is 50–100 ms and it is polled, so the altitude measurement is
  100–200 ms old. A loop's bandwidth is bounded by its delay (ω_n θ well under 0.5 rad keeps the
  delay's phase lag small at crossover); with 0.15 s of range delay that is ω_n ≲ 2–3 rad/s, which
  is where the default already sits. Shortening the ToF budget trades range for latency, and the
  2.25 m range question (CLAUDE.md) is open, so the delay is a measurement too.

Where the gains live: `blimp_params_t`, defaults in `params.c`; in the simulator they change at
run time through `ctl.set_params()` (docs/09 §7); on the blimp they change by rebuild and flash
until `PARAM_SET` arrives (control-loop plan increment 5), which is an argument for doing the
open-loop identification first, when no gain is on the vehicle at all.

## 2. Methods, and what this plan takes from each

**Model-based placement (what `params.c` does).** Measure m_eff, I_z and the damping, choose ω_n
and ζ, compute the gains. Transparent, and it keeps the gains in physical units with a reason for
each. It needs the plant numbers, which is what the step tests below give. This is the primary
path.

**SIMC from a step response** (Skogestad 2003; Skogestad and Grimholt's chapter and the DYCOPS 2013
paper in the sources; the constants below were read against both on 2026-10-28). Fit a
first-order-plus-delay model (K, τ, θ) to a measured step response, then one knob, τ_c, the
closed-loop time constant. For a first-order process, Kc = τ / (K (τ_c + θ)) and
τ_I = min(τ, 4 (τ_c + θ)); for an integrating one, Kc = 1 / (k′ (τ_c + θ)) and τ_I = 4 (τ_c + θ);
for an integrating process with a lag, k′ e^(−θs) / (s (τs + 1)), the same Kc and τ_I with
τ_D = τ (table 1 of the chapter; the derivative time is the series form's). Thrust to heading and
thrust to height are that last case: the yaw rate and the climb rate answer thrust as a lag, and
the heading and the height integrate them, so the tool uses the integrating-with-lag rule and
converts the series PID to the parallel form `blimp_pid_step` uses (kp = Kc (1 + τ_D/τ_I),
kd = Kc τ_D, ki = Kc/τ_I, with ki left at zero by decision). Grimholt and Skogestad's θ/3 (a
derivative time of θ/3 for a first-order process, or τ + θ/3 in place of τ in the PI rule) is for
delay-dominant processes and is not used here: the lag, not the delay, is what the derivative
counteracts. τ_c = θ is Skogestad's "tight" choice; with θ near 0.05 s it would ask for a
bandwidth the 100 gf budget cannot deliver, so the tool's default is τ_c = 0.5 s (a closed-loop
bandwidth near 2 rad/s, where the placement already sits). The chapter gives GM 2.96 and PM 46.9°
for the integrating rule at τ_c = θ; a larger τ_c is more robust. The same step test that gives
m_eff and I_z gives these, so the tool prints both; SIMC is the cross-check on the model-based
numbers and the fallback if the model does not fit.

**Relay feedback autotuning** (Åström and Hägglund 1984). Replace the controller by a relay of
amplitude d; the loop oscillates at the ultimate period with a bounded amplitude a; Ku ≈ 4d/(πa);
then Ziegler–Nichols or Tyreus–Luyben gains from Ku and Tu. No model needed and the oscillation
is small by construction. It belongs on the vehicle (the link's latency would corrupt it), so it
is an option for increment 3 or later, for the yaw loop on a tether, if the step fits are poor.
Not the first choice: it tunes for disturbance rejection and gives a lively setpoint response.

**Ziegler–Nichols closed loop.** Raising the gain until the vehicle oscillates. Not on a 100 gf
budget with a tether; the open-loop reaction-curve variant is the same data as SIMC with worse
robustness. Not used.

**Grey-box identification as GT-MAB did it** (Tao et al. 2020 and the Georgia Tech blimp papers in
`docs/reference/`): swing tests for the pendulum modes, release and tow tests for translation,
parameters fitted to the logged motion. docs/07 §8 already adapts it; what changes here is the
instrument: the capture logs the gyro and the range sensor at 50 Hz on the vehicle, so the yaw and
heave experiments no longer need a phone camera, and the surge experiment still does (there is no
speed sensor until the optical flow reaches the S3).

**Blimp specifics that shape the tests.** Added mass makes m_eff larger than the weight (the sphere
alone adds half the displaced air); drag is quadratic at the speeds flown (docs/07 §1.3), so a
linear fit holds only near one operating point and the tests run at the duties the race will use;
the vehicle is trimmed 6 gf heavy, so climbing is driven and descending is gravity-limited, which
makes the altitude loop asymmetric and is the reason integral action starts at zero (windup on the
slow side); and in `core/`'s mixer yaw needs some surge, so a yaw step on the bench is not a pure
torque unless both front motors are driven differentially, which the DRV8833s allow and the mixer
does not yet.

## 3. The instrument

A step test is: the blimp held so one axis is free, a capture running, a step of duty on one pair
from the link, the capture downloaded, the response fitted.

- **Stimulus.** `python software/step_test.py yaw --steps 0.15 0.25 0.30 --mirror --go` (or
  `heave`) streams the whole schedule from the schedule's own clock: a lead of zeros, each
  amplitude held `--hold` seconds and followed by `--gap` seconds of zeros for the decay, then
  the mirror on FR. Without `--go` it prints the schedule and stops; amplitudes are signed (a
  negative one drives the pair the other way) and `--limit 0.75` caps them: the scale's figures
  (below) make the thrust a few gf at 0.3 duty, so the steps are larger than `blimp_link.py
  motor`'s 0.3 and still far from any cap; it refuses to start while
  another source holds the motors, and it ends with zeros and `MOTOR_STOP` three times on Ctrl-C,
  on any error and at the end. The Control page is not the stimulus: it ramps the command over
  0.4 s, which is right for a hand and wrong for identification.
- **Record.** `step_test.py` starts a capture that covers the schedule before the first command,
  downloads it afterwards and saves it as `captures/step-<axis>-<date>.bin` with a `.json` beside
  it (the schedule, the telemetry before and after, the firmware, the fits); `blimp_link.py
  capture` and the Bench page do the same by hand. The record carries the gyro
  (deg/s), the accelerometer, the range (mm), the barometer and the **throttle readback**, which
  is the duty the driver held: the fit uses it as the stimulus, so what was commanded and what
  was applied cannot disagree silently.
- **Fit.** `step_test.py` fits as it finishes; by hand,
  `python software/step_response.py captures/<test>.bin --axis yaw|heave --all --tau-c 0.5`.
  It finds every duty change held for 0.5 s (up and down, so Y2's spin-down and H2's descent
  come out of the same run), removes the baseline before each (a constant for the rate, a
  straight line for the range, so a steady sink before a heave step is the equilibrium it needs
  and the sink rate is reported), and fits by least squares on a grid: yaw as a first-order rate
  response of `gyro_z_dps` to the front differential FL−FR; heave as the range `tof_mm`
  integrating a first-order climb rate of the rear pair. It prints K per unit duty, τ, θ, the
  rise and 95 % times from the fit, and the SIMC gains for the chosen τ_c, then a summary against
  the pass criteria of §4 (K within 20 % across steps, τ up against τ down, θ). Two quality
  figures: the fit percentage (MATLAB's NRMSE; docs/07 asks for 90 %) and the residual as a
  multiple of the baseline's own noise, which is the one to trust when the hold is long (a flat
  tail adds noise and no spread, and the percentage drops although the model is right). Checked:
  on synthetic responses it recovers K within 3 %, τ within 10 % and θ to one sample for the rate
  (the range's smooth onset trades θ against τ by a few samples, so L1 takes the delay from the
  gyro); against `mock_blimp.py`, whose yaw and climb rates now lag their commands by about
  0.3 s, eight yaw steps come out within 4 % of each other and two heave steps within 2 %.
- **From the fit to `params.c`.** Given the thrust map (`--thrust-map captures/thrust-map.json`
  from the stand, or a flat `--gf-per-unit`) and `--half-span` l_f, the tool does the conversion:
  yaw at steady state D_yaw ω = F(u) l_f, so D_yaw = F(u) l_f / (K u) with ω in rad/s, and
  I_z = τ D_yaw; heave D_z v = 2 F(u), so D_z = 2 F(u) / (K u), m_eff = τ D_z, with F(u) one
  motor's thrust at the step's amplitude. It prints three things: `params.c`'s placement
  (kp = J ω_n², kd = 2 ζ ω_n J, the constants at the top of the file) and the kd that gives the
  same ζ once the measured drag is counted (the formula ignores drag, and on the mock's draggy
  airframe it over-damps to ζ ≈ 1.9); the SIMC PID converted with the same F(u) and l_f, as the
  second opinion; and ω_n θ against 0.5 rad. The sign of K says whether the gyro's z and the
  front differential agree with `core/`'s frame (yaw positive counter-clockwise from above): a
  positive FL−FR turns the nose right, so K should come out negative, and the tool says so when
  it does not (it does on the mock, whose toy heading turns the other way).

## 4. Test plan

Props on only with the vehicle held or tethered; STOP within reach; one person's hands free. Each
test is a capture file with a `.json` note beside it; the fit's printout goes into the note.

**P0, prerequisites.**
- Thrust stand per motor, forward and reverse, at the real cell's voltage, duty 0.1 … 1.0 in
  steps of 0.1: `python software/thrust_map.py run --motor FL --reverse --go` holds each duty
  from its own thread, records a short capture per point (the cell voltage, the readback, the
  IMU's vibration) and asks for the scale reading; the tare before and after; one CSV per motor
  under `captures/`. Then `thrust_map.py fit captures/thrust-*.csv` fits F(u) = a u² + b u per
  motor and direction, prints the Rule 13 sum of the four at full duty and the duty at which it
  reaches 100 gf, the spread between motors and the reverse ratio, and writes `thrust-map.json`
  for the step fits. Produces F(u) and `motor_max_thrust_n` (the strongest motor's F(1): a duty
  computed from a weaker curve over-delivers on a stronger one).
- Buoyancy trim to about 6 gf heavy (Rule 11 allows 10), weighed.
- The sign tests of control-loop plan increment 3 on the IMU (nose up: positive pitch rate; left
  side up: positive roll rate; nose left: positive yaw rate).
- The VL53L1X timing budget decided and its latency measured (L1 below).
- The firmware at a build with the Wi-Fi drop fix (`9d1cdca` or later); the motors held from the
  link do not cut during a capture download (`docs/motor-setpoint-ownership.md`, T2).

**L1, latency.** Command to motion: the delay θ the fits report, on every step test; expect the
20 ms tick plus the motor's spin-up, under 100 ms. Range sensor: drop the ToF's target (a board)
by hand while the IMU sees the motion, or step the rear pair and compare the range's θ with the
gyro's from the same capture; expect the timing budget plus one poll. Both bound ω_n.

**Y1, yaw step, tethered.** Hang the vehicle by a thread from the top of the envelope on the yaw
axis so it rotates freely; the thread's torsion is small against the envelope's drag and shows as
a slow drift back, which the baseline absorbs. Steps of the front differential: FL 0.25 then 0.40
then 0.55 with FR at 0 (the plan first said 0.15 to 0.30; the scale's figures for the rear pair,
`docs/history/pid-bench-2026-10-28/`, make that under 1 gf, too little to see), each held 4 s,
6 s apart, then the mirror (FR driven): `step_test.py yaw --mirror --go`, one capture of about
62 s, every step and every spin-down fitted and summarised at the end. Pass: fit above 90 % or residual
under 1.5× noise on at least four of six steps; K within 20 % between amplitudes (the linear
regime) and between directions (symmetry; a difference is a thrust-line offset to record); θ under
0.1 s. Produces D_yaw, I_z, and the SIMC rate-loop gains.

**Y2, yaw spin-down.** From Y1's capture, the decay after each step ends: the rate falls as
exp(−t/τ) with the same τ as the rise. docs/07 §8 experiment 3 as planned, without the camera.
Pass: the two τ agree within 20 %. If the decay is visibly not exponential (fast first, then
slow), the drag is quadratic at that rate and the linear fit is good only around the step's
amplitude; record both τ.

**H1, heave step.** The vehicle trimmed and free, in a room with a clear floor under it, the range
sensor between 0.4 and 1.8 m (inside its reliable range), a person with a hand above the
envelope. Steps of the rear pair: RL = RR = 0.40, 0.55, 0.70, in the sign that pushes the vehicle
up: the scale found the negative command the stronger direction on both rear motors (RR 8.5 g
against 5.0 g at 0.75 duty), so check by hand on the mounted vehicle which sign lifts, and give
the amplitudes that sign (`step_test.py heave --steps 0.40 0.55 0.70 --hold 3 --gap 8 --go`, or
`--steps -0.40 -0.55 -0.70`), one capture. With the 6 gf trim and the stronger direction the
pair gives about 5, 8 and 12 gf, so only the two larger steps climb; the fit does not mind, it
reads the change of velocity against the sinking baseline. Release the vehicle before the first step and let it sink steadily into each step
rather than holding it: the fit removes a straight-line baseline, so the steady sink is the
equilibrium it needs, and the slope it reports is H2's sink rate. Pass: fit above 90 %;
K (climb rate per unit duty) within 20 % between amplitudes; θ consistent with L1. Produces D_z,
m_eff, and the SIMC gains for the altitude loop.

**H2, descent under trim.** From H1's capture, the sink after each step ends (fitted as the
down-step) and the baseline slope before each up-step: velocity reaches a terminal value set by
the trim weight over drag. The terminal sink rate against the weighed trim
gives D_z a second time, independently of the thrust map; and it is the descent rate the altitude
loop has to live with, which decides how far above the band the vehicle may overshoot.

**S1, simulation with the measured numbers.** Put m_eff, I_z, D and F(u) into `params.c` and the
scene, rebuild, run `tools/sim_sweep.py` and closed-loop steps through `ctl.set_params()`
(docs/09 §7). The simulator is our own model, so it checks the structure (overshoot, the
yaw-surge coupling, the asymmetric altitude response) and the margins, not the final digits.
Pass: the Rule 13 sweep still passes; no sustained oscillation on either loop at the chosen ω_n;
the lap-time tables updated with the settings stated.

**C1–C4, closed loop on the vehicle.** Need control-loop plan increment 3 (`blimp_step()` on the
S3, `MODE_SET`, the kill latch) and want increment 5 (`PARAM_SET`, so a gain change is a packet
and not a flash).
- C1, altitude alone, yaw held by the Y1 thread: setpoint steps 1.2 → 1.7 → 1.2 m. Pass: 95 %
  within 4 s, overshoot under 0.1 m, holds within ±0.05 m for 30 s, no limit cycle; the sink
  side slower, as H2 predicts.
- C2, heading alone, altitude held by hand or a loose vertical tether: 90° steps both ways.
  Pass: settles within 3 s (docs/07 §2.1's anchor), overshoot under 10°, no limit cycle; the
  surge the mixer adds during the turn recorded.
- C3, both loops, free in the room: station-keeping for 60 s, then C1 and C2's steps again.
  Pass: no cross-coupling (a heading step does not move the altitude by more than 0.1 m and the
  other way round).
- C4, disturbance: a fan across the vehicle at the hover; a tap on the envelope. Pass: recovery
  without the integral (there is none) inside the band; if a steady offset remains, that is the
  evidence docs/07 §4.3 asks for before any integral term is added, with its clamp.
Each gets a capture; the fits of C1 and C2 (the tool's `--axis` on the setpoint column once
increment 3 fills it) give the closed-loop time constant to compare with τ_c.

**R1, relay autotune (optional).** Only if Y1 fits poorly: a relay mode on the S3 that switches the
front differential between ±d around the heading setpoint on the tether, logging the limit cycle
into a capture; Ku and Tu from it, gains from Tyreus–Luyben, checked against C2.

## 5. What each test feeds, and where the numbers go

| Produces | Goes to |
|---|---|
| F(u) per motor, `motor_max_thrust_n`, the Rule 13 sum | `params.c` (replaces the 35 gf placeholder), docs/07 §1.2 table, the sweep in `test_mixer.c` |
| D_yaw, I_z (Y1, Y2) | `params.c` `IZ_KGM2`, docs/07 §1.2 |
| D_z, m_eff (H1, H2), the terminal sink rate | `params.c` `M_EFF_KG`, `HOVER_TRIM_GF`, docs/07 §1.2 |
| θ per axis (L1) | the ω_n choice in `params.c`, the ToF timing budget in the firmware |
| the closed-loop results (C1–C4) | the gains in `params.c` with the capture's file name in the commit, docs/10's status table, README's results |

## 6. Order, and what blocks what

1. P0 is unblocked today and is most of the value: the thrust map alone fixes every loop gain's
   scale.
2. Y1, Y2, H1, H2 and L1 run on the firmware as it is: `step_test.py` drives the link's
   `MOTOR_SET`, which is the firmware's passthrough, and the capture already records everything
   the fits need. One bench session for yaw, one for heave.
3. S1 follows in an afternoon.
4. C1–C4 wait for increment 3; doing them by rebuilding per gain is possible but slow, so
   increment 5's `PARAM_SET` is worth pulling forward.
5. The GUI's motor-ownership change (`docs/motor-setpoint-ownership.md`) should land before the
   closed-loop sessions: a 3 s cut from a repaint during C1 is a failed test for the wrong reason.

Open questions the tests answer on the way: whether the ToF reaches 2.25 m under the hall's light
(it sets the altitude loop's sensor at race height), whether the magnetometer's heading survives
the motors' field (docs/07 §8 experiment 5; the heading loop's measurement), and whether reverse
thrust on the front pair is worth a mixer change (a pure yaw torque without surge).

## 7. Bench runbook: the session's commands, in order (2026-10-28)

Before anything: the branch `pid-tuning` checked out (the tools live only there), the ground
station **closed** while a tool drives the motors (one client at a time; the tools refuse to
start while the GUI holds the motors, and the blimp answers whoever sent the last packet), the
laptop on the blimp's Wi-Fi (`ICARUS-…`, from the system's Wi-Fi menu or the ground station's
Join button before closing it), every command from the repo root, any Python 3.8+. Props on
means: the vehicle held or tethered, nothing in a propeller's disc, STOP within reach (Ctrl-C in
the tool's terminal stops the motors at once).

**0. Link, firmware, tools.**

```
python software/blimp_link.py status             # telemetry: failsafe idle, link alive; a sensors line under it
python software/blimp_link.py capture status     # the firmware string (a build at 9d1cdca or later) and the 400 s buffer
python software/thrust_map.py --selftest
python software/step_response.py --selftest
```

If the firmware string is older than `9d1cdca`, the access point drops the laptop every 14 s and
every test will show restarts that are not the vehicle's: reflash first (`docs/wifi-drop-report.md`).

**1. P0, the thrust stand** (one motor on the stand, props on, the real cell, the other three
motors untouched). Done on 2026-10-28 for RL and RR with a scale, before the tool existed:
`docs/history/pid-bench-2026-10-28/` has the CSV, the fit (`thrust-map.json`, the file the step
commands below take) and what it found: the negative command is the stronger direction on both
rear motors, RR gives 1.6 times RL, and the thrust at 0.3 duty is 1 to 2 gf. FL and FR are
still to be measured, with the stand or the same scale method, at any time: the step fits are
saved and re-converted from their captures once the map has them.

```
python software/thrust_map.py run --motor FL --reverse             # prints the 20 points, drives nothing
python software/thrust_map.py run --motor FL --reverse --go        # about 5 min per motor
python software/thrust_map.py run --motor RL --reverse --go        # then FR, then RR
python software/thrust_map.py fit captures/thrust-*.csv            # F(u) per motor, Rule 13, captures/thrust-map.json
```

The first prompt is the scale reading with the motor off (the tare). Then per point the tool
settles 1.5 s, records a 3 s capture (the cell voltage, the readback, the vibration) and asks for
the reading while the motor keeps running: a number sends it, `r` repeats the point, `s` skips
it, `q` ends the sweep keeping what was read. `--lever` is thrust over reading if the rig is a
lever; `--duties 0.2 0.4 0.6 0.8 1.0` shortens a sweep; the tare is asked again at the end (the
drift is written in the CSV). Record from the fit: F(1) per motor and the spread, the reverse
ratio, the duty at which four motors reach 100 gf, the cell voltage; `MOTOR_MAX_GF` for
`params.c` is the strongest motor's F(1). A "readback … plan T2" note on a point means the
motors cut during the capture download: write it down, it is the T2 result.

**2. The rest of P0.** The IMU sign check with the gondola in hand (the ground station's
Sensors panel, or the sensors line of `blimp_link.py status`): nose up, left side up, nose left
should each give a positive rate in `core/`'s frame; note the gyro's z sign in particular, it
decides what the yaw fit's sign means. Measure l_f, the front motors' half span (AGENTS.md puts
the surge pair at ±20 cm, so `--half-span 0.20`, not `params.c`'s 0.10). Weigh the trim (about
6 gf heavy). Then the push direction of each pair by hand, the vehicle held: `python
software/blimp_link.py motor 0 0.5 0 0.5 --seconds 2 --limit 0.5` (the rear pair, positive), the
same with `0 -0.5 0 -0.5`: which sign lifts, and which is stronger; then `0.5 0 0 0` and
`-0.5 0 0 0` for FL: which sign pushes the nose forward. The sign that lifts is the sign the heave
steps take; a pair whose stronger direction is the wrong one for `core/`'s frame is a finding
for increment 3 (propellers flipped, or the sign inverted per motor in the firmware). Close the
ground station.

**3. Y1 and Y2, yaw** (the inflated, trimmed vehicle on a thread from the top of the envelope,
free to turn; props on).

```
python software/step_test.py yaw --mirror                                            # the schedule, 62 s
python software/step_test.py yaw --mirror --go --thrust-map docs/history/pid-bench-2026-10-28/thrust-map.json --half-span 0.20
```

Watch the readback follow each step in the printed lines. At the end every step and every
spin-down is fitted and summarised. Pass: four of six steps fit (90 %, or residual under 1.5× the
noise), K within 20 % across amplitudes and between FL and FR, τ up and τ down within 20 %, θ
under 0.1 s. A SIGN line means the gyro's z and the front differential disagree with `core/`'s
frame: not a failure, a fact for increment 3's frame mapping. If the spread is large, repeat
with `--steps 0.20 0.25 0.30` (the linear regime) and keep both runs.

**4. H1 and H2, heave** (trimmed and free, a clear floor, the range sensor between 0.4 and 1.8 m,
a hand above the envelope; props on).

```
python software/step_test.py heave --steps 0.40 0.55 0.70 --hold 3 --gap 8           # the schedule, 35 s
python software/step_test.py heave --steps 0.40 0.55 0.70 --hold 3 --gap 8 --go --thrust-map docs/history/pid-bench-2026-10-28/thrust-map.json
```

With the sign the hand test found lifts (`--steps -0.40 -0.55 -0.70` if negative). Release the
vehicle before the first step and let it sink steadily into each step rather than holding it. Pass: fit above 90 %, K within 20 % across the three amplitudes; the baseline slope
the fit reports is H2's sink rate; θ here is the range sensor's, and L1's delay comes from the
yaw fits.

**5. Keep the data.** `captures/` is gitignored. Copy `thrust-*.csv`, `thrust-map.json` and the
`step-*.json` notes (the fits are in them; the `.bin` too, a few hundred kB each) into
`docs/history/pid-bench-<date>/`, and put the numbers in §5's table, docs/07 §1.2 and
`params.c` with the capture's file name in the commit.

**If a tool refuses.** "the motors are already driven": the ground station or another tool holds
them, STOP there and close it. "no telemetry": the Wi-Fi. "no LOG_INFO": a firmware without the
capture. "longer than the blimp's buffer": a schedule over 400 s, shorten `--gap`. A fit that
says the delay is at the search limit: the response never started, check the readback and the
motor order.

## Sources

- Skogestad, *Simple analytic rules for model reduction and PID controller tuning*, J. Process
  Control 13 (2003) 291–309; the lecture notes
  [SIMC tuning (NTNU, 2024)](https://skoge.folk.ntnu.no/prosessregulering/lectures/2024/SiS6SIMC_tuning.pdf)
  and the paper
  [Probably the best simple PID tuning rules in the world](https://researchgate.net/profile/Sigurd-Skogestad/publication/254233805_Probably_the_best_simple_PID_tuning_rules_in_the_world/links/00b7d53ce0f666d194000000/Probably-the-best-simple-PID-tuning-rules-in-the-world.pdf);
  the derivative extension,
  [Grimholt and Skogestad, DYCOPS 2013](https://skoge.folk.ntnu.no/publications/2013/grimholt-dycops/0122.pdf);
  the chapter with every rule in one table (its table 1 is what `step_response.py` implements),
  [Skogestad and Grimholt, *The SIMC method for smooth PID controller tuning*](https://skoge.folk.ntnu.no/publications/2012/skogestad-improved-simc-pid/old-submitted/simcpid.pdf).
- Åström and Hägglund, *Automatic tuning of simple regulators with specifications on phase and
  amplitude margins*, Automatica 20 (1984) 645–651; a readable account in
  [Control Engineering, "Relay method automates PID loop tuning"](https://www.controleng.com/relay-method-automates-pid-loop-tuning).
- Tao, Hou, Zhang, *Modeling and identification of coupled translational and rotational motion of
  underactuated indoor miniature autonomous blimps*, ICARCV 2020 (`docs/reference/document.pdf`),
  and the other blimp papers in `docs/reference/README.md`; a thesis that reuses the GT-MAB
  identification on a small indoor blimp,
  [Politecnico di Torino](https://webthesis.biblio.polito.it/27754/1/tesi.pdf).
- This repo: docs/07 §4 (loops), §7 (ladder), §8 (experiments); docs/09 §7 (tuning in the
  simulator); `core/src/params.c` (the derivation of the defaults); `docs/control-loop-plan.md`
  (increments 3 and 5).
