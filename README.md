# IAR-LARC 2026 — Autonomous Airship Racing

Project workspace for Universidad Militar Nueva Granada's entry into **IAR-LARC 2026**
(*International Airship Racing at LARC*), Bogotá, **2–6 November 2026**.

> **This is not a generic "blimp drone" project.** It is a *time-trial race*: an autonomous
> helium airship must physically touch four tethered red balloons in sequence, twice over
> (rectangle course and figure-eight course). Lowest total time wins. Every design decision
> below follows from that, and from the fact that **thrust is capped at 100 grams-force**.

---

## 1. Status and the clock

Today is **21 September 2026**. The competition is in **6 weeks**.

| Milestone | Date | Days left | Status |
|---|---|---:|---|
| Pre-registration | 14 Sep 2026 | **−7** | ⚠️ **LAPSED — must be resolved first** |
| Early-bird fee (100,000 COP/team) | 28 Sep 2026 | **7** | open |
| Regular fee (140,000 COP/team) | 29 Sep – 5 Oct | 14 | open |
| **Team Description Paper due** | **12 Oct 2026** | **21** | not started |
| Competition | 2–6 Nov 2026 | 42 | — |

The pre-registration deadline has already passed. **Nothing else matters until we know whether
the team can still enter.**

### Do these three things today

1. **Email the organizers about late entry.**
   `larslarc@unimilitar.edu.co`, cc `dantonio@oakland.edu` and `saldana@lehigh.edu`.
   Ask: can we still pre-register? Draft questions are in [docs/01-competition-brief.md](docs/01-competition-brief.md).
2. **Confirm the event dates in writing.** The organizer's own site contradicts itself —
   `larc.html` says Nov 2–6, the contact page says Nov 4–6.
3. ~~Get the professor's PCB files~~ **Done (21 Sep):** the board is the Seeed x Max
   Imagination **ESP-FLY** (XIAO ESP32S3, MPU-6050, four MOSFET-driven brushed motors).
   Still needed from the professor: the board and gondola **on a scale**, and a
   **thrust-stand number** for one motor+prop.

### Where the software stands (21 Sep, evening)

The control core was rewritten for the real airframe (four fixed unidirectional
motors, no altitude sensor) and **completes both courses in simulation**: course 1
best lap 21.6 s, course 2 best lap 54.1 s, on the confirmed hardware with no extra
sensors. The firmware builds for the ESP32-S3 with the real pin map. It has never
been flashed. [docs/10-known-issues.md](docs/10-known-issues.md) is the live status
page; its section 3 makes the one hardware recommendation that matters: **add a
downward time-of-flight sensor** (~2 g, ~USD 3) to the existing I2C bus.

---

## 2. The rules that constrain the design

All verified verbatim against the official rulebook PDF (`IAR_LARC2026.pdf`, dated 27 Mar 2026,
linked from <https://sites.google.com/oakland.edu/iar-larc/rules>), then independently re-checked
by a second pass.

| Rule | Constraint |
|---|---|
| 10 | Helium volume ≤ that of a 50-inch balloon (rulebook says "approximately 1 m³") |
| 11 | Neutrally buoyant to within **±10 grams-force** |
| 13 | **Total thrust across all motors ≤ 100 gf** (servos excluded) |
| 14 | Bounding box ≤ **1 m × 1 m × 1.5 m** |
| 1 | Fully autonomous once started — no human intervention |
| 1 / 15 | **Offboard computation is permitted**; only *sensing* must be onboard |
| 5–6 | 2 courses × up to 3 loops; best time per course; sum of both = score; lowest wins |
| 21 | Waypoints are 22-inch red mylar balloons, tethered, centre at 2 m height |
| — | Arena 10 m × 5 m; waypoints form a 4 m × 2 m rectangle |

**Two contradictions in the rulebook itself** (ask the organizers, design to the stricter reading):

- Rule 12 says "there are no explicit size limitations"; Rule 14 gives a hard bounding box.
- Rule 10's "50-inch balloon ≈ 1 m³" is geometrically a **1.27 m sphere — which does not fit
  inside Rule 14's 1 m × 1 m box.** The two rules cannot both be satisfied by a sphere.

**Rule 1 is the most useful rule in the book.** Offboard computation being legal means the
vision and planning can run on a laptop, with the airship carrying only sensors and a radio.
That removes the hardest constraint from the ESP32-S3.

---

## 3. The physics that decides everything

This section is our own analysis, not something copied from a reference build. It is the
reason the design does not simply clone the baseline.

### 3.1 Neutral buoyancy fixes the vehicle's mass

Rule 11 forces near-neutral buoyancy. A neutrally buoyant body displaces its own mass in air, so:

```
m_total  =  ρ_air × V
```

The vehicle's mass is **not a free parameter**. It is set by the envelope volume. You cannot
"build it lighter" without also building it smaller.

### 3.2 Added mass makes it worse

An accelerating body drags surrounding air with it. For a sphere the added mass is half the
displaced air mass. So the effective inertia is:

```
m_eff  =  m_total + ½ ρ_air V  =  1.5 × ρ_air × V
```

### 3.3 Therefore: acceleration is inversely proportional to volume

With thrust capped at `F = 100 gf = 0.981 N`:

```
a_max  =  F / (1.5 × ρ_air × V)
```

At Bogotá (see 3.4, ρ_air ≈ 0.89 kg/m³):

| Envelope volume | Equivalent sphere | Vehicle mass (forced) | a_max | Time to cross a 4 m leg* |
|---:|---:|---:|---:|---:|
| 0.15 m³ | 0.66 m (26") | 134 g | **4.9 m/s²** | ~1.9 s |
| 0.25 m³ | 0.78 m (31") | 223 g | **2.9 m/s²** | ~2.4 s |
| 0.40 m³ | 0.91 m (36") | 356 g | **1.8 m/s²** | ~3.0 s |
| 0.785 m³ | 1×1×1.5 m ellipsoid (box limit) | 699 g | **0.94 m/s²** | ~4.2 s |

\* accelerate-then-decelerate, drag neglected; indicative only.

> ### The single most important strategic conclusion
>
> **The smallest envelope that can lift our electronics is the fastest vehicle.** Building a
> big airship to "use the full 1 m³ allowance" would make us roughly **five times slower**.
> Every gram we save on electronics lets us shrink the envelope, which compounds directly
> into lap time. Mass reduction is not housekeeping here — it *is* the competitive strategy.
>
> Counterpart findings: a *streamlined* envelope is genuinely ~7% faster, but breaks the 1.5 m
> bounding-box limit at useful volumes and degrades badly in sideslip — **build a sphere.** And a
> two-rotor bicopter **cannot move sideways at all**, which makes uncorrectable cross-track error
> the dominant accuracy problem; budgeting ~15 gf to a lateral trim thruster costs ~0.5 s per
> course and buys valid runs. Full analysis, target lap times and the rule exploits in
> [docs/05-race-strategy.md](docs/05-race-strategy.md).

### 3.4 Bogotá's altitude: a sizing problem, not a performance problem

UMNG is at ~2640 m, where ρ_air ≈ 0.89 kg/m³ against 1.225 at sea level — about **73% of
sea-level density**, so about 73% of the lift per litre. The obvious reading is "we lose a
quarter of our payload". That reading is only correct if the envelope size is fixed.

**If we buy an off-the-shelf balloon**, it is correct and it hurts: that envelope lifts
~27% less in Bogotá than the spec sheet (written at sea level) claims, and the envelope's own
weight is a fixed tax that does not shrink with the air. Every build video, tutorial and paper
quotes sea-level lift unless it says otherwise.

**If we size our own envelope**, the penalty very nearly cancels. To carry the same
180 g of envelope + electronics:

| | Sea level | Bogotá | Change |
|---|---:|---:|---:|
| Envelope volume needed | 0.170 m³ | 0.235 m³ | **+38%** |
| Sphere diameter | 0.69 m (27") | 0.77 m (30") | +11% |
| Total vehicle mass (neutral) | 208 g | 209 g | **~0%** |
| Added mass | 104 g | 104 g | **~0%** |
| Max acceleration | 3.14 m/s² | 3.13 m/s² | **~0%** |
| Frontal area (drag) | — | — | +24% |
| Top speed (drag-limited) | — | — | **+5%** |

The bigger balloon at altitude weighs the same as the smaller balloon at sea level, because
neutral buoyancy means mass *is* displaced-air mass either way. Acceleration and added mass are
unchanged; thinner air even offsets the larger frontal area, leaving top speed marginally better.

> **So: altitude is not a performance crisis. It is a sizing and logistics constraint.**
> We need a ~38% larger envelope than a sea-level build of the same vehicle — which matters
> for fitting Rule 14's bounding box, for helium consumption per fill, and for handling a
> larger balloon in a small arena. It does not make the vehicle slower.

This only holds if we are free to choose the envelope. If Rule 10 pins us to one specific
commercial balloon, we are back in the first case and payload gets tight — which is exactly
why the clarification in §3.5 matters.

### 3.5 The "50-inch balloon" is not 1 m³

The rulebook says a 50-inch balloon is ~1 m³. The MochiSwarm paper says its 50-inch foil balloon
"provides 120 grams of net buoyancy". **These disagree by roughly 9×** — a true 50-inch sphere is
1.07 m³ and would lift over 1 kg at sea level.

The resolution is that **"50-inch" is the flat, uninflated size of the foil balloon.** Inflated,
it is a lens-shaped envelope of roughly **0.17 m³** — back-solved from the 120 gf figure, and
equivalent to a 27-inch sphere. The rulebook's "approximately 1 cubic meter" is an idealised-sphere
calculation that does not describe the product it links to.

Three consequences:

- The volume cap is **far less restrictive than it sounds**. Rule 10 also says *"teams may use any
  balloon shape or configuration"*, so we can build a custom envelope. Rule 14's bounding box
  (0.785 m³ maximum) binds long before Rule 10 does.
- Every lift figure quoted from the reference platform should be read as **~0.17 m³, not 1 m³**.
- Combined with §3.3, none of this is a constraint we will approach — we want to be *smaller* than
  the baseline, not bigger.

**Still measure it, in week 1.** Buy one balloon, inflate it, measure the inflated diameter in
three axes, and measure net lift by adding weights until neutral. The envelope film's areal density
is independently uncertain by a factor of two (42 vs 85 g/m²), which swings usable payload at
Bogotá between ~200 g and ~88 g. One afternoon converts the largest remaining guess into a number.
Procedure in [docs/00-action-now.md §C](docs/00-action-now.md).

---

## 4. What we inherit

The rulebook names its own baseline design, and it is **the exact repo the professor already
sent**: MochiSwarm / BlimpSwarm from Lehigh (Saldaña's SwarmsLab) — a "bicopter" blimp with a
spherical foil envelope and two tilting rotors.

| Asset | Use to us |
|---|---|
| `TongshuWu1/mochi-swarm-sim` (MuJoCo) | The blimp dynamics model and a tethered-balloon physics primitive — genuinely useful, lets us tune control before hardware exists. ⚠️ **Its `mochi_BlimpRace_balloon_course*.xml` files are NOT the IAR course** (verified: 9 multi-coloured balloons over 90 m and 36 m, vs the official 4 red balloons in a 4×2 m rectangle). We must author our own scene. ⚠️ No licence file — ask the owner before redistributing. |
| MochiSwarm paper (arXiv 2503.03077, ICRA 2025) | Baseline vehicle: XIAO ESP32-S3, 200 Hz control loop, 18.7 g foam-core frame, brushless rotors + microservo tilt stacks, ESP-NOW link, Nicla Vision doing detection at 10 Hz |
| `espressif/esp-drone` | Crazyflie-derived **quadcopter** firmware. Reusable: HAL, sensor drivers, param/log system, comms patterns. Not reusable: mixer, attitude controller, estimator tuning. Its reference board is ESP32-**S2**, so expect pin remapping. GPLv3 — matters if we publish. |
| CircuitDigest article | Generic ESP32 quadcopter tutorial. Useful only for MOSFET motor-drive and PWM patterns. |

**Unresolved conflict:** the rulebook's Appendix A says the baseline uses *coreless brushed*
motors; the BlimpSwarm docs BOM says *brushless 1102 13500 kV* with ESCs. That flips the entire
motor-driver design. Being settled in the running deep-dive.

---

## 5. Plan

### Phase 0 — Unblock (now → 28 Sep)
Registration resolved · rulebook clarifications emailed · professor's PCB audited ·
**parts ordered** (customs into Colombia is the critical path) · balloon lift measured ·
ESP-IDF + MuJoCo toolchains running.

### Phase 1 — Simulation and bench (29 Sep → 5 Oct)
~~Closed-loop control in MuJoCo on the real courses~~ (done 21 Sep) · electronics
bring-up on the bench (flash, motor-ID mode, IMU axis check) · thrust stand
characterisation · ground-station perception (target bearing/range, search hint,
any-balloon altitude fix - [docs/10 section 4](docs/10-known-issues.md)) ·
envelope and gondola v1.

### Phase 2 — First flight + paper (6 Oct → 12 Oct)
Tethered then free flight, manual · yaw and altitude hold ·
**Team Description Paper submitted 12 Oct**.

### Phase 3 — Autonomy (13 Oct → 26 Oct)
Waypoint detection · guidance law · full autonomous course runs · timed laps.

### Phase 4 — Harden and travel (27 Oct → 1 Nov)
Robustness against drafts and WiFi congestion · spares · field kit · packing · dry run.

### Phase 5 — Compete (2–6 Nov)

Procurement lead time is the dominant schedule risk. Order long-lead items **this week**,
before the design is finalised, choosing parts that survive design changes.

---

## 6. Documents

| Doc | Contents |
|---|---|
| [docs/01-competition-brief.md](docs/01-competition-brief.md) | Rules, deadlines, logistics, questions for the organizers |
| [docs/02-hardware.md](docs/02-hardware.md) | Sizing, weight budget, BOM, actuation, **PCB questions for the professor** |
| [docs/03-software-architecture.md](docs/03-software-architecture.md) | Firmware architecture, esp-drone verdict, autonomy, repo layout |
| [docs/04-risk-and-troubleshooting.md](docs/04-risk-and-troubleshooting.md) | Risk register, symptom→cause→fix tables, checklists, field kit |
| **[docs/05-race-strategy.md](docs/05-race-strategy.md)** | **Target lap times, envelope sizing/shape verdict, rule exploits, touch detection** |
| [docs/07-control-design.md](docs/07-control-design.md) | **Plant model, mixer formulas, control-loop gains, estimator/yaw-drift plan, guidance pseudocode, bring-up ladder, system-ID plan, file-level inheritance** |
| **[docs/10-known-issues.md](docs/10-known-issues.md)** | **What is broken, unresolved or deferred — read before touching the code** |
| [docs/08-firmware-setup.md](docs/08-firmware-setup.md) | ESP-IDF on Windows: verified setup, and the traps |
| [research/raw-research-digest.md](research/raw-research-digest.md) | All 15 investigations with fact-check verdicts and sources |
| [research/gap-research.md](research/gap-research.md) | Follow-up research on what the first pass missed |

Everything in `docs/` is drafted from researched sources with confidence levels attached.
Anything marked unverified is unverified — check before spending money on it.

---

## 7. Top risks

1. **We cannot register.** Pre-registration lapsed. Resolve before anything else.
2. **Parts do not arrive in time.** 6 weeks, international shipping, Colombian customs.
3. **The envelope/lift discrepancy (§3.5) is worse than assumed** and the electronics do not fly.
4. **The TDP is missed** on 12 Oct — a deliverable that is easy to forget while building hardware.
5. **Arena air currents** (HVAC) exceed 100 gf of thrust authority. Test in a large hall early,
   not just a lab.

Full register with mitigations: [docs/04-risk-and-troubleshooting.md](docs/04-risk-and-troubleshooting.md).
