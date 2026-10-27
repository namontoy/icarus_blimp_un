# Race strategy

Derived by reading the primary rulebook (`../reference/IAR_LARC2026_rulebook.txt`) directly and
working the physics, rather than from the reference builds. Every number here is reproducible
from the assumptions stated; none of it is quoted from a source.

---

## 1. Three rules that are worth more than they look

### 1.1 The clock does not start until you touch waypoint 1 (Rule 4)

> *"Timing: The timer starts when the airship touches the first waypoint and stops when the final
> waypoint is successfully touched."*

**The entire launch, stabilisation and approach to waypoint 1 is untimed.** Nothing rewards
rushing it. The vehicle should take as long as it needs to: settle to neutral trim, let the gyro
bias estimate converge, zero the yaw reference against a known visual landmark, acquire waypoint 1,
align, and only then make contact and start the clock.

Combined with Rule 5 — *"up to three loops in a sequence without any human intervention"* — the
**repositioning between loops is also untimed**, because each loop's clock restarts at the next
touch of waypoint 1. After finishing a loop, the vehicle can drift back slowly, re-stabilise, and
re-acquire before starting the next attempt.

> Design consequence: build an explicit, unhurried `PREPARE` state, and re-enter it between loops.
> Time spent there is free. Confirm this reading with the organizers — it is a real advantage and
> we should not be the only team that noticed.

### 1.2 Any envelope shape is legal (Rule 10)

> *"Teams may use any balloon shape or configuration, as long as the total buoyant volume does not
> exceed this limit."*

We are not restricted to the commercial balloon the rule links to. We can build a custom envelope.
Section 3 shows what to do with that freedom — and, surprisingly, the answer is *not* "make it
streamlined".

### 1.3 The bounding box is the real limit, not the volume cap

Rule 10 allows ~1 m³. Rule 14 caps the vehicle at 1 m × 1 m × 1.5 m, *explicitly including the
envelope*. The largest ellipsoid fitting that box is **0.785 m³** — so Rule 14 binds first and
Rule 10's 1 m³ is unreachable. Rule 12's "no explicit size limitations" is contradicted by Rule 14
in the same section; design to Rule 14.

This is moot in practice, because §3 shows we want to be nowhere near either limit.

---

## 2. What the reference platform's numbers actually mean

The MochiSwarm paper states verbatim:

> *"The spherical foil balloon has 50-inch diameter and contains helium gas that provides 120 grams
> of net buoyancy."*

A true 50-inch sphere is 1.07 m³ and would produce **~1130 gf** of gross lift at sea level — nine
times the stated figure. The two cannot both be true. Back-solving from 120 gf of net buoyancy
gives an effective volume of **~0.17 m³**, equivalent to a 27-inch sphere.

**Conclusion: "50-inch" is the flat/uninflated size of the foil balloon.** Inflated, it is a
lens-shaped envelope of roughly 0.17 m³. The rulebook's "≈1 cubic meter" is an idealised-sphere
calculation that does not describe the product it links to.

This matters twice over: it means the volume cap is far less restrictive than it sounds, and it
means **every lift figure quoted from the reference platform should be read as ~0.17 m³, not 1 m³.**
Verify by inflating one and weighing its lift — procedure in [00-action-now.md §C](00-action-now.md).

---

## 3. The physics of going fast here

Rule 11 forces near-neutral buoyancy, so vehicle mass equals displaced-air mass: `m = ρ_air · V`.
Added mass for a sphere is another `½ ρ_air V`. Thrust is capped at 100 gf regardless of size.
So effective inertia is `1.5 ρ_air V` and **acceleration is inversely proportional to envelope volume.**

### 3.1 Volume dominates

Simulated 4 m leg, accelerate then brake, arriving at 0.4 m/s, Bogotá (ρ = 0.89 kg/m³), spherical envelope:

| Envelope volume | Sphere diameter | Effective inertia | 4 m leg | 2 m leg | 4.47 m diagonal |
|---:|---:|---:|---:|---:|---:|
| 0.17 m³ | 0.69 m | 227 g | **1.94 s** | 1.30 s | 2.08 s |
| 0.25 m³ | 0.78 m | 334 g | **2.31 s** | 1.55 s | 2.47 s |
| 0.40 m³ | 0.91 m | 534 g | **2.85 s** | 1.91 s | 3.05 s |

Going from 0.17 m³ to 0.40 m³ costs **47% more time per leg**. This is the single biggest lever
we control.

### 3.2 Shape: a real but narrow advantage to streamlining — still build a sphere

> **Corrected.** An earlier version of this section claimed shape "barely matters" (~4%) on the
> grounds that both shapes have the same added mass and that a sphere is omnidirectional. Both
> premises were wrong. Added mass is strongly shape-dependent, and *omnidirectionality comes from
> the actuation, not the hull* (§3.6). The corrected numbers are below; the recommendation survives,
> but for different and weaker reasons.

Added mass depends on shape. For a sphere the coefficient is 0.5 of displaced air. For a 3:1 prolate
ellipsoid moving **along its axis** it is only ~0.11 (Lamb's coefficients, as used in the airship
literature). So a streamlined hull is both less draggy *and* meaningfully lighter in effective
inertia:

| Envelope (V = 0.25 m³) | Added-mass coeff. | Effective inertia | C_d | 4 m leg | 2 m leg |
|---|---:|---:|---:|---:|---:|
| Sphere, 0.78 m | 0.50 | 334 g | 0.47 | **2.31 s** | 1.55 s |
| Prolate ellipsoid 3:1, 0.54 × 1.63 m | 0.11 | 247 g | 0.10 | **1.91 s** | 1.33 s |

That is a 17% gain per leg, not 4%. But a bicopter must **yaw to face its direction of travel**
(§3.6), and yaw is where the ellipsoid pays it back. A sphere has *zero* added moment of inertia
about any axis — by symmetry, rotating it induces no net fluid angular momentum. An elongated hull
has a large one:

| Envelope | Rigid I_z | Added I_z | Total | Yaw accel | 90° turn |
|---|---:|---:|---:|---:|---:|
| Sphere, 0.78 m | 8.3 g·m² | **0** | 8.3 g·m² | 17.7 rad/s² | **0.42 s** |
| Ellipsoid 3:1 | 13.7 g·m² | 8.9 g·m² | 22.5 g·m² | 6.5 rad/s² | **0.69 s** |

Over Course 1 (legs of 4, 2, 4 m with two corners):

| | Legs | Corners | **Total** |
|---|---:|---:|---:|
| Sphere | 6.16 s | 2 × 0.42 s | **7.00 s** |
| Ellipsoid 3:1 | 5.15 s | 2 × 0.69 s | **6.53 s** |

**The ellipsoid wins by ~0.5 s, about 7%.** That is a genuine advantage — and it is also well inside
the uncertainty of my drag coefficients, added-mass coefficients and corner model.

> **Recommendation: still build a sphere**, but as an engineering judgement rather than an
> aerodynamic one:
>
> - **The bounding box punishes elongation.** Rule 14 caps length at 1.5 m. A 3:1 ellipsoid of
>   0.25 m³ is 1.63 m long — **illegal**. To fit, either drop to 0.196 m³ at 3:1, or reduce fineness
>   to ~2.5:1, which gives back much of the aerodynamic gain.
> - **Sideslip is brutal on a streamlined hull.** The numbers above assume motion along the axis.
>   Any cross-track error puts the hull broadside, where its drag and added mass are several times
>   worse. A sphere is indifferent to direction of travel; an ellipsoid is only good when pointed
>   correctly, and §3.6 says pointing correctly is exactly what a bicopter is bad at.
> - **A sphere is a purchase; an ellipsoid is a fabrication project.** With six weeks, custom
>   gore-panel envelope welding is schedule risk we do not need.
>
> The baseline platform's spherical envelope is the right call here. Revisit only if the team ends
> up with spare weeks and a working vehicle.

| Ellipsoid fineness | Max size within Rule 14's 1.5 m | Max volume |
|---|---|---:|
| 2.0:1 | 0.75 m × 1.50 m | 0.442 m³ |
| 2.5:1 | 0.60 m × 1.50 m | 0.283 m³ |
| 3.0:1 | 0.50 m × 1.50 m | 0.196 m³ |

### 3.3 Altitude is a sizing problem, not a speed problem

At Bogotá's 2640 m, ρ_air ≈ 0.89 vs 1.225 kg/m³ at sea level. For the same electronics we need
**~38% more envelope volume** — but that larger envelope, in thinner air, has the *same* mass and
the *same* added mass, so **acceleration is unchanged** (3.14 vs 3.13 m/s² for a 180 g payload).
Thinner air even offsets the larger frontal area, leaving top speed ~5% better.

Altitude costs us envelope size and helium volume per fill. It does not cost us lap time.
(If we were locked to a fixed off-the-shelf balloon, it *would* cost ~27% of payload — which is why
building our own envelope, permitted by Rule 10, is worth doing.)

### 3.4 Target times

Course 1 is the rectangle perimeter: legs of 4 m, 2 m, 4 m (timed from touching WP1 to touching the
final waypoint). Course 2 is the figure-eight, using the 4.47 m diagonals.

| Envelope | Settle 1 s/waypoint | Settle 2 s/waypoint |
|---|---|---|
| 0.17 m³ | Course 1 **7.2 s** · Course 2 **8.1 s** · total **15.3 s** | 9.2 · 10.1 · **19.3 s** |
| 0.25 m³ | Course 1 **8.2 s** · Course 2 **9.3 s** · total **17.4 s** | 10.2 · 11.3 · **21.4 s** |
| 0.40 m³ | Course 1 **9.6 s** · Course 2 **10.9 s** · total **20.6 s** | 11.6 · 12.9 · **24.6 s** |

These are physics floors assuming perfect control, no air movement and instant detection. A
realistic first working vehicle will be **two to four times slower**. Treat ~20 s as the theoretical
limit and anything under 60 s as a competitive result until we have evidence otherwise.

> **Reality check on the settle time.** The 1–2 s per waypoint above is an assumption. The Georgia
> Tech GT-MAB thesis measured **2–3 s just to settle a 90° turn** on a comparable non-holonomic
> blimp (§3.6), before any balloon re-acquisition or touch confirmation. If we land at their
> numbers rather than ours, Course 1 alone is ~12–15 s and the total is ~25–30 s. The physical
> floor for the turn itself is 0.42 s (§3.2), so the gap is entirely controller quality — which is
> the single highest-leverage thing to work on.

### 3.5 A quarter of the lap is spent at the waypoints

At 1 s of settling per waypoint, 2 s of a 7.2 s Course 1 run is spent not travelling — **28% of the
lap.** At 2 s of settling it is 39%. And settle time is the part most likely to blow out on a real
vehicle, where the balloon must be re-acquired visually, the approach aligned, and contact confirmed.

> **Where to spend engineering effort:** the terminal approach and waypoint transition logic is worth
> at least as much as the vehicle's physical design, and it is much cheaper to iterate. A vehicle
> that never loses sight of the next target and transitions crisply will beat a faster vehicle that
> hunts around at each balloon.

---

### 3.6 The bicopter cannot move sideways — and that is the real problem

A two-rotor tilting bicopter directly actuates **surge, heave and yaw**. It **cannot generate a
lateral (sway) body force at all.** This is structural, not a tuning deficiency, and three
independent sources state it:

- The MochiSwarm paper's own allocation is `f = Σ fᵢ·[cos φᵢ, 0, sin φᵢ]` — the y-component is
  identically zero for any tilt angle. The paper says the blimp *"is incapable of generating force
  in the y_B direction."*
- The Stuttgart/MPI RL-blimp paper: *"the blimp does not have a lateral movement control."*
- The Georgia Tech GT-MAB thesis, for its non-holonomic controller: *"sideway drift of the blimp
  cannot be corrected by FCS1.0 due to the lack of lateral controllability, causing large
  cross-track error."*

So the vehicle must yaw to face wherever it wants to go, and **any cross-track error can only be
corrected by turning and driving — never by sliding across.** On a course of 2–4 m legs where the
target is a 50–60 cm balloon, that is the dominant accuracy problem.

GT-MAB measured what it costs. Comparing their non-holonomic controller against a holonomic one
with added lateral authority, on a four-waypoint rectangular course much like ours:

| | Non-holonomic (bicopter-like) | Holonomic (lateral authority added) |
|---|---|---|
| 90° turn settling | **2–3 s** | — |
| Behaviour at waypoint | overshoots past it, carried by momentum | — |
| Waypoint switching radius | *"Poor"* | **0.2 m** |

Their 2–3 s per turn is far worse than the 0.42 s physical floor computed in §3.2, because it
includes killing and redirecting translational momentum with a conservatively-tuned controller.
The gap between 0.42 s and 2–3 s is controller quality — which is where the engineering effort pays.

> ### Recommendation: add a small dedicated lateral thruster
>
> Rule 13 caps **total** thrust at 100 gf but says nothing about how it is distributed, and servos
> are excluded from the budget entirely. Reserving a modest slice for a sideways-pointing thruster
> buys the cross-track authority that GT-MAB's data says is the difference between *"poor"* and
> 0.2 m waypoint accuracy.
>
> What it costs, on the 0.78 m sphere:
>
> | Surge budget | 4 m leg | Course 1 (3 legs) | Cost vs 100 gf |
> |---:|---:|---:|---:|
> | 100 gf | 2.31 s | 6.16 s | — |
> | 90 gf | 2.43 s | 6.48 s | +0.32 s |
> | **85 gf** | **2.49 s** | **6.65 s** | **+0.49 s** |
> | 70 gf | 2.73 s | 7.29 s | +1.13 s |
>
> **Reserving 15 gf costs about half a second per course.** If it converts even one invalid run
> into a valid one, it pays for itself many times over — under Rule 7 a missed waypoint voids the
> entire run, and under Rule 6 only valid times score at all.
>
> This is a deliberate departure from the rulebook's baseline design. Validate it in the simulator
> before committing hardware — but note that `mochi-swarm-sim`'s shipped `BlimpRace` course files
> are **not** the IAR course (9 multi-coloured balloons spread over 90 m / 36 m, versus the official
> 4 red balloons in a 4 × 2 m rectangle). We author our own MuJoCo scene at the correct scale and
> reuse only the blimp model and the tethered-balloon primitive.

---

## 4. Touching the balloon without popping it

Two rules constrain contact, and together they create a design problem nobody's reference material
addresses:

- **Rule 3:** a waypoint counts only if the airship *physically touches* the balloon.
- **Rule 17:** no sharp edges, pointed components or rigid protrusions; external surfaces must
  *"minimize the risk of puncturing the waypoint balloons"*.
- **Rule 16:** must not sever, damage or lift the tether anchor.

So we must reliably make contact with a 50–60 cm mylar balloon at 2 m height, softly, and **know
that we did** — because the state machine has to advance to the next waypoint, autonomously, with
no human in the loop.

Options for touch detection, best first:

1. **Vision proximity + inertial confirmation.** The target's apparent size crosses a threshold,
   then the IMU registers the contact deceleration. No added mass, no protrusions, no rule risk.
2. **Compliant whisker with a microswitch.** Reliable and unambiguous, but a protrusion — it must be
   soft and clearly non-puncturing to satisfy Rule 17, and it adds mass at the worst place.
3. **Dead reckoning.** Drive a fixed distance past the last good detection. Simple, no hardware,
   but silently fails, and a missed touch invalidates the whole run under Rule 7.

Recommend (1), with (3) as a timeout fallback so the vehicle never stalls forever at one waypoint.
Prototype the contact geometry early with a real 22-inch mylar balloon — the envelope will be
bumping into it at roughly its own diameter, and how a sphere-on-sphere contact behaves at 0.4 m/s
is something to find out on a bench, not in a match.

---

## 5. Design conclusions

1. **Spherical envelope**, as small as will lift the electronics with ±10 gf trim margin. Not 36″
   by default — 36″ is the baseline's choice, and smaller is faster if the electronics allow.
2. **Minimising electronics mass is the competitive strategy**, because it lets the envelope shrink,
   which compounds straight into lap time.
3. **Do not build a streamlined envelope**, despite it being genuinely ~7% faster (§3.2). It breaks
   the 1.5 m bounding-box limit at useful volumes, degrades badly in sideslip — which a bicopter
   cannot correct — and is a fabrication project we do not have weeks for.
4. **Do not use the full volume allowance.** A max-size vehicle would be ~5× slower.
5. **Add a small lateral trim thruster** (~15 gf of the 100 gf budget, §3.6). A bicopter structurally
   cannot move sideways, and uncorrectable cross-track error is the measured failure mode on
   comparable vehicles. Costs ~0.5 s per course; buys valid runs.
6. **Invest in terminal guidance and waypoint transitions** — nearly a third of the lap, the part most
   likely to fail on a real vehicle, and the place where published vehicles lose 2–3 s per corner
   against a 0.42 s physical floor.
7. **Exploit the untimed phases** (pre-first-touch and between loops) for stabilisation and yaw
   re-zeroing.
8. **Offboard computation is legal** (Rule 1); only sensing must be onboard (Rule 15). Put the vision
   pipeline on a laptop unless there is a reason not to.
