# Hardware

> **Read [`../README.md` §3](../README.md) first.** This document sizes the envelope by asking
> *"how much payload can this balloon lift?"* — which is the right question when choosing from
> off-the-shelf balloons. The README asks the complementary question: *"how does envelope size
> affect lap time?"* The answer there is that acceleration is inversely proportional to envelope
> volume, because neutral buoyancy forces vehicle mass to equal displaced-air mass while thrust
> stays capped at 100 gf.
>
> **Reconciled sizing rule:** pick the **smallest** envelope that lifts the final electronics mass
> with the ±10 gf trim margin — not the most comfortable one. Section 1.3's 36-inch (0.40 m³)
> recommendation is the *safe* choice; a 31-inch (0.25 m³) envelope would accelerate ~60% harder
> if the electronics fit inside its payload. Which is achievable depends entirely on the envelope
> film areal density, the unverified number this document's own §1.2 sensitivity check identifies
> as the highest-value open item. **Measure it in week 1 before buying anything else.**

This section covers the physical design of the airship: envelope sizing (sea level vs. Bogotá altitude), a gram-by-gram weight budget, a concrete bill of materials, the recommended actuation layout, the gondola/frame concept, and an exhaustive question list to run past the professor about his existing ESP32-S3 PCB. It is built entirely from the research corpus provided; every number that is not a direct rulebook quote or a directly-sourced spec is explicitly flagged as an estimate.

> **Rules used as the design baseline (IAR-LARC 2026, primary-source rulebook, downloaded and verified):** helium volume ≤ ~1 m³ ("50-inch diameter balloon" equivalent, Rule 10); buoyancy neutral within ±10 gf (Rule 11); total thrust ≤ 100 gf across **all** motors combined, servos excluded (Rule 13); robot bounding box ≤ 1 m × 1 m × 1.5 m (Rule 14) — **note Rule 12 says "there are no explicit size limitations," directly contradicting Rule 14; this is a genuine, unresolved contradiction in the rulebook itself, confirmed by two independent people who each downloaded and read the PDF text directly.** All sensing must be onboard the vehicle; computation (e.g., an offboard laptop) is explicitly allowed (Rules 1 & 15). The rulebook's own official "Baseline Design" (Appendix A/B) is the Mochiswarm/BlimpSwarm bicopter platform — the same repos the professor already sent — so this section leans on that platform as the primary engineering reference wherever the corpus supports it.

> **A derived tension worth flagging up front, before any purchase:** the rule's own reference example for the volume cap (a "50-inch diameter balloon," ≈1.27 m) does not fit inside the 1 m × 1 m horizontal footprint of Rule 14's bounding box if built as a literal sphere. In other words, **the bounding box, not the stated 1 m³ helium-volume cap, is very likely the real binding constraint on envelope diameter** — this isn't stated anywhere in the rulebook text, it falls out of combining Rules 10 and 14 arithmetically. Recommend getting this in writing from the organizers (Diego S. D'Antonio, dantonio@oakland.edu; David Saldana, saldana@lehigh.edu) alongside the already-flagged Rule 12/14 contradiction, before finalizing envelope purchase.

---

## 1. Envelope Sizing: Sea Level vs. Bogotá Altitude (~2,640 m)

### 1.1 Physics setup

Net lift per unit volume of helium = ρ_air − ρ_He, at the local pressure and temperature. Both densities are computed from the ideal gas law (ρ = P / (R_specific · T)), with R_air = 287.05 J/(kg·K) and R_He = 2077 J/(kg·K), and ρ_He cross-checked against the NIST reference value (0.1786 g/L at 0 °C, 1 atm).

| Parameter | Sea level (ISA standard) | Bogotá (~2,640 m) | Confidence |
|---|---|---|---|
| Pressure, P | 101.325 kPa | ~74.2–76.7 kPa *(two independent estimates: an ISA lapse-rate model gives 73.4 kPa; a real station reading gives ~76.7 kPa; midpoint 74.2 kPa used below)* | **Uncertain** — no authoritative Colombian meteorological source could be retrieved (IDEAM's own climate-characterization PDF for Bogotá 404'd) |
| Temperature, T | 15 °C / 288.15 K (ISA standard) | ~14 °C / 287 K *(Bogotá's real average — notably warmer than the ISA model's −2 °C prediction at this altitude, because Bogotá is equatorial)* | **Uncertain**, but using the warmer real-world figure is already the *conservative* (lower-lift) choice vs. the colder ISA prediction |
| ρ_air | 1.225 kg/m³ | 0.901 kg/m³ | Derived |
| ρ_He | 0.169 kg/m³ | 0.125 kg/m³ | Derived (NIST reference, ideal-gas-scaled) |
| **Net lift (ρ_air − ρ_He)** | **1.056 g / L of He** | **0.776 g / L of He** | Derived — **a 26.5% reduction in gross lift per liter vs. sea level** |

Using the higher-pressure station reading (76.7 kPa) instead of the ISA-model estimate gives ~0.80 g/L instead of 0.776 g/L — i.e., the 0.776 g/L figure used below is already the conservative (lower) end of the plausible range, which is the right side to err on for a ±10 gf buoyancy-tolerance vehicle.

**Net *payload* loss is worse than gross lift loss**, because envelope film mass is fixed regardless of altitude — the film doesn't get lighter, but the gas lifting it does. This is shown explicitly in the table below.

### 1.2 Worked sizing table (idealized sphere)

Assumptions, flagged: envelope film areal density ≈ **42 g/m²**, derived from a real DIY heat-sealed nylon/EVOH/PE laminate envelope (120 in × 29.17 in, 0.843 m³, 241 g — a real, dimensioned, weighed product, medium confidence). A commercial 61 cm foil party balloon (Qualatex "Bubble") nets only 40 g of payload lift, which back-calculates to a much higher effective areal density (~72–85 g/m², likely due to seams/valve/tape overhead being a bigger fraction of a smaller balloon) — **treat 42 g/m² as an optimistic case and ~85 g/m² as a pessimistic sensitivity check**, both shown below for the recommended size. Real inflated foil balloons are pillow/lens-shaped, not true spheres, and hold somewhat less volume than an idealized sphere of the same nominal diameter — so treat all diameters below as idealized-sphere equivalents, to be confirmed against the actual product once sourced.

| Sphere diameter | Volume V = (4/3)πr³ | Surface area SA = 4πr² | Gross lift @ sea level (V × 1,056 g/m³) | Gross lift @ Bogotá (V × 776 g/m³) | Film mass @ 42 g/m² | **Net payload @ sea level** | **Net payload @ Bogotá** |
|---|---|---|---|---|---|---|---|
| 0.60 m (24 in) | 0.113 m³ | 1.13 m² | 119 g | 88 g | 48 g | 71 g | 40 g |
| 0.70 m (28 in) | 0.180 m³ | 1.54 m² | 190 g | 139 g | 65 g | 125 g | 74 g |
| 0.80 m (31 in) | 0.268 m³ | 2.01 m² | 283 g | 208 g | 84 g | 199 g | 124 g |
| **0.91 m (36 in) — recommended** | **0.400 m³** | **2.63 m²** | **423 g** | **311 g** | **110 g** | **313 g** | **200 g** |
| 1.00 m (39 in, ≈ bounding-box limit) | 0.524 m³ | 3.14 m² | 553 g | 406 g | 132 g | 421 g | 274 g |
| 1.27 m (50 in — rulebook's own reference size) | 1.072 m³ | 5.03 m² | 1,132 g | 832 g | 211 g | 921 g | 621 g |**⚠ exceeds the 1×1 m bounding-box footprint if built as a sphere**

**Sensitivity check at the recommended 0.91 m size, using the pessimistic 85 g/m² film estimate instead of 42 g/m²:** film mass ≈ 223 g → net payload ≈ 199 g at sea level, but only **~88 g at Bogotá**. This is a large swing (200 g optimistic vs. 88 g pessimistic at altitude) driven entirely by one unverified number — **getting an actual grams/m² (or filled-vs-empty weight) figure from whichever balloon supplier is used is the single highest-value open item in this whole section.**

### 1.3 Recommendation

**Primary recommendation: a 36-inch (~91 cm) nominal-diameter foil/mylar sphere balloon.** This:
- Matches the rulebook-endorsed baseline design's own choice (BlimpSwarm's reference BOM uses a 36-inch balloon at $4; the rulebook's Appendix A also cites "~36-inch diameter" as the baseline envelope size) — i.e., it's not an arbitrary pick, it's what the competition's own reference platform already uses.
- Fits comfortably inside the 1 m × 1 m horizontal footprint of Rule 14's bounding box, with margin for the gondola.
- Sits well under the rule's ~1 m³ helium-volume cap (0.40 m³ vs. 1.0 m³).
- Gives an estimated Bogotá net payload of **~200 g (optimistic) to ~88 g (pessimistic)** — see the sensitivity note above.

**Conditional escalation:** if the organizers confirm in writing that (a) Rule 12's "no size limit" language governs over Rule 14's bounding box, or (b) the bounding box is measured on the rigid gondola/structure only and not the compliant balloon envelope (a convention seen in some other LTA competitions, not confirmed for this one), **size up toward 45–50 inches** to match MochiSwarm's own actual choice (their 50-inch balloon gives ~120 gf net buoyancy at sea level in their published paper) for much more comfortable margin. Do not purchase a >1 m envelope until this is confirmed.

---

## 2. Gram-by-Gram Weight Budget

Target: keep dry all-up mass (gondola + electronics + actuation + battery, **excluding** the envelope itself, which is budgeted separately in Section 1) at **~150 g**, leaving roughly 50 g of the optimistic-case ~200 g Bogotá net payload (Section 1) as trim/ballast margin against the rule's ±10 gf buoyancy tolerance. This target is an engineering placeholder, not itself sourced from the corpus — treat it as the number to test against once the TBD lines below are filled in.

| Component | Mass | Source / confidence | Notes |
|---|---|---|---|
| Gondola/frame (laser-cut foam-core, tool-free notch assembly) | **18.7 g** | Sourced — MochiSwarm, arXiv 2503.03077, high confidence (independently re-verified against the primary PDF) | Reference design's actual frame; final mass depends on the footprint needed to fit the professor's PCB + battery + two motor pods |
| ESP32-S3 flight-controller PCB (professor's existing board) | **TBD – source needed** | Not sourced. Espressif's own ESP32-S3-WROOM-1 datasheet gives module *dimensions* (25.5 × 13.97 × 3.1 mm) but no mass figure at all | Bare-module analogues only, not this board: a bare XIAO ESP32-S3 flight-controller board is reported at ~1.7 g (21 × 17.5 mm, Lehigh 2023 low-cost design); an older bare ESP32-WROOM-32 module was community-measured at ~2 g. A **populated custom PCB** (regulator, connectors, motor-driver MOSFETs) will weigh more than a bare module. Budgeted here as a **~10 g placeholder reserve pending actual measurement** — this is an engineering guess, not a sourced number. **Weigh the actual board (Section 6, Q14).** |
| Actuation — 2× motor | **TBD – source needed** (part not yet finalized) | One reference point: a 1102-class brushless motor is reported at ~2.79 g each (medium confidence, aggregated hobby-vendor data, not a datasheet) | Depends on the Section 4 topology decision and on what driver hardware Section 6 reveals is already on the PCB |
| Actuation — 2× servo (if vectored bicopter is used) | **TBD – source needed** | No mass given anywhere in the corpus for the BlimpSwarm reference servo ($3.50); a "smallest commercially available servo" class exists at ~0.45–0.5 g but with no specific sourced part/price | |
| Actuation — 2× ESC / motor driver | **TBD – source needed** | An AGFRC Mini 4A sub-gram ESC (0.5 g) is cited only as an illustrative alternative in the corpus, not a confirmed chosen part | If a MOSFET+PWM driver stage already exists on the professor's PCB (very plausible, see Section 6), this line may be **$0 / negligible added mass** |
| Battery, 1S LiPo | **~7–14 g** (capacity-dependent) | 250 mAh: +7 g, inferred from ESP-FLY's own bare-vs.-with-battery mass delta (18 g → 25 g, sourced). 500 mAh: ~14 g (medium confidence, secondary retailer data) | Capacity trades runtime against buoyancy margin — prefer the smallest capacity that covers required test/flight duration |
| Attachment hardware (elastic cord/clips) | **TBD – source needed** | Priced in the reference BOM ($9.49) but not weighed anywhere in the corpus | Typically light; budget as a small reserve |
| Wiring, connectors, tape, misc. | **TBD – source needed** | No sourced figure anywhere in the corpus | Engineering reserve only, not sourced |
| **Known-sourced subtotal (excluding all TBD lines)** | **~31 g** (frame 18.7 g + motors ~5.6 g + battery-low-end ~7 g) | — | This is a floor, not the real total — most of the vehicle's actual mass is currently in TBD lines |
| **Target dry AUW (excl. envelope)** | **~150 g** | Engineering target, not sourced | ~119 g of margin above the known-sourced floor is reserved for the PCB, servos, ESCs, and misc. hardware once measured |

**Margin discussion:** against the ~150 g target, the recommended 36-inch envelope gives ~200 g of Bogotá net payload under the optimistic film-density case (leaving ~50 g for buoyancy-trim ballast) but as little as ~88 g under the pessimistic case (a **negative margin** against the 150 g target). This single unresolved number (film areal density) currently has more effect on whether the design closes than any BOM line item — resolve it early.

---

## 3. Bill of Materials

All prices are as quoted in the reference source material (mostly the BlimpSwarm/MochiSwarm reference BOM); none have been independently re-confirmed against a Colombia-reachable vendor. Where the corpus gives no part number or price, this is stated explicitly rather than invented.

### 3.1 Actuation (assuming the recommended 2-motor vectored-bicopter topology, Section 4)

| Item | Part / model | Qty | Unit mass | Unit price | Why this part | Alternative | Colombia sourcing / lead-time flag |
|---|---|---|---|---|---|---|---|
| Brushless motor | BetaFPV 1102, 13500 KV | 2 | ~2.79 g *(medium confidence, aggregated vendor data — not a datasheet figure)* | $10.00 | Exact motor in the rulebook-endorsed BlimpSwarm reference BOM; sized to the ~50 gf/side thrust budget derived in Section 4 | Coreless brushed "615/720"-class motor (matches the rulebook's own simpler Appendix-A baseline text *and* the professor's own CircuitDigest/ESP-FLY reference material — needs only a MOSFET driver, no ESC) | Hobby-FPV specialty part (US retailers like BetaFPV/GetFPV/Pyrodrone, or AliExpress/Banggood). **No shipping-time data exists in the corpus** — verify international shipping/customs to Colombia before relying on any delivery date. |
| Propeller | Gemfan 2015, 2-blade | 2 | TBD – source needed | $0.63 | Paired prop for the 1102-class motor in the reference BOM | 30 mm tri-blade prop (as used on ESP-FLY) | Typically bundled with the motor order; same caution as above |
| ESC / motor driver | **TBD – source needed** (no model given anywhere in the corpus) | 2 | TBD – source needed | $10.00 | Needed only if the brushless-motor path is chosen | MOSFET + flyback-diode driver stage (SI2300-class N-MOSFET + 1N4148 + ~10 kΩ gate pulldown) — the exact circuit already documented in the professor's own reference videos/article; may already be implemented on his PCB | Branded ESCs: same specialty-hobby caution as motors. Discrete MOSFETs/diodes: common, lower-risk parts, typically stocked at China-based board houses that already ship internationally. |
| Micro servo (thrust vectoring) | **TBD – source needed** (BlimpSwarm's own reference servo has no stated part number) | 2 | TBD – source needed | $3.50 | Enables the vectored-thrust bicopter topology recommended in Section 4 | A sub-gram servo class exists (~0.45–0.5 g) but no specific sourced part or price was found — **TBD**; or omit servos entirely and use the fixed 3-motor alternative (Section 4) | Common hobby-RC part class; verify Colombia shipping |
| Motor-driver discretes (if building the driver stage in-house) | SI2300 N-MOSFET; 1N4148 diode | 2 per channel | TBD – source needed | ~$0.55/unit (SI2300 — the corpus itself flags a pricing discrepancy between two extractions of the same source, ~$0.55–$2.20 for a set of 4; treat as approximate); ~$0.02/unit (1N4148, from "~$0.08 for 4") | The exact circuit documented in the professor's own reference material (a MOSFET-driver tutorial video and the CircuitDigest article he sent) — may already be populated on his PCB (Section 6, Q11) | Common SMD passives, generally low sourcing risk |

**Thrust budget note:** with 2 motors, the rule's 100 gf total-thrust cap splits to **50 gf per side** — this exact split is even given as a worked example in the rulebook's own Rule 13 text.

### 3.2 Envelope & Lifting Gas

| Item | Part / model | Qty | Unit mass | Unit price | Why this part | Alternative | Colombia sourcing / logistics flag |
|---|---|---|---|---|---|---|---|
| Envelope | 36-inch (~91 cm) foil/mylar sphere balloon — **TBD, specific supplier SKU needed** | 1 (+1–2 spares recommended) | TBD – source needed (Section 1's 42–85 g/m² range is an idealized-sphere estimate, not a spec for a specific product) | $4.00 (BlimpSwarm's own reference price; **not confirmed available at that price from a Colombia-reachable supplier**) | Matches the rulebook-endorsed baseline design's size; computed net payload of ~90–310 g gives working margin (Section 1) | 40–50 in balloon (more margin, matches MochiSwarm's actual choice) **only if** the Rule 14 bounding-box question is resolved in the team's favor (Section 1) | **HIGH PRIORITY.** Source locally in Colombia — an inflated or empty foil envelope is fragile/bulky to ship internationally. The one US brand referenced in the corpus's reference material (Qualatex Microfoil/Bubble) is reportedly discontinued as of mid-2024 (unverified, single-source claim). **Sempertex** (Barranquilla, Colombia) is a plausible domestic balloon manufacturer to contact directly — its product listing could not be confirmed reachable this session (404), so treat as a lead, not a locked-in source. |
| Helium fill | N/A — bulk gas, no part number | As needed for bench testing + event day | N/A | **TBD – source needed.** (US industrial wholesale reference only, ~$12/m³ per USGS 2026 — this is *not* a Colombia retail price and should not be used for budgeting) | Lifting gas | Purity-grade naming is inconsistent across every source checked (the same rough tier is called "Grade A," "Grade 4.5," or "99.995%" by different suppliers) — get an actual written purity certificate from whichever local supplier is used, don't rely on a grade name | **CRITICAL LOGISTICS FLAG.** Compressed/filled helium effectively cannot be shipped internationally for this kind of project (hazmat/customs) — **it must be sourced locally in Bogotá, full stop.** Sempertex advertises 99.995%-purity cylinders marketed for balloons, but the specific product page returned a 404 both times it was checked — price and cylinder size are unknown. **Resolve this by direct phone/email contact well before the event; it is independent of and more time-critical than any part ordered from abroad.** |

### 3.3 Sensors / Vision (optional — pending Section 6 answers about what the professor's PCB already has)

| Item | Part / model | Qty | Unit mass | Unit price | Why this part | Alternative | Lead-time flag |
|---|---|---|---|---|---|---|---|
| Ultrasonic rangefinder | GY-US42 | 1 | TBD – source needed | $11.50 | Altitude/proximity sensing; part of the BlimpSwarm reference design's own optional sensor suite | ST VL53L1X time-of-flight sensor (up to 4 m range, up to 50 Hz) — the exact part used on esp-drone's own position-hold extension board | Not addressed in the corpus; verify sourcing directly |
| Barometer | BMP390 | 1 | TBD – source needed | $12.50 | Altitude sensing; part of the same optional sensor suite (note: barometric altitude alone is coarse, roughly ±25 cm real-world accuracy — pair with a ToF/ultrasonic sensor rather than relying on it alone) | DPS310 or MS5611 (all three commonly share the I2C address pair 0x76/0x77 — check for bus conflicts if the PCB already has one of these) | Not addressed in the corpus; verify sourcing |
| 9-DoF IMU | Adafruit product #4754 | 1 | TBD – source needed | $25.00 | Attitude + heading. A magnetometer matters more for a blimp than a quadcopter: yaw authority is weak/slow (differential thrust from small motors) and flights last much longer, so uncorrected gyro drift accumulates for far longer before it can be bounded by an absolute heading reference — this is a physics-based inference from the platform, not a directly cited rulebook or datasheet claim | ICM-42688-P (modern low-noise 6-axis, **no** magnetometer) or LSM9DS1 (integrated 9-DoF including magnetometer) — final pick depends on what Section 6 reveals is already on the professor's board | Adafruit ships from the US; verify Colombia shipping/customs cost and timeline |
| Vision co-processor (optional) | Arduino Nicla Vision | 1 | TBD – source needed | $115.00 | The exact board used by the rulebook's own baseline design for onboard red-balloon color-blob detection at 10 Hz. MochiSwarm's own authors explicitly offload vision *off* the ESP32-S3 because, in their words, image processing "induces a high computation overhead" on that class of MCU | Given IAR-LARC's actual task is single-color (red), single-shape (balloon) waypoint touching — not multi-object or opponent detection — a simpler onboard HSV/color-blob pipeline running directly on the ESP32-S3's own camera path may suffice without this $115 add-on. This is a design decision to make deliberately, not a mandatory BOM line. | Official Arduino-brand board; verify international shipping/customs cost and timeline to Colombia |

---

## 4. Actuation Topology

**Recommendation: a 2-motor vectored-thrust "bicopter."** One thrust pod per side of the gondola; each pod = 1 motor + 1 servo that tilts the pod's thrust vector between roughly forward and vertical. Yaw comes from commanding unequal thrust *magnitude* between the two sides; net vertical trim comes from tilting both pods together.

**Why this topology, not a quadcopter-style 4-motor layout or something more exotic:**
- This is not an arbitrary choice — it is **literally the IAR-LARC rulebook's own official "Baseline Design"** (Appendix A/B), built on the same two repos the professor already shared (`mochi-swarm-sim`, and the BlimpSwarm firmware it models). Following it means starting from a competition-endorsed reference with reusable firmware: BlimpSwarm's `Differential.cpp` and `mochi-swarm-sim`'s `differential.py` both already implement the mixer this topology needs — `servo_angle = atan2(Fz, Fx)`, `thrust = hypot(Fz, Fx)`, split between the two rotors according to commanded yaw torque — independently confirmed to match between the simulator and the real firmware.
- It minimizes motor count against the rule's hard 100 gf total-thrust cap: **2 motors → 50 gf per side**, which is even given as a worked example in the rulebook's own Rule 13 text. A 3- or 5-motor layout slices the same fixed 100 gf budget into smaller per-motor allocations (Rule 13's own worked example: 5 motors → 20 gf each) for no obvious control benefit at this vehicle's scale.

**Alternatives considered and set aside (for now):**
- **3-motor fixed layout** (2 differential side thrusters + 1 fixed vertical lift prop, e.g. Blimpduino 2): simpler control logic (no vectoring servos), but adds a third motor/driver channel and further divides the 100 gf budget.
- **Passive spin-stabilized layout** (2 fixed motors at the tips of a rotating axle, e.g. Lehigh's "Spinning Blimp"): the simplest possible control (bang-bang, no mixing math at all — that specific build used brushed motors specifically because their slow speed-up/slow-down made bang-bang the practical choice), but the vehicle continuously rotates, which is a poor match for a task that needs to hold a heading toward each waypoint balloon in sequence.
- **5-thruster fully holonomic layout** (2× horizontal-differential + 2× vertical + 1× lateral, e.g. Georgia Tech's GT-MAB): maximum controllability, but heaviest/most complex, and its exact BOM and mass numbers could not be independently verified anywhere in the corpus (the primary thesis PDF was unreachable in every attempt) — not recommended as a starting point.
- **Fin-only actuation** (4 flapping fins, no propellers at all, relying on passive pendulum stability, e.g. ArduPilot's official "Blimp" vehicle type): an interesting low-power alternative, but never validated against this specific competition's requirements in any source found — a longer-shot alternative, not primary.

**Thrust budget:** Rule 13 caps total thrust at 100 gf across all motors combined (servos are explicitly excluded from this budget). Split evenly across a 2-motor bicopter: **≤50 gf per side.**

**Thrust required to counter expected air currents (worked estimate).** No measured air-current or HVAC data for the actual UMNG competition venue exists anywhere in the corpus — this is a genuine open item; recommend an on-site characterization of drafts near doors/vents before the event. As a bounding estimate, steady-state aerodynamic drag on the recommended ~0.40 m³ (36-inch) envelope is:

**F_d = ½ · ρ_air · Cd · A_ref · v²**, where A_ref = V^(2/3) ≈ 0.543 m² (the volumetric reference-area convention used in the airship-drag literature cited in the corpus), Cd ≈ 0.025–0.05 (a commonly-cited range for airship-shaped bodies — flagged medium/low confidence; the primary source for this range could not be independently re-opened this session), ρ_air ≈ 0.901 kg/m³ (Bogotá).

| Assumed indoor air current | Drag force (Cd = 0.025) | Drag force (Cd = 0.05) | As % of the 100 gf thrust budget |
|---|---|---|---|
| 0.3 m/s (gentle draft) | 0.06 gf | 0.11 gf | ~0.06–0.11% |
| 0.5 m/s (moderate draft) | 0.16 gf | 0.31 gf | ~0.16–0.31% |
| 1.0 m/s (brisk draft) | 0.62 gf | 1.25 gf | ~0.62–1.25% |

Even at a brisk 1 m/s indoor draft, steady drag is **under 1.3 gf — negligible next to the 100 gf allowance.** This matches a cross-cutting finding independently confirmed twice in the corpus: because the rulebook's 100 gf thrust cap sits at almost exactly the same order of magnitude as a similar-class vehicle's own net buoyancy (~120 gf reported for a comparable 50-inch-balloon design at sea level), **thrust is intentionally starved relative to lift — the binding constraint is control authority and the envelope's own added (virtual) mass during acceleration, not aerodynamic drag from ambient air currents.** Practical takeaway: don't under-spec the motors to "save weight against drag" — spend the full legal 100 gf, because the real demand is accelerating the envelope's displaced-air inertia and holding a heading against unmeasured drafts or other blimps' wake, not overcoming steady airflow.

---

## 5. Gondola / Frame Concept & Envelope Attachment

**Structure.** A flat, boat/keel-shaped gondola holding — low and along the centerline, for center-of-gravity reasons — the ESP32-S3 PCB, battery, and wiring, with two motor-pod arms extending to port and starboard carrying the vectoring servo + motor of the bicopter layout from Section 4.

**Material — two proven options, both compatible with the team's stated flexibility on 3D printing (design new parts or print suggested ones):**
1. **Laser-cut foam-core board, notch/friction-lock assembly, no glue or screws.** This is the method used by both current academic reference designs found in the corpus — MochiSwarm's own 18.7 g frame, and the Lehigh "Spinning Blimp"'s similar foam-core folded cage + balsa axle. Lightest option found and fastest to iterate/rebuild in the field, which matters given the team's stated hardware comfort level.
2. **3D-printed LW-PLA** (chemically-foaming PLA, e.g. ColorFabb/Polymaker) at 10–20% infill. Foams to roughly 3× its printed volume above ~230 °C nozzle temperature; the vendor's own page claims up to ~65% weight reduction vs. standard PLA at equivalent wall thickness (independently re-confirmed against that page). Use this if the team prefers printing over cutting foam board.

**Attachment to the envelope: elastic cord/string, not tape or adhesive.** Both directly-verified recent builds — MochiSwarm, and the 2023 low-cost Lehigh design's "X-configuration sewing-elastic bands with plastic tension clips" — attach the gondola with elastic specifically to avoid stressing or puncturing the balloon film, and to let the gondola swing and absorb shock rather than transmitting it rigidly into the envelope's seam. Recommend 2–4 elastic loops distributed around the envelope's lower hemisphere/equator, cinched to a light harness or directly to gondola mounting points.

**Stability principle.** Keep the gondola's center of gravity well below the envelope's center of buoyancy — the standard passive "pendulum" stability mechanism for any lighter-than-air vehicle (like a keel on a boat), and the reason the PCB and battery should sit low and central, with the motor pods further out and higher. **Caveat:** the same restoring torque that gives this passive stability also produces a slow pendulum ("swing") oscillation mode that the flight controller has to actively damp rather than fight. This is reported as the explicit research focus of at least one academic miniature-blimp platform (Georgia Tech's GT-MAB) — though that platform's specific numbers could not be independently verified anywhere in the corpus, the qualitative phenomenon is well-supported by standard LTA-vehicle physics and by two other independently-cited robotics papers. **Design the control loop to expect and damp this oscillation from the outset, not as an afterthought.**

---

## 6. Questions to Ask the Professor About His Existing ESP32-S3 PCB

> **Mostly answered (21 Sep 2026).** The board is the Seeed x Max Imagination **ESP-FLY**
> flight controller: XIAO ESP32S3 (native USB, no UART bridge), bare MPU-6050 on I2C
> (SDA GPIO5, SCL GPIO6, address 0x68, INT on GPIO8), four SI2300 N-MOSFETs with 1N4148
> flyback diodes and 10 kΩ gate pull-downs on GPIO7/4/3/1, red/green LEDs on GPIO43/9, JST
> battery connector, 1S. **No magnetometer, no barometer, no ToF, no ESC, no servo.** Pin
> numbers come from the ESP-FLY repository's own `sdkconfig`; the battery-sense pin (GPIO2,
> 10k/10k) is inferred from the schematic and must be checked with a meter. Still open:
> the board's and gondola's **mass** and one motor's **thrust curve**. The single
> recommended addition is a **VL53L1X downward ToF** on the I2C header
> ([docs/10 section 3](10-known-issues.md)). The list below is kept for the record.

His answers to these should be enough to decide, per line, whether to **use as-is**, **modify**, or **respin** the board. Organized by category.

**A. Chip / module identity**
1. Which exact ESP32-S3 module is populated — a bare chip, or a certified module (WROOM-1 / WROOM-1U / WROOM-2 / MINI-1)? Which flash/PSRAM variant, and how much PSRAM is actually populated (this bounds any future camera/vision use)?
2. Does the board expose ESP32-S3's native USB (OTG/CDC) for flashing and debug, or does it rely on a separate USB-UART bridge chip?

**B. Power**
3. What battery connector, chemistry, and capacity is the board designed for (JST-PH 2.0 / 1S LiPo is the pattern across every reference design found — confirm this matches)?
4. Is there an onboard battery-charging IC (e.g., TP4056 / MCP73831), or is charging done off-board?
5. Is there bulk decoupling capacitance (roughly 100–470 µF, low-ESR) placed directly at the ESP32-S3 module's power pins, separate from any regulator-board capacitors? Has any WiFi-TX-induced brownout or unexpected reset been observed during bench testing? (This is the single most commonly reported failure mode for small-battery ESP32-S3 projects in the corpus.)
6. Is there a battery-voltage-monitoring divider already on an ADC pin? Has ESP32-S3's known ADC nonlinearity (worst around 2.5–3 V) been accounted for or calibrated?

**C. Sensors**
7. What IMU (and barometer/magnetometer, if any) is already routed/soldered, and at which I2C address/bus? (Several common barometer parts share the address pair 0x76/0x77 — check for conflicts before adding another sensor.)
8. Is a magnetometer present? If not, is there a spare I2C/SPI breakout on the board to add one? (Flagged above as more important for a blimp than for a quadcopter, given weak/slow yaw authority and long flight durations.)
9. Is a camera module populated or planned? If so, which sensor (OV2640/OV3660/OV5640/etc.), and is there enough PSRAM populated for the intended resolution?

**D. Actuation interfaces**
10. What motor-driver topology is already implemented on the board — discrete MOSFET + PWM (brushed-motor-only, matching the exact circuit in the CircuitDigest article and ESP-FLY reference material the professor already sent), or an ESC/DShot-capable path (brushless-capable)? How many motor channels and how many servo channels are actually routed, and to which GPIOs?
11. If MOSFET-driven, which part numbers are populated (e.g., SI2300-class), and what's the flyback-diode / gate-pulldown configuration? (This determines whether Section 3's "motor-driver discretes" BOM line is needed at all, or is already built.)

**E. RF / comms**
12. Has actual WiFi range/RSSI been bench-tested on this board? Is there a documented antenna keep-out zone (roughly ≥15 mm copper-free on all layers), or is the antenna part of a certified module (lower RF risk) vs. a custom PCB trace antenna (needs its own tuning/certification)?
13. What communication architecture is intended — fully onboard autonomy, or an offboard-compute link (WiFi/ESP-NOW) to a ground laptop? Rule 15 requires sensing onboard but explicitly permits offboard computation (Rule 1) — this materially changes whether the PCB needs a robust telemetry-grade link or just a manual-override channel for testing.

**F. Mechanical / weight**
14. Has the populated board (with connectors) actually been weighed? What is its exact mass? **(This is the single biggest unknown feeding the entire weight budget in Section 2 — resolving it should be the very first physical task.)**
15. What are the board's physical dimensions, mounting-hole pattern, and connector placement? (Directly shapes the gondola frame design in Section 5.)

**G. Firmware / software readiness**
16. What firmware is the board currently running or intended to run — Espressif's esp-drone (Crazyflie-derived), a from-scratch sketch, or something else? Which ESP-IDF (or Arduino-ESP32) version is it built against? (esp-drone/ESP-FLY-lineage firmware is pinned to the ESP-IDF `release/v5.0` branch specifically; newer components such as micro-ROS target IDF v5.4+ and would need a version-reconciliation effort to combine with it.)
17. Has the board been flight-tested at all, even on a bench thrust-stand rig? If so, what specific failure modes were seen (resets, motor-direction/wiring issues, sensor dropouts)?

**H. Quantity / spares / documentation**
18. How many of these PCBs exist (assembled and bare)? Are spares available in case of damage during integration or at the competition?
19. Are schematic, Gerber, and BOM files available for this board? (Would let the team answer most of the above from the files directly, without needing to physically probe the hardware.)

**Decision guide.** Broadly: **use as-is** if the board already has a matching motor-driver topology to whatever Section 4 finalizes on, has adequate decoupling capacitance with no observed brownouts, and either already has (or doesn't need) a magnetometer and camera path. **Modify** if the gaps are additive — missing decoupling caps, a missing sensor breakout, a different firmware branch, a connector swap. **Respin** if the gaps are structural — e.g., no motor-driver path at all for the chosen actuation approach, insufficient PSRAM for a planned camera use case, an unusable/untested antenna, or a chip variant that can't support the required peripheral count.

---

## Key open items to resolve (cross-cutting, ranked by how many downstream numbers they affect)

1. **Weigh the professor's existing ESP32-S3 PCB.** This single number currently blocks finalizing the weight budget (Section 2) and therefore the final envelope size decision (Section 1).
2. **Get the actual envelope film areal density (or filled weight) from whichever balloon supplier is used** — the corpus's own uncertainty range (42–85 g/m²) swings the Bogotá net-payload estimate from ~200 g down to ~88 g for the recommended size.
3. **Get the Rule 12 vs. Rule 14 size-limit contradiction — and the derived volume-vs-bounding-box tension — resolved in writing by the organizers** before purchasing an envelope.
4. **Resolve helium sourcing in Bogotá** (contact Sempertex directly, or another local industrial-gas distributor) — this is independent of, and more time-critical than, any part ordered from abroad, since filled gas cannot practically be shipped internationally.
5. **Characterize actual venue air currents** (HVAC vents, doors) on-site before finalizing thrust margins, since no such data exists in the corpus and the current drag estimate is built on assumed draft speeds.
6. Several BOM prices are quoted from a 2024/2025-era US reference platform and are not confirmed available at those prices, or at all, from a Colombia-reachable supplier — verify before budgeting.


---

## Flagged uncertainties

- Bogotá atmospheric pressure/temperature (74.2–76.7 kPa, ~287 K) used throughout the sizing calculation are engineering estimates, not sourced from an authoritative Colombian meteorological reference (IDEAM's page was unreachable in the underlying research).
- Envelope film areal density (42 g/m², used as the primary sizing assumption) is derived from one DIY builder's envelope, not from a Colombia-based balloon supplier's spec; the real value could be as high as ~85 g/m², which would cut the recommended envelope's Bogotá net payload from ~200 g down to ~88 g.
- The IAR-LARC 2026 rulebook contains an unresolved internal contradiction (Rule 12 'no explicit size limitations' vs. Rule 14's 1×1×1.5 m bounding box); this dossier designs to the explicit 1×1×1.5 m cap as the conservative choice, but this should be confirmed in writing with the organizers before final envelope purchase.
- A further tension not stated anywhere in the rulebook itself: a sphere at the rule's own reference volume (~1 m³, '50-inch diameter') is ~1.27 m across, which would exceed the 1×1 m horizontal bounding-box footprint if built as a literal sphere. This dossier's sizing recommendation treats the bounding box as the binding constraint — confirm with organizers.
- The professor's existing ESP32-S3 PCB's mass, exact module/PSRAM variant, sensor population, and motor-driver topology are all unknown and are the source of most of the 'TBD' lines in the weight budget and BOM; the 19-item checklist in Section 6 is meant to close this gap.
- No measured air-current or HVAC data for the actual competition venue exists anywhere in the source research; the thrust-vs-drag calculation in Section 4 uses assumed indoor draft speeds (0.3–1.0 m/s) and an airship drag coefficient (Cd ≈ 0.025–0.05) that could not be independently re-verified against a primary source.
- Several BOM prices reflect a 2024/2025-era US-based reference platform's component costs (BlimpSwarm) and are not confirmed to be available, or priced the same, from any Colombia-reachable supplier.
- The claim that Qualatex Microfoil/Bubble balloons were discontinued as of mid-2024 is a single, unverified source in the underlying research and should not be treated as settled.