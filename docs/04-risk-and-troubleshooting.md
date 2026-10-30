
# Risk Register & Troubleshooting Playbook

Vehicle: ESP32-S3 bicopter-style airship (Mochiswarm/BlimpSwarm-derived), ≤1 m³ helium, ≤100 gf total thrust, ±10 gf buoyancy, ≤1×1×1.5 m envelope. Competition: IAR-LARC 2026, UMNG Bogotá (elevation 2,640 m), 10 m×5 m arena, 4× red 22 in mylar waypoint balloons at 2 m height, fully autonomous, all sensing onboard (computation may be offboard).

## 1. Risk Register

Ranked by expected damage (likelihood × impact), highest first.

| # | Risk | Category | Likelihood | Impact | Early warning sign | Mitigation | Contingency |
|---|------|----------|-----------|--------|--------------------|-----------|-------------|
| 1 | **Buoyancy budget doesn't close at Bogotá altitude.** Net lift/L at 2,640 m (~0.776 g/L) is ~26% below sea level (~1.056 g/L), and because envelope film mass is fixed, net *payload* loss is worse (~36% in a worked 0.70 m³ example: 296 g→189 g). A design that's neutrally buoyant at your bench (near sea level, if outside Bogotá) will be heavy in Bogotá. | Technical | High | High | Vehicle sinks or needs excess ballast-negative thrust to hover when first inflated on-site. | Do all mass budgeting against Bogotá air/He density, not sea-level tables; build in ≥30 g of spare lift margin (bigger envelope or lighter gondola) beyond the ±10 gf rule tolerance. | Carry a second, larger spare envelope (up to the 1 m³/50 in-equivalent cap) and extra helium to re-fill/upsize on-site rather than stripping electronics under time pressure. |
| 2 | **Pre-registration deadline (Sept 14, 2026) has already passed** relative to today (Sept 21, 2026); team may not be validly entered. | Schedule/Rule | High | High | No confirmation email/portal access from LARC registration system. | Email larslarc@unimilitar.edu.co and dantonio@oakland.edu *today* asking whether late/rolling pre-registration is possible; register on the paid portal (asistiendo.com) before Sept 28 to at least lock early-bird pricing while the pre-reg question is resolved. | If pre-registration is hard-blocked, request an exception in writing citing the new-category (first-edition) status of IAR; escalate through David Saldaña (co-organizer, saldana@lehigh.edu) if Oakland contact is slow. |
| 3 | **Helium sourcing/refill in Bogotá is undocumented.** No mention of organizer-supplied helium anywhere on UMNG/Oakland IAR pages; Sempertex's advertised 99.995% balloon-helium product page 404s on direct check. | Logistical | High | High | You reach competition week without a confirmed local helium vendor or organizer-provided gas. | Call Sempertex (Barranquilla, Colombian balloon-gas manufacturer) directly by phone (not just their web page) 4-6 weeks out; simultaneously email organizers asking if helium is provided on-site; get a written quote/cylinder-size confirmation, not just a page reference. | Identify a backup local industrial-gas distributor near Bogotá (hardware-store/party-supply helium tanks are usually only ~92-95% pure and will under-fill your lift budget — budget for it) and price two vendors in case one falls through. |
| 4 | **WiFi/2.4 GHz congestion in the arena.** ESP32-S3 has exactly one 2.4 GHz radio and no 5 GHz option at all; ESP-NOW shares the same band as every other team's WiFi/AP and is not immune to interference; in `WIFI_MODE_APSTA` ESP-NOW is forced onto whatever channel the STA link negotiates. With many teams' vehicles + ground stations running simultaneously, packet loss/latency will spike. | Technical/Competition environment | High | High | Control link that worked fine at home degrades, stutters, or drops entirely once other teams' hardware powers up nearby. | Design the failsafe (buoyancy-neutral, thrust-to-zero) assuming the link *will* drop during a run; keep packets small (fit ESP-NOW v1's 250-byte limit); test the exact firmware in a deliberately crowded WiFi environment (busy coffee shop/library) before travel, not just at home. | Bring a dedicated non-WiFi backup link (e.g., a small ESP-NOW-only or 900 MHz radio pair reserved purely for a kill/reset command) that doesn't depend on the congested WiFi band. |
| 5 | **Arena air currents (HVAC drafts, doors opening, other vehicles' prop wash) overpower the vehicle.** Total thrust is capped at 100 gf and the added-mass/drag of a ~1 m envelope is large relative to that thrust budget — this is an inherently low-control-authority vehicle. | Technical | High | High/Medium | Vehicle drifts noticeably off a straight commanded path during any practice session, worse near doors/vents. | During the scheduled practice-time window (Rule 23 gives dedicated arena time), specifically test near doors/HVAC vents, not just in the room's center; request the organizers disable/redirect HVAC airflow near the course if possible. | If drift is unmanageable, bias the flight plan to favor the course direction least affected by the dominant draft (test both loop directions during practice) rather than trying to out-thrust the disturbance. |
| 6 | **Control loop fights the vehicle's own passive pendulum stability**, producing growing oscillation instead of settling (CG-below-CB restoring torque is a feature, not a bug, but a fast quad-style attitude PID can actively excite it). | Technical | Medium | High | Bench/tethered test shows swinging that grows over time rather than damping out after a bump. | Do not port a quadcopter attitude-PID cascade wholesale; tune specifically for damping (rate/derivative-heavy, low proportional gain) and validate in `mochi-swarm-sim` before first tethered flight. | If oscillation persists, add passive mechanical damping (small drag fin/disk on the gondola) rather than continuing to chase gains in firmware under time pressure. |
| 7 | **Vision/perception fails to reliably detect the red balloon under actual venue lighting**, or the single ESP32-S3 core stalls the flight-control loop while running detection. | Technical | Medium-High | High | Detector works at home/lab but loses track or false-triggers under fluorescent/mixed venue lighting; control loop timing jitters when vision runs. | Calibrate LAB/HSV thresholds on-site under the arena's actual lighting using the get-gains → fill-grid → Gaussian-fit workflow (proven pattern from the reference platform), not thresholds tuned at home; if using onboard vision, benchmark actual achieved FPS with the flight-control loop also running, don't assume headroom. | Move detection to an offboard laptop over WiFi (explicitly rules-legal — only *sensing* must be onboard, Rule 15) if the ESP32-S3 can't hold both jobs; keep the raw camera stream as the only onboard vision duty. |
| 8 | **Rule 12 vs Rule 14 contradiction on vehicle size** — Rule 12 says "no explicit size limitations," Rule 14 gives an explicit 1×1×1.5 m bounding box in the same document. | Competition-rule (unknown-rules) | Medium | Medium-High | You discover during a pre-competition safety inspection that your envelope (sized to the "no limit" reading) doesn't fit the box. | Design to the explicit 1×1×1.5 m cap regardless of Rule 12's wording; email dantonio@oakland.edu now for written clarification and keep the reply on file. | If already oversized, a smaller backup envelope (see Risk #1's spare-envelope plan) doubles as the size-compliant fallback. |
| 9 | **WiFi-TX brownout resets the ESP32-S3 mid-flight.** WiFi TX current bursts to ~180-355 mA can sag a small 1S LiPo below the ~2.4 V brownout threshold, especially under load from motors simultaneously. | Technical/Electronics | Medium | High | Random resets or brief "hangs" that correlate with WiFi activity (telemetry burst, reconnect) during bench tests, not with motor load alone. | Confirm/add bulk capacitance (100-1000 µF low-ESR + 100 nF ceramic) directly at the ESP32-S3 module's power pins, separate from any regulator-board caps; verify battery C-rating margin. | If resets persist under time pressure, throttle telemetry/vision transmission rate to reduce burst frequency as a stopgap while the capacitor fix is validated. |
| 10 | **Motor thrust budget (100 gf total, servos excluded) is marginal against real drag + added mass**, especially for tight turns inside the 4 m×2 m waypoint rectangle within the loop-time window. | Technical | Medium | Medium-High | Bench thrust-stand measurement of the actual assembled bicopter comes in noticeably under the expected/simulated value, or turns take visibly longer than planned in sim. | Static thrust-test the actual built motor/prop/driver combination early (don't trust datasheet thrust numbers); cross-check against `mochi-swarm-sim`'s course geometry to confirm the planned turning radius is achievable within the thrust cap. | If margin is too thin, reduce envelope size/drag (accepting less lift margin, trade against Risk #1) rather than expect more thrust, which is rule-capped. |
| 11 | **Firmware hardcodes a fixed waypoint order**, but the rulebook states the visiting order is announced per-run, not fixed in advance. | Competition-rule/Technical | Medium | Medium | Code review shows waypoint sequence as a compile-time constant/array literal. | Make the waypoint sequence a runtime-loadable parameter (e.g., set at arm-time via a short command or config read), not a hardcoded array, from the first working autonomy build onward. | If discovered late, a quick field patch (recompile+flash with the announced order baked in) is possible in the minutes before a run only if you rehearse that exact turnaround during practice. |
| 12 | **Helium leak/envelope degradation between practice and the scored match** (latex loses ~1-2%/hour, useful life ~8-12 hours; mylar/foil lasts days but seam/valve leaks are still possible). | Technical/Logistical | Medium | Medium | Envelope feels "soft" or the vehicle trims heavy on match morning despite being neutral the prior evening. | Use mylar/foil envelope only (not latex) for anything spanning the multi-day event; soap-water leak-test the fill valve and seams the night before each competition day. | Keep the spare helium tank (Risk #3) and a top-off routine scheduled ~30 min before each scored run to allow the gas to reach thermal equilibrium before final trim. |
| 13 | **Travel/shipping logistics for an inflated (or refillable) envelope across international travel to Bogotá.** No documentation found on whether helium/inflated envelopes can be carried, or must be re-sourced entirely on arrival. | Logistical | Medium | Medium | Realizing at trip-packing time that the envelope can't legally/practically fly inflated. | Plan to travel with the envelope deflated/empty and source helium entirely in Bogotá (see Risk #3); confirm this plan explicitly with the professor's travel arrangements weeks ahead, not the week of. | If local helium sourcing falls through entirely, identify whether the venue (UMNG) or another competing team can share a supply arrangement — ask organizers to broker this. |
| 14 | **Motor CW/CCW or driver-channel mismatch** causes a flip/uncontrolled yaw at first throttle-up — the single most commonly reported failure mode in every ESP32-S3 micro-vehicle build in this project's reference lineage (ESP-FLY, BlimpSwarm). | Technical (bench) | Medium-High | Low-Medium (bench-catchable) | Vehicle spins/flips instead of translating cleanly the first time thrust is applied. | Before final assembly, bench-test each motor individually with the prop OFF to confirm spin direction and driver-channel mapping against the wiring diagram; only mount props once every channel is confirmed. | If discovered after assembly, swap prop handedness (not motor wiring) as the faster fix, since props are cheaper/faster to re-seat than re-soldering driver channels. |
| 15 | **Team Description Paper (TDP) deadline (Oct 12, 2026) missed.** | Schedule | Low-Medium | High | No draft exists by early October. | Assign TDP ownership and a draft date (e.g., Oct 1) now, independent of hardware progress — treat it as a hard-blocking deliverable, not an afterthought. | If at risk, submit a minimal-but-complete draft by the deadline and revise afterward if the format allows; a late/thin TDP is recoverable, a missed one may not be. |
| 16 | **Event date inconsistency** — IAR page states Nov 2-6, 2026; the main UMNG mecatronica page states Nov 4-6, 2026 for the same event. | Competition-rule (unknown-rules)/Logistical | Low-Medium | Medium | Flight/hotel booked for the wrong window, arriving to find check-in/inspection already underway (or waiting 2 unplanned days). | Email organizers now for the authoritative date range and get it in writing before booking any travel. | If already booked for the shorter window, arrive on Nov 2 regardless (covers both possible ranges) and treat Nov 2-3 as buffer/setup days if the main event turns out to start Nov 4. |
| 17 | **Own vehicle punctures a target balloon** (Rule 17 explicitly bans sharp/rigid puncturing protrusions, since the airship must physically touch fragile mylar waypoints repeatedly). | Competition-rule/Technical | Medium | Medium | Bench mock-up has any exposed screw tip, motor mount edge, or rigid corner facing outward. | Add a soft foam bumper/ring around any gondola surface that will contact a waypoint balloon; inspect for exposed fasteners before every run. | If a puncture happens during practice, it's a cheap fix (foam bumper retrofit); if it happens during a scored run, expect the run to be invalidated per Rule 7 — treat bumper design as non-optional, not cosmetic. |
| 18 | **GPL-3.0 obligations from esp-drone/Crazyflie-derived firmware** — if the team's controller code is a derivative work built on esp-drone and the vehicle/source is handed to or inspected by judges, this may count as "conveying" under GPLv3, obligating release of corresponding source for the team's own modifications. | Competition-rule/Legal | Low | Low-Medium | Team assumes custom control code can stay private while still being layered on GPL-3.0 firmware. | Have a 5-minute conversation with the professor about this before assuming any part of the codebase can stay closed; check the TDP/competition rules for their own source-disclosure requirements (which may make this moot anyway). | If it becomes an issue, isolating blimp-specific control logic into a separate, cleanly-licensed module (even if it still links against GPL code) at least clarifies what is being disclosed. |

## 2. Subsystem Troubleshooting Playbook (bench-with-vehicle-in-hand)

### Envelope / Trim

| Symptom | Likely cause | Fix |
|---|---|---|
| Vehicle sinks steadily over the course of a session | Helium diffusion through seams/valve, or gas cooling (temperature drop reduces volume/lift) | Top off helium ~20-30 min before each scored run to let gas reach room temperature before final trim; use mylar/foil envelope, not latex |
| Vehicle rises uncontrollably once electronics are mounted | Net buoyancy exceeds +10 gf tolerance | Trim with small ballast increments (washers/fishing weights) on a 0.1 g-resolution scale until within the ±10 gf window |
| Was neutral at home/lower altitude, heavy in Bogotá | Didn't rebudget for ~0.776 g/L net lift at 2,640 m vs ~1.056 g/L at sea level | Recompute lift using Bogotá air density before finalizing envelope size; don't trust a sea-level trim session as final |
| Envelope feels soft by the next morning | Overnight leak at the fill valve or a seam, not fabric diffusion | Soap-water leak test the valve/seams the night before; carry a spare tank |
| Persistent lean to one side at rest | Ballast unevenly placed, or gondola suspension points asymmetric | Rebalance ballast placement; verify gondola hangs symmetrically under the envelope's center |
| Growing pendulum-like swinging after any bump | Passive CB-above-CG restoring torque underdamped, or fought by an overly aggressive control loop | Add a small drag fin/disk for mechanical damping, and/or retune control to damp rather than correct fast |

### Propulsion

| Symptom | Likely cause | Fix |
|---|---|---|
| Vehicle spins/flips instead of translating on first throttle-up | Motor CW/CCW mismatch or reversed driver channel (most common failure in this build lineage) | Bench-test each motor individually, prop off, before final assembly; confirm spin direction against the wiring diagram |
| One motor clearly weaker or unresponsive | Blown driver MOSFET (SI2300/SI2302), bad solder joint, or reversed flyback diode | Multimeter continuity check per channel; verify diode band orientation matches schematic |
| Throttle response feels laggy/mushy | Inherent brushed-coreless motor spin-up lag (worse than brushless) | Add feedforward/anticipation in firmware rather than expecting crisp linear response; consider bang-bang control if timing dominates |
| Can't turn crisply enough within loop time despite "enough" thrust on paper | 100 gf total-thrust cap is genuinely tight against added mass/drag of a ~1 m envelope | Bench thrust-test the *actual assembled* motor/prop/driver combo (not datasheet numbers); reduce envelope drag rather than expect more thrust |
| Motor housing warm/hot after a short bench run | Stalled rotor, prop rubbing the frame, or overcurrent beyond driver rating | Check mechanical clearance; verify driver current rating against measured motor stall current |

### Electronics / Power

| Symptom | Likely cause | Fix |
|---|---|---|
| Random resets that correlate with WiFi activity, not motor load | 1S LiPo sag under ~180-355 mA WiFi-TX current spikes, below brownout threshold (~2.4 V) | Add 100-1000 µF low-ESR bulk cap + 100 nF ceramic directly at the ESP32-S3 module's power pins; confirm battery C-rating |
| Battery percentage reading jumps around/is wrong | ESP32-S3 ADC nonlinearity in the 2.5-3 V input range, uncalibrated divider | Two-point calibrate (4.2 V, 3.0 V) in firmware; add a 100 nF filter cap on the ADC pin |
| IMU readings noisy, don't match physical motion | IMU not rigidly mounted; gondola/mount flex | Stiffen the IMU mount; avoid flexible foam/glue directly under the sensor |
| IMU fails to init or gives garbage over I2C | Counterfeit/clone MPU6050-class chip failing above ~100-400 kHz, or address conflict on the bus | Drop I2C clock to 100 kHz; bench-test the IMU alone before integrating with the rest of the sensor stack |
| Board won't boot after gondola is sealed | No physical reflash access planned; strapping-pin conflict | Plan an OTA firmware-update path *before* final sealing; verify boot-strap pins aren't driven by other peripherals |
| Yaw estimate drifts steadily over a multi-minute flight | No magnetometer; gyro-only integration drifting over a long, low-yaw-rate flight | Add a magnetometer, or plan periodic re-zeroing at known headings if one isn't available |

### Control

| Symptom | Likely cause | Fix |
|---|---|---|
| Oscillation grows instead of settling after a disturbance | Attitude control actively fighting/exciting the passive pendulum mode | Retune for rate/derivative-heavy damping, low proportional gain; don't port a quadcopter's fast attitude PID wholesale |
| Vehicle consistently overshoots each waypoint | Control law assumes fast response; doesn't account for high added mass/slow deceleration | Model added mass explicitly; increase braking distance/lead time before each waypoint |
| Works in `mochi-swarm-sim` but not on hardware | Mixer copied from a 4-motor quad template (`NBR_OF_MOTORS`-style assumption), not adapted to a 2-3 motor bicopter | Rewrite the power-distribution/mixer specifically for the actual motor count and geometry |
| Vehicle can't follow the announced waypoint order | Sequence hardcoded at compile time instead of runtime-configurable | Make waypoint order a runtime parameter set at arm-time, not a compiled-in array |

### Comms

| Symptom | Likely cause | Fix |
|---|---|---|
| ESP-NOW telemetry works on the bench, drops when WiFi is also active | Single-radio channel lock — ESP-NOW forced onto whatever channel the WiFi STA/AP negotiated | Explicitly pin/verify both links to the same channel, or use a channel-sync helper pattern; consider physically separating the two links |
| Control link degrades badly on competition day vs at home | 2.4 GHz congestion from many teams' APs/telemetry in the same hall | Test beforehand in a deliberately crowded WiFi environment; keep packets small; have a non-WiFi fallback for critical commands |
| Ground station stops responding but vehicle keeps flying open-loop | UDP/socket/port collision on the ground-station side, not a vehicle failure | Log ground-station timeout separately from vehicle-side telemetry loss; don't assume the vehicle crashed |
| Range is shorter indoors than open-air spec | Normal indoor WiFi attenuation for this hardware class | Arena is only 10 m×5 m, well within typical indoor range if antenna keep-out area on the PCB wasn't violated — check that first if range is unexpectedly poor |

### Perception

| Symptom | Likely cause | Fix |
|---|---|---|
| Balloon detector loses track or never finds the target | Color thresholds tuned under different lighting than the venue | Recalibrate on-site under actual competition lighting (grid-fill + Gaussian-fit workflow), don't trust thresholds tuned at home |
| Vehicle "scores" a touch it didn't make | No dwell/confirm-frame gating; single-frame false positive | Require several consecutive confirming frames before committing to a "touched" state |
| Flight control stutters whenever vision runs | Single core doing both flight control and image processing, resource contention | Offload detection to an offboard laptop over WiFi (rules-legal — only sensing must be onboard); keep only the camera stream onboard |
| Re-acquisition after losing the target takes too long | No active search behavior; camera stays pointed the wrong way | Implement a spiral-search yaw-sweep pattern rather than passively waiting for the target to reappear |

## 3. Pre-Flight Checklist (bench, before any test/practice flight)

1. Weigh the fully assembled vehicle on a precision (0.1 g) scale; confirm within ±10 gf of the target for the *current* fill and room temperature.
2. Visual envelope inspection: no punctures, seams intact, fill valve sealed.
3. Battery: full charge, no swelling, connector secure.
4. Spin-test each motor individually with props OFF — confirm direction and no abnormal noise/heat.
5. Prop installation check — correct CW/CCW per motor, tight, undamaged.
6. Sensor sanity check via serial log — plausible values, no NaN/garbage from IMU/baro.
7. Control-link test at expected range — confirm command latency is acceptable.
8. Vision pipeline live check — point at an actual red balloon sample, confirm detection under current lighting.
9. Firmware version check — matches the last known-good tested build.
10. Confirm waypoint sequence is runtime-settable, not hardcoded (loaded value is a placeholder until match time).
11. Confirm failsafe behavior: loss of link → zero thrust (drift/rise), not an active "descend."
12. Confirm assembled bounding box fits within 1×1×1.5 m with margin.

## 4. Pre-Match Checklist (at the arena, immediately before a scored run)

1. Get the exact waypoint visiting order for this run/course from officials.
2. Re-weigh/re-trim buoyancy — account for the venue's temperature/humidity vs. the last test (let topped-off helium settle ~20-30 min).
3. Confirm the WiFi/ESP-NOW channel assignment with organizers to avoid collision with other teams' links.
4. Battery: full charge confirmed, spare battery staged and charged.
5. Visual check: no new punctures/soft spots on the envelope since the last test.
6. Test the control link from the actual start position — confirm no dead zone toward the far end of the course.
7. Recheck vision calibration under the arena's *actual* lighting at that time of day (not a hallway or hotel-room test).
8. Confirm autonomy-only mode is engaged — no manual-override path active (Rule 1 compliance).
9. Confirm start position is within the 2 m start circle.
10. Brief the team on the abort/retrieval procedure if the vehicle drifts out of bounds or the link is lost.
11. Confirm the start/scoring protocol and timing with officials before the run begins.
12. Confirm which of the up-to-3 loop attempts this is, and the current best time to beat.

## 5. Field Kit / Spares List (competition day)

**Electronics:** 2-3 spare ESP32-S3 boards, fully flashed and bench-tested; spare IMU/baro/magnetometer breakout if used; spare MOSFET drivers (SI2300/SI2302); soldering iron, solder, flux; multimeter; spare JST connectors and wire; heat shrink; precision screwdriver set; tweezers; magnifying loupe.

**Propulsion:** Spare motors (matched CW/CCW pairs); several spare propeller sets; spare micro servos; spare motor-driver boards.

**Power:** 3-4 charged spare 1S LiPo batteries; charger with wall/car adapter; LiPo-safe storage bag; standalone voltage checker.

**Envelope:** One fully spare mylar/foil envelope (pre-cut/sealed); a second helium tank (given sourcing uncertainty, see Risk #3) with regulator/fill nozzle; mylar-safe repair tape; soap-water leak-test bottle; fill funnel.

**Ballast/trim:** Precision gram scale (0.1 g resolution); assorted small washers/fishing weights; adhesive dots or hook-and-loop tape for ballast attachment/removal.

**Structural:** Spare gondola frame (pre-cut foam-core or pre-printed); elastic cord/string for envelope-gondola attachment; zip ties; gorilla/kite tape; hot glue gun and sticks; any spare 3D-printed parts.

**Computing:** Laptop with the full dev environment pre-installed *offline* (toolchain, no reliance on venue WiFi to fetch anything); several data-capable USB-C cables; a portable, isolated WiFi router/AP for controlled bench testing away from venue congestion; backup laptop if available.

**Documentation:** Printed rulebook, printed wiring diagrams/pinout, laminated copies of the pre-flight and pre-match checklists, team contact list.

**Misc:** Marker/label maker for wiring; small flashlight/headlamp; basic first-aid; small toolkit (pliers, wire cutters, hex keys); phone/camera to document any issue for later debugging; spare SD card if used for logging.

## 6. Top 5 Things Most Likely to Kill This Project — and the Cheapest Experiment to De-Risk Each Early

1. **The buoyancy budget never closes at Bogotá altitude.** Cheapest de-risking experiment: weigh the actual assembled PCB + battery + motors + gondola *today*, and separately inflate one real envelope (of the exact type/size you intend to use) and measure its *actual* lift on a kitchen/postal scale — not the ideal-gas-law number — before recomputing the Bogotá-altitude budget. This alone catches an infeasible design weeks before it becomes a travel problem.

2. **The control loop fights the vehicle's own passive stability, producing an uncontrollable pendulum swing.** Cheapest de-risking experiment: before ever inflating a balloon, run the bicopter mixer and controller in `mochi-swarm-sim` against the actual course XML files, and separately hang the bare gondola on a string to physically observe and time its natural pendulum period — tune damping against that measured period, not a guess.

3. **The comms link collapses in a crowded competition hall.** Cheapest de-risking experiment: take the exact chosen radio stack (WiFi SoftAP, ESP-NOW, or both) into a deliberately congested real-world WiFi environment (a busy campus building or coffee shop with 20+ visible networks) and log packet loss/latency — this costs nothing but an afternoon and will surface the channel-lock and congestion problems months before competition day.

4. **The 100 gf thrust budget can't overcome real drag/added mass to turn within the loop-time window.** Cheapest de-risking experiment: static thrust-test the *actual* assembled 2-motor bicopter (motors + props + driver, on the bench, held down) to get real thrust numbers, then feed those numbers into `mochi-swarm-sim`'s course geometry to see if the planned turning radius and loop time are achievable — this is a half-day bench test that avoids discovering an underpowered design at the arena.

5. **The perception pipeline can't reliably find the red balloon, or stalls the flight-control loop.** Cheapest de-risking experiment: bench-test the color-blob detection (onboard or offboard) against a real waypoint-spec balloon under two or three different lighting conditions (fluorescent, LED, daylight) using whatever camera hardware you already have, while simultaneously running the flight-control loop, and measure both detection reliability and control-loop timing jitter — this is a one-afternoon test that validates (or kills) the entire perception architecture before it's wired into the final vehicle.
