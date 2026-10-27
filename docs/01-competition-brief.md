## COMPETITION BRIEF — IAR‑LARC 2026 (International Airship Racing)

> **Rulebook status: LOCATED AND READ.** Unlike a typical "we never found the PDF" situation, the actual category rulebook (`IAR_LARC2026.pdf`, dated **March 27, 2026**) was downloaded directly and read in full, and cross-checked twice more in independent verification passes. It is publicly downloadable with a plain HTTP request — no login needed — at:
> **https://mecatronica.umng.edu.co/index.html/rules/IAR_LARC2026.pdf**
> (also linked, behind a Google Drive viewer, from https://sites.google.com/oakland.edu/iar-larc/rules)
>
> So the facts below are mostly **CONFIRMED against the primary rulebook text**, not inferred. The exceptions — flagged explicitly — are: (1) a real, unresolved **date contradiction** across the organizers' own pages, (2) a real, unresolved **self-contradiction inside the rulebook** about vehicle size, (3) a handful of logistics items (helium supply, exact schedule, RF rules) that genuinely are **undocumented anywhere** and need a direct email, and (4) one item the task asked about — a "qualification video" deliverable — that **could not be found in any source** and should not be assumed to exist.
>
> ⏰ **Time-sensitive:** as of today (2026‑09‑21), the early-bird registration window closes in **about one week** (Sept 28, 2026). See Registration below.

---

### 1. Organizers & Contacts

| Role | Name | Contact |
|---|---|---|
| IAR Competition Chair | Diego Salazar D'Antonio (Oakland University) | dantonio@oakland.edu |
| IAR Co‑organizer | David Saldaña (Lehigh University, SwarmsLab/AIRLab) | saldana@lehigh.edu |
| General LARC/host contact | UMNG / IEEE Colombia Section | larslarc@unimilitar.edu.co · +57 601 6500000 |

*(All CONFIRMED — rulebook header page; https://sites.google.com/oakland.edu/iar-larc/home; https://mecatronica.umng.edu.co/index.html/larc.html)*

Notably, David Saldaña and Diego D'Antonio are **also the authors of the "MochiSwarm"/BlimpSwarm platform** the rulebook names as its own baseline design (see §7). This is not a coincidence to flag to the organizers — it means the professor's reference material (the mochi-swarm-sim repo, the Lehigh videos) already comes from the same lab that writes the rules.

---

### 2. Edition, Dates, Venue

- **Event**: IAR‑LARC — "International Airship Racing at LARC" — a **brand-new category**, first edition, running alongside the **26th LARC** (Latin American Robotics Competition) / **23rd LARS** (Latin American Robotics Symposium). ✅ CONFIRMED: the IAR site itself states *"brings together teams from Colombia, the United States, Mexico, and Ecuador in its first edition."* (https://sites.google.com/oakland.edu/iar-larc/home) — this resolves what was originally an open question; **IAR did not exist before 2026**.
- **Host**: Universidad Militar Nueva Granada (UMNG), Bogotá, Colombia, with IEEE Colombia Section.
- **Venue**: Aula Máxima, UMNG, Carrera 11 #101‑80, Bogotá D.C. — capacity 450, ~45 min from El Dorado Int'l Airport (BOG), SITP "Calle 100" transit stop ~2 min walk, 50+ parking spaces, on-site WiFi + catering. ✅ CONFIRMED (https://mecatronica.umng.edu.co/index.html/venue.html)
- **Dates** — ⚠️ **UNRESOLVED CONTRADICTION across the organizers' own site**:
  - IAR-specific page: **Nov 2–6, 2026** (https://sites.google.com/oakland.edu/iar-larc/home)
  - UMNG's own general contact/index page: **Nov 4–6, 2026** (https://mecatronica.umng.edu.co/index.html/index.html)
  - This was checked independently twice and the discrepancy is real, not a scraping artifact. Working assumption until clarified: **Nov 2–3 may be IAR-specific check‑in/inspection/practice days**, with the joint LARC/LARS program starting Nov 4, finals on Nov 6 — but this is a guess, not confirmed. **Ask the organizers before booking travel.**

---

### 3. The Blimp Category: Game & Scoring

**⚠️ Important — do not confuse this with other blimp competitions.** Several comparable events exist in the wider literature (e.g., "Defend the Republic," GMU/Indiana/Lehigh — a hoop-scoring, opponent-vs-opponent game). **IAR‑LARC is a different, much simpler format: solo, fully autonomous waypoint racing against the clock, one team on the course at a time.** None of DTR's numbers (50 cu ft helium, 100 g negative buoyancy, hoops, halves/halftimes) apply here — do not reuse them.

**Confirmed objective (verbatim from Rulebook §1–2):**
> *"Teams design, build, and deploy an autonomous robotic airship capable of navigating a predefined circuit composed of multiple waypoints... Each waypoint is represented by a small red balloon tethered to the ground, which the vehicle must physically touch... Teams are allowed up to three attempts (loops)... The fastest successful loop wins."*

| Element | Confirmed detail | Source |
|---|---|---|
| Courses | Two: **Course 1** = rectangle perimeter path; **Course 2** = figure‑eight ("infinity") path, same footprint | Rules 18–20 |
| Arena | **10 m × 5 m** overall course area | Fig. 3–4, rulebook p.5 |
| Waypoints | 4 per course, arranged in a **4 m × 2 m rectangle** inside the arena | Fig. 3–4 |
| Start region | 2 m‑diameter circle, centered on waypoint 1 | Fig. 3–4 |
| Waypoint object | Red **mylar** balloon, commercially available **22‑inch** size, inflates to ~50–60 cm diameter, helium‑filled, ground‑tethered | Rule 21 |
| Waypoint height | **2.0 m** above ground, measured to balloon **center** | Rule 22 |
| Visit order | **Not fixed** by the printed diagrams — *"will be specified prior to each run"* (Rule 2) → firmware must accept a runtime‑configurable sequence, not a hard‑coded one | Rule 2 |
| Attempts | Up to **3 loops per course**; best (fastest) *valid* time on each course is kept | Rule 5 |
| Scoring | **Total time = sum of the best time on Course 1 + best time on Course 2. Lowest total wins.** | Rule 6 |
| Invalid‑run conditions (no time recorded) | Wrong waypoint order; any human intervention; leaving the competition area; safety violation | Rule 7 |
| Practice time | Each team gets **dedicated, exclusive** practice time on the arena (no other teams' vehicles present) | Rule 23 |

**Implication for strategy** (not a rule, an inference): because an invalid run scores nothing at all, and only the *best* of 3 loops counts, a slower-but-reliable run beats a fast-but-flaky one. Design for robustness over raw speed.

---

### 4. Hard Constraints

| Constraint | Value | Status | Source |
|---|---|---|---|
| Helium / buoyant volume | ≤ volume of a **50‑inch diameter balloon (~1 m³)** | ✅ CONFIRMED | Rule 10 |
| Neutral buoyancy | Net vertical force (lift − weight) within **±10 grams‑force** | ✅ CONFIRMED | Rule 11 |
| Total propulsion thrust | **≤ 100 gf combined**, across ALL motors; **servos excluded** from this budget (worked examples: 1 motor→100gf, 2→50gf ea, 5→20gf ea) | ✅ CONFIRMED | Rule 13 |
| Vehicle size / bounding box | **⚠️ SELF‑CONTRADICTORY IN THE RULEBOOK ITSELF:** Rule 12 states *"There are no explicit size limitations on the airship"*; Rule 14, one clause later, states the *"maximum allowable dimensions... are **1 m × 1 m × 1.5 m**, measured as the smallest axis‑aligned bounding box."* | ⚠️ CONTRADICTION — treat **1×1×1.5 m** as the binding number for design safety, but get written confirmation | Rules 12 & 14 |
| Autonomy | Fully autonomous once the run starts — **no manual control or remote piloting** | ✅ CONFIRMED | Rule 1 |
| Off‑board compute | **Explicitly permitted**: *"Onboard or offboard computation is permitted, but no human intervention is allowed during execution."* | ✅ CONFIRMED — a ground-station laptop may run the heavy vision/planning code | Rule 1 |
| Onboard sensing | **Required**: *"All sensing used for navigation and task execution must be performed onboard the robot."* | ✅ CONFIRMED — this is the clause that rules out external motion‑capture/tracking cameras as the localization source (but does NOT forbid streaming onboard-sensor data to a laptop for processing) | Rule 15 |
| Safety — balloon contact | No sharp/pointed/rigid protrusions intended to puncture or damage objects; must not sever, damage, or intentionally lift the waypoint's tether anchor | ✅ CONFIRMED | Rules 16–17 |

**Reading the autonomy rule correctly matters:** Rule 1 (offboard compute OK) and Rule 15 (sensing must be onboard) work together — you *can* stream telemetry/video from onboard sensors to a ground laptop for heavy processing (vision, path planning) and send commands back autonomously, as long as no human touches the controls and the *sensors themselves* are on the vehicle. A ceiling-mounted tracking/mocap rig used as the state‑estimation source would violate Rule 15.

---

### 5. Registration Deadlines & Deliverables

⏰ **The early‑bird window closes in ~1 week from today (2026‑09‑21).**

| Milestone | Date | Status |
|---|---|---|
| Pre‑register | **Sept 14, 2026** | ✅ CONFIRMED — **already passed** as of today; ask organizers whether late/rolling pre‑registration is still accepted |
| Early‑bird registration | Through **Sept 28, 2026** | ✅ CONFIRMED — closes soon, prioritize |
| Regular registration | **Sept 29 – Oct 5, 2026** | ✅ CONFIRMED |
| Team Description Paper (TDP) due | **Oct 12, 2026** | ✅ CONFIRMED |
| "Qualification video" | — | ❌ **NOT FOUND in any source consulted.** Only the TDP deadline above was located. Do not assume a video-submission requirement exists until an organizer confirms it — it may live inside a general/overall LARC rulebook that was never located (see §6). |

**Fees (Colombian Pesos, COP)** — ✅ CONFIRMED, https://mecatronica.umng.edu.co/index.html/larc.html:

| Registration window | Team (1 robot + 1 student) | Additional member (IEEE / non‑IEEE) | Mentor (IEEE / non‑IEEE) |
|---|---|---|---|
| Early‑bird (until Sept 28) | 100,000 COP | 50,000 / 70,000 COP | 90,000 / 120,000 COP |
| Regular (Sept 29–Oct 5) | 140,000 COP | 70,000 / 90,000 COP | 110,000 / 140,000 COP |

Registration portal: https://www.asistiendo.com/prod/0020101/0403U2R8V1/reg/login.asp

---

### 6. Logistics

| Item | Status | Detail |
|---|---|---|
| Event schedule | ✅ CONFIRMED (day-level only) | Day 1 (Nov 2): arrival/check‑in, team registration + safety inspection, practice sessions, welcome briefing. Days (Nov 3–5): opening, qualification matches & technical evaluations, poster/project presentations, networking. Final day (Nov 6): finals, judging, awards, closing. **Exact hour-by-hour times are marked TBD by the organizers themselves.** |
| Helium supply | ❌ **UNDOCUMENTED ANYWHERE** — checked across the IAR home/rules/schedule/venue pages and the UMNG venue page; zero mentions of on-site helium provisioning found in two independent research passes. Must be asked directly. | — |
| Altitude effect (Bogotá, ~2,640 m) | ⚠️ Physics inference, NOT a rulebook fact | Lower air density at altitude reduces buoyant lift per liter of helium by roughly a quarter to a third versus sea level (depending on the specific worked calculation used). Plan to **re‑ballast/re‑trim on site** rather than assuming a sea-level buoyancy test transfers directly to Bogotá. |
| Communications / community channel | ⚠️ Rulebook's own Slack invite link is a **non-functional placeholder** (`https://join.slack.com/`, no workspace token) | Get the real invite link from the organizers directly. |
| Baseline reference design | ✅ CONFIRMED | Rulebook Appendix A/B **explicitly names "Mochiswarm"/BlimpSwarm** (Lehigh, arXiv:2503.03077) as the official baseline design, and **names `github.com/TongshuWu1/mochi-swarm-sim` as the official simulator** — i.e., the exact repo the professor already has is the competition's own recommended reference implementation, not incidental inspiration. The "Blimp sample" page on the IAR site also embeds Lehigh Swarms Club build videos (one of which is the professor's video #3). |
| RF / WiFi coexistence rules | ❌ Not found | No documented channel/frequency allocation for multiple teams' simultaneous WiFi/ESP‑NOW links was located anywhere in the rulebook or site. Worth asking, since several teams' airships may operate in the hall concurrently. |
| General/overall LARC rulebook | ❌ Not located | Only the IAR **category-specific** rules PDF was found. A general LARC rulebook (covering conduct, IP, appeals, visas, safety certification, and possibly a video-deliverable requirement) may exist separately and was never located. |

---

### 7. Confirmed vs. Assumed — Summary

**✅ CONFIRMED (verbatim from the primary rulebook / organizer pages, sources given above):**
Objective and scoring format; 2‑course structure and arena/waypoint geometry; helium volume cap; buoyancy tolerance; thrust cap; autonomy + offboard‑compute + onboard‑sensing rules; balloon‑safety rules; registration fee schedule and TDP deadline; organizer identities/contacts; venue address and access details; official baseline design/simulator naming; IAR is a first-time (2026) category.

**⚠️ ASSUMED / UNCONFIRMED — resolve before locking in design or travel:**
1. Exact event date range (Nov 2–6 vs. Nov 4–6 — contradiction on the organizer's own site).
2. Which governs vehicle size: Rule 12 ("no limit") or Rule 14 (1×1×1.5 m) — rulebook self-contradicts.
3. Whether the Sept 14 pre‑registration deadline can still be met late.
4. Whether helium is supplied on-site or must be sourced/imported by teams.
5. Whether a "qualification video" deliverable exists at all (not found; may live in an unlocated general LARC rulebook).
6. Exact hour-by-hour schedule (currently day-level/TBD only).
7. Any RF/WiFi channel restrictions for concurrent team operation.
8. Real Slack/community invite link.

---

### 8. Design Decisions Hostage to These Unknowns

- **Envelope/gondola sizing** cannot be finalized with full confidence until the Rule 12/14 size contradiction is resolved (design to 1×1×1.5 m as the safe assumption, but confirm in writing).
- **Helium logistics for the trip** (bring pressurized cylinders vs. buy locally vs. rely on the venue) depends entirely on the undocumented on-site-helium question — this affects both budget and whether the balloon is shipped inflated, empty, or purchased in Bogotá.
- **Travel/hotel booking** is blocked on resolving the Nov 2 vs. Nov 4 start-date discrepancy.
- **Whether to prepare any video deliverable** (script, editing time, recording equipment) is blocked on confirming whether a "qualification video" requirement actually exists.
- **Communications architecture** (whether to add a dedicated non‑WiFi radio, e.g., to dodge crowded‑hall 2.4 GHz congestion) is a lower-confidence decision without knowing if the venue enforces any RF/channel allocation among teams.

---

### 9. Questions to Email the Organizers

Send to **dantonio@oakland.edu**, cc **saldana@lehigh.edu** and **larslarc@unimilitar.edu.co**:

1. Can you confirm the actual event date range — is IAR Nov 2–6, 2026, while the general LARC/LARS program runs Nov 4–6 (i.e., is Nov 2–3 IAR-specific check‑in/inspection/practice)? Your own site currently shows both Nov 2–6 and Nov 4–6 on different pages.
2. Rule 12 states there are "no explicit size limitations," while Rule 14 gives a 1 m × 1 m × 1.5 m maximum bounding box. Which one governs? Please confirm the binding maximum dimension in writing.
3. The pre‑registration deadline (Sept 14, 2026) has passed. Can our team still pre‑register, and does missing it affect eligibility for the early‑bird fee (through Sept 28)?
4. Is helium supplied on-site by the organizers, or must each team source/purchase/import helium locally in Bogotá? If supplied, what purity/quantity per team?
5. Is there a required "qualification video" submission (separate from the Team Description Paper due Oct 12), and if so, what is its format, length, and deadline?
6. What is the required format/length/template for the Team Description Paper (TDP) — is it IAR-specific or a shared LARC-wide template?
7. What tolerance will judges apply on competition day to the 2 m waypoint‑height spec and the ±10 gf neutral‑buoyancy check (the rulebook mentions accounting for "environmental conditions" without giving a number)?
8. What exactly counts as disqualifying "human intervention" between the 3 allowed loops on a course — e.g., is a manual battery swap or vehicle reset between loops permitted, or must all 3 loops run back‑to‑back untouched?
9. Are there any RF/WiFi frequency‑band or channel allocation rules for simultaneous team operation, given multiple airships may be flying/telemetering in the same hall at once?
10. Does a second onboard microcontroller/coprocessor dedicated purely to vision (e.g., a companion vision board) count as compliant "onboard sensing," or are there restrictions on the number/type of onboard sensing devices?
11. Can you share the exact hour‑by‑hour schedule for Nov 2–6 once available (currently published only as day-level blocks marked TBD)?
12. Can you share the real Slack (or other community/chat) invite link? The one embedded in the current rulebook PDF (`https://join.slack.com/`) is a non‑functional placeholder.
13. Is there a general/overall LARC rulebook (covering registration logistics, conduct, IP, appeals, safety certification) separate from the IAR category-specific rules PDF, and if so, can you share it?
14. Since this is the first year for IAR, are there any past test runs, practice videos, or lessons-learned notes from your own team's development (beyond the "Mochiswarm"/BlimpSwarm baseline) that you can share with participating teams?


---

## Flagged uncertainties

- The Nov 2-6 vs Nov 4-6 event date contradiction across the organizer's own site pages was independently reproduced twice in the source research but never resolved — I present it as an open contradiction rather than picking one date.
- No source in the corpus mentions a required 'qualification video' deliverable; I explicitly flagged this as not found rather than inventing content to satisfy the task's phrasing, since the task itself only asked me to include it 'if established.'
- Whether the Sept 14, 2026 pre-registration deadline having passed actually blocks new registrations, or is just a soft/rolling suggested date, is unconfirmed — I flagged it as needing direct organizer confirmation rather than assuming either way.
- The Rule 12 vs Rule 14 size contradiction is presented as unresolved per the source material; I recommended treating 1x1x1.5m as the safe design target but this is my own risk-management suggestion, not an organizer ruling.
- I treated https://mecatronica.umng.edu.co/index.html/rules/IAR_LARC2026.pdf as the primary rulebook citation since multiple independent verification passes in the corpus confirmed it downloads directly via plain HTTP; I did not re-fetch it myself in this task since I was instructed to draft from the supplied corpus only.