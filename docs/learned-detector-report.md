# target_cam learned detector — status report

**Date:** 2026-10-15
**Project:** `firmware/projects/target_cam` (branch `target-cam-learned-detector`)
**Hardware:** AI-Thinker ESP32-CAM (OV2640, 160×120 RGB565) on an ESP32-CAM-MB
programmer, ESP-IDF v5.5.5. The task statement is
[`claude_code_prompt_learned detector.md`](claude_code_prompt_learned%20detector.md).

This report covers Phase 0 (groundwork) and Phase A1 (labelled frames and the
stage 0 baseline). Later phases append to it. Each section keeps **measured**
(numbers from the board or from `eval_frames.py` on labelled frames) apart from
**reasoned** (what the numbers suggest, not yet tested). §7 lists what is not
established.

**Current status in one line:** pacing is locked to the sensor at 80.0 ms on
every row (measured, 3752 rows). On 20 hand-labelled red-balloon frames, a
well-chosen HSV calibration already finds every balloon, but **which
calibration you choose swings false blobs from 0 to 38 on the same frames**,
and at pixel level the best setting matches only about half of each balloon.
That calibration sensitivity, and the pixel figures, are what the learned
colour table (Phase A2) has to beat.

---

## 1. The detector being improved

Stage 0 is the existing HSV detector (`main/blob_detect.c`). A pixel matches if
its hue is within `hue ± tol` and its saturation and value are above `sat` and
`val`. The matching pixels are grouped into 8-connected blobs, and the up to 4
largest blobs of at least `area` px are reported, largest first. The ESP32-S3
will steer to blob 1.

Stages 1–3 (a learned colour table, a blob classifier, a small int8 CNN) are
planned on top of it. Each is selected with `!stage N` and falls back to the
highest stage whose models are all valid.

## 2. Phase 0: pacing (measured, 2026-10-14)

The camera's loop is now paced by the sensor's own frame clock (`sensor_us` =
79 968 µs), not by a software timer.

| Recording | Rows | dt ≠ 80 | Rate | Lost | Notes |
|---|---|---|---|---|---|
| `run_20261014_232303.csv`, 300 s, stage 0, a `k` command every 5 s | 3752 | **0** | 12.505 rows/s | 0 | `proc_us` p50 7.7 ms (pixel pass 6.8 ms). `loop_us` p50 8.5 ms, max 34.9 ms. The max is only on rows where the ~400 B `#!CFG` reply was printed, which leaves 45 ms of margin. |
| `run_20261014_232812.csv`, `!stage 3` with no models | 251 | **0** | — | 0 | The camera fell back to `stage_eff=0` on every row, as designed. |

At 115200 baud a CSV v2 row takes 11.5–12.2 ms on the wire. That is a
separate budget from `loop_us`, because the UART's FIFO drains after the print
returns.

## 3. The labelled set (Phase A1)

### 3.1 Capture

All frames were taken on 2026-10-15 in one room, with AWB frozen (`w0`) and the
camera's raw 160×120 RGB565 bytes saved alongside their `.json` settings. The
balloon is a red foil balloon: an oval body with a neck, about 1.3–1.5 times
taller than wide in the image.

| Session | Time | Frames | Calibration | Balloon range |
|---|---|---|---|---|
| `red_s1` | 21:50–22:03 | 7, each with the balloon | **Recalibrated before almost every frame** (hue 0–21, tol 24–38, sat 79–127, val 20–53) | 0.9–2.9 m, 343–3026 px |
| `red_s2` | 22:19–22:32 | 7 with the balloon + **6 clutter-only** | **One setting**: the factory `red` preset (hue 0 ± 12, sat ≥ 110, val ≥ 50) | 1.25–5.2 m, 126–1830 px |

At 4.2 and 5.2 m the balloon is 10 px wide (145 and 126 px of balloon).

The two sessions are in the same room, 15–30 minutes apart, so they are
**not** independent venues (§7).

The 20 frames and their labels are committed under
`firmware/projects/target_cam/host/testdata/red_s{1,2}/`. Five earlier frames
of a green balloon (AWB on) in `target_cam/data/` were used only to try out the
tools.

### 3.2 Labels

Each frame has a `<stem>.labels.png`, painted by hand in tcview's Label tab:
0 unlabelled, 1 balloon, 2 background. "Rest -> background" leaves a 2 px band
around the balloon unlabelled. Rim pixels mix balloon and wall colour, so they
are kept out of both training and scoring.

A balloon, for scoring, is an 8-connected component of label 1 of at least
4 px, with its bounding box. Specular highlights and the dark reflections on
the foil were labelled as balloon: deciding which balloon colours a colour
table can afford to match is the training's job, not the label's.

### 3.3 Scoring rule (`host/tools/eval_frames.py`, shared with the Label tab through `tcview/labels.py`)

- **TP:** a labelled balloon with a reported blob's centroid inside its box.
- **FN:** a balloon no reported blob landed in.
- **FP:** a reported blob in no balloon's box.
- **split:** a second blob in an already-found balloon. It is not a false
  detection, but it costs a report slot.
- **blob 1:** is blob 1 a balloon? On a clutter-only frame, any blob 1 is a
  false target.
- **Pixel precision / recall:** the matched mask against balloon and
  background labels, counting labelled pixels only.

Centroids use the camera's convention (pixel *i* spans [i, i+1)). The first
version of `labels.py` used pixel indices, a 0.5 px offset that would have
scored a centroid in a box's last column as a false detection. The host test
`test_component_centroid_matches_the_c_detector` now guards this.

Each frame is run through the camera's own C pipeline (the shared library),
so these are the numbers the board would produce for the same bytes.

## 4. Stage 0 baseline (measured)

### 4.1 Session 2, its own single calibration (the `red` preset)

| | TP | FN | FP | split | blob 1 is the balloon | false blob 1 on clutter-only | pixel P | pixel R |
|---|---|---|---|---|---|---|---|---|
| red_s2 | 5 | **2** | 0 | 1 | 5 / 7 | **0 / 6** | 75.7 % | **37.3 %** |

The two misses are the 4.2 m and 5.2 m frames. The preset matches only a few
of the balloon's dark-red pixels, fewer than the 25 px area floor. The tight
window never produces a false blob, but it doesn't see the balloon at range.

### 4.2 One calibration across all frames: how much the choice matters

Each of session 1's seven calibrations was applied unchanged to the frames
(`eval_frames.py --cfg`):

| Calibration (from red_s1 frame) | on red_s1: FP / FN | on red_s2: FP / FN | red_s2 false blob 1 (of 6) | red_s2 pixel P / R |
|---|---|---|---|---|
| `90cm_wood_door_2` (hue 0 ± 24, sat 110, val 35) | 0 / 0 | **1 / 0** | **0** | 48.3 / 60.8 % |
| `170cm_wall_shelve` | 1 / 0 | 5 / 0 | 2 | 45.2 / 59.2 % |
| `270cm_door` | 2 / 0 | 6 / 1 | 2 | 32.3 / 59.5 % |
| `290cm_doors` | 6 / 0 | 9 / 0 | 3 | 28.5 / 70.4 % |
| `90cm_wood_door` | 8 / 0 | 18 / 0 | 2 | 29.1 / 66.4 % |
| `110cm_black_shelve` | 8 / 0 | 20 / 0 | 3 | 24.2 / 72.5 % |
| `130cm_wood` (hue 21 ± 38) | 13 / 1 | **38 / 2** | **6** | 10.3 / 61.8 % |

The red_s1 frames scored with their own per-frame settings gave TP 7, FN 0,
FP 3, pixel P 69.6 % and R 72.1 %. That figure is optimistic and is not used as
the baseline: in flight there is one setting.

### 4.3 The baseline A2 is judged against

The fair comparison for a learned table is held-out, so the HSV baseline is
held-out too: a setting chosen on one session, scored on the other.

| Direction | Setting | TP | FN | FP | split | false blob 1 | pixel P | pixel R |
|---|---|---|---|---|---|---|---|---|
| s1 → s2 | best on s1 (`90cm_wood_door_2`) | 7 | 0 | 1 | 0 | 0 / 6 | 48.3 % | 60.8 % |
| s2 → s1 | s2's preset | 6 | 1 | 0 | 2 | — | 92.4 % | 44.9 % |

- The s1 → s2 FP is in `165cm_shelve`. The s2 → s1 FN is `290cm_doors`, the
  smallest balloon in s1.
- At blob level, a well-chosen HSV window is already close to perfect on these
  20 frames. The weakness is the pixel level, and how fragile the choice of
  setting is.

## 5. Where stage 0 goes wrong (measured on the frames, causes reasoned)

**False pixels.** These are mostly dark brown wood: louvred doors and
furniture, with median hue 23–51 and V 41–90.
- A window that reaches past hue ~20, as `130cm_wood` (hue 21 ± 38) does,
  matches whole door panels. It produced a false blob 1 on 6 of 6 clutter-only
  frames.
- Tighter windows leave isolated speckle, which rarely reaches the 25 px blob
  floor.
- In session 1, only 19–55 % of the false pixels sit in the top quarter of the
  background's edge strength, so they are mostly not edge fringes. On the green
  frames, 55–84 % did.
- Whether the speckle is sensor noise in the dark has not been measured.

**Missed pixels.** These are dark blue-purple reflections on the foil (median
hue 199–321, often V < 50), the shadowed side of the balloon, and white
highlights (S < 50, V ≈ 255). The balloon mirrors a dim room, so much of its
surface isn't red. In the backlit `130cm_window` frame, the preset matched only
15 % of the balloon, essentially the top highlight rim.

**What a learned colour table can and cannot fix (reasoned):**
- **It can** use the fact that, in these frames, the balloon's red sits around
  hue 330–360 with S ≥ 100, wood around hue 20–50, and the reflections are dark
  purple. A table can include purple and magenta and exclude orange-brown. A
  single convex HSV box can't do both.
- **It cannot** separate colours that really overlap: dark red balloon against
  dark wood, or a white highlight against a white wall. Excluding these costs
  balloon pixels, and including them costs false pixels. That is stage B's
  job, using shape, fill and edges.
- **The real gain to expect** is pixel recall and stability across settings
  more than blob counts, which are already near the ceiling at these ranges.
  The far frames (≥ 4 m) are where pixel recall decides TP versus FN.

## 6. How Phase A2 will be evaluated

- **Two-fold by session.** Train the table on red_s1 and score it on red_s2,
  then the reverse. Compare each fold with the §4.3 row for the same
  direction. Consecutive frames are near-duplicates, so splitting by frame
  would leak the answer.
- **Report all of it.** Blob TP/FN/FP/split, false blob 1 on the 6 clutter-only
  frames, and pixel P/R, so a gain in one figure can't hide a loss in another.
- **Check it on the board.** `csv_report.py` at stage 1 must still show
  dt = 80.0 on every row. Also report `t_px_us` at stage 1 against stage 0's
  6.8 ms.

## 7. Not established

- **Generalisation to another venue.** Both sessions are one room on one
  evening, with one balloon. A held-out session here measures robustness to
  position, distance and calibration, not to a new room or light. The plan
  expects retraining at the flight venue.
- **The flight venue's lighting.** These frames are dim (balloon V mostly
  50–110). A brighter venue will change both the baseline and the trained
  table.
- **Live-scene behaviour.** The S1/S2 CSV scenes (60 s each, balloon + clutter
  and clutter only) planned for A1 were not recorded, so jumps per minute in a
  moving scene are not measured. Frames are single snapshots.
- **Statistical weight.** 14 balloon instances and 6 clutter-only frames. One
  frame changes recall by 7 points. Differences of one or two blobs between
  stages are within what a different set of 20 frames could produce.
- **The cause of the dark-wood speckle** (sensor noise is suspected, not
  measured).
- **Range beyond 5.2 m** is untested.
