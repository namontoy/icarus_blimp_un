# target_cam learned detector — status report

**Date:** 2026-10-15, updated 2026-10-16
**Project:** `firmware/projects/target_cam` (branch `target-cam-learned-detector`)
**Hardware:** AI-Thinker ESP32-CAM (OV2640, 160×120 RGB565) on an ESP32-CAM-MB
programmer, ESP-IDF v5.5.5. The task statement is
[`claude_code_prompt_learned detector.md`](claude_code_prompt_learned%20detector.md).

This report covers Phase 0 (groundwork), Phase A1 (labelled frames and the
stage 0 baseline), Phase A2 (the learned colour table, stage 1, §8) and the
plan for the next frames (§9). Later phases append to it. Each section keeps
**measured** (numbers from the board or from `eval_frames.py` on labelled
frames) apart from **reasoned** (what the numbers suggest, not yet tested).
§7 lists what is not established.

**Current status in one line:** pacing is locked to the sensor at 80.0 ms on
every row (measured, 3752 rows). On 20 hand-labelled red-balloon frames, a
well-chosen HSV calibration already finds every balloon, but **which
calibration you choose swings false blobs from 0 to 38 on the same frames**,
and at pixel level the best setting matches only about half of each balloon.
That calibration sensitivity, and the pixel figures, are what the learned
colour table (Phase A2) has to beat.

**Phase A2 in one line:** held out by session, the learned table finds all 14
balloons (HSV: 13) with blob 1 the balloon every time, and raises pixel F1
from 54 to 66 % (s1 → s2) and from 60 to 75 % (s2 → s1). It pays one extra
false second blob over the two folds. On the board (§8.5) it keeps 80.0 ms on
every row, and on one still scene it cuts the pixel pass from 8.5 to 4.7 ms
and the total detection time from 9.6 to 8.8 ms. On that scene it finds the
balloon in every row, where HSV finds it in 43 %.

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

---

## 8. Phase A2: the learned colour table (stage 1)

### 8.1 What it is

Stage 1 replaces the HSV test with a table: one bit for each of the 65 536
RGB565 values, 8 kB (`main/color_lut.{c,h}`). Everything after the pixel test
(cells, labelling, the area floor, the blob order) is stage 0's, so any change
in what is reported comes from which pixels match.

- **Why a table.** An RGB565 pixel has only 65 536 values, so the decision for
  each can be stored outright. The accepted set can then have any shape,
  where the HSV test is one box. On the board it is one load and one shift per
  pixel instead of the HSV arithmetic.
- **Where it lives.** It is compiled in as a model record
  (`main/models/lut_model.h`), checked at boot by `color_lut_load()` (magic,
  version, size, CRC-32) and copied to an 8 kB DRAM buffer. A refused record
  is reported in the boot log and `#!CFG` (`lut=bad`), and `!stage 1` then
  falls back to stage 0. A RAM copy keeps random lookups off the flash cache,
  and it is what Phase D's NVS upload will fill, through the same check.
- **One pixel test.** `blob_pixel_matches()` reads the table when
  `blob_cfg_t.lut` is set. The detector's passes, `blob_mask()`, the camera's
  `m` dump and the viewer's overlay all go through it.

### 8.2 Training (`host/train/train_lut.py`)

1. **Count** balloon and background labels per RGB565 value, per session.
   Unlabelled rim pixels count for neither.
2. **Smooth** both counts with a 3-D Gaussian on the 32 × 64 × 32 grid. The
   20 frames show about 2 500 distinct balloon colours, and the next frame
   will show their neighbours. sigma is in 8-bit units on every axis (a green
   step is 4.0 units, a red or blue step 8.2). The kernel's peak is 1, so a
   smoothed count reads as "pixels' worth of evidence at this colour".
3. **Decide.** A colour is in when it is k times likelier among balloon pixels
   than among background (each count over its own total) and has at least
   `min_evidence` balloon evidence. A colour with no evidence stays out.

**Choosing sigma, k and min_evidence** is by leave-one-session-out over 168
settings (sigma 0–16, k 2–30, min_evidence 0.5–2). The objective is the
held-out pixel F1, averaged over the two folds. A setting that puts a false
blob 1 on any held-out clutter-only frame is excluded.

What was tried first (measured): ranking by blob F0.5, with a sum-normalised
kernel. It picked sigma 12, k 3, whose held-out pixel recall (39 % and 43 %)
was **below** the HSV baseline. On 14 balloons one frame moves blob recall by
7 points, so blob counts are too coarse to choose between settings. They follow
from the pixels, and the pixel counts (17 753 balloon, 366 247 background)
are not too coarse. With the absolute evidence floor, the sum-normalised
kernel also made `min_evidence` mean something different for every sigma.

**Result:** sigma 8, k 7, min_evidence 0.5. The neighbourhood is flat: sigma
6–8 with k 5–7 are all within 0.006 pixel F1 of it, so the choice is not on a
knife edge. The final table is trained on both sessions: 7 321 colours
accepted, train id 1792123782. `main/models/lut_model.json` records the
parameters, the frames and the search, and a host test regenerates the
committed bytes from it.

### 8.3 Held-out scores (measured, `train_lut.py`)

Each table is trained on one session and scored on the other. The HSV rows are
§4.3.

| Direction | Detector | TP | FN | FP | split | false blob 1 | px P | px R | px F1 |
|---|---|---|---|---|---|---|---|---|---|
| s1 → s2 | HSV, best setting on s1 | 7 | 0 | 1 | 0 | 0 / 6 | 48.3 % | 60.8 % | 53.8 % |
| s1 → s2 | **table trained on s1** | 7 | 0 | 1 | 0 | 0 / 6 | **73.8 %** | 60.0 % | **66.2 %** |
| s2 → s1 | HSV, s2's preset | 6 | 1 | 0 | 2 | — | 92.4 % | 44.9 % | 60.4 % |
| s2 → s1 | **table trained on s2** | **7** | **0** | 1 | **0** | — | 91.9 % | **63.4 %** | **75.0 %** |

- **The gain is in the pixels, as §5 expected.** Precision went up where HSV was
  loose (s1 → s2) and recall where it was tight (s2 → s1), with no loss on the
  other figure beyond 0.8 points.
- **The far and backlit frames**, with the table trained on s1 only:

  | Frame | HSV preset | Table |
  |---|---|---|
  | `420cm_2do_door` | missed, recall 26 % | found, recall 43 % |
  | `520cm_2do_room` | missed, recall 12 % | found, recall 22 % |
  | `130cm_window` | recall 15 % | recall 28 % |

- **The cost:** one more false blob over both folds (2 against 1). Both are
  second blobs, in `130cm_wood` (s1) and `130cm_window` (s2). Blob 1 is the
  balloon in all 14 balloon frames, and in no clutter-only frame is there a
  false blob 1.
- **Caveat:** sigma, k and min_evidence were chosen on these same two folds,
  so the table rows are slightly optimistic. A session kept out of all of
  this, such as the students' `red_s3`, is the unbiased test.

On its own training frames the final table scores TP 14, FN 0, FP 2, pixel
P 88.1 %, R 64.9 % (`eval_frames.py --stage 1 host/testdata`). This is
training data, so it shows the table loaded and works, not how good it is.

### 8.4 What the table learned (measured on the 20 frames)

The final table is compared with s2's HSV settings (hue 0 ± 12, sat ≥ 110,
val ≥ 50; 3 875 colours) over all labelled pixels:

| Pixel group (HSV) | Balloon px | Table accepts | HSV accepts | Background px | Table accepts | HSV accepts |
|---|---|---|---|---|---|---|
| red, hue 330–20, S ≥ 100 | 12 442 | 84.0 % | 59.9 % | 12 673 | 8.4 % | 9.7 % |
| purple/blue, hue 180–330 | 1 823 | 15.4 % | 0 % | 71 499 | 0.1 % | 0 % |
| highlights, S < 50 | 1 050 | 34.4 % | 0 % | 87 888 | 0.3 % | 0 % |
| wood hue 20–60, S ≥ 50 | 1 287 | 15.2 % | 0 % | 71 717 | 0.1 % | 0 % |

- **Most of the gain is inside red.** The table takes 84 % of the balloon's
  saturated red against 60 %, while accepting less of the background's red.
  No single hue window with two floors can match that.
- **It reaches outside the box only a little.** It takes 15 % of the purple
  reflections and 34 % of the highlights, at a 0.1–0.3 % background cost.
- **Not fixed: red on the background.** 8.4 % of saturated-red background
  pixels still match. These are colours the balloon and the room really share
  (§5), which is stage B's job.

### 8.5 On the board (measured, 2026-10-15)

The firmware is `bc92ee1-dirty` (the A2 working tree). The camera was still,
pointed at the red balloon with clutter in view (the camera's own range
estimate: 1.57 m median).
Two `csv_report.py` recordings were taken without moving anything: stage 1
for 120 s (`data/run_20261015_233153.csv`), then stage 0 for 60 s
(`data/run_20261015_233703.csv`).

- **The table loaded and matches the PC's.** `#!CFG` reported `lut=ok
  lut_id=1792123782`. With `g` at stage 1, the camera's own row and the PC
  library's result for the same frame agreed exactly: 2 blobs, blob 1 area
  1099 px, 1501 matched pixels. Neither the compiled-in table nor the byte
  order differs between the camera and the PC.

| Same scene | Stage 0 (HSV) | Stage 1 (table) |
|---|---|---|
| rows, dt ≠ 80 ms | 752, **0** | 1502, **0** |
| pixel pass `t_px_us` p50 / max | 8 541 / 8 598 µs | **4 698 / 4 698 µs** |
| labelling (`proc_us − t_px_us`) p50 | 1.0 ms | 4.1 ms |
| total `proc_us` p50 / max | 9 592 / 10 864 µs | **8 798 / 9 159 µs** |
| worst `loop_us` (first row excluded) | 13.4 ms | 11.9 ms |
| matched pixels p50 | 338 | 1 499 |
| rows with a target | 42.7 % | **100 %** |
| blob 1 area when found, p50 | 37 px | 1 073 px |
| jumps (blob 1 moves > 10 px) | 82 | **0** |

- **The pixel pass is 45 % faster and its time does not vary.** At stage 1
  every pixel costs the same load and shift, so the time is 4 698 µs on every
  row. The HSV pass took 6.8 ms in the Phase 0 scene and 8.5 ms in this one,
  so its cost depends on the colours in view. The drop is smaller than on the
  PC (62 %). *Reasoned, not measured:* at about 59 CPU cycles per pixel, most
  of the per-pixel time goes to fetching the pixel and adding it to its cell,
  not to the colour test.
- **Labelling costs more because more pixels match.** Its time grows with the
  matched pixels, which went from 338 to 1 499. The pixel pass saves more
  than labelling adds, so the total still drops by 0.8 ms.
- **HSV saw this balloon only in pieces.** Its blob 1 had a median of 37 px,
  and it switched between pieces 82 times in a minute. The table's blob 1 is
  the whole balloon, in the same place on every row (cx ≈ 91 px). In the
  grabbed frame its box covers the balloon and runs 3–4 px past it on the
  right. That is one or two 2-px cells. *Reasoned:* the cause is either
  blended edge colours that the blur accepted, or the box rounding up to whole
  cells. The centroid is about 0.3° off at most.
- **A false second blob is present on every stage 1 row** (median az −3.5°,
  el +18.8°: the orange box in the grabbed frame). It is wood in the
  background (identified on the image by the user), the same kind of error
  as the held-out false second blob in `130cm_wood` (§8.3): a colour that the
  balloon and the wood share. The balloon stays blob 1, so the flight output is
  not affected, but it is the case stage B is meant to remove.
- **The 38.3 ms `loop_us` in the stage 0 recording is its first row.** That
  iteration handled `!stage 0` and printed `#!CFG`, which the report's margin
  line includes.

This is one scene, so it shows the direction of the effect and the timing,
not a miss rate. The S1/S2 scripted scenes (§6) have not been repeated at
stage 1.

### 8.6 Not established (A2)

- **Every §7 limit applies.** That includes one room and one evening, so the
  table is a model of this room's light and walls.
- **Exposure and white balance.** The table was trained with AWB frozen (`w0`).
  How far a different exposure moves the balloon's colours off the table has
  not been measured. Retraining at the venue is the plan (README,
  "Retraining the colour table").
- **The parameter search is small-sample.** A one-frame change in either
  session can reorder settings within the flat top.

---

## 9. Next collection: plan (2026-10-16)

Nothing in this section is measured yet. It sets out how the next frames
will be used, written before they are taken, so the use can't be fitted to
the results afterwards. The procedure for students is in the frame-collection
guide.

### 9.1 `red_s3`: an unbiased test first, training data second

- **Score before training.** The table's sigma, k and min_evidence were
  chosen on `red_s1` and `red_s2` (§8.3), so its scores on those sessions are
  slightly optimistic. Run `eval_frames.py --stage 1` on `red_s3` (and
  `--stage 0`, for the baseline) before `red_s3` is used for anything else.
  Those are the first scores of the table on frames that had no part in
  making it.
- **Then retrain on all three sessions,** with the leave-one-session-out
  search repeated over three folds.
- **The same session rules as before** (AWB frozen with `w0`, one
  calibration, no reset), because the table learns raw colours (§8.6).
- **What it should add:** wood, which gives the false second blob in
  `130cm_wood` and on the board (§8.5); frames at 4–8 m; backlit frames; and
  about a third of the frames with no balloon. A different room or light would
  be worth more than any of these (§7).

### 9.2 `orange_s1`: orange balloons as negatives

The competition rules mention only red balloons, so for this detector an
orange balloon is never a target. It is something the detector must not
report.

- **Its own folder, with the orange balloon painted as background.** The
  labels then say what the red detector should do. `eval_frames.py` counts a
  blob on the orange balloon as a false positive, and a false blob 1 on a
  frame with only the orange balloon as a false target. A frame with both
  balloons has the red one painted as balloon.
- **Stage B is where it should help** (reasoned). An orange balloon has the
  shape, fill and edges of the target, so a blob classifier can only reject
  it on colour. Without such examples, B could learn that "round and filled"
  means balloon.
- **The colour table does not train on it at first.** A red balloon's lit
  side drifts toward orange (`blob_detect.c`, the hue window comment), so
  orange-balloon pixels counted as background would also push some red
  highlight colours out of the table. `train_lut.py` reads only the folders
  it is given, so leaving `orange_s1` out needs no code. The plan is to
  measure first: score the current table on `orange_s1`. Then train with it
  only if the table accepts the orange balloon, and only if `red_s3`'s pixel
  recall does not drop.
- **The folder split is not a clean test** for B, because the same room and
  light as `red_s3` leak across it. It is a separate folder so that it can be
  left out of the table's training, not as an independent session.

### 9.3 Frames for Phase B1

Frames saved now are also Phase B1 data. The PC library runs the camera's
pipeline (shown equal on a grabbed frame in §8.5), so it can rebuild each
blob, its features and its 24 × 24 crop from a saved frame. The pixel labels
then label the blobs: a blob whose centre is inside a painted balloon is a
balloon, and any other is not. Saved frames also survive a retrained table,
because the blobs can be rebuilt from the pixels. Blobs collected live with
`!collect` cannot be. Their limit is volume: at about 3.3 s per frame they
give tens of blobs, against hundreds from `!collect`.
