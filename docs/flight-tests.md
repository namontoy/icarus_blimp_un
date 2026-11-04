# Flight tests: where they stand and how to run the next one

**Status, 2026-11-02 night.** The blimp climbs, holds 1 m (or 2 m) and its heading, goes 2 m
along the line and stops there (test 5), and always lands on its own. The square (test 7)
found that `core/` keeps learning the gyro's bias in the air and turns the heading with it (96°
off by the fourth leg); `--flight-bias-tau 600` holds that off until the `core/` fix. The
balloon flights (`target_test.py`) found the balloon from about 2.2 m, locked onto it,
approached, backed off when too close (flight 5) and landed every time. Open: the camera's
detection (about 2.2 m with the crumpled 34 cm test balloon), the nose lagging a seen balloon
by 11–20° and turns that creep forward (both the front pair's reverse), and the open-loop drift.

**The forward camera runs `optflow_cam` since the evening of 2026-11-02** (the optical-flow
tests, `docs/history/flow-s3-2026-11-02.md`): flash namontoy's `target_cam` back before any
balloon flight, or `hold follow` refuses (no rows). **Next:** the turn-on-the-spot calibration
of `front_reverse_ratio` (heading steps at 1 m, as test 2, at 0.74 and lower), then a trim
check; then the square again with `--flight-bias-tau 600`. The detection goes to namontoy (a
taut race-size balloon, the colours tuned in this room).

**2026-11-03: a second room with a two-balloon course** (below, "The course room"): balloon 1
2 m ahead of the start, balloon 2 2 m to its right, both 1.2 m up. `software/course_test.py`
flies it from the ground, balloons in order: find, follow, approach, touch, move away, turn to
the next, land. Not flown yet; runs C0–C5, C0 (motors off) first, because the touch needs its
numbers.

**2026-11-03 afternoon: two cameras, and the optical flow in every run.** The forward
AI-Thinker goes back to `target_cam` (UART1, GPIO6, as before); the downward camera is the
ESP32-S3-WROOM CAM with an OV3660 running `optflow_cam` (namontoy's `optflow-s3-ov3660`, merged
at `ac1ea94`): its IO14 → 470 Ω → the S3's GPIO18, the grounds joined, and **never GPIO17 to its
IO13**, the camera's PCLK. The S3 reads it from `i2c_tools` at `3fe7911` on. Every capture then
holds the flow beside the gyro and the ToF, and `software/flow_track.py` turns a run's into a
track over the floor: the first measurement of where the blimp went, against the team's report.
F0 (motors off, with C0) calibrates the camera in this room first. The two cameras have not yet
run on the S3 together, and the new board's current on the 5 V rail is not measured.

**2026-11-04: assisted flight from the ground station** (branch `assisted-control`,
`docs/assisted-control.md`). The Control page's Fly switch: core/'s loops hold the height and the
heading, the sticks move the setpoints, and Land brings the blimp down. Same `hold`, no flash, not
yet flown; its runs are A0–A5 there. It sets the bench parameters, the bias workaround and the
trim itself. Close any script first: one source drives the motors at a time.

This is a living page: after each run, update the status above and the run's line in the
order below. What each run measured, with captures and numbers, goes in the session record:
`docs/history/flights-2026-11-01.md` covers 2026-11-01 and 02; start a new dated file for a new
day.

## Before a session

- **The room:** A, B, C, D are the corners of a 2 m square taped on the floor. A to B runs along
  a tape line with strips at 1, 2, 3 and 4 m from A; C and D are on the far-wall side, the
  blimp's left when it faces B from A. The room's air pushes the blimp toward the far wall.
- **The blimp:** on the floor (never on the black launch stand: the ToF then misreads and the
  run refuses to start), the nose along the run's direction, still for a minute before the
  first run after a power-up (the estimator needs 60 s of stillness; the scripts wait for it).
  Firmware `i2c_tools` at `3fe7911` or later from `pid-tuning` (the downward camera's UART2);
  both cameras powered, and `python software/blimp_link.py console cam` showing the forward
  camera's rows and the optical flow's, about 12 a second each. `course_test.py` says the flow's
  rate at its start and warns when there is none (the run still flies, without a track).
- **The PC:** joined to the blimp's Wi-Fi (`ICARUS-<mac>`; the ground station's "Join blimp
  Wi-Fi" button does it) and close to the blimp: the link runs at −80 to −86 dBm under the
  metallized envelope. Then **close the ground station, or switch its motors off**: one source
  drives the motors at a time.
- **The battery:** `python software/blimp_link.py console battery`, and each capture's
  `bat_v`. The cells used so far read 3.67–3.73 V at rest when put in (the divider is not yet
  checked against a meter, so the absolute value may be off). While hovering, a good cell read
  3.3–3.4 V; the cell of T3 read 3.67 V at rest but **2.8–2.9 V hovering**: set aside a cell
  (or a connector) that sags like that until it has been measured. Swap below about 3.65 V at
  rest before a flight longer than a minute.

## Rules

- **Nothing moves before the team says so.** The team sets up and says "start" (a flight) or
  "check" (a motors-off camera check); only then does a script run. The people do every
  physical step; the operator (or Claude) runs the scripts and reads the captures.
- **Every run passes the bench parameter set**, `software/bench_params.txt`. A reboot or a
  battery swap restores `core/src/params.c`'s simulator placeholders, with which the blimp does
  not leave the floor. The values are the ones kept on 2026-10-31
  (`docs/history/envelope-2026-10-31.md`) plus `front_reverse_ratio` 0.74 (reverse on).
- **Every flight passes the battery's trim:** the rear motors lean forward, and without a trim
  the blimp climbs at 45° and drifts metres forward. The trim depends on the battery type:
  **−0.6 gf for the new type** used from test 5 on 2026-11-02 (the blimp 5.1 gf heavy with it),
  −1 gf for the earlier cells. A new type, or a big change in the blimp's weight, needs a
  trim check: climb, 20 s at 1 m, land (`--legs 0 --settle 20`); the drift halved for each
  0.2 gf less trim. Use one battery type for all the tests and the race.
- **Every flight ends in a landing** (the team, 2026-11-02, after the first balloon flight
  stopped its motors at 2.6 m and the blimp had to be caught): the scripts land when the
  balloon is not found, when the ToF reads too high or goes silent for 1 s, on Ctrl-C and on
  any error in the script. Two cases still stop the motors in the air: the spin and tilt kills
  (they mean the controller itself misbehaves) and a lost Wi-Fi link (`core/`'s watchdog after
  1 s; landing by itself there is a firmware and `core/` change still to make). Keep a spotter.
- **At most 2.3 m.** The ToF reads the floor to about 2.5 m in the flight room and loses it
  beyond; `target_test.py` refuses a higher `--alt`.
- **Every flight with turns passes `--flight-bias-tau 600`** (`target_test.py`'s default;
  `square_test.py` needs it given) until `core/` stops learning the gyro's bias while armed.
- Limits: the firmware's 40 % duty ceiling and 10 % deadband (its defaults since 2026-11-01);
  the scripts kill on yaw over 62 °/s, roll or pitch over 120 °/s, and land on a ToF over the
  height plus 0.3 m (`square_test.py`: a kill over 1.5 m at its 1 m).
- Captures land in the gitignored `captures/` of the PC that ran them; give each run `--notes`.
- The forward camera's firmware and its detector (`firmware/projects/target_cam/`) are
  namontoy's: settings and changes go through them.

## Commands

From the repo root, in Git Bash (PowerShell: `$BP = (Get-Content software/bench_params.txt) -split ' '`,
then `--param @BP`):

```bash
BP=$(cat software/bench_params.txt)

# camera check, motors off, with the beeps that pace the balloon holder (10 s lead)
python software/bench_cues.py watch cam.log -- python -u software/cam_watch.py --seconds 62

# --trim: -0.6 for the new battery type, -1 for the earlier cells (Rules)

# T3, the follow hover: climb, 5 s settle, 40 s following the balloon, landing
python software/bench_cues.py fly t3.log -- python -u software/square_test.py --legs 0 \
    --settle 5 --hold 40 --follow --trim -0.6 --wait 180 --param $BP --notes "T3 ..." --go

# test 5, one leg along the line: stops at 2 m (B)
python -u software/square_test.py --legs 1 --push 3 3 --coast 3.5 --brake 3 4 --hold 10 \
    --trim -0.6 --wait 180 --param $BP --notes "test 5 ..." --go

# a trim check: climb, 20 s at 1 m, land; tell where it landed against the start
python -u software/square_test.py --legs 0 --settle 20 --trim -0.6 --wait 180 --param $BP --go

# test 7, the square: A-B-C-D-A, legs as test 5, left turns on the spot
python -u software/square_test.py --legs 4 --push 3 3 --coast 3.5 --brake 3 4 --hold 5 \
    --trim -0.6 --flight-bias-tau 600 --wait 180 --param $BP --notes "test 7 ..." --go

# the balloon flight: climb to 2 m, look, scan +-60 deg, follow, 40 cm toward / away /
# toward with the proximity guard, land (the balloon's centre about 1.8-1.9 m, 2.2 m away)
python -u software/target_test.py --trim -0.6 --param $BP --notes "balloon flight ..." --go

# the course room (2026-11-03): balloon 1 alone, then the course; the patches from C0
python -u software/course_test.py --course 2,0 --close-px <b1> --trim -0.6 --param $BP --notes "C3 ..." --go
python -u software/course_test.py --close-px <b1> <b2> --trim -0.6 --param $BP --notes "C4 ..." --go

# the downward camera here (F0, motors off; the blimp carried at about 1.3 m; 480 px: the S3-WROOM CAM)
python software/flow_test.py room-floor still --seconds 30 --width 480 --window
python software/flow_test.py room-fwd move --moved 0.5 --width 480 --window --marks Out,Back \
    --say "Wait 5 s. Carry the blimp 50 cm from S toward balloon 1 in about 5 s, stop 3 s, back to S, keep still"
python software/flow_test.py room-left move --moved 0.5 --width 480 --window --marks Out,Back \
    --say "Wait 5 s. Step 50 cm to the blimp's LEFT, its nose still toward balloon 1, stop 3 s, back to S, keep still"
python software/flow_test.py room-floor tilt --width 480 --window
python software/flow_test.py --calibrate captures/flow-room-fwd-move-<date>.bin captures/flow-room-left-move-<date>.bin \
    --moved 0.5 --width 480 --cal-out captures/flow-cal-room.json
python software/flow_test.py --analyse captures/flow-room-fwd-move-<date>.bin --moved 0.5 --width 480 \
    --cal captures/flow-cal-room.json      # motion 1's direction is <fwd>; the left run's gives <left>

# after every flight (C1-C5): the optical flow's track over the course, and its plot
python software/flow_track.py captures/<run>.bin --cal captures/flow-cal-room.json --image-dirs <fwd> <left> \
    --tilt captures/flow-room-floor-tilt-<date>.json --svg captures/<run>-track.svg

# afterwards
python software/blimp_log.py captures/<name>.bin                 # summary
python software/blimp_log.py captures/<name>.bin --csv out.csv   # every 20 ms record, in units
```

`bench_cues.py` beeps from the PC's speakers (Windows; elsewhere the terminal bell). `watch`:
short beeps at 3, 2, 1, a long high beep at the start, one beep every 10 s (go to the next
spot), one at 55 s, two long beeps at the end. `fly`: a long high beep when the follow starts,
one every 10 s for 60 s, two long beeps when the landing starts.

Each capture holds every 20 ms tick: the IMU, magnetometer, barometer, ToF, battery, the four
motor commands, what `core/` asked for, and the camera's latest detection (`cam0_detected`,
`cam0_cx` of 160 px, `cam0_area`). There is no position: nothing on board measures it, so the
team's report of where the blimp went (photos against the tape) is the only record of it.

## The order from here

| # | Run | Status |
|---|---|---|
| 1–3, H0–H3, 4 | reverse, turns, push and coast, hovers to find the trim, push and brake | done (session record) |
| T0–T2 | the camera's link, its sign, its range | done |
| T3 | follow the balloon while hovering | flown twice; done for the controller (turns to a seen balloon, holds when it is lost, finds it again); the left step was never detected |
| check | the walk-in camera check | done: reliable from about 2.2 m in, patches of 26–70 px |
| 5 | one leg, A to B | done: push 3 gf 3 s, coast 3.5 s, brake 3 gf 4 s stops it at 2 m (trim −0.6, new battery); it drifts afterwards |
| 7 | the square | flown: the legs right, the heading 96° off by the end (`core/`'s gyro-bias learning); fly again with `--flight-bias-tau 600` |
| B1–B6 | balloon flights, `target_test.py` | done: not seen from 3 m; found from about 2.2 m, approach, guard and back-off worked (B5); the nose lags 11–20° |
| **R** | **turn on the spot**: heading steps at 1 m (as test 2) at `front_reverse_ratio` 0.74 and lower, then a trim check | **next**; in the course room it is C2 |
| 7b | the square with the workaround | after R |
| T4 | approach and touch | after the detection is fixed (needs 4 m); B5 did a short one; C3 is one at 2 m |
| T5 | hold a distance from the balloon | needs code and a flash |
| T6 | a balloon course | from the ground: C4 below; the race mode on the S3 still to come |
| **F0** | **the course room, the downward camera**, motors off: the floor's texture, the scale, the forward and left directions, the tilt (`flow_test.py`) | **next** (2026-11-03), with C0 |
| **C0** | **the course room**: the camera, motors off: each balloon's patch from 2 m to the touch, nothing else red | **next** (2026-11-03) |
| C1 | the course room: a hover at 1.3 m, this room's air | after C0 |
| C2 | the course room: the right turn on the spot (R) | after C1 |
| C3 | the course room: balloon 1 alone, `course_test.py --course 2,0` | after C2 |
| C4 | the course room: both balloons, `course_test.py` | after C3 |
| C5 | the course room: C4 three times, as the race's three loops | after C4 |
| A0–A5 | assisted flight from the Steam Deck (`docs/assisted-control.md`) | not flown; needs no flash |

**The walk-in check (motors off).** The background: the darkest plain one available, no window
or lamp behind the balloon, nothing red, orange or brown in view, light falling on the balloon
from the blimp's side. The blimp still on A (or held steady at about 1 m), its nose toward the
3 m strip. The balloon straight ahead, about 1 m up, on its string or a stick. Run the camera
check command; on the beeps: start on the 3 m strip; at 10 s walk slowly toward the blimp; at
20 s pass about 2.2 m; at 30 s stop at 1.5 m; at 40 s walk slowly back; at 60 s back at 3 m.
Read the log for the distance from which the balloon is detected in nearly every line with a
steady area. A whole 34 cm balloon should give about 470 px at 2 m and 210 px at 3 m; much less
means the camera sees only pieces of it. If no distance is solid, the camera's colours need
looking at in `tcview` (a USB-serial adapter on the CAM's UART0) with namontoy; their commit
`2423029` (not flashed) has a new colour table and a white-balance correction
(`docs/learned-detector-report.md` §10.9–10.10).

**T3 (follow while hovering).** Hover at 1 m over A, following the balloon held straight ahead
at the distance the check found (d). Starting at the follow's long beep, 10 s per spot: centre;
to the blimp's left by d x 0.36 (20°; 0.7 m at 2 m, 1.1 m at 3 m); centre; right; centre;
hidden; centre again. Pass: it turns toward each new spot within about 5 s, stops facing it
without swinging, holds its heading while the balloon is hidden, and turns back when it
reappears. Tell: where it turned and where it did not, and where it landed against A.

**T4 (approach and touch).** The balloon tethered at the 4 m mark, about 1 m up. The blimp
climbs at A, starts following, then a gentle push carries it along the line while it keeps
aiming; a sideways current shows as the balloon sliding off centre, and the follow turns into
it. `--legs 1 --follow` with `--push 3 3` and a coast; after that push the blimp moves at about
0.3 m/s (reasoned from test 4c), so about 12 s of coast reaches 4 m. Needs the detection to
hold from 4 m down to under 1 m. Tell: whether it touched, and how far to the side it passed.

**5 (one leg, A to B).** Push, coast, brake to stop at 2.0 m: test 4c's push 3 gf 3 s and
brake 3 gf 4 s stopped within about 1 m, so try about 3.5 s of coast between them
(`--coast 3.5`). Tell: how far before or past B it stopped, and how far to the side.

**T5 (hold a distance).** Hover 3 m from the balloon, move the balloon back 1 m: the blimp
should follow and stop 3 m from it again. It uses the camera's range as feedback, which cancels
drift along the line of sight. It needs a range loop in `core/`, the firmware passing the range
on, a simulator check and a flash, and `d255` on the camera.

**7 (the square).** A → B → C → D → A: each side a push, coast and brake, 5 s still, a left
turn on the spot, 5 s still, then the landing; about 2 minutes. The room's air will push it a
metre or more toward the far wall, which only the balloon tests correct.

**T6 (a balloon course).** Two to four balloons around the room, touched in order: the race in
miniature. It needs `core/`'s race guidance running on the S3 (today the S3 flies only the bench
hold).

**R (turn on the spot).** At 1 m over A, heading 0 → +90 → 0 → −90 → 0 as in test 2, with
`--param front_reverse_ratio=<r>` after the bench set: first 0.74, then lower (about 0.55).
Tell: whether each turn stays on the spot or creeps forward, and where it landed. The ratio
that turns in place then needs a trim check (`--legs 0 --settle 20`), since it also sets how
strong the trim and the brake are.

**The balloon flight (`target_test.py`).** One red balloon in view, its centre about 1.8–1.9 m
(level with the camera at the 2.0 m hover; above that it leaves the top of the view as the
blimp closes in), the blimp on the floor about 2.2 m away facing it. The proximity guard's
350 px was set with the 34 cm balloon: a bigger balloon trips it farther away, so say the size.
Tell: whether it turned to the balloon, how close it got, whether it backed off, where it landed.

## The course room (2026-11-03)

The race in miniature, in a second room (the team's photo of 2026-11-03). From the start S,
balloon 1 is 2.0 m straight ahead; balloon 2 is 2.0 m from balloon 1 on the blimp's right, at
about a right angle; both hang with their centres about 1.2 m up. `course_test.py --course`
takes each balloon's floor point in metres from S, x along the start's nose and y to its left,
so its default `2,0 2,-2` is this room. **Measure S to balloon 2's floor point:** 2.83 m means
the right angle. Any other distance changes the course (`--course 2,0 <x>,<y>`): say it.

What `course_test.py` does (the detail is in its docstring; `--selftest` flies it against a
stand-in with no hardware):

- climbs to 1.3 m looking for balloon 1; once seen, the nose follows it (`hold follow`);
- approaches in pulses: 3 gf for 1.8 s (about 0.15 m/s), a coast of up to 5 s, the brake,
  then 2 s still while the nose aims again. Each pulse flies straight, the nose held where the
  balloon was last seen;
- **close** (the balloon's patch at `--close-px`): coasts on into the balloon for 2.5 s,
  brakes, moves back about as far, holds still;
- turns toward balloon 2: its bearing from where balloon 1 was last seen (−76° on the plan),
  a look, then a ±45° scan if needed. Balloon 1 is ignored there, by its bearing and its size;
- balloon 2 the same, then the landing. Every end is a landing, as in `target_test.py`.

The report gives each balloon's "seen" and "close" times from the start of the hold: the lap,
as near as the ground can tell. Whether the envelope touched is the team's call.

**Setting up.**

- Nothing red, orange or pink in the camera's sweep but the two balloons. Take out the red
  balloon in the cluster by the left wall (behind the blue one), and replace the orange disc
  under balloon 1 with tape of another colour: from the floor at S the camera sees it right under
  balloon 1.
- About 1 m clear around both lines: over a chair or a desk the ToF reads short and the height
  loop climbs (the ToF guard lands it at 1.6 m over the floor).
- The tethers straight up from the floor points, nothing slack across the path.
- The PC a few metres from the course, out of the camera's view.
- The forward camera on `target_cam` again (it ran `optflow_cam` on the evening of
  2026-11-02): `course_test.py` refuses to start without its rows.
- The downward camera on GPIO18 and the S3 at `3fe7911` or later (Before a session), its
  view nothing but the floor. F0's still run says whether this floor has texture enough at
  1.3 m; a plain one needs tape strips or a patterned mat along the two lines.

**F0 (the downward camera here, motors off; with C0, the blimp carried the same way).** One
person carries the blimp with the gondola at about 1.3 m and level (its ToF gives the height);
`--window` shows when each run records. The commands are under Commands, all with `--width 480`:

1. **still**, 30 s at S: the floor's texture (the blocks matched, of 16) and the drift at
   rest. Under about 8 blocks, or many frames refused, the floor is too plain here: tape or a
   mat first, or the tracks will have holes.
2. **forward**: from S 0.5 m along the line to balloon 1 in about 5 s, 3 s still, back to S.
   Its analysis's first motion gives `<fwd>`, the direction in which the image moves when the
   gondola goes forward.
3. **left**: the same 0.5 m to the blimp's left, a sideways step with the nose kept toward
   balloon 1: `<left>`, about 90° from `<fwd>` one way or the other (the image is mirrored).
4. **tilt**: at S, the blimp rocked by hand about 5° each way, about once a second: 10 s nose
   up and down, 10 s side to side. The shift a swing makes against the gyro; its fit goes in the
   run's `.json`, which `flow_track.py --tilt` reads.
5. **`--calibrate`** the forward and left runs: f and the matcher's offset, from the ToF's
   heights, into `captures/flow-cal-room.json`. f near 430 px means a 58° lens; the desk
   calibration of 2026-11-02 (`flow-cal.json`) was the old camera's and does not apply.

Tell: whatever the window or the reports said, and whether the blimp stayed level.

**C0 (the camera here, motors off).** The blimp carried by hand, the gondola at about 1.3 m,
the nose forward, `cam_watch.py` running under `bench_cues.py watch`:

1. Along the first line toward balloon 1, on the beeps: S (2.0 m) at 0 s, 1.5 m at 10 s,
   1.0 m at 20 s, 0.7 m at 30 s, the envelope just touching the balloon at 40 s, back to
   1.0 m at 50 s. Read the patch at each mark, from where it is detected, and its elevation at
   2 m.
2. The same along the second line toward balloon 2 (a pair of hearts: another size).
3. At S, then about 0.5 m short of balloon 1, the blimp turned slowly through a full circle:
   nothing but the two balloons may be detected; note where balloon 2 appears.

`--close-px` for each balloon is its patch about 0.4 m before the touch, the distance the
2.5 s coast covers (reasoned: about 0.15 m/s). If balloon 1 reads more than about 5° above
the camera's level at 2 m, raise `--alt` by 0.1 m (3.5 cm per degree at 2 m); 1.3 m comes from
the balloon flights, where a balloon centred 0.1–0.2 m under the hover height read level.

**C1 (a hover here).** From S, nose toward balloon 1: climb, 20 s at 1.3 m, land.
`square_test.py --legs 0 --alt 1.3 --settle 20 --trim -0.6 --wait 180 --param $BP --go`. Tell
where it landed against S: that is this room's air. The selftest's stand-in flies the course
with steady air up to about 1 cm/s (0.4 m in 40 s). At 2–3 cm/s across the course (the first
room's 1–2 m in 40 s is 2.5–5) it often lands without balloon 2, and with the air to its left
it can touch balloon 1 a second time and log it as balloon 2: nothing on board measures
position. If C1 drifts a metre or more, look for the source (a door, a vent, the air
conditioning) before C3.

**C2 (the right turn: R, here).** 80° right and back at 1.3 m, as the course turns at balloon
1: `hold_test.py --steps 0 0 -80 0 --dwell 10 --hold-alt 1.3 --climb-rate 0.12 --surge -0.6
--param $BP --go`, then again with `front_reverse_ratio=0.55` added after `$BP`. Tell: whether
each turn stayed on the spot or crept forward, and where it landed. `hold_test.py` has no
`--flight-bias-tau`, so its turns teach `core/` some bias, a few degrees in 40 s (test 7).

**C3 (balloon 1 alone).** `course_test.py --course 2,0 --close-px <b1> ...` (Commands). Tell:
whether it touched and how (a brush, a push, the balloon swung away), how far past the balloon
it went, and where it landed. Short of the balloon: a larger `--close-px` or a longer
`--touch`; a hard push: the opposite.

**C4 (the course).** `course_test.py --close-px <b1> <b2> ...`. Tell, balloon by balloon,
whether it touched, and where it landed.

**C5 (three loops).** The race allows three attempts per course and counts the fastest
(docs/01-competition-brief.md): C4 three times in a row, the same settings, a fresh cell, the
three logs' "close" times in the session record.

**After every flight (C1–C5): the track.** `flow_track.py` on the run's capture (Commands):
where the flow says the blimp went, phase by phase, how far from each balloon's floor point it
was when it went in to touch, and an SVG of the track over S and the balloons. Set its end
against where the team saw it land: C1's hover is the first check, and the runs after it say
how far to trust it (reasoned: the heading drifts a few degrees a minute, a refused frame is a
hole in the track, the scale is the calibration's). C1's track is the room's air measured; C2's
shows whether a turn crept forward and how far. Nothing on board uses the flow yet: the track
is the ground's, after the run.

## Known problems

- **`core/` learns the gyro's bias in the air** and the heading hold turns the blimp with it
  (test 7: 96° off). Workaround `--flight-bias-tau 600`; the fix is in `core/` (no learning
  while armed; `docs/10-known-issues.md`).
- **The nose lags a seen balloon by 11–20°, and turns creep forward** (B4–B6): the front pair's
  reverse gives about three quarters of what is commanded and, near zero, nothing for a moment.
  Calibrate `front_reverse_ratio` (R); a small `yaw.ki` may also help the follow.
- **The ToF loses the floor beyond about 2.5 m** in the flight room: 2.3 m at most.
- **A lost Wi-Fi link stops the motors** in the air after 1 s (`core/`'s watchdog); landing by
  itself instead needs a firmware and `core/` change.
- **About 6 of the camera's 13 rows a second reach the PC** over Wi-Fi (the S3 has them all,
  and `hold follow` runs there).
- **The camera's detection** is the blocker (T3, the check, B1–B6: found from about 2.2 m
  with the crumpled balloon, never from 3 m). The follow itself is good.
- **The room's air** drifts the blimp 1–2 m toward the far wall in about 40 s, differently each
  run. No trim fixes it; the balloon tests are the correction. The IMU cannot measure it (the
  session record has the numbers), and the rear motors cannot push sideways.
- **The brake is weaker than the push:** 3 gf for 3 s needs 3 gf for 4 s to stop.
- **The brake dips the height** 9–12 cm; the height loop recovers in about 8 s.
- **The camera reads about 3° right** of a balloon on the tape line.
- **The trim lives in the script,** a constant −1 gf. It belongs in `core/`, scaled with the
  rear pair's thrust, before the race.
- **The front pair's fixed commands weaken with the battery.** The rear pair is in the height
  loop and compensates; the trim, the push and the brake do not. At 3.22 V hovering (3.40 V in
  H3) the trim lost about 6 % and the follow hover drifted 4–5 m forward in 100 s. Fly charged
  cells (3.7 V or more at rest, about 3.4 V hovering) and keep follow tests to about 40 s.
  The fix: scale the thrust-to-command conversion with the battery voltage where the firmware
  turns `core/`'s thrusts into motor commands (a flash).
- **The test balloon is a poor stand-in:** crumpled 34 cm foil, mostly glare and folds; the
  race uses taut 22-inch (50–60 cm) red mylar balloons (Rule 21). If a balloon's other face is
  not red, keep the red face toward the blimp.
