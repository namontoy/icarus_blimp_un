# Flight tests: where they stand and how to run the next one

**Status, 2026-11-02 evening.** The blimp climbs, holds 1 m and its heading, goes 2 m along
the line and stops there (test 5), and lands on its own; it follows the red balloon with its
nose when the forward camera sees the balloon (T3 done for the controller). Two open problems:
the camera's detection (the crumpled 34 cm foil test balloon is found reliably only within about
2 m, and only when a good face looks at the camera), and the open-loop drift, which grows with
the flight's length and changes with the battery (each battery type needs its own trim). **Next:
test 7 (the square), on a freshly charged cell of the new type with `--trim -0.6`.** The
detection goes to namontoy (a taut race-size balloon, the colours tuned in this room).

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
  Firmware `i2c_tools` at `408d96e` or later from `pid-tuning`; the forward camera powered.
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
- Limits: the firmware's 40 % duty ceiling and 10 % deadband (its defaults since 2026-11-01);
  the scripts kill on yaw over 62 °/s, roll or pitch over 120 °/s, ToF over 1.5 m.
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
| **7** | **the square** | **next, on a fresh cell** |
| T4 | approach and touch | after the detection is fixed (needs 4 m) |
| T5 | hold a distance from the balloon | needs code and a flash |
| 7 | the square | later |
| T6 | a balloon course | needs the race mode |

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

## Known problems

- **The camera's detection** is the blocker (T3, the check). The follow itself is good.
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
