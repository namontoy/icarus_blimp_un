# Race day: the course from the ground station

2026-11-05, branch `assisted-control` (on top of the assisted flight of 2026-11-04). **Built and
tested against `mock_blimp.py` only; not flown.** Nothing here needs a reflash: the blimp runs the
same `hold` commands the flight scripts have flown since 2026-10-31.

## What there is

- **The race sheet**, `software/race_course.json`: how many balloons, where each hangs (its floor
  point from the start and the height of its centre), how far the floor runs from the start on
  each side, the patch at which the blimp goes in to touch, how many loops. `software/race_course.py`
  reads, checks and prints it (`python software/race_course.py`).
- **The Course panel** on the Control page, in *Fly: Autonomous*: the sheet drawn from above, to
  scale, with the legs the course flies and the reach of the camera from the start, and the same
  numbers in boxes. Every change is written to the file at once; *Reload* reads it back after an
  edit by hand or a `git pull`.
- **Fly the course** (A on the pad): the course flown from the station by the flight thread the
  assisted mode uses (`assist.py`), with `course_test.py`'s sequence in place of the sticks:
  the climb to the first balloon's height, the look, the nose on the balloon, straight pulses toward
  it, the touch, the move away, the turn to the next, each balloon's own height, the landing.
  Land, X, Y, STOP, B and every automatic end (the ToF high or silent, a frozen window, a link gone
  for more than 3 s) work as in the assisted flight (`docs/assisted-control.md`). A capture of the
  plan's length starts by itself; the Bench page downloads it afterwards.
- **The same sheet from a terminal**: `course_test.py --course-file software/race_course.json`,
  the fallback if the window is not to hand.

### Why the Control page

One source drives the motors at a time, and the Control page already owns the hand-over between
the Flight page's motors, the sticks, an assisted flight and the STOP latch. Its pad bindings
are the same in every mode (A starts, X or Y lands, B stops), which is what the operator's thumbs
know under race stress; the dead *Go autonomous* button sat there already; and the Flight page
stays the monitor (camera and sensors) during the race. A fourth page would have duplicated the
arming, the landing and the pad for one button.

## At the venue: what to measure

From where the gondola will stand at the start, S, with its nose toward balloon 1:

1. **Each balloon, in race order**: metres ahead of S along the nose, and metres to S's **left**
   (a balloon on the right is a negative left), of its floor point (the foot of its tether, or the
   point under it); and the **height of its centre** over the floor.
2. **The field**: how far the floor runs from S ahead, behind, to the left and to the right
   (walls, tape, the organisers' lines). It serves the sketch and the checks only: nothing on board
   measures position, so the blimp finds each balloon with its camera, as `course_test.py` does.
3. **The close patch** (`close_px`): the motors-off camera check with the venue's own balloons
   (`docs/flight-tests.md`, C0; `software/cam_watch.py`): the patch about 0.4 m before the envelope
   would touch. The sheet carries 1500 until measured, a reasoned guess for a taut race balloon.
4. **The battery's trim**, in the Trim box as for every flight (−0.6 gf for the new battery type).

The heights: the gondola holds each balloon's centre height plus *fly above* (0.1 m by default: the
course room of 2026-11-03 flew 1.3 m at balloons of 1.2 m). A balloon higher than about 2.2 m
cannot be flown: the ToF loses the floor beyond 2.5 m, and the hold is capped at 2.3 m.

Legs: the camera has found a balloon from about 2.2 m. A balloon farther than that from where the
last one leaves the blimp (0.5 m short of it) is a **warning on the sheet**, not a refusal: the
blimp looks for it, scans ±45°, and lands if it does not see it. There is no blind leg toward
where a balloon should be; on a long leg the team should expect a landing, and move the start or
ask about the spacing. A leg under 1 m, or over 6 m, is refused.

## Filling the sheet

On the Deck: Control page, *Fly: Autonomous*, the Course panel: Count, then each balloon's row,
the field, Close, Fly above, Loops. The line under the sketch says what the sheet is, or why it
cannot be flown, or what it warns about. Or edit `software/race_course.json` by hand (or ask the
assistant to), commit, push, and `git pull` on the Deck, then *Reload*.

## Flying it

1. The rules of `docs/flight-tests.md` hold: nothing moves before the team says so, a spotter,
   clear floor, one battery type, the trim.
2. The blimp on the floor at S, nose toward balloon 1, gondola still. The forward camera on
   `target_cam` (the course refuses to start without its rows). Nothing red in the camera's sweep
   but the balloons.
3. The station joined to the blimp's Wi-Fi, *Fly: Autonomous*, the trim, a look at the sketch.
4. *Fly the course* (or A). The preparation is the assisted flight's: the bench parameters, the
   driver's limits, a camera row, 60 s of stillness (the count on the page), the ToF seeing the
   floor; then the climb. The Course panel's line follows the phase and the sketch marks the
   balloon being flown to; the events (seen, close, the pulses, the turns) are in the console
   dock's *Events*; the header's Drive pill reads AUTO.
5. At any point *Land*, X, Y or L lands it; STOP, B, Space or Esc coasts the motors where it is.
6. Afterwards: Bench page, *Download*: the capture holds the whole run.

The terminal fallback, from the repo root:

```bash
python software/course_test.py --course-file software/race_course.json --trim -0.6 \
    --param $(cat software/bench_params.txt) --notes "race ..."          # the plan
python software/course_test.py --course-file software/race_course.json --trim -0.6 \
    --param $(cat software/bench_params.txt) --notes "race ..." --go     # PROPS ON
```

## What is autonomous here, and what is not

The sequencing runs on the ground, from the camera's rows the S3 relays and the hold's status, as
every flight test has; `core/`'s height and heading loops run on the blimp, and `hold follow`
turns the nose to the balloon on board. So the Wi-Fi link is part of the loop: `core/` zeroes the
hold after 1 s of silence (the motors stop in the air), and the station lands the blimp if the
link comes back after more than 3 s. The firmware's own mode switch (`MODE_SET`,
`docs/control-loop-plan.md` increment 3) and `core/`'s guidance on the blimp remain to be built;
the rulebook's "autonomous" is met in the sense that nobody steers.

## Measured, and reasoned

Flown, and used as they are: the climb and landing rates, the kills, the 2.3 m ceiling, the trim,
the pulse (3 gf for 1.8 s, about 0.15 m/s), the touch's 2.5 s coast, the camera's 2.2 m reach
(`docs/flight-tests.md`). Reasoned, not flown: `close_px` 1500; the per-balloon height move
(during the turn, the look waits for the ToF to come within 0.1 m of the new height); the window
and the mock's camera, which is a disc in a toy. `course_test.py --selftest` flies the sequence,
with heights and loops, against its stand-in; `tests/test_course.py` flies it on the station's
thread against the mock, with Land, STOP and a missing camera.

## Limits

- Not flown. The first flight should be one balloon at 2 m (C3 in `docs/flight-tests.md`), from
  the page, before the course.
- Everything the course inherits from `course_test.py` and the assisted flight: turns creep
  forward, the nose lags, the room's air moves the blimp 1–2 m in 40 s, no position hold, the
  camera's detection with a taut balloon still to be tuned by namontoy.
- The sheet's field does not fence the flight: a leg that overshoots a wall is not stopped by it.
- The X and Y buttons and this page's Autonomous mode have not run on a Deck.
