# Assisted control from the ground station

2026-11-04, branch `assisted-control`. **Built and tested against `mock_blimp.py` only; not yet
flown.** Nothing here needs a reflash: any S3 firmware with `hold altitude <m> <m/s>` (`408d96e`
or later; the blimp runs `aa13a19` or `3fe7911`) has every command it uses.

## The question

The Control page drives the four motors open loop. The team asked whether it should use the
control loops instead, so that setting the height to 1 m makes the blimp hold 1 m while the
operator goes forward, back and turns, and whether it needs a Land button.

**It is worth doing.** The reasons, from what the flights have shown:

- **In manual the operator is the height loop.** A blimp about 6 gf heavy hovers with both rear
  motors near the 40 % ceiling, so the heave stick is held near full for the whole flight. The
  vehicle answers seconds later, so a thumb can only chase it.
- **The loops already fly.** `hold` has held 1 m and 2 m and a heading through every test since
  2026-10-31, the climb and the landing work, and every scripted flight has landed itself.
- **The cost is low.** The firmware needs no change, so the risk sits in one new ground module.

What assist does not do: it holds height and heading, not position. Nothing on board measures
where the blimp is or how fast it moves, so the room's air still drifts it (1–2 m in 40 s in the
flight room) and stopping is still the operator's job.

## How it was built, and why that way

| Option | What it is | For | Against |
|---|---|---|---|
| **Setpoints over the console (built)** | The station sends `hold altitude`, `hold heading`, `hold surge` on the UDP console, as the flight scripts do | No flash, every command has flown, the team can try it today | A console line per change; yaw reaches the ground only through `hold`, about 3 times a second |
| A new link packet | `HOLD_SET` on UDP 7200, with the estimate back in telemetry | Lower latency, no text parsing, and the firmware could land by itself on a lost link | Firmware, protocol and a flash, for a feel nobody has tried yet |
| Rate commands in `core/` | Yaw rate and climb rate loops in `blimp_step()` | The cleanest stick feel | A `core/` change with sim and bench work before the first try |

The built option comes first because it answers the real question, whether assisted flying is
better, at the lowest cost. If it is, the packet is the next step (see the end of this page).

The rule that control logic lives only in `core/` holds: `software/ground_station/assist.py`
shapes a stick into a setpoint and runs the checks a flight script runs. The loops are `core/`'s.

## What the operator does

Control page, **Fly: Assisted**, then Take off (A on the pad). The sticks, in mode 2:

| Control | Manual | Assisted |
|---|---|---|
| Left stick ↕ | rear pair | moves the height setpoint, up to 0.12 m/s; let go and it holds |
| Left stick ↔ | front pair's difference | swings the heading, up to 20 °/s; let go and it stops and holds |
| Right stick ↕ | front pair together | a push of up to the Surge box (3 gf), on top of the trim |
| A | arm | take off |
| X or Y, or L on the keyboard | | **land** (before the motors start: call the take-off off) |
| B, Space, Esc, STOP | coast | coast in the air; the blimp falls, so a last resort |
| D-pad ↑ ↓ | limit ±5 % | height ±0.1 m |
| D-pad ← → | | trim ∓0.1 gf, also in flight |

Height, Surge and Trim are number boxes on the page. The default trim is −0.6 gf, for the new
battery type. Nothing measures speed, so the way to brake is to pull the right stick back.

Land is greyed out in manual, because a hold cannot start in the air (Limits, below).

### Take-off

The first press does what a flight script does before it starts the motors:

1. Refuses if there is no telemetry or a motor is driven.
2. Ends any hold left on the blimp, sets `software/bench_params.txt` (a reboot restores
   `params.c`'s placeholders, with which the blimp does not leave the floor) and the driver's
   40 % and 10 %.
3. Waits for 60 s of stillness, with the count on the page. Keep the gondola still.
4. Checks that the ToF sees the floor.
5. Sets `gyro_bias_tau_s` to 600 s for the flight (docs/flight-tests.md, Known problems) and puts
   it back to 20 s at the end.
6. Starts the hold: heading 0 where the nose points, the trim, and the climb at 0.12 m/s.

A second flight waits out the minute again, because a flight resets the count.

### Every way a flight ends

| Trigger | What happens |
|---|---|
| Land | the landing |
| ToF over the setpoint by 0.3 m (a desk or chair passed underneath, then the floor again) | the landing |
| ToF silent for 1 s | the landing |
| Window frozen for 3 s | the landing, run by the flight's own thread |
| The gamepad goes away while driving | the landing (in manual this is a STOP) |
| Link back after more than 3 s away | the landing |
| Link back within 3 s | the flight goes on |
| STOP, B | motors coast in the air, `hold off` clears KILLED |
| Yaw over 62 °/s, roll or pitch over 120 °/s | killed: motors coast in the air, as in the scripts |
| A stop from elsewhere (`motor -s`, another station) | seen at the next poll; the flight ends |
| The blimp reboots | the flight ends; no motors, since the hold is gone with it |
| The window closes (including Steam's Exit) | STOP |
| The window dies with no goodbye | the hold stays on the blimp and `core/` zeroes it after 1 s; the next window ends it on first contact, using a marker file in the system's temp folder. Without the marker a hold belongs to a script and is left alone |

The landing is `hold_test.py`'s: down at 0.1 m/s to 0.30 m, then 0.03 m/s, motors off at a ToF of
0.09 m. The sticks are out of it, and the trim stays on. A lost link stops the motors in the air
after 1 s, and nothing on the ground can change that.

While a flight is up, the Flight page's motors stay off, the Wi-Fi button is locked, and the
console dock passes reads only, because a `tof` or `imu` command takes its sensor away from the
stream the loop reads (CLAUDE.md, "Sensor stream"). A capture from the Bench page works as usual;
start it before Take off.

## Numbers: measured, and reasoned

Measured on the blimp, and used as they are:

- Climb 0.12 m/s (floor to 1 m in 8 s, 2026-10-31), landing rates 0.1 and 0.03 m/s, the landing's
  0.30 m and 0.09 m, kills at 62 and 120 °/s, 2.3 m maximum height, trim −0.6 gf, push 3 gf.

Reasoned, not flown; these are the first things to check, all in `assist.py`'s `Tuning`:

- **Turn feel.** The heading loop damps on the yaw rate, so a moving setpoint is followed about
  kd / kp = 1.7 s behind. The setpoint may lead the nose by 25°, which gives about 15 °/s at full
  stick. On release it goes to 0.7 s of turn ahead of the nose, which should brake the turn.
- **The console's rate.** Setpoints go out at most 10 times a second and `hold` is polled 3 times
  a second. The scripts send far less. At −80 to −86 dBm, whether that stays comfortable is
  unknown. A line that is lost goes out again, because each poll says what the blimp really holds,
  and the kills do not wait for the console: they run on every SENSORS packet.
- **Height with a push on.** The brake dips the height 9–12 cm for about 8 s (test 5), so stick
  work on surge will show in the height.

Checked in `test_assist.py`, `test_control.py`, `test_window.py` and against the mock only: the
flow, every end in the table above, the pad's buttons, the page at 1280 × 800. The mock's loop is
a toy, so those tests say the logic is right and nothing about how the real blimp feels.

## Limits

- **Take-off is from the floor, and Land does not work from a manual flight.** `core/` arms only
  after 60 s of stillness, and a turn resets the count (`blimp_estimator_converging()`), so
  `hold` usually refuses to start in the air. Flying manual and then pressing Land could not be
  relied on, so the page does not offer it.
- **No position hold** (above).
- **A desk or chair under the blimp** reads as the floor coming up, so the height loop climbs;
  the 0.3 m guard then lands it. Keep to clear floor.
- **Turns creep forward, the nose lags, and the front pair's fixed commands weaken with the
  battery** (docs/flight-tests.md, Known problems): assist inherits all of these.
- The X and Y buttons, the D-pad's ← → and this page have not run on a Deck. Land is on both X
  and Y because drivers disagree on those two codes.

## First flights

The rules of docs/flight-tests.md apply: nothing moves before the team says so, a spotter, clear
floor. Start a capture on the Bench page before each. After `A0`, each run has a question.

| # | Run | Tell |
|---|---|---|
| A0 | On the Deck with `--demo`: both stick directions, A, X, Y, B, the D-pad, in Gaming mode | anything that does not do what the legend says |
| A1 | Height 0.5 m, Take off, hands off for 20 s, Land | did it hold, how far it drifted, where it landed |
| A2 | 1.0 m. D-pad steps, then the stick, up and down | does it follow and stop where it is told |
| A3 | 1.0 m. Stick left about 90°, let go; the same right | how far it runs on after letting go, and whether it creeps forward |
| A4 | 1.0 m. Half stick forward for 3 s, then pull back to stop | how it stops, and how much the height dips |
| A5 | 0.5 m. Press Land mid-turn; then a flight ended with B | that each does what the table says |

If A3's overshoot is wrong, change `stop_lead_s` and `heading_lead_deg`. If the height is sluggish
or chases, that is the loop's gains, in `software/bench_params.txt` as for every flight.

## Next, if it flies well

- Steering during an operator's landing: it takes 15–25 s, and the air moves the blimp meanwhile.
- A `HOLD_SET` packet and the yaw and height estimates in telemetry: lower latency, and a console
  free for the operator.
- Let a hold start in the air (arm on "has converged once since boot" rather than "is still
  now"): that is what would give manual flight a Land button, and a switch to assisted in the air.
- A landing on the blimp when the link is lost, instead of stopping the motors. Both of these
  last two are `core/` and firmware changes and need a flash.
- A brake from the optical flow, once the flow reaches the S3's loop: the first real speed signal.
