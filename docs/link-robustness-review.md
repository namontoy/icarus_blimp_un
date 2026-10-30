# The link and the window: robustness review and decisions (2026-10-29)

The goal, in the team's words: a robust, professional system in which using the window never
changes what the blimp does, and communication problems never make it do anything strange.
Moving the cursor over a downloaded capture must not stop the motors or change their speed, and
the same window flies the blimp by hand for testing, so it must not be able to disturb it either.

How this was reached: four reviewers (vehicle firmware, ground station, safety and protocol, test
and integration) each read the code and took a position on nine questions, then cross-examined
each other's positions with failure scenarios; a moderator checked every disputed claim against
the code and wrote the decisions. The team then made the judgement calls (§3.2). Findings marked
*measured* were run; *reasoned* ones were read from the code. `docs/motor-setpoint-ownership.md`
is the history before this (the GUI-repaint motor cuts, options A-D, the bridge thread that owns
the command since 2026-10-28).

## 1. The goal as bench tests

1. **The window cannot change the motors.** Ten minutes with a duty held on the stand while
   someone loads a 400 s capture, hovers and zooms the traces, resizes the window, switches pages,
   exports a CSV with the file dialog open, uses the console and downloads a capture: the motor
   readback in the telemetry never changes, no failsafe or link-lost event, the bridge's send gap
   stays under 200 ms (the Events tab reports any longer one, §4), on the laptop and on the Deck.
2. **Communication faults end in a known state.** Program killed, Wi-Fi off, board reset, a second
   station, a 1.5 s stall: the motors are off within 600 ms of the last motor packet (500 ms link
   timeout plus margin), the window shows the cause within 1 s, and what follows is the rule of
   §3.2 (a short Wi-Fi drop resumes ramped; anything else waits for the operator).
3. **STOP always arrives.** STOP pressed during a download with the console busy: motors off within
   250 ms, no STOP lost.

## 2. What the review found in the code

| Finding | Where | How found | Status |
|---|---|---|---|
| A MOTOR_SET read just before a STOP could reach the wire after it; the motors then ran on until the 1 s failsafe | `software/ground_station/blimp_bridge.py` (sent outside the lock) | reasoned; `tests/test_bridge.py` now checks it with an 80 ms send in flight | **fixed** §4 |
| After a Wi-Fi outage or a blimp reboot the standing command came back at full value, with nothing to say so | `blimp_bridge.py`; the firmware accepts `MOTOR_SET` in any state (`net_link.c`) | reasoned | **fixed** on the ground §4; the firmware side is increment 3 |
| B on the pad during a frozen window could be undone when the window thawed (still armed) | `station.py`, `control.py` | reasoned; `tests/test_window.py` checks the fix | **fixed** §4 |
| The "20 Hz" stream ran at about 15.7 Hz on Windows: a 50 ms wait rounds up to the 15.6 ms tick | `blimp_bridge.py` | measured by the review, median gap 63.6 ms | **fixed**, 40 sends in 2 s |
| One C call that holds Python's GIL stops every thread of the program, the motor stream and the heartbeat included | the process model | measured: sorting 3 M floats held it 1.18 s, send gap 1246 ms, on this laptop, Python 3.12.10 | **made visible** §4; the real Bench workload is unmeasured |
| The Wi-Fi Join/Leave button worked while motors ran (leaving drops the link they run on) | `app.py`, `station.py` | reasoned | **fixed** §4 |
| The console over Wi-Fi runs any command; `motor --failsafe 0` switches the motor failsafe off, after which a console-driven motor has no timer at all | `net_console.c` (`esp_console_run`), `cmd_motor.c` (`--failsafe`) | reasoned, both lines read | **fixed** F4 §5 |
| Any valid packet keeps the 500 ms link timeout alive: a heartbeat, a capture command, or a second station (the laptop and the Deck on one access point) | `net_link.c` (`s.last_rx_us` set for every packet type) | reasoned | **fixed** F2 §5 |
| `MOTOR_SET` sequence numbers are never read: a stale or reordered packet is applied | `net_link.c` `handle_datagram` | reasoned | **fixed** F3 §5 |
| Every `MOTOR_SET` rewrites all 8 PWM channels even when nothing changed (harmless to the motors: the LEDC applies a duty at the next period, so the same value gives the same output) | `cmd_motor.c` `motors_set_all` | reasoned | **fixed** F1 §5 |
| Telemetry's `link_age_ms` is the age of any packet, so with heartbeats flowing it hides gaps in the motor stream; `link_check.py`'s "longest the blimp waited" reads it | `net_link.c` | reasoned | **proposal F6**; read `link_check.py`'s figure as a lower bound until then |
| Telemetry goes to whoever sent last, so two stations steal it from each other | `net_link.c` | reasoned | **proposal F5** |
| The reset reason is never reported, so a brownout reboot looks like a Wi-Fi gap | `main/` (no `esp_reset_reason`) | reasoned | **proposal F6** |
| The default Wi-Fi password is in the repo and the framing has only a CRC: anyone with the repo at the venue can drive the motors | `Kconfig.projbuild`, `app.py` | reasoned | **proposal F9** |
| AGENTS.md's "reverse about 74 % of forward" is a lab-supply single-motor figure; through the DRV8833 the rear pair's reverse is the stronger direction | `docs/history/pid-bench-2026-10-28/` | measured 2026-10-28 | open: correct AGENTS.md with the FL/FR data |

## 3. Decisions

### 3.1 Keep repeating the command; make the blimp smart about it

The team asked for "send it only when it changes, with a keep-alive". The review kept the repeat,
for two reasons, and moved the "only when it changes" to where it matters:

- **The keep-alive should carry the value.** An empty keep-alive and a repeated `MOTOR_SET` cost the
  same: one 28-byte packet, 560 B/s at 20 Hz, about a tenth of what the blimp sends back. But a
  repeat repairs a lost change 50 ms later, while a change sent once and lost leaves the blimp on
  the old value until the next change, with nobody noticing.
- **A heartbeat proves the program runs, not that anyone is steering.** The heartbeat comes from
  its own thread and kept going while the window was frozen; a liveness signal has to come from
  the thing that carries the operator's intent, which the repeated command does.

So: a change leaves at once, an unchanged command is repeated at 20 Hz (§4); on the blimp, a
repeat with the same values should only restart the timers and not touch the PWM (F1), and only
motor packets should count as motor liveness (F2).

### 3.2 The team's choices (2026-10-29)

| Situation | The review recommended | The team chose |
|---|---|---|
| The window freezes while flying by hand | stop after 0.4 s on the Control page, 3 s on the Flight page | **3 s everywhere**: a short stall rides through |
| Wi-Fi drops about 1 s while flying by hand | stay off until someone re-arms | **resume automatically**, ramped from zero over 1 s, if the link returns within 3 s; a longer outage drops the command and waits for the operator. Not after a blimp reboot: a reset under load is often a brownout, and resuming would repeat it |
| A separate program for the link | only if a stall measurement shows the need | **no**: handle the threads properly in the station itself; and a **race mode** in which the window only watches, with STOP its one command, since the race allows no manual control |
| Switching page while the pad drives | keep driving, with an always-visible indicator | **keep driving**; the header's Drive pill says who drives, on every page |

The resume choice agrees with the control-loop plan's D6 (in MANUAL a link loss zeroes the motors,
it does not latch); D5's latched `MOTOR_STOP` is untouched, and the ground never sends a STOP for a
short outage, only zero.

### 3.3 Other decisions of the review

- **Duty slew on the vehicle:** an actuator-protection limiter in the firmware's output stage, never
  in `core/` (it would sit inside the plant the PID bench measures), limiting only rises and
  reversals, with STOP and every failsafe bypassing it. Built only if the rail test of §6 fails.
- **Per-motor thrust curves:** the thrust-to-duty table belongs in the firmware's output stage,
  per motor and direction; `core/` gets per-motor maxima in `blimp_params_t` through `/core-change`
  so the Rule 13 cap stays enforced there. Both wait for FL and FR to be measured.
- **The S3's receive path:** pace the capture download (F7) and measure; a separate receive task
  only if STOP-to-coast exceeds 100 ms (p99) during a download.
- **Verification:** the offscreen tests are in the repo and in `tools/verify.py --gui` (§4); the
  bench gates are §6.

## 4. What changed in the ground station (2026-10-29, branch `link-robustness`)

`blimp_bridge.py`'s docstring has the rules; in short:

- A change goes out at once (0.5 ms worst over six changes, *measured* offscreen); an unchanged
  command is repeated on a deadline clock, so 20 Hz holds on Windows too (40 sends in 2 s); calling
  `set_motors` with the same values adds nothing to the wire.
- STOP wins: the stream sends under the lock `stop()` takes, and `stop()` raises a flag before
  asking for it, so at most the one send already in flight precedes it.
- Wi-Fi drop: nothing from the blimp for 0.6 s and the stream sends zero (the blimp's own 500 ms
  timeout has coasted it already); a link back within 3 s resumes, ramped over 1 s; a longer outage
  drops the command and latches.
- Blimp reboot: the telemetry's `rx_ok` going back (its `t_us` wraps every 71.6 min, so it cannot
  tell) drops the command and latches.
- B on the pad latches the bridge from the pad's thread, so a frozen window cannot drive again when
  it thaws; the window disarms on the latch and its STOP clears it.
- Any send gap of 200 ms or more while a command stands is an Events warning ("the motor stream
  paused … ms"), and the worst is kept (`gap_max_s`): criterion 1's measurement, in every session.
- The Wi-Fi Join/Leave button refuses while a command stands.
- `--race`: the window is a monitor. `set_motors` is refused in the bridge, "Motors enabled" and
  "Manual control" are disabled, the console refuses `motor`, STOP works.
- The header's **Drive** pill: OFF, FLIGHT, CONTROL, LINK LOST, RESUMING or RACE.
- `software/mock_blimp.py` gained `radio_on` and `reboot()` for the tests.
- Tests: `software/ground_station/tests/` (the bridge alone, the window with its GUI thread blocked,
  the Control page; the scratch suites of 2026-10-28 moved in, with free ports), run by
  `python tools/verify.py --gui` (about 1 min). All pass; none of this has run against the blimp.

## 5. Firmware changes and proposals

F1 to F4 and F8 were made on 2026-10-29 on branch `link-robustness`, with the team's agreement
(namontoy owns `net_link.c` and `ctl_task.c`); `mock_blimp.py` follows F2 and F3. They build
with ESP-IDF v5.5.5 and ran on the board on 2026-10-30 (`docs/history/bench-2026-10-30.md`):
F2's motor-only timeout stopped the motors 0.50 s after the last `MOTOR_SET` with the heartbeat
still on, and F4 refused `motor --failsafe 0` over the air. F1's brake-then-zero check was
skipped by the team's choice (they coast only; a brake comes only from an explicit console
command), and F3's stale drop was not exercised (no packet was lost in 582). The rest are
proposals.

| # | Change | Where | Status |
|---|---|---|---|
| F1 | Write the PWM only when a motor's value changes (`s_out[]` holds what the pins were written for; a brake, a coast or the failsafe clear it, so the next command always lands); always feed the failsafe | `cmd_motor.c` `motors_set_all` | **done** |
| F2 | `last_motor_rx_us`, set only by an accepted `MOTOR_SET` or `MOTOR_STOP`; the link timeout uses it, so a heartbeat, a capture command or another station no longer keeps a motor command alive. `link` shows the last motor command's age | `net_link.c` | **done** |
| F3 | Drop a `MOTOR_SET` whose 16-bit seq is not newer than the last motor packet's from the same sender within the timeout (signed difference ≤ 0), counted as `motor commands out of date` in `link`; a newer `MOTOR_STOP` moves the mark; `MOTOR_STOP` is obeyed from anyone, always. Per sender and per session, because a restarted host or a second station starts its own count | `net_link.c` | **done** |
| F4 | Over the air (`net_console_is_caller()`), refuse `motor --failsafe`, and `--hold` while the failsafe is off. Not done: a cap on console run time (a console run is finite and STOP still coasts it), and from increment 3, refusing motor and sensor commands while armed or autonomous | `net_console.c`, `cmd_motor.c` | **done**, the cap and the increment-3 part open |
| F5 | Telemetry and sensors to every station that sent within the last second (the access point allows two), not only the last sender | `net_link.c` | an hour |
| F6 | Telemetry (protocol version 2): `reset_reason`, `motor_age_ms` (motor packets only), the worst motor-packet gap of the last second, the stale count | `link_protocol.h`, `net_link.c`, `blimp_link.py`, `mock_blimp.py` | half a day |
| F7 | Download pacing: at most 2 chunks per link-task pass instead of up to 8; `CONFIG_LWIP_UDP_RECVMBOX_SIZE` 6 → 16 in `sdkconfig.defaults.esp32s3` | `net_link.c`, config | an hour |
| F8 | Pin the brownout level and the watchdogs in `sdkconfig.defaults.esp32s3` at the values the generated `sdkconfig` held, task watchdog still without panic | config | **done** (no change on the board) |
| F9 | The Wi-Fi password out of the repo (and out of `app.py`'s default) | `Kconfig.projbuild`, `app.py` | minutes |
| F10 | Increment 3 as planned (D5, D6), plus: the autonomous modes ignore `MOTOR_SET` (race mode's firmware half); a link loss in MANUAL zeroes without latching (the team's resume choice depends on it) | `ctl_task.c`, `net_link.c` | the increment |

Checks after flashing F1 to F4 (props off): `python software/link_check.py --timers` should
report the motors at zero about 0.5 s after the `MOTOR_SET` stops with heartbeats still on
(1 s before F2), failsafe `idle`; `link` on the console shows `motor commands out of date 0` on a
quiet bench; `motor --failsafe 0` from the window's console answers that it is refused over the
air, and works on USB; the Flight page holding a duty, a brake from the console (`motor -b`) and
then "Zero all" must leave the motors coasting, not braked (F1's trap).

## 6. Bench measurements still owed

| Measurement | Decides | Threshold |
|---|---|---|
| `python software/link_check.py --seconds 30`, then `--download 120 --timers` | the receive path (F7, a receive task) | STOP-to-coast p99 under 100 ms during the download |
| `tools/wifi_soak.py` 10 min with a download running (Linux) | the radio | no drop |
| scope the 5 V and 3.3 V rails on a 0 → 100 % step and a full reversal, all four motors, real cell | the duty slew | build it if 3.3 V dips below 3.0 V or an I2C error or brownout appears |
| scope the 8 DRV8833 inputs through power-up, USB open and a forced reset | pull-downs, and a task-watchdog panic | pull-downs if an input rises above 0.8 V; the panic only after |
| a Bench session with the motors held, laptop and Deck, watching Events for "the motor stream paused" | the separate link program | any gap of 200 ms or more: look again at moving the link out of the window's process |
| FL and FR on the scale, forward and reverse, 3 repeats | per-motor curves, `core/` maxima | — |

## 7. Not verified

Nothing in §4 has run against the blimp. Carried from the review without a check: the PX4 offboard
and ArduPilot override timeouts cited as practice, whether the DRV8833 holds its inputs low by
itself, how Qt's timers behave under the native Windows file dialog, and how long `sendto` can block
on the S3 when its transmit buffers run out.
