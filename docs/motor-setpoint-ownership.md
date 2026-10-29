# Who holds the motor command: why the GUI can stop the motors, and what to do about it

**Date:** 2026-10-28
**Branch:** `s3-wireless-link`
**Status:** the analysis (§1–§6, written on the morning of 2026-10-28) and what was done the same
day on branch `motor-ownership` (§7). Measured figures are marked; the rest is
reasoned from the code as it is today. The bench observation that started this: with the motors
driven from the Flight page, going to the Bench tab and moving the mouse or using its controls
made the motors cut and restart.

## 1. The chain as it was (until 2026-10-28; §7 has the new one)

Three timers decide whether a motor turns, and every one of them can only ever stop it:

| Where | What must happen | Or else | Value |
|---|---|---|---|
| Ground station, GUI thread (`panels.MotorsPanel`) | the panel's `QTimer` re-sends the slider setpoint to the bridge | the bridge's deadman trips | every 100 ms (`MOTOR_SEND_MS`) |
| Ground station, bridge thread (`blimp_bridge.BlimpBridge`) | the setpoint it holds is younger than `DEADMAN_S` | it sends `MOTOR_STOP` twice, drops the setpoint, logs "setpoint from the GUI went stale for 500 ms: motors released" | 0.5 s; it streams `MOTOR_SET` at 20 Hz while fresh |
| Blimp, link task (`net_link.c`) | any valid packet arrives within `CONFIG_NET_LINK_TIMEOUT_MS` | `motors_coast()`, "link lost" in the log, `link_alive` 0 in telemetry | 500 ms |
| Blimp, motor driver (`cmd_motor.c`) | a thrust command (`MOTOR_SET` or console `motor`) arrives within `CONFIG_MOTOR_FAILSAFE_MS` while any motor is driven | coast, failsafe `TRIPPED` in telemetry | 1000 ms, checked every 50 ms |

The heartbeat (`blimp_link.BlimpLink`, 10 Hz, its own thread) keeps the link alive but feeds
neither motor timer. So the only thing that keeps a motor turning is a `MOTOR_SET` every 50 ms,
and the only thing that keeps those going is a GUI-thread timer firing within every 500 ms.

That design was deliberate (`docs/ground-station-plan.md` §6): a closed tab, a frozen window, a
dead laptop and a dropped Wi-Fi all end in coasting motors within about a second. What it did not
foresee is that the GUI thread is also the thread that paints, and painting can take longer than
the margin.

## 2. What blocks the GUI thread (measured)

Offscreen on the Windows laptop, 2026-10-28, with the trace plot of the Bench page showing a
synthetic accelerometer capture:

| Capture | `on_data` replot | one repaint | one hover (mouse move + repaint) |
|---|---|---|---|
| 60 s, 3,000 records | 38 ms | 44 ms | 36 ms |
| 400 s, 20,000 records | 293 ms | 231 ms | 231 ms |

The trace plot repaints itself on every mouse move for the hover crosshair, and each repaint
rebuilds every series as a `QPainterPath` point by point in Python (`widgets.TracePlot.paintEvent`).
A hand moving over a 400 s capture queues one 230 ms repaint after another; the 100 ms setpoint
timer only runs between them, and the bridge's 500 ms margin is gone after three. The buffer went
from 120 s to 400 s today, which is why this grew from a nuisance into the behaviour seen. The same
path runs when the group combo or the time window changes, and once more when a download
finishes. A 60 s capture stays well inside the margin, which is why it did not show earlier.

Other GUI-thread work that can take long enough: the window being resized or moved (every panel
repaints, the sensor sparklines at 30 Hz included), and anything that blocks the Qt event loop,
of which there is none in the code today (the capture, console, Wi-Fi and camera all run on
their own threads).

What this looks like on the blimp: `MOTOR_STOP` arrives, the motors coast, the readback bars drop
to zero; a few hundred milliseconds later the next setpoint refresh restarts the 20 Hz stream and
the motors spin back up from zero, with the DRV8833s stepping the duty in one go. That is the cut
and restart.

## 3. Why it is a design fault and not only a performance bug

- The vehicle's state depends on the rendering load of a laptop. Any future panel that paints a
  lot (traces, a camera frame, a map) reintroduces it, and so does a slower laptop or the Steam
  Deck.
- The restart is a step: the DRV8833 goes from coasting to the full commanded duty in one tick,
  which is the worst thing for the shared 5 V rail (`cmd_motor.c` header: break-away current, the
  step-up sags, the sensors brown out).
- The control-loop plan's D5 makes it worse before it makes it better: once `MOTOR_STOP` latches
  `KILLED`, a GUI stall will kill the vehicle and demand a deliberate re-arm. That is the right
  semantics for a real stop and the wrong one for a 600 ms repaint. So the stall path has to be
  fixed before increment 3 lands.
- "Hold the last command on the MCU" is what the plan already intends for `MANUAL`: the loop
  applies the latest link `cmd[4]` every tick (D5), and the GUI's job is to say what the command
  is and to prove the link is alive, not to repaint it 20 times a second from the thread that also
  draws plots.

## 4. Options

**A. Make the GUI fast enough.** Cache each series' `QPainterPath` in `TracePlot.set_data` /
`set_window` and only redraw the crosshair on hover (a painter path draws in a few ms); decode and
convert captures on the capture thread (already so for `to_units`; the list building in `replot`
can move there too). Necessary for a usable Bench page with 400 s captures regardless, and cheap.
Not sufficient: it leaves the vehicle's safety margin tied to UI latency.

**B. The bridge thread owns the standing command (ground side).** The window tells the bridge
*armed / setpoint / stop* as events; the bridge thread streams `MOTOR_SET` at 20 Hz for as long as
it is armed, on its own clock, and nothing on the GUI thread has to tick. The GUI-thread watchdog
stays but as a last resort with a margin no repaint reaches: the GUI thread pings the bridge every
100 ms and the bridge stops only after about 3 s without a ping, logging it as `crit`. A frozen
window then still ends in coasting motors, three seconds later instead of half a second, while the
firmware's two timers still catch a dead laptop or a dropped radio within a second. STOP gets
paths that do not need the GUI thread: the gamepad reader's thread can call the bridge's stop
directly (B on the Deck), as the Ctrl-C handler and window close already do. No firmware change.

**C. The blimp holds the standing command (what was asked for).** `MOTOR_SET` sets the command and
the control task applies it every tick; liveness comes from any packet, so the 10 Hz heartbeat
from the link thread keeps it alive and the GUI need only send changes. This is D5/D6 of the
control-loop plan with one amendment: the motor failsafe's "a thrust command within 1 s" must
accept the heartbeat as a feed in `MANUAL` (or the bridge re-sends the standing command at a low
rate, 5 Hz, from its own thread, which also covers a lost datagram). Link loss still coasts within
500 ms, the kill still latches, and a stall on the ground changes nothing on the vehicle. Needs
increment 3 (`MODE_SET`, the latches, the loop owning the motors) to be unambiguous, because a
standing command on the vehicle must be paired with an explicit arm and a sticky kill.

**D. Ramp on the vehicle.** Whatever the ownership, the loop's output stage should slew the duty
(full scale in a few hundred milliseconds, a tunable in `params`) so a restart, an arm or a hand on
a stick never steps the DRV8833s. The Control page already ramps on the ground; the vehicle is the
right place because every source goes through it.

## 5. Recommendation

1. Now, ground side only: A and B. B is a day's work in `blimp_bridge.py`, `panels.py` and
   `control.py` (the panels already route everything through the bridge), and it removes the
   coupling for both the Motors panel and the Control page. A removes the stalls themselves. The
   0.5 s deadman becomes a 3 s watchdog with a `crit` event, and the setpoint refresh leaves the
   GUI thread.
2. With increment 3: C and D, as amendments to D5/D6 (heartbeat as liveness in `MANUAL`; a 5 Hz
   re-send from the bridge; a duty slew in the output stage), and `mock_blimp.py` models the hold.
3. Keep the firmware's 500 ms link timeout and 1 s motor failsafe as they are: they are the
   protection against the cases the ground cannot see, and they never fired in the observed
   behaviour.

What is lost: a window that is frozen but whose process is alive keeps the motors going for up to
3 s instead of 0.5 s. What is gained: the motors do what the operator last asked until the
operator, a link loss or a real fault says otherwise, which is what a control should do.

## 6. Tests to settle the three causes

All with the propellers off and a low duty held from the Flight page. Read the Events tab and the
pills before and after each.

**T1, GUI stall (cause 1).** Bench tab, no capture downloaded: move the mouse over the empty plot
for ten seconds. Then download a capture of 60 s, then one of 300 s, and repeat. Expected today:
nothing at 0 and 60 s; at 300 s the readback drops to zero and the Events tab shows "setpoint from
the GUI went stale for 500 ms: motors released". After B: never. After A alone: not at 300 s on
this laptop; still possible on a slower one.

**T2, link congestion during a download (cause 2).** Motors held, start a 120 s capture, wait for
it, download it while the motors run. Expected: `MOTOR_SET` keeps flowing (the host sends it from
the bridge thread, the download from the capture thread), but the S3's link task is busy building
and sending chunks. Pass if the FAILSAFE pill never reads TRIPPED and `link` on the console shows
no "link lost" in the USB log; fail otherwise, and then the cure is on the S3 (a receive task of
its own, or chunks sent with a yield between them), not on the ground. Also note the LINK pill's
age during the burst. The instrument for this, with the station closed, is
`python software/link_check.py --download 120 --timers` (2026-10-29): it holds a small duty from
its own thread, records the capture, downloads it while still holding, and prints the command
latency, the loss as the blimp counts it and the longest the blimp waited for a packet of ours,
for the hold and for the download separately; a wait near 500 ms during the download is this
cause, measured.

**T3, supply coupling (cause 3).** Motors held at a fixed duty with a current meter in the 5 V rail
(or the battery divider once increment 2 records `bat_mv` in the capture). Send `help` from the
console, then run T2's download. Pass if the rail and the motor sound stay put while the readback
is constant. A dip that coincides with the S3 transmitting, with the readback unchanged, is the
rail, and the cure is electrical (a separate supply or a bulk capacitor at the step-up output),
not software. The IMU dropping out at the same instants is the same cause.

**T4, build check.** Confirm the S3 runs a build with the Wi-Fi drop fix (`9d1cdca` or later, the
boot log's `git describe`). Before it, the access point dropped the laptop about every 14 s, and
every drop is a 500 ms link-timeout coast followed by a restart, unrelated to the window.

T1 is the one that reproduces the report; run it first. T2 and T3 are worth one bench session each
because the download and the console are things the operator will do with the motors on, and
cause 3 is the one no software change can fix.

## 7. What was done (2026-10-28, branch `motor-ownership`)

Options A and B of §4, as §5 recommended; C and D stay with control-loop plan increment 3.

**B, the bridge owns the standing command** (`blimp_bridge.py`, `station.py`, `panels.py`,
`control.py`, `app.py`, `gamepad.py`):

- `BlimpBridge.set_motors()` sets the standing command, and the bridge's thread repeats it to the
  blimp at 20 Hz for as long as it stands, on its own clock. The Motors panel sends on change
  only (a slider, the limit, Zero, Enable); its 100 ms refresh timer is gone. The Control page
  still sends every 50 ms while armed, because its ramp changes the command every tick; a stall
  leaves the last command standing.
- Liveness is a ping: the station's 100 ms tick calls `bridge.ping()`. If the pings stop for
  `WATCHDOG_S` = 3 s while a command stands, the bridge sends `MOTOR_STOP` twice, drops the
  command, logs a `crit` event and latches: `set_motors()` is refused until `stop()`. The window
  learns of the trip from the snapshot (`tel['tripped']`) when its thread comes back and makes it
  the operator's STOP (`MainWindow.on_tel`): controls off, latch cleared, button flashed. A
  thawed window therefore never restarts the motors by itself.
- B on the gamepad reaches `bridge.stop()` from the pad reader's thread (`Gamepad(on_press=…)`,
  `Station._pad_press`), GUI thread or not; the Control page still does the window's part on its
  own tick.
- The firmware's timers are untouched: 500 ms link timeout, 1 s motor failsafe.

**A, the trace plot** (`widgets.TracePlot`): one `QPainterPath` per series, built once per data,
window and size and cached; a series denser than two samples per pixel column keeps only each
column's lowest and highest sample, in time order (the line covers the same pixels; on a
20,000-sample series with a gap and a spike the decimated and the full path's bounding boxes agree
within a pixel); and the pen is 1 px, which Qt strokes on its cosmetic fast path. The pen was most
of it: a 1.5 px pen with round joins goes through the general stroker.

Measured offscreen on the Windows laptop (a 1200 × 420 plot, three series, painted into an RGB32
image like the window's backing store; medians of 10):

| | 60 s, 3,000 samples × 3 | 400 s, 20,000 samples × 3 |
|---|---|---|
| hover repaint, old pen, paths already cached | 161 ms | 1,095 ms |
| hover repaint, 1 px pen and decimation | 2.3 ms | 3.4 ms |
| first paint after a download (the paths are built once) | 44 ms | 83 ms |
| one `drawPath` of 20,000 points, 1.5 px pen with round joins | | 333 ms |
| the same path, 1 px pen | | 4.6 ms |

§2's 231 ms was measured in the window on a narrower plot; the figures here are the probe's and
comparable only with each other. Before the cache the paths were rebuilt on every repaint, which
added about 110 ms to each of the old figures on the 400 s capture.

**Checked offscreen** (`--demo`, 30 checks, 2026-10-28): a command stands 1.5 s with no refresh at
all; with no pings the bridge releases 3.1 s after the last sign of life, logs `crit`, refuses
`set_motors()` and nothing restarts; STOP clears the latch; pings every 100 ms keep a command
standing for 4 s. In the window: a GUI thread blocked for 1.2 s with a duty held leaves the motors
running (the old chain cut them at 0.5 s); one blocked for 3.6 s ends with the motors released,
"Motors enabled" off and the latch cleared; the Control page's command survives a 1.2 s stall and
ramps to zero on key-up; B from the reader's thread stops the blimp while the GUI thread is
blocked. The earlier Control-page checks still pass. Not yet run on the vehicle: T1 of §6 is the
confirmation, and T2 to T4 are still worth their sessions.

**For the operator** nothing changes in the hands: the Motors and Control hints now say that a
command stands until changed and that a window frozen for 3 s releases the motors. What was traded:
a frozen window keeps the motors going for up to 3 s instead of 0.5 s, as §5 accepted.
