"""The assisted flight against the in-process demo blimp: the setpoints' pure
parts, 'hold' as the firmware prints it, then whole flights: the preparation
and what refuses it, the sticks, the landing, and every other end (STOP, the
kills, a frozen window, the ToF, a link that drops, a reboot, a window that
died). assist.py's docstring has the rules; docs/assisted-control.md why."""
import os
import tempfile
import threading
import time

import harness as H
from harness import check, near, wait_for

import blimp_link as bl
from software.ground_station import assist as Ax
from software.ground_station import blimp_bridge as BB

# ------------------------------------------------------------ pure parts
T = Ax.Tuning()
check(Ax.move_height(1.0, 0.0, 0.1, 1.0, T) == 1.0, 'height: no stick, no move')
check(near(Ax.move_height(1.0, 1.0, 1.0, 1.0, T), 1.0 + T.climb_rate_mps), 'height: full stick for 1 s is the climb rate')
check(near(Ax.move_height(1.0, -0.5, 1.0, 1.0, T), 1.0 - 0.5 * T.climb_rate_mps), 'height: half stick down')
check(near(Ax.move_height(1.29, 1.0, 1.0, 1.0, T), 1.0 + T.height_lead_m),
      'height: the stick cannot run the setpoint past the lead over the blimp')
check(Ax.move_height(2.0, 1.0, 0.1, 1.0, T) == 2.0,
      'height: a setpoint already past the lead (the number box) is not pulled back by the stick')
check(Ax.move_height(0.31, -1.0, 1.0, 0.31, T) == Ax.HEIGHT_MIN_M and Ax.move_height(2.29, 1.0, 1.0, 2.29, T) == Ax.HEIGHT_MAX_M,
      'height: clamped to the flying range')
check(near(Ax.move_heading(0.0, 1.0, 0.5, 0.0, T), -0.5 * T.turn_rate_dps), 'heading: stick right turns clockwise')
check(near(Ax.move_heading(0.0, -1.0, 0.5, 0.0, T), 0.5 * T.turn_rate_dps), 'heading: stick left turns counter-clockwise')
check(near(Ax.move_heading(0.0, -1.0, 5.0, 0.0, T), T.heading_lead_deg), 'heading: the lead over the nose is limited')
check(near(Ax.move_heading(175.0, -1.0, 0.5, 170.0, T), -175.0), 'heading: wraps through 180')
check(near(Ax.stop_heading(10.0, 20.0, T), 10.0 + 20.0 * T.stop_lead_s), 'heading: let go, just ahead of the turn')

# 'hold' as ctl_task.c prints it, on the blimp's own formats
text = ('mode HOLD, armed\nestimator: converged (still 143 of 60 s)\n'
        'heading: estimate -12.5 deg (0 = the nose at arming), rate +3.1 deg/s; held at -10.0 deg, error +2.5 deg\n'
        "camera: no target ('cam' for the link)\n"
        'altitude: estimate 1.02 m (ToF); held at 1.20 m, moving at 0.12 m/s, now at 1.08 m\nsurge: -0.6 gf\n'
        'output: fx -0.60 gf  fz 5.11 gf  tz 0.210 mN m;  throttle FL 0.100 RL 0.800 FR 0.120 RR 0.800\n')
st = Ax.parse_hold(text)
check(st and st['mode'] == 'HOLD' and st['armed'] and st['converged'] and st['still_s'] == 143,
      'parse: mode, armed, estimator: %s' % st)
check(st['yaw_deg'] == -12.5 and st['rate_dps'] == 3.1 and st['alt_m'] == 1.02 and st['alt_sp_m'] == 1.08,
      "parse: yaw, rate, height and core/'s moving setpoint")
st = Ax.parse_hold('mode MANUAL\nestimator: converging (still 12 of 60 s)\nheading: estimate nan deg (0 = the nose at '
                   'arming), rate nan deg/s; not held\naltitude: estimate nan m (no source); not held\n')
check(st and not st['converged'] and st['still_s'] == 12 and st['yaw_deg'] is None and st['alt_m'] is None,
      'parse: nan fields are None')
st = Ax.parse_hold('mode HOLD, NOT armed: the estimator is converging, motors at zero\n'
                   'estimator: converging (still 3 of 60 s)\naltitude: estimate 0.08 m (ToF); held at 1.00 m\n')
check(st and not st['armed'] and st['alt_sp_m'] == 1.0, 'parse: NOT armed; a fixed setpoint')
st = Ax.parse_hold(text)
check(st['held_heading_deg'] == -10.0 and st['held_alt_m'] == 1.2 and st['surge_gf'] == -0.6,
      'parse: what the blimp says it holds')
check(Ax.parse_hold('held: heading on, altitude on, surge -0.6 gf, follow off\n') is None,
      "parse: another command's late reply is not a status")
params = Ax.read_bench_params()
check(len(params) >= 17 and ('front_reverse_ratio', '0.74') in params, 'bench_params.txt reads: %d values' % len(params))

# ----------------------------------------------------------------- the flights
events = []
port, console_port = H.free_ports()
b = BB.DemoBlimp(lambda lvl, txt: events.append((lvl, txt)), port=port, console_port=console_port).start()
mock = b.mock
check(b.link.wait_alive(3.0), 'demo blimp alive')
a = b.assist
# The same flight at the toy's speed: a real landing from 1 m takes about 15 s.
a.tuning = Ax.Tuning(climb_rate_mps=0.4, land_rate_mps=0.25, final_rate_mps=0.2)
a.marker = os.path.join(tempfile.mkdtemp(prefix='assist-test-'), 'marker')


class Pinger:
    """The station's tick, without a window."""

    def __init__(self):
        self.on = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.on:
            b.ping()
            time.sleep(0.1)


pinger = Pinger()


def said(text, level=None):
    return any(text in t and (level is None or lvl == level) for lvl, t in events)


def on_floor():
    """Each flight starts as a real one: on the floor, the minute of stillness done."""
    mock.hold_mode, mock.alt_sp = 'MANUAL', None
    mock.hold = {'heading': None, 'altitude': None, 'rate': 0.0, 'surge': 0.0}
    mock.height_m, mock.vz, mock.tof_blind = 0.07, 0.0, False
    mock.yaw_rate = 0.0
    mock.motor = [0.0] * 4
    mock.still_s = 60.0
    time.sleep(0.15)


def fly(height=1.0, trim=-0.6, full=3.0):
    on_floor()
    events.clear()
    why = b.takeoff(height, trim, full)
    check(why is None, 'take-off accepted: %s' % why)
    check(wait_for(lambda: a.snapshot()['state'] == 'flying', 6.0), 'preparation done, flying')
    check(wait_for(lambda: abs(mock.height_m - height) < 0.1, 8.0), 'at %.1f m: %.2f m' % (height, mock.height_m))


def idle(t=25.0):
    return wait_for(lambda: a.snapshot()['state'] == 'idle', t)


def landed():
    """A landing that reached the floor: a timed-out one is idle as well."""
    return idle() and said('landed', 'ok') and mock.height_m < 0.12 and not any(mock.motor)


# what refuses a flight, nothing started
on_floor()
mock.height_m = 1.0
check(b.takeoff(1.0, -0.6, 3.0) is None and idle(6.0), 'off the floor: the flight ends in its preparation')
check(said('does not see the floor', 'warn') and mock.hold_mode == 'MANUAL', 'off the floor: refused, no hold started')
on_floor()
mock.set_all([0.2, 0, 0, 0])
time.sleep(0.15)
check(b.takeoff(1.0, -0.6, 3.0) is None and idle(3.0) and said('the motors are driven', 'warn'),
      'a motor driven: refused')
mock.coast()

# the minute of stillness: shown, waited for, and it can be called off
on_floor()
mock.still_s = 30.0
check(b.takeoff(1.0, -0.6, 3.0) is None, 'take-off with the estimator at 30 s')
check(wait_for(lambda: 'still' in a.snapshot()['detail'], 4.0), 'the wait shows: %s' % a.snapshot()['detail'])
a.land()
check(idle(3.0) and said('called off') and mock.hold_mode == 'MANUAL', 'Land during the wait calls it off, no hold')
on_floor()
mock.still_s = 58.5
check(b.takeoff(1.0, -0.6, 3.0) is None, 'take-off 1.5 s before the estimator is ready')
check(wait_for(lambda: a.snapshot()['state'] == 'flying', 6.0) and mock.hold_mode == 'HOLD', 'it waits, then starts')
a.land()
check(landed(), 'and lands')

# a whole flight
events.clear()
fly(1.0)
check(mock.params['motor_max_thrust_n'] == 0.033 and mock.params['alt.kp'] == 0.12
      and mock.params['front_reverse_ratio'] == 0.74, 'the bench parameters are on the blimp')
check(mock.params['gyro_bias_tau_s'] == 600.0, "core/'s bias learning held off for the flight")
check(os.path.exists(a.marker), 'the marker says a flight is up')
check(near(mock.hold['surge'], -0.6) and mock.hold['heading'] == 0.0 and near(mock.hold['altitude'], 1.0),
      'the trim, heading 0, the height: %s' % mock.hold)
check(b.standing and not b.set_motors([0.5, 0, 0, 0]), 'set_motors is refused in the flight')
s = a.snapshot()
check(s['state'] == 'flying' and s['tof_m'] is not None and s['yaw_deg'] is not None, 'the snapshot fills: %s' % s)

# the height: the number box, the stick, a D-pad step
a.set_height(1.5)
check(wait_for(lambda: near(mock.hold['altitude'], 1.5, 0.001), 1.0), 'height: a new setpoint goes out')
check(wait_for(lambda: abs(mock.height_m - 1.5) < 0.1, 4.0), 'height: the blimp follows: %.2f m' % mock.height_m)
a.step_height(-0.1)
check(wait_for(lambda: near(mock.hold['altitude'], 1.4, 0.001), 1.0), 'height: a D-pad step')
a.set_sticks(0.0, 0.0, 1.0)
time.sleep(0.5)
a.set_sticks(0.0, 0.0, 0.0)
time.sleep(0.3)
h = mock.hold['altitude']
check(1.45 < h <= 1.75, 'height: the stick moved it: %.2f m' % h)
time.sleep(0.5)
check(near(mock.hold['altitude'], h, 0.001), 'height: let go, it stays')

# the heading: it follows the stick, the lead is limited, and it stops when let go
a.set_sticks(0.0, -1.0, 0.0)        # left: counter-clockwise, heading up
time.sleep(2.0)
lead = Ax.wrap180(mock.hold['heading'] - mock.yaw_est())
check(mock.hold['heading'] > 5.0 and lead <= a.tuning.heading_lead_deg + 8.0,
      'heading: turning left, %.1f deg, %.1f ahead of the nose' % (mock.hold['heading'], lead))
a.set_sticks(0.0, 0.0, 0.0)
time.sleep(1.5)
held = mock.hold['heading']
check(abs(Ax.wrap180(held - mock.yaw_est())) < 8.0, 'heading: let go, the setpoint is at the nose (%.1f, nose %.1f)'
      % (held, mock.yaw_est()))
time.sleep(0.5)
check(near(mock.hold['heading'], held, 0.01), 'heading: and it holds there')

# the surge: the trim plus the stick
a.set_sticks(1.0, 0.0, 0.0)
check(wait_for(lambda: near(mock.hold['surge'], 2.4, 0.01), 1.0), 'surge: full stick is the trim plus the full push: %.2f'
      % mock.hold['surge'])
a.set_sticks(-0.5, 0.0, 0.0)
check(wait_for(lambda: near(mock.hold['surge'], -2.1, 0.01), 1.0), 'surge: half back: %.2f' % mock.hold['surge'])
a.set_sticks(0.0, 0.0, 0.0)
check(wait_for(lambda: near(mock.hold['surge'], -0.6, 0.01), 1.0), 'surge: let go, the trim alone')
a.tune(trim_gf=-0.8)
check(wait_for(lambda: near(mock.hold['surge'], -0.8, 0.01), 1.0), 'surge: the trim changed in flight')
a.tune(trim_gf=-0.6)

# the landing
events.clear()
a.land()
check(wait_for(lambda: a.snapshot()['state'] == 'landing', 1.0), 'land: landing')
a.set_sticks(1.0, 1.0, 1.0)
time.sleep(0.3)
check(near(mock.hold['surge'], -0.6, 0.01), 'land: the sticks are out of it')
check(landed() and mock.hold_mode == 'MANUAL', 'land: on the floor, motors off, hold off: %.2f m' % mock.height_m)
check(mock.params['gyro_bias_tau_s'] == 20.0 and not os.path.exists(a.marker), 'land: bias learning back, no marker')
check(b.set_motors([0.0] * 4), 'land: manual control is free again')
b.stop()

# STOP: at once, in the air, with nothing following it
fly(1.0)
t0 = time.perf_counter()
b.stop()
check(mock.hold_mode in ('KILLED', 'MANUAL') and not any(mock.motor), 'STOP: the motors coast at once')
check(idle(3.0), 'STOP: the flight ends')
check(mock.hold_mode == 'MANUAL' and mock.params['gyro_bias_tau_s'] == 20.0 and not os.path.exists(a.marker),
      "STOP: 'hold off' cleared KILLED, bias learning back, no marker")
check(mock.height_m > 0.5, 'STOP: no landing: it stopped where it was (%.2f m)' % mock.height_m)

# STOP from the pad's thread, which latches
fly(1.0)
b.stop(False, latch=True)
check(idle(3.0) and mock.hold_mode == 'MANUAL', 'pad B: the flight ends')
check(b.takeoff(1.0, -0.6, 3.0) is not None, 'pad B: no take-off while the latch stands')
b.stop()

# a stop from somewhere else: the console's own 'motor -s', or another station
fly(1.0)
b.console.run('motor -s', timeout=2.0, idle=1.0)
check(idle(3.0) and said('left the hold', 'crit'), 'a stop from elsewhere: seen at the next poll, flight over')

# the kills: spin and tilt do not land
fly(1.0)
t_end = time.monotonic() + 0.6
while time.monotonic() < t_end and a.snapshot()['state'] == 'flying':
    mock.yaw_rate = 90.0
    time.sleep(0.005)
check(idle(3.0) and said('KILLED', 'crit') and 'yaw' in [t for _, t in events if 'KILLED' in t][0],
      'kill: a spin coasts the motors: %s' % [t for _, t in events if 'KILLED' in t])
check(mock.hold_mode == 'MANUAL' and not any(mock.motor) and mock.height_m > 0.5, 'kill: no landing')

# ... and a console line stuck on a weak link does not hold a kill up
fly(1.0)
real_run = a.console.run


def slow_run(line, **kw):
    time.sleep(0.35)
    return real_run(line, **kw)


a.console.run = slow_run
time.sleep(0.1)
t_end = time.monotonic() + 0.25
while time.monotonic() < t_end:
    mock.yaw_rate = 90.0
    time.sleep(0.005)
check(mock.hold_mode == 'KILLED' or not any(mock.motor), 'kill: within 0.25 s, with the console taking 0.35 s a line')
a.console.run = real_run
check(idle(4.0) and said('KILLED', 'crit'), 'kill: and the flight ends')

# A STOP before the hold's first tick latches nothing on the blimp (cmd_motor.c,
# stop_while_owned()): the flight must not send another setpoint, and 'hold off' ends it.
lines = []


def stop_at_start(line, **kw):
    lines.append(line)
    out = real_run(line, **kw)
    if line == 'hold heading 0':
        a.abort('info', 'assisted flight stopped')
    return out


on_floor()
events.clear()
a.console.run = stop_at_start
check(b.takeoff(1.0, -0.6, 3.0) is None and idle(6.0), 'stop as the hold starts: the flight ends')
a.console.run = real_run
check('hold heading 0' in lines and not any(l.startswith(('hold surge', 'hold altitude')) for l in lines),
      'stop as the hold starts: no setpoint follows it')
check(lines[-2:][0] == 'hold off' or 'hold off' in lines, "stop as the hold starts: 'hold off' ends it")
check(mock.hold_mode == 'MANUAL' and not any(mock.motor), 'stop as the hold starts: nothing runs')

# A setpoint lost behind another line's late reply: the poll sees it and sends it again.
fly(1.0)
mock.hold['surge'] = 0.0
check(wait_for(lambda: near(mock.hold['surge'], -0.6, 0.01), 1.5), 'lost: the surge goes out again')
mock.hold['heading'] = 40.0
check(wait_for(lambda: abs(mock.hold['heading']) < 1.0, 1.5), 'lost: the heading goes out again')
mock.hold['altitude'] = 1.4
check(wait_for(lambda: near(mock.hold['altitude'], 1.0, 0.001), 1.5), 'lost: the height goes out again')

# A landing's first act is the height, ahead of any poll, and it survives a lost line too.
lines = []


def log_run(line, **kw):
    lines.append(line)
    return real_run(line, **kw)


time.sleep(0.5)
a.console.run = log_run
a.land()
check(wait_for(lambda: bool(lines), 1.0) and lines[0].startswith('hold altitude 0.30'),
      "land: the height goes first: %s" % lines[:1])
check(wait_for(lambda: near(mock.hold['altitude'], 0.3, 0.001), 1.0), 'land: on its way down')
mock.hold['altitude'] = 1.0
check(wait_for(lambda: near(mock.hold['altitude'], 0.3, 0.001), 1.5), 'land: a lost landing setpoint goes out again')
a.console.run = real_run
check(landed(), 'and lands')

# 'hold off' believed only on its own words: another line's reply does not end the flight's cleanup
fly(1.0)
stale = [2]


def stale_off(line, **kw):
    if line == 'hold off' and stale[0]:
        stale[0] -= 1
        return 'held: heading on, altitude on, surge -0.6 gf, follow off\n'
    return real_run(line, **kw)


a.console.run = stale_off
b.stop()
check(idle(3.0) and stale[0] == 0, 'hold off: a late reply is not taken for it')
a.console.run = real_run
check(mock.hold_mode == 'MANUAL' and not os.path.exists(a.marker), 'hold off: asked again until it says so')

# check_leftover() keeps out of a flight, whatever the marker says
fly(1.0)
a._clear_leftover()
check(mock.hold_mode == 'HOLD' and os.path.exists(a.marker), 'leftover: a flight under way is left alone')
a.land()
check(landed(), 'and lands')

# an event callback that fails must not leave the window armed on a dead flight
on_floor()
keep = a.on_event


def boom(level, text):
    if 'does not see the floor' in text:
        raise RuntimeError('a broken callback')
    keep(level, text)


threading.excepthook = lambda args: None
a.on_event = boom
mock.height_m = 1.0
check(b.takeoff(1.0, -0.6, 3.0) is None and idle(6.0), 'a callback that raises: the state still goes back to idle')
a.on_event = keep
threading.excepthook = threading.__excepthook__

# landing triggers: a frozen window, the ToF high, the ToF gone, a pad gone
fly(1.0)
pinger.on = False
check(wait_for(lambda: a.snapshot()['state'] == 'landing', 5.0) and said('stopped responding'),
      'a window frozen for 3 s: it lands')
check(landed(), 'and lands, with no window')
pinger = Pinger()

fly(1.0)
mock.height_m = 1.5
check(wait_for(lambda: a.snapshot()['state'] == 'landing', 2.0) and said('over the setpoint'),
      'the ToF over the setpoint (a desk passed underneath): it lands')
check(landed(), 'and lands')

fly(1.0)
a.set_height(2.0)
time.sleep(0.5)
a.set_height(0.5)
time.sleep(1.0)
check(a.snapshot()['state'] == 'flying', 'a setpoint coming down is not over the setpoint')
a.land()
check(landed(), 'and lands')

fly(1.0)
mock.tof_blind = True
check(wait_for(lambda: a.snapshot()['state'] == 'landing', 3.0) and said('silent'), 'the ToF silent for 1 s: it lands')
mock.tof_blind = False
check(landed(), 'and lands')

# the link: a short drop carries on, a long one lands
fly(1.0)
mock.radio_on = False
time.sleep(0.9)
check(a.snapshot()['link_lost'], 'link: a drop shows')
mock.radio_on = True
check(wait_for(lambda: said('the hold goes on', 'ok'), 3.0) and a.snapshot()['state'] == 'flying',
      'link: back within 3 s, the flight goes on')
a.land()
check(landed(), 'and lands')

fly(1.0)
mock.radio_on = False
time.sleep(3.6)
check(a.snapshot()['state'] == 'flying', 'link: nothing to do while it is gone')
mock.radio_on = True
check(wait_for(lambda: a.snapshot()['state'] == 'landing', 3.0) and said('the link was gone'),
      'link: gone more than 3 s, it lands when it returns')
check(landed(), 'and lands')

# core/'s watchdog in the mock: the motors stopped during the drop
fly(1.0)
mock.radio_on = False
time.sleep(1.5)
check(not any(mock.motor), "link: core/'s watchdog stopped the motors after 1 s")
mock.radio_on = True
check(wait_for(lambda: mock.motor[1] > 0.0, 2.0), 'link: and the hold takes them up again')
a.land()
check(landed(), 'and lands')

# a reboot in the air
fly(1.0)
mock.reboot()
check(idle(4.0) and said('the blimp restarted', 'crit'), 'reboot: the flight ends: %s' % [t for _, t in events][-2:])
check(wait_for(lambda: b.link.alive, 3.0), 'reboot: the link comes back')

# a window that died in a flight: the next one ends the hold it left
on_floor()
mock.hold_mode, mock.hold = 'HOLD', {'heading': 0.0, 'altitude': 1.0, 'rate': 0.12, 'surge': -0.6}
open(a.marker, 'w').close()
events.clear()
a.check_leftover()
check(wait_for(lambda: mock.hold_mode == 'MANUAL', 3.0) and said('window that died', 'crit'), 'leftover: ended')
check(wait_for(lambda: not os.path.exists(a.marker), 2.0), 'leftover: the marker gone')

# a script's hold, with no marker, is left alone
on_floor()
mock.hold_mode, mock.hold = 'HOLD', {'heading': 0.0, 'altitude': 1.0, 'rate': 0.12, 'surge': 0.0}
a.check_leftover()
time.sleep(0.5)
check(mock.hold_mode == 'HOLD', "no marker: a script's hold is left alone")
on_floor()

# guards off (the manual race of 2026-11-05): a blind ToF does not land it, a
# spin does not kill it; Land still lands it
on_floor()
events.clear()
check(b.takeoff(1.0, -0.6, 3.0, guards=False) is None, 'guards off: accepted')
check(wait_for(lambda: a.snapshot()['state'] == 'flying', 6.0) and said('guards off', 'warn')
      and not a.snapshot()['guards'], 'guards off: flying, said so, in the snapshot')
mock.tof_blind = True
time.sleep(2.0)
check(a.snapshot()['state'] == 'flying', 'guards off: a blind ToF does not land it')
mock.tof_blind = False
mock.yaw_rate = 200.0
time.sleep(0.6)
check(a.snapshot()['state'] == 'flying' and mock.hold_mode == 'HOLD', 'guards off: a spin does not kill it')
mock.yaw_rate = 0.0
a.land()
check(landed(), 'guards off: Land still lands it')

# the window's close ends a flight
fly(1.0)
pinger.on = False
b.close()
check(mock.hold_mode == 'MANUAL' and not any(mock.motor) and not os.path.exists(a.marker),
      'close: hold off, the motors coast, no marker')

# a firmware that does not answer: nothing starts
port2, console2 = H.free_ports()
ev2 = []


class DeadConsole:
    def run(self, line, timeout=0, idle=0, echo=None):
        raise TimeoutError('no answer')

    def flush(self):
        pass

    def close(self):
        pass


r = BB.DemoBlimp(lambda lvl, txt: ev2.append((lvl, txt)), port=port2, console_port=console2).start()
check(r.link.wait_alive(3.0), 'second demo blimp alive')
r.assist.console = DeadConsole()
r.assist.marker = os.path.join(tempfile.mkdtemp(prefix='assist-test-'), 'marker')
check(r.takeoff(1.0, -0.6, 3.0) is None and wait_for(lambda: not r.assist.active, 8.0), 'a silent console ends the attempt')
check(any('no answer' in t for _, t in ev2) and r.mock.hold_mode == 'MANUAL', 'a silent console: said, and nothing started')
r.close()

# race mode: no flight at all
port3, console3 = H.free_ports()
r = BB.DemoBlimp(lambda lvl, txt: None, port=port3, console_port=console3, race=True).start()
check(r.link.wait_alive(3.0), 'race blimp alive')
check('race' in (r.takeoff(1.0, -0.6, 3.0) or ''), 'race: refused')
r.close()

raise SystemExit(H.finish())
