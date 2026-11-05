"""Assisted flight: the sticks move setpoints, and core/'s loops fly them.

In the Control page's manual mode the operator is the height loop: the rear
pair sits near its ceiling just to hover (a blimp about 6 gf heavy), so the
heave stick is held near full for the whole flight, and every dip is corrected
by thumb against a vehicle that answers seconds later. Assisted mode gives that
job to the loops that already fly the bench tests. The firmware's hold
('hold altitude', 'hold heading', 'hold surge') runs core/'s height and heading
loops on setpoints from the ground; square_test.py, target_test.py and
course_test.py sequence those setpoints on a clock or on what the camera sees,
and this does the same from a stick:

- height: the stick moves the setpoint at up to the climb rate, a number box or
  the D-pad steps it, and core/ ramps toward it. Let go and it holds;
- heading: the stick moves the setpoint at up to the turn rate, and let go it
  holds. The heading loop's damping acts on the yaw rate, so a moving setpoint
  is followed about kd / kp = 1.7 s behind: the setpoint may lead the nose by
  HEADING_LEAD_DEG at most, and on release it comes back to just ahead of the
  nose (stop_heading()), or the turn would run on by the whole lead;
- surge: open loop, as it is in the hold: the trim (flight-tests.md: without it
  the blimp climbs at 45 deg and drifts metres forward) plus the stick. Nothing
  on board measures speed, so a brake is the operator pulling the stick back.

No control logic lives here (AGENTS.md): the loops are core/'s, and this shapes
a stick into a setpoint and runs the checks a flight script runs. The console
carries the setpoints, not a new packet, so the blimp needs no reflash: every
command used here has flown (docs/assisted-control.md has the reasoning and
what a firmware version would add).

One thread per flight, the bridge's way of owning a standing command
(blimp_bridge.py): the window only states what the sticks say (set_sticks()),
and everything the blimp is told comes from here, so a window that freezes
cannot leave a push standing. The flight's rules are docs/flight-tests.md's:

- before the motors: the bench parameters (a reboot restores params.c's
  placeholders, with which the blimp does not leave the floor), the driver's
  limits, the gondola still for 60 s, the ToF seeing the floor, the gyro's bias
  learning held off for the flight;
- every end is a landing: Land, the ToF over the setpoint by OVER_HEIGHT_M, the
  ToF silent, a window frozen for 3 s, a gamepad that goes away, a link back
  after more than 3 s;
- two ends are not: the spin and tilt kills (a controller misbehaving cannot be
  trusted to land) and STOP, the operator's. Both coast the motors in the air,
  and MOTOR_STOP latches KILLED on the blimp, so a setpoint still on its way
  cannot start them again. The kills run on the link's receive thread, on every
  SENSORS packet, so a console line waiting on a weak link cannot delay them;
- a link lost for more than 1 s stops the motors on the blimp (core/'s
  watchdog), which nothing here can change; with the link back the hold resumes
  from where the blimp is, ramped.

The console is UDP and the link runs at -80 dBm and worse, so a line can be
lost and a reply can come late. Each 'hold' poll therefore doubles as the
acknowledgement: what it says the blimp holds replaces what was believed sent,
so any setpoint that did not arrive goes out again.

Limits worth knowing: a hold can only start while core/'s estimator has counted
60 s of stillness, which a flight resets, so take-off is from the floor, a
second flight waits out the minute, and manual flight cannot be handed over in
the air. The estimated yaw reaches the ground only through 'hold', polled
every POLL_S, so the lead limit runs a poll behind.

The autonomous flight (2026-11-05, race day) is the same thread with the
course in place of the sticks: takeoff() with a CoursePlan (plan_course(), from
the race sheet of race_course.py) prepares as above, climbs to the first
balloon's height, and then runs course_test.py's sequence (find, follow, the
pulses, the touch, the move away, the turn to the next, each balloon's own
height) through CourseFlight, whose console, checks and ends are this
flight's: Land, STOP, the kills, the ToF, the window and the link end it the
same way, and the landing is the same. The script's guidance is not copied:
CourseFlight is course_test.Flight with its send(), tick() and the start
replaced. The firmware runs core/'s loops as before; what is autonomous is the
sequencing, on the ground, as docs/flight-tests.md's runs were.
"""
import dataclasses
import os
import re
import tempfile
import threading
import time

import course_test as ct
import hold_test as ht
import race_course as rc

HEIGHT_MIN_M = ht.LAND_SLOW_M   # below this the way down is Land, which knows about the floor
HEIGHT_MAX_M = 2.30             # flight-tests.md: the VL53L1X loses the floor beyond about 2.5 m
HEIGHT_DEFAULT_M = 1.0
HEIGHT_STEP_M = 0.10            # one D-pad press
TRIM_DEFAULT_GF = -0.6          # flight-tests.md: the new battery type; -1 gf for the earlier cells
TRIM_MAX_GF = 3.0
TRIM_STEP_GF = 0.1
SURGE_FULL_DEFAULT_GF = 3.0     # test 5's push; full stick, the trim on top
SURGE_FULL_MAX_GF = 6.0         # about both front motors at the 40 % ceiling
HOLD_SURGE_MAX_GF = 30.0        # the firmware's 'hold surge' range
BIAS_TAU_GROUND_S = 20.0        # core/src/params.c's gyro_bias_tau_s
BIAS_TAU_FLIGHT_S = 600.0       # flight-tests.md: core/ learns the bias in the air until that is fixed
DRIVER_MAX_DUTY = 40.0          # the team's limit, the firmware's default since 2026-11-01; set anyway,
DRIVER_DEADBAND = 10.0          # since the thrust stand leaves the driver plain

SEND_S = 0.10           # setpoints go out at most this often: a console line each, so not every 20 ms
POLL_S = 0.33           # 'hold', for the yaw, its rate and core/'s moving height setpoint
CMD_WAIT_S = 0.4        # one console line: the link runs at -80 dBm and worse under the envelope
SURGE_EPS_GF = 0.05     # a change smaller than these is not worth a console line
HEADING_EPS_DEG = 0.5
HEIGHT_EPS_M = 0.01
WATCHDOG_S = 3.0        # blimp_bridge.py's: the window must ping within this
LINK_LOST_S = 0.6
RESUME_WINDOW_S = 3.0   # blimp_bridge.py's: a longer outage does not simply carry on
BENCH_PARAMS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'bench_params.txt')


@dataclasses.dataclass
class Tuning:
    """A flight's shape: flown values by default, faster in the tests."""
    climb_rate_mps: float = 0.12    # what the rear pair managed at the 40 % ceiling, floor to 1 m in 8 s (2026-10-31)
    land_rate_mps: float = 0.10     # the scripts' landing, to LAND_SLOW_M
    final_rate_mps: float = 0.03    # and from there to the floor
    turn_rate_dps: float = 20.0     # full stick. A clean 90 deg turn peaked at 41 deg/s; the kill is at 62
    heading_lead_deg: float = 25.0  # about 15 deg/s steady, through the 1.7 s lag (reasoned, not flown)
    height_lead_m: float = 0.30     # the landing ramp trailed by this much on the envelope
    stop_lead_s: float = 0.7        # released, the setpoint goes this far ahead of the turn: under
                                    # kd / kp, so the loop brakes instead of coasting (reasoned)
    max_yaw_dps: float = 62.0       # the scripts' kills
    max_tilt_dps: float = 120.0
    over_height_m: float = 0.30     # target_test.py's landing trigger
    tof_lost_s: float = 1.0
    still_wait_s: float = 150.0     # for the estimator's 60 s, with room to set the gondola down first


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def clamp(v, a, b):
    return a if v < a else b if v > b else v


def move_height(target_m, stick, dt, tof_m, tuning):
    """The height setpoint after dt of stick (up positive). A stick cannot run
    the setpoint further than height_lead_m from the blimp, or a blimp that
    cannot keep up would go on climbing after the stick is let go; a setpoint
    already further away, from the number box, is left where it is."""
    if stick == 0.0:
        return target_m
    new = target_m + stick * tuning.climb_rate_mps * dt
    if tof_m is not None:
        if stick > 0:
            new = max(target_m, min(new, tof_m + tuning.height_lead_m))
        else:
            new = min(target_m, max(new, tof_m - tuning.height_lead_m))
    return clamp(new, HEIGHT_MIN_M, HEIGHT_MAX_M)


def move_heading(target_deg, stick_right, dt, yaw_deg, tuning):
    """The heading setpoint after dt of stick. The stick is right positive,
    as the page's yaw axis; core/'s heading is counter-clockwise positive."""
    if stick_right == 0.0:
        return target_deg
    lead = wrap180(target_deg - yaw_deg)
    step = -stick_right * tuning.turn_rate_dps * dt
    if step > 0:
        new = max(lead, min(lead + step, tuning.heading_lead_deg))
    else:
        new = min(lead, max(lead + step, -tuning.heading_lead_deg))
    return wrap180(yaw_deg + new)


def stop_heading(yaw_deg, rate_dps, tuning):
    """Where the heading setpoint goes when the stick is let go."""
    return wrap180(yaw_deg + rate_dps * tuning.stop_lead_s)


def parse_hold(text):
    """The 'hold' command's reply (ctl_task.c, print_hold()), or None when it
    is not one: a late reply can arrive in place of this one on a weak link.
    A field the firmware printed as nan is None."""
    m = re.search(r'^mode (\w+)', text, re.M)
    e = re.search(r'estimator: (\w+) \(still (\d+) of 60 s\)', text)
    if not m or not e:
        return None
    out = {'mode': m.group(1), 'armed': ', armed' in text, 'converged': e.group(1) == 'converged',
           'still_s': int(e.group(2)), 'yaw_deg': None, 'rate_dps': None, 'alt_m': None, 'alt_sp_m': None,
           'held_heading_deg': None, 'held_alt_m': None, 'surge_gf': None}
    h = re.search(r'heading: estimate ([-+\d.]+) deg.*?rate ([-+\d.]+) deg/s(?:; held at ([-+\d.]+) deg)?', text)
    if h:
        out['yaw_deg'], out['rate_dps'] = float(h.group(1)), float(h.group(2))
        if h.group(3):
            out['held_heading_deg'] = float(h.group(3))
    a = re.search(r'altitude: estimate ([-+\d.]+) m \(([^)]*)\)'
                  r'(?:; held at ([-+\d.]+) m(?:, moving at [\d.]+ m/s, now at ([-+\d.]+) m)?)?', text)
    if a:
        out['alt_m'] = float(a.group(1))
        if a.group(3):
            out['held_alt_m'] = float(a.group(3))
            out['alt_sp_m'] = float(a.group(4) or a.group(3))
    s = re.search(r'^surge: ([-+\d.]+) gf', text, re.M)
    if s:
        out['surge_gf'] = float(s.group(1))
    return out


def read_bench_params(path=BENCH_PARAMS):
    """software/bench_params.txt as [(name, value)], the set every run passes."""
    with open(path) as f:
        words = f.read().split()
    out = []
    for w in words:
        name, sep, value = w.partition('=')
        if not sep or not name or not value:
            raise ValueError("%s: '%s' is not name=value" % (path, w))
        out.append((name, value))
    return out


class CoursePlan:
    """A race sheet made ready to fly: course_test.py's arguments, its legs,
    the plan's lines and the capture's length, s. plan_course() builds it."""

    def __init__(self, args, legs, lines, cap_s, sheet):
        self.args, self.legs, self.lines, self.cap_s, self.sheet = args, legs, lines, cap_s, sheet
        self.alts = list(args.alt)


def plan_course(sheet, trim_gf, extra=()):
    """(CoursePlan, None), or (None, why) when the sheet cannot be flown.
    extra: more of course_test.py's arguments, as its command line takes
    them (the tests shorten the phases with it)."""
    why = rc.check(sheet)
    if why:
        return None, why
    try:
        args = ct.parser().parse_args(list(extra))
    except SystemExit:
        return None, 'course_test.py did not take %r' % (list(extra),)
    rc.apply(args, sheet)
    ct.resolve(args)
    args.trim = clamp(trim_gf, -TRIM_MAX_GF, TRIM_MAX_GF)
    why = ct.check(args)
    if why:
        return None, why
    legs = ct.legs(args.course, args.standoff)
    lines, cap_s = ct.plan(args, legs)
    return CoursePlan(args, legs, lines, cap_s, rc.normalise(sheet)), None


class _End(Exception):
    """Ends the flight, with this event for the operator."""

    def __init__(self, level, text):
        super().__init__(text)
        self.level, self.text = level, text


class Assist:
    """One assisted flight at a time. takeoff(), set_sticks(), set_height(),
    tune(), land(), abort() and ping() may be called from any thread; the
    flight runs on its own, and snapshot() says how it goes."""

    def __init__(self, link, console, on_event, tuning=None, params_path=BENCH_PARAMS, marker=None):
        self.link, self.console, self.on_event = link, console, on_event
        self.tuning = tuning or Tuning()
        self.params_path = params_path
        # The marker says a flight may be up. If this window dies with one, the
        # hold stays on the blimp: core/'s watchdog zeroes it, but the next
        # ground packet wakes it again, so the next window would start the
        # motors with nobody flying. check_leftover() ends it; without the
        # marker a hold is a script's, and left alone.
        self.marker = marker or os.path.join(tempfile.gettempdir(), 'icarus-assisted-flight')
        self._lock = threading.Lock()
        self._con_lock = threading.RLock()      # the flight and check_leftover() share the console's socket
        self._wake = threading.Event()
        self._thread = None
        self._state = 'idle'            # idle, preparing, flying, landing
        self._detail = ''
        self._stop_why = None           # (level, text) once something ended the flight
        self._land_why = None
        self._sticks = (0.0, 0.0, 0.0)  # surge, yaw (right positive), heave
        self._height_req = None
        self._trim = TRIM_DEFAULT_GF
        self._surge_full = SURGE_FULL_DEFAULT_GF
        self._ping_at = time.monotonic()
        self._height = HEIGHT_DEFAULT_M
        self._heading = 0.0
        self._surge = 0.0
        self._sent = {}
        self._order = ('surge', 'heading', 'height')
        self._alt_rate = self.tuning.climb_rate_mps
        self._kills = None              # tripped(), from the hold's start to its end
        self._yaw = self._rate = self._alt_sp = self._tof = None
        self._tof_at = 0.0
        self._sample = None
        self._lost_at = None
        self._hold_on = self._tau_set = False
        self._course = None             # a CoursePlan while an autonomous flight is up
        self._course_lines = []         # its log, course_test.py's lines

    # ----------------------------------------------------------- from outside

    @property
    def active(self):
        return self._state != 'idle'

    @property
    def _name(self):
        return 'autonomous flight' if self._course is not None else 'assisted flight'

    def takeoff(self, height_m, trim_gf, surge_full_gf, course=None):
        """Starts the flight: None, or why not. It may still end before the
        motors start: the reason then comes as an event. With a CoursePlan
        the flight is the course: the height is its first balloon's and the
        sticks do nothing."""
        with self._lock:
            if self._state != 'idle':
                return 'a flight is already under way'
            self._state, self._detail = 'preparing', 'checking the blimp'
            self._stop_why = self._land_why = self._height_req = None
            self._sticks = (0.0, 0.0, 0.0)
            self._course, self._course_lines = course, []
            self._height = clamp(course.alts[0] if course is not None else height_m, HEIGHT_MIN_M, HEIGHT_MAX_M)
            self._trim = clamp(trim_gf, -TRIM_MAX_GF, TRIM_MAX_GF)
            self._surge_full = clamp(surge_full_gf, 0.0, SURGE_FULL_MAX_GF)
            self._ping_at = time.monotonic()
        self._wake.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name='course-flight' if course is not None else 'assisted-flight')
        self._thread.start()
        return None

    def ping(self):
        """The window is alive: the station's tick, every 100 ms."""
        self._ping_at = time.monotonic()

    def set_sticks(self, surge, yaw, heave):
        with self._lock:
            self._sticks = (clamp(surge, -1.0, 1.0), clamp(yaw, -1.0, 1.0), clamp(heave, -1.0, 1.0))

    def set_height(self, height_m):
        """A height setpoint from the number box or the D-pad: core/ ramps to it."""
        with self._lock:
            self._height_req = clamp(height_m, HEIGHT_MIN_M, HEIGHT_MAX_M)

    def step_height(self, delta_m):
        with self._lock:
            base = self._height_req if self._height_req is not None else self._height
            self._height_req = clamp(base + delta_m, HEIGHT_MIN_M, HEIGHT_MAX_M)

    def tune(self, trim_gf=None, surge_full_gf=None):
        """The trim and the full-stick push, also in flight: a new cell can
        need a different trim, and the drift is what shows it."""
        with self._lock:
            if trim_gf is not None:
                self._trim = clamp(trim_gf, -TRIM_MAX_GF, TRIM_MAX_GF)
            if surge_full_gf is not None:
                self._surge_full = clamp(surge_full_gf, 0.0, SURGE_FULL_MAX_GF)

    def land(self, why='the operator asked'):
        """Starts the landing; before the motors start, it calls the flight off."""
        with self._lock:
            if self._state == 'idle' or self._land_why is not None:
                return
            self._land_why = why
        self._wake.set()

    def abort(self, level, why):
        """Ends the flight at once: its 'hold off' and nothing else goes out.
        The caller has stopped the motors (blimp_bridge.stop()), or will."""
        with self._lock:
            if self._state == 'idle' or self._stop_why is not None:
                return
            self._stop_why = (level, why)
        self._wake.set()

    def blimp_restarted(self):
        self.abort('crit', 'the blimp restarted during the flight: the hold is gone, and so are the bench parameters')

    def snapshot(self):
        with self._lock:
            return {'state': self._state, 'detail': self._detail, 'height_m': self._height,
                    'heading_deg': self._heading, 'surge_gf': self._surge, 'trim_gf': self._trim,
                    'yaw_deg': self._yaw, 'rate_dps': self._rate, 'alt_sp_m': self._alt_sp, 'tof_m': self._tof,
                    'link_lost': self._lost_at is not None, 'course': self._course is not None}

    def check_leftover(self):
        """After the link comes up: if this computer's last window died in a
        flight, end the hold it left behind before it starts the motors."""
        if os.path.exists(self.marker):
            threading.Thread(target=self._clear_leftover, daemon=True, name='assist-leftover').start()

    def on_sensors(self):
        """The link's receive thread, on every SENSORS packet: the kills, here
        so that no console line can hold them up."""
        kills = self._kills
        why = kills() if kills is not None else None
        if why:
            self._kills = None
            for _ in range(3):          # MOTOR_STOP: it needs no console, and latches KILLED all the same
                try:
                    self.link.stop_motors(False)
                except OSError:
                    pass
            self.abort('crit', self._name + ' KILLED: %s; the motors coast' % why)

    def close(self):
        self.abort('info', 'the window closed: the motors coast')
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=4.0)
        self.console.close()

    # ------------------------------------------------------------ the console

    def _cmd(self, line, tries=1):
        """One console line and its reply, or None when none came. The commands
        are idempotent, so a reply lost on a weak link is only asked for again.
        A reply can still be another line's, one that came late: whatever
        depends on it checks its text, or the next poll corrects it."""
        with self._con_lock:
            for _ in range(tries):
                try:
                    self.console.flush()
                    return self.console.run(line, timeout=CMD_WAIT_S, idle=CMD_WAIT_S)
                except (TimeoutError, ConnectionError, OSError):
                    pass
        return None

    def _expect(self, line, *texts):
        """A step of the preparation: the reply, once it is this line's (it
        holds one of texts). The flight does not start without it."""
        for _ in range(4):
            reply = self._cmd(line)
            if reply and any(t in reply for t in texts):
                return reply
        raise _End('warn', self._name + " not started: no answer to '%s' (is a long command running in the "
                           "console, or is the link down?)" % line)

    def _off(self):
        """'hold off', believed only on its own words: the reply of another
        line must not pass for it and leave a hold behind."""
        for _ in range(4):
            reply = self._cmd('hold off')
            if reply and 'hold off:' in reply:
                return True
        return False

    def _clear_leftover(self):
        with self._con_lock:            # a take-off now waits, and finds this done
            if self.active:
                return                  # its preparation ends any hold it finds
            reply = self._cmd('hold', tries=4)
            st = parse_hold(reply) if reply else None
            if st is None:
                return                  # the next take-off ends it anyway
            if st['mode'] == 'MANUAL' or self._off():
                if st['mode'] != 'MANUAL':
                    self.on_event('crit', "a hold from a window that died was still on the blimp (mode %s): "
                                          "'hold off' sent, the motors coast" % st['mode'])
                self._mark(False)

    def _mark(self, on):
        try:
            if on:
                open(self.marker, 'w').close()
            elif os.path.exists(self.marker):
                os.remove(self.marker)
        except OSError:
            pass

    # -------------------------------------------------------------- the flight

    def _set(self, state=None, detail=None):
        with self._lock:
            if state is not None:
                self._state = state
            if detail is not None:
                self._detail = detail

    def _check(self):
        """Raises when the flight was ended from outside."""
        with self._lock:
            why = self._stop_why
        if why:
            raise _End(*why)

    def _nap(self, s):
        self._wake.wait(s)
        self._wake.clear()

    def _run(self):
        level, text = 'crit', self._name + ' ended'
        try:
            self._prepare()
            why = self._fly_course() if self._course is not None else self._fly()
            level, text = self._land(why)
        except _End as e:
            level, text = e.level, e.text
        except Exception as e:          # a bug here must not leave a hold running with nobody on it
            level, text = 'crit', self._name + ': %r: motors off' % e
        try:
            self._finish()
            self.on_event(level, text)
        finally:                        # whatever happens, the window must not stay armed on a dead flight
            with self._lock:
                self._state, self._detail = 'idle', ''
                self._sticks = (0.0, 0.0, 0.0)
                self._yaw = self._rate = self._alt_sp = None
                self._lost_at = None
                self._course = None

    def _prepare(self):
        self._sent, self._hold_on, self._tau_set = {}, False, False
        self._order = ('surge', 'heading', 'height')
        self._alt_rate = self.tuning.climb_rate_mps
        self._heading = 0.0
        self._lost_at = None
        self._alt_sp = self._yaw = self._rate = None
        self._tof, self._tof_at, self._sample = None, time.monotonic(), None
        try:
            params = read_bench_params(self.params_path)
        except (OSError, ValueError) as e:
            raise _End('warn', self._name + ' not started: the bench parameters: %s' % e)
        tel = self.link.telemetry
        if not self.link.alive or not tel:
            raise _End('warn', self._name + ' not started: no telemetry from the blimp')
        if any(abs(v) > 0.005 for v in tel['motor'].values()):
            raise _End('warn', self._name + ' not started: the motors are driven (%s); one source at a time'
                       % ' '.join('%s %+.2f' % kv for kv in tel['motor'].items()))
        st = parse_hold(self._expect('hold', 'estimator:'))
        if st is None:
            raise _End('warn', self._name + " not started: this firmware's 'hold' reply is not understood")
        if st['mode'] != 'MANUAL':      # KILLED by the last STOP, or a hold left behind on the floor
            self._expect('hold off', 'hold off:')
        self._set(detail='setting the bench parameters')
        for name, value in params:
            self._check_prep()
            reply = self._expect('param %s %s' % (name, value), name)
            if '(was ' not in reply and 'is already' not in reply:
                raise _End('warn', self._name + ' not started: param %s %s: %s'
                           % (name, value, reply.strip().splitlines()[0] if reply.strip() else 'no reply'))
        reply = self._expect('motor --max-duty %g --deadband %g' % (DRIVER_MAX_DUTY, DRIVER_DEADBAND),
                             'duty ceiling', 'takes')
        if 'takes' in reply:
            raise _End('warn', self._name + ' not started: the driver refused its limits: %s' % reply.strip())
        # The floor's minute learns the bias at the ground rate, whatever a
        # flight cut short left behind.
        self._expect('param gyro_bias_tau_s %g' % BIAS_TAU_GROUND_S, 'gyro_bias_tau_s')
        if self._course is not None:
            # The course is flown on the forward camera's rows: without them
            # the climb would end in the scan and a landing, a wasted minute
            # of stillness. The row comes as the S3 relays it (LINK_PKT_CAMERA).
            self._set(detail='waiting for a row from the forward camera')
            t_end = time.monotonic() + 3.0
            while not (self.link.camera and self.link.camera['row']) and time.monotonic() < t_end:
                self._check_prep()
                time.sleep(0.05)
            if not (self.link.camera and self.link.camera['row']):
                raise _End('warn', self._name + " not started: no row from the forward camera over the link in 3 s "
                                                "(is it running target_cam, not optflow_cam, and does 'cam' show its "
                                                "rows?)")
        self._wait_still()
        self._check_prep()
        self._set(detail='checking that the ToF sees the floor')
        why = ht.floor_check(self.link)
        if why:
            raise _End('warn', self._name + ' not started: %s' % why)
        self._check_prep()
        self._tau_set = True
        self._expect('param gyro_bias_tau_s %g' % BIAS_TAU_FLIGHT_S, 'gyro_bias_tau_s')

        self._mark(True)
        self._hold_on = True            # from here on, every end sends 'hold off'
        self._kills = ht.make_kills(self.link, self.tuning.max_yaw_dps, self.tuning.max_tilt_dps)[0]
        with self._lock:
            self._surge = self._trim
        # A STOP that reaches the blimp before the hold's first tick finds the
        # motors not yet the loop's, so it latches nothing (cmd_motor.c,
        # stop_while_owned()) and the hold arms all the same. Only 'hold off'
        # ends that, so each step here looks for a stop before the next one.
        for key, value, line in (('heading', 0.0, 'hold heading 0'),
                                 ('surge', self._surge, 'hold surge %.2f' % self._surge),
                                 ('height', self._height, 'hold altitude %.2f %.3f' % (self._height, self._alt_rate))):
            self._check()
            reply = self._expect(line, 'held:', 'refused')
            if 'refused' in reply:
                self._check()
                raise _End('warn', self._name + ' not started: %s' % reply.strip().splitlines()[-1])
            self._sent[key] = value
        self._check()

    def _check_prep(self):
        """The preparation can be called off with Land as well as with STOP."""
        self._check()
        with self._lock:
            why = self._land_why
        if why:
            raise _End('info', 'take-off called off: nothing started')

    def _wait_still(self):
        """core/ arms only after 60 s of stillness, and only 'hold' says how
        far along it is. Motion restarts the count, so the operator sees it."""
        t_end = time.monotonic() + self.tuning.still_wait_s
        while True:
            self._check_prep()
            reply = self._cmd('hold')
            st = parse_hold(reply) if reply else None
            if st and st['converged']:
                return
            if st:
                self._set(detail='keep the gondola still: %d of 60 s' % st['still_s'])
            if time.monotonic() > t_end:
                raise _End('warn', self._name + ' not started: the estimator did not settle in %.0f s (it needs '
                                   'the gondola still for 60 s)' % self.tuning.still_wait_s)
            self._nap(0.5)

    def _sense(self, now):
        s = self.link.sensors
        if s is not None and s is not self._sample:
            self._sample = s
            if s['tof_valid'] and s['tof_status'] == 0:
                self._tof, self._tof_at = s['tof_mm'] / 1000.0, now

    def _poll(self):
        reply = self._cmd('hold')
        st = parse_hold(reply) if reply else None
        if st is None:
            return None
        if st['mode'] != 'HOLD':
            self._check()               # our own STOP gets here first, as a plain stop
            raise _End('crit', "the blimp left the hold (mode %s): a stop from somewhere else?" % st['mode'])
        if st['yaw_deg'] is not None:
            self._yaw, self._rate = st['yaw_deg'], st['rate_dps']
        if st['alt_sp_m'] is not None:
            self._alt_sp = st['alt_sp_m']
        # What the blimp says it holds is what it has, whatever a reply said:
        # a line lost behind another's late reply goes out again from here.
        # The tolerances are the printing's rounding, with room.
        for key, got, tol in (('heading', st['held_heading_deg'], 2 * HEADING_EPS_DEG),
                              ('height', st['held_alt_m'], 2 * HEIGHT_EPS_M),
                              ('surge', st['surge_gf'], 2 * SURGE_EPS_GF)):
            if got is not None and abs(got - self._sent[key]) > tol:
                self._sent[key] = got
        return st

    def _put(self, key, value, line):
        """False when no answer came: not marked as sent, so it goes again."""
        reply = self._cmd(line)
        if reply is None:
            return False
        if 'held:' in reply:
            self._sent[key] = value
        elif 'refused' in reply:
            self._check()
            raise _End('crit', 'the blimp refused a setpoint: %s' % reply.strip().splitlines()[-1])
        return True

    def _send(self):
        """The axes the blimp does not have yet, in self._order. Each is the
        whole setpoint, so a lost line costs one send; the first that gets no
        answer ends the round, so a dead link costs one wait, not three."""
        lines = {'surge': (self._surge, 'hold surge %.2f' % self._surge,
                           abs(self._surge - self._sent['surge']) >= SURGE_EPS_GF),
                 'heading': (self._heading, 'hold heading %.1f' % self._heading,
                             abs(wrap180(self._heading - self._sent['heading'])) >= HEADING_EPS_DEG),
                 'height': (self._height, 'hold altitude %.2f %.3f' % (self._height, self._alt_rate),
                            abs(self._height - self._sent['height']) >= HEIGHT_EPS_M)}
        for key in self._order:
            value, line, due = lines[key]
            if due and not self._put(key, value, line):
                return

    def _fly(self):
        """Until a landing is due: returns why."""
        t = self.tuning
        self._set('flying', '')
        now = last = time.monotonic()
        self._tof_at = now
        next_send = next_poll = now
        turning = False
        while True:
            self._check()
            now = time.monotonic()
            dt = min(0.2, now - last)
            last = now
            self._sense(now)
            with self._lock:
                land_why, ping_at = self._land_why, self._ping_at
                sticks, height_req = self._sticks, self._height_req
                self._height_req = None
                trim, full = self._trim, self._surge_full
            if land_why:
                return land_why
            if now - ping_at > WATCHDOG_S:
                return 'the window stopped responding for %d s' % int(WATCHDOG_S)

            rx = self.link.last_rx
            if now - rx > LINK_LOST_S:
                if self._lost_at is None:
                    self._lost_at = rx
                    self.on_event('warn', 'nothing from the blimp for %.1f s: core/ stops the motors after 1 s and '
                                          'takes the hold up again when the link returns' % (now - rx))
                time.sleep(0.02)
                continue                # nothing sent can arrive, and the sticks wait
            if self._lost_at is not None:
                gone = now - self._lost_at
                self._lost_at = None
                self._tof_at = now
                if gone > RESUME_WINDOW_S:
                    return 'the link was gone for %.1f s' % gone
                self.on_event('ok', 'the link is back after %.1f s: the hold goes on' % gone)

            if now - self._tof_at > t.tof_lost_s:
                return 'the ToF has been silent for %.0f s' % t.tof_lost_s
            top = max(self._height, self._alt_sp if self._alt_sp is not None else 0.0)
            if self._tof is not None and self._tof > top + t.over_height_m:
                return 'the ToF reads %.2f m, over the setpoint by more than %.1f m' % (self._tof, t.over_height_m)

            surge, yaw, heave = sticks
            self._surge = clamp(trim + surge * full, -HOLD_SURGE_MAX_GF, HOLD_SURGE_MAX_GF)
            if height_req is not None:
                self._height = height_req
            self._height = move_height(self._height, heave, dt, self._tof, t)
            if yaw != 0.0:
                self._heading = move_heading(self._heading, yaw, dt, self._yaw if self._yaw is not None
                                             else self._heading, t)
                turning = True
            elif turning:
                turning = False         # let go: stop the nose where it is now, not where the setpoint led
                self._poll()
                next_poll = now + POLL_S
                if self._yaw is not None:
                    self._heading = stop_heading(self._yaw, self._rate, t)
            if now >= next_send:
                self._send()
                next_send = now + SEND_S
            if now >= next_poll:
                self._poll()
                next_poll = now + POLL_S
            time.sleep(0.02)

    # ------------------------------------------------------------ the course

    def _fly_course(self):
        """The race sheet's balloons in order, course_test.py's sequence on
        this flight's checks (CourseFlight), from the climb to the last move
        away: why to land. A line the blimp refused or never answered lands
        it too; a STOP or a kill ends it through _check(), as in _fly()."""
        plan = self._course
        f = CourseFlight(self, plan)
        self._set('flying', 'the climb to %.2f m, looking for balloon 1' % plan.alts[0])
        f.t_hold = time.monotonic()
        self._tof_at = f.t_hold
        try:
            stop = f.run_legs()
        except RuntimeError as e:
            self._check()
            stop = ('land', str(e))
        finally:
            self._height = f.alt if f.alt_prev is None else max(f.alt, f.alt_prev)
        if f.s3_follow:                 # the landing holds the heading; a follow left on would take it back
            self._cmd('hold follow off', tries=2)
        f.end = stop[1] if stop else 'the course is done'
        self.on_event('ok' if stop is None else 'warn', 'the course: %s' % f.summary())
        return f.end

    def _course_guard(self, now, f):
        """Why the course must land now, or None: _fly()'s ends, on the course
        flight f. A short link loss is waited out here (STOP can still end
        it); a longer one lands the blimp when the link returns."""
        t = self.tuning
        with self._lock:
            land_why, ping_at = self._land_why, self._ping_at
        if land_why:
            return land_why
        if now - ping_at > WATCHDOG_S:
            return 'the window stopped responding for %d s' % int(WATCHDOG_S)
        rx = self.link.last_rx
        if now - rx > LINK_LOST_S:
            if self._lost_at is None:
                self._lost_at = rx
                self.on_event('warn', 'nothing from the blimp for %.1f s: core/ stops the motors after 1 s and '
                                      'takes the hold up again when the link returns' % (now - rx))
            while now - self.link.last_rx > LINK_LOST_S:
                self._check()
                with self._lock:
                    land_why, ping_at = self._land_why, self._ping_at
                if land_why:
                    return land_why
                if now - ping_at > WATCHDOG_S:
                    return 'the window stopped responding for %d s' % int(WATCHDOG_S)
                time.sleep(0.02)
                now = time.monotonic()
            gone = now - self._lost_at
            self._lost_at = None
            self._tof_at = now
            if gone > RESUME_WINDOW_S:
                return 'the link was gone for %.1f s' % gone
            self.on_event('ok', 'the link is back after %.1f s: the course goes on' % gone)
        if now - self._tof_at > t.tof_lost_s:
            return 'the ToF has been silent for %.0f s' % t.tof_lost_s
        if self._tof is not None:
            top = max(f.alt_top(self._tof), self._alt_sp if self._alt_sp is not None else 0.0)
            if self._tof > top + t.over_height_m:
                return 'the ToF reads %.2f m, over %.2f m by more than %.1f m' % (self._tof, top, t.over_height_m)
        return None

    def _course_line(self, s):
        """course_test.py's log, line by line: a phase to the state line and
        the window's log, an event to the log, a command's echo kept for the
        record only."""
        self._course_lines.append(s)
        m = re.match(r'\s*[-\d.]+ s\s+(.*)', s)
        text = (m.group(1) if m else s).strip()
        if text.startswith('-- '):
            self._set(detail=text[3:])
            self.on_event('info', 'course: ' + text[3:])
        elif ': held:' in text or ': hold starting' in text:
            return
        else:
            level = ('ok' if (' is seen' in text or ' is close' in text) else
                     'warn' if ('not seen' in text or 'lost' in text or 'WARNING' in text) else 'info')
            self.on_event(level, 'course: ' + text)

    def _land(self, why):
        """hold_test.land(), on this flight's checks: down at the landing rate
        to LAND_SLOW_M, then at the final rate, motors off once the ToF reads
        LAND_CUT_M. The sticks are out of it; STOP is not. The height goes out
        first and on the last figures to hand: the reason for this landing may
        be that the blimp is too high, and on a weak link a poll is a wait.
        Returns the event, (level, text)."""
        t = self.tuning
        self._set('landing', why)
        self.on_event('info', 'landing: %s' % why)
        self._order = ('height', 'surge', 'heading')
        with self._lock:
            self._surge = self._trim    # the trim stays on: the rear pair still pushes
        if self._yaw is not None:
            self._heading = stop_heading(self._yaw, self._rate, t)
        sp0 = max(self._alt_sp if self._alt_sp is not None else self._height, self._tof or 0.0)
        fast_s = max(0.0, sp0 - ht.LAND_SLOW_M) / t.land_rate_mps
        end_s = ht.landing_time(sp0, t.land_rate_mps, t.final_rate_mps)
        t0 = time.monotonic()
        next_send, next_poll = t0, t0 + POLL_S
        while True:
            self._check()
            now = time.monotonic()
            el = now - t0
            self._sense(now)
            if sp0 > ht.LAND_SLOW_M and el < fast_s:
                self._height, self._alt_rate = ht.LAND_SLOW_M, t.land_rate_mps
            else:
                self._height, self._alt_rate = ht.LAND_FLOOR_M, t.final_rate_mps
            if now >= next_send:
                self._send()
                next_send = now + SEND_S
            if self._tof is not None and now - self._tof_at < 0.5 and self._tof <= ht.LAND_CUT_M:
                return 'ok', 'landed: ToF %.2f m, motors off' % self._tof
            if el >= end_s:
                return 'warn', 'the landing timed out after %.0f s: ToF %s, motors off' % (
                    el, '%.2f m' % self._tof if self._tof is not None else 'not reading')
            if now >= next_poll:
                self._poll()
                next_poll = now + POLL_S
            time.sleep(0.02)

    def _finish(self):
        """Every end: the hold off, and the bias learning back. A stop can
        leave KILLED behind, and only 'hold off' clears it."""
        self._kills = None
        if self._hold_on:
            if not self._off():
                for _ in range(3):      # the same end by another road, if the link still carries anything
                    try:
                        self.link.stop_motors(False)
                    except OSError:
                        pass
                self.on_event('crit', "'hold off' did not answer: MOTOR_STOP sent. A hold may be left on the blimp; "
                                      'the next window or take-off ends it')
            else:
                self._mark(False)
        if self._tau_set:
            reply = self._cmd('param gyro_bias_tau_s %g' % BIAS_TAU_GROUND_S, tries=3)
            if not reply or 'gyro_bias_tau_s' not in reply:
                self.on_event('warn', 'gyro_bias_tau_s was not put back to %g s: the next take-off does it, or a '
                                      'reboot' % BIAS_TAU_GROUND_S)


class CourseFlight(ct.Flight):
    """course_test.py's Flight on an Assist: its sequence (find, follow, the
    pulses, the touch, the move away, the turn to the next, each balloon's
    height) with this flight's console, checks and ends in place of the
    script's. The kills run on the link's thread (Assist.on_sensors) and end
    it through _check(); the landing is Assist._land()'s, from the stop
    run_legs() returns; 'hold' is polled as the assisted flight polls it, so
    the page's readouts and the poll-as-acknowledgement work as there."""

    def __init__(self, assist, plan):
        super().__init__(assist.link, None, plan.args, plan.legs, assist._course_line)
        self.ax = assist
        self.next_poll = 0.0

    def send(self, cmd):
        reply = self.ax._cmd(cmd, tries=3)
        if reply is None:
            raise RuntimeError("no answer to '%s'" % cmd)
        last = reply.strip().splitlines()[-1] if reply.strip() else '?'
        self.log('%6.1f s  %s: %s' % (self.clock(), cmd, last))
        if 'refused' in reply or 'not a number' in reply or 'Unrecognized' in reply:
            self.ax._check()
            raise RuntimeError("the blimp refused '%s': %s" % (cmd, last))
        return reply

    def set_alt(self, k):
        stop = super().set_alt(k)
        self.ax._height = self.alt      # the landing starts from the height held
        return stop

    def tick(self, guard=True):
        ax = self.ax
        ax._check()
        now = time.monotonic()
        ax._sense(now)
        self.heading.update()
        if self.track is not None:
            self.track.update()
        why = ax._course_guard(now, self)
        if why:
            return ('land', why)
        stop = self.over_time()
        if stop:
            return stop
        if guard and self.following and self.at_height:
            why = self.track.near(self.close_px, self.args.close_el)
            if why:
                return ('close', why)
        if now >= self.next_poll:
            st = ax._poll()
            if st is not None and st['yaw_deg'] is not None:
                self.heading.reset(st['yaw_deg'])
            self.next_poll = now + POLL_S
        return None
