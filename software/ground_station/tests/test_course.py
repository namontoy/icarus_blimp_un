"""The race sheet and the autonomous flight against the in-process demo
blimp: the plan from a sheet, the Course panel's sketch and its file, then
whole courses with the phases shortened: a balloon found and not reached (the
pulses run out, the landing), a balloon touched and the course done, two
balloons at different heights, Land in the course, STOP in the course, and a
take-off refused without camera rows. The sequence itself is course_test.py's
(its --selftest covers it against a stand-in); this checks it on the flight
thread, with the station's console, checks and ends (docs/race-day.md)."""
import os
import tempfile
import threading
import time

import harness as H
from harness import check, near, wait_for

from PyQt6.QtWidgets import QApplication

import race_course as rc
from software.ground_station import assist as Ax
from software.ground_station import blimp_bridge as BB
from software.ground_station import course as Cs
from software.ground_station import theme as T

# ------------------------------------------------------------ the plan
sheet = rc.normalise(rc.DEFAULT)
plan, why = Ax.plan_course(sheet, -0.6)
check(plan is not None and why is None, 'plan: the default sheet flies: %s' % why)
check(plan.alts == [1.3, 1.3] and plan.args.trim == -0.6 and plan.args.close_px == [1500] and len(plan.legs) == 2,
      'plan: heights, trim, close patch and legs from the sheet')
check(200 < plan.cap_s <= 400 and any('balloon 2 at' in s for s in plan.lines), 'plan: the capture and the lines')
plan2, why = Ax.plan_course(dict(sheet, balloons=[{'x_m': 9.0, 'y_m': 0.0, 'height_m': 1.2}]), -0.6)
check(plan2 is None and 'outside the field' in why, 'plan: a balloon outside the field is refused: %s' % why)
plan3, why = Ax.plan_course(sheet, -0.6, extra=['--no-such-flag'])
check(plan3 is None and why, 'plan: a bad extra argument is a reason, not a crash')

# ------------------------------------------------------------ the panel
app = QApplication.instance() or QApplication([])
T.apply(app)
sk = Cs.CourseSketch()
sk.resize(400, 300)
img = sk.grab().toImage()
red = sum(1 for x in range(0, 400, 4) for y in range(0, 300, 4) if img.pixelColor(x, y).red() > 180
          and img.pixelColor(x, y).green() < 90)
check(red > 5, 'sketch: paints, with red balloons on it (%d samples)' % red)


class _Station:
    def __init__(self):
        self.events = []

    def log(self, level, text):
        self.events.append((level, text))


path = os.path.join(tempfile.mkdtemp(prefix='course-test-'), 'race_course.json')
st = _Station()
panel = Cs.CoursePanel(st, path)
check(len(panel.sheet['balloons']) == 2 and not os.path.exists(path), 'panel: the default sheet without a file')
panel.count_in.setValue(3)
check(os.path.exists(path) and len(rc.load(path)['balloons']) == 3 and not panel.rows[2][1].isHidden()
      and panel.rows[3][1].isHidden(), 'panel: a third balloon: a row, and the file written')
b3 = rc.load(path)['balloons'][2]
check(b3['x_m'] == 3.5 and b3['y_m'] == -2.0 and b3['height_m'] == 1.2, 'panel: the new balloon beside the last: %s' % b3)
panel.rows[2][3].setValue(1.7)
check(rc.load(path)['balloons'][2]['height_m'] == 1.7, 'panel: a height edit reaches the file')
panel.field_in['ahead_m'].setValue(3.0)
check('Cannot be flown' in panel.check_l.text() and 'outside the field' in panel.check_l.text(),
      'panel: the check says why the sheet cannot be flown: %s' % panel.check_l.text())
rc.save(dict(sheet, name='by hand'), path)
panel.load()
check(panel.sheet['name'] == 'by hand' and len(panel.sheet['balloons']) == 2 and panel.count_in.value() == 2
      and panel.rows[2][1].isHidden(), 'panel: Reload reads the file back')
far = rc.normalise(dict(sheet, balloons=[{'x_m': 2.0, 'y_m': 0.0, 'height_m': 1.2},
                                         {'x_m': 5.5, 'y_m': 0.0, 'height_m': 1.2}]))
check(rc.check(far) is None and any('reach' in w for w in rc.warnings(far)),
      'sheet: a leg beyond the camera\'s reach flies, with a warning: %s' % rc.warnings(far))
rc.save(far, path)
panel.load()
check('reach' in panel.check_l.text(), 'panel: the warning on the page: %s' % panel.check_l.text())
panel.set_flight({'state': 'flying', 'detail': 'pulse 2 toward balloon 2 at -76 deg: +3.0 gf'})
check(panel.sketch.current == 1 and 'pulse 2' in panel.flight_l.text(), 'panel: the flight marks its balloon')
panel.set_flight({'state': 'idle'})
check(panel.sketch.current is None, 'panel: idle, no mark')

# ------------------------------------------------------------ the flights
events = []
port, console_port = H.free_ports()
b = BB.DemoBlimp(lambda lvl, txt: events.append((lvl, txt)), port=port, console_port=console_port).start()
mock = b.mock
check(b.link.wait_alive(3.0), 'demo blimp alive')
a = b.assist
a.tuning = Ax.Tuning(climb_rate_mps=0.4, land_rate_mps=0.25, final_rate_mps=0.2)
a.marker = os.path.join(tempfile.mkdtemp(prefix='course-test-'), 'marker')


class Scene:
    """The forward camera over a floor with balloons, for the demo blimp,
    whose toy airframe has a heading and a height but no place: the place is
    kept here, as course_test.py's stand-in keeps it, from the surge the
    ground holds (the trim cancels the rear pair's lean, so -0.6 gf is no
    push), times gain (the test's pulses are a sixth of the flown ones, so
    48 covers the flown distance with room; 0 and the blimp never moves). The nearest
    balloon within 27 deg of the nose and 0.3..2.6 m is the disc, sized as
    the stand-in sizes it (a 45 cm balloon through a 160 px lens); none in
    view, no disc."""

    def __init__(self, balloons, gain=0.0):
        self.b, self.gain = balloons, gain
        self.x = self.y = self.v = 0.0
        self.t = None

    def __call__(self, t):
        import math
        if self.t is not None:
            dt = min(0.2, t - self.t)
            f = (mock.hold.get('surge', 0.0) + 0.6) if mock.hold_mode == 'HOLD' and mock.height_m > 0.12 else 0.0
            self.v += (f if f >= 0 else 0.75 * f) * 0.00981 / 0.33 * self.gain * dt
            if abs(f) < 0.1 and abs(self.v) < 0.5:
                self.v = 0.0            # what a brake leaves over (the toy has no drag) stops the blimp, not drifts it
            yaw = math.radians(mock.yaw_est())
            self.x += self.v * math.cos(yaw) * dt
            self.y += self.v * math.sin(yaw) * dt
        self.t = t
        best = None
        for bx, by in self.b:
            dx, dy = bx - self.x, by - self.y
            d = math.hypot(dx, dy)
            bearing = (math.degrees(math.atan2(dy, dx)) - mock.yaw_est() + 180.0) % 360.0 - 180.0
            if 0.3 < d < 2.6 and abs(bearing) <= 27.0:
                r = 144.0 * 0.45 / d / 2.0
                if best is None or r > best[2]:
                    best = (80.0 - bearing * 160.0 / 54.0, 60.0, r)
        return best or (-200.0, -200.0, 5.0)


# The phases shortened: a pulse in about a second, the look in one. The brake
# lasts a third longer than the push, as the flown 1.8 s and 2.4 s do: the
# reverse is three quarters of the forward, and a brake that does not take
# back the push's speed leaves the toy drifting (no drag) past the balloon.
FAST = ['--push', '3', '0.3', '--coast', '0.3', '--brake', '3', '0.4', '--hold', '0.3', '--look', '1',
        '--settle', '0.5', '--scan', '0', '--turn-time', '2', '--lost', '2', '--max-pulses', '3',
        '--climb-rate', '0.4', '--touch', '0.3']


class Pinger:
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
    mock.hold_mode, mock.alt_sp = 'MANUAL', None
    mock.hold = {'heading': None, 'altitude': None, 'rate': 0.0, 'surge': 0.0, 'follow': False}
    mock.height_m, mock.vz, mock.tof_blind = 0.07, 0.0, False
    mock.yaw_rate, mock.heading = 0.0, 0.0
    mock.motor = [0.0] * 4
    mock.still_s = 60.0
    time.sleep(0.15)


def state():
    return a.snapshot()['state']


def fly(sheet_, trim=-0.6, gain=0.0):
    on_floor()
    events.clear()
    mock.cam_scene = Scene([(b['x_m'], b['y_m']) for b in sheet_['balloons']], gain)
    plan_, why_ = Ax.plan_course(sheet_, trim, FAST)
    check(plan_ is not None, 'the plan: %s' % why_)
    why_ = b.takeoff(plan_.alts[0], trim, 0.0, course=plan_)
    check(why_ is None, 'the course accepted: %s' % why_)
    check(wait_for(lambda: state() == 'flying', 6.0), 'preparation done, flying')
    check(a.snapshot()['course'], 'the snapshot says it is the course')
    return plan_


def idle(t=40.0):
    return wait_for(lambda: state() == 'idle', t)


def landed():
    return idle() and said('landed', 'ok') and mock.height_m < 0.12 and not any(mock.motor)


one = rc.normalise(dict(sheet, balloons=[{'x_m': 2.0, 'y_m': 0.0, 'height_m': 1.2}]))

# found, followed, never close (the blimp does not move): the pulses run out and it lands
fly(one)
check(wait_for(lambda: said('balloon 1 is seen', 'ok'), 8.0), 'balloon 1 seen in the climb')
check(wait_for(lambda: mock.hold.get('follow'), 3.0), "'hold follow' reached the blimp")
check(wait_for(lambda: said('pulse 1 toward balloon 1'), 6.0), 'the first pulse')
check(wait_for(lambda: said('not close after 3 pulses', 'warn'), 12.0), 'the pulses run out: a landing, said why')
check(landed(), 'landed after the course: ToF %.2f m, hold %s' % (mock.height_m, mock.hold_mode))
check(mock.hold_mode == 'MANUAL' and not mock.hold.get('follow'), 'hold off at the end')
check(said('the course: balloon 1 seen at', 'warn'), 'the summary in the log')

# the pulses carry it to the balloon: close on its patch, the touch, the move away, the course done
close = rc.normalise(dict(one, close_px=4000))
fly(close, gain=48.0)
check(wait_for(lambda: said('balloon 1 is close', 'ok'), 16.0), 'balloon 1 close on its patch')
check(wait_for(lambda: said('the course is done'), 12.0), 'the course done')
check(landed(), 'landed after the touch')

# two balloons, the second higher: the hold climbs between them
two = rc.normalise(dict(sheet, close_px=4000, balloons=[{'x_m': 2.0, 'y_m': 0.0, 'height_m': 1.2},
                                                       {'x_m': 2.0, 'y_m': -2.0, 'height_m': 1.7}]))
fly(two, gain=48.0)
check(wait_for(lambda: said('balloon 1 is close', 'ok'), 16.0), 'two: balloon 1 close')
check(wait_for(lambda: said('the hold goes from 1.30 to 1.80 m'), 12.0), 'two: the height move to balloon 2')
check(wait_for(lambda: near(mock.hold['altitude'] or 0.0, 1.8, 0.001), 3.0), "two: 'hold altitude 1.8' reached the blimp")
check(wait_for(lambda: said('balloon 2 is seen', 'ok'), 20.0), 'two: balloon 2 seen after the turn')
check(wait_for(lambda: said('the course is done') or said('landing:'), 25.0), 'two: the course ends: %s'
      % [t for _, t in events if t.startswith('the course')])
check(landed(), 'two: landed')

# Land in the course
fly(one)
check(wait_for(lambda: said('pulse 1 toward balloon 1'), 8.0), 'land: in the first pulse')
a.land()
check(wait_for(lambda: state() == 'landing', 3.0) and said('landing: the operator asked', 'info'), 'land: landing')
check(landed(), 'land: down')

# STOP in the course: the motors coast, KILLED on the blimp, hold off clears it
fly(one)
check(wait_for(lambda: said('pulse 1 toward balloon 1'), 8.0), 'stop: in the first pulse')
b.stop(False)
check(idle(5.0) and said('the flight stopped', 'info'), 'stop: the flight ends at once')
check(mock.hold_mode == 'MANUAL' and not any(mock.motor), 'stop: hold off sent, motors coasting')

# no camera rows: refused before the minute of stillness
mock.camera = 'silent'
time.sleep(1.5)                 # the next packet is health only: no row
on_floor()
events.clear()
plan_, _ = Ax.plan_course(one, -0.6, FAST)
check(b.takeoff(plan_.alts[0], -0.6, 0.0, course=plan_) is None, 'no rows: the take-off starts its checks')
check(idle(8.0) and said('no row from the forward camera', 'warn'), 'no rows: refused, said why')
check(mock.hold_mode == 'MANUAL' and not any(mock.motor), 'no rows: nothing started')
mock.camera = 'rows'

pinger.on = False
b.close()
raise SystemExit(H.finish())
