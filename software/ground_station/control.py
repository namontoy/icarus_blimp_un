"""The Control page: flying the blimp by hand from this computer, before and
between the autonomous runs.

The race is autonomous, so this page is for the minutes around it: the first
tethered hover, a trim check with the propellers on, bringing the blimp back
by hand. The operator drives surge, yaw and heave, the three degrees of
freedom the airframe has (AGENTS.md: no sideways force), from the keyboard
or a gamepad, and the page mixes them into the four motor duties the link
carries (MOTOR_SET, in FL RL FR RR order) with the geometry of core/'s
mixer: the front pair's sum is surge and its difference is yaw, the rear pair
is heave. That is open loop and per motor, which is what the firmware's
MANUAL mode is defined to take (docs/control-loop-plan.md D5), so nothing
here is a controller. When blimp_step() runs on the blimp, going autonomous
is a MODE_SET away; the button for it is already on this page, disabled
until that packet exists (increment 3 of the plan).

A fourth axis, roll, since 2026-11-03, off by default (the Roll switch,
remembered): the rear pair's difference from the right stick's free axis or
Q / E, mixed like the front pair's yaw, RL harder to roll right. The team
wants to see whether driving the heave pair apart does anything when a gust
rocks the blimp. On paper it cannot turn it: the pair pushes along z at
±5.5 cm (AGENTS.md), so its difference is a torque about the fore-aft axis,
about 0.4 gf·m at the 40 % ceiling, which the pendulum of a gondola under a
sphere pulls back; a turn needs the front pair. What it costs is lift: at
hover both rear motors sit near the ceiling (about 6.8 gf for a blimp 6 gf
heavy), so a difference comes out of one of them and the blimp sinks. Hence
opt-in, each rear motor clamped on its own so the roll never pulls the other
one down too, and the capture to say what happened.

Two input modes, kept in QSettings and settable with --mode: PC (the
keyboard, plus a gamepad if one is plugged in) and Steam Deck (the Deck's own
controls through gamepad.py, the legend in the Deck's names, the hints hidden
for its 1280 × 800 screen). The safety chain is the Flight page's: the page
sends the mixed command every TICK_MS while armed (it ramps, so it changes
every tick), the bridge's thread holds the last one and repeats it to the
blimp, its watchdog releases the motors if the window stops responding for
3 s, and the blimp coasts on its own timers after that. STOP (the button,
Space, Esc, or B on the pad, which the pad reader's thread also sends to the
bridge directly) disarms and coasts; a pad that disappears while it was
driving is a STOP; arming needs
telemetry from the blimp and the sticks at rest, so the motors never jump.
Only one source drives at a time: arming here switches the Flight page's
"Motors enabled" off, and the other way round (app.py). Leaving the page
does not disarm: the pad keeps driving while another page is in view (the
team's choice of 2026-10-29: looking at a plot must not change the motors),
and the header's Drive pill and STOP stay in view on every page. With
--race the page cannot arm at all: no manual control during the race.

Two ways to fly, the Fly switch, remembered (since 2026-11-04). Manual is
everything above. Assisted (assist.py) gives height and heading to core/'s
loops: the sticks move the setpoints, surge stays a push, and Land brings the
blimp down. Its flight runs on its own thread, so this page only says what the
sticks and boxes say: the arming below is the take-off, STOP and the watchdog
work as above, a pad that disappears lands instead of stopping, and the
Flight page's motors stay off until the flight ends. Assisted starts from the
floor and Land works only there: a hold cannot start in the air (assist.py).

The third way, Autonomous (2026-11-05, race day), is the course: the race
sheet (race_course.py, the Course panel beside this one: the balloons, their
heights, the field) flown by the same flight thread with course_test.py's
sequence in place of the sticks (assist.py, CourseFlight). "Fly the course",
or A, prepares as Take off does and climbs to the first balloon's height; the
sticks do nothing; Land, X, Y, STOP, B and every automatic end work as in
Assisted. The firmware's own mode switch (MODE_SET, docs/control-loop-plan.md
increment 3) does not exist, and core/'s guidance does not run on the blimp:
what is autonomous is the sequencing, on the ground, as every flight test
has been. The "Go autonomous" placeholder this page carried until then is
this mode.
"""
from PyQt6.QtCore import QLocale, Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import QButtonGroup, QComboBox, QDoubleSpinBox, QGridLayout, QSizePolicy, QSpinBox, QWidget

import race_course as rc

from . import assist as Ax
from .course import CoursePanel
from . import theme as T
from .panels import MOTORS, Panel, button, code, h3, info_text, notice
from .widgets import (CentreBar, FlowBox, InfoIcon, Readout, SegBar, StickPad, Switch, TopView, clamp, hbox, label,
                      set_color, sfmt)

TICK_MS = 50            # pad, keys, ramp and send; the bridge streams to the blimp at 20 Hz anyway
DEADZONE = 0.08         # of the stick's travel: a thumb at rest is never a command
EXPO = 0.30             # cubic share of the response: fine near the centre, full at the edge
SLEW_S = 0.40           # full scale takes this long: the keyboard's steps become ramps, a stick flick too
KEY_HALF = 0.5          # a key alone, as a fraction of the axis; with Shift the whole axis
LIMIT_DEFAULT = 30
MODES = ('pc', 'deck')
MODE_NAMES = {'pc': 'PC', 'deck': 'Steam Deck'}
# RC transmitter conventions: in mode 2 the left stick is throttle and rudder,
# which for a blimp is heave and yaw, and the right stick's vertical is surge.
LAYOUTS = (('mode2', 'Mode 2: L heave + yaw, R surge', 'left stick: heave and yaw · right stick: surge'),
           ('mode1', 'Mode 1: L surge + yaw, R heave', 'left stick: surge and yaw · right stick: heave'))
# The ones the page acts on, on their press edge.
PAD_BUTTONS = ('a', 'b', 'x', 'y', 'start', 'up', 'down', 'left', 'right')
AXES = ('surge', 'yaw', 'heave', 'roll')            # the command's order everywhere on the page


# ------------------------------------------------------------ pure parts

def shape_axis(v, deadzone=DEADZONE, expo=EXPO):
    """A stick position to a command: nothing inside the dead band, then a
    blend of linear and cubic over the rest of the travel, so a small
    deflection moves little and the edge still gives everything."""
    a = abs(v)
    if a <= deadzone:
        return 0.0
    u = min(1.0, (a - deadzone) / (1.0 - deadzone))
    out = expo * u ** 3 + (1.0 - expo) * u
    return out if v > 0 else -out


def teleop_mix(surge, yaw, heave, limit, roll=0.0):
    """Surge, yaw (right positive), heave and roll (right positive) in −1…+1
    to the four duties the link carries, FL RL FR RR, scaled by the limit.
    The front pair is sum and difference: the left motor pushes harder to
    turn right. When the sum would exceed one motor, both are scaled
    together so the turn the operator asked for keeps its proportion
    (core/'s mixer instead protects yaw and clamps surge, the right policy
    for a controller, but a hand on a stick expects what it sees). The rear
    pair is heave, equal on both, plus the roll as its difference, RL harder
    to roll right; negative heave reverses them, which the DRV8833s allow.
    The rear pair is clamped motor by motor, not scaled together: the lift
    is what keeps the blimp off the floor, so a roll at full heave takes
    from one motor and never pulls the other down with it."""
    fl, fr = surge + yaw, surge - yaw
    m = max(abs(fl), abs(fr), 1.0)
    fl, fr = fl / m, fr / m
    rl, rr = clamp(heave + roll, -1.0, 1.0), clamp(heave - roll, -1.0, 1.0)
    return [clamp(v * limit, -1.0, 1.0) for v in (fl, rl, fr, rr)]


def slew(current, target, step):
    """One step of a rate limit toward the target."""
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target


def pad_axes(axes, layout):
    """The pad's sticks to (surge, yaw, heave, roll) under a stick layout;
    the right stick's X, the axis both layouts leave free, is the roll."""
    if layout == 'mode1':
        return axes['ly'], axes['lx'], axes['ry'], axes['rx']
    return axes['ry'], axes['lx'], axes['ly'], axes['rx']


def pad_names(layout, roll=False, assisted=False):
    """((left x, left y), (right x, right y)) axis names for the pads; the
    right stick's X is named only while the roll is on. Assisted, yaw and
    heave move setpoints, and there is no roll: the hold has no such axis."""
    rx = 'roll' if roll and not assisted else ''
    yaw, heave = ('turn', 'height') if assisted else ('yaw', 'heave')
    if layout == 'mode1':
        return (yaw, 'surge'), (rx, heave)
    return (yaw, heave), (rx, 'surge')


KEYS = {Qt.Key.Key_W: ('surge', 1), Qt.Key.Key_S: ('surge', -1),
        Qt.Key.Key_D: ('yaw', 1), Qt.Key.Key_A: ('yaw', -1),
        Qt.Key.Key_Right: ('yaw', 1), Qt.Key.Key_Left: ('yaw', -1),
        Qt.Key.Key_Up: ('heave', 1), Qt.Key.Key_Down: ('heave', -1),
        Qt.Key.Key_E: ('roll', 1), Qt.Key.Key_Q: ('roll', -1)}


def key_axes(held, shift):
    """Held keys to (surge, yaw, heave, roll): opposite keys cancel."""
    out = {a: 0 for a in AXES}
    for k in held:
        if k in KEYS:
            axis, sign = KEYS[k]
            out[axis] += sign
    gain = 1.0 if shift else KEY_HALF
    return tuple(clamp(out[a], -1, 1) * gain for a in AXES)


# ------------------------------------------------------------------- page

LEGEND_PC = (('W / S', 'surge: forward / back'), ('A / D, ← / →', 'yaw: left / right'),
             ('↑ / ↓', 'heave: up / down'), ('Q / E', 'roll: the rear pair apart, while Roll is on'),
             ('Shift', 'full stick; a key alone gives half'),
             ('Space, Esc', 'STOP'), ('gamepad', 'if one is plugged in, as in Steam Deck mode'))
LEGEND_DECK = {'mode2': (('left stick', 'heave ↕ · yaw ↔'), ('right stick', 'surge ↕')),
               'mode1': (('left stick', 'surge ↕ · yaw ↔'), ('right stick', 'heave ↕'))}
LEGEND_DECK_BUTTONS = (('A', 'arm manual control (sticks at rest)'), ('B', 'STOP'),
                       ('D-pad ↑ / ↓', 'limit +5 / −5 %'))
LEGEND_ASSIST_PC = (('W / S', 'surge: a push forward / back'), ('A / D, ← / →', 'turn; let go and it holds the heading'),
                    ('↑ / ↓', 'move the height; let go and it holds'), ('Shift', 'full stick; a key alone gives half'),
                    ('L', 'land'), ('Space, Esc', 'STOP: the motors coast in the air'),
                    ('gamepad', 'if one is plugged in, as in Steam Deck mode'))
# The buttons only: the pads name the sticks, and the Deck's 800 px have no row to spare.
LEGEND_ASSIST_DECK = (('A', 'take off (sticks at rest)'), ('X or Y', 'land'), ('B', 'STOP: coast in the air'),
                         ('D-pad ↑ / ↓', 'height ±%.1f m' % Ax.HEIGHT_STEP_M),
                         ('D-pad ← / →', 'trim −%.1f / +%.1f gf' % (Ax.TRIM_STEP_GF, Ax.TRIM_STEP_GF)))
LEGEND_AUTO_PC = (('L', 'land'), ('Space, Esc', 'STOP: the motors coast in the air'),
                  ('sticks and keys', 'nothing: the course flies itself'))
LEGEND_AUTO_DECK = (('A', 'fly the course (sticks at rest)'), ('X or Y', 'land'), ('B', 'STOP: coast in the air'),
                    ('D-pad ← / →', 'trim −%.1f / +%.1f gf' % (Ax.TRIM_STEP_GF, Ax.TRIM_STEP_GF)),
                    ('sticks', 'nothing: the course flies itself'))
KINDS = ('manual', 'assisted', 'auto')
KIND_NAMES = {'manual': 'manual', 'assisted': 'assisted', 'auto': 'autonomous'}


def legend(rows):
    return '<br>'.join('%s&nbsp; <span style="color:%s">%s</span>' % (code(k), T.MUTED, v) for k, v in rows)


class ControlPage(QWidget):
    """Command (mode, arming, source, legend) | Sticks (the pads and the
    axes) | Motors (the blimp's readback, as on the Flight page)."""
    armed_changed = pyqtSignal(bool)
    stop_requested = pyqtSignal()           # B on the pad: the window's STOP, flash included
    mode_changed = pyqtSignal(str)          # 'pc' or 'deck'
    HINT = ('Pick the input, arm, then fly: the page mixes surge, yaw and heave into the four motors, and the '
            'station repeats the last command to the blimp until you change it. Let go and everything ramps back '
            'to zero; STOP or a lost pad coast the motors at once, a window frozen for 3 s releases them. Arming '
            'needs telemetry and the sticks at rest. Roll (off by default) adds the rear pair\'s difference from '
            'the right stick\'s free axis or Q / E, an experiment: on paper a tilt, not a turn, paid for in lift at '
            'hover.')
    HINT_AUTO = ('Autonomous: the course on the race sheet (the Course panel), flown from here on the blimp\'s loops '
                 'the way the flight tests were: the climb to the first balloon\'s height, the look, the nose on '
                 'the balloon, straight pulses toward it, the touch, the move away, the turn to the next, each '
                 'balloon\'s own height, and the landing. Fly the course prepares as Take off does (bench '
                 'parameters, 60 s of stillness, the ToF, a row from the forward camera) and needs the trim. The '
                 'sticks do nothing; Land brings it down at any point, STOP coasts the motors where it is. The '
                 'camera is what finds each balloon: nothing on board measures position.')
    HINT_ASSIST = ('Assisted: the blimp\'s own loops keep the height and the heading, and the sticks move what they '
                   'keep. Take off sets the bench parameters, waits for 60 s of stillness on the floor and climbs to '
                   'the height in the box. The height stick and the D-pad move the setpoint, the turn stick swings '
                   'the nose and stops it when let go, and the surge stick pushes on top of the trim. Nothing '
                   'measures speed, so pull the stick back to brake. Land comes down by itself and stops the motors. '
                   'It also lands by itself if the height reading goes wrong, the window freezes or the pad goes '
                   'away. STOP coasts the motors where the blimp is, and it falls: a last resort.')
    LAND_MANUAL = ('Land needs an assisted flight: the blimp\'s landing runs on its hold, and a hold can only start '
                   'on the floor (the estimator needs 60 s of stillness, which flying resets). In manual, let go and '
                   'the blimp sinks on its trim, faster than a landing.')

    def __init__(self, station, settings):
        super().__init__()
        self.station, self.settings = station, settings
        self.armed = False
        self.alive = False
        self.tel = None
        self.mode = 'pc'
        self.layout_id = 'mode2'
        self.limit = LIMIT_DEFAULT / 100.0
        self.cmd = [0.0] * len(AXES)            # surge, yaw, heave, roll after the ramp
        self.target = (0.0,) * len(AXES)
        self.raw = (0.0,) * len(AXES)           # the pad's sticks before shaping
        self.held, self.shift = set(), False
        self.roll_on = False                    # the Roll switch: the rear pair's difference, off by default
        self.kind = 'manual'                    # the Fly switch: manual, assisted or auto (the course)
        self.assisted = False                   # a flight on core/'s loops: assisted or auto
        self.flight = {}                        # assist.py's snapshot, each telemetry tick
        self.pad = None                         # the last gamepad snapshot
        self.pad_was = {b: False for b in PAD_BUTTONS}
        self.source = 'keys'                    # what moved last: 'keys' or 'pad'
        self._compact = False

        self.command = self._build_command()
        self.sticks = self._build_sticks()
        self.course_panel = CoursePanel(station)       # in the sticks' place while the course is the way to fly
        self.motors = self._build_motors()
        self.command.setFixedWidth(340)
        self.motors.setFixedWidth(340)
        self.setLayout(hbox(self.command, self.sticks, self.course_panel, self.motors, spacing=12,
                            margins=(0, 12, 0, 0)))
        self.layout().setStretchFactor(self.sticks, 1)
        self.layout().setStretchFactor(self.course_panel, 1)
        self.setMinimumHeight(200)

        mode = settings.value('control/mode')
        self.set_mode(mode if mode in MODES else 'pc', save=False)
        lay = settings.value('control/layout')
        self.set_layout(lay if lay in [l[0] for l in LAYOUTS] else 'mode2', save=False)
        lim = settings.value('control/limit')
        if lim not in (None, ''):
            self.limit_in.setValue(int(lim))
        self.set_roll(str(settings.value('control/roll')).lower() == 'true', save=False)
        for key, box in (('control/height', self.height_in), ('control/trim', self.trim_in),
                         ('control/surge', self.surge_in)):
            v = settings.value(key)
            if v not in (None, ''):
                box.setValue(float(v))
        fly = settings.value('control/fly')
        if fly not in KINDS:        # the key before the course: a window of 2026-11-04 remembered Assisted
            fly = 'assisted' if str(settings.value('control/assisted')).lower() == 'true' else 'manual'
        self.set_kind(fly, save=False)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.tick)
        self._timer.start(TICK_MS)
        self.render()

    # ------------------------------------------------------------- panels

    def _build_command(self):
        pn = Panel('Command', info_text('Who drives, and how far.', self.HINT, self.HINT_ASSIST, self.HINT_AUTO))
        lay = pn.lay
        self.mode_b = {}
        group = QButtonGroup(self)
        group.setExclusive(True)
        row = hbox(spacing=4)
        for m in MODES:
            b = button(MODE_NAMES[m], 'toggle', tip={'pc': 'keyboard, plus a gamepad if one is plugged in',
                                                     'deck': "the Steam Deck's own controls; hints hidden for its screen"}[m])
            b.setCheckable(True)
            b.clicked.connect(lambda _=False, m=m: self.set_mode(m))
            group.addButton(b)
            row.addWidget(b)
            self.mode_b[m] = b
        row.addStretch(1)
        lay.addWidget(h3('Input'))
        lay.addSpacing(6)
        lay.addLayout(row)
        lay.addSpacing(8)
        self.layout_in = QComboBox()
        for lid, text, _long in LAYOUTS:
            self.layout_in.addItem(text, lid)
        self.layout_in.currentIndexChanged.connect(lambda i: self.set_layout(self.layout_in.itemData(i)))
        lay.addWidget(self.layout_in)
        lay.addSpacing(8)
        self.source_l = label('—', 12, color=T.MUTED, mono=True, wrap=True)
        lay.addWidget(self.source_l)
        self.pad_notice = notice('')
        lay.addSpacing(6)
        lay.addWidget(self.pad_notice)
        lay.addSpacing(14)

        lay.addWidget(h3('Fly'))
        lay.addSpacing(6)
        self.kind_b = {}
        kinds = QButtonGroup(self)
        kinds.setExclusive(True)
        row = hbox(spacing=4)
        for kind, text, tip in (('manual', 'Manual', 'the sticks are the motors: you are the height loop'),
                                ('assisted', 'Assisted', "the blimp's loops hold height and heading: the sticks move "
                                                         'what they hold'),
                                ('auto', 'Autonomous', 'the course on the race sheet, flown from here on the '
                                                       "blimp's loops: the sticks do nothing")):
            b = button(text, 'toggle', tip=tip)
            b.setCheckable(True)
            b.clicked.connect(lambda _=False, kind=kind: self.set_kind(kind))
            kinds.addButton(b)
            row.addWidget(b)
            self.kind_b[kind] = b
        row.addStretch(1)
        lay.addLayout(row)
        lay.addSpacing(10)
        self.arm_sw = Switch('Manual control')
        self.arm_sw.setToolTip('arm: the sticks drive the motors · off: coast (A / B on the pad)')
        self.arm_sw.toggled.connect(self._arm_toggled)
        if self.station.race:
            self.arm_sw.setEnabled(False)
            self.arm_sw.setToolTip('race mode: no manual control')
        self.limit_in = QSpinBox()
        self.limit_in.setRange(5, 100)
        self.limit_in.setSingleStep(5)
        self.limit_in.setValue(LIMIT_DEFAULT)
        self.limit_in.setKeyboardTracking(False)
        self.limit_in.setFixedWidth(64)
        self.limit_in.setToolTip("full stick is this command (1 is the blimp's duty ceiling); "
                                 'D-pad up and down step it')
        self.limit_in.valueChanged.connect(self._limit_changed)
        lim = QWidget()
        lim.setLayout(hbox(label('Limit', color=T.MUTED), self.limit_in, label('%', color=T.MUTED), spacing=6))
        self.roll_sw = Switch('Roll')
        self.roll_sw.setToolTip("the rear pair apart, RL harder to roll right, from the right stick's ↔ or Q / E; "
                                'off: the rear pair is heave only. An experiment: on paper the pair at ±5.5 cm '
                                'tilts, it does not turn, and at hover the difference comes out of the lift')
        self.roll_sw.toggled.connect(self.set_roll)
        controls = FlowBox(hgap=16, vgap=10)
        controls.add(self.arm_sw)
        controls.add(lim)
        controls.add(self.roll_sw)
        lay.addWidget(controls)
        self.manual_box = controls

        self.takeoff_b = button('Take off', 'primary', tip='bench parameters, 60 s of stillness, then the climb '
                                                           '(A on the pad)')
        self.takeoff_b.clicked.connect(lambda: self.arm(self.source))
        self.land_b = button('Land', 'land', tip='down to the floor and motors off (X or Y on the pad, L on the keyboard)')
        self.land_b.clicked.connect(self.land)
        self.land_note = InfoIcon(self.LAND_MANUAL)
        self.height_in = self._number(Ax.HEIGHT_MIN_M, Ax.HEIGHT_MAX_M, Ax.HEIGHT_STEP_M, Ax.HEIGHT_DEFAULT_M, 2,
                                      'the height to hold, m; in the air it is the setpoint, and the stick and the '
                                      'D-pad move it', self._height_changed)
        self.surge_in = self._number(0.5, Ax.SURGE_FULL_MAX_GF, 0.5, Ax.SURGE_FULL_DEFAULT_GF, 1,
                                     'full stick is this push, gf; test 5 flew 3 gf (about 0.15 m/s after 3 s)',
                                     self._surge_changed)
        self.trim_in = self._number(-Ax.TRIM_MAX_GF, Ax.TRIM_MAX_GF, Ax.TRIM_STEP_GF, Ax.TRIM_DEFAULT_GF, 1,
                                    'the push with the stick at rest, gf: the rear motors lean forward, so without '
                                    'it the blimp drifts. -0.6 for the new battery type, -1 for the earlier cells; '
                                    'the D-pad nudges it in flight', self._trim_changed)
        boxes = FlowBox(hgap=12, vgap=8)
        self.box_w = {}
        for name, box, unit in (('Height', self.height_in, 'm'), ('Surge', self.surge_in, 'gf'),
                                ('Trim', self.trim_in, 'gf')):
            w = QWidget()
            w.setLayout(hbox(label(name, color=T.MUTED), box, label(unit, color=T.MUTED), spacing=6))
            boxes.add(w)
            self.box_w[name.lower()] = w
        lay.addWidget(boxes)
        self.assist_box = boxes
        lay.addSpacing(8)
        lay.addLayout(hbox(self.takeoff_b, 6, self.land_b, 6, self.land_note, None, spacing=0))
        lay.addSpacing(8)
        self.state_l = label('DISARMED', 13, 600, mono=True)
        lay.addWidget(self.state_l)
        lay.addSpacing(14)

        self.legend_h = h3('Keys')
        lay.addWidget(self.legend_h)
        lay.addSpacing(4)
        self.legend_l = label('', 12, wrap=True)
        self.legend_l.setTextFormat(Qt.TextFormat.RichText)
        lay.addWidget(self.legend_l)
        lay.addStretch(1)
        return pn

    def _number(self, lo, hi, step, value, decimals, tip, changed):
        """A number box. It gives the focus back when done: the page's keys and
        Space as STOP are off while a box has it (app.py, Keys)."""
        box = QDoubleSpinBox()
        box.setRange(lo, hi)
        box.setSingleStep(step)
        box.setDecimals(decimals)
        box.setLocale(QLocale.c())      # 1.00 as every other number on the page, whatever the system's locale
        box.setValue(value)
        box.setKeyboardTracking(False)
        box.setFixedWidth(78)
        box.setToolTip(tip)
        box.valueChanged.connect(changed)
        box.editingFinished.connect(box.clearFocus)
        return box

    def _build_sticks(self):
        pn = Panel('Sticks', 'the command the page derives; papaya while armed')
        lay = pn.lay
        self.pad_l, self.pad_r = StickPad('Left stick'), StickPad('Right stick')
        self.pads_row = hbox(None, self.pad_l, 24, self.pad_r, None, spacing=0)
        lay.addSpacing(6)
        lay.addLayout(self.pads_row)
        lay.addSpacing(18)
        g = QGridLayout()
        g.setHorizontalSpacing(10)
        g.setVerticalSpacing(10)
        g.setColumnMinimumWidth(0, 56)
        g.setColumnMinimumWidth(2, 64)
        g.setColumnStretch(1, 1)
        self.axis_bars, self.axis_vals, self.axis_names = {}, {}, {}
        self.roll_row = []                      # hidden while the roll is off, so the panel reads as before
        for i, (name, words) in enumerate((('surge', 'back · forward'), ('yaw', 'left · right'),
                                           ('heave', 'down · up'), ('roll', 'RR harder · RL harder'))):
            bar, val = CentreBar(), label('+0.00', 13, mono=True)
            val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            row = (label(name, 12, 700, T.MUTED, spacing=0.08, caps=True), bar, val, label(words, 11, color=T.FAINT))
            for col, w in enumerate(row):
                g.addWidget(w, i, col)
            self.axis_bars[name], self.axis_vals[name], self.axis_names[name] = bar, val, row[0]
            if name == 'roll':
                self.roll_row = list(row)
        lay.addLayout(g)
        lay.addSpacing(12)
        self.mix_l = label('', 12, color=T.FAINT, mono=True, wrap=True)
        lay.addWidget(self.mix_l)
        self.height_r, self.heading_r = Readout('Height', 'm'), Readout('Heading', '°')
        self.surge_r = Readout('Surge', 'gf')
        self.flight_box = QWidget()
        row = hbox(self.height_r, self.heading_r, self.surge_r, spacing=10)
        # Sized for the Deck, where the panel is about 510 px: a setpoint that
        # elides to '1.…' is the one number the operator came to read.
        for r, stretch in ((self.height_r, 3), (self.heading_r, 3), (self.surge_r, 2)):
            r.value.setFont(T.font(15, 600, mono=True))
            row.setStretchFactor(r, stretch)
        self.flight_box.setLayout(row)
        lay.addWidget(self.flight_box)
        lay.addSpacing(6)
        self.flight_l = label('', 12, color=T.MUTED, wrap=True)
        lay.addWidget(self.flight_l)
        lay.addStretch(1)
        return pn

    def _build_motors(self):
        pn = Panel('Motors', info_text("The blimp's readback, not the command.",
                                       'Arrows and rings follow the telemetry; a command that did not arrive '
                                       'shows as nothing here.'))
        lay = pn.lay
        self.top = TopView()
        self.top.set_max_width(300)
        lay.addWidget(self.top)
        lay.addSpacing(10)
        g = QGridLayout()
        g.setHorizontalSpacing(8)
        g.setVerticalSpacing(4)
        g.setColumnMinimumWidth(0, 30)
        g.setColumnMinimumWidth(2, 56)
        g.setColumnStretch(1, 1)
        self.rb_bars, self.rb_vals = {}, {}
        for i, m in enumerate(MOTORS):
            bar, val = SegBar(), label('—', 13, mono=True)
            val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            g.addWidget(label(m, 14, 900, T.MUTED, spacing=0.08), i, 0)
            g.addWidget(bar, i, 1)
            g.addWidget(val, i, 2)
            self.rb_bars[m], self.rb_vals[m] = bar, val
        lay.addLayout(g)
        lay.addStretch(1)
        return pn

    # ------------------------------------------------------------ settings

    def set_mode(self, mode, save=True):
        self.mode = mode
        for m, b in self.mode_b.items():
            b.setChecked(m == mode)
        deck = mode == 'deck'
        self.legend_h.setText('Controls' if deck else 'Keys')       # assisted or not
        if save:
            self.settings.setValue('control/mode', mode)
            self.station.log('info', 'control: %s mode' % MODE_NAMES[mode])
        self._legend()
        self.render()
        self.mode_changed.emit(mode)

    def set_layout(self, lid, save=True):
        self.layout_id = lid
        idx = [l[0] for l in LAYOUTS].index(lid)
        self.layout_in.setToolTip(LAYOUTS[idx][2])
        if self.layout_in.currentIndex() != idx:
            self.layout_in.blockSignals(True)
            self.layout_in.setCurrentIndex(idx)
            self.layout_in.blockSignals(False)
        self._pad_axes()
        if save:
            self.settings.setValue('control/layout', lid)
        self._legend()

    def _pad_axes(self):
        (lx, ly), (rx, ry) = pad_names(self.layout_id, self.roll_on, self.assisted)
        self.pad_l.set_axes(lx, ly)
        self.pad_r.set_axes(rx, ry)

    def _legend(self):
        if self.kind == 'auto':
            self.legend_l.setText(legend(LEGEND_AUTO_DECK if self.mode == 'deck' else LEGEND_AUTO_PC))
        elif self.assisted:
            self.legend_l.setText(legend(LEGEND_ASSIST_DECK if self.mode == 'deck' else LEGEND_ASSIST_PC))
        elif self.mode == 'deck':
            rows = LEGEND_DECK[self.layout_id]
            if self.roll_on:
                rows = tuple((k, v + ' · roll ↔' if k == 'right stick' else v) for k, v in rows)
            self.legend_l.setText(legend(rows + LEGEND_DECK_BUTTONS))
        else:
            self.legend_l.setText(legend(LEGEND_PC))

    def compact(self, on):
        self._compact = on
        for pad in (self.pad_l, self.pad_r):
            pad.set_size(150 if on else 190)
        self.top.set_max_width(240 if on else 300)

    def _limit_changed(self, pct):
        self.limit = clamp(pct / 100.0, 0.05, 1.0)
        self.settings.setValue('control/limit', pct)
        self.station.log('warn' if self.limit > 0.5 else 'info', 'control: limit %d %%' % pct)
        self.render()

    def set_roll(self, on, save=True):
        """The roll axis on or off. Off, the right stick's X and Q / E are
        nothing, and the axis row and the pad's label go, so the page reads
        as it did; on, a command already on the stick ramps in like any
        other, and off again ramps it out: nothing jumps either way."""
        on = bool(on)
        self.roll_on = on
        if self.roll_sw.isChecked() != on:
            self.roll_sw.blockSignals(True)
            self.roll_sw.setChecked(on)
            self.roll_sw.blockSignals(False)
        for w in self.roll_row:
            w.setVisible(on and not self.assisted)
        self._pad_axes()
        if save:
            self.settings.setValue('control/roll', on)
            if on:
                self.station.log('warn', 'control: roll on: the rear pair apart from the right stick and Q / E; '
                                         'at hover the difference comes out of the lift')
            else:
                self.station.log('info', 'control: roll off: the rear pair is heave only')
        self._legend()
        self.render()

    def set_kind(self, kind, save=True):
        """Manual, assisted or auto (the course). Not in the air: a hold
        cannot start there, and a running one has no open-loop way out."""
        if kind not in KINDS:
            kind = 'manual'
        if not self.armed:
            self.kind = kind
            if save:
                self.settings.setValue('control/fly', kind)
                self.settings.setValue('control/assisted', kind == 'assisted')
                self.station.log('info', 'control: %s flight' % KIND_NAMES[kind])
        elif kind != self.kind:
            self.station.log('warn', 'control: land or disarm before changing how you fly')
        kind = self.kind
        self.assisted = kind != 'manual'
        auto = kind == 'auto'
        for k, b in self.kind_b.items():
            b.setChecked(k == kind)
        self.manual_box.setVisible(not self.assisted)
        self.assist_box.setVisible(self.assisted)
        self.box_w['height'].setVisible(kind == 'assisted')
        self.box_w['surge'].setVisible(kind == 'assisted')
        self.takeoff_b.setVisible(self.assisted)
        self.takeoff_b.setText('Fly the course' if auto else 'Take off')
        self.takeoff_b.setToolTip('bench parameters, 60 s of stillness, a camera row, then the climb to balloon 1 '
                                  '(A on the pad)' if auto else
                                  'bench parameters, 60 s of stillness, then the climb (A on the pad)')
        self.land_note.setVisible(not self.assisted)
        self.mix_l.setVisible(not self.assisted)
        self.flight_box.setVisible(kind == 'assisted')
        self.flight_l.setVisible(kind == 'assisted')
        self.sticks.setVisible(not auto)
        self.course_panel.setVisible(auto)
        for w in self.roll_row:
            w.setVisible(self.roll_on and not self.assisted)
        self.axis_names['yaw'].setText('turn' if self.assisted else 'yaw')
        self.axis_names['heave'].setText('height' if self.assisted else 'heave')
        self._pad_axes()
        self._legend()
        self.render()

    def set_assisted(self, on, save=True):
        """Manual or assisted, the switch before the course (2026-11-04)."""
        self.set_kind('assisted' if on else 'manual', save)

    def _height_changed(self, v):
        if self.armed and self.assisted:
            self.station.assist_height(v)

    def _surge_changed(self, v):
        self.settings.setValue('control/surge', v)
        if self.armed and self.assisted:
            self.station.assist_tune(surge_full_gf=v)

    def _trim_changed(self, v):
        self.settings.setValue('control/trim', v)
        if self.armed and self.assisted:
            self.station.assist_tune(trim_gf=v)

    # -------------------------------------------------------------- arming

    def _arm_toggled(self, on):
        if on:
            if not self.arm(self.source):
                self.arm_sw.blockSignals(True)
                self.arm_sw.setChecked(False)
                self.arm_sw.blockSignals(False)
        else:
            self.disarm()

    def arm(self, source):
        """True when armed. Refused, with the reason in the events, without
        telemetry (the readback could not show what the motors do) or with a
        stick off centre (the motors would jump)."""
        if self.armed:
            return True
        if self.station.race:
            self.station.log('warn', 'control: race mode, no manual control')
            return False
        if not self.alive:
            self.station.log('warn', 'control: no telemetry from the blimp, not arming')
            return False
        if any(abs(v) > 0 for v in self.target):
            self.station.log('warn', 'control: sticks at rest and keys up before arming')
            return False
        by = 'gamepad' if source == 'pad' else 'keyboard'
        if self.kind == 'auto':
            sheet = self.course_panel.sheet_now()
            why = self.station.course_takeoff(sheet, self.trim_in.value())
            if why:
                self.station.log('warn', 'control: not flying the course: %s' % why)
                return False
            self.flight = {'state': 'preparing'}
            self.station.log('info', 'the course from the %s: %s; trim %+.1f gf'
                             % (by, rc.summary(sheet), self.trim_in.value()))
        elif self.assisted:
            why = self.station.assist_takeoff(self.height_in.value(), self.trim_in.value(), self.surge_in.value())
            if why:
                self.station.log('warn', 'control: not taking off: %s' % why)
                return False
            self.settings.setValue('control/height', self.height_in.value())
            self.flight = {'state': 'preparing'}
            self.station.log('info', 'assisted flight from the %s: to %.2f m, trim %+.1f gf, full stick %.1f gf'
                             % (by, self.height_in.value(), self.trim_in.value(), self.surge_in.value()))
        self.armed = True
        self.cmd = [0.0] * len(AXES)
        if not self.assisted:
            self.arm_sw.blockSignals(True)
            self.arm_sw.setChecked(True)
            self.arm_sw.blockSignals(False)
            self.station.log('info', 'manual control armed from the %s, limit %d %%%s'
                             % (by, round(self.limit * 100), ', roll on' if self.roll_on else ''))
        self.armed_changed.emit(True)
        self.render()
        return True

    def disarm(self, send_stop=True):
        """Off, with a STOP unless the caller sends its own (the window's
        STOP does, for both pages at once)."""
        self.cmd = [0.0] * len(AXES)
        if not self.armed:
            self.render()
            return
        self.armed = False
        self.arm_sw.blockSignals(True)
        self.arm_sw.setChecked(False)
        self.arm_sw.blockSignals(False)
        if send_stop:
            self.station.stop(False)
        self.armed_changed.emit(False)
        self.render()

    def land(self):
        """The Land button, X or Y on the pad, L on the keyboard. Before the
        motors start, it calls the take-off off."""
        if self.assisted and self.armed:
            self.station.assist_land()
        elif not self.assisted:
            self.station.log('warn', 'control: Land is part of an assisted flight; in manual, let go and it sinks')

    # ---------------------------------------------------------------- keys

    def key(self, k, pressed):
        """From the window's key filter: True when the key is one of ours."""
        if k in (Qt.Key.Key_Shift,):
            self.shift = pressed
            return True
        if k == Qt.Key.Key_L and self.assisted:
            if pressed:
                self.land()
            return True
        if k not in KEYS:
            return False
        if pressed:
            self.held.add(k)
        else:
            self.held.discard(k)
        return True

    def release_keys(self):
        """The window lost focus or the page changed: a release may never
        come, and a stuck key must not keep a motor running."""
        self.held.clear()
        self.shift = False

    # ---------------------------------------------------------------- tick

    def tick(self):
        pad = self.station.pad_state()
        was = self.pad
        self.pad = pad
        if was is not None and was['connected'] and not pad['connected']:
            if self.armed and self.source == 'pad' and self.assisted:
                self.station.log('crit', 'control: the gamepad went away in the flight: landing')
                self.station.assist_land('the gamepad went away')
            elif self.armed and self.source == 'pad':
                self.station.log('crit', 'control: the gamepad went away while driving: STOP')
                self.stop_requested.emit()
            else:
                self.station.log('warn', 'control: gamepad disconnected')
        elif pad['connected'] and (was is None or not was['connected']):
            self.station.log('ok', 'control: gamepad %s' % pad['name'])
        if pad['connected']:
            for b in PAD_BUTTONS:
                now = pad['buttons'][b]
                if now and not self.pad_was[b]:
                    self._pad_button(b)
                self.pad_was[b] = now
            self.raw = pad_axes(pad['axes'], self.layout_id)
        else:
            self.raw = (0.0,) * len(AXES)
            self.pad_was = {b: False for b in PAD_BUTTONS}
        roll = self.roll_on and not self.assisted
        if not roll:                            # off, the right stick's X and Q / E are nothing
            self.raw = self.raw[:3] + (0.0,)
        shaped = tuple(shape_axis(v) for v in self.raw)
        keys = key_axes(self.held, self.shift)
        if not roll:
            keys = keys[:3] + (0.0,)
        if any(shaped):
            self.source = 'pad'
        elif any(keys):
            self.source = 'keys'
        self.target = tuple(clamp(s + k, -1.0, 1.0) for s, k in zip(shaped, keys))
        step = TICK_MS / 1000.0 / SLEW_S
        self.cmd = [slew(c, t, step) for c, t in zip(self.cmd, self.target)]
        if self.armed and self.kind == 'assisted':
            # The turn and the height are rates, ramped by what they move, and
            # a let-go must be seen at once: the unramped target. The surge is
            # a push on the motors, so it ramps as it does in manual.
            self.station.assist_sticks(self.cmd[0], self.target[1], self.target[2])
        elif self.armed and not self.assisted:
            self.station.set_motors(teleop_mix(self.cmd[0], self.cmd[1], self.cmd[2], self.limit, self.cmd[3]))
        self.render()

    def _pad_button(self, b):
        if b == 'a':
            self.arm('pad')
        elif b == 'b':
            self.stop_requested.emit()
        elif b in ('x', 'y'):
            # Both: nothing else uses them, and these two are the pair whose
            # event codes drivers disagree on (the kernel's BTN_NORTH is BTN_X).
            # A and B are checked on a Deck; X and Y are not yet, and a Land
            # that does nothing is not a thing to find out in the air.
            self.land()
        elif b in ('up', 'down') and self.kind == 'assisted':
            step = Ax.HEIGHT_STEP_M if b == 'up' else -Ax.HEIGHT_STEP_M
            if self.armed:
                self.station.assist_step_height(step)
            else:
                self.height_in.setValue(self.height_in.value() + step)
        elif b in ('left', 'right') and self.assisted:
            self.trim_in.setValue(self.trim_in.value() + (Ax.TRIM_STEP_GF if b == 'right' else -Ax.TRIM_STEP_GF))
        elif b == 'up' and not self.assisted:
            self.limit_in.setValue(self.limit_in.value() + 5)
        elif b == 'down' and not self.assisted:
            self.limit_in.setValue(self.limit_in.value() - 5)
        elif b == 'start':
            self.station.log('info', 'control: Start does nothing; A arms, takes off or flies the course, by the '
                                     'Fly switch')

    # ----------------------------------------------------------- readback

    def on_tel(self, m):
        self.tel, self.alive = m.get('tel'), bool(m.get('alive'))
        self.flight = m.get('assist') or {}
        if self.armed and self.assisted and self.flight.get('state', 'idle') == 'idle':
            self.disarm(send_stop=False)        # landed, called off or killed; assist.py said why
        self.render_readback()
        if self.assisted:
            self.render_flight()
        if self.kind == 'auto':
            self.course_panel.set_flight(self.flight)

    def set_stale(self):
        self.alive = False
        self.render_readback()

    def render_readback(self):
        live = self.tel is not None and self.alive
        values = {}
        for m in MOTORS:
            v = self.tel['motor'][m] if live else float('nan')
            values[m] = v
            self.rb_vals[m].setText(sfmt(v, 3))
            self.rb_bars[m].set(v)
        self.top.set(values)

    def render_flight(self):
        """The Sticks panel's readouts and the Command panel's buttons, from
        assist.py's snapshot. The boxes follow the flight, unless being typed in."""
        f = self.flight
        state = f.get('state', 'idle')
        up = state in ('flying', 'landing')
        tof, yaw = f.get('tof_m'), f.get('yaw_deg')
        self.height_r.set('%s → %.2f' % ('—' if tof is None else '%.2f' % tof, f['height_m']) if up else '—')
        self.heading_r.set('%s → %+.0f' % ('—' if yaw is None else '%+.0f' % yaw, f['heading_deg']) if up else '—')
        self.surge_r.set('%+.1f' % f['surge_gf'] if up else '—')
        if f.get('link_lost'):
            self.flight_l.setText('Link lost: the blimp stops its motors after 1 s, and the hold goes on if the link '
                                  'returns.')
            set_color(self.flight_l, T.AMBER)
        else:
            self.flight_l.setText(f.get('detail') or ('Flying: let go and it holds.' if state == 'flying' else ''))
            set_color(self.flight_l, T.MUTED)
        # Flying only: the landing's setpoints would leave the box at 0.30 m,
        # and the next take-off would climb to that. Assisted only: the
        # course's heights are the sheet's, not the box's.
        if state == 'flying' and self.kind == 'assisted' and not self.height_in.hasFocus():
            self.height_in.blockSignals(True)
            self.height_in.setValue(f['height_m'])
            self.height_in.blockSignals(False)
        self.land_b.setText('Call off' if state == 'preparing' else 'Land')

    # -------------------------------------------------------------- render

    def render(self):
        surge, yaw, heave, roll = self.cmd
        pad = self.pad
        live = self.armed
        if self.layout_id == 'mode1':
            self.pad_l.set(yaw, surge, (pad['axes']['lx'], pad['axes']['ly']) if pad and pad['connected'] else None,
                           DEADZONE, live)
            self.pad_r.set(roll, heave, (pad['axes']['rx'], pad['axes']['ry']) if pad and pad['connected'] else None,
                           DEADZONE, live)
        else:
            self.pad_l.set(yaw, heave, (pad['axes']['lx'], pad['axes']['ly']) if pad and pad['connected'] else None,
                           DEADZONE, live)
            self.pad_r.set(roll, surge, (pad['axes']['rx'], pad['axes']['ry']) if pad and pad['connected'] else None,
                           DEADZONE, live)
        for name, v in zip(AXES, self.cmd):
            self.axis_bars[name].set(v, live)
            self.axis_vals[name].setText(sfmt(v, 2))
        fl, rl, fr, rr = teleop_mix(surge, yaw, heave, self.limit, roll)
        self.mix_l.setText('to the blimp  FL %+.3f  FR %+.3f  RL %+.3f  RR %+.3f%s'
                           % (fl, fr, rl, rr, '' if self.armed else '  (not armed: nothing sent)'))
        # state line
        if self.armed and self.assisted:
            state = self.flight.get('state', 'preparing')
            flying = 'AUTONOMOUS' if self.kind == 'auto' else 'ASSISTED'
            self.state_l.setText('%s · %s' % ({'flying': flying, 'landing': 'LANDING'}.get(state, 'PREPARING'),
                                              'gamepad' if self.source == 'pad' else 'keyboard'))
            set_color(self.state_l, T.AMBER if state == 'landing' else T.PAPAYA)
        elif self.armed:
            self.state_l.setText('ARMED · %s · limit %d %%%s'
                                 % ('gamepad' if self.source == 'pad' else 'keyboard', round(self.limit * 100),
                                    ' · roll' if self.roll_on else ''))
            set_color(self.state_l, T.PAPAYA)
        elif not self.alive:
            self.state_l.setText('DISARMED · no telemetry')
            set_color(self.state_l, T.FAINT)
        else:
            self.state_l.setText('ON THE FLOOR · ready' if self.assisted else 'DISARMED · ready')
            set_color(self.state_l, T.UN_GREEN)
        self.land_b.setEnabled(self.assisted and self.armed and self.flight.get('state') != 'landing')
        self.takeoff_b.setEnabled(self.assisted and not self.armed and not self.station.race)
        # source line and the pad notice
        if pad is None or not pad['connected']:
            why = (pad or {}).get('error') or 'no gamepad'
            self.source_l.setText('keyboard · gamepad: none')
            self.source_l.setToolTip(why)
            if self.mode == 'deck':
                self.pad_notice.setText('The Deck\'s controls are not visible: %s. In Desktop mode the pad is an '
                                        'event device of the hid-steam driver; the keyboard still works.' % why)
            self.pad_notice.setVisible(self.mode == 'deck')
        else:
            self.source_l.setText('gamepad: %s (%s)' % (pad['name'], pad['backend']))
            self.source_l.setToolTip('')
            self.pad_notice.setVisible(False)
