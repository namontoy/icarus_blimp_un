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

Two input modes, kept in QSettings and settable with --mode: PC (the
keyboard, plus a gamepad if one is plugged in) and Steam Deck (the Deck's own
controls through gamepad.py, the legend in the Deck's names, the hints hidden
for its 1280 × 800 screen). The safety chain is the Flight page's: the
setpoint is refreshed every TICK_MS only while armed, the bridge releases the
motors 0.5 s after the refresh stops, and the blimp coasts on its own timers
after that. STOP (the button, Space, Esc, or B on the pad) disarms and
coasts; a pad that disappears while it was driving is a STOP; arming needs
telemetry from the blimp and the sticks at rest, so the motors never jump.
Only one source drives at a time: arming here switches the Flight page's
"Motors enabled" off, and the other way round (app.py).
"""
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import QButtonGroup, QComboBox, QGridLayout, QSizePolicy, QSpinBox, QWidget

from . import theme as T
from .panels import MOTORS, Panel, button, code, h3, notice
from .widgets import CentreBar, FlowBox, SegBar, StickPad, Switch, TopView, clamp, hbox, label, set_color, sfmt

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
PAD_BUTTONS = ('a', 'b', 'start', 'up', 'down')     # the ones the page acts on, on their press edge


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


def teleop_mix(surge, yaw, heave, limit):
    """Surge, yaw (right positive) and heave in −1…+1 to the four duties the
    link carries, FL RL FR RR, scaled by the limit. The front pair is sum and
    difference: the left motor pushes harder to turn right. When the sum
    would exceed one motor, both are scaled together so the turn the
    operator asked for keeps its proportion (core/'s mixer instead protects
    yaw and clamps surge, the right policy for a controller, but a hand on a
    stick expects what it sees). The rear pair is heave, equal on both;
    negative heave reverses them, which the DRV8833s allow."""
    fl, fr = surge + yaw, surge - yaw
    m = max(abs(fl), abs(fr), 1.0)
    fl, fr = fl / m, fr / m
    return [clamp(v * limit, -1.0, 1.0) for v in (fl, heave, fr, heave)]


def slew(current, target, step):
    """One step of a rate limit toward the target."""
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target


def pad_axes(axes, layout):
    """The pad's sticks to (surge, yaw, heave) under a stick layout."""
    if layout == 'mode1':
        return axes['ly'], axes['lx'], axes['ry']
    return axes['ry'], axes['lx'], axes['ly']


def pad_names(layout):
    """((left x, left y), (right x, right y)) axis names for the pads."""
    if layout == 'mode1':
        return ('yaw', 'surge'), ('', 'heave')
    return ('yaw', 'heave'), ('', 'surge')


KEYS = {Qt.Key.Key_W: ('surge', 1), Qt.Key.Key_S: ('surge', -1),
        Qt.Key.Key_D: ('yaw', 1), Qt.Key.Key_A: ('yaw', -1),
        Qt.Key.Key_Right: ('yaw', 1), Qt.Key.Key_Left: ('yaw', -1),
        Qt.Key.Key_Up: ('heave', 1), Qt.Key.Key_Down: ('heave', -1)}


def key_axes(held, shift):
    """Held keys to (surge, yaw, heave): opposite keys cancel."""
    out = {'surge': 0, 'yaw': 0, 'heave': 0}
    for k in held:
        if k in KEYS:
            axis, sign = KEYS[k]
            out[axis] += sign
    gain = 1.0 if shift else KEY_HALF
    return tuple(clamp(out[a], -1, 1) * gain for a in ('surge', 'yaw', 'heave'))


# ------------------------------------------------------------------- page

LEGEND_PC = (('W / S', 'surge: forward / back'), ('A / D, ← / →', 'yaw: left / right'),
             ('↑ / ↓', 'heave: up / down'), ('Shift', 'full stick; a key alone gives half'),
             ('Space, Esc', 'STOP'), ('gamepad', 'if one is plugged in, as in Steam Deck mode'))
LEGEND_DECK = {'mode2': (('left stick', 'heave ↕ · yaw ↔'), ('right stick', 'surge ↕')),
               'mode1': (('left stick', 'surge ↕ · yaw ↔'), ('right stick', 'heave ↕'))}
LEGEND_DECK_BUTTONS = (('A', 'arm manual control (sticks at rest)'), ('B', 'STOP'),
                       ('D-pad ↑ / ↓', 'limit +5 / −5 %'), ('Start (menu)', 'autonomous: not in the firmware yet'))


def legend(rows):
    return '<br>'.join('%s&nbsp; <span style="color:%s">%s</span>' % (code(k), T.MUTED, v) for k, v in rows)


class ControlPage(QWidget):
    """Command (mode, arming, source, legend) | Sticks (the pads and the three
    axes) | Motors (the blimp's readback, as on the Flight page)."""
    armed_changed = pyqtSignal(bool)
    stop_requested = pyqtSignal()           # B on the pad: the window's STOP, flash included
    HINT = ('Pick the input, arm, then fly: the page mixes surge, yaw and heave into the four motors and repeats '
            'the setpoint 20 times a second while armed. Let go and everything ramps back to zero; STOP, a lost '
            'pad or a frozen window coast the motors within half a second. Arming needs telemetry and the sticks '
            'at rest. "Go autonomous" waits for the firmware\'s mode switch.')

    def __init__(self, station, settings):
        super().__init__()
        self.station, self.settings = station, settings
        self.armed = False
        self.alive = False
        self.tel = None
        self.mode = 'pc'
        self.layout_id = 'mode2'
        self.limit = LIMIT_DEFAULT / 100.0
        self.cmd = [0.0, 0.0, 0.0]              # surge, yaw, heave after the ramp
        self.target = (0.0, 0.0, 0.0)
        self.raw = (0.0, 0.0, 0.0)              # the pad's sticks before shaping
        self.held, self.shift = set(), False
        self.pad = None                         # the last gamepad snapshot
        self.pad_was = {b: False for b in PAD_BUTTONS}
        self.source = 'keys'                    # what moved last: 'keys' or 'pad'
        self._compact = False

        self.command = self._build_command()
        self.sticks = self._build_sticks()
        self.motors = self._build_motors()
        self.command.setFixedWidth(340)
        self.motors.setFixedWidth(340)
        self.setLayout(hbox(self.command, self.sticks, self.motors, spacing=12, margins=(0, 12, 0, 0)))
        self.layout().setStretchFactor(self.sticks, 1)
        self.setMinimumHeight(200)

        mode = settings.value('control/mode')
        self.set_mode(mode if mode in MODES else 'pc', save=False)
        lay = settings.value('control/layout')
        self.set_layout(lay if lay in [l[0] for l in LAYOUTS] else 'mode2', save=False)
        lim = settings.value('control/limit')
        if lim not in (None, ''):
            self.limit_in.setValue(int(lim))
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.tick)
        self._timer.start(TICK_MS)
        self.render()

    # ------------------------------------------------------------- panels

    def _build_command(self):
        pn = Panel('Command', 'who drives, and how far')
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

        lay.addWidget(h3('Arm'))
        lay.addSpacing(6)
        self.arm_sw = Switch('Manual control')
        self.arm_sw.setToolTip('arm: the sticks drive the motors · off: coast (A / B on the pad)')
        self.arm_sw.toggled.connect(self._arm_toggled)
        self.limit_in = QSpinBox()
        self.limit_in.setRange(5, 100)
        self.limit_in.setSingleStep(5)
        self.limit_in.setValue(LIMIT_DEFAULT)
        self.limit_in.setKeyboardTracking(False)
        self.limit_in.setFixedWidth(64)
        self.limit_in.setToolTip('full stick is this duty; D-pad up and down step it')
        self.limit_in.valueChanged.connect(self._limit_changed)
        lim = QWidget()
        lim.setLayout(hbox(label('Limit', color=T.MUTED), self.limit_in, label('%', color=T.MUTED), spacing=6))
        controls = FlowBox(hgap=16, vgap=10)
        controls.add(self.arm_sw)
        controls.add(lim)
        lay.addWidget(controls)
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
        lay.addSpacing(14)

        lay.addWidget(h3('Hand-off'))
        lay.addSpacing(6)
        self.auto_b = button('Go autonomous', 'primary',
                             tip='hand the vehicle to blimp_step() on the blimp: needs the MODE_SET packet')
        self.auto_b.setEnabled(False)
        lay.addLayout(hbox(self.auto_b, None))
        lay.addSpacing(4)
        self.auto_note = label('Not in the firmware yet: the mode switch comes with increment 3 of '
                               'docs/control-loop-plan.md (MODE_SET).', 11, color=T.FAINT, wrap=True)
        lay.addWidget(self.auto_note)
        lay.addSpacing(12)
        self.hint = label(self.HINT, 12, color=T.FAINT, wrap=True)
        lay.addWidget(self.hint)
        lay.addStretch(1)
        return pn

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
        self.axis_bars, self.axis_vals = {}, {}
        for i, (name, words) in enumerate((('surge', 'back · forward'), ('yaw', 'left · right'),
                                           ('heave', 'down · up'))):
            bar, val = CentreBar(), label('+0.00', 13, mono=True)
            val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            g.addWidget(label(name, 12, 700, T.MUTED, spacing=0.08, caps=True), i, 0)
            g.addWidget(bar, i, 1)
            g.addWidget(val, i, 2)
            g.addWidget(label(words, 11, color=T.FAINT), i, 3)
            self.axis_bars[name], self.axis_vals[name] = bar, val
        lay.addLayout(g)
        lay.addSpacing(12)
        self.mix_l = label('', 12, color=T.FAINT, mono=True, wrap=True)
        lay.addWidget(self.mix_l)
        lay.addStretch(1)
        return pn

    def _build_motors(self):
        pn = Panel('Motors', "the blimp's readback, not the command")
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
        lay.addSpacing(10)
        self.rb_note = label('', 12, color=T.FAINT, wrap=True)
        lay.addWidget(self.rb_note)
        lay.addStretch(1)
        return pn

    # ------------------------------------------------------------ settings

    def set_mode(self, mode, save=True):
        self.mode = mode
        for m, b in self.mode_b.items():
            b.setChecked(m == mode)
        deck = mode == 'deck'
        self.legend_h.setText('Controls' if deck else 'Keys')
        if save:
            self.settings.setValue('control/mode', mode)
            self.station.log('info', 'control: %s mode' % MODE_NAMES[mode])
        self._legend()
        self._hints()
        self.render()

    def set_layout(self, lid, save=True):
        self.layout_id = lid
        idx = [l[0] for l in LAYOUTS].index(lid)
        self.layout_in.setToolTip(LAYOUTS[idx][2])
        if self.layout_in.currentIndex() != idx:
            self.layout_in.blockSignals(True)
            self.layout_in.setCurrentIndex(idx)
            self.layout_in.blockSignals(False)
        (lx, ly), (rx, ry) = pad_names(lid)
        self.pad_l.set_axes(lx, ly)
        self.pad_r.set_axes(rx, ry)
        if save:
            self.settings.setValue('control/layout', lid)
        self._legend()

    def _legend(self):
        if self.mode == 'deck':
            self.legend_l.setText(legend(LEGEND_DECK[self.layout_id] + LEGEND_DECK_BUTTONS))
        else:
            self.legend_l.setText(legend(LEGEND_PC))

    def _hints(self):
        show = self.mode != 'deck' and not self._compact
        self.hint.setVisible(show)
        self.auto_note.setVisible(show)
        self.rb_note.setText('' if self.mode == 'deck' else
                             'Arrows and rings follow the telemetry; a command that did not arrive shows as '
                             'nothing here.')

    def compact(self, on):
        self._compact = on
        for pad in (self.pad_l, self.pad_r):
            pad.set_size(150 if on else 190)
        self.top.set_max_width(240 if on else 300)
        self._hints()

    def _limit_changed(self, pct):
        self.limit = clamp(pct / 100.0, 0.05, 1.0)
        self.settings.setValue('control/limit', pct)
        self.station.log('warn' if self.limit > 0.5 else 'info', 'control: limit %d %%' % pct)
        self.render()

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
        if not self.alive:
            self.station.log('warn', 'control: no telemetry from the blimp, not arming')
            return False
        if any(abs(v) > 0 for v in self.target):
            self.station.log('warn', 'control: sticks at rest and keys up before arming')
            return False
        self.armed = True
        self.cmd = [0.0, 0.0, 0.0]
        self.arm_sw.blockSignals(True)
        self.arm_sw.setChecked(True)
        self.arm_sw.blockSignals(False)
        self.station.log('info', 'manual control armed from the %s, limit %d %%'
                         % ('gamepad' if source == 'pad' else 'keyboard', round(self.limit * 100)))
        self.armed_changed.emit(True)
        self.render()
        return True

    def disarm(self, send_stop=True):
        """Off, with a STOP unless the caller sends its own (the window's
        STOP does, for both pages at once)."""
        self.cmd = [0.0, 0.0, 0.0]
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

    # ---------------------------------------------------------------- keys

    def key(self, k, pressed):
        """From the window's key filter: True when the key is one of ours."""
        if k in (Qt.Key.Key_Shift,):
            self.shift = pressed
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
            if self.armed and self.source == 'pad':
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
            self.raw = (0.0, 0.0, 0.0)
            self.pad_was = {b: False for b in PAD_BUTTONS}
        shaped = tuple(shape_axis(v) for v in self.raw)
        keys = key_axes(self.held, self.shift)
        if any(shaped):
            self.source = 'pad'
        elif any(keys):
            self.source = 'keys'
        self.target = tuple(clamp(s + k, -1.0, 1.0) for s, k in zip(shaped, keys))
        step = TICK_MS / 1000.0 / SLEW_S
        self.cmd = [slew(c, t, step) for c, t in zip(self.cmd, self.target)]
        if self.armed:
            self.station.set_motors(teleop_mix(self.cmd[0], self.cmd[1], self.cmd[2], self.limit))
        self.render()

    def _pad_button(self, b):
        if b == 'a':
            self.arm('pad')
        elif b == 'b':
            self.stop_requested.emit()
        elif b == 'up':
            self.limit_in.setValue(self.limit_in.value() + 5)
        elif b == 'down':
            self.limit_in.setValue(self.limit_in.value() - 5)
        elif b == 'start':
            self.station.log('warn', 'control: autonomous mode is not in the firmware yet (MODE_SET, '
                                     'control-loop-plan increment 3)')

    # ----------------------------------------------------------- readback

    def on_tel(self, m):
        self.tel, self.alive = m.get('tel'), bool(m.get('alive'))
        self.render_readback()

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

    # -------------------------------------------------------------- render

    def render(self):
        surge, yaw, heave = self.cmd
        pad = self.pad
        live = self.armed
        if self.layout_id == 'mode1':
            self.pad_l.set(yaw, surge, (pad['axes']['lx'], pad['axes']['ly']) if pad and pad['connected'] else None,
                           DEADZONE, live)
            self.pad_r.set(0.0, heave, (pad['axes']['rx'], pad['axes']['ry']) if pad and pad['connected'] else None,
                           DEADZONE, live)
        else:
            self.pad_l.set(yaw, heave, (pad['axes']['lx'], pad['axes']['ly']) if pad and pad['connected'] else None,
                           DEADZONE, live)
            self.pad_r.set(0.0, surge, (pad['axes']['rx'], pad['axes']['ry']) if pad and pad['connected'] else None,
                           DEADZONE, live)
        for name, v in (('surge', surge), ('yaw', yaw), ('heave', heave)):
            self.axis_bars[name].set(v, live)
            self.axis_vals[name].setText(sfmt(v, 2))
        fl, rl, fr, rr = teleop_mix(surge, yaw, heave, self.limit)
        self.mix_l.setText('to the blimp  FL %+.3f  FR %+.3f  RL %+.3f  RR %+.3f%s'
                           % (fl, fr, rl, rr, '' if self.armed else '  (not armed: nothing sent)'))
        # state line
        if self.armed:
            self.state_l.setText('ARMED · %s · limit %d %%'
                                 % ('gamepad' if self.source == 'pad' else 'keyboard', round(self.limit * 100)))
            set_color(self.state_l, T.PAPAYA)
        elif not self.alive:
            self.state_l.setText('DISARMED · no telemetry')
            set_color(self.state_l, T.FAINT)
        else:
            self.state_l.setText('DISARMED · ready')
            set_color(self.state_l, T.UN_GREEN)
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
