"""The Control page offscreen: the pure parts (mix, shaping, keys, the evdev
parser), the platform's pad reader, then the demo window: keys, arming, a fake
gamepad, mutual exclusion with the Flight page's motors, STOP, the Steam Deck
mode and the saved settings, then an assisted flight from the page (test_assist.py
has the flight's own rules)."""
import os
import tempfile
import time

import harness as H
from harness import check, near

from PyQt6.QtCore import QEvent, QSettings, Qt
from PyQt6.QtGui import QKeyEvent
from PyQt6.QtWidgets import QApplication

from software.ground_station import app as A
from software.ground_station import assist as Ax
from software.ground_station import control as Cm
from software.ground_station import gamepad as G

# ------------------------------------------------------------ pure parts
m = Cm.teleop_mix(1, 0, 0, 0.3)
check(all(near(x, y) for x, y in zip(m, [0.3, 0, 0.3, 0])), 'mix: surge only -> front pair at the limit')
m = Cm.teleop_mix(1, 1, 0, 1.0)
check(all(near(x, y) for x, y in zip(m, [1, 0, 0, 0])), 'mix: full surge + full right yaw -> FL 1, FR 0 (scaled)')
m = Cm.teleop_mix(0, 0.5, -0.4, 1.0)
check(all(near(x, y) for x, y in zip(m, [0.5, -0.4, -0.5, -0.4])), 'mix: yaw right alone -> FL +, FR -; heave down reverses both rear')
m = Cm.teleop_mix(0, 0, 1, 0.3, 0.5)
check(all(near(x, y) for x, y in zip(m, [0, 0.3, 0, 0.15])), 'mix: roll at full heave takes from RR only, RL stays at the limit')
m = Cm.teleop_mix(0, 0, 0, 1.0, -1)
check(all(near(x, y) for x, y in zip(m, [0, -1, 0, 1])), 'mix: roll left alone -> RL reversed, RR up')
m = Cm.teleop_mix(0, 0, -0.6, 1.0, -0.6)
check(all(near(x, y) for x, y in zip(m, [0, -1, 0, 0])), 'mix: roll with heave down: RL clamped at full reverse, RR at zero')
check(Cm.shape_axis(0.05) == 0.0 and near(Cm.shape_axis(1.0), 1.0) and near(Cm.shape_axis(-1.0), -1.0),
      'shape: dead band and full travel')
xs = [Cm.shape_axis(i / 50.0) for i in range(51)]
check(all(b >= a for a, b in zip(xs, xs[1:])), 'shape: monotonic')
check(Cm.key_axes({Qt.Key.Key_W}, False) == (0.5, 0.0, 0.0, 0.0), 'keys: W alone is half surge')
check(Cm.key_axes({Qt.Key.Key_W, Qt.Key.Key_S}, True) == (0.0, 0.0, 0.0, 0.0), 'keys: W and S cancel')
check(Cm.key_axes({Qt.Key.Key_Left, Qt.Key.Key_Up}, True) == (0.0, -1.0, 1.0, 0.0), 'keys: Left + Up with Shift')
check(Cm.key_axes({Qt.Key.Key_E}, True) == (0.0, 0.0, 0.0, 1.0) and Cm.key_axes({Qt.Key.Key_Q}, False) == (0.0, 0.0, 0.0, -0.5),
      'keys: E is roll right, Q alone half roll left')
check(Cm.slew(0.0, 1.0, 0.125) == 0.125 and Cm.slew(0.1, 0.0, 0.125) == 0.0, 'slew: step and arrival')
check(Cm.pad_axes({'lx': 0.1, 'ly': 0.2, 'rx': 0.3, 'ry': 0.4, 'lt': 0, 'rt': 0}, 'mode2') == (0.4, 0.1, 0.2, 0.3),
      'pad axes mode 2: right Y surge, left X yaw, left Y heave, right X roll')
check(Cm.pad_axes({'lx': 0.1, 'ly': 0.2, 'rx': 0.3, 'ry': 0.4, 'lt': 0, 'rt': 0}, 'mode1') == (0.2, 0.1, 0.4, 0.3),
      'pad axes mode 1: left Y surge, left X yaw, right Y heave, right X roll')

# evdev parsing without a device: a 64-bit input_event stream
ev = G.Evdev.__new__(G.Evdev)
ev.fd, ev.path, ev.dev_name, ev.error = 3, '/dev/input/event9', 'Steam Deck', ''
ev.absinfo = {G.ABS_X: (-32768, 32767, 1000), G.ABS_Y: (-32768, 32767, 1000), G.ABS_RX: (-32768, 32767, 0),
              G.ABS_RY: (-32768, 32767, 0), G.ABS_Z: (0, 255, 0), G.ABS_RZ: (0, 255, 0),
              G.ABS_HAT0X: (-1, 1, 0), G.ABS_HAT0Y: (-1, 1, 0)}
ev.ax, ev.btn = {}, set()
S = G.INPUT_EVENT
buf = b''.join(S.pack(0, 0, t, c, v) for t, c, v in (
    (G.EV_ABS, G.ABS_Y, -32768), (G.EV_ABS, G.ABS_X, 500), (G.EV_ABS, G.ABS_RY, 16384), (G.EV_ABS, G.ABS_RZ, 255),
    (G.EV_KEY, 0x130, 1), (G.EV_KEY, 0x13b, 1), (G.EV_ABS, G.ABS_HAT0Y, -1), (G.EV_SYN, 0, 0)))
ev.feed(buf)
s = ev.decode()
check(s['connected'] and s['name'] == 'Steam Deck', 'evdev: decoded device')
check(near(s['axes']['ly'], 1.0, 1e-3) and s['axes']['lx'] == 0.0 and near(s['axes']['ry'], -0.5, 1e-3)
      and s['axes']['rx'] == 0.0
      and near(s['axes']['rt'], 1.0), 'evdev: Y inverted (up positive), flat zone, half right stick, trigger')
check(s['buttons']['a'] and s['buttons']['start'] and s['buttons']['up'] and not s['buttons']['b'],
      'evdev: buttons and the hat as D-pad')
ev.feed(S.pack(0, 0, G.EV_KEY, 0x130, 0))
check(not ev.decode()['buttons']['a'], 'evdev: release')

# the platform reader on this machine: must come up and report, pad or not
pad = G.Gamepad().start()
time.sleep(0.4)
st = pad.state()
print('     gamepad here: backend=%s connected=%s error=%r seq=%d' % (st['backend'], st['connected'], st['error'],
                                                                      st['seq']))
check(st['seq'] > 0 or pad.backend is None, 'gamepad: the reader thread polls')
pad.stop()

# ------------------------------------------------------------ the window
app = QApplication([])
ini = H.settings_path()
args = A.parse_args(['--demo'])
station, win = A.build(args, app, QSettings(ini, QSettings.Format.IniFormat), ports=H.free_ports())
win.resize(1680, 1000)
win.show()
station.start()


def spin(s):
    H.pump(app, s)


def key(k, pressed, mod=Qt.KeyboardModifier.NoModifier):
    e = QKeyEvent(QEvent.Type.KeyPress if pressed else QEvent.Type.KeyRelease, k, mod)
    app.sendEvent(win, e)


def readback():
    t = station.blimp.link.telemetry
    return {k: round(v, 3) for k, v in t['motor'].items()} if t else None


spin(1.5)
c = win.control
win.show_page('control')
spin(0.3)
check(c.alive, 'control: telemetry alive in demo')
check(win.pages.currentWidget() is c, 'control: page shown')
check(not c.armed and 'ready' in c.state_l.text(), 'control: disarmed and ready: %s' % c.state_l.text())

# keys before arming move the display, not the motors; arming refuses while a key is held
key(Qt.Key.Key_W, True)
spin(0.6)
check(near(c.cmd[0], 0.5, 1e-6), 'keys: W ramps the surge display to 0.5 while disarmed: %.3f' % c.cmd[0])
c.arm_sw.click()
spin(0.1)
check(not c.armed, 'arm: refused while a key is held')
key(Qt.Key.Key_W, False)
spin(0.6)
check(c.cmd[0] == 0.0, 'keys: release ramps back to zero')
check(readback() == {'FL': 0, 'RL': 0, 'FR': 0, 'RR': 0}, 'nothing reached the mock while disarmed: %s' % readback())

# arm from the switch, fly with the keyboard, see the mock's readback
c.arm_sw.click()
spin(0.1)
check(c.armed and c.arm_sw.isChecked(), 'arm: armed from the switch')
key(Qt.Key.Key_W, True)
spin(0.8)
rb = readback()
check(rb and near(rb['FL'], 0.15, 0.002) and near(rb['FR'], 0.15, 0.002) and rb['RL'] == 0 and rb['RR'] == 0,
      'fly: W at limit 30 %% -> mock readback FL/FR 0.150: %s' % rb)
key(Qt.Key.Key_D, True)
spin(0.8)
rb = readback()
check(rb and rb['FL'] > rb['FR'] and near(rb['FL'], 0.3, 0.002) and near(rb['FR'], 0.0, 0.002),
      'fly: W + D (turn right): FL harder than FR, scaled to the limit: %s' % rb)
key(Qt.Key.Key_W, False)
key(Qt.Key.Key_D, False)
key(Qt.Key.Key_Down, True)
key(Qt.Key.Key_Shift, True)
spin(0.8)
rb = readback()
check(rb and near(rb['RL'], -0.3, 0.002) and near(rb['RR'], -0.3, 0.002) and rb['FL'] == 0,
      'fly: Shift + Down: rear pair reversed at the full limit: %s' % rb)
key(Qt.Key.Key_Down, False)
key(Qt.Key.Key_Shift, False)
spin(0.8)
check(readback() == {'FL': 0, 'RL': 0, 'FR': 0, 'RR': 0}, 'fly: keys up -> zero on the mock')

# the roll: off by default, E is nothing; on, Shift + E splits the rear pair, RL harder
check(not c.roll_on and not c.roll_sw.isChecked() and c.pad_r.xn == '' and not c.roll_row[0].isVisible(),
      'roll: off by default, no label on the right pad, no axis row')
key(Qt.Key.Key_E, True)
key(Qt.Key.Key_Shift, True)
spin(0.6)
check(c.cmd[3] == 0.0 and readback() == {'FL': 0, 'RL': 0, 'FR': 0, 'RR': 0}, 'roll off: E does nothing')
c.roll_sw.click()
spin(0.8)
rb = readback()
check(c.roll_on and rb and near(rb['RL'], 0.3, 0.002) and near(rb['RR'], -0.3, 0.002) and rb['FL'] == 0,
      'roll on: Shift + E -> RL at the limit, RR reversed: %s' % rb)
check('roll' in c.state_l.text() and c.pad_r.xn == 'roll' and c.roll_row[0].isVisible(),
      'roll on: in the state line, on the right pad and as an axis row')
key(Qt.Key.Key_E, False)
key(Qt.Key.Key_Shift, False)
spin(0.8)
check(readback() == {'FL': 0, 'RL': 0, 'FR': 0, 'RR': 0}, 'roll: keys up -> zero on the mock')
c.set_roll(False)
spin(0.1)
check(not c.roll_sw.isChecked() and c.pad_r.xn == '' and not c.roll_row[0].isVisible(),
      'roll off again: label and row gone')

# a key held across a window deactivation is released
key(Qt.Key.Key_W, True)
spin(0.3)
app.sendEvent(win, QEvent(QEvent.Type.WindowDeactivate))
spin(0.6)
check(not c.held and c.cmd[0] == 0.0, 'keys: WindowDeactivate releases a held key')

# STOP from the window disarms and the mock coasts
key(Qt.Key.Key_Up, True)
spin(0.5)
win.stop()
key(Qt.Key.Key_Up, False)
spin(0.4)
check(not c.armed and not c.arm_sw.isChecked(), 'STOP: control disarmed')
check(readback() == {'FL': 0, 'RL': 0, 'FR': 0, 'RR': 0}, 'STOP: mock at zero')

# mutual exclusion with the Flight page's motors
c.arm_sw.click()
spin(0.1)
check(c.armed, 'exclusion: armed again')
win.motors.enable.click()
spin(0.1)
check(win.motors.enabled and not c.armed, 'exclusion: Motors enabled disarms the Control page')
c.arm_sw.click()
spin(0.1)
check(c.armed and not win.motors.enabled, 'exclusion: arming the Control page switches Motors enabled off')
c.disarm()
spin(0.2)

# a fake gamepad: A arms, the left stick drives heave (mode 2), unplugging while driving is a STOP
fake = [G.empty_state('xinput', 'no gamepad connected')]
station.pad_state = lambda: dict(fake[0], axes=dict(fake[0]['axes']), buttons=dict(fake[0]['buttons']))
pad_on = G.empty_state('xinput')
pad_on.update(connected=True, name='XInput controller 1')
fake[0] = pad_on
spin(0.2)
check('XInput controller 1' in c.source_l.text(), 'pad: shown as the source: %s' % c.source_l.text())
pressed = dict(pad_on, buttons=dict(pad_on['buttons'], a=True), axes=dict(pad_on['axes']))
fake[0] = pressed
spin(0.15)
fake[0] = pad_on
spin(0.1)
check(c.armed and c.source == 'keys' or c.armed, 'pad: A arms')
side = dict(pad_on, axes=dict(pad_on['axes'], rx=0.9), buttons=dict(pad_on['buttons']))
fake[0] = side
spin(0.6)
check(c.target[3] == 0.0 and readback() == {'FL': 0, 'RL': 0, 'FR': 0, 'RR': 0},
      'pad: the right stick\'s X is nothing while the roll is off')
c.set_roll(True)
spin(0.8)
rb = readback()
want = Cm.shape_axis(0.9) * 0.3
check(rb and near(rb['RL'], want, 0.003) and near(rb['RR'], -want, 0.003) and rb['FL'] == 0,
      'pad: roll on, right stick to the right -> RL +%.3f, RR -%.3f: %s' % (want, want, rb))
c.set_roll(False)
spin(0.8)
check(readback() == {'FL': 0, 'RL': 0, 'FR': 0, 'RR': 0}, 'pad: roll off ramps the rear pair back to zero')
up = dict(pad_on, axes=dict(pad_on['axes'], ly=0.9), buttons=dict(pad_on['buttons']))
fake[0] = up
spin(0.8)
rb = readback()
want = Cm.shape_axis(0.9) * 0.3
check(rb and near(rb['RL'], want, 0.003) and near(rb['RR'], want, 0.003) and rb['FL'] == 0,
      'pad: left stick up -> heave on the rear pair (%.3f): %s' % (want, rb))
check(c.source == 'pad', 'pad: source is the pad')
fake[0] = G.empty_state('xinput', 'no gamepad connected')
spin(0.5)
check(not c.armed, 'pad: unplugged while driving -> disarmed by STOP')
check(readback() == {'FL': 0, 'RL': 0, 'FR': 0, 'RR': 0}, 'pad: mock at zero after the STOP')
events = [e['text'] for e in win.dock.events]
check(any('went away while driving' in t for t in events), 'pad: crit event logged')

# D-pad steps the limit; Start explains itself
fake[0] = pad_on
spin(0.1)
before = c.limit_in.value()
fake[0] = dict(pad_on, buttons=dict(pad_on['buttons'], up=True), axes=dict(pad_on['axes']))
spin(0.15)
fake[0] = pad_on
spin(0.1)
check(c.limit_in.value() == before + 5, 'pad: D-pad up steps the limit to %d' % c.limit_in.value())
fake[0] = dict(pad_on, buttons=dict(pad_on['buttons'], start=True), axes=dict(pad_on['axes']))
spin(0.15)
fake[0] = pad_on
spin(0.1)
check(any('Start does nothing' in e['text'] for e in win.dock.events), 'pad: Start says it is not bound')

# Steam Deck mode on a 1280 x 800 screen
c.set_mode('deck')
win.resize(1280, 800)
spin(0.4)
check(c.mode == 'deck' and c.mode_b['deck'].isChecked() and set(c.kind_b) == {'manual', 'assisted', 'auto'},
      'deck: mode set, the three ways to fly on the page')
check('left stick' in c.legend_l.text(), 'deck: legend in the pad\'s names')
c.set_roll(True)
spin(0.1)
check('roll' in c.legend_l.text() and c.pad_r.xn == 'roll', 'deck: the roll in the legend and on the right pad')
c.set_layout('mode1')
spin(0.1)
check(c.pad_l.yn == 'surge' and c.pad_r.yn == 'heave', 'layout: mode 1 relabels the pads')
fake[0] = G.empty_state('xinput', 'no gamepad connected')
spin(0.3)
check(c.pad_notice.isVisible(), 'deck: the missing-pad notice shows')

# settings persisted
settings = QSettings(ini, QSettings.Format.IniFormat)
settings.sync()
check(settings.value('control/mode') == 'deck' and settings.value('control/layout') == 'mode1'
      and str(settings.value('control/roll')).lower() == 'true', 'settings: mode, layout and roll saved')

# ------------------------------------------------------- an assisted flight
check(win.minimumSizeHint().width() - win.header.pills['demo'].sizeHint().width() <= 1280,
      "deck: the window fits the Deck's 1280 px, the demo's pill aside: %d" % win.minimumSizeHint().width())
check(not win.header.sub.isVisible(), 'deck: the subtitle gives way')
c.set_mode('pc')
spin(0.1)
check(win.header.sub.isVisible(), 'pc: the subtitle is back')
c.set_mode('deck')
c.set_layout('mode2')
c.set_roll(False)
fake[0] = pad_on
spin(0.2)
ax = station.blimp.assist
mock = station.blimp.mock
ax.tuning = Ax.Tuning(climb_rate_mps=0.4, land_rate_mps=0.25, final_rate_mps=0.2)
ax.marker = os.path.join(tempfile.mkdtemp(prefix='assist-test-'), 'marker')


def press(name):
    fake[0] = dict(pad_on, buttons=dict(pad_on['buttons'], **{name: True}), axes=dict(pad_on['axes']))
    spin(0.12)
    fake[0] = pad_on
    spin(0.1)


def stick(**axes):
    fake[0] = dict(pad_on, axes=dict(pad_on['axes'], **axes), buttons=dict(pad_on['buttons']))


def state():
    return station.blimp.assist.snapshot()['state']


def wait_state(want, t):
    end = time.monotonic() + t
    while time.monotonic() < end and state() != want:
        spin(0.02)
    return state() == want


def said(text, level=None):
    return any(text in e['text'] and (level is None or e['level'] == level) for e in win.dock.events)


def on_floor():
    mock.height_m, mock.vz, mock.still_s = 0.07, 0.0, 60.0
    spin(0.15)


check(not c.assisted and c.kind_b['manual'].isChecked() and c.arm_sw.isVisible() and not c.flight_box.isVisible(),
      'assisted: manual by default, the page as it was')
check(not c.land_b.isEnabled() and c.land_note.isVisible(), 'manual: Land greyed out, its note says why')
press('y')
check(said('Land is part of an assisted flight', 'warn') and not c.armed, 'manual: Y only says why')
c.kind_b['assisted'].click()
spin(0.1)
check(c.assisted and c.flight_box.isVisible() and c.assist_box.isVisible() and not c.manual_box.isVisible()
      and not c.land_note.isVisible() and c.takeoff_b.isVisible(), 'assisted: its controls replace the manual ones')
check(c.pad_l.yn == 'height' and c.pad_l.xn == 'turn' and c.pad_r.yn == 'surge', 'assisted: the pads say what they move')
check('land' in c.legend_l.text() and 'take off' in c.legend_l.text(), 'assisted: the legend says what the buttons do')
check(near(c.height_in.value(), 1.0) and near(c.trim_in.value(), -0.6) and near(c.surge_in.value(), 3.0),
      'assisted: the defaults: 1 m, the trim of the new battery type, the 3 gf push')

# take off from the pad; the height box, the D-pad and the stick move the setpoint
on_floor()
stick(ly=0.5)
spin(0.2)
press('a')
check(not c.armed, 'take off: refused with a stick off centre')
stick()
spin(0.2)
press('down')
check(near(c.height_in.value(), 0.9), 'on the floor: the D-pad steps the height to fly to')
press('up')
press('a')
check(c.armed and state() != 'idle', 'take off: A starts it')
check(win.header.pills['drive'].value == 'PREPARING', 'drive pill: PREPARING')
check(wait_state('flying', 6.0), 'take off: flying')
spin(0.2)
check(c.state_l.text().startswith('ASSISTED') and win.header.pills['drive'].value == 'ASSIST', 'flying: the state line and the pill')
end = time.monotonic() + 6.0
while time.monotonic() < end and abs(mock.height_m - 1.0) > 0.08:
    spin(0.05)
check(abs(mock.height_m - 1.0) <= 0.08, 'flying: at the box\'s height: %.2f m' % mock.height_m)
check('\u2192' in c.height_r.value.text(), 'flying: the readout shows height and setpoint: %s' % ascii(c.height_r.value.text()))
win.resize(1280, 800)
spin(0.3)
c.height_r.set('2.30 \u2192 2.30')
c.heading_r.set('+179 \u2192 -179')
spin(0.02)
for r in (c.height_r, c.heading_r):
    fits = r.value.fontMetrics().horizontalAdvance(r.value.text()) <= r.value.width()
    check(fits, "deck: the widest %s fits, or the setpoint would elide: %d px" % (ascii(r.value.text()), r.value.width()))
c.height_in.setValue(1.3)
spin(0.3)
check(near(mock.hold['altitude'], 1.3, 0.001), 'flying: the box moves the setpoint')
press('up')
spin(0.3)
check(near(mock.hold['altitude'], 1.4, 0.001) and near(c.height_in.value(), 1.4, 0.011),
      'flying: the D-pad steps it, and the box follows: %.2f' % c.height_in.value())
press('right')
spin(0.3)
check(near(mock.hold['surge'], -0.5, 0.001) and near(c.trim_in.value(), -0.5), 'flying: the D-pad nudges the trim')
press('left')
stick(ry=1.0)
spin(0.9)
check(near(mock.hold['surge'], -0.6 + 3.0, 0.11), 'flying: the right stick pushes: %.2f gf' % mock.hold['surge'])
stick()
spin(0.9)
check(near(mock.hold['surge'], -0.6, 0.06), 'flying: let go, the trim: %.2f gf' % mock.hold['surge'])
stick(lx=-1.0)
spin(1.0)
check(mock.hold['heading'] > 3.0, 'flying: the left stick left turns counter-clockwise: %.1f' % mock.hold['heading'])
stick()
spin(0.3)

# while it flies: the Flight page, the console, the Wi-Fi and the kind of flight stay out
win.motors.enable.click()
spin(0.1)
check(not win.motors.enabled and c.armed and state() == 'flying', 'flying: Motors enabled is refused and the flight goes on')
c.kind_b['manual'].click()
spin(0.1)
check(c.assisted and c.kind_b['assisted'].isChecked(), 'flying: no change to manual in the air')
con = []
station.con_text.connect(con.append)
station.console('tof -n 3')
spin(0.1)
check(any('only reads go through' in t for t in con), 'flying: a sensor command is refused in the console')
station.console('hold')
spin(0.3)
check(any('mode HOLD' in t for t in con), 'flying: reading the hold goes through')
station.console('param alt.kp 2')
spin(0.1)
check(mock.params['alt.kp'] != 2.0, 'flying: a parameter change is refused')
station.wifi_action('disconnect')
spin(0.1)
check(said('not while a motor command stands'), 'flying: the Wi-Fi button is locked')

# Y lands, and the page comes back by itself
press('x')
check(wait_state('landing', 2.0), 'land: X starts the landing, as Y does')
spin(0.2)
check(c.state_l.text().startswith('LANDING') and win.header.pills['drive'].value == 'LANDING'
      and not c.land_b.isEnabled(), 'land: the state line, the pill, the button')
check(wait_state('idle', 25.0), 'land: down')
spin(0.3)
check(said('landed', 'ok') and not c.armed and mock.hold_mode == 'MANUAL', 'land: disarmed, hold off')
check('ready' in c.state_l.text() and win.header.pills['drive'].value == 'OFF', 'land: ready: %s' % c.state_l.text())
check(c.height_in.value() > 1.0, 'land: the box keeps the height flown, not the landing: %.2f' % c.height_in.value())

# B coasts in the air, from the pad's own path and the window's
on_floor()
press('a')
check(wait_state('flying', 6.0), 'stop: flying again')
spin(1.0)
press('b')
check(wait_state('idle', 3.0), 'stop: B ends the flight')
spin(0.2)
check(not c.armed and mock.hold_mode == 'MANUAL' and not any(mock.motor), 'stop: disarmed, the motors coast')

# a pad that goes away lands, where in manual it stops
on_floor()
press('a')
check(wait_state('flying', 6.0), 'pad gone: flying')
stick(ly=0.3)
spin(0.3)
fake[0] = G.empty_state('xinput', 'no gamepad connected')
check(wait_state('landing', 2.0) and said('went away in the flight', 'crit'), 'pad gone: it lands')
check(wait_state('idle', 25.0), 'pad gone: down')
spin(0.2)
check(not c.armed and said('landed', 'ok'), 'pad gone: landed')

# the settings, and back to manual
fake[0] = pad_on
spin(0.2)
c.surge_in.setValue(2.0)
c.trim_in.setValue(-1.0)
settings = QSettings(ini, QSettings.Format.IniFormat)
settings.sync()
check(str(settings.value('control/assisted')).lower() == 'true' and float(settings.value('control/trim')) == -1.0
      and float(settings.value('control/surge')) == 2.0 and settings.value('control/height') is not None,
      'settings: the kind of flight, the trim, the surge and the height saved')
c.kind_b['manual'].click()
spin(0.1)
check(not c.assisted and c.manual_box.isVisible() and c.pad_l.yn == 'heave', 'manual again: the page as it was')

# ------------------------------------------------------- the course (test_course.py has the flight)
c.kind_b['auto'].click()
spin(0.2)
check(c.kind == 'auto' and c.assisted and c.course_panel.isVisible() and not c.sticks.isVisible(),
      'autonomous: the Course panel takes the sticks\' place')
check(c.takeoff_b.isVisible() and c.takeoff_b.text() == 'Fly the course' and c.takeoff_b.isEnabled()
      and c.box_w['trim'].isVisible() and not c.box_w['height'].isVisible() and not c.box_w['surge'].isVisible(),
      'autonomous: Fly the course, Land and the trim; no height or surge box')
check('fly the course' in c.legend_l.text() and 'land' in c.legend_l.text(), 'autonomous: the legend')
check(win.minimumSizeHint().width() - win.header.pills['demo'].sizeHint().width() <= 1280,
      "autonomous: the page still fits the Deck's 1280 px: %d" % win.minimumSizeHint().width())
check(c.course_panel.check_l.text().startswith('2 balloons'), 'autonomous: the sheet checks: %s'
      % c.course_panel.check_l.text())
settings = QSettings(ini, QSettings.Format.IniFormat)
settings.sync()
check(settings.value('control/fly') == 'auto' and str(settings.value('control/assisted')).lower() == 'false',
      'autonomous: remembered as the way to fly')
c.kind_b['manual'].click()
spin(0.1)
check(c.kind == 'manual' and c.sticks.isVisible() and not c.course_panel.isVisible(), 'manual again: the sticks are back')

win.close()
spin(0.2)
A.teardown(station, win)
raise SystemExit(H.finish())
