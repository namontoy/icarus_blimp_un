"""The Control page offscreen: the pure parts (mix, shaping, keys, the evdev
parser), the platform's pad reader, then the demo window: keys, arming, a fake
gamepad, mutual exclusion with the Flight page's motors, STOP, the Steam Deck
mode and the saved settings."""
import time

import harness as H
from harness import check, near

from PyQt6.QtCore import QEvent, QSettings, Qt
from PyQt6.QtGui import QKeyEvent
from PyQt6.QtWidgets import QApplication

from software.ground_station import app as A
from software.ground_station import control as Cm
from software.ground_station import gamepad as G

# ------------------------------------------------------------ pure parts
m = Cm.teleop_mix(1, 0, 0, 0.3)
check(all(near(x, y) for x, y in zip(m, [0.3, 0, 0.3, 0])), 'mix: surge only -> front pair at the limit')
m = Cm.teleop_mix(1, 1, 0, 1.0)
check(all(near(x, y) for x, y in zip(m, [1, 0, 0, 0])), 'mix: full surge + full right yaw -> FL 1, FR 0 (scaled)')
m = Cm.teleop_mix(0, 0.5, -0.4, 1.0)
check(all(near(x, y) for x, y in zip(m, [0.5, -0.4, -0.5, -0.4])), 'mix: yaw right alone -> FL +, FR -; heave down reverses both rear')
check(Cm.shape_axis(0.05) == 0.0 and near(Cm.shape_axis(1.0), 1.0) and near(Cm.shape_axis(-1.0), -1.0),
      'shape: dead band and full travel')
xs = [Cm.shape_axis(i / 50.0) for i in range(51)]
check(all(b >= a for a, b in zip(xs, xs[1:])), 'shape: monotonic')
check(Cm.key_axes({Qt.Key.Key_W}, False) == (0.5, 0.0, 0.0), 'keys: W alone is half surge')
check(Cm.key_axes({Qt.Key.Key_W, Qt.Key.Key_S}, True) == (0.0, 0.0, 0.0), 'keys: W and S cancel')
check(Cm.key_axes({Qt.Key.Key_Left, Qt.Key.Key_Up}, True) == (0.0, -1.0, 1.0), 'keys: Left + Up with Shift')
check(Cm.slew(0.0, 1.0, 0.125) == 0.125 and Cm.slew(0.1, 0.0, 0.125) == 0.0, 'slew: step and arrival')
check(Cm.pad_axes({'lx': 0.1, 'ly': 0.2, 'rx': 0.3, 'ry': 0.4, 'lt': 0, 'rt': 0}, 'mode2') == (0.4, 0.1, 0.2),
      'pad axes mode 2: right Y surge, left X yaw, left Y heave')
check(Cm.pad_axes({'lx': 0.1, 'ly': 0.2, 'rx': 0.3, 'ry': 0.4, 'lt': 0, 'rt': 0}, 'mode1') == (0.2, 0.1, 0.4),
      'pad axes mode 1: left Y surge, left X yaw, right Y heave')

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
check(any('not in the firmware yet' in e['text'] for e in win.dock.events), 'pad: Start logs the hand-off note')

# Steam Deck mode on a 1280 x 800 screen
c.set_mode('deck')
win.resize(1280, 800)
spin(0.4)
check(c.mode == 'deck' and c.mode_b['deck'].isChecked() and not c.hint.isVisible(), 'deck: mode set, hint hidden')
check('left stick' in c.legend_l.text(), 'deck: legend in the pad\'s names')
c.set_layout('mode1')
spin(0.1)
check(c.pad_l.yn == 'surge' and c.pad_r.yn == 'heave', 'layout: mode 1 relabels the pads')
fake[0] = G.empty_state('xinput', 'no gamepad connected')
spin(0.3)
check(c.pad_notice.isVisible(), 'deck: the missing-pad notice shows')

# settings persisted
settings = QSettings(ini, QSettings.Format.IniFormat)
settings.sync()
check(settings.value('control/mode') == 'deck' and settings.value('control/layout') == 'mode1',
      'settings: mode and layout saved')

win.close()
spin(0.2)
A.teardown(station, win)
raise SystemExit(H.finish())
