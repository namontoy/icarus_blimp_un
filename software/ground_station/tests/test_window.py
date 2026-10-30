"""The whole demo window with its GUI thread deliberately blocked, the pad's B
from the reader thread, the Drive pill, a Wi-Fi drop seen from the window,
the Wi-Fi button's lock, a page switch while the pad drives, a capture that
records the gains, and race mode (docs/motor-setpoint-ownership.md,
docs/link-robustness-review.md)."""
import json
import os
import tempfile
import threading
import time

import harness as H
from harness import check, near, pump

from PyQt6.QtCore import QSettings, Qt
from PyQt6.QtWidgets import QApplication

from software.ground_station import app as A
from software.ground_station import control as Cm
from software.ground_station import gamepad as G
from software.ground_station import station as S

# ------------------------------------------------------- the gamepad's press edge


class FakePad:
    name, error = 'fake', ''

    def __init__(self):
        self.n = 0

    def poll(self):
        self.n += 1
        s = G.empty_state('fake')
        s['connected'], s['name'] = True, 'fake'
        s['buttons']['b'] = 3 <= self.n <= 8          # one press, held across several polls
        return s


presses = []
pad = G.Gamepad(backend=FakePad(), on_press=presses.append).start()
time.sleep(0.5)
pad.stop()
check(presses == ['b'], 'gamepad: one press edge reported from the reader thread (%s)' % presses)

# ------------------------------------------------------------------- the window
app = QApplication([])


def open_window(argv):
    settings = QSettings(H.settings_path(), QSettings.Format.IniFormat)
    station, win = A.build(A.parse_args(argv), app, settings, ports=H.free_ports())
    win.resize(1680, 1000)
    win.show()
    evs = []
    station.ev.connect(evs.append)
    station.start()
    return station, win, evs


def said(evs, text, level=None):
    return any(text in e['text'] and (level is None or e['level'] == level) for e in evs)


station, win, wev = open_window(['--demo', '--mode', 'pc'])
pump(app, 2.0)
check(station.alive, 'window: demo telemetry alive')
mock = station.blimp.mock
drive = win.header.pills['drive']
check(drive.value == 'OFF', 'drive pill: OFF at start')
win.motors.set_enabled(True)
win.motors.sliders['FL'].setValue(50)          # 50 % of the 30 % limit
pump(app, 0.5)
check(near(mock.motor[0], 0.15), 'window: FL 0.15 reaches the mock')
check(drive.value == 'FLIGHT', 'drive pill: FLIGHT while Motors enabled is on')
station.wifi_action('connect')
check(said(wev, 'not while a motor command stands'), 'wifi: join and leave refused while the motors run')
time.sleep(1.2)                                # the GUI thread blocked, as by a repaint from hell
pump(app, 0.3)
check(near(mock.motor[0], 0.15) and win.motors.enabled,
      'window: a 1.2 s GUI stall leaves the motors running (the old chain cut them at 0.5 s)')
check(not said(wev, 'released'), 'window: no release event')

# a Wi-Fi drop seen from the window: the motors stay enabled, the pill says what happens
mock.radio_on = False
pump(app, 1.0)
check(drive.value == 'LINK LOST' and win.motors.enabled, 'drive pill: LINK LOST, Motors enabled stays on')
mock.radio_on = True
pump(app, 0.4)
check(drive.value == 'RESUMING', 'drive pill: RESUMING during the ramp')
pump(app, 1.2)
check(drive.value == 'FLIGHT' and near(mock.motor[0], 0.15), 'drive pill: FLIGHT again, FL back to 0.15')

time.sleep(3.6)                                # frozen past the watchdog
pump(app, 0.5)
check(mock.motor[0] == 0.0, 'window: motors released after the 3 s watchdog')
check(not win.motors.enabled, 'window: the window STOPped itself, Motors enabled is off')
check(not station.blimp.snapshot()['tripped'], 'window: its STOP cleared the latch')
check(said(wev, 'stopped responding', 'crit'), 'window: crit event logged')
check(said(wev, 'STOP sent'), 'window: STOP sent event logged')

# the Control page
win.show_page('control')
pump(app, 0.3)
check(win.control.arm('keys'), 'control: armed from the keyboard')
pump(app, 0.2)
check(drive.value == 'CONTROL', 'drive pill: CONTROL while armed')
win.control.key(Qt.Key.Key_W, True)
pump(app, 0.7)
check(mock.motor[0] > 0.1 and mock.motor[2] > 0.1, 'control: W drives the front pair (FL %.3f FR %.3f)'
      % (mock.motor[0], mock.motor[2]))
time.sleep(1.2)
pump(app, 0.3)
check(mock.motor[0] > 0.1 and win.control.armed, 'control: a 1.2 s stall keeps the command standing')
win.control.key(Qt.Key.Key_W, False)
pump(app, 0.8)
check(mock.motor[0] == 0.0 and win.control.armed, 'control: key up ramps to zero, still armed')
win.stop()
pump(app, 0.3)
check(not win.control.armed and not station.blimp.snapshot()['standing'], 'control: STOP disarms and drops the command')
check(drive.value == 'OFF', 'drive pill: OFF after STOP')

# a page switch while the pad drives: it keeps driving (the team's choice, 2026-10-29)
fake = [G.empty_state('xinput', 'no gamepad connected')]
real_pad_state = station.pad_state
station.pad_state = lambda: dict(fake[0], axes=dict(fake[0]['axes']), buttons=dict(fake[0]['buttons']))
pad_on = G.empty_state('xinput')
pad_on.update(connected=True, name='XInput controller 1')
fake[0] = pad_on
pump(app, 0.2)
fake[0] = dict(pad_on, buttons=dict(pad_on['buttons'], a=True), axes=dict(pad_on['axes']))
pump(app, 0.15)
fake[0] = pad_on
pump(app, 0.1)
check(win.control.armed, 'page switch: A on the pad arms')
fake[0] = dict(pad_on, axes=dict(pad_on['axes'], ly=0.9), buttons=dict(pad_on['buttons']))
pump(app, 0.8)
want = Cm.shape_axis(0.9) * 0.3
check(near(mock.motor[1], want, 0.003), 'page switch: the left stick drives heave, RL %.3f' % mock.motor[1])
win.show_page('bench')
pump(app, 0.8)
check(win.control.armed and near(mock.motor[1], want, 0.003),
      'page switch: on the Bench page the pad still drives, RL %.3f' % mock.motor[1])
check(drive.value == 'CONTROL', 'page switch: the Drive pill says so on every page')
fake[0] = pad_on
pump(app, 0.6)

# B on the pad goes to the bridge from the reader's thread, GUI thread or not, and latches
win.show_page('flight')
pump(app, 0.2)
check(win.control.armed, 'pad B: still armed before the press')
fake[0] = dict(pad_on, axes=dict(pad_on['axes'], ly=0.9), buttons=dict(pad_on['buttons']))
pump(app, 0.8)
check(mock.motor[1] > 0.05, 'pad B: the pad drives again (RL %.3f)' % mock.motor[1])
threading.Timer(0.3, lambda: station._pad_press('b')).start()
time.sleep(1.0)                                # the GUI thread blocked while B is pressed
check(mock.motor[1] == 0.0, 'pad B: STOP reached the blimp while the GUI thread was blocked')
check(station.blimp.snapshot()['tripped'], 'pad B: the bridge holds the latch until the window catches up')
pump(app, 0.4)                                 # the window thaws with the stick still up
check(not win.control.armed and mock.motor[1] == 0.0,
      'pad B: the thawed window disarmed instead of driving again')
check(not station.blimp.snapshot()['tripped'], "pad B: the window's STOP cleared the latch")
station.pad_state = real_pad_state

# a capture's .json names the gains it ran with, both sets when a 'param' change came mid-capture
S.CAPTURES_DIR = tempfile.mkdtemp(prefix='ground-station-captures-')
caps = []
station.cap.connect(caps.append)
station.capture('start', 2)
pump(app, 0.6)
station.console('param alt.kp 0.5')
pump(app, 0.6)
station.capture('download', notes='params test')
end = time.monotonic() + 10.0
while time.monotonic() < end and not any(c.get('phase') == 'saved' for c in caps):
    pump(app, 0.1)
saved = [c for c in caps if c.get('phase') == 'saved']
check(bool(saved), 'capture: downloaded and saved')
if saved:
    with open(os.path.splitext(saved[0]['file'])[0] + '.json') as f:
        pr = json.load(f).get('params') or {}
    check(pr.get('values', {}).get('alt.kp') == 0.5 and pr.get('changed') == ['alt.kp'],
          'capture: the .json names the gains at the download (%s)' % pr.get('values', {}).get('alt.kp'))
    check(pr.get('at_start', {}).get('values', {}).get('alt.kp') == 1.336,
          'capture: and the set at the start, since a change came between')
check(said(wev, 'parameters changed during it', 'warn'), 'capture: the window says the gains changed mid-capture')
A.teardown(station, win)

# ------------------------------------------------------------------ race mode
station, win, wev = open_window(['--demo', '--race'])
con = []
station.con_text.connect(con.append)
pump(app, 1.5)
mock = station.blimp.mock
check(station.alive, 'race: demo telemetry alive')
check('race' in win.windowTitle(), 'race: the title says so: %s' % win.windowTitle())
check(win.header.pills['drive'].value == 'RACE', 'race: the Drive pill reads RACE')
check(not win.motors.enable.isEnabled() and not win.control.arm_sw.isEnabled(),
      'race: Motors enabled and Manual control are disabled')
win.motors.set_enabled(True)
check(not win.motors.enabled, 'race: Motors enabled refused')
check(not win.control.arm('keys'), 'race: arming refused')
station.console('motor -m 0 -t 0.2')
pump(app, 0.3)
check(any('race mode: motor commands are refused' in t for t in con), 'race: the console refuses motor')
station.console('help')
pump(app, 1.0)
check(any('Print the list of registered commands' in t for t in con), 'race: other console commands run')
station.console('param alt.kp 0.9')
pump(app, 0.3)
check(any('parameter changes are refused' in t for t in con) and mock.params['alt.kp'] == 1.336,
      'race: the console refuses a parameter change')
station.console('param alt.')
pump(app, 1.0)
check(any('changes since boot' in t for t in con), 'race: reading the parameters still works')
check(mock.motor == [0.0] * 4, 'race: nothing reached the motors')
win.stop()
pump(app, 0.3)
check(said(wev, 'STOP sent'), 'race: STOP still works')
A.teardown(station, win)

raise SystemExit(H.finish())
