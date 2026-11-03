"""The ground station's window: header, the Flight, Control and Bench pages, the dock, and main().

    python -m software.ground_station --demo                  # no hardware
    python -m software.ground_station                         # the blimp at 192.168.4.1

One process: the Station (station.py) runs the blimp link, the camera and the
Wi-Fi on their own threads and signals the window on the GUI thread.

Safety on this side: the bridge's thread owns the standing motor command and
repeats it to the blimp while "Motors enabled" is on or the Control page is
armed; the window only sends changes, STOP (the button, Esc, or Space when
no text field has focus; it also switches the motors off) and, from the
station's tick, a 100 ms ping that proves this thread alive. Closing the
window stops the motors first. Behind that, the bridge's watchdog releases
the motors if the window stops responding for 3 s, a Wi-Fi drop zeroes them
and resumes them ramped if the link returns within 3 s, a blimp reboot
releases them, and B on the pad stops them from the pad's own thread; each
of those latches the bridge, and the window STOPs itself when it sees the
latch (on_tel). --race makes the window a monitor: no motor command but
STOP. The header's Drive pill says which of these holds.
docs/motor-setpoint-ownership.md and docs/link-robustness-review.md have the
history.
"""
import argparse
import collections
import math
import signal
import sys
import time

from PyQt6 import sip
from PyQt6.QtCore import QEvent, QObject, QRect, QSettings, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QPainter
from PyQt6.QtWidgets import (QAbstractSpinBox, QApplication, QComboBox, QLineEdit, QMainWindow,
                             QStackedWidget, QVBoxLayout, QWidget)

from . import theme as T
from .bench import BenchPage
from .control import MODES, ControlPage
from .panels import CameraPanel, Dock, MotorsPanel, SensorsPanel, button, load_dock, save_dock
from .station import Station
from .widgets import (C, DockSplitter, DockTab, FullScreenButton, InfoIcon, Pill, StopButton, Wordmark, clamp, hbox,
                      hline_polygon, label)

TEL_STALE_S = 1.5
FRAME_MS = 33               # plots, staleness and camera ageing, about 30 Hz
DOCK_MIN_PX = 110
PANELS_MIN_PX = 320         # the dock never squeezes the window's top part below this
COMPACT_BELOW_PX = 860      # a laptop screen: smaller gauges, no motor hint
LINK_WINDOW_S = 1.0         # the Link pill: the longest wait over this long, shown this often
LINK_SLOW_MS = 250          # a wait past this shows at once
# The dock's height is kept under this key. It was 'dock/h' until 2026-11-02,
# when the default went down from 230 px to 160 px on 1080p: a new key lets
# every station start from the new default once, instead of the height it
# had kept from the old one.
DOCK_H_KEY = 'dock/height'


def dock_default_px(h):
    """First-run height: about a sixth of the window, 160 px on a 1080p screen:
    the console's last lines, its input and the command chips. It was a
    quarter (230 px) until 2026-11-02, which hid the bottom of the sensors
    panel on a 1080p screen."""
    return clamp(round(h * 0.16), 130, 170)


class Header(QWidget):
    """Wordmark, the Wi-Fi join, the status pills, full screen and STOP, over
    the papaya stripe and a hairline."""

    def __init__(self, demo):
        super().__init__()
        self.demo = demo
        sub = label('Universidad Nacional de Colombia <span style="color:%s">·</span> blimp ground station'
                    % T.PAPAYA, 12, color=T.MUTED, spacing=0.12, caps=True)
        brand = QVBoxLayout()
        brand.setSpacing(4)
        brand.setContentsMargins(0, 0, 0, 0)
        brand.addWidget(Wordmark())
        brand.addWidget(sub)
        self.wifi_b = button('Join blimp Wi-Fi', tip="join the blimp's access point from this computer")
        self.wifi_b.setEnabled(False)
        # What the join says (the network, this computer's address, an error)
        # sits behind an i, amber when it is an error; until 2026-11-02 it was
        # a line of text beside the button, elided to "on the b…" on 1080p
        self.wifi_note = InfoIcon('—')
        # The pills whose value is a number keep the width of their widest
        # usual value (Pill.reserve), so changing digits no longer move the
        # row; the others change width only when their state changes, which is
        # news. Reserving those too would have widened the header by 124 px,
        # and it is already wider than the Steam Deck's 1280
        self.pills = {
            'drive': Pill('Drive', 'what drives the motors: the Flight page, the Control page or nothing; '
                                   'RACE: this window only watches, STOP is its one command'),
            'wifi': Pill('Wi-Fi', "this computer's connection to the blimp's access point"),
            'link': Pill('Link', 'UDP link to the blimp: the longest wait for a packet over the last second, '
                                 'shown once a second (telemetry comes 20 times a second, so about 50 ms when all '
                                 'is well); a wait past %d ms shows at once' % LINK_SLOW_MS, reserve='999 ms'),
            'fs': Pill('Failsafe', 'motor driver failsafe on the blimp'),
            'sta': Pill('STA', "stations on the blimp's access point", reserve='99'),
            'cam': Pill('Cam', "the forward camera's rows, relayed by the S3"),
            'demo': Pill('Demo data', 'nothing here touches a real blimp', 'demo', ''),
        }
        self.pills['demo'].setVisible(demo)
        self._ages = collections.deque()    # (t, ms) of the last second's telemetry ages, for the Link pill
        self._link_shown = (None, 0.0)      # (state, when) of what the pill last showed
        self.full_b = FullScreenButton()
        self.full_b.setToolTip('full screen (F11)')
        self.full_b.setFocusPolicy(Qt.FocusPolicy.TabFocus)
        self.stop_b = StopButton()
        self.stop_b.setToolTip('Coast all four motors (Space or Esc)')
        lay = hbox(brand, 12, self.wifi_b, 6, self.wifi_note, None, spacing=0, margins=(4, 14, 4, 10))
        for p in self.pills.values():
            lay.addWidget(p)
            lay.addSpacing(8)
        lay.addSpacing(10)
        lay.addWidget(self.full_b)
        lay.addSpacing(18)
        lay.addWidget(self.stop_b)
        self.setLayout(lay)

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(C(T.PAPAYA))         # the one McLaren line in the window
        p.drawPolygon(hline_polygon((0, 0), (260, 0), (254, 3), (0, 3)))
        p.fillRect(QRect(0, self.height() - 1, self.width(), 1), C(T.BORDER))

    def on_wifi(self, w):
        """Joined or not is judged by the station from this computer's own
        address, so the pill is honest even when the network was joined by hand."""
        b, note, pill = self.wifi_b, self.wifi_note, self.pills['wifi']
        if self.demo:
            b.setEnabled(False)
            T.restyle(b, state='')
            note.setText('Demo: no radio involved.')
            note.set_tone('')
            pill.set('idle', 'DEMO')
            return
        if not w or w.get('backend') == 'none':
            b.setEnabled(False)
            T.restyle(b, state='')
            note.setText((w or {}).get('error') or 'No Wi-Fi control on this system: join ICARUS-… by hand.')
            note.set_tone('warn' if (w or {}).get('error') else '')
            pill.set('ok' if w and w.get('joined') else 'idle', w['ip'] if w and w.get('joined') else '—')
            return
        busy = bool(w.get('busy'))
        b.setEnabled(not busy)
        if busy:
            b.setText('Leaving…' if w.get('phase') == 'leaving' else 'Joining…')
            T.restyle(b, state='busy')
        elif w.get('joined'):
            b.setText('Leave blimp Wi-Fi')
            T.restyle(b, state='up')
        else:
            b.setText('Join blimp Wi-Fi')
            T.restyle(b, state='')
        if w.get('joined'):
            note.setText('%s · this computer is %s' % (w.get('ssid') or 'on the blimp network', w.get('ip')))
        elif w.get('error'):
            note.setText(w['error'])
        else:
            c = w.get('candidates') or []
            note.setText('known: ' + ', '.join(c) if c else 'no ICARUS-… network known yet')
        note.set_tone('warn' if w.get('error') and not w.get('joined') else '')
        pill.set('live' if busy else 'ok' if w.get('joined') else 'warn',
                 (w.get('phase') or '').upper() if busy else w.get('ip') if w.get('joined') else 'DOWN')

    def on_tel(self, m):
        tel = m.get('tel')
        if m.get('alive') and tel:
            self._link(m.get('age_s'), 'ok' if tel.get('link_alive') else 'warn')
            fs = tel.get('failsafe')
            self.pills['fs'].set('crit' if fs == 'TRIPPED' else 'live' if fs == 'armed' else 'ok', fs)
            self.pills['sta'].set('ok' if tel.get('stations', 0) > 0 else 'warn', str(tel.get('stations')))
        else:
            self._ages.clear()
            self.pills['link'].set('crit', 'NO TELEMETRY')
            self.pills['fs'].set('idle', '—')
            self.pills['sta'].set('idle', '—')

    def _link(self, age_s, state):
        """The Link pill. The age of the last packet at a 10 Hz tick is a
        sawtooth between 0 and 50 ms (telemetry comes at 20 Hz, the tick
        samples it at any phase), and shown every tick it flickered through
        digits that meant nothing. It shows the longest wait in the last
        second instead, once a second, rounded up to 10 ms; a change of state
        (STALE, link_alive) and a wait past LINK_SLOW_MS show at once and
        follow every tick while they last."""
        now = time.monotonic()
        if age_s is not None:
            self._ages.append((now, age_s * 1000))
        while self._ages and now - self._ages[0][0] > LINK_WINDOW_S:
            self._ages.popleft()
        worst = max((a for _, a in self._ages), default=None)
        p = self.pills['link']
        if p.state == state and now - self._link_shown[1] < LINK_WINDOW_S and \
                not (worst is not None and worst > LINK_SLOW_MS):
            return
        self._link_shown = (state, now)
        p.set(state, '%d ms' % (math.ceil(worst / 10) * 10) if worst is not None else '—')

    def set_drive(self, race, m, flight, control):
        """The Drive pill: the window's own view of who drives (flight,
        control) over the bridge's (m: link lost, resuming)."""
        if race:
            self.pills['drive'].set('demo', 'RACE')
        elif m.get('standing') and m.get('link_lost'):
            self.pills['drive'].set('crit', 'LINK LOST')
        elif m.get('standing') and m.get('ramp', 1.0) < 1.0:
            self.pills['drive'].set('warn', 'RESUMING')
        elif control:
            self.pills['drive'].set('live', 'CONTROL')
        elif flight:
            self.pills['drive'].set('live', 'FLIGHT')
        else:
            self.pills['drive'].set('idle', 'OFF')

    def on_cam_status(self, st):
        # Rows reach the window through the S3's relay (CAMERA packets): NONE until the first one, UP
        # while rows are fresh, NO ROWS when the relay is alive but the camera is not delivering, DOWN
        # when the relay's packets stopped too. LIVE is the demo's painted frames.
        if not st.get('source') or st['source'] == 'none' or not st.get('packets'):
            self.pills['cam'].set('idle', 'NONE')
        elif not st.get('connected'):
            self.pills['cam'].set('warn', 'DOWN')
        elif st.get('live'):
            self.pills['cam'].set('live', 'LIVE')
        elif st.get('rows_fresh'):
            self.pills['cam'].set('ok', 'UP')
        else:
            self.pills['cam'].set('warn', 'NO ROWS')


class PageBar(QWidget):
    """Flight | Control | Bench under the header: the live view, flying by
    hand (control.py), or the bench page of captures and checks (bench.py).
    The header and the dock serve all three, so STOP and the console are
    always at hand."""
    changed = pyqtSignal(str)
    # Each page's one-line description is its tab's tip (it was a line of
    # faint text on the right of the bar until 2026-11-02)
    NOTES = {'flight': 'motors, camera and sensors, live',
             'control': 'fly by hand from the keyboard or a gamepad, then hand off',
             'bench': 'captures, calibration and checks of the blimp on the bench'}

    def __init__(self):
        super().__init__()
        self.tabs = {'flight': DockTab('Flight'), 'control': DockTab('Control'), 'bench': DockTab('Bench')}
        for name, t in self.tabs.items():
            t.clicked.connect(lambda _=False, name=name: self.changed.emit(name))
            t.setToolTip(self.NOTES[name])
        self.setLayout(hbox(self.tabs['flight'], 2, self.tabs['control'], 2, self.tabs['bench'], None, spacing=0,
                            margins=(0, 4, 0, 0)))

    def set(self, name):
        for n, t in self.tabs.items():
            t.setChecked(n == name)

    def paintEvent(self, e):
        p = QPainter(self)
        p.fillRect(QRect(0, self.height() - 1, self.width(), 1), C(T.BORDER))


class MainWindow(QMainWindow):
    def __init__(self, station, settings):
        super().__init__()
        self.station, self.settings = station, settings
        self.setWindowTitle('Icarus Ground Station' + (' · race' if station.race else ''))
        self.tel_at, self.alive = 0.0, False
        self.wifi = None
        self._compact = None
        self._dock_h = None
        self._was_maximized = False

        self.header = Header(station.demo)
        self.motors = MotorsPanel(station)
        self.camera = CameraPanel(station)
        self.sensors = SensorsPanel(station.demo)
        self.motors.setFixedWidth(340)
        self.sensors.setFixedWidth(400)
        grid = QWidget()
        grid.setLayout(hbox(self.motors, self.camera, self.sensors, spacing=12, margins=(0, 12, 0, 0)))
        grid.layout().setStretchFactor(self.camera, 1)
        grid.setMinimumHeight(200)
        self.control = ControlPage(station, settings)
        self.bench = BenchPage(station)
        self.pages = QStackedWidget()
        self.page_index = {}
        for name, page in (('flight', grid), ('control', self.control), ('bench', self.bench)):
            self.page_index[name] = self.pages.addWidget(page)
        self.pagebar = PageBar()
        self.pagebar.changed.connect(self.show_page)
        self.dock = Dock(station)
        self.split = DockSplitter()
        self.split.addWidget(self.pages)
        self.split.addWidget(self.dock)
        self.split.setStretchFactor(0, 1)
        self.split.setStretchFactor(1, 0)
        self.split.splitterMoved.connect(self._dock_moved)
        self.split.handle_released.connect(self._save_dock_h)
        self.split.handle_reset.connect(self._reset_dock_h)
        self.dock.collapsed_changed.connect(self._collapsed)

        root = QWidget()
        # Clicking empty space takes the focus out of the console's input, so
        # Space works as STOP again without hunting for a neutral spot.
        root.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        rl = QVBoxLayout(root)
        rl.setContentsMargins(16, 0, 16, 10)
        rl.setSpacing(0)
        rl.addWidget(self.header)
        rl.addWidget(self.pagebar)
        rl.addWidget(self.split, 1)
        self.setCentralWidget(root)

        h = self.header
        h.stop_b.clicked.connect(self.stop)
        h.full_b.clicked.connect(self.toggle_full)
        h.wifi_b.clicked.connect(lambda: station.wifi_action(
            'disconnect' if self.wifi and self.wifi.get('joined') else 'connect'))

        station.tel.connect(self.on_tel)
        station.sen.connect(self.sensors.on_sen)
        station.cam_frame.connect(self.camera.on_frame)
        station.cam_row.connect(self.camera.on_row)
        station.cam_status.connect(self.camera.on_status)
        station.cam_status.connect(h.on_cam_status)
        station.con_text.connect(self.dock.write)
        station.con_end.connect(self.dock.con_end)
        station.ev.connect(self.dock.add_event)
        station.wifi.connect(self.on_wifi)
        station.cap.connect(self.bench.capture.on_cap)
        station.cap_data.connect(self.bench.on_data)
        # One source drives the motors at a time: arming the Control page
        # switches the Flight page off and the other way round, else two
        # standing commands would alternate on the wire.
        self.control.armed_changed.connect(lambda on: on and self.motors.enabled and self.motors.set_enabled(False))
        self.motors.enable.toggled.connect(lambda on: on and self.control.disarm(send_stop=False))
        self.control.stop_requested.connect(self.stop)

        self._frame = QTimer(self)
        self._frame.timeout.connect(self.frame)
        self._frame.start(FRAME_MS)

        geo = settings.value('window/geometry')
        if geo is None or not self.restoreGeometry(geo):
            scr = QApplication.primaryScreen().availableGeometry()
            self.resize(min(1680, scr.width()), min(1000, scr.height()))
        load_dock(settings, self.dock)
        page = settings.value('window/page')
        self.show_page(page if page in self.page_index else 'flight')
        # value(..., type=int) turns a missing key into 0, not None, and 0
        # would pin the dock at its minimum on a first run.
        h = settings.value(DOCK_H_KEY)
        self._dock_h = int(h) if h not in (None, '') else None

    # ------------------------------------------------------------ station in

    def on_tel(self, m):
        if m.get('tripped'):
            # The bridge latched: its watchdog, a long outage or a reboot
            # released the motors, or B on the pad stopped them from the pad's
            # thread. Make it the operator's STOP: the controls go off, the
            # latch clears (blimp_bridge.py), the button flashes.
            self.stop()
        was_alive = self.alive
        self.tel_at, self.alive = time.monotonic(), bool(m.get('alive'))
        self.header.on_tel(m)
        self.header.set_drive(self.station.race, m, self.motors.enabled, self.control.armed)
        self.motors.on_tel(m)
        self.control.on_tel(m)
        self.sensors.on_tel_alive(self.alive)
        if self.alive and not was_alive:
            self._ask_capture()

    def on_wifi(self, w):
        self.wifi = w
        self.header.on_wifi(w)

    def show_page(self, name):
        if self.pages.currentWidget() is self.control and name != 'control':
            self.control.release_keys()
        self.pages.setCurrentIndex(self.page_index[name])
        self.pagebar.set(name)
        self.settings.setValue('window/page', name)
        if name == 'bench':
            self._ask_capture()

    def _ask_capture(self):
        """The Bench page's length box is bounded by the blimp's buffer, which
        only a LOG_INFO reveals: ask once, when the page is in view and the
        link is up, so the limit is known before the first Start."""
        if self.alive and self.pages.currentWidget() is self.bench and self.bench.capture.info is None:
            self.station.capture('status')

    def frame(self):
        self.sensors.frame()
        self.camera.age()
        if self.alive and self.tel_at and time.monotonic() - self.tel_at > TEL_STALE_S:
            self.alive = False
            self.motors.set_stale()
            self.control.set_stale()
            self.sensors.on_tel_alive(False)
            self.header.pills['link'].set('crit', 'STALE')

    # ---------------------------------------------------------------- safety

    def stop(self):
        self.control.disarm(send_stop=False)      # the Flight page's stop_all sends it
        self.motors.stop_all(False)
        self.header.stop_b.flash()

    # ---------------------------------------------------------- window, dock

    def toggle_full(self):
        if self.isFullScreen():
            self.showMaximized() if self._was_maximized else self.showNormal()
        else:
            self._was_maximized = self.isMaximized()
            self.showFullScreen()

    def _dock_max(self):
        return max(DOCK_MIN_PX, self.height() - PANELS_MIN_PX)

    def _apply_dock_h(self):
        total = sum(self.split.sizes())
        if self.dock.collapsed:
            bar = self.dock.bar_height()
            self.dock.setFixedHeight(bar)
            self.split.handle(1).setEnabled(False)
            self.split.handle(1).update()
            # The splitter keeps each slot's size until told: without this the
            # dock shrank to its bar inside a slot still as tall as the console
            # had been, and the panels gained nothing (found 2026-11-02)
            self.split.setSizes([total - bar, bar])
            return
        self.dock.setMinimumHeight(DOCK_MIN_PX)
        self.dock.setMaximumHeight(self._dock_max())
        self.split.handle(1).setEnabled(True)
        # _dock_h stays what the user chose (None: the default for this
        # height). Storing the clamped value would keep whatever this computed
        # before the window had its size, which is the 110 px floor.
        want = self._dock_h if self._dock_h is not None else dock_default_px(self.height())
        h = clamp(want, DOCK_MIN_PX, self._dock_max())
        self.split.setSizes([total - h, h])

    def _dock_moved(self, pos, index):
        if not self.dock.collapsed:
            self._dock_h = self.split.sizes()[1]

    def _save_dock_h(self):
        if self._dock_h is not None:
            self.settings.setValue(DOCK_H_KEY, self._dock_h)

    def _reset_dock_h(self):
        self._dock_h = None
        self._apply_dock_h()
        self.settings.remove(DOCK_H_KEY)

    def _collapsed(self, on):
        self._apply_dock_h()
        save_dock(self.settings, self.dock)

    def showEvent(self, e):
        super().showEvent(e)
        QTimer.singleShot(0, self._apply_dock_h)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        compact = self.height() < COMPACT_BELOW_PX
        if compact != self._compact:
            self._compact = compact
            self.motors.compact(compact)
            self.sensors.compact(compact)
            self.control.compact(compact)
        if self.isVisible():
            self._apply_dock_h()

    def closeEvent(self, e):
        if self.motors.enabled or self.control.armed:
            self.control.disarm(send_stop=False)
            self.motors.stop_all(False)
        self.settings.setValue('window/geometry', self.saveGeometry())
        save_dock(self.settings, self.dock)
        self._save_dock_h()
        self._frame.stop()
        self.station.close()
        super().closeEvent(e)


class Keys(QObject):
    """Esc always stops; Space stops unless a text field has focus; F11 toggles
    full screen. On the application, so they work whatever has focus. While
    the Control page is shown, its flying keys (control.KEYS, Shift) go to it
    on press and release, unless a text field has focus; losing the window's
    activation releases them all, since the release would never arrive."""
    TYPING = (QLineEdit, QAbstractSpinBox, QComboBox)

    def __init__(self, win):
        super().__init__(win)
        self.win = win

    def eventFilter(self, obj, e):
        if e.type() == QEvent.Type.WindowDeactivate and obj is self.win:
            self.win.control.release_keys()
            return False
        if e.type() not in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease) or not isinstance(obj, QWidget) \
                or obj.window() is not self.win:
            return False
        k = e.key()
        flying = (self.win.pages.currentWidget() is self.win.control
                  and not isinstance(QApplication.focusWidget(), self.TYPING))
        if e.type() == QEvent.Type.KeyRelease:
            return flying and not e.isAutoRepeat() and self.win.control.key(k, False)
        if flying and not e.isAutoRepeat() and self.win.control.key(k, True):
            return True
        if flying and e.isAutoRepeat() and self.win.control.key(k, True):
            return True                 # a held key repeats; it is already held
        if k == Qt.Key.Key_Escape or (k == Qt.Key.Key_Space and
                                      not isinstance(QApplication.focusWidget(), self.TYPING)):
            self.win.stop()
            return True
        if k == Qt.Key.Key_F11:
            self.win.toggle_full()
            return True
        return False


def parse_args(argv=None):
    ap = argparse.ArgumentParser(prog='python -m software.ground_station',
                                 description='Icarus blimp ground station (one native window).')
    ap.add_argument('--demo', action='store_true', help='mock blimp and demo camera in-process, no hardware')
    ap.add_argument('--blimp', default='192.168.4.1', help='blimp address (default %(default)s)')
    ap.add_argument('--wifi-ssid', default=None,
                    help="the blimp's network, ICARUS-XXXXXX; otherwise a stored profile or a scan decides")
    ap.add_argument('--wifi-password', default='icarusblimp', help='its WPA2 key (default %(default)s)')
    ap.add_argument('--wifi-prefix', default='ICARUS', help='SSID prefix to look for (default %(default)s)')
    ap.add_argument('--mode', choices=MODES, default=None,
                    help="the Control page's input: pc (keyboard, plus a gamepad if present) or deck (the "
                         "Steam Deck's controls); remembered, so it is needed once")
    ap.add_argument('--geometry', metavar='WxH', default=None,
                    help='open at this size instead of the remembered one, e.g. 1280x800 to see the window as '
                         'the Steam Deck shows it')
    ap.add_argument('--page', choices=('flight', 'control', 'bench'), default=None,
                    help='open on this page instead of the remembered one')
    ap.add_argument('--race', action='store_true',
                    help='race mode: the window only watches; no motor command leaves it but STOP (the race is '
                         'autonomous, and no manual control is allowed during it)')
    return ap.parse_args(argv)


def wifi_opts(args):
    return {'prefix': args.wifi_prefix, 'password': args.wifi_password, 'ssid': args.wifi_ssid,
            'blimp_ip': args.blimp}


def build(args, app, settings=None, ports=None):
    """The station and its window, not yet started: main(), screens.py and the
    tests share this. settings: a QSettings to use instead of the real one, so
    a script's run leaves the operator's window as it was. ports: (link,
    console) for the demo blimp, so a test does not collide with a mock
    already on the default ones."""
    T.apply(app)
    station = Station(args.demo, args.blimp, wifi_opts(args), race=args.race, ports=ports)
    win = MainWindow(station, settings if settings is not None else QSettings('Icarus', 'GroundStation'))
    if args.mode:
        win.control.set_mode(args.mode, save=True)
    if args.page:
        win.show_page(args.page)
    if args.geometry:
        try:
            w, h = (int(v) for v in args.geometry.lower().split('x'))
        except ValueError:
            sys.exit('--geometry wants WIDTHxHEIGHT, e.g. 1280x800')
        win.setWindowState(Qt.WindowState.WindowNoState)
        win.resize(w, h)
    app.installEventFilter(Keys(win))
    return station, win


def teardown(station, win):
    """Destroy the window and station while Python is still whole: main() and
    the tests share this. Left to the interpreter's exit, PyQt destroys them
    while walking its own map of wrappers, and the Python methods a dying
    widget still calls (the app-wide Keys filter sees every event) add
    wrappers mid-walk. The map is reallocated under the walk and the exit
    segfaults: 7 runs in 8 under PYTHONMALLOC=debug before this, 2026-10-27."""
    station.close()
    sip.delete(win)         # Keys is its child, so the app-wide filter goes with it
    sip.delete(station)


def main(argv=None):
    args = parse_args(argv)
    app = QApplication(sys.argv[:1])
    app.setApplicationName('Icarus Ground Station')
    station, win = build(args, app)
    win.show()
    station.start()
    # Ctrl-C in the terminal closes the window (and so stops the motors). Qt's
    # loop never returns to Python on its own, so a timer lets the handler run.
    signal.signal(signal.SIGINT, lambda *_: win.close())
    tick = QTimer()
    tick.timeout.connect(lambda: None)
    tick.start(200)
    rc = app.exec()
    tick.stop()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    teardown(station, win)
    sys.exit(rc)


if __name__ == '__main__':
    main()
