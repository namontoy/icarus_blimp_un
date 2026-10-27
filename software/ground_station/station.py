"""The ground station's back end inside the GUI process: the blimp link, the
camera, the computer's Wi-Fi and the console, as one QObject.

Ported from the Tornado version's server.Hub, with Qt signals in place of the
WebSocket messages (table in docs/ground-station-plan.md §4). The bridges call
back from their own threads; a signal emitted from another thread is queued
onto the GUI thread by Qt, which is the job loop.add_callback used to do, so
no slot ever runs off the GUI thread.

Construct it, connect the window to its signals, then start(): nothing is
emitted before start(), so no event is lost to a signal with no receiver yet.
"""
import collections
import threading
import time

from PyQt6.QtCore import QObject, QTimer, pyqtSignal
from PyQt6.QtGui import QImage

from .blimp_bridge import DEADMAN_S, STREAM_HZ, BlimpBridge, DemoBlimp
from .camera import make_camera
from .wifi import Wifi, WifiError

TICK_MS = 100           # telemetry/sensor refresh of the window
WIFI_TICKS = 20         # Wi-Fi state check every 2 s: a routing lookup, no subprocess


class Station(QObject):
    tel = pyqtSignal(dict)                  # link state plus the latest telemetry, every tick
    sen = pyqtSignal(dict)                  # a new sensor packet
    cam_frame = pyqtSignal(QImage, dict)    # frame, {seq, t_ms, w, h, row}
    cam_row = pyqtSignal(dict)              # a camera CSV row without a frame
    cam_status = pyqtSignal(dict)
    con_text = pyqtSignal(str)              # console output, chunk by chunk
    con_end = pyqtSignal()                  # the command finished
    # The plan named this one event(dict); QObject.event() is Qt's event
    # handler, and a signal of that name would shadow it.
    ev = pyqtSignal(dict)                   # {ts, level, text}
    wifi = pyqtSignal(dict)                 # Wifi.status() plus backend/phase

    def __init__(self, demo, blimp_host, wifi_opts=None, parent=None):
        """wifi_opts: Wifi()'s keyword arguments, or None for no radio control."""
        super().__init__(parent)
        self.demo = demo
        self.blimp_host = blimp_host
        self._wifi_opts = None if demo else wifi_opts   # the demo blimp is on localhost: no radio to join
        self.radio = None
        self.blimp = None
        self.camera = None
        self._last_sensors_rx = -1
        self._ticks = 0
        self._wifi_last = None
        self._console_q = collections.deque()
        self._console_ev = threading.Event()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.tick)
        self._closed = False

    def start(self):
        if self._wifi_opts is not None:
            self.radio = Wifi(**self._wifi_opts)
        if self.demo:
            self.blimp = DemoBlimp(self.log)
        else:
            self.blimp = BlimpBridge(self.blimp_host, 7200, 7201, self.log)
        self.blimp.start()
        self.camera = make_camera(self.demo, self._cam_frame, self._cam_row, self.cam_status.emit, self.log)
        self.camera.start()
        threading.Thread(target=self._console_loop, daemon=True, name='console').start()
        self.log('info', 'ground station up: %s' % ('demo blimp and demo camera' if self.demo else
                                                    'blimp at %s' % self.blimp_host))
        if self.radio is not None and self.radio.error:
            self.log('warn', 'wifi: %s' % self.radio.error)
        self.cam_status.emit(self.camera.status())
        self._wifi_last = self.wifi_status()
        self.wifi.emit(self._wifi_last)
        self._timer.start(TICK_MS)
        return self

    # --------------------------------------------------------------- events

    def log(self, level, text):
        """Any thread."""
        print('[%s] %-4s %s' % (time.strftime('%H:%M:%S'), level, text), flush=True)
        self.ev.emit({'ts': time.time(), 'level': level, 'text': text})

    # ------------------------------------------------------------- periodic

    def tick(self):
        snap = self.blimp.snapshot()
        self.tel.emit({'alive': snap['alive'], 'age_s': snap['age_s'], 'rx': snap['rx'], 'bad': snap['bad'],
                       'streaming': snap['streaming'], 'tel': snap['tel']})
        if snap['sen'] is not None and snap['sensors_rx'] != self._last_sensors_rx:
            self._last_sensors_rx = snap['sensors_rx']
            self.sen.emit(dict(snap['sen']))
        self._ticks += 1
        if self.radio is not None and self._ticks % WIFI_TICKS == 0:
            st = self.wifi_status()
            if st != self._wifi_last:           # a manual join or drop shows up here too
                self._wifi_last = st
                self.wifi.emit(st)

    # --------------------------------------------------------------- motors

    def set_motors(self, cmd):
        self.blimp.set_motors(cmd)

    def stop(self, brake=False):
        self.blimp.stop(brake)

    # --------------------------------------------------------------- camera

    def _cam_frame(self, png, w, h, seq, t_ms, row):
        """Camera thread: decode here so the GUI thread only paints."""
        if self._closed:
            return
        img = QImage.fromData(png, 'PNG')
        self.cam_frame.emit(img, {'seq': seq, 't_ms': t_ms, 'w': w, 'h': h, 'row': row})

    def _cam_row(self, row):
        self.cam_row.emit(row)

    def camera_command(self, action, **kw):
        ok, text = self.camera.command(action, **kw)
        self.log('info' if ok else 'warn', 'camera: %s' % text)
        self.cam_status.emit(self.camera.status())

    # ---------------------------------------------------------------- wi-fi

    def wifi_status(self):
        if self.radio is None:
            return {'backend': 'none', 'phase': 'demo' if self.demo else 'idle', 'joined': False}
        return self.radio.status()

    def wifi_action(self, action):
        """The join itself runs on its own thread: it blocks for up to half a minute."""
        if self.radio is None:
            self.log('warn', 'wifi: demo mode has no radio to join' if self.demo else 'wifi: no radio control')
            return
        if self.radio.busy:
            self.log('warn', 'wifi: already %s' % self.radio.phase)
            return
        threading.Thread(target=self._wifi_run, args=(action,), daemon=True, name='wifi').start()

    def _wifi_run(self, action):
        push = lambda: self.wifi.emit(self.wifi_status())
        try:
            if action == 'connect':
                self.log('info', 'wifi: joining the blimp network...')
                ssid = self.radio.connect(on_phase=push)
                self.log('ok', 'wifi: joined %s, this computer is %s' % (ssid, self.radio.status()['ip']))
            elif action == 'disconnect':
                self.radio.disconnect(on_phase=push)
                self.log('info', 'wifi: left the blimp network')
        except WifiError as e:
            self.log('warn', 'wifi: %s' % e)
        push()

    # -------------------------------------------------------------- console

    def console(self, line):
        self._console_q.append(line)
        self._console_ev.set()

    def _console_loop(self):
        while True:
            self._console_ev.wait()
            self._console_ev.clear()
            while self._console_q:
                line = self._console_q.popleft()
                self.con_text.emit('> %s\n' % line)
                try:
                    self.blimp.run_console(line, self.con_text.emit)
                except (TimeoutError, ConnectionError) as e:
                    self.con_text.emit('[%s]\n' % e)
                self.con_end.emit()

    # ---------------------------------------------------------------- close

    @property
    def deadman_s(self):
        return DEADMAN_S

    @property
    def stream_hz(self):
        return STREAM_HZ

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._timer.stop()
        if self.camera is not None:
            self.camera.stop()
        if self.blimp is not None:
            self.blimp.close()
