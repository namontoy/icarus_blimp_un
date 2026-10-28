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
import json
import os
import threading
import time

from PyQt6.QtCore import QObject, QTimer, pyqtSignal
from PyQt6.QtGui import QImage

import blimp_link as bl
import blimp_log

from .blimp_bridge import DEADMAN_S, STREAM_HZ, BlimpBridge, DemoBlimp
from .camera import make_camera
from .gamepad import Gamepad, empty_state
from .wifi import Wifi, WifiError

TICK_MS = 100           # telemetry/sensor refresh of the window
WIFI_TICKS = 20         # Wi-Fi state check every 2 s: a routing lookup, no subprocess
CAP_POLL_S = 0.5        # progress of a running capture
CAPTURES_DIR = 'captures'   # under the working directory; gitignored


def save_capture(info, data, notes, source, folder=CAPTURES_DIR):
    """The .bin blimp_log.py reads, and a .json beside it with the header, the
    operator's notes and the summary figures, so a capture found a month later
    still says what it was. Returns (path, header, records, summary)."""
    path = bl.capture_path(folder=folder)
    blimp_log.write_file(path, info['raw'], data)
    h, records = blimp_log.decode(info['raw'], data)
    summ = blimp_log.summary(h, records)
    meta = {'file': os.path.basename(path), 'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'source': source,
            'notes': notes, 'header': h, 'records': len(records), 'duration_s': summ['duration_s'],
            'dt_ms': summ['dt_ms'], 'gaps': summ['gaps'], 'exec_us': summ['exec_us'],
            'fresh_hz': summ['fresh_hz'], 'flags_seen': summ['flags_seen']}
    with open(os.path.splitext(path)[0] + '.json', 'w') as f:
        json.dump(meta, f, indent=2)
    return path, h, records, summ


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
    cap = pyqtSignal(dict)                  # the capture: {phase, info, progress, file, error}
    cap_data = pyqtSignal(dict)             # a capture downloaded and decoded: {header, records, rows, summary, file}

    def __init__(self, demo, blimp_host, wifi_opts=None, parent=None):
        """wifi_opts: Wifi()'s keyword arguments, or None for no radio control."""
        super().__init__(parent)
        self.demo = demo
        self.blimp_host = blimp_host
        self._wifi_opts = None if demo else wifi_opts   # the demo blimp is on localhost: no radio to join
        self.radio = None
        self.blimp = None
        self.camera = None
        self.pad = None                     # the gamepad reader (gamepad.py), for the Control page
        self.alive = False                  # telemetry within the last second, as of the last tick
        self._ticks = 0
        self._wifi_last = None
        self._console_q = collections.deque()
        self._console_ev = threading.Event()
        self._cap_q = collections.deque()
        self._cap_ev = threading.Event()
        self._cap_watch = False             # poll the blimp while a capture runs
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.tick)
        self._closed = False

    def start(self):
        if self._wifi_opts is not None:
            self.radio = Wifi(**self._wifi_opts)
        if self.demo:
            self.blimp = DemoBlimp(self.log, self._packet)
        else:
            self.blimp = BlimpBridge(self.blimp_host, 7200, 7201, self.log, self._packet)
        self.blimp.start()
        self.camera = make_camera(self.demo, self._cam_frame, self._cam_row, self.cam_status.emit, self.log)
        self.camera.start()
        self.pad = Gamepad().start()
        threading.Thread(target=self._console_loop, daemon=True, name='console').start()
        threading.Thread(target=self._cap_loop, daemon=True, name='capture').start()
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
        self.alive = bool(snap['alive'])
        self.tel.emit({'alive': snap['alive'], 'age_s': snap['age_s'], 'rx': snap['rx'], 'bad': snap['bad'],
                       'streaming': snap['streaming'], 'tel': snap['tel']})
        self._ticks += 1
        if self.radio is not None and self._ticks % WIFI_TICKS == 0:
            st = self.wifi_status()
            if st != self._wifi_last:           # a manual join or drop shows up here too
                self._wifi_last = st
                self.wifi.emit(st)

    def _packet(self, kind, d):
        """Receive thread: every SENSORS packet reaches the window, in the
        order it arrived. Until 2026-10-28 the window took the newest one at
        each 10 Hz tick, so half of a 20 Hz stream was dropped and the plots
        and the capture disagreed; a signal emitted here is queued onto the
        GUI thread by Qt, one per packet."""
        if kind == 'sensors' and not self._closed:
            self.sen.emit(dict(d))

    # --------------------------------------------------------------- motors

    def pad_state(self):
        """The gamepad's latest snapshot (gamepad.empty_state() before start()
        or on a platform without a backend). Polled by the Control page at its
        own tick rather than signalled: a stick is a continuous input, and the
        page wants the value at the moment it mixes, not a queue of them."""
        if self.pad is None:
            return empty_state()
        return self.pad.state()

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

    # -------------------------------------------------------------- capture

    def capture(self, action, seconds=60, notes=''):
        """start, stop, clear, status or download, run on the capture thread;
        the answers come back on the cap signal, a download also on cap_data."""
        self._cap_q.append((action, seconds, notes))
        self._cap_ev.set()

    def _cap_loop(self):
        while not self._closed:
            self._cap_ev.wait(CAP_POLL_S if self._cap_watch else None)
            self._cap_ev.clear()
            if self._closed:
                return
            while self._cap_q:
                self._cap_run(*self._cap_q.popleft())
            if self._cap_watch and not self._cap_q:
                try:
                    info = self.blimp.capture.info()
                except (TimeoutError, bl.CaptureError, OSError) as e:
                    self._cap_watch = False
                    self.cap.emit({'phase': 'error', 'error': str(e)})
                    continue
                self._cap_watch = info['state'] == 1
                self.cap.emit({'phase': 'status', 'info': info})
                if not self._cap_watch:
                    self.log('ok', 'capture %d finished: %d records on the blimp' % (info['capture_id'],
                                                                                   info['count']))

    def _cap_run(self, action, seconds, notes):
        cap = self.blimp.capture
        try:
            if action == 'start':
                info = cap.start(seconds)
                self._cap_watch = True
                self.log('info', 'capture %d started: %d s' % (info['capture_id'], seconds))
                self.cap.emit({'phase': 'status', 'info': info})
            elif action == 'stop':
                info = cap.stop()
                self._cap_watch = False
                self.log('info', 'capture stopped at %d records' % info['count'])
                self.cap.emit({'phase': 'status', 'info': info})
            elif action == 'clear':
                info = cap.clear()
                self.cap.emit({'phase': 'status', 'info': info})
            elif action == 'status':
                info = cap.info()
                self._cap_watch = info['state'] == 1
                self.cap.emit({'phase': 'status', 'info': info})
            elif action == 'download':
                info = cap.info()
                if info['state'] == 1:
                    info = cap.stop()
                    self._cap_watch = False
                    self.log('info', 'capture stopped for the download at %d records' % info['count'])
                if info['count'] == 0:
                    self.log('warn', 'capture: nothing stored on the blimp')
                    self.cap.emit({'phase': 'status', 'info': info})
                    return
                self.cap.emit({'phase': 'download', 'info': info, 'progress': 0.0})
                info, data = cap.download(
                    progress=lambda k, n: self.cap.emit({'phase': 'download', 'info': info, 'progress': k / n}))
                path, h, records, summ = save_capture(info, data, notes,
                                                      'demo' if self.demo else self.blimp_host)
                self.log('ok', 'capture %d: %d records saved to %s' % (info['capture_id'], len(records), path))
                self.cap.emit({'phase': 'saved', 'info': info, 'file': path})
                self.cap_data.emit({'header': h, 'records': records, 'rows': blimp_log.to_units(h, records),
                                    'summary': summ, 'file': path})
        except (TimeoutError, bl.CaptureError, OSError, ValueError) as e:
            self._cap_watch = False
            self.log('warn', 'capture: %s' % e)
            self.cap.emit({'phase': 'error', 'error': str(e)})

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
        self._cap_ev.set()
        if self.camera is not None:
            self.camera.stop()
        if self.pad is not None:
            self.pad.stop()
        if self.blimp is not None:
            self.blimp.close()
