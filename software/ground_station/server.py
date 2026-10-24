"""The ground station's back end: one Tornado process serving the page and a
WebSocket, bridging the blimp link, the camera, the console and the
computer's Wi-Fi to it.

Messages are JSON, one object per WebSocket frame, with a `t` field; the
table is in docs/ground-station-plan.md §4. Everything that arrives from the
bridges' threads is hopped onto the IOLoop with add_callback, so handlers
never touch Tornado from another thread.

Two ways to show the page: its own window (window.py, pywebview) when that is
installed, or the system browser. In window mode Tornado runs on a thread
and the window takes the main thread, which is what pywebview requires.
"""
import argparse
import asyncio
import base64
import collections
import json
import math
import os
import sys
import threading
import time
import webbrowser

import tornado.ioloop
import tornado.web
import tornado.websocket

from . import window
from .blimp_bridge import DEADMAN_S, STREAM_HZ, BlimpBridge, DemoBlimp
from .camera import make_camera
from .wifi import Wifi, WifiError

WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')
TICK_MS = 100           # telemetry/sensor push to the browser
WIFI_TICKS = 20         # Wi-Fi state check every 2 s: a routing lookup, no subprocess


def _clean(obj):
    """JSON cannot carry NaN or infinity; the camera rows have both."""
    if isinstance(obj, float):
        return None if (math.isnan(obj) or math.isinf(obj)) else obj
    if isinstance(obj, dict):
        return {k: _clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_clean(v) for v in obj]
    return obj


class Hub:
    def __init__(self, loop, demo, blimp_host, cam_spec, wifi=None):
        self.loop = loop
        self.demo = demo
        self.clients = set()
        self.events = collections.deque(maxlen=200)
        self._last_sensors_rx = -1
        self._ticks = 0
        self._wifi_last = None
        if demo:
            self.blimp = DemoBlimp(self.event)
            cam_spec = cam_spec or 'demo'
            self.wifi = None            # the demo blimp is on localhost: no radio to join
        else:
            self.blimp = BlimpBridge(blimp_host, 7200, 7201, self.event)
            self.wifi = wifi
        self.blimp.start()
        self.camera = make_camera(cam_spec, self._cam_frame, self._cam_row, self._cam_status, self.event)
        self.camera.start()
        self._console_worker = threading.Thread(target=self._console_loop, daemon=True, name='console')
        self._console_q = collections.deque()
        self._console_ev = threading.Event()
        self._console_worker.start()
        self.event('info', 'ground station up: %s' % ('demo blimp and demo camera' if demo else
                                                      'blimp at %s, camera %s' % (blimp_host, cam_spec or 'none')))
        if self.wifi is not None and self.wifi.error:
            self.event('warn', 'wifi: %s' % self.wifi.error)

    # ------------------------------------------------------------ broadcast

    def broadcast(self, msg, only=None):
        """IOLoop thread only."""
        data = json.dumps(_clean(msg), separators=(',', ':'))
        for c in (only,) if only else list(self.clients):
            try:
                c.write_message(data)
            except tornado.websocket.WebSocketClosedError:
                self.clients.discard(c)

    def _post(self, msg, only=None):
        """Any thread."""
        self.loop.add_callback(self.broadcast, msg, only)

    def event(self, level, text):
        ev = {'t': 'ev', 'ts': time.time(), 'level': level, 'text': text}
        self.events.append(ev)
        print('[%s] %-4s %s' % (time.strftime('%H:%M:%S'), level, text), flush=True)
        self._post(ev)

    # ----------------------------------------------------------- periodic

    def tick(self):
        snap = self.blimp.snapshot()
        self.broadcast({'t': 'tel', 'alive': snap['alive'], 'age_s': snap['age_s'], 'rx': snap['rx'],
                        'bad': snap['bad'], 'streaming': snap['streaming'], 'tel': snap['tel']})
        if snap['sen'] is not None and snap['sensors_rx'] != self._last_sensors_rx:
            self._last_sensors_rx = snap['sensors_rx']
            self.broadcast(dict(snap['sen'], t='sen'))
        self._ticks += 1
        if self.wifi is not None and self._ticks % WIFI_TICKS == 0:
            st = self.wifi_status()
            if st != self._wifi_last:           # a manual join or drop shows up here too
                self._wifi_last = st
                self.broadcast(st)

    # ------------------------------------------------------------- camera

    def _cam_frame(self, png, w, h, seq, t_ms, row):
        self._post({'t': 'cam', 'seq': seq, 't_ms': t_ms, 'w': w, 'h': h,
                    'png': 'data:image/png;base64,' + base64.b64encode(png).decode('ascii'), 'row': row})

    def _cam_row(self, row):
        self._post({'t': 'camrow', 'row': row})

    def _cam_status(self, st):
        self._post(dict(st, t='camstat'))

    # -------------------------------------------------------------- wi-fi

    def wifi_status(self):
        if self.wifi is None:
            return {'t': 'wifistat', 'backend': 'none', 'phase': 'demo', 'joined': False}
        return dict(self.wifi.status(), t='wifistat')

    def wifi_action(self, action):
        """From the IOLoop thread; the join itself runs on its own thread
        because it blocks for up to half a minute."""
        if self.wifi is None:
            self.event('warn', 'wifi: demo mode has no radio to join')
            return
        if self.wifi.busy:
            self.event('warn', 'wifi: already %s' % self.wifi.phase)
            return
        threading.Thread(target=self._wifi_run, args=(action,), daemon=True, name='wifi').start()

    def _wifi_run(self, action):
        push = lambda: self._post(self.wifi_status())
        try:
            if action == 'connect':
                self.event('info', 'wifi: joining the blimp network...')
                ssid = self.wifi.connect(on_phase=push)
                self.event('ok', 'wifi: joined %s, this computer is %s' % (ssid, self.wifi.status()['ip']))
            elif action == 'disconnect':
                self.wifi.disconnect(on_phase=push)
                self.event('info', 'wifi: left the blimp network')
        except WifiError as e:
            self.event('warn', 'wifi: %s' % e)
        push()

    # ------------------------------------------------------------ console

    def console(self, client, line):
        self._console_q.append((client, line))
        self._console_ev.set()

    def _console_loop(self):
        while True:
            self._console_ev.wait()
            self._console_ev.clear()
            while self._console_q:
                client, line = self._console_q.popleft()
                self._post({'t': 'con', 'text': '> %s\n' % line}, client)
                try:
                    self.blimp.run_console(line, lambda chunk, c=client: self._post({'t': 'con', 'text': chunk}, c))
                except (TimeoutError, ConnectionError) as e:
                    self._post({'t': 'con', 'text': '[%s]\n' % e}, client)
                self._post({'t': 'con_end'}, client)

    def hello(self):
        return {'t': 'hello', 'demo': self.demo, 'blimp': self.blimp.host,
                'camera': self.camera.status(), 'deadman_s': DEADMAN_S, 'stream_hz': STREAM_HZ,
                'wifi': self.wifi_status(), 'events': list(self.events)}

    def close(self):
        self.camera.stop()
        self.blimp.close()


class WSHandler(tornado.websocket.WebSocketHandler):
    def initialize(self, hub):
        self.hub = hub

    def open(self):
        self.hub.clients.add(self)
        self.hub.broadcast(self.hub.hello(), only=self)

    def on_close(self):
        self.hub.clients.discard(self)
        if not self.hub.clients:
            self.hub.blimp.release()

    def on_message(self, message):
        try:
            m = json.loads(message)
        except ValueError:
            return
        t = m.get('t')
        if t == 'motor':
            self.hub.blimp.set_motors(m.get('cmd', []))
        elif t == 'stop':
            self.hub.blimp.stop(bool(m.get('brake', False)))
        elif t == 'con':
            line = str(m.get('line', '')).strip()
            if line:
                self.hub.console(self, line)
        elif t == 'cam':
            kw = {k: v for k, v in m.items() if k not in ('t', 'action')}
            ok, text = self.hub.camera.command(str(m.get('action', '')), **kw)
            self.hub.event('info' if ok else 'warn', 'camera: %s' % text)
            self.hub.broadcast(dict(self.hub.camera.status(), t='camstat'))
        elif t == 'wifi':
            self.hub.wifi_action(str(m.get('action', '')))


class PageHandler(tornado.web.StaticFileHandler):
    def set_extra_headers(self, path):
        self.set_header('Cache-Control', 'no-cache')   # edits to web/ show on reload


def make_app(hub):
    return tornado.web.Application([
        (r'/ws', WSHandler, {'hub': hub}),
        (r'/(.*)', PageHandler, {'path': WEB, 'default_filename': 'index.html'}),
    ])


class Server:
    """Tornado on whichever thread calls run(); ready is set once it listens
    (or failed, with error set), so the window can wait for the address."""

    def __init__(self, args):
        self.a = args
        self.loop = None
        self.hub = None
        self.url = None
        self.error = None
        self.ready = threading.Event()
        self.open_browser = False

    def run(self):
        a = self.a
        try:
            asyncio.set_event_loop(asyncio.new_event_loop())
            self.loop = tornado.ioloop.IOLoop.current()
            wifi = None if a.demo else Wifi(prefix=a.wifi_prefix, password=a.wifi_password,
                                            ssid=a.wifi_ssid, blimp_ip=a.blimp)
            self.hub = Hub(self.loop, a.demo, a.blimp, a.cam, wifi)
            make_app(self.hub).listen(a.port, address=a.bind)
        except (OSError, RuntimeError) as e:
            self.error = str(e)
            self.ready.set()
            return
        self.url = 'http://%s:%d/' % ('127.0.0.1' if a.bind == '0.0.0.0' else a.bind, a.port)
        print('ground station at %s   (Ctrl+C stops)' % self.url, flush=True)
        tornado.ioloop.PeriodicCallback(self.hub.tick, TICK_MS).start()
        if self.open_browser:
            self.loop.call_later(0.5, lambda: webbrowser.open(self.url))
        self.ready.set()
        try:
            self.loop.start()
        finally:
            self.hub.close()

    def stop(self):
        """Any thread."""
        if self.loop is not None:
            self.loop.add_callback(self.loop.stop)


def main(argv=None):
    p = argparse.ArgumentParser(description='Ground station for the blimp (see docs/ground-station-plan.md)')
    p.add_argument('--demo', action='store_true', help='mock blimp and demo camera in-process, no hardware')
    p.add_argument('--blimp', default='192.168.4.1', help='blimp address (default %(default)s)')
    p.add_argument('--cam', default=None, help="camera serial port (COM5, /dev/ttyUSB0), or 'demo'")
    p.add_argument('--port', type=int, default=8765, help='HTTP port (default %(default)s)')
    p.add_argument('--bind', default='127.0.0.1',
                   help='address to listen on; 0.0.0.0 lets a tablet on the same Wi-Fi open the page')
    g = p.add_mutually_exclusive_group()
    g.add_argument('--window', action='store_true',
                   help='show the page in its own window (the default when pywebview is installed)')
    g.add_argument('--browser', action='store_true', help='open the system browser instead of a window')
    g.add_argument('--no-browser', action='store_true', help='serve only; open the address yourself')
    p.add_argument('--wifi-ssid', default=None,
                   help="the blimp's network, ICARUS-XXXXXX; otherwise a stored profile or a scan decides")
    p.add_argument('--wifi-password', default='icarusblimp', help='its WPA2 key (default %(default)s)')
    p.add_argument('--wifi-prefix', default='ICARUS', help='SSID prefix to look for (default %(default)s)')
    a = p.parse_args(argv)

    if a.window and not window.available():
        sys.exit('the window needs pywebview: pip install pywebview   (or run with --browser)')
    mode = 'none' if a.no_browser else 'browser' if a.browser else \
        'window' if (a.window or window.available()) else 'browser'
    if mode == 'browser' and not a.browser:
        # The usual reason: a virtual environment whose Python lacks the package.
        print('pywebview is not installed for %s: opening the browser instead '
              '(pip install pywebview gives the window)' % sys.executable, flush=True)

    srv = Server(a)
    if mode == 'window':
        t = threading.Thread(target=srv.run, name='tornado', daemon=True)
        t.start()
        srv.ready.wait()
        if srv.error:
            sys.exit('cannot start: %s' % srv.error)
        try:
            window.run(srv.url)             # blocks until the window closes
        finally:
            srv.stop()
            t.join(timeout=5)
        return

    srv.open_browser = (mode == 'browser')
    try:
        srv.run()
    except KeyboardInterrupt:
        pass
    if srv.error:
        sys.exit('cannot start: %s' % srv.error)
