"""The ground station's back end: one Tornado process serving the page and a
WebSocket, bridging the blimp link, the camera and the console to it.

Messages are JSON, one object per WebSocket frame, with a `t` field; the
table is in docs/ground-station-plan.md §4. Everything that arrives from the
bridges' threads is hopped onto the IOLoop with add_callback, so handlers
never touch Tornado from another thread.
"""
import argparse
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

from .blimp_bridge import DEADMAN_S, STREAM_HZ, BlimpBridge, DemoBlimp
from .camera import make_camera

WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')
TICK_MS = 100           # telemetry/sensor push to the browser


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
    def __init__(self, loop, demo, blimp_host, cam_spec):
        self.loop = loop
        self.demo = demo
        self.clients = set()
        self.events = collections.deque(maxlen=200)
        self._last_sensors_rx = -1
        if demo:
            self.blimp = DemoBlimp(self.event)
            cam_spec = cam_spec or 'demo'
        else:
            self.blimp = BlimpBridge(blimp_host, 7200, 7201, self.event)
        self.blimp.start()
        self.camera = make_camera(cam_spec, self._cam_frame, self._cam_row, self._cam_status, self.event)
        self.camera.start()
        self._console_worker = threading.Thread(target=self._console_loop, daemon=True, name='console')
        self._console_q = collections.deque()
        self._console_ev = threading.Event()
        self._console_worker.start()
        self.event('info', 'ground station up: %s' % ('demo blimp and demo camera' if demo else
                                                      'blimp at %s, camera %s' % (blimp_host, cam_spec or 'none')))

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

    # ------------------------------------------------------------- camera

    def _cam_frame(self, png, w, h, seq, t_ms, row):
        self._post({'t': 'cam', 'seq': seq, 't_ms': t_ms, 'w': w, 'h': h,
                    'png': 'data:image/png;base64,' + base64.b64encode(png).decode('ascii'), 'row': row})

    def _cam_row(self, row):
        self._post({'t': 'camrow', 'row': row})

    def _cam_status(self, st):
        self._post(dict(st, t='camstat'))

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
                'events': list(self.events)}

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


class PageHandler(tornado.web.StaticFileHandler):
    def set_extra_headers(self, path):
        self.set_header('Cache-Control', 'no-cache')   # edits to web/ show on reload


def make_app(hub):
    return tornado.web.Application([
        (r'/ws', WSHandler, {'hub': hub}),
        (r'/(.*)', PageHandler, {'path': WEB, 'default_filename': 'index.html'}),
    ])


def main(argv=None):
    p = argparse.ArgumentParser(description='Ground station for the blimp (see docs/ground-station-plan.md)')
    p.add_argument('--demo', action='store_true', help='mock blimp and demo camera in-process, no hardware')
    p.add_argument('--blimp', default='192.168.4.1', help='blimp address (default %(default)s)')
    p.add_argument('--cam', default=None, help="camera serial port (COM5, /dev/ttyUSB0), or 'demo'")
    p.add_argument('--port', type=int, default=8765, help='HTTP port (default %(default)s)')
    p.add_argument('--bind', default='127.0.0.1',
                   help='address to listen on; 0.0.0.0 lets a tablet on the same Wi-Fi open the page')
    p.add_argument('--no-browser', action='store_true')
    a = p.parse_args(argv)

    loop = tornado.ioloop.IOLoop.current()
    try:
        hub = Hub(loop, a.demo, a.blimp, a.cam)
    except (OSError, RuntimeError) as e:
        sys.exit('cannot start: %s' % e)
    app = make_app(hub)
    try:
        app.listen(a.port, address=a.bind)
    except OSError as e:
        sys.exit('cannot listen on %s:%d: %s' % (a.bind, a.port, e))
    url = 'http://%s:%d/' % ('127.0.0.1' if a.bind == '0.0.0.0' else a.bind, a.port)
    print('ground station at %s   (Ctrl+C stops)' % url, flush=True)
    tornado.ioloop.PeriodicCallback(hub.tick, TICK_MS).start()
    if not a.no_browser:
        loop.call_later(0.5, lambda: webbrowser.open(url))
    try:
        loop.start()
    except KeyboardInterrupt:
        pass
    finally:
        hub.close()
