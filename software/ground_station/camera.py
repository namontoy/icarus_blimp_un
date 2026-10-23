"""Camera sources for the ground station: the ESP32-CAM over serial, or a demo.

Each source runs its own thread and calls back with frames, rows and status:

    on_frame(png_bytes, w, h, seq, t_ms, row_or_None)
    on_row(row_dict)                 the camera's CSV row (tcview.protocol.COLUMNS)
    on_status(dict)                  connection, baud, counters
    on_event(level, text)

Frames are delivered as PNG (the browser decodes that natively) built here from
the camera's raw RGB565, so the page never sees the sensor's byte order.

SerialCamera reuses the parser from the target_cam viewer,
firmware/projects/target_cam/host/viewer/tcview/protocol.py, which is Qt-free.
That project lives on the target-cam branches; until it is merged this source
reports it as missing and the page runs on DemoCamera instead.
"""
import math
import os
import struct
import sys
import threading
import time
import zlib

try:
    import numpy as np
except ImportError:                 # pure-Python fallbacks below, slower but fine at QQVGA
    np = None

W, H = 160, 120
FOV_DEG = 54.0                      # target_cam default horizontal field of view


# ------------------------------------------------------------------ images

def png_rgb(w, h, rgb):
    """A PNG from packed 8-bit RGB rows, standard library only."""
    raw = b''.join(b'\x00' + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))

    def chunk(tag, data):
        body = tag + data
        return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body) & 0xFFFFFFFF)

    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def rgb565_to_rgb(data, w, h, big_endian):
    """Raw RGB565 words -> packed RGB888 bytes, replicating the top bits into the
    low ones as the viewer does, so full scale is 255 and not 248."""
    n = w * h
    if np is not None:
        px = np.frombuffer(data, dtype='>u2' if big_endian else '<u2', count=n).astype(np.uint32)
        r, g, b = (px >> 11) & 0x1F, (px >> 5) & 0x3F, px & 0x1F
        out = np.empty((n, 3), dtype=np.uint8)
        out[:, 0] = (r << 3) | (r >> 2)
        out[:, 1] = (g << 2) | (g >> 4)
        out[:, 2] = (b << 3) | (b >> 2)
        return out.tobytes()
    fmt = ('>' if big_endian else '<') + '%dH' % n
    out = bytearray(n * 3)
    for i, px in enumerate(struct.unpack(fmt, data[:n * 2])):
        r, g, b = (px >> 11) & 0x1F, (px >> 5) & 0x3F, px & 0x1F
        out[3 * i] = (r << 3) | (r >> 2)
        out[3 * i + 1] = (g << 2) | (g >> 4)
        out[3 * i + 2] = (b << 3) | (b >> 2)
    return bytes(out)


# ------------------------------------------------------------------ sources

class CameraSource:
    name = 'none'

    def __init__(self, on_frame, on_row, on_status, on_event):
        self.on_frame, self.on_row, self.on_status, self.on_event = on_frame, on_row, on_status, on_event

    def start(self):
        return self

    def stop(self):
        pass

    def command(self, action, **kw):
        """Returns (ok, text)."""
        return False, 'no camera configured (start with --cam PORT or --cam demo)'

    def status(self):
        return {'source': self.name, 'connected': False, 'live': False}


class DemoCamera(CameraSource):
    """A red balloon drifting over a wall with a wood patch, at the camera's own
    12.5 frames per second: rows every tick, frames when asked. The verdict and
    the blob geometry follow the drawn disc, so the page's overlay lines up."""
    name = 'demo'
    TICK_S = 0.08                   # the sensor's frame period (80 ms at XCLK 20 MHz)

    def __init__(self, *cbs):
        super().__init__(*cbs)
        self._live = False
        self._grab = 0
        self._seq = 0
        self._t0 = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name='demo_cam')
        # Background: a grey wall with a slight gradient and a wood-coloured
        # patch lower-left, as the viewer's fake camera draws it.
        self._bg = bytearray(W * H * 3)
        for y in range(H):
            for x in range(W):
                v = int(150 + 30 * x / W - 20 * y / H)
                r, g, b = v, int(v * 0.98), int(v * 0.95)
                if x < 45 and y > 80:
                    r, g, b = 150, 95, 60
                i = 3 * (y * W + x)
                self._bg[i], self._bg[i + 1], self._bg[i + 2] = r, g, b

    def start(self):
        self._thread.start()
        self.on_status(self.status())
        return self

    def stop(self):
        self._stop.set()

    def status(self):
        return {'source': self.name, 'connected': True, 'live': self._live, 'baud': 0,
                'port': 'demo', 'frames_ok': self._seq, 'frames_bad': 0}

    def command(self, action, **kw):
        if action == 'grab':
            self._grab += 1
            return True, 'frame requested'
        if action == 'live':
            self._live = bool(kw.get('on', not self._live))
            self.on_status(self.status())
            return True, 'live %s' % ('on' if self._live else 'off')
        if action in ('connect', 'disconnect'):
            return True, 'demo camera is always connected'
        if action == 'baud':
            return True, 'demo camera has no baud rate'
        if action == 'send':
            return True, 'demo camera ignores raw commands'
        return False, 'unknown camera action %r' % action

    def _scene(self, t):
        """Disc centre and radius at time t: a slow loop that comes closer and
        goes away, leaving the frame now and then so NO TARGET also shows."""
        cx = 80 + 62 * math.sin(t * 0.21)
        cy = 60 + 30 * math.sin(t * 0.33 + 1.0)
        r = 9 + 11 * (0.5 + 0.5 * math.sin(t * 0.17))
        return cx, cy, r

    def _row(self, t_ms, cx, cy, r):
        visible = -r + 4 < cx < W + r - 4 and -r + 4 < cy < H + r - 4
        x0, x1 = max(0, cx - r), min(W - 1, cx + r)
        y0, y1 = max(0, cy - r), min(H - 1, cy + r)
        area = int(math.pi * r * r) if visible else 0
        clipped = 1 if visible and (x0 == 0 or y0 == 0 or x1 == W - 1 or y1 == H - 1) else 0
        az = (cx - W / 2) / W * FOV_DEG
        el = -(cy - H / 2) / H * FOV_DEG * 0.75
        # 550 mm balloon through a pinhole: range = f * D / w, with f such that
        # 40 px across reads 1.1 m, as the viewer's fake camera is calibrated.
        range_mm = int(44000 / max(2 * r, 1)) if visible else 0
        row = {'t_ms': t_ms, 'ok': 1 if visible else 0, 'cx_px': round(cx, 1), 'cy_px': round(cy, 1),
               'az_deg': round(az, 2), 'el_deg': round(el, 2), 'area_px': area,
               'w_px': int(x1 - x0), 'h_px': int(y1 - y0), 'fill': 0.78, 'range_mm': range_mm,
               'clipped': clipped, 'blobs': 1 if visible else 0, 'matched_px': int(area * 0.9),
               'dt_ms': 80, 'proc_us': 21500, 'stage': 1, 'conf': 0.93 if visible else -1.0,
               'b2_conf': -1.0, 'b3_conf': -1.0, 'b4_conf': -1.0, 't_px_us': 9800, 't_b_us': 1200,
               't_c_us': float('nan'), 'loop_us': 61000, 'csv_v': 2}
        for k in range(2, 5):
            row['b%d_az' % k] = row['b%d_el' % k] = row['b%d_range_mm' % k] = 0
        return row

    def _frame(self, cx, cy, r):
        buf = bytearray(self._bg)
        for y in range(max(0, int(cy - r)), min(H, int(cy + r) + 1)):
            dy = y - cy
            half = math.sqrt(max(0.0, r * r - dy * dy))
            xa, xb = max(0, int(cx - half)), min(W - 1, int(cx + half))
            if xb < xa:
                continue
            shade = 1.0 - 0.35 * abs(dy) / r
            px = bytes((int(205 * shade), int(28 * shade), int(36 * shade)))
            buf[3 * (y * W + xa):3 * (y * W + xb + 1)] = px * (xb - xa + 1)
        return png_rgb(W, H, bytes(buf))

    def _loop(self):
        next_t = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(0.02, next_t - now))
                continue
            next_t += self.TICK_S
            t = now - self._t0
            t_ms = int(t * 1000)
            cx, cy, r = self._scene(t)
            row = self._row(t_ms, cx, cy, r)
            if self._live or self._grab > 0:
                self._grab = max(0, self._grab - 1)
                self._seq += 1
                self.on_frame(self._frame(cx, cy, r), W, H, self._seq, t_ms, row)
            else:
                self.on_row(row)


class SerialCamera(CameraSource):
    """The ESP32-CAM on its USB serial port, speaking the target_cam protocol.

    Not yet run against hardware from here (docs/ground-station-plan.md §8):
    the parser is the viewer's and is tested there; the serial handling below
    follows the two lessons recorded in tcview/link.py, DTR and RTS low before
    opening so the board is not reset, and one command at a time."""
    name = 'serial'
    BOOT_BAUD = 115200
    REVERT_S = 3.0

    def __init__(self, port, *cbs):
        super().__init__(*cbs)
        self.port = port
        self.baud = self.BOOT_BAUD
        self._P = _load_tcview_protocol()
        import serial  # pyserial
        self._serial_mod = serial
        self._ser = None
        self._live = False
        self._pending_frame = None      # (png, w, h, seq, t_ms, arrived) waiting for its row
        self._want_baud = None
        self._baud_at = 0.0
        self._last_row_at = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name='serial_cam')
        self.parser = self._P.StreamParser()
        self.rows = 0

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()

    def status(self):
        p = self.parser
        return {'source': self.name, 'connected': self._ser is not None, 'live': self._live,
                'port': self.port, 'baud': self.baud, 'rows': self.rows,
                'frames_ok': p.frames_ok, 'frames_bad': p.frames_bad, 'junk': p.junk_lines}

    def _write(self, text):
        if self._ser is None:
            return False
        try:
            self._ser.write(b'\n' + text.encode('ascii') + b'\n')   # a newline first flushes a half-line
            return True
        except (OSError, ValueError):
            return False

    def command(self, action, **kw):
        if action == 'connect':
            return (True, 'connecting') if self._ser is None else (True, 'already open')
        if action == 'disconnect':
            self._disconnect()
            return True, 'port closed'
        if self._ser is None:
            return False, 'camera port is not open'
        if action == 'grab':
            return (self._write('g'), 'frame requested')
        if action == 'live':
            on = bool(kw.get('on', not self._live))
            ok = self._write('v1' if on else 'v0')
            if ok:
                self._live = on
            return ok, 'live %s' % ('on' if on else 'off')
        if action == 'baud':
            b = int(kw.get('baud', self.BOOT_BAUD))
            if b not in self._P.BAUDS:
                return False, 'baud must be one of %s' % self._P.BAUDS
            self._want_baud = b
            return self._write('B%d' % b), 'asked the camera for %d baud' % b
        if action == 'send':
            return self._write(str(kw.get('text', ''))), 'sent'
        return False, 'unknown camera action %r' % action

    def _open(self):
        try:
            ser = self._serial_mod.Serial()
            ser.port = self.port
            ser.baudrate = self.BOOT_BAUD
            ser.timeout = 0.05
            ser.dtr = False
            ser.rts = False
            ser.exclusive = True
            ser.open()
        except (OSError, ValueError) as e:
            self.on_event('crit', 'camera port %s: %s' % (self.port, e))
            return False
        self._ser = ser
        self.baud = self.BOOT_BAUD
        self.parser.reset()
        self.on_event('ok', 'camera port %s open at %d' % (self.port, self.baud))
        self.on_status(self.status())
        return True

    def _disconnect(self):
        if self._ser is not None:
            try:
                if self.baud != self.BOOT_BAUD:
                    self._write('B%d' % self.BOOT_BAUD)
                    time.sleep(0.2)
                self._ser.close()
            except OSError:
                pass
            self._ser = None
            self._live = False
            self.on_status(self.status())

    def _set_baud(self, baud):
        self._ser.baudrate = baud
        self.baud = baud
        self.parser.reset()
        self._write('k')                # tells the camera we can hear it here
        self._baud_at = time.monotonic()
        self.on_event('ok', 'camera link now at %d baud' % baud)
        self.on_status(self.status())

    def _loop(self):
        P = self._P
        last_status = 0.0
        while not self._stop.is_set():
            if self._ser is None:
                if not self._open():
                    time.sleep(2.0)
                    continue
            try:
                data = self._ser.read(4096)
            except (OSError, ValueError) as e:
                self.on_event('crit', 'camera port lost: %s' % e)
                self._ser = None
                continue
            now = time.monotonic()
            events = self.parser.feed(data, now) if data else []
            events += self.parser.expire(now, self.baud)
            for ev in events:
                self._dispatch(ev, now)
            # The camera reverts to 115200 by itself if it hears nothing valid
            # for 3 s after a switch; follow it down if rows stopped.
            if self.baud != self.BOOT_BAUD and now - max(self._last_row_at, self._baud_at) > self.REVERT_S + 1.0:
                self.on_event('warn', 'nothing heard at %d baud, back to %d' % (self.baud, self.BOOT_BAUD))
                self._set_baud(self.BOOT_BAUD)
            if self._pending_frame and now - self._pending_frame[5] > 0.5:
                png, w, h, seq, t_ms, _ = self._pending_frame
                self._pending_frame = None
                self.on_frame(png, w, h, seq, t_ms, None)
            if now - last_status > 1.0:
                last_status = now
                self.on_status(self.status())

    def _dispatch(self, ev, now):
        P = self._P
        if isinstance(ev, P.Row):
            self.rows += 1
            self._last_row_at = now
            if self._pending_frame and abs(ev.v.get('t_ms', -1) - self._pending_frame[4]) < 1:
                png, w, h, seq, t_ms, _ = self._pending_frame
                self._pending_frame = None
                self.on_frame(png, w, h, seq, t_ms, ev.v)
            else:
                self.on_row(ev.v)
        elif isinstance(ev, P.Frame):
            rgb = rgb565_to_rgb(ev.data, ev.w, ev.h, ev.big_endian)
            self._pending_frame = (png_rgb(ev.w, ev.h, rgb), ev.w, ev.h, ev.seq, ev.t_ms, now)
        elif isinstance(ev, P.FrameError):
            self.on_event('warn', 'camera %s %s: %s' % (ev.kind.lower(), ev.seq, ev.reason))
        elif isinstance(ev, P.Baud):
            if self._want_baud == ev.baud:
                self._want_baud = None
                self._set_baud(ev.baud)
        elif isinstance(ev, P.Nak):
            self.on_event('warn', 'camera refused %r: %s' % (ev.cmd, ev.reason))
        elif isinstance(ev, P.Cfg):
            self._live = bool(ev.v.get('v', ev.v.get('image', 0)))
            self.on_status(dict(self.status(), cfg=ev.v))
        elif isinstance(ev, P.Comment):
            if ev.text.startswith('#!') or ev.text.startswith('# '):
                self.on_event('info', 'camera: %s' % ev.text[:120])


def _load_tcview_protocol():
    """tcview.protocol from the target_cam viewer, wherever this checkout has it."""
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(os.path.dirname(here))
    viewer = os.path.join(repo, 'firmware', 'projects', 'target_cam', 'host', 'viewer')
    if not os.path.isdir(os.path.join(viewer, 'tcview')):
        raise RuntimeError('the target_cam viewer (firmware/projects/target_cam/host/viewer) is not '
                           'on this branch; the camera protocol parser lives there. Use --cam demo, '
                           'or merge the target-cam branch.')
    if viewer not in sys.path:
        sys.path.insert(0, viewer)
    from tcview import protocol
    return protocol


def make_camera(spec, *cbs):
    """spec: None, 'demo', or a serial port name."""
    if not spec:
        return CameraSource(*cbs)
    if spec.lower() == 'demo':
        return DemoCamera(*cbs)
    return SerialCamera(spec, *cbs)
