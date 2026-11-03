"""Camera sources for the ground station: the rows the S3 relays from the
forward camera, and the demo painter that gives them a picture.

The cameras are wired to the S3, never to this computer: each ESP32-CAM sends
the S3 what it extracts from its pictures (target_cam's CSV rows on UART1), and
the S3 is the only thing the ground station talks to, over Wi-Fi
(docs/inter-board-link-plan.md). Since 2026-11-01 the S3 relays each row as a
LINK_PKT_CAMERA packet (blimp_link.decode_camera, docs/ground-station-plan.md
§8 phase 4), plus a once-a-second health packet when no row came, and
LinkCamera turns that stream into the callbacks the window expects:

    on_row(row_dict)                 one detection row, target_cam's CSV columns
    on_frame(png_bytes, w, h, seq, t_ms, row)
    on_status(dict)                  link state and counters, see status()
    on_event(level, text)

No frame travels over the link: target_cam sends the S3 rows only, so Grab and
Live exist only in --demo, where a painter draws a frame from the row (a red
disc on the demo wall, as the old DemoCamera did; its scene now lives in
software/mock_blimp.py, which sends the rows the demo window shows). Outside
the demo they are refused with a message that says so. Frames are delivered as
PNG (the station decodes them with Qt's QImage.fromData), so the GUI never
sees a camera's pixel format.

A LinkCamera has no thread of its own. The station's receive callback calls
feed(), the station's 10 Hz tick calls poll(), and a lock covers the counters
between those two threads. Callbacks run outside the lock.

This is built 2026-11-01 and run against mock_blimp.py and in --demo; it has
not run against the blimp yet.
"""
import math
import struct
import threading
import time
import zlib

W, H = 160, 120
FOV_DEG = 54.0                      # target_cam default horizontal field of view

CONNECTED_S = 3.0                   # a CAMERA packet within this long: the S3 relay is alive
ROWS_FRESH_S = 2.0                  # a row within this long: the camera is delivering (as the panel's NO DATA)
STATUS_REFRESH_S = 1.0              # the counters are re-sent at least this often

NO_FRAMES = 'the link carries no frames: target_cam sends the S3 rows only'

TRACK_GATE_DEG = 6.0                # a blob this close to where a slot's blob was is the same balloon
TRACK_HOLD_S = 0.5                  # a slot whose blob went missing keeps it, dimmed, this long


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


class BlobTracker:
    """Keeps the camera's extra blobs (2..4) in the same numbered slot from row
    to row, for the panel only (2026-11-02).

    target_cam lists blobs 2-4 in its own order every frame, so two balloons
    of about the same size trade numbers twelve times a second and the table,
    the plot and the picture flicker. Here each blob of a new row takes the
    slot whose last position is nearest, within TRACK_GATE_DEG of az/el; a blob
    with no slot near it takes the lowest free one (or, all taken, the one
    seen longest ago); a slot whose blob is missing keeps its last values,
    marked held, for TRACK_HOLD_S, so a blob lost for a frame or two does not
    blink out. Blob 1 is never touched: it is the camera's own target, the
    one core/ would follow, and if the camera switches targets the panel
    shows it. Nothing here reaches the blimp."""
    SLOTS = (2, 3, 4)

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.slots = {}             # k -> {'az', 'el', 'r', 'c', 't'}

    def reset(self):
        self.slots.clear()

    def update(self, row):
        """The row's extra blobs in their slots: [{'k', 'az', 'el', 'r', 'c',
        'held'}] in slot order."""
        now = self.clock()
        new = []
        if row and row.get('ok'):
            for k in range(2, min(int(row.get('blobs') or 0), 4) + 1):
                az, el = row.get('b%d_az' % k), row.get('b%d_el' % k)
                if _num(az) and _num(el):
                    new.append({'az': az, 'el': el, 'r': row.get('b%d_range_mm' % k),
                                'c': row.get('b%d_conf' % k), 't': now})
        # Nearest pairs first, each slot and each blob used once
        pairs = sorted((math.hypot(b['az'] - s['az'], b['el'] - s['el']), k, j)
                       for k, s in self.slots.items() for j, b in enumerate(new))
        taken_k, taken_j = set(), set()
        for d, k, j in pairs:
            if d <= TRACK_GATE_DEG and k not in taken_k and j not in taken_j:
                self.slots[k] = new[j]
                taken_k.add(k)
                taken_j.add(j)
        for k in [k for k, s in self.slots.items() if k not in taken_k and now - s['t'] > TRACK_HOLD_S]:
            del self.slots[k]
        for j, b in enumerate(new):
            if j in taken_j:
                continue
            free = [k for k in self.SLOTS if k not in self.slots]
            k = free[0] if free else min((k for k in self.slots if k not in taken_k),
                                         key=lambda k: self.slots[k]['t'], default=None)
            if k is None:
                continue
            self.slots[k] = b
            taken_k.add(k)
        return [{'k': k, 'az': s['az'], 'el': s['el'], 'r': s['r'], 'c': s['c'], 'held': k not in taken_k}
                for k, s in sorted(self.slots.items())]


# ------------------------------------------------------------------ images

def png_rgb(w, h, rgb):
    """A PNG from packed 8-bit RGB rows, standard library only."""
    raw = b''.join(b'\x00' + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))

    def chunk(tag, data):
        body = tag + data
        return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body) & 0xFFFFFFFF)

    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


_bg_cache = []


def _background():
    """A grey wall with a slight gradient and a wood-coloured patch lower-left,
    as the viewer's fake camera draws it; built once, on the first frame."""
    if not _bg_cache:
        bg = bytearray(W * H * 3)
        for y in range(H):
            for x in range(W):
                v = int(150 + 30 * x / W - 20 * y / H)
                r, g, b = v, int(v * 0.98), int(v * 0.95)
                if x < 45 and y > 80:
                    r, g, b = 150, 95, 60
                i = 3 * (y * W + x)
                bg[i], bg[i + 1], bg[i + 2] = r, g, b
        _bg_cache.append(bytes(bg))
    return _bg_cache[0]


def paint_demo_frame(row):
    """The demo wall with a red disc where the row says the balloon is: the
    centroid cx, cy and radius max(w, h) / 2, shaded like a sphere. Nothing is
    drawn for a row without a detection."""
    buf = bytearray(_background())
    if row.get('ok'):
        cx, cy = row['cx_px'], row['cy_px']
        r = max(row['w_px'], row['h_px']) / 2.0
        r = max(r, 1.0)
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
        return False, 'no camera source'

    def status(self):
        return {'source': self.name, 'connected': False, 'live': False}


class LinkCamera(CameraSource):
    """The forward camera as the S3 relays it. feed(d) takes the dict that
    blimp_link.decode_camera returns, from the receive thread; poll() runs on
    the GUI thread at the station tick.

    A row is new when its frame_seq differs from the last one seen, so a
    repeated packet shows nothing twice, and a health-only packet (d['row']
    false) only refreshes the counters: it never reaches on_row. A frame_seq
    gap counts as skipped rows (the S3 sends at most one packet per tick, so
    rows faster than that are dropped on the blimp); a number that goes DOWN is
    the S3 rebooting, and counts nothing.

    With a painter (demo) and Live on, or a Grab pending, the new row goes to
    on_frame as a painted picture instead of to on_row; the window's frame
    carries the row, so the panel still gets it."""
    name = 'blimp'

    def __init__(self, on_frame, on_row, on_status, on_event, painter=None, clock=time.monotonic):
        super().__init__(on_frame, on_row, on_status, on_event)
        self.painter = painter
        self._clock = clock
        self._lock = threading.Lock()
        self._live = False
        self._grab = 0
        self._packets = self._rows = self._skipped = 0
        self._frames_ok = self._frames_bad = 0
        self._last_pkt = self._last_row = None      # clock times
        self._seq = None
        self._d = {}                                # the latest packet's health fields
        self._emitted = None
        self._emitted_at = None
        self._closed = False

    def start(self):
        self.on_status(self.status())
        return self

    def stop(self):
        with self._lock:
            self._closed = True

    # ---------------------------------------------------------------- input

    def feed(self, d):
        """Receive thread. One decoded CAMERA packet."""
        now = self._clock()
        paint = False
        row = None
        with self._lock:
            if self._closed:
                return
            self._packets += 1
            self._last_pkt = now
            self._d = d
            if d.get('row') and d.get('frame_seq') != self._seq:
                seq = d['frame_seq']
                if self._seq is not None and seq > self._seq + 1:
                    self._skipped += seq - self._seq - 1
                self._seq = seq
                self._rows += 1
                self._last_row = now
                row = dict(d)
                if self.painter is not None and (self._live or self._grab > 0):
                    paint = True
                    self._grab = max(0, self._grab - 1)
        if row is None:
            return
        if paint:
            try:
                png = self.painter(row)
            except Exception as e:      # a painter bug must not stop the rows
                with self._lock:
                    self._frames_bad += 1
                self.on_event('warn', 'camera: demo frame failed: %s' % e)
                self.on_row(row)
                return
            with self._lock:
                self._frames_ok += 1
            self.on_frame(png, W, H, row['frame_seq'], row['t_ms'], row)
        else:
            self.on_row(row)

    def poll(self):
        """GUI thread, every station tick: emit the status when its summary
        changed (a link going up or down, a row stream starting or stopping),
        and at least once a second so the counters move."""
        st = self.status()
        key = (st['connected'], st['uart'], st['rows_fresh'], st['live'], st['streaming_off'], st['csv_v'],
               st['bearing_sign'])
        now = self._clock()
        if (key != self._emitted or self._emitted_at is None
                or now - self._emitted_at >= STATUS_REFRESH_S):
            self._emitted, self._emitted_at = key, now
            self.on_status(st)

    # --------------------------------------------------------------- output

    def command(self, action, **kw):
        if self.painter is None and action in ('grab', 'live'):
            return False, NO_FRAMES
        if action == 'grab':
            with self._lock:
                self._grab += 1
            return True, 'frame requested'
        if action == 'live':
            with self._lock:
                self._live = bool(kw.get('on', not self._live))
                live = self._live
            return True, 'live %s' % ('on' if live else 'off')
        return False, 'unknown camera action %r' % action

    def status(self):
        now = self._clock()
        with self._lock:
            d = self._d
            return {'source': 'demo' if self.painter is not None else 'blimp',
                    'connected': self._last_pkt is not None and now - self._last_pkt < CONNECTED_S,
                    'packets': self._packets,
                    'uart': bool(d.get('uart')),
                    'rows_fresh': self._last_row is not None and now - self._last_row < ROWS_FRESH_S,
                    'frames': self.painter is not None, 'live': self._live,
                    'frames_ok': self._frames_ok, 'frames_bad': self._frames_bad,
                    'rows': self._rows, 'skipped': self._skipped,
                    'rows_s': d.get('rows_last_s', 0), 'rejected': d.get('rejected', 0),
                    'overflows': d.get('overflows', 0), 'bytes': d.get('bytes', 0),
                    'csv_v': d.get('csv_v', 0), 'streaming_off': bool(d.get('streaming_off')),
                    'bearing_sign': d.get('bearing_sign', 0), 'row_age_ms': d.get('row_age_ms')}


def make_camera(demo, *cbs):
    """In demo mode the camera paints frames from the rows the demo blimp
    sends; otherwise it shows the rows alone."""
    if demo:
        return LinkCamera(*cbs, painter=paint_demo_frame)
    return LinkCamera(*cbs)
