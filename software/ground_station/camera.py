"""Camera sources for the ground station: for now only a demo.

The cameras are wired to the S3, never to this computer: each ESP32-CAM sends
the S3 what it extracts from its pictures (detections, optical flow), and the
S3 is the only thing the ground station talks to, over Wi-Fi
(docs/inter-board-link-plan.md). The link has no packet for camera data yet,
so outside --demo the source is CameraSource, which has nothing to show. A
source fed by the S3 slots in here when that packet exists.

Each source runs its own thread and calls back with frames, rows and status:

    on_frame(png_bytes, w, h, seq, t_ms, row_or_None)
    on_row(row_dict)                 one detection row, target_cam's CSV columns
    on_status(dict)                  connection, counters
    on_event(level, text)

The frame callback is kept for the on-demand calibration grabs the
inter-board plan expects to relay through the S3; not implemented yet, and
only DemoCamera draws frames today. Frames are delivered as PNG (the station
decodes them with Qt's QImage.fromData), so the GUI never sees a camera's
pixel format.
"""
import math
import struct
import threading
import time
import zlib

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
        return False, 'no camera source: the link carries no camera data yet'

    def status(self):
        return {'source': self.name, 'connected': False, 'live': False}


class DemoCamera(CameraSource):
    """A red balloon drifting over a wall with a wood patch, at the camera's own
    12.5 frames per second: rows every tick, frames when asked. The verdict and
    the blob geometry follow the drawn disc, so the GUI's overlay lines up."""
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
        # Joined, not just signalled, so no frame callback reaches the GUI
        # after it has closed.
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def status(self):
        return {'source': self.name, 'connected': True, 'live': self._live,
                'frames_ok': self._seq, 'frames_bad': 0}

    def command(self, action, **kw):
        if action == 'grab':
            self._grab += 1
            return True, 'frame requested'
        if action == 'live':
            self._live = bool(kw.get('on', not self._live))
            self.on_status(self.status())
            return True, 'live %s' % ('on' if self._live else 'off')
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


def make_camera(demo, *cbs):
    """The demo camera in demo mode; otherwise no source, until the S3 relays
    the cameras' data over the link."""
    return DemoCamera(*cbs) if demo else CameraSource(*cbs)
