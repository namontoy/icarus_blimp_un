"""The forward camera's rows on the ground station (2026-11-01): LinkCamera
against synthetic decode_camera dicts and an injected clock, then the real
chain, a non-demo Station and its window against mock_blimp.MockBlimp on
localhost in each of its camera modes ('rows', 'silent', 'off'), and a demo
window's Grab. Built against the mock; not run against the blimp yet."""
import threading

import harness as H
from harness import check, pump

from PyQt6.QtCore import QSettings
from PyQt6.QtWidgets import QApplication

import mock_blimp

from software.ground_station import app as A
from software.ground_station import camera as Cam
from software.ground_station import station as S
from software.ground_station import theme as T

# ------------------------------------------------------------- LinkCamera units


def packet(seq, row=True, **kw):
    """The shape blimp_link.decode_camera returns (contract section 3)."""
    d = {'t_us': 0, 'row_t_us': 0, 'frame_seq': seq, 'flags': 0, 'row': row, 'uart': True, 'streaming_off': False,
         'csv_v': 2, 'bearing_sign': -1, 'row_age_ms': 10 if row else None, 'rows_last_s': 12, 'bytes': 5000,
         'rejected': 0, 'overflows': 0, 'info_lines': 1,
         't_ms': 1000 * seq if row else 0, 'ok': 1, 'cx_px': 80.0, 'cy_px': 60.0, 'az_deg': 0.0, 'el_deg': 0.0,
         'area_px': 400, 'w_px': 22, 'h_px': 22, 'fill': 0.78, 'range_mm': 2000.0, 'clipped': 0, 'blobs': 1,
         'matched_px': 400, 'dt_ms': 80.0, 'proc_us': 20000, 'stage': 1, 'conf': -1.0, 'loop_us': 60000}
    d.update(kw)
    return d


class Rec:
    def __init__(self):
        self.rows, self.frames, self.status, self.events = [], [], [], []

    def cbs(self):
        return (lambda *a: self.frames.append(a), self.rows.append, self.status.append,
                lambda lvl, txt: self.events.append((lvl, txt)))


now = [100.0]
clock = lambda: now[0]          # noqa: E731
r = Rec()
cam = Cam.LinkCamera(*r.cbs(), clock=clock).start()
st = cam.status()
check(len(r.status) == 1 and st['source'] == 'blimp' and not st['connected'] and not st['frames'],
      'camera: starts disconnected, no frames, one status at start')
for k in ('source', 'connected', 'uart', 'rows_fresh', 'frames', 'live', 'frames_ok', 'frames_bad', 'packets',
          'rows', 'skipped', 'rows_s', 'rejected', 'overflows', 'bytes', 'csv_v', 'streaming_off', 'bearing_sign',
          'row_age_ms'):
    check(k in st, 'camera: status has %r' % k)

cam.feed(packet(1))
cam.feed(packet(1))                                 # the same row again: nothing new
check(len(r.rows) == 1 and cam.status()['rows'] == 1, 'camera: a frame_seq reaches on_row once')
check(r.rows[0]['t_ms'] == 1000 and r.rows[0]['ok'] == 1, 'camera: the row dict arrives as fed')
cam.feed(packet(2))
cam.feed(packet(5))                                 # 3 and 4 were lost on the way
check(len(r.rows) == 3 and cam.status()['skipped'] == 2, 'camera: a gap in frame_seq counts as skipped (%d)'
      % cam.status()['skipped'])
cam.feed(packet(3))                                 # the S3 rebooted: a lower number, nothing skipped
cam.feed(packet(4))
check(len(r.rows) == 5 and cam.status()['skipped'] == 2, 'camera: a reset to a lower frame_seq is not a gap')
cam.feed(packet(4, row=False, bytes=5100))
check(len(r.rows) == 5 and cam.status()['packets'] == 7, 'camera: a health-only packet never calls on_row')
check(cam.status()['bytes'] == 5100, 'camera: ... but refreshes the health counters')
check(not r.frames, 'camera: no frame without a painter')

st = cam.status()
check(st['connected'] and st['rows_fresh'] and st['uart'] and st['bearing_sign'] == -1 and st['rows_s'] == 12,
      'camera: connected, rows fresh, uart, sign and rows/s from the packet')
now[0] += 1.5
cam.feed(packet(4, row=False))
st = cam.status()
check(st['connected'] and st['rows_fresh'], 'camera: 1.5 s after a row: still fresh')
now[0] += 1.0
cam.feed(packet(4, row=False))
st = cam.status()
check(st['connected'] and not st['rows_fresh'], 'camera: 2.5 s after a row: connected on health packets, rows stale')
now[0] += 3.5
st = cam.status()
check(not st['connected'] and not st['rows_fresh'], 'camera: 3.5 s of silence: not connected')
cam.feed(packet(4, row=False, uart=False))
check(cam.status()['connected'] and not cam.status()['uart'], 'camera: a packet reconnects; uart down is reported')
cam.feed(packet(4, row=False, streaming_off=True))
check(cam.status()['streaming_off'], 'camera: streaming off is reported')

# poll(): a status when the summary changes, otherwise once a second
n = len(r.status)
cam.poll()
check(len(r.status) == n + 1, 'camera: the first poll emits a status')
cam.poll()
check(len(r.status) == n + 1, 'camera: a second poll at once emits nothing')
now[0] += 0.5
cam.feed(packet(4, row=False, streaming_off=True))
cam.poll()
check(len(r.status) == n + 1, 'camera: half a second later, same summary, still nothing')
now[0] += 0.6
cam.poll()
check(len(r.status) == n + 2, 'camera: after a second the counters are sent again')
cam.feed(packet(4, row=False, csv_v=1))
cam.poll()
check(len(r.status) == n + 3, 'camera: a changed summary (csv version) is sent at once')

# Grab and Live without a painter are refused with the reason
ok, text = cam.command('grab')
check(not ok and 'no frames' in text, 'camera: Grab refused without a painter (%s)' % text)
ok, text = cam.command('live', on=True)
check(not ok and not cam.status()['live'], 'camera: Live refused without a painter')

# with a painter
r2 = Rec()
now[0] = 500.0
cam2 = Cam.LinkCamera(*r2.cbs(), painter=lambda row: b'PNG%d' % row['frame_seq'], clock=clock).start()
check(cam2.status()['source'] == 'demo' and cam2.status()['frames'], 'camera: with a painter the source is demo')
cam2.feed(packet(1))
check(len(r2.rows) == 1 and not r2.frames, 'camera: a row goes to on_row until a frame is asked for')
ok, text = cam2.command('grab')
cam2.feed(packet(2, row=False))
check(ok and not r2.frames, 'camera: a grab waits for a row, a health packet does not serve it')
cam2.feed(packet(2))
cam2.feed(packet(3))
check(len(r2.frames) == 1 and len(r2.rows) == 2, 'camera: one grab, one frame; the next row is a row again')
png, w, h, seq, t_ms, row = r2.frames[0]
check((png, w, h, seq, t_ms) == (b'PNG2', 160, 120, 2, 2000) and row['frame_seq'] == 2,
      'camera: the frame carries png, size, frame_seq, t_ms and its row')
check(cam2.command('live', on=True)[0] and cam2.status()['live'], 'camera: Live on')
cam2.feed(packet(4))
cam2.feed(packet(5))
check(len(r2.frames) == 3 and len(r2.rows) == 2 and cam2.status()['frames_ok'] == 3,
      'camera: with Live every row is a frame (%d frames)' % len(r2.frames))
cam2.stop()
cam2.feed(packet(6))
check(len(r2.frames) == 3, 'camera: nothing is delivered after stop()')

# the demo painter
png = Cam.paint_demo_frame(packet(1))
check(png[:8] == b'\x89PNG\r\n\x1a\n', 'painter: a PNG with a disc')
png = Cam.paint_demo_frame(packet(1, ok=0, w_px=0, h_px=0))
check(png[:8] == b'\x89PNG\r\n\x1a\n', 'painter: a PNG for a row with no detection')

# ------------------------------------------------------------- the real chain

app = QApplication([])


def open_real(camera):
    """A non-demo station and window against a MockBlimp on localhost."""
    link, con = H.free_ports()
    mock = mock_blimp.MockBlimp('127.0.0.1', link, con, False, camera=camera)
    threading.Thread(target=mock.serve, daemon=True, name='mock_blimp').start()
    T.apply(app)
    station = S.Station(False, '127.0.0.1', None, ports=(link, con))
    win = A.MainWindow(station, QSettings(H.settings_path(), QSettings.Format.IniFormat))
    win.resize(1680, 1000)
    win.show()
    evs = []
    station.ev.connect(evs.append)
    station.start()
    return mock, station, win, evs


def rows_seen(win):
    return win.camera.row is not None


mock, station, win, evs = open_real('rows')
cam_pill = win.header.pills['cam']
check(H.wait_for(lambda: (pump(app, 0.05), rows_seen(win))[1], 6.0), 'rows: the mock\'s rows reach the camera panel')
pump(app, 1.0)
row = win.camera.row
check(row.get('ok') in (0, 1) and 'az_deg' in row and 'frame_seq' in row, 'rows: the panel holds a decoded row')
check(cam_pill.value == 'UP' and cam_pill.state == 'ok', 'rows: the Cam pill is UP (%s)' % cam_pill.value)
st = station.camera.status()
check(st['source'] == 'blimp' and st['rows'] >= 10 and st['uart'] and st['bearing_sign'] == -1,
      'rows: status after a second: %d rows, uart, sign -1' % st['rows'])
check(st['skipped'] <= 2, 'rows: nearly no row skipped locally (%d)' % st['skipped'])
check(not win.camera.b_grab.isEnabled() and not win.camera.b_live.isEnabled(),
      'rows: Grab and Live are disabled without a painter')
check('rows only' in win.camera.note.text(), 'rows: the note says the link carries rows (%s)' % win.camera.note.text())
check('core sign -1' in win.camera.health.text(), 'rows: the health line shows the core sign (%s)'
      % win.camera.health.text())
check(win.camera.view.img is None and win.camera.view.row is not None, 'rows: the view draws from the row, no image')
station.camera_command('grab')
check(any('no frames' in e['text'] for e in evs), 'rows: a Grab is answered with why it cannot be done')
# the mock's camera stops talking: the rows go stale, then the relay's packets stop with the mock
mock.camera = 'silent'
pump(app, 3.0)
check(cam_pill.value == 'NO ROWS' and cam_pill.state == 'warn', 'rows: when the rows stop the pill says NO ROWS (%s)'
      % cam_pill.value)
check(win.camera.v_word.text() == 'NO DATA', 'rows: the verdict goes to NO DATA')
A.teardown(station, win)

mock, station, win, evs = open_real('silent')
pump(app, 2.5)
cam_pill = win.header.pills['cam']
check(cam_pill.value == 'NO ROWS', 'silent: the Cam pill is NO ROWS (%s)' % cam_pill.value)
sub = win.camera.v_sub.text()
check(win.camera.v_word.text() == 'NO DATA' and 'hears nothing' in sub, 'silent: the verdict says why (%s)' % sub)
check(station.camera.status()['uart'] and station.camera.status()['rows'] == 0, 'silent: uart up, no rows')
check(win.camera.view.row is None and 'hears nothing' in win.camera.view.note, 'silent: the view says why too')
A.teardown(station, win)

mock, station, win, evs = open_real('off')
pump(app, 2.5)
cam_pill = win.header.pills['cam']
check(cam_pill.value == 'NONE', 'off: the Cam pill stays NONE (%s)' % cam_pill.value)
sub = win.camera.v_sub.text()
check('camera relay' in sub, 'off: the verdict says the firmware may predate the relay (%s)' % sub)
A.teardown(station, win)

# ----------------------------------------------------------- the demo window

settings = QSettings(H.settings_path(), QSettings.Format.IniFormat)
station, win = A.build(A.parse_args(['--demo']), app, settings, ports=H.free_ports())
win.resize(1680, 1000)
win.show()
station.start()
check(H.wait_for(lambda: (pump(app, 0.05), rows_seen(win))[1], 6.0), 'demo: rows reach the panel')
check(win.camera.b_grab.isEnabled() and win.camera.b_live.isEnabled(), 'demo: Grab and Live are enabled')
win.camera.b_grab.click()
check(H.wait_for(lambda: (pump(app, 0.05), win.camera.view.img is not None)[1], 4.0),
      'demo: Grab produces a frame in the view')
check(win.camera.view.meta['w'] == 160 and win.camera.view.meta['row'] is not None, 'demo: the frame carries its row')
check(not win.camera.view.img.isNull(), 'demo: the painted frame decodes')
A.teardown(station, win)

raise SystemExit(H.finish())
