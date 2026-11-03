#!/usr/bin/env python3
"""Where the gondola went over the floor, from the downward camera's optical
flow in a flight's capture (2026-11-03).

    python software/flow_track.py captures/course-20261103-151200.bin --cal captures/flow-cal-room.json \\
        --image-dirs 95 -172 --tilt captures/flow-room-tilt-20261103-143000.json --svg course-track.svg
    python software/flow_track.py --selftest

Nothing on board measures position: until now the team's report of where the
blimp landed was the only record of it (docs/flight-tests.md). The S3 logs
every frame of optflow_cam in the capture beside the gyro and the ToF
(blimp_log's flow_* columns, since 2026-11-02), and that is enough to draw
the track afterwards, in the start's frame: x along the start's nose, y to
its left, as course_test.py's --course, so the balloons go on the same plot.

Per frame (the records whose 'flow' bit is set):

1. The shift in px, image x right and y down: the matcher's median over its
   blocks. A frame it refused (ok = 0) moves the track by nothing and is
   counted: no information is not zero motion, and the share of such frames
   is how much of the track is missing. So are the rows the S3 never got
   (gaps in the row counter) and the frames without a ToF reading.
2. Minus the matcher's constant offset (--cal, from flow_test.py --calibrate).
3. Minus the tilt: a swinging gondola shifts the image with no motion over
   the floor, f x the angle (about 7.5 px a degree at f = 430). --tilt takes
   the px per degree per gyro axis that flow_test.py's tilt run fitted, and
   its lag. The yaw axis is left out: a turn about the camera's own axis
   moves the median block by nothing, and the tilt run hardly turns it, so
   its coefficient there is noise that a turn on the spot would multiply.
4. Into metres: times the ToF's distance (the camera and the ToF both look at
   the floor) over the focal length f in px (--cal, or --f-px).
5. From the image's axes into the gondola's: --image-dirs, the directions in
   which the image moved in flow_test.py's two runs that carried the gondola
   forward and to its left (its "direction of the motion in the image").
   Two directions, so a mirrored image (the S3-WROOM CAM's is) needs no flag.
   Without them the track stays in image axes, unturned.
6. From the gondola's into the start's: the heading, the gyro's yaw (the
   IMU's body z, --imu-axes as CONFIG_CTL_IMU_AXES) integrated from the first
   record, its bias the median over the records before the motors start (the
   flight scripts' 2 s of zero thrust once the estimator has converged).

The flight script's .json beside the capture gives the phases, the events
(course_test.py's 'seen' and 'close' per balloon) and the course: the report
gives the track at each, and how far from each balloon's floor point the
blimp was when it went in to touch.

Reasoned, not measured: the track's error. The scale is the calibration's
(2 % on the desk, 2026-11-02); the heading drifts with what is left of the
gyro's bias, a few degrees a minute; the camera sits off the yaw axis, so a
turn on the spot draws a small arc; on the floor (before the climb and after
the landing) the camera is too close to match and its frames are refused.
Set the end against the team's report of where it landed before trusting a
run.

Standard library only.
"""
import argparse
import json
import math
import os
import random
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blimp_log  # noqa: E402

FLOW_BIT = 1 << blimp_log.FRESH.index('flow')
TOF_MAX_AGE_S = 0.3   # a frame takes the ToF's latest reading up to this old (it runs at 50 Hz)
STILL_MIN = 25        # zero-thrust records at the start for the gyro's bias: half a second
LEAD_S = 2.0          # hold_test.LEAD_S, when a .json does not say where the hold started
BALLOON_R_M = 0.28    # Rule 21's 22-inch balloon, for the plot


def parse_axes(text):
    """CONFIG_CTL_IMU_AXES' form, '-Y +X +Z': per body axis (x, y, z), the
    gyro column it comes from and its sign."""
    out = []
    for p in text.split():
        if len(p) != 2 or p[0] not in '+-' or p[1].upper() not in 'XYZ':
            raise ValueError('%r is not an axis like +X or -Y' % p)
        out.append(('xyz'.index(p[1].lower()), 1.0 if p[0] == '+' else -1.0))
    if len(out) != 3 or sorted(i for i, _ in out) != [0, 1, 2]:
        raise ValueError('%r: three axes, each chip axis once' % text)
    return out


def image_to_body(fwd_deg, left_deg):
    """The 2x2 matrix that takes an image shift to the gondola's move (both
    in units of the distance over f): the inverse of [u_fwd u_left], the unit
    vectors of the image's motion when the gondola went forward and left."""
    a, b = math.radians(fwd_deg), math.radians(left_deg)
    m00, m01, m10, m11 = math.cos(a), math.cos(b), math.sin(a), math.sin(b)
    det = m00 * m11 - m01 * m10
    if abs(det) < 0.5:
        raise ValueError('--image-dirs %g and %g are within 30 deg of parallel: a forward and a left '
                         'move should be about 90 deg apart in the image' % (fwd_deg, left_deg))
    return ((m11 / det, -m01 / det), (-m10 / det, m00 / det))


def track(rows, f_px, to_body=None, tilt=None, offset_px_s=(0.0, 0.0), imu_axes=None):
    """The track from rows in blimp_log.to_units' form. tilt: flow_test.py's
    tilt fit ({'cx': [3], 'cy': [3], 'lag_ms'}), or None."""
    imu_axes = imu_axes or parse_axes('-Y +X +Z')
    n = len(rows)
    dts = [r['dt_ms'] / 1000.0 if math.isfinite(r['dt_ms']) else 0.02 for r in rows]
    first = next((k for k, r in enumerate(rows) if any(r['throttle_%s' % m] for m in blimp_log.MOTORS)), n)
    if first >= STILL_MIN:
        bias = [statistics.median(r['gyro_%s_dps' % a] for r in rows[:first]) for a in 'xyz']
        bias_from = 'the %d still records before the motors started (%.1f s)' % (first, first * 0.02)
    else:
        bias = [0.0, 0.0, 0.0]
        bias_from = 'none: only %d still records before the motors started, so the heading drifts' % first
    zi, zs = imu_axes[2]
    psi = [0.0] * n                     # deg, + counter-clockwise from the start's nose
    for k in range(1, n):
        psi[k] = psi[k - 1] + zs * (rows[k]['gyro_%s_dps' % 'xyz'[zi]] - bias[zi]) * dts[k]

    tof_m, last, last_t = [None] * n, None, None
    for k, r in enumerate(rows):
        if r['tof_status'] == 0 and r['tof_mm'] > 0:
            last, last_t = r['tof_mm'] / 1000.0, r['t_s']
        if last is not None and r['t_s'] - last_t <= TOF_MAX_AGE_S:
            tof_m[k] = last

    lag = int(round(tilt['lag_ms'] / 20.0)) if tilt else 0
    tilt_axes = [k for k in range(3) if k != zi]
    c = {'frames': 0, 'used': 0, 'refused': 0, 'no_height': 0, 'lost_rows': 0}
    x = y = path = 0.0
    pts = [{'t': rows[0]['t_s'] if rows else 0.0, 'x': 0.0, 'y': 0.0, 'hdg': 0.0, 'h': None}]
    prev_i = prev_seq = None
    for i, r in enumerate(rows):
        if not r['fresh'] & FLOW_BIT:
            continue
        c['frames'] += 1
        if prev_seq is not None:
            c['lost_rows'] += (r['flow_seq'] - prev_seq - 1) % 256
        prev_seq = r['flow_seq']
        p, prev_i = prev_i, i
        if p is None:
            continue                    # its shift is from a frame before the capture
        if not r['flow_ok']:
            c['refused'] += 1
            continue
        h = tof_m[i]
        if h is None:
            c['no_height'] += 1
            continue
        dt = r['t_s'] - rows[p]['t_s']
        dx = r['flow_dx_px'] - offset_px_s[0] * dt
        dy = r['flow_dy_px'] - offset_px_s[1] * dt
        # The frame pair happened lag records before its row reached the S3.
        a, b = max(0, p - lag), max(0, i - lag)
        if tilt:
            turn = [0.0, 0.0, 0.0]
            for k in range(a + 1, b + 1):
                for ax in tilt_axes:
                    turn[ax] += (rows[k]['gyro_%s_dps' % 'xyz'[ax]] - bias[ax]) * dts[k]
            dx -= sum(tilt['cx'][ax] * turn[ax] for ax in tilt_axes)
            dy -= sum(tilt['cy'][ax] * turn[ax] for ax in tilt_axes)
        s = h / f_px
        if to_body:
            bx = s * (to_body[0][0] * dx + to_body[0][1] * dy)
            by = s * (to_body[1][0] * dx + to_body[1][1] * dy)
            hd = math.radians(0.5 * (psi[a] + psi[b]))
            ex, ey = math.cos(hd) * bx - math.sin(hd) * by, math.sin(hd) * bx + math.cos(hd) * by
        else:
            ex, ey = s * dx, s * dy
        x += ex
        y += ey
        path += math.hypot(ex, ey)
        c['used'] += 1
        pts.append({'t': r['t_s'], 'x': x, 'y': y, 'hdg': psi[i], 'h': h})
    far = max(pts, key=lambda q: math.hypot(q['x'], q['y']))
    return {'points': pts, 'counts': c, 'bias_dps': bias, 'bias_from': bias_from, 'lag_ms': lag * 20,
            'heading_end_deg': psi[-1] if n else 0.0, 'path_m': path, 'end': (x, y),
            'farthest': (far['x'], far['y'], far['t']), 'f_px': f_px, 'tilt': bool(tilt),
            'frame': 'start' if to_body else 'image'}


def at(pts, t):
    """The track's point at time t (the last one at or before it)."""
    best = pts[0]
    for q in pts:
        if q['t'] > t:
            break
        best = q
    return best


def marks_of(meta):
    """(phases, events, course, offset) from a flight script's .json."""
    t = (meta or {}).get('test', {})
    off = t.get('hold_start_in_capture_s', LEAD_S)
    return t.get('phases', []), t.get('events', []), t.get('course_m') or [], off


def report(res, meta=None):
    c = res['counts']
    print('%d flow frames: %d used, %d refused by the matcher, %d without a ToF reading, %d rows the S3 '
          'never got' % (c['frames'], c['used'], c['refused'], c['no_height'], c['lost_rows']))
    if c['frames'] and c['used'] < 0.8 * c['frames']:
        print('  only %.0f %% of the frames moved the track: it is short by what the others missed '
              '(on the floor, before the climb and after the landing, they are refused anyway)'
              % (100.0 * c['used'] / c['frames']))
    print('f = %.0f px%s; heading from the gyro, bias from %s; %+.0f deg at the end'
          % (res['f_px'], ', the tilt taken out (lag %d ms)' % res['lag_ms'] if res['tilt'] else ', no tilt removed',
             res['bias_from'], res['heading_end_deg']))
    if res['frame'] == 'image':
        print('NO --image-dirs: the track is in the image\'s axes (x right, y down), not the room\'s')
        where = 'image axes'
    else:
        where = "the start's frame (x along its nose, y to its left)"
    x, y = res['end']
    print('track in %s: ends at x %+.2f, y %+.2f m, %.2f m from the start; farthest %.2f m at %.0f s; '
          'path %.1f m' % (where, x, y, math.hypot(x, y), math.hypot(*res['farthest'][:2]), res['farthest'][2],
                           res['path_m']))
    phases, events, course, off = marks_of(meta)
    pts = res['points']
    if phases:
        print('  %-34s %16s %16s %8s' % ('phase', 'from x, y m', 'to x, y m', 'moved'))
        for ph in phases:
            t0, t1, label = ph[0], ph[1], ph[2]
            a, b = at(pts, t0 + off), at(pts, t1 + off)
            print('  %-34s %+6.2f, %+6.2f   %+6.2f, %+6.2f   %5.2f m'
                  % (label, a['x'], a['y'], b['x'], b['y'], math.hypot(b['x'] - a['x'], b['y'] - a['y'])))
    for e in events:
        q = at(pts, e['t'] + off)
        line = '  %5.1f s  %-6s balloon %s at x %+.2f, y %+.2f m' % (e['t'], e['event'], e.get('balloon', '?'),
                                                                   q['x'], q['y'])
        k = e.get('balloon')
        if res['frame'] == 'start' and isinstance(k, int) and 1 <= k <= len(course):
            bx, by = course[k - 1]
            line += ', %.2f m from its floor point' % math.hypot(bx - q['x'], by - q['y'])
        print(line)


def write_csv(res, path):
    with open(path, 'w') as f:
        f.write('t_s,x_m,y_m,heading_deg,height_m\n')
        for q in res['points']:
            f.write('%.3f,%.4f,%.4f,%.2f,%s\n' % (q['t'], q['x'], q['y'], q['hdg'],
                                                  '%.3f' % q['h'] if q['h'] is not None else ''))


def write_svg(res, meta, path, title=''):
    """The track over the course, seen from above: the start's nose up, its
    left to the left, a 1 m grid."""
    phases, events, course, off = marks_of(meta)
    pts = res['points']
    if res['frame'] != 'start':
        course = []
    xs = [q['x'] for q in pts] + [b[0] for b in course] + [0.0]
    ys = [q['y'] for q in pts] + [b[1] for b in course] + [0.0]
    x0, x1 = math.floor(min(xs) - 0.5), math.ceil(max(xs) + 0.5)
    y0, y1 = math.floor(min(ys) - 0.5), math.ceil(max(ys) + 0.5)
    k = min(120.0, 800.0 / max(x1 - x0, y1 - y0))
    pad = 40.0
    w, hgt = (y1 - y0) * k + 2 * pad, (x1 - x0) * k + 2 * pad + 30

    def u(yv):
        return pad + (y1 - yv) * k

    def v(xv):
        return pad + 30 + (x1 - xv) * k

    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%.0f" height="%.0f" font-family="sans-serif" '
           'font-size="13">' % (w, hgt),
           '<rect width="100%" height="100%" fill="#ffffff"/>',
           '<text x="%.0f" y="22" fill="#222">%s</text>' % (pad, title or 'optical-flow track')]
    for gx in range(int(x0), int(x1) + 1):
        out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#e2e2e2"/>' % (u(y1), v(gx), u(y0), v(gx)))
        out.append('<text x="%.1f" y="%.1f" fill="#999" font-size="11">x %d m</text>' % (u(y0) + 4, v(gx) - 3, gx))
    for gy in range(int(y0), int(y1) + 1):
        out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#e2e2e2"/>' % (u(gy), v(x1), u(gy), v(x0)))
    for n, (bx, by) in enumerate(course, 1):
        out.append('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="#f3c0c0" stroke="#c0392b"/>'
                   % (u(by), v(bx), BALLOON_R_M * k))
        out.append('<text x="%.1f" y="%.1f" fill="#c0392b">B%d</text>' % (u(by) + BALLOON_R_M * k + 4, v(bx) + 4, n))
    out.append('<polyline fill="none" stroke="#1f5fa8" stroke-width="2" points="%s"/>'
               % ' '.join('%.1f,%.1f' % (u(q['y']), v(q['x'])) for q in pts))
    out.append('<rect x="%.1f" y="%.1f" width="10" height="10" fill="#222"/>' % (u(0) - 5, v(0) - 5))
    out.append('<text x="%.1f" y="%.1f" fill="#222">S</text>' % (u(0) + 8, v(0) + 4))
    for e in events:
        q = at(pts, e['t'] + off)
        out.append('<circle cx="%.1f" cy="%.1f" r="4" fill="#e67e22"/>' % (u(q['y']), v(q['x'])))
        out.append('<text x="%.1f" y="%.1f" fill="#a04f00" font-size="11">%s %s</text>'
                   % (u(q['y']) + 6, v(q['x']) - 6, e['event'], e.get('balloon', '')))
    ex, ey = res['end']
    out.append('<circle cx="%.1f" cy="%.1f" r="6" fill="none" stroke="#222" stroke-width="2"/>' % (u(ey), v(ex)))
    out.append('<text x="%.1f" y="%.1f" fill="#222">end</text>' % (u(ey) + 9, v(ex) + 4))
    if res['frame'] != 'start':
        out.append('<text x="%.0f" y="%.0f" fill="#c0392b">image axes: no --image-dirs</text>' % (pad, hgt - 8))
    out.append('</svg>')
    with open(path, 'w') as f:
        f.write('\n'.join(out) + '\n')


# ---------------------------------------------------------------- selftest

def _flight(fwd_deg=100.0, left_deg=10.0, f_px=430.0, lag=2, swing_deg=3.0, refuse=(), drop=(),
            bias=(1.4, -0.3, 0.6), still_s=2.0, seed=1):
    """Rows of a made-up flight, 50 records a second: still on the floor,
    a 5 s climb to 1.3 m, 2 m forward in 10 s, a turn of -90 deg on the spot
    in 6 s, 2 m forward in 10 s, 3 s of hover; a pendulum swing on the gyro's
    x and y throughout the flight; flow frames every 80 ms, lag records late.
    The truth is (2, -2) at the end. refuse: (t0, t1) spans whose frames the
    matcher refuses in flight; drop: (t0, t1) spans whose rows never arrive."""
    rnd = random.Random(seed)
    tilt_true = ([0.0, 7.5, 0.0], [7.5, 0.0, 0.0])      # px per deg per gyro axis
    ua = (math.cos(math.radians(fwd_deg)), math.sin(math.radians(fwd_deg)))
    ul = (math.cos(math.radians(left_deg)), math.sin(math.radians(left_deg)))
    legs = [(still_s, 0, 0, 0), (5.0, 0, 0, 0), (10.0, 0.2, 0, 0), (6.0, 0, 0, -15.0), (10.0, 0.2, 0, 0),
            (3.0, 0, 0, 0)]
    state = []                                # per record: (t, vx_b, vy_b, yaw rate, swing x, swing y, flying)
    t = 0.0
    for k, (dur, vx, vy, r) in enumerate(legs):
        for _ in range(int(round(dur / 0.02))):
            t += 0.02
            fly = k > 0
            sx = swing_deg * 2 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * t) if fly else 0.0
            sy = swing_deg * 2 * math.pi * 0.4 * math.cos(2 * math.pi * 0.4 * t + 1.0) if fly else 0.0
            state.append((t, vx, vy, r, sx, sy, fly))
    rows, seq = [], 0
    for i, (t, vx, vy, r, sx, sy, fly) in enumerate(state):
        row = {'t_s': t - state[0][0], 'dt_ms': 20.0 if i else float('nan'), 'fresh': 0,
               'gyro_x_dps': sx + bias[0] + rnd.gauss(0, 0.05), 'gyro_y_dps': sy + bias[1] + rnd.gauss(0, 0.05),
               'gyro_z_dps': r + bias[2] + rnd.gauss(0, 0.05),
               'tof_mm': 1300 if fly else 50, 'tof_status': 0,
               'flow_seq': 0, 'flow_ok': 0, 'flow_dx_px': 0.0, 'flow_dy_px': 0.0}
        for m in blimp_log.MOTORS:
            row['throttle_%s' % m] = 0.3 if fly else 0.0
        rows.append(row)
    for i in range(4, len(rows), 4):
        p = i - 4
        t = rows[i]['t_s']
        seq = (seq + 1) % 256
        if any(a <= t < b for a, b in drop):
            continue
        rows[i]['fresh'] = FLOW_BIT
        rows[i]['flow_seq'] = seq
        a, b = max(0, p - lag), max(0, i - lag)
        win = state[a + 1:b + 1]
        if not all(s[6] for s in win) or any(a0 <= t < b0 for a0, b0 in refuse):
            continue                          # on the floor: too close to match; or refused
        dxb = sum(s[1] for s in win) * 0.02
        dyb = sum(s[2] for s in win) * 0.02
        tx = sum(s[4] for s in win) * 0.02
        ty = sum(s[5] for s in win) * 0.02
        kf = f_px / 1.3
        rows[i]['flow_ok'] = 1
        rows[i]['flow_dx_px'] = kf * (dxb * ua[0] + dyb * ul[0]) + tilt_true[0][0] * tx + tilt_true[0][1] * ty
        rows[i]['flow_dy_px'] = kf * (dxb * ua[1] + dyb * ul[1]) + tilt_true[1][0] * tx + tilt_true[1][1] * ty
    return rows, {'cx': [0.0, 7.5, 0.4], 'cy': [7.5, 0.0, -0.3], 'lag_ms': lag * 20}


def selftest():
    fails = []

    def check(ok, what):
        print('  %-66s %s' % (what, 'ok' if ok else 'FAIL'))
        if not ok:
            fails.append(what)

    def err(res, x=2.0, y=-2.0):
        return math.hypot(res['end'][0] - x, res['end'][1] - y)

    def worst_side(res):
        """The largest distance from the true path's first leg, x 0..2 at y 0."""
        return max(abs(q['y']) for q in res['points'] if 0.1 < q['x'] < 1.9 and q['t'] < 18.5)

    rows, tilt = _flight()
    tb = image_to_body(100.0, 10.0)
    full = track(rows, 430.0, tb, tilt)
    check(err(full) < 0.05, 'the course, tilt out: ends %.3f m from (2, -2)' % err(full))
    floor = full['counts']['refused']
    check(20 <= floor <= 30 and full['counts']['used'] > 400 and full['frame'] == 'start',
          "the floor's 2 s: %d frames refused, moved nothing; %d used" % (floor, full['counts']['used']))
    check(abs(full['heading_end_deg'] + 90.0) < 2.0, 'the heading: %.1f deg at the end (-90)' % full['heading_end_deg'])
    check(full['bias_from'].startswith('the 100 still'), 'the gyro bias from the still records')
    raw = track(rows, 430.0, tb, None)
    check(worst_side(full) < 0.02 and worst_side(raw) > 2 * worst_side(full),
          'the swing: %.3f m off the line with the tilt out, %.3f without' % (worst_side(full), worst_side(raw)))
    rows_m, tilt_m = _flight(fwd_deg=-30.0, left_deg=-120.0)
    mir = track(rows_m, 430.0, image_to_body(-30.0, -120.0), tilt_m)
    check(err(mir) < 0.05, 'a mirrored image (left clockwise of forward): %.3f m' % err(mir))
    img = track(rows, 430.0, None, tilt)
    check(img['frame'] == 'image' and abs(img['path_m'] - 4.0) < 0.15,
          'no --image-dirs: image axes, path %.2f m of 4' % img['path_m'])
    rows_r, tilt_r = _flight(refuse=[(9.0, 10.0)])
    ref = track(rows_r, 430.0, tb, tilt_r)
    extra = ref['counts']['refused'] - floor
    check(12 <= extra <= 13 and abs(ref['end'][0] - 1.8) < 0.05,
          '1 s refused in the first leg: %d more counted, ends x %.2f (1.8)' % (extra, ref['end'][0]))
    rows_d, tilt_d = _flight(drop=[(20.0, 20.5)])
    drp = track(rows_d, 430.0, tb, tilt_d)
    check(drp['counts']['lost_rows'] >= 5, '0.5 s of rows lost: %d counted' % drp['counts']['lost_rows'])
    rows_s, tilt_s = _flight(still_s=0.2)
    nob = track(rows_s, 430.0, tb, tilt_s)
    check(nob['bias_from'].startswith('none'), 'no still records: no bias, and it says so')
    try:
        image_to_body(10.0, 20.0)
        check(False, 'near-parallel --image-dirs refused')
    except ValueError:
        check(True, 'near-parallel --image-dirs refused')
    try:
        parse_axes('-Y +Y +Z')
        check(False, "'-Y +Y +Z' refused")
    except ValueError:
        check(True, "'-Y +Y +Z' refused")
    check(parse_axes('-Y +X +Z') == [(1, -1.0), (0, 1.0), (2, 1.0)], 'CONFIG_CTL_IMU_AXES read')
    print('selftest: %s' % ('ok' if not fails else '%d FAILED' % len(fails)))
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('capture', nargs='?', help="a flight's .bin (its .json beside it gives the phases and events)")
    ap.add_argument('--cal', metavar='JSON', help="flow_test.py --calibrate's file: f and the matcher's offset")
    ap.add_argument('--f-px', type=float,
                    help='the focal length in px instead of --cal (unmeasured guess for the S3-WROOM CAM at HVGA, '
                         'a 58 deg lens: 430)')
    ap.add_argument('--image-dirs', type=float, nargs=2, metavar=('FWD', 'LEFT'),
                    help="deg: flow_test.py's direction of the image's motion when the gondola went forward, "
                         "and left")
    ap.add_argument('--tilt', metavar='JSON', help="a flow_test.py tilt run's .json: the tilt taken out")
    ap.add_argument('--imu-axes', default='-Y +X +Z', help='CONFIG_CTL_IMU_AXES (default %(default)s)')
    ap.add_argument('--csv', help='write the track, one row per frame used')
    ap.add_argument('--svg', help='draw the track over the course')
    ap.add_argument('--selftest', action='store_true', help='the track of a made-up flight, no hardware')
    args = ap.parse_args(argv)
    if args.selftest:
        return selftest()
    if not args.capture:
        ap.error('give a capture, or --selftest')
    cal = json.load(open(args.cal)) if args.cal else None
    f_px = args.f_px or (cal['f_px'] if cal else None)
    if not f_px:
        sys.exit('the focal length: --cal from flow_test.py --calibrate, or --f-px')
    tilt = None
    if args.tilt:
        tilt = json.load(open(args.tilt)).get('metrics', {}).get('tilt_fit')
        if not tilt:
            sys.exit('%s has no tilt fit: is it a flow_test.py tilt run where the gondola rocked?' % args.tilt)
    try:
        to_body = image_to_body(*args.image_dirs) if args.image_dirs else None
        axes = parse_axes(args.imu_axes)
    except ValueError as e:
        sys.exit(str(e))
    h, records = blimp_log.read_file(args.capture)
    rows = blimp_log.to_units(h, records)
    side = os.path.splitext(args.capture)[0] + '.json'
    meta = json.load(open(side)) if os.path.exists(side) else None
    res = track(rows, f_px, to_body, tilt, tuple(cal['bias_px_s']) if cal else (0.0, 0.0), axes)
    if not res['counts']['frames']:
        sys.exit('no flow frames in %s: was the downward camera running optflow_cam and wired to GPIO18, '
                 'with the S3 at 3fe7911 or later?' % args.capture)
    print(os.path.basename(args.capture) + (' (%s)' % meta['notes'] if meta and meta.get('notes') else ''))
    report(res, meta)
    if args.csv:
        write_csv(res, args.csv)
        print('track written to %s' % args.csv)
    if args.svg:
        write_svg(res, meta, args.svg, os.path.basename(args.capture))
        print('plot written to %s' % args.svg)
    return 0


if __name__ == '__main__':
    sys.exit(main())
