#!/usr/bin/env python3
"""The optical-flow surface tests through the S3: no motors, no envelope.

The forward camera's module runs optflow_cam pointed down (or at a wall), on
the same UART1 wires target_cam uses; the S3 logs each flow frame in its
capture beside the gyro, the ToF and the battery (ctl_types.h, log_record_t;
since 2026-11-02). One run is one capture of one surface and one action:

  still    nothing moves: texture, and the drift at rest
  move     the surface (or the gondola) slid --moved m along a tape and back
  slow     the same at about a third of the speed (the blimp's drift at 1 m)
  tilt     the gondola rocked by hand over a still surface: the false shift a
           tilt makes, against the gyro (the swing in flight, 1-3 deg/s)
  turn     the gondola turned 90 deg on the spot and back
  height   the gondola raised about 30 cm and lowered back

    python software/flow_test.py towel-blue move --dist 0.5 --moved 0.3
    python software/flow_test.py floor still --seconds 30

It checks that flow rows arrive ('cam'), zeroes the console's sum, records
the capture (the action happens while it runs; nothing beeps), downloads it
to captures/flow-<surface>-<action>-<date>.bin with a .json beside it, and
prints the analysis. --analyse FILE.bin redoes the analysis of a saved run.

Pixels become millimetres with the distance to the surface: --dist, taped,
else the ToF's median when the ToF looks at the same surface. The focal length
is 144 px across 160 px, the forward module's measured 58 deg lens (AGENTS.md;
the October board's 54 deg lens gave 157). A drift in px/s does not depend on
the distance, so it is also given "as at 1 m", the flight height.

Standard library only.
"""
import argparse
import json
import math
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blimp_link as bl  # noqa: E402
import blimp_log  # noqa: E402
import hold_test as ht  # noqa: E402

F_PX = 144.0
# A frame counts as moving above this speed. At rest the matcher's jitter is
# about 0.015 px a frame (docs/esp32-cam-optical-flow-report.md §8.1); the
# slowest motion worth measuring, 20 mm/s at 1 m, is about 0.23 px a frame.
MOVING_PX = 0.08
FLOW_BIT = 1 << blimp_log.FRESH.index('flow')

ACTIONS = {
    'still': (30, 'Keep everything still, and nobody in the view, until the end.'),
    'move': (40, 'Wait 5 s. Slide the surface (or the gondola) {moved:.0f} cm along the tape in about 5 s, '
                 'stop 3 s, slide it back to the start mark, then keep still until the end.'),
    'slow': (60, 'Wait 5 s. Slide the surface (or the gondola) {moved:.0f} cm in about 15 s, stop 3 s, '
                 'slide it back in about 15 s, then keep still until the end.'),
    'tilt': (40, 'Surface still. Wait 5 s, then rock the gondola by hand about 5 deg each way, about once a '
                 'second: 10 s nose up and down, 10 s side to side, then hold it still.'),
    'turn': (40, 'Wait 5 s. Turn the gondola 90 deg on the spot in about 5 s, wait 3 s, turn it back, '
                 'then keep still.'),
    'height': (40, 'Wait 5 s. Raise the gondola about 30 cm in about 5 s, wait 3 s, lower it back, '
                   'then keep still.'),
}


# ---------------------------------------------------------------- analysis

def frames_of(h, records):
    """One dict per flow frame (the records whose 'flow' bit is set), with the
    gyro's angle change since the previous frame for the tilt fit."""
    rows = blimp_log.to_units(h, records)
    out, prev_i = [], None
    for i, (row, rec) in enumerate(zip(rows, records)):
        if not rec['fresh'] & FLOW_BIT:
            continue
        out.append({'i': i, 't': row['t_s'], 'ok': row['flow_ok'], 'valid': row['flow_valid'],
                    'dx': row['flow_dx_px'] if row['flow_ok'] else 0.0,
                    'dy': row['flow_dy_px'] if row['flow_ok'] else 0.0,
                    'spread': row['flow_spread_px'], 'prev_i': prev_i})
        prev_i = i
    return rows, out


def gyro_bias(rows, still):
    """deg/s per axis: the median over the records of still frames, so the
    MPU-6050's offset (about 1.4 deg/s on x here) is not read as a tilt."""
    picked = [rows[i] for i in still] or rows
    return [statistics.median(r['gyro_%s_dps' % a] for r in picked) for a in 'xyz']


def gyro_turn(rows, a, b, bias):
    """Degrees turned per axis over records a+1 .. b."""
    out = [0.0, 0.0, 0.0]
    for r in rows[a + 1:b + 1]:
        dt = r['dt_ms'] / 1000.0 if math.isfinite(r['dt_ms']) else 0.02
        for k, ax in enumerate('xyz'):
            out[k] += (r['gyro_%s_dps' % ax] - bias[k]) * dt
    return out


def lstsq(X, y):
    """Least squares for a few columns, by the normal equations."""
    n = len(X[0])
    # A whisker on the diagonal: an axis that never turned (the yaw gyro
    # while the gondola rocks) gets a coefficient of 0, not a singular matrix.
    A = [[sum(r[i] * r[j] for r in X) + (1e-6 if i == j else 0.0) for j in range(n)] for i in range(n)]
    v = [sum(r[i] * t for r, t in zip(X, y)) for i in range(n)]
    for c in range(n):                       # Gauss-Jordan with partial pivoting
        p = max(range(c, n), key=lambda k: abs(A[k][c]))
        if abs(A[p][c]) < 1e-12:
            return None
        A[c], A[p], v[c], v[p] = A[p], A[c], v[p], v[c]
        for k in range(n):
            if k != c:
                m = A[k][c] / A[c][c]
                A[k] = [a - m * b for a, b in zip(A[k], A[c])]
                v[k] -= m * v[c]
    return [v[i] / A[i][i] for i in range(n)]


def tilt_fit(rows, frames, bias):
    """Fit the frame shifts to the gyro's turn over the same frame, at the lag
    that fits best. A tilt of theta shifts the image by f x theta, 2.5 px per
    degree at f = 144 px, whatever the distance; the lag is how late a row
    reaches the S3 after its frame pair (the matching, about 55 ms, and the
    wire). None when the gondola hardly turned."""
    usable = [f for f in frames if f['ok'] and f['prev_i'] is not None]
    if len(usable) < 30:
        return None
    best = None
    for lag in range(0, 9):                  # records of 20 ms: 0 .. 160 ms
        X, ydx, ydy = [], [], []
        for f in usable:
            a, b = f['prev_i'] - lag, f['i'] - lag
            if a < 0:
                continue
            X.append(gyro_turn(rows, a, b, bias))
            ydx.append(f['dx'])
            ydy.append(f['dy'])
        if len(X) < 30:
            continue
        span = max(math.hypot(x[0], x[1]) for x in X)
        cx, cy = lstsq(X, ydx), lstsq(X, ydy)
        if cx is None or cy is None:
            continue
        # Judged over the frames that turned (0.1 deg or more): a slide in
        # the same run is real motion, which no gyro explains, and frames of
        # zero turn add nothing to the coefficients either.
        turned = [(xr, dx, dy) for xr, dx, dy in zip(X, ydx, ydy) if math.hypot(*xr) >= 0.1]
        if len(turned) < 10:
            continue
        left = sum((dx - sum(c * x for c, x in zip(cx, xr))) ** 2 + (dy - sum(c * x for c, x in zip(cy, xr))) ** 2
                   for xr, dx, dy in turned)
        raw = sum(dx * dx + dy * dy for _, dx, dy in turned)
        fit = {'lag_ms': lag * 20, 'cx': cx, 'cy': cy, 'frames_turning': len(turned),
               'rms_raw_px': math.sqrt(raw / len(turned)), 'rms_left_px': math.sqrt(left / len(turned)),
               'max_turn_per_frame_deg': span,
               'rms_turn_deg': [math.sqrt(sum(x[k] ** 2 for x in X) / len(X)) for k in range(3)]}
        if best is None or fit['rms_left_px'] < best['rms_left_px']:
            best = fit
    if best is None or best['max_turn_per_frame_deg'] < 0.3:
        return None
    # px per degree, per gyro axis that was rocked: the length of that axis's
    # column of the map to the image (the map also holds the camera's
    # mounting rotation). An axis hardly turned has no column worth reading.
    top = max(best['rms_turn_deg'])
    best['px_per_deg_axis'] = {ax: math.hypot(best['cx'][k], best['cy'][k])
                               for k, ax in enumerate('xyz') if best['rms_turn_deg'][k] >= 0.3 * top}
    best['px_per_deg'] = statistics.mean(best['px_per_deg_axis'].values())
    return best


def stretches_of(frames):
    """Stretches of motion, each measured from its own start to its own end
    (the drift while still does not add in). A pause under half a second
    stays one stretch."""
    speed = [math.hypot(f['dx'], f['dy']) for f in frames]
    smooth = [statistics.mean(speed[max(0, i - 2):i + 3]) for i in range(len(frames))]
    moving = [s > MOVING_PX for s in smooth]
    pos, x, y = [], 0.0, 0.0
    for f in frames:
        x += f['dx']
        y += f['dy']
        pos.append((x, y))
    out, i = [], 0
    while i < len(frames):
        if not moving[i]:
            i += 1
            continue
        a = i
        while i < len(frames) and (moving[i] or any(moving[i:i + 6])):
            i += 1
        b = i - 1
        p0 = pos[max(0, a - 3)]
        dx, dy = pos[b][0] - p0[0], pos[b][1] - p0[1]
        if b - a >= 3 and math.hypot(dx, dy) >= 1.0:
            out.append({'a': a, 'b': b, 'px': math.hypot(dx, dy), 'dir_deg': math.degrees(math.atan2(dy, dx)),
                        's': frames[b]['t'] - frames[a]['t'], 'px_per_frame': statistics.median(speed[a:b + 1]),
                        'lost': sum(1 for f in frames[a:b + 1] if not f['ok'])})
    return out, moving, pos


def analyse(h, records, dist_m=None, moved_m=None, f_px=F_PX):
    rows, frames = frames_of(h, records)
    if len(frames) < 10:
        return {'error': 'only %d flow frames in the capture: was the camera running optflow_cam?' % len(frames)}
    res = {'frames': len(frames)}
    dur = frames[-1]['t'] - frames[0]['t']
    res['frames_per_s'] = (len(frames) - 1) / dur if dur > 0 else float('nan')
    tof = [r['tof_mm'] for r in rows if r['tof_status'] == 0 and r['tof_mm'] > 0]
    res['tof_median_m'] = statistics.median(tof) / 1000.0 if tof else None
    res['dist_m'] = dist_m if dist_m else res['tof_median_m']
    res['dist_source'] = 'taped' if dist_m else ('ToF' if res['tof_median_m'] else None)
    mm_px = res['dist_m'] * 1000.0 / f_px if res['dist_m'] else None
    res['mm_per_px'] = mm_px

    valid = sorted(f['valid'] for f in frames)
    res['measured_pct'] = 100.0 * sum(f['ok'] for f in frames) / len(frames)
    res['valid_median'] = valid[len(valid) // 2]
    res['valid_p10'] = valid[len(valid) // 10]
    res['spread_median_px'] = statistics.median(f['spread'] for f in frames if f['ok']) if any(
        f['ok'] for f in frames) else None

    st, moving, pos = stretches_of(frames)
    still = [k for k, f in enumerate(frames) if not moving[k] and f['ok']]
    bias = gyro_bias(rows, [frames[k]['i'] for k in still])
    # Drift: the summed shift over the still frames only, per second.
    if len(still) > 12:
        sx = sum(frames[k]['dx'] for k in still)
        sy = sum(frames[k]['dy'] for k in still)
        t_still = len(still) / res['frames_per_s']
        res['drift_px_s'] = math.hypot(sx, sy) / t_still
        res['drift_mm_s'] = res['drift_px_s'] * mm_px if mm_px else None
        res['drift_mm_s_at_1m'] = res['drift_px_s'] * 1000.0 / f_px
        res['jitter_px'] = statistics.median(math.hypot(frames[k]['dx'], frames[k]['dy']) for k in still)
    res['stretches'] = []
    for s in st:
        d = {'px': s['px'], 'dir_deg': s['dir_deg'], 's': s['s'], 'px_per_frame': s['px_per_frame'],
             'lost_frames': s['lost']}
        if mm_px:
            d['mm'] = s['px'] * mm_px
            if moved_m:
                d['pct_of_moved'] = 100.0 * d['mm'] / (moved_m * 1000.0)
        res['stretches'].append(d)
    end = pos[-1]
    res['net_px'] = math.hypot(*end)
    res['net_mm'] = res['net_px'] * mm_px if mm_px else None
    turned = gyro_turn(rows, 0, len(rows) - 1, bias)
    tilt_trace, acc = [], [0.0, 0.0, 0.0]
    for r in rows[1:]:
        dt = r['dt_ms'] / 1000.0 if math.isfinite(r['dt_ms']) else 0.02
        for k, ax in enumerate('xyz'):
            acc[k] += (r['gyro_%s_dps' % ax] - bias[k]) * dt
        tilt_trace.append(list(acc))
    res['gyro_range_deg'] = [max(t[k] for t in tilt_trace) - min(t[k] for t in tilt_trace) for k in range(3)] \
        if tilt_trace else None
    res['gyro_net_deg'] = turned
    res['tilt_fit'] = tilt_fit(rows, frames, bias)
    return res


def report(res, moved_m=None):
    if 'error' in res:
        print(res['error'])
        return
    print('%d flow frames, %.1f a second' % (res['frames'], res['frames_per_s']))
    if res['dist_m']:
        print('distance %.2f m (%s%s): %.2f mm per pixel'
              % (res['dist_m'], res['dist_source'],
                 '; ToF %.2f m' % res['tof_median_m'] if res['tof_median_m'] and res['dist_source'] == 'taped'
                 else '', res['mm_per_px']))
    else:
        print('no distance (no --dist and no ToF reading): shifts in pixels only')
    print('texture   measured in %.0f %% of frames; blocks matched median %d of 25 (worst 10 %%: %d); '
          'spread median %s px' % (res['measured_pct'], res['valid_median'], res['valid_p10'],
                                   '%.3f' % res['spread_median_px'] if res['spread_median_px'] is not None else '-'))
    if 'drift_px_s' in res:
        s = 'at rest   drift %.3f px/s' % res['drift_px_s']
        if res.get('drift_mm_s') is not None:
            s += ' = %.1f mm/s here' % res['drift_mm_s']
        s += ', %.1f mm/s as at 1 m; jitter %.3f px a frame' % (res['drift_mm_s_at_1m'], res['jitter_px'])
        print(s)
    for n, d in enumerate(res['stretches'], 1):
        s = 'motion %d  %.1f px' % (n, d['px'])
        if 'mm' in d:
            s += ' = %.0f mm' % d['mm']
            if 'pct_of_moved' in d:
                s += ' (%.0f %% of %.0f)' % (d['pct_of_moved'], moved_m * 1000.0)
        s += ', direction %+.0f deg in the image; %.1f s at %.2f px a frame; %d frames lost' % (
            d['dir_deg'], d['s'], d['px_per_frame'], d['lost_frames'])
        print(s)
    if not res['stretches']:
        print('motion    none')
    print('end       %.1f px%s from the start' % (res['net_px'], ' = %.0f mm' % res['net_mm'] if res['net_mm'] else ''))
    if res['gyro_range_deg']:
        print('gyro      turned through x %.1f, y %.1f, z %.1f deg (range over the run)' % tuple(res['gyro_range_deg']))
    tf = res['tilt_fit']
    if tf:
        print('tilt      %s px per deg of tilt (f = 144 px gives 2.51); rows about %d ms late; over the %d frames '
              'that turned, shift rms %.3f px, %.3f left after the gyro (%.0f %%)'
              % (', '.join('%s %.2f' % kv for kv in tf['px_per_deg_axis'].items()), tf['lag_ms'],
                 tf['frames_turning'], tf['rms_raw_px'], tf['rms_left_px'],
                 100.0 * tf['rms_left_px'] / tf['rms_raw_px'] if tf['rms_raw_px'] else 0.0))


# ---------------------------------------------------------------- the run

def run(args):
    seconds, instruction = ACTIONS[args.action]
    seconds = args.seconds or seconds
    link = bl.BlimpLink(args.host, args.port).start()
    con = bl.BlimpConsole(args.host, args.console_port)
    data = info = None
    try:
        if not link.wait_alive(3.0):
            sys.exit('no packets from the blimp: is this PC on its Wi-Fi?')
        status = ht.console(con, 'cam')
        line = next((ln for ln in status.splitlines() if ln.startswith('optical flow')), None)
        if line is None or ' 0 rows/s' in line:
            sys.exit("no optical-flow rows on the S3 ('cam' below): is the camera running optflow_cam, "
                     "powered, and wired to GPIO6?\n" + status)
        print(line)
        ht.console(con, 'cam --zero')
        cap = bl.CaptureClient(link)
        if cap.info()['state'] == 1:
            cap.stop()
        cap.start(int(math.ceil(seconds)) + 1)
        print('logging %.0f s: %s' % (seconds, instruction.format(moved=args.moved * 100.0)))
        t0 = time.monotonic()
        next_mark = 5.0
        while time.monotonic() - t0 < seconds:
            if time.monotonic() - t0 >= next_mark:
                print('  %.0f s' % next_mark, flush=True)
                next_mark += 5.0
            time.sleep(0.1)
        cap.stop()
        info, data = cap.download(progress=lambda a, n: print('\r%d / %d chunks' % (a, n), end='', flush=True))
        print()
        after = ht.console(con, 'cam')
    finally:
        con.close()
        link.close()
    if not data:
        sys.exit('no capture')

    stem = 'flow-%s-%s' % (args.surface, args.action)
    path = args.out or bl.capture_path(stem, args.folder)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    blimp_log.write_file(path, info['raw'], data)
    h, records = blimp_log.decode(info['raw'], data)
    res = analyse(h, records, args.dist, args.moved if args.action in ('move', 'slow') else None, args.f_px)
    print()
    report(res, args.moved)
    meta = {'file': os.path.basename(path), 'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'source': 'flow_test',
            'notes': args.notes, 'header': h, 'records': len(records),
            'test': {'surface': args.surface, 'action': args.action, 'seconds': seconds, 'dist_m': args.dist,
                     'moved_m': args.moved, 'f_px': args.f_px, 'instruction': instruction,
                     'cam_before': status, 'cam_after': after},
            'metrics': res}
    with open(os.path.splitext(path)[0] + '.json', 'w') as f:
        json.dump(meta, f, indent=2)
    print('\nsaved %s and its .json' % path)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('surface', nargs='?', help='a short name for the surface, e.g. towel-blue')
    ap.add_argument('action', nargs='?', choices=sorted(ACTIONS), help='what happens during the run')
    ap.add_argument('--dist', type=float, help='lens to surface, m, taped (default: the ToF median)')
    ap.add_argument('--moved', type=float, default=0.3, help='how far a move slides, m (default %(default)s)')
    ap.add_argument('--seconds', type=float, help='the capture length (default per action)')
    ap.add_argument('--f-px', type=float, default=F_PX, help='focal length at 160 px wide (default %(default)s)')
    ap.add_argument('--analyse', metavar='BIN', help='analyse a saved run instead of recording one')
    ap.add_argument('--host', default=bl.BLIMP_IP)
    ap.add_argument('--port', type=int, default=bl.LINK_PORT)
    ap.add_argument('--console-port', type=int, default=bl.CONSOLE_PORT)
    ap.add_argument('--folder', default='captures')
    ap.add_argument('--out', help='the .bin (default <folder>/flow-<surface>-<action>-<date>.bin)')
    ap.add_argument('--notes', default='')
    args = ap.parse_args(argv)
    if args.dist is not None and not 0.05 <= args.dist <= 5.0:
        sys.exit('--dist is 0.05 .. 5 m')
    if args.analyse:
        h, records = blimp_log.read_file(args.analyse)
        side = os.path.splitext(args.analyse)[0] + '.json'
        moved = args.moved
        if os.path.exists(side) and args.dist is None:
            t = json.load(open(side)).get('test', {})
            args.dist = t.get('dist_m')
            moved = t.get('moved_m', moved) if t.get('action') in ('move', 'slow') else None
        report(analyse(h, records, args.dist, moved, args.f_px), moved)
        return 0
    if not args.surface or not args.action:
        ap.error('give a surface and an action, or --analyse FILE.bin')
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
