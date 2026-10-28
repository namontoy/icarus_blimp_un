"""The thrust stand (plan P0): sweep one motor's duty over the link while
the operator reads the scale, then fit the thrust map F(u) and the numbers
params.c and the step fits need.

    python software/thrust_map.py run --motor FL                    # dry run: prints the schedule
    python software/thrust_map.py run --motor FL --go               # the bench: PROPS ON, motor on the stand
    python software/thrust_map.py run --motor FL --reverse --go     # forward duties, then reverse
    python software/thrust_map.py fit captures/thrust-*.csv         # F(u) per motor and direction, Rule 13
    python software/thrust_map.py --selftest

The rig (docs/07 section 8, experiment 4): the motor and its propeller on a
pivot, or pushing straight down on a 0.1 g kitchen scale, nothing in the
propeller's disc, the real cell as the supply (the map is a function of the
cell's voltage, so the capture records bat_v beside every point). The tool
holds each duty from its own thread at 20 Hz (the firmware's motor failsafe
wants a command every second, and input() blocks), waits --settle seconds
for the motor to spin up, records --hold seconds of the capture on the blimp
(the cell voltage, the duty the driver applied, the IMU's vibration), then
asks for the scale reading in grams while the motor keeps running. The tare
(the rig's weight at zero duty) is asked before the sweep and again after
it; the thrust is (reading - tare) x --lever, the rig's arm ratio (1.0 when
the motor pushes straight on the scale). Every point goes to a CSV row, and
every plateau's capture is kept beside it.

Why forward and reverse separately: the DRV8833 drives both ways, the
propeller is pitched for one, and AGENTS.md's 74 % is one measurement. Why
per motor: four motors from one batch differ and the mixer assumes one
curve, so the fit prints the spread and the choice of motor_max_thrust_n is
a decision with a number behind it (the strongest motor for Rule 13: a duty
computed from a weaker curve over-delivers on a stronger motor).

The fit is F(u) = a u^2 + b u through the origin (a brushed motor's thrust
goes with rpm^2 and rpm about with the voltage, so with the duty) and, as the
check, the affine F = c (u - u0) above a dead band u0. The quadratic goes to
thrust-map.json, which step_response.py reads to turn a step fit into
core/'s units. Standard library only.
"""
import argparse
import csv
import json
import math
import os
import random
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blimp_link as bl  # noqa: E402
import blimp_log  # noqa: E402

MOTORS = bl.MOTOR_ORDER
DUTIES = [round(0.1 * k, 1) for k in range(1, 11)]
FIELDS = ('date', 'motor', 'direction', 'duty', 'reading_g', 'tare_g', 'lever', 'thrust_gf', 'bat_v', 'readback',
          'vib_g', 'records', 'note', 'capture')
RULE13_GF = 100.0


class MotorHolder:
    """Streams the current command at `hz` from its own thread, so a motor
    keeps running while the main thread waits on the operator."""

    def __init__(self, link, hz=20.0):
        self.link = link
        self.period = 1.0 / hz
        self.cmd = [0.0] * 4
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._t.start()
        return self

    def set(self, cmd):
        self.cmd = list(cmd)

    def _loop(self):
        while not self._stop.is_set():
            try:
                self.link.set_motors(*self.cmd)
            except OSError:
                pass
            self._stop.wait(self.period)

    def stop(self):
        """Zeros, then MOTOR_STOP three times (UDP: say it more than once)."""
        self.cmd = [0.0] * 4
        time.sleep(2 * self.period)
        self._stop.set()
        self._t.join(timeout=1.0)
        for _ in range(3):
            try:
                self.link.stop_motors()
            except OSError:
                pass
            time.sleep(0.05)


# --------------------------------------------------------------------- run

def _mean(vals):
    vals = [v for v in vals if v == v]
    return sum(vals) / len(vals) if vals else float('nan')


def plateau(cap, motor, hold, path):
    """A capture of `hold` seconds while the duty is held: the cell voltage,
    the duty the driver applied (the readback) and the IMU's vibration,
    averaged; the .bin kept at path when given."""
    cap.start(max(1, int(math.ceil(hold))))
    time.sleep(hold + 0.2)
    cap.stop()
    info, data = cap.download()
    h, records = blimp_log.decode(info['raw'], data)
    rows = blimp_log.to_units(h, records)
    if path:
        blimp_log.write_file(path, info['raw'], data)
    mags = [math.sqrt(r['acc_x_g'] ** 2 + r['acc_y_g'] ** 2 + r['acc_z_g'] ** 2) for r in rows]
    mags = [v for v in mags if v == v]
    vib = math.sqrt(sum((v - _mean(mags)) ** 2 for v in mags) / len(mags)) if len(mags) > 1 else float('nan')
    return {'bat_v': _mean([r['bat_v'] for r in rows]), 'readback': _mean([r['throttle_%s' % motor] for r in rows]),
            'vib_g': vib, 'records': len(rows), 'fw': h['fw']}


def ask(prompt):
    """One line from the operator; EOF (a pipe ran out) counts as quit."""
    try:
        return input(prompt).strip().lower()
    except EOFError:
        print()
        return 'q'


def ask_reading(prompt):
    """A float, or one of the letters r (repeat), s (skip), q (quit)."""
    while True:
        s = ask(prompt)
        if s in ('r', 's', 'q'):
            return s
        try:
            return float(s)
        except ValueError:
            print('  a number in grams, or r / s / q')


def write_rows(path, rows):
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        for r in rows:
            w.writerow({k: ('%.6g' % v if isinstance(v, float) else v) for k, v in r.items()})


def run(args):
    duties = sorted(set(args.duties))
    bad = [d for d in duties if d <= 0 or d > args.limit]
    if bad:
        sys.exit('duties must be in (0, %.2f]: %s' % (args.limit, bad))
    if args.motor not in MOTORS:
        sys.exit('motor must be one of %s' % ', '.join(MOTORS))
    mi = MOTORS.index(args.motor)
    points = [(d, 'forward') for d in duties] + ([(d, 'reverse') for d in duties] if args.reverse else [])
    os.makedirs(args.folder, exist_ok=True)
    stamp = time.strftime('%Y%m%d-%H%M%S')
    out = args.out or os.path.join(args.folder, 'thrust-%s-%s.csv' % (args.motor, stamp))
    print('thrust stand, motor %s, lever ratio %.3f, %d points: %s'
          % (args.motor, args.lever, len(points), ', '.join('%s %.2f' % (d[1][0], d[0]) for d in points)))
    print('each: settle %.1f s, capture %.1f s, read the scale, rest %.1f s; rows go to %s'
          % (args.settle, args.hold, args.rest, out))
    if not args.go:
        print('dry run. PROPS ON for the real thing: the motor on the stand, nothing in the propeller disc, '
              'the cell as the supply; then add --go')
        return 0
    link = bl.BlimpLink(args.host, args.port).start()
    if not link.wait_alive(3.0):
        link.close()
        sys.exit('no telemetry from %s:%d' % (args.host, args.port))
    tel = link.telemetry
    if any(abs(v) > 0.005 for v in tel['motor'].values()):
        link.close()
        sys.exit('the motors are already driven (%s): one source at a time' % tel['motor'])
    cap = bl.CaptureClient(link)
    have_cap = True
    try:
        info = cap.info()
        if info['state'] == 1:
            cap.stop()
        print('firmware %s, capture at %.0f ms' % (info['fw'] or '?', info['period_us'] / 1000.0))
    except TimeoutError:
        have_cap = False
        print('no capture on this firmware: bat_v, readback and vibration will be blank')
    holder = MotorHolder(link, args.rate).start()
    rows = []
    date = time.strftime('%Y-%m-%d %H:%M')
    try:
        tare = ask_reading('motor off: the scale reading in g (the rig alone): ')
        if tare in ('r', 's', 'q'):
            return 1
        rows.append({'date': date, 'motor': args.motor, 'direction': 'tare', 'duty': 0.0, 'reading_g': tare,
                     'tare_g': tare, 'lever': args.lever, 'thrust_gf': 0.0, 'bat_v': float('nan'),
                     'readback': 0.0, 'vib_g': float('nan'), 'records': 0, 'note': 'before', 'capture': ''})
        k = 0
        while k < len(points):
            d, direction = points[k]
            cmd = [0.0] * 4
            cmd[mi] = d if direction == 'forward' else -d
            holder.set(cmd)
            time.sleep(args.settle)
            note = []
            tel = link.telemetry
            if tel and tel['failsafe'] == 'TRIPPED':
                note.append('failsafe TRIPPED at the start')
            res = None
            path = ''
            if have_cap:
                path = os.path.join(args.folder, 'thrust-%s-%s-%s-u%03d.bin'
                                    % (args.motor, stamp, direction[0], int(round(100 * d))))
                try:
                    res = plateau(cap, args.motor, args.hold, path)
                except (TimeoutError, bl.CaptureError) as e:
                    note.append('capture: %s' % e)
                    path = ''
            else:
                time.sleep(args.hold)
            tel = link.telemetry
            rb = tel['motor'][args.motor] if tel else float('nan')
            if tel and abs(rb - cmd[mi]) > 0.02:
                note.append('readback %+.3f after the capture download (command %+.3f): plan T2, the motors cut'
                            % (rb, cmd[mi]))
            ans = ask_reading('%s %s %.2f (readback %+.3f%s): scale reading in g [r repeat, s skip, q quit]: '
                              % (args.motor, direction, d, rb, (', %.2f V' % res['bat_v']) if res else ''))
            holder.set([0.0] * 4)
            if ans == 'q':
                print('stopped by the operator after %d points' % len(rows))
                break
            if ans == 'r':
                time.sleep(args.rest)
                continue
            if ans == 's':
                k += 1
                time.sleep(args.rest)
                continue
            thrust = (ans - tare) * args.lever
            rows.append({'date': date, 'motor': args.motor, 'direction': direction, 'duty': d, 'reading_g': ans,
                         'tare_g': tare, 'lever': args.lever, 'thrust_gf': thrust,
                         'bat_v': res['bat_v'] if res else float('nan'),
                         'readback': res['readback'] if res else rb, 'vib_g': res['vib_g'] if res else float('nan'),
                         'records': res['records'] if res else 0, 'note': '; '.join(note),
                         'capture': os.path.basename(path)})
            print('  %s %s %.2f: %.1f gf%s' % (args.motor, direction, d, thrust, (' ' + '; '.join(note)) if note else ''))
            k += 1
            time.sleep(args.rest)
        tare2 = ask_reading('motor off again: the scale reading in g: ')
        if isinstance(tare2, float):
            rows.append({'date': date, 'motor': args.motor, 'direction': 'tare', 'duty': 0.0, 'reading_g': tare2,
                         'tare_g': tare, 'lever': args.lever, 'thrust_gf': (tare2 - tare) * args.lever,
                         'bat_v': float('nan'), 'readback': 0.0, 'vib_g': float('nan'), 'records': 0,
                         'note': 'after: drift %.1f g' % (tare2 - tare), 'capture': ''})
    finally:
        holder.stop()
        link.close()
    write_rows(out, rows)
    print('%d rows written to %s' % (len(rows), out))
    return 0


# --------------------------------------------------------------------- fit

def quad_fit(points):
    """F = a u^2 + b u through the origin, least squares. points: [(u, F)]."""
    s4 = sum(u ** 4 for u, _ in points)
    s3 = sum(u ** 3 for u, _ in points)
    s2 = sum(u ** 2 for u, _ in points)
    s2f = sum(u * u * f for u, f in points)
    s1f = sum(u * f for u, f in points)
    det = s4 * s2 - s3 * s3
    if det == 0:
        return None
    a = (s2f * s2 - s1f * s3) / det
    b = (s4 * s1f - s3 * s2f) / det
    return a, b


def affine_fit(points):
    """F = c u + d, least squares; u0 = -d / c is the dead band it implies."""
    n = len(points)
    mu = sum(u for u, _ in points) / n
    mf = sum(f for _, f in points) / n
    sxx = sum((u - mu) ** 2 for u, _ in points)
    if sxx == 0:
        return None
    c = sum((u - mu) * (f - mf) for u, f in points) / sxx
    d = mf - c * mu
    return c, d


def r_squared(points, fn):
    mf = sum(f for _, f in points) / len(points)
    ss = sum((f - mf) ** 2 for _, f in points)
    sr = sum((f - fn(u)) ** 2 for u, f in points)
    return 1.0 - sr / ss if ss > 0 else float('nan')


def read_rows(files):
    rows = []
    for path in files:
        with open(path, newline='') as f:
            for r in csv.DictReader(f):
                rows.append({k: (float(v) if k in ('duty', 'reading_g', 'tare_g', 'lever', 'thrust_gf', 'bat_v',
                                                    'readback', 'vib_g') and v not in ('', 'nan') else v)
                             for k, v in r.items()})
    return rows


def fit_rows(rows, files=()):
    """The thrust map: per motor and direction, the quadratic and the affine
    fit, F at full and at 0.3 duty, the mean cell voltage; then Rule 13."""
    groups = {}
    for r in rows:
        if r['direction'] not in ('forward', 'reverse') or r.get('thrust_gf') is None:
            continue
        groups.setdefault((r['motor'], r['direction']), []).append(r)
    motors = {}
    for (motor, direction), rs in sorted(groups.items()):
        pts = [(abs(r['duty']), abs(r['thrust_gf'])) for r in rs]
        signs = [r['thrust_gf'] >= 0 for r in rs if abs(r['thrust_gf']) > 0.5]
        q = quad_fit(pts)
        af = affine_fit(pts)
        entry = {'n': len(pts), 'points': sorted(pts), 'bat_v': _mean([r['bat_v'] for r in rs if isinstance(r['bat_v'], float)])}
        if q:
            a, b = q
            entry.update(a=a, b=b, r2=r_squared(pts, lambda u: a * u * u + b * u), f_full_gf=a + b,
                         f_030_gf=a * 0.09 + b * 0.3)
        if af:
            c, d = af
            entry.update(c=c, d=d, u0=-d / c if c else float('nan'), r2_affine=r_squared(pts, lambda u: c * u + d))
        entry['sign_mixed'] = bool(signs) and (all(signs) != any(signs))
        motors.setdefault(motor, {})[direction] = entry
    out = {'date': time.strftime('%Y-%m-%d'), 'files': [os.path.basename(f) for f in files], 'motors': motors,
           'model': 'F_gf = a u^2 + b u, u the duty 0..1; reverse separately'}
    fwd = {m: d['forward'] for m, d in motors.items() if 'forward' in d and 'a' in d['forward']}
    if fwd:
        fulls = {m: e['f_full_gf'] for m, e in fwd.items()}
        mean_full = sum(fulls.values()) / len(fulls)
        total = sum(fulls.values()) + (4 - len(fulls)) * mean_full
        ma = sum(e['a'] for e in fwd.values()) / len(fwd)
        mb = sum(e['b'] for e in fwd.values()) / len(fwd)
        # the duty at which four mean motors sum to 100 gf: 4 (a u^2 + b u) = 100
        disc = mb * mb + 4 * ma * RULE13_GF / 4.0
        u100 = (-mb + math.sqrt(disc)) / (2 * ma) if ma and disc >= 0 else (RULE13_GF / 4.0 / mb if mb else float('nan'))
        out['rule13'] = {'measured_motors': sorted(fulls), 'f_full_gf': fulls, 'sum_full_gf': total,
                         'under_cap': total <= RULE13_GF, 'duty_at_100gf': u100 if u100 <= 1.0 else None,
                         'motor_max_gf_strongest': max(fulls.values()), 'motor_max_gf_weakest': min(fulls.values()),
                         'spread_pct': 100.0 * (max(fulls.values()) - min(fulls.values())) / mean_full}
        rev = {m: d['reverse']['f_full_gf'] / fulls[m] for m, d in motors.items()
               if m in fulls and 'reverse' in d and 'a' in d['reverse']}
        if rev:
            out['reverse_ratio'] = rev
    return out


def thrust_gf(tmap, motor, u, direction='forward'):
    """F(u) in gf for one motor from a fit JSON; a motor without a curve
    gets the mean of the measured ones (and the caller should say so)."""
    u = abs(u)
    e = tmap['motors'].get(motor, {}).get(direction)
    if e is None or 'a' not in e:
        es = [d[direction] for d in tmap['motors'].values() if direction in d and 'a' in d[direction]]
        if not es:
            raise ValueError('no %s curve in the thrust map' % direction)
        a = sum(x['a'] for x in es) / len(es)
        b = sum(x['b'] for x in es) / len(es)
    else:
        a, b = e['a'], e['b']
    return a * u * u + b * u


def format_fit(m):
    lines = ['%-6s %-8s %3s %9s %9s %7s %6s %8s %7s %7s' % ('motor', 'dir', 'n', 'a (gf)', 'b (gf)', 'R^2', 'F(1)',
                                                             'F(0.3)', 'u0', 'cell V')]
    for motor, dirs in sorted(m['motors'].items()):
        for direction, e in sorted(dirs.items()):
            if 'a' not in e:
                lines.append('%-6s %-8s %3d  too few points' % (motor, direction, e['n']))
                continue
            lines.append('%-6s %-8s %3d %9.2f %9.2f %7.3f %6.1f %8.1f %7.3f %7.2f%s'
                         % (motor, direction, e['n'], e['a'], e['b'], e['r2'], e['f_full_gf'], e['f_030_gf'],
                            e.get('u0', float('nan')), e['bat_v'], '  SIGNS MIXED' if e['sign_mixed'] else ''))
    r = m.get('rule13')
    if r:
        lines.append('Rule 13: four motors forward at full duty give %.1f gf%s (%s measured%s), cap %.0f gf: %s'
                     % (r['sum_full_gf'], '' if len(r['measured_motors']) == 4 else ' (missing motors taken as the mean)',
                        ', '.join(r['measured_motors']), '' if len(r['measured_motors']) == 4 else ' of 4',
                        RULE13_GF, 'under the cap' if r['under_cap'] else 'OVER THE CAP: the mixer\'s cap must hold'))
        if r['duty_at_100gf']:
            lines.append('  the sum reaches %.0f gf at duty %.2f on every motor' % (RULE13_GF, r['duty_at_100gf']))
        lines.append('  params.c MOTOR_MAX_GF: %.1f (the strongest; the weakest is %.1f, spread %.0f %%). The Rule 13 '
                     'basis is the strongest motor; a per-motor curve in the firmware is what closes the spread'
                     % (r['motor_max_gf_strongest'], r['motor_max_gf_weakest'], r['spread_pct']))
    if m.get('reverse_ratio'):
        lines.append('  reverse / forward at full duty: %s (AGENTS.md: about 0.74)'
                     % ', '.join('%s %.2f' % kv for kv in sorted(m['reverse_ratio'].items())))
    return '\n'.join(lines)


def fit(args):
    rows = read_rows(args.files)
    if not rows:
        sys.exit('no rows in %s' % ', '.join(args.files))
    m = fit_rows(rows, args.files)
    print(format_fit(m))
    out = args.out or os.path.join(os.path.dirname(args.files[0]) or '.', 'thrust-map.json')
    with open(out, 'w') as f:
        json.dump(m, f, indent=2)
    print('written to %s' % out)
    return 0


# ---------------------------------------------------------------- selftest

def selftest():
    rnd = random.Random(2)
    truth = {'FL': (9.0, 7.0), 'RL': (8.5, 6.8), 'FR': (9.5, 7.3), 'RR': (9.2, 7.1)}     # F(1) = 16, 15.3, 16.8, 16.3 gf
    rows = []
    for motor, (a, b) in truth.items():
        for direction, scale in (('forward', 1.0), ('reverse', 0.74)):
            for u in DUTIES:
                f = scale * (a * u * u + b * u) + 0.3 * rnd.gauss(0, 1)
                rows.append({'date': 'test', 'motor': motor, 'direction': direction, 'duty': u, 'reading_g': 100 + f,
                             'tare_g': 100.0, 'lever': 1.0, 'thrust_gf': f, 'bat_v': 3.9, 'readback': u,
                             'vib_g': 0.01 * u, 'records': 150, 'note': '', 'capture': ''})
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'thrust-test.csv')
        write_rows(path, rows[:20])
        write_rows(path, rows[20:])           # append, one header
        back = read_rows([path])
        ok = len(back) == len(rows) and abs(back[3]['thrust_gf'] - rows[3]['thrust_gf']) < 1e-4
        print('csv round trip: %s' % ('ok' if ok else 'FAIL'))
        m = fit_rows(back, [path])
    print(format_fit(m))
    for motor, (a, b) in truth.items():
        e = m['motors'][motor]['forward']
        good = abs(e['f_full_gf'] - (a + b)) < 0.03 * (a + b) and e['r2'] > 0.99
        good = good and abs(m['reverse_ratio'][motor] - 0.74) < 0.05
        good = good and abs(thrust_gf(m, motor, 0.5) - (a * 0.25 + b * 0.5)) < 0.5
        ok = ok and good
        print('  %s: F(1) %.2f (true %.2f), reverse %.2f: %s' % (motor, e['f_full_gf'], a + b, m['reverse_ratio'][motor],
                                                                 'ok' if good else 'FAIL'))
    r = m['rule13']
    good = r['under_cap'] and abs(r['sum_full_gf'] - 64.4) < 2 and r['duty_at_100gf'] is None
    ok = ok and good
    print('  rule 13: %.1f gf, %s' % (r['sum_full_gf'], 'ok' if good else 'FAIL'))
    good = abs(thrust_gf(m, 'XX', 1.0) - 16.1) < 0.5     # an unmeasured motor gets the mean curve
    ok = ok and good
    print('  mean curve for a motor without one: %s' % ('ok' if good else 'FAIL'))
    print('selftest: %s' % ('ok' if ok else 'FAIL'))
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--selftest', action='store_true', help='fit synthetic readings, no hardware')
    sub = ap.add_subparsers(dest='cmd')
    r = sub.add_parser('run', help='sweep one motor on the stand; dry run without --go')
    r.add_argument('--host', default=bl.BLIMP_IP)
    r.add_argument('--port', type=int, default=bl.LINK_PORT)
    r.add_argument('--motor', required=True, help='FL, RL, FR or RR')
    r.add_argument('--duties', type=float, nargs='+', default=DUTIES, help='default 0.1 .. 1.0')
    r.add_argument('--reverse', action='store_true', help='after the forward duties, the same in reverse')
    r.add_argument('--limit', type=float, default=1.0, help='largest |duty| allowed (default %(default)s)')
    r.add_argument('--lever', type=float, default=1.0, help='thrust / scale reading, from the rig\'s arms')
    r.add_argument('--settle', type=float, default=1.5, help='s from the command to the capture')
    r.add_argument('--hold', type=float, default=3.0, help='s of capture per point, the motor running')
    r.add_argument('--rest', type=float, default=2.0, help='s off between points')
    r.add_argument('--rate', type=float, default=20.0, help='MOTOR_SET per second')
    r.add_argument('--folder', default='captures', help='where the CSV and the plateau captures go')
    r.add_argument('--out', help='the CSV (default <folder>/thrust-<motor>-<date>.csv; appends)')
    r.add_argument('--go', action='store_true', help='really drive the motor (PROPS ON, on the stand)')
    f = sub.add_parser('fit', help='fit F(u) from one or more run CSVs')
    f.add_argument('files', nargs='+')
    f.add_argument('--out', help='the JSON (default thrust-map.json beside the first CSV)')
    args = ap.parse_args(argv)
    if args.selftest:
        return 0 if selftest() else 1
    if args.cmd == 'run':
        return run(args)
    if args.cmd == 'fit':
        return fit(args)
    ap.print_help()
    return 1


if __name__ == '__main__':
    sys.exit(main())
