"""Open-loop step tests on the vehicle (plan Y1, Y2, H1, H2 and L1): the
schedule streamed from this computer, the capture on the blimp, every step
fitted, all of it saved beside the capture.

    python software/step_test.py yaw                                     # dry run: prints the schedule
    python software/step_test.py yaw --steps 0.25 0.40 0.55 --mirror --go
    python software/step_test.py heave --steps 0.40 0.55 0.70 --hold 3 --gap 8 --go
    python software/step_test.py heave --steps -0.40 -0.55 -0.70 --go    # if the negative command is what pushes up
    python software/step_test.py yaw --go --thrust-map captures/thrust-map.json --half-span 0.10
    python software/step_test.py --host 127.0.0.1 yaw --go               # against mock_blimp.py

  yaw     the front differential: FL at the amplitude with FR at 0, and with
          --mirror the same on FR afterwards (the symmetry check of Y1)
  heave   the rear pair: RL = RR = amplitude

An amplitude is signed: a negative one drives the same motors the other
way, for a vehicle whose propellers push up on a negative command (the
scale said so for the rear pair on 2026-10-28, docs/history/pid-bench-
2026-10-28). The fit does not care about the sign; core/'s frame does, and
the fit says when they disagree. |amplitude| is capped by --limit, 0.75 by
default: the measured thrust is small (a few gf at 0.3 duty), so the
amplitudes are larger than blimp_link.py's 0.3 and still under the cap.

The schedule is --lead seconds of zeros for the baseline, then for every
amplitude a step held --hold seconds and --gap seconds of zeros for the
decay (Y2's spin-down, H2's descent), then the mirror. The command at each
tick comes from the schedule's clock, so a late tick never shifts a step;
the steps are square (no ramp: the Control page ramps, which is right for a
hand and wrong for identification). The capture is started before the lead
and covers the schedule; then the motors are stopped, the capture
downloaded and saved as captures/step-<axis>-<date>.bin with a .json beside
it (the schedule, telemetry before and after, the firmware, the fits), and
every step is fitted with step_response.py, in core/ units when a thrust
map is given.

Protocol reminders (docs/pid-tuning-plan.md section 4): yaw on a thread
from the top of the envelope, free to turn; heave trimmed and free with the
range sensor between 0.4 and 1.8 m, let the vehicle sink steadily before
each step (the fit removes a straight-line baseline, so a steady sink is
the equilibrium it needs, and the sink rate it reports is H2's number).

Safety: props on, so without --go this is a dry run; --limit caps the
amplitude at 0.30 as blimp_link.py does; the stream ends with zeros and
MOTOR_STOP three times on Ctrl-C, on any error and at the end; nothing runs
without telemetry, or while another source holds the motors. Standard
library only.
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blimp_link as bl  # noqa: E402
import blimp_log  # noqa: E402
import step_response as sr  # noqa: E402

ZERO = [0.0, 0.0, 0.0, 0.0]


def schedule(axis, steps, lead, hold, gap, mirror):
    """[(t0, t1, [FL, RL, FR, RR], label)], zeros between the steps."""
    segs = []
    t = [0.0]

    def seg(dur, cmd, label):
        segs.append((t[0], t[0] + dur, list(cmd), label))
        t[0] += dur

    seg(lead, ZERO, 'baseline')
    sides = (['FL'] + (['FR'] if mirror else [])) if axis == 'yaw' else ['RL+RR']
    for side in sides:
        for a in steps:
            if axis == 'yaw':
                cmd = [a, 0.0, 0.0, 0.0] if side == 'FL' else [0.0, 0.0, a, 0.0]
            else:
                cmd = [0.0, a, 0.0, a]
            seg(hold, cmd, '%s %+.2f' % (side, a))
            seg(gap, ZERO, 'zero')
    return segs


def command_at(segs, el):
    for s in segs:
        if s[0] <= el < s[1]:
            return s
    return None


def format_schedule(segs):
    return '\n'.join('  %6.2f .. %6.2f s  %-10s FL %+.2f RL %+.2f FR %+.2f RR %+.2f'
                     % (s[0], s[1], s[3], *s[2]) for s in segs)


def stream(link, segs, rate, log):
    """MOTOR_SET at `rate` from the schedule's clock until it ends; zeros and
    MOTOR_STOP whatever happens."""
    period = 1.0 / rate
    t0 = time.monotonic()
    k = 0
    last = None
    try:
        while True:
            el = time.monotonic() - t0
            s = command_at(segs, el)
            if s is None:
                break
            link.set_motors(*s[2])
            if s[3] != last:
                tel = link.telemetry
                log('%6.2f s  %-10s %s' % (el, s[3], bl.format_telemetry(tel) if tel else ''))
                last = s[3]
            k += 1
            time.sleep(max(0.0, t0 + k * period - time.monotonic()))
    finally:
        for _ in range(3):
            link.set_motors(*ZERO)
            time.sleep(0.05)
        for _ in range(3):
            link.stop_motors()
            time.sleep(0.05)


def read_params(host, console_port):
    """The controller's parameters as the test starts, for the .json
    (blimp_link.parse_params()), or {'error': why}: an open-loop step needs
    none, but a capture that names its gains can be compared with a later
    closed-loop one."""
    con = bl.BlimpConsole(host, console_port)
    try:
        return bl.read_params(con)
    except (TimeoutError, ConnectionError, ValueError, OSError) as e:
        return {'error': str(e)}
    finally:
        con.close()


def run(args):
    if args.axis == 'heave' and args.mirror:
        sys.exit('--mirror is for yaw (FL then FR); heave drives both rear motors')
    bad = [a for a in args.steps if a == 0 or abs(a) > args.limit]
    if bad:
        sys.exit('|amplitude| must be in (0, %.2f]: %s; raise --limit deliberately' % (args.limit, bad))
    segs = schedule(args.axis, args.steps, args.lead, args.hold, args.gap, args.mirror)
    total = segs[-1][1]
    cap_s = int(math.ceil(total)) + 2
    print('%s step test, %d steps, %.1f s, capture of %d s:' % (args.axis, len(args.steps) * (2 if args.mirror else 1),
                                                                  total, cap_s))
    print(format_schedule(segs))
    if not args.go:
        print('dry run. PROPS ON for the real thing: the vehicle on its tether (yaw) or trimmed and free (heave), '
              'STOP within reach; then add --go')
        return 0
    tmap = None
    if args.thrust_map:
        with open(args.thrust_map) as f:
            tmap = json.load(f)
    link = bl.BlimpLink(args.host, args.port).start()
    try:
        if not link.wait_alive(3.0):
            sys.exit('no telemetry from %s:%d' % (args.host, args.port))
        tel0 = link.telemetry
        if any(abs(v) > 0.005 for v in tel0['motor'].values()):
            sys.exit('the motors are already driven (%s): one source at a time' % tel0['motor'])
        if tel0['failsafe'] == 'TRIPPED':
            print('note: the motor failsafe is TRIPPED; the first MOTOR_SET clears it')
        cap = bl.CaptureClient(link)
        try:
            info = cap.info()
        except TimeoutError as e:
            sys.exit(str(e))
        if info['state'] == 1:
            cap.stop()
        print('firmware %s, capture at %.0f ms, up to %d s' % (info['fw'] or '?', info['period_us'] / 1000.0,
                                                                info['max_seconds']))
        params = read_params(args.host, args.console_port)
        if 'error' in params:
            print('note: the parameters are not recorded (%s)' % params['error'])
        try:
            cap.start(cap_s)
        except (TimeoutError, bl.CaptureError) as e:
            sys.exit('capture: %s' % e)
        time.sleep(0.3)                    # the first records before the first command
        lines = []

        def log(s):
            print(s)
            lines.append(s)

        stream(link, segs, args.rate, log)
        time.sleep(0.5)
        tel1 = link.telemetry
        try:
            cap.stop()
            info, data = cap.download(progress=lambda k, n: print('\r%d / %d chunks' % (k, n), end='', flush=True))
            print()
        except (TimeoutError, bl.CaptureError) as e:
            sys.exit('download: %s (the capture may still be on the blimp: blimp_link.py capture download)' % e)
    finally:
        link.close()
    if not data:
        sys.exit('the capture is empty')
    path = args.out or bl.capture_path('step-%s' % args.axis, args.folder)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    blimp_log.write_file(path, info['raw'], data)
    h, records = blimp_log.decode(info['raw'], data)
    summ = blimp_log.summary(h, records)
    print('%d records (%.1f s, %d gaps) saved to %s' % (len(records), summ['duration_s'], summ['gaps'], path))
    meta = {'file': os.path.basename(path), 'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'source': 'step_test',
            'notes': args.notes, 'header': h, 'records': len(records), 'duration_s': summ['duration_s'],
            'dt_ms': summ['dt_ms'], 'gaps': summ['gaps'], 'exec_us': summ['exec_us'], 'fresh_hz': summ['fresh_hz'],
            'flags_seen': summ['flags_seen'], 'params': params,
            'test': {'axis': args.axis, 'steps': args.steps, 'lead_s': args.lead, 'hold_s': args.hold,
                     'gap_s': args.gap, 'mirror': args.mirror, 'rate_hz': args.rate, 'limit': args.limit,
                     'host': args.host, 'schedule': [{'t0': s[0], 't1': s[1], 'cmd': s[2], 'label': s[3]} for s in segs],
                     'telemetry_before': tel0, 'telemetry_after': tel1, 'log': lines,
                     'tau_c': args.tau_c, 'thrust_map': args.thrust_map, 'gf_per_unit': args.gf_per_unit,
                     'half_span_m': args.half_span, 'wn': args.wn, 'zeta': args.zeta}}
    if not args.no_fit:
        rows = blimp_log.to_units(h, records)
        results = sr.analyse_all(rows, args.axis, args.tau_c)
        fits, text = [], []
        for k, r in enumerate(results, 1):
            n_per_unit = sr.thrust_per_unit(args.axis, r['du'], args.gf_per_unit, tmap)
            ph = sr.physical(r, n_per_unit, args.half_span, args.wn, args.zeta) if n_per_unit else None
            text.append('[%d] %s' % (k, sr.report(r, ph)))
            fits.append(sr.serialisable(r, ph))
        text.append(sr.summarise(results))
        print('\n'.join(text))
        meta['fits'] = fits
        meta['fit_report'] = text
    side = os.path.splitext(path)[0] + '.json'
    with open(side, 'w') as f:
        json.dump(meta, f, indent=2)
    print('notes and fits in %s' % side)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default=bl.BLIMP_IP)
    ap.add_argument('--port', type=int, default=bl.LINK_PORT)
    ap.add_argument('--console-port', type=int, default=bl.CONSOLE_PORT,
                    help="the console, for the 'param' listing in the .json (default %(default)s)")
    ap.add_argument('axis', choices=sorted(sr.AXES))
    ap.add_argument('--steps', type=float, nargs='+', default=[0.25, 0.40, 0.55],
                    help='amplitudes, per unit duty, signed (default %(default)s)')
    ap.add_argument('--hold', type=float, default=4.0, help='s each step is held (default %(default)s)')
    ap.add_argument('--gap', type=float, default=6.0, help='s of zeros after each step (default %(default)s)')
    ap.add_argument('--lead', type=float, default=2.0, help='s of zeros before the first step')
    ap.add_argument('--mirror', action='store_true', help='yaw: repeat the steps on FR after FL')
    ap.add_argument('--limit', type=float, default=0.75, help='largest |amplitude| allowed (default %(default)s)')
    ap.add_argument('--rate', type=float, default=20.0, help='MOTOR_SET per second')
    ap.add_argument('--folder', default='captures')
    ap.add_argument('--out', help='the .bin (default <folder>/step-<axis>-<date>.bin)')
    ap.add_argument('--notes', default='', help='free text for the .json')
    ap.add_argument('--no-fit', action='store_true', help='save only; fit later with step_response.py --all')
    ap.add_argument('--tau-c', type=float, default=0.5, help='SIMC closed-loop time constant, s')
    ap.add_argument('--thrust-map', help='thrust_map.py fit JSON, for the fits in core/ units')
    ap.add_argument('--gf-per-unit', type=float, help='one motor\'s thrust per unit duty, gf, instead of the map')
    ap.add_argument('--half-span', type=float, default=0.10, help='l_f, m (default params.c\'s placeholder)')
    ap.add_argument('--wn', type=float, default=2.0)
    ap.add_argument('--zeta', type=float, default=1.0)
    ap.add_argument('--go', action='store_true', help='really drive the motors (PROPS ON)')
    return run(ap.parse_args(argv))


if __name__ == '__main__':
    sys.exit(main())
