"""The square and the runs that calibrate it, on the firmware's bench hold:
climb to a height, hold it, then legs (a push, a coast, a brake) with turns
on the spot between them, then the landing. Height and heading are closed
loop in core/; the legs are open loop, timed pushes and brakes, because
nothing on the blimp measures where it is (core/'s estimator has attitude,
heading and height only). Their lengths come from the tape on the floor and
a video, and the push, coast and brake times are what those calibrate.

    python software/square_test.py --legs 0 --settle 10                        # landing test: climb, hold, land
    python software/square_test.py --legs 1 --push 2 4 --coast 8               # push and coast (calibration 3)
    python software/square_test.py --legs 1 --push 2 4 --coast 2 --brake 2 3  # push and brake (4), one leg (5)
    python software/square_test.py --legs 4 --turn 90 --push 2 4 --coast 2 --brake 2 3   # the square
    ... --go --param front_reverse_ratio=0.74 yaw.kp=... alt.kp=...            # really fly it

Each leg goes along the heading the blimp holds: --push GF S pushes both
front motors forward with GF in total for S s ('hold surge'), --coast S
lets it run on with the motors idle, --brake GF S pushes back with GF for S
s (a negative 'hold surge'; core/ makes a brake only with front_reverse_ratio
above 0, so the tool refuses a brake without it), then --hold s still. Then
it turns by --turn deg (+ is counter-clockwise from above, to the left), for
--turn-time s, holds --hold s again, and the next leg starts. Without
reverse a turn is an arc that pushes the blimp forward; with it, a couple.
The first leg goes the way the nose points when the hold arms: heading 0.

The climb: 'hold altitude --alt --climb-rate', core/ moving the height
setpoint from where the blimp stands (core 0.4.0), then --settle s at the
height before the first leg. The landing is hold_test.py's (land(): down
at --land-rate to 0.30 m, then --final-rate to the floor, motors off at a
ToF of 0.09 m, about 2 cm above where the gondola stands).

The report: the plan, every command with this computer's clock (to match
the video: the motors' sound marks each push and brake as well), and per
phase from the capture, the heading error, the height and the front motors.
The legs' lengths are not in it: the team reads them off the tape.

What has to be true first, as for hold_test.py: the estimator converged (60
s still), no motor driven, this tool's link alive. The driver is set to
--max-duty and --deadband before the start (the team's 40 % and 10 %, a
boot's default since 2026-11-01; set anyway, since the thrust stand leaves
it plain). Kills, from here: yaw over --max-rate, roll or pitch over
--max-tilt-rate, the ToF over --max-alt; the end, Ctrl-C and any error send
'hold off', and MOTOR_STOP three times if the console does not answer.
Standard library only.
"""
import argparse
import json
import math
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blimp_link as bl  # noqa: E402
import blimp_log  # noqa: E402
import hold_test as ht  # noqa: E402

CAPTURE_MAX_S = 400   # CONFIG_CAPTURE_MAX_SECONDS: a longer capture is refused
TOF_FLOOR_M = ht.LAND_FLOOR_M   # where the climb starts from, for the plan's timing


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def schedule(args):
    """[(t0, t1, label, kind, heading_deg, surge_gf)] from the moment the hold
    starts. kind: 'climb', 'settle', 'push', 'coast', 'brake', 'hold', 'turn'."""
    out = []
    t = 0.0

    def add(dur, label, kind, heading, surge):
        nonlocal t
        if dur > 0:
            out.append((t, t + dur, label, kind, heading, surge))
            t += dur

    climb_s = max(0.0, args.alt - TOF_FLOOR_M) / args.climb_rate
    add(climb_s, 'climb to %.2f m' % args.alt, 'climb', 0.0, 0.0)
    add(args.settle, 'settle', 'settle', 0.0, 0.0)
    heading = 0.0
    for leg in range(1, args.legs + 1):
        if leg > 1:
            heading = wrap180(heading + args.turn)
            add(args.turn_time, 'turn %d to %+.0f deg' % (leg - 1, heading), 'turn', heading, 0.0)
            add(args.hold, 'hold after turn %d' % (leg - 1), 'hold', heading, 0.0)
        add(args.push[1], 'leg %d push %.1f gf' % (leg, args.push[0]), 'push', heading, args.push[0])
        add(args.coast, 'leg %d coast' % leg, 'coast', heading, 0.0)
        if args.brake:
            add(args.brake[1], 'leg %d brake %.1f gf' % (leg, args.brake[0]), 'brake', heading, -args.brake[0])
        add(args.hold, 'hold after leg %d' % leg, 'hold', heading, 0.0)
    return out


def phase_metrics(rows, phases, offset, front):
    """Per phase, from the capture: the heading error (rms, max), the ToF
    height (min, mean, max), the peak yaw rate, and the extreme throttles of
    the front pair (the names in front; a brake shows as negative)."""
    out = []
    for t0, t1, label, kind, heading, surge in phases:
        a, b = t0 + offset + ht.EDGE_S, t1 + offset - ht.EDGE_S
        seg = [r for r in rows if a <= r['t_s'] < b and r['mode'] != 'MANUAL']
        if len(seg) < 5:
            out.append({'phase': label, 'note': 'too few records (killed before it?)'})
            continue
        err = [abs(math.degrees(r['hold_err_rad'])) for r in seg if not math.isnan(r['hold_err_rad'])]
        alt = [r['tof_mm'] / 1000.0 for r in seg if r['tof_status'] == 0]
        front_u = [r['throttle_%s' % m] for r in seg for m in front]
        out.append({'phase': label, 'kind': kind, 'heading_deg': heading, 'surge_gf': surge,
                    'heading_err_rms_deg': math.sqrt(sum(e * e for e in err) / len(err)) if err else float('nan'),
                    'heading_err_max_deg': max(err) if err else float('nan'),
                    'alt_min_m': min(alt) if alt else float('nan'),
                    'alt_mean_m': sum(alt) / len(alt) if alt else float('nan'),
                    'alt_max_m': max(alt) if alt else float('nan'),
                    'peak_yaw_rate_dps': max(abs(r['gyro_z_dps']) for r in seg),
                    'front_min': min(front_u), 'front_max': max(front_u)})
    return out


def front_names(motor_reply):
    """The names of the motors at core/'s front positions, from the motor
    map in a 'motor' reply (cmd_motor.h): the capture logs throttles by name,
    and the team's names are mirrored against core/'s positions (the motor
    named FL sits at FR). ('FL', 'FR') when the table cannot be read."""
    try:
        import step_test
        pos = step_test.parse_motor_status(motor_reply)['positions']
        names = tuple(sorted(n for n, p in pos.items() if p in ('FL', 'FR')))
        return names if len(names) == 2 else ('FL', 'FR')
    except ValueError:
        return ('FL', 'FR')


def run(args):
    phases = schedule(args)
    total = phases[-1][1] if phases else 0.0
    land_s = ht.landing_time(args.alt, args.land_rate, args.final_rate) if args.land else 0.0
    cap_s = int(math.ceil(ht.LEAD_S + total + land_s)) + 3
    print('square test: %d leg%s, %.0f s in the air before the landing, capture of %d s'
          % (args.legs, '' if args.legs == 1 else 's', total, cap_s))
    for t0, t1, label, kind, heading, surge in phases:
        print('  %6.1f .. %6.1f s  %-26s heading %+5.0f deg  surge %+5.1f gf' % (t0, t1, label, heading, surge))
    if args.land:
        print('then a landing: %.2f m/s down to %.2f m, %.2f m/s to the floor, motors off at a ToF of %.2f m '
              '(at most %.0f s)' % (args.land_rate, ht.LAND_SLOW_M, args.final_rate, ht.LAND_CUT_M, land_s))
    print('driver %g %% ceiling, %g %% deadband; kills: yaw over %.0f deg/s, roll or pitch over %.0f deg/s, '
          'ToF over %.2f m' % (args.max_duty, args.deadband, args.max_rate, args.max_tilt_rate, args.max_alt))
    if args.param:
        print('param before the start: %s' % ' '.join(args.param))
    if cap_s > CAPTURE_MAX_S:
        sys.exit('the run needs a %d s capture and the blimp keeps %d s: fewer legs or shorter phases'
                 % (cap_s, CAPTURE_MAX_S))
    if not args.go:
        print('dry run. PROPS ON for the real thing: the blimp on its start mark, nose along the first leg, '
              'the tape down and the camera running; then add --go')
        return 0

    lines = []

    def log(s):
        print(s)
        lines.append(s)

    link = bl.BlimpLink(args.host, args.port).start()
    con = bl.BlimpConsole(args.host, args.console_port)
    started = killed = cap_started = False
    data = info = tel0 = None
    params = {}
    t_cap = t_hold = None
    peak = {'yaw': 0.0, 'tilt': 0.0}
    front = ('FL', 'FR')

    def clock():
        return time.monotonic() - t_hold if t_hold is not None else 0.0

    def stamp():
        # this computer's wall clock, to the tenth: what the video is matched to
        t = time.time()
        return time.strftime('%H:%M:%S', time.localtime(t)) + ('.%d' % int((t % 1) * 10))

    def send(cmd):
        reply = ht.console(con, cmd)
        last = reply.strip().splitlines()[-1] if reply.strip() else '?'
        log('%6.1f s  %s  %s: %s' % (clock(), stamp(), cmd, last))
        if 'refused' in reply or 'not a number' in reply or 'Unrecognized' in reply:
            sys.exit('the firmware refused %r:\n%s' % (cmd, reply))
        return reply

    try:
        if not link.wait_alive(3.0):
            sys.exit('no telemetry from %s:%d' % (args.host, args.port))
        tel0 = link.telemetry
        if any(abs(v) > 0.005 for v in tel0['motor'].values()):
            sys.exit('the motors are already driven (%s): one source at a time' % tel0['motor'])
        for p in args.param:
            name, _, value = p.partition('=')
            reply = ht.console(con, 'param %s %s' % (name, value))
            log('param %s %s: %s' % (name, value, reply.strip().splitlines()[0] if reply.strip() else '?'))
            if '(was ' not in reply and 'is already' not in reply:
                sys.exit('param %s was not set; nothing started' % p)
        reply = ht.console(con, 'motor --max-duty %g --deadband %g' % (args.max_duty, args.deadband))
        log(reply.strip().splitlines()[0] if reply.strip() else '?')
        if 'takes' in reply:
            sys.exit('the driver refused --max-duty %g --deadband %g; nothing started' % (args.max_duty, args.deadband))
        front = front_names(reply)
        log('the front pair, by name: %s' % ' '.join(front))
        params = bl.read_params(con)
        ratio = params.get('values', {}).get('front_reverse_ratio')
        if args.brake and not (ratio and ratio > 0):
            sys.exit('a brake needs front_reverse_ratio above 0 (it is %s): add --param front_reverse_ratio=0.74, '
                     'or drop --brake' % ratio)
        if args.legs > 1 and not (ratio and ratio > 0):
            log('note: front_reverse_ratio is %s, so the turns are arcs that push the blimp forward' % ratio)
        t_wait = time.monotonic()
        while True:
            state = ht.console(con, 'hold')
            m = re.search(r'estimator: (\w+) \(still (\d+) of 60 s\)', state)
            if m and m.group(1) == 'converged':
                break
            if time.monotonic() - t_wait > args.wait:
                sys.exit('the estimator did not converge in %.0f s (it needs the gondola still for 60 s):\n%s'
                         % (args.wait, state))
            print('\rwaiting for the gondola to be still: %s of 60 s ' % (m.group(2) if m else '?'), end='',
                  flush=True)
            time.sleep(1.0)
        print()

        cap = bl.CaptureClient(link)
        if cap.info()['state'] == 1:
            cap.stop()
        cap.start(cap_s)
        cap_started = True
        t_cap = time.monotonic()
        time.sleep(ht.LEAD_S)

        tripped, peak = ht.make_kills(link, args.max_rate, args.max_tilt_rate, args.max_alt)
        reply = ht.console(con, 'hold heading 0')
        t_hold = time.monotonic()
        log('%6.1f s  %s  hold heading 0: %s' % (0.0, stamp(), reply.strip().splitlines()[-1]))
        if 'refused' in reply:
            sys.exit('the firmware refused the hold:\n' + reply)
        started = True
        send('hold altitude %g %g' % (args.alt, args.climb_rate))

        # The phases follow one another from the hold's start: each one's
        # commands go out as the one before ends, then it runs to its t1.
        heading_now, surge_now = 0.0, 0.0
        next_poll = 0.0
        for t0, t1, label, kind, heading, surge in phases:
            if heading != heading_now:
                send('hold heading %g' % heading)
                heading_now = heading
            if surge != surge_now:
                send('hold surge %g' % surge)
                surge_now = surge
            log('%6.1f s  %s  -- %s (planned from %.1f s)' % (clock(), stamp(), label, t0))
            while clock() < t1:
                why = tripped()
                if why:
                    killed = True
                    ht.console(con, 'hold kill')
                    log('%6.1f s  KILLED: %s' % (clock(), why))
                    break
                if time.monotonic() >= next_poll:
                    log('%6.1f s  %s' % (clock(), ht.hold_line(ht.console(con, 'hold'))))
                    next_poll = time.monotonic() + args.poll
                time.sleep(0.02)
            if killed:
                break

        if started and not killed:
            if surge_now:
                send('hold surge 0')
            if args.land:
                killed = ht.land(con, link, args.alt, args.land_rate, args.final_rate, tripped, log, clock,
                                 args.poll)
        log('peak rates seen from here: yaw %.0f deg/s, roll/pitch %.0f deg/s' % (peak['yaw'], peak['tilt']))
    except KeyboardInterrupt:
        log('Ctrl-C')
    finally:
        try:
            log(ht.console(con, 'hold off').strip())
        except (TimeoutError, ConnectionError, OSError) as e:
            log('hold off did not answer (%s): MOTOR_STOP x3, the hold is KILLED until hold off' % e)
            for _ in range(3):
                link.stop_motors()
                time.sleep(0.05)
        if cap_started:
            try:
                cap = bl.CaptureClient(link)
                cap.stop()
                info, data = cap.download(progress=lambda a, n: print('\r%d / %d chunks' % (a, n), end='',
                                                                       flush=True))
                print()
            except (TimeoutError, bl.CaptureError) as e:
                print('download: %s (try blimp_link.py capture download)' % e)
        con.close()
        link.close()
    if not data:
        sys.exit('no capture')

    path = args.out or bl.capture_path('square', args.folder)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    blimp_log.write_file(path, info['raw'], data)
    h, records = blimp_log.decode(info['raw'], data)
    rows = blimp_log.to_units(h, records)
    offset = (t_hold - t_cap) if started else ht.LEAD_S
    res = phase_metrics(rows, phases, offset, front)
    print('  %-26s %8s %8s %17s %8s %15s' % ('phase', 'hdg rms', 'hdg max', 'height min/mean/max', 'yaw max',
                                              'front min/max'))
    for r in res:
        if 'note' in r:
            print('  %-26s %s' % (r['phase'], r['note']))
            continue
        print('  %-26s %6.1f d %6.1f d   %4.2f %4.2f %4.2f m %5.0f d/s   %+5.2f %+5.2f'
              % (r['phase'], r['heading_err_rms_deg'], r['heading_err_max_deg'], r['alt_min_m'], r['alt_mean_m'],
                 r['alt_max_m'], r['peak_yaw_rate_dps'], r['front_min'], r['front_max']))
    meta = {'file': os.path.basename(path), 'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'source': 'square_test',
            'notes': args.notes, 'header': h, 'records': len(records), 'params': params,
            'test': {'legs': args.legs, 'turn_deg': args.turn, 'alt_m': args.alt, 'climb_rate_mps': args.climb_rate,
                     'settle_s': args.settle, 'push': args.push, 'coast_s': args.coast, 'brake': args.brake,
                     'hold_s': args.hold, 'turn_time_s': args.turn_time, 'land': args.land,
                     'land_rate_mps': args.land_rate, 'final_rate_mps': args.final_rate,
                     'phases': [list(p) for p in phases], 'hold_start_in_capture_s': offset,
                     'param': args.param, 'max_duty_pct': args.max_duty, 'deadband_pct': args.deadband,
                     'max_rate_dps': args.max_rate, 'max_tilt_rate_dps': args.max_tilt_rate,
                     'max_alt_m': args.max_alt, 'killed_from_ground': killed, 'telemetry_before': tel0,
                     'log': lines},
            'metrics': res}
    side = os.path.splitext(path)[0] + '.json'
    with open(side, 'w') as f:
        json.dump(meta, f, indent=2)
    print('%d records saved to %s, notes in %s' % (len(records), path, side))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default=bl.BLIMP_IP)
    ap.add_argument('--port', type=int, default=bl.LINK_PORT)
    ap.add_argument('--console-port', type=int, default=bl.CONSOLE_PORT)
    ap.add_argument('--legs', type=int, default=4, help='legs to fly, 0 .. 8 (default %(default)s; 0: climb, '
                                                      'settle and land)')
    ap.add_argument('--turn', type=float, default=90.0,
                    help='the turn between legs, deg, + counter-clockwise (left) (default %(default)s)')
    ap.add_argument('--alt', type=float, default=1.0, help='the height, m (default %(default)s)')
    ap.add_argument('--climb-rate', type=float, default=0.12,
                    help='the climb from the floor, m/s (default %(default)s: about what the rear pair '
                         'managed at the 40 %% ceiling, floor to 1 m in 8 s on 2026-10-31)')
    ap.add_argument('--settle', type=float, default=5.0,
                    help='s at the height before the first leg (default %(default)s)')
    ap.add_argument('--push', type=float, nargs=2, default=[2.0, 4.0], metavar=('GF', 'S'),
                    help='each leg\'s push: both front motors together, gf, for s (default 2 gf, 4 s)')
    ap.add_argument('--coast', type=float, default=2.0, help='s with the motors idle after the push (default %(default)s)')
    ap.add_argument('--brake', type=float, nargs=2, metavar=('GF', 'S'),
                    help='each leg\'s brake: gf backwards for s (needs front_reverse_ratio > 0); none by default')
    ap.add_argument('--hold', type=float, default=5.0,
                    help='s still after each leg and after each turn (default %(default)s)')
    ap.add_argument('--turn-time', type=float, default=8.0,
                    help='s given to each turn before the hold after it (default %(default)s)')
    ap.add_argument('--no-land', dest='land', action='store_false',
                    help='end without the landing (the motors stop where the vehicle is)')
    ap.add_argument('--land-rate', type=float, default=0.1, help='the landing down to 0.30 m, m/s (default %(default)s)')
    ap.add_argument('--final-rate', type=float, default=0.03,
                    help='the landing from 0.30 m to the floor, m/s (default %(default)s)')
    ap.add_argument('--param', nargs='*', default=[], metavar='NAME=VALUE', help="'param' sets before the start")
    ap.add_argument('--max-duty', type=float, default=40.0,
                    help="'motor --max-duty' before the start, percent (default %(default)s, the team's)")
    ap.add_argument('--deadband', type=float, default=10.0,
                    help="'motor --deadband' before the start, percent (default %(default)s)")
    ap.add_argument('--max-rate', type=float, default=62.0,
                    help='kill above this yaw rate, deg/s (default %(default)s: a clean 90 deg turn peaked at 41)')
    ap.add_argument('--max-tilt-rate', type=float, default=120.0,
                    help='kill above this roll or pitch rate, deg/s (default %(default)s)')
    ap.add_argument('--max-alt', type=float, help='kill above this ToF height, m (default --alt + 0.5)')
    ap.add_argument('--poll', type=float, default=1.0, help="s between 'hold' status lines (default %(default)s)")
    ap.add_argument('--wait', type=float, default=90.0,
                    help="s to wait for core/'s estimator to converge (default %(default)s)")
    ap.add_argument('--folder', default='captures')
    ap.add_argument('--out', help='the .bin (default <folder>/square-<date>.bin)')
    ap.add_argument('--notes', default='')
    ap.add_argument('--go', action='store_true', help='really fly it (PROPS ON)')
    args = ap.parse_args(argv)

    if not 0 <= args.legs <= 8:
        sys.exit('--legs is 0 .. 8')
    if not math.isfinite(args.turn) or abs(args.turn) > 180:
        sys.exit('--turn is -180 .. 180 deg')
    if not math.isfinite(args.alt) or not 0.3 <= args.alt <= 2.5:
        sys.exit('--alt is 0.3 .. 2.5 m (the room is 3 m high)')
    if not math.isfinite(args.climb_rate) or not 0.02 <= args.climb_rate <= 0.5:
        sys.exit("--climb-rate is 0.02 .. 0.5 m/s (the firmware's 'hold altitude' rate)")
    for name, v in (('--settle', args.settle), ('--coast', args.coast), ('--hold', args.hold),
                    ('--turn-time', args.turn_time)):
        if not math.isfinite(v) or not 0.0 <= v <= 60.0:
            sys.exit('%s is 0 .. 60 s' % name)
    for name, pair in (('--push', args.push), ('--brake', args.brake)):
        if pair is None:
            continue
        gf, s = pair
        if not (math.isfinite(gf) and 0.0 < gf <= 30.0 and math.isfinite(s) and 0.0 < s <= 30.0):
            sys.exit("%s is GF S: 0 < GF <= 30 gf (the firmware's 'hold surge' range), 0 < S <= 30 s" % name)
    if not math.isfinite(args.land_rate) or not 0.02 <= args.land_rate <= 0.5:
        sys.exit('--land-rate is 0.02 .. 0.5 m/s')
    if not math.isfinite(args.final_rate) or not 0.01 <= args.final_rate <= 0.2:
        sys.exit('--final-rate is 0.01 .. 0.2 m/s')
    if not (10.0 <= args.max_duty <= 100.0 and 0.0 <= args.deadband < args.max_duty):
        sys.exit('--max-duty is 10 .. 100 and --deadband 0 .. below it (percent)')
    if not (args.max_rate > 0 and args.max_tilt_rate > 0):
        sys.exit('--max-rate and --max-tilt-rate are > 0 deg/s')
    if args.max_alt is None:
        args.max_alt = args.alt + 0.5
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
