"""Closed-loop heading steps on the vehicle: the firmware's bench hold
('hold', control-loop increment 3's slice of 2026-10-30) runs core/'s own
yaw loop on the S3; this tool keeps the link alive, steps the setpoint,
watches the spin, records a capture and always ends with 'hold off'.

    python software/hold_test.py --steps 0 30 0 -30 0                 # dry run: prints the plan
    python software/hold_test.py --steps 0 30 0 -30 0 --dwell 8 --go \\
        --param yaw.kp=0.0008 yaw.kd=0.0008 --deadband 10
    python software/hold_test.py --steps 0 --dwell 20 --go            # hold where it points, 20 s

A heading is in degrees, + counter-clockwise seen from above, from where the
nose points when the hold arms (core/'s estimator latches the yaw origin
there). Gains are core/'s, in SI: yaw.kp in N m/rad, yaw.kd in N m s/rad;
--param sets any 'param' field before the start (RAM only: a reboot restores
params.c), and the listing is saved in the .json either way.

What has to be true first, and the firmware refuses otherwise: the estimator
has seen the gondola still for 60 s since boot or since the last hold; no
motor is driven; and this tool's link is alive, which it is from the moment
it starts (its heartbeats are what core/'s link watchdog needs: the hold
drops to zero thrust within link_timeout_s, 1 s, if this tool dies).

Safety, in layers: --max-rate (default 120 deg/s) kills the hold from here
the moment the gyro in the 50 Hz SENSORS stream exceeds it, which is what a
sign error or too much gain looks like on a tether; the end of the schedule,
Ctrl-C and any error send 'hold off', and MOTOR_STOP three times if the
console does not answer (a STOP kills the hold on the blimp: KILLED until
'hold off'); core/'s own limits (yaw.out_min/out_max, Rule 13) bound the
torque, and lowering yaw.out_max with --param is the way to cap it on the
rig. Standard library only.
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

LEAD_S = 2.0          # zero-thrust baseline in the capture before the hold arms


def plan(steps, dwell):
    """[(t0, t1, heading_deg)] from the moment the hold starts."""
    return [(k * dwell, (k + 1) * dwell, h) for k, h in enumerate(steps)]


def console(con, line, log=None):
    text = con.run(line, timeout=5.0, idle=2.0)
    if log:
        for ln in text.strip().splitlines():
            log('    > ' + ln)
    return text


def hold_line(text):
    """One compact line from 'hold''s reply."""
    m = re.search(r'heading: estimate ([-+\d.]+) deg.*?rate ([-+\d.]+) deg/s(?:; held at ([-+\d.]+) deg, '
                  r'error ([-+\d.]+) deg)?', text)
    o = re.search(r'throttle FL ([\d.]+) RL ([\d.]+) FR ([\d.]+) RR ([\d.]+)', text)
    mode = re.search(r'mode (\w+)', text)
    parts = [mode.group(1) if mode else '?']
    if m:
        parts.append('hdg %+6.1f  rate %+6.1f' % (float(m.group(1)), float(m.group(2))))
        if m.group(3):
            parts.append('sp %+6.1f  err %+6.1f' % (float(m.group(3)), float(m.group(4))))
    if o:
        parts.append('FL %.3f RL %.3f FR %.3f RR %.3f' % tuple(float(v) for v in o.groups()))
    return '  '.join(parts)


def metrics(rows, marks):
    """Per setpoint: overshoot past it, time into a 5 deg band and staying,
    the final error (mean, std) over the last 2 s, the peak throttle; from
    the capture's hold columns (blimp_log.py)."""
    out = []
    for k, (t0, t1, sp) in enumerate(marks):
        seg = [r for r in rows if t0 <= r['t_s'] < t1 and not math.isnan(r['hold_err_rad'])]
        if len(seg) < 10:
            out.append({'setpoint_deg': sp, 'note': 'too few records'})
            continue
        err = [math.degrees(r['hold_err_rad']) for r in seg]
        e0 = err[0]
        # overshoot: the error crossing zero and going past it, against the
        # direction it started in
        over = max((-e if e0 > 0 else e) for e in err) if abs(e0) > 1.0 else float('nan')
        settle = float('nan')
        for i in range(len(err)):
            if all(abs(e) <= 5.0 for e in err[i:]):
                settle = seg[i]['t_s'] - t0
                break
        tail = [e for r, e in zip(seg, err) if r['t_s'] >= t1 - 2.0]
        mean = sum(tail) / len(tail)
        std = math.sqrt(sum((e - mean) ** 2 for e in tail) / len(tail))
        peak = max(max(abs(r['throttle_%s' % m]) for m in blimp_log.MOTORS) for r in seg)
        out.append({'setpoint_deg': sp, 'start_error_deg': e0, 'overshoot_deg': over, 'settle_s': settle,
                    'final_error_deg': mean, 'final_std_deg': std, 'peak_throttle': peak})
    return out


def run(args):
    marks = plan(args.steps, args.dwell)
    total = marks[-1][1]
    cap_s = int(math.ceil(LEAD_S + total)) + 3
    print('heading hold, %d setpoints, %.0f s, capture of %d s:' % (len(marks), total, cap_s))
    for t0, t1, h in marks:
        print('  %6.1f .. %6.1f s  heading %+6.1f deg' % (t0, t1, h))
    if args.param:
        print('param before the start: %s' % ' '.join(args.param))
    if args.deadband is not None:
        print('motor deadband before the start: %.1f %%' % args.deadband)
    if args.max_duty is not None:
        print('motor duty ceiling before the start: %.0f %%' % args.max_duty)
    if args.surge:
        print('surge held throughout: %.2f gf (both front motors at about %.2f gf; yaw differs around it)'
              % (args.surge, args.surge / 2))
    if not args.go:
        print('dry run. PROPS ON for the real thing: the gondola on its swivel, nothing near the props, '
              'STOP within reach; then add --go')
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
    try:
        if not link.wait_alive(3.0):
            sys.exit('no telemetry from %s:%d' % (args.host, args.port))
        tel0 = link.telemetry
        if any(abs(v) > 0.005 for v in tel0['motor'].values()):
            sys.exit('the motors are already driven (%s): one source at a time' % tel0['motor'])
        for p in args.param:
            name, _, value = p.partition('=')
            reply = console(con, 'param %s %s' % (name, value))
            log('param %s %s: %s' % (name, value, reply.strip().splitlines()[0] if reply.strip() else '?'))
            if '(was ' not in reply and 'is already' not in reply:
                sys.exit('param %s was not set; nothing started' % p)
        if args.max_duty is not None or args.deadband is not None:
            opts = ''.join([' --max-duty %g' % args.max_duty if args.max_duty is not None else '',
                            ' --deadband %g' % args.deadband if args.deadband is not None else ''])
            reply = console(con, 'motor' + opts)
            log(reply.strip())
            if 'takes' in reply:
                sys.exit('motor%s was refused; nothing started' % opts)
        params = bl.read_params(con)
        # core/ arms only after 60 s of stillness; wait for it rather than
        # refuse, printing the count, up to --wait seconds (hands off meanwhile)
        t_wait = time.monotonic()
        while True:
            state = console(con, 'hold')
            m = re.search(r'estimator: (\w+) \(still (\d+) of 60 s\)', state)
            if m and m.group(1) == 'converged':
                break
            if time.monotonic() - t_wait > args.wait:
                sys.exit('the estimator did not converge in %.0f s (it needs the gondola still for 60 s; '
                         'a hand on it, or the ToF covered, resets the count):\n%s' % (args.wait, state))
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
        time.sleep(LEAD_S)

        k = -1
        next_poll = 0.0
        max_seen = 0.0
        while True:
            now = time.monotonic()
            el = now - t_hold if t_hold is not None else 0.0
            want = next((i for i, m in enumerate(marks) if m[0] <= el < m[1]), None)
            if want is None:
                break
            if want != k:
                k = want
                reply = console(con, 'hold heading %g' % marks[k][2])
                if t_hold is None:
                    t_hold = time.monotonic()
                    started = 'refused' not in reply
                log('%6.1f s  hold heading %+g: %s' % (time.monotonic() - t_hold, marks[k][2],
                                                      reply.strip().splitlines()[-1]))
                if 'refused' in reply:
                    sys.exit('the firmware refused the hold:\n' + reply)
                if k == 0 and args.surge:
                    reply = console(con, 'hold surge %g' % args.surge)
                    log('%6.1f s  hold surge %g: %s' % (time.monotonic() - t_hold, args.surge,
                                                       reply.strip().splitlines()[-1]))
                    if 'refused' in reply:
                        sys.exit('the firmware refused the surge:\n' + reply)
            s = link.sensors
            if s and s['imu_valid']:
                rate = max(abs(g) for g in s['gyro'])
                max_seen = max(max_seen, rate)
                if rate > args.max_rate:
                    killed = True
                    console(con, 'hold kill')
                    log('%6.1f s  KILLED: |gyro| %.0f deg/s over --max-rate %.0f' % (el, rate, args.max_rate))
                    break
            if now >= next_poll:
                log('%6.1f s  %s' % (el, hold_line(console(con, 'hold'))))
                next_poll = now + args.poll
            time.sleep(0.02)
        log('peak |gyro| seen from here: %.0f deg/s' % max_seen)
    except KeyboardInterrupt:
        log('Ctrl-C')
    finally:
        try:
            log(console(con, 'hold off').strip())
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

    path = args.out or bl.capture_path('hold-heading', args.folder)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    blimp_log.write_file(path, info['raw'], data)
    h, records = blimp_log.decode(info['raw'], data)
    rows = blimp_log.to_units(h, records)
    offset = (t_hold - t_cap) if started else LEAD_S
    marks_cap = [(t0 + offset, t1 + offset, sp) for t0, t1, sp in marks]
    res = metrics(rows, marks_cap)
    for r in res:
        if 'note' in r:
            print('  %+6.1f deg: %s' % (r['setpoint_deg'], r['note']))
            continue
        print('  %+6.1f deg: from %+6.1f, overshoot %5.1f deg, inside 5 deg after %4.1f s, final %+5.1f +- %.1f deg, '
              'peak throttle %.2f' % (r['setpoint_deg'], r['start_error_deg'], r['overshoot_deg'], r['settle_s'],
                                      r['final_error_deg'], r['final_std_deg'], r['peak_throttle']))
    meta = {'file': os.path.basename(path), 'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'source': 'hold_test',
            'notes': args.notes, 'header': h, 'records': len(records), 'params': params,
            'test': {'steps_deg': args.steps, 'dwell_s': args.dwell, 'hold_start_in_capture_s': offset,
                     'param': args.param, 'deadband_pct': args.deadband, 'max_duty_pct': args.max_duty,
                     'surge_gf': args.surge, 'max_rate_dps': args.max_rate,
                     'killed_from_ground': killed, 'telemetry_before': tel0, 'log': lines},
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
    ap.add_argument('--steps', type=float, nargs='+', default=[0.0], help='heading setpoints, deg (default 0)')
    ap.add_argument('--dwell', type=float, default=8.0, help='s at each setpoint (default %(default)s)')
    ap.add_argument('--param', nargs='*', default=[], metavar='NAME=VALUE', help="'param' sets before the start")
    ap.add_argument('--deadband', type=float, help="'motor --deadband' before the start, percent")
    ap.add_argument('--max-duty', type=float, help="'motor --max-duty' before the start, percent (the team: 30)")
    ap.add_argument('--surge', type=float, default=0.0,
                    help="'hold surge' for the whole run, gf (default 0): keeps both front motors turning, so "
                         "a yaw correction speeds one up and slows the other instead of starting one from rest")
    ap.add_argument('--max-rate', type=float, default=120.0, help='kill above this |gyro|, deg/s (default %(default)s)')
    ap.add_argument('--poll', type=float, default=1.0, help="s between 'hold' status lines (default %(default)s)")
    ap.add_argument('--wait', type=float, default=90.0,
                    help="s to wait for core/'s estimator to converge before giving up (default %(default)s)")
    ap.add_argument('--folder', default='captures')
    ap.add_argument('--out', help='the .bin (default <folder>/hold-heading-<date>.bin)')
    ap.add_argument('--notes', default='')
    ap.add_argument('--go', action='store_true', help='really run it (PROPS ON)')
    args = ap.parse_args(argv)
    if any(not math.isfinite(h) or abs(h) > 180 for h in args.steps):
        sys.exit('headings are -180 .. 180 deg')
    if args.dwell <= 0 or args.dwell * len(args.steps) > 300:
        sys.exit('--dwell must be > 0 and the whole run under 300 s (the capture buffer)')
    if not math.isfinite(args.surge) or not 0.0 <= args.surge <= 30.0:
        sys.exit("--surge is 0 .. 30 gf (the firmware's 'hold surge' range)")
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
