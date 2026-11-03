#!/usr/bin/env python3
"""The balloon flight (2026-11-02): climb to --alt, look for the forward
camera's red balloon there, turn to it, then move toward it, back, and toward
it again, following it with the nose ('hold follow') throughout; always end
in a landing.

    python software/target_test.py --trim -0.6 --param $(cat software/bench_params.txt)        # the plan
    python software/target_test.py --trim -0.6 --param $(cat software/bench_params.txt) --go   # PROPS ON

The setup it was written for: the balloon about 2 m up and 3 m ahead, the
blimp on the floor with its nose toward it.

1. The climb to --alt (2.0 m), at --climb-rate. The balloon counts as seen
   when K of the camera's last N frames detect it (--seen K N; 13 frames a
   second, relayed by the S3); seen, the nose turns to it ('hold follow').
   Not seen within --look s of reaching the height, the scan: the nose turns
   left in --scan-step steps to --scan deg, back, right to -(--scan) deg and
   back, --scan-dwell s at each (the second try drifted sideways in the climb
   and held its nose straight, so the balloon was off the side of the 58 deg
   view at 2 m); not seen by the end of it, the landing.
2. --settle s following at the height.
3. The moves, each a push and a counter-push along the nose, then --hold s
   still: toward the balloon --push then --brake (aimed at about 40 cm: 3 gf
   3 s and 3 gf 4 s went about 1 m in test 4c, 1 m was too much in the third
   try, and the distance goes with the square of the times), away from it
   the mirror (-GF for the brake's S, then +GF for the push's S: a backward
   push does about three quarters of a forward one), toward it again.
4. The landing, as hold_test.py's.

The proximity guard, from the moment the nose follows: the third try ran
into the balloon. Too close is the patch's median at --near-px or more, the
balloon --near-el deg or more above the camera's level once at the height,
or the balloon lost right after big patches; then --backoff backwards, 3 s
still, no more moves toward it, and the landing. A stand-in for a range: the
camera's own range is wrong closer than about 2 m with the shiny balloon (T2).

Every end is a landing (the team, 2026-11-02, after the first try stopped the
motors at 2.6 m and the blimp had to be caught): the balloon not seen, the
ToF over --alt + 0.3 m, the ToF silent for 1 s, Ctrl-C, or an error in this
script. The first try showed why the height stays at 2.3 m or below: the
VL53L1X read to 2.49 m and lost the floor by 2.7 m, after which core/ has no
height. On a ToF dropout the landing starts at once: the setpoint comes down,
and core/ takes the blimp back as soon as the ToF reads again. Two ends are
not landings: the yaw and tilt kills (a controller misbehaving cannot be
trusted to land; 'hold kill', the motors stop), and a lost link (core/ zeroes
the hold after 1 s without this tool's packets). A second Ctrl-C during the
landing stops the motors.

--flight-bias-tau (default 600 s) is square_test.py's stopgap against core/
learning the gyro's bias in flight.

This is a test harness, not the race's guidance: it sequences the bench
hold's setpoints from the ground on what the camera reports, as square_test.py
sequences them on a clock. The search and approach for the race belong in
core/'s guidance. Standard library only.
"""
import argparse
import collections
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
import square_test as sq  # noqa: E402

ALT_MAX_M = 2.3       # the ToF read to 2.49 m and lost the floor by 2.7 m (target-20261102-155347)
TOF_LOST_S = 1.0      # a ToF silent this long starts the landing
HIGH_M = 0.3          # the ToF this far over --alt starts the landing
# The proximity guard (2026-11-02, after target-20261102-162334 ran into the
# balloon): the camera's range is wrong closer than about 2 m with the shiny
# balloon (T2), but its patch grows and, for a balloon above the camera, rises
# in the image as the blimp closes in: 137 px at +3..7 deg while following,
# 431-454 px at +8..13 deg in the push a few seconds before the contact, and the
# balloon then left the top of the 45 deg tall view (+22.5 deg).
NEAR_AREAS = 5        # the patch is the median of the last this many detections
NEAR_LOST = 0.7       # lost with the last patches above this fraction of --near-px: lost close up


class Sightings:
    """The camera's last N frames, from the CAMERA packets the S3 relays: one
    per row of target_cam's (12-13 a second), plus a health-only one at least
    every second, which is not a frame."""

    def __init__(self, link, n):
        self.link = link
        self.frames = collections.deque(maxlen=n)
        self.areas = collections.deque(maxlen=NEAR_AREAS)
        self.seq = None
        self.last = None         # the latest packet with a detection
        self.total = self.hits = 0

    def update(self):
        c = self.link.camera
        if c is not None and c['row'] and c['frame_seq'] != self.seq:
            self.seq = c['frame_seq']
            self.frames.append(bool(c['ok']))
            self.total += 1
            if c['ok']:
                self.last = c
                self.hits += 1
                self.areas.append(c['area_px'])

    def seen(self, k):
        return sum(self.frames) >= k

    def near(self, near_px, near_el_deg):
        """Why the balloon looks too close, or None: the patch's median over
        the last detections, the last detection's height in the image (pass
        near_el_deg None in the climb, where a balloon seen from below sits
        high in the image however far it is), or every recent frame empty
        right after big patches."""
        if len(self.areas) >= 3:
            med = sorted(self.areas)[len(self.areas) // 2]
            if med >= near_px:
                return 'the patch is %d px (median of the last %d), at or over %d' % (med, len(self.areas), near_px)
            if not any(self.frames) and len(self.frames) == self.frames.maxlen and \
                    max(self.areas) >= NEAR_LOST * near_px:
                return 'the balloon was lost right after patches of up to %d px' % max(self.areas)
        if near_el_deg is not None and self.last is not None and self.frames and self.frames[-1] and \
                self.last['el_deg'] >= near_el_deg:
            return 'the balloon is %.1f deg above the camera\'s level, at or over %.0f' % (self.last['el_deg'],
                                                                                         near_el_deg)
        return None

    def text(self):
        c = self.last
        return '%d/%d frames%s' % (sum(self.frames), len(self.frames),
                                   ', last az %+.1f el %+.1f deg %d px' % (c['az_deg'], c['el_deg'], c['area_px'])
                                   if c else '')


def tof_m(link):
    s = link.sensors
    return s['tof_mm'] / 1000.0 if s and s['tof_valid'] and s['tof_status'] == 0 else None


def scan_headings(half_deg, step_deg):
    """The scan's headings, deg, + left: out to +half and back, out to -half
    and back to 0; [] when half is 0."""
    n = int(round(half_deg / step_deg)) if half_deg > 0 else 0
    left = [step_deg * i for i in range(1, n + 1)] + [step_deg * i for i in range(n - 1, -1, -1)]
    return left + [-h for h in left if h] + ([0.0] if n else [])


def moves(args):
    """[(label, [(surge_gf, s), ...])] without the trim: toward, away, toward."""
    (pg, ps), (bg, bs) = args.push, args.brake
    toward = [(pg, ps), (-bg, bs)]
    away = [(-bg, bs), (pg, ps)]
    return [('toward the balloon', toward), ('away from it', away), ('toward it again', toward)]


def run(args):
    plan = moves(args)
    scan = scan_headings(args.scan, args.scan_step)
    scan_s = len(scan) * args.scan_dwell
    climb_s = (args.alt - ht.LAND_FLOOR_M) / args.climb_rate
    moves_s = sum(s for _, m in plan for _, s in m) + args.hold * len(plan)
    land_s = ht.landing_time(args.alt + HIGH_M, args.land_rate, args.final_rate)
    cap_s = int(math.ceil(ht.LEAD_S + climb_s + 5.0 + args.look + scan_s + args.settle + moves_s
                          + args.backoff[1] + 3.0 + land_s)) + 5
    print('balloon flight: climb at %.2f m/s to %.2f m; seen when %d of the last %d frames detect it, and the nose '
          'turns to it; not seen %.0f s after reaching the height: %s'
          % (args.climb_rate, args.alt, args.seen[0], args.seen[1], args.look,
             'the scan, then land if still not seen' if scan else 'land'))
    if scan:
        print('the scan: %s deg, %.0f s each (%.0f s)' % (' '.join('%+.0f' % s for s in scan), args.scan_dwell,
                                                         scan_s))
    print('then %.0f s following, and the moves (trim %+.1f gf added to every surge):' % (args.settle, args.trim))
    for label, m in plan:
        print('  %-20s %s, then %.0f s still' % (label, ', '.join('%+.1f gf %.1f s' % (g, s) for g, s in m), args.hold))
    print('too close (the patch %d px, or %.0f deg above level once at the height, or lost right after big '
          'patches): %+.1f gf %.1f s, 3 s still, the landing' % (args.near_px, args.near_el, -args.backoff[0],
                                                                args.backoff[1]))
    print('then the landing (at most %.0f s); capture of %d s' % (land_s, cap_s))
    print('lands early on: the ToF over %.2f m, the ToF silent %.0f s, Ctrl-C, an error here; kills (motors off): '
          'yaw over %.0f deg/s, roll or pitch over %.0f deg/s'
          % (args.alt + HIGH_M, TOF_LOST_S, args.max_rate, args.max_tilt_rate))
    if cap_s > sq.CAPTURE_MAX_S:
        sys.exit('the run needs a %d s capture and the blimp keeps %d s' % (cap_s, sq.CAPTURE_MAX_S))
    if not args.go:
        print('dry run. PROPS ON for the real thing: the blimp on the floor, nose toward the balloon, one red '
              'balloon in the camera\'s view, the ground station closed; then add --go')
        return 0

    lines, events = [], []

    def log(s):
        print(s)
        lines.append(s)

    link = bl.BlimpLink(args.host, args.port).start()
    con = bl.BlimpConsole(args.host, args.console_port)
    started = killed = landed = cap_started = tau_set = False
    data = info = tel0 = None
    params = {}
    t_cap = t_hold = None
    peak = {'yaw': 0.0, 'tilt': 0.0}
    tripped = None
    front = ('FL', 'FR')
    phases = []              # (t0, t1, label, kind, heading, surge) as flown, for square_test's metrics
    ground_tau = 20.0        # core/src/params.c's gyro_bias_tau_s, unless --param sets another
    for p in args.param:
        name, _, value = p.partition('=')
        if name == 'gyro_bias_tau_s':
            ground_tau = float(value)
    eye = Sightings(link, args.seen[1])
    state = {'next_poll': 0.0, 'tof_t': time.monotonic()}

    def clock():
        return time.monotonic() - t_hold if t_hold is not None else 0.0

    def send(cmd):
        reply = ht.console(con, cmd)
        last = reply.strip().splitlines()[-1] if reply.strip() else '?'
        log('%6.1f s  %s: %s' % (clock(), cmd, last))
        if 'refused' in reply or 'not a number' in reply or 'Unrecognized' in reply:
            raise RuntimeError('the firmware refused %r: %s' % (cmd, last))
        return reply

    def tick(guard=True):
        """The kills, the landing triggers, the proximity guard (once the
        nose follows the balloon) and the status line, every 20 ms of the
        flight. None, ('kill', why), ('land', why) or ('near', why)."""
        eye.update()
        why = tripped()
        if why:
            return ('kill', why)
        if guard and state.get('following'):
            why = eye.near(args.near_px, args.near_el if state.get('at_height') else None)
            if why:
                return ('near', why)
        h = tof_m(link)
        now = time.monotonic()
        if h is not None:
            state['tof_t'] = now
            if h > args.alt + HIGH_M:
                return ('land', 'the ToF reads %.2f m, over %.2f m' % (h, args.alt + HIGH_M))
        elif now - state['tof_t'] > TOF_LOST_S:
            return ('land', 'the ToF has not read for %.0f s' % TOF_LOST_S)
        if now >= state['next_poll']:
            try:
                log('%6.1f s  %s  cam %s' % (clock(), ht.hold_line(ht.console(con, 'hold')), eye.text()))
            except TimeoutError:
                log('%6.1f s  no answer to the status poll (a Wi-Fi drop?)' % clock())
            state['next_poll'] = now + args.poll
        return None

    def phase(label, kind, seconds, surge):
        t0 = clock()
        log('%6.1f s  -- %s' % (t0, label))
        send('hold surge %g' % (surge + args.trim))
        while clock() < t0 + seconds:
            stop = tick()
            if stop:
                return stop
            time.sleep(0.02)
        phases.append((t0, clock(), label, kind, 0.0, surge + args.trim))
        return None

    def backoff(why):
        """Too close: --backoff GF S backwards, 3 s still; None, or a kill or
        landing trigger met on the way (the guard is off meanwhile)."""
        events.append({'t': clock(), 'event': 'near', 'why': why, 'cam': eye.last})
        log('%6.1f s  too close: %s; backing off, and no more moves toward it' % (clock(), why))
        for label, g, s in (('back off', -args.backoff[0], args.backoff[1]), ('still after backing off', 0.0, 3.0)):
            t0 = clock()
            send('hold surge %g' % (g + args.trim))
            while clock() < t0 + s:
                stop = tick(guard=False)
                if stop:
                    return stop
                time.sleep(0.02)
            phases.append((t0, clock(), label, 'backoff', 0.0, g + args.trim))
        return None

    def landing(why):
        """The landing from the height's setpoint; True when it ended on a kill."""
        log('%6.1f s  landing: %s' % (clock(), why))
        try:
            send('hold surge %g' % args.trim)
        except (RuntimeError, TimeoutError) as e:
            log('%6.1f s  the trim was not set for the landing (%s); landing anyway' % (clock(), e))
        t_land = clock()
        k = ht.land(con, link, args.alt, args.land_rate, args.final_rate, tripped, log, clock, args.poll)
        phases.append((t_land, clock(), 'landing', 'land', 0.0, args.trim))
        return k

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
        front = sq.front_names(reply)
        params = ht.read_params(con, log)
        ratio = params.get('values', {}).get('front_reverse_ratio')
        if not (ratio and ratio > 0):
            sys.exit('the moves away from the balloon and the brakes need front_reverse_ratio above 0 (it is %s): '
                     'add --param front_reverse_ratio=0.74' % ratio)
        reply = ht.console(con, 'cam')
        log(reply.strip().splitlines()[0] if reply.strip() else '?')
        m = re.search(r'(\d+) rows/s', reply)
        if not m or int(m.group(1)) == 0:
            sys.exit("the flight needs the forward camera's rows, and 'cam' shows none:\n" + reply)
        t_cam = time.monotonic() + 3.0
        while link.camera is None and time.monotonic() < t_cam:
            time.sleep(0.05)
        if link.camera is None:
            sys.exit('no CAMERA packet over the link in 3 s: the flight reads the rows from here')
        if args.flight_bias_tau:
            send('param gyro_bias_tau_s %g' % ground_tau)
        t_wait = time.monotonic()
        while True:
            try:
                st = ht.console(con, 'hold')
            except TimeoutError:
                st = ''
            m = re.search(r'estimator: (\w+) \(still (\d+) of 60 s\)', st)
            if m and m.group(1) == 'converged':
                break
            if time.monotonic() - t_wait > args.wait:
                sys.exit('the estimator did not converge in %.0f s (it needs the gondola still for 60 s):\n%s'
                         % (args.wait, st))
            print('\rwaiting for the gondola to be still: %s of 60 s ' % (m.group(2) if m else '?'), end='',
                  flush=True)
            time.sleep(1.0)
        print()
        why = ht.floor_check(link)
        if why:
            sys.exit(why + '\nnothing started')
        log('the ToF sees the floor')
        if args.flight_bias_tau:
            tau_set = True
            send('param gyro_bias_tau_s %g' % args.flight_bias_tau)

        cap = bl.CaptureClient(link)
        if cap.info()['state'] == 1:
            cap.stop()
        cap.start(cap_s)
        cap_started = True
        t_cap = time.monotonic()
        time.sleep(ht.LEAD_S)

        tripped, peak = ht.make_kills(link, args.max_rate, args.max_tilt_rate, None)
        reply = ht.console(con, 'hold heading 0')
        t_hold = time.monotonic()
        log('%6.1f s  hold heading 0: %s' % (0.0, reply.strip().splitlines()[-1]))
        if 'refused' in reply:
            sys.exit('the firmware refused the hold:\n' + reply)
        started = True
        state['tof_t'] = time.monotonic()
        send('hold surge %g' % args.trim)
        send('hold altitude %.2f %.3f' % (args.alt, args.climb_rate))

        # 1. The climb and the look.
        log('%6.1f s  -- the climb to %.2f m, looking' % (clock(), args.alt))
        t_reached = None
        t_give_up = climb_s + 10.0     # the height counts as reached by then, whatever the ToF says
        following = False
        stop = None
        scan_k, t_next_step = -1, None   # -1: not scanning yet
        while True:
            stop = tick()
            if stop:
                break
            h = tof_m(link)
            if eye.seen(args.seen[0]) and not following:
                following = state['following'] = True
                events.append({'t': clock(), 'event': 'seen', 'tof_m': h, 'cam': eye.last})
                log('%6.1f s  the balloon is seen at a ToF of %s: %s' % (
                    clock(), '%.2f m' % h if h is not None else '?', eye.text()))
                send('hold follow')
            if t_reached is None and ((h is not None and h >= args.alt - 0.05) or clock() > t_give_up):
                t_reached = clock()
                state['at_height'] = True
                log('%6.1f s  at the height (ToF %s)' % (clock(), '%.2f m' % h if h is not None else '?'))
            if t_reached is not None and following:
                break
            if t_reached is not None and clock() > t_reached + args.look and (t_next_step is None
                                                                               or clock() >= t_next_step):
                scan_k += 1
                if scan_k >= len(scan):
                    stop = ('land', 'the balloon was not seen in %.0f s at the height%s' % (
                        args.look, ' nor in the scan' if scan else ''))
                    break
                if scan_k == 0:
                    # A glimpse (fewer frames than --seen) says which side to
                    # sweep first: target_cam's az is + to the right, the
                    # headings + to the left (163538 glimpsed it 20 deg right
                    # and swept left first).
                    if eye.last is not None and eye.last['az_deg'] > 0:
                        scan = [-hd for hd in scan]
                    log('%6.1f s  -- not seen ahead: the scan, %s first%s' % (
                        clock(), 'right' if scan[0] < 0 else 'left',
                        ' (a glimpse at az %+.1f deg)' % eye.last['az_deg'] if eye.last is not None else ''))
                send('hold heading %g' % scan[scan_k])
                t_next_step = clock() + args.scan_dwell
            time.sleep(0.02)
        phases.append((0.0, clock(), 'the climb and the look', 'climb', 0.0, args.trim))

        # 2 and 3. Following at the height, then the moves.
        if not stop:
            stop = phase('settle, following', 'follow', args.settle, 0.0)
            for label, m in plan:
                if stop:
                    break
                for k, (g, s) in enumerate(m):
                    stop = phase('%s: %+.1f gf' % (label, g), 'push' if k == 0 else 'brake', s, g)
                    if stop:
                        break
                if not stop:
                    stop = phase('still after %s' % label, 'hold', args.hold, 0.0)

        # Too close at any point: back off before the landing.
        if stop and stop[0] == 'near':
            near_why = stop[1]
            stop = backoff(near_why) or ('land', 'backed off: %s' % near_why)

        # 4. Always a landing, unless a kill.
        if stop and stop[0] == 'kill':
            killed = True
            ht.console(con, 'hold kill')
            log('%6.1f s  KILLED: %s' % (clock(), stop[1]))
        else:
            killed = landing(stop[1] if stop else 'the moves are done')
            landed = not killed
        log('peak rates seen from here: yaw %.0f deg/s, roll/pitch %.0f deg/s' % (peak['yaw'], peak['tilt']))
    except (Exception, KeyboardInterrupt) as e:
        # Whatever went wrong after the take-off (a refused command, a lost
        # reply, a bug here, Ctrl-C), the blimp lands before 'hold off'.
        if not started:
            raise
        log('%6.1f s  %s' % (clock(), 'Ctrl-C' if isinstance(e, KeyboardInterrupt) else
                             'error: %s: %s' % (e.__class__.__name__, e)))
        if not (killed or landed):
            try:
                killed = landing('after the %s' % ('Ctrl-C' if isinstance(e, KeyboardInterrupt) else 'error'))
                landed = not killed
            except BaseException as e2:     # a second Ctrl-C, or the link gone: the motors stop below
                log('%6.1f s  the landing did not finish (%s): the motors stop' % (clock(), e2.__class__.__name__))
    finally:
        try:
            log(ht.console(con, 'hold off').strip())
        except (TimeoutError, ConnectionError, OSError) as e:
            log('hold off did not answer (%s): MOTOR_STOP x3, the hold is KILLED until hold off' % e)
            for _ in range(3):
                link.stop_motors()
                time.sleep(0.05)
        if tau_set:
            try:
                log('param gyro_bias_tau_s %g: %s' % (ground_tau, ht.console(
                    con, 'param gyro_bias_tau_s %g' % ground_tau).strip().splitlines()[0]))
            except (TimeoutError, ConnectionError, OSError, IndexError) as e:
                log('gyro_bias_tau_s was not put back to %g s (%s): the next run sets it, or a reboot' % (ground_tau, e))
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

    path = args.out or bl.capture_path('target', args.folder)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    blimp_log.write_file(path, info['raw'], data)
    h, records = blimp_log.decode(info['raw'], data)
    rows = blimp_log.to_units(h, records)
    offset = (t_hold - t_cap) if started else ht.LEAD_S
    res = sq.phase_metrics(rows, phases, offset, front)
    cam_rows = [r for r in rows if r['cam0_seq']]
    seen = [r for r in cam_rows if r['cam0_detected']]
    print('  %-34s %8s %8s %17s %8s %15s' % ('phase', 'hdg rms', 'hdg max', 'height min/mean/max', 'yaw max',
                                              'front min/max'))
    for r in res:
        if 'note' in r:
            print('  %-34s %s' % (r['phase'], r['note']))
            continue
        print('  %-34s %6.1f d %6.1f d   %4.2f %4.2f %4.2f m %5.0f d/s   %+5.2f %+5.2f'
              % (r['phase'], r['heading_err_rms_deg'], r['heading_err_max_deg'], r['alt_min_m'], r['alt_mean_m'],
                 r['alt_max_m'], r['peak_yaw_rate_dps'], r['front_min'], r['front_max']))
    print('the balloon in %d of %d records with a camera row (%d of %d relayed frames seen from here)'
          % (len(seen), len(cam_rows), eye.hits, eye.total))
    meta = {'file': os.path.basename(path), 'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'source': 'target_test',
            'notes': args.notes, 'header': h, 'records': len(records), 'params': params,
            'test': {'alt_m': args.alt, 'climb_rate_mps': args.climb_rate, 'look_s': args.look, 'seen': args.seen,
                     'scan_deg': scan, 'scan_dwell_s': args.scan_dwell, 'near_px': args.near_px,
                     'near_el_deg': args.near_el, 'backoff': args.backoff,
                     'settle_s': args.settle, 'push': args.push, 'brake': args.brake, 'hold_s': args.hold,
                     'trim_gf': args.trim, 'flight_bias_tau_s': args.flight_bias_tau,
                     'land_rate_mps': args.land_rate, 'final_rate_mps': args.final_rate,
                     'phases': [list(p) for p in phases], 'events': events, 'hold_start_in_capture_s': offset,
                     'param': args.param, 'max_duty_pct': args.max_duty, 'deadband_pct': args.deadband,
                     'max_rate_dps': args.max_rate, 'max_tilt_rate_dps': args.max_tilt_rate,
                     'killed_from_ground': killed, 'landed': landed, 'telemetry_before': tel0, 'log': lines},
            'metrics': res}
    with open(os.path.splitext(path)[0] + '.json', 'w') as f:
        json.dump(meta, f, indent=1, default=str)
    print('%d records saved to %s, notes in %s' % (len(records), path, os.path.splitext(path)[0] + '.json'))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default=bl.BLIMP_IP)
    ap.add_argument('--port', type=int, default=bl.LINK_PORT)
    ap.add_argument('--console-port', type=int, default=bl.CONSOLE_PORT)
    ap.add_argument('--alt', type=float, default=2.0,
                    help='the height, m (default %%(default)s; at most %.1f, under where the ToF lost the floor)'
                         % ALT_MAX_M)
    ap.add_argument('--climb-rate', type=float, default=0.12, help='the climb, m/s (default %(default)s)')
    ap.add_argument('--look', type=float, default=5.0,
                    help='s at the height for the balloon to be seen before the landing (default %(default)s)')
    ap.add_argument('--scan', type=float, default=60.0,
                    help='the scan\'s half-width, deg each side (default %(default)s; 0: no scan, land)')
    ap.add_argument('--scan-step', type=float, default=15.0, help='its heading steps, deg (default %(default)s)')
    ap.add_argument('--scan-dwell', type=float, default=2.0,
                    help='s at each step (default %(default)s: about 7.5 deg/s, so the balloon stays in the '
                         '58 deg view for several seconds as it sweeps past)')
    ap.add_argument('--seen', type=int, nargs=2, default=[3, 10], metavar=('K', 'N'),
                    help='seen: K of the last N camera frames detect it (default 3 10, under a second: the '
                         'crumpled test balloon showed in 1 frame of 3 at 3 m, and no check has seen a false one)')
    ap.add_argument('--settle', type=float, default=5.0, help='s following at the height first (default %(default)s)')
    ap.add_argument('--push', type=float, nargs=2, default=[3.0, 1.8], metavar=('GF', 'S'),
                    help="the move's push toward the balloon (default 3 gf 1.8 s)")
    ap.add_argument('--brake', type=float, nargs=2, default=[3.0, 2.4], metavar=('GF', 'S'),
                    help='its counter-push (default 3 gf 2.4 s: with the push, aimed at about 40 cm; 3 s and '
                         '4 s went about 1 m in test 4c, and the distance goes with the square of the times)')
    ap.add_argument('--near-px', type=int, default=350,
                    help='too close: the patch\'s median at or over this, px (default %(default)s; 431-454 px came '
                         'a few seconds before the contact on 2026-11-02)')
    ap.add_argument('--near-el', type=float, default=15.0,
                    help='too close: the balloon this far above the camera\'s level, deg (default %(default)s; '
                         'the view ends at +22.5)')
    ap.add_argument('--backoff', type=float, nargs=2, default=[3.0, 3.0], metavar=('GF', 'S'),
                    help='too close: this far backwards first, then 3 s still and the landing (default 3 gf 3 s)')
    ap.add_argument('--hold', type=float, default=5.0, help='s still after each move (default %(default)s)')
    ap.add_argument('--trim', type=float, default=0.0,
                    help='gf added to every surge, as square_test.py\'s (-0.6 for the new battery type, -1 for the '
                         'earlier cells: docs/flight-tests.md)')
    ap.add_argument('--flight-bias-tau', type=float, default=600.0, metavar='S',
                    help="core/'s gyro-bias time constant in the air, as square_test.py's (default %(default)s; "
                         '0 leaves it alone)')
    ap.add_argument('--land-rate', type=float, default=0.1, help='the landing down to 0.30 m, m/s (default %(default)s)')
    ap.add_argument('--final-rate', type=float, default=0.03,
                    help='the landing from 0.30 m to the floor, m/s (default %(default)s)')
    ap.add_argument('--param', nargs='*', default=[], metavar='NAME=VALUE', help="'param' sets before the start")
    ap.add_argument('--max-duty', type=float, default=40.0)
    ap.add_argument('--deadband', type=float, default=10.0)
    ap.add_argument('--max-rate', type=float, default=62.0, help='kill above this yaw rate, deg/s (default %(default)s)')
    ap.add_argument('--max-tilt-rate', type=float, default=120.0,
                    help='kill above this roll or pitch rate, deg/s (default %(default)s)')
    ap.add_argument('--poll', type=float, default=1.0, help="s between status lines (default %(default)s)")
    ap.add_argument('--wait', type=float, default=180.0,
                    help="s to wait for core/'s estimator to converge (default %(default)s)")
    ap.add_argument('--folder', default='captures')
    ap.add_argument('--out', help='the .bin (default <folder>/target-<date>.bin)')
    ap.add_argument('--notes', default='')
    ap.add_argument('--go', action='store_true', help='really fly it (PROPS ON)')
    args = ap.parse_args(argv)

    if not (math.isfinite(args.alt) and 0.5 <= args.alt <= ALT_MAX_M):
        sys.exit('--alt is 0.5 .. %.1f m: the ToF lost the floor by 2.7 m (2026-11-02)' % ALT_MAX_M)
    if not (math.isfinite(args.climb_rate) and 0.02 <= args.climb_rate <= 0.5):
        sys.exit("--climb-rate is 0.02 .. 0.5 m/s (the firmware's 'hold altitude' rate)")
    if not (math.isfinite(args.scan) and 0.0 <= args.scan <= 180.0):
        sys.exit('--scan is 0 .. 180 deg')
    if not (math.isfinite(args.scan_step) and 5.0 <= args.scan_step <= 45.0):
        sys.exit('--scan-step is 5 .. 45 deg: the view is 58 deg wide, and a bigger step is a turn, not a scan')
    if not (math.isfinite(args.scan_dwell) and 0.5 <= args.scan_dwell <= 10.0):
        sys.exit('--scan-dwell is 0.5 .. 10 s')
    k, n = args.seen
    if not 1 <= k <= n <= 40:
        sys.exit('--seen is K N with 1 <= K <= N <= 40')
    for name, v, top in (('--look', args.look, 60.0), ('--settle', args.settle, 60.0), ('--hold', args.hold, 60.0)):
        if not (math.isfinite(v) and 0.0 <= v <= top):
            sys.exit('%s is 0 .. %g s' % (name, top))
    if not 20 <= args.near_px <= 5000:
        sys.exit('--near-px is 20 .. 5000 px')
    if not (math.isfinite(args.near_el) and 5.0 <= args.near_el <= 22.0):
        sys.exit('--near-el is 5 .. 22 deg (the view ends at +22.5)')
    for name, (gf, s) in (('--push', args.push), ('--brake', args.brake), ('--backoff', args.backoff)):
        if not (math.isfinite(gf) and 0.0 < gf <= 10.0 and math.isfinite(s) and 0.0 < s <= 10.0):
            sys.exit('%s is GF S with 0 < GF <= 10 gf and 0 < S <= 10 s' % name)
    if not (math.isfinite(args.trim) and -10.0 <= args.trim <= 10.0):
        sys.exit('--trim is -10 .. 10 gf')
    if args.flight_bias_tau and not (math.isfinite(args.flight_bias_tau) and 0.5 <= args.flight_bias_tau <= 600.0):
        sys.exit("--flight-bias-tau is 0.5 .. 600 s (the S3's 'param' range for gyro_bias_tau_s), or 0")
    if not (0.02 <= args.land_rate <= 0.5 and 0.01 <= args.final_rate <= 0.2):
        sys.exit('--land-rate is 0.02 .. 0.5 m/s and --final-rate 0.01 .. 0.2 m/s')
    if not (10.0 <= args.max_duty <= 100.0 and 0.0 <= args.deadband < args.max_duty):
        sys.exit('--max-duty is 10 .. 100 and --deadband 0 .. below it (percent)')
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
