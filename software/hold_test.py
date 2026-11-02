"""Closed-loop heading steps on the vehicle: the firmware's bench hold
('hold', control-loop increment 3's slice of 2026-10-30) runs core/'s own
yaw loop on the S3; this tool keeps the link alive, steps the setpoint,
watches the spin, records a capture and always ends with 'hold off'.

    python software/hold_test.py --steps 0 30 0 -30 0                 # dry run: prints the plan
    python software/hold_test.py --steps 0 30 0 -30 0 --dwell 8 --go \\
        --param yaw.kp=0.0008 yaw.kd=0.0008 --deadband 10
    python software/hold_test.py --steps 0 --dwell 20 --go            # hold where it points, 20 s
    python software/hold_test.py --follow --dwell 60 --go --surge 0.5 # turn toward the camera's balloon
    python software/hold_test.py --altitude --steps 1.3 1.6 1.3 --dwell 30 --go   # height steps, m

--altitude runs 'hold altitude' instead (the plan's C1): the steps are
heights above the floor in metres, the yaw is left free, and the report is
in metres from the capture's ToF. With --heading DEG it also holds that heading
throughout (the plan's C3, both loops); the other way round, --hold-alt M
holds a height during heading steps. It kills from here above --max-alt
(default the highest height + 0.5 m), as well as on --max-rate. On a thread
from above, the thread is a floor: the vehicle cannot sink below where it
hangs, so put every step above that height.

Whenever a height is held, the run ends with a landing (--no-land skips
it; land() and LAND_SLOW_M below): core/ moves the height down at
--land-rate to 0.30 m and at --final-rate on toward the floor, following
the descent instead of damping it (core 0.4.0), and the motors stop once
the ToF reads 0.09 m, about 2 cm above where the gondola stands. Without a
landing a 4.8 g heavy vehicle fell from 1 m and reached the floor at about
0.5 m/s (2026-10-31); the first landing, the setpoint stepped from here down
to 0.2 m, still dropped the last 0.13 m. --climb-rate approaches the first
height the same way instead of stepping to it.

--follow runs 'hold follow' instead of the steps: core/ turns the gondola
toward the forward camera's balloon (each new sighting sets the heading to
yaw + bearing) and holds where it was last seen; the report gives the
camera's row rate, how often the balloon was in view and the error while it
was. Check the bearing's sign first, motors off: software/cam_watch.py.

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
the moment the yaw rate in the 50 Hz SENSORS stream exceeds it, which is what
a sign error or too much gain looks like on a tether, and --max-tilt-rate
(default 120 deg/s) does the same for roll and pitch (a collision); the two
are apart since the gondola's harmless 2 Hz nodding tripped a single 62 deg/s
limit on two flights (make_kills()); the end of the schedule,
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
# A setpoint's window in the capture is placed from this computer's clock, so
# it can be a tick off the blimp's: one record of the next setpoint inside it
# read as a 30 deg overshoot (2026-10-31, T2). Trimmed at both ends.
EDGE_S = 0.1
# The landing (2026-11-01). core/ moves the height itself ('hold altitude <m>
# <m/s>', core 0.4.0), damping the error's rate, so the vehicle follows the
# descent; the setpoint resent from here every 0.25 s, with the damping on the
# climb rate, trailed by 0.3 m and the last 0.4 m sped up (150723). Down at
# --land-rate to LAND_SLOW_M, then at --final-rate toward the floor, and the
# motors stop once the ToF reads LAND_CUT_M, about 2 cm above where the
# gondola stands: it settles those on its trim instead of dropping 0.13 m.
LAND_SLOW_M = 0.30    # where the slow final approach begins
LAND_FLOOR_M = 0.07   # the ToF with the gondola standing on the floor (2026-10-31)
LAND_CUT_M = 0.09     # motors off at or below this ToF reading
LAND_WAIT_S = 4.0     # after the final ramp, if the ToF never reads LAND_CUT_M


def landing_time(sp0, land_rate, final_rate):
    """Seconds a landing from sp0 m takes at most."""
    fast = max(0.0, sp0 - LAND_SLOW_M) / land_rate
    slow = (min(sp0, LAND_SLOW_M) - LAND_FLOOR_M) / final_rate
    return fast + max(0.0, slow) + LAND_WAIT_S


def landing_s(args):
    """Seconds the landing adds after the schedule (0 without one)."""
    sp0 = args.steps[-1] if args.altitude else args.hold_alt
    if sp0 is None or not args.land:
        return 0.0
    return landing_time(sp0, args.land_rate, args.final_rate)


def make_kills(link, max_yaw_dps, max_tilt_dps, max_alt_m=None):
    """The kills on the 50 Hz SENSORS stream: tripped() gives the reason or
    None, and peak holds the largest yaw and tilt rates seen, deg/s. The
    stream's gyro is in the chip's axes, and the chip's z is the body's up
    (CONFIG_CTL_IMU_AXES "-Y +X +Z"), so yaw is z and roll and pitch are x and
    y. Split since 2026-11-01: a yaw rate is what a sign error or too much gain
    looks like, but the gondola also nods at about 2 Hz on its clip, and that
    pitch tripped a 62 deg/s kill on two flights that were holding (154445,
    171718)."""
    peak = {'yaw': 0.0, 'tilt': 0.0}

    def tripped():
        s = link.sensors
        if s and s['imu_valid']:
            gx, gy, gz = s['gyro']
            yaw, tilt = abs(gz), max(abs(gx), abs(gy))
            peak['yaw'] = max(peak['yaw'], yaw)
            peak['tilt'] = max(peak['tilt'], tilt)
            if yaw > max_yaw_dps:
                return 'yaw %.0f deg/s over --max-rate %.0f' % (yaw, max_yaw_dps)
            if tilt > max_tilt_dps:
                return 'roll/pitch %.0f deg/s over --max-tilt-rate %.0f' % (tilt, max_tilt_dps)
        if (max_alt_m is not None and s and s['tof_valid'] and s['tof_status'] == 0
                and s['tof_mm'] / 1000.0 > max_alt_m):
            return 'ToF %.2f m over --max-alt %.2f' % (s['tof_mm'] / 1000.0, max_alt_m)
        return None
    return tripped, peak


def land(con, link, sp0, land_rate, final_rate, tripped, log, clock, poll_s=1.0):
    """The landing from a held sp0 m (see LAND_SLOW_M): two 'hold altitude'
    moves, the kills watched, a status line every poll_s. Returns True when it
    ended on a kill (hold killed here), False when it landed or timed out;
    the caller's 'hold off' stops the motors either way. clock() is the
    caller's run time, for the log."""
    t_land = time.monotonic()
    fast_s = max(0.0, sp0 - LAND_SLOW_M) / land_rate
    end_s = landing_time(sp0, land_rate, final_rate)
    phase = 0
    next_poll = 0.0
    log('%6.1f s  landing from %.2f m: %sto the floor at %.2f m/s, motors off at a ToF of %.2f m'
        % (clock(), sp0, ('to %.2f m at %.2f m/s, then ' % (LAND_SLOW_M, land_rate)) if sp0 > LAND_SLOW_M else '',
           final_rate, LAND_CUT_M))
    while True:
        now = time.monotonic()
        el = now - t_land
        want = 1 if (sp0 > LAND_SLOW_M and el < fast_s) else 2
        if want != phase:
            phase = want
            cmd = ('hold altitude %.2f %.3f' % (LAND_SLOW_M, land_rate) if phase == 1 else
                   'hold altitude %.2f %.3f' % (LAND_FLOOR_M, final_rate))
            reply = console(con, cmd)
            log('%6.1f s  %s: %s' % (clock(), cmd, reply.strip().splitlines()[-1] if reply.strip() else '?'))
            if 'refused' in reply or 'not a number' in reply:
                log('%6.1f s  the landing was refused: motors off here' % clock())
                return False
        why = tripped()
        if why:
            console(con, 'hold kill')
            log('%6.1f s  KILLED: %s' % (clock(), why))
            return True
        s = link.sensors
        tof = s['tof_mm'] / 1000.0 if s and s['tof_valid'] and s['tof_status'] == 0 else None
        if tof is not None and tof <= LAND_CUT_M:
            log('%6.1f s  landed: ToF %.2f m, motors off' % (clock(), tof))
            return False
        if el >= end_s:
            log('%6.1f s  landing timed out: ToF %s, motors off' % (
                clock(), '%.2f m' % tof if tof is not None else 'not reading'))
            return False
        if now >= next_poll:
            log('%6.1f s  %s' % (clock(), hold_line(console(con, 'hold'))))
            next_poll = now + poll_s
        time.sleep(0.02)


def plan(steps, dwell, follow=False):
    """[(t0, t1, heading_deg)] from the moment the hold starts; with follow,
    one phase whose setpoint is None (the camera's target)."""
    if follow:
        return [(0.0, dwell, None)]
    return [(k * dwell, (k + 1) * dwell, h) for k, h in enumerate(steps)]


def console(con, line, log=None, tries=2):
    """One console command and its reply. A reply lost to a Wi-Fi drop is
    asked for once more: every command these tools send is idempotent (param
    sets, 'motor --max-duty', 'hold ...'), and on 2026-11-01 the link ran at
    -82 .. -86 dBm with the S3's antenna under the metallized envelope."""
    for k in range(tries):
        try:
            text = con.run(line, timeout=5.0, idle=2.0)
            break
        except TimeoutError:
            if k == tries - 1:
                raise
            con.drain()
    if log:
        for ln in text.strip().splitlines():
            log('    > ' + ln)
    return text


def floor_check(link, seconds=1.5, max_mm=300):
    """None if the ToF sees the floor through `seconds` before a hold arms,
    otherwise why not. A blinded ToF (status 2, 'signal fail', reading 1.4 to
    3 m with the gondola on the floor, 2026-11-02) leaves core/ without a
    height: its hold then commands no lift, which is safe, and the run's
    pushes go into a blimp sitting on the floor, which is a wasted run."""
    t_end = time.monotonic() + seconds
    seen = bad = 0
    last = worst = None
    while time.monotonic() < t_end:
        s = link.sensors
        if s is not None and s is not last:
            seen += 1
            if not (s['tof_valid'] and s['tof_status'] == 0 and s['tof_mm'] <= max_mm):
                bad += 1
                worst = s
            last = s
        time.sleep(0.02)
    if not seen:
        return 'no sensor packets from the blimp in %.1f s, so the ToF cannot be checked' % seconds
    if bad:
        return ('the ToF does not see the floor: %d of %d readings bad (one: %s, %d mm, status %d). Is something '
                'under the gondola, such as the black launch stand or a cable?'
                % (bad, seen, 'valid' if worst['tof_valid'] else 'invalid', worst['tof_mm'], worst['tof_status']))
    return None


def read_params(con, log=print):
    """The parameter listing, for the run's notes. The listing is the longest
    reply the console gives, and on a weak link it can fail to arrive whole
    however often it is asked for (-82 dBm on 2026-11-01): then
    front_reverse_ratio alone, which the runs check, and 'partial' in the notes."""
    try:
        return bl.read_params(con)
    except (ValueError, TimeoutError) as e:
        log('the parameter listing did not come through whole (%s); reading front_reverse_ratio alone' % e)
    reply = console(con, 'param front_reverse_ratio')
    m = re.search(r'^front_reverse_ratio\s+(\S+)', reply, re.M)
    return {'values': {'front_reverse_ratio': float(m.group(1))} if m else {}, 'partial': True}


def hold_line(text):
    """One compact line from 'hold''s reply."""
    m = re.search(r'heading: estimate ([-+\d.]+) deg.*?rate ([-+\d.]+) deg/s(?:; held at ([-+\d.]+) deg, '
                  r'error ([-+\d.]+) deg)?', text)
    o = re.search(r'throttle FL ([\d.]+) RL ([\d.]+) FR ([\d.]+) RR ([\d.]+)', text)
    f = re.search(r'FOLLOWING the target, last seen at ([-+\d.]+) deg, error ([-+\d.]+) deg', text)
    cam = re.search(r'camera: target at bearing ([-+\d.]+) deg', text)
    alt = re.search(r'altitude: estimate ([-+\d.]+) m \(([^)]*)\)(?:; held at ([-+\d.]+) m'
                    r'(?:, moving at [\d.]+ m/s, now at ([-+\d.]+) m)?)?', text)
    mode = re.search(r'mode (\w+)', text)
    parts = [mode.group(1) if mode else '?']
    if m:
        parts.append('hdg %+6.1f  rate %+6.1f' % (float(m.group(1)), float(m.group(2))))
        if m.group(3):
            parts.append('sp %+6.1f  err %+6.1f' % (float(m.group(3)), float(m.group(4))))
    if alt and alt.group(3):
        # a moving setpoint (core 0.4.0): the error is against where it is now
        h, sp = float(alt.group(1)), float(alt.group(4) or alt.group(3))
        parts.append('alt %5.2f  sp %5.2f%s  err %+5.2f%s' % (h, sp, '>' if alt.group(4) else ' ', sp - h,
                                                            '' if alt.group(2) == 'ToF' else ' (no ToF)'))
    if f:
        parts.append('seen at %+6.1f  err %+6.1f' % (float(f.group(1)), float(f.group(2))))
    elif 'FOLLOWING, nothing seen yet' in text:
        parts.append('nothing seen yet')
    if 'FOLLOWING' in text:
        parts.append('cam %+6.1f' % float(cam.group(1)) if cam else 'cam  none ')
    if o:
        parts.append('FL %.3f RL %.3f FR %.3f RR %.3f' % tuple(float(v) for v in o.groups()))
    return '  '.join(parts)


def metrics(rows, marks):
    """Per setpoint: overshoot past it, time into a 5 deg band and staying,
    the final error (mean, std) over the last 2 s, the peak throttle; from
    the capture's hold columns (blimp_log.py)."""
    out = []
    for k, (t0, t1, sp) in enumerate(marks):
        seg = [r for r in rows if t0 + EDGE_S <= r['t_s'] < t1 - EDGE_S and not math.isnan(r['hold_err_rad'])]
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
        # a kill ends the capture early: no last 2 s, so no final error
        mean = sum(tail) / len(tail) if tail else float('nan')
        std = math.sqrt(sum((e - mean) ** 2 for e in tail) / len(tail)) if tail else float('nan')
        peak = max(max(abs(r['throttle_%s' % m]) for m in blimp_log.MOTORS) for r in seg)
        out.append({'setpoint_deg': sp, 'start_error_deg': e0, 'overshoot_deg': over, 'settle_s': settle,
                    'final_error_deg': mean, 'final_std_deg': std, 'peak_throttle': peak})
    return out


def altitude_metrics(rows, marks, band_m=0.05):
    """Per height setpoint, from the capture's ToF (slant range; the tilt on
    a thread is a few degrees, under 1 % of the height): overshoot past it in
    m, time into a +-band_m band and staying, the final error (mean, std) over
    the last 2 s, the peak throttle. Records with a ToF status are left out."""
    out = []
    for t0, t1, sp in marks:
        seg = [r for r in rows if t0 + EDGE_S <= r['t_s'] < t1 - EDGE_S and r['mode'] != 'MANUAL'
               and r['tof_status'] == 0]
        if len(seg) < 10:
            out.append({'setpoint_m': sp, 'note': 'too few records'})
            continue
        err = [sp - r['tof_mm'] / 1000.0 for r in seg]
        e0 = err[0]
        over = max((-e if e0 > 0 else e) for e in err) if abs(e0) > 0.02 else float('nan')
        settle = float('nan')
        for i in range(len(err)):
            if all(abs(e) <= band_m for e in err[i:]):
                settle = seg[i]['t_s'] - t0
                break
        tail = [e for r, e in zip(seg, err) if r['t_s'] >= t1 - 2.0]
        mean = sum(tail) / len(tail) if tail else float('nan')
        std = math.sqrt(sum((e - mean) ** 2 for e in tail) / len(tail)) if tail else float('nan')
        peak = max(max(abs(r['throttle_%s' % m]) for m in blimp_log.MOTORS) for r in seg)
        out.append({'setpoint_m': sp, 'start_error_m': e0, 'overshoot_m': over, 'settle_s': settle,
                    'final_error_m': mean, 'final_std_m': std, 'peak_throttle': peak})
    return out


def follow_metrics(rows, t0, t1):
    """The follow phase: how many camera rows the loop saw and how many had
    the balloon, and the yaw error core/ steered on (to where the balloon was
    last seen) while it was in view: the capture's cam0 and hold columns."""
    seg = [r for r in rows if t0 <= r['t_s'] < t1 and r['mode'] != 'MANUAL']
    if len(seg) < 10:
        return {'note': 'too few records'}
    frames = detected = 0
    prev = None
    for r in seg:
        if r['cam0_seq'] != prev:
            frames += 1
            detected += 1 if r['cam0_detected'] else 0
            prev = r['cam0_seq']
    seen = [abs(math.degrees(r['hold_err_rad'])) for r in seg if r['cam0_detected']]
    seen.sort()
    dur = seg[-1]['t_s'] - seg[0]['t_s']
    return {'seconds': dur, 'camera_rows_per_s': frames / dur if dur > 0 else float('nan'),
            'rows_with_balloon_pct': 100.0 * detected / frames if frames else float('nan'),
            'abs_error_mean_deg': sum(seen) / len(seen) if seen else float('nan'),
            'abs_error_p90_deg': seen[int(0.9 * (len(seen) - 1))] if seen else float('nan'),
            'peak_rate_dps': max(abs(r['gyro_z_dps']) for r in seg),
            'peak_throttle': max(max(abs(r['throttle_%s' % m]) for m in blimp_log.MOTORS) for r in seg)}


def run(args):
    marks = plan(args.steps, args.dwell, args.follow)
    total = marks[-1][1]
    alt_held = args.altitude or args.hold_alt is not None
    cap_s = int(math.ceil(LEAD_S + total + landing_s(args))) + 3
    if args.follow:
        print('target follow, %.0f s, capture of %d s: the gondola turns toward the forward camera\'s balloon'
              % (total, cap_s))
    elif args.altitude:
        print('altitude hold, %d setpoints, %.0f s, capture of %d s, kill above %.2f m:'
              % (len(marks), total, cap_s, args.max_alt))
        for t0, t1, h in marks:
            print('  %6.1f .. %6.1f s  altitude %5.2f m' % (t0, t1, h))
        if args.heading is not None:
            print('heading held throughout at %+.1f deg' % args.heading)
    else:
        print('heading hold, %d setpoints, %.0f s, capture of %d s:' % (len(marks), total, cap_s))
        for t0, t1, h in marks:
            print('  %6.1f .. %6.1f s  heading %+6.1f deg' % (t0, t1, h))
        if args.hold_alt is not None:
            print('altitude held throughout at %.2f m, kill above %.2f m' % (args.hold_alt, args.max_alt))
    if args.climb_rate:
        print('the first height is approached at %.2f m/s' % args.climb_rate)
    if landing_s(args):
        print('then a landing: %.2f m/s down to %.2f m, %.2f m/s to the floor, motors off at a ToF of %.2f m '
              '(at most %.0f s)' % (args.land_rate, LAND_SLOW_M, args.final_rate, LAND_CUT_M, landing_s(args)))
    print('kills: yaw over %.0f deg/s, roll or pitch over %.0f deg/s%s'
          % (args.max_rate, args.max_tilt_rate, ', ToF over %.2f m' % args.max_alt if alt_held else ''))
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
        params = read_params(con, log)
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
        if args.altitude or args.hold_alt is not None:
            why = floor_check(link)
            if why:
                sys.exit(why + '\nnothing started')
            log('the ToF sees the floor')

        cap = bl.CaptureClient(link)
        if cap.info()['state'] == 1:
            cap.stop()
        cap.start(cap_s)
        cap_started = True
        t_cap = time.monotonic()
        time.sleep(LEAD_S)

        k = -1
        next_poll = 0.0
        tripped, peak = make_kills(link, args.max_rate, args.max_tilt_rate, args.max_alt if alt_held else None)

        while True:
            now = time.monotonic()
            el = now - t_hold if t_hold is not None else 0.0
            want = next((i for i, m in enumerate(marks) if m[0] <= el < m[1]), None)
            if want is None:
                break
            if want != k:
                k = want
                cmd = ('hold follow' if marks[k][2] is None else
                       'hold %s %g' % ('altitude' if args.altitude else 'heading', marks[k][2]))
                if k == 0 and args.altitude and args.climb_rate:
                    cmd += ' %g' % args.climb_rate
                reply = console(con, cmd)
                if t_hold is None:
                    t_hold = time.monotonic()
                    started = 'refused' not in reply
                log('%6.1f s  %s: %s' % (time.monotonic() - t_hold, cmd, reply.strip().splitlines()[-1]))
                if 'refused' in reply:
                    sys.exit('the firmware refused the hold:\n' + reply)
                if k == 0 and args.altitude and args.heading is not None:
                    reply = console(con, 'hold heading %g' % args.heading)
                    log('%6.1f s  hold heading %g: %s' % (time.monotonic() - t_hold, args.heading,
                                                         reply.strip().splitlines()[-1]))
                    if 'refused' in reply:
                        sys.exit('the firmware refused the heading:\n' + reply)
                if k == 0 and args.hold_alt is not None:
                    acmd = 'hold altitude %g' % args.hold_alt + (' %g' % args.climb_rate if args.climb_rate else '')
                    reply = console(con, acmd)
                    log('%6.1f s  %s: %s' % (time.monotonic() - t_hold, acmd, reply.strip().splitlines()[-1]))
                    if 'refused' in reply:
                        sys.exit('the firmware refused the altitude:\n' + reply)
                if k == 0 and args.surge:
                    reply = console(con, 'hold surge %g' % args.surge)
                    log('%6.1f s  hold surge %g: %s' % (time.monotonic() - t_hold, args.surge,
                                                       reply.strip().splitlines()[-1]))
                    if 'refused' in reply:
                        sys.exit('the firmware refused the surge:\n' + reply)
            why = tripped()
            if why:
                killed = True
                console(con, 'hold kill')
                log('%6.1f s  KILLED: %s' % (el, why))
                break
            if now >= next_poll:
                log('%6.1f s  %s' % (el, hold_line(console(con, 'hold'))))
                next_poll = now + args.poll
            time.sleep(0.02)

        # the landing (land(), LAND_SLOW_M above): core/ moves the height to
        # the floor, and the motors stop about 2 cm above it
        if started and not killed and landing_s(args):
            sp0 = args.steps[-1] if args.altitude else args.hold_alt
            killed = land(con, link, sp0, args.land_rate, args.final_rate, tripped, log,
                          lambda: time.monotonic() - t_hold, args.poll)
        log('peak rates seen from here: yaw %.0f deg/s, roll/pitch %.0f deg/s' % (peak['yaw'], peak['tilt']))
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

    path = args.out or bl.capture_path('hold-altitude' if args.altitude else 'hold-heading', args.folder)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    blimp_log.write_file(path, info['raw'], data)
    h, records = blimp_log.decode(info['raw'], data)
    rows = blimp_log.to_units(h, records)
    offset = (t_hold - t_cap) if started else LEAD_S
    marks_cap = [(t0 + offset, t1 + offset, sp) for t0, t1, sp in marks]
    if args.follow:
        res = follow_metrics(rows, marks_cap[0][0], marks_cap[0][1])
        if 'note' in res:
            print('  follow: %s' % res['note'])
        else:
            print('  follow, %.0f s: camera %.1f rows/s, balloon in %.0f %% of them; |error| while in view '
                  'mean %.1f deg, 90 %% under %.1f deg; peak rate %.0f deg/s, peak throttle %.2f'
                  % (res['seconds'], res['camera_rows_per_s'], res['rows_with_balloon_pct'],
                     res['abs_error_mean_deg'], res['abs_error_p90_deg'], res['peak_rate_dps'],
                     res['peak_throttle']))
        res = [res]
    elif args.altitude:
        res = altitude_metrics(rows, marks_cap)
        for r in res:
            if 'note' in r:
                print('  %5.2f m: %s' % (r['setpoint_m'], r['note']))
                continue
            print('  %5.2f m: from %+5.2f, overshoot %5.2f m, inside 5 cm after %4.1f s, final %+5.2f +- %.2f m, '
                  'peak throttle %.2f' % (r['setpoint_m'], r['start_error_m'], r['overshoot_m'], r['settle_s'],
                                          r['final_error_m'], r['final_std_m'], r['peak_throttle']))
        if args.heading is not None:
            # the heading loop's error over the whole run, past its first 5 s
            err = [abs(math.degrees(r['hold_err_rad'])) for r in rows
                   if marks_cap[0][0] + 5.0 <= r['t_s'] < marks_cap[-1][1] - EDGE_S
                   and not math.isnan(r['hold_err_rad'])]
            if err:
                hd = {'abs_error_rms_deg': math.sqrt(sum(e * e for e in err) / len(err)),
                      'abs_error_max_deg': max(err)}
                print('  heading %+.1f deg: |error| rms %.1f deg, max %.1f deg'
                      % (args.heading, hd['abs_error_rms_deg'], hd['abs_error_max_deg']))
                res.append(dict(hd, heading_deg=args.heading))
    else:
        res = metrics(rows, marks_cap)
    for r in ([] if args.follow or args.altitude else res):
        if 'note' in r:
            print('  %+6.1f deg: %s' % (r['setpoint_deg'], r['note']))
            continue
        print('  %+6.1f deg: from %+6.1f, overshoot %5.1f deg, inside 5 deg after %4.1f s, final %+5.1f +- %.1f deg, '
              'peak throttle %.2f' % (r['setpoint_deg'], r['start_error_deg'], r['overshoot_deg'], r['settle_s'],
                                      r['final_error_deg'], r['final_std_deg'], r['peak_throttle']))
    if args.hold_alt is not None:
        # what each heading step did to the height (the plan's C3 coupling);
        # the first setpoint's window from 15 s on, past the climb off the floor
        for k, (t0, t1, sp) in enumerate(marks_cap):
            alt = [r['tof_mm'] / 1000.0 for r in rows
                   if t0 + (15.0 if k == 0 else EDGE_S) <= r['t_s'] < t1 - EDGE_S
                   and r['tof_status'] == 0 and r['mode'] != 'MANUAL']
            if not alt:
                continue
            mean = sum(alt) / len(alt)
            dev = max(abs(a - args.hold_alt) for a in alt)
            print('  %+6.1f deg: altitude %.2f m mean, %.2f .. %.2f, |off| up to %.2f m'
                  % (sp, mean, min(alt), max(alt), dev))
            res[k]['altitude_mean_m'] = mean
            res[k]['altitude_max_off_m'] = dev
    meta = {'file': os.path.basename(path), 'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'source': 'hold_test',
            'notes': args.notes, 'header': h, 'records': len(records), 'params': params,
            'test': {'follow': args.follow, 'altitude': args.altitude,
                     'steps_deg': None if args.follow or args.altitude else args.steps,
                     'steps_m': args.steps if args.altitude else None, 'max_alt_m': args.max_alt,
                     'heading_deg': args.heading, 'hold_alt_m': args.hold_alt,
                     'land_rate_mps': args.land_rate if landing_s(args) else None,
                     'final_rate_mps': args.final_rate if landing_s(args) else None,
                     'climb_rate_mps': args.climb_rate, 'max_tilt_rate_dps': args.max_tilt_rate,
                     'dwell_s': args.dwell, 'hold_start_in_capture_s': offset,
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
    ap.add_argument('--steps', type=float, nargs='+', default=[0.0],
                    help='heading setpoints, deg (default 0); with --altitude, heights in m')
    ap.add_argument('--follow', action='store_true',
                    help="'hold follow' for --dwell s instead of the steps: turn toward the forward camera's "
                         "balloon (check the sign first with cam_watch.py)")
    ap.add_argument('--altitude', action='store_true',
                    help="'hold altitude' steps instead of heading steps (the plan's C1); --steps in m")
    ap.add_argument('--max-alt', type=float,
                    help='with --altitude, kill above this ToF height, m (default the highest step + 0.5)')
    ap.add_argument('--heading', type=float,
                    help='with --altitude, also hold this heading throughout, deg (the plan\'s C3)')
    ap.add_argument('--hold-alt', type=float,
                    help='with heading steps, also hold this height throughout, m (the plan\'s C3)')
    ap.add_argument('--no-land', dest='land', action='store_false',
                    help='end without the landing ramp (the motors stop where the vehicle is)')
    ap.add_argument('--land-rate', type=float, default=0.1,
                    help='the landing down to %.2f m, m/s (default %%(default)s)' % LAND_SLOW_M)
    ap.add_argument('--final-rate', type=float, default=0.03,
                    help='the landing from %.2f m to the floor, m/s (default %%(default)s)' % LAND_SLOW_M)
    ap.add_argument('--climb-rate', type=float, default=0.0,
                    help='approach the first height at this rate, m/s; 0 (the default) steps to it, as the '
                         'tuning runs did')
    ap.add_argument('--dwell', type=float, default=8.0, help='s at each setpoint (default %(default)s)')
    ap.add_argument('--param', nargs='*', default=[], metavar='NAME=VALUE', help="'param' sets before the start")
    ap.add_argument('--deadband', type=float,
                    help="'motor --deadband' before the start, percent (a boot gives 10 since 2026-11-01)")
    ap.add_argument('--max-duty', type=float,
                    help="'motor --max-duty' before the start, percent (the team: 40, a boot's default since "
                         "2026-11-01; given anyway, it also undoes a plain driver left by the thrust stand)")
    ap.add_argument('--surge', type=float, default=0.0,
                    help="'hold surge' for the whole run, gf (default 0): keeps both front motors turning, so "
                         "a yaw correction speeds one up and slows the other instead of starting one from rest; "
                         "negative pushes backwards (core 0.4.0, with front_reverse_ratio > 0), the counter-push "
                         "for the rear pair's forward lean")
    ap.add_argument('--max-rate', type=float, default=120.0,
                    help='kill above this yaw rate, deg/s (default %(default)s); until 2026-11-01 any axis')
    ap.add_argument('--max-tilt-rate', type=float, default=120.0,
                    help='kill above this roll or pitch rate, deg/s (default %(default)s): the gondola nods '
                         'at about 2 Hz on its clip without harm, and a wall hit gave 88 deg/s')
    ap.add_argument('--poll', type=float, default=1.0, help="s between 'hold' status lines (default %(default)s)")
    ap.add_argument('--wait', type=float, default=90.0,
                    help="s to wait for core/'s estimator to converge before giving up (default %(default)s)")
    ap.add_argument('--folder', default='captures')
    ap.add_argument('--out', help='the .bin (default <folder>/hold-heading-<date>.bin)')
    ap.add_argument('--notes', default='')
    ap.add_argument('--go', action='store_true', help='really run it (PROPS ON)')
    args = ap.parse_args(argv)
    if args.altitude:
        if args.follow:
            sys.exit('--altitude and --follow are separate runs')
        # the firmware's own range (HOLD_ALT_MIN_M .. HOLD_ALT_MAX_M in ctl_task.c)
        if any(not math.isfinite(h) or not 0.2 <= h <= 3.0 for h in args.steps):
            sys.exit('heights are 0.2 .. 3.0 m')
        if args.max_alt is None:
            args.max_alt = max(args.steps) + 0.5
        if args.heading is not None and (not math.isfinite(args.heading) or abs(args.heading) > 180):
            sys.exit('--heading is -180 .. 180 deg')
        if args.hold_alt is not None:
            sys.exit('--hold-alt goes with heading steps; with --altitude the heights are --steps')
    elif args.heading is not None:
        sys.exit('--heading goes with --altitude; heading steps are --steps')
    elif any(not math.isfinite(h) or abs(h) > 180 for h in args.steps):
        sys.exit('headings are -180 .. 180 deg')
    if args.hold_alt is not None:
        if args.follow:
            sys.exit('--hold-alt and --follow are separate runs')
        if not math.isfinite(args.hold_alt) or not 0.2 <= args.hold_alt <= 3.0:
            sys.exit('--hold-alt is 0.2 .. 3.0 m')
        if args.max_alt is None:
            args.max_alt = args.hold_alt + 0.5
    if not math.isfinite(args.land_rate) or not 0.02 <= args.land_rate <= 0.5:
        sys.exit('--land-rate is 0.02 .. 0.5 m/s')
    if not math.isfinite(args.final_rate) or not 0.01 <= args.final_rate <= 0.2:
        sys.exit('--final-rate is 0.01 .. 0.2 m/s')
    if not math.isfinite(args.climb_rate) or not (args.climb_rate == 0.0 or 0.02 <= args.climb_rate <= 0.5):
        sys.exit("--climb-rate is 0 (a step) or 0.02 .. 0.5 m/s (the firmware's 'hold altitude' rate)")
    if args.climb_rate and not (args.altitude or args.hold_alt is not None):
        sys.exit('--climb-rate goes with a held height (--altitude or --hold-alt)')
    if not (args.max_rate > 0 and args.max_tilt_rate > 0):
        sys.exit('--max-rate and --max-tilt-rate are > 0 deg/s')
    if args.dwell <= 0 or args.dwell * (1 if args.follow else len(args.steps)) > 300:
        sys.exit('--dwell must be > 0 and the whole run under 300 s (the capture buffer)')
    if not math.isfinite(args.surge) or not -30.0 <= args.surge <= 30.0:
        sys.exit("--surge is -30 .. 30 gf (the firmware's 'hold surge' range)")
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
