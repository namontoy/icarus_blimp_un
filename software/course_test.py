#!/usr/bin/env python3
"""The balloon course (2026-11-03): the race in miniature on the bench hold.
Balloons in order: for each, look for it where the course puts it, turn the
nose to it ('hold follow'), close in with short moves, touch it, back away,
and turn to the next; after the last, the landing. Every end is a landing,
as in target_test.py.

    python software/course_test.py --close-px 1500 --trim -0.6 --param $(cat software/bench_params.txt)        # the plan
    python software/course_test.py --close-px 1500 --trim -0.6 --param $(cat software/bench_params.txt) --go   # PROPS ON
    python software/course_test.py --selftest      # the sequence against a stand-in, no hardware

The course (--course X,Y ...): each balloon's floor point in metres from
where the gondola stands at the start, x along its nose, y to its left (+
counter-clockwise, as 'hold heading'). The default is the room of 2026-11-03:
balloon 1 2 m straight ahead, balloon 2 2 m to the right of it, both centred
about 1.2 m up (docs/flight-tests.md, "The course room"). Nothing on board
measures position, so the course only says where to look, and each touch is
a position fix: after a balloon the blimp should be --standoff m short of it
on the bearing it last saw it on, and the next balloon's bearing from there
is the heading it turns to. The room's air moves the blimp 1-2 m in 40 s
(2026-11-01), so the plan's own bearings (from the start, and from the
straight line between balloons) can be tens of degrees off by then.

For each balloon:

1. Look. Balloon 1 during the climb to --alt; each next one after 'hold
   heading' turns the nose to its bearing (done within TURNED_DEG, or after
   --turn-time s). A detection counts only when its bearing from the start's
   nose (the ground's heading, below, plus core/'s bearing of it) is within
   --window deg of where the balloon should be: the balloon just touched,
   the next one or a stray red thing is no detection. Seen is K of the last
   N frames (--seen). Not seen --look s after the height or the turn, the
   scan around that bearing, as target_test.py's; not seen by its end, the
   landing.
2. Seen: 'hold follow' (the S3 turns the nose to the camera's balloon), the
   window narrows to TRACK_DEG around the detections, --settle s following.
3. The approach, in pulses, each test 5's leg with the camera ending its
   coast: --push, then a coast of at most --coast s with the nose following,
   then --brake and --hold s still; at most --max-pulses. 3 gf for 1.8 s
   gives about 0.15 m/s (reasoned from test 5), so a pulse covers about
   1.2 m. Not 40 cm steps: their 5 cm/s on average is what the room's air
   drifts the blimp sideways (1-2 m in 40 s on 2026-11-01), and the
   selftest's blimp never reached the balloon that way. A pulse starts only
   on a balloon seen in the last --lost s; lost longer, it waits --look s for
   it (core/ holds the nose where it was last seen), then looks once more
   around there, then lands.
4. Close: target_test.py's proximity guard, here the cue to touch: the
   patch's median at --close-px, or the balloon --close-el deg above the
   camera's level, or lost right after big patches. In the push or the
   coast, the blimp coasts on into the balloon for --touch s (the envelope
   reaches past the camera, so the camera's last good view comes before the
   contact), then the brake for what was pushed; from about rest (in the
   brake, still), a push first. Then the brake's mirror away from the
   balloon, and --hold s still. The close moment is the balloon's time in
   the report, the nearest the ground gets to the race's lap times; the
   contact itself is not measured, so the team says whether it touched.

--close-px has no default: it is the patch the course's balloons give at
about 0.4 m more than the distance at which the envelope touches them (the
--touch coast's 2.5 s at 0.15 m/s), from the camera check
(docs/flight-tests.md, C0). The 350 px of the crumpled
34 cm balloon (target_test.py) would fire metres away from a whole taut one.
One value for every balloon, or one per balloon.

The heading on the ground: no live yaw comes over the link, so it is the last
'hold' poll's estimate (every --poll s) plus the SENSORS packets' gyro
integrated since.

The rest is target_test.py's: the kills (yaw over --max-rate, roll or pitch
over --max-tilt-rate: 'hold kill'), the landing on the ToF over --alt + 0.3
m or silent for 1 s, on Ctrl-C and on any error here, --trim on every surge,
--flight-bias-tau 600 s, the capture and its .json.

This is a test harness, not the race's guidance (that belongs in core/): it
sequences the bench hold's setpoints from the ground on what the camera
reports. Standard library only.
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
import square_test as sq  # noqa: E402
import target_test as tt  # noqa: E402

# The room of 2026-11-03 (the team's photo): the start, 2 m to balloon 1,
# then 2 m to balloon 2 on the blimp's right, at about a right angle; both
# balloons centred about 1.2 m up.
COURSE_ROOM = ((2.0, 0.0), (2.0, -2.0))
TURNED_DEG = 10.0     # the turn to the next balloon is done within this
TRACK_DEG = 25.0      # once seen, a detection farther than this from the last is another red thing
NEAR_TOUCHED = 0.5    # until the next balloon is seen, a patch this fraction of --close-px is the one just touched
LEG_MIN_M = 1.0       # a balloon closer than this to where the last one leaves the blimp is not a course


def point(text):
    """'X,Y' in metres, for --course."""
    try:
        x, y = (float(v) for v in text.split(','))
    except ValueError:
        raise argparse.ArgumentTypeError('%r is not X,Y in metres' % text)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise argparse.ArgumentTypeError('%r is not X,Y in metres' % text)
    return (x, y)


def legs(course, standoff):
    """Per balloon, (bearing_deg, distance_m, (x, y)): the balloon's bearing
    (+ left of the start's nose) and distance from where the blimp should be
    when it looks for it, which is the start for the first and --standoff m
    short of the previous balloon, on the line it came in on, for the rest."""
    out = []
    px, py = 0.0, 0.0
    for bx, by in course:
        dx, dy = bx - px, by - py
        d = math.hypot(dx, dy)
        out.append((math.degrees(math.atan2(dy, dx)), d, (px, py)))
        f = max(0.0, d - standoff) / d if d > 0 else 0.0
        px, py = px + dx * f, py + dy * f
    return out


def bearing_after(course, k, seen_deg, standoff):
    """Balloon k+1's bearing, deg, after balloon k (0-based k): the touch is
    a position fix, the blimp --standoff m short of balloon k on the bearing
    it last saw it on, wherever the air took it on the way there."""
    (bx, by), (nx, ny) = course[k - 1], course[k]
    h = math.radians(seen_deg)
    px, py = bx - standoff * math.cos(h), by - standoff * math.sin(h)
    return math.degrees(math.atan2(ny - py, nx - px))


def close_px(args, k):
    return args.close_px[k] if len(args.close_px) > 1 else args.close_px[0]


class Heading:
    """core/'s heading as the ground can know it, deg, + counter-clockwise
    from above: no live yaw comes over the link, so it is the last 'hold'
    poll's estimate plus the SENSORS packets' gyro z integrated since (the
    chip's z is the body's up, CONFIG_CTL_IMU_AXES "-Y +X +Z"). The stream's
    gyro still has its bias, a fraction of a degree a second, which adds
    under a degree between polls a second apart; across a gap of half a
    second in the packets the next poll puts it right instead."""

    def __init__(self, link):
        self.link = link
        self.deg = None
        self.t_us = None

    def reset(self, deg=0.0):
        """From a known heading: 0 when the hold arms ('heading 0 = where the
        nose points now'), or a poll's estimate."""
        s = self.link.sensors
        self.deg = deg
        self.t_us = s['t_us'] if s else None

    def poll(self, reply):
        m = re.search(r'heading: estimate ([-+\d.]+) deg', reply)
        if m:
            self.reset(float(m.group(1)))

    def update(self):
        s = self.link.sensors
        if self.deg is None or s is None or s['t_us'] == self.t_us:
            return
        dt = ((s['t_us'] - self.t_us) & 0xFFFFFFFF) / 1e6 if self.t_us is not None else 1.0
        self.t_us = s['t_us']
        if s['imu_valid'] and dt < 0.5:
            self.deg = sq.wrap180(self.deg + s['gyro'][2] * dt)


class Track(tt.Sightings):
    """target_test.py's Sightings for one balloon of several. A detection
    counts only when its bearing from the start's nose (the ground's heading
    plus core/'s bearing of it, + left) lies within `window` deg of where this
    balloon should be; anything else counts as no detection. Once seen
    (lock()), the expectation moves with each detection, within TRACK_DEG.
    Only the camera's main blob is looked at: it is the one the S3's follow
    turns to."""

    def __init__(self, link, n, heading, expect_deg, window_deg, touched=None):
        super().__init__(link, n)
        self.heading = heading
        self.expect = expect_deg
        self.window = window_deg
        # The balloon just touched, until this one is seen: (bearing deg,
        # close px) or None. Any patch within TRACK_DEG of where it was last
        # seen is that balloon: a course turns at each balloon, so the next
        # one is not meant to be there, and if it is, the nearer balloon
        # hides it (the camera reports the biggest patch). So is a patch of
        # NEAR_TOUCHED x close px or more anywhere, under about a metre away.
        # The window alone did not do: the selftest's air carried the blimp
        # past balloon 1 until it hung 7-13 deg from balloon 2, inside it.
        self.touched = touched
        self.locked = False
        self.rejected = 0
        self.last_bearing = None     # deg, of self.last when it came in
        self.last_rejected = None    # deg, the latest red thing outside the window
        self.t_last = None           # time.monotonic() of the latest detection that counted

    def bearing(self, c):
        h = self.heading.deg
        return None if h is None else sq.wrap180(h + c['bearing_sign'] * c['az_deg'])

    def update(self):
        c = self.link.camera
        if c is None or not c['row'] or c['frame_seq'] == self.seq:
            return
        self.seq = c['frame_seq']
        ok = bool(c['ok'])
        b = self.bearing(c) if ok else None
        if b is not None and (abs(sq.wrap180(b - self.expect)) > self.window or self.was_touched(b, c['area_px'])):
            ok = False
            self.rejected += 1
            self.last_rejected = b
        self.frames.append(ok)
        self.total += 1
        if ok:
            self.last = c
            self.last_bearing = b
            self.hits += 1
            self.areas.append(c['area_px'])
            self.t_last = time.monotonic()
            if self.locked and b is not None:
                self.expect = b

    def was_touched(self, b, area):
        if self.locked or self.touched is None:
            return False
        deg, px = self.touched
        return area >= NEAR_TOUCHED * px or abs(sq.wrap180(b - deg)) <= TRACK_DEG

    def lock(self):
        self.locked = True
        self.window = min(self.window, TRACK_DEG)
        if self.last_bearing is not None:
            self.expect = self.last_bearing

    def recent(self, seconds):
        return self.t_last is not None and time.monotonic() - self.t_last <= seconds

    def text(self):
        s = super().text()
        if self.rejected:
            s += '; %d outside the window (latest at %+.0f deg)' % (self.rejected, self.last_rejected)
        return s


class Flight:
    """One course on the bench hold, from 'hold heading 0' to the end of the
    landing: the sequence in the module's docstring. run() does what comes
    before (the parameters, the checks, the capture) and after ('hold off',
    the download); selftest() flies it against a stand-in."""

    def __init__(self, link, con, args, course_legs, log):
        self.link, self.con, self.args, self.legs, self.log = link, con, args, course_legs, log
        self.climb_s = max(0.0, args.alt - ht.LAND_FLOOR_M) / args.climb_rate
        self.heading = Heading(link)
        self.track = None
        self.tripped = lambda: None
        self.peak = {'yaw': 0.0, 'tilt': 0.0}
        self.t_hold = None
        self.started = self.killed = self.landed = False
        self.following = self.at_height = False   # following: this balloon is seen, the close cue armed
        self.s3_follow = False                    # 'hold follow' is on
        self.touched = None                       # (last bearing deg, close px) of the balloon just touched
        self.close_px = 0
        self.ran = 0.0          # s the last phase ran
        self.phases = []        # (t0, t1, label, kind, heading, surge) as flown, for square_test's metrics
        self.events = []
        self.closes = []        # (balloon, t): the course's times
        self.end = None         # why the flight ended
        self.tof_t = self.next_poll = 0.0

    def clock(self):
        return time.monotonic() - self.t_hold if self.t_hold is not None else 0.0

    def send(self, cmd):
        reply = ht.console(self.con, cmd)
        last = reply.strip().splitlines()[-1] if reply.strip() else '?'
        self.log('%6.1f s  %s: %s' % (self.clock(), cmd, last))
        if 'refused' in reply or 'not a number' in reply or 'Unrecognized' in reply:
            raise RuntimeError('the firmware refused %r: %s' % (cmd, last))
        return reply

    def aim(self, deg):
        """The nose held at deg, the S3's follow off. The heading goes first:
        it waits while the follow is on and holds the moment the follow goes
        off, so the nose never swings back to an older heading."""
        self.send('hold heading %g' % deg)
        if self.s3_follow:
            self.send('hold follow off')
            self.s3_follow = False

    def follow(self):
        if not self.s3_follow:
            self.send('hold follow')
            self.s3_follow = True

    def tick(self, guard=True):
        """The kills, the landing triggers, the close cue (following, at the
        height) and the status line, every 20 ms of the flight. None,
        ('kill', why), ('land', why) or ('close', why)."""
        self.heading.update()
        if self.track is not None:
            self.track.update()
        why = self.tripped()
        if why:
            return ('kill', why)
        if guard and self.following and self.at_height:
            why = self.track.near(self.close_px, self.args.close_el)
            if why:
                return ('close', why)
        h = tt.tof_m(self.link)
        now = time.monotonic()
        if h is not None:
            self.tof_t = now
            if h > self.args.alt + tt.HIGH_M:
                return ('land', 'the ToF reads %.2f m, over %.2f m' % (h, self.args.alt + tt.HIGH_M))
        elif now - self.tof_t > tt.TOF_LOST_S:
            return ('land', 'the ToF has not read for %.0f s' % tt.TOF_LOST_S)
        if now >= self.next_poll:
            try:
                reply = ht.console(self.con, 'hold')
                self.heading.poll(reply)
                self.log('%6.1f s  %s  cam %s' % (self.clock(), ht.hold_line(reply),
                                                  self.track.text() if self.track else '-'))
            except TimeoutError:
                self.log('%6.1f s  no answer to the status poll (a Wi-Fi drop?)' % self.clock())
            self.next_poll = now + self.args.poll
        return None

    def phase(self, label, kind, seconds, surge, guard=True, until=None):
        """--trim + surge gf for `seconds` s; None, or the stop that ended it
        early (self.ran says how long it ran). until() true ends it early
        with None."""
        t0 = self.clock()
        self.log('%6.1f s  -- %s' % (t0, label))
        self.send('hold surge %g' % (surge + self.args.trim))
        stop = None
        while self.clock() < t0 + seconds and not (until and until()):
            stop = self.tick(guard)
            if stop:
                break
            time.sleep(0.02)
        self.ran = self.clock() - t0
        self.phases.append((t0, self.clock(), label, kind, self.track.expect if self.track else 0.0,
                            surge + self.args.trim))
        return stop

    def find(self, k, expect, again=False):
        """Look for balloon k+1 around `expect` deg until it is seen and the
        nose follows it: None, or a stop. The first during the climb; the
        others (and a second look, `again`) after turning to `expect`."""
        a = self.args
        n = k + 1
        self.track = Track(self.link, a.seen[1], self.heading, expect, a.window, touched=self.touched)
        self.following = False
        t0 = self.clock()
        climbing = k == 0 and not again
        if climbing:
            self.log('%6.1f s  -- the climb to %.2f m, looking for balloon 1 at %+.0f deg' % (t0, a.alt, expect))
            give_up = t0 + self.climb_s + 10.0     # the height counts as reached by then, whatever the ToF says
        else:
            self.log('%6.1f s  -- %s balloon %d at %+.0f deg' % (t0, 'looking again for' if again else 'the turn to',
                                                                 n, expect))
            self.aim(expect)
            give_up = t0 + a.turn_time
        scan = [expect + o for o in tt.scan_headings(a.scan, a.scan_step)]
        t_ready, scan_k, t_next = None, -1, None
        while True:
            stop = self.tick()
            if stop:
                return stop
            if not self.following and self.track.seen(a.seen[0]):
                self.track.lock()
                self.following = True
                h = tt.tof_m(self.link)
                self.events.append({'t': self.clock(), 'event': 'seen', 'balloon': n, 'tof_m': h,
                                    'heading_deg': self.heading.deg, 'bearing_deg': self.track.expect,
                                    'cam': self.track.last})
                self.log('%6.1f s  balloon %d is seen at %+.0f deg (ToF %s): %s' % (
                    self.clock(), n, self.track.expect, '%.2f m' % h if h is not None else '?', self.track.text()))
                self.follow()
            if t_ready is None:
                h = tt.tof_m(self.link)
                if climbing:
                    done = (h is not None and h >= a.alt - 0.05) or self.clock() > give_up
                else:
                    done = self.following or self.clock() > give_up or (
                        self.heading.deg is not None and abs(sq.wrap180(self.heading.deg - expect)) <= TURNED_DEG)
                if done:
                    t_ready = self.clock()
                    self.at_height = True
                    self.log('%6.1f s  %s (heading %s, ToF %s)' % (
                        self.clock(), 'at the height' if climbing else 'turned',
                        '%+.0f deg' % self.heading.deg if self.heading.deg is not None else '?',
                        '%.2f m' % h if h is not None else '?'))
            if t_ready is not None and self.following:
                return None
            if t_ready is not None and self.clock() > t_ready + a.look and (t_next is None or self.clock() >= t_next):
                scan_k += 1
                if scan_k >= len(scan):
                    return ('land', 'balloon %d was not seen in %.0f s %s%s' % (
                        n, a.look, 'at the height' if climbing else 'after the turn',
                        ' nor in the scan' if scan else ''))
                if scan_k == 0:
                    # A glimpse (fewer frames than --seen) says which side to
                    # sweep first, as in target_test.py.
                    g = self.track.last_bearing
                    if g is not None and sq.wrap180(g - expect) < 0:
                        scan = [2.0 * expect - s for s in scan]
                    self.log('%6.1f s  -- balloon %d not seen: the scan, %s first%s' % (
                        self.clock(), n, 'right' if sq.wrap180(scan[0] - expect) < 0 else 'left',
                        ' (a glimpse at %+.0f deg)' % g if g is not None else ''))
                self.send('hold heading %g' % scan[scan_k])
                t_next = self.clock() + a.scan_dwell
            time.sleep(0.02)

    def approach(self, k):
        """Pulses toward balloon k+1 until it is close: ('close', why,
        pushed), pushed being the seconds of --push whose speed is still on
        (0 from about rest); or another stop; ('land', why) after
        --max-pulses."""
        a = self.args
        n = k + 1
        (pg, ps), (bg, bs) = a.push, a.brake
        looked_again = False
        for p in range(1, a.max_pulses + 1):
            if not self.track.recent(a.lost):
                self.log('%6.1f s  balloon %d not seen for %.0f s: %.0f s still, the nose where it was last seen'
                         % (self.clock(), n, a.lost, a.look))
                stop = self.phase('balloon %d lost: still' % n, 'hold', a.look, 0.0,
                                  until=lambda: self.track.recent(0.5))
                if stop:
                    return stop + (0.0,) if stop[0] == 'close' else stop
                if not self.track.recent(0.5):
                    if looked_again:
                        return ('land', 'balloon %d was lost again' % n)
                    looked_again = True
                    stop = self.find(k, self.track.expect, again=True)
                    if stop:
                        return stop + (0.0,) if stop[0] == 'close' else stop
            # A pulse flies straight, the nose held at the balloon's latest
            # bearing: with the follow on, the brake pushed along a nose that
            # had turned since the push, and the speed left over carried the
            # selftest's blimp 0.5 m past the balloon's side. The follow
            # aims again while the blimp is still.
            self.aim(self.track.expect)
            stop = self.phase('pulse %d toward balloon %d at %+.0f deg: %+.1f gf' % (p, n, self.track.expect, pg),
                              'push', ps, pg)
            pushed = self.ran
            if not stop:
                stop = self.phase('pulse %d: coast' % p, 'coast', a.coast, 0.0)
            if stop:
                return stop + (pushed,) if stop[0] == 'close' else stop
            stop = self.phase('pulse %d: brake %+.1f gf' % (p, -bg), 'brake', bs, -bg)
            if stop and stop[0] == 'close':
                # Close in the brake: finish it, then the touch from about rest.
                return self.phase('pulse %d: the rest of the brake' % p, 'brake', bs - self.ran, -bg,
                                  guard=False) or stop + (0.0,)
            if not stop:
                self.follow()
                stop = self.phase('pulse %d: still, following' % p, 'hold', a.hold, 0.0)
            if stop:
                return stop + (0.0,) if stop[0] == 'close' else stop
        return ('land', 'balloon %d was not close after %d pulses' % (n, a.max_pulses))

    def touch(self, k, why, pushed):
        """On into the balloon, the brake, the move away; None, or a stop
        met on the way (the close cue is off meanwhile). pushed: s of --push
        whose speed is still on. A push cut short by the close cue is
        finished first, and from about rest a whole push comes first: the
        selftest's blimp, cut a second into its push, coasted at 8 cm/s and
        stopped 0.5 m short."""
        a = self.args
        n = k + 1
        (pg, ps), (bg, bs) = a.push, a.brake
        self.closes.append((n, self.clock()))
        self.events.append({'t': self.clock(), 'event': 'close', 'balloon': n, 'why': why,
                            'heading_deg': self.heading.deg, 'cam': self.track.last})
        self.log('%6.1f s  balloon %d is close: %s; on into it for %.1f s, then the brake and away'
                 % (self.clock(), n, why, a.touch))
        # The nose stays where the balloon was last seen, through the touch
        # and the move away: close up the camera sees part of the balloon or
        # none, and the follow swung the selftest's nose 70 deg after a
        # balloon passing by, so that the move away went sideways.
        self.aim(self.track.expect)
        moves = []           # (label, kind, seconds, surge)
        if pushed < ps - 0.05:
            moves.append(('touch balloon %d: %+.1f gf' % (n, pg), 'touch', ps - pushed, pg))
        moves += [('touch balloon %d: coast into it' % n, 'touch', a.touch, 0.0),
                  ('touch: brake %+.1f gf' % -bg, 'brake', bs, -bg),
                  ('away from balloon %d: %+.1f gf' % (n, -bg), 'backoff', bs, -bg),
                  ('away: brake %+.1f gf' % pg, 'brake', ps, pg),
                  ('still after balloon %d' % n, 'hold', a.hold, 0.0)]
        for label, kind, seconds, surge in moves:
            stop = self.phase(label, kind, seconds, surge, guard=False)
            if stop:
                return stop
        return None

    def landing(self, why):
        """The landing from the height's setpoint; True when it ended on a kill."""
        self.log('%6.1f s  landing: %s' % (self.clock(), why))
        try:
            self.send('hold surge %g' % self.args.trim)
        except (RuntimeError, TimeoutError) as e:
            self.log('%6.1f s  the trim was not set for the landing (%s); landing anyway' % (self.clock(), e))
        t0 = self.clock()
        k = ht.land(self.con, self.link, self.args.alt, self.args.land_rate, self.args.final_rate, self.tripped,
                    self.log, self.clock, self.args.poll)
        self.phases.append((t0, self.clock(), 'landing', 'land', 0.0, self.args.trim))
        return k

    def fly(self):
        a = self.args
        try:
            self.tripped, self.peak = ht.make_kills(self.link, a.max_rate, a.max_tilt_rate, None)
            reply = ht.console(self.con, 'hold heading 0')
            self.t_hold = time.monotonic()
            self.log('%6.1f s  hold heading 0: %s' % (0.0, reply.strip().splitlines()[-1] if reply.strip() else '?'))
            if 'refused' in reply:
                raise SystemExit('the firmware refused the hold:\n' + reply)
            self.started = True
            self.heading.reset()
            self.tof_t = time.monotonic()
            self.send('hold surge %g' % a.trim)
            self.send('hold altitude %.2f %.3f' % (a.alt, a.climb_rate))
            stop = None
            for k, (expect, _, _) in enumerate(self.legs):
                if k > 0:
                    self.touched = (self.track.expect, self.close_px)     # balloon k's own close size
                    planned, expect = expect, bearing_after(a.course, k, self.track.expect, a.standoff)
                    self.log('%6.1f s  balloon %d was last seen at %+.0f deg: balloon %d should be at %+.0f deg '
                             '(%+.0f on the plan)' % (self.clock(), k, self.track.expect, k + 1, expect, planned))
                self.close_px = close_px(a, k)
                stop = self.find(k, expect)
                if not stop:
                    stop = self.phase('balloon %d: settle, following' % (k + 1), 'follow', a.settle, 0.0)
                if not stop:
                    stop = self.approach(k)
                if stop and stop[0] == 'close':
                    stop = self.touch(k, stop[1], stop[2] if len(stop) > 2 else 0.0)
                if stop:
                    break
            if stop and stop[0] == 'kill':
                self.killed = True
                self.end = 'killed: %s' % stop[1]
                ht.console(self.con, 'hold kill')
                self.log('%6.1f s  KILLED: %s' % (self.clock(), stop[1]))
            else:
                self.end = stop[1] if stop else 'the course is done'
                self.killed = self.landing(self.end)
                self.landed = not self.killed
        except (Exception, KeyboardInterrupt) as e:
            # Whatever went wrong after the take-off (a refused command, a
            # lost reply, a bug here, Ctrl-C), the blimp lands before 'hold off'.
            if not self.started:
                raise
            what = 'Ctrl-C' if isinstance(e, KeyboardInterrupt) else 'error: %s: %s' % (e.__class__.__name__, e)
            self.log('%6.1f s  %s' % (self.clock(), what))
            self.end = self.end or what
            if not (self.killed or self.landed):
                try:
                    self.killed = self.landing('after the %s' % ('Ctrl-C' if isinstance(e, KeyboardInterrupt)
                                                                 else 'error'))
                    self.landed = not self.killed
                except BaseException as e2:     # a second Ctrl-C, or the link gone: run() stops the motors
                    self.log('%6.1f s  the landing did not finish (%s): the motors stop' % (
                        self.clock(), e2.__class__.__name__))
        self.log('the course: %s; %s' % (self.summary(), 'killed' if self.killed else 'landed' if self.landed
                                         else 'not landed'))
        self.log('peak rates seen from here: yaw %.0f deg/s, roll/pitch %.0f deg/s' % (self.peak['yaw'],
                                                                                      self.peak['tilt']))

    def summary(self):
        parts = []
        for e in self.events:
            if e['event'] == 'seen':
                parts.append('balloon %d seen at %.1f s' % (e['balloon'], e['t']))
            elif e['event'] == 'close':
                parts.append('close at %.1f s' % e['t'])
        return ', '.join(parts + ['ended at %.1f s (%s)' % (self.clock(), self.end)])


def plan(args, course_legs):
    """The plan's lines and the capture's length, s."""
    a = args
    scan = tt.scan_headings(a.scan, a.scan_step)
    scan_s = len(scan) * a.scan_dwell
    climb_s = max(0.0, a.alt - ht.LAND_FLOOR_M) / a.climb_rate
    pulse_s = a.push[1] + a.coast + a.brake[1] + a.hold
    touch_s = 2.0 * (a.push[1] + a.brake[1]) + a.touch + a.hold     # a push, on into it, the brake, away, still
    find_s = a.look + scan_s
    land_s = ht.landing_time(a.alt + tt.HIGH_M, a.land_rate, a.final_rate)
    worst = ht.LEAD_S + land_s + 5.0
    for k in range(len(course_legs)):
        worst += ((climb_s + 10.0) if k == 0 else a.turn_time) + find_s + a.settle + a.max_pulses * pulse_s
        worst += a.look + a.turn_time + find_s + touch_s     # one more look after a loss
    cap_s = min(int(math.ceil(worst)), sq.CAPTURE_MAX_S)
    out = ['the course (x along the start\'s nose, y to its left, m; the balloons about 1.2 m up):']
    for k, ((bx, by), (b, d, (px, py))) in enumerate(zip(a.course, course_legs)):
        out.append('  balloon %d at (%.2f, %+.2f): looked for at %+.0f deg +-%.0f, %.2f m from %s; close at %d px'
                   % (k + 1, bx, by, b, a.window, d, 'the start' if k == 0 else
                      '(%.2f, %+.2f), %.1f m short of balloon %d (then from where balloon %d was last seen)'
                      % (px, py, a.standoff, k, k), close_px(a, k)))
    out.append('climb at %.2f m/s to %.2f m; seen when %d of the last %d frames detect it there, and the nose turns to '
               'it; not seen %.0f s after the height or the turn (done within %.0f deg, or %.0f s): the scan, '
               '%s deg around it, %.0f s each, then land' % (
                   a.climb_rate, a.alt, a.seen[0], a.seen[1], a.look, TURNED_DEG, a.turn_time,
                   ' '.join('%+.0f' % s for s in scan) or 'none', a.scan_dwell))
    out.append('each balloon: %.0f s following, then pulses of %+.1f gf %.1f s, a coast of up to %.0f s, %+.1f gf '
               '%.1f s and %.0f s still (surges with the trim), at most %d; a pulse needs the balloon seen in the last '
               '%.0f s' % (a.settle, a.push[0] + a.trim, a.push[1], a.coast, -a.brake[0] + a.trim, a.brake[1],
                           a.hold, a.max_pulses, a.lost))
    out.append('close (the patch at --close-px, %.0f deg above the camera\'s level, or lost right after big patches): '
               'on into it for %.1f s, the brake, the mirror away, %.0f s still; then the next balloon, or the '
               'landing (at most %.0f s)' % (a.close_el, a.touch, a.hold, land_s))
    out.append('trim %+.1f gf on every surge; lands early on the ToF over %.2f m or silent %.0f s, Ctrl-C, an error '
               'here; kills (motors off): yaw over %.0f deg/s, roll or pitch over %.0f deg/s' % (
                   a.trim, a.alt + tt.HIGH_M, tt.TOF_LOST_S, a.max_rate, a.max_tilt_rate))
    out.append('capture of %d s%s' % (cap_s, '' if worst <= sq.CAPTURE_MAX_S else
                                      ' (the most the blimp keeps; the slowest course would take %.0f s)' % worst))
    return out, cap_s


def run(args):
    course_legs = legs(args.course, args.standoff)
    lines_plan, cap_s = plan(args, course_legs)
    for s in lines_plan:
        print(s)
    if not args.go:
        print('dry run. PROPS ON for the real thing: the blimp on the floor at the start, nose toward balloon 1, '
              'no other red in the room, the ground station closed; then add --go')
        return 0

    lines = []

    def log(s):
        print(s)
        lines.append(s)

    link = bl.BlimpLink(args.host, args.port).start()
    con = bl.BlimpConsole(args.host, args.console_port)
    flight = Flight(link, con, args, course_legs, log)
    cap_started = tau_set = False
    data = info = tel0 = None
    params = {}
    t_cap = None
    front = ('FL', 'FR')
    ground_tau = 20.0        # core/src/params.c's gyro_bias_tau_s, unless --param sets another
    for p in args.param:
        name, _, value = p.partition('=')
        if name == 'gyro_bias_tau_s':
            ground_tau = float(value)
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
            sys.exit('the brakes and the moves away need front_reverse_ratio above 0 (it is %s): '
                     'add --param front_reverse_ratio=0.74' % ratio)
        reply = ht.console(con, 'cam')
        log(reply.strip().splitlines()[0] if reply.strip() else '?')
        # The downward camera's optical flow goes into the capture for
        # flow_track.py (2026-11-03); nothing in the flight uses it, so a
        # missing camera is said, not refused.
        m = re.search(r'optical flow \(optflow_cam\): (\d+) rows/s', reply)
        if m and int(m.group(1)) > 0:
            log('the downward camera: %s flow rows/s, in the capture' % m.group(1))
        else:
            log("WARNING: %s: this run's capture will have no track (the downward camera on GPIO18, "
                "the S3 at 3fe7911 or later?)" % ('no optical-flow rows' if m else "no optical-flow line in 'cam'"))
        # A row, detected or not, is target_cam's: the S3 relays only those as
        # CAMERA rows. The forward module ran optflow_cam on the evening of
        # 2026-11-02, whose rows reach 'cam''s count but never this packet.
        t_cam = time.monotonic() + 3.0
        while not (link.camera and link.camera['row']) and time.monotonic() < t_cam:
            time.sleep(0.05)
        if not (link.camera and link.camera['row']):
            sys.exit("no row from target_cam over the link in 3 s: is the forward camera running target_cam "
                     "(not optflow_cam), and does 'cam' show its rows?\n" + reply)
        if args.flight_bias_tau:
            flight.send('param gyro_bias_tau_s %g' % ground_tau)
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
            flight.send('param gyro_bias_tau_s %g' % args.flight_bias_tau)

        cap = bl.CaptureClient(link)
        if cap.info()['state'] == 1:
            cap.stop()
        cap.start(cap_s)
        cap_started = True
        t_cap = time.monotonic()
        time.sleep(ht.LEAD_S)
        flight.fly()
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
                log('gyro_bias_tau_s was not put back to %g s (%s): the next run sets it, or a reboot'
                    % (ground_tau, e))
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

    path = args.out or bl.capture_path('course', args.folder)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    blimp_log.write_file(path, info['raw'], data)
    h, records = blimp_log.decode(info['raw'], data)
    rows = blimp_log.to_units(h, records)
    offset = (flight.t_hold - t_cap) if flight.started else ht.LEAD_S
    res = sq.phase_metrics(rows, flight.phases, offset, front)
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
    print('a red patch in %d of %d records with a camera row' % (len(seen), len(cam_rows)))
    print('the course: %s' % flight.summary())
    meta = {'file': os.path.basename(path), 'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'source': 'course_test',
            'notes': args.notes, 'header': h, 'records': len(records), 'params': params,
            'test': {'course_m': args.course, 'legs': course_legs, 'standoff_m': args.standoff,
                     'alt_m': args.alt, 'climb_rate_mps': args.climb_rate, 'look_s': args.look, 'seen': args.seen,
                     'window_deg': args.window, 'scan_deg': args.scan, 'scan_step_deg': args.scan_step,
                     'scan_dwell_s': args.scan_dwell, 'turn_time_s': args.turn_time, 'close_px': args.close_px,
                     'close_el_deg': args.close_el, 'settle_s': args.settle, 'push': args.push, 'brake': args.brake,
                     'coast_s': args.coast, 'touch_s': args.touch, 'hold_s': args.hold,
                     'max_pulses': args.max_pulses, 'lost_s': args.lost,
                     'trim_gf': args.trim, 'flight_bias_tau_s': args.flight_bias_tau,
                     'land_rate_mps': args.land_rate, 'final_rate_mps': args.final_rate,
                     'phases': [list(p) for p in flight.phases], 'events': flight.events,
                     'closes': flight.closes, 'end': flight.end, 'hold_start_in_capture_s': offset,
                     'param': args.param, 'max_duty_pct': args.max_duty, 'deadband_pct': args.deadband,
                     'max_rate_dps': args.max_rate, 'max_tilt_rate_dps': args.max_tilt_rate,
                     'killed_from_ground': flight.killed, 'landed': flight.landed, 'telemetry_before': tel0,
                     'log': lines},
            'metrics': res}
    with open(os.path.splitext(path)[0] + '.json', 'w') as f:
        json.dump(meta, f, indent=1, default=str)
    print('%d records saved to %s, notes in %s' % (len(records), path, os.path.splitext(path)[0] + '.json'))
    return 0


# ---- the selftest's stand-in ----------------------------------------------

class _World:
    """The selftest's stand-in for the blimp, the S3's bench hold and the
    forward camera, stepped every 20 ms: the heading turns at up to 25 deg/s
    toward the held heading (or the follow's), the height follows its
    setpoint, the surge pushes 0.33 kg (with the air it carries along), with
    no drag and reverse at three quarters, the rear pair leans forward by the trim's
    0.6 gf, and the camera sees 45 cm balloons centred 1.2 m up, within
    29 deg of the nose, 22.5 deg of its level and 0.3..2.6 m, the biggest
    patch first, 12.5 frames a second. It has the S3's console replies that
    this tool reads. Not a model of the blimp to tune against: it exercises
    the sequence."""

    DT = 0.02

    def __init__(self, balloons, drift=(0.0, 0.0)):
        self.t = 1000.0
        self.balloons = list(balloons)
        self.drift = drift           # m/s, the room's air
        self.x = self.y = self.yaw = self.rate = self.vx = self.vy = 0.0
        self.z = 0.07
        self.mode = 'IDLE'
        self.hold = {'heading': 0.0, 'heading_on': False, 'alt': None, 'alt_to': None, 'alt_rate': 0.0,
                     'surge': 0.0, 'follow': False}
        self.follow_valid, self.follow_yaw = False, 0.0
        self.seq, self.next_frame = 0, 0.0
        self.contacts = []           # (balloon index, t) on each new contact
        self.touching = set()
        self.tof_gap = None          # (t0, t1): the ToF reads nothing
        self.spin_at = None          # t: from then a yaw rate the kill must catch
        self.camera = None
        self.sense()

    def step(self):
        dt, h = self.DT, self.hold
        self.t += dt
        armed = self.mode == 'ARMED'
        if armed and h['alt'] is not None:
            if h['alt_rate'] > 0:
                d = h['alt_to'] - h['alt']
                h['alt'] += max(-h['alt_rate'] * dt, min(h['alt_rate'] * dt, d))
            else:
                h['alt'] = h['alt_to']
            self.z += (h['alt'] - self.z) * dt / 1.5
        else:
            self.z = max(0.07, self.z - 0.05 * dt)
        airborne = self.z > 0.12
        target = None
        if armed and h['follow'] and self.follow_valid:
            target = self.follow_yaw
        elif armed and h['heading_on'] and not h['follow']:
            target = h['heading']
        self.rate = max(-25.0, min(25.0, 1.5 * sq.wrap180(target - self.yaw))) if target is not None \
            else self.rate * 0.9
        if self.spin_at is not None and self.t >= self.spin_at:
            self.rate = 90.0
        self.yaw = sq.wrap180(self.yaw + self.rate * dt)
        if armed and airborne:
            f = h['surge'] + 0.6
            a = (f if f >= 0 else 0.75 * f) * 0.00981 / 0.33
            # No drag: the air barely slows the blimp (test 3, 2026-11-01),
            # and a step's brake stops what its push gained (test 5).
            self.vx += a * math.cos(math.radians(self.yaw)) * dt
            self.vy += a * math.sin(math.radians(self.yaw)) * dt
        elif not airborne:
            self.vx = self.vy = 0.0
        if airborne:
            self.x += (self.vx + self.drift[0]) * dt
            self.y += (self.vy + self.drift[1]) * dt
        for i, (bx, by) in enumerate(self.balloons):
            if math.hypot(bx - self.x, by - self.y) < 0.5 and abs(self.z - 1.2) < 0.5:
                if i not in self.touching:
                    self.contacts.append((i, self.t))
                self.touching.add(i)
            else:
                self.touching.discard(i)
        if self.t >= self.next_frame:
            self.next_frame = self.t + 0.08
            self.frame()
        self.sense()

    def frame(self):
        self.seq += 1
        best = None
        zc = self.z - 0.1            # the camera, about 0.1 m under what the ToF reads (2026-11-02)
        for bx, by in self.balloons:
            dx, dy = bx - self.x, by - self.y
            d = math.hypot(dx, dy)
            b = sq.wrap180(math.degrees(math.atan2(dy, dx)) - self.yaw)
            el = math.degrees(math.atan2(1.2 - zc, d))
            if 0.3 < d < 2.6 and abs(b) <= 29.0 and abs(el) <= 22.5:
                area = min(19200.0, math.pi / 4.0 * (144.0 * 0.45 / d) ** 2)
                if best is None or area > best[2]:
                    best = (b, el, area)
        b, el, area = best or (0.0, 0.0, 0.0)
        self.camera = {'row': True, 'frame_seq': self.seq, 'ok': int(best is not None), 'az_deg': -b,
                       'el_deg': el, 'area_px': int(area), 'bearing_sign': -1}
        if best is not None and self.mode == 'ARMED' and self.hold['follow']:
            self.follow_yaw, self.follow_valid = sq.wrap180(self.yaw + b), True

    def sense(self):
        gap = self.tof_gap is not None and self.tof_gap[0] <= self.t < self.tof_gap[1]
        self.sensors = {'t_us': int(self.t * 1e6) & 0xFFFFFFFF, 'imu_valid': True, 'gyro': (0.0, 0.0, self.rate),
                        'tof_valid': not gap, 'tof_status': 0, 'tof_mm': int(self.z * 1000)}

    def status(self):
        h = self.hold
        held = ('; held at %+.1f deg, error %+.1f deg' % (h['heading'], sq.wrap180(h['heading'] - self.yaw))
                if h['heading_on'] and not h['follow'] else '')
        return ('mode %s\nestimator: converged (still 60 of 60 s)\n'
                'heading: estimate %+.1f deg (0 = the nose at arming), rate %+.1f deg/s%s\n'
                'altitude: estimate %.2f m (ToF); held at %.2f m\nsurge: %.1f gf\n'
                'output: fx 0.00 gf  fz 0.00 gf  tz 0.000 mN m;  throttle FL 0.100 RL 0.700 FR 0.100 RR 0.700\n'
                % ('KILLED' if self.mode == 'KILLED' else 'CLOSED_LOOP, armed', self.yaw, self.rate, held, self.z,
                   h['alt'] if h['alt'] is not None else 0.0, h['surge']))


class _Console:
    """The 'hold' console of the S3, as far as this tool uses it."""

    def __init__(self, world):
        self.w = world

    def drain(self):
        pass

    def close(self):
        pass

    def run(self, line, timeout=5.0, idle=2.0):
        w, words = self.w, line.split()
        if words == ['hold']:
            return w.status()
        if words[0] != 'hold' or len(words) < 2:
            return 'Unrecognized command\n'
        what, h = words[1], w.hold
        if what == 'kill':
            w.mode = 'KILLED'
            return 'all motors coasting; the hold is KILLED until \'hold off\'\n'
        if w.mode == 'KILLED':
            return 'refused: KILLED by a stop; \'hold off\' first, then start again\n'
        on = len(words) < 3 or words[2] != 'off'
        if what == 'heading':
            h['heading_on'] = on
            h['heading'] = float(words[2]) if on else h['heading']
        elif what == 'altitude':
            if h['alt'] is None:
                h['alt'] = w.z
            h['alt_to'] = float(words[2])
            h['alt_rate'] = float(words[3]) if len(words) > 3 else 0.0
        elif what == 'surge':
            h['surge'] = float(words[2])
        elif what == 'follow':
            h['follow'] = on
            w.follow_valid = w.follow_valid and on
        else:
            return 'usage\n'
        starting = w.mode != 'ARMED'
        w.mode = 'ARMED'
        return (('hold starting: armed at the next tick, heading 0 = where the nose points now\n' if starting else '')
                + 'held: heading %s, altitude on, surge %.1f gf, follow %s\n'
                % ('on' if h['heading_on'] else 'off', h['surge'], 'on' if h['follow'] else 'off'))


class _Clock:
    """time.monotonic() and time.sleep() in the stand-in's time."""

    def __init__(self):
        self.w = None

    def monotonic(self):
        return self.w.t

    def sleep(self, seconds):
        for _ in range(max(1, int(round(seconds / _World.DT)))):
            self.w.step()


def selftest():
    """The sequence against _World in simulated time: the room's course, the
    same with the room's air, the second balloon missing (the gate keeps the
    first one out of its scan), no balloon at all, a ToF dropout, a spin."""
    global time
    saved = time, ht.time
    clock = _Clock()
    time = ht.time = clock
    failed = []

    def fly(balloons, extra=(), setup=None, drift=(0.0, 0.0)):
        args = parser().parse_args(['--close-px', '5000', '--trim', '-0.6'] + list(extra))
        clock.w = w = _World(balloons, drift)
        lines = []
        f = Flight(w, _Console(w), args, legs(args.course, args.standoff), lines.append)
        if setup:
            setup(w)
        f.fly()
        return f, w, lines

    def check(name, ok, lines=()):
        print('  %-58s %s' % (name, 'ok' if ok else 'FAILED'))
        if not ok:
            failed.append(name)
            for s in list(lines)[-30:]:
                print('      ' + s)

    def order(w):
        out = []
        for i, _ in w.contacts:
            if i not in out:
                out.append(i)
        return out

    def attributed(f, w):
        """Every close came before a contact with that same balloon (the
        stand-in's clock runs from 1000 s; the flight's from the hold)."""
        t0 = f.t_hold
        return all(any(i == n - 1 and 0.0 <= t - t0 - tc <= 10.0 for i, t in w.contacts) for n, tc in f.closes)

    try:
        lg = legs(COURSE_ROOM, 0.5)
        check('the room\'s legs: 0 deg 2 m, then -76 deg 2.06 m from (1.5, 0)',
              abs(lg[0][0]) < 1e-9 and abs(lg[0][1] - 2.0) < 1e-9 and abs(lg[1][0] + 76.0) < 0.1
              and abs(lg[1][1] - 2.062) < 0.01 and abs(lg[1][2][0] - 1.5) < 1e-9)
        check('balloon 2 after balloon 1 last seen at -45 deg: -81 deg',
              abs(bearing_after(COURSE_ROOM, 1, -45.0, 0.5) + 81.4) < 0.1)

        f, w, lines = fly(COURSE_ROOM)
        check('the course: both balloons, in order, then the landing',
              [n for n, _ in f.closes] == [1, 2] and order(w) == [0, 1] and attributed(f, w) and f.landed
              and w.z <= 0.1 and f.end == 'the course is done', lines)

        # Air for the whole flight. Up to about 1 cm/s the stand-in flies the
        # course whichever way it blows; at 2-3 cm/s (the old room's 1-2 m in
        # 40 s is 2.5-5) it often lands without balloon 2, and with the air
        # to the left it may touch balloon 1 again and log it as balloon 2:
        # 2 m or more of drift lines them up, and nothing on board measures
        # position. The team's account of what was touched is the record.
        f, w, lines = fly(COURSE_ROOM, drift=(0.0, 0.01))
        check('1 cm/s of air to the left: both balloons',
              [n for n, _ in f.closes] == [1, 2] and order(w) == [0, 1] and attributed(f, w) and f.landed, lines)

        f, w, lines = fly(COURSE_ROOM, drift=(0.02, -0.01))
        check('2 cm/s along balloon 1\'s line, 1 cm/s to the right: both balloons',
              [n for n, _ in f.closes] == [1, 2] and order(w) == [0, 1] and attributed(f, w) and f.landed, lines)

        f, w, lines = fly(COURSE_ROOM, drift=(0.0, 0.03))
        check('3 cm/s to the left: balloon 1, then a landing, no false balloon 2',
              [n for n, _ in f.closes] == [1] and attributed(f, w) and f.landed, lines)

        f, w, lines = fly(COURSE_ROOM[:1], ['--scan', '60'])
        check('balloon 2 missing: balloon 1 kept out of its scan, then land',
              [n for n, _ in f.closes] == [1] and order(w) == [0] and f.track.rejected > 0 and f.landed
              and 'balloon 2 was not seen' in f.end, lines)

        f, w, lines = fly([])
        check('no balloon: the climb, the scan, land',
              not f.closes and f.landed and 'balloon 1 was not seen' in f.end, lines)

        def gap(w):
            w.tof_gap = (w.t + 30.0, w.t + 31.5)
        f, w, lines = fly(COURSE_ROOM, setup=gap)
        check('a ToF dropout in the approach: land', f.landed and 'ToF' in f.end, lines)

        def spin(w):
            w.spin_at = w.t + 30.0
        f, w, lines = fly(COURSE_ROOM, setup=spin)
        check('a spin: killed, no landing', f.killed and not f.landed and w.mode == 'KILLED', lines)
    finally:
        time, ht.time = saved
    print('selftest: %s' % ('FAILED: ' + ', '.join(failed) if failed else 'ok'))
    return 1 if failed else 0


def parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default=bl.BLIMP_IP)
    ap.add_argument('--port', type=int, default=bl.LINK_PORT)
    ap.add_argument('--console-port', type=int, default=bl.CONSOLE_PORT)
    ap.add_argument('--course', type=point, nargs='+', default=list(COURSE_ROOM), metavar='X,Y',
                    help="the balloons in order, m from the start, x along the start's nose, y to its left "
                         '(default the room of 2026-11-03: 2,0 2,-2)')
    ap.add_argument('--close-px', type=int, nargs='+', metavar='PX',
                    help='close: the patch\'s median at or over this, px; one for all, or one per balloon '
                         '(required: from the camera check, docs/flight-tests.md C0)')
    ap.add_argument('--close-el', type=float, default=15.0,
                    help='close: the balloon this far above the camera\'s level, deg (default %(default)s; the '
                         'view ends at +22.5)')
    ap.add_argument('--alt', type=float, default=1.3,
                    help='the height, m (default %%(default)s: the camera about level with balloons centred '
                         '1.2 m up; at most %.1f)' % tt.ALT_MAX_M)
    ap.add_argument('--climb-rate', type=float, default=0.12, help='the climb, m/s (default %(default)s)')
    ap.add_argument('--standoff', type=float, default=0.5,
                    help='m short of a balloon where the blimp should be after touching it and moving away, for the '
                         'next one\'s bearing (default %(default)s)')
    ap.add_argument('--window', type=float, default=40.0,
                    help='a detection counts within this of where the balloon should be, deg (default %(default)s)')
    ap.add_argument('--turn-time', type=float, default=8.0,
                    help='s the turn to the next balloon gets before the look starts anyway (default %(default)s; '
                         '90 deg took 4.8-7.2 s on 2026-11-01)')
    ap.add_argument('--look', type=float, default=5.0,
                    help='s at the height or after the turn before the scan (default %(default)s)')
    ap.add_argument('--scan', type=float, default=45.0,
                    help='the scan\'s half-width around where the balloon should be, deg (default %(default)s; 0: '
                         'no scan, land)')
    ap.add_argument('--scan-step', type=float, default=15.0, help='its heading steps, deg (default %(default)s)')
    ap.add_argument('--scan-dwell', type=float, default=2.0, help='s at each step (default %(default)s)')
    ap.add_argument('--seen', type=int, nargs=2, default=[3, 10], metavar=('K', 'N'),
                    help='seen: K of the last N relayed camera frames detect it there (default 3 10)')
    ap.add_argument('--settle', type=float, default=5.0,
                    help='s following each balloon before the steps (default %(default)s)')
    ap.add_argument('--push', type=float, nargs=2, default=[3.0, 1.8], metavar=('GF', 'S'),
                    help="a pulse's push toward the balloon (default 3 gf 1.8 s, about 0.15 m/s)")
    ap.add_argument('--coast', type=float, default=5.0,
                    help='its coast, following, unless the balloon is close first, s (default %(default)s)')
    ap.add_argument('--brake', type=float, nargs=2, default=[3.0, 2.4], metavar=('GF', 'S'),
                    help="its counter-push (default 3 gf 2.4 s: stops the push's speed, as in test 5 and "
                         "target_test.py's moves)")
    ap.add_argument('--hold', type=float, default=2.0, help='s still after each pulse (default %(default)s)')
    ap.add_argument('--touch', type=float, default=2.5,
                    help='close: s of coast on into the balloon before the brake (default %(default)s, about 0.4 m)')
    ap.add_argument('--max-pulses', type=int, default=4,
                    help='pulses toward a balloon before the landing (default %(default)s, about 5 m)')
    ap.add_argument('--lost', type=float, default=3.0,
                    help='a pulse starts only on a balloon seen in the last this many s (default %(default)s)')
    ap.add_argument('--trim', type=float, default=0.0,
                    help='gf added to every surge, as square_test.py\'s (-0.6 for the new battery type, -1 for the '
                         'earlier cells: docs/flight-tests.md)')
    ap.add_argument('--flight-bias-tau', type=float, default=600.0, metavar='S',
                    help="core/'s gyro-bias time constant in the air, as square_test.py's (default %(default)s; "
                         '0 leaves it alone)')
    ap.add_argument('--land-rate', type=float, default=0.1,
                    help='the landing down to 0.30 m, m/s (default %(default)s)')
    ap.add_argument('--final-rate', type=float, default=0.03,
                    help='the landing from 0.30 m to the floor, m/s (default %(default)s)')
    ap.add_argument('--param', nargs='*', default=[], metavar='NAME=VALUE', help="'param' sets before the start")
    ap.add_argument('--max-duty', type=float, default=40.0)
    ap.add_argument('--deadband', type=float, default=10.0)
    ap.add_argument('--max-rate', type=float, default=62.0,
                    help='kill above this yaw rate, deg/s (default %(default)s)')
    ap.add_argument('--max-tilt-rate', type=float, default=120.0,
                    help='kill above this roll or pitch rate, deg/s (default %(default)s)')
    ap.add_argument('--poll', type=float, default=1.0, help="s between status lines (default %(default)s)")
    ap.add_argument('--wait', type=float, default=180.0,
                    help="s to wait for core/'s estimator to converge (default %(default)s)")
    ap.add_argument('--folder', default='captures')
    ap.add_argument('--out', help='the .bin (default <folder>/course-<date>.bin)')
    ap.add_argument('--notes', default='')
    ap.add_argument('--go', action='store_true', help='really fly it (PROPS ON)')
    ap.add_argument('--selftest', action='store_true', help='fly the sequence against a stand-in, no hardware')
    return ap


def check(args):
    """Why the arguments cannot fly, or None."""
    a = args
    if not 1 <= len(a.course) <= 8:
        return '--course takes 1 to 8 balloons'
    for k, (_, d, _) in enumerate(legs(a.course, a.standoff)):
        if not LEG_MIN_M <= d <= 6.0:
            return ('balloon %d is %.2f m from where the blimp looks for it: %.1f .. 6 m (the camera found the '
                    'balloon from about 2.2 m on 2026-11-02)' % (k + 1, d, LEG_MIN_M))
    if not a.close_px:
        return ('--close-px is required: the patch of these balloons about 0.4 m before the envelope touches '
                'them, from the camera check (docs/flight-tests.md, C0)')
    if len(a.close_px) not in (1, len(a.course)):
        return '--close-px takes one value, or one per balloon (%d)' % len(a.course)
    if not all(20 <= p <= 19200 for p in a.close_px):
        return '--close-px is 20 .. 19200 px (the image is 160 x 120)'
    if not (math.isfinite(a.alt) and 0.5 <= a.alt <= tt.ALT_MAX_M):
        return '--alt is 0.5 .. %.1f m: the ToF lost the floor by 2.7 m (2026-11-02)' % tt.ALT_MAX_M
    if not (math.isfinite(a.climb_rate) and 0.02 <= a.climb_rate <= 0.5):
        return "--climb-rate is 0.02 .. 0.5 m/s (the firmware's 'hold altitude' rate)"
    if not (math.isfinite(a.standoff) and 0.0 <= a.standoff <= 2.0):
        return '--standoff is 0 .. 2 m'
    if not (math.isfinite(a.window) and 10.0 <= a.window <= 90.0):
        return '--window is 10 .. 90 deg'
    if not (math.isfinite(a.close_el) and 5.0 <= a.close_el <= 22.0):
        return '--close-el is 5 .. 22 deg (the view ends at +22.5)'
    if not (math.isfinite(a.scan) and 0.0 <= a.scan <= 90.0):
        return '--scan is 0 .. 90 deg'
    if not (math.isfinite(a.scan_step) and 5.0 <= a.scan_step <= 45.0):
        return '--scan-step is 5 .. 45 deg: the view is 58 deg wide, and a bigger step is a turn, not a scan'
    if not (math.isfinite(a.scan_dwell) and 0.5 <= a.scan_dwell <= 10.0):
        return '--scan-dwell is 0.5 .. 10 s'
    k, n = a.seen
    if not 1 <= k <= n <= 40:
        return '--seen is K N with 1 <= K <= N <= 40'
    for name, v, lo, hi in (('--look', a.look, 0.0, 60.0), ('--settle', a.settle, 0.0, 60.0),
                            ('--hold', a.hold, 0.0, 30.0), ('--turn-time', a.turn_time, 2.0, 30.0),
                            ('--lost', a.lost, 0.5, 30.0), ('--coast', a.coast, 0.0, 15.0),
                            ('--touch', a.touch, 0.0, 8.0)):
        if not (math.isfinite(v) and lo <= v <= hi):
            return '%s is %g .. %g s' % (name, lo, hi)
    if not 1 <= a.max_pulses <= 10:
        return '--max-pulses is 1 .. 10'
    for name, (gf, s) in (('--push', a.push), ('--brake', a.brake)):
        if not (math.isfinite(gf) and 0.0 < gf <= 10.0 and math.isfinite(s) and 0.0 < s <= 10.0):
            return '%s is GF S with 0 < GF <= 10 gf and 0 < S <= 10 s' % name
    if not (math.isfinite(a.trim) and -10.0 <= a.trim <= 10.0):
        return '--trim is -10 .. 10 gf'
    if a.flight_bias_tau and not (math.isfinite(a.flight_bias_tau) and 0.5 <= a.flight_bias_tau <= 600.0):
        return "--flight-bias-tau is 0.5 .. 600 s (the S3's 'param' range for gyro_bias_tau_s), or 0"
    if not (0.02 <= a.land_rate <= 0.5 and 0.01 <= a.final_rate <= 0.2):
        return '--land-rate is 0.02 .. 0.5 m/s and --final-rate 0.01 .. 0.2 m/s'
    if not (10.0 <= a.max_duty <= 100.0 and 0.0 <= a.deadband < a.max_duty):
        return '--max-duty is 10 .. 100 and --deadband 0 .. below it (percent)'
    return None


def main(argv=None):
    args = parser().parse_args(argv)
    if args.selftest:
        return selftest()
    why = check(args)
    if why:
        sys.exit(why)
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
