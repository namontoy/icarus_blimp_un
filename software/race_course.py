#!/usr/bin/env python3
"""The race course on one sheet: the balloons, their heights and the field's
limits as measured at the venue, in software/race_course.json. The ground
station's Course panel draws and edits the sheet, the Autonomous mode of its
Control page flies it (assist.py), and course_test.py flies the same sheet
from a terminal (--course-file), so the numbers are typed once.

    python software/race_course.py                      # print the sheet and its legs
    python software/race_course.py --selftest

The frame is course_test.py's: the start S is where the gondola stands at
the start with its nose toward balloon 1; x is metres ahead of S along the
nose, y metres to S's left (+ counter-clockwise, as 'hold heading'). Each
balloon is its floor point (the foot of its tether) and the height of its
centre over the floor. The field is how far the floor runs from S: ahead,
behind, to the left and to the right (the walls, the tape, the organisers'
lines). Nothing on board measures position (AGENTS.md), so the field serves
the sketch and the checks: a balloon outside it is a typo, and a leg longer
than the camera finds a balloon is refused, as course_test.py refuses it.

The sheet:

    {"name": "...",
     "field": {"ahead_m": 6.0, "behind_m": 1.0, "left_m": 3.0, "right_m": 3.0},
     "balloons": [{"x_m": 2.0, "y_m": 0.0, "height_m": 1.2}, ...],
     "fly_above_m": 0.1, "close_px": 1500, "loops": 1, "time_limit_s": 300, "notes": "..."}

fly_above_m: the gondola holds each balloon's centre height plus this. The
camera sits on the gondola and sees the balloon a little above its level
when the gondola is at the balloon's height (the course room of 2026-11-03
flew 1.3 m at balloons of 1.2 m). close_px: the patch at which the blimp goes
in to touch, course_test.py's --close-px, from the camera check with the
venue's own balloons; one value for every balloon. loops: the balloons in
order this many times, the race's loops. time_limit_s: the race's flight time
(5 min, the team, 2026-11-05): the course lands with just the landing's time
left. Standard library only.
"""
import argparse
import copy
import json
import math
import os
import sys

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'race_course.json')
ALT_MIN_M = 0.5           # course_test.py's --alt range: the ToF lost the floor by 2.7 m (2026-11-02)
ALT_MAX_M = 2.3
MAX_BALLOONS = 8
MAX_LOOPS = 5
LEG_MAX_M = 6.0           # course_test.py's limit on a leg
LEG_MIN_M = 1.0
CAM_REACH_M = 2.2         # the camera found the balloon from about 2.2 m (2026-11-02): a longer leg is a warning

# The course room of 2026-11-03 until the venue's numbers replace it: balloon
# 1 2 m ahead, balloon 2 2 m to its right, both 1.2 m up.
DEFAULT = {
    'name': 'the course room (2026-11-03); replace with the venue',
    'field': {'ahead_m': 6.0, 'behind_m': 1.0, 'left_m': 3.0, 'right_m': 3.0},
    'balloons': [{'x_m': 2.0, 'y_m': 0.0, 'height_m': 1.2}, {'x_m': 2.0, 'y_m': -2.0, 'height_m': 1.2}],
    'fly_above_m': 0.1,
    'close_px': 1500,
    'loops': 1,
    'time_limit_s': 300,
    'notes': 'x ahead of the start along the nose, y to its left, in metres; the start is where the gondola '
             'stands, nose toward balloon 1. Measure each balloon\'s floor point and the height of its centre, '
             'and the floor\'s extent from the start on all four sides.',
}


def _num(v, name):
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise ValueError('%s is not a number: %r' % (name, v))
    if not math.isfinite(f):
        raise ValueError('%s is not a number: %r' % (name, v))
    return f


def normalise(sheet):
    """A copy with every key present and every number a float (loops and
    close_px ints); ValueError on a value that is not one."""
    if not isinstance(sheet, dict):
        raise ValueError('the sheet is not an object')
    out = copy.deepcopy(DEFAULT)
    out['name'] = str(sheet.get('name', out['name']))
    out['notes'] = str(sheet.get('notes', ''))
    field = sheet.get('field', {})
    if not isinstance(field, dict):
        raise ValueError('field is not an object')
    for k in out['field']:
        out['field'][k] = _num(field.get(k, out['field'][k]), 'field.' + k)
    balloons = sheet.get('balloons', [])
    if not isinstance(balloons, list):
        raise ValueError('balloons is not a list')
    out['balloons'] = []
    for i, b in enumerate(balloons):
        if not isinstance(b, dict):
            raise ValueError('balloon %d is not an object' % (i + 1))
        out['balloons'].append({k: _num(b.get(k, 0.0 if k != 'height_m' else 1.2), 'balloon %d %s' % (i + 1, k))
                                for k in ('x_m', 'y_m', 'height_m')})
    out['fly_above_m'] = _num(sheet.get('fly_above_m', out['fly_above_m']), 'fly_above_m')
    out['close_px'] = int(round(_num(sheet.get('close_px', out['close_px']), 'close_px')))
    out['loops'] = int(round(_num(sheet.get('loops', 1), 'loops')))
    out['time_limit_s'] = _num(sheet.get('time_limit_s', 300), 'time_limit_s')
    return out


def load(path=DEFAULT_PATH):
    """The sheet at path, normalised; the default when the file is missing.
    ValueError on a file that is not a sheet, OSError on one that cannot be
    read."""
    if not os.path.exists(path):
        return copy.deepcopy(DEFAULT)
    with open(path, encoding='utf-8') as f:
        try:
            return normalise(json.load(f))
        except json.JSONDecodeError as e:
            raise ValueError('%s is not JSON: %s' % (path, e))


def save(sheet, path=DEFAULT_PATH):
    sheet = normalise(sheet)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(sheet, f, indent=1)
        f.write('\n')
    os.replace(tmp, path)
    return sheet


def points(sheet):
    """The balloons' floor points, (x, y) m, in order, the loops included."""
    pts = [(b['x_m'], b['y_m']) for b in sheet['balloons']]
    return pts * max(1, int(sheet['loops']))


def altitudes(sheet):
    """The gondola's height at each balloon, m: the centre plus fly_above_m,
    within the ToF's range, the loops included."""
    alts = [min(ALT_MAX_M, max(ALT_MIN_M, b['height_m'] + sheet['fly_above_m'])) for b in sheet['balloons']]
    return alts * max(1, int(sheet['loops']))


def legs(sheet, standoff_m=0.5):
    """course_test.py's legs: (bearing deg, distance m, from (x, y)) per balloon."""
    out = []
    px, py = 0.0, 0.0
    for bx, by in points(sheet):
        dx, dy = bx - px, by - py
        d = math.hypot(dx, dy)
        out.append((math.degrees(math.atan2(dy, dx)), d, (px, py)))
        f = max(0.0, d - standoff_m) / d if d > 0 else 0.0
        px, py = px + dx * f, py + dy * f
    return out


def check(sheet, standoff_m=0.5):
    """Why the sheet cannot be flown, or None."""
    try:
        s = normalise(sheet)
    except ValueError as e:
        return str(e)
    f = s['field']
    for k, v in f.items():
        if not 0.5 <= v <= 30.0:
            return 'field.%s is %g m: 0.5 .. 30 m' % (k, v)
    n = len(s['balloons'])
    if not 1 <= n <= MAX_BALLOONS:
        return 'the course takes 1 to %d balloons (%d given)' % (MAX_BALLOONS, n)
    if not 1 <= s['loops'] <= MAX_LOOPS:
        return 'loops is %d: 1 .. %d' % (s['loops'], MAX_LOOPS)
    if not -0.5 <= s['fly_above_m'] <= 0.5:
        return 'fly_above_m is %g: -0.5 .. 0.5 m' % s['fly_above_m']
    if not 20 <= s['close_px'] <= 19200:
        return 'close_px is %d: 20 .. 19200 (the image is 160 x 120)' % s['close_px']
    if s['time_limit_s'] != 0 and not 30.0 <= s['time_limit_s'] <= 900.0:
        return 'time_limit_s is %g: 30 .. 900 s, or 0 for none (the race allows 5 min)' % s['time_limit_s']
    for i, b in enumerate(s['balloons']):
        if not -f['behind_m'] <= b['x_m'] <= f['ahead_m']:
            return 'balloon %d is at x %+.2f m, outside the field (%.1f m behind to %.1f m ahead)' % (
                i + 1, b['x_m'], f['behind_m'], f['ahead_m'])
        if not -f['right_m'] <= b['y_m'] <= f['left_m']:
            return 'balloon %d is at y %+.2f m, outside the field (%.1f m right to %.1f m left)' % (
                i + 1, b['y_m'], f['right_m'], f['left_m'])
        if not 0.3 <= b['height_m'] <= 2.5:
            return 'balloon %d hangs at %.2f m: 0.3 .. 2.5 m (the ToF loses the floor beyond about 2.5 m)' % (
                i + 1, b['height_m'])
    for k, (_, d, _) in enumerate(legs(s, standoff_m)):
        if not LEG_MIN_M <= d <= LEG_MAX_M:
            return ('balloon %d is %.2f m from where the blimp looks for it: %.0f .. %.0f m (the camera found the '
                    'balloon from about 2.2 m on 2026-11-02)' % (k % n + 1, d, LEG_MIN_M, LEG_MAX_M))
    return None


def warnings(sheet, standoff_m=0.5):
    """What a sheet that checks can still cost, as lines: a leg beyond the
    camera's reach (about 2.2 m, 2026-11-02) means the blimp looks for a
    balloon it cannot see yet from where the last one left it: it scans, and
    lands if the scan finds nothing (course_test.py flies no blind leg)."""
    out = []
    try:
        s = normalise(sheet)
    except ValueError:
        return out
    n = len(s['balloons'])
    for k, (_, d, _) in enumerate(legs(s, standoff_m)):
        if d > CAM_REACH_M and k < n:
            out.append('balloon %d is %.1f m from where the blimp looks for it, beyond the camera\'s reach (about '
                       '%.1f m): it scans, flies blind pulses toward where it should be, and lands if it still does '
                       'not see it' % (k + 1, d, CAM_REACH_M))
    return out


def apply(args, sheet):
    """course_test.py's arguments from the sheet, where none were given on the
    command line (None): the course, one --alt per balloon, --close-px."""
    if args.course is None:
        args.course = points(sheet)
    if args.alt is None:
        args.alt = altitudes(sheet)
    if args.close_px is None:
        args.close_px = [sheet['close_px']]
    if getattr(args, 'time_limit', None) is None:
        args.time_limit = sheet['time_limit_s']
    return args


def summary(sheet):
    s = normalise(sheet)
    f = s['field']
    parts = ['%d balloon%s' % (len(s['balloons']), '' if len(s['balloons']) == 1 else 's')]
    if s['loops'] > 1:
        parts[0] += ' x %d loops' % s['loops']
    parts.append('field %.1f m ahead, %.1f behind, %.1f left, %.1f right' % (f['ahead_m'], f['behind_m'],
                                                                            f['left_m'], f['right_m']))
    parts.append(', '.join('%d at (%.1f, %+.1f) %.2f m up' % (i + 1, b['x_m'], b['y_m'], b['height_m'])
                           for i, b in enumerate(s['balloons'])))
    return '; '.join(parts)


def describe(sheet, standoff_m=0.5):
    """The sheet as lines, for a terminal."""
    s = normalise(sheet)
    out = ['%s' % s['name'], '  ' + summary(s), '  fly %.2f m above each centre; close at %d px; the hold at:'
           % (s['fly_above_m'], s['close_px'])]
    n = len(s['balloons'])
    for k, ((b, d, (px, py)), alt) in enumerate(zip(legs(s, standoff_m), altitudes(s))):
        out.append('  balloon %d%s: %.2f m, looked for at %+.0f deg from (%.2f, %+.2f)' % (
            k % n + 1, ' (loop %d)' % (k // n + 1) if s['loops'] > 1 else '', alt, b, px, py))
    why = check(s, standoff_m)
    out.append('  ' + ('cannot be flown: ' + why if why else 'checks pass'))
    for w in warnings(s, standoff_m):
        out.append('  warning: ' + w)
    return out


def selftest():
    failed = []

    def ok(name, cond):
        print('  %-60s %s' % (name, 'ok' if cond else 'FAILED'))
        if not cond:
            failed.append(name)

    s = normalise(DEFAULT)
    ok('the default sheet checks', check(s) is None)
    ok('the default is the course room', points(s) == [(2.0, 0.0), (2.0, -2.0)] and altitudes(s) == [1.3, 1.3])
    lg = legs(s)
    ok('its legs are course_test.py\'s', abs(lg[0][0]) < 1e-9 and abs(lg[1][0] + 76.0) < 0.1
       and abs(lg[1][1] - 2.062) < 0.01)
    s2 = normalise(dict(s, loops=2))
    ok('loops repeat the balloons', len(points(s2)) == 4 and len(altitudes(s2)) == 4 and check(s2) is None)
    s3 = normalise(dict(s, balloons=[{'x_m': 2.0, 'y_m': 0.0, 'height_m': 1.9}], fly_above_m=0.1))
    ok('a height is the centre plus fly_above', abs(altitudes(s3)[0] - 2.0) < 1e-9)
    s4 = normalise(dict(s, balloons=[{'x_m': 2.0, 'y_m': 0.0, 'height_m': 2.4}]))
    ok('the hold is capped at the ToF\'s range', altitudes(s4)[0] == ALT_MAX_M)
    ok('a balloon outside the field is refused',
       'outside the field' in (check(dict(s, balloons=[{'x_m': 7.0, 'y_m': 0.0, 'height_m': 1.2}])) or ''))
    ok('a balloon too near the last is refused',
       'from where the blimp looks' in (check(dict(s, balloons=s['balloons'] + [{'x_m': 2.2, 'y_m': -2.0,
                                                                                 'height_m': 1.2}])) or ''))
    ok('no balloon is refused', 'takes 1 to' in (check(dict(s, balloons=[])) or ''))
    far = dict(s, balloons=[{'x_m': 2.0, 'y_m': 0.0, 'height_m': 1.2}, {'x_m': 5.0, 'y_m': 0.0, 'height_m': 1.2}])
    ok('a leg beyond the camera\'s reach is a warning, not a refusal',
       check(far) is None and len(warnings(far)) == 1 and 'balloon 2' in warnings(far)[0] and not warnings(s))
    ok('a word for a number is refused', 'not a number' in (check(dict(s, close_px='big')) or ''))
    ok('a missing key takes the default', normalise({'balloons': [{'x_m': 2}]})['balloons'][0]['height_m'] == 1.2)

    import tempfile
    d = tempfile.mkdtemp(prefix='race-course-')
    path = os.path.join(d, 'race_course.json')
    ok('a missing file loads the default', load(path) == normalise(DEFAULT))
    save(dict(s, name='venue', loops=3), path)
    back = load(path)
    ok('save and load round trip', back['name'] == 'venue' and back['loops'] == 3 and back['balloons'] == s['balloons'])
    with open(path, 'w') as f:
        f.write('{not json')
    try:
        load(path)
        ok('a broken file raises', False)
    except ValueError:
        ok('a broken file raises', True)

    class A:
        course = alt = close_px = time_limit = None
    a = apply(A(), s2)
    ok('apply fills course_test.py\'s arguments', a.course == points(s2) and a.alt == altitudes(s2)
       and a.close_px == [1500] and a.time_limit == 300)
    ok('a time limit under half a minute is refused', 'time_limit_s' in (check(dict(s, time_limit_s=10)) or ''))
    print('selftest: %s' % ('FAILED: ' + ', '.join(failed) if failed else 'ok'))
    return 1 if failed else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description='The race course sheet: print it, check it.')
    ap.add_argument('path', nargs='?', default=DEFAULT_PATH, help='the sheet (default %(default)s)')
    ap.add_argument('--standoff', type=float, default=0.5, help="course_test.py's --standoff, m (default %(default)s)")
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args(argv)
    if args.selftest:
        return selftest()
    try:
        sheet = load(args.path)
    except (OSError, ValueError) as e:
        sys.exit(str(e))
    for line in describe(sheet, args.standoff):
        print(line)
    return 0 if check(sheet, args.standoff) is None else 1


if __name__ == '__main__':
    sys.exit(main())
