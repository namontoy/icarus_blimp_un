"""The 'param' command checked on the blimp (firmware main/cmd_param.c,
2026-10-30): the bench checks that need the board, run over the console and
printed PASS, FAIL or SKIP, with the parameters left as they were found.

    python software/param_check.py                       # about 15 s
    python software/param_check.py --reboot              # then a power cycle: params.c comes back
    python software/param_check.py --host 127.0.0.1      # against mock_blimp.py

What it checks, and why each matters:

  listing     'param' answers (a firmware from before 2026-10-30 says
              "Unrecognized command"), and every row its first line counts
              arrived: the full listing is about 3 kB, more than one datagram
              of the UDP console.
  set         a value is stored, the change count moves, the row is marked.
  the loop    'ctl' says the control task took that change at its next tick
              (param_take(), cmd_param.h).
  refusals    a typo (40 for 0.40), text, and a clamp minimum above its
              maximum are refused and change nothing; a negative value, which
              argtable would have taken for an option, is accepted.
  validator   lowering heave_budget_n below hover_thrust_n makes core/'s
              blimp_params_validate(), compiled into the S3, pull the
              feed-forward down with it (skipped on the mock, which has
              neither field).
  timing      the hand-over takes a spinlock once per tick: the loop's
              overruns and skipped ticks are counted over a quiet stretch and
              over a burst of sets, which should add none.
  --reboot    after a power cycle the count is back to 0 and nothing is
              marked: RAM only, params.c is the one place gains are kept.

It talks only to the console (UDP 7201), so the ground station may stay open,
but not with its console dock in use: the firmware runs one command at a time.
It refuses to run while a motor is driven, because from increment 3 on these
values are the controller's gains. Two checks stay by hand: the boot log on
USB (no errors, 'param' in the banner) and a capture from the Bench page,
whose .json must carry the listing under "params". Standard library only.
"""
import argparse
import os
import re
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blimp_link as bl  # noqa: E402

BURST = 20                  # sets in the timing burst
QUIET_S = 2.0               # the stretch it is compared with

_PERIOD = re.compile(r'period ([\d.]+)/([\d.]+)/([\d.]+) ms')
_EXEC = re.compile(r'exec (\d+)/(\d+)/(\d+) us')
_HOLDS = re.compile(r'holds the set after (\d+) of (\d+)')

results = []


def report(status, what, detail=''):
    results.append(status)
    print('%-5s %s%s' % (status, what, ('  (%s)' % detail) if detail else ''), flush=True)


def check(cond, what, detail=''):
    report('PASS' if cond else 'FAIL', what, detail)
    return cond


class Console:
    def __init__(self, host, port):
        self.con = bl.BlimpConsole(host, port)
        self.times = []

    def run(self, line, timeout=5.0):
        t0 = time.perf_counter()
        out = self.con.run(line, timeout=timeout, idle=timeout)
        self.times.append(time.perf_counter() - t0)
        return out

    def params(self):
        return bl.parse_params(self.run('param'))

    def ctl(self):
        """The figures of 'ctl': the last window, the cumulative counters, and
        which 'param' change the loop holds."""
        out = self.run('ctl')
        d = {'text': out}
        for key in ('overruns', 'missed'):
            m = re.search(key + r' (\d+)', out)
            d[key] = int(m.group(1)) if m else None
        m = _PERIOD.search(out)
        d['period_ms'] = tuple(float(v) for v in m.groups()) if m else None
        m = _EXEC.search(out)
        d['exec_us'] = tuple(int(v) for v in m.groups()) if m else None
        m = _HOLDS.search(out)
        d['holds'] = (int(m.group(1)), int(m.group(2))) if m else None
        return d

    def close(self):
        self.con.close()


def motors_driven(con):
    """The command column of 'motor', or None when it cannot be read."""
    cmds = []
    for line in con.run('motor').splitlines():
        parts = line.split()
        if parts and parts[0] in bl.MOTOR_ORDER and parts[-1].endswith('%'):
            try:
                cmds.append(float(parts[-2]))
            except ValueError:
                pass
    return cmds if len(cmds) == len(bl.MOTOR_ORDER) else None


def restore(con, start):
    """Every parameter back to what it was: params.c's own value through
    'param --reset' (exact, where a typed copy of the printed digits might
    not be), anyone's earlier change by its printed value. Passes repeat,
    because one value can clamp another (heave_budget_n and hover_thrust_n)."""
    if not start['changed']:
        con.run('param --reset')
    for _ in range(3):
        now = con.params()
        todo = [n for n in start['values']
                if n in now['values'] and (now['values'][n] != start['values'][n]
                                           or (n in now['changed']) != (n in start['changed']))]
        if not todo:
            return True
        for n in todo:
            if n in start['changed']:
                con.run('param %s %r' % (n, start['values'][n]))
            else:
                con.run('param --reset %s' % n)
    return False


def run_checks(con, args):
    try:
        start = con.params()
    except ValueError as e:
        report('FAIL', "'param' answers with a listing", str(e))
        return
    listing = con.run('param')
    rows = len(bl.parse_params(listing)['values'])
    check(rows == start['count'], "the full listing arrives over Wi-Fi: %d of %d rows" % (rows, start['count']),
          '%d bytes' % len(listing))
    print('      %d parameters, %d changes since boot, %d marked %s' % (
        start['count'], start['changes'], len(start['changed']),
        start['changed'] if start['changed'] else ''))

    ctl0 = con.ctl()
    if ctl0['holds'] is None:
        report('FAIL', "'ctl' says which 'param' change the loop holds", ctl0['text'].strip()[:120])
    else:
        check(ctl0['holds'][0] == start['changes'] == ctl0['holds'][1],
              'the loop holds the current set: after %d of %d changes' % ctl0['holds'])

    # set, and the loop takes it
    kp0 = start['values']['alt.kp']
    kp1 = 0.4 if kp0 != 0.4 else 0.5
    out = con.run('param alt.kp %g' % kp1)
    after = con.params()
    check('alt.kp = %g' % kp1 in out and after['values']['alt.kp'] == kp1
          and after['changes'] == start['changes'] + 1 and 'alt.kp' in after['changed'],
          'param alt.kp %g: stored, counted, marked' % kp1, out.splitlines()[0] if out else 'no answer')
    ctl1 = con.ctl()
    check(ctl1['holds'] is not None and ctl1['holds'][0] == after['changes'],
          'the loop took it at its next tick: after %s of %s changes'
          % (ctl1['holds'] if ctl1['holds'] else ('?', '?')))

    # refusals, and a negative value
    for line, why in (('param alt.kp 40', 'a typo for 0.40, outside [0, 20]'),
                      ('param alt.kp abc', 'not a number')):
        out = con.run(line)
        now = con.params()
        check('not changed' in out and now['values']['alt.kp'] == kp1 and now['changes'] == after['changes'],
              '%s: refused, nothing changed' % line, why)
    if 'alt.i_min' in start['values'] and 'alt.i_max' in start['values']:
        lo = -0.1 if start['values']['alt.i_min'] != -0.1 else -0.11
        out = con.run('param alt.i_min %g' % lo)
        now = con.params()
        check(now['values']['alt.i_min'] == lo, 'param alt.i_min %g: a negative value is accepted' % lo,
              out.splitlines()[0] if out else 'no answer')
        above = start['values']['alt.i_max'] + 0.1
        out = con.run('param alt.i_min %g' % above)
        now = con.params()
        check('not changed' in out and now['values']['alt.i_min'] == lo,
              'param alt.i_min %g: above alt.i_max, refused' % above)
    else:
        report('SKIP', 'negative value and clamp order', 'no alt.i_min/alt.i_max in this listing')

    # core/'s validator, running on the S3
    v = start['values']
    if 'heave_budget_n' in v and 'hover_thrust_n' in v and v['hover_thrust_n'] > 0.01:
        low = round(v['hover_thrust_n'] / 2, 4)
        out = con.run('param heave_budget_n %g' % low)
        now = con.params()
        check('also clamped' in out and 'hover_thrust_n' in out and now['values']['hover_thrust_n'] <= low + 1e-6,
              'param heave_budget_n %g: blimp_params_validate() pulls hover_thrust_n down with it' % low,
              'hover_thrust_n %g' % now['values']['hover_thrust_n'])
    else:
        report('SKIP', "core/'s validator on the S3", 'no heave_budget_n/hover_thrust_n in this listing')

    # timing: a quiet stretch against a burst of sets
    q0 = con.ctl()
    time.sleep(QUIET_S)
    q1 = con.ctl()
    con.times = []
    for i in range(BURST):
        con.run('param alt.kp %g' % (0.4 if i % 2 else 0.5))
    rtt = sorted(con.times)
    b1 = con.ctl()
    if None in (q0['overruns'], q1['overruns'], b1['overruns'], q0['missed'], q1['missed'], b1['missed']):
        report('FAIL', "'ctl' reports overruns and missed ticks", b1['text'].strip()[:120])
    else:
        quiet = (q1['overruns'] - q0['overruns'], q1['missed'] - q0['missed'])
        burst = (b1['overruns'] - q1['overruns'], b1['missed'] - q1['missed'])
        detail = 'quiet %.0f s: +%d overruns, +%d missed' % (QUIET_S, quiet[0], quiet[1])
        if sum(quiet):
            detail += '; the quiet stretch had some too, so not only the sets'
        check(sum(burst) == 0, '%d sets in %.1f s: +%d overruns, +%d missed ticks in the loop'
              % (BURST, sum(rtt), burst[0], burst[1]), detail)
    print('      a set takes %.0f ms median, %.0f ms worst, over the air' % (
        1000 * statistics.median(rtt), 1000 * rtt[-1]))
    if b1['period_ms'] and b1['exec_us']:
        print('      last window: period %.2f/%.2f/%.2f ms, exec %d/%d/%d us (min/max/mean)'
              % (b1['period_ms'] + b1['exec_us']))
    b2 = con.ctl()
    final = con.params()
    check(b2['holds'] is not None and b2['holds'][0] == final['changes'],
          'and the loop holds the last of them: after %s of %s'
          % (b2['holds'] if b2['holds'] else ('?', '?')))

    check(restore(con, start), 'every parameter back as it was found')

    if args.reboot:
        input('\nPower-cycle the blimp now: unplug whatever powers it (the cell, or the USB cable) and plug\n'
              'it back. The laptop should rejoin its Wi-Fi by itself; if it does not, rejoin it. Then press\n'
              'Enter. ')
        deadline = time.monotonic() + args.reboot_wait
        fresh, why = None, ''
        while time.monotonic() < deadline:
            try:
                fresh = con.params()
                break
            except (TimeoutError, ConnectionError, OSError, ValueError) as e:
                why = str(e)
                time.sleep(2.0)
        if fresh is None:
            report('FAIL', 'the blimp answers after the power cycle', why)
        else:
            check(fresh['changes'] == 0 and not fresh['changed'],
                  "after the power cycle: 0 changes, nothing marked (params.c's values)",
                  '%d changes, marked %s' % (fresh['changes'], fresh['changed']))
    else:
        report('SKIP', 'reboot restores params.c', 'run with --reboot')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default=bl.BLIMP_IP)
    ap.add_argument('--console-port', type=int, default=bl.CONSOLE_PORT)
    ap.add_argument('--reboot', action='store_true', help='end with a power cycle and check the defaults return')
    ap.add_argument('--reboot-wait', type=float, default=90.0, help='s to wait for the console after it '
                    '(default %(default)s)')
    args = ap.parse_args(argv)

    con = Console(args.host, args.console_port)
    try:
        try:
            cmds = motors_driven(con)
        except (TimeoutError, ConnectionError) as e:
            sys.exit('no console at %s:%d: %s\nIs this computer on the blimp\'s Wi-Fi (ICARUS-...)?'
                     % (args.host, args.console_port, e))
        if cmds is None:
            sys.exit("could not read the motor commands from 'motor'; not running")
        if any(abs(c) > 0.005 for c in cmds):
            sys.exit('a motor is driven (%s): run this with the vehicle at rest' % cmds)
        print("'param' on %s:%d\n" % (args.host, args.console_port))
        try:
            run_checks(con, args)
        except (TimeoutError, ConnectionError) as e:
            report('FAIL', 'the console stopped answering', str(e))
    finally:
        con.close()

    print('\nBy hand: the boot log on USB (no errors, \'param\' in the banner), and a Bench-page capture\n'
          'whose .json carries "params".')
    failed = results.count('FAIL')
    print('\n%d passed, %d failed, %d skipped' % (results.count('PASS'), failed, results.count('SKIP')))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
