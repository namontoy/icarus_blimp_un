#!/usr/bin/env python3
"""Beeps from the PC that pace the people in the room, around a bench command.

The balloon tests (docs/flight-tests.md) have someone move the red balloon on
a timeline while the blimp watches or follows it, out of sight of the screen;
these beeps are their clock. The command's output is shown and kept in LOG.

    python software/bench_cues.py watch LOG -- python -u software/cam_watch.py --seconds 62
        motors off: 10 s to get in place (a short beep at 3, 2 and 1), a
        long high beep when the check starts, one beep at 10, 20, 30, 40 and
        50 s (go to the next spot), one at 55 s, two long beeps at 60 s (done).
    python software/bench_cues.py fly LOG -- python -u software/square_test.py ... --follow ...
        no lead (nobody holds the blimp): a long high beep when the script
        logs the start of the follow, one beep every 10 s for 60 s (go to the
        next spot), two long beeps when it logs the landing.
    --quiet prints the cues instead of beeping, to try it.

Standard library only. Windows beeps through winsound (the speakers); other
systems ring the terminal bell, which may be silent.
"""
import subprocess
import sys
import threading
import time

QUIET = '--quiet' in sys.argv
T0 = time.monotonic()


def beep(n=1, ms=200, hz=1000):
    for _ in range(n):
        if QUIET:
            print('[beep %d ms at %.1f s]' % (ms, time.monotonic() - T0), flush=True)
            time.sleep(ms / 1000.0)
        elif sys.platform == 'win32':
            import winsound
            winsound.Beep(hz, ms)
        else:
            sys.stderr.write('\a')
            sys.stderr.flush()
            time.sleep(ms / 1000.0)
        time.sleep(0.15)


def beep_at(times, start):
    """Beep at each (seconds after start, count, ms), from a thread, so the
    command's output keeps flowing."""
    def run():
        for s, n, ms in times:
            wait = start + s - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            beep(n, ms)
    threading.Thread(target=run, daemon=True).start()


def main():
    argv = [a for a in sys.argv[1:] if a != '--quiet']
    if len(argv) < 4 or argv[0] not in ('watch', 'fly') or '--' not in argv:
        sys.exit(__doc__)
    mode, log_path, cmd = argv[0], argv[1], argv[argv.index('--') + 1:]
    log = open(log_path, 'w', encoding='utf-8')
    if mode == 'watch':
        for left in range(10, 0, -1):
            print('starting in %d s' % left, flush=True)
            if left <= 3:
                beep(1, 120, 700)
                time.sleep(0.73)
            else:
                time.sleep(1.0)
        beep(1, 600, 1200)
        beep_at([(10, 1, 250), (20, 1, 250), (30, 1, 250), (40, 1, 250), (50, 1, 250), (55, 1, 250),
                 (60, 2, 600)], time.monotonic())
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    for line in proc.stdout:
        sys.stdout.write(line)
        log.write(line)
        if mode == 'fly' and '-- follow the balloon' in line:
            beep(1, 600, 1200)
            beep_at([(s, 1, 250) for s in range(10, 70, 10)], time.monotonic())
        if mode == 'fly' and 'landing from' in line:
            beep(2, 600)
    proc.wait()
    log.close()
    return proc.returncode


if __name__ == '__main__':
    sys.exit(main())
