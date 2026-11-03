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
    --window (2026-11-03) also shows the timeline on the PC's screen, for a
        room that can see it: what to do now in big letters, the seconds to
        the next beep, the next step, and the command's latest line (green
        when cam_watch.py sees a balloon). --plan NAME picks the steps, one per
        10 s, from PLANS (the course room's camera check, C0); --steps
        'A|B|C' gives them. The window closes itself a few seconds after the
        command ends.

    python software/bench_cues.py watch c0-sign.log --window --plan sign -- python -u software/cam_watch.py --seconds 62

Standard library only (the window is tkinter, which ships with Python).
Windows beeps through winsound (the speakers); other systems ring the
terminal bell, which may be silent.
"""
import math
import subprocess
import sys
import threading
import time

QUIET = '--quiet' in sys.argv
T0 = time.monotonic()
LEAD_S = 10           # watch: seconds to get in place before the check starts
STEP_S = 10.0         # a beep, and a step, every this many seconds
STEPS_N = 6           # steps in a check: 0, 10, ... 50 s; the end at 60 s

# The camera check of the course room (docs/flight-tests.md, C0): (what the
# window says above the steps, the steps), one step per 10 s.
PLANS = {
    'sign': ("The sign check: the balloon about 1.5 m in front of the blimp's nose, level with the camera",
             ['Centre, 1.5 m ahead', "To the blimp's LEFT, 0.5 m", 'Centre', "To the blimp's RIGHT, 0.5 m",
              'Centre', 'Hidden behind your back']),
    'walk1': ('Balloon 1: carry the blimp along the first line, gondola at 1.3 m, nose to the balloon',
              ['At the start: 2.0 m', '1.5 m from balloon 1', '1.0 m', '0.7 m', 'The envelope just touching it',
               'Back to 1.0 m']),
    'walk2': ('Balloon 2: carry the blimp along the second line, gondola at 1.3 m, nose to the balloon',
              ['2.0 m from balloon 2', '1.5 m', '1.0 m', '0.7 m', 'The envelope just touching it',
               'Back to 1.0 m']),
    'circle': ('The circle: turn the blimp slowly to its right, one full turn in 60 s, gondola at 1.3 m',
               ['Nose to balloon 1', '60 deg to the right', '120 deg', '180 deg: facing away', '240 deg',
                '300 deg, back toward balloon 1']),
}


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


def cues(mode, log_path, cmd, state):
    """The beeps and the command; state is what the window shows: lead_left
    (s before the start), t_start (the check's or the follow's start), line
    (the command's latest), done."""
    log = open(log_path, 'w', encoding='utf-8')
    try:
        if mode == 'watch':
            for left in range(LEAD_S, 0, -1):
                state['lead_left'] = left
                print('starting in %d s' % left, flush=True)
                if left <= 3:
                    beep(1, 120, 700)
                    time.sleep(0.73)
                else:
                    time.sleep(1.0)
            state['lead_left'] = 0
            state['t_start'] = time.monotonic()
            beep(1, 600, 1200)
            beep_at([(10, 1, 250), (20, 1, 250), (30, 1, 250), (40, 1, 250), (50, 1, 250), (55, 1, 250),
                     (60, 2, 600)], state['t_start'])
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in proc.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            log.write(line)
            state['line'] = line.strip()
            if mode == 'fly' and '-- follow the balloon' in line:
                state['t_start'] = time.monotonic()
                beep(1, 600, 1200)
                beep_at([(s, 1, 250) for s in range(10, 70, 10)], state['t_start'])
            if mode == 'fly' and 'landing from' in line:
                beep(2, 600)
        proc.wait()
        state['rc'] = proc.returncode
    finally:
        log.close()
        state['done'] = True


class Window:
    """The timeline on the PC's screen for the people in the room: the step
    to do now in big letters, a countdown to the next beep, the next step,
    and the command's latest line. Closing it does not stop the command."""

    def __init__(self, title, heading, steps, state):
        import tkinter as tk
        self.tk, self.steps, self.state = tk, steps, state
        self.t_done = None
        self.root = tk.Tk()
        self.root.title(title)
        self.root.attributes('-topmost', True)
        tk.Label(self.root, text=heading, wraplength=900, justify='center', font=('Segoe UI', 14)).pack(
            padx=14, pady=(10, 4))
        self.now = tk.Label(self.root, text='GET READY', font=('Segoe UI', 40, 'bold'), width=24, wraplength=900,
                            bg='#777777', fg='white', pady=16)
        self.now.pack(fill='x', padx=10)
        self.clock = tk.Label(self.root, text='', font=('Segoe UI', 30))
        self.clock.pack(pady=4)
        self.next = tk.Label(self.root, text='', font=('Segoe UI', 20), fg='#444444')
        self.next.pack(pady=(0, 8))
        self.cam = tk.Label(self.root, text='', font=('Consolas', 13), wraplength=900, justify='left',
                            bg='#777777', fg='white', padx=8, pady=6)
        self.cam.pack(fill='x', padx=10, pady=(0, 12))
        self.root.after(100, self.update)

    def _set(self, widget, **kw):
        try:
            widget.config(**kw)
        except self.tk.TclError:
            pass

    def step(self, k):
        return self.steps[k] if k < len(self.steps) else 'Step %d' % (k + 1)

    def update(self):
        s, now = self.state, time.monotonic()
        if s.get('done'):
            self.t_done = self.t_done or now
            self._set(self.now, text='DONE', bg='#777777')
            self._set(self.clock, text='')
            self._set(self.next, text='this window closes by itself')
            if now - self.t_done > 4.0:
                self.root.destroy()
                return
        elif s.get('t_start') is None:
            self._set(self.now, text='GET READY: %s' % self.step(0), bg='#c07a10')
            left = s.get('lead_left')
            self._set(self.clock, text='starting in %d s' % left if left else 'waiting for the start')
            self._set(self.next, text='')
        else:
            el = now - s['t_start']
            k = int(el // STEP_S)
            if k >= STEPS_N:
                self._set(self.now, text='STAY STILL: the end', bg='#777777')
                self._set(self.clock, text='')
                self._set(self.next, text='')
            else:
                self._set(self.now, text=self.step(k), bg='#1e8c3a')
                self._set(self.clock, text='%d s' % math.ceil(STEP_S - el % STEP_S))
                self._set(self.next, text='next: %s' % (self.step(k + 1) if k + 1 < STEPS_N else 'the end'))
        line = s.get('line', '')
        seen = 'balloon:' in line
        self._set(self.cam, text=('SEEN   ' if seen else '') + line, bg='#1e8c3a' if seen else '#777777')
        self.root.after(100, self.update)

    def run(self):
        self.root.mainloop()


def main():
    argv = sys.argv[1:]
    cut = argv.index('--') if '--' in argv else len(argv)
    pre, cmd = [a for a in argv[:cut] if a != '--quiet'], argv[cut + 1:]
    window = '--window' in pre
    pre = [a for a in pre if a != '--window']
    heading, steps = '', []
    for opt in ('--plan', '--steps'):
        if opt in pre:
            i = pre.index(opt)
            if i + 1 >= len(pre):
                sys.exit('%s takes a value' % opt)
            value = pre[i + 1]
            del pre[i:i + 2]
            if opt == '--plan':
                if value not in PLANS:
                    sys.exit('--plan is one of %s' % ', '.join(sorted(PLANS)))
                heading, steps = PLANS[value]
            else:
                steps = [s.strip() for s in value.split('|')]
    if len(pre) != 2 or pre[0] not in ('watch', 'fly') or not cmd:
        sys.exit(__doc__)
    mode, log_path = pre
    state = {'t_start': None, 'line': '', 'done': False, 'rc': None}
    if not window:
        cues(mode, log_path, cmd, state)
        return state['rc']
    worker = threading.Thread(target=cues, args=(mode, log_path, cmd, state), daemon=True)
    worker.start()
    Window('Bench cues: %s' % log_path, heading, steps, state).run()
    worker.join()     # a window closed early leaves the command to finish
    return state['rc']


if __name__ == '__main__':
    sys.exit(main())
