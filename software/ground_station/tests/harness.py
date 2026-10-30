"""What the ground station's offscreen tests share: the import paths, the
check log, free UDP ports for the demo blimp, a throwaway settings file, and
the Qt event pump.

Each test is a script that exits non-zero on a failed check:

    python software/ground_station/tests/test_bridge.py
    python tools/verify.py --gui          # all of them, with the repo's .venv

They need PyQt6 (the .venv has it) but no hardware and no display: Qt runs
on its offscreen platform unless QT_QPA_PLATFORM says otherwise. The demo
blimp (software/mock_blimp.py, in-process) gets free ports, so a mock already
running on 7200 is left alone.
"""
import os
import socket
import sys
import tempfile
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
for _p in (os.path.join(ROOT, 'software'), ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

failures = []


def check(cond, what):
    print(('ok   ' if cond else 'FAIL ') + what, flush=True)
    if not cond:
        failures.append(what)
    return cond


def near(a, b, tol=1e-6):
    return abs(a - b) <= tol


def free_ports(n=2):
    """n UDP ports nothing listens on right now, all held at once while they
    are picked so they differ."""
    socks = []
    for _ in range(n):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('127.0.0.1', 0))
        socks.append(s)
    ports = [s.getsockname()[1] for s in socks]
    for s in socks:
        s.close()
    return ports


def settings_path():
    """An empty QSettings ini in a fresh temporary folder, so a run leaves the
    operator's window settings as they were."""
    return os.path.join(tempfile.mkdtemp(prefix='ground-station-test-'), 'settings.ini')


def wait_for(cond, timeout, step=0.01):
    """True as soon as cond() is, False after timeout."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return cond()


def pump(app, seconds):
    """Run the Qt event loop by hand for this long."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(0.005)


def finish():
    print('\n%d checks failed' % len(failures) if failures else '\nall checks passed', flush=True)
    for f in failures:
        print('  ' + f)
    return 1 if failures else 0
