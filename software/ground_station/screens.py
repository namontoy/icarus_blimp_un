"""Render the ground station's pages to PNG files with no display and no hardware.

    python -m software.ground_station.screens                    # build/screens/*.png, PC and Steam Deck sets
    python -m software.ground_station.screens --out docs/images  # somewhere else
    python -m software.ground_station.screens --set deck         # one set only

For looking at the window as it would appear on a laptop (1680 × 1000, PC
input) and on the Steam Deck (1280 × 800, Steam Deck input) without owning
either, and for putting an honest picture in a report. The demo blimp and demo
camera run in-process, as `--demo` does, so the pages carry live-looking data;
the Control page is shown disarmed (its pads at rest, the Deck set with its
"controls not visible" notice, since no pad is plugged in here). A throwaway
QSettings keeps the run from touching the operator's remembered window.

Qt's offscreen platform draws with the bundled fonts exactly as the real window
does, so the pixels are the real thing; only the window frame is absent.
"""
import argparse
import os
import subprocess
import sys
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
# blimp_link.py and blimp_log.py live one level up, in software/, as __main__.py arranges too.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt6.QtCore import QSettings
from PyQt6.QtWidgets import QApplication

from . import app as A

SETS = {'pc': {'size': (1680, 1000), 'mode': 'pc', 'note': 'laptop, PC input'},
        'deck': {'size': (1280, 800), 'mode': 'deck', 'note': 'Steam Deck, 1280 x 800, Steam Deck input'}}
PAGES = ('flight', 'control', 'bench')
SETTLE_S = 1.6          # the demo blimp's telemetry and sensors take about a second to fill the plots


def spin(app, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(0.005)


def render(out, sets, pages):
    app = QApplication(sys.argv[:1])
    paths = []
    for name in sets:
        s = SETS[name]
        ini = os.path.join(out, '_%s.ini' % name)
        if os.path.exists(ini):
            os.remove(ini)
        args = A.parse_args(['--demo', '--mode', s['mode']])
        station, win = A.build(args, app, QSettings(ini, QSettings.Format.IniFormat))
        win.resize(*s['size'])
        win.show()
        station.start()
        spin(app, SETTLE_S)
        for page in pages:
            win.show_page(page)
            spin(app, 0.4)
            path = os.path.join(out, '%s-%s.png' % (name, page))
            win.grab().save(path)
            paths.append(path)
            print('%s  (%s, %s)' % (path, s['note'], page))
        win.close()
        spin(app, 0.2)
        A.teardown(station, win)
        if os.path.exists(ini):
            os.remove(ini)
    return paths


def main(argv=None):
    ap = argparse.ArgumentParser(prog='python -m software.ground_station.screens',
                                 description='Render the ground station pages to PNG files, offscreen, no hardware.')
    ap.add_argument('--out', default=os.path.join('build', 'screens'), help='folder for the PNGs (default %(default)s)')
    ap.add_argument('--set', choices=tuple(SETS), action='append', help='pc or deck; default both')
    ap.add_argument('--page', choices=PAGES, action='append', help='default all three')
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    sets, pages = args.set or tuple(SETS), args.page or PAGES
    if len(sets) > 1:
        # One process per set: the demo blimp binds UDP 7200/7201 and its serve
        # thread keeps them until the process ends, so a second set in the
        # same process could not start its own mock.
        for name in sets:
            cmd = [sys.executable, '-m', 'software.ground_station.screens', '--out', args.out, '--set', name]
            for p in pages:
                cmd += ['--page', p]
            if subprocess.call(cmd) != 0:
                sys.exit('the %s set failed' % name)
        return
    render(args.out, sets, pages)


if __name__ == '__main__':
    main()
