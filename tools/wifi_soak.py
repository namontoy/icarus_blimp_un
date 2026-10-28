#!/usr/bin/env python3
"""wifi_soak.py -- hold the S3's Wi-Fi link open for a while and count how often it drops.

    python tools/wifi_soak.py baseline --connection ICARUS-EB03F9            300 s
    python tools/wifi_soak.py isr_on --connection ICARUS-EB03F9 --seconds 600

WHY THIS EXISTS. On 2026-10-28 the access point dropped the laptop every ~14 s
with one firmware and never with the previous one, and the person at the bench
could not tell good from bad in under a few minutes of watching. This script
made the comparison mechanical, and found the cause (esp_timer ISR dispatch,
docs/wifi-drop-report.md). Run it before and after any change that can touch
the radio: sdkconfig, an ESP-IDF upgrade, a new high-priority task or timer.

WHAT IT DOES. Opens the S3's USB console (which resets the chip) and logs it
with timestamps; after 5 s has NetworkManager re-join the access point; then
sends link heartbeats through software/blimp_link.py and counts, per second,
the TELEMETRY packets that arrive. Afterwards it reads the system journal for
the same span. Summary line:

    beacon losses   wpa_supplicant's CTRL-EVENT-BEACON-LOSS: the laptop stopped
                    hearing the AP (the symptom of 2026-10-28)
    connects        CTRL-EVENT-CONNECTED: one is the join this script makes
    S3 left         the firmware's "station ... left" lines; one is the
                    re-join at the start
    boots           "rst:" lines in the console after the port was opened
    silent seconds  seconds with no telemetry at all

A good run reads 0 beacon losses, at most 2 connects, at most 1 left, 0 silent
seconds. It flashes nothing: flash the build under test first, with
`idf.py -p /dev/ttyACM0 flash` or esptool.

LINUX ONLY: it drives nmcli and reads journalctl (the user must be able to
read the journal, e.g. through the adm or systemd-journal group). It needs
pyserial, which the ESP-IDF Python environment already has. It never sends a
motor command. Logs go to build/soak/<label>/ (serial.log, telemetry.log,
summary.txt).
"""
import argparse
import datetime
import os
import re
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'software'))

from blimp_link import BlimpLink  # noqa: E402

try:
    import serial
except ImportError:
    sys.exit('wifi_soak.py needs pyserial; run it with the ESP-IDF Python, or pip install '
             'pyserial into the env you use')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('label', help='name of this run; logs go to build/soak/<label>/')
    ap.add_argument('--connection', required=True,
                    help="NetworkManager profile of the blimp's access point (ICARUS-xxxxxx)")
    ap.add_argument('--seconds', type=int, default=300, help='soak length (default %(default)s)')
    ap.add_argument('--port', default='/dev/ttyACM0', help='USB console (default %(default)s)')
    ap.add_argument('--host', default='192.168.4.1', help='the blimp (default %(default)s)')
    args = ap.parse_args()

    out = os.path.join(ROOT, 'build', 'soak', args.label)
    os.makedirs(out, exist_ok=True)
    since = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    stop = threading.Event()
    t0 = time.monotonic()

    def serial_logger():
        s = serial.Serial(args.port, 115200, timeout=0.2)
        with open(os.path.join(out, 'serial.log'), 'w') as f:
            buf = b''
            while not stop.is_set():
                buf += s.read(4096)
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    f.write('%8.2f %s\n' % (time.monotonic() - t0,
                                            line.decode(errors='replace').rstrip()))
                    f.flush()
        s.close()

    th = threading.Thread(target=serial_logger, daemon=True)
    th.start()
    # Opening the port resets the S3; give the access point time to come up.
    time.sleep(5)
    subprocess.run(['nmcli', 'con', 'down', args.connection], capture_output=True)
    r = subprocess.run(['nmcli', '--wait', '30', 'con', 'up', args.connection],
                       capture_output=True, text=True)
    print('nmcli up:', ((r.stdout + r.stderr).strip().splitlines() or ['?'])[-1])
    t_join = time.monotonic() - t0

    link = BlimpLink(args.host, 7200).start()
    silent, first_silent, heap = 0, None, []
    with open(os.path.join(out, 'telemetry.log'), 'w') as f:
        last = 0
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            time.sleep(1.0)
            n = link.rx_count
            d, last = n - last, n
            t = time.monotonic() - t0
            h = link.telemetry['free_heap'] if link.telemetry else 0
            f.write('%8.1f rx %3d heap %d\n' % (t, d, h))
            f.flush()
            if d == 0:
                silent += 1
                if first_silent is None:
                    first_silent = t
            elif h:
                heap.append(h)
    link.close()
    stop.set()
    th.join(2)

    j = subprocess.run(['journalctl', '--since', since, '--no-pager', '-o', 'cat'],
                       capture_output=True, text=True).stdout
    ser = open(os.path.join(out, 'serial.log')).read()
    summary = ('%s: %d s, joined at %.0f s; beacon losses %d, connects %d, S3 left %d, boots %d, '
               'silent seconds %d (first at %s), heap %s..%s' % (
                   args.label, args.seconds, t_join,
                   j.count('CTRL-EVENT-BEACON-LOSS'),
                   j.count('CTRL-EVENT-CONNECTED'),
                   len(re.findall(r'net_wifi: station .* left', ser)),
                   ser.count('rst:'),
                   silent, '%.0f s' % first_silent if first_silent else '-',
                   min(heap) if heap else '-', max(heap) if heap else '-'))
    if not j:
        summary += ' (journal unreadable: beacon losses and connects not counted)'
    print(summary)
    with open(os.path.join(out, 'summary.txt'), 'w') as f:
        f.write(summary + '\n')


if __name__ == '__main__':
    main()
