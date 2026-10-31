"""The forward camera through the blimp, live, motors off: what the S3
receives from target_cam on its UART and the bearing core/ would steer on.
The check before any balloon-follow run (hold_test.py --follow):

    python software/cam_watch.py                 # 60 s of lines, 3 a second
    python software/cam_watch.py --seconds 0     # until Ctrl-C

1. Wiring: the rows/s figure should be about 12 (target_cam paces at 12.5).
   0 with 'nothing has arrived' is the wire, the camera's power or its
   ground; rows rejected or 'no row parses' is the baud or another firmware.
2. Sign: hold the balloon about 1.5 m in front of the nose, then move it to
   the blimp's own LEFT (seen from behind, looking where the nose points: the
   side of the motor the team calls FR). The bearing must go + and the line
   say LEFT. If it says RIGHT, the camera is mirrored or turned for this
   mounting: 'cam --sign 1' on the console fixes it until the next boot, and
   CONFIG_CAM_FWD_BEARING_SIGN for good. Then to the right: - and RIGHT.
3. Range: walk the balloon away until the detection drops, and note how far.

Console only (UDP 7201), so the ground station may stay open. Standard
library only.
"""
import argparse
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blimp_link as bl  # noqa: E402


def summary(text):
    """One line from 'cam''s reply."""
    rate = re.search(r'(\d+) rows/s', text)
    age = re.search(r'last row (\d+) ms ago', text)
    det = re.search(r'latest: DETECTED\s+az ([-+\d.]+) deg\s+el ([-+\d.]+) deg\s+cx (\d+)\s+cy (\d+)\s+area (\d+)',
                    text)
    bearing = re.search(r"core/'s bearing: ([-+\d.]+) deg, to the blimp's (\w+)", text)
    parts = ['%3s rows/s' % (rate.group(1) if rate else '?')]
    if age and int(age.group(1)) > 500:
        parts.append('LAST ROW %s ms AGO' % age.group(1))
    if 'nothing has arrived' in text:
        parts.append('NOTHING HAS ARRIVED: power, the U0T -> GPIO6 wire, the grounds')
    elif 'no row parses' in text:
        parts.append('BYTES BUT NO ROWS: the baud, or not target_cam (cam --raw)')
    elif det and bearing:
        parts.append('balloon: bearing %+6.1f deg  %-5s  (camera az %+6.1f, el %+5.1f, cx %3s cy %3s, area %4s px)'
                     % (float(bearing.group(1)), bearing.group(2), float(det.group(1)), float(det.group(2)),
                        det.group(3), det.group(4), det.group(5)))
    elif 'nothing detected' in text:
        parts.append('no balloon')
    if 'streaming off' in text:
        parts.append("THE CAMERA'S ROWS ARE OFF: cam --tx s")
    return '  '.join(parts)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default=bl.BLIMP_IP)
    ap.add_argument('--console-port', type=int, default=bl.CONSOLE_PORT)
    ap.add_argument('--seconds', type=float, default=60.0, help='0: until Ctrl-C (default %(default)s)')
    ap.add_argument('--period', type=float, default=0.33, help='s between lines (default %(default)s)')
    args = ap.parse_args(argv)

    con = bl.BlimpConsole(args.host, args.console_port)
    try:
        first = con.run('cam', timeout=5.0, idle=1.0)
        if 'UART1' not in first:
            sys.exit("no 'cam' command on the blimp: flash the firmware with the forward camera's link\n" + first)
        print(first.strip())
        print('-' * 72)
        t0 = time.monotonic()
        while args.seconds <= 0 or time.monotonic() - t0 < args.seconds:
            text = con.run('cam', timeout=3.0, idle=0.3)
            print('%6.1f s  %s' % (time.monotonic() - t0, summary(text)), flush=True)
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    except (TimeoutError, ConnectionError, OSError) as e:
        sys.exit('console: %s (on the blimp\'s Wi-Fi?)' % e)
    finally:
        con.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
