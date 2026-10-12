#!/usr/bin/env python3
"""
Watch the CSV stream from the ESP32-CAM optical flow firmware.

Two modes:

  live     one refreshing line, for waving the camera around over a floor and
           seeing immediately whether the numbers make sense
  log      append raw CSV to a file, for a measured run you want to analyse

The firmware's '#' lines (banner, command replies) are passed through as-is.

Usage:
  ./flow_monitor.py --port /dev/ttyUSB0
  ./flow_monitor.py --port /dev/ttyUSB0 --log run1.csv
  ./flow_monitor.py --port /dev/ttyUSB0 --cmd h1500     # set height, then watch
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

COLUMNS = ["t_ms", "ok", "dx_px", "dy_px", "vx_mm_s", "vy_mm_s",
           "x_mm", "y_mm", "spread_px", "valid", "total", "flat",
           "edge", "ambig", "dt_ms", "proc_us"]


def parse_row(line):
    parts = line.split(",")
    if len(parts) != len(COLUMNS):
        return None
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        return None
    return dict(zip(COLUMNS, vals))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log", help="append raw CSV rows to this file")
    ap.add_argument("--cmd", action="append", default=[],
                    help="send this command on connect (repeatable)")
    ap.add_argument("--seconds", type=float,
                    help="stop after this long (default: run until Ctrl-C)")
    args = ap.parse_args()

    logf = open(args.log, "a") if args.log else None
    ser = serial.Serial(args.port, args.baud, timeout=1.0)

    # The ESP32 resets when the port opens; give the banner time to arrive.
    time.sleep(0.3)
    for c in args.cmd:
        ser.write((c + "\n").encode())
        time.sleep(0.1)

    started = time.time()
    rows = 0
    ok_rows = 0
    # Rolling stats, so a bad patch of floor shows up as it happens.
    recent = []

    print("# connected to %s at %d baud; Ctrl-C to stop" % (args.port, args.baud))
    try:
        while True:
            if args.seconds and time.time() - started > args.seconds:
                break
            raw = ser.readline().decode("utf-8", "replace").strip()
            if not raw:
                continue
            if raw.startswith("#") or raw.startswith("I (") or raw.startswith("E ("):
                print(raw)
                continue

            row = parse_row(raw)
            if row is None:
                print("?? " + raw)
                continue

            rows += 1
            if logf:
                logf.write(raw + "\n")
                logf.flush()

            if row["ok"]:
                ok_rows += 1
            recent.append(row)
            if len(recent) > 20:
                recent.pop(0)

            fps = len(recent) / max(
                1e-6, (recent[-1]["t_ms"] - recent[0]["t_ms"]) / 1000.0
            ) if len(recent) > 1 else 0.0
            yield_pct = 100.0 * ok_rows / rows

            sys.stdout.write(
                "\rflow %+7.2f,%+7.2f px  vel %+7.0f,%+7.0f mm/s  "
                "pos %+8.1f,%+8.1f mm  spread %4.2f  blocks %2d/%2d  "
                "%4.1f FPS  proc %4.0f us  yield %5.1f%%   "
                % (row["dx_px"], row["dy_px"], row["vx_mm_s"], row["vy_mm_s"],
                   row["x_mm"], row["y_mm"], row["spread_px"],
                   int(row["valid"]), int(row["total"]), fps,
                   row["proc_us"], yield_pct)
            )
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if logf:
            logf.close()
        print("\n# %d rows, %d with a measurement (%.1f%%)"
              % (rows, ok_rows, 100.0 * ok_rows / rows if rows else 0.0))


if __name__ == "__main__":
    main()
