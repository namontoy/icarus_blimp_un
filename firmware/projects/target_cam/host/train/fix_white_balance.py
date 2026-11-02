#!/usr/bin/env python3
"""
Correct the white balance of saved target_cam frames, after the fact (2026-11-01).

Every frame in red_s1-s5 and orange_s3-s5 was taken with `w0`, frozen gains
that were never recorded (docs/learned-detector-report.md §10.8), and the
2026-10-31 sessions with fixed gains calibrated in one room and carried to
others. The same red balloon then moves about 15-25 degrees of hue from one
session to the next (§10.7), more than the gap to a pink or orange one, and
neither the colour table nor a blob classifier can learn across that.

What this does, per session folder:

 1. Per frame, estimate the gains that make its near-grey pixels grey: walls,
    floor, ceiling, paper. Pixels that are clipped, very dark, red-dominant
    (balloon candidates even without paint) or painted balloon in the frame's
    .labels.png are left out. It iterates: under the current estimate, keep
    the pixels within 35 % of grey on every channel and average them again,
    so a coloured surface stops pulling once the estimate is close.
 2. Take the median over the session's frames (one lighting per session; a
    single frame dominated by a brown door does not decide it), separately
    for each set of camera gains the frames record (camera_cfg wb_gain): a
    session can hold frames taken before and after a `Wc`. --per-frame uses
    each frame's own estimate instead.
 3. Apply the gains in linear light (the OV2640's output is gamma encoded,
    taken as 2.2) with green held at 1, as the sensor's own gains do, and
    write the frame back as RGB565 in its own byte order.

The output folder gets the corrected .rgb565, a .png preview, the .labels.png
unchanged (no pixel moves, so the paint still fits) and the .json with a
"wb_correction" entry naming the method and the gains. The source folders are
never written.

What it cannot do (reasoned): undo mixed light (lamp plus window: one gain
per frame cannot balance both), bring back clipped highlights, or avoid
amplifying noise and the 5-bit steps of R and B where it boosts them. A room
with little grey in it biases the estimate; --gains R,B sets a session's
gains by hand (e.g. read from a wall you trust).

Measured 2026-11-01 on red_s1-s5's painted balloon pixels: the spread of the
sessions' median hue fell from 16 to 9 degrees, and the background's median
chroma fell in every session.

Usage:
  fix_white_balance.py data/originals/red_s6 ...          # -> data/sessions/red_s6
  fix_white_balance.py --out /tmp/wb --gains 1.3,1.2 data/originals/red_s4

It refuses to write into a session folder that already holds frames: the ones
in data/sessions/ carry labels made after the correction (2026-11-02), which a
second run would replace with the source's. --overwrite allows it.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "viewer"))
from tcview import labels as L          # noqa: E402

DATA = HERE.parents[1] / "data"         # target_cam/data (data/README.md)
GAMMA = 2.2
NEAR_GREY = 0.35        # max |channel/mean - 1| for a pixel to count as grey
MIN_GREY_PX = 300       # fewer than this: fall back to every usable pixel


def to_rgb(px: np.ndarray) -> np.ndarray:
    """RGB565 values -> float RGB in 0..1, each channel on its own scale."""
    return np.dstack([((px >> 11) & 31) / 31.0, ((px >> 5) & 63) / 63.0, (px & 31) / 31.0])


def to_565(rgb: np.ndarray) -> np.ndarray:
    q = np.clip(rgb, 0, 1)
    r = np.round(q[..., 0] * 31).astype(np.uint16)
    g = np.round(q[..., 1] * 63).astype(np.uint16)
    b = np.round(q[..., 2] * 31).astype(np.uint16)
    return (r << 11) | (g << 5) | b


def frame_gains(rgb: np.ndarray, balloon: np.ndarray) -> tuple[np.ndarray, int]:
    """Gains (R, G=1, B) that make this frame's near-grey pixels grey, and how
    many pixels decided it (0: no estimate, gains are 1)."""
    lin = rgb ** GAMMA
    mx, mn = rgb.max(-1), rgb.min(-1)
    usable = (mx < 0.97) & (mn > 0.06) & ~balloon
    usable &= ~((rgb[..., 0] > 1.4 * rgb[..., 1]) & (rgb[..., 0] > 1.4 * rgb[..., 2]))
    gains = np.ones(3)
    n = 0
    for _ in range(4):
        c = lin * gains
        with np.errstate(invalid="ignore", divide="ignore"):
            dev = np.abs(c / c.mean(-1, keepdims=True) - 1).max(-1)
        sel = usable & (dev < NEAR_GREY)
        if sel.sum() < MIN_GREY_PX:
            sel = usable
        n = int(sel.sum())
        if n < 50:
            return np.ones(3), 0
        m = c[sel].mean(0)
        gains = gains * (m[1] / m)
    return gains, n


def load(stem: Path):
    sf = L.load_frame(stem)
    f = sf.frame
    px = np.frombuffer(f.data, ">u2" if f.big_endian else "<u2").reshape(f.h, f.w)
    lab = L.load_labels(sf.label_path, f.w, f.h)
    return sf, px, lab


def write_preview(path: Path, rgb: np.ndarray):
    from PyQt6.QtGui import QImage
    im = np.ascontiguousarray((np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8))
    h, w = im.shape[:2]
    QImage(im.data, w, h, 3 * w, QImage.Format.Format_RGB888).save(str(path))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path, help="one folder per capture session")
    ap.add_argument("--out", type=Path, default=DATA / "sessions",
                    help="output root (default: target_cam/data/sessions)")
    ap.add_argument("--overwrite", action="store_true",
                    help="write into session folders that already hold frames (replaces their labels)")
    ap.add_argument("--per-frame", action="store_true", help="each frame's own gains, not the session median")
    ap.add_argument("--gains", help="R,B gains for every session given, instead of estimating")
    args = ap.parse_args(argv)

    out_root = args.out
    manual = None
    if args.gains:
        r, b = (float(x) for x in args.gains.split(","))
        manual = np.array([r, 1.0, b])

    for d in args.sessions:
        stems = L.list_frames(d)
        if not stems:
            print("%s: no saved frames" % d)
            continue
        frames = [load(s) for s in stems]
        est = [frame_gains(to_rgb(px), lab == L.BALLOON) for _, px, lab in frames]
        g = np.array([e[0] for e in est])
        good = np.array([e[1] > 0 for e in est])
        cam = [str((sf.meta.get("camera_cfg") or {}).get("wb_gain", "unrecorded"))
               for sf, _, _ in frames]
        group = {}
        for key in set(cam):
            sel = good & np.array([c == key for c in cam])
            group[key] = manual if manual is not None else (
                np.median(g[sel], 0) if sel.any() else np.ones(3))
        session = group[max(set(cam), key=cam.count)]
        od = out_root / d.resolve().name
        if od.resolve() == d.resolve():
            raise SystemExit("%s: the output would overwrite the source" % d)
        if L.list_frames(od) and not args.overwrite:
            raise SystemExit("%s already holds frames (and their labels); "
                             "--overwrite replaces them" % od)
        od.mkdir(parents=True, exist_ok=True)
        how = "manual" if manual is not None else ("per frame" if args.per_frame else "session median")
        print("%-20s %3d frames  gains R %.2f B %.2f (%s; per-frame R %.2f..%.2f, B %.2f..%.2f)"
              % (d.name, len(stems), session[0], session[2], how,
                 g[:, 0].min(), g[:, 0].max(), g[:, 2].min(), g[:, 2].max()))
        for key in sorted(set(cam)):
            if len(group) > 1:
                print("%20s camera gains %-12s %2d frames -> R %.2f B %.2f"
                      % ("", key, cam.count(key), group[key][0], group[key][2]))
        for (sf, px, lab), (fg, n), key in zip(frames, est, cam):
            gains = fg if (args.per_frame and manual is None and n) else group[key]
            fixed = np.clip((to_rgb(px) ** GAMMA) * gains, 0, 1) ** (1 / GAMMA)
            base = od / sf.stem.name
            to_565(fixed).astype(">u2" if sf.frame.big_endian else "<u2").tofile(
                str(base) + ".rgb565")
            meta = dict(sf.meta)
            meta["wb_correction"] = {
                "method": "grey pixels, %s, linear light (gamma %.1f)" % (how, GAMMA),
                "gains_rgb": [round(float(x), 4) for x in gains],
                "source": str(sf.stem),
                "tool": "host/train/fix_white_balance.py",
            }
            Path(str(base) + ".json").write_text(json.dumps(meta, indent=2))
            if sf.label_path.exists():
                shutil.copyfile(sf.label_path, str(base) + L.LABEL_SUFFIX)
            write_preview(Path(str(base) + ".png"), to_rgb(to_565(fixed)))
    print("written under", out_root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
