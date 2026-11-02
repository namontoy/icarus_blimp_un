#!/usr/bin/env python3
"""
Pick the red, individual balloons out of the Open Images download (2026-11-01).

fetch_openimages_balloons.py brings every Balloon image: bunches, arches,
balloon walls, hot-air balloons, every colour. What target_cam needs is what
the race shows it: a few balloons, each one apart, red (the waypoints are red
balloons, Rule 13's course). This script reads OUT/boxes.csv and the images
and keeps an image only when

  - it has at most --max-balloons boxes (default 4), none of them group-of
    (one box around a cluster) and none a depiction (a drawing, a print);
  - its largest box covers at least --min-box-frac of the frame (default
    0.005), so there is a balloon big enough to look at;
  - no person confirmed a hot-air balloon in it (the image-level labels
    "Hot air balloon" and "Hot air ballooning" in OUT/image_labels.csv,
    which the downloader writes; --exclude-label adds others). The Balloon
    class includes hot-air balloons, about a fifth of the red ones, and their
    shape is not the race's;
  - every box is red (--require all, the default) or at least one is
    (--require any): a box is red when at least --red-frac (default 0.40)
    of the pixels inside its inscribed ellipse are red. A balloon fills that
    ellipse and not the corners of its box, which are background.

A pixel is red when its hue lies in --hue-min..--hue-max degrees (default
-10..12: pink and magenta lie on the blue side of red, orange balloons
photograph at 15-35 on the other), its saturation is at least
--min-sat (0.55; pink balloons are pale red) and its value at least --min-val
(0.20).
This is a coarse screen on Flickr JPEGs, not the camera's colour table: it is
there to sort photos, and its thresholds were set by looking at the contact
sheets it writes, which is how its result should be checked too. Foil
balloons fail it more often than latex ones: their highlights are white and
their reflections take the colour of the room.

What the screen cannot tell apart (a red beach ball, a balloon animal, a
hot-air balloon nobody labelled, a box drawn on the wrong thing) goes in
OUT/red_reject.txt, one ImageID per line with a comment after '#', kept
outside red_individual/ so a rerun does not lose it. Look at every sheet
after a run and add what does not belong.

Output, in OUT/red_individual/ (OUT defaults to data/external/openimages_balloon/):
  images/<id>.jpg     hard links to the downloaded files (no second copy)
  labels/<id>.txt     the YOLO labels, unchanged
  selection.csv       every box of the kept images with its red fraction
  rejected_red.csv    images that passed the count but failed the colour
  sheets/sheet_NN.jpg contact sheets, 6x5, boxes drawn and red fraction printed

Usage (from target_cam/, in the targetcam env; needs numpy and PyQt6, which
the viewer installs):
  python host/train/select_red_balloons.py
  python host/train/select_red_balloons.py --require any --max-balloons 3
"""
from __future__ import annotations

import argparse
import csv
import os
import shutil
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PyQt6.QtCore import QRectF, Qt
from PyQt6.QtGui import QColor, QFont, QGuiApplication, QImage, QPainter, QPen

TARGET_CAM = Path(__file__).resolve().parents[2]

SHEET_COLS, SHEET_ROWS, CELL = 6, 5, 320


def load_rgb(path: Path, max_side: int = 640) -> np.ndarray | None:
    """Decode with Qt (no OpenCV or Pillow in the viewer's env), scaled so
    the long side is max_side: the colour screen needs no more."""
    img = QImage(str(path))
    if img.isNull():
        return None
    if max(img.width(), img.height()) > max_side:
        img = img.scaled(max_side, max_side, Qt.AspectRatioMode.KeepAspectRatio,
                         Qt.TransformationMode.SmoothTransformation)
    img = img.convertToFormat(QImage.Format.Format_RGB888)
    w, h, bpl = img.width(), img.height(), img.bytesPerLine()
    buf = np.frombuffer(img.constBits().asstring(img.sizeInBytes()), np.uint8)
    return buf.reshape(h, bpl)[:, :w * 3].reshape(h, w, 3).copy()


def red_mask(rgb: np.ndarray, hue_min: float, hue_max: float, min_sat: float,
             min_val: float) -> np.ndarray:
    f = rgb.astype(np.float32) / 255.0
    r, g, b = f[..., 0], f[..., 1], f[..., 2]
    mx, mn = f.max(axis=2), f.min(axis=2)
    delta = mx - mn
    sat = np.where(mx > 0, delta / np.maximum(mx, 1e-6), 0.0)
    # Hue of a pixel whose maximum is red: 60 * (g - b) / delta, in -60..60.
    # Red pixels are exactly those whose maximum is the red channel and whose
    # hue is near 0, so the other two branches of the hue formula are not needed.
    hue = 60.0 * (g - b) / np.maximum(delta, 1e-6)
    return ((mx == r) & (hue >= hue_min) & (hue <= hue_max)
            & (sat >= min_sat) & (mx >= min_val))


def ellipse_red_frac(mask: np.ndarray, box: dict) -> float:
    h, w = mask.shape
    x0, x1 = float(box["XMin"]) * w, float(box["XMax"]) * w
    y0, y1 = float(box["YMin"]) * h, float(box["YMax"]) * h
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    rx, ry = max((x1 - x0) / 2, 0.5), max((y1 - y0) / 2, 0.5)
    ys, xs = np.mgrid[int(y0):max(int(np.ceil(y1)), int(y0) + 1),
                      int(x0):max(int(np.ceil(x1)), int(x0) + 1)]
    ys, xs = np.clip(ys, 0, h - 1), np.clip(xs, 0, w - 1)
    inside = ((xs + 0.5 - cx) / rx) ** 2 + ((ys + 0.5 - cy) / ry) ** 2 <= 1.0
    if not inside.any():
        return 0.0
    return float(mask[ys[inside], xs[inside]].mean())


def box_frac(b: dict) -> float:
    return (float(b["XMax"]) - float(b["XMin"])) * (float(b["YMax"]) - float(b["YMin"]))


def write_sheets(rows: list[tuple[str, list[dict], list[float]]], images: Path,
                 sheets: Path) -> int:
    sheets.mkdir(parents=True, exist_ok=True)
    for old in sheets.glob("sheet_*.jpg"):
        old.unlink()
    per = SHEET_COLS * SHEET_ROWS
    font = QFont("Monospace", 10)
    n_sheets = 0
    for start in range(0, len(rows), per):
        sheet = QImage(SHEET_COLS * CELL, SHEET_ROWS * CELL, QImage.Format.Format_RGB888)
        sheet.fill(QColor(30, 30, 30))
        p = QPainter(sheet)
        p.setFont(font)
        for k, (image_id, boxes, fracs) in enumerate(rows[start:start + per]):
            img = QImage(str(images / f"{image_id}.jpg"))
            if img.isNull():
                continue
            img = img.scaled(CELL - 4, CELL - 22, Qt.AspectRatioMode.KeepAspectRatio,
                             Qt.TransformationMode.SmoothTransformation)
            ox = (k % SHEET_COLS) * CELL + 2
            oy = (k // SHEET_COLS) * CELL + 2
            p.drawImage(ox, oy, img)
            p.setPen(QPen(QColor(0, 255, 0), 2))
            for b in boxes:
                p.drawRect(QRectF(ox + float(b["XMin"]) * img.width(),
                                  oy + float(b["YMin"]) * img.height(),
                                  box_frac_w(b) * img.width(),
                                  (float(b["YMax"]) - float(b["YMin"])) * img.height()))
            p.setPen(QColor(230, 230, 230))
            p.drawText(ox, oy + CELL - 8, f"{image_id[:8]} red "
                       + " ".join(f"{f:.2f}" for f in fracs))
        p.end()
        n_sheets += 1
        sheet.save(str(sheets / f"sheet_{n_sheets:02d}.jpg"), "JPG", 85)
    return n_sheets


def box_frac_w(b: dict) -> float:
    return float(b["XMax"]) - float(b["XMin"])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", type=Path, default=TARGET_CAM / "data" / "external" / "openimages_balloon")
    ap.add_argument("--max-balloons", type=int, default=4)
    ap.add_argument("--min-box-frac", type=float, default=0.005)
    ap.add_argument("--require", choices=["all", "any"], default="all")
    ap.add_argument("--red-frac", type=float, default=0.40)
    ap.add_argument("--hue-min", type=float, default=-10.0)
    ap.add_argument("--hue-max", type=float, default=12.0)
    ap.add_argument("--min-sat", type=float, default=0.55)
    ap.add_argument("--exclude-label", action="append", default=[],
                    metavar="NAME", help="also drop images a person confirmed this "
                    "image-level label in (DisplayName, e.g. 'Toy'); repeatable")
    ap.add_argument("--min-val", type=float, default=0.20)
    ap.add_argument("--no-sheets", action="store_true")
    args = ap.parse_args()

    app = QGuiApplication(sys.argv[:1])  # QPainter and the JPEG plugin need one
    src = args.src
    images = src / "images"
    with (src / "boxes.csv").open(newline="") as f:
        by_image: dict[str, list[dict]] = {}
        for b in csv.DictReader(f):
            by_image.setdefault(b["ImageID"], []).append(b)

    excluded_names = {"Hot air balloon", "Hot air ballooning", *args.exclude_label}
    excluded_ids: set[str] = set()
    labels_csv = src / "image_labels.csv"
    if labels_csv.exists():
        with labels_csv.open(newline="", encoding="utf-8") as f:
            for r in csv.DictReader(f):
                if r["Confidence"] == "1" and r["DisplayName"] in excluded_names:
                    excluded_ids.add(r["ImageID"])
    else:
        print(f"no {labels_csv}: hot-air balloons are not excluded "
              "(run fetch_openimages_balloons.py again to get it)")

    reject_txt = src / "red_reject.txt"
    if reject_txt.exists():
        for line in reject_txt.read_text().splitlines():
            image_id = line.split("#", 1)[0].strip()
            if image_id:
                excluded_ids.add(image_id)

    counted = {"too many": 0, "group-of": 0, "depiction": 0, "small": 0,
               "label or reject list": 0, "no file": 0}
    candidates = []
    for image_id in sorted(by_image):
        bs = by_image[image_id]
        if len(bs) > args.max_balloons:
            counted["too many"] += 1
        elif any(b["IsGroupOf"] == "1" for b in bs):
            counted["group-of"] += 1
        elif any(b["IsDepiction"] == "1" for b in bs):
            counted["depiction"] += 1
        elif max(box_frac(b) for b in bs) < args.min_box_frac:
            counted["small"] += 1
        elif image_id in excluded_ids:
            counted["label or reject list"] += 1
        elif not (images / f"{image_id}.jpg").exists():
            counted["no file"] += 1
        else:
            candidates.append(image_id)
    print(f"{len(by_image)} images; {len(candidates)} with 1..{args.max_balloons} "
          "single balloons, big enough; dropped "
          + ", ".join(f"{n} {k}" for k, n in counted.items() if n))

    kept, rejected = [], []
    for n, image_id in enumerate(candidates, 1):
        rgb = load_rgb(images / f"{image_id}.jpg")
        if rgb is None:
            continue
        mask = red_mask(rgb, args.hue_min, args.hue_max, args.min_sat, args.min_val)
        bs = by_image[image_id]
        fracs = [ellipse_red_frac(mask, b) for b in bs]
        red = [fr >= args.red_frac for fr in fracs]
        ok = all(red) if args.require == "all" else any(red)
        (kept if ok else rejected).append((image_id, bs, fracs))
        if n % 100 == 0 or n == len(candidates):
            sys.stdout.write(f"\r  colour: {n}/{len(candidates)}, {len(kept)} red")
            sys.stdout.flush()
    print()

    out = src / "red_individual"
    if out.exists():
        shutil.rmtree(out)  # derived entirely from src; rebuilt every run
    (out / "images").mkdir(parents=True)
    (out / "labels").mkdir()
    fields = ["ImageID", "Split", "XMin", "XMax", "YMin", "YMax", "RedFrac",
              "IsOccluded", "IsTruncated", "IsInside"]
    for name, rows in (("selection.csv", kept), ("rejected_red.csv", rejected)):
        with (out / name).open("w", newline="") as f:
            w = csv.DictWriter(f, fields, extrasaction="ignore")
            w.writeheader()
            for image_id, bs, fracs in rows:
                for b, fr in zip(bs, fracs):
                    w.writerow({**b, "RedFrac": f"{fr:.3f}"})
    for image_id, _, _ in kept:
        os.link(images / f"{image_id}.jpg", out / "images" / f"{image_id}.jpg")
        lab = src / "labels" / f"{image_id}.txt"
        if lab.exists():
            shutil.copy(lab, out / "labels" / lab.name)
    (out / "classes.txt").write_text("balloon\n")
    sheets = 0 if args.no_sheets else write_sheets(kept, images, out / "sheets")
    n_boxes = sum(len(bs) for _, bs, _ in kept)
    print(f"{len(kept)} images ({n_boxes} balloons) in {out}"
          + (f"; {sheets} contact sheets in {out / 'sheets'}" if sheets else ""))
    del app


if __name__ == "__main__":
    main()
