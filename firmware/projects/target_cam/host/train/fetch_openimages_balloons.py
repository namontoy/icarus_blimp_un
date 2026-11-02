#!/usr/bin/env python3
"""
Download the Open Images "Balloon" boxes and their images (2026-11-01).

Open Images V7 has a boxable class Balloon (/m/01j51): about 3 430 images and
14 100 boxes over train, validation and test, mostly party latex and foil
balloons with some hot-air balloons. It is the only large public set of boxed
balloon photos. What it is good for here, and what it is not:

  - Not the stage 1 colour table. main/models/lut_model.* is learned from the
    OV2640's own RGB565 values under its fixed white-balance gains; a Flickr
    JPEG has other colours, so its pixels would teach the table the wrong
    camera. Keep training the table on our own sessions (train_lut.py).
  - Good for what is about shape: the blob features of phase B, a tiny
    detector, and hard negatives (red things that are not balloons, balloons
    that are not red). Foil (mylar) balloons are in it, unlabelled as such:
    the red and foil ones have to be picked out by hand.

What the script does, each step cached under OUT/cache/ so a second run (or a
run with more splits) starts where the last one stopped:

 1. The box annotations of each split, filtered to Balloon. Train's file is
    2.2 GB and is streamed, never stored: only the balloon lines are kept. A
    broken stream resumes with an HTTP Range request from the last whole line,
    because the university network drops long downloads.
 2. The image metadata of the same images (author, licence, Flickr URLs,
    rotation), streamed the same way (train's file is 640 MB). The licence and
    author are kept for attribution: the images are listed as CC BY 2.0, which
    requires crediting the author; the boxes are Google's, CC BY 4.0.
 3. The human-verified image-level labels of the same images (train's file
    is 2.7 GB, streamed the same way): which other things a person confirmed
    (1) or denied (0) in each picture. select_red_balloons.py uses them to
    drop hot-air balloons, which the Balloon class includes.
 4. The images themselves, from the public CVDF bucket on S3 (no account),
    in parallel. One already on disk is not fetched again.
 5. OUT/boxes.csv (every box of every kept image, with its flags),
    OUT/images.csv (one row per image: split, size of box, licence, author,
    URLs), OUT/image_labels.csv (ImageID, label, its name, 1 or 0),
    OUT/labels/<id>.txt (YOLO format, class 0 = balloon, normalized centre
    and size) and OUT/ATTRIBUTION.md.

Which images are kept. The filters act on whole images, never on single
boxes, because a dropped box leaves a balloon in the picture with no label,
and a detector trained on it learns that a balloon is background:

  --exclude-group-of   drop an image with any group-of box (one box drawn
                       around a cluster of balloons, no box per balloon)
  --exclude-depiction  drop an image with any depiction (a drawing, a print,
                       a toy) rather than a real balloon
  --min-box-frac F     keep an image only if its largest box covers at least
                       F of the frame; small boxes in it are kept as labels
  --rotated skip|keep  the metadata gives some images a Rotation (degrees to
                       turn the file counter-clockwise to stand it upright).
                       Open Images does not say whether those boxes refer to
                       the file or to the turned image, so by default such
                       images are skipped (a few percent); "keep" downloads
                       them as they are and records the angle in images.csv.
  --max-images N       a fixed sample: the first N kept images by ImageID,
                       which is a hash, so the sample is random but the same
                       every run.

Usage (from target_cam/, in the viewer's targetcam env or any Python 3.8+;
standard library only):
  python host/train/fetch_openimages_balloons.py --splits validation test --dry-run
  python host/train/fetch_openimages_balloons.py --splits validation --max-images 20
  python host/train/fetch_openimages_balloons.py            (all splits, ~3 400 images, ~1 GB)

The default output, data/external/openimages_balloon/, is inside data/, which git
ignores: the images are not ours to commit.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import csv
import io
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

BALLOON = "/m/01j51"

# Train's boxes are the V6 file (V7 did not change the boxes); validation and
# test are the V5 files, which V6 and V7 still point to.
BOX_URLS = {
    "train": "https://storage.googleapis.com/openimages/v6/oidv6-train-annotations-bbox.csv",
    "validation": "https://storage.googleapis.com/openimages/v5/validation-annotations-bbox.csv",
    "test": "https://storage.googleapis.com/openimages/v5/test-annotations-bbox.csv",
}
# The metadata with the Rotation column; the boxable train set has not changed
# since this release.
META_URLS = {
    "train": "https://storage.googleapis.com/openimages/2018_04/train/train-images-boxable-with-rotation.csv",
    "validation": "https://storage.googleapis.com/openimages/2018_04/validation/validation-images-with-rotation.csv",
    "test": "https://storage.googleapis.com/openimages/2018_04/test/test-images-with-rotation.csv",
}
# Human-verified image-level labels (V7) and the names of all 20 931 classes.
LABEL_URLS = {
    "train": "https://storage.googleapis.com/openimages/v7/oidv7-train-annotations-human-imagelabels.csv",
    "validation": "https://storage.googleapis.com/openimages/v7/oidv7-val-annotations-human-imagelabels.csv",
    "test": "https://storage.googleapis.com/openimages/v7/oidv7-test-annotations-human-imagelabels.csv",
}
CLASS_NAMES_URL = "https://storage.googleapis.com/openimages/v7/oidv7-class-descriptions.csv"
IMAGE_URL = "https://open-images-dataset.s3.amazonaws.com/{split}/{image_id}.jpg"

USER_AGENT = "icarus-blimp-target-cam/1.0 (fetch_openimages_balloons.py)"
TARGET_CAM = Path(__file__).resolve().parents[2]

BOX_FIELDS = ["ImageID", "Split", "XMin", "XMax", "YMin", "YMax", "Confidence",
              "IsOccluded", "IsTruncated", "IsGroupOf", "IsDepiction", "IsInside"]
IMAGE_FIELDS = ["ImageID", "Split", "Boxes", "MaxBoxFrac", "Rotation", "License",
                "Author", "AuthorProfileURL", "Title", "OriginalURL",
                "OriginalLandingURL"]


def log(msg: str) -> None:
    print(msg, flush=True)


def open_url(url: str, start: int = 0, timeout: float = 60):
    headers = {"User-Agent": USER_AGENT}
    if start:
        headers["Range"] = f"bytes={start}-"
    return urllib.request.urlopen(urllib.request.Request(url, headers=headers),
                                  timeout=timeout)


def stream_filter(url: str, dest: Path, keep, label: str, attempts: int = 20) -> None:
    """Stream a CSV from url and write its header and the lines keep() accepts
    to dest. Progress lives in dest.part (the lines kept so far) and
    dest.offset (bytes of the source consumed, always at a line end), so a
    dropped connection, or a killed run, resumes from there instead of from
    zero. dest itself appears only once the whole file has been read."""
    if dest.exists():
        return
    part = dest.with_name(dest.name + ".part")
    off_file = dest.with_name(dest.name + ".offset")
    offset = int(off_file.read_text()) if part.exists() and off_file.exists() else 0
    if offset == 0:
        part.write_bytes(b"")
    failures = 0
    while True:
        try:
            resp = open_url(url, offset)
            if offset and resp.status != 206:
                log(f"  {label}: the server ignored the resume; starting over")
                resp.close()
                offset = 0
                part.write_bytes(b"")
                continue
            total = offset + int(resp.headers.get("Content-Length") or 0)
            t0, last = time.monotonic(), 0.0
            with resp, part.open("ab") as out:
                for raw in resp:
                    if not raw.endswith(b"\n"):
                        break  # the stream ended inside a line: resume before it
                    if offset == 0 or keep(raw):
                        out.write(raw)
                    offset += len(raw)
                    now = time.monotonic()
                    if now - last > 5:
                        out.flush()
                        off_file.write_text(str(offset))
                        last = now
                        if total:
                            rate = (offset / 1e6) / max(now - t0, 1e-3)
                            sys.stdout.write(f"\r  {label}: {offset / 1e6:7.0f} of "
                                             f"{total / 1e6:.0f} MB ({rate:.1f} MB/s)  ")
                            sys.stdout.flush()
                out.flush()
                off_file.write_text(str(offset))
            if total and offset < total:
                raise IOError(f"stream ended at {offset} of {total} bytes")
            if total:
                sys.stdout.write("\n")
            break
        except (urllib.error.URLError, OSError, IOError) as exc:
            failures += 1
            if failures >= attempts:
                raise SystemExit(f"{label}: giving up after {failures} failures "
                                 f"({exc}); run again to resume at byte {offset}")
            wait = min(60, 2 ** failures)
            log(f"\n  {label}: {exc}; resuming at byte {offset} in {wait} s")
            time.sleep(wait)
    part.replace(dest)
    off_file.unlink(missing_ok=True)


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def fetch_boxes(split: str, cache: Path) -> list[dict]:
    dest = cache / f"{split}-balloon-boxes.csv"
    needle = b"," + BALLOON.encode() + b","
    stream_filter(BOX_URLS[split], dest, lambda line: needle in line,
                  f"{split} boxes")
    rows = [r for r in read_csv(dest) if r["LabelName"] == BALLOON]
    for r in rows:
        r["Split"] = split
    return rows


def fetch_meta(split: str, ids: set[str], cache: Path) -> dict[str, dict]:
    dest = cache / f"{split}-balloon-images.csv"
    wanted = {i.encode() for i in ids}
    stream_filter(META_URLS[split], dest,
                  lambda line: line.split(b",", 1)[0] in wanted,
                  f"{split} metadata")
    return {r["ImageID"]: r for r in read_csv(dest)}


def fetch_image_labels(split: str, ids: set[str], cache: Path) -> list[dict]:
    dest = cache / f"{split}-balloon-imagelabels.csv"
    wanted = {i.encode() for i in ids}
    stream_filter(LABEL_URLS[split], dest,
                  lambda line: line.split(b",", 1)[0] in wanted,
                  f"{split} image labels")
    return read_csv(dest)


def fetch_class_names(cache: Path) -> dict[str, str]:
    dest = cache / "class-descriptions.csv"
    stream_filter(CLASS_NAMES_URL, dest, lambda line: True, "class names")
    with dest.open(newline="", encoding="utf-8") as f:
        return {row[0]: row[1] for row in csv.reader(f) if len(row) >= 2}


def flag(row: dict, name: str) -> bool:
    return row.get(name, "0") == "1"


def box_frac(row: dict) -> float:
    return (float(row["XMax"]) - float(row["XMin"])) * (float(row["YMax"]) - float(row["YMin"]))


def select(boxes: list[dict], meta: dict[str, dict], args) -> tuple[list[str], dict]:
    """Apply the image-level filters; return the kept ImageIDs and the count
    of images each filter removed."""
    by_image: dict[str, list[dict]] = {}
    for b in boxes:
        by_image.setdefault(b["ImageID"], []).append(b)
    dropped = {"group-of": 0, "depiction": 0, "small": 0, "rotated": 0}
    kept = []
    for image_id in sorted(by_image):
        bs = by_image[image_id]
        if args.exclude_group_of and any(flag(b, "IsGroupOf") for b in bs):
            dropped["group-of"] += 1
            continue
        if args.exclude_depiction and any(flag(b, "IsDepiction") for b in bs):
            dropped["depiction"] += 1
            continue
        if max(box_frac(b) for b in bs) < args.min_box_frac:
            dropped["small"] += 1
            continue
        rot = rotation(meta.get(image_id, {}))
        if args.rotated == "skip" and rot not in (0, None):
            dropped["rotated"] += 1
            continue
        kept.append(image_id)
    return kept, dropped


def rotation(m: dict) -> int | None:
    value = (m.get("Rotation") or "").strip()
    if value in ("", "nan"):
        return None
    return int(float(value))


def download(job: tuple[str, str, Path], attempts: int = 4) -> tuple[str, str]:
    split, image_id, path = job
    if path.exists() and path.stat().st_size > 0:
        return image_id, "have"
    tmp = path.with_name(path.name + ".part")
    for n in range(attempts):
        try:
            with open_url(IMAGE_URL.format(split=split, image_id=image_id), timeout=30) as r:
                data = r.read()
            if not data.startswith(b"\xff\xd8"):
                return image_id, "not a JPEG"
            tmp.write_bytes(data)
            tmp.replace(path)
            return image_id, "new"
        except urllib.error.HTTPError as exc:
            if exc.code == 404:
                return image_id, "missing (404)"
            err = str(exc)
        except (urllib.error.URLError, OSError) as exc:
            err = str(exc)
        time.sleep(2 ** n)
    return image_id, f"failed: {err}"


def write_outputs(out: Path, kept: list[str], boxes: list[dict],
                  meta: dict[str, dict], have: set[str],
                  image_labels: list[dict], names: dict[str, str]) -> None:
    keep = set(kept) & have
    by_image: dict[str, list[dict]] = {}
    for b in boxes:
        if b["ImageID"] in keep:
            by_image.setdefault(b["ImageID"], []).append(b)

    with (out / "boxes.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, BOX_FIELDS, extrasaction="ignore")
        w.writeheader()
        for image_id in sorted(by_image):
            w.writerows(by_image[image_id])

    labels = out / "labels"
    labels.mkdir(exist_ok=True)
    for image_id, bs in by_image.items():
        lines = []
        for b in bs:
            x0, x1 = float(b["XMin"]), float(b["XMax"])
            y0, y1 = float(b["YMin"]), float(b["YMax"])
            lines.append(f"0 {(x0 + x1) / 2:.6f} {(y0 + y1) / 2:.6f} "
                         f"{x1 - x0:.6f} {y1 - y0:.6f}\n")
        (labels / f"{image_id}.txt").write_text("".join(lines))
    (out / "classes.txt").write_text("balloon\n")

    with (out / "image_labels.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["ImageID", "LabelName", "DisplayName", "Confidence"])
        for r in sorted(image_labels, key=lambda r: (r["ImageID"], r["LabelName"])):
            if r["ImageID"] in keep:
                w.writerow([r["ImageID"], r["LabelName"], names.get(r["LabelName"], ""),
                            int(float(r["Confidence"]))])

    licences: dict[str, int] = {}
    with (out / "images.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, IMAGE_FIELDS, extrasaction="ignore")
        w.writeheader()
        for image_id in sorted(by_image):
            bs = by_image[image_id]
            m = meta.get(image_id, {})
            lic = m.get("License") or "unknown"
            licences[lic] = licences.get(lic, 0) + 1
            w.writerow({**m, "ImageID": image_id, "Split": bs[0]["Split"],
                        "Boxes": len(bs),
                        "MaxBoxFrac": f"{max(box_frac(b) for b in bs):.4f}",
                        "Rotation": rotation(m) if rotation(m) is not None else "",
                        "License": lic})

    lic_lines = "\n".join(f"- {n} images: {lic}" for lic, n in
                          sorted(licences.items(), key=lambda kv: -kv[1]))
    (out / "ATTRIBUTION.md").write_text(f"""# Open Images: Balloon

Fetched {time.strftime('%Y-%m-%d')} by host/train/fetch_openimages_balloons.py.

- Bounding boxes: Open Images Dataset V7, Google LLC, CC BY 4.0
  (https://storage.googleapis.com/openimages/web/index.html).
- Images: from Flickr through the Open Images dataset. Google lists them as
  CC BY 2.0 but does not guarantee the licence of each image. The author and
  licence of every image are in images.csv (Author, AuthorProfileURL,
  License, OriginalLandingURL); credit them in anything published.

{len(by_image)} images:
{lic_lines}

Citation: Kuznetsova et al., "The Open Images Dataset V4", IJCV 2020.
Not for redistribution from this repository: data/ is gitignored.
""")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--splits", nargs="+", default=["train", "validation", "test"],
                    choices=list(BOX_URLS))
    ap.add_argument("--out", type=Path, default=TARGET_CAM / "data" / "external" / "openimages_balloon")
    ap.add_argument("--max-images", type=int, default=0, help="0: all")
    ap.add_argument("--workers", type=int, default=16)
    ap.add_argument("--exclude-group-of", action="store_true")
    ap.add_argument("--exclude-depiction", action="store_true")
    ap.add_argument("--min-box-frac", type=float, default=0.0)
    ap.add_argument("--rotated", choices=["skip", "keep"], default="skip")
    ap.add_argument("--dry-run", action="store_true",
                    help="fetch the annotations and count; no images")
    args = ap.parse_args()

    out = args.out
    cache = out / "cache"
    images = out / "images"
    cache.mkdir(parents=True, exist_ok=True)

    boxes: list[dict] = []
    image_labels: list[dict] = []
    meta: dict[str, dict] = {}
    split_of: dict[str, str] = {}
    for split in args.splits:
        log(f"{split}: balloon boxes")
        b = fetch_boxes(split, cache)
        ids = {r["ImageID"] for r in b}
        log(f"  {len(b)} boxes in {len(ids)} images")
        log(f"{split}: image metadata")
        m = fetch_meta(split, ids, cache)
        missing = len(ids - m.keys())
        if missing:
            log(f"  {missing} images have no metadata (no licence record; downloaded anyway)")
        log(f"{split}: image-level labels")
        lab = fetch_image_labels(split, ids, cache)
        log(f"  {len(lab)} labels")
        boxes += b
        image_labels += lab
        meta.update(m)
        split_of.update({i: split for i in ids})

    kept, dropped = select(boxes, meta, args)
    total = len(split_of)
    log(f"\n{total} images with balloons; dropped "
        + ", ".join(f"{n} {why}" for why, n in dropped.items() if n)
        + ("" if any(dropped.values()) else "none")
        + f"; {len(kept)} kept")
    if args.max_images and len(kept) > args.max_images:
        kept = kept[:args.max_images]
        log(f"  sample of {len(kept)} (first by ImageID)")
    if args.dry_run:
        return

    images.mkdir(exist_ok=True)
    jobs = [(split_of[i], i, images / f"{i}.jpg") for i in kept]
    counts: dict[str, int] = {}
    have: set[str] = set()
    failures: list[str] = []
    with concurrent.futures.ThreadPoolExecutor(args.workers) as pool:
        for n, (image_id, status) in enumerate(pool.map(download, jobs), 1):
            key = status.split(":")[0]
            counts[key] = counts.get(key, 0) + 1
            if status in ("new", "have"):
                have.add(image_id)
            else:
                failures.append(f"{image_id},{status}")
            if n % 50 == 0 or n == len(jobs):
                sys.stdout.write(f"\r  images: {n}/{len(jobs)}  "
                                 + "  ".join(f"{k} {v}" for k, v in counts.items()))
                sys.stdout.flush()
    sys.stdout.write("\n")
    if failures:
        (out / "failed.txt").write_text("\n".join(failures) + "\n")
        log(f"  {len(failures)} not downloaded, listed in {out / 'failed.txt'}; "
            "run again to retry")

    write_outputs(out, kept, boxes, meta, have, image_labels, fetch_class_names(cache))
    log(f"\n{len(have)} images in {images}\nlabels, boxes.csv, images.csv, image_labels.csv and "
        f"ATTRIBUTION.md in {out}")


if __name__ == "__main__":
    main()
