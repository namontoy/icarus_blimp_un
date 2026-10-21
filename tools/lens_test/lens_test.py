#!/usr/bin/env python3
"""lens_test.py -- measure camera modules on an ESP32-CAM so they can be told apart.

    python tools/lens_test/lens_test.py --port COM3 info
    python tools/lens_test/lens_test.py --port COM3 live --label A
    python tools/lens_test/lens_test.py --port COM3 snap --label A
    python tools/lens_test/lens_test.py table [--markdown]

The board runs tools/lens_test/esp32cam and is reached either over the USB
cable (--port COM3, the one used for flashing; the laptop keeps its own Wi-Fi)
or over the board's access point (--host, default 192.168.4.1). Lens cards go
to --out (default build/lens_test/). OpenCV and pyserial are needed once:
    .venv\\Scripts\\python -m pip install -r tools/lens_test/requirements.txt

WHY THIS EXISTS. The marking on an OV2640 module's flex names the flex, not the
lens: the same marking is sold with 68 and 120 degree lenses, with and without
an infrared filter (camera report of 2026-10-21). Only measuring tells modules
apart. Five numbers do it:

  field of view   two clicks on a tape measure at a known distance;
                  standard lens ~52 deg horizontal, wide ~105, fisheye 130+
  sharpness       Laplacian variance of the image centre; relative, so compare
                  modules on the same target at the same distance
  vignetting      corner / centre brightness on a plain, evenly lit white wall;
                  wide lenses fall well below 1 even with lens correction on
  distortion      bow of a straight edge near the picture border, in % of its
                  length; ~0 for the standard lens, several % for wide and fisheye
  IR response     brightest spot with a TV remote pressed at the lens and
                  auto-exposure locked; a module without an IR-cut filter saturates

Everything is filed as a lens card, build/lens_test/<label>/card.json, next to
the images it came from, so the physical module can be tagged with its label.

Hotkeys in the live view:
  n  new module (asks for a label)     t  add a note
  s  save a full-size still            f  field of view (2 clicks)
  d  distortion (3 clicks)             v  record vignetting (white wall)
  k  record sharpness (asks distance)  i  record IR peak
  l  toggle lens correction (LENC)     e  lock / unlock auto-exposure
  c  toggle the sensor's colour bars   h  help      q  quit
The card is saved after every change.

Bench tool only: no perception logic lives here (AGENTS.md: that is core/'s job).
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass, field, fields
from datetime import datetime
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit("lens_test.py needs OpenCV and NumPy in the project venv:\n"
             "  .venv\\Scripts\\python -m pip install -r tools/lens_test/requirements.txt")

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = REPO_ROOT / "build" / "lens_test"
DEFAULT_HOST = "192.168.4.1"
WINDOW = "lens_test"
FONT = cv2.FONT_HERSHEY_SIMPLEX

HELP = """
  n new module    t note    s still    f field of view    d distortion
  v vignetting    k sharpness    i IR peak    l LENC    e AE lock    c colour bars
  h this help     q quit
"""


# --------------------------------------------------------------------------- #
# Board client                                                                #
# --------------------------------------------------------------------------- #

class Board:
    """HTTP client for tools/lens_test/esp32cam (control on :80, stream on :81)."""

    def __init__(self, host: str, timeout: float = 10.0) -> None:
        self.host = host
        self.timeout = timeout

    def _get(self, path: str, timeout: float | None = None) -> bytes:
        url = f"http://{self.host}{path}"
        with urllib.request.urlopen(url, timeout=timeout or self.timeout) as resp:
            return resp.read()

    def info(self) -> dict:
        return json.loads(self._get("/info"))

    def status(self) -> dict:
        return json.loads(self._get("/status"))

    def set(self, var: str, val: int) -> str:
        return self._get(f"/set?var={var}&val={val}").decode(errors="replace")

    def capture(self, size: str = "MAX") -> bytes:
        """One still. Switching the sensor to full size and back takes a second
        or two on the board, hence the long timeout and the retries."""
        last: Exception | None = None
        for _ in range(3):
            try:
                return self._get(f"/capture?size={size}", timeout=40)
            except (urllib.error.URLError, TimeoutError, OSError) as exc:
                last = exc
                time.sleep(1.0)
        raise RuntimeError(f"capture failed: {last}")

    def stream_url(self, size: str) -> str:
        return f"http://{self.host}:81/stream?size={size}"

    def frames(self, size: str) -> "MjpegReader":
        return MjpegReader(self.stream_url(size))


class MjpegReader(threading.Thread):
    """Reads the MJPEG stream in the background and keeps only the newest frame.

    Frames are cut at the JPEG start/end markers instead of the multipart
    headers, so the reader also works with the stock CameraWebServer firmware."""

    def __init__(self, url: str) -> None:
        super().__init__(daemon=True)
        self.url = url
        self.error: Exception | None = None
        self._lock = threading.Lock()
        self._jpg: bytes | None = None
        self._count = 0
        self._stop = threading.Event()
        self._resp = None

    def run(self) -> None:
        try:
            self._resp = urllib.request.urlopen(self.url, timeout=10)
        except Exception as exc:  # noqa: BLE001 - surfaced to the UI as-is
            self.error = exc
            return
        buf = b""
        try:
            while not self._stop.is_set():
                chunk = self._resp.read(16384)
                if not chunk:
                    break
                buf += chunk
                while True:
                    start = buf.find(b"\xff\xd8")
                    if start < 0:
                        buf = buf[-1:]
                        break
                    end = buf.find(b"\xff\xd9", start + 2)
                    if end < 0:
                        buf = buf[start:]
                        break
                    jpg = buf[start:end + 2]
                    buf = buf[end + 2:]
                    with self._lock:
                        self._jpg = jpg
                        self._count += 1
        except Exception as exc:  # noqa: BLE001
            if not self._stop.is_set():
                self.error = exc
        finally:
            try:
                self._resp.close()
            except Exception:  # noqa: BLE001
                pass

    def latest(self) -> tuple[bytes | None, int]:
        with self._lock:
            return self._jpg, self._count

    def stop(self) -> None:
        self._stop.set()
        try:
            if self._resp is not None:
                self._resp.close()
        except Exception:  # noqa: BLE001
            pass
        self.join(timeout=2.0)


class SerialBoard:
    """Same calls as Board, over the USB serial cable (UART0, --port/--baud).

    One request, one reply, under a lock shared with the frame thread. A frame
    reply is a header line "FRM <bytes> <w> <h>", exactly <bytes> of JPEG and a
    trailer, so no log line or JPEG byte can be mistaken for anything else."""

    def __init__(self, port: str, baud: int) -> None:
        try:
            import serial  # pyserial, listed in tools/lens_test/requirements.txt
        except ImportError:
            sys.exit("the serial mode needs pyserial:\n"
                     "  .venv\\Scripts\\python -m pip install -r tools/lens_test/requirements.txt")
        self.host = port
        self.lock = threading.RLock()
        self.ser = serial.Serial(port, baud, timeout=0.5, write_timeout=5)
        self._pending = b""
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self._cmd("quiet", "OK")    # the first command also silences the board's log output
        self._cmd("ping", "PONG")

    def _readline(self, end: float) -> bytes | None:
        """A complete line (with its newline) or None at the deadline."""
        while time.time() < end:
            chunk = self.ser.readline()
            if not chunk:
                continue
            self._pending += chunk
            if self._pending.endswith(b"\n"):
                line, self._pending = self._pending, b""
                return line
        return None

    def _cmd(self, cmd: str, expect: str, timeout: float = 8.0) -> str:
        with self.lock:
            self._pending = b""
            self.ser.reset_input_buffer()
            self.ser.write((cmd + "\n").encode())
            end = time.time() + timeout
            while True:
                line = self._readline(end)
                if line is None:
                    raise TimeoutError(f"no reply to {cmd!r} on {self.host}")
                text = line.decode("utf-8", "replace").rstrip("\r\n")
                if text.startswith(expect):
                    return text
                if text.startswith("ERR"):
                    raise RuntimeError(f"{cmd}: {text}")

    def info(self) -> dict:
        return json.loads(self._cmd("info", "JSON ")[5:])

    def status(self) -> dict:
        return json.loads(self._cmd("status", "JSON ")[5:])

    def set(self, var: str, val: int) -> str:
        return self._cmd(f"set {var} {val}", "OK")

    def capture(self, size: str = "MAX") -> bytes:
        return self.frame(size, timeout=40.0)

    def frame(self, size: str, timeout: float = 15.0) -> bytes:
        with self.lock:
            self._pending = b""
            self.ser.reset_input_buffer()
            self.ser.write(f"cap {size}\n".encode())
            end = time.time() + timeout
            while True:
                line = self._readline(end)
                if line is None:
                    raise TimeoutError("no frame from the board")
                if line.startswith(b"FRM "):
                    try:
                        n = int(line.split()[1])
                    except (IndexError, ValueError):
                        raise RuntimeError(f"bad frame header {line!r}") from None
                    data = self._read_exact(n, end)
                    trailer = self._read_exact(5, end)
                    if data is None or trailer != b"\nEND\n":
                        raise RuntimeError("frame cut short (bad cable, or lower --baud and the firmware's rate)")
                    return data
                if line.startswith(b"ERR"):
                    raise RuntimeError(line.decode("utf-8", "replace").strip())

    def _read_exact(self, n: int, end: float) -> bytes | None:
        buf = bytearray()
        while len(buf) < n and time.time() < end:
            chunk = self.ser.read(min(65536, n - len(buf)))
            if chunk:
                buf.extend(chunk)
        return bytes(buf) if len(buf) == n else None

    def frames(self, size: str) -> "SerialFrames":
        return SerialFrames(self, size)


class SerialFrames(threading.Thread):
    """Polls "cap <size>" back to back; the same interface as MjpegReader."""

    def __init__(self, board: SerialBoard, size: str) -> None:
        super().__init__(daemon=True)
        self.board = board
        self.size = size
        self.error: Exception | None = None
        self._lock = threading.Lock()
        self._jpg: bytes | None = None
        self._count = 0
        self._stop = threading.Event()

    def run(self) -> None:
        failures = 0
        while not self._stop.is_set():
            try:
                jpg = self.board.frame(self.size)
            except (RuntimeError, TimeoutError, OSError) as exc:
                failures += 1
                self.error = exc
                if failures in (1, 10, 100):
                    print(f"  stream: {exc}")
                time.sleep(0.5)
                continue
            failures = 0
            self.error = None
            with self._lock:
                self._jpg = jpg
                self._count += 1
            time.sleep(0.003)   # let a waiting capture or setting take the port

    def latest(self) -> tuple[bytes | None, int]:
        with self._lock:
            return self._jpg, self._count

    def stop(self) -> None:
        self._stop.set()
        self.join(timeout=20.0)


# --------------------------------------------------------------------------- #
# Measurements                                                                #
# --------------------------------------------------------------------------- #

def decode(jpg: bytes):
    return cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)


def gray_of(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def centre_roi(gray, frac: float = 0.25):
    h, w = gray.shape
    rh, rw = max(2, int(h * frac / 2)), max(2, int(w * frac / 2))
    cy, cx = h // 2, w // 2
    return gray[cy - rh:cy + rh, cx - rw:cx + rw]


def sharpness(gray) -> float:
    """Variance of the Laplacian over the central quarter of the frame. Scene and
    frame size dependent, so only compare like with like."""
    return float(cv2.Laplacian(centre_roi(gray), cv2.CV_64F).var())


def vignetting(gray, frac: float = 0.10) -> tuple[float, float, float]:
    """(corner/centre, centre mean, corner mean) over 10 % patches."""
    h, w = gray.shape
    ph, pw = max(2, int(h * frac)), max(2, int(w * frac))
    corners = [gray[:ph, :pw], gray[:ph, w - pw:], gray[h - ph:, :pw], gray[h - ph:, w - pw:]]
    centre = gray[h // 2 - ph // 2:h // 2 + ph // 2, w // 2 - pw // 2:w // 2 + pw // 2]
    centre_mean = float(centre.mean())
    corner_mean = float(np.mean([c.mean() for c in corners]))
    ratio = corner_mean / centre_mean if centre_mean > 0 else 0.0
    return ratio, centre_mean, corner_mean


def peak(gray) -> int:
    """Brightest 5x5 spot, which is what an IR remote's LED becomes."""
    return int(cv2.blur(gray, (5, 5)).max())


def solve_fov(u1: float, u2: float, w: int, h: int, distance_m: float, width_m: float) -> dict:
    """Field of view from two clicked pixel columns u1, u2 that are width_m apart
    on a wall distance_m in front of the lens, with the wall square to the axis.

    Pinhole model: distance * ((u2-cx)/f - (u1-cx)/f) = width, so f = span*D/W
    exactly, wherever the pair sits in the frame. Equidistant (fisheye) model:
    distance * (tan((u2-cx)/f) - tan((u1-cx)/f)) = width, solved by bisection.
    Both are reported; they agree for a rectilinear lens and the distortion
    measurement says which one to believe for a wide one."""
    cx = (w - 1) / 2.0
    u1, u2 = sorted((float(u1), float(u2)))
    span = u2 - u1
    if span < 2:
        raise ValueError("the two points must be apart horizontally")
    diag = math.hypot(w, h)

    f_pin = span * distance_m / width_m
    pin = {k: math.degrees(2 * math.atan(v / (2 * f_pin))) for k, v in (("h", w), ("v", h), ("d", diag))}

    def resid(f: float) -> float:
        return distance_m * (math.tan((u2 - cx) / f) - math.tan((u1 - cx) / f)) - width_m

    lo = max(abs(u1 - cx), abs(u2 - cx)) / (math.pi / 2) * 1.001 + 1e-9
    hi = f_pin * 100.0
    f_eq = float("nan")
    if resid(lo) > 0 > resid(hi):
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            if resid(mid) > 0:
                lo = mid
            else:
                hi = mid
        f_eq = 0.5 * (lo + hi)
    eq = {k: (math.degrees(v / f_eq) if f_eq == f_eq else float("nan"))
          for k, v in (("h", w), ("v", h), ("d", diag))}

    return {
        "hfov_pinhole_deg": round(pin["h"], 1), "vfov_pinhole_deg": round(pin["v"], 1),
        "dfov_pinhole_deg": round(pin["d"], 1), "f_px_pinhole": round(f_pin, 1),
        "hfov_equidistant_deg": round(eq["h"], 1), "vfov_equidistant_deg": round(eq["v"], 1),
        "dfov_equidistant_deg": round(eq["d"], 1), "f_px_equidistant": round(f_eq, 1),
        "span_px": round(span, 1), "coverage": round(span / w, 3),
    }


def solve_distortion(pts: list[tuple[float, float]], w: int, h: int) -> dict:
    """Bow of a straight edge from three clicks (end, middle, end): the middle
    point's distance from the chord, in % of the chord. Positive = barrel (the
    middle bows away from the image centre), negative = pincushion."""
    (x1, y1), (x2, y2), (x3, y3) = pts
    chord = math.hypot(x3 - x1, y3 - y1)
    if chord < 2:
        raise ValueError("the end points must be apart")
    sag = abs((x3 - x1) * (y2 - y1) - (y3 - y1) * (x2 - x1)) / chord
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    mid = ((x1 + x3) / 2, (y1 + y3) / 2)
    bow = math.hypot(x2 - cx, y2 - cy) - math.hypot(mid[0] - cx, mid[1] - cy)
    radius = sum(math.hypot(x - cx, y - cy) for x, y in pts) / 3 / (math.hypot(w, h) / 2)
    return {
        "sag_pct": round(100 * sag / chord * (1 if bow >= 0 else -1), 2),
        "kind": "barrel" if bow >= 0 else "pincushion",
        "chord_px": round(chord, 1),
        "radius_frac": round(radius, 2),
    }


# --------------------------------------------------------------------------- #
# Lens cards                                                                  #
# --------------------------------------------------------------------------- #

@dataclass
class Card:
    label: str
    created: str = ""
    sensor: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)
    fov: dict | None = None
    distortion: dict | None = None
    vignetting: dict = field(default_factory=dict)
    sharpness: list = field(default_factory=list)
    ir: dict | None = None
    stills: list = field(default_factory=list)


def now() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def clean_label(label: str) -> str:
    label = re.sub(r"[^A-Za-z0-9_.-]+", "_", label.strip()).strip("_")
    return label or "module"


def card_dir(out: Path, label: str) -> Path:
    return out / label


def load_card(out: Path, label: str, info: dict | None = None) -> Card:
    path = card_dir(out, label) / "card.json"
    if path.exists():
        data = json.loads(path.read_text(encoding="utf-8"))
        names = {f.name for f in fields(Card)}
        card = Card(**{k: v for k, v in data.items() if k in names})
    else:
        card = Card(label=label, created=now())
    if info and info.get("camera_ok"):
        card.sensor = {k: info.get(k) for k in ("sensor", "pid", "ver", "midh", "midl",
                                                 "max_width", "max_height", "support_jpeg")}
    return card


def save_card(out: Path, card: Card) -> Path:
    folder = card_dir(out, card.label)
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / "card.json"
    path.write_text(json.dumps(asdict(card), indent=2), encoding="utf-8")
    return path


def lens_class(card: Card) -> str:
    if not card.fov:
        return "?"
    h_pin = card.fov.get("hfov_pinhole_deg", 0.0)
    h_eq = card.fov.get("hfov_equidistant_deg", 0.0)
    if h_pin < 70:
        return "standard"
    if h_eq == h_eq and h_eq > 125:
        return "fisheye"
    return "wide"


# --------------------------------------------------------------------------- #
# Small UI helpers                                                            #
# --------------------------------------------------------------------------- #

def ask_float(prompt: str, default: float) -> float:
    while True:
        try:
            raw = input(f"  {prompt} [{default:g}]: ").strip()
        except EOFError:
            return default
        if not raw:
            return default
        try:
            value = float(raw.replace(",", "."))
            if value > 0:
                return value
        except ValueError:
            pass
        print("  please type a positive number")


def ask_text(prompt: str, default: str = "") -> str:
    try:
        raw = input(f"  {prompt}{f' [{default}]' if default else ''}: ").strip()
    except EOFError:
        return default
    return raw or default


def put_lines(img, lines: list[str], origin: tuple[int, int] = (10, 10)) -> None:
    x, y = origin
    pad, step = 6, 22
    width = max(cv2.getTextSize(t, FONT, 0.55, 1)[0][0] for t in lines) + 2 * pad
    height = step * len(lines) + pad
    box = img[y:y + height, x:x + width]
    if box.size:
        img[y:y + height, x:x + width] = (box * 0.35).astype(img.dtype)
    for i, text in enumerate(lines):
        cv2.putText(img, text, (x + pad, y + step * (i + 1) - 4), FONT, 0.55, (255, 255, 255), 1, cv2.LINE_AA)


def pick_points(img, n: int, hint: str) -> list[tuple[float, float]] | None:
    """Show img scaled to fit the screen, collect n left clicks in full-image
    pixel coordinates. Enter accepts, u undoes, Esc cancels."""
    scale = min(1.0, 1400 / img.shape[1], 850 / img.shape[0])
    disp = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else img
    pts: list[tuple[float, float]] = []
    win = "lens_test: click"

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < n:
            pts.append((x / scale, y / scale))

    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, on_mouse)
    try:
        while True:
            frame = disp.copy()
            shown = [(int(px * scale), int(py * scale)) for px, py in pts]
            for a, b in zip(shown, shown[1:]):
                cv2.line(frame, a, b, (0, 200, 255), 1, cv2.LINE_AA)
            for i, p in enumerate(shown):
                cv2.circle(frame, p, 7, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.putText(frame, str(i + 1), (p[0] + 9, p[1] - 9), FONT, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            put_lines(frame, [hint, f"{len(pts)}/{n} points    Enter: accept    u: undo    Esc: cancel"])
            cv2.imshow(win, frame)
            key = cv2.waitKey(30) & 0xFF
            if key == 27:
                return None
            if key in (13, 10, 32) and len(pts) == n:
                return pts
            if key == ord("u") and pts:
                pts.pop()
    finally:
        cv2.destroyWindow(win)


def draw_points(img, pts, colour=(0, 0, 255)):
    out = img.copy()
    shown = [(int(x), int(y)) for x, y in pts]
    for a, b in zip(shown, shown[1:]):
        cv2.line(out, a, b, (0, 200, 255), 2, cv2.LINE_AA)
    for i, p in enumerate(shown):
        cv2.circle(out, p, 12, colour, 3, cv2.LINE_AA)
        cv2.putText(out, str(i + 1), (p[0] + 14, p[1] - 14), FONT, 1.2, colour, 3, cv2.LINE_AA)
    return out


# --------------------------------------------------------------------------- #
# Live session                                                                #
# --------------------------------------------------------------------------- #

class Session:
    def __init__(self, board, out: Path, label: str, stream_size: str) -> None:
        self.board = board
        self.out = out
        self.stream_size = stream_size
        self.info = board.info()
        if not self.info.get("camera_ok"):
            sys.exit(f"the board reports no camera: {self.info.get('error')} - {self.info.get('hint', '')}")
        self.card = load_card(out, label, self.info)
        status = board.status()
        self.lenc = bool(status.get("lenc", 1))
        self.ae_locked = not status.get("aec", 1)
        # The OV2640 driver reports the colour-bar flag as set after init even
        # though the bars are off; force a known state instead of trusting it.
        self.colorbar = False
        board.set("colorbar", 0)
        self.last_distance = 1.0
        self.last_width = 1.0
        self.reader = None
        self.start_stream()
        print(f"sensor {self.info.get('sensor')} PID {self.info.get('pid')}, "
              f"max {self.info.get('max_width')}x{self.info.get('max_height')}; card: {self.card.label}")

    # -- stream ------------------------------------------------------------ #

    def start_stream(self) -> None:
        if self.reader is not None:
            self.reader.stop()
        self.reader = self.board.frames(self.stream_size)
        self.reader.start()

    def frames(self, n: int = 10, timeout: float = 8.0) -> list:
        """n consecutive decoded stream frames (after discarding one, so a
        setting changed just before is already in effect)."""
        out: list = []
        last = self.reader.latest()[1]
        skipped = False
        t0 = time.time()
        while len(out) < n and time.time() - t0 < timeout:
            jpg, count = self.reader.latest()
            if count == last or jpg is None:
                time.sleep(0.01)
                continue
            last = count
            if not skipped:
                skipped = True
                continue
            img = decode(jpg)
            if img is not None:
                out.append(img)
        return out

    def capture(self):
        jpg = self.board.capture("MAX")
        img = decode(jpg)
        if img is None:
            raise RuntimeError("the still did not decode as a JPEG")
        return jpg, img

    def save(self) -> None:
        save_card(self.out, self.card)

    def folder(self) -> Path:
        folder = card_dir(self.out, self.card.label)
        folder.mkdir(parents=True, exist_ok=True)
        return folder

    # -- actions ----------------------------------------------------------- #

    def act_new(self) -> None:
        self.save()
        label = clean_label(ask_text("label for the next module (write it on the module too)"))
        self.card = load_card(self.out, label, self.info)
        self.save()
        print(f"  card: {card_dir(self.out, label) / 'card.json'}")

    def act_note(self) -> None:
        note = ask_text("note (marking on the flex, housing, where it came from)")
        if note:
            self.card.notes.append({"when": now(), "text": note})
            self.save()

    def act_still(self) -> None:
        jpg, img = self.capture()
        name = f"still_{len(self.card.stills) + 1}.jpg"
        (self.folder() / name).write_bytes(jpg)
        self.card.stills.append({"file": name, "size": [img.shape[1], img.shape[0]], "when": now()})
        self.save()
        print(f"  saved {name} ({img.shape[1]}x{img.shape[0]})")

    def act_fov(self) -> None:
        print("\nField of view. Board square to a wall with a tape measure stuck across it,"
              " at a measured distance from the lens.")
        distance = ask_float("distance from the lens to the wall [m]", self.last_distance)
        print("  taking a full-size still ...")
        jpg, img = self.capture()
        pts = pick_points(img, 2, "Click the tape near the LEFT edge, then near the RIGHT edge (same height)")
        if not pts:
            print("  cancelled")
            return
        width = ask_float("tape reading difference between the two clicks [m]", self.last_width)
        self.last_distance, self.last_width = distance, width
        try:
            res = solve_fov(pts[0][0], pts[1][0], img.shape[1], img.shape[0], distance, width)
        except ValueError as exc:
            print(f"  {exc}")
            return
        res.update({"distance_m": distance, "width_m": width,
                    "points": [[round(x, 1), round(y, 1)] for x, y in pts],
                    "image": "fov.jpg", "image_size": [img.shape[1], img.shape[0]], "when": now()})
        self.card.fov = res
        cv2.imwrite(str(self.folder() / "fov.jpg"), draw_points(img, pts))
        self.save()
        print(f"  horizontal {res['hfov_pinhole_deg']} deg pinhole / {res['hfov_equidistant_deg']} deg equidistant,"
              f" vertical {res['vfov_pinhole_deg']} / {res['vfov_equidistant_deg']},"
              f" f = {res['f_px_pinhole']} px; clicks cover {res['coverage'] * 100:.0f} % of the width"
              f" -> {lens_class(self.card)}")
        if res["coverage"] < 0.6:
            print("  tip: click closer to the picture edges, or move back, so the models agree better")

    def act_distortion(self) -> None:
        print("\nDistortion. A straight edge (door frame, table edge, the tape) running close"
              " to one border of the picture.")
        jpg, img = self.capture()
        pts = pick_points(img, 3, "Click one END of the straight edge, its MIDDLE, then the other END")
        if not pts:
            print("  cancelled")
            return
        try:
            res = solve_distortion(pts, img.shape[1], img.shape[0])
        except ValueError as exc:
            print(f"  {exc}")
            return
        res.update({"points": [[round(x, 1), round(y, 1)] for x, y in pts],
                    "image": "distortion.jpg", "when": now()})
        self.card.distortion = res
        cv2.imwrite(str(self.folder() / "distortion.jpg"), draw_points(img, pts))
        self.save()
        print(f"  bow {res['sag_pct']} % of the chord ({res['kind']}), measured at"
              f" {res['radius_frac'] * 100:.0f} % of the half-diagonal")

    def act_vignetting(self) -> None:
        print("\nVignetting. Fill the whole picture with a plain, evenly lit white wall.")
        frames = self.frames()
        if not frames:
            print("  no frames")
            return
        vals = [vignetting(gray_of(f)) for f in frames]
        ratio = float(np.mean([v[0] for v in vals]))
        centre = float(np.mean([v[1] for v in vals]))
        key = "lenc_on" if self.lenc else "lenc_off"
        self.card.vignetting[key] = {"corner_over_centre": round(ratio, 3), "centre_mean": round(centre, 1),
                                     "ae_locked": self.ae_locked, "stream_size": self.size_str(frames[0]),
                                     "when": now()}
        self.save()
        print(f"  corner/centre = {ratio:.3f} with LENC {'on' if self.lenc else 'off'}")
        if centre > 235:
            print("  the centre is saturated: lock exposure (e) or dim the light and repeat")
        elif centre < 40:
            print("  too dark to trust: more light, or unlock exposure (e), and repeat")
        if len(self.card.vignetting) == 1:
            print("  press l to toggle LENC and v again to record the other state")

    def act_sharpness(self) -> None:
        print("\nSharpness. A printed text sheet filling the centre of the picture.")
        distance = ask_float("distance from the lens to the sheet [m]", self.last_distance)
        frames = self.frames()
        if not frames:
            print("  no frames")
            return
        value = float(np.mean([sharpness(gray_of(f)) for f in frames]))
        entry = {"distance_m": distance, "value": round(value, 1), "stream_size": self.size_str(frames[0]),
                 "when": now()}
        self.card.sharpness = [s for s in self.card.sharpness if s.get("distance_m") != distance] + [entry]
        self.card.sharpness.sort(key=lambda s: s["distance_m"])
        self.save()
        print(f"  sharpness {value:.1f} at {distance:g} m (compare only with the same sheet and distance)")

    def act_ir(self) -> None:
        if not self.ae_locked:
            print("  tip: lock exposure first (e) so modules are compared at the same gain")
        print("\nIR response. In a dim room, press a TV remote's button with the LED 20 cm"
              " from the lens, and keep it pressed ...")
        time.sleep(1.5)
        frames = self.frames()
        if not frames:
            print("  no frames")
            return
        peaks = [peak(gray_of(f)) for f in frames]
        value = int(max(peaks))
        mean = float(np.mean([gray_of(f).mean() for f in frames]))
        self.card.ir = {"peak": value, "frame_mean": round(mean, 1), "ae_locked": self.ae_locked,
                        "stream_size": self.size_str(frames[0]), "when": now()}
        self.save()
        verdict = ("saturated: no IR-cut filter, or the remote is too close" if value >= 250 else
                   "visible but not saturated" if value >= 120 else "faint: an IR-cut filter is in place")
        print(f"  peak {value} (frame mean {mean:.0f}) -> {verdict}")

    def act_lenc(self) -> None:
        self.lenc = not self.lenc
        print(f"  lenc -> {int(self.lenc)}: {self.board.set('lenc', int(self.lenc))}")

    def act_ae(self) -> None:
        self.ae_locked = not self.ae_locked
        on = 0 if self.ae_locked else 1
        self.board.set("aec", on)
        self.board.set("agc", on)
        print(f"  auto-exposure and gain {'locked at the current values' if self.ae_locked else 'back to auto'}")

    def act_colorbar(self) -> None:
        self.colorbar = not self.colorbar
        print(f"  colorbar -> {int(self.colorbar)}: {self.board.set('colorbar', int(self.colorbar))}")

    @staticmethod
    def size_str(img) -> str:
        return f"{img.shape[1]}x{img.shape[0]}"

    # -- main loop --------------------------------------------------------- #

    def overlay(self, img, fps: float):
        gray = gray_of(img)
        h, w = gray.shape
        sharp = sharpness(gray)
        ratio, centre, _ = vignetting(gray)
        pk = peak(gray)
        view = img.copy()
        rh, rw = int(h * 0.125), int(w * 0.125)
        cv2.rectangle(view, (w // 2 - rw, h // 2 - rh), (w // 2 + rw, h // 2 + rh), (0, 200, 255), 1)
        ph, pw = int(h * 0.10), int(w * 0.10)
        for x0, y0 in ((0, 0), (w - pw, 0), (0, h - ph), (w - pw, h - ph)):
            cv2.rectangle(view, (x0, y0), (x0 + pw, y0 + ph), (0, 200, 255), 1)
        cv2.line(view, (w // 2 - 15, h // 2), (w // 2 + 15, h // 2), (0, 0, 255), 1)
        cv2.line(view, (w // 2, h // 2 - 15), (w // 2, h // 2 + 15), (0, 0, 255), 1)
        lines = [
            f"[{self.card.label}]  {self.card.sensor.get('sensor', '?')} {self.card.sensor.get('pid', '')}"
            f"   {w}x{h}  {fps:4.1f} fps",
            f"sharp {sharp:7.1f}   corner/centre {ratio:5.3f} (centre {centre:5.1f})   peak {pk:3d}",
            f"LENC {'on ' if self.lenc else 'off'}   AE {'LOCKED' if self.ae_locked else 'auto  '}"
            f"   bars {'on' if self.colorbar else 'off'}",
        ]
        if self.card.fov:
            f = self.card.fov
            lines.append(f"FOV {f['hfov_pinhole_deg']:.0f}x{f['vfov_pinhole_deg']:.0f} deg pinhole,"
                         f" {f['hfov_equidistant_deg']:.0f} eq -> {lens_class(self.card)}")
        tail = []
        if self.card.distortion:
            tail.append(f"bow {self.card.distortion['sag_pct']:+.1f} %")
        if self.card.ir:
            tail.append(f"IR peak {self.card.ir['peak']}")
        if self.card.sharpness:
            tail.append("sharp " + ", ".join(f"{s['distance_m']:g}m:{s['value']:.0f}" for s in self.card.sharpness))
        if tail:
            lines.append("   ".join(tail))
        lines.append("h: help   q: quit")
        put_lines(view, lines)
        return view

    def run(self) -> None:
        print(HELP)
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        last_count = -1
        shown = None
        fps, fps_n, fps_t = 0.0, 0, time.time()
        actions = {
            ord("n"): self.act_new, ord("t"): self.act_note, ord("s"): self.act_still,
            ord("f"): self.act_fov, ord("d"): self.act_distortion, ord("v"): self.act_vignetting,
            ord("k"): self.act_sharpness, ord("i"): self.act_ir, ord("l"): self.act_lenc,
            ord("e"): self.act_ae, ord("c"): self.act_colorbar, ord("h"): lambda: print(HELP),
        }
        try:
            while True:
                if not self.reader.is_alive():
                    if self.reader.error and shown is None:
                        sys.exit(f"stream failed: {self.reader.error}")
                    time.sleep(1.0)
                    self.start_stream()
                jpg, count = self.reader.latest()
                if jpg is not None and count != last_count:
                    img = decode(jpg)
                    if img is not None:
                        last_count = count
                        fps_n += 1
                        shown = self.overlay(img, fps)
                t = time.time()
                if t - fps_t >= 1.0:
                    fps, fps_n, fps_t = fps_n / (t - fps_t), 0, t
                if shown is None:
                    blank = np.zeros((240, 640, 3), np.uint8)
                    put_lines(blank, [f"waiting for the stream from {self.board.host} ..."])
                    cv2.imshow(WINDOW, blank)
                else:
                    cv2.imshow(WINDOW, shown)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                action = actions.get(key)
                if action is None:
                    continue
                try:
                    action()
                except (RuntimeError, urllib.error.URLError, OSError) as exc:
                    print(f"  failed: {exc}")
        finally:
            self.save()
            self.reader.stop()
            cv2.destroyAllWindows()
            print(f"card saved: {card_dir(self.out, self.card.label) / 'card.json'}")


# --------------------------------------------------------------------------- #
# Commands                                                                    #
# --------------------------------------------------------------------------- #

def make_board(args):
    return SerialBoard(args.port, args.baud) if args.port else Board(args.host)


def cmd_info(args) -> int:
    info = make_board(args).info()
    print(json.dumps(info, indent=2))
    if info.get("camera_ok"):
        print(f"\n{info['sensor']} (PID {info['pid']}), max {info['max_width']}x{info['max_height']}, "
              f"{'hardware JPEG' if info.get('support_jpeg') else 'software JPEG'}")
    else:
        print(f"\nno camera: {info.get('error')} - {info.get('hint', '')}")
    return 0


def cmd_live(args) -> int:
    size = args.stream_size or ("QVGA" if args.port else "VGA")   # serial carries ~90 kB/s
    Session(make_board(args), Path(args.out), clean_label(args.label), size).run()
    return 0


def cmd_snap(args) -> int:
    board = make_board(args)
    out = Path(args.out)
    card = load_card(out, clean_label(args.label), board.info())
    jpg = board.capture(args.size)
    img = decode(jpg)
    folder = card_dir(out, card.label)
    folder.mkdir(parents=True, exist_ok=True)
    name = f"still_{len(card.stills) + 1}.jpg"
    (folder / name).write_bytes(jpg)
    size = [img.shape[1], img.shape[0]] if img is not None else None
    card.stills.append({"file": name, "size": size, "when": now()})
    save_card(out, card)
    print(f"saved {folder / name} {size}")
    return 0


def cmd_reboot(args) -> int:
    if not args.port:
        print("reboot works over --port only")
        return 2
    print(make_board(args)._cmd("reboot", "OK"))   # noqa: SLF001 - one-off command, same channel
    return 0


def cmd_table(args) -> int:
    out = Path(args.out)
    cards = [load_card(out, p.parent.name) for p in sorted(out.glob("*/card.json"))]
    if not cards:
        print(f"no lens cards under {out}")
        return 1
    header = ["label", "sensor", "HFOV pin", "HFOV eq", "VFOV pin", "class", "bow %",
              "corner/centre", "IR peak", "sharpness", "notes"]
    rows = []
    for c in cards:
        fov = c.fov or {}
        vig = c.vignetting.get("lenc_on") or c.vignetting.get("lenc_off") or {}
        rows.append([
            c.label, f"{c.sensor.get('sensor', '?')} {c.sensor.get('pid', '')}".strip(),
            fmt(fov.get("hfov_pinhole_deg")), fmt(fov.get("hfov_equidistant_deg")),
            fmt(fov.get("vfov_pinhole_deg")), lens_class(c),
            fmt(c.distortion.get("sag_pct")) if c.distortion else "-",
            fmt(vig.get("corner_over_centre"), 2) if vig else "-",
            str(c.ir["peak"]) if c.ir else "-",
            " ".join(f"{s['distance_m']:g}m:{s['value']:.0f}" for s in c.sharpness) or "-",
            "; ".join(n["text"] for n in c.notes) or "-",
        ])
    if args.markdown:
        print("| " + " | ".join(header) + " |")
        print("|" + "---|" * len(header))
        for r in rows:
            print("| " + " | ".join(r) + " |")
    else:
        widths = [max(len(x) for x in col) for col in zip(header, *rows)]
        for r in (header, *rows):
            print("  ".join(x.ljust(w) for x, w in zip(r, widths)))
    print(f"\n{len(cards)} card(s) under {out}")
    return 0


def fmt(value, digits: int = 1) -> str:
    if value is None or (isinstance(value, float) and value != value):
        return "-"
    return f"{value:.{digits}f}"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port of the board (COM3): use the USB cable instead of Wi-Fi")
    ap.add_argument("--baud", type=int, default=921600,
                    help="serial rate; must match the firmware's 'Lens test' menu (default 921600)")
    ap.add_argument("--host", default=DEFAULT_HOST, help="board address over Wi-Fi (default: its own access point)")
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="where lens cards go (default build/lens_test)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("info", help="sensor identity").set_defaults(fn=cmd_info)

    live = sub.add_parser("live", help="live view with measurements and hotkeys")
    live.add_argument("--label", default="module", help="lens-card label to start with")
    live.add_argument("--stream-size", default=None, help="live frame size (default VGA over Wi-Fi, QVGA over serial)")
    live.set_defaults(fn=cmd_live)

    snap = sub.add_parser("snap", help="save one still to a lens card")
    snap.add_argument("--label", default="module")
    snap.add_argument("--size", default="MAX")
    snap.set_defaults(fn=cmd_snap)

    sub.add_parser("reboot", help="restart the board (re-probes the camera); --port only").set_defaults(fn=cmd_reboot)

    table = sub.add_parser("table", help="summarise every lens card")
    table.add_argument("--markdown", action="store_true", help="table ready for docs/10")
    table.set_defaults(fn=cmd_table)

    args = ap.parse_args(argv)
    try:
        return args.fn(args)
    except urllib.error.URLError as exc:
        print(f"cannot reach the board at {args.host}: {exc.reason}\n"
              f"  Is the laptop on the board's Wi-Fi ('lens-test'), or did you pass --host?"
              f" With the USB cable, pass --port COMx instead.")
        return 2
    except (OSError, TimeoutError) as exc:
        print(f"serial port {args.port}: {exc}\n"
              f"  Is the board plugged in, the port free (no monitor open), and the firmware the lens-test one?")
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
