"""
The camera's own detector, called from Python.

host/libblob_detect.so is ../main/blob_detect.c compiled for this PC: the same
source file the firmware runs. So the viewer's mask overlay, its per-pixel
H/S/V readout and its region statistics are the camera's arithmetic, not a
Python re-implementation that could drift from it (the hue of a pixel, for
example, comes from integer 5/6/5-bit maths that a float HSV would not
reproduce exactly).

The library is rebuilt with `make lib` whenever the C source is newer.
"""
from __future__ import annotations

import ctypes as C
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HOST_DIR = Path(__file__).resolve().parents[2]          # target_cam/host
MAIN_DIR = HOST_DIR.parent / "main"
LIB_PATH = HOST_DIR / "libblob_detect.so"
SOURCES = [MAIN_DIR / "blob_detect.c", MAIN_DIR / "blob_detect.h"]

BLOB_MAX_REPORT = 4
PRESETS = ["red", "blue", "green", "orange"]
CAL_PATCH = 16          # target_main.c: the centre patch 'c' samples


class BlobCfg(C.Structure):
    _fields_ = [(n, C.c_int) for n in
                ("width", "height", "cell", "big_endian", "hue", "hue_tol",
                 "sat_min", "val_min", "min_area")]


class Blob(C.Structure):
    _fields_ = [("cx", C.c_float), ("cy", C.c_float), ("area", C.c_int),
                ("x0", C.c_int), ("y0", C.c_int), ("x1", C.c_int), ("y1", C.c_int),
                ("clipped", C.c_int)]


class BlobResult(C.Structure):
    _fields_ = [("found", C.c_int), ("n_report", C.c_int), ("n_blobs", C.c_int),
                ("matched", C.c_int), ("b", Blob * BLOB_MAX_REPORT)]


class BlobColor(C.Structure):
    _fields_ = [("hue", C.c_int), ("hue_spread", C.c_int), ("sat", C.c_int),
                ("val", C.c_int)]


def _stale() -> bool:
    if not LIB_PATH.exists():
        return True
    built = LIB_PATH.stat().st_mtime
    return any(s.stat().st_mtime > built for s in SOURCES)


def load_library() -> C.CDLL:
    if _stale():
        r = subprocess.run(["make", "-C", str(HOST_DIR), "lib"],
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise RuntimeError("building libblob_detect.so failed:\n" + r.stdout + r.stderr)
    lib = C.CDLL(str(LIB_PATH))
    P = C.POINTER
    lib.blob_cfg_valid.argtypes = [P(BlobCfg)]
    lib.blob_workspace_bytes.argtypes = [P(BlobCfg)]
    lib.blob_workspace_bytes.restype = C.c_size_t
    lib.blob_rgb565_hsv.argtypes = [C.c_uint16, P(C.c_int), P(C.c_int), P(C.c_int)]
    lib.blob_rgb565_hsv.restype = None
    lib.blob_pixel_matches.argtypes = [P(BlobCfg), C.c_uint16]
    lib.blob_mask.argtypes = [P(BlobCfg), C.c_char_p, C.c_void_p]
    lib.blob_detect.argtypes = [P(BlobCfg), C.c_char_p, C.c_void_p, P(BlobResult)]
    lib.blob_sample.argtypes = [P(BlobCfg), C.c_char_p, C.c_int, C.c_int, C.c_int,
                                C.c_int, P(BlobColor)]
    lib.blob_sample.restype = None
    lib.blob_preset_apply.argtypes = [P(BlobCfg), C.c_int]
    return lib


_LIB = None


def lib() -> C.CDLL:
    global _LIB
    if _LIB is None:
        _LIB = load_library()
    return _LIB


@dataclass
class Settings:
    """The detector half of the camera's #!CFG."""
    hue: int = 0
    tol: int = 12
    sat: int = 110
    val: int = 50
    area: int = 25
    be: int = 1
    w: int = 160
    h: int = 120
    cell: int = 2

    def c(self) -> BlobCfg:
        return BlobCfg(self.w, self.h, self.cell, self.be, self.hue, self.tol,
                       self.sat, self.val, self.area)

    @classmethod
    def from_cfg(cls, v: dict) -> "Settings":
        s = cls()
        for k in ("hue", "tol", "sat", "val", "area", "be", "w", "h", "cell"):
            if isinstance(v.get(k), int):
                setattr(s, k, v[k])
        return s


def preset(name_or_index) -> tuple:
    """(hue, tol, sat, val) of a firmware preset, read from the C table."""
    i = PRESETS.index(name_or_index) if isinstance(name_or_index, str) else name_or_index
    c = Settings().c()
    if not lib().blob_preset_apply(C.byref(c), i):
        raise ValueError(name_or_index)
    return c.hue, c.hue_tol, c.sat_min, c.val_min


_LUT = None


def hsv_lut() -> np.ndarray:
    """(65536, 3) int16: H, S, V of every RGB565 value, from blob_rgb565_hsv().
    Built once (~0.1 s); after that any pixel's colour is an array lookup."""
    global _LUT
    if _LUT is None:
        f = lib().blob_rgb565_hsv
        h, s, v = C.c_int(), C.c_int(), C.c_int()
        out = np.empty((65536, 3), dtype=np.int16)
        for px in range(65536):
            f(px, C.byref(h), C.byref(s), C.byref(v))
            out[px] = (h.value, s.value, v.value)
        _LUT = out
    return _LUT


def hsv(px: int) -> tuple:
    h, s, v = C.c_int(), C.c_int(), C.c_int()
    lib().blob_rgb565_hsv(px, C.byref(h), C.byref(s), C.byref(v))
    return h.value, s.value, v.value


def mask(st: Settings, frame: bytes) -> np.ndarray:
    """(h, w) uint8, 1 where the pixel passes the camera's colour test."""
    out = np.zeros((st.h, st.w), dtype=np.uint8)
    c = st.c()
    lib().blob_mask(C.byref(c), frame, out.ctypes.data_as(C.c_void_p))
    return out


def detect(st: Settings, frame: bytes) -> BlobResult:
    c = st.c()
    if not lib().blob_cfg_valid(C.byref(c)):
        raise ValueError("invalid detector configuration")
    work = (C.c_uint32 * ((lib().blob_workspace_bytes(C.byref(c)) + 3) // 4))()
    r = BlobResult()
    lib().blob_detect(C.byref(c), frame, C.cast(work, C.c_void_p), C.byref(r))
    return r


def sample(st: Settings, frame: bytes, x0: int, y0: int, w: int, h: int) -> BlobColor:
    """blob_sample() on a rectangle, clipped to the frame (the C does not check)."""
    x1, y1 = min(st.w, x0 + w), min(st.h, y0 + h)
    x0, y0 = max(0, x0), max(0, y0)
    out = BlobColor()
    if x1 > x0 and y1 > y0:
        c = st.c()
        lib().blob_sample(C.byref(c), frame, x0, y0, x1 - x0, y1 - y0, C.byref(out))
    return out


@dataclass
class Calibration:
    hue: int
    tol: int
    sat: int
    val: int
    refused: bool       # the firmware would refuse: too grey to separate from walls
    warning: str


def calibrate_from(col: BlobColor) -> Calibration:
    """Thresholds from a sampled colour - the same rule as calibrate() in
    target_main.c, so 'Use as target' on the PC and 'c' on the camera agree."""
    tol = min(40, max(10, 3 * col.hue_spread))
    sat = max(40, col.sat // 2)
    val = max(25, col.val * 2 // 5)
    warn = ""
    refused = col.sat < 70
    if refused:
        warn = ("saturation %d is too low to separate from walls; the camera's 'c' "
                "would refuse this" % col.sat)
    elif col.hue_spread > 20:
        warn = "hue spread %d deg: the region is not one colour" % col.hue_spread
    return Calibration(col.hue, tol, sat, val, refused, warn)
