"""
Image & calibration screen.

Shows the camera's real pixels (fetched over the serial link as raw RGB565),
lets you read the colour of any pixel or region, and tune the detector by hand.

Everything drawn over the image is computed on the PC by the camera's own C
detector (libblob_detect.so) with the values the controls show - so moving a
slider updates the mask at once, before anything is sent. The camera's own
answer for the same frame (its CSV row) is drawn as white crosses. When the
controls are all "applied", the PC boxes and the camera crosses must agree; if
they ever do not, something other than the thresholds differs, and that is
worth knowing.

While frames are being sent the camera's loop is slowed by the transfer, so
dt_ms and FPS on this screen are NOT what flight will see. The Flight data
screen is the one to trust for timing.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QRectF, QSettings, Qt
from PyQt6.QtGui import QColor, QImage, QPen
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QGridLayout,
                             QGroupBox, QHBoxLayout, QLabel, QMessageBox,
                             QPushButton, QScrollArea, QSizePolicy, QVBoxLayout,
                             QWidget, QGraphicsRectItem)

from . import detector as D
from . import protocol as P
from .widgets import AMBER, GREEN, GREY, RED, HueBar, ParamControl, chip

DATA_DIR = Path(__file__).resolve().parents[3] / "data"      # target_cam/data
W, H = 160, 120
SCALE = 4

# Control key -> (command format, #!CFG key)
PARAMS = {
    "hue": ("H%d", "hue"),
    "tol": ("T%d", "tol"),
    "sat": ("S%d", "sat"),
    "val": ("V%d", "val"),
    "area": ("a%d", "area"),
    "mm": ("d%d", "mm"),
    "fov": ("F%.1f", "fov"),
}

# The detector levels '!stage N' selects (tc_pipeline.h). Each falls back to
# the highest one whose models the camera has.
STAGE_NAMES = ["0  HSV window", "1  colour table", "2  blob classifier", "3  CNN"]


class ImagePane(pg.GraphicsLayoutWidget):
    def __init__(self, on_hover, on_region):
        super().__init__()
        self.setMinimumSize(W * SCALE + 4, H * SCALE + 4)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.ci.setContentsMargins(0, 0, 0, 0)
        self.vb = self.addViewBox(lockAspect=True, invertY=True, enableMouse=False)
        self.vb.setMenuEnabled(False)
        self.vb.setRange(QRectF(0, 0, W, H), padding=0)
        self.img = pg.ImageItem(axisOrder="row-major")
        self.overlay = pg.ImageItem(axisOrder="row-major")
        self.vb.addItem(self.img)
        self.vb.addItem(self.overlay)
        blank = np.zeros((H, W, 3), dtype=np.uint8)
        blank[:] = 40
        self.img.setImage(blank, levels=(0, 255))

        # Where the camera's 'c' samples: the 16x16 centre patch.
        c = QGraphicsRectItem((W - D.CAL_PATCH) / 2, (H - D.CAL_PATCH) / 2,
                              D.CAL_PATCH, D.CAL_PATCH)
        c.setPen(pg.mkPen((255, 255, 255, 120), width=1, style=Qt.PenStyle.DashLine,
                          cosmetic=True))
        self.vb.addItem(c)

        # PC rerun of the detector: a box per reported blob.
        self.boxes = []
        self.labels = []
        for k in range(D.BLOB_MAX_REPORT):
            r = QGraphicsRectItem(0, 0, 1, 1)
            r.setPen(pg.mkPen((255, 230, 0) if k == 0 else (255, 150, 0), width=2,
                              cosmetic=True))
            r.setVisible(False)
            self.vb.addItem(r)
            t = pg.TextItem(str(k + 1), color=(255, 230, 0), anchor=(0, 1))
            t.setVisible(False)
            self.vb.addItem(t)
            self.boxes.append(r)
            self.labels.append(t)
        # The camera's own answer for this frame, from its CSV row.
        self.cam_marks = pg.ScatterPlotItem(symbol="+", size=16, pen=pg.mkPen("w", width=2),
                                            brush=None)
        self.vb.addItem(self.cam_marks)

        self.roi = pg.RectROI([(W - D.CAL_PATCH) / 2, (H - D.CAL_PATCH) / 2],
                              [D.CAL_PATCH, D.CAL_PATCH], pen=pg.mkPen((0, 220, 255), width=2),
                              snapSize=1.0, translateSnap=True, scaleSnap=True,
                              maxBounds=QRectF(0, 0, W, H))
        self.roi.addScaleHandle([0, 0], [1, 1])
        self.vb.addItem(self.roi)
        self.roi.sigRegionChanged.connect(lambda *_: on_region())
        self.scene().sigMouseMoved.connect(self._moved)
        self._on_hover = on_hover

    def _moved(self, pos):
        p = self.vb.mapSceneToView(pos)
        x, y = int(math.floor(p.x())), int(math.floor(p.y()))
        self._on_hover(x, y if 0 <= x < W and 0 <= y < H else None)

    def region(self):
        x, y = self.roi.pos()
        w, h = self.roi.size()
        return int(round(x)), int(round(y)), max(1, int(round(w))), max(1, int(round(h)))


class CalibTab(QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main
        self.frame: P.Frame | None = None
        self.frame_row: dict | None = None
        self.frame_time = 0.0
        self.cfg: dict = {}
        self.pending_cmds: dict = {}        # command text -> ParamControl
        self._hover = None

        root = QHBoxLayout(self)
        left = QVBoxLayout()
        root.addLayout(left)
        self.pane = ImagePane(self._hover_at, self._update_region)
        left.addWidget(self.pane)

        self.hover_lbl = QLabel("hover over the image for a pixel's colour")
        self.hover_lbl.setStyleSheet("font-family: monospace;")
        left.addWidget(self.hover_lbl)
        self.frame_lbl = QLabel("no frame yet: press Grab frame")
        self.frame_lbl.setWordWrap(True)
        left.addWidget(self.frame_lbl)
        self.compare_lbl = QLabel("")
        self.compare_lbl.setWordWrap(True)
        self.compare_lbl.setStyleSheet("font-family: monospace;")
        left.addWidget(self.compare_lbl)

        disp = QHBoxLayout()
        disp.addWidget(QLabel("Overlay:"))
        self.overlay_mode = QComboBox()
        self.overlay_mode.addItems(["matching pixels tinted", "mask only (white = match)", "off"])
        self.overlay_mode.currentIndexChanged.connect(lambda *_: self._redraw())
        disp.addWidget(self.overlay_mode)
        self.show_boxes = QCheckBox("PC detection boxes")
        self.show_boxes.setChecked(True)
        self.show_boxes.toggled.connect(lambda *_: self._redraw())
        disp.addWidget(self.show_boxes)
        disp.addStretch(1)
        left.addLayout(disp)

        # ---- second column: the controls ----
        # The image column also holds what a capture session uses (grab, baud,
        # save) and the camera's settings state, so the bottom of the controls
        # column is never in the way of a capture. Every control panel can be
        # hidden with the strip of buttons at the right edge (remembered between
        # runs), and the column scrolls if what is shown is still too tall.
        mid = self._column(root)
        panels = []                         # (strip label, group box)

        region = QGroupBox("Region (drag / resize the cyan box)")
        rl = QVBoxLayout(region)
        rb = QHBoxLayout()
        self.use_btn = QPushButton("Use region as target")
        self.use_btn.setToolTip("Sets hue/tolerance/sat/val from the region with the same "
                                "rule as the camera's 'c', and sends them.")
        self.use_btn.clicked.connect(self._use_region)
        rb.addWidget(self.use_btn)
        self.use_warn = QLabel("")
        self.use_warn.setWordWrap(True)
        rb.addWidget(self.use_warn, 1)
        rl.addLayout(rb)
        self.region_lbl = QLabel("-")
        self.region_lbl.setStyleSheet("font-family: monospace;")
        self.region_lbl.setWordWrap(True)
        rl.addWidget(self.region_lbl)
        self.hist = pg.PlotWidget()
        self.hist.setFixedHeight(110)
        self.hist.setMouseEnabled(False, False)
        self.hist.setMenuEnabled(False)
        self.hist.hideButtons()
        self.hist.setXRange(0, 360, padding=0)
        self.hist.getAxis("left").setWidth(30)
        self.hist.setLabel("bottom", "hue of region pixels with sat >= 30")
        self.bars = pg.BarGraphItem(x=np.arange(5, 360, 10), height=np.zeros(36), width=9,
                                    brushes=[pg.mkBrush(QColor.fromHsv(h, 255, 230))
                                             for h in range(5, 360, 10)])
        self.hist.addItem(self.bars)
        self.win_lo = pg.InfiniteLine(angle=90, pen=pg.mkPen("w", style=Qt.PenStyle.DashLine))
        self.win_hi = pg.InfiniteLine(angle=90, pen=pg.mkPen("w", style=Qt.PenStyle.DashLine))
        self.hist.addItem(self.win_lo)
        self.hist.addItem(self.win_hi)
        rl.addWidget(self.hist)
        mid.addWidget(region)
        panels.append(("Region", region))

        self.settings_lbl = QLabel()
        self.settings_lbl.setWordWrap(True)

        g = QGroupBox("Image link")
        gl = QGridLayout(g)
        self.grab_btn = QPushButton("Grab frame")
        self.grab_btn.clicked.connect(lambda: self.main.send("g"))
        gl.addWidget(self.grab_btn, 0, 0)
        self.live_chk = QCheckBox("Live (a frame every loop)")
        self.live_chk.clicked.connect(lambda on: self.main.send("v1" if on else "v0"))
        gl.addWidget(self.live_chk, 0, 1)
        gl.addWidget(QLabel("Baud:"), 1, 0)
        bl = QHBoxLayout()
        self.baud_combo = QComboBox()
        for b in P.BAUDS:
            self.baud_combo.addItem("%d  (frame %.2f s)" % (b, P.frame_seconds(W * H * 2, b)), b)
        bl.addWidget(self.baud_combo)
        sb = QPushButton("Set")
        sb.clicked.connect(lambda: self.main.send("B%d" % self.baud_combo.currentData()))
        bl.addWidget(sb)
        gl.addLayout(bl, 1, 1)
        self.link_lbl = QLabel()
        self.link_lbl.setWordWrap(True)
        gl.addWidget(self.link_lbl, 2, 0, 1, 2)
        left.addWidget(g)

        g = QGroupBox("Colour test (what counts as the target)")
        gl = QVBoxLayout(g)
        # The stage decides which colour test runs. From stage 1 up it is the
        # colour table, and the HSV controls below no longer change which
        # pixels match: they are kept, and used again at stage 0.
        sl = QHBoxLayout()
        sl.addWidget(QLabel("Detector stage:"))
        self.stage_combo = QComboBox()
        self.stage_combo.addItems(STAGE_NAMES)
        sl.addWidget(self.stage_combo)
        b = QPushButton("Set")
        b.setToolTip("!stage N. Not saved until 'Save to camera'.")
        b.clicked.connect(lambda: self.main.send("!stage %d" % self.stage_combo.currentIndex()))
        sl.addWidget(b)
        sl.addStretch(1)
        gl.addLayout(sl)
        self.stage_lbl = QLabel()
        self.stage_lbl.setWordWrap(True)
        gl.addWidget(self.stage_lbl)
        pl = QHBoxLayout()
        pl.addWidget(QLabel("Preset:"))
        self.preset_combo = QComboBox()
        self.preset_combo.addItems(D.PRESETS)
        pl.addWidget(self.preset_combo)
        pb = QPushButton("Apply preset")
        pb.clicked.connect(lambda: self.main.send("p " + self.preset_combo.currentText()))
        pl.addWidget(pb)
        self.preset_lbl = QLabel()
        pl.addWidget(self.preset_lbl, 1)
        gl.addLayout(pl)
        self.huebar = HueBar()
        gl.addWidget(self.huebar)
        self.p = {}
        self.p["hue"] = ParamControl("Hue", 0, 359, unit=" deg")
        self.p["tol"] = ParamControl("Tolerance", 1, 90, unit=" deg")
        self.p["sat"] = ParamControl("Min sat", 0, 255)
        self.p["val"] = ParamControl("Min value", 0, 255)
        for k in ("hue", "tol", "sat", "val"):
            gl.addWidget(self.p[k])
        cb = QHBoxLayout()
        cal = QPushButton("Calibrate on centre (camera 'c')")
        cal.setToolTip("The camera samples the dashed 16x16 centre patch of its NEXT frame.")
        cal.clicked.connect(lambda: self.main.send("c"))
        cb.addWidget(cal)
        self.cal_lbl = QLabel()
        self.cal_lbl.setWordWrap(True)
        cb.addWidget(self.cal_lbl, 1)
        gl.addLayout(cb)
        mid.addWidget(g)
        panels.append(("Colour", g))

        g = QGroupBox("Blobs and geometry")
        gl = QVBoxLayout(g)
        self.p["area"] = ParamControl("Min area", 1, 2000, unit=" px")
        self.p["mm"] = ParamControl("Target size", 0, 5000, slider=False, unit=" mm")
        self.p["fov"] = ParamControl("h-FOV", 10, 150, step=0.1, decimals=1, slider=False,
                                     unit=" deg")
        for k in ("area", "mm", "fov"):
            gl.addWidget(self.p[k])
        mid.addWidget(g)
        panels.append(("Blobs", g))

        g = QGroupBox("Sensor")
        gl = QHBoxLayout(g)
        self.awb_chk = QCheckBox("Auto white balance")
        self.awb_chk.clicked.connect(lambda on: self.main.send("w1" if on else "w0"))
        self.awb_chk.setToolTip("Unticked = white balance frozen at the gains it has now.")
        gl.addWidget(self.awb_chk)
        self.order_btn = QPushButton("Byte order: ?")
        self.order_btn.setToolTip("Flips how the camera reads RGB565 ('o'). Big-endian was "
                                  "confirmed correct on 2026-10-13; the wrong one looks like "
                                  "speckle everywhere.")
        self.order_btn.clicked.connect(lambda: self.main.send("o"))
        gl.addWidget(self.order_btn)
        mid.addWidget(g)
        panels.append(("Sensor", g))

        g = QGroupBox("Camera flash memory")
        gl = QHBoxLayout(g)
        b = QPushButton("Save to camera")
        b.setToolTip("!save: the camera boots with these settings from now on.")
        b.clicked.connect(lambda: self.main.send("!save"))
        gl.addWidget(b)
        b = QPushButton("Reload saved")
        b.clicked.connect(lambda: self.main.send("!load"))
        gl.addWidget(b)
        b = QPushButton("Factory defaults")
        b.clicked.connect(self._factory)
        gl.addWidget(b)
        mid.addWidget(g)
        panels.append(("Flash", g))

        g = QGroupBox("Files")
        gl = QHBoxLayout(g)
        b = QPushButton("Save frame...")
        b.setToolTip("PNG for viewing, the raw .rgb565 bytes and the settings as .json "
                     "(enough to replay this frame through the host tests).")
        b.clicked.connect(self._save_frame)
        gl.addWidget(b)
        self.file_lbl = QLabel()
        self.file_lbl.setWordWrap(True)
        gl.addWidget(self.file_lbl, 1)
        left.addWidget(g)
        left.addWidget(self.settings_lbl)
        for col in (left, mid):
            col.addStretch(1)
        root.addLayout(self._panel_strip(panels))

        # The firmware defaults, until the camera's #!CFG says otherwise.
        for key, v in (("hue", 0), ("tol", 12), ("sat", 110), ("val", 50), ("area", 25),
                       ("mm", 550), ("fov", 54.0)):
            self.p[key]._set_widgets(v)
        for key, ctl in self.p.items():
            ctl.changed.connect(lambda _v: self._redraw())
            ctl.commit.connect(lambda v, k=key: self._commit(k, v))
        self._redraw()

    @staticmethod
    def _column(root: QHBoxLayout) -> QVBoxLayout:
        """A vertically scrolling control column, added to root."""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        w = QWidget()
        col = QVBoxLayout(w)
        scroll.setWidget(w)
        root.addWidget(scroll, 1)
        return col

    @staticmethod
    def _panel_strip(panels: list) -> QVBoxLayout:
        """One toggle button per panel: checked = shown. The choice is kept
        in QSettings, so a panel hidden once stays hidden next session."""
        store = QSettings("icarus_blimp", "tcview")
        strip = QVBoxLayout()
        for name, box in panels:
            b = QPushButton(name)
            b.setCheckable(True)
            b.setFixedWidth(70)
            b.setToolTip("Show or hide the '%s' panel" % box.title())
            key = "calib/show/" + name
            shown = store.value(key, True, type=bool)
            b.toggled.connect(box.setVisible)
            # A checkable button looks almost the same both ways in most
            # styles; grey text says "hidden" at a glance. Set on each toggle,
            # because a :checked stylesheet rule did not repaint reliably.
            b.toggled.connect(lambda on, b=b: b.setStyleSheet("" if on else "color: grey;"))
            b.setChecked(not shown)         # so the setChecked below always toggles
            b.setChecked(shown)
            b.toggled.connect(lambda on, k=key: store.setValue(k, on))
            strip.addWidget(b)
        strip.addStretch(1)
        return strip

    # ---- from the main window ------------------------------------------------

    def settings(self) -> D.Settings:
        """The detector settings the controls show (may not be applied yet)."""
        s = D.Settings.from_cfg(self.cfg)
        for k in ("hue", "tol", "sat", "val", "area"):
            setattr(s, k, self.p[k].local)
        if self.frame is not None:
            s.be = int(self.frame.big_endian)
        return s

    def stage(self) -> int:
        """The stage the camera was asked for. The PC runs the same request
        through the same fallback, so it shows what the camera computes."""
        return int(self.cfg.get("stage", 0))

    def _show_stage(self, v: dict):
        req, eff = v.get("stage", 0), v.get("stage_eff", 0)
        if 0 <= req < len(STAGE_NAMES):
            self.stage_combo.setCurrentIndex(req)
        pc = D.models()
        cam_id = v.get("lut_id", 0)
        if v.get("lut") == "ok" and pc.lut and cam_id != pc.lut_id:
            chip(self.stage_lbl, "Colour table MISMATCH: camera %s, PC %d. The PC overlay "
                 "is not what the camera computes; reflash, or retrain on the PC." % (
                     cam_id, pc.lut_id), RED)
        elif eff >= 1 and not pc.lut:
            chip(self.stage_lbl, "Camera runs stage %d, but the PC has no colour table "
                 "(main/models/lut_model.bin): the overlay falls back to HSV." % eff, RED)
        elif eff != req:
            chip(self.stage_lbl, "Stage %d requested, running %d (colour table %s)." % (
                req, eff, v.get("lut", "?")), AMBER)
        elif eff >= 1:
            chip(self.stage_lbl, "Stage %d: colour table %s decides the pixels. Hue, tol, sat, "
                 "val and 'c' have no effect until stage 0." % (eff, cam_id), GREEN)
        else:
            chip(self.stage_lbl, "Stage 0: the HSV window below decides the pixels.", GREY)

    def on_cfg(self, v: dict):
        self.cfg = v
        self._show_stage(v)
        for key, (_, ck) in PARAMS.items():
            if ck in v:
                self.p[key].set_camera(v[ck])
        self.live_chk.setChecked(v.get("image") == 1)
        self.awb_chk.setChecked(v.get("awb") == 1)
        self.order_btn.setText("Byte order: %s-endian" % ("big" if v.get("be") == 1 else "LITTLE"))
        preset = v.get("preset", "?")
        self.preset_lbl.setText("camera: %s" % preset)
        if preset in D.PRESETS:
            self.preset_combo.setCurrentText(preset)
        st = v.get("settings")
        if st == "saved":
            chip(self.settings_lbl, "Camera settings: SAVED in flash - a power cycle keeps them",
                 GREEN)
        elif st == "defaults":
            chip(self.settings_lbl, "Camera settings: factory defaults (nothing saved)", GREY)
        else:
            chip(self.settings_lbl, "Camera settings: MODIFIED, NOT SAVED - a power cycle loses "
                 "them. Press 'Save to camera' when done.", AMBER)
        self._redraw()

    def on_frame(self, frame: P.Frame):
        self.frame = frame
        self.frame_row = None
        self.frame_time = time.monotonic()
        self._redraw()
        self._update_region()

    def on_row(self, row: P.Row):
        if self.frame is not None and self.frame_row is None and \
                int(row.v["t_ms"]) == self.frame.t_ms:
            self.frame_row = row.v
            self._redraw()

    def on_command_done(self, cmd: str, ok: bool, reason: str):
        ctl = self.pending_cmds.pop(cmd, None)
        if ctl is not None:
            ctl.done(ok)

    def on_cal(self, cal: P.Cal):
        v = cal.v
        txt = "camera sampled hue %s +/-%s, sat %s, val %s" % (
            v.get("hue"), v.get("spread"), v.get("sat"), v.get("val"))
        if cal.ok:
            chip(self.cal_lbl, txt + ": applied", GREEN)
        else:
            chip(self.cal_lbl, txt + ": REFUSED (too grey)", RED)

    def tick(self):
        """1 Hz from the main window: link figures."""
        link = self.main.link
        if link is None:
            self.link_lbl.setText("not connected")
            return
        c = link.counters
        age = time.monotonic() - self.frame_time if self.frame else None
        self.link_lbl.setText(
            "now at %d baud, %.1f kB/s on the wire.  frames ok %d, bad %d (CRC/truncated)%s.\n"
            "While frames are sent the camera's dt_ms/FPS are NOT flight timing."
            % (link.baud, c.rate() / 1000, c.frames_ok, c.frames_bad,
               "" if age is None else ", last %.1f s ago" % age))

    # ---- actions ---------------------------------------------------------------

    def _commit(self, key, v):
        fmt, _ = PARAMS[key]
        cmd = fmt % v
        self.pending_cmds[cmd] = self.p[key]
        self.main.send(cmd)

    def _use_region(self):
        if self.frame is None:
            return
        x, y, w, h = self.pane.region()
        col = D.sample(self.settings(), self.frame.data, x, y, w, h)
        cal = D.calibrate_from(col)
        if cal.warning:
            chip(self.use_warn, cal.warning, RED if cal.refused else AMBER)
        else:
            chip(self.use_warn, "hue %d +/-%d, sat >= %d, val >= %d sent" % (
                cal.hue, cal.tol, cal.sat, cal.val), GREEN)
        for k, v in (("hue", cal.hue), ("tol", cal.tol), ("sat", cal.sat), ("val", cal.val)):
            self.p[k].set_local(v, send=True)

    def _factory(self):
        if QMessageBox.question(self, "Factory defaults",
                                "Erase the settings saved in the camera and go back to the "
                                "firmware defaults?") == QMessageBox.StandardButton.Yes:
            self.main.send("!reset")

    def _save_frame(self):
        if self.frame is None:
            self.file_lbl.setText("no frame to save")
            return
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        # The dialog opens where the last frame went: a capture session is one
        # subfolder (the training splits train/validation by session), chosen
        # once rather than navigated to for every frame.
        folder = getattr(self, "_save_dir", DATA_DIR)
        if not folder.is_dir():
            folder = DATA_DIR
        stem = time.strftime("frame_%Y%m%d_%H%M%S") + "_seq%d" % self.frame.seq
        path, _ = QFileDialog.getSaveFileName(self, "Save frame", str(folder / (stem + ".png")),
                                              "PNG (*.png)")
        if not path:
            return
        base = Path(path).with_suffix("")
        self._save_dir = base.parent
        rgb = np.ascontiguousarray(self.frame.rgb())
        QImage(rgb.data, W, H, 3 * W, QImage.Format.Format_RGB888).copy().save(str(base) + ".png")
        Path(str(base) + ".rgb565").write_bytes(self.frame.data)
        meta = {"seq": self.frame.seq, "t_ms": self.frame.t_ms, "w": W, "h": H,
                "fmt": self.frame.fmt, "camera_cfg": self.cfg,
                "camera_row": self.frame_row,
                "viewer_controls": {k: c.local for k, c in self.p.items()},
                "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
        Path(str(base) + ".json").write_text(json.dumps(meta, indent=2))
        self.file_lbl.setText("saved %s.{png,rgb565,json}" % base.name)

    # ---- drawing ---------------------------------------------------------------

    def _hover_at(self, x, y):
        self._hover = None if y is None else (x, y)
        self._show_hover()

    def _show_hover(self):
        if self._hover is None or self.frame is None:
            return
        x, y = self._hover
        px = int(self.frame.pixels()[y, x])
        hh, s, v = (int(a) for a in D.hsv_lut()[px])
        r, g, b = (int(a) for a in P.rgb565_to_rgb888(np.array([[px]], dtype=np.uint16))[0, 0])
        st = self.settings()
        m = D.mask(st, self.frame.data, self.stage())[y, x]
        self.hover_lbl.setText("x %3d y %3d  rgb565 %04x  RGB %3d %3d %3d  ->  H %3d  S %3d  "
                               "V %3d   %s" % (x, y, px, r, g, b, hh, s, v,
                                              "MATCH" if m else "no match"))

    def _update_region(self):
        if self.frame is None:
            return
        x, y, w, h = self.pane.region()
        x1, y1 = min(W, x + w), min(H, y + h)
        px = self.frame.pixels()[max(0, y):y1, max(0, x):x1].ravel()
        if px.size == 0:
            return
        hsv = D.hsv_lut()[px]
        col = D.sample(self.settings(), self.frame.data, x, y, w, h)
        sat, val = hsv[:, 1], hsv[:, 2]
        q = lambda a: "%3d/%3d/%3d" % tuple(np.percentile(a, [10, 50, 90]).astype(int))
        st = self.settings()
        m = D.mask(st, self.frame.data, self.stage())[max(0, y):y1, max(0, x):x1]
        self.region_lbl.setText(
            "%dx%d at (%d,%d): hue %d +/-%d (camera's sampler)\n"
            "sat p10/50/90 %s   val p10/50/90 %s\n"
            "matching now: %d of %d px (%.0f%%)"
            % (x1 - max(0, x), y1 - max(0, y), x, y, col.hue, col.hue_spread, q(sat), q(val),
               int(m.sum()), m.size, 100.0 * m.sum() / m.size))
        coloured = hsv[sat >= 30, 0]
        counts, _ = np.histogram(coloured, bins=36, range=(0, 360))
        self.bars.setOpts(height=counts)
        self.huebar.set_mark(col.hue)

    def _redraw(self):
        st = self.settings()
        self.huebar.set_window(st.hue, st.tol)
        self.win_lo.setPos((st.hue - st.tol) % 360)
        self.win_hi.setPos((st.hue + st.tol) % 360)
        pane = self.pane
        if self.frame is None:
            return
        rgb = self.frame.rgb()
        mask = D.mask(st, self.frame.data, self.stage())
        mode = self.overlay_mode.currentIndex()
        if mode == 1:
            pane.img.setImage(np.repeat(mask[..., None] * 255, 3, axis=2).astype(np.uint8),
                              levels=(0, 255))
            pane.overlay.clear()
        else:
            pane.img.setImage(rgb, levels=(0, 255))
            if mode == 0:
                ov = np.zeros((H, W, 4), dtype=np.uint8)
                ov[mask == 1] = (0, 255, 255, 150)
                pane.overlay.setImage(ov, levels=(0, 255))
            else:
                pane.overlay.clear()

        r = D.pipeline(st, self.frame.data, self.stage()).r
        for k in range(D.BLOB_MAX_REPORT):
            vis = self.show_boxes.isChecked() and k < r.n_report
            pane.boxes[k].setVisible(vis)
            pane.labels[k].setVisible(vis)
            if vis:
                b = r.b[k]
                pane.boxes[k].setRect(b.x0, b.y0, b.x1 - b.x0 + 1, b.y1 - b.y0 + 1)
                pane.labels[k].setPos(b.x0, b.y0)
                pane.labels[k].setText("%d%s" % (k + 1, " clipped" if b.clipped else ""))

        cam = []
        row = self.frame_row
        if row and row["ok"]:
            cam.append((row["cx_px"], row["cy_px"]))
            f = (W / 2) / math.tan(math.radians(self.cfg.get("fov", 54.0)) / 2)
            for k in range(2, int(min(row["blobs"], 4)) + 1):
                az, el = row["b%d_az" % k], row["b%d_el" % k]
                cam.append((W / 2 + f * math.tan(math.radians(az)),
                            H / 2 - f * math.tan(math.radians(el))))
        if cam:
            pane.cam_marks.setData(pos=cam)
        else:
            pane.cam_marks.clear()

        b0 = r.b[0]
        pc = ("PC  (controls): %s  blobs %d  area %4d  matched %5d"
              % ("TARGET   " if r.found else "NO TARGET", r.n_blobs, b0.area, r.matched))
        if row is None:
            cm = "camera (its row): not received for this frame"
        else:
            cm = ("camera (its row): %s  blobs %d  area %4d  matched %5d"
                  % ("TARGET   " if row["ok"] else "NO TARGET", row["blobs"], row["area_px"],
                     row["matched_px"]))
        self.compare_lbl.setText(pc + "\n" + cm)
        age = time.monotonic() - self.frame_time
        self.frame_lbl.setText("frame seq %d, camera t %.1f s, %s; received %.1f s ago" % (
            self.frame.seq, self.frame.t_ms / 1000, self.frame.fmt, age))
        self._show_hover()
