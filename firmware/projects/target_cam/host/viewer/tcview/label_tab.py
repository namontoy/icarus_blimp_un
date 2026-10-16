"""
Label frames: paint which pixels are balloon and which are background.

Works on saved frames (Calib tab, Save frame), with or without a camera: open
a folder, pick a frame, paint, and the labels are written next to it as
<stem>.labels.png (format in labels.py). They are saved when you move to
another frame, and with Save (Ctrl+S).

The quick way through a frame:
  1. Ellipse tool, Balloon class: drag a box around the balloon. A balloon is
     round, so this covers most of it in one stroke; fix the edge with the
     brush (Erase takes pixels back to unlabelled).
  2. "Rest -> background": every pixel not yet labelled becomes background,
     except a band around the balloon, which stays unlabelled because the rim
     is a mix of both colours.
  3. Check the rim band and any second balloon, then move on (PgDown).
  A frame with no balloon is labelled by "Rest -> background" alone.

The reflections on the mylar are part of the balloon: label them balloon.
Whether the colour table should match a white highlight is the training's
decision (it counts how often that colour is balloon versus background), not
the label's.

Right mouse button erases with the current tool. Keys (while this tab is
shown): 1 balloon, 2 background, 0 erase, B brush, R rectangle, E ellipse,
Ctrl+Z undo, Ctrl+S save, PgDown / PgUp next / previous frame.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QRectF, Qt
from PyQt6.QtGui import QKeySequence, QShortcut
from PyQt6.QtWidgets import (QButtonGroup, QCheckBox, QComboBox, QFileDialog,
                             QGraphicsEllipseItem,
                             QGraphicsRectItem, QGroupBox, QHBoxLayout, QLabel,
                             QListWidget, QPushButton, QRadioButton, QSizePolicy,
                             QSlider, QSpinBox, QVBoxLayout, QWidget)

from . import detector as D
from . import labels as L
from .calib_tab import DATA_DIR, SCALE, STAGE_NAMES

# Label colours over the image. Neither is red or green, so a balloon of
# either colour stays readable under its tint.
TINT = {L.BALLOON: (0, 200, 255), L.BACKGROUND: (255, 160, 0)}
UNDO_DEPTH = 50


class PaintItem(pg.ImageItem):
    """The label overlay; it also takes the mouse. Item coordinates are frame
    pixels (the ImageItem is not scaled; the ViewBox is)."""

    def __init__(self, tab: "LabelTab"):
        super().__init__(axisOrder="row-major")
        self.tab = tab
        self.setAcceptHoverEvents(True)

    def mouseClickEvent(self, ev):
        if ev.button() in (Qt.MouseButton.LeftButton, Qt.MouseButton.RightButton):
            ev.accept()
            self.tab.click(ev.pos(), erase=ev.button() == Qt.MouseButton.RightButton)

    def mouseDragEvent(self, ev):
        if ev.button() not in (Qt.MouseButton.LeftButton, Qt.MouseButton.RightButton):
            ev.ignore()
            return
        ev.accept()
        erase = ev.button() == Qt.MouseButton.RightButton
        if ev.isStart():
            self.tab.drag_start(ev.buttonDownPos(), erase)
        self.tab.drag_move(ev.pos(), erase)
        if ev.isFinish():
            self.tab.drag_end(ev.pos(), erase)

    def hoverEvent(self, ev):
        self.tab.hover(None if ev.isExit() else ev.pos())


class LabelCanvas(pg.GraphicsLayoutWidget):
    def __init__(self, tab: "LabelTab", w: int, h: int):
        super().__init__()
        self.setMinimumSize(w * SCALE + 4, h * SCALE + 4)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.ci.setContentsMargins(0, 0, 0, 0)
        self.vb = self.addViewBox(lockAspect=True, invertY=True, enableMouse=False)
        self.vb.setMenuEnabled(False)
        self.vb.setRange(QRectF(0, 0, w, h), padding=0)
        self.img = pg.ImageItem(axisOrder="row-major")
        self.vb.addItem(self.img)
        self.paint = PaintItem(tab)
        self.vb.addItem(self.paint)
        pen = pg.mkPen((255, 255, 255), width=1, style=Qt.PenStyle.DashLine, cosmetic=True)
        self.rect_preview = QGraphicsRectItem(0, 0, 1, 1)
        self.ell_preview = QGraphicsEllipseItem(0, 0, 1, 1)
        for it in (self.rect_preview, self.ell_preview):
            it.setPen(pen)
            it.setVisible(False)
            self.vb.addItem(it)
        # The detector's answer on this frame, with the settings the camera had.
        self.det_boxes = []
        for k in range(D.BLOB_MAX_REPORT):
            r = QGraphicsRectItem(0, 0, 1, 1)
            r.setPen(pg.mkPen((255, 230, 0) if k == 0 else (255, 120, 0), width=2,
                              cosmetic=True))
            r.setVisible(False)
            self.vb.addItem(r)
            self.det_boxes.append(r)
        # The balloons the labels define (what eval_frames scores against).
        self.bal_boxes = []


class LabelTab(QWidget):
    def __init__(self, folder: Path | None = None):
        super().__init__()
        self.folder = Path(folder or DATA_DIR)
        self.stems: list[Path] = []
        self.sf: L.SavedFrame | None = None
        self.lab = np.zeros((120, 160), dtype=np.uint8)
        self.rgb = np.zeros((120, 160, 3), dtype=np.uint8)
        self.dirty = False
        self.undo: list[np.ndarray] = []
        self._drag0 = None
        self._last = None
        self._det = None                # detector result and mask for this frame
        self._mask = None
        self._stage_eff = 0

        root = QHBoxLayout(self)
        left = QVBoxLayout()
        root.addLayout(left)
        self.canvas = LabelCanvas(self, 160, 120)
        left.addWidget(self.canvas)
        self.hover_lbl = QLabel("hover over the image")
        self.hover_lbl.setStyleSheet("font-family: monospace;")
        left.addWidget(self.hover_lbl)
        self.sum_lbl = QLabel("")
        self.sum_lbl.setStyleSheet("font-family: monospace;")
        self.sum_lbl.setWordWrap(True)
        left.addWidget(self.sum_lbl)
        self.det_lbl = QLabel("")
        self.det_lbl.setStyleSheet("font-family: monospace;")
        self.det_lbl.setWordWrap(True)
        left.addWidget(self.det_lbl)
        left.addStretch(1)

        right = QVBoxLayout()
        root.addLayout(right, 1)

        g = QGroupBox("Frames")
        gl = QVBoxLayout(g)
        fl = QHBoxLayout()
        self.folder_lbl = QLabel()
        self.folder_lbl.setWordWrap(True)
        fl.addWidget(self.folder_lbl, 1)
        b = QPushButton("Open folder...")
        b.clicked.connect(self._choose_folder)
        fl.addWidget(b)
        b = QPushButton("Rescan")
        b.setToolTip("Look again for frames saved since the folder was opened")
        b.clicked.connect(self.rescan)
        fl.addWidget(b)
        gl.addLayout(fl)
        self.listw = QListWidget()
        self.listw.currentRowChanged.connect(self._pick)
        gl.addWidget(self.listw, 1)
        nl = QHBoxLayout()
        b = QPushButton("< Previous")
        b.clicked.connect(lambda: self.step(-1))
        nl.addWidget(b)
        b = QPushButton("Next >")
        b.clicked.connect(lambda: self.step(1))
        nl.addWidget(b)
        gl.addLayout(nl)
        right.addWidget(g, 1)

        g = QGroupBox("Paint")
        gl = QVBoxLayout(g)
        cl = QHBoxLayout()
        self.cls_group = QButtonGroup(self)
        for text, v in (("Balloon (1)", L.BALLOON), ("Background (2)", L.BACKGROUND),
                        ("Erase (0)", L.UNLABELLED)):
            rb = QRadioButton(text)
            self.cls_group.addButton(rb, v)
            cl.addWidget(rb)
        self.cls_group.button(L.BALLOON).setChecked(True)
        gl.addLayout(cl)
        tl = QHBoxLayout()
        self.tool_group = QButtonGroup(self)
        for i, text in enumerate(("Brush (B)", "Rectangle (R)", "Ellipse (E)")):
            rb = QRadioButton(text)
            self.tool_group.addButton(rb, i)
            tl.addWidget(rb)
        self.tool_group.button(2).setChecked(True)
        gl.addLayout(tl)
        bl = QHBoxLayout()
        bl.addWidget(QLabel("Brush radius"))
        self.radius = QSpinBox()
        self.radius.setRange(0, 15)
        self.radius.setValue(2)
        self.radius.setSuffix(" px")
        self.radius.setToolTip("0 = one pixel")
        bl.addWidget(self.radius)
        bl.addStretch(1)
        gl.addLayout(bl)
        al = QHBoxLayout()
        b = QPushButton("Rest -> background")
        b.setToolTip("Every unlabelled pixel becomes background, except a band around the "
                     "balloon (mixed rim colours stay unlabelled)")
        b.clicked.connect(self.fill_rest)
        al.addWidget(b)
        al.addWidget(QLabel("band"))
        self.band = QSpinBox()
        self.band.setRange(0, 6)
        self.band.setValue(2)
        self.band.setSuffix(" px")
        al.addWidget(self.band)
        al.addStretch(1)
        gl.addLayout(al)
        al = QHBoxLayout()
        b = QPushButton("Undo")
        b.clicked.connect(self.do_undo)
        al.addWidget(b)
        b = QPushButton("Clear all")
        b.clicked.connect(self.clear)
        al.addWidget(b)
        self.save_btn = QPushButton("Save")
        self.save_btn.clicked.connect(self.save)
        al.addWidget(self.save_btn)
        gl.addLayout(al)
        right.addWidget(g)

        g = QGroupBox("Show")
        gl = QVBoxLayout(g)
        ol = QHBoxLayout()
        ol.addWidget(QLabel("Label opacity"))
        self.opacity = QSlider(Qt.Orientation.Horizontal)
        self.opacity.setRange(0, 100)
        self.opacity.setValue(45)
        self.opacity.valueChanged.connect(lambda *_: self._draw_labels())
        ol.addWidget(self.opacity, 1)
        gl.addLayout(ol)
        self.show_det = QCheckBox("Detector boxes (the camera's settings for this frame)")
        self.show_det.setChecked(True)
        self.show_det.toggled.connect(lambda *_: self._draw_detection())
        gl.addWidget(self.show_det)
        sl = QHBoxLayout()
        sl.addWidget(QLabel("Detector stage:"))
        self.stage_combo = QComboBox()
        self.stage_combo.addItems(STAGE_NAMES[:2])
        self.stage_combo.setToolTip("Stage 1 uses the committed colour table "
                                    "(main/models/lut_model.bin) instead of the HSV window.")
        self.stage_combo.currentIndexChanged.connect(lambda *_: self._draw_detection())
        sl.addWidget(self.stage_combo, 1)
        gl.addLayout(sl)
        right.addWidget(g)

        for keys, fn in (("1", lambda: self.cls_group.button(L.BALLOON).setChecked(True)),
                         ("2", lambda: self.cls_group.button(L.BACKGROUND).setChecked(True)),
                         ("0", lambda: self.cls_group.button(L.UNLABELLED).setChecked(True)),
                         ("B", lambda: self.tool_group.button(0).setChecked(True)),
                         ("R", lambda: self.tool_group.button(1).setChecked(True)),
                         ("E", lambda: self.tool_group.button(2).setChecked(True)),
                         ("Ctrl+Z", self.do_undo), ("Ctrl+S", self.save),
                         ("PgDown", lambda: self.step(1)), ("PgUp", lambda: self.step(-1))):
            sc = QShortcut(QKeySequence(keys), self)
            sc.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
            sc.activated.connect(fn)

        self.rescan()

    # ---- frames ------------------------------------------------------------------

    def entered(self):
        """The tab was shown: pick up frames saved meanwhile in the Calib tab."""
        self.rescan()

    def _choose_folder(self):
        d = QFileDialog.getExistingDirectory(self, "Folder of saved frames", str(self.folder))
        if d:
            self.open_folder(Path(d))

    def open_folder(self, folder: Path):
        self.save_if_dirty()
        self.folder = Path(folder)
        self.sf = None
        self.rescan()

    def rescan(self):
        cur = self.sf.stem if self.sf else None
        self.stems = L.list_frames(self.folder) if self.folder.is_dir() else []
        self.folder_lbl.setText("%s  (%d frames)" % (self.folder, len(self.stems)))
        self.listw.blockSignals(True)
        self.listw.clear()
        for s in self.stems:
            self.listw.addItem(self._item_text(s))
        self.listw.blockSignals(False)
        if cur in self.stems:
            self.listw.setCurrentRow(self.stems.index(cur))
        elif self.stems:
            self.listw.setCurrentRow(0)

    def _item_text(self, stem: Path) -> str:
        done = Path(str(stem) + L.LABEL_SUFFIX).exists()
        return ("[x] " if done else "[ ] ") + str(stem.relative_to(self.folder))

    def _pick(self, row: int):
        if not 0 <= row < len(self.stems):
            return
        if self.sf is not None and self.sf.stem == self.stems[row]:
            return
        self.save_if_dirty()
        self.load(self.stems[row])

    def step(self, d: int):
        if self.stems:
            self.listw.setCurrentRow(max(0, min(len(self.stems) - 1, self.listw.currentRow() + d)))

    def load(self, stem: Path):
        try:
            self.sf = L.load_frame(stem)
            f = self.sf.frame
            self.lab = L.load_labels(self.sf.label_path, f.w, f.h)
        except (OSError, ValueError) as e:
            self.sf = None
            self.sum_lbl.setText("cannot load %s: %s" % (stem.name, e))
            return
        self.rgb = f.rgb()
        self.canvas.img.setImage(self.rgb, levels=(0, 255))
        self.undo.clear()
        self.dirty = False
        self._draw_detection()          # first: the label summary scores against it
        self._draw_labels()

    # ---- painting ----------------------------------------------------------------

    def _cls(self, erase: bool) -> int:
        return L.UNLABELLED if erase else self.cls_group.checkedId()

    def _tool(self) -> int:
        return self.tool_group.checkedId()

    def _push_undo(self):
        self.undo.append(self.lab.copy())
        del self.undo[:-UNDO_DEPTH]

    def dab(self, x: float, y: float, cls: int):
        """A disc of the brush radius around pixel (x, y)."""
        r = self.radius.value()
        h, w = self.lab.shape
        xi, yi = int(math.floor(x)), int(math.floor(y))
        y0, y1, x0, x1 = max(0, yi - r), min(h, yi + r + 1), max(0, xi - r), min(w, xi + r + 1)
        if y0 >= y1 or x0 >= x1:
            return
        yy, xx = np.mgrid[y0:y1, x0:x1]
        # (r + 0.5)^2 gives round discs: r=0 one pixel, r=1 the full 3x3.
        disc = (yy - yi) ** 2 + (xx - xi) ** 2 <= (r + 0.5) ** 2
        self.lab[y0:y1, x0:x1][disc] = cls

    def fill_shape(self, ax: float, ay: float, bx: float, by: float, ellipse: bool,
                   cls: int):
        """Fill the pixels whose centres are inside the box (or the ellipse
        inscribed in it) with corners (ax, ay), (bx, by) in pixel units."""
        x0, x1 = sorted((ax, bx))
        y0, y1 = sorted((ay, by))
        h, w = self.lab.shape
        yy, xx = np.mgrid[0:h, 0:w]
        px, py = xx + 0.5, yy + 0.5
        if ellipse:
            rx, ry = max((x1 - x0) / 2, 0.5), max((y1 - y0) / 2, 0.5)
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            m = ((px - cx) / rx) ** 2 + ((py - cy) / ry) ** 2 <= 1.0
        else:
            m = (px >= x0) & (px <= x1) & (py >= y0) & (py <= y1)
        self.lab[m] = cls

    def click(self, pos, erase: bool):
        if self.sf is None or self._tool() != 0:
            return
        self._push_undo()
        self.dab(pos.x(), pos.y(), self._cls(erase))
        self._changed()

    def drag_start(self, pos, erase: bool):
        if self.sf is None:
            return
        self._push_undo()
        self._drag0 = (pos.x(), pos.y())
        self._last = self._drag0

    def drag_move(self, pos, erase: bool):
        if self.sf is None or self._drag0 is None:
            return
        x, y = pos.x(), pos.y()
        if self._tool() == 0:
            # Fill the gap since the last event: a fast stroke moves several px.
            lx, ly = self._last
            n = max(1, int(math.hypot(x - lx, y - ly) * 2))
            for i in range(1, n + 1):
                self.dab(lx + (x - lx) * i / n, ly + (y - ly) * i / n, self._cls(erase))
            self._last = (x, y)
            self._draw_labels()
        else:
            ax, ay = self._drag0
            it = self.canvas.ell_preview if self._tool() == 2 else self.canvas.rect_preview
            it.setRect(QRectF(min(ax, x), min(ay, y), abs(x - ax), abs(y - ay)))
            it.setVisible(True)

    def drag_end(self, pos, erase: bool):
        if self.sf is None or self._drag0 is None:
            return
        if self._tool() != 0:
            ax, ay = self._drag0
            self.fill_shape(ax, ay, pos.x(), pos.y(), self._tool() == 2, self._cls(erase))
        self.canvas.rect_preview.setVisible(False)
        self.canvas.ell_preview.setVisible(False)
        self._drag0 = None
        self._changed()

    def fill_rest(self):
        if self.sf is None:
            return
        self._push_undo()
        self.lab = L.rest_to_background(self.lab, self.band.value())
        self._changed()

    def clear(self):
        if self.sf is None:
            return
        self._push_undo()
        self.lab[:] = L.UNLABELLED
        self._changed()

    def do_undo(self):
        if self.undo:
            self.lab = self.undo.pop()
            self._changed()

    def _changed(self):
        self.dirty = True
        self._draw_labels()

    # ---- saving ------------------------------------------------------------------

    def save(self):
        if self.sf is None:
            return
        L.save_labels(self.sf.label_path, self.lab)
        self.dirty = False
        row = self.stems.index(self.sf.stem) if self.sf.stem in self.stems else -1
        if row >= 0:
            self.listw.item(row).setText(self._item_text(self.sf.stem))
        self._draw_labels()

    def save_if_dirty(self):
        if self.dirty:
            self.save()

    # ---- drawing -----------------------------------------------------------------

    def hover(self, pos):
        if pos is None or self.sf is None:
            self.hover_lbl.setText("hover over the image")
            return
        x, y = int(math.floor(pos.x())), int(math.floor(pos.y()))
        h, w = self.lab.shape
        if not (0 <= x < w and 0 <= y < h):
            return
        r, g, b = self.rgb[y, x]
        name = {L.UNLABELLED: "unlabelled", L.BALLOON: "balloon", L.BACKGROUND: "background"}
        self.hover_lbl.setText("(%3d,%3d)  rgb %3d %3d %3d  %s"
                               % (x, y, r, g, b, name[int(self.lab[y, x])]))

    def _draw_labels(self):
        h, w = self.lab.shape
        a = int(round(self.opacity.value() * 2.55))
        ov = np.zeros((h, w, 4), dtype=np.uint8)
        for v, rgb in TINT.items():
            ov[self.lab == v] = rgb + (a,)
        self.canvas.paint.setImage(ov, levels=(0, 255))
        for it in self.canvas.bal_boxes:
            self.canvas.vb.removeItem(it)
        self.canvas.bal_boxes = []
        bl = L.balloons(self.lab)
        for c in bl:
            r = QGraphicsRectItem(c.x0, c.y0, c.x1 - c.x0 + 1, c.y1 - c.y0 + 1)
            r.setPen(pg.mkPen(TINT[L.BALLOON], width=1, style=Qt.PenStyle.DotLine,
                              cosmetic=True))
            self.canvas.vb.addItem(r)
            self.canvas.bal_boxes.append(r)
        if self.sf is None:
            self.sum_lbl.setText("no frame: open a folder of saved frames")
            return
        n = self.lab.size
        self.sum_lbl.setText(
            "%s%s\nballoon %5d px  background %5d px  unlabelled %5d px (%.0f %%)\n"
            "balloons (labelled objects): %d%s" % (
                self.sf.stem.name, "   * not saved" if self.dirty else "   saved",
                (self.lab == L.BALLOON).sum(), (self.lab == L.BACKGROUND).sum(),
                (self.lab == L.UNLABELLED).sum(), 100.0 * (self.lab == L.UNLABELLED).sum() / n,
                len(bl), "".join("  [%d px at %.0f,%.0f]" % (c.area, c.cx, c.cy) for c in bl[:4])))
        self._draw_score()

    def _draw_detection(self):
        for r in self.canvas.det_boxes:
            r.setVisible(False)
        if self.sf is None:
            self.det_lbl.setText("")
            return
        st = self.sf.settings()
        stage = self.stage_combo.currentIndex()
        tr = D.pipeline(st, self.sf.frame.data, stage)
        self._det, self._mask = tr.r, D.mask(st, self.sf.frame.data, stage)
        self._stage_eff = tr.stage
        res = tr.r
        if self.show_det.isChecked():
            for k in range(res.n_report):
                b = res.b[k]
                self.canvas.det_boxes[k].setRect(b.x0, b.y0, b.x1 - b.x0 + 1, b.y1 - b.y0 + 1)
                self.canvas.det_boxes[k].setVisible(True)
        self._draw_score()

    def _draw_score(self):
        """The eval_frames verdict for this frame, live while labelling."""
        res = self._det
        if self.sf is None or res is None:
            return
        st = self.sf.settings()
        s = L.score(self.sf.stem.name, self.lab,
                    [(res.b[k].cx, res.b[k].cy) for k in range(res.n_report)], self._mask)
        test = ("hue %d +/-%d, sat >= %d, val >= %d" % (st.hue, st.tol, st.sat, st.val)
                if self._stage_eff == 0 else "colour table %d" % D.models().lut_id)
        self.det_lbl.setText(
            "detector, stage %d (%s): %d blob(s) reported\n"
            "vs labels: TP %d  FN %d  FP %d  split %d  blob 1 = %s\n"
            "pixels: precision %s %%  recall %s %%" % (
                self._stage_eff, test, res.n_report, s.tp, s.fn, s.fp, s.split, s.b1,
                _pct(L.ratio(s.px_tp, s.px_tp + s.px_fp)), _pct(L.ratio(s.px_tp, s.px_tp + s.px_fn))))


def _pct(x: float) -> str:
    return "-" if math.isnan(x) else "%.1f" % (100 * x)
