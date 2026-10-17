"""
The Blobs tab: collect and label the data stage 2's classifier (Phase B2)
will learn from.

Each item is one blob: its 24x24 crop, framed in the colour of its label
(grey "?", green balloon, red not a balloon). Items come from two places
(blobset.py explains both): '!collect 1' records from the camera, which
arrive unlabelled, and saved labelled frames, whose blobs are labelled from
the paint. Labelling is by hand here: select one or many (shift / ctrl /
drag) and press B, N or U, or double-click to cycle one.

The agreement line is the B1 board check. The camera computes the features
and crops in flight, the PC computes them for training, and the classifier is
only valid if the two are the same numbers. When a record arrives for a frame
that was also grabbed ('g' or 'v1'), the PC re-runs the same C code on those
pixels and compares.
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
from PyQt6.QtCore import QSize, Qt
from PyQt6.QtGui import (QColor, QIcon, QImage, QKeySequence, QPainter, QPen, QPixmap,
                         QShortcut)
from PyQt6.QtWidgets import (QAbstractItemView, QFileDialog, QHBoxLayout, QLabel,
                             QListView, QListWidget, QListWidgetItem, QMessageBox,
                             QPlainTextEdit, QPushButton, QVBoxLayout, QWidget)

from . import blobset as B
from . import detector as D
from . import protocol as P
from .calib_tab import DATA_DIR
from .ui import theme
from .ui.widgets import GripSplitter
from .widgets import AMBER, GREEN, GREY, RED, chip

ICON = 3 * P.CROP           # crops shown at x3: a 24 px balloon edge is visible
BIG = 8 * P.CROP            # the selected crop in the side panel
LABEL_COLOUR = {B.BALLOON: GREEN, B.NOT_BALLOON: RED, B.UNLABELLED: GREY}
LABEL_TEXT = {B.BALLOON: "balloon", B.NOT_BALLOON: "not", B.UNLABELLED: "?"}
CYCLE = {B.UNLABELLED: B.BALLOON, B.BALLOON: B.NOT_BALLOON, B.NOT_BALLOON: B.UNLABELLED}


def crop_pixmap(s: B.Sample, size: int, border: int = 0) -> QPixmap:
    rgb = np.ascontiguousarray(s.rgb())
    img = QImage(rgb.data, P.CROP, P.CROP, 3 * P.CROP, QImage.Format.Format_RGB888).copy()
    pm = QPixmap(size + 2 * border, size + 2 * border)
    pm.fill(QColor(LABEL_COLOUR[s.label]))
    p = QPainter(pm)
    # Nearest neighbour: the classifier sees these pixels, not smoothed ones.
    p.drawImage(border, border, img.scaled(size, size))
    if border == 0:
        p.setPen(QPen(QColor(LABEL_COLOUR[s.label]), 3))
        p.drawRect(1, 1, size - 3, size - 3)
    p.end()
    return pm


class BlobsTab(QWidget):
    def __init__(self, main=None):
        super().__init__()
        self.main = main
        self.samples: list[B.Sample] = []
        self.cfg: dict = {}
        self.last_frame: P.Frame | None = None
        self.agree = B.Agreement()
        self.dirty = False
        self.autosave_dir: Path | None = DATA_DIR / "blobs"
        self._live_t = []           # arrival times, for the rate

        lay = QVBoxLayout(self)
        top = QHBoxLayout()
        self.collect_btn = QPushButton("Collect from camera")
        self.collect_btn.setCheckable(True)
        self.collect_btn.setToolTip(
            "'!collect 1': every 4th frame, and every frame sent to the PC ('g', 'v1'), "
            "the camera sends the features and a 24x24 crop of each reported blob. "
            "115200 is enough. It slows the loop by the transfer time, so "
            "rows taken while collecting are not flight timing.")
        self.collect_btn.toggled.connect(self._collect_toggled)
        top.addWidget(self.collect_btn)
        self.collect_chip = QLabel()
        top.addWidget(self.collect_chip)
        for text, slot, tip in (
                ("Add from labelled frames...", self._ask_folder,
                 "Every blob the stage 1 pipeline finds on the saved frames in a folder "
                 "(subfolders too), labelled from the painted labels"),
                ("Load dataset...", self._ask_load, "Append the samples of a .npz"),
                ("Save dataset...", self._ask_save, "All samples, labelled or not, to a .npz"),
                ("Clear", self._ask_clear, "Forget every sample")):
            b = QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            top.addWidget(b)
        top.addStretch(1)
        self.count_lbl = QLabel()
        top.addWidget(self.count_lbl)
        lay.addLayout(top)

        ag = QHBoxLayout()
        self.agree_chip = QLabel()
        ag.addWidget(self.agree_chip)
        self.agree_lbl = QLabel()
        self.agree_lbl.setWordWrap(True)
        ag.addWidget(self.agree_lbl, 1)
        lay.addLayout(ag)

        split = GripSplitter(Qt.Orientation.Horizontal)
        self.grid = QListWidget()
        self.grid.setViewMode(QListView.ViewMode.IconMode)
        self.grid.setIconSize(QSize(ICON, ICON))
        self.grid.setGridSize(QSize(ICON + 10, ICON + 22))
        self.grid.setResizeMode(QListView.ResizeMode.Adjust)
        self.grid.setMovement(QListView.Movement.Static)
        self.grid.setUniformItemSizes(True)
        self.grid.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.grid.itemSelectionChanged.connect(self._show_selected)
        self.grid.itemDoubleClicked.connect(self._cycle)
        split.addWidget(self.grid)

        side = QWidget()
        sl = QVBoxLayout(side)
        self.big = QLabel()
        self.big.setFixedSize(BIG, BIG)
        sl.addWidget(self.big)
        self.info = QPlainTextEdit()
        self.info.setReadOnly(True)
        theme.mono(self.info)
        sl.addWidget(self.info, 1)
        split.addWidget(side)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 1)
        lay.addWidget(split, 1)

        hint = QLabel("Select blobs (shift / ctrl / drag) and press B balloon, N not a "
                      "balloon, U unlabelled; double-click cycles one. Live records come "
                      "unlabelled; blobs from labelled frames are labelled from the paint.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: %s;" % GREY)
        lay.addWidget(hint)

        for key, lab in (("B", B.BALLOON), ("N", B.NOT_BALLOON), ("U", B.UNLABELLED)):
            sc = QShortcut(QKeySequence(key), self.grid)
            sc.setContext(Qt.ShortcutContext.WidgetShortcut)
            sc.activated.connect(lambda lab=lab: self.set_label(self.selected(), lab))
        self._refresh_counts()
        self._refresh_agreement()
        self._refresh_collect()

    # ---- events from the camera ---------------------------------------------

    def on_cfg(self, v: dict):
        self.cfg = v
        self._refresh_collect()

    def on_disconnected(self):
        """The link is gone: whether the camera collects is unknown. The
        samples stay."""
        self.cfg = {}
        self._refresh_collect()

    def on_frame(self, f: P.Frame):
        self.last_frame = f

    def on_blob(self, rec: P.BlobRec):
        if rec.fv != P.FEAT_VERSION:
            # Features from another definition cannot be mixed into this set.
            self._say("# [viewer] blob record with feature version %d ignored (the viewer "
                      "has %d)" % (rec.fv, P.FEAT_VERSION))
            return
        f = self.last_frame
        if f is not None and f.t_ms == rec.t_ms:
            self.agree.add(rec, f, D.Settings.from_cfg(self.cfg))
            self._refresh_agreement()
        now = time.monotonic()
        self._live_t = [t for t in self._live_t if now - t < 10.0] + [now]
        self.add([B.from_record(rec)])

    # ---- samples ------------------------------------------------------------

    def add(self, samples: list[B.Sample]):
        if not samples:
            return
        bottom = self.grid.verticalScrollBar().value() >= self.grid.verticalScrollBar().maximum()
        for s in samples:
            self.samples.append(s)
            it = QListWidgetItem()
            it.setData(Qt.ItemDataRole.UserRole, len(self.samples) - 1)
            self._paint_item(it, s)
            self.grid.addItem(it)
        if bottom:
            self.grid.scrollToBottom()
        self.dirty = True
        self._refresh_counts()

    def _paint_item(self, it: QListWidgetItem, s: B.Sample):
        it.setIcon(QIcon(crop_pixmap(s, ICON)))
        it.setText(LABEL_TEXT[s.label])
        it.setToolTip("%s %s  blob %d  area %d" % (s.source, s.ref, s.k, s.area))

    def selected(self) -> list[int]:
        return sorted(it.data(Qt.ItemDataRole.UserRole) for it in self.grid.selectedItems())

    def set_label(self, idx: list[int], lab: int):
        for i in idx:
            self.samples[i].label = lab
            self._paint_item(self.grid.item(i), self.samples[i])
        if idx:
            self.dirty = True
            self._refresh_counts()
            self._show_selected()

    def _cycle(self, it: QListWidgetItem):
        i = it.data(Qt.ItemDataRole.UserRole)
        self.set_label([i], CYCLE[self.samples[i].label])

    def clear(self):
        self.samples = []
        self.grid.clear()
        self.dirty = False
        self._refresh_counts()

    def add_folder(self, folder: Path) -> int:
        """Blobs of the labelled frames under folder, except frames already
        in the set (adding a folder twice, or a parent, counts each once)."""
        have = {s.ref for s in self.samples if s.source == "frame"}
        new = [s for s in B.from_folder(Path(folder), stage=1) if s.ref not in have]
        self.add(new)
        return len(new)

    def save_to(self, path: Path):
        B.save(Path(path), self.samples)
        self.dirty = False
        self._say("# [viewer] %d blobs saved to %s" % (len(self.samples), path))

    def load_from(self, path: Path) -> int:
        new = B.load(Path(path))
        self.add(new)
        return len(new)

    def autosave(self) -> Path | None:
        """On close: records exist only on the wire, so an unsaved session is
        written rather than lost. Nothing is asked, so closing never blocks."""
        if not self.dirty or not self.samples or self.autosave_dir is None:
            return None
        self.autosave_dir.mkdir(parents=True, exist_ok=True)
        path = self.autosave_dir / time.strftime("blobs_autosave_%Y%m%d_%H%M%S.npz")
        self.save_to(path)
        return path

    # ---- buttons --------------------------------------------------------------

    def _collect_toggled(self, on: bool):
        if self.main is not None:
            self.main.send("!collect %d" % int(on))

    def _ask_folder(self):
        d = QFileDialog.getExistingDirectory(self, "Saved frames to take blobs from",
                                             str(DATA_DIR))
        if d:
            n = self.add_folder(Path(d))
            self._say("# [viewer] %d blobs from the labelled frames under %s" % (n, d))

    def _ask_load(self):
        p, _ = QFileDialog.getOpenFileName(self, "Load blob dataset", str(DATA_DIR),
                                           "Blob datasets (*.npz)")
        if p:
            try:
                self.load_from(Path(p))
            except (ValueError, KeyError, OSError) as e:
                QMessageBox.warning(self, "Load dataset", "%s: %s" % (p, e))

    def _ask_save(self):
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        p, _ = QFileDialog.getSaveFileName(
            self, "Save blob dataset", str(DATA_DIR / time.strftime("blobs_%Y%m%d_%H%M%S.npz")),
            "Blob datasets (*.npz)")
        if p:
            self.save_to(Path(p if p.endswith(".npz") else p + ".npz"))

    def _ask_clear(self):
        if self.dirty and QMessageBox.question(
                self, "Clear", "%d blobs, not saved since the last change. Forget them?"
                % len(self.samples)) != QMessageBox.StandardButton.Yes:
            return
        self.clear()

    # ---- display --------------------------------------------------------------

    def _say(self, text: str):
        if self.main is not None:
            self.main._say(text)

    def _refresh_collect(self):
        on = bool(self.cfg.get("collect", 0))
        self.collect_btn.blockSignals(True)
        self.collect_btn.setChecked(on)
        self.collect_btn.blockSignals(False)
        if "collect" not in self.cfg:
            chip(self.collect_chip, "camera: unknown", GREY)
        elif on:
            chip(self.collect_chip, "camera: collecting, %.1f blobs/s" % (
                len(self._live_t) / 10.0), AMBER)
        else:
            chip(self.collect_chip, "camera: not collecting", GREY)

    def _refresh_counts(self):
        c = B.counts(self.samples)
        self.count_lbl.setText("%d blobs: %d balloon, %d not, %d ?%s" % (
            c["n"], c["balloon"], c["not"], c["unlabelled"], "  (unsaved)" if self.dirty else ""))

    def _refresh_agreement(self):
        a = self.agree
        if not a.checked and not a.missing:
            chip(self.agree_chip, "board = PC: not checked", GREY)
            self.agree_lbl.setText("Collect, then send 'g': the frame comes with its records, "
                                   "and the PC recomputes them on the same pixels.")
            return
        chip(self.agree_chip, "board = PC" if a.ok() else "board != PC", GREEN if a.ok() else RED)
        self.agree_lbl.setText(a.text())

    def tick(self):
        """1 Hz, from the main window: the collection rate."""
        now = time.monotonic()
        self._live_t = [t for t in self._live_t if now - t < 10.0]
        self._refresh_collect()

    def _show_selected(self):
        idx = self.selected()
        if not idx:
            self.big.clear()
            self.info.setPlainText("")
            return
        s = self.samples[idx[-1]]
        self.big.setPixmap(crop_pixmap(s, BIG))
        names = D.feat_names()
        lines = ["%s  %s  blob %d" % (s.source, s.ref, s.k),
                 "label     %s" % LABEL_TEXT[s.label],
                 "stage %d   conf %.3g   fw %s   fv %d" % (s.stage, s.conf, s.fw, s.fv),
                 "centroid  %.1f, %.1f   area %d" % (s.cx, s.cy, s.area),
                 "box       %d,%d - %d,%d" % s.box,
                 "crop      x %d  y %d  side %d" % s.crop_box, ""]
        lines += ["%-11s %10.4g" % (n, v) for n, v in zip(names, s.feat)]
        if len(idx) > 1:
            lines += ["", "(%d selected; showing the last)" % len(idx)]
        self.info.setPlainText("\n".join(lines))
