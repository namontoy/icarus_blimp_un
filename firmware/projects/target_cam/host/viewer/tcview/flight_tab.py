"""
Flight data screen: only what leaves the camera in flight.

Entering this screen turns the image and blob collection off (v0,
!collect 0) and returns the link to 115200 baud, so what arrives is exactly the CSV stream the ESP32-S3 will read - same
rows, same rate, same timing. Nothing here comes from the PC detector: every
figure is decoded from the camera's own rows.

The green "flight-identical" banner is shown only when the camera's own #!CFG
confirms image off, collection off, 115200 baud and streaming on, and no
frame has arrived recently. It is the camera's word, not the viewer's assumption.

The pacing check counts every row whose dt_ms is not the configured period
(80.0 ms at 12.5 FPS). The hard requirement is zero such rows with every stage
enabled, so the count runs since connection (or the last reset), not over a
window: one late frame in ten minutes must still show.
"""
from __future__ import annotations

import math
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                             QPushButton, QTableWidget, QTableWidgetItem,
                             QVBoxLayout, QWidget)

from . import protocol as P
from .calib_tab import DATA_DIR
from .ui import theme
from .widgets import AMBER, GREEN, GREY, RED, chip

W, H = 160, 120
HISTORY_S = 30.0


class FlightTab(QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main
        self.cfg: dict = {}
        self.rows: deque = deque()          # (pc_time, row dict)
        self.last_row: P.Row | None = None
        self.last_frame_at = 0.0
        self.rec = None
        self.rec_n = 0
        self.rec_path = None
        self.n_rows = 0                     # since connection / reset
        self.n_dt_bad = 0
        self.worst_dt = 0.0

        root = QVBoxLayout(self)
        self.banner = QLabel()
        self.banner.setWordWrap(True)
        root.addWidget(self.banner)

        top = QHBoxLayout()
        root.addLayout(top, 1)

        col = QVBoxLayout()
        top.addLayout(col)
        self.status = QLabel("NO DATA")
        f = QFont()
        f.setPointSize(26)
        f.setBold(True)
        self.status.setFont(f)
        self.status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status.setMinimumWidth(360)
        col.addWidget(self.status)

        self.table = QTableWidget(4, 4)
        self.table.setHorizontalHeaderLabels(["az deg", "el deg", "range mm", "notes"])
        self.table.setVerticalHeaderLabels(["1 (largest)", "2", "3", "4"])
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setFixedHeight(150)
        self.table.horizontalHeader().setStretchLastSection(True)
        col.addWidget(self.table)

        self.detail = QLabel()
        theme.mono(self.detail)
        col.addWidget(self.detail)

        g = QGroupBox("Link health (from the rows themselves)")
        gl = QVBoxLayout(g)
        self.health = QLabel()
        theme.mono(self.health)
        gl.addWidget(self.health)
        col.addWidget(g)

        rec = QHBoxLayout()
        self.rec_btn = QPushButton("Record")
        self.rec_btn.setCheckable(True)
        self.rec_btn.toggled.connect(self._record)
        rec.addWidget(self.rec_btn)
        self.rec_lbl = QLabel("rows are written verbatim to target_cam/data/")
        self.rec_lbl.setWordWrap(True)
        rec.addWidget(self.rec_lbl, 1)
        rst = QPushButton("Reset counts")
        rst.setToolTip("Zero the dt != period count, e.g. after changing the stage")
        rst.clicked.connect(self.reset_counts)
        rec.addWidget(rst)
        col.addLayout(rec)
        col.addStretch(1)

        # Field of view: where the blobs are, in degrees off the nose.
        self.fov = pg.PlotWidget(title="field of view (az right, el up), deg")
        self.fov.setAspectLocked(True)
        self.fov.showGrid(x=True, y=True, alpha=0.3)
        self.fov.setMouseEnabled(False, False)
        self.fov.setMinimumSize(380, 300)
        self.fov_rect = pg.PlotCurveItem(pen=pg.mkPen("w", width=2))
        self.fov.addItem(self.fov_rect)
        self.blobs = pg.ScatterPlotItem(pxMode=False)
        self.fov.addItem(self.blobs)
        self.blob_lbls = [pg.TextItem(str(k + 1), color="w", anchor=(0.5, 0.5))
                          for k in range(4)]
        for t in self.blob_lbls:
            self.fov.addItem(t)
        top.addWidget(self.fov, 1)

        plots = pg.GraphicsLayoutWidget()
        self.p_az = plots.addPlot(row=0, col=0, title="az deg (blob 1)")
        self.p_el = plots.addPlot(row=1, col=0, title="el deg (blob 1)")
        self.p_rg = plots.addPlot(row=2, col=0, title="range mm (blob 1; gaps = clipped / off)")
        for p in (self.p_az, self.p_el, self.p_rg):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setXRange(-HISTORY_S, 0, padding=0)
            p.setMouseEnabled(x=False, y=True)
        self.p_el.setXLink(self.p_az)
        self.p_rg.setXLink(self.p_az)
        self.p_rg.setLabel("bottom", "seconds (camera clock)")
        self.c_az = self.p_az.plot(pen=pg.mkPen(theme.TOKENS["series_az"], width=2), connect="finite")
        self.c_el = self.p_el.plot(pen=pg.mkPen(theme.TOKENS["series_el"], width=2), connect="finite")
        self.c_rg = self.p_rg.plot(pen=pg.mkPen(theme.TOKENS["series_range"], width=2), connect="finite")
        plots.setMinimumHeight(330)
        root.addWidget(plots, 1)

        self.raw = QLabel("last raw line: -")
        theme.mono(self.raw)
        self.raw.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.raw.setWordWrap(True)
        root.addWidget(self.raw)

    # ---- from the main window ------------------------------------------------

    def entered(self):
        """Put the link in its flight state: image and collection off, 115200."""
        if self.cfg.get("image") == 1:
            self.main.send("v0")
        if self.cfg.get("collect") == 1:
            self.main.send("!collect 0")
        link = self.main.link
        if link is not None and link.baud != P.BOOT_BAUD:
            self.main.send("B%d" % P.BOOT_BAUD)
        self.main.send("k")

    def on_cfg(self, v: dict):
        self.cfg = v

    def on_disconnected(self):
        """The link is gone: no #!CFG any more, and a recording ends here,
        so a file never spans two connections."""
        self.cfg = {}
        if self.rec:
            path, n = self.rec_path, self.rec_n
            self.rec_btn.setChecked(False)
            self.main._say("# [viewer] link closed: recording stopped, %s (%d rows)" % (path, n))

    def on_frame(self, _frame):
        self.last_frame_at = time.monotonic()

    def on_row(self, row: P.Row):
        now = time.monotonic()
        self.rows.append((now, row.v))
        while self.rows and now - self.rows[0][0] > HISTORY_S + 2:
            self.rows.popleft()
        self.last_row = row
        v = row.v
        if v["dt_ms"] > 0:                  # the first row after boot has none
            self.n_rows += 1
            if abs(v["dt_ms"] - self._period()) >= 0.05:
                self.n_dt_bad += 1
                self.worst_dt = max(self.worst_dt, v["dt_ms"])
        if self.rec:
            if self.rec_n == 0:
                # The header names the columns of the rows that actually arrive,
                # so a v1 camera's file still reads back correctly.
                self.rec.write(",".join(P.columns_for(int(v["csv_v"]))) + "\n")
            self.rec.write(row.raw + "\n")
            self.rec_n += 1

    def _period(self) -> float:
        p = self.cfg.get("period_ms")
        return float(p) if isinstance(p, (int, float)) and p > 0 else 80.0

    def reset_counts(self):
        self.n_rows = self.n_dt_bad = 0
        self.worst_dt = 0.0

    def tick(self):
        """10 Hz from the main window."""
        self._banner()
        if self.last_row is None:
            return
        v = self.last_row.v
        self.raw.setText("last raw line: " + self.last_row.raw)
        stale = time.monotonic() - self.rows[-1][0] > 1.0
        if stale:
            chip(self.status, "NO ROWS", GREY)
        elif v["ok"]:
            chip(self.status, "TARGET", GREEN)
        else:
            chip(self.status, "NO TARGET", RED)
        self._table(v)
        self._fov(v)
        self._plots()
        self._health()
        if self.rec:
            self.rec_lbl.setText("recording %s: %d rows" % (self.rec_path.name, self.rec_n))

    # ---- pieces ------------------------------------------------------------------

    def _banner(self):
        c = self.cfg
        why = []
        if not c:
            why.append("no #!CFG from the camera yet")
        else:
            if c.get("image") != 0:
                why.append("image sending is ON")
            if c.get("collect") == 1:
                why.append("blob collection is ON ('!collect')")
            if c.get("baud") != P.BOOT_BAUD:
                why.append("link at %s baud, not 115200" % c.get("baud"))
            if c.get("stream") != 1:
                why.append("CSV stream is OFF ('s')")
        if time.monotonic() - self.last_frame_at < 3.0:
            why.append("a frame arrived in the last 3 s")
        link = self.main.link
        if link is not None and link.pending():
            why.append("commands still in flight")
        if why:
            chip(self.banner, "NOT flight-identical: " + "; ".join(why), AMBER)
            return
        txt = ("FLIGHT-IDENTICAL: the camera reports image off, 115200 baud, stream on - "
               "these rows are exactly what the ESP32-S3 will receive.")
        if c.get("settings") == "modified":
            chip(self.banner, txt + "  BUT the settings are not saved: after a power "
                 "cycle the camera would fly different ones.", AMBER)
        else:
            chip(self.banner, txt + "  Settings: %s." % c.get("settings"), GREEN)

    def _table(self, v):
        rows = [(v["az_deg"], v["el_deg"], v["range_mm"],
                 "clipped (no range)" if v["clipped"] else "")]
        for k in (2, 3, 4):
            rows.append((v["b%d_az" % k], v["b%d_el" % k], v["b%d_range_mm" % k], ""))
        n = int(v["blobs"]) if v["ok"] else 0
        for i, (az, el, rg, note) in enumerate(rows):
            present = i < min(n, 4)
            cells = (["%.2f" % az, "%.2f" % el, "%.0f" % rg if rg >= 0 else "-", note]
                     if present else ["", "", "", ""])
            for j, t in enumerate(cells):
                self.table.setItem(i, j, QTableWidgetItem(t))
        self.detail.setText(
            "blob 1: area %4d px  box %3dx%-3d  fill %.2f  centroid (%.1f, %.1f)\n"
            "blobs in frame %d (4 reported at most)   matched px %d"
            % (v["area_px"], v["w_px"], v["h_px"], v["fill"], v["cx_px"], v["cy_px"],
               v["blobs"], v["matched_px"]))

    def _fov(self, v):
        hfov = float(self.cfg.get("fov", 54.0))
        f = (W / 2) / math.tan(math.radians(hfov) / 2)
        vfov = 2 * math.degrees(math.atan((H / 2) / f))
        a, e = hfov / 2, vfov / 2
        self.fov_rect.setData([-a, a, a, -a, -a], [-e, -e, e, e, -e])
        self.fov.setXRange(-a * 1.1, a * 1.1, padding=0)
        self.fov.setYRange(-e * 1.1, e * 1.1, padding=0)
        spots = []
        for t in self.blob_lbls:
            t.setVisible(False)
        if v["ok"]:
            n = int(min(v["blobs"], 4))
            mm = float(self.cfg.get("mm", 0) or 0)
            for k in range(1, n + 1):
                if k == 1:
                    az, el = v["az_deg"], v["el_deg"]
                    d_px = max(v["w_px"], v["h_px"])
                    size = 2 * math.degrees(math.atan(d_px / 2 / f))
                    clipped = bool(v["clipped"])
                else:
                    az, el, rg = v["b%d_az" % k], v["b%d_el" % k], v["b%d_range_mm" % k]
                    size = 2 * math.degrees(math.atan(mm / 2 / rg)) if mm > 0 and rg > 0 else 2.0
                    clipped = rg < 0 and mm > 0
                spots.append({"pos": (az, el), "size": max(size, 1.0),
                              "pen": pg.mkPen(RED if clipped else "w", width=3 if clipped else 1),
                              "brush": pg.mkBrush(255, 40, 40, 150 if k == 1 else 80)})
                self.blob_lbls[k - 1].setPos(az, el)
                self.blob_lbls[k - 1].setVisible(True)
        self.blobs.setData(spots)

    def _plots(self):
        if not self.rows:
            return
        t_now = self.rows[-1][1]["t_ms"]
        t = np.array([(r["t_ms"] - t_now) / 1000.0 for _, r in self.rows])
        ok = np.array([r["ok"] for _, r in self.rows]) > 0
        az = np.where(ok, [r["az_deg"] for _, r in self.rows], np.nan)
        el = np.where(ok, [r["el_deg"] for _, r in self.rows], np.nan)
        rg = np.array([r["range_mm"] for _, r in self.rows], dtype=float)
        rg[(~ok) | (rg < 0)] = np.nan
        keep = t >= -HISTORY_S
        self.c_az.setData(t[keep], az[keep])
        self.c_el.setData(t[keep], el[keep])
        self.c_rg.setData(t[keep], rg[keep])

    def _health(self):
        now = time.monotonic()
        recent = [r for ts, r in self.rows if now - ts <= 5.0]
        if len(recent) < 2:
            self.health.setText("waiting for rows")
            return
        span = (recent[-1]["t_ms"] - recent[0]["t_ms"]) / 1000.0
        rate = (len(recent) - 1) / span if span > 0 else 0.0
        dt = np.array([r["dt_ms"] for r in recent if r["dt_ms"] > 0])
        pu = np.array([r["proc_us"] for r in recent])
        okp = 100.0 * sum(1 for r in recent if r["ok"]) / len(recent)
        link = self.main.link
        bps = link.counters.rate() if link else 0.0
        c = self.cfg
        last = recent[-1]

        def p50max(key):
            a = np.array([r[key] for r in recent], dtype=float)
            a = a[np.isfinite(a)]
            return "p50 %6.0f max %6.0f" % (np.median(a), a.max()) if a.size else "  (v1 row: not reported)"

        bad = ("%d of %d rows, worst %.1f" % (self.n_dt_bad, self.n_rows, self.worst_dt)
               if self.n_dt_bad else "0 of %d rows" % self.n_rows)
        self.health.setText(
            "stage      %d running (requested %s; lut %s, clf %s, cnn %s)  csv v%d\n"
            "rows/s     %5.1f   (camera fps setting %s)\n"
            "dt_ms      mean %6.1f  jitter (sd) %5.1f  max %6.1f\n"
            "dt != %-4g %s\n"
            "proc_us    mean %6.0f  max %6.0f\n"
            "t_px_us    %s\n"
            "t_b_us     %s\n"
            "t_c_us     %s\n"
            "loop_us    %s   (budget %d us)\n"
            "ok         %5.1f %% of rows in the last 5 s\n"
            "wire       %6.0f bytes/s at %s baud (%.1f %% of capacity)"
            % (last["stage"], c.get("stage", "?"), c.get("lut", "?"), c.get("clf", "?"),
               c.get("cnn", "?"), last["csv_v"],
               rate, c.get("fps", "?"),
               dt.mean() if dt.size else 0, dt.std() if dt.size else 0,
               dt.max() if dt.size else 0, self._period(), bad,
               pu.mean(), pu.max(),
               p50max("t_px_us"), p50max("t_b_us"), p50max("t_c_us"), p50max("loop_us"),
               int(self._period() * 1000), okp, bps,
               link.baud if link else "?",
               100.0 * bps * 10 / link.baud if link and link.baud else 0))

    def _record(self, on):
        if on:
            DATA_DIR.mkdir(parents=True, exist_ok=True)
            self.rec_path = DATA_DIR / time.strftime("flight_%Y%m%d_%H%M%S.csv")
            self.rec = open(self.rec_path, "w")
            self.rec_n = 0                  # the header goes in with the first row
            self.rec_btn.setText("Stop recording")
        else:
            if self.rec:
                self.rec.close()
                self.rec_lbl.setText("saved %s (%d rows)" % (self.rec_path, self.rec_n))
            self.rec = None
            self.rec_btn.setText("Record")
