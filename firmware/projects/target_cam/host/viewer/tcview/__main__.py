"""
target_cam viewer.

  python -m tcview --port /dev/ttyACM0     the real camera (ESP32-CAM-MB programmer)
  python -m tcview --fake                  a synthetic camera on a pseudo-terminal

Two screens: "Image & calibration" (see the real pixels, tune the detector by
hand, save to the camera's flash) and "Flight data" (only the CSV the ESP32-S3
will receive). The console at the bottom shows everything else the camera
says, and takes raw commands.
"""
from __future__ import annotations

import argparse
import sys
import time

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (QApplication, QHBoxLayout, QLabel, QLineEdit,
                             QMainWindow, QPlainTextEdit, QPushButton,
                             QSplitter, QTabWidget, QVBoxLayout, QWidget)
from PyQt6.QtCore import Qt

from . import detector as D
from . import protocol as P
from .calib_tab import CalibTab
from .flight_tab import FlightTab
from .link import SerialLink


class MainWindow(QMainWindow):
    def __init__(self, port: str, title_note: str = ""):
        super().__init__()
        self.setWindowTitle("target_cam viewer - %s%s" % (port, title_note))
        self.port = port
        self.link: SerialLink | None = None

        self.tabs = QTabWidget()
        self.calib = CalibTab(self)
        self.flight = FlightTab(self)
        self.tabs.addTab(self.calib, "Image && calibration")
        self.tabs.addTab(self.flight, "Flight data (what the ESP32-S3 gets)")
        self.tabs.currentChanged.connect(self._tab_changed)

        console = QWidget()
        cl = QVBoxLayout(console)
        cl.setContentsMargins(0, 0, 0, 0)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(3000)
        self.log.setStyleSheet("font-family: monospace;")
        cl.addWidget(self.log)
        ent = QHBoxLayout()
        ent.addWidget(QLabel("Command:"))
        self.entry = QLineEdit()
        self.entry.setPlaceholderText("any camera command, e.g. ?  m  H350  p red  !save  "
                                      "(the console shows the camera's # replies)")
        self.entry.returnPressed.connect(self._entry)
        ent.addWidget(self.entry, 1)
        self.show_rows = QPushButton("Show CSV rows")
        self.show_rows.setCheckable(True)
        ent.addWidget(self.show_rows)
        cl.addLayout(ent)

        split = QSplitter(Qt.Orientation.Vertical)
        split.addWidget(self.tabs)
        split.addWidget(console)
        split.setStretchFactor(0, 5)
        split.setStretchFactor(1, 1)
        self.setCentralWidget(split)

        self.state_lbl = QLabel()
        self.stats_lbl = QLabel()
        self.statusBar().addWidget(self.state_lbl, 1)
        self.statusBar().addPermanentWidget(self.stats_lbl)

        self._t1 = QTimer(self, interval=1000)
        self._t1.timeout.connect(self._tick1)
        self._t1.start()
        self._t10 = QTimer(self, interval=100)
        self._t10.timeout.connect(self.flight.tick)
        self._t10.start()
        self._last_rows = (time.monotonic(), 0)

    # ---- link ----------------------------------------------------------------

    def connect_link(self):
        self.link = SerialLink(self.port)
        self.link.event.connect(self._event)
        self.link.command_done.connect(self._done)
        self.link.state.connect(self._state)
        self.link.baud_changed.connect(lambda b: self._say("# [viewer] port now at %d baud" % b))
        self.link.start()
        # Ask for the config at once: the camera may have booted long ago.
        self.send("k")

    def send(self, cmd: str):
        if self.link is None:
            return
        self._say("> " + cmd)
        self.link.send(cmd)

    def _say(self, text):
        self.log.appendPlainText(text)

    def _state(self, text):
        self.state_lbl.setText(text)
        self._say("# [viewer] " + text)

    def _done(self, cmd, ok, reason):
        if not ok:
            self._say("# [viewer] command %r failed: %s" % (cmd, reason))
        self.calib.on_command_done(cmd, ok, reason)

    def _event(self, ev):
        if isinstance(ev, P.Row):
            self.calib.on_row(ev)
            self.flight.on_row(ev)
            if self.show_rows.isChecked():
                self._say(ev.raw)
        elif isinstance(ev, P.Frame):
            self.calib.on_frame(ev)
            self.flight.on_frame(ev)
        elif isinstance(ev, P.Cfg):
            self.calib.on_cfg(ev.v)
            self.flight.on_cfg(ev.v)
        elif isinstance(ev, P.Cal):
            self.calib.on_cal(ev)
        elif isinstance(ev, P.Comment):
            self._say(ev.text)
        elif isinstance(ev, P.Nak):
            self._say("#!NAK %s -- %s" % (ev.cmd, ev.reason))
        elif isinstance(ev, P.FrameError):
            self._say("# [viewer] frame %s dropped: %s" % (ev.seq, ev.reason))
        elif isinstance(ev, P.Baud):
            self._say("#!BAUD %d" % ev.baud)

    def _entry(self):
        cmd = self.entry.text().strip()
        self.entry.clear()
        if cmd:
            self.send(cmd)

    def _tab_changed(self, i):
        if self.tabs.widget(i) is self.flight:
            self.flight.entered()

    def _tick1(self):
        self.calib.tick()
        if self.link is None:
            return
        c = self.link.counters
        t0, n0 = self._last_rows
        now = time.monotonic()
        rps = (c.rows - n0) / (now - t0) if now > t0 else 0
        self._last_rows = (now, c.rows)
        self.stats_lbl.setText(
            "%d baud | %.1f kB/s | rows %.1f/s | frames ok %d bad %d | junk lines %d | "
            "NAK %d retries %d timeouts %d" % (
                self.link.baud, c.rate() / 1000, rps, c.frames_ok, c.frames_bad,
                c.junk_lines, c.naks, c.retries, c.timeouts))

    def closeEvent(self, e):
        if self.link is not None:
            self.link.stop()
        if self.flight.rec:
            self.flight.rec_btn.setChecked(False)
        super().closeEvent(e)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m tcview", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port of the camera, e.g. /dev/ttyACM0")
    ap.add_argument("--fake", action="store_true", help="use a synthetic camera instead")
    ap.add_argument("--fake-corrupt", type=int, default=0, metavar="N",
                    help="with --fake: inject a log line into every Nth frame")
    ap.add_argument("--quit-after", type=float, metavar="S",
                    help="close after S seconds (smoke tests)")
    args = ap.parse_args(argv)
    if not args.port and not args.fake:
        ap.error("give --port PORT or --fake")

    app = QApplication(sys.argv[:1])
    D.hsv_lut()             # ~0.3 s once, so the first hover does not stall
    fake = None
    port = args.port
    note = ""
    if args.fake:
        from .fake_cam import FakeCamera
        fake = FakeCamera(corrupt_every=args.fake_corrupt).start()
        port, note = fake.port, " (FAKE camera)"
    win = MainWindow(port, note)
    win.resize(1400, 1000)
    win.show()
    win.connect_link()
    if args.quit_after:
        QTimer.singleShot(int(args.quit_after * 1000), win.close)
    rc = app.exec()
    if fake:
        fake.stop()
    return rc


if __name__ == "__main__":
    sys.exit(main())
