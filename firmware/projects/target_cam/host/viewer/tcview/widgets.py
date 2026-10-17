"""Small widgets shared by the two tabs."""
from __future__ import annotations

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QPainter, QPen
from PyQt6.QtWidgets import (QDoubleSpinBox, QHBoxLayout, QLabel, QSlider,
                             QSpinBox, QWidget)

from .ui import theme

# The four state colours the tabs pass to chip(). They are the theme's tone
# dots, so the same names still work where a tab draws with them directly
# (blob crop borders, the clipped-box pen on the Flight FOV plot).
GREEN = theme.TOKENS["ok_dot"]
AMBER = theme.TOKENS["warn_dot"]
RED = theme.TOKENS["err_dot"]
GREY = theme.TOKENS["text_faint"]
TONE = {GREEN: "ok", AMBER: "warn", RED: "err", GREY: "neutral"}


def chip(label: QLabel, text: str, colour: str):
    """A state chip in the theme's tone for `colour`. The stylesheet is set
    only when the tone changes: re-polishing on every row would be wasted
    work at the Flight tab's 10 Hz refresh."""
    label.setText(text)
    css = theme.chip_css(TONE.get(colour, "neutral"))
    if label.styleSheet() != css:
        label.setStyleSheet(css)


class ParamControl(QWidget):
    """One camera setting: slider + spin box + "is this what the camera uses"
    indicator.

    `local` is what the control shows; `camera` is what the last #!CFG said.
    Moving the slider changes `local` at once (the PC overlay follows it), and
    the value is committed - sent to the camera - on release, or 400 ms after a
    keyboard / spin-box change. The indicator turns green only when the
    camera's own #!CFG reports the same value.
    """
    changed = pyqtSignal(object)        # local value, live
    commit = pyqtSignal(object)         # send this to the camera

    def __init__(self, name: str, lo, hi, step=1, decimals=0, slider=True, unit=""):
        super().__init__()
        self.decimals = decimals
        self.scale = 10 ** decimals
        self.camera = None
        self.in_flight = False
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.label = QLabel(name)
        self.label.setMinimumWidth(78)
        lay.addWidget(self.label)
        self.slider = None
        if slider:
            self.slider = QSlider(Qt.Orientation.Horizontal)
            self.slider.setRange(int(lo * self.scale), int(hi * self.scale))
            self.slider.setSingleStep(int(step * self.scale))
            lay.addWidget(self.slider, 1)
        if decimals:
            self.spin = QDoubleSpinBox()
            self.spin.setDecimals(decimals)
            self.spin.setSingleStep(step)
        else:
            self.spin = QSpinBox()
        self.spin.setRange(lo, hi)
        self.spin.setSuffix(unit)
        self.spin.setKeyboardTracking(False)
        self.spin.setMinimumWidth(80)
        lay.addWidget(self.spin)
        self.status = QLabel()
        self.status.setMinimumWidth(70)
        lay.addWidget(self.status)
        self._debounce = QTimer(self, singleShot=True, interval=400)
        self._debounce.timeout.connect(self._commit)
        self._syncing = False
        if self.slider:
            self.slider.valueChanged.connect(self._slider_moved)
            self.slider.sliderReleased.connect(self._commit)
        self.spin.valueChanged.connect(self._spin_changed)
        self._refresh()

    @property
    def local(self):
        v = self.spin.value()
        return round(v, self.decimals) if self.decimals else int(v)

    def dragging(self) -> bool:
        return bool(self.slider and self.slider.isSliderDown()) or self._debounce.isActive()

    def _set_widgets(self, v):
        self._syncing = True
        self.spin.setValue(v)
        if self.slider:
            self.slider.setValue(int(round(v * self.scale)))
        self._syncing = False

    def _slider_moved(self, raw):
        if self._syncing:
            return
        v = raw / self.scale if self.decimals else raw
        self._syncing = True
        self.spin.setValue(v)
        self._syncing = False
        self.changed.emit(self.local)
        self._refresh()
        if not self.slider.isSliderDown():
            self._debounce.start()

    def _spin_changed(self, v):
        if self._syncing:
            return
        if self.slider:
            self._syncing = True
            self.slider.setValue(int(round(v * self.scale)))
            self._syncing = False
        self.changed.emit(self.local)
        self._refresh()
        self._debounce.start()

    def _commit(self):
        self._debounce.stop()
        if self.camera is None or self.local != self.camera:
            self.in_flight = True
            self.commit.emit(self.local)
        self._refresh()

    def set_local(self, v, send=False):
        self._set_widgets(v)
        self.changed.emit(self.local)
        if send:
            self._commit()
        self._refresh()

    def set_camera(self, v):
        """From #!CFG. Adopted as the shown value unless the user is mid-change."""
        self.camera = v
        if not self.dragging() and not self.in_flight and v != self.local:
            self._set_widgets(v)
            self.changed.emit(self.local)
        self._refresh()

    def done(self, ok: bool):
        self.in_flight = False
        if not ok and self.camera is not None and not self.dragging():
            self._set_widgets(self.camera)
            self.changed.emit(self.local)
        self._refresh()

    def _refresh(self):
        if self.camera is None:
            chip(self.status, "no camera", GREY)
        elif self.in_flight:
            chip(self.status, "sending", AMBER)
        elif self.local == self.camera:
            chip(self.status, "applied", GREEN)
        else:
            chip(self.status, "not sent", AMBER)


class HueBar(QWidget):
    """The hue circle unrolled, with the accepted window [hue-tol, hue+tol]
    left bright and everything else dimmed; an optional tick marks a sampled
    hue. Red sits at both ends, so a red window shows as two bright pieces."""

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(18)
        self.hue, self.tol, self.mark = 0, 12, None

    def set_window(self, hue, tol):
        self.hue, self.tol = hue, tol
        self.update()

    def set_mark(self, hue):
        self.mark = hue
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        w, h = self.width(), self.height()
        for x in range(w):
            hue = int(x * 360 / w) % 360
            d = abs(hue - self.hue) % 360
            inside = min(d, 360 - d) <= self.tol
            c = QColor.fromHsv(hue, 255, 255 if inside else 70)
            p.setPen(c)
            p.drawLine(x, 0, x, h)
        if self.mark is not None:
            x = int(self.mark * w / 360)
            p.setPen(QPen(Qt.GlobalColor.white, 2))
            p.drawLine(x, 0, x, h)
            p.setPen(QPen(Qt.GlobalColor.black, 1))
            p.drawLine(x - 2, 0, x - 2, h)
            p.drawLine(x + 2, 0, x + 2, h)
        p.end()
