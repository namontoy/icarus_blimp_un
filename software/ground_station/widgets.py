"""The custom-painted pieces of the ground station's window.

Each one redraws what the page drew with SVG, a canvas or a clip-path, from
the same coordinates: the top view, compass and range gauge keep their SVG
viewBox and are scaled into the widget, so the numbers below can be checked
against the old web/index.html in git history. Colours and fonts come from
theme.py and nothing else.
"""
import math
import time

from PyQt6.QtCore import QEvent, QPoint, QPointF, QRect, QRectF, QSize, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QFontMetrics, QPainter, QPainterPath, QPen, QPolygonF
from PyQt6.QtWidgets import (QAbstractButton, QHBoxLayout, QLabel, QLayout, QPushButton, QSizePolicy, QSlider,
                             QSplitter, QSplitterHandle, QToolTip, QWidget)

from . import theme as T

HISTORY_S = 10.0            # the sensor plots' window
HOVER_BAND_M = (1.85, 2.25)
GAUGE_MAX_M = 3.0
FOV_DEG = 54.0              # target_cam's horizontal field of view


def C(c, alpha=None):
    q = QColor(c)
    if alpha is not None:
        q.setAlphaF(alpha)
    return q


def isnum(v):
    return isinstance(v, (int, float)) and not (isinstance(v, float) and math.isnan(v))


def fmt(v, d=2):
    return ('%.*f' % (d, v)) if isnum(v) else '—'


def sfmt(v, d=2):
    return ('%+.*f' % (d, v)) if isnum(v) else '—'


def clamp(v, a, b):
    return max(a, min(b, v))


def label(text='', px=14, weight=400, color=T.FG, mono=False, spacing=0.0, caps=False, wrap=False):
    """A QLabel in one of the page's text styles."""
    lb = QLabel(text)
    lb.setFont(T.font(px, weight, mono, spacing, caps))
    set_color(lb, color)
    lb.setWordWrap(wrap)
    return lb


def set_color(lb, color):
    lb.setStyleSheet('color: %s; background: transparent;' % color)


def hline_polygon(*pts):
    return QPolygonF([QPointF(x, y) for x, y in pts])


# --------------------------------------------------------------- layouts

class FlowBox(QWidget):
    """Rows that wrap, as CSS flex-wrap did for the button rows and chips.

    A widget that places its own children rather than a QLayout subclass, so
    no QLayoutItem is owned half by Python and half by C++ (Qt deletes the item
    a Python takeAt() returns while the Python list may still hold it)."""

    def __init__(self, hgap=8, vgap=8):
        super().__init__()
        self._kids = []
        self._h, self._v = hgap, vgap
        sp = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sp.setHeightForWidth(True)
        self.setSizePolicy(sp)

    def add(self, w):
        w.setParent(self)
        w.installEventFilter(self)      # a child's hint or visibility changing re-flows the rows
        self._kids.append(w)
        self.updateGeometry()

    def eventFilter(self, obj, e):
        if e.type() in (QEvent.Type.LayoutRequest, QEvent.Type.Show, QEvent.Type.Hide):
            self.updateGeometry()
            self._place(self.width(), dry=False)
        return False

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, w):
        return self._place(w, dry=True)

    def sizeHint(self):
        return self.minimumSizeHint()

    def minimumSizeHint(self):
        s = QSize()
        for w in self._kids:
            if not w.isHidden():
                s = s.expandedTo(w.sizeHint())
        return s

    def resizeEvent(self, e):
        self._place(self.width(), dry=False)

    def _place(self, width, dry):
        x, y, line_h = 0, 0, 0
        rows = []                               # widgets per row, to centre each vertically
        row = []
        for w in self._kids:
            if w.isHidden():
                continue
            hint = w.sizeHint()
            if row and x + hint.width() > width:
                rows.append((row, y, line_h))
                x, y, line_h, row = 0, y + line_h + self._v, 0, []
            row.append((w, x, hint))
            x += hint.width() + self._h
            line_h = max(line_h, hint.height())
        if row:
            rows.append((row, y, line_h))
        if not dry:
            for items, ry, rh in rows:
                for w, ix, hint in items:
                    w.setGeometry(QRect(ix, ry + (rh - hint.height()) // 2, hint.width(), hint.height()))
        return (y + line_h) if rows else 0


def hbox(*items, spacing=8, margins=(0, 0, 0, 0)):
    lay = QHBoxLayout()
    lay.setSpacing(spacing)
    lay.setContentsMargins(*margins)
    for it in items:
        if it is None:
            lay.addStretch(1)
        elif isinstance(it, int):
            lay.addSpacing(it)
        elif isinstance(it, QLayout):
            lay.addLayout(it)
        else:
            lay.addWidget(it)
    return lay


# ---------------------------------------------------------------- frames

class ChamferFrame(QWidget):
    """The panel shape: the 14 px cut top-right corner and the 2 px green line
    along the top, as the page's clip-path and ::before drew them."""

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h, c = self.width(), self.height(), T.CHAMFER
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(C(T.PANEL))
        p.drawPolygon(hline_polygon((0, 0), (w - c, 0), (w, c), (w, h), (0, h)))
        p.setBrush(C(T.UN_GREEN))
        p.drawPolygon(hline_polygon((0, 0), (w - c, 0), (w - c + 2, 2), (0, 2)))


class Pill(QWidget):
    """A header status pill: a state-coloured left border, a cut corner, an
    upper-case name and a mono value."""
    BORDER = {'ok': T.UN_GREEN, 'warn': T.AMBER, 'crit': T.UN_RED, 'live': T.PAPAYA, 'demo': T.PAPAYA,
              'idle': T.FAINT}

    def __init__(self, name, tip='', state='idle', value='—'):
        super().__init__()
        self.name, self.state, self.value = name, state, value
        self.setToolTip(tip)
        self._fn = T.font(11, 700, spacing=0.1, caps=True)
        self._fv = T.font(11, 600, mono=True)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

    def set(self, state, value):
        if (state, value) != (self.state, self.value):
            changed_w = len(str(value)) != len(str(self.value))
            self.state, self.value = state, value
            if changed_w:
                self.updateGeometry()
            self.update()

    def sizeHint(self):
        w = QFontMetrics(self._fn).horizontalAdvance(self.name)
        if self.value:
            w += 4 + QFontMetrics(self._fv).horizontalAdvance(str(self.value))
        return QSize(3 + 10 + w + 16, 26)

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        crit, demo = self.state == 'crit', self.state == 'demo'
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(C(T.PAPAYA if demo else T.CRIT_BG if crit else T.RAISED))
        p.drawPolygon(hline_polygon((0, 0), (w - 8, 0), (w, h), (0, h)))
        p.setBrush(C(self.BORDER.get(self.state, T.FAINT)))
        p.drawRect(QRectF(0, 0, 3, h))
        x = 13
        p.setFont(self._fn)
        p.setPen(C(T.PAPAYA_INK if demo else T.FG if crit else T.MUTED))
        r = QRectF(x, 0, w - x, h)
        p.drawText(r, Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, self.name)
        if self.value:
            x += QFontMetrics(self._fn).horizontalAdvance(self.name) + 4
            p.setFont(self._fv)
            p.setPen(C(T.PAPAYA_INK if demo else T.FG))
            p.drawText(QRectF(x, 0, w - x, h), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
                       str(self.value))


class Wordmark(QWidget):
    """ICARUS, 900 weight, with the I in the university's green."""

    def __init__(self):
        super().__init__()
        self._f = T.font(30, 900, spacing=0.16)
        fm = QFontMetrics(self._f)
        self.setFixedSize(fm.horizontalAdvance('ICARUS'), fm.ascent() - 2)

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        p.setFont(self._f)
        fm = QFontMetrics(self._f)
        y = self.height()
        p.setPen(C(T.UN_GREEN))
        p.drawText(0, y, 'I')
        p.setPen(C(T.FG))
        p.drawText(fm.horizontalAdvance('I'), y, 'CARUS')


class ElidedLabel(QLabel):
    """One line that ends in … when squeezed, instead of being cut mid-word
    at the widget's edge. The full text stays in the tool tip."""

    def __init__(self, text=''):
        super().__init__()
        self._full = text
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        self.setText(text)

    def setText(self, text):
        self._full = text
        self.setToolTip(text)
        self._elide()

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._elide()

    def _elide(self):
        fm = QFontMetrics(self.font())
        super().setText(fm.elidedText(self._full, Qt.TextElideMode.ElideRight, max(0, self.width())))


class FullScreenButton(QPushButton):
    """The header's full-screen toggle. The page used ⛶, which neither bundled
    font has, so the four corner brackets are drawn over the sheet's button."""

    def __init__(self):
        super().__init__()
        self.setProperty('kind', 'full')
        self.setFixedSize(34, 30)

    def paintEvent(self, e):
        super().paintEvent(e)
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setPen(QPen(C(T.FG if self.underMouse() else T.MUTED), 1.6))
        c = QPointF(self.width() / 2, self.height() / 2)
        r, k = 7.0, 4.0                          # half-size of the square, bracket arm
        for sx in (-1, 1):
            for sy in (-1, 1):
                corner = QPointF(c.x() + sx * r, c.y() + sy * r)
                p.drawLine(corner, QPointF(corner.x() - sx * k, corner.y()))
                p.drawLine(corner, QPointF(corner.x(), corner.y() - sy * k))
        p.end()


class StopButton(QAbstractButton):
    """The papaya STOP: chamfered, black weight; it flashes white on press."""

    def __init__(self):
        super().__init__()
        self.setText('STOP')
        self._f = T.font(22, 900, spacing=0.18)
        self._flash = False
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFocusPolicy(Qt.FocusPolicy.TabFocus)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

    def sizeHint(self):
        fm = QFontMetrics(self._f)
        return QSize(24 + fm.horizontalAdvance('STOP') + 30, fm.height() + 20)

    def flash(self):
        from PyQt6.QtCore import QTimer
        self._flash = True
        self.update()
        QTimer.singleShot(180, self._unflash)

    def _unflash(self):
        self._flash = False
        self.update()

    def enterEvent(self, e):
        self.update()

    def leaveEvent(self, e):
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        down = self.isDown() or self._flash
        if down:                                # transform: scale(0.97) about the centre
            p.translate(w / 2, h / 2)
            p.scale(0.97, 0.97)
            p.translate(-w / 2, -h / 2)
        bg = C(T.FG) if down else C(T.PAPAYA).lighter(108) if self.underMouse() else C(T.PAPAYA)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(bg)
        p.drawPolygon(hline_polygon((0, 0), (w - 12, 0), (w, h), (0, h)))
        if self.hasFocus():
            p.setPen(QPen(C(T.FG), 2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRect(QRectF(2, 2, w - 16, h - 4))
        p.setFont(self._f)
        p.setPen(C(T.PAPAYA_INK))
        p.drawText(QRectF(24, 0, w - 54, h), Qt.AlignmentFlag.AlignCenter, 'STOP')


class Switch(QAbstractButton):
    """The page's toggle: a square track with a sliding square knob, then the label."""

    def __init__(self, text, small=False, checked=False):
        super().__init__()
        self.setCheckable(True)
        self.setChecked(checked)
        self.setText(text)
        self._small = small
        self._f = T.font(14, 400 if small else 600)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFocusPolicy(Qt.FocusPolicy.TabFocus)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

    def sizeHint(self):
        fm = QFontMetrics(self._f)
        return QSize(38 + 8 + fm.horizontalAdvance(self.text()) + 2, max(22, fm.height()))

    def paintEvent(self, e):
        p = QPainter(self)
        on = self.isChecked()
        y = (self.height() - 20) // 2
        p.setPen(QPen(C(T.UN_GREEN if on else T.BORDER), 1))
        p.setBrush(C(T.UN_GREEN_DEEP if on else T.RAISED_2))
        p.drawRect(QRectF(0.5, y + 0.5, 37, 19))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(C(T.FG if on else T.FAINT))
        p.drawRect(QRectF(20 if on else 3, y + 3, 14, 14))
        if self.hasFocus():
            p.setPen(QPen(C(T.PAPAYA), 2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRect(QRectF(1, y + 1, 36, 18))
        p.setFont(self._f)
        p.setPen(C(T.MUTED if self._small else T.FG))
        if not self.isEnabled():
            p.setOpacity(0.45)
        p.drawText(QRectF(46, 0, self.width() - 46, self.height()),
                   Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, self.text())


class ThrustSlider(QSlider):
    """−100…100 percent of the limit; a double click puts it back at zero."""

    def __init__(self):
        super().__init__(Qt.Orientation.Horizontal)
        self.setRange(-100, 100)
        self.setSingleStep(1)
        self.setPageStep(10)
        self.setValue(0)

    def mouseDoubleClickEvent(self, e):
        if self.isEnabled():
            self.setValue(0)


class SegBar(QWidget):
    """Twenty segments of readback: green forward, blue reversed."""

    def __init__(self):
        super().__init__()
        self.v = float('nan')
        self.setFixedHeight(6)

    def set(self, v):
        if not (v == self.v or (not isnum(v) and not isnum(self.v))):
            self.v = v
            self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        n, gap = 20, 2
        seg = (self.width() - gap * (n - 1)) / n
        on = round(abs(self.v) * 20) if isnum(self.v) else 0
        lit = C(T.S_Y if isnum(self.v) and self.v < 0 else T.UN_GREEN)
        p.setPen(Qt.PenStyle.NoPen)
        for i in range(n):
            p.setBrush(lit if i < on else C(T.RAISED_2))
            p.drawRect(QRectF(i * (seg + gap), 0, seg, self.height()))


class _ViewBox(QWidget):
    """A widget drawn in a fixed SVG viewBox, scaled to fit and centred, with
    a maximum width as the page's max-width did."""
    VB = (100, 100)

    def __init__(self, max_w):
        super().__init__()
        self.set_max_width(max_w)

    def set_max_width(self, max_w):
        self._max_w = max_w
        self.setFixedHeight(round(max_w * self.VB[1] / self.VB[0]))
        self.setMinimumWidth(max_w // 2)
        self.update()

    def begin(self):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        k = min(self.width(), self._max_w) / self.VB[0]
        k = min(k, self.height() / self.VB[1])
        p.translate((self.width() - self.VB[0] * k) / 2, (self.height() - self.VB[1] * k) / 2)
        p.scale(k, k)
        return p

    @staticmethod
    def text(p, x, y, s, px, color, weight=400, mono=False, spacing=0.0, align='middle'):
        """SVG text: (x, y) is the baseline at the anchor. Fonts are sized in
        viewBox units, so they scale with the drawing as SVG text did."""
        f = T.font(10, weight, mono, spacing)
        f.setPointSizeF(px)        # the painter's scale turns these units into pixels
        p.setFont(f)
        p.setPen(C(color))
        w = QFontMetrics(f).horizontalAdvance(s)
        dx = {'start': 0, 'middle': -w / 2, 'end': -w}[align]
        p.drawText(QPointF(x + dx, y), s)


class TopView(_ViewBox):
    """The blimp from above, to scale: the "Dónde van los motores" layout
    (2026-10-10) with the bar shortened to 24 cm between the thrusters. One
    transverse bar under the two DRV8833s at the back of the 14 × 6 cm
    gondola carries all four motors: lift M1 (RL) and M2 (RR) at ±5.5 cm,
    just outside the gondola, and thrust M4 (FL) and M3 (FR) at ±12 cm. The
    camera and magnetometer stay alone at the front, away from the motors.
    The envelope is the 66 cm rounded cube from the design basis, cropped
    to its sides so the gondola can be drawn large enough to read.

    The F/R names are the firmware's (motor_id_t), kept from the corner
    layout; on this bar F means thrust and R means lift. Thrust arrows scale
    with readback, forward from the propeller or backward from the motor
    (blue when reversed); the lift rings grow with readback."""
    VB = (320, 192)
    CX, CY, K = 160, 76, 4.6                   # origin at the gondola centre, 4.6 units per cm
    ENVELOPE_CM, CORNER = 66.0, 0.35            # width, corner radius as a fraction of it
    GONDOLA_CM = (6.0, 14.0)
    BAR_Y_CM = -5.0                             # behind the gondola centre
    PROP_R_CM = 2.0                             # 40 mm propellers
    POS = {'FL': -12.0, 'FR': 12.0, 'RL': -5.5, 'RR': 5.5}      # x along the bar, cm
    MNUM = {'FL': 'M4', 'FR': 'M3', 'RL': 'M1', 'RR': 'M2'}
    ARROW_MAX = 40.0                            # arrow length at full readback, units
    RING_GROW = 12.0                            # lift ring growth at full readback, units

    def __init__(self):
        super().__init__(340)
        self.v = {m: float('nan') for m in self.POS}

    def set(self, values):
        self.v = dict(values)
        self.update()

    def pt(self, x_cm, y_cm):
        """Body coordinates (cm, x right, y forward) to viewBox units."""
        return QPointF(self.CX + x_cm * self.K, self.CY - y_cm * self.K)

    def rect(self, x0, y0, x1, y1):
        """A rectangle from two body-coordinate corners."""
        return QRectF(self.pt(x0, y1), self.pt(x1, y0))

    def paintEvent(self, e):
        p = self.begin()
        # SVG user units: px values from the CSS are in these units too
        dpi_fix = 72.0 / self.logicalDpiY()
        k, h = self.K, self.ENVELOPE_CM / 2

        p.setClipRect(QRectF(0, 0, *self.VB))         # the envelope runs past the top and bottom
        p.setPen(QPen(C(T.BORDER), 1.5))
        p.setBrush(C(T.RAISED))
        p.drawRoundedRect(self.rect(-h, -h, h, h), self.CORNER * 2 * h * k, self.CORNER * 2 * h * k)
        self.text(p, self.CX, 15, 'FRONT', 9 * dpi_fix, T.FAINT, spacing=0.12)
        self.text(p, self.CX, 31, 'envelope ≈ 66 cm wide', 8.5 * dpi_fix, T.FAINT)
        self.text(p, self.CX, 185, 'outer pair: surge and yaw · inner pair: lift', 9 * dpi_fix, T.FAINT,
                  spacing=0.12)

        # The bar first: it runs under the board.
        by = self.BAR_Y_CM
        p.setPen(QPen(C(T.MUTED), 2.5, cap=Qt.PenCapStyle.RoundCap))
        p.drawLine(self.pt(self.POS['FL'], by), self.pt(self.POS['FR'], by))
        gw, gl = self.GONDOLA_CM[0] / 2, self.GONDOLA_CM[1] / 2
        p.setPen(QPen(C(T.FAINT), 1))
        p.setBrush(C(T.RAISED_2))
        p.drawRoundedRect(self.rect(-gw, -gl, gw, gl), 3, 3)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(C(T.FAINT))
        for x0 in (-1.8, 0.2):                                   # the two DRV8833s
            p.drawRect(self.rect(x0, by - 0.7, x0 + 1.6, by + 0.7))
        p.setPen(QPen(C(T.MUTED), 0.8))
        p.setBrush(C(T.BG))
        p.drawRect(self.rect(-0.9, gl - 0.4, 0.9, gl + 0.3))      # camera at the front edge
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(C(T.AMBER))
        c = self.pt(0, gl - 1.0)                                   # magnetometer
        p.drawPolygon(hline_polygon((c.x(), c.y() - 3), (c.x() + 3, c.y()), (c.x(), c.y() + 3),
                                    (c.x() - 3, c.y())))

        # Dimension: thruster to thruster.
        dy = self.pt(0, by).y() + 56
        x0, x1 = self.pt(self.POS['FL'], 0).x(), self.pt(self.POS['FR'], 0).x()
        p.setPen(QPen(C(T.FAINT), 1))
        p.drawLine(QPointF(x0, dy), QPointF(x1, dy))
        for x in (x0, x1):
            p.drawLine(QPointF(x, dy - 4), QPointF(x, dy + 4))
        self.text(p, self.CX, dy + 13, '%.0f cm' % (self.POS['FR'] - self.POS['FL']), 9 * dpi_fix,
                  T.FAINT, mono=True)

        pr = self.PROP_R_CM * k
        for m, x in self.POS.items():
            o = self.pt(x, by)
            v = self.v.get(m)
            col = C(T.S_Y if isnum(v) and v < 0 else T.UN_GREEN)
            if m[0] == 'F':
                # motor body along the bar, propeller seen edge-on in front of it
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(C(T.MUTED))
                p.drawRoundedRect(self.rect(x - 0.45, by - 1.0, x + 0.45, by + 0.8), 1.5, 1.5)
                prop_y = self.pt(0, by + 1.2).y()
                p.setPen(QPen(C(T.MUTED), 2.5, cap=Qt.PenCapStyle.RoundCap))
                p.drawLine(QPointF(o.x() - pr, prop_y), QPointF(o.x() + pr, prop_y))
                ln = abs(v) * self.ARROW_MAX if isnum(v) else 0
                if ln >= 2:
                    d = 1 if v < 0 else -1                       # +1 is backward (down)
                    y1 = self.pt(0, by - 1.0).y() if v < 0 else prop_y - 1.5
                    tip = y1 + d * ln
                    head = min(12.0, ln)
                    p.setPen(QPen(col, 3, cap=Qt.PenCapStyle.FlatCap))
                    p.drawLine(QPointF(o.x(), y1), QPointF(o.x(), tip - d * (head - 1)))
                    p.setPen(Qt.PenStyle.NoPen)
                    p.setBrush(col)
                    p.drawPolygon(hline_polygon((o.x(), tip), (o.x() + 6, tip - d * head),
                                                (o.x() - 6, tip - d * head)))
                lx = o.x() + (-1 if x < 0 else 1) * (pr + 5)
                align = 'end' if x < 0 else 'start'
                self.text(p, lx, o.y() + 3, m, 11 * dpi_fix, T.MUTED, weight=700, align=align)
                self.text(p, lx, o.y() + 13, self.MNUM[m], 8.5 * dpi_fix, T.FAINT, align=align)
            else:
                # propeller disc seen from above, the hub at its centre
                p.setPen(QPen(C(T.MUTED), 1, Qt.PenStyle.DashLine))
                p.setBrush(C(T.RAISED_2, 0.6))
                p.drawEllipse(o, pr, pr)
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(C(T.MUTED))
                p.drawEllipse(o, 2, 2)
                grow = abs(v) * self.RING_GROW if isnum(v) else 0
                if grow >= 1:
                    p.setOpacity(0.9)
                    p.setPen(QPen(col, 3))
                    p.setBrush(Qt.BrushStyle.NoBrush)
                    p.drawEllipse(o, pr + grow, pr + grow)
                    p.setOpacity(1)
                self.text(p, o.x(), o.y() + pr + self.RING_GROW + 12, m, 11 * dpi_fix, T.MUTED, weight=700)
                self.text(p, o.x(), o.y() + pr + self.RING_GROW + 22, self.MNUM[m], 8.5 * dpi_fix, T.FAINT)


class Compass(_ViewBox):
    """A fixed dial with the needle as the blimp's nose: rotate(heading)."""
    VB = (120, 120)

    def __init__(self):
        super().__init__(140)
        self.heading = None

    def set(self, heading):
        self.heading = heading
        self.update()

    def paintEvent(self, e):
        p = self.begin()
        u = 72.0 / self.logicalDpiY()
        p.setPen(QPen(C(T.BORDER), 1.5))
        p.setBrush(C(T.BG))
        p.drawEllipse(QPointF(60, 60), 54, 54)
        p.setPen(QPen(C(T.FAINT), 1))
        for a in range(0, 360, 30):
            r1, rad = (46 if a % 90 == 0 else 50), math.radians(a)
            p.drawLine(QPointF(60 + r1 * math.sin(rad), 60 - r1 * math.cos(rad)),
                       QPointF(60 + 54 * math.sin(rad), 60 - 54 * math.cos(rad)))
        for s, x, y in (('N', 60, 16), ('E', 108, 64), ('S', 60, 110), ('W', 12, 64)):
            self.text(p, x, y, s, 9 * u, T.MUTED, weight=700)
        p.save()
        p.translate(60, 60)
        p.rotate(self.heading or 0.0)
        p.translate(-60, -60)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(C(T.UN_RED))
        p.drawPolygon(hline_polygon((60, 14), (65, 60), (60, 66), (55, 60)))
        p.setBrush(C(T.MUTED))
        p.drawPolygon(hline_polygon((60, 106), (65, 60), (60, 54), (55, 60)))
        p.restore()
        p.setBrush(C(T.FG))
        p.drawEllipse(QPointF(60, 60), 4, 4)
        self.text(p, 60, 88, '%.0f°' % self.heading if isnum(self.heading) else '—', 14 * u, T.FG,
                  weight=600, mono=True)


class TofGauge(_ViewBox):
    """0–3 m, the hover band shaded, the reading as a fill and a papaya line."""
    VB = (160, 120)

    def __init__(self):
        super().__init__(170)
        self.m = None
        self.ok = False

    def set(self, metres, ok):
        self.m, self.ok = metres, ok
        self.update()

    def paintEvent(self, e):
        p = self.begin()
        u = 72.0 / self.logicalDpiY()
        p.setPen(QPen(C(T.BORDER), 1))
        p.setBrush(C(T.BG))
        p.drawRect(QRectF(20, 10, 120, 100))
        y1 = 110 - HOVER_BAND_M[1] / GAUGE_MAX_M * 100
        y0 = 110 - HOVER_BAND_M[0] / GAUGE_MAX_M * 100
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(C(T.UN_GREEN, 0.18))
        p.drawRect(QRectF(20, y1, 120, y0 - y1))
        mm = 0.0
        while mm <= GAUGE_MAX_M + 1e-9:
            y = 110 - mm / GAUGE_MAX_M * 100
            p.setPen(QPen(C(T.GRID), 1))
            p.drawLine(QPointF(20, y), QPointF(140, y))
            self.text(p, 142, y + 2.5, '%.1f' % mm, 7 * u, T.FAINT, align='start')
            mm += 0.5
        if self.m is not None:
            y = 110 - clamp(self.m / GAUGE_MAX_M, 0, 1) * 100
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(C(T.UN_GREEN_DEEP, 0.6))
            p.drawRect(QRectF(20, y, 120, 110 - y))
            p.setPen(QPen(C(T.PAPAYA), 2))
            p.drawLine(QPointF(14, y), QPointF(146, y))
        txt = ('%.2f m' % self.m if self.ok else '— m') if self.m is not None else '—'
        self.text(p, 80, 64, txt, 15 * u, T.FG, weight=600, mono=True)


class Legend(QWidget):
    """x y z with a short bar of each series colour."""

    def __init__(self):
        super().__init__()
        self._f = T.font(11)
        fm = QFontMetrics(self._f)
        self.setFixedSize(3 * (14 + 3 + fm.horizontalAdvance('x')) + 2 * 8, fm.height())

    def paintEvent(self, e):
        p = QPainter(self)
        p.setFont(self._f)
        fm = QFontMetrics(self._f)
        x, mid = 0, self.height() / 2
        for name, col in zip('xyz', T.SERIES):
            p.fillRect(QRectF(x, mid - 1, 14, 3), C(col))
            x += 17
            p.setPen(C(T.MUTED))
            p.drawText(QPointF(x, mid + fm.ascent() / 2 - 1), name)
            x += fm.horizontalAdvance(name) + 8


class Readout(QWidget):
    """label · value · unit over a hairline, as the page's .readout grid."""

    def __init__(self, name, unit=''):
        super().__init__()
        self.value = label('—', 18, 600, mono=True)
        self.unit = label(unit, 11, color=T.FAINT)
        self.unit.setMinimumWidth(18)
        lay = hbox(label(name, 11, color=T.FAINT, spacing=0.08, caps=True), None, self.value, self.unit,
                   spacing=6, margins=(0, 3, 0, 3))
        lay.setAlignment(Qt.AlignmentFlag.AlignBaseline)
        self.setLayout(lay)

    def set(self, text):
        self.value.setText(text)

    def paintEvent(self, e):
        p = QPainter(self)
        p.fillRect(QRect(0, self.height() - 1, self.width(), 1), C(T.BORDER))


# ----------------------------------------------------------------- plots

class Sparkline(QWidget):
    """Ten seconds of three series: grid, auto or symmetric range, the
    −10 s/now labels, a dot on the last point, and on hover a crosshair at the
    nearest sample with its values in a tool tip."""

    def __init__(self, unit, digits, symmetric):
        super().__init__()
        self.unit, self.digits, self.symmetric = unit, digits, symmetric
        self.t, self.data = [], [[], [], []]
        self.hover_x = None
        self.setMouseTracking(True)
        self.setMinimumHeight(44)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._f = T.font(10, mono=True)

    def set_data(self, t, data):
        self.t, self.data = t, data

    def mouseMoveEvent(self, e):
        self.hover_x = e.position().x()

    def leaveEvent(self, e):
        self.hover_x = None
        QToolTip.hideText()

    PAD = (36, 8, 6, 14)        # left, right, top, bottom

    def _range(self):
        lo, hi = math.inf, -math.inf
        for s in self.data:
            for v in s:
                if isnum(v):
                    lo, hi = min(lo, v), max(hi, v)
        if not lo < hi:
            lo, hi = -1.0, 1.0
        pad = (hi - lo) * 0.15 or 1.0
        lo, hi = lo - pad, hi + pad
        if self.symmetric:
            m = max(abs(lo), abs(hi))
            lo, hi = -m, m
        return lo, hi

    def _nearest(self, now):
        if self.hover_x is None or not self.t:
            return None
        pl, pr = self.PAD[0], self.PAD[1]
        tt = now - HISTORY_S + clamp((self.hover_x - pl) / (self.width() - pl - pr), 0, 1) * HISTORY_S
        return min(range(len(self.t)), key=lambda i: abs(self.t[i] - tt))

    def refresh(self):
        """Called from the window's frame timer: repaint, and keep the tool tip
        on the hovered sample while the plot scrolls under the pointer."""
        self.update()
        k = self._nearest(time.monotonic())
        if k is not None:
            d = self.digits
            tip = '%.1f s   x %s  y %s  z %s %s' % (self.t[k] - time.monotonic(), sfmt(self.data[0][k], d),
                                                    sfmt(self.data[1][k], d), sfmt(self.data[2][k], d), self.unit)
            pos = self.mapToGlobal(QPoint(int(self.hover_x) + 14, -30 + self.height() // 2))
            QToolTip.showText(pos, tip, self)

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(QRect(0, 0, w, h), C(T.BG))
        p.setPen(QPen(C(T.BORDER), 1))
        p.drawRect(QRectF(0.5, 0.5, w - 1, h - 1))
        pl, pr, pt, pb = self.PAD
        now = time.monotonic()
        tmin = now - HISTORY_S
        lo, hi = self._range()
        X = lambda tt: pl + (tt - tmin) / HISTORY_S * (w - pl - pr)
        Y = lambda v: pt + (1 - (v - lo) / (hi - lo)) * (h - pt - pb)
        p.setFont(self._f)
        fm = QFontMetrics(self._f)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
        for k in range(3):
            v = lo + (hi - lo) * k / 2
            y = Y(v)
            p.setPen(QPen(C(T.GRID), 1))
            p.drawLine(QPointF(pl, y), QPointF(w - pr, y))
            p.setPen(C(T.FAINT))
            s = '%.*f' % (self.digits, v)
            p.drawText(QPointF(pl - 4 - fm.horizontalAdvance(s), y + 3), s)
        if lo < 0 < hi:
            p.setPen(QPen(C(T.BORDER), 1))
            p.drawLine(QPointF(pl, Y(0)), QPointF(w - pr, Y(0)))
        p.setPen(C(T.FAINT))
        p.drawText(QPointF(pl, h - 3), '−%d s' % HISTORY_S)
        p.drawText(QPointF(w - pr - fm.horizontalAdvance('now'), h - 3), 'now')
        p.setPen(C(T.MUTED))
        p.drawText(QPointF(pl + 4, pt + 9), self.unit)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setClipRect(QRectF(pl - 4, 0, w - pl - pr + 8, h))
        n = len(self.t)
        for s, col in zip(self.data, T.SERIES):
            path, started = QPainterPath(), False
            for k in range(n):
                v = s[k]
                if not isnum(v):
                    started = False
                    continue
                pt_ = QPointF(X(self.t[k]), Y(v))
                if started:
                    path.lineTo(pt_)
                else:
                    path.moveTo(pt_)
                    started = True
            p.setPen(QPen(C(col), 2, cap=Qt.PenCapStyle.RoundCap, join=Qt.PenJoinStyle.RoundJoin))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawPath(path)
            if n and isnum(s[-1]):
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(C(col))
                p.drawEllipse(QPointF(X(self.t[-1]), Y(s[-1])), 3, 3)
        k = self._nearest(now)
        if k is not None:
            x = round(X(self.t[k])) + 0.5
            p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
            p.setPen(QPen(C(T.MUTED), 1))
            p.drawLine(QPointF(x, pt), QPointF(x, h - pb))


class FovPlot(QWidget):
    """The reported blobs on the camera's field of view, 2:1 like the 300×150
    canvas it replaces; the main blob green, the others grey, sized by range."""

    def __init__(self):
        super().__init__()
        self.blobs = []
        sp = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        sp.setHeightForWidth(True)
        self.setSizePolicy(sp)
        self._f = T.font(10, mono=True)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, w):
        return w // 2

    def sizeHint(self):
        return QSize(300, 150)

    def set(self, blobs):
        self.blobs = blobs
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        W, H = self.width(), self.height()
        p.fillRect(QRect(0, 0, W, H), C(T.BG))
        hx, hy = FOV_DEG / 2, FOV_DEG * 0.75 / 2
        p.setPen(QPen(C(T.GRID), 1))
        for a in range(-20, 21, 10):
            x = round(W / 2 + a / hx * W / 2) + 0.5
            p.drawLine(QPointF(x, 0), QPointF(x, H))
        for el in range(-20, 21, 10):
            y = round(H / 2 - el / hy * H / 2) + 0.5
            p.drawLine(QPointF(0, y), QPointF(W, y))
        p.setPen(QPen(C(T.BORDER), 1))
        p.drawLine(QPointF(W // 2 + 0.5, 0), QPointF(W // 2 + 0.5, H))
        p.drawLine(QPointF(0, H // 2 + 0.5), QPointF(W, H // 2 + 0.5))
        p.setFont(self._f)
        p.setPen(C(T.FAINT))
        p.drawText(QPointF(6, H - 6), 'field of view ±%g° × ±%g°' % (hx, hy))
        for b in self.blobs:
            x, y = W / 2 + b['az'] / hx * W / 2, H / 2 - b['el'] / hy * H / 2
            r = clamp(12000 / max(b['r'], 300), 3, 22)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(C(T.UN_GREEN if b.get('main') else T.MUTED, 0.85))
            p.drawEllipse(QPointF(x, y), r, r)
            p.setPen(C(T.FG))
            p.drawText(QPointF(x + r + 3, y + 3), str(b['k']))
        p.setPen(QPen(C(T.BORDER), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRect(QRectF(0.5, 0.5, W - 1, H - 1))


class CameraView(QWidget):
    """The 160×120 frame scaled without smoothing and letterboxed 4:3, with the
    detection overlay from the frame's own row: green box, papaya crosshair and
    range label. Rotate 180° turns frame and overlay together; the faint centre
    cross stays put. The overlay's line widths follow the page's 640×480
    canvas, scaled to the drawn size."""

    def __init__(self):
        super().__init__()
        self.img = None
        self.meta = None
        self.rotate = False
        self.setMinimumHeight(120)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_frame(self, img, meta):
        self.img, self.meta = img, meta
        self.update()

    def set_rotate(self, on):
        self.rotate = on
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        W, H = self.width(), self.height()
        p.fillRect(QRect(0, 0, W, H), C(T.CAM_BG))
        inner = QRectF(1, 1, W - 2, H - 2)
        if self.meta is None:
            f = T.font(13, spacing=0.1, caps=True)
            p.setFont(f)
            p.setPen(C(T.FAINT))
            p.drawText(inner, Qt.AlignmentFlag.AlignCenter, 'no frame yet · Grab or Live')
        else:
            m = self.meta
            fw, fh = m['w'], m['h']
            s = min(inner.width() / fw, inner.height() / fh)
            r = QRectF(inner.x() + (inner.width() - fw * s) / 2, inner.y() + (inner.height() - fh * s) / 2,
                       fw * s, fh * s)
            k = r.width() / 640.0
            p.save()
            if self.rotate:
                p.translate(r.center())
                p.rotate(180)
                p.translate(-r.center())
            p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, False)
            if self.img is not None and not self.img.isNull():
                p.drawImage(r, self.img)
            row = m.get('row')
            if row and row.get('ok'):
                p.setRenderHint(QPainter.RenderHint.Antialiasing)
                sx = r.width() / fw
                bx, by = r.x() + (row['cx_px'] - row['w_px'] / 2) * sx, r.y() + (row['cy_px'] - row['h_px'] / 2) * sx
                bw, bh = row['w_px'] * sx, row['h_px'] * sx
                p.setPen(QPen(C(T.UN_GREEN), max(1.0, 3 * k)))
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawRect(QRectF(bx, by, bw, bh))
                cx, cy = r.x() + row['cx_px'] * sx, r.y() + row['cy_px'] * sx
                p.setPen(QPen(C(T.PAPAYA), max(1.0, 2 * k)))
                p.drawLine(QPointF(cx - 10 * k, cy), QPointF(cx + 10 * k, cy))
                p.drawLine(QPointF(cx, cy - 10 * k), QPointF(cx, cy + 10 * k))
                f = T.font(max(8, round(18 * k)), 700)
                p.setFont(f)
                p.setPen(C(T.UN_GREEN))
                ty = by - 6 * k
                if ty - r.y() < 16 * k:
                    ty = by + bh + 20 * k
                p.drawText(QPointF(bx + 4 * k, ty), '%.2f m' % (row['range_mm'] / 1000))
            p.restore()
            p.setRenderHint(QPainter.RenderHint.Antialiasing, False)
            p.setPen(QPen(C(T.FG, 0.25), 1))
            c = r.center()
            p.drawLine(QPointF(c.x(), c.y() - 14 * k), QPointF(c.x(), c.y() + 14 * k))
            p.drawLine(QPointF(c.x() - 14 * k, c.y()), QPointF(c.x() + 14 * k, c.y()))
            stamp = 'frame %s · t %.1f s · %d×%d%s' % (m['seq'], m['t_ms'] / 1000, fw, fh,
                                                     ' · rotated' if self.rotate else '')
            f = T.font(11, mono=True)
            p.setFont(f)
            fm = QFontMetrics(f)
            box = QRectF(8, H - 6 - fm.height() - 2, fm.horizontalAdvance(stamp) + 12, fm.height() + 2)
            p.fillRect(box, C(T.CAM_BG, 0.7))
            p.setPen(C(T.MUTED))
            p.drawText(box, Qt.AlignmentFlag.AlignCenter, stamp)
        p.setPen(QPen(C(T.BORDER), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRect(QRectF(0.5, 0.5, W - 1, H - 1))


# ------------------------------------------------------------------ dock

class DockTab(QAbstractButton):
    """A dock tab: upper case, a green underline when selected; the Events tab
    also carries the "n crit · n warn" badge and a papaya dot when unread."""

    def __init__(self, text):
        super().__init__()
        self.setText(text)
        self.setCheckable(True)
        self.badge, self.badge_color, self.new = '', T.FAINT, False
        self._f = T.font(14, 700, spacing=0.08, caps=True)
        self._fb = T.font(11, 500, mono=True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFocusPolicy(Qt.FocusPolicy.TabFocus)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

    def set_badge(self, text, color):
        self.badge, self.badge_color = text, color
        self.updateGeometry()
        self.update()

    def set_new(self, on):
        if on != self.new:
            self.new = on
            self.updateGeometry()
            self.update()

    def sizeHint(self):
        w = QFontMetrics(self._f).horizontalAdvance(self.text().upper())
        if self.badge:
            w += 6 + QFontMetrics(self._fb).horizontalAdvance(self.badge)
        if self.new:
            w += 14
        return QSize(12 + w + 12, 32)

    def enterEvent(self, e):
        self.update()

    def leaveEvent(self, e):
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        on = self.isChecked()
        h = self.height()
        p.setFont(self._f)
        p.setPen(C(T.FG if on or self.underMouse() else T.MUTED))
        fm = QFontMetrics(self._f)
        x = 12
        base = (h - 2 + fm.ascent() - fm.descent()) / 2
        p.drawText(QPointF(x, base), self.text())
        x += fm.horizontalAdvance(self.text().upper())
        if self.badge:
            x += 6
            p.setFont(self._fb)
            p.setPen(C(self.badge_color))
            p.drawText(QPointF(x, base), self.badge)
            x += QFontMetrics(self._fb).horizontalAdvance(self.badge)
        if self.new:
            p.fillRect(QRectF(x + 7, base - fm.ascent() / 2 - 3, 7, 7), C(T.PAPAYA))
        if on:
            p.fillRect(QRect(0, h - 2, self.width(), 2), C(T.UN_GREEN))
        if self.hasFocus():
            p.setPen(QPen(C(T.PAPAYA), 1))
            p.drawRect(QRectF(0.5, 0.5, self.width() - 1, h - 3))


class EventRow(QWidget):
    """time · level · text, a hairline under it, a red tint for crit."""
    LEVEL = {'ok': T.UN_GREEN, 'warn': T.AMBER, 'crit': T.UN_RED, 'info': T.MUTED}

    def __init__(self, ev):
        super().__init__()
        self.crit = ev['level'] == 'crit'
        ts = label(time.strftime('%H:%M:%S', time.localtime(ev['ts'])), 12, mono=True, color=T.FAINT)
        ts.setFixedWidth(62)
        lv = label(ev['level'], 10, 600, mono=True, color=self.LEVEL.get(ev['level'], T.MUTED),
                   spacing=0.08, caps=True)
        lv.setFixedWidth(44)
        tx = label(ev['text'], 12, mono=True, wrap=True)
        tx.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        lay = hbox(ts, lv, spacing=8, margins=(0, 3, 0, 4))
        lay.addWidget(tx, 1)
        self.setLayout(lay)

    def paintEvent(self, e):
        p = QPainter(self)
        if self.crit:
            p.fillRect(self.rect(), C(T.EV_CRIT_BG))
        p.fillRect(QRect(0, self.height() - 1, self.width(), 1), C(T.BORDER))


class _DockHandle(QSplitterHandle):
    """The dock's drag handle: the 56×3 bar on the gap above it, green on
    hover or drag; a double click puts the dock back at its default height."""

    def __init__(self, orientation, splitter):
        super().__init__(orientation, splitter)
        self.setCursor(Qt.CursorShape.SizeVerCursor)
        self.setToolTip('drag to resize · double-click to reset')
        self._down = False

    def enterEvent(self, e):
        self.update()

    def leaveEvent(self, e):
        self.update()

    def mousePressEvent(self, e):
        self._down = True
        self.update()
        super().mousePressEvent(e)

    def mouseReleaseEvent(self, e):
        self._down = False
        self.update()
        super().mouseReleaseEvent(e)
        self.splitter().handle_released.emit()

    def mouseDoubleClickEvent(self, e):
        self.splitter().handle_reset.emit()

    def paintEvent(self, e):
        if not self.isEnabled():
            return
        p = QPainter(self)
        col = T.UN_GREEN if (self._down or self.underMouse()) else T.FAINT
        p.fillRect(QRect(self.width() // 2 - 28, self.height() // 2 - 1, 56, 3), C(col))


class DockSplitter(QSplitter):
    """The vertical splitter between the three panels and the dock; its one
    handle is the dock's drag handle."""
    handle_released = pyqtSignal()
    handle_reset = pyqtSignal()

    def __init__(self):
        super().__init__(Qt.Orientation.Vertical)
        self.setHandleWidth(12)
        self.setChildrenCollapsible(False)

    def createHandle(self):
        return _DockHandle(self.orientation(), self)
