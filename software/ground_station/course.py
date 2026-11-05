"""The Course panel: the race sheet (race_course.py) drawn and edited, on the
Control page's Autonomous mode (2026-11-05, race day).

The sheet is the one file the team fills at the venue: how many balloons, where
each hangs (its floor point from the start, and the height of its centre), and
how far the floor runs from the start on each side. This panel draws it from
above, to scale, with the legs the course flies and the reach of the camera
from the start, so a typo shows as a balloon in the wrong place before the
blimp looks for it there; the boxes under the sketch edit the same numbers,
and every change is written back to software/race_course.json, which
course_test.py --course-file reads too. Nothing on board measures position,
so the field is for the sketch and the checks (race_course.check()), not for
the flight: the blimp finds each balloon with its camera, as course_test.py
does.

The frame: x ahead of the start along the nose, up on the sketch; y to the
start's left, left on the sketch (the blimp seen from above, as the Motors
panel draws it). A flight marks the balloon it is on.
"""
import math
import os

from PyQt6.QtCore import QLocale, QPointF, QRectF, Qt, pyqtSignal
from PyQt6.QtGui import QPainter, QPainterPath, QPen
from PyQt6.QtWidgets import QDoubleSpinBox, QGridLayout, QSizePolicy, QSpinBox, QWidget

import race_course as rc

from . import theme as T
from .panels import Panel, button, h3, info_text
from .widgets import C, FlowBox, hbox, label, set_color

CAM_REACH_M = 2.2           # the camera found the balloon from about 2.2 m (2026-11-02)
CAM_HALF_DEG = 29.0         # half the forward camera's measured 58 deg
BALLOON_R_M = 0.2           # a race balloon, about 40 cm across


class CourseSketch(QWidget):
    """The field from above, to scale: the start S at the bottom of its
    field with the nose up, the balloons as red discs numbered in order with
    their heights, the legs as dashed lines (the plan's, from the start and
    from standoff short of each balloon), the camera's reach from S as a
    faint wedge, a 1 m grid. The balloon a flight is on gets a papaya ring."""

    MARGIN = 28

    def __init__(self):
        super().__init__()
        self.sheet = rc.normalise(rc.DEFAULT)
        self.current = None
        self.setMinimumHeight(220)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_sheet(self, sheet):
        try:
            self.sheet = rc.normalise(sheet)
        except ValueError:
            return
        self.update()

    def set_current(self, k):
        if k != self.current:
            self.current = k
            self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        p.fillRect(self.rect(), C(T.BG))
        s, f, m = self.sheet, self.sheet['field'], self.MARGIN
        x0, x1 = -f['behind_m'], f['ahead_m']       # ahead, m
        y0, y1 = -f['right_m'], f['left_m']         # left, m
        w, h = max(1.0, self.width() - 2 * m), max(1.0, self.height() - 2 * m)
        k = min(w / (y1 - y0), h / (x1 - x0))       # px per m
        cx, cy = self.width() / 2.0, self.height() / 2.0
        xm, ym = (x0 + x1) / 2.0, (y0 + y1) / 2.0

        def pt(x, y):
            return QPointF(cx - (y - ym) * k, cy - (x - xm) * k)

        # the field and its grid
        rect = QRectF(pt(x1, y1), pt(x0, y0))
        p.fillRect(rect, C(T.RAISED))
        p.setPen(QPen(C(T.GRID), 1))
        for gx in range(int(math.ceil(x0)), int(math.floor(x1)) + 1):
            p.drawLine(pt(gx, y0), pt(gx, y1))
        for gy in range(int(math.ceil(y0)), int(math.floor(y1)) + 1):
            p.drawLine(pt(x0, gy), pt(x1, gy))
        p.setPen(QPen(C(T.BORDER), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRect(rect)

        # the camera's reach from the start: a wedge, faint
        wedge = QPainterPath(pt(0.0, 0.0))
        for i in range(0, 25):
            a = math.radians(-CAM_HALF_DEG + 2 * CAM_HALF_DEG * i / 24.0)
            wedge.lineTo(pt(CAM_REACH_M * math.cos(a), CAM_REACH_M * math.sin(a)))
        wedge.closeSubpath()
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(C(T.UN_GREEN, 0.10))
        p.drawPath(wedge)

        # the legs: from the start, then from standoff short of each balloon
        pts = rc.points(s)[:len(s['balloons'])]
        pen = QPen(C(T.MUTED), 1.2, Qt.PenStyle.DashLine)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        px, py = 0.0, 0.0
        for bx, by in pts:
            p.drawLine(pt(px, py), pt(bx, by))
            dx, dy = bx - px, by - py
            d = math.hypot(dx, dy)
            fr = max(0.0, d - 0.5) / d if d > 0 else 0.0
            px, py = px + dx * fr, py + dy * fr

        # the start: a nose up, and S
        o = pt(0.0, 0.0)
        nose = QPainterPath(QPointF(o.x(), o.y() - 9))
        nose.lineTo(QPointF(o.x() - 6, o.y() + 6))
        nose.lineTo(QPointF(o.x() + 6, o.y() + 6))
        nose.closeSubpath()
        p.setPen(QPen(C(T.FG), 1))
        p.setBrush(C(T.PAPAYA))
        p.drawPath(nose)
        self._text(p, o.x() + 10, o.y() + 4, 'S', 11, T.FG, 700, mono=True)

        # the balloons
        r = max(7.0, BALLOON_R_M * k)
        for i, (bx, by) in enumerate(pts):
            c = pt(bx, by)
            if i == self.current:
                p.setPen(QPen(C(T.PAPAYA), 3))
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawEllipse(c, r + 5, r + 5)
            p.setPen(QPen(C(T.BG), 1))
            p.setBrush(C(T.UN_RED))
            p.drawEllipse(c, r, r)
            self._text(p, c.x(), c.y() + 4, str(i + 1), 11, T.FG, 700, mono=True, align='middle')
            # the height under the disc: beside it, two balloons 2 m apart
            # ran into each other on the Deck's small sketch
            self._text(p, c.x(), c.y() + r + 12, '%.2f m' % s['balloons'][i]['height_m'], 10, T.MUTED, mono=True,
                       align='middle')

        # the field's extents, on its edges
        self._text(p, rect.center().x(), rect.top() - 6, '%.1f m ahead' % f['ahead_m'], 10, T.FAINT, align='middle')
        self._text(p, rect.center().x(), rect.bottom() + 14, '%.1f m behind' % f['behind_m'], 10, T.FAINT,
                   align='middle')
        self._text(p, rect.left() - 4, rect.center().y() + 4, '%.1f m left' % f['left_m'], 10, T.FAINT, align='end')
        self._text(p, rect.right() + 4, rect.center().y() + 4, '%.1f m right' % f['right_m'], 10, T.FAINT)
        self._text(p, 4, self.height() - 6, '1 m grid', 10, T.FAINT)

    @staticmethod
    def _text(p, x, y, s, px, color, weight=400, mono=False, align='start'):
        f = T.font(px, weight, mono)
        p.setFont(f)
        p.setPen(C(color))
        w = p.fontMetrics().horizontalAdvance(s)
        dx = {'start': 0, 'middle': -w / 2, 'end': -w}[align]
        p.drawText(QPointF(x + dx, y), s)


class CoursePanel(Panel):
    """The sketch over the sheet's numbers. Every change goes to the file at
    once; Reload reads it back (after an edit by hand or a git pull)."""
    changed = pyqtSignal()
    HINT = ('Measure at the venue, from where the gondola will stand at the start with its nose toward balloon 1: '
            'each balloon\'s floor point (metres ahead, metres to the left; a balloon on the right is a negative '
            'left) and the height of its centre, and how far the floor runs ahead, behind, left and right. The '
            'blimp flies each balloon\'s height plus "fly above" (the camera sits on the gondola and sees the balloon '
            'a little above its level). "Close px" is the patch at which it goes in to touch, from a motors-off '
            'camera check with these balloons (docs/flight-tests.md, C0). Loops repeat the balloons in order. The '
            'faint wedge is how far the camera has found a balloon from the start. Nothing on board measures '
            'position: the field is for this sketch and the checks, and the blimp finds each balloon with its '
            'camera. The file is software/race_course.json; course_test.py --course-file flies the same sheet from '
            'a terminal.')

    def __init__(self, station, path=None):
        super().__init__('Course', info_text('The race sheet: where the balloons hang, how big the floor is.',
                                             self.HINT))
        self.station = station
        self.path = path or rc.DEFAULT_PATH
        self.sheet = rc.normalise(rc.DEFAULT)
        self._loading = False
        lay = self.lay
        self.sketch = CourseSketch()
        lay.addWidget(self.sketch, 1)
        lay.addSpacing(6)
        self.check_l = label('', 12, wrap=True)
        lay.addWidget(self.check_l)
        lay.addSpacing(8)

        lay.addWidget(h3('Balloons'))
        lay.addSpacing(4)
        self.count_in = self._int(1, rc.MAX_BALLOONS, 1, 'how many balloons the course has, in order')
        self.loops_in = self._int(1, rc.MAX_LOOPS, 1, 'the balloons in order this many times')
        self.close_in = self._int(20, 19200, 100, 'close: the patch at which the blimp goes in to touch, px, from '
                                                  'the camera check with these balloons')
        self.above_in = self._number(-0.5, 0.5, 0.05, 2, 'the gondola flies each centre\'s height plus this, m')
        head = FlowBox(hgap=12, vgap=6)
        for name, box, unit in (('Count', self.count_in, ''), ('Loops', self.loops_in, ''),
                                ('Close', self.close_in, 'px'), ('Fly above', self.above_in, 'm')):
            head.add(self._field(name, box, unit))
        lay.addWidget(head)
        lay.addSpacing(6)
        g = QGridLayout()
        g.setHorizontalSpacing(8)
        g.setVerticalSpacing(4)
        for col, text in enumerate(('#', 'ahead m', 'left m', 'centre m')):
            g.addWidget(label(text, 11, color=T.FAINT, spacing=0.06, caps=True), 0, col)
        self.rows = []
        for i in range(rc.MAX_BALLOONS):
            num = label('%d' % (i + 1), 13, 700, T.UN_RED, mono=True)
            x = self._number(-30.0, 30.0, 0.1, 2, 'metres ahead of the start (negative: behind)')
            y = self._number(-30.0, 30.0, 0.1, 2, 'metres to the start\'s left (negative: to its right)')
            z = self._number(0.3, 2.5, 0.1, 2, 'the height of the balloon\'s centre over the floor, m')
            for col, w in enumerate((num, x, y, z)):
                g.addWidget(w, i + 1, col)
            self.rows.append((num, x, y, z))
        g.setColumnStretch(4, 1)
        lay.addLayout(g)
        lay.addSpacing(10)

        lay.addWidget(h3('Field from the start'))
        lay.addSpacing(4)
        self.field_in = {}
        fb = FlowBox(hgap=12, vgap=6)
        for key, name in (('ahead_m', 'Ahead'), ('behind_m', 'Behind'), ('left_m', 'Left'), ('right_m', 'Right')):
            box = self._number(0.5, 30.0, 0.5, 1, 'how far the floor runs %s from the start, m' % name.lower())
            self.field_in[key] = box
            fb.add(self._field(name, box, 'm'))
        lay.addWidget(fb)
        lay.addSpacing(8)
        self.reload_b = button('Reload', tip='read the sheet from the file again (after an edit by hand or a pull)')
        self.reload_b.clicked.connect(self.load)
        self.file_l = label(os.path.basename(self.path), 11, color=T.FAINT, mono=True)
        lay.addLayout(hbox(self.reload_b, 8, self.file_l, None, spacing=0))
        lay.addSpacing(8)
        self.flight_l = label('', 12, color=T.MUTED, wrap=True)
        lay.addWidget(self.flight_l)
        self.load()                     # no stretch at the end: the room a tall window has goes to the sketch

    # ---------------------------------------------------------------- boxes

    def _number(self, lo, hi, step, decimals, tip):
        box = QDoubleSpinBox()
        box.setRange(lo, hi)
        box.setSingleStep(step)
        box.setDecimals(decimals)
        box.setLocale(QLocale.c())
        box.setKeyboardTracking(False)
        box.setFixedWidth(78)
        box.setToolTip(tip)
        box.valueChanged.connect(self._edited)
        box.editingFinished.connect(box.clearFocus)
        return box

    def _int(self, lo, hi, step, tip):
        box = QSpinBox()
        box.setRange(lo, hi)
        box.setSingleStep(step)
        box.setKeyboardTracking(False)
        box.setFixedWidth(78 if hi > 99 else 56)
        box.setToolTip(tip)
        box.valueChanged.connect(self._edited)
        box.editingFinished.connect(box.clearFocus)
        return box

    @staticmethod
    def _field(name, box, unit):
        w = QWidget()
        items = [label(name, color=T.MUTED), box]
        if unit:
            items.append(label(unit, color=T.MUTED))
        w.setLayout(hbox(*items, spacing=6))
        return w

    # ---------------------------------------------------------------- sheet

    def load(self):
        """The file into the boxes and the sketch; the default when there is
        none, and the last good sheet when the file is broken (said)."""
        try:
            sheet = rc.load(self.path)
        except (OSError, ValueError) as e:
            self.station.log('warn', 'course: %s' % e)
            sheet = self.sheet
        self._show(sheet)

    def _show(self, sheet):
        self.sheet = rc.normalise(sheet)
        s = self.sheet
        self._loading = True
        try:
            self.count_in.setValue(len(s['balloons']))
            self.loops_in.setValue(s['loops'])
            self.close_in.setValue(s['close_px'])
            self.above_in.setValue(s['fly_above_m'])
            for i, (num, x, y, z) in enumerate(self.rows):
                b = s['balloons'][i] if i < len(s['balloons']) else None
                if b is not None:
                    x.setValue(b['x_m'])
                    y.setValue(b['y_m'])
                    z.setValue(b['height_m'])
                for w in (num, x, y, z):
                    w.setVisible(b is not None)
            for key, box in self.field_in.items():
                box.setValue(s['field'][key])
        finally:
            self._loading = False
        self._refresh()

    def _edited(self, *_):
        """A box changed: the sheet from the boxes, to the file and the sketch."""
        if self._loading:
            return
        n = self.count_in.value()
        was = len(self.sheet['balloons'])
        balloons = []
        for i in range(n):
            num, x, y, z = self.rows[i]
            if i >= was:
                # A new row: beside the last balloon, at its height, so the
                # check does not refuse it as too near before it is typed in
                last = balloons[-1] if balloons else {'x_m': 0.0, 'y_m': 0.0, 'height_m': 1.2}
                self._loading = True
                try:
                    x.setValue(last['x_m'] + 1.5)
                    y.setValue(last['y_m'])
                    z.setValue(last['height_m'])
                finally:
                    self._loading = False
            balloons.append({'x_m': x.value(), 'y_m': y.value(), 'height_m': z.value()})
        for i, (num, x, y, z) in enumerate(self.rows):
            for w in (num, x, y, z):
                w.setVisible(i < n)
        self.sheet = rc.normalise({'name': self.sheet['name'], 'notes': self.sheet['notes'],
                                   'field': {k: b.value() for k, b in self.field_in.items()},
                                   'balloons': balloons, 'fly_above_m': self.above_in.value(),
                                   'close_px': self.close_in.value(), 'loops': self.loops_in.value()})
        try:
            rc.save(self.sheet, self.path)
        except OSError as e:
            self.station.log('warn', 'course: the sheet was not saved: %s' % e)
        self._refresh()
        self.changed.emit()

    def _refresh(self):
        self.sketch.set_sheet(self.sheet)
        why = rc.check(self.sheet)
        warn = rc.warnings(self.sheet)
        if why:
            self.check_l.setText('Cannot be flown: %s' % why)
            set_color(self.check_l, T.AMBER)
        elif warn:
            self.check_l.setText('%s. Warning: %s' % (rc.summary(self.sheet), '; '.join(warn)))
            set_color(self.check_l, T.AMBER)
        else:
            self.check_l.setText(rc.summary(self.sheet))
            set_color(self.check_l, T.MUTED)

    def sheet_now(self):
        """The sheet as the boxes have it."""
        return rc.normalise(self.sheet)

    # --------------------------------------------------------------- flight

    def set_flight(self, f):
        """assist.py's snapshot: the phase on the status line, the balloon
        being flown to marked on the sketch."""
        state = f.get('state', 'idle')
        detail = f.get('detail') or ''
        if state == 'idle':
            self.flight_l.setText('')
            self.sketch.set_current(None)
            return
        if f.get('link_lost'):
            self.flight_l.setText('Link lost: the blimp stops its motors after 1 s, and the course goes on if the '
                                  'link returns within 3 s.')
            set_color(self.flight_l, T.AMBER)
        else:
            self.flight_l.setText({'preparing': 'Preparing: ', 'landing': 'Landing: '}.get(state, '') + detail)
            set_color(self.flight_l, T.AMBER if state == 'landing' else T.MUTED)
        k = None
        if state == 'flying':
            m = _balloon_no(detail)
            if m is not None:
                k = (m - 1) % max(1, len(self.sheet['balloons']))
        self.sketch.set_current(k)


def _balloon_no(text):
    """The balloon number named in a phase, or None."""
    import re
    m = re.search(r'balloon (\d+)', text)
    return int(m.group(1)) if m else None
