"""The window's three panels and the console/events dock.

Each panel ports one section of the old page (web/index.html and app.js in git
history) and keeps its rules: the readback bars draw the blimp's telemetry,
never the slider; the camera side ages to NO DATA after 2 s without a row; the
sensor notices appear only while the link is up. Painting lives in widgets.py,
colours and fonts in theme.py; this file is layout and behaviour.
"""
import collections
import math
import statistics
import time

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (QFrame, QGridLayout, QLabel, QLineEdit, QPlainTextEdit, QPushButton,
                             QScrollArea, QSizePolicy, QSpinBox, QStackedWidget, QVBoxLayout, QWidget)

from . import theme as T
from .camera import BlobTracker
from .widgets import (HISTORY_S, HOVER_BAND_M, ChamferFrame, CameraView, Compass, DockTab, ElidedValue, EventRow,
                      FlowBox, FovPlot, InfoIcon, Legend, Readout, SegBar, Sparkline, Switch, ThrustSlider, TofGauge,
                      TopView, clamp, fmt, hbox, isnum, label, set_color, sfmt)

MOTORS = ('FL', 'RL', 'FR', 'RR')           # wire order (motor_id_t)
PAIR = {'FL': 'FR', 'FR': 'FL', 'RL': 'RR', 'RR': 'RL'}
SEN_STALE_S = 1.5
CAM_ROW_STALE_S = 2.0
CAM_TEXT_EVERY_S = 0.25     # the camera's verdict, readouts and table: four refreshes a second
CAM_HEALTH_EVERY_S = 1.0    # its counters line: one
TOF_STATUS = {0: 'valid', 1: 'sigma fail', 2: 'signal fail', 3: 'below min range', 4: 'out of bounds',
              5: 'hardware fail', 6: 'no update', 7: 'wraparound', 255: 'no measurement'}
QUICK = ('help', 'motor --status', 'wifi', 'link', 'imu -n 3', 'tof -n 3', 'mag -n 3', 'baro', 'motor -s')
CON_HISTORY = 50
CON_TIMEOUT_MS = 320000     # the Send button comes back even if a reply never ends
EVENTS_MAX = 200


def step_button(text, tip):
    """A square −/+ button beside a slider. Held down, it repeats."""
    b = QPushButton(text)
    b.setFont(T.font(15, 600, mono=True))
    b.setStyleSheet('padding: 0;')
    b.setFixedSize(26, 24)
    b.setFocusPolicy(Qt.FocusPolicy.TabFocus)
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    b.setAutoRepeat(True)
    b.setAutoRepeatDelay(400)
    b.setAutoRepeatInterval(80)
    b.setToolTip(tip)
    b.setEnabled(False)
    return b


def button(text, kind=None, tip=''):
    """A .btn: upper case and slightly spaced, which the sheet cannot do."""
    b = QPushButton(text)
    b.setFont(T.font(12, 600, spacing=0.04, caps=True))
    b.setFocusPolicy(Qt.FocusPolicy.TabFocus)
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    if kind:
        b.setProperty('kind', kind)
    if tip:
        b.setToolTip(tip)
    return b


def h3(text):
    return label(text, 12, 700, T.MUTED, spacing=0.1, caps=True)


def code(text):
    """Inline <code> in rich text: mono, green, as the page's code element."""
    return '<span style="font-family:\'%s\'; color:%s;">%s</span>' % (T.MONO_FAMILY, T.UN_GREEN, text)


def notice(html):
    lb = QLabel(html)
    lb.setProperty('role', 'notice')
    lb.setWordWrap(True)
    lb.setTextFormat(Qt.TextFormat.RichText)
    lb.setFont(T.font(12))
    lb.hide()
    return lb


def scroll(widget):
    """A panel body: scrolls only when the window is too small for it."""
    sa = QScrollArea()
    sa.setWidgetResizable(True)
    sa.setFrameShape(QFrame.Shape.NoFrame)
    sa.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    sa.setWidget(widget)
    return sa


def info_text(*parts):
    """An info icon's note from paragraphs."""
    return '<br><br>'.join(p for p in parts if p)


class Panel(ChamferFrame):
    """Head (title, its info icon) over a scrolling body.

    The note that explains a panel sits behind the i beside its title
    (widgets.InfoIcon) since 2026-11-02; until then it was a line of faint text
    on the right of the head, and with the hints under the controls the
    window read as much as it showed. note_visible keeps a note on the head
    for a panel whose note is a reading, not an explanation (the traces'
    file)."""

    def __init__(self, title, note='', note_visible=False):
        super().__init__()
        if note_visible:
            self.note = label(note, 12, color=T.FAINT, wrap=True)    # wraps under its own column when narrow
            self.note.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.note.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            head = hbox(label(title, 15, 700, spacing=0.08, caps=True), self.note, spacing=12,
                        margins=(16, 12, 16, 8))
            head.setStretch(1, 1)
        else:
            self.note = InfoIcon(note)
            head = hbox(label(title, 15, 700, spacing=0.08, caps=True), 6, self.note, None, spacing=0,
                        margins=(16, 12, 16, 8))
        self.body = QWidget()
        self.lay = QVBoxLayout(self.body)
        self.lay.setContentsMargins(16, 4, 16, 14)
        self.lay.setSpacing(0)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        outer.addLayout(head)
        outer.addWidget(scroll(self.body), 1)


# ----------------------------------------------------------------- motors

class MotorsPanel(Panel):
    """Four sliders in wire order, readback under each, the top view above.

    Safety on this side: the setpoint goes to the station when a slider, the
    limit or the enable switch changes, and the bridge's own thread repeats
    it to the blimp for as long as "Motors enabled" is on (blimp_bridge.py
    owns the standing command; its watchdog releases the motors if the
    window stops responding, and the window then STOPs itself). Until
    2026-10-28 a 100 ms timer here was the liveness, and a long repaint
    cut the motors. STOP zeroes the sliders and switches the motors off.
    The limit clamps every slider."""
    HINT = ('Enable, then move a slider. The setpoint stands until you change it or STOP; a window that stops '
            'responding for 3 s releases the motors. If the Wi-Fi drops the blimp coasts, and the setpoint comes '
            'back, ramped over 1 s, if the link returns within 3 s.')
    RACE_HINT = 'Race mode: this window only watches. STOP is the one command it sends to the motors.'

    def __init__(self, station):
        # The value is the motor command, signed: the firmware maps a command
        # c onto [deadband, ceiling], db + (top − db)·|c|. Since 2026-11-01 it
        # boots at 10 % and 40 %, so 0.003 is 10.1 % duty and 1 is 40 %; with
        # 'motor --max-duty 100 --deadband 0' (the thrust stand) c is its duty.
        super().__init__('Motors', info_text("Signed command, −1 … +1; 1 is the blimp's duty ceiling. The bars "
                                             "under each slider show the blimp's readback, not the slider.",
                                             None if station.race else self.HINT))
        self.station = station
        self.enabled = False
        self.limit = 0.30
        self.alive = False
        self.tel = None
        self.top = TopView()
        self.lay.addWidget(self.top)
        self.lay.addSpacing(8)
        # Off by default: each slider drives its own motor. On, a slider also
        # sets the other motor of its pair (FL with FR, RL with RR). It sits
        # in its own row above the sliders so its state is seen before a
        # slider is touched.
        self.pairs = Switch('Move pairs together')
        self.pairs.setToolTip('On: FL moves FR and RL moves RR, and the other way round')
        # The note takes what the switch leaves and elides, so this row never
        # asks for more than the panel's 340 px (it asked for 342 and the body
        # was clipped by two pixels, 2026-10-28).
        pairs_note = ElidedValue('FL+FR · RL+RR')
        pairs_note.setFont(T.font(11, mono=True))
        set_color(pairs_note, T.FAINT)
        pairs_row = hbox(self.pairs, pairs_note, spacing=12)
        pairs_row.setStretchFactor(pairs_note, 1)
        self.lay.addLayout(pairs_row)
        self.lay.addSpacing(10)
        self.sliders, self.vals, self.bars, self.steps = {}, {}, {}, {}
        tower = QGridLayout()
        tower.setHorizontalSpacing(8)
        tower.setVerticalSpacing(4)
        tower.setColumnMinimumWidth(0, 30)
        tower.setColumnMinimumWidth(4, 56)
        tower.setColumnStretch(2, 1)
        for i, m in enumerate(MOTORS):
            sl = ThrustSlider()
            sl.setEnabled(False)
            sl.setToolTip('%s command, percent of the limit · double-click for zero' % m)
            sl.valueChanged.connect(lambda v, m=m: self._slid(m, v))
            # Steps of 1 on the slider's −100…100 scale, i.e. 1 % of the limit.
            # setValue() goes through valueChanged, so a step follows the pair
            # switch and is sent like a drag.
            minus = step_button('−', '%s: one step down (1 %% of the limit) · hold to repeat' % m)
            plus = step_button('+', '%s: one step up (1 %% of the limit) · hold to repeat' % m)
            minus.clicked.connect(lambda _c, sl=sl: sl.setValue(sl.value() - 1))
            plus.clicked.connect(lambda _c, sl=sl: sl.setValue(sl.value() + 1))
            val = label('+0.000', 13, mono=True)
            val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            bar = SegBar()
            tower.addWidget(label(m, 14, 900, T.MUTED, spacing=0.08), 2 * i, 0)
            tower.addWidget(minus, 2 * i, 1)
            tower.addWidget(sl, 2 * i, 2)
            tower.addWidget(plus, 2 * i, 3)
            tower.addWidget(val, 2 * i, 4)
            tower.addWidget(bar, 2 * i + 1, 0, 1, 5)
            if i < 3:
                tower.setRowMinimumHeight(2 * i + 1, 6)
            self.sliders[m], self.vals[m], self.bars[m], self.steps[m] = sl, val, bar, (minus, plus)
        self.lay.addLayout(tower)
        self.lay.addSpacing(12)

        self.enable = Switch('Motors enabled')
        self.enable.toggled.connect(self._enable_toggled)
        self.limit_in = QSpinBox()
        self.limit_in.setRange(5, 100)
        self.limit_in.setSingleStep(5)
        self.limit_in.setValue(30)
        self.limit_in.setKeyboardTracking(False)      # act on Enter or focus-out, as the page's change event
        self.limit_in.setFixedWidth(64)
        self.limit_in.valueChanged.connect(self._limit_changed)
        lim = QWidget()
        lim.setLayout(hbox(label('Limit', color=T.MUTED), self.limit_in, label('%', color=T.MUTED), spacing=6))
        zero = button('Zero all')
        zero.clicked.connect(self.zero)
        controls = FlowBox(hgap=16, vgap=10)
        for w in (self.enable, lim, zero):
            controls.add(w)
        self.lay.addWidget(controls)
        # The how-to is in the panel's info; race mode keeps its one line in
        # view, because it says why nothing here moves a motor
        self.hint = label(self.RACE_HINT, 12, color=T.FAINT, wrap=True) if station.race else None
        if self.hint is not None:
            self.lay.addSpacing(10)
            self.lay.addWidget(self.hint)
        self.lay.addStretch(1)
        if station.race:
            self.enable.setEnabled(False)
            self.enable.setToolTip('race mode: no manual control')

    # setpoints out

    def command(self):
        return [clamp(self.sliders[m].value() / 100 * self.limit, -1, 1) for m in MOTORS]

    def send(self):
        if self.enabled:
            self.station.set_motors(self.command())

    def _slid(self, m, v):
        if self.pairs.isChecked():
            other = self.sliders[PAIR[m]]
            other.blockSignals(True)
            other.setValue(v)
            other.blockSignals(False)
        self.send()

    def _enable_toggled(self, on):
        if on:
            self.set_enabled(True)
        else:
            self.stop_all()

    def set_enabled(self, on):
        if on and self.station.race:
            self.station.log('warn', 'race mode: the motors stay off from this window')
            on = False
        self.enabled = on
        self.enable.blockSignals(True)
        self.enable.setChecked(on)
        self.enable.blockSignals(False)
        for m, sl in self.sliders.items():
            sl.setEnabled(on)
            for b in self.steps[m]:
                b.setEnabled(on)
        if on:
            self.send()
            self.station.log('info', 'motors enabled, limit %d %%' % round(self.limit * 100))

    def _set_sliders(self, value):
        for sl in self.sliders.values():
            sl.blockSignals(True)
            sl.setValue(value)
            sl.blockSignals(False)

    def stop_all(self, brake=False):
        self.station.stop(brake)
        self._set_sliders(0)
        if self.enabled:
            self.set_enabled(False)

    def zero(self):
        self._set_sliders(0)
        self.send()

    def _limit_changed(self, pct):
        self.limit = clamp(pct / 100, 0.05, 1.0)
        self.station.log('warn' if self.limit > 0.5 else 'info', 'command limit %d %%' % round(self.limit * 100))
        self.send()

    # readback in

    def on_tel(self, m):
        self.tel, self.alive = m.get('tel'), bool(m.get('alive'))
        self.render()

    def set_stale(self):
        self.alive = False
        self.render()

    def render(self):
        live = self.tel is not None and self.alive
        values = {}
        for m in MOTORS:
            v = self.tel['motor'][m] if live else float('nan')
            values[m] = v
            self.vals[m].setText(sfmt(v, 3))
            self.bars[m].set(v)
        self.top.set(values)

    def compact(self, on):
        self.top.set_max_width(260 if on else 340)
        if self.hint is not None:
            self.hint.setVisible(not on)


# ----------------------------------------------------------------- camera

class BlobTable(QWidget):
    """# · az · el · range · conf, the main blob in green.

    All four rows are always there, an empty slot as a faint dash, so the
    table never changes height and nothing under it moves when a blob comes
    or goes (until 2026-11-02 a row appeared and vanished with each blob, a
    dozen times a second). A blob the tracker holds after it went missing is
    faint too."""
    HEAD = ('#', 'az °', 'el °', 'range m', 'conf')

    def __init__(self):
        super().__init__()
        g = QGridLayout(self)
        g.setContentsMargins(0, 0, 0, 0)
        g.setHorizontalSpacing(8)
        g.setVerticalSpacing(0)
        right = Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
        left = Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
        for c, h in enumerate(self.HEAD):
            lb = label(h, 12, 500, T.FAINT, mono=True)
            lb.setAlignment(left if c == 0 else right)
            lb.setContentsMargins(4, 2, 4, 2)
            g.addWidget(lb, 0, c)
        rule = QFrame()
        rule.setFixedHeight(1)
        rule.setStyleSheet('background: %s;' % T.BORDER)
        g.addWidget(rule, 1, 0, 1, 5)
        self.cells = []
        for r in range(4):
            row = []
            for c in range(5):
                lb = label('', 12, mono=True)
                lb.setAlignment(left if c == 0 else right)
                lb.setContentsMargins(4, 2, 4, 2)
                g.addWidget(lb, r + 2, c)
                row.append(lb)
            self.cells.append(row)
        self.set([])

    def set(self, blobs):
        """blobs: [{'k', 'az', 'el', 'r', 'c', 'main'?, 'held'?}], each in row k."""
        by_k = {b['k']: b for b in blobs}
        for r, row in enumerate(self.cells):
            b = by_k.get(r + 1)
            if b is None:
                texts, color = (str(r + 1), '—', '—', '—', '—'), T.FAINT
            else:
                texts = (str(b['k']), sfmt(b['az'], 1), sfmt(b['el'], 1),
                         fmt(b['r'] / 1000, 2) if isnum(b['r']) and b['r'] >= 0 else '—',
                         fmt(b['c'], 2) if isnum(b['c']) and b['c'] >= 0 else '—')
                color = T.UN_GREEN if b.get('main') else T.FAINT if b.get('held') else T.FG
            for lb, text in zip(row, texts):
                if lb.text() != text:
                    lb.setText(text)
                if lb.property('ink') != color:        # a style sheet change re-polishes: only on a change
                    lb.setProperty('ink', color)
                    set_color(lb, color)


class CameraPanel(Panel):
    """The frame with its overlay and the camera buttons, beside the verdict,
    readouts, field-of-view plot, blob table and link health.

    Two speeds since 2026-11-02. The pictures (the balloons in the view, the
    dots on the field of view) follow every row, 12.5 a second: motion reads
    well at that rate. The text (verdict, readouts, table) refreshes four
    times a second, from the rows since its last refresh: the verdict by
    majority (a tie takes the newest row), blob 1's numbers as the median of
    the rows that saw it, so each figure shown is one the camera measured and
    a one-frame outlier never reaches the screen, and the lag is one refresh.
    Averaging was the other choice and was turned down: it lags more, shows
    values the camera never saw (the mean of two balloons when the target
    switches) and means nothing for a verdict. Blobs 2-4 keep their numbers
    through camera.BlobTracker, so the view, the plot and the table agree and
    nothing trades places. The counters line refreshes once a second. All of
    it is display: the rows reach nothing else from here."""
    INFO = ('The forward ESP32-CAM (target_cam), through the S3: every row it detects, 12.5 a second, 160 × 120. '
            'Bearing and elevation are the camera\'s own angles (bearing + to the right; core/ steers on the core '
            'sign times it). Range comes from the balloon\'s apparent size and is not calibrated. Pictures follow '
            'every row; the numbers refresh four times a second, as the median of the rows in between, and blobs '
            '2-4 keep their numbers while they move.')

    def __init__(self, station):
        super().__init__('Camera', self.INFO)
        self.station = station
        self.status = {}
        self.row, self.row_at = None, 0.0
        self.row_times = collections.deque()
        self._aged = True
        self.tracker = BlobTracker()
        self.extra = []                 # blobs 2-4 in their tracked slots, from the latest row
        self.window = []                # rows since the text last refreshed
        self._text_at, self._dirty = 0.0, False
        self._health_at = 0.0

        self.view = CameraView()
        self.b_grab = button('Grab frame')
        self.b_live = button('Live', 'toggle')
        self.b_rotate = button('Rotate 180°', 'toggle')
        for b in (self.b_live, self.b_rotate):
            b.setCheckable(True)
        self.b_grab.clicked.connect(lambda: self.station.camera_command('grab'))
        # Live follows the camera's status, not the click: the click asks, the status answers
        self.b_live.clicked.connect(lambda: (self.b_live.setChecked(bool(self.status.get('live'))),
                                             self.station.camera_command('live', on=not self.status.get('live'))))
        self.b_rotate.toggled.connect(self.view.set_rotate)
        buttons = FlowBox(hgap=8, vgap=8)
        for w in (self.b_grab, self.b_live, self.b_rotate):
            buttons.add(w)
        main = QVBoxLayout()
        main.setSpacing(10)
        main.addWidget(self.view, 1)
        main.addWidget(buttons)

        self.verdict = QFrame()
        self.verdict.setObjectName('verdict')
        self.v_word = label('NO DATA', 24, 900, T.MUTED, spacing=0.12)
        self.v_sub = label("waiting for the camera's rows", 12, color=T.MUTED, wrap=True)
        vl = QVBoxLayout(self.verdict)
        vl.setContentsMargins(14, 10, 14, 10)
        vl.setSpacing(4)
        vl.addWidget(self.v_word)
        vl.addWidget(self.v_sub)
        self.ro = {'az': Readout('Bearing', '°'), 'el': Readout('Elevation', '°'), 'range': Readout('Range', 'm'),
                   'conf': Readout('Conf')}
        ro = QGridLayout()
        ro.setHorizontalSpacing(12)
        ro.setVerticalSpacing(6)
        for i, k in enumerate(('az', 'el', 'range', 'conf')):
            ro.addWidget(self.ro[k], i // 2, i % 2)
        self.ro['az'].setToolTip("the camera's own azimuth, + to the right; core's bearing is the core sign times this")
        # 'Confidence' and the value did not fit a 144 px column: the name lost its last letter (2026-11-02)
        self.ro['conf'].setToolTip("the detector's confidence, 0 to 1; — while the camera does not compute it "
                                   "(its classifier stage is not trained yet)")
        self.fov = FovPlot()
        self.blobs = BlobTable()
        self.health = label('—', 11, color=T.FAINT, mono=True)
        side = QWidget()
        side.setFixedWidth(300)
        sl = QVBoxLayout(side)
        sl.setContentsMargins(0, 0, 0, 0)
        sl.setSpacing(10)
        for w in (self.verdict, ro, self.fov, self.blobs, self.health):
            sl.addLayout(w) if isinstance(w, QGridLayout) else sl.addWidget(w)
        sl.addStretch(1)
        side_scroll = scroll(side)
        side_scroll.setFixedWidth(300 + 8)       # room for the scroll bar when it shows

        self.lay.addLayout(hbox(main, side_scroll, spacing=14))
        self.lay.setStretch(0, 1)
        self.render_side()

    def on_status(self, st):
        self.status = st
        src = st.get('source')
        have = bool(src) and src != 'none'
        # The link carries the camera's rows, not its frames (target_cam sends the S3 rows only), so
        # Grab and Live exist only where a painter draws frames, in the demo
        frames = bool(st.get('frames'))
        self.note.setText(info_text(self.INFO, ('Source: %s, frames painted from the rows.' % src if frames
                                                else 'Source: %s, rows only: no frames on the link.' % src)
                                    if have else 'No camera source.'))
        self._health_at = 0.0           # a status change shows in the counters line at once
        self.b_grab.setEnabled(frames)
        self.b_live.setEnabled(frames)
        tip = '' if frames else 'the link carries no frames: target_cam sends the S3 rows only'
        self.b_grab.setToolTip(tip)
        self.b_live.setToolTip(tip)
        self.b_live.setChecked(bool(st.get('live')))
        self.render_side()

    def on_frame(self, img, meta):
        self.view.set_frame(img, meta)
        if meta.get('row'):
            self.on_row(meta['row'])

    def on_row(self, row):
        """Every row: the pictures now, the text at its own rate (age())."""
        now = time.monotonic()
        self.row, self.row_at = row, now
        self.extra = self.tracker.update(row)
        self.view.set_row(row, extra=self.extra)
        self.fov.set(self._fov_blobs(row))
        self.window.append(row)
        self.row_times.append(now)
        while self.row_times and now - self.row_times[0] > CAM_ROW_STALE_S:
            self.row_times.popleft()
        self._aged = False
        self._dirty = True
        if now - self._text_at >= CAM_TEXT_EVERY_S:
            self.render_side()

    def age(self):
        """Frame timer (30 Hz): the text's next refresh, and NO DATA once,
        when the rows stop."""
        now = time.monotonic()
        if self.row is not None and not self._aged and now - self.row_at > CAM_ROW_STALE_S:
            self._aged = True
            self.tracker.reset()
            self.extra = []
            self.render_side()      # also tells the view why there is no row
        elif self._dirty and now - self._text_at >= CAM_TEXT_EVERY_S:
            self.render_side()

    def _fov_blobs(self, row):
        blobs = [{'k': 1, 'az': row['az_deg'], 'el': row['el_deg'], 'r': row.get('range_mm'), 'main': True}] \
            if row and row.get('ok') else []
        return blobs + [b for b in self.extra if isnum(b['az']) and isnum(b['el'])]

    def render_side(self):
        now = time.monotonic()
        row = self.row
        fresh = row is not None and now - self.row_at < CAM_ROW_STALE_S
        win, self.window = (self.window or ([row] if row else [])), []
        self._text_at, self._dirty = now, False
        ok, figs = False, {}
        if fresh:
            seen = [r for r in win if r.get('ok')]
            ok = 2 * len(seen) > len(win) or (2 * len(seen) == len(win) and bool(win[-1].get('ok')))
            if ok:
                figs = self._medians(seen)
        if not fresh:
            state, word, color = 'idle', 'NO DATA', T.MUTED
            sub = self.no_data_reason()
            self.view.set_row(None, sub)
            self.fov.set([])
        elif ok:
            n = figs['blobs']
            state, word, color = 'target', 'TARGET', T.UN_GREEN
            sub = '%d blob%s · area %d px%s' % (n, '' if n == 1 else 's', figs['area'],
                                                ' · clipped at the edge' if figs['clipped'] else '')
        else:
            n = win[-1].get('blobs', 0)
            state, word, color = 'none', 'NO TARGET', T.FG
            sub = '%d blob%s · stage %s' % (n, '' if n == 1 else 's', win[-1].get('stage'))
        if self.verdict.property('state') != state:
            T.restyle(self.verdict, state=state)
            set_color(self.v_word, color)
        self.v_word.setText(word)
        self.v_sub.setText(sub)
        self.ro['az'].set(sfmt(figs['az'], 1) if ok else '—')
        self.ro['el'].set(sfmt(figs['el'], 1) if ok else '—')
        self.ro['range'].set(fmt(figs['range'] / 1000, 2) if ok and figs['range'] is not None else '—')
        self.ro['conf'].set(fmt(figs['conf'], 2) if ok and figs['conf'] is not None else '—')
        blobs = []
        if ok:
            blobs.append({'k': 1, 'az': figs['az'], 'el': figs['el'],
                          'r': figs['range'] if figs['range'] is not None else -1, 'c': figs['conf'], 'main': True})
        if fresh:
            blobs += self.extra
        self.blobs.set(blobs)
        if now - self._health_at >= CAM_HEALTH_EVERY_S:
            self._health_at = now
            self._render_health()

    @staticmethod
    def _medians(rows):
        """Blob 1's figures over the rows that saw it: each the lower median,
        a value one of the rows carried. A range only when most rows had one
        (the camera gives -1 while the balloon is clipped)."""
        def med(key, keep=lambda v: True):
            vals = [r[key] for r in rows if isnum(r.get(key)) and keep(r[key])]
            return statistics.median_low(vals) if vals else None
        ranged = [r for r in rows if isnum(r.get('range_mm')) and r['range_mm'] >= 0]
        return {'az': med('az_deg'), 'el': med('el_deg'), 'area': med('area_px') or 0,
                'range': med('range_mm', lambda v: v >= 0) if 2 * len(ranged) >= len(rows) else None,
                'conf': med('conf', lambda v: v >= 0), 'blobs': med('blobs') or 1,
                'clipped': bool(rows[-1].get('clipped'))}

    def _render_health(self):
        """One short line; everything else the counters say is in its tip."""
        st = self.status
        if not st.get('source') or st['source'] == 'none':
            self.health.setText('no camera source')
            self.health.setToolTip('')
            return
        rps = len(self.row_times) / CAM_ROW_STALE_S
        # Short enough for the 300 px column (the Cam pill says whether the relay is up)
        line = '%s%.1f rows/s · %s skipped · %s rejected' % (
            '' if st.get('connected') else 'link down · ', rps, st.get('skipped', 0), st.get('rejected', 0))
        sign = st.get('bearing_sign')
        tip = ['%.1f rows/s here, %s on the S3' % (rps, st.get('rows_s', '—')),
               'skipped (frame_seq gaps): %s' % st.get('skipped', 0),
               'rejected by the S3 (lines that were not rows): %s' % st.get('rejected', 0),
               'CSV format: %s' % ('v%d' % st['csv_v'] if st.get('csv_v') else '—'),
               "core sign: %s (core/'s bearing = sign x the camera's az)" % ('%+d' % sign if sign else '—')]
        if st.get('frames'):
            tip.append('frames ok %d, bad %d' % (st.get('frames_ok') or 0, st.get('frames_bad') or 0))
        if self.health.text() != line:
            self.health.setText(line)
        self.health.setToolTip('<br>'.join(tip))

    def no_data_reason(self):
        """Why there is no fresh row, from the one place each cause shows: the
        relay's packets, the S3's camera UART, its byte count, the rows."""
        st = self.status
        if not st.get('source') or st['source'] == 'none':
            return 'no camera source'
        if not st.get('packets'):
            return 'no camera packets from the blimp: firmware before the camera relay?'
        if not st.get('connected'):
            return 'the blimp stopped sending camera packets'
        if not st.get('uart'):
            return 'the S3 camera UART is not running'
        if self.row is None:
            if not st.get('bytes'):
                return 'the S3 hears nothing from the camera: wiring or power'
            return 'bytes arrive but no rows: baud, or not target_cam'
        if st.get('streaming_off'):
            return 'rows stopped: the camera says streaming off'
        return 'rows stopped: none from the camera for 2 s'


# ---------------------------------------------------------------- sensors

def xyz_row(grid, r, key):
    k = label(key, 11, color=T.FAINT, spacing=0.06, caps=True)
    grid.addWidget(k, r, 0)
    cells = []
    for c in range(3):
        lb = label('—', 13, mono=True)
        lb.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        grid.addWidget(lb, r, c + 1)
        cells.append(lb)
    return cells


def xyz_grid():
    g = QGridLayout()
    g.setHorizontalSpacing(8)
    g.setVerticalSpacing(2)
    g.setColumnMinimumWidth(0, 70)
    for c in (1, 2, 3):
        g.setColumnStretch(c, 1)
    return g


class SensorsPanel(Panel):
    """IMU values and ten seconds of plots, attitude from gravity, heading,
    the range below against the hover band, barometer and magnetometer."""

    def __init__(self, demo):
        super().__init__('Sensors', 'The IMU, magnetometer, barometer and VL53L1X on the blimp, as its sensor '
                                    'packets bring them: the plots hold the last ten seconds.')
        self.demo = demo
        self.sen, self.sen_at, self.seen = None, 0.0, False
        self.alive = False
        self.t = collections.deque()
        self.acc = [collections.deque() for _ in range(3)]
        self.gyro = [collections.deque() for _ in range(3)]

        self.missing = notice('No sensor packet from the blimp yet. The firmware streams its sensors from boot; '
                              '%s in the console shows what it found, and %s starts the stream if it is off.'
                              % (code('sensors'), code('sensors --on')))
        self.stale = notice('Sensor stream paused: no packet for more than 1.5 s. A console sensor command (%s, %s, '
                            '%s, %s) borrows the chips while it runs; otherwise ask %s.'
                            % (code('imu'), code('mag'), code('baro'), code('tof'), code('sensors')))
        for n in (self.missing, self.stale):
            self.lay.addWidget(n)
            self.lay.addSpacing(10)
        self._notice_gaps = (self.lay.itemAt(1), self.lay.itemAt(3))

        g = xyz_grid()
        self.acc_l = xyz_row(g, 0, 'acc g')
        self.gyro_l = xyz_row(g, 1, 'gyro °/s')
        self.plot_acc = Sparkline('g', 2, symmetric=False)
        self.plot_gyro = Sparkline('°/s', 1, symmetric=True)
        imu = QVBoxLayout()
        imu.setSpacing(0)
        imu.addLayout(hbox(h3('IMU'), None, Legend(), spacing=10))
        imu.addSpacing(6)
        imu.addLayout(g)
        imu.addSpacing(12)
        imu.addWidget(self.plot_acc, 1)
        imu.addSpacing(6)
        imu.addWidget(self.plot_gyro, 1)
        self.lay.addLayout(imu, 1)
        self.gap1 = 10
        self.lay.addSpacing(10)

        self.att = {'roll': Readout('Roll', '°'), 'pitch': Readout('Pitch', '°'), 'temp': Readout('IMU temp', '°C')}
        self.compass = Compass()
        left = QVBoxLayout()
        left.setSpacing(4)
        left.addWidget(h3('Attitude'))
        left.addSpacing(2)
        for w in self.att.values():
            left.addWidget(w)
        left.addStretch(1)
        right = QVBoxLayout()
        right.setSpacing(6)
        right.addWidget(h3('Heading'))
        right.addWidget(self.compass)
        right.addStretch(1)
        self.lay.addLayout(self._two(left, right))
        self.lay.addSpacing(10)

        self.tof = TofGauge()
        self.tof_status = label('—', 12, color=T.FAINT, mono=True, wrap=True)
        tl = QVBoxLayout()
        tl.setSpacing(6)
        tl.addLayout(hbox(h3('Range below'), 6, InfoIcon('The VL53L1X time-of-flight sensor, looking down; the '
                                                         'band is the hover height, 1.85–2.25 m.'), None, spacing=0))
        tl.addWidget(self.tof)
        tl.addWidget(self.tof_status)
        tl.addStretch(1)
        # Altitude is the standard-atmosphere figure the 'baro' command prints;
        # the packet's own baro_alt_m is relative to the first reading since
        # boot, so it reads about 0 on the bench and gets its own row.
        self.baro = {'p': Readout('Pressure', 'hPa'), 't': Readout('Temp', '°C'),
                     'alt': Readout('Altitude', 'm'), 'rel': Readout('Since boot', 'm')}
        self.baro['t'].setToolTip("the air temperature at the barometer ('Temperature' did not fit its column)")
        bl = QVBoxLayout()
        bl.setSpacing(4)
        bl.addWidget(h3('Barometer'))
        bl.addSpacing(2)
        for w in self.baro.values():
            bl.addWidget(w)
        bl.addStretch(1)
        self.lay.addLayout(self._two(tl, bl))
        # Full width, on the IMU's grid: three signed values with three
        # decimals don't fit beside a key in half of a 400 px panel.
        self.lay.addSpacing(10)
        self.lay.addWidget(h3('Magnetometer'))
        self.lay.addSpacing(4)
        mg = xyz_grid()
        self.mag_l = xyz_row(mg, 0, 'gauss')
        self.lay.addLayout(mg)

    @staticmethod
    def _two(a, b):
        """Two equal columns. Each sits in a widget whose width hint is
        ignored, or the wider block's hint wins the space and the gauge in
        the other shrinks to a stub."""
        lay = hbox(spacing=12)
        for inner in (a, b):
            col = QWidget()
            col.setLayout(inner)
            col.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            lay.addWidget(col, 1)
        return lay

    def on_tel_alive(self, alive):
        self.alive = alive

    def on_sen(self, s):
        now = time.monotonic()
        self.sen, self.sen_at, self.seen = s, now, True
        self.t.append(now)
        for i in range(3):
            self.acc[i].append(s['acc'][i] if s.get('imu_valid') else float('nan'))
            self.gyro[i].append(s['gyro'][i] if s.get('imu_valid') else float('nan'))
        while self.t and now - self.t[0] > HISTORY_S:
            self.t.popleft()
            for d in self.acc + self.gyro:
                d.popleft()
        self.render_text()

    def render_text(self):
        s = self.sen
        if s.get('imu_valid'):
            for i in range(3):
                self.acc_l[i].setText(sfmt(s['acc'][i], 3))
                self.gyro_l[i].setText(sfmt(s['gyro'][i], 1))
            self.att['temp'].set(fmt(s.get('imu_temp_c'), 1))
            ax, ay, az = s['acc']
            self.att['roll'].set(sfmt(math.degrees(math.atan2(ay, az)), 1))
            self.att['pitch'].set(sfmt(math.degrees(math.atan2(-ax, math.hypot(ay, az))), 1))
        if s.get('mag_valid'):
            for i in range(3):
                self.mag_l[i].setText(sfmt(s['mag'][i], 3))
            self.compass.set(s.get('heading_deg'))             # fixed dial, the needle is the nose
        if s.get('baro_valid'):
            self.baro['p'].set(fmt(s['pressure_pa'] / 100, 1))
            self.baro['t'].set(fmt(s.get('baro_temp_c'), 1))
            self.baro['alt'].set(fmt(s.get('baro_alt_std_m'), 1))
            self.baro['rel'].set(sfmt(s.get('baro_alt_m'), 2))
        if s.get('tof_valid'):
            m, st = s['tof_mm'] / 1000, s.get('tof_status')
            ok = st == 0
            self.tof.set(m, ok)
            band = ' · in the hover band' if ok and HOVER_BAND_M[0] <= m <= HOVER_BAND_M[1] else ''
            self.tof_status.setText('status %s: %s%s' % (st, TOF_STATUS.get(st, 'other'), band))

    def frame(self):
        """Frame timer: the plots scroll even when no packet arrives, and the
        two notices follow the link and the stream's age."""
        t = list(self.t)
        self.plot_acc.set_data(t, [list(d) for d in self.acc])
        self.plot_gyro.set_data(t, [list(d) for d in self.gyro])
        self.plot_acc.refresh()
        self.plot_gyro.refresh()
        stale = self.seen and time.monotonic() - self.sen_at > SEN_STALE_S
        self._show(self.missing, 0, self.alive and not self.demo and not self.seen)
        self._show(self.stale, 1, self.alive and stale)

    def _show(self, w, k, on):
        if w.isVisible() != on and (on or not w.isHidden()):
            w.setVisible(on)
            self._notice_gaps[k].spacerItem().changeSize(0, 10 if on else 0)
            self.lay.invalidate()

    def compact(self, on):
        self.compass.set_max_width(112 if on else 140)
        self.tof.set_max_width(140 if on else 170)


# ------------------------------------------------------------------- dock

class ConsoleInput(QLineEdit):
    """The command line, with ↑/↓ through the last 50 commands."""

    def __init__(self):
        super().__init__()
        self.history, self.index = [], -1
        self.setPlaceholderText('help')

    def remember(self, line):
        self.history.append(line)
        del self.history[:-CON_HISTORY]
        self.index = -1

    def keyPressEvent(self, e):
        h = self.history
        if e.key() == Qt.Key.Key_Up and h:
            self.index = len(h) - 1 if self.index < 0 else max(0, self.index - 1)
            self.setText(h[self.index])
        elif e.key() == Qt.Key.Key_Down and h:
            if self.index < 0:
                return
            self.index = min(len(h), self.index + 1)
            self.setText('' if self.index == len(h) else h[self.index])
            if self.index == len(h):
                self.index = -1
        else:
            super().keyPressEvent(e)


class Dock(ChamferFrame):
    """The console and the event log as two tabs under the panels. The
    window's splitter handle above it sets its height; the button on the bar
    collapses it to the bar and the panels take the room (MainWindow.
    _apply_dock_h). Height, tab and collapsed state are kept in QSettings by
    the window."""
    collapsed_changed = pyqtSignal(bool)
    NOTES = {'console': "The firmware's %s prompt, over the air: one command at a time; the chips under it send "
                        "the common ones." % code('i2c-tools&gt;'),
             'events': 'Link, failsafe, setpoint and camera events, the newest at the bottom.'}

    def __init__(self, station):
        super().__init__()
        self.station = station
        self.tab = 'console'
        self.collapsed = False
        self.busy = False
        self._busy_token = 0
        self.events = collections.deque()
        self.counts = {'crit': 0, 'warn': 0}

        self.t_console = DockTab('Console')
        self.t_events = DockTab('Events')
        self.t_events.set_badge('0 crit · 0 warn', T.FAINT)
        self.t_console.clicked.connect(lambda: self.show_tab('console'))
        self.t_events.clicked.connect(lambda: self.show_tab('events'))
        self.note = InfoIcon()
        self.toggle = QPushButton('▾')
        self.toggle.setProperty('kind', 'dock-toggle')
        self.toggle.setFocusPolicy(Qt.FocusPolicy.TabFocus)
        self.toggle.setToolTip('hide the console and events, and give the panels the room')
        self.toggle.clicked.connect(lambda: self.set_collapsed(not self.collapsed))
        self.bar = QWidget()
        self.bar.setLayout(hbox(self.t_console, 2, self.t_events, 8, self.note, None, self.toggle, spacing=0,
                                margins=(16, 4, 16, 0)))

        # console
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setMaximumBlockCount(5000)
        self.out.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.inp = ConsoleInput()
        self.inp.returnPressed.connect(self._submit)
        self.send_b = button('Send', 'primary')
        self.send_b.clicked.connect(self._submit)
        chips = FlowBox(hgap=6, vgap=6)
        for q in QUICK:
            b = QPushButton(q)
            b.setProperty('kind', 'chip')
            b.setFocusPolicy(Qt.FocusPolicy.TabFocus)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.clicked.connect(lambda _=False, q=q: self.send(q))
            chips.add(b)
        con = QWidget()
        cl = QVBoxLayout(con)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(8)
        cl.addWidget(self.out, 1)
        cl.addLayout(hbox(label('i2c-tools>', 12, color=T.UN_GREEN, mono=True), self.inp, self.send_b, spacing=8))
        cl.addWidget(chips)

        # events
        self.ev_box = QWidget()
        self.ev_lay = QVBoxLayout(self.ev_box)
        self.ev_lay.setContentsMargins(0, 0, 0, 0)
        self.ev_lay.setSpacing(0)
        self.ev_lay.addStretch(1)
        self.ev_scroll = scroll(self.ev_box)
        self.ev_scroll.verticalScrollBar().rangeChanged.connect(
            lambda lo, hi: self.ev_scroll.verticalScrollBar().setValue(hi))

        self.stack = QStackedWidget()
        self.stack.addWidget(con)
        self.stack.addWidget(self.ev_scroll)
        self.body = QWidget()
        bl = QVBoxLayout(self.body)
        bl.setContentsMargins(16, 8, 16, 12)
        bl.addWidget(self.stack)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 2, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(self.bar)
        lay.addWidget(self.body, 1)
        self.show_tab('console', focus=False)

    def bar_height(self):
        return 2 + self.bar.sizeHint().height() + 4

    # tabs and collapse

    def show_tab(self, name, focus=True):
        if self.collapsed:
            self.set_collapsed(False)
        self.tab = name
        self.t_console.setChecked(name == 'console')
        self.t_events.setChecked(name == 'events')
        self.stack.setCurrentIndex(0 if name == 'console' else 1)
        self.note.setText(self.NOTES[name])
        if name == 'events':
            self.t_events.set_new(False)
            sb = self.ev_scroll.verticalScrollBar()
            sb.setValue(sb.maximum())
        elif focus:
            self.inp.setFocus()

    def set_collapsed(self, on):
        if on == self.collapsed:
            return
        self.collapsed = on
        self.body.setVisible(not on)
        self.toggle.setText('▴' if on else '▾')
        self.toggle.setToolTip('show the console and events' if on else
                               'hide the console and events, and give the panels the room')
        if not on and self.tab == 'events':
            self.t_events.set_new(False)
        self.collapsed_changed.emit(on)

    # console

    def send(self, line):
        line = line.strip()
        if not line:
            return
        if self.busy:
            self.write('[a command is still running on the blimp]\n')
            return
        self.busy = True
        self.send_b.setEnabled(False)
        self.inp.remember(line)
        self.station.console(line)
        self._busy_token += 1
        token = self._busy_token
        QTimer.singleShot(CON_TIMEOUT_MS, lambda: self._done() if self._busy_token == token else None)

    def _submit(self):
        self.send(self.inp.text())
        self.inp.clear()

    def write(self, text):
        sb = self.out.verticalScrollBar()
        cur = self.out.textCursor()
        cur.movePosition(cur.MoveOperation.End)
        cur.insertText(text)
        sb.setValue(sb.maximum())

    def _done(self):
        self.busy = False
        self.send_b.setEnabled(True)

    def con_end(self):
        self._busy_token += 1
        self._done()

    # events

    def add_event(self, ev):
        self.events.append(ev)
        if ev['level'] in self.counts:
            self.counts[ev['level']] += 1
        self.ev_lay.insertWidget(self.ev_lay.count() - 1, EventRow(ev))
        while len(self.events) > EVENTS_MAX:
            old = self.events.popleft()
            if old['level'] in self.counts:
                self.counts[old['level']] -= 1
            w = self.ev_lay.takeAt(0).widget()
            w.deleteLater()
        nc, nw = self.counts['crit'], self.counts['warn']
        self.t_events.set_badge('%d crit · %d warn' % (nc, nw), T.UN_RED if nc else T.AMBER if nw else T.FAINT)
        # the tab is not in view: the dot says there is something to read
        if self.tab != 'events' or self.collapsed:
            self.t_events.set_new(True)


def load_dock(settings, dock):
    """The dock's tab and collapsed state from QSettings; its height is the window's."""
    dock.show_tab('events' if settings.value('dock/tab') == 'events' else 'console', focus=False)
    dock.set_collapsed(settings.value('dock/collapsed', False, type=bool))


def save_dock(settings, dock):
    settings.setValue('dock/tab', dock.tab)
    settings.setValue('dock/collapsed', dock.collapsed)
