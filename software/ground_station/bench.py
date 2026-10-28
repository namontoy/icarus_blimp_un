"""The Bench page: the blimp on the bench, beside the flight page.

Where the tests, calibrations and checks live, as against the flight page's
live view: the things that take a minute and leave a file behind. Today that
is the capture (the control loop's records, recorded on the blimp and fetched
over the link; capture.c and docs/control-loop-plan.md D7, D8) with a trace
viewer over the result. The page is a row of panels so that a magnetometer
calibration from a capture, a parameter editor and the loop's timing can
join it as panels of their own (increments 4 and 5 of the plan).

The panels only ask the station; the station's capture thread talks to the
blimp and signals back (cap for state and progress, cap_data for a decoded
capture), so nothing here blocks on the network.
"""
import math
import os

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QComboBox, QDoubleSpinBox, QFileDialog, QLineEdit, QProgressBar, QSpinBox, QWidget

from . import theme as T
from .panels import Panel, button, code, h3
from .widgets import ElidedLabel, FlowBox, Readout, TracePlot, hbox, isnum, label, set_color

import blimp_log

MOTORS = ('FL', 'RL', 'FR', 'RR')           # wire order (motor_id_t)
MAG_CHIP = {0: 'no mag', 1: 'HMC5883L', 2: 'QMC5883L'}
BARO_CHIP = {0: 'no baro', 1: 'BMP180', 2: 'BMP280', 3: 'BME280'}

# What the trace viewer can show: (title, unit, digits, [(series, row key, scale)]).
# The keys are blimp_log.to_units() columns.
GROUPS = (
    ('Accelerometer', 'g', 3, [(a, 'acc_%s_g' % a, 1) for a in 'xyz']),
    ('Gyroscope', '°/s', 2, [(a, 'gyro_%s_dps' % a, 1) for a in 'xyz']),
    ('Magnetometer', 'gauss', 3, [(a, 'mag_%s_gauss' % a, 1) for a in 'xyz']),
    ('Pressure', 'hPa', 2, [('p', 'pressure_pa', 0.01)]),
    ('Barometer temperature', '°C', 2, [('T', 'baro_temp_c', 1)]),
    ('Humidity', '%RH', 1, [('h', 'humidity_pct', 1)]),
    ('Range below', 'mm', 0, [('ToF', 'tof_mm', 1)]),
    ('Throttle readback', '−1 … +1', 3, [(m, 'throttle_%s' % m, 1) for m in MOTORS]),
    ('Setpoint', '−1 … +1', 3, [(m, 'setpoint_%s' % m, 1) for m in MOTORS]),
    ('Loop execution time', 'µs', 0, [('exec', 'exec_us', 1)]),
    ('Tick step', 'ms', 3, [('dt', 'dt_ms', 1)]),
    ('Battery', 'V', 3, [('bat', 'bat_v', 1)]),
)


# ---------------------------------------------------------------- capture

class CapturePanel(Panel):
    """Length, Start / Stop / Download / Clear, the blimp's state with a
    progress bar, the figures of the last LOG_INFO, notes for the file."""
    HINT = ('The blimp writes one 72-byte record per 20 ms tick of its control loop into PSRAM: IMU, magnetometer '
            'and barometer, the range below, the motor readback, the loop\'s execution time and which sensors '
            'were fresh that tick. Start, wait, Download: the records come over the UDP link in chunks, the '
            'window asks again for any that were lost, and the capture is saved under %s as a .bin with a .json '
            'beside it. Opening the USB port would reset the blimp and lose its buffer, so a capture is always '
            'fetched over the air.' % code('captures/'))

    def __init__(self, station):
        super().__init__('Capture', "the loop's records, one per 20 ms, kept on the blimp until fetched")
        self.station = station
        self.info, self.phase, self.file, self.error = None, 'idle', None, ''
        self.data = None                    # the last decoded capture (for the CSV export)

        self.seconds = QSpinBox()
        self.seconds.setRange(1, 600)
        self.seconds.setValue(60)
        self.seconds.setKeyboardTracking(False)
        self.seconds.setFixedWidth(72)
        self.seconds.setToolTip("length of the next capture, up to the blimp's buffer")
        # The blimp's buffer bounds the length (LOG_INFO.max_seconds). Until
        # it has answered once, the box allows anything and the blimp refuses
        # what does not fit; it never shortens a capture (2026-10-28).
        self.max_l = label('buffer size: ask the blimp', 11, color=T.FAINT)
        self.b_start = button('Start', 'primary', 'record a new capture on the blimp; the previous one there is lost')
        self.b_stop = button('Stop', tip='end the capture now; what was recorded stays on the blimp')
        self.b_down = button('Download', tip='fetch the records over the link and save them under captures/')
        self.b_clear = button('Clear', tip="discard the blimp's records")
        self.b_csv = button('Export CSV…', tip='the downloaded capture, every record in units')
        for b, a in ((self.b_start, 'start'), (self.b_stop, 'stop'), (self.b_down, 'download'),
                     (self.b_clear, 'clear')):
            b.clicked.connect(lambda _=False, a=a: self.act(a))
        self.b_csv.clicked.connect(self.export_csv)
        self.notes = QLineEdit()
        self.notes.setPlaceholderText('what this capture is: "at rest on the desk", "FL at 0.2"…')
        self.notes.setToolTip('saved in the .json beside the capture')

        self.lay.addLayout(hbox(label('Length', color=T.MUTED), self.seconds, label('s', color=T.MUTED), 10,
                                self.max_l, None, spacing=6))
        self.lay.addSpacing(8)
        row = FlowBox(hgap=8, vgap=8)
        for b in (self.b_start, self.b_stop, self.b_down, self.b_clear):
            row.add(b)
        self.lay.addWidget(row)
        self.lay.addSpacing(14)
        self.state_l = label('—', 13, 600, mono=True, wrap=True)
        self.bar = QProgressBar()
        self.bar.setRange(0, 1000)
        self.bar.setValue(0)
        self.bar.setTextVisible(False)
        self.bar.setFixedHeight(8)
        self.lay.addWidget(h3('On the blimp'))
        self.lay.addSpacing(4)
        self.lay.addWidget(self.state_l)
        self.lay.addSpacing(6)
        self.lay.addWidget(self.bar)
        self.lay.addSpacing(8)
        self.ro = {'records': Readout('Records'), 'length': Readout('Length', 's'), 'buffer': Readout('Buffer', 's'),
                   'fw': Readout('Firmware')}
        for k in ('records', 'length', 'buffer', 'fw'):
            self.lay.addWidget(self.ro[k])
        # The scales line (ranges and both chips) is too long for a readout's
        # value column at 18 px, so it gets a row of its own in console size
        # that wraps instead of widening the panel (2026-10-28).
        self.lay.addSpacing(6)
        self.lay.addWidget(label('Scales', 11, color=T.FAINT, spacing=0.08, caps=True))
        self.lay.addSpacing(2)
        self.scales_l = label('—', 12, 500, mono=True, wrap=True)
        self.lay.addWidget(self.scales_l)
        self.lay.addSpacing(12)
        self.lay.addWidget(h3('Notes'))
        self.lay.addSpacing(4)
        self.lay.addWidget(self.notes)
        self.lay.addSpacing(10)
        self.lay.addLayout(hbox(self.b_csv, None))
        self.lay.addSpacing(6)
        self.file_l = ElidedLabel('no capture downloaded yet')
        self.file_l.setFont(T.font(11, mono=True))
        set_color(self.file_l, T.FAINT)
        self.lay.addWidget(self.file_l)
        self.lay.addSpacing(12)
        self.lay.addWidget(label(self.HINT, 12, color=T.FAINT, wrap=True))
        self.lay.addStretch(1)
        self.render()

    # station out

    def act(self, action):
        if action == 'download':
            self.station.capture('download', notes=self.notes.text().strip())
        else:
            self.station.capture(action, seconds=self.seconds.value())
        self.phase = 'busy'
        self.render()

    def export_csv(self):
        if not self.data:
            return
        stem = os.path.splitext(self.data['file'])[0] if self.data.get('file') else 'capture'
        path, _ = QFileDialog.getSaveFileName(self, 'Export the capture as CSV', stem + '.csv', 'CSV (*.csv)')
        if not path:
            return
        n = blimp_log.write_csv(path, self.data['header'], self.data['records'])
        self.station.log('ok', 'capture: %d rows written to %s' % (n, path))

    # station in

    def on_cap(self, d):
        self.phase = d.get('phase', 'status')
        if d.get('info') is not None:
            self.info = d['info']
        self.error = d.get('error', '')
        if self.phase == 'download':
            self.bar.setValue(int(1000 * d.get('progress', 0.0)))
        elif self.phase == 'saved':
            self.file = d.get('file')
            self.bar.setValue(1000)
        self.render()

    def on_data(self, d):
        self.data = d
        self.render()

    def render(self):
        i = self.info
        running = bool(i) and i['state'] == 1
        stored = bool(i) and i['count'] > 0
        busy = self.phase in ('busy', 'download')
        hz = 1e6 / i['period_us'] if i and i['period_us'] else 50.0
        if self.phase == 'error':
            text, color = 'ERROR  %s' % self.error, T.UN_RED
        elif self.phase == 'busy':
            text, color = 'waiting for the blimp…', T.MUTED
        elif self.phase == 'download':
            text, color = 'downloading %d records…' % i['count'], T.PAPAYA
        elif i is None:
            text, color = 'not asked yet', T.FAINT
        elif i['result'] == 1:
            text, color = 'the blimp has no capture buffer (PSRAM?)', T.UN_RED
        elif running:
            text, color = 'RECORDING  %.1f of %.0f s' % (i['count'] / hz, i['wanted'] / hz), T.PAPAYA
        elif stored:
            text, color = 'DONE  %d records, %.1f s  ·  %s' % (
                i['count'], i['count'] / hz, 'saved' if self.phase == 'saved' else 'not downloaded'), T.UN_GREEN
        else:
            text, color = 'idle, nothing stored on the blimp', T.MUTED
        self.state_l.setText(text)
        set_color(self.state_l, color)
        if running:
            self.bar.setValue(int(1000 * i['count'] / max(1, i['wanted'])))
        elif self.phase not in ('download', 'saved'):
            self.bar.setValue(1000 if stored else 0)
        self.ro['records'].set(str(i['count']) if i else '—')
        self.ro['length'].set('%.1f' % (i['count'] / hz) if i else '—')
        self.ro['buffer'].set(str(i['max_seconds']) if i else '—')
        self.scales_l.setText('±%d g · ±%d °/s · %s · %s' % (i['imu_accel_g'], i['imu_gyro_dps'],
                                                            MAG_CHIP.get(i['mag_chip'], '?'),
                                                            BARO_CHIP.get(i['baro_chip'], '?')) if i else '—')
        self.ro['fw'].set((i['fw'] or '?') if i else '—')
        if i and i['max_seconds'] > 0:
            self.seconds.setMaximum(i['max_seconds'])
            self.max_l.setText('up to %d s on this blimp' % i['max_seconds'])
        self.file_l.setText(self.file or 'no capture downloaded yet')
        self.b_start.setEnabled(not busy and not running and (i is None or i['result'] != 1))
        self.b_stop.setEnabled(running and not busy)
        self.b_down.setEnabled(bool(i) and (stored or running) and not busy)
        self.b_clear.setEnabled(stored and not running and not busy)
        self.b_csv.setEnabled(self.data is not None)


# ----------------------------------------------------------------- traces

class TracePanel(Panel):
    """One group of the downloaded capture at a time, over a time window,
    with blimp_log's summary under it."""

    def __init__(self):
        super().__init__('Traces', 'download a capture to see it here')
        self.header, self.rows = None, []
        self.group = QComboBox()
        for g in GROUPS:
            self.group.addItem(g[0])
        self.group.currentIndexChanged.connect(self.replot)
        self.from_s, self.to_s = QDoubleSpinBox(), QDoubleSpinBox()
        for sb in (self.from_s, self.to_s):
            sb.setDecimals(1)
            sb.setSingleStep(1.0)
            sb.setRange(0.0, 0.0)
            sb.setKeyboardTracking(False)
            sb.setFixedWidth(76)
            sb.valueChanged.connect(self._window)
        self.plot = TracePlot()
        self.summary = label('', 12, mono=True, color=T.MUTED, wrap=True)
        self.summary.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.lay.addLayout(hbox(label('Show', color=T.MUTED), self.group, 16, label('from', color=T.MUTED),
                                self.from_s, label('to', color=T.MUTED), self.to_s, label('s', color=T.MUTED), None,
                                spacing=6))
        self.lay.addSpacing(8)
        self.lay.addWidget(self.plot, 1)
        self.lay.addSpacing(8)
        self.lay.addWidget(h3('Summary'))
        self.lay.addSpacing(4)
        self.lay.addWidget(self.summary)
        self.summary.setText('Nothing downloaded yet. The summary gives the record count, the tick step and its '
                             'jitter, the execution time, how often each sensor was fresh, and the mean and '
                             'spread of every channel.')

    def on_data(self, d):
        self.header, self.rows = d['header'], d['rows']
        end = self.rows[-1]['t_s'] if self.rows else 0.0
        for sb in (self.from_s, self.to_s):
            sb.blockSignals(True)
            sb.setRange(0.0, max(end, 0.1))
        self.from_s.setValue(0.0)
        self.to_s.setValue(max(end, 0.1))
        for sb in (self.from_s, self.to_s):
            sb.blockSignals(False)
        self.summary.setText(blimp_log.format_summary(d['summary']))
        self.note.setText('%s · %d records · hover for the values at a sample' % (
            os.path.basename(d['file']) if d.get('file') else 'capture', len(self.rows)))
        self.replot()

    def replot(self):
        if not self.rows:
            self.plot.set_data([], [])
            return
        _, unit, digits, specs = GROUPS[self.group.currentIndex()]
        t = [r['t_s'] for r in self.rows]
        series = []
        for i, (name, key, scale) in enumerate(specs):
            vals = [r[key] * scale if isnum(r[key]) else math.nan for r in self.rows]
            series.append((name, vals, T.SERIES4[i % len(T.SERIES4)]))
        self.plot.set_data(t, series, unit, digits)
        self._window()

    def _window(self):
        a, b = self.from_s.value(), self.to_s.value()
        if b <= a:
            b = a + 0.1
        self.plot.set_window(a, b)


# ------------------------------------------------------------------- page

class BenchPage(QWidget):
    def __init__(self, station):
        super().__init__()
        self.capture = CapturePanel(station)
        self.capture.setFixedWidth(360)
        self.traces = TracePanel()
        self.setLayout(hbox(self.capture, self.traces, spacing=12, margins=(0, 12, 0, 0)))
        self.layout().setStretchFactor(self.traces, 1)
        self.setMinimumHeight(200)

    def on_data(self, d):
        self.traces.on_data(d)
        self.capture.on_data(d)
