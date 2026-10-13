"""ESP-Drone bench GUI: Wi-Fi connect/disconnect, 4 motor PWM sliders, MPU-6050 view.

REMOVE THE PROPELLERS before moving the sliders.

Uses the same direct cflib param writes as motor_diag.py (motorPowerSet.*), which
avoids the cfclient Parameters tab stalling on MISC_GET_DEFAULT_VALUE.
The Wi-Fi part uses nmcli (NetworkManager). Close cfclient before running.

Run in the blimp-tools conda env (see software/SETUP.md):
    python software/drone_gui.py

Space or Esc = STOP (all motors to 0).
"""
import math
import os
import subprocess
import sys
import threading
import time
from collections import deque

import cflib.crtp
import pyqtgraph as pg
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crtp.udpdriver import UdpDriver
from PyQt6.QtCore import QObject, QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import (QBrush, QColor, QFont, QKeySequence, QPainter, QPen, QPolygonF,
                         QShortcut)
from PyQt6.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QDoubleSpinBox, QFrame,
                             QGridLayout, QGroupBox, QHBoxLayout, QLabel, QMainWindow,
                             QProgressBar, QPushButton, QRadioButton, QSlider, QVBoxLayout,
                             QWidget)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # works from any cwd
from drone_dynamics import (MOTOR_LAYOUT, describe, max_tendency,  # noqa: E402
                            planar_tendency, yaw_tendency)

URI = 'udp://192.168.43.42:2390'
CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cfcache')
SSID_PREFIX = 'ESP-DRONE'
WIFI_PASSWORD = '12345678'      # CONFIG_WIFI_PASSWORD
DRONE_IP_PREFIX = '192.168.43.'
HISTORY = 200                   # 10 s at 50 ms
DEFAULT_LIMIT_PCT = 30.0
MOTORS = (1, 2, 3, 4)

GREY, ORANGE, GREEN, RED = '#888888', '#d98a00', '#1a9c3c', '#d02020'
RX_TIMEOUT_S = 2.0              # linked but no log data for this long -> link lost
CONNECT_TIMEOUT_S = 15.0        # Wi-Fi up but CRTP setup never finished
STAB_MIN_HZ = 100               # stabilizer.rtStab below this = stabilizer loop stalled
STAB_GRACE_S = 2.5              # rtStab is a 1 s rate counter; ignore it right after connect
PM_STATES = {0: 'En bateria', 1: 'Cargando', 2: 'Cargada', 3: 'Bateria baja',
             4: 'Apagando'}     # PMStates, hal/interface/pm_esplane.h
PM_LOW_POWER = 3


# cflib's UdpDriver binds the fixed port 2399 and blocks in recvfrom() with no
# timeout. On Linux close() does not wake a recvfrom blocked in another thread, so
# the port stays bound and the next connect fails with EADDRINUSE. A receive
# timeout lets the reader thread notice the closed link and release the socket.
_udp_connect = UdpDriver.connect


def _udp_connect_with_timeout(self, *args, **kwargs):
    _udp_connect(self, *args, **kwargs)
    self.socket.settimeout(0.2)


UdpDriver.connect = _udp_connect_with_timeout


def nmcli(*args, timeout=20):
    r = subprocess.run(['nmcli', *args], capture_output=True, text=True, timeout=timeout)
    return r.returncode, r.stdout.strip(), r.stderr.strip()


def wifi_iface():
    _, out, _ = nmcli('-t', '-f', 'DEVICE,TYPE', 'device')
    for line in out.splitlines():
        dev, _, typ = line.partition(':')
        if typ == 'wifi':
            return dev
    return None


def active_ssid(iface):
    _, out, _ = nmcli('-t', '-f', 'ACTIVE,SSID', 'dev', 'wifi', 'list', 'ifname', iface)
    for line in out.splitlines():
        if line.startswith('yes:'):
            return line[4:]
    return None


class WifiWorker(QObject):
    """Runs nmcli off the GUI thread. Signals carry the attempt id so the GUI can
    ignore results of an attempt the user already cancelled."""
    status = pyqtSignal(int, str, str)  # attempt, text, color
    ready = pyqtSignal(int, str)        # attempt, ssid: Wi-Fi associated and IP obtained
    failed = pyqtSignal(int, str)       # attempt, message

    def __init__(self):
        super().__init__()
        self.iface = None
        self.ssid = None
        self.we_connected = False

    def connect(self, attempt):
        threading.Thread(target=self._connect, args=(attempt,), daemon=True).start()

    def _connect(self, attempt):
        try:
            self.status.emit(attempt, 'Buscando Wi-Fi...', ORANGE)
            self.iface = wifi_iface()
            if not self.iface:
                raise RuntimeError('No hay interfaz Wi-Fi')
            cur = active_ssid(self.iface)
            if cur and cur.startswith(SSID_PREFIX):
                self.ssid = cur
            else:
                _, out, _ = nmcli('-t', '-f', 'SSID,SIGNAL', 'dev', 'wifi', 'list',
                                  'ifname', self.iface, '--rescan', 'yes', timeout=30)
                found = []
                for line in out.splitlines():
                    ssid, _, sig = line.rpartition(':')
                    if ssid.startswith(SSID_PREFIX) and sig.isdigit():
                        found.append((int(sig), ssid))
                if not found:
                    raise RuntimeError(f'No se encontro ninguna red {SSID_PREFIX}_*')
                self.ssid = max(found)[1]
                self.status.emit(attempt, f'Asociando a {self.ssid}...', ORANGE)
                rc, _, err = nmcli('dev', 'wifi', 'connect', self.ssid, 'password',
                                   WIFI_PASSWORD, 'ifname', self.iface, timeout=45)
                if rc != 0:
                    raise RuntimeError(err or 'nmcli connect fallo')
                self.we_connected = True
                nmcli('connection', 'modify', 'id', self.ssid, 'connection.autoconnect', 'no')
            for _ in range(20):
                _, ip, _ = nmcli('-g', 'IP4.ADDRESS', 'dev', 'show', self.iface)
                if ip.startswith(DRONE_IP_PREFIX):
                    break
                time.sleep(0.5)
            else:
                raise RuntimeError(f'Sin IP {DRONE_IP_PREFIX}x en {self.iface}')
            self.ready.emit(attempt, self.ssid)
        except Exception as e:  # noqa: BLE001
            self.failed.emit(attempt, str(e))

    def drop(self):
        """Blocking: bring our Wi-Fi connection down (only if we created it)."""
        if self.we_connected and self.ssid:
            nmcli('connection', 'down', 'id', self.ssid)
        self.we_connected = False


class DroneLink(QObject):
    connected = pyqtSignal()
    lost = pyqtSignal(str)
    info = pyqtSignal(str, str)         # param name, value

    def __init__(self):
        super().__init__()
        self.cf = None
        self.lock = threading.Lock()
        self.imu = deque(maxlen=HISTORY)    # (t, ax, ay, az, gx, gy, gz)
        self.att = {}
        self.pwm = {}
        self.pm = {}
        self.ready = False              # param TOC loaded: set_value will not block
        self.last_rx = time.monotonic()

    def open(self):
        self.ready = False
        self.last_rx = time.monotonic()
        with self.lock:
            self.imu.clear()
            self.att, self.pwm, self.pm = {}, {}, {}
        self.cf = Crazyflie(rw_cache=CACHE_DIR)
        self.cf.fully_connected.add_callback(self._fully_connected)
        self.cf.connection_failed.add_callback(lambda uri, msg: self.lost.emit(msg))
        self.cf.connection_lost.add_callback(lambda uri, msg: self.lost.emit(msg))
        self.cf.open_link(URI)

    def close(self):
        """Blocking (~1.5 s): zero the motors and close the link. Call off the GUI thread."""
        cf, was_ready = self.cf, self.ready
        self.cf = None
        self.ready = False
        if cf is None:
            return
        try:
            if was_ready:
                self._zero(cf)
                time.sleep(0.2)
            cf.close_link()
        except Exception:  # noqa: BLE001
            pass

    def _fully_connected(self, _uri):
        self.last_rx = time.monotonic()
        self.ready = True
        threading.Thread(target=self._read_info, daemon=True).start()
        self._start_logs()
        self.connected.emit()

    def _read_info(self):
        for name in ('system.selftestPassed', 'imu_tests.mpu6050'):
            try:
                self.info.emit(name, str(self.cf.param.get_value(name)))
            except Exception as e:  # noqa: BLE001
                self.info.emit(name, f'err: {e}')

    def _start_logs(self):
        def make(name, period, variables, cb):
            conf = LogConfig(name=name, period_in_ms=period)
            for v, t in variables:
                conf.add_variable(v, t)
            conf.data_received_cb.add_callback(cb)
            conf.error_cb.add_callback(lambda lc, msg: print(f'log {lc.name}: {msg}'))
            self.cf.log.add_config(conf)
            conf.start()

        make('imu', 50, [(f'acc.{a}', 'float') for a in 'xyz'] +
             [(f'gyro.{a}', 'float') for a in 'xyz'], self._on_imu)
        make('att', 50, [('stabilizer.roll', 'float'), ('stabilizer.pitch', 'float'),
                         ('stabilizer.yaw', 'float'), ('sys.armed', 'int8_t'),
                         ('sys.canfly', 'int8_t'), ('stabilizer.rtStab', 'float')],
             self._on_att)
        make('pwm', 100, [(f'pwm.m{m}_pwm', 'uint32_t') for m in MOTORS], self._on_pwm)
        make('pm', 500, [('pm.vbat', 'float'), ('pm.batteryLevel', 'uint8_t'),
                         ('pm.state', 'int8_t')], self._on_pm)

    def _on_imu(self, _ts, d, _lc):
        self.last_rx = time.monotonic()
        with self.lock:
            self.imu.append((time.monotonic(), d['acc.x'], d['acc.y'], d['acc.z'],
                             d['gyro.x'], d['gyro.y'], d['gyro.z']))

    def _on_att(self, _ts, d, _lc):
        self.last_rx = time.monotonic()
        with self.lock:
            self.att = dict(d)

    def _on_pwm(self, _ts, d, _lc):
        self.last_rx = time.monotonic()
        with self.lock:
            self.pwm = dict(d)

    def _on_pm(self, _ts, d, _lc):
        self.last_rx = time.monotonic()
        with self.lock:
            self.pm = dict(d)

    def snapshot(self):
        with self.lock:
            return list(self.imu), dict(self.att), dict(self.pwm), dict(self.pm)

    def _set(self, name, value):
        cf = self.cf
        if cf is None or not self.ready:
            return
        try:
            cf.param.set_value(name, str(value))
        except Exception as e:  # noqa: BLE001
            print(f'set {name}: {e}')

    def set_motor(self, m, value):
        self._set(f'motorPowerSet.m{m}', value)

    def set_enable(self, on):
        self._set('motorPowerSet.enable', 1 if on else 0)

    @staticmethod
    def _zero(cf):
        for m in MOTORS:
            cf.param.set_value(f'motorPowerSet.m{m}', '0')
        cf.param.set_value('motorPowerSet.enable', '0')

    def stop_all(self):
        for m in MOTORS:
            self.set_motor(m, 0)
        self.set_enable(False)


def duty_color(pct):
    if not pct:
        return QColor(GREY)
    return QColor(GREEN if pct < 30 else ORANGE if pct < 60 else RED)


class DroneView(QWidget):
    """Top view of the quad: per motor, the applied duty and a rotation arrow; at the
    centre, the open-loop XY and yaw tendency computed by drone_dynamics."""
    CAPTION_H = 40
    PLANAR_COLOR, YAW_COLOR = QColor('#1f6feb'), QColor('#8250df')
    MIN_TENDENCY = 0.01     # below this the arrows are hidden (PWM rounding noise)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(300, 330 + self.CAPTION_H)
        # Arrow lengths are drawn relative to the largest tendency reachable within the
        # slider limit, so they stay readable at bench duties; captions stay absolute.
        self.full_scale = max_tendency(DEFAULT_LIMIT_PCT)
        self.actual = {}        # motor -> applied duty % (from pwm log), missing = unknown
        self.requested = {}     # motor -> slider %

    def set_values(self, actual, requested):
        self.actual, self.requested = dict(actual), dict(requested)
        self.update()

    def set_duty_limit(self, limit_pct):
        self.full_scale = max_tendency(limit_pct)
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        s = min(self.width(), self.height() - self.CAPTION_H)
        ox, oy = (self.width() - s) / 2, (self.height() - self.CAPTION_H - s) / 2
        cx, cy = ox + s / 2, oy + s / 2 + s * 0.03
        arm, disc, ring = s * 0.25, s * 0.10, s * 0.135
        fg = self.palette().windowText().color()

        p.setPen(fg)
        p.setFont(QFont(self.font().family(), 10, QFont.Weight.Bold))
        p.drawText(QRectF(ox, oy, s, s * 0.08), Qt.AlignmentFlag.AlignCenter, 'FRENTE ▲')

        centers = {m: QPointF(cx + sx * arm, cy + sy * arm)
                   for m, (sx, sy, _) in MOTOR_LAYOUT.items()}
        p.setPen(QPen(QColor('#666666'), s * 0.03, cap=Qt.PenCapStyle.RoundCap))
        p.drawLine(centers[1], centers[3])
        p.drawLine(centers[2], centers[4])
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor('#444444'))
        p.drawRoundedRect(QRectF(cx - s * 0.08, cy - s * 0.1, s * 0.16, s * 0.2), 6, 6)

        for m, (_, _, rot) in MOTOR_LAYOUT.items():
            c = centers[m]
            act = self.actual.get(m)
            color = duty_color(act)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(self.palette().window())      # opaque base so the arm is hidden
            p.drawEllipse(c, disc, disc)
            p.setPen(QPen(QColor('#555555'), 1.5))
            p.setBrush(QBrush(QColor(color.red(), color.green(), color.blue(), 45)))
            p.drawEllipse(c, disc, disc)
            p.setPen(fg)
            p.setFont(QFont(self.font().family(), 11, QFont.Weight.Bold))
            p.drawText(QRectF(c.x() - disc, c.y() - disc * 0.75, 2 * disc, disc),
                       Qt.AlignmentFlag.AlignCenter, f'M{m}')
            p.setFont(QFont(self.font().family(), 8))
            p.drawText(QRectF(c.x() - disc, c.y() + disc * 0.05, 2 * disc, disc * 0.7),
                       Qt.AlignmentFlag.AlignCenter, rot)
            start, span = (135.0, -270.0) if rot == 'CW' else (45.0, 270.0)
            self._arc_arrow(p, c, ring, start, span, color, s)

            p.setPen(fg)
            p.setFont(QFont(self.font().family(), 11, QFont.Weight.Bold))
            p.drawText(QRectF(c.x() - s * 0.2, c.y() - ring - s * 0.075, s * 0.4, s * 0.07),
                       Qt.AlignmentFlag.AlignCenter, '–' if act is None else f'{act:.1f} %')
            p.setFont(QFont(self.font().family(), 8))
            p.drawText(QRectF(c.x() - s * 0.2, c.y() + ring + s * 0.005, s * 0.4, s * 0.05),
                       Qt.AlignmentFlag.AlignCenter, f'pedido {self.requested.get(m, 0)} %')

        self._draw_tendency(p, cx, cy, arm, s, ox, oy, fg)
        p.end()

    def _draw_tendency(self, p, cx, cy, arm, s, ox, oy, fg):
        """Open-loop XY arrow from the centre of mass plus a yaw arc around the body."""
        parts = []
        if self.actual:
            dx, dy, mag = planar_tendency(self.actual)
            if mag >= self.MIN_TENDENCY * self.full_scale:
                size = s * 0.035
                length = max(min(mag / self.full_scale, 1.0) * arm * 1.6, size * 1.5)
                ex, ey = cx + dx * length, cy + dy * length
                p.setPen(QPen(self.PLANAR_COLOR, s * 0.02, cap=Qt.PenCapStyle.RoundCap))
                p.drawLine(QPointF(cx, cy), QPointF(ex - dx * size, ey - dy * size))
                self._head(p, ex - dx * size, ey - dy * size, dx, dy, size, self.PLANAR_COLOR)
                parts.append(f'XY: {describe(dx, dy)} {mag * 100:.0f} %')
            yaw = yaw_tendency(self.actual)
            if abs(yaw) >= self.MIN_TENDENCY * self.full_scale:
                sweep = max(min(abs(yaw) / self.full_scale, 1.0) * 300.0, 30.0)
                # centred on the front of the body; Qt angles are CCW-positive
                start, span = (90 - sweep / 2, sweep) if yaw > 0 else (90 + sweep / 2, -sweep)
                self._arc_arrow(p, QPointF(cx, cy), s * 0.15, start, span, self.YAW_COLOR, s)
                parts.append(f"Giro: {'CCW' if yaw > 0 else 'CW'} {abs(yaw) * 100:.0f} %")
        caption = 'Tendencia  ' + '\n'.join(parts) if parts else (
            'Tendencia: equilibrado' if self.actual else 'Tendencia: sin datos')
        p.setPen(fg)
        p.setFont(QFont(self.font().family(), 9, QFont.Weight.Bold))
        p.drawText(QRectF(ox - 20, oy + s, s + 40, self.CAPTION_H),
                   Qt.AlignmentFlag.AlignCenter, caption)

    @classmethod
    def _arc_arrow(cls, p, c, r, start, span, color, s):
        """Arc of `span` degrees from `start` (Qt: CCW-positive) with an arrowhead."""
        p.setPen(QPen(color, max(2.0, s * 0.012), cap=Qt.PenCapStyle.RoundCap))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawArc(QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r), int(start * 16), int(span * 16))
        end = math.radians(start + span)
        ex, ey = c.x() + r * math.cos(end), c.y() - r * math.sin(end)
        # Direction of travel at the end of the arc (screen coordinates, y down)
        if span < 0:
            tx, ty = math.sin(end), math.cos(end)
        else:
            tx, ty = -math.sin(end), -math.cos(end)
        cls._head(p, ex, ey, tx, ty, s * 0.03, color)

    @staticmethod
    def _head(p, x, y, tx, ty, size, color):
        """Triangular arrowhead with its base centred at (x, y), pointing along (tx, ty)."""
        nx, ny = -ty, tx
        head = QPolygonF([QPointF(x + tx * size, y + ty * size),
                          QPointF(x + nx * size * 0.7, y + ny * size * 0.7),
                          QPointF(x - nx * size * 0.7, y - ny * size * 0.7)])
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(color)
        p.drawPolygon(head)


class MainWindow(QMainWindow):
    torn_down = pyqtSignal()            # background teardown finished

    def __init__(self):
        super().__init__()
        self.setWindowTitle('ESP-Drone bench')
        self.wifi = WifiWorker()
        self.link = DroneLink()
        self.busy = False           # True from Connect click until fully disconnected
        self.linked = False         # CRTP link fully up
        self.tearing = False        # teardown in progress
        self.connecting_since = None  # monotonic time CRTP setup started
        self.linked_since = None      # monotonic time the link became fully usable
        self._final = ('Desconectado', GREY)
        self.attempt = 0            # bumped on connect and teardown; stale Wi-Fi results ignored
        self.pending = {}           # motor -> value not yet sent
        self.sent = {m: 0 for m in MOTORS}
        self.controls_on = False    # motor controls usable (set by enable_controls)

        self.wifi.status.connect(self.on_wifi_status)
        self.wifi.ready.connect(self.on_wifi_ready)
        self.wifi.failed.connect(self.on_wifi_failed)
        self.torn_down.connect(self.on_torn_down)
        self.link.connected.connect(self.on_linked)
        self.link.lost.connect(self.on_failed)
        self.link.info.connect(self.on_info)

        root = QWidget()
        self.setCentralWidget(root)
        lay = QVBoxLayout(root)
        lay.addLayout(self._build_conn_bar())
        self.lbl_warn = QLabel()
        self.lbl_warn.setWordWrap(True)
        self.lbl_warn.setStyleSheet(f'background:{RED};color:white;font-weight:bold;padding:6px')
        self.lbl_warn.hide()
        lay.addWidget(self.lbl_warn)
        body = QHBoxLayout()
        body.addWidget(self._build_motors(), 0)
        view_box = QGroupBox('Vista superior')
        self.drone_view = DroneView()
        QVBoxLayout(view_box).addWidget(self.drone_view)
        body.addWidget(view_box, 0)
        body.addWidget(self._build_imu(), 1)
        lay.addLayout(body)

        QShortcut(QKeySequence(Qt.Key.Key_Space), self, activated=self.stop)
        QShortcut(QKeySequence(Qt.Key.Key_Escape), self, activated=self.stop)

        self.timer = QTimer(self, interval=50)
        self.timer.timeout.connect(self.tick)
        self.timer.start()
        self.enable_controls(False)
        self.resize(1400, 660)

    # ---- UI construction
    def _build_conn_bar(self):
        bar = QHBoxLayout()
        self.btn_conn = QPushButton('Conectar')
        self.btn_conn.clicked.connect(self.toggle_connection)
        self.lbl_status = QLabel()
        self.set_status('Desconectado', GREY)
        self.lbl_info = QLabel('')
        warn = QLabel('QUITAR HELICES')
        warn.setStyleSheet(f'color:{RED};font-weight:bold')
        self.bar_batt = QProgressBar(minimum=0, maximum=100, format='%p %')
        self.bar_batt.setFixedWidth(120)
        self.lbl_batt = QLabel()
        self._reset_battery_ui()
        bar.addWidget(self.btn_conn)
        bar.addWidget(self.lbl_status)
        bar.addWidget(self.lbl_info, 1)
        bar.addWidget(QLabel('Bateria'))
        bar.addWidget(self.bar_batt)
        bar.addWidget(self.lbl_batt)
        bar.addWidget(warn)
        return bar

    def _build_motors(self):
        box = QGroupBox('Motores (PWM)')
        v = QVBoxLayout(box)
        self.chk_enable = QCheckBox('Habilitar motores')
        self.chk_enable.toggled.connect(self.on_enable)
        v.addWidget(self.chk_enable)
        mode = QHBoxLayout()
        self.rb_single = QRadioButton('Individual')
        self.rb_all = QRadioButton('Todos (M1-M4)')
        self.rb_single.setChecked(True)
        self.mode_group = QButtonGroup(self)
        for rb in (self.rb_single, self.rb_all):
            self.mode_group.addButton(rb)
            mode.addWidget(rb)
        self.rb_all.toggled.connect(self.on_mode_changed)
        v.addLayout(mode)
        lim = QHBoxLayout()
        lim.addWidget(QLabel('Limite max %'))
        self.spin_limit = QDoubleSpinBox(minimum=1, maximum=100, value=DEFAULT_LIMIT_PCT,
                                         decimals=0)
        self.spin_limit.valueChanged.connect(self.on_limit)
        lim.addWidget(self.spin_limit)
        v.addLayout(lim)

        cols = QHBoxLayout()
        self.sliders, self.lbl_req, self.lbl_pwm = {}, {}, {}
        for m in MOTORS:
            col = QVBoxLayout()
            s = QSlider(Qt.Orientation.Vertical, minimum=0, maximum=int(DEFAULT_LIMIT_PCT),
                        minimumHeight=260)
            s.valueChanged.connect(lambda val, m=m: self.on_slider(m, val))
            req = QLabel('0 %')
            pwm = QLabel('pwm: -')
            for w in (req, pwm):
                w.setAlignment(Qt.AlignmentFlag.AlignHCenter)
            col.addWidget(QLabel(f'M{m}'), 0, Qt.AlignmentFlag.AlignHCenter)
            col.addWidget(s, 1, Qt.AlignmentFlag.AlignHCenter)
            col.addWidget(req)
            col.addWidget(pwm)
            cols.addLayout(col)
            self.sliders[m], self.lbl_req[m], self.lbl_pwm[m] = s, req, pwm

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        cols.addWidget(sep)
        col = QVBoxLayout()
        self.slider_all = QSlider(Qt.Orientation.Vertical, minimum=0,
                                  maximum=int(DEFAULT_LIMIT_PCT), minimumHeight=260)
        self.slider_all.valueChanged.connect(self.on_master)
        self.lbl_req_all = QLabel('0 %')
        self.lbl_req_all.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        lbl_all = QLabel('ALL')
        lbl_all.setStyleSheet('font-weight:bold')
        col.addWidget(lbl_all, 0, Qt.AlignmentFlag.AlignHCenter)
        col.addWidget(self.slider_all, 1, Qt.AlignmentFlag.AlignHCenter)
        col.addWidget(self.lbl_req_all)
        col.addWidget(QLabel(''))       # keeps the row aligned with the pwm labels
        cols.addLayout(col)
        v.addLayout(cols)

        self.btn_stop = QPushButton('STOP (Espacio)')
        self.btn_stop.setStyleSheet(f'background:{RED};color:white;font-weight:bold;padding:8px')
        self.btn_stop.clicked.connect(self.stop)
        v.addWidget(self.btn_stop)
        return box

    def _build_imu(self):
        box = QGroupBox('MPU-6050')
        v = QVBoxLayout(box)
        grid = QGridLayout()
        self.vals = {}
        rows = [('acc [g]', 'x y z'), ('gyro [deg/s]', 'x y z')]
        for r, (title, axes) in enumerate(rows):
            grid.addWidget(QLabel(title), r, 0)
            for c, a in enumerate(axes.split()):
                lbl = QLabel('-')
                lbl.setMinimumWidth(70)
                grid.addWidget(QLabel(a), r, 1 + 2 * c)
                grid.addWidget(lbl, r, 2 + 2 * c)
                self.vals[(title[:3], a)] = lbl
        grid.addWidget(QLabel('actitud [deg]'), 2, 0)
        for c, a in enumerate(('roll', 'pitch', 'yaw')):
            lbl = QLabel('-')
            grid.addWidget(QLabel(a), 2, 1 + 2 * c)
            grid.addWidget(lbl, 2, 2 + 2 * c)
            self.vals[('att', a)] = lbl
        self.lbl_accnorm = QLabel('-')
        self.lbl_flags = QLabel('-')
        grid.addWidget(QLabel('|acc| [g]'), 3, 0)
        grid.addWidget(self.lbl_accnorm, 3, 2)
        grid.addWidget(QLabel('armed / canfly'), 3, 3)
        grid.addWidget(self.lbl_flags, 3, 4)
        self.lbl_stab = QLabel('-')
        grid.addWidget(QLabel('estabilizador'), 3, 5)
        grid.addWidget(self.lbl_stab, 3, 6)
        v.addLayout(grid)

        colors = ('#d62728', '#2ca02c', '#1f77b4')
        self.curves = {}
        for key, title in (('acc', 'Acelerometro [g]'), ('gyro', 'Giroscopio [deg/s]')):
            pw = pg.PlotWidget(title=title)
            pw.showGrid(x=True, y=True)
            pw.addLegend()
            self.curves[key] = [pw.plot(pen=pg.mkPen(c, width=2), name=a)
                                for c, a in zip(colors, 'xyz')]
            v.addWidget(pw, 1)
        return box

    # ---- state helpers
    def set_status(self, text, color):
        self.lbl_status.setText(text)
        self.lbl_status.setStyleSheet(f'color:{color};font-weight:bold')

    def enable_controls(self, on):
        self.chk_enable.setEnabled(on)
        self.btn_stop.setEnabled(on)
        self.spin_limit.setEnabled(on)
        self.rb_single.setEnabled(on)
        self.rb_all.setEnabled(on)
        self.controls_on = on
        self._apply_mode_enables()

    def _apply_mode_enables(self):
        """Only the slider set of the selected mode is usable, and only with controls on."""
        on = self.controls_on
        all_mode = self.rb_all.isChecked()
        for s in self.sliders.values():
            s.setEnabled(on and not all_mode)
        self.slider_all.setEnabled(on and all_mode)

    # ---- connection flow: Wi-Fi -> CRTP; teardown in reverse
    def toggle_connection(self):
        if not self.busy:
            self.busy = True
            self.btn_conn.setText('Desconectar')
            self.lbl_info.setText('')
            self.attempt += 1
            self.wifi.connect(self.attempt)
        else:
            self.teardown('Desconectando...')

    def on_wifi_status(self, attempt, text, color):
        if attempt == self.attempt and not self.tearing:
            self.set_status(text, color)

    def on_wifi_failed(self, attempt, msg):
        if attempt == self.attempt:
            self.on_failed(msg)

    def on_wifi_ready(self, attempt, ssid):
        if attempt != self.attempt or self.tearing:
            return
        self.set_status(f'Abriendo CRTP ({ssid})...', ORANGE)
        self.connecting_since = time.monotonic()
        self.link.open()

    def on_linked(self):
        if self.tearing:
            return
        self.connecting_since = None
        self.linked_since = time.monotonic()
        self.linked = True
        self._reset_motor_ui()
        self.link.stop_all()
        self.enable_controls(True)
        self.set_status(f'Conectado ({self.wifi.ssid})', GREEN)

    def on_info(self, name, value):
        short = name.split('.')[-1]
        self.lbl_info.setText(f'{self.lbl_info.text()}  {short}={value}'.strip())

    def on_failed(self, msg):
        if not self.busy:
            return
        self.teardown(f'Error: {msg}', color=RED)

    def teardown(self, text, color=ORANGE):
        """Non-blocking: stop motors, close CRTP, then drop Wi-Fi, all off the GUI thread."""
        if self.tearing:
            return
        self.tearing = True
        self.attempt += 1               # a Wi-Fi attempt still in flight must not reconnect
        self.connecting_since = None
        self.set_status('Desconectando...', ORANGE)
        self._final = (text, color) if color == RED else ('Desconectado', GREY)
        self.btn_conn.setEnabled(False)
        self.enable_controls(False)
        self._reset_motor_ui()
        self._reset_battery_ui()
        self.drone_view.set_values({}, {})
        self.lbl_warn.hide()
        self.lbl_stab.setText('-')
        self.linked = False
        self.linked_since = None
        threading.Thread(target=self._teardown_worker, daemon=True).start()

    def _teardown_worker(self):
        try:
            self.link.close()
            self.wifi.drop()
        finally:
            self.torn_down.emit()

    def on_torn_down(self):
        self.busy = False
        self.tearing = False
        self.btn_conn.setText('Conectar')
        self.btn_conn.setEnabled(True)
        self.set_status(*self._final)

    # ---- battery
    def _reset_battery_ui(self):
        self.bar_batt.setValue(0)
        self.bar_batt.setFormat('-')
        self.bar_batt.setStyleSheet('')
        self.lbl_batt.setText('-')

    def _show_battery(self, pm):
        level, state = pm['pm.batteryLevel'], pm['pm.state']
        if level < 20 or state == PM_LOW_POWER:
            color = RED
        elif level < 50:
            color = ORANGE
        else:
            color = GREEN
        self.bar_batt.setValue(level)
        self.bar_batt.setFormat('%p %')
        self.bar_batt.setStyleSheet(f'QProgressBar::chunk{{background:{color}}}')
        self.lbl_batt.setText(f"{pm['pm.vbat']:.2f} V · {PM_STATES.get(state, state)}")

    # ---- motors
    def _reset_motor_ui(self):
        """Sliders and enable checkbox back to 0/off without sending anything."""
        self.pending.clear()
        self.sent = {m: 0 for m in MOTORS}
        for m in MOTORS:
            self.sliders[m].blockSignals(True)
            self.sliders[m].setValue(0)
            self.sliders[m].blockSignals(False)
            self.lbl_req[m].setText('0 %')
        self.slider_all.blockSignals(True)
        self.slider_all.setValue(0)
        self.slider_all.blockSignals(False)
        self.lbl_req_all.setText('0 %')
        self.chk_enable.blockSignals(True)
        self.chk_enable.setChecked(False)
        self.chk_enable.blockSignals(False)

    def on_slider(self, m, pct):
        self.lbl_req[m].setText(f'{pct} %')
        self.pending[m] = round(pct / 100.0 * 65535)

    def on_master(self, pct):
        self.lbl_req_all.setText(f'{pct} %')
        for m in MOTORS:
            self.sliders[m].blockSignals(True)
            self.sliders[m].setValue(pct)
            self.sliders[m].blockSignals(False)
            self.on_slider(m, pct)

    def on_mode_changed(self, _checked):
        # Zero everything on a mode switch so no motor jumps to a stale value
        enabled = self.chk_enable.isChecked()
        self._reset_motor_ui()
        self.pending = {m: 0 for m in MOTORS}
        self.sent = {m: None for m in MOTORS}     # force the zeros out on the next tick
        self.chk_enable.blockSignals(True)
        self.chk_enable.setChecked(enabled)       # keep the enable state the user chose
        self.chk_enable.blockSignals(False)
        self._apply_mode_enables()

    def on_limit(self, lim):
        for s in (*self.sliders.values(), self.slider_all):
            s.setMaximum(int(lim))     # clamps current value and emits valueChanged
        self.drone_view.set_duty_limit(lim)

    def on_enable(self, on):
        if self.linked:
            self.link.set_enable(on)

    def stop(self):
        if not self.linked:
            return
        self._reset_motor_ui()
        self.link.stop_all()

    # ---- periodic update
    def tick(self):
        # cflib's UDP driver never reports link errors, so watch for silence ourselves
        now = time.monotonic()
        if self.busy and not self.tearing:
            if self.linked and now - self.link.last_rx > RX_TIMEOUT_S:
                self.teardown('Error: sin datos del drone', RED)
            elif (self.connecting_since is not None
                  and now - self.connecting_since > CONNECT_TIMEOUT_S):
                self.teardown('Error: el drone no responde (CRTP)', RED)
        if not self.linked:
            return
        for m, val in list(self.pending.items()):
            if val != self.sent[m]:
                self.link.set_motor(m, val)
                self.sent[m] = val
        self.pending.clear()

        imu, att, pwm, pm = self.link.snapshot()
        if pm:
            self._show_battery(pm)
        actual = {}
        for m in MOTORS:
            raw = pwm.get(f'pwm.m{m}_pwm')
            if raw is not None:
                self.lbl_pwm[m].setText(f'pwm: {raw >> 8}/255')
                actual[m] = (raw >> 8) / 256 * 100     # motors.c keeps the top 8 bits
        self.drone_view.set_values(actual, {m: self.sliders[m].value() for m in MOTORS})
        if att:
            for a in ('roll', 'pitch', 'yaw'):
                self.vals[('att', a)].setText(f"{att[f'stabilizer.{a}']:.1f}")
            self.lbl_flags.setText(f"{att['sys.armed']} / {att['sys.canfly']}")
            # The link can be fine while the stabilizer loop is stuck (gyro calibration
            # or MPU-6050 interrupt): logs then carry zeros and motorPowerSet is ignored.
            hz = att['stabilizer.rtStab']
            self.lbl_stab.setText(f'{hz:.0f} Hz')
            if hz < STAB_MIN_HZ and now - self.linked_since > STAB_GRACE_S:
                self.lbl_warn.setText(
                    f'Estabilizador detenido ({hz:.0f} Hz): el drone no procesa sensores '
                    'ni motores. Deja el drone quieto o reinicialo.')
                self.lbl_warn.show()
            else:
                self.lbl_warn.hide()
        if imu:
            last = imu[-1]
            for i, a in enumerate('xyz'):
                self.vals[('acc', a)].setText(f'{last[1 + i]:.3f}')
                self.vals[('gyr', a)].setText(f'{last[4 + i]:.1f}')
            self.lbl_accnorm.setText(f'{(last[1] ** 2 + last[2] ** 2 + last[3] ** 2) ** 0.5:.3f}')
            t = [s[0] - last[0] for s in imu]
            for i in range(3):
                self.curves['acc'][i].setData(t, [s[1 + i] for s in imu])
                self.curves['gyro'][i].setData(t, [s[4 + i] for s in imu])

    def closeEvent(self, event):
        self.linked = False
        self.link.close()       # blocking is fine on exit
        self.wifi.drop()
        super().closeEvent(event)


def main():
    cflib.crtp.init_drivers()
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
