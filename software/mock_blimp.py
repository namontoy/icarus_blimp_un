#!/usr/bin/env python3
"""A stand-in for the S3 firmware's wireless link, on localhost, no hardware.

Speaks the same UDP packet and console protocol as net_link.c and
net_console.c, with the same two safety timers, so blimp_link.py and any GUI
built on it can be exercised on a laptop alone:

    python software/mock_blimp.py                 # ports 7200/7201 on 127.0.0.1
    python software/blimp_link.py --host 127.0.0.1 status
    python software/blimp_link.py --host 127.0.0.1 motor 0.2 0 0 0
    python software/blimp_link.py --host 127.0.0.1 console "motor --status"

It is a model of the link, not of the sensors: the console knows 'help',
'motor --status', 'motor -s', 'wifi', 'link', 'param' (a subset of the
table, in the firmware's format) and 'ctl' (an ideal loop's figures), and
answers anything else the way the firmware does ("Unrecognized command"). The capture (LOG_CMD,
LOG_INFO, LOG_CHUNK) is modelled too: records at 50 Hz from the same toy
airframe, so 'blimp_link.py capture' and the ground station's Bench page can
be exercised without a blimp. Keep it in step with link_protocol.h;
blimp_link.py's structs are the shared definition.
"""
import argparse
import math
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blimp_link as bl  # noqa: E402
import blimp_log  # noqa: E402

LINK_TIMEOUT_S = 0.5        # CONFIG_NET_LINK_TIMEOUT_MS
MOTOR_FAILSAFE_S = 1.0      # CONFIG_MOTOR_FAILSAFE_MS
TELEMETRY_HZ = 20           # CONFIG_NET_LINK_TELEMETRY_HZ
PEER_HOLD_MULT = 10
CAP_PERIOD_US = 20000       # CTL_PERIOD_US: one record per control tick
CAP_MAX_S = 400             # CONFIG_CAPTURE_MAX_SECONDS
# The counts the mock's records carry are the firmware's at its default
# attach settings: +/-2 g (16384 LSB/g), +/-250 deg/s (131 LSB/(deg/s)), a
# QMC5883L at 12000 LSB/gauss, a BME280.
ACC_LSB, GYRO_LSB, MAG_LSB = 16384.0, 131.0, 12000.0
# A subset of the 'param' table (firmware main/param_table.c) with params.c's
# values of 2026-10-30: the PID gains and the hover feed-forward, enough for
# the ground station's capture record and its tests. The mock keeps them and
# prints them; it has no controller to give them to.
PARAMS = (      # name, default, accepted min, max, unit
    ('hover_thrust_n', 0.0588399, 0.0, 0.0980665, 'N'),
    ('alt.kp', 1.336, 0.0, 20.0, 'N/m'),
    ('alt.ki', 0.0, 0.0, 10.0, 'N/(m s)'),
    ('alt.kd', 1.336, 0.0, 20.0, 'N s/m'),
    ('alt.i_min', -0.06, -0.980665, 0.980665, 'N'),
    ('alt.i_max', 0.15, -0.980665, 0.980665, 'N'),
    ('yaw.kp', 0.0332, 0.0, 2.0, 'N m/rad'),
    ('yaw.ki', 0.0, 0.0, 2.0, 'N m/(rad s)'),
    ('yaw.kd', 0.0332, 0.0, 2.0, 'N m s/rad'),
    ('surge.kp', 0.3, 0.0, 10.0, 'N/m'),
    ('surge.ki', 0.0, 0.0, 10.0, 'N/(m s)'),
    ('surge.kd', 0.0, 0.0, 10.0, 'N s/m'),
)


class MockBlimp:
    def __init__(self, host, port, console_port, verbose):
        self.verbose = verbose
        self.t0 = time.monotonic()
        self.motor = [0.0] * 4
        self.failsafe_on = True
        self.last_thrust_cmd = 0.0      # motors_failsafe_feed()
        self.tripped = False
        self.link_motors = False
        self.peer = None
        self.last_rx = 0.0
        self.rx_ok = self.rx_bad = self.tx_seq = 0
        self.lost_logged = False
        # net_link.c since 2026-10-29: the link timeout runs from the last
        # accepted motor packet, and a MOTOR_SET not newer than the last one
        # from the same sender, within the timeout, is dropped and counted.
        self.last_motor_rx = 0.0
        self.motor_from = None
        self.motor_seq = 0
        self.rx_stale = 0
        self.radio_on = True            # tests: False is the Wi-Fi gone both ways (reboot() is the other fault)
        self.params = {name: default for name, default, *_ in PARAMS}
        self.param_changes = 0
        # A toy airframe for the sensor packet: heading turns with the front
        # pair's difference, height rises with the rear pair, and the IMU sees
        # gravity plus motor vibration. Enough for a GUI to show motion; not a
        # model of the blimp. The yaw rate and the climb rate lag their commands
        # (an inertia over a drag), so the step tests of software/step_test.py
        # have a time constant to fit against the mock.
        self.heading = 42.0
        self.yaw_rate = 0.0
        self.height_m = 1.50
        self.vz = 0.0
        self.last_step = time.monotonic()
        self.cap = {'state': 0, 'id': 0, 'wanted': 0, 'records': bytearray(), 't_start': 0, 'next_us': None}
        self.lock = threading.Lock()
        self.link = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.link.bind((host, port))
        self.link.settimeout(0.01)
        self.con = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.con.bind((host, console_port))

    # ---- motor driver model -------------------------------------------
    def set_all(self, cmds):
        with self.lock:
            if any(math.isnan(c) or math.isinf(c) for c in cmds):
                return False
            self.motor = [max(-1.0, min(1.0, c)) for c in cmds]
            self.last_thrust_cmd = time.monotonic()
            self.tripped = False
            return True

    def coast(self):
        with self.lock:
            self.motor = [0.0] * 4
            self.last_thrust_cmd = time.monotonic()
            self.tripped = False

    def reboot(self):
        """Tests: a reset as the ground sees it. Motors off, the clock and the
        packet counters from zero, and no peer until a packet names one."""
        with self.lock:
            self.motor = [0.0] * 4
            self.tripped = False
        self.link_motors = False
        self.rx_ok = self.rx_bad = self.rx_stale = 0
        self.t0 = time.monotonic()
        self.peer = None
        self.last_rx = 0.0
        self.motor_from = None
        self.last_motor_rx = 0.0
        self.params = {name: default for name, default, *_ in PARAMS}     # RAM only, as on the blimp
        self.param_changes = 0

    def failsafe_state(self):
        if not self.failsafe_on:
            return 0
        if self.tripped:
            return 3
        return 2 if any(self.motor) else 1

    def failsafe_tick(self, now):
        if self.failsafe_on and any(self.motor) and not self.tripped \
                and now - self.last_thrust_cmd > MOTOR_FAILSAFE_S:
            with self.lock:
                self.motor = [0.0] * 4
                self.tripped = True
            self.log('E failsafe: no thrust command for %d ms - all motors coasting'
                     % int((now - self.last_thrust_cmd) * 1000))

    # ---- link ------------------------------------------------------------
    def log(self, msg):
        if self.verbose:
            print('[%7.3f] %s' % (time.monotonic() - self.t0, msg))

    def handle(self, data, addr, now):
        try:
            ptype, seq, body = bl.unpack(data)
        except ValueError:
            self.rx_bad += 1
            return
        if ptype == bl.PKT_HEARTBEAT and len(body) == 0:
            pass
        elif ptype == bl.PKT_MOTOR_SET and len(body) == bl._MOTOR_SET.size:
            cmds = bl._MOTOR_SET.unpack(body)
            if self.motor_stale(seq, addr, now):
                self.rx_stale += 1
            elif self.set_all(cmds):
                self.link_motors = any(cmds)
                self.note_motor(seq, addr, now)
            else:
                self.log('W MOTOR_SET rejected: ESP_ERR_INVALID_ARG')
        elif ptype == bl.PKT_MOTOR_STOP and len(body) == bl._MOTOR_STOP.size:
            self.coast()
            self.link_motors = False
            if not self.motor_stale(seq, addr, now):
                self.note_motor(seq, addr, now)
        elif ptype == bl.PKT_LOG_CMD and len(body) == bl._LOG_CMD.size:
            self.log_cmd(body, addr)
        else:
            self.rx_bad += 1
            return
        self.peer = addr
        self.last_rx = now
        self.rx_ok += 1

    def motor_stale(self, seq, addr, now):
        """net_link.c's motor_packet_stale(): same sender, within the timeout,
        and a sequence number not newer (16-bit, signed difference)."""
        if self.motor_from != addr or now - self.last_motor_rx >= LINK_TIMEOUT_S:
            return False
        d = (seq - self.motor_seq) & 0xFFFF
        return d == 0 or d >= 0x8000

    def note_motor(self, seq, addr, now):
        self.motor_from, self.motor_seq, self.last_motor_rx = addr, seq, now

    def link_timeout(self, now):
        if self.link_motors and now - self.last_motor_rx > LINK_TIMEOUT_S:
            self.link_motors = False
            self.coast()
            self.log('W link lost: no motor command for %d ms, all motors coasting'
                     % int((now - self.last_motor_rx) * 1000))

    def telemetry(self, now):
        if self.peer is None or now - self.last_rx > LINK_TIMEOUT_S * PEER_HOLD_MULT:
            return
        age_ms = min(65535, int((now - self.last_rx) * 1000))
        body = bl._TELEMETRY.pack(int((now - self.t0) * 1e6) & 0xFFFFFFFF, *self.motor,
                                  self.failsafe_state(), int(age_ms < LINK_TIMEOUT_S * 1000),
                                  int(self.link_motors), 1, self.rx_ok, self.rx_bad,
                                  180 * 1024, age_ms, 0)
        self.tx_seq += 1
        self.link.sendto(bl.pack(bl.PKT_TELEMETRY, self.tx_seq, body), self.peer)

    def step(self, now):
        """Advance the toy airframe to now."""
        dt = min(0.2, now - self.last_step)
        self.last_step = now
        fl, rl, fr, rr = self.motor
        # 25 deg/s per unit of asymmetry at steady state, reached with a 0.3 s lag
        self.yaw_rate += (25.0 * (fl - fr) - self.yaw_rate) * (1.0 - math.exp(-dt / 0.3))
        self.heading = (self.heading + self.yaw_rate * dt) % 360.0
        lift = 0.8 * (rl + rr) - 0.05                            # m/s^2, slightly heavy at rest
        self.vz += lift * dt
        self.vz *= 0.96                                          # drag
        self.height_m = max(0.05, min(3.0, self.height_m + self.vz * dt))

    def model(self, now):
        """What the sensors see, in units."""
        fl, rl, fr, rr = self.motor
        yaw_rate = self.yaw_rate
        vib = 0.02 * sum(abs(c) for c in self.motor)
        rnd = lambda s: (hash((int(now * 1000), s)) % 1000 / 500.0 - 1.0)   # cheap noise in -1..1
        tilt = 0.03 * (fl + fr)                                  # the front pair pitches the nose
        h = math.radians(self.heading)
        tof_mm = int(self.height_m * 1000 + 8 * rnd(8))
        return {'acc': (0.01 + tilt + vib * rnd(1), -0.02 + vib * rnd(2), 0.93 + vib * rnd(3)),
                'gyro': (0.3 * rnd(4), -2.66 + 0.3 * rnd(5), yaw_rate + 0.5 * rnd(6)),
                'mag': (0.24 * math.cos(h), -0.24 * math.sin(h), 0.31),
                'pressure': 75000.0 - 12.0 * self.height_m + 2.0 * rnd(7), 'temp': 22.4,
                'imu_temp': 31.5 + rnd(9), 'tof_mm': max(0, tof_mm),
                'tof_status': 0 if self.height_m < 2.4 else 2,      # "signal fail" out of reach
                'bat_v': 3.95 + 0.02 * rnd(10)}

    def sensors(self, now):
        if self.peer is None or now - self.last_rx > LINK_TIMEOUT_S * PEER_HOLD_MULT:
            return
        m = self.model(now)
        body = bl._SENSORS.pack(int((now - self.t0) * 1e6) & 0xFFFFFFFF, *m['acc'], *m['gyro'], m['imu_temp'],
                                *m['mag'], self.heading, m['pressure'], m['temp'], self.height_m - 1.5,
                                m['tof_mm'], m['tof_status'],
                                bl.SENSORS_IMU | bl.SENSORS_MAG | bl.SENSORS_BARO | bl.SENSORS_TOF)
        self.tx_seq += 1
        self.link.sendto(bl.pack(bl.PKT_SENSORS, self.tx_seq, body), self.peer)

    # ---- capture ----------------------------------------------------------
    def send_log_info(self, addr, result):
        c = self.cap
        body = bl._LOG_INFO.pack(c['state'], result, bl.LOG_RECORD_BYTES, 1, c['id'], CAP_PERIOD_US,
                                 len(c['records']) // bl.LOG_RECORD_BYTES, CAP_MAX_S * 1000000 // CAP_PERIOD_US,
                                 c['wanted'], c['t_start'], 2, 250, MAG_LSB, 2, 3, CAP_MAX_S, b'mock')
        self.tx_seq += 1
        self.link.sendto(bl.pack(bl.PKT_LOG_INFO, self.tx_seq, body), addr)

    def log_cmd(self, body, addr):
        cmd, count, seconds, first = bl._LOG_CMD.unpack(body)
        c = self.cap
        per_s = 1000000 // CAP_PERIOD_US
        if cmd == bl.LOG_CMD_INFO:
            self.send_log_info(addr, 0)
        elif cmd == bl.LOG_CMD_START:
            if c['state'] == 1:
                self.send_log_info(addr, 2)
                return
            if (seconds or 60) > CAP_MAX_S:
                self.send_log_info(addr, 5)          # refused, as capture.c does
                return
            c.update(state=1, id=c['id'] + 1, wanted=(seconds or 60) * per_s, records=bytearray(),
                     t_start=0, next_us=None)
            self.log('I capture %d: %d s started' % (c['id'], c['wanted'] // per_s))
            self.send_log_info(addr, 0)
        elif cmd == bl.LOG_CMD_STOP:
            if c['state'] == 1:
                c['state'] = 2 if c['records'] else 0
            self.send_log_info(addr, 0)
        elif cmd == bl.LOG_CMD_CLEAR:
            c.update(state=0, id=c['id'] + 1, wanted=0, records=bytearray())
            self.send_log_info(addr, 0)
        elif cmd == bl.LOG_CMD_GET:
            n = len(c['records']) // bl.LOG_RECORD_BYTES
            rb, cr = bl.LOG_RECORD_BYTES, bl.LOG_CHUNK_RECORDS
            for k in range(max(1, min(count, bl.LOG_GET_MAX_CHUNKS))):
                f = first + k * cr
                if f >= n:
                    self.send_log_info(addr, 3)
                    return
                m = min(cr, n - f)
                data = bytes(c['records'][f * rb:(f + m) * rb]) + bytes(rb * (cr - m))
                self.tx_seq += 1
                self.link.sendto(bl.pack(bl.PKT_LOG_CHUNK, self.tx_seq,
                                         bl._LOG_CHUNK_HEAD.pack(c['id'], m, 0, f) + data), addr)
                if m < cr:
                    return
        else:
            self.send_log_info(addr, 4)

    def capture_tick(self, now):
        """One record per CAP_PERIOD_US while a capture runs, from the same
        airframe the SENSORS packet reads, as the firmware's log stage would
        write it: counts, fixed point, fresh bits at each chip's own rate."""
        c = self.cap
        if c['state'] != 1:
            return
        now_us = int((now - self.t0) * 1e6)
        if c['next_us'] is None or now_us - c['next_us'] > 5 * CAP_PERIOD_US:
            c['next_us'] = now_us                             # first record, or resync after a stall
        # This loop is paced by a 10 ms socket timeout on a desktop clock, so
        # it may be late by a tick or two; the records still get the nominal
        # timestamps, one per period, as the firmware's timer-driven loop
        # would produce, because the step between them is what a capture is
        # checked on.
        while now_us >= c['next_us'] and c['state'] == 1:
            self._capture_record(now, c['next_us'])
            c['next_us'] += CAP_PERIOD_US

    def _capture_record(self, now, t_us):
        c = self.cap
        m = self.model(now)
        k = len(c['records']) // bl.LOG_RECORD_BYTES
        if k == 0:
            c['t_start'] = t_us & 0xFFFFFFFF
        rec = {'t_us': t_us & 0xFFFFFFFF,
               'acc': tuple(int(a * ACC_LSB) for a in m['acc']),
               'gyro': tuple(int(g * GYRO_LSB) for g in m['gyro']),
               'mag': tuple(int(v * MAG_LSB) for v in m['mag']),
               'baro_p_dpa': int(m['pressure'] * 10), 'baro_t_cc': int(m['temp'] * 100), 'baro_h_cpct': 4520,
               'tof_mm': m['tof_mm'], 'tof_status': m['tof_status'],
               'fresh': 3 | (4 if k % 10 == 0 else 0) | (8 if k % 5 == 0 else 0),     # baro 5 Hz, ToF 10 Hz
               'mode': 0, 'fs_flags': 0,
               'cam': [{'seq': 0, 'detected': 0, 'cx': 0, 'cy': 0, 'area': 0}] * 2,
               'setpoint': (0, 0, 0, 0), 'throttle': tuple(int(v * 32767) for v in self.motor),
               'bat_mv': int(m['bat_v'] * 1000), 'exec_us': 300 + (k * 7) % 40}
        c['records'] += blimp_log.encode_record(rec)
        if len(c['records']) // bl.LOG_RECORD_BYTES >= c['wanted']:
            c['state'] = 2
            self.log('I capture %d done: %d records' % (c['id'], c['wanted']))

    def link_loop(self):
        period = 1.0 / TELEMETRY_HZ
        next_tx = time.monotonic() + period
        while True:
            try:
                data, addr = self.link.recvfrom(256)
            except (socket.timeout, ConnectionResetError):
                # The reset is Windows reporting that a telemetry packet we
                # sent found no listener (the client closed). Not an error.
                data = None
            now = time.monotonic()
            if data and self.radio_on:
                self.handle(data, addr, now)
            self.link_timeout(now)
            self.failsafe_tick(now)
            self.step(now)
            self.capture_tick(now)
            if now >= next_tx:
                if self.radio_on:
                    self.telemetry(now)
                    self.sensors(now)
                next_tx += period
                if next_tx < now:
                    next_tx = now + period

    # ---- console ---------------------------------------------------------
    def run_command(self, line):
        fs = {0: 'off', 1: 'idle - nothing is being driven', 2: 'armed', 3: '*** TRIPPED ***'}
        if line == 'help':
            return ('help \n  Print the list of registered commands\n\n'
                    'motor [-m <id>] [-t <-1..1>] [-s] [--status]\n  Drive the two DRV8833 motor drivers\n\n'
                    'wifi [--start] [--stop]\n  Status of the blimp\'s Wi-Fi access point\n\n'
                    'link \n  Status of the UDP command/telemetry link\n\n'
                    'param [name|prefix [value]] | --reset [name]\n  The controller\'s parameters, live in RAM\n')
        if line.split()[:1] == ['param']:
            return self.param_command(line.split()[1:])
        if line == 'ctl':
            # ctl_task.c's answer, for an ideal loop: the mock has no task to time.
            return ('control task on core 1, priority 20, period 20000 us, task dispatch, %d ticks since boot\n'
                    'last window: 50 ticks   period 20.00/20.00/20.00 ms   exec 0/0/0 us (min/max/mean)   '
                    'overruns 0   missed 0   timer silent 0\n'
                    'stages: none in the mock; this window is an ideal one\n'
                    "parameters: the loop holds the set after %d of %d 'param' changes (read by nothing until "
                    'increment 3)\n' % (int((time.monotonic() - self.t0) * 50), self.param_changes,
                                        self.param_changes))
        if line in ('motor', 'motor --status'):
            rows = ''.join('%-6s %-12s %-9s %+10.3f %9.1f%%\n' % (n, w, g, c, 100 * abs(c))
                           for n, w, g, c in zip(bl.MOTOR_ORDER, ('#1 IN1/IN2', '#1 IN3/IN4', '#2 IN1/IN2', '#2 IN3/IN4'),
                                                 ('13/12', '11/10', '9/8', '7/14'), self.motor))
            return ('PWM 19999 Hz, 11 bit (2048 steps), slow decay, deadband 0.0%%\n'
                    'failsafe 1000 ms, %s\nnSLEEP not on a GPIO - must be strapped high in hardware\n\n'
                    '%-6s %-12s %-9s %10s %10s\n%s' % (fs[self.failsafe_state()], 'motor', 'driver',
                                                       'GPIO', 'command', 'duty', rows))
        if line in ('motor -s', 'motor --stop'):
            self.coast()
            self.link_motors = False
            return 'all motors coasting\n'
        if line == 'wifi':
            return ('access point UP\nSSID ICARUS-MOCK00   channel 1   WPA2   up to 2 station(s)\n'
                    'address 127.0.0.1   UDP link port 7200, console port 7201\n1 station(s) associated\n')
        if line == 'link':
            return ('UDP port 7200   telemetry 20 Hz   timeout 500 ms\nrx ok %d   rx rejected %d   '
                    'motor commands out of date %d\n' % (self.rx_ok, self.rx_bad, self.rx_stale))
        if line == 'slow':                       # exercises streamed output
            out = []
            for i in range(5):
                self.con.sendto(('sample %d\n' % i).encode(), self.con_peer)
                time.sleep(0.3)
            return ''.join(out)
        return 'Unrecognized command\n'

    def param_command(self, args):
        """'param' as main/cmd_param.c answers it, over PARAMS only."""
        table = {p[0]: p for p in PARAMS}

        def listing(prefix):
            rows = [p for p in PARAMS if p[0].startswith(prefix)]
            if not rows:
                return "no parameter named or starting with '%s'; 'param' lists them\n" % prefix
            differ = sum(self.params[p[0]] != p[1] for p in PARAMS)
            out = ['%d parameters; %d changes since boot; %d differ from params.c (marked *); RAM only, '
                   'a reboot restores params.c\n' % (len(PARAMS), self.param_changes, differ),
                   '%-22s %12s  %-12s %s\n' % ('name', 'value', 'unit', 'accepted')]
            for name, default, lo, hi, unit in rows:
                v = self.params[name]
                out.append('%-22s %12.6g  %-12s [%g, %g]%s\n' % (name, v, unit, lo, hi,
                                                                 '  *' if v != default else ''))
            return ''.join(out)

        def put(name, value):
            _, _, lo, hi, unit = table[name]
            if not lo <= value <= hi:
                return '%s: %g outside the accepted range; not changed (accepted: [%g, %g] %s)\n' % (
                    name, value, lo, hi, unit)
            trial = dict(self.params, **{name: value})
            if trial.get('alt.i_min', 0.0) > trial.get('alt.i_max', 0.0):      # param_table.c's order rule
                return ("%s: %g would put a PID clamp's minimum above its maximum; not changed "
                        "(accepted: [%g, %g] %s)\n" % (name, value, lo, hi, unit))
            was = self.params[name]
            if value == was:
                return '%s is already %g %s; nothing changed\n' % (name, value, unit)
            self.params[name] = value
            self.param_changes += 1
            return '%s = %g %s (was %g); %d changes since boot\n' % (name, value, unit, was, self.param_changes)

        if not args:
            return listing('')
        if args[0] in ('--reset', '-r'):
            if len(args) == 1:
                if self.params != {p[0]: p[1] for p in PARAMS}:
                    self.params = {p[0]: p[1] for p in PARAMS}
                    self.param_changes += 1
                return '%d changes since boot\n' % self.param_changes
            return put(args[1], table[args[1]][1]) if args[1] in table else "no parameter '%s'\n" % args[1]
        if len(args) == 1:
            return listing(args[0])
        if args[0] not in table:
            return "no parameter '%s'; 'param' lists them\n" % args[0]
        try:
            value = float(args[1])
        except ValueError:
            return "%s: '%s' is not a number; not changed\n" % (args[0], args[1])
        return put(args[0], value)

    def console_loop(self):
        while True:
            try:
                data, addr = self.con.recvfrom(512)
            except ConnectionResetError:
                continue            # Windows, see link_loop
            line = data.decode('utf-8', 'replace').strip()
            if not line:
                continue
            self.con_peer = addr
            self.log('console: %s' % line)
            out = self.run_command(line)
            for i in range(0, len(out), 1400):
                self.con.sendto(out[i:i + 1400].encode(), addr)
            self.con.sendto(bl.EOT, addr)

    def serve(self):
        threading.Thread(target=self.console_loop, daemon=True).start()
        self.link_loop()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=bl.LINK_PORT)
    p.add_argument('--console-port', type=int, default=bl.CONSOLE_PORT)
    p.add_argument('-q', '--quiet', action='store_true')
    a = p.parse_args()
    print('mock blimp on %s: link %d, console %d (Ctrl+C stops)' % (a.host, a.port, a.console_port))
    try:
        MockBlimp(a.host, a.port, a.console_port, not a.quiet).serve()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
