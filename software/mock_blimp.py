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
'motor --status', 'motor -s', 'wifi' and 'link', and answers anything else
the way the firmware does ("Unrecognized command"). Keep it in step with
link_protocol.h; blimp_link.py's structs are the shared definition.
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

LINK_TIMEOUT_S = 0.5        # CONFIG_NET_LINK_TIMEOUT_MS
MOTOR_FAILSAFE_S = 1.0      # CONFIG_MOTOR_FAILSAFE_MS
TELEMETRY_HZ = 20           # CONFIG_NET_LINK_TELEMETRY_HZ
PEER_HOLD_MULT = 10


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
        # A toy airframe for the sensor packet: heading turns with the front
        # pair's difference, height rises with the rear pair, and the IMU sees
        # gravity plus motor vibration. Enough for a GUI to show motion; not a
        # model of the blimp.
        self.heading = 42.0
        self.height_m = 1.50
        self.vz = 0.0
        self.last_step = time.monotonic()
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
            ptype, _, body = bl.unpack(data)
        except ValueError:
            self.rx_bad += 1
            return
        if ptype == bl.PKT_HEARTBEAT and len(body) == 0:
            pass
        elif ptype == bl.PKT_MOTOR_SET and len(body) == bl._MOTOR_SET.size:
            cmds = bl._MOTOR_SET.unpack(body)
            if self.set_all(cmds):
                self.link_motors = any(cmds)
            else:
                self.log('W MOTOR_SET rejected: ESP_ERR_INVALID_ARG')
        elif ptype == bl.PKT_MOTOR_STOP and len(body) == bl._MOTOR_STOP.size:
            self.coast()
            self.link_motors = False
        else:
            self.rx_bad += 1
            return
        self.peer = addr
        self.last_rx = now
        self.rx_ok += 1

    def link_timeout(self, now):
        if self.link_motors and now - self.last_rx > LINK_TIMEOUT_S:
            self.link_motors = False
            self.coast()
            self.log('W link lost: no packet for %d ms, all motors coasting'
                     % int((now - self.last_rx) * 1000))

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

    def sensors(self, now):
        if self.peer is None or now - self.last_rx > LINK_TIMEOUT_S * PEER_HOLD_MULT:
            return
        dt = min(0.2, now - self.last_step)
        self.last_step = now
        fl, rl, fr, rr = self.motor
        yaw_rate = 25.0 * (fl - fr)                              # deg/s per unit of asymmetry
        self.heading = (self.heading + yaw_rate * dt) % 360.0
        lift = 0.8 * (rl + rr) - 0.05                            # m/s^2, slightly heavy at rest
        self.vz += lift * dt
        self.vz *= 0.96                                          # drag
        self.height_m = max(0.05, min(3.0, self.height_m + self.vz * dt))
        vib = 0.02 * sum(abs(c) for c in self.motor)
        rnd = lambda s: (hash((int(now * 1000), s)) % 1000 / 500.0 - 1.0)   # cheap noise in -1..1
        tilt = 0.03 * (fl + fr)                                  # the front pair pitches the nose
        acc = (0.01 + tilt + vib * rnd(1), -0.02 + vib * rnd(2), 0.93 + vib * rnd(3))
        gyro = (0.3 * rnd(4), -2.66 + 0.3 * rnd(5), yaw_rate + 0.5 * rnd(6))
        h = math.radians(self.heading)
        mag = (0.24 * math.cos(h), -0.24 * math.sin(h), 0.31)
        pressure = 75000.0 - 12.0 * self.height_m + 2.0 * rnd(7)
        tof_mm = int(self.height_m * 1000 + 8 * rnd(8))
        tof_status = 0 if self.height_m < 2.4 else 2                  # "signal fail" out of reach
        body = bl._SENSORS.pack(int((now - self.t0) * 1e6) & 0xFFFFFFFF, *acc, *gyro, 31.5 + rnd(9),
                                *mag, self.heading, pressure, 22.4, self.height_m - 1.5,
                                max(0, tof_mm), tof_status,
                                bl.SENSORS_IMU | bl.SENSORS_MAG | bl.SENSORS_BARO | bl.SENSORS_TOF)
        self.tx_seq += 1
        self.link.sendto(bl.pack(bl.PKT_SENSORS, self.tx_seq, body), self.peer)

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
            if data:
                self.handle(data, addr, now)
            self.link_timeout(now)
            self.failsafe_tick(now)
            if now >= next_tx:
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
                    'link \n  Status of the UDP command/telemetry link\n')
        if line in ('motor', 'motor --status'):
            rows = ''.join('%-6s %-12s %-9s %+10.3f %9.1f%%\n' % (n, w, g, c, 0 if c == 0 else 12 + 88 * abs(c))
                           for n, w, g, c in zip(bl.MOTOR_ORDER, ('#1 IN1/IN2', '#1 IN3/IN4', '#2 IN1/IN2', '#2 IN3/IN4'),
                                                 ('13/12', '11/10', '9/8', '7/14'), self.motor))
            return ('PWM 19999 Hz, 11 bit (2048 steps), slow decay, deadband 12.0%%\n'
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
            return 'UDP port 7200   telemetry 20 Hz   timeout 500 ms\nrx ok %d   rx rejected %d\n' % (self.rx_ok, self.rx_bad)
        if line == 'slow':                       # exercises streamed output
            out = []
            for i in range(5):
                self.con.sendto(('sample %d\n' % i).encode(), self.con_peer)
                time.sleep(0.3)
            return ''.join(out)
        return 'Unrecognized command\n'

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
