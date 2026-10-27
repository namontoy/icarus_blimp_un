#!/usr/bin/env python3
"""Ground link for the blimp's S3 bench firmware over its Wi-Fi access point.

Transport only, standard library only: UDP datagrams in the format of
firmware/projects/i2c_tools/main/link_protocol.h on the packet port, and the
text console on its own port. Runs with any Python 3.8+ on Windows, Linux or
macOS. Join the blimp's network first (SSID ICARUS-xxxxxx; key and ports are
the firmware's "Wireless link" menuconfig settings). The blimp is 192.168.4.1.

    python software/blimp_link.py selftest                     # no hardware needed
    python software/blimp_link.py status                       # telemetry for 5 s
    python software/blimp_link.py console                      # interactive console over the air
    python software/blimp_link.py console "motor --status"     # one command
    python software/blimp_link.py motor 0.2 0 0 0 --seconds 2  # FL RL FR RR, signed, -1..1
    python software/blimp_link.py stop

REMOVE THE PROPELLERS before any motor command. A MOTOR_SET is a standing
command only while it keeps arriving: 'motor' repeats it at --rate Hz, and the
firmware coasts every motor CONFIG_NET_LINK_TIMEOUT_MS (500 ms by default)
after the last packet, or CONFIG_MOTOR_FAILSAFE_MS (1 s) after the last motor
command, whichever comes first. Ctrl+C sends a stop.
"""
import argparse
import socket
import struct
import sys
import threading
import time
import zlib

BLIMP_IP = '192.168.4.1'        # ESP-IDF's default soft-AP address
LINK_PORT = 7200                # CONFIG_NET_LINK_PORT
CONSOLE_PORT = 7201             # CONFIG_NET_CONSOLE_PORT

MAGIC = 0xB1A7                  # LINK_PROTO_MAGIC
VERSION = 1                     # LINK_PROTO_VERSION
PKT_HEARTBEAT = 0x01
PKT_MOTOR_SET = 0x02
PKT_MOTOR_STOP = 0x03
PKT_TELEMETRY = 0x81
PKT_SENSORS = 0x82

MOTOR_ORDER = ('FL', 'RL', 'FR', 'RR')      # motor_id_t order, as on the wire
FAILSAFE = {0: 'off', 1: 'idle', 2: 'armed', 3: 'TRIPPED'}
SENSORS_IMU, SENSORS_MAG, SENSORS_BARO, SENSORS_TOF = 1, 2, 4, 8     # link_sensors_t.valid
EOT = b'\x04'                   # closes every console response

_HDR = struct.Struct('<HBBHH')              # magic, version, type, seq, reserved
_CRC = struct.Struct('<I')
_MOTOR_SET = struct.Struct('<4f')
_MOTOR_STOP = struct.Struct('<B3x')
_TELEMETRY = struct.Struct('<I4f4B3IHH')    # t_us, cmd[4], failsafe, alive, link_motors,
                                            # stations, rx_ok, rx_bad, free_heap, age_ms, reserved
_SENSORS = struct.Struct('<I3f3ff3fffffHBB')    # t_us, acc[3] g, gyro[3] dps, imu temp,
                                                # mag[3] gauss, heading deg, pressure Pa,
                                                # baro temp, baro alt m, tof mm, tof status, valid
# The same sizes are _Static_assert'ed in link_protocol.h.
assert _HDR.size + _CRC.size == 12
assert _HDR.size + _MOTOR_SET.size + _CRC.size == 28
assert _HDR.size + _MOTOR_STOP.size + _CRC.size == 16
assert _HDR.size + _TELEMETRY.size + _CRC.size == 52
assert _HDR.size + _SENSORS.size + _CRC.size == 76


# ----------------------------------------------------------------- codec

def crc32(data):
    return zlib.crc32(data) & 0xFFFFFFFF


def pack(ptype, seq, body=b''):
    """Header + body + CRC-32, the CRC computed with its own field zeroed."""
    frame = _HDR.pack(MAGIC, VERSION, ptype, seq & 0xFFFF, 0) + body
    return frame + _CRC.pack(crc32(frame + b'\0\0\0\0'))


def unpack(datagram):
    """(type, seq, body) of a valid packet; ValueError otherwise."""
    if len(datagram) < _HDR.size + _CRC.size:
        raise ValueError('too short')
    magic, version, ptype, seq, _ = _HDR.unpack_from(datagram)
    if magic != MAGIC:
        raise ValueError('bad magic 0x%04X' % magic)
    if version != VERSION:
        raise ValueError('protocol version %d, this tool speaks %d' % (version, VERSION))
    (sent,) = _CRC.unpack_from(datagram, len(datagram) - 4)
    if crc32(datagram[:-4] + b'\0\0\0\0') != sent:
        raise ValueError('bad CRC')
    return ptype, seq, datagram[_HDR.size:-4]


def decode_telemetry(body):
    f = _TELEMETRY.unpack(body)
    return {'t_us': f[0], 'motor': dict(zip(MOTOR_ORDER, f[1:5])),
            'failsafe': FAILSAFE.get(f[5], str(f[5])), 'link_alive': bool(f[6]),
            'link_motors': bool(f[7]), 'stations': f[8], 'rx_ok': f[9], 'rx_bad': f[10],
            'free_heap': f[11], 'link_age_ms': f[12]}


SEA_LEVEL_PA = 101325.0                     # cmd_baro.c's SEA_LEVEL_PA


def std_altitude_m(pressure_pa):
    """Standard-atmosphere altitude, the formula cmd_baro.c's altitude_m()
    uses, so this matches the 'baro' command's alt[m] column. The packet's
    baro_alt_m is that same altitude minus the first reading since boot."""
    if not pressure_pa > 0:
        return float('nan')
    return 44330.0 * (1.0 - (pressure_pa / SEA_LEVEL_PA) ** (1.0 / 5.255))


def decode_sensors(body):
    f = _SENSORS.unpack(body)
    valid = f[17]
    return {'t_us': f[0],
            'imu_valid': bool(valid & SENSORS_IMU), 'acc': f[1:4], 'gyro': f[4:7], 'imu_temp_c': f[7],
            'mag_valid': bool(valid & SENSORS_MAG), 'mag': f[8:11], 'heading_deg': f[11],
            'baro_valid': bool(valid & SENSORS_BARO), 'pressure_pa': f[12], 'baro_temp_c': f[13],
            'baro_alt_m': f[14], 'baro_alt_std_m': std_altitude_m(f[12]),
            'tof_valid': bool(valid & SENSORS_TOF), 'tof_mm': f[15], 'tof_status': f[16]}


def format_sensors(s):
    parts = ['t=%8.3fs' % (s['t_us'] / 1e6)]
    if s['imu_valid']:
        parts.append('acc %+.2f %+.2f %+.2f g  gyro %+6.1f %+6.1f %+6.1f dps' % (*s['acc'], *s['gyro']))
    if s['mag_valid']:
        parts.append('hdg %5.1f' % s['heading_deg'])
    if s['baro_valid']:
        parts.append('%.0f Pa %.1f C %.1f m' % (s['pressure_pa'], s['baro_temp_c'], s['baro_alt_std_m']))
    if s['tof_valid']:
        parts.append('tof %4d mm%s' % (s['tof_mm'], '' if s['tof_status'] == 0 else ' (status %d)' % s['tof_status']))
    return '  '.join(parts)


def format_telemetry(t):
    motors = ' '.join('%s %+.3f' % (m, t['motor'][m]) for m in MOTOR_ORDER)
    return ('t=%8.3fs  %s  failsafe %-7s link %s%s  sta %d  rx %d/%d  heap %dk' % (
        t['t_us'] / 1e6, motors, t['failsafe'], 'alive' if t['link_alive'] else 'STALE',
        ' (motors)' if t['link_motors'] else '', t['stations'], t['rx_ok'], t['rx_bad'],
        t['free_heap'] // 1024))


# ------------------------------------------------------------ packet link

class BlimpLink:
    """Heartbeats out at heartbeat_hz on a thread, telemetry in on another.

    The firmware answers whoever sent the last valid packet, so the first
    heartbeat is what starts the telemetry stream. `telemetry` holds the
    latest decoded packet, `alive` says whether one arrived within a second.
    """

    def __init__(self, host=BLIMP_IP, port=LINK_PORT, heartbeat_hz=10.0):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self.seq = 0
        self.lock = threading.Lock()
        self.telemetry = None
        self.sensors = None             # latest decoded SENSORS packet, if the firmware sends them
        self.on_packet = None           # optional callback(kind, dict) from the receive thread
        self.last_rx = 0.0
        self.rx_count = 0
        self.sensors_count = 0
        self.rx_bad = 0
        self._hb_period = 1.0 / heartbeat_hz
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._rx_loop, daemon=True),
                         threading.Thread(target=self._hb_loop, daemon=True)]

    def start(self):
        for t in self._threads:
            t.start()
        return self

    def close(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self.sock.close()

    @property
    def alive(self):
        return time.monotonic() - self.last_rx < 1.0

    def wait_alive(self, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.alive:
                return True
            time.sleep(0.05)
        return False

    def send(self, ptype, body=b''):
        with self.lock:
            self.seq += 1
            self.sock.sendto(pack(ptype, self.seq, body), self.addr)

    def heartbeat(self):
        self.send(PKT_HEARTBEAT)

    def set_motors(self, fl, rl, fr, rr):
        """Signed thrust commands, -1..1, in motor_id_t order. Repeat to hold."""
        self.send(PKT_MOTOR_SET, _MOTOR_SET.pack(fl, rl, fr, rr))

    def stop_motors(self, brake=False):
        self.send(PKT_MOTOR_STOP, _MOTOR_STOP.pack(1 if brake else 0))

    def _hb_loop(self):
        while not self._stop.is_set():
            try:
                self.heartbeat()
            except OSError:
                pass
            self._stop.wait(self._hb_period)

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            except ConnectionResetError:
                # Windows only: an ICMP "port unreachable" for a packet we
                # sent earlier comes back as this error on the next receive.
                # Nobody is listening on the blimp's port (yet); keep going.
                continue
            except OSError:
                break
            try:
                ptype, _, body = unpack(data)
            except ValueError:
                self.rx_bad += 1
                continue
            if ptype == PKT_TELEMETRY and len(body) == _TELEMETRY.size:
                self.telemetry = decode_telemetry(body)
                self.last_rx = time.monotonic()
                self.rx_count += 1
                if self.on_packet:
                    self.on_packet('telemetry', self.telemetry)
            elif ptype == PKT_SENSORS and len(body) == _SENSORS.size:
                self.sensors = decode_sensors(body)
                self.last_rx = time.monotonic()
                self.sensors_count += 1
                if self.on_packet:
                    self.on_packet('sensors', self.sensors)


# ---------------------------------------------------------------- console

class BlimpConsole:
    """One command line per datagram; the reply ends with a lone EOT byte."""

    def __init__(self, host=BLIMP_IP, port=CONSOLE_PORT):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)

    def close(self):
        self.sock.close()

    def run(self, line, timeout=300.0, idle=10.0, echo=None):
        """Send one line and return its whole output as text.

        `echo(text)` is called with each chunk as it arrives, for commands
        that print over many seconds. `idle` bounds the silence between two
        chunks, `timeout` the whole command; a slow 'tof -n 200' prints every
        sample, so idle is what matters in practice.
        """
        self.sock.sendto(line.encode('utf-8') + b'\n', self.addr)
        chunks = []
        start = last = time.monotonic()
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except ConnectionResetError:
                # Windows reports an ICMP "port unreachable" this way.
                raise ConnectionError('nothing is listening on %s:%d (is the firmware\'s '
                                      'console over UDP up?)' % self.addr)
            except socket.timeout:
                now = time.monotonic()
                if now - last > idle:
                    raise TimeoutError('no answer from the blimp for %.0f s' % idle
                                       if not chunks else
                                       'output stopped %.0f s ago without end-of-response' % idle)
                if now - start > timeout:
                    raise TimeoutError('command still running after %.0f s' % timeout)
                continue
            last = time.monotonic()
            if data == EOT:
                break
            text = data.decode('utf-8', 'replace')
            chunks.append(text)
            if echo:
                echo(text)
        return ''.join(chunks)

    def interactive(self):
        print("console over the air; 'help' lists commands, 'exit' leaves (Ctrl+C too)")

        def show(text):
            sys.stdout.write(text)
            sys.stdout.flush()

        while True:
            try:
                line = input('blimp> ').strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return
            if line in ('exit', 'quit'):
                return
            if not line:
                continue
            try:
                self.run(line, echo=show)
            except (TimeoutError, ConnectionError) as e:
                print('[%s]' % e)
            except KeyboardInterrupt:
                print('\n[interrupted; the command may still be running on the blimp]')


# -------------------------------------------------------------------- CLI

def _need_link(args):
    link = BlimpLink(args.host, args.port).start()
    if not link.wait_alive(2.0):
        link.close()
        sys.exit('no telemetry from %s:%d - is this laptop on the blimp\'s Wi-Fi, and '
                 'does the USB console show the access point and the UDP link up?'
                 % (args.host, args.port))
    return link


def cmd_status(args):
    link = _need_link(args)
    try:
        end = time.monotonic() + args.seconds
        shown = -1
        while time.monotonic() < end:
            if link.rx_count != shown and link.telemetry:
                shown = link.rx_count
                print(format_telemetry(link.telemetry))
                if link.sensors:
                    print('   ', format_sensors(link.sensors))
            elif not link.alive:
                print('no telemetry for 1 s')
            time.sleep(args.interval)
    finally:
        link.close()


def cmd_motor(args):
    cmds = (args.fl, args.rl, args.fr, args.rr)
    worst = max(abs(c) for c in cmds)
    if worst > args.limit:
        sys.exit('refusing |command| %.3f above the --limit of %.3f; raise it deliberately'
                 % (worst, args.limit))
    link = _need_link(args)
    period = 1.0 / args.rate
    print('props off? sending %s at %.0f Hz for %.1f s; Ctrl+C stops'
          % (' '.join('%s %+.3f' % p for p in zip(MOTOR_ORDER, cmds)), args.rate, args.seconds))
    try:
        end = time.monotonic() + args.seconds
        next_print = 0.0
        while time.monotonic() < end:
            link.set_motors(*cmds)
            now = time.monotonic()
            if now >= next_print and link.telemetry:
                print(format_telemetry(link.telemetry))
                next_print = now + 0.5
            time.sleep(period)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        for _ in range(3):              # UDP: say it more than once
            link.stop_motors(brake=False)
            time.sleep(0.05)
        time.sleep(0.3)
        if link.telemetry:
            print('after stop:', format_telemetry(link.telemetry))
        link.close()


def cmd_stop(args):
    link = BlimpLink(args.host, args.port).start()
    try:
        for _ in range(3):
            link.stop_motors(brake=args.brake)
            time.sleep(0.05)
        if link.wait_alive(2.0):
            print(format_telemetry(link.telemetry))
        else:
            print('stop sent; no telemetry back')
    finally:
        link.close()


def cmd_console(args):
    con = BlimpConsole(args.host, args.console_port)
    try:
        if args.line:
            line = ' '.join(args.line)
            try:
                sys.stdout.write(con.run(line, timeout=args.timeout, idle=args.idle))
            except (TimeoutError, ConnectionError) as e:
                sys.exit(str(e))
        else:
            con.interactive()
    finally:
        con.close()


def cmd_selftest(args):
    # The CRC variant link_protocol.h documents, by its standard check value.
    assert crc32(b'123456789') == 0xCBF43926

    hb = pack(PKT_HEARTBEAT, 7)
    assert len(hb) == 12 and unpack(hb) == (PKT_HEARTBEAT, 7, b'')

    ms = pack(PKT_MOTOR_SET, 65535, _MOTOR_SET.pack(0.25, -0.5, 0.0, 1.0))
    assert len(ms) == 28
    ptype, seq, body = unpack(ms)
    assert (ptype, seq) == (PKT_MOTOR_SET, 65535)
    assert _MOTOR_SET.unpack(body) == (0.25, -0.5, 0.0, 1.0)

    st = pack(PKT_MOTOR_STOP, 1, _MOTOR_STOP.pack(1))
    assert len(st) == 16 and unpack(st)[2] == b'\x01\x00\x00\x00'

    tel = pack(PKT_TELEMETRY, 3, _TELEMETRY.pack(
        1234567, 0.1, 0.2, -0.3, 0.0, 2, 1, 1, 1, 100, 2, 180000, 42, 0))
    assert len(tel) == 52
    t = decode_telemetry(unpack(tel)[2])
    assert t['t_us'] == 1234567 and t['failsafe'] == 'armed' and t['link_alive']
    assert t['link_motors'] and t['stations'] == 1 and t['link_age_ms'] == 42
    assert abs(t['motor']['FR'] + 0.3) < 1e-6 and t['rx_bad'] == 2
    format_telemetry(t)

    sen = pack(PKT_SENSORS, 4, _SENSORS.pack(
        99, 0.01, -0.02, 0.98, 1.5, -0.5, 12.0, 31.2, 0.21, -0.05, 0.40, 123.4,
        75012.0, 22.5, 1.37, 1520, 0, SENSORS_IMU | SENSORS_MAG | SENSORS_BARO | SENSORS_TOF))
    assert len(sen) == 76
    s = decode_sensors(unpack(sen)[2])
    assert s['imu_valid'] and s['tof_valid'] and s['tof_mm'] == 1520 and s['tof_status'] == 0
    assert abs(s['heading_deg'] - 123.4) < 1e-3 and abs(s['acc'][2] - 0.98) < 1e-6
    s2 = decode_sensors(unpack(pack(PKT_SENSORS, 5, _SENSORS.pack(*([0] * 17), SENSORS_TOF)))[2])
    assert not s2['imu_valid'] and s2['tof_valid']
    format_sensors(s)

    for bad, why in ((ms[:-1], 'truncated'),
                     (ms[:10] + bytes([ms[10] ^ 0x01]) + ms[11:], 'bit flip'),
                     (bytes([0, 0]) + ms[2:], 'magic'),
                     (ms[:2] + bytes([VERSION + 1]) + ms[3:], 'version')):
        try:
            unpack(bad)
        except ValueError:
            pass
        else:
            raise AssertionError('accepted a %s packet' % why)

    print('blimp_link self-check OK')


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--host', default=BLIMP_IP, help='blimp address (default %(default)s)')
    p.add_argument('--port', type=int, default=LINK_PORT, help='packet port (default %(default)s)')
    p.add_argument('--console-port', type=int, default=CONSOLE_PORT,
                   help='console port (default %(default)s)')
    sub = p.add_subparsers(dest='cmd', required=True)

    sub.add_parser('selftest', help='codec round-trips, no hardware').set_defaults(fn=cmd_selftest)

    s = sub.add_parser('status', help='print telemetry')
    s.add_argument('--seconds', type=float, default=5.0)
    s.add_argument('--interval', type=float, default=0.2, help='print period in s')
    s.set_defaults(fn=cmd_status)

    c = sub.add_parser('console', help='run commands over the air; no line = interactive')
    c.add_argument('line', nargs='*')
    c.add_argument('--timeout', type=float, default=300.0, help='whole command, s')
    c.add_argument('--idle', type=float, default=10.0, help='silence between chunks, s')
    c.set_defaults(fn=cmd_console)

    m = sub.add_parser('motor', help='stream a MOTOR_SET, then stop. PROPS OFF.')
    for name in MOTOR_ORDER:
        m.add_argument(name.lower(), type=float, help='%s command, -1..1' % name)
    m.add_argument('--seconds', type=float, default=2.0)
    m.add_argument('--rate', type=float, default=20.0, help='packets per second')
    m.add_argument('--limit', type=float, default=0.3,
                   help='refuse |command| above this (default %(default)s)')
    m.set_defaults(fn=cmd_motor)

    st = sub.add_parser('stop', help='coast (or --brake) all four motors')
    st.add_argument('--brake', action='store_true')
    st.set_defaults(fn=cmd_stop)

    args = p.parse_args(argv)
    args.fn(args)


if __name__ == '__main__':
    main()
