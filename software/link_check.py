"""Link check: what the blimp sees of our motor stream, and what we see of its
packets, measured on the bench with the ground station closed.

    python software/link_check.py                         # 20 s: latency, loss, gaps, rates
    python software/link_check.py --download 60           # the same during a 60 s capture's download (plan T2)
    python software/link_check.py --timers                # then the firmware's two timeouts, measured
    python software/link_check.py --host 127.0.0.1        # against mock_blimp.py

What it measures, and why each matters (docs/motor-setpoint-ownership.md,
docs/wireless-link.md):

  command latency   a MOTOR_SET with a new value goes out; the time until the
                    blimp's telemetry echoes it back. Telemetry comes at 20 Hz,
                    so 50 ms of this is the sampling, not the link.
  loss              the blimp counts every valid packet it receives (rx_ok in
                    its telemetry); against what this script sent, the loss.
  age on the blimp  telemetry carries link_age_ms, how long the blimp had been
                    waiting for a packet of ours when it sent that telemetry.
                    Its maximum is the worst gap in our stream as the blimp
                    saw it; the link timeout coasts the motors at 500 ms.
  rates and gaps    telemetry and sensor packets per second, and the longest
                    silence between telemetry packets, as this side sees them.

The stream is the bridge's: MOTOR_SET at --rate (20 Hz) from a thread of its
own, the heartbeat at 10 Hz from blimp_link, exactly what the ground station
sends with a command standing. The duty is tiny (--duty 0.10, stepping by
0.01 for the latency probes); run it with the propellers off anyway. With
--download, a capture of that many seconds is recorded first and then
fetched while the stream continues, which is the operator's download-with-
motors-on case; the figures are reported for the hold and for the download
separately. --timers ends by stopping the MOTOR_SET while the heartbeat goes
on (the motor failsafe should coast within about 1 s) and then stopping
everything (link_alive should fall within 500 ms). Standard library only.
"""
import argparse
import os
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blimp_link as bl  # noqa: E402


class Stream:
    """MOTOR_SET at `hz` on its own thread, FL at `duty`, the others 0; counts
    what it sends."""

    def __init__(self, link, hz, duty):
        self.link, self.period, self.duty = link, 1.0 / hz, duty
        self.sent = 0
        self.on = True
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._t.start()
        return self

    def _loop(self):
        next_t = time.monotonic()
        while not self._stop.is_set():
            if self.on:
                try:
                    self.link.set_motors(self.duty, 0.0, 0.0, 0.0)
                    self.sent += 1
                except OSError:
                    pass
            next_t += self.period
            time.sleep(max(0.0, next_t - time.monotonic()))

    def stop(self):
        self._stop.set()
        self._t.join(timeout=1.0)


class Meter:
    """Everything received, with timestamps, from the link's receive thread."""

    def __init__(self):
        self.tel_t, self.sen_t = [], []
        self.age_max = 0
        self.rx_ok_first = self.rx_ok_last = None
        self.failsafe = set()
        self.lock = threading.Lock()
        self.latest = None

    def packet(self, kind, d):
        now = time.monotonic()
        with self.lock:
            if kind == 'telemetry':
                self.tel_t.append(now)
                self.latest = d
                self.age_max = max(self.age_max, d['link_age_ms'])
                if self.rx_ok_first is None:
                    self.rx_ok_first = d['rx_ok']
                self.rx_ok_last = d['rx_ok']
                self.failsafe.add(d['failsafe'])
            elif kind == 'sensors':
                self.sen_t.append(now)

    def reset(self):
        with self.lock:
            self.tel_t, self.sen_t = [], []
            self.age_max = 0
            self.rx_ok_first = self.rx_ok_last = None
            self.failsafe = set()

    def report(self, seconds, sent, label):
        with self.lock:
            tel, sen = list(self.tel_t), list(self.sen_t)
            age, fs = self.age_max, sorted(self.failsafe)
            got = (self.rx_ok_last - self.rx_ok_first) if self.rx_ok_first is not None else None
        gaps = [b - a for a, b in zip(tel, tel[1:])]
        lines = ['%s, %.1f s:' % (label, seconds)]
        lines.append('  telemetry %.1f/s (longest silence %.0f ms), sensors %.1f/s'
                     % (len(tel) / seconds, 1000 * max(gaps) if gaps else float('nan'), len(sen) / seconds))
        if got is not None:
            # rx_ok is sampled at telemetry packets, so a few packets are always in flight: a
            # window under 100 packets cannot tell loss from that lag.
            lines.append('  our packets: %d sent, the blimp counted %d: %s'
                         % (sent, got, ('loss %.1f %%' % (100.0 * max(0, sent - got) / sent)) if sent >= 100
                            else 'too few for a loss figure (a few are always in flight)'))
        lines.append('  the longest the blimp waited for one of ours: %d ms (it coasts at 500)%s'
                     % (age, '' if age < 250 else '  <-- close to the link timeout'))
        lines.append('  failsafe states seen: %s' % ', '.join(fs))
        return '\n'.join(lines)


def latency_probe(link, stream, meter, value, timeout=2.0):
    """Change the standing duty to `value` and time the telemetry echo."""
    stream.duty = value
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        with meter.lock:
            d = meter.latest
        if d and abs(d['motor']['FL'] - value) < 1e-6:
            return time.monotonic() - t0
        time.sleep(0.002)
    return None


def run_phase(link, stream, meter, seconds, base, label, probe_every=1.0):
    """Stream for `seconds`, probing the latency every `probe_every` s with a
    new duty value; return the report."""
    meter.reset()
    sent0 = stream.sent + 0
    hb0 = link.seq
    lat = []
    end = time.monotonic() + seconds
    k = 0
    while time.monotonic() < end:
        value = base + 0.01 * (k % 5)
        k += 1
        t = latency_probe(link, stream, meter, value)
        if t is not None:
            lat.append(t)
        time.sleep(max(0.0, probe_every - (t or 0.0)))
    sent = link.seq - hb0          # heartbeats and MOTOR_SETs both carry the sequence
    out = meter.report(seconds, sent, label)
    if lat:
        out += '\n  command to readback: median %.0f ms, max %.0f ms over %d probes (telemetry is 50 ms apart)' \
               % (1000 * statistics.median(lat), 1000 * max(lat), len(lat))
    else:
        out += '\n  command to readback: no probe echoed within 2 s'
    return out


def timers(link, stream, meter):
    """The firmware's two timeouts, as this side can see them."""
    lines = []
    stream.on = False                           # heartbeats continue
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        with meter.lock:
            d = meter.latest
        if d and d['motor']['FL'] == 0.0:
            break
        time.sleep(0.005)
    dt = time.monotonic() - t0
    with meter.lock:
        d = meter.latest
    lines.append('MOTOR_SET stopped, heartbeat on: motors at zero after %.2f s, failsafe %s (the motor failsafe is 1 s)'
                 % (dt, d['failsafe'] if d else '?') if dt < 3.0 else
                 'MOTOR_SET stopped, heartbeat on: the motors did NOT stop within 3 s')
    link.heartbeat = lambda: None               # no more heartbeats; the receive thread stays, and the blimp
    t0 = time.monotonic()                       # keeps sending telemetry to us for a few seconds
    while time.monotonic() - t0 < 3.0:
        with meter.lock:
            d = meter.latest
        if d and not d['link_alive']:
            break
        time.sleep(0.005)
    dt = time.monotonic() - t0
    lines.append('everything stopped: link_alive fell after %.2f s (the link timeout is 0.5 s)' % dt if dt < 3.0 else
                 'everything stopped: link_alive still set after 3 s (telemetry may have stopped first)')
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default=bl.BLIMP_IP)
    ap.add_argument('--port', type=int, default=bl.LINK_PORT)
    ap.add_argument('--seconds', type=float, default=20.0, help='the hold phase (default %(default)s)')
    ap.add_argument('--rate', type=float, default=20.0, help='MOTOR_SET per second (default %(default)s)')
    ap.add_argument('--duty', type=float, default=0.10, help='FL duty held (default %(default)s); props off')
    ap.add_argument('--download', type=int, metavar='S', help='record a capture of S seconds, then download it '
                                                              'while the stream goes on (plan T2)')
    ap.add_argument('--timers', action='store_true', help='end by measuring the firmware\'s two timeouts')
    args = ap.parse_args(argv)
    if not 0 < args.duty <= 0.3:
        sys.exit('--duty must be in (0, 0.3]')
    meter = Meter()
    link = bl.BlimpLink(args.host, args.port)
    link.on_packet = meter.packet
    link.start()
    if not link.wait_alive(3.0):
        link.close()
        sys.exit('no telemetry from %s:%d' % (args.host, args.port))
    tel = link.telemetry
    if any(abs(v) > 0.005 for v in tel['motor'].values()):
        link.close()
        sys.exit('the motors are already driven (%s): close the ground station first' % tel['motor'])
    print('blimp at %s: failsafe %s, link %s; streaming FL %.2f at %.0f Hz, props off?'
          % (args.host, tel['failsafe'], 'alive' if tel['link_alive'] else 'dead', args.duty, args.rate))
    cap = bl.CaptureClient(link) if args.download else None
    stream = Stream(link, args.rate, args.duty).start()
    try:
        if cap:
            info = cap.info()
            if info['state'] == 1:
                cap.stop()
            cap.start(args.download)
            print('capture of %d s recording; holding meanwhile' % args.download)
            print(run_phase(link, stream, meter, args.download + 1.0, args.duty, 'hold while recording'))
            meter.reset()
            hb0 = link.seq
            t0 = time.monotonic()
            got = {}

            def progress(k, n):
                got['k'], got['n'] = k, n

            lat = []
            done = threading.Event()

            def fetch():
                try:
                    info2, data = cap.download(progress=progress)
                    got['bytes'] = len(data)
                except Exception as e:           # report, do not stop the stream
                    got['error'] = str(e)
                done.set()

            threading.Thread(target=fetch, daemon=True).start()
            k = 0
            while not done.is_set():
                t = latency_probe(link, stream, meter, args.duty + 0.01 * (k % 5))
                k += 1
                if t is not None:
                    lat.append(t)
                time.sleep(0.5)
            dl = time.monotonic() - t0
            print(meter.report(dl, link.seq - hb0, 'download of %d chunks (%s) while holding'
                               % (got.get('n', 0), ('%d bytes' % got['bytes']) if 'bytes' in got else got.get('error', '?'))))
            if lat:
                print('  command to readback during the download: median %.0f ms, max %.0f ms over %d probes'
                      % (1000 * statistics.median(lat), 1000 * max(lat), len(lat)))
            with meter.lock:
                d = meter.latest
            print('  readback after the download: FL %+.3f (held %+.3f): %s'
                  % (d['motor']['FL'], stream.duty, 'held' if abs(d['motor']['FL'] - stream.duty) < 1e-6 else 'LOST'))
        else:
            print(run_phase(link, stream, meter, args.seconds, args.duty, 'hold'))
        if args.timers:
            print(timers(link, stream, meter))
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        stream.stop()
        for _ in range(3):
            try:
                link.stop_motors(False)
            except OSError:
                pass
            time.sleep(0.05)
        link.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
