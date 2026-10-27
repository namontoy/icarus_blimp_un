"""The ground station's side of the blimp link: packets in, setpoints out, with the deadman.

Wraps blimp_link.BlimpLink (UDP packets: heartbeat out, telemetry and sensors
in) and blimp_link.BlimpConsole. The one piece of logic that matters for safety
lives here: the GUI's motor setpoint is repeated to the blimp at STREAM_HZ
only while the GUI keeps refreshing it. A window that froze, a GUI that
closed or a laptop that died all stop the stream within DEADMAN_S, and the
firmware's own timeouts take it from there (docs/ground-station-plan.md §6).

Everything is thread-based, like blimp_link itself; the station's Qt signals
carry results onto the GUI thread. on_event(level, text) may be called from
any thread.
"""
import threading
import time

import blimp_link as bl

DEADMAN_S = 0.5     # the GUI must refresh its setpoint within this
STREAM_HZ = 20.0    # MOTOR_SET rate to the blimp while the setpoint is fresh


class BlimpBridge:
    def __init__(self, host, port, console_port, on_event, on_packet=None):
        self.host, self.port, self.console_port = host, port, console_port
        self.on_event = on_event
        self._extra_packet = on_packet
        self.link = bl.BlimpLink(host, port)
        self.link.on_packet = self._packet
        self.console = bl.BlimpConsole(host, console_port)
        self._console_lock = threading.Lock()   # one command at a time on the wire
        self._lock = threading.Lock()
        self._cmd = None
        self._cmd_at = 0.0
        self._streaming = False
        self._last_failsafe = None
        self._last_alive = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)

    def start(self):
        self.link.start()
        self._thread.start()
        return self

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.link.close()
        self.console.close()

    # ----------------------------------------------------------- packets in

    def _packet(self, kind, d):
        if kind == 'telemetry':
            fs = d['failsafe']
            if fs != self._last_failsafe and fs == 'TRIPPED':
                self.on_event('crit', 'motor failsafe TRIPPED on the blimp: all motors coasting')
            self._last_failsafe = fs
        if self._extra_packet:
            self._extra_packet(kind, d)

    def snapshot(self):
        """Link state plus the latest packets, for one broadcast. Also notices
        telemetry appearing and disappearing, which is the only way the
        operator learns that the Wi-Fi dropped."""
        alive = self.link.alive
        if self._last_alive is not None and alive != self._last_alive:
            if alive:
                self.on_event('ok', 'telemetry from the blimp is back')
            else:
                self.on_event('warn', 'no telemetry from the blimp for 1 s')
        self._last_alive = alive
        age = (time.monotonic() - self.link.last_rx) if self.link.last_rx else None
        return {'alive': alive, 'age_s': age, 'rx': self.link.rx_count, 'bad': self.link.rx_bad,
                'sensors_rx': self.link.sensors_count, 'streaming': self._streaming,
                'host': self.host, 'tel': self.link.telemetry, 'sen': self.link.sensors}

    # ---------------------------------------------------------- setpoints out

    def set_motors(self, cmd):
        """cmd: four signed commands in FL, RL, FR, RR order. The GUI clamps to
        its limit; this clamps to the driver's range so a bug cannot exceed it."""
        vals = []
        for c in list(cmd)[:4]:
            c = float(c)
            if c != c:          # NaN
                c = 0.0
            vals.append(max(-1.0, min(1.0, c)))
        while len(vals) < 4:
            vals.append(0.0)
        with self._lock:
            self._cmd = vals
            self._cmd_at = time.monotonic()

    def stop(self, brake=False):
        with self._lock:
            self._cmd = None
            self._streaming = False
        for _ in range(3):                      # UDP: say it more than once
            try:
                self.link.stop_motors(brake)
            except OSError:
                pass
            time.sleep(0.02)
        self.on_event('ok', 'STOP sent: all motors %s' % ('braked' if brake else 'coasting'))

    def release(self):
        """The GUI went away: nothing is driving any more."""
        with self._lock:
            self._cmd = None

    def _stream_loop(self):
        period = 1.0 / STREAM_HZ
        while not self._stop.wait(period):
            with self._lock:
                cmd = self._cmd
                fresh = cmd is not None and time.monotonic() - self._cmd_at < DEADMAN_S
            if fresh:
                try:
                    self.link.set_motors(*cmd)
                except OSError:
                    pass
                self._streaming = True
            elif self._streaming:
                self._streaming = False
                driven = cmd is not None and any(cmd)
                with self._lock:
                    self._cmd = None
                for _ in range(2):
                    try:
                        self.link.stop_motors(False)
                    except OSError:
                        pass
                if driven:
                    self.on_event('warn', 'setpoint from the GUI went stale for %d ms: '
                                          'motors released' % int(DEADMAN_S * 1000))

    # ---------------------------------------------------------------- console

    def run_console(self, line, echo):
        """Blocking; call from a worker thread. echo(text) gets each chunk as it
        arrives; the return value is the whole output."""
        with self._console_lock:
            return self.console.run(line, timeout=300.0, idle=15.0, echo=echo)


class DemoBlimp(BlimpBridge):
    """mock_blimp.MockBlimp in-process on localhost, with the same bridge on top,
    so the GUI can be developed and shown with nothing plugged in."""

    def __init__(self, on_event, on_packet=None, port=bl.LINK_PORT, console_port=bl.CONSOLE_PORT):
        import mock_blimp
        self.mock = mock_blimp.MockBlimp('127.0.0.1', port, console_port, verbose=False)
        threading.Thread(target=self.mock.serve, daemon=True, name='mock_blimp').start()
        super().__init__('127.0.0.1', port, console_port, on_event, on_packet)
