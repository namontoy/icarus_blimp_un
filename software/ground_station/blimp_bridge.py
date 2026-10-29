"""The ground station's side of the blimp link: packets in, the standing motor command out.

Wraps blimp_link.BlimpLink (UDP packets: heartbeat out, telemetry and sensors
in), blimp_link.BlimpConsole and blimp_link.CaptureClient (the capture, for
the Bench page). The one piece of logic that matters for safety lives here,
and since 2026-10-28 it is this: the bridge's own thread OWNS the standing
motor command. The GUI tells it the command (set_motors) when something
changes, and STOP when the operator says so; the thread repeats the command
to the blimp at STREAM_HZ for as long as it stands, on its own clock, so
nothing the GUI thread does, or fails to do, between two changes reaches the
motors. Before this the GUI thread had to refresh the setpoint every 100 ms
against a 0.5 s deadman here, and one repaint of a long capture (230 ms,
measured) was enough to cut the motors (docs/motor-setpoint-ownership.md).

What still protects against a frozen window: the GUI thread pings the bridge
every 100 ms (the station's tick); if the pings stop for WATCHDOG_S while a
command stands, this thread sends MOTOR_STOP, drops the command, logs a crit
event and LATCHES: set_motors is ignored until stop() is called, so a window
that thaws cannot restart the motors by itself (it sees the trip in the
snapshot and STOPs, which clears the latch and puts its controls off). A
dead process, a dead laptop or a dropped radio are the firmware's timers'
job (500 ms link, 1 s motor failsafe), as before. stop() may be called from
any thread, and is: the window, the Ctrl-C handler, and the gamepad reader's
thread for B.

Everything is thread-based, like blimp_link itself; the station's Qt signals
carry results onto the GUI thread. on_event(level, text) may be called from
any thread.
"""
import threading
import time

import blimp_link as bl

WATCHDOG_S = 3.0    # the GUI thread must ping within this while a command stands; no repaint takes that long
STREAM_HZ = 20.0    # MOTOR_SET rate to the blimp while a command stands


class BlimpBridge:
    def __init__(self, host, port, console_port, on_event, on_packet=None):
        self.host, self.port, self.console_port = host, port, console_port
        self.on_event = on_event
        self._extra_packet = on_packet
        self.link = bl.BlimpLink(host, port)
        self.link.on_packet = self._packet
        self.capture = bl.CaptureClient(self.link)     # used from the station's capture thread only
        self.console = bl.BlimpConsole(host, console_port)
        self._console_lock = threading.Lock()   # one command at a time on the wire
        self._lock = threading.Lock()
        self._cmd = None                        # the standing command, or None
        self._ping_at = time.monotonic()        # the GUI thread's last sign of life
        self._tripped = False                   # the watchdog fired; set_motors is refused until stop()
        self.trips = 0
        self._streaming = False
        self._last_failsafe = None
        self._last_alive = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._stream_loop, daemon=True, name='motor-stream')

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
        with self._lock:
            standing, tripped = self._cmd is not None, self._tripped
        return {'alive': alive, 'age_s': age, 'rx': self.link.rx_count, 'bad': self.link.rx_bad,
                'sensors_rx': self.link.sensors_count, 'streaming': self._streaming, 'standing': standing,
                'tripped': tripped, 'host': self.host, 'tel': self.link.telemetry, 'sen': self.link.sensors}

    # ---------------------------------------------------------- setpoints out

    def ping(self):
        """The GUI thread is alive: called from the station's tick, every 100 ms."""
        self._ping_at = time.monotonic()

    def set_motors(self, cmd):
        """cmd: four signed commands in FL, RL, FR, RR order. It stands, and is
        repeated to the blimp, until the next set_motors, stop() or the
        watchdog. The GUI clamps to its limit; this clamps to the driver's
        range so a bug cannot exceed it. False, and nothing sent, while the
        watchdog's latch is set."""
        vals = []
        for c in list(cmd)[:4]:
            c = float(c)
            if c != c:          # NaN
                c = 0.0
            vals.append(max(-1.0, min(1.0, c)))
        while len(vals) < 4:
            vals.append(0.0)
        with self._lock:
            if self._tripped:
                return False
            self._cmd = vals
            self._ping_at = time.monotonic()        # a command is a sign of life too
        return True

    def stop(self, brake=False):
        """Any thread. Drops the standing command, clears the watchdog's latch,
        and says so to the blimp three times (UDP)."""
        with self._lock:
            self._cmd = None
            self._tripped = False
            self._streaming = False
        for _ in range(3):
            try:
                self.link.stop_motors(brake)
            except OSError:
                pass
            time.sleep(0.02)
        self.on_event('ok', 'STOP sent: all motors %s' % ('braked' if brake else 'coasting'))

    def _stream_loop(self):
        period = 1.0 / STREAM_HZ
        while not self._stop.wait(period):
            with self._lock:
                cmd = self._cmd
                stale = cmd is not None and time.monotonic() - self._ping_at > WATCHDOG_S
                if stale:
                    self._cmd = None
                    self._tripped = True
                    self.trips += 1
            if stale:
                self._streaming = False
                for _ in range(2):
                    try:
                        self.link.stop_motors(False)
                    except OSError:
                        pass
                self.on_event('crit', 'the window stopped responding for %d s while a motor command stood: '
                                      'motors released; STOP to clear' % int(WATCHDOG_S))
                continue
            if cmd is not None:
                try:
                    self.link.set_motors(*cmd)
                except OSError:
                    pass
                self._streaming = True
            else:
                self._streaming = False

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
