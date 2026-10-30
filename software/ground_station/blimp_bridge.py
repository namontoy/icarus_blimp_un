"""The ground station's side of the blimp link: packets in, the standing motor command out.

Wraps blimp_link.BlimpLink (UDP packets: heartbeat out, telemetry and sensors
in), blimp_link.BlimpConsole and blimp_link.CaptureClient (the capture, for
the Bench page). The one piece of logic that matters for safety lives here:
the bridge's own thread OWNS the standing motor command (since 2026-10-28;
docs/motor-setpoint-ownership.md). The GUI tells it the command (set_motors)
and STOP; nothing else the GUI thread does, or fails to do, reaches the
motors. What goes to the blimp follows the rules the team settled on
2026-10-29 (docs/link-robustness-review.md):

- A change goes out at once; an unchanged command is repeated at STREAM_HZ
  on the thread's own clock. The repeat is the keep-alive and it carries the
  value, so a packet the radio loses is repaired by the next one instead of
  leaving the blimp on an old value; the firmware's link timeout (500 ms)
  and motor failsafe (1 s) both count on it. A set_motors with the same
  values changes nothing on the wire.
- STOP wins. The thread sends under the lock stop() takes, so a MOTOR_SET
  read before a STOP can never reach the wire after it.
- A frozen window. The GUI thread pings every 100 ms (the station's tick);
  WATCHDOG_S without one while a command stands sends MOTOR_STOP, drops the
  command, logs a crit event and LATCHES: set_motors is refused until stop(),
  so a window that thaws cannot restart the motors by itself (it sees the
  trip in the snapshot and STOPs, which clears the latch and puts its
  controls off). 3 s, the team's choice: a short stall rides through.
- A lost link. Nothing from the blimp for LINK_LOST_S means its own 500 ms
  timeout has coasted the motors, as far as this side can know, so the
  stream sends zero meanwhile: what the blimp does anyway, made explicit, so
  a blimp that still hears us does the same thing. A link back within
  RESUME_WINDOW_S resumes the command, ramped from zero over RESUME_RAMP_S;
  a longer outage drops it and latches, like the watchdog.
- A reboot of the blimp. Its count of our packets (telemetry rx_ok) goes
  back; the command is dropped and latched. A reset under load is often a
  brownout, and resuming would repeat it.
- Race mode. set_motors is refused, always: during the race the window only
  watches, and STOP is the one command it may send (the race is autonomous,
  and the firmware's link loss changes nothing there: control-loop plan D6).
- stop() may be called from any thread, and is: the window, the Ctrl-C
  handler, and the gamepad reader's thread for B, which latches so a window
  that missed the press cannot drive again before it has disarmed.

The thread also measures its own send gaps: a gap of STALL_WARN_S or more
while a command stands means something held the interpreter (the GIL) and is
reported, since the blimp coasts at 500 ms. Everything is thread-based, like
blimp_link itself; the station's Qt signals carry results onto the GUI
thread. on_event(level, text) may be called from any thread.
"""
import threading
import time

import blimp_link as bl

WATCHDOG_S = 3.0        # the GUI thread must ping within this while a command stands
STREAM_HZ = 20.0        # MOTOR_SET rate while a command stands: the repeat is the keep-alive
LINK_LOST_S = 0.6       # nothing from the blimp this long: its 500 ms link timeout has coasted the motors
RESUME_WINDOW_S = 3.0   # a link back within this resumes the command; a longer outage drops and latches it
RESUME_RAMP_S = 1.0     # a resumed command rises from zero over this
STALL_WARN_S = 0.2      # a send gap this long is reported: the review's pass mark, 2.5x under the 500 ms timeout


def clean_command(cmd):
    """Four finite commands in FL, RL, FR, RR order, clamped to the driver's
    range, so a bug upstream cannot exceed it."""
    vals = []
    for c in list(cmd)[:4]:
        c = float(c)
        if c != c:          # NaN
            c = 0.0
        vals.append(max(-1.0, min(1.0, c)))
    while len(vals) < 4:
        vals.append(0.0)
    return vals


class BlimpBridge:
    def __init__(self, host, port, console_port, on_event, on_packet=None, race=False):
        self.host, self.port, self.console_port = host, port, console_port
        self.on_event = on_event
        self.race = race
        self._extra_packet = on_packet
        self.link = bl.BlimpLink(host, port)
        self.link.on_packet = self._packet
        self.capture = bl.CaptureClient(self.link)     # used from the station's capture thread only
        self.console = bl.BlimpConsole(host, console_port)
        self._console_lock = threading.Lock()   # one command at a time on the wire
        self._lock = threading.Lock()           # the command's state, and every motor packet sent
        self._cmd = None                        # the standing command, or None
        self._ping_at = time.monotonic()        # the GUI thread's last sign of life
        self._tripped = False                   # set_motors is refused until stop()
        self.trips = 0
        self._lost_at = None                    # the blimp's last packet, while the link is lost
        self._resumed_at = None                 # the link came back; the ramp runs from here
        self._ramp = 1.0
        self._sent_at = None                    # the last MOTOR_SET, for the stall check
        self.gap_max_s = 0.0                    # the longest send gap while a command stood
        self._streaming = False
        self._stop_pending = False              # a stop() wants the lock
        self._last_failsafe = None
        self._last_alive = None
        self._last_rx_ok = None
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread = threading.Thread(target=self._stream_loop, daemon=True, name='motor-stream')

    def start(self):
        self.link.start()
        self._thread.start()
        return self

    def close(self):
        self._stop.set()
        self._wake.set()
        self._thread.join(timeout=1.0)
        self.link.close()
        self.console.close()

    @property
    def standing(self):
        return self._cmd is not None

    # ----------------------------------------------------------- packets in

    def _packet(self, kind, d):
        """The link's receive thread."""
        if kind == 'telemetry':
            fs = d['failsafe']
            if fs != self._last_failsafe and fs == 'TRIPPED':
                self.on_event('crit', 'motor failsafe TRIPPED on the blimp: all motors coasting')
            self._last_failsafe = fs
            # rx_ok counts every valid packet the blimp received since it booted,
            # from any station, so it only goes back when the blimp restarts. Its
            # t_us cannot tell: a u32 of microseconds wraps every 71.6 minutes.
            rx_ok = d['rx_ok']
            if self._last_rx_ok is not None and rx_ok < self._last_rx_ok:
                with self._lock:
                    standing = self._cmd is not None
                    if standing:
                        self._release_locked()
                self.on_event('crit' if standing else 'warn',
                              'the blimp restarted (its count of our packets went from %d to %d)%s'
                              % (self._last_rx_ok, rx_ok, ': motors released, not resumed; STOP to clear'
                                 if standing else ''))
            self._last_rx_ok = rx_ok
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
            lost, ramp = self._lost_at is not None, self._ramp
        return {'alive': alive, 'age_s': age, 'rx': self.link.rx_count, 'bad': self.link.rx_bad,
                'sensors_rx': self.link.sensors_count, 'streaming': self._streaming, 'standing': standing,
                'tripped': tripped, 'link_lost': lost, 'ramp': ramp, 'race': self.race,
                'gap_max_s': self.gap_max_s, 'host': self.host, 'tel': self.link.telemetry,
                'sen': self.link.sensors}

    # ---------------------------------------------------------- setpoints out

    def ping(self):
        """The GUI thread is alive: called from the station's tick, every 100 ms."""
        self._ping_at = time.monotonic()

    def set_motors(self, cmd):
        """cmd: four signed commands in FL, RL, FR, RR order. It stands, and is
        repeated to the blimp, until the next set_motors, stop(), the watchdog
        or a long outage; a change goes out at once. The GUI clamps to its
        limit, this to the driver's range. False, and nothing sent, while
        latched or in race mode."""
        vals = clean_command(cmd)
        with self._lock:
            if self.race or self._tripped:
                return False
            changed = vals != self._cmd
            self._cmd = vals
            self._ping_at = time.monotonic()        # a command is a sign of life too
        if changed:
            self._wake.set()
        return True

    def stop(self, brake=False, latch=False):
        """Any thread. Drops the standing command and says so to the blimp
        three times (UDP), the first under the lock so nothing the stream had
        read goes out after it. Clears the latch, unless latch: then
        set_motors stays refused until the next plain stop()."""
        # Raised before the lock is asked for: a lock is not a queue, and a
        # stream thread looping on it could take it again first.
        self._stop_pending = True
        with self._lock:
            self._stop_pending = False
            self._cmd = None
            self._tripped = latch
            self._lost_at = self._resumed_at = None
            self._ramp = 1.0
            self._streaming = False
            self._send_stop(brake)
        for _ in range(2):
            time.sleep(0.02)
            self._send_stop(brake)
        self.on_event('ok', 'STOP sent: all motors %s' % ('braked' if brake else 'coasting'))

    def _send_stop(self, brake):
        try:
            self.link.stop_motors(brake)
        except OSError:
            pass

    def _release_locked(self):
        """Under the lock: drop the command, latch, and tell the blimp twice."""
        self._cmd = None
        self._tripped = True
        self.trips += 1
        self._lost_at = self._resumed_at = None
        self._ramp = 1.0
        self._streaming = False
        for _ in range(2):
            self._send_stop(False)

    def _stream_loop(self):
        """Wakes at STREAM_HZ on a deadline, so the rate averages to it even
        where a wait rounds up to the OS tick (Windows: 15.6 ms, which made
        a plain 50 ms wait a 64 ms one), and at once on a change."""
        period = 1.0 / STREAM_HZ
        next_t = time.monotonic()
        while True:
            self._wake.wait(max(0.0, next_t - time.monotonic()))
            self._wake.clear()
            if self._stop.is_set():
                return
            now = time.monotonic()
            self._step(now)
            if now >= next_t:
                next_t += period
                if next_t <= now:           # fell behind (a stall): start the grid again
                    next_t = now + period
            else:
                next_t = now + period       # woken by a change: the next repeat one period later

    def _step(self, now):
        events = []
        with self._lock:
            if self._stop_pending:
                return                  # a stop() is waiting for this lock: it goes first
            cmd = self._cmd
            if cmd is None:
                self._streaming = False
                self._sent_at = None
                return
            if now - self._ping_at > WATCHDOG_S:
                self._release_locked()
                events.append(('crit', 'the window stopped responding for %d s while a motor command stood: '
                                       'motors released; STOP to clear' % int(WATCHDOG_S)))
            else:
                last = self.link.last_rx
                if last and now - last > LINK_LOST_S:
                    if self._lost_at is None:
                        self._lost_at = last
                        events.append(('warn', 'nothing from the blimp for %.1f s: its link timeout has coasted the '
                                               'motors; sending zero until the link returns' % (now - last)))
                    self._resumed_at = None
                    self._ramp = 0.0
                    if now - self._lost_at > RESUME_WINDOW_S:
                        self._release_locked()
                        events.append(('crit', 'the link was gone for %d s while a motor command stood: motors '
                                               'released, not resumed; STOP to clear' % int(RESUME_WINDOW_S)))
                else:
                    if self._lost_at is not None:
                        events.append(('ok', 'the link is back after %.1f s: resuming the motor command, ramped '
                                             'over %.0f s' % (now - self._lost_at, RESUME_RAMP_S)))
                        self._lost_at = None
                        self._resumed_at = now
                    if self._resumed_at is not None:
                        self._ramp = min(1.0, (now - self._resumed_at) / RESUME_RAMP_S)
                        if self._ramp >= 1.0:
                            self._resumed_at = None
                    else:
                        self._ramp = 1.0
            if self._cmd is not None:
                if self._sent_at is not None:
                    gap = now - self._sent_at
                    self.gap_max_s = max(self.gap_max_s, gap)
                    if gap >= STALL_WARN_S:
                        events.append(('warn', 'the motor stream paused %d ms: something held this program; the '
                                               'blimp coasts at 500' % round(gap * 1000)))
                try:
                    self.link.set_motors(*[c * self._ramp for c in cmd])
                except OSError:
                    pass
                self._sent_at = now
                self._streaming = True
            else:
                self._sent_at = None
        for level, text in events:
            self.on_event(level, text)

    # ---------------------------------------------------------------- console

    def run_console(self, line, echo):
        """Blocking; call from a worker thread. echo(text) gets each chunk as it
        arrives; the return value is the whole output."""
        with self._console_lock:
            return self.console.run(line, timeout=300.0, idle=15.0, echo=echo)


class DemoBlimp(BlimpBridge):
    """mock_blimp.MockBlimp in-process on localhost, with the same bridge on top,
    so the GUI can be developed and shown with nothing plugged in."""

    def __init__(self, on_event, on_packet=None, port=bl.LINK_PORT, console_port=bl.CONSOLE_PORT, race=False):
        import mock_blimp
        self.mock = mock_blimp.MockBlimp('127.0.0.1', port, console_port, verbose=False)
        threading.Thread(target=self.mock.serve, daemon=True, name='mock_blimp').start()
        super().__init__('127.0.0.1', port, console_port, on_event, on_packet, race=race)
