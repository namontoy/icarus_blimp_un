"""The bridge alone, against the in-process demo blimp: the standing command,
the watchdog and its latch, send on change, the stream's rate, STOP against a
send in flight, a Wi-Fi drop with its ramped resume, a long outage, a reboot,
race mode, pad B's latch, the stall report (blimp_bridge.py's docstring
has the rules; docs/link-robustness-review.md why) and the controller's
parameters read over the console."""
import socket
import threading
import time

import harness as H
from harness import check, near, wait_for

import blimp_link as bl
from software.ground_station import blimp_bridge as BB

events = []
port, console_port = H.free_ports()
b = BB.DemoBlimp(lambda lvl, txt: events.append((lvl, txt)), port=port, console_port=console_port).start()
mock = b.mock
check(b.link.wait_alive(3.0), 'demo blimp alive')

# Every motor packet as it leaves, in order: ('set', t, values) or ('stop', t). perf_counter,
# because time.monotonic() ticks at 15.6 ms on Windows.
wire = []
_set, _stop = b.link.set_motors, b.link.stop_motors


def rec_set(*v):
    _set(*v)
    wire.append(('set', time.perf_counter(), v))


def rec_stop(brake=False):
    _stop(brake)
    wire.append(('stop', time.perf_counter()))


b.link.set_motors, b.link.stop_motors = rec_set, rec_stop


def sets_since(t0):
    return [w for w in wire if w[0] == 'set' and w[1] >= t0]


class Pinger:
    """The station's tick, without a window: a ping every 100 ms."""

    def __init__(self):
        self.on = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.on:
            b.ping()
            time.sleep(0.1)


def fl():
    return mock.motor[0]


# ------------------------------------------------- standing command, watchdog
check(b.set_motors([0.2, 0, 0, 0]), 'set_motors accepted')
time.sleep(1.5)
check(near(fl(), 0.2), 'the command stands 1.5 s with no refresh from the GUI')
t0 = time.monotonic()
wait_for(lambda: fl() == 0.0, 5.0)
since_cmd = time.monotonic() - t0 + 1.5
snap = b.snapshot()
check(snap['tripped'] and not snap['standing'], 'the watchdog tripped with no pings')
check(2.8 < since_cmd < 3.7, 'released %.1f s after the last sign of life (3 s watchdog)' % since_cmd)
check(any(lvl == 'crit' and 'stopped responding' in t for lvl, t in events), 'crit event for the watchdog')
check(not b.set_motors([0.2, 0, 0, 0]), 'set_motors refused while latched')
time.sleep(0.3)
check(fl() == 0.0, 'nothing restarted while latched')
b.stop()
check(not b.snapshot()['tripped'] and b.set_motors([0.1, 0, 0, 0]), 'STOP clears the latch')
for _ in range(40):
    b.ping()
    time.sleep(0.1)
check(near(fl(), 0.1) and not b.snapshot()['tripped'], 'pings every 100 ms keep the command standing for 4 s')
b.stop()
time.sleep(0.2)
check(fl() == 0.0 and not b.snapshot()['standing'], 'STOP drops the command')

# --------------------------------------------------- send on change, the rate
ping = Pinger()
b.set_motors([0.1, 0, 0, 0])
time.sleep(0.3)
lat = []
for k in range(6):
    time.sleep(0.011 * k + 0.05)        # land the change at different phases of the 50 ms grid
    v = 0.12 + 0.01 * k
    t0 = time.perf_counter()
    b.set_motors([v, 0, 0, 0])
    if wait_for(lambda: any(near(w[2][0], v) for w in sets_since(t0)), 0.5, 0.001):
        lat.append(min(w[1] for w in sets_since(t0) if near(w[2][0], v)) - t0)
check(len(lat) == 6 and max(lat) < 0.02,
      'a change leaves at once, not at the next repeat: worst %.1f ms over %d changes'
      % (1000 * max(lat or [1]), len(lat)))
t0 = time.perf_counter()
end = t0 + 1.0
while time.perf_counter() < end:            # the Control page's habit: the same command every few ms
    b.set_motors([0.17, 0, 0, 0])
    time.sleep(0.005)
n = len(sets_since(t0 + 0.06))           # after the one change at t0
check(17 <= n <= 22, 'repeating an unchanged command adds nothing: %d sends in 0.94 s at 20 Hz' % n)
t0 = time.perf_counter()
time.sleep(2.0)
n = len(sets_since(t0))
check(37 <= n <= 43, 'the repeat runs at 20 Hz on the deadline grid: %d sends in 2 s' % n)

# ---------------------------------------------------- STOP against a slow send
# A send that takes 80 ms stands for the stream thread being descheduled
# between reading the command and putting it on the wire. Before 2026-10-29
# it read under the lock and sent outside it, so this MOTOR_SET landed after
# all three STOPs and drove the motors again until the 1 s failsafe.
b.set_motors([0.2, 0, 0, 0])
time.sleep(0.2)
in_send = threading.Event()


def slow_set(*v):
    in_send.set()
    time.sleep(0.08)
    rec_set(*v)


b.link.set_motors = slow_set
in_send.clear()
in_send.wait(1.0)
mark = len(wire)
b.stop()
b.link.set_motors = rec_set
time.sleep(0.25)
# Both sends append after they return, under the bridge's lock, so the log's
# order is the wire's.
after = [w[0] for w in wire[mark:]]
check('stop' in after and 'set' not in after[after.index('stop'):],
      'no MOTOR_SET reaches the wire after the STOP: %s' % ' '.join(after))
check(fl() == 0.0, 'the motors stay off after a STOP that met a send in flight')

# ------------------------------------------------- a Wi-Fi drop, then resume
b.set_motors([0.3, 0, 0, 0])
time.sleep(0.5)
check(near(fl(), 0.3), 'FL 0.3 before the drop')
mock.radio_on = False
time.sleep(1.2)
snap = b.snapshot()
check(snap['link_lost'] and snap['standing'] and not snap['tripped'], 'link lost seen, the command still stands')
check(fl() == 0.0, "the blimp's own 500 ms timeout coasted it")
check(all(v == 0.0 for v in wire[-1][2]) if wire[-1][0] == 'set' else False, 'the stream sends zero meanwhile')
check(any(lvl == 'warn' and 'sending zero' in t for lvl, t in events), 'warn event for the drop')
mock.radio_on = True
time.sleep(0.3)
mid = fl()
check(0.0 < mid < 0.25, 'link back: the command ramps up, FL %.3f 0.3 s in' % mid)
time.sleep(1.0)
check(near(fl(), 0.3) and b.snapshot()['ramp'] == 1.0, 'and reaches the standing 0.3 within the 1 s ramp')
check(any(lvl == 'ok' and 'link is back' in t for lvl, t in events), 'ok event for the resume')

# ------------------------------------------------------------ a long outage
mock.radio_on = False
time.sleep(3.8)
snap = b.snapshot()
check(snap['tripped'] and not snap['standing'], 'an outage over 3 s drops the command and latches')
check(any(lvl == 'crit' and 'link was gone' in t for lvl, t in events), 'crit event for the long outage')
mock.radio_on = True
time.sleep(0.6)
check(fl() == 0.0 and not b.set_motors([0.3, 0, 0, 0]), 'nothing resumes after it; set_motors refused')
b.stop()

# ----------------------------------------------------------------- a reboot
b.set_motors([0.25, 0, 0, 0])
time.sleep(0.5)
check(near(fl(), 0.25), 'FL 0.25 before the reboot')
mock.reboot()
time.sleep(0.5)
snap = b.snapshot()
check(snap['tripped'] and not snap['standing'], 'a reboot (rx_ok going back) drops the command and latches')
check(fl() == 0.0, 'and the rebooted blimp is left with its motors off')
check(any(lvl == 'crit' and 'restarted' in t for lvl, t in events), 'crit event for the reboot')
b.stop()

# ------------------------------------------------------------- pad B's latch
b.set_motors([0.2, 0, 0, 0])
time.sleep(0.2)
b.stop(False, latch=True)
check(b.snapshot()['tripped'] and not b.set_motors([0.2, 0, 0, 0]),
      'pad B: STOP latches, so a thawing window cannot drive again')
b.stop()
check(b.set_motors([0.0, 0, 0, 0]), "pad B: the window's STOP clears it")
b.stop()

# --------------------------------------------------------------- the stall
b.set_motors([0.1, 0, 0, 0])
time.sleep(0.2)
with b._lock:
    b._sent_at = time.monotonic() - 0.4     # as if the thread had been held 0.4 s
time.sleep(0.15)
check(any(lvl == 'warn' and 'stream paused' in t for lvl, t in events), 'a send gap over 200 ms is reported')
check(b.snapshot()['gap_max_s'] >= 0.4, 'and kept as the worst gap: %.2f s' % b.snapshot()['gap_max_s'])
b.stop()

# ------------------------------- the firmware's rules, as the mock models them
# net_link.c since 2026-10-29: only motor packets keep a command alive, an
# out-of-date MOTOR_SET from the same sender is dropped, and MOTOR_STOP is
# obeyed from anyone. The bridge streams no command here; b.link sends by hand.
time.sleep(0.1)
b.link.set_motors(0.2, 0, 0, 0)          # once; the link's heartbeat thread goes on at 10 Hz
time.sleep(0.2)
check(near(fl(), 0.2), 'mock: a single MOTOR_SET drives')
time.sleep(0.6)
check(fl() == 0.0, 'mock: heartbeats alone do not keep it; the motor-only link timeout coasted it')
b.link.set_motors(0.2, 0, 0, 0)
time.sleep(0.05)
stale0 = mock.rx_stale
old = (b.link.seq - 3) & 0xFFFF          # older than the MOTOR_SET just sent
b.link.sock.sendto(bl.pack(bl.PKT_MOTOR_SET, old, bl._MOTOR_SET.pack(0.9, 0, 0, 0)), b.link.addr)
time.sleep(0.05)
check(mock.rx_stale == stale0 + 1 and near(fl(), 0.2), 'mock: an out-of-date MOTOR_SET is dropped and counted')
other = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
other.sendto(bl.pack(bl.PKT_MOTOR_STOP, 1, bl._MOTOR_STOP.pack(0)), b.link.addr)
other.close()
time.sleep(0.05)
check(fl() == 0.0, "mock: another station's STOP is obeyed")

# ------------------------------------------- the parameters, through the console
# The firmware's 'param' (main/cmd_param.c, 2026-10-30) in the mock's subset;
# blimp_link.parse_params() reads the same format from both.
p0 = b.read_params()
check(p0['changes'] == 0 and near(p0['values']['alt.kp'], 1.336) and not p0['changed'],
      "params: read over the console, params.c's values, nothing changed")
out = b.run_console('param alt.kp 0.4', None)
check('alt.kp = 0.4' in out, 'params: a set answers with the value in effect: %r' % out.strip())
out = b.run_console('param alt.kp 40', None)
check('not changed' in out and mock.params['alt.kp'] == 0.4, 'params: a value outside the range is refused')
p1 = b.read_params()
check(p1['changes'] == 1 and p1['values']['alt.kp'] == 0.4 and p1['changed'] == ['alt.kp'],
      'params: the listing shows the change, its count and its mark')
held = threading.Event()
release = threading.Event()


def hold_console():
    with b._console_lock:            # a long console command, as 'tof -n 200' would be
        held.set()
        release.wait(2.0)


threading.Thread(target=hold_console, daemon=True).start()
held.wait(1.0)
t0 = time.monotonic()
try:
    b.read_params(wait=0.2)
    busy = False
except TimeoutError:
    busy = True
release.set()
check(busy and time.monotonic() - t0 < 0.5, 'params: a busy console makes the read give up after its wait')
ping.on = False
b.close()

# ---------------------------------------------------------------- race mode
rport, rconsole = H.free_ports()
race_events = []
r = BB.DemoBlimp(lambda lvl, txt: race_events.append((lvl, txt)), port=rport, console_port=rconsole,
                 race=True).start()
check(r.link.wait_alive(3.0), 'race: demo blimp alive')
check(not r.set_motors([0.3, 0, 0, 0]), 'race: set_motors refused')
time.sleep(0.3)
check(r.mock.motor == [0.0] * 4 and not r.snapshot()['standing'], 'race: nothing reaches the motors')
r.stop()
check(any('STOP sent' in t for _, t in race_events), 'race: STOP still goes out')
r.close()

raise SystemExit(H.finish())
