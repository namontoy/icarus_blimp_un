"""A gamepad for the Control page: the Steam Deck's built-in controls, or any
Xbox-style pad on a laptop, read on a thread and offered as one snapshot.

    python software/ground_station/gamepad.py          # print the state until Ctrl-C

Standard library only, like the rest of the link side, so the Deck needs no
package beyond PyQt6:

- Windows: XInput through ctypes (xinput1_4.dll, the API every Xbox-class
  pad speaks; a DualShock needs a translator such as DS4Windows to appear).
- Linux, SteamOS included: the kernel's event devices (/dev/input/event*),
  picking the first one that has a gamepad's south button and an X axis, as
  the kernel's gamepad spec (Documentation/input/gamepad.rst) defines them.
  The Deck's controls come from the hid-steam driver as the device "Steam
  Deck"; it leaves its keyboard-and-mouse ("lizard") emulation while someone
  holds the event device open, which this reader does. Not yet tried on a
  Deck (2026-10-28): what is tried is the parsing, on synthetic events.

The snapshot is normalised so the page never sees a backend: sticks as
lx, ly, rx, ry in −1…+1 with right and forward (up) positive (XInput and
evdev disagree on the sign of Y), triggers lt, rt in 0…1, buttons by their
Xbox-layout names, the D-pad as four buttons. A pad that is unplugged shows
as connected=False, and whoever drives the motors from it must treat that as
a stop (control.py does).
"""
import ctypes
import os
import struct
import sys
import threading
import time

POLL_HZ = 60.0
SCAN_S = 1.0                # how often to look for a pad that is not there

AXES = ('lx', 'ly', 'rx', 'ry', 'lt', 'rt')
BUTTONS = ('a', 'b', 'x', 'y', 'lb', 'rb', 'back', 'start', 'guide', 'ls', 'rs', 'up', 'down', 'left', 'right')


def empty_state(backend='none', error=''):
    return {'connected': False, 'name': '', 'backend': backend, 'error': error, 'seq': 0,
            'axes': {a: 0.0 for a in AXES}, 'buttons': {b: False for b in BUTTONS}}


def _clamp(v):
    return -1.0 if v < -1.0 else 1.0 if v > 1.0 else v


# ---------------------------------------------------------------- Windows

class _XInputGamepad(ctypes.Structure):
    _fields_ = [('wButtons', ctypes.c_ushort), ('bLeftTrigger', ctypes.c_ubyte), ('bRightTrigger', ctypes.c_ubyte),
                ('sThumbLX', ctypes.c_short), ('sThumbLY', ctypes.c_short),
                ('sThumbRX', ctypes.c_short), ('sThumbRY', ctypes.c_short)]


class _XInputState(ctypes.Structure):
    _fields_ = [('dwPacketNumber', ctypes.c_ulong), ('Gamepad', _XInputGamepad)]


XINPUT_BUTTONS = {'up': 0x0001, 'down': 0x0002, 'left': 0x0004, 'right': 0x0008, 'start': 0x0010, 'back': 0x0020,
                  'ls': 0x0040, 'rs': 0x0080, 'lb': 0x0100, 'rb': 0x0200, 'guide': 0x0400,
                  'a': 0x1000, 'b': 0x2000, 'x': 0x4000, 'y': 0x8000}


class XInput:
    """XInputGetState on the first of the four slots that answers. Microsoft
    says polling an empty slot is slow, so the empty ones are tried once a
    second, the connected one every poll."""
    name = 'xinput'

    def __init__(self):
        self.dll = None
        for lib in ('xinput1_4', 'xinput1_3', 'xinput9_1_0'):
            try:
                self.dll = ctypes.WinDLL(lib)
                break
            except OSError:
                continue
        if self.dll is None:
            raise OSError('no XInput DLL on this Windows')
        self.dll.XInputGetState.argtypes = (ctypes.c_ulong, ctypes.POINTER(_XInputState))
        self.dll.XInputGetState.restype = ctypes.c_ulong
        self.slot = None
        self._next_scan = 0.0

    def poll(self):
        """The decoded state, or None while no pad is connected."""
        st = _XInputState()
        if self.slot is not None:
            if self.dll.XInputGetState(self.slot, ctypes.byref(st)) == 0:
                return self.decode(st, self.slot)
            self.slot = None
        now = time.monotonic()
        if now < self._next_scan:
            return None
        self._next_scan = now + SCAN_S
        for i in range(4):
            if self.dll.XInputGetState(i, ctypes.byref(st)) == 0:
                self.slot = i
                return self.decode(st, i)
        return None

    @staticmethod
    def decode(st, slot):
        g = st.Gamepad
        s = empty_state('xinput')
        s['connected'], s['name'] = True, 'XInput controller %d' % (slot + 1)
        s['axes'] = {'lx': _clamp(g.sThumbLX / 32767.0), 'ly': _clamp(g.sThumbLY / 32767.0),
                     'rx': _clamp(g.sThumbRX / 32767.0), 'ry': _clamp(g.sThumbRY / 32767.0),
                     'lt': g.bLeftTrigger / 255.0, 'rt': g.bRightTrigger / 255.0}
        s['buttons'] = {k: bool(g.wButtons & m) for k, m in XINPUT_BUTTONS.items()}
        return s


# ------------------------------------------------------------------ Linux

EV_SYN, EV_KEY, EV_ABS = 0x00, 0x01, 0x03
ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ, ABS_HAT0X, ABS_HAT0Y = 0, 1, 2, 3, 4, 5, 0x10, 0x11
# BTN_SOUTH .. BTN_THUMBR, the gamepad spec's names; the Xbox letters are the
# positions, which is what a legend on the screen has to say.
EVDEV_BUTTONS = {0x130: 'a', 0x131: 'b', 0x133: 'x', 0x134: 'y', 0x136: 'lb', 0x137: 'rb', 0x13a: 'back',
                 0x13b: 'start', 0x13c: 'guide', 0x13d: 'ls', 0x13e: 'rs',
                 0x220: 'up', 0x221: 'down', 0x222: 'left', 0x223: 'right'}
BTN_SOUTH = 0x130
KEY_MAX, ABS_MAX = 0x2ff, 0x3f
# struct input_event: struct timeval, __u16 type, __u16 code, __s32 value;
# native sizes, so 24 bytes on a 64-bit kernel and 16 on a 32-bit one.
INPUT_EVENT = struct.Struct('llHHi')
ABSINFO = struct.Struct('iiiiii')        # value, minimum, maximum, fuzz, flat, resolution


def _ior(nr, size):
    """_IOR('E', nr, size) as <linux/ioctl.h> builds it on x86 and arm."""
    return (2 << 30) | (size << 16) | (ord('E') << 8) | nr


def _has_bit(bits, n):
    return bool(bits[n >> 3] & (1 << (n & 7)))


class Evdev:
    """The first /dev/input/event* that is a gamepad, read non-blocking.
    Opening needs read permission on the device: on most distributions the
    `input` group has it, and a udev rule gives it to the seat; without it
    the error names the path so the fix is clear."""
    name = 'evdev'
    EVIOCGNAME = _ior(0x06, 256)
    EVIOCGBIT_KEY = _ior(0x20 + EV_KEY, (KEY_MAX + 1) // 8)
    EVIOCGBIT_ABS = _ior(0x20 + EV_ABS, (ABS_MAX + 1) // 8)

    def __init__(self):
        import fcntl                    # Linux only; the import is the platform check
        self._fcntl = fcntl
        self.fd, self.path, self.dev_name = None, None, ''
        self.absinfo = {}               # code -> (min, max, flat)
        self.ax = {}                    # code -> last raw value
        self.btn = set()                # codes held down
        self.error = ''
        self._next_scan = 0.0

    # device

    def _scan(self):
        denied = None
        try:
            names = sorted(n for n in os.listdir('/dev/input') if n.startswith('event'))
        except OSError:
            names = []
        for n in names:
            path = '/dev/input/' + n
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except PermissionError:
                denied = path
                continue
            except OSError:
                continue
            try:
                keys = bytearray((KEY_MAX + 1) // 8)
                axes = bytearray((ABS_MAX + 1) // 8)
                self._fcntl.ioctl(fd, self.EVIOCGBIT_KEY, keys)
                self._fcntl.ioctl(fd, self.EVIOCGBIT_ABS, axes)
                if not (_has_bit(keys, BTN_SOUTH) and _has_bit(axes, ABS_X)):
                    os.close(fd)
                    continue
                name = bytearray(256)
                self._fcntl.ioctl(fd, self.EVIOCGNAME, name)
                self.dev_name = name.split(b'\0', 1)[0].decode('utf-8', 'replace')
                self.absinfo = {}
                for code in (ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ, ABS_HAT0X, ABS_HAT0Y):
                    if _has_bit(axes, code):
                        info = bytearray(ABSINFO.size)
                        self._fcntl.ioctl(fd, _ior(0x40 + code, ABSINFO.size), info)
                        value, lo, hi, _fuzz, flat, _res = ABSINFO.unpack(bytes(info))
                        self.absinfo[code] = (lo, hi, flat)
                        self.ax[code] = value
            except OSError:
                os.close(fd)
                continue
            self.fd, self.path, self.error = fd, path, ''
            return
        self.error = ('%s: permission denied (add this user to the input group, or a udev rule)' % denied
                      if denied else 'no gamepad under /dev/input')

    def _close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd, self.path = None, None
        self.ax, self.btn = {}, set()

    def poll(self):
        if self.fd is None:
            now = time.monotonic()
            if now >= self._next_scan:
                self._next_scan = now + SCAN_S
                self._scan()
            if self.fd is None:
                return None
        try:
            while True:
                self.feed(os.read(self.fd, INPUT_EVENT.size * 64))
        except BlockingIOError:
            pass
        except OSError as e:                # unplugged
            self.error = '%s: %s' % (self.path, e.strerror or e)
            self._close()
            return None
        return self.decode()

    # parsing, kept apart from the I/O so it can be checked without a device

    def feed(self, buf):
        n = INPUT_EVENT.size
        for off in range(0, len(buf) - n + 1, n):
            _s, _us, typ, code, value = INPUT_EVENT.unpack_from(buf, off)
            if typ == EV_ABS:
                self.ax[code] = value
            elif typ == EV_KEY:
                if value:
                    self.btn.add(code)
                else:
                    self.btn.discard(code)

    def _norm(self, code, invert=False):
        """A signed axis to −1…+1 about the centre of its range, with the
        driver's flat zone as the dead band it already is."""
        if code not in self.absinfo:
            return 0.0
        lo, hi, flat = self.absinfo[code]
        v = self.ax.get(code, (lo + hi) / 2.0)
        mid, half = (lo + hi) / 2.0, max(1.0, (hi - lo) / 2.0)
        d = v - mid
        if abs(d) <= flat:
            d = 0
        out = _clamp(d / half)
        return -out if invert else out

    def _unit(self, code):
        if code not in self.absinfo:
            return 0.0
        lo, hi, _ = self.absinfo[code]
        return _clamp((self.ax.get(code, lo) - lo) / max(1.0, float(hi - lo)))

    def decode(self):
        s = empty_state('evdev')
        s['connected'], s['name'] = True, self.dev_name or self.path
        s['axes'] = {'lx': self._norm(ABS_X), 'ly': self._norm(ABS_Y, invert=True),   # evdev: down is positive
                     'rx': self._norm(ABS_RX), 'ry': self._norm(ABS_RY, invert=True),
                     'lt': self._unit(ABS_Z), 'rt': self._unit(ABS_RZ)}
        b = {k: False for k in BUTTONS}
        for code in self.btn:
            if code in EVDEV_BUTTONS:
                b[EVDEV_BUTTONS[code]] = True
        hx, hy = self.ax.get(ABS_HAT0X, 0), self.ax.get(ABS_HAT0Y, 0)
        b['left'], b['right'] = b['left'] or hx < 0, b['right'] or hx > 0
        b['up'], b['down'] = b['up'] or hy < 0, b['down'] or hy > 0
        s['buttons'] = b
        return s


# ------------------------------------------------------------------ reader

def make_backend():
    """The platform's backend, or (None, why)."""
    if sys.platform == 'win32':
        try:
            return XInput(), ''
        except OSError as e:
            return None, str(e)
    if sys.platform.startswith('linux'):
        try:
            return Evdev(), ''
        except ImportError as e:
            return None, str(e)
    return None, 'no gamepad support on %s: keyboard only' % sys.platform


class Gamepad:
    """Polls the backend at POLL_HZ on its own thread; state() is a copy of
    the latest snapshot, so a caller on the GUI thread never waits on a
    driver. seq counts snapshots, so a page can tell a frozen reader from a
    still stick."""

    def __init__(self, backend=None, on_press=None):
        """on_press(button) is called from the reader's thread on every press
        edge: the station routes B to the bridge's STOP from here, so the
        stop does not wait for the GUI thread."""
        if backend is None:
            backend, err = make_backend()
        else:
            err = ''
        self.backend = backend
        self.on_press = on_press
        self._buttons = {}
        self._state = empty_state(backend.name if backend else 'none', err)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._seq = 0

    def start(self):
        if self.backend is not None and self._thread is None:
            self._thread = threading.Thread(target=self._loop, daemon=True, name='gamepad')
            self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def state(self):
        with self._lock:
            s = dict(self._state)
        s['axes'], s['buttons'] = dict(s['axes']), dict(s['buttons'])
        return s

    def _loop(self):
        period = 1.0 / POLL_HZ
        while not self._stop.wait(period):
            try:
                s = self.backend.poll()
            except Exception as e:          # a driver surprise must not kill the reader
                s = None
                err = '%s: %s' % (self.backend.name, e)
            else:
                err = getattr(self.backend, 'error', '')
            if s is None:
                s = empty_state(self.backend.name, err or 'no gamepad connected')
            self._seq += 1
            s['seq'] = self._seq
            with self._lock:
                self._state = s
            if self.on_press is not None:
                for b, down in s['buttons'].items():
                    if down and not self._buttons.get(b):
                        try:
                            self.on_press(b)
                        except Exception as e:     # the callback's trouble must not kill the reader
                            print('gamepad: on_press(%s): %s' % (b, e))
                self._buttons = dict(s['buttons'])


def main():
    pad = Gamepad().start()
    print('backend: %s' % (pad.backend.name if pad.backend else 'none'))
    try:
        while True:
            s = pad.state()
            if s['connected']:
                ax = ' '.join('%s%+.2f' % (k, s['axes'][k]) for k in AXES)
                held = ' '.join(k for k in BUTTONS if s['buttons'][k])
                print('\r%-24s %s  [%s]%s' % (s['name'][:24], ax, held, ' ' * 10), end='', flush=True)
            else:
                print('\r%s%s' % (s['error'] or 'no gamepad', ' ' * 40), end='', flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    finally:
        pad.stop()


if __name__ == '__main__':
    main()
