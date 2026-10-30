"""Join the blimp's access point from the computer running the ground station.

Windows uses netsh wlan. Since Windows 11 24H2, netsh refuses to list the
networks in range or to report the current SSID unless location services are
on for console apps (Wi-Fi names count as location data), so nothing here
depends on a scan: the join is by profile name. A profile named after the
SSID is created from the known password when none exists; the SSID itself is
ICARUS-<last three MAC bytes>, and comes from --wifi-ssid, from a profile a
previous manual join left behind, or from a scan when Windows allows one.
Whether the join worked is read from the routing table: the address this
computer would send from to reach 192.168.4.1 is a 192.168.4.x one only once
it is on the blimp's network. That check is also language-independent, where
parsing netsh's localised output would not be.

Linux uses nmcli the way the esp-drone GUI did (drone_gui.py, out of the repo
since 2026-10-30, in its history): scan for the strongest
SSID with the prefix, connect with the password, then turn autoconnect off
so the laptop does not keep jumping back to the blimp after the session.

Anything else reports no backend and the window says to join by hand.
"""
import os
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time
from xml.sax.saxutils import escape

JOIN_TIMEOUT_S = 25.0       # association plus DHCP on the S3's server
LEAVE_TIMEOUT_S = 6.0


class WifiError(RuntimeError):
    pass


def local_ip_toward(host):
    """The address this computer would use to reach host, from the routing
    table. No packet leaves: a UDP socket's connect() only picks a route."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((host, 1))
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def _run(args, timeout=30):
    kw = {}
    if sys.platform == 'win32':
        kw['creationflags'] = subprocess.CREATE_NO_WINDOW
    try:
        return subprocess.run(args, capture_output=True, text=True, errors='replace',
                              timeout=timeout, **kw)
    except FileNotFoundError:
        raise WifiError('%s is not installed' % args[0])
    except subprocess.TimeoutExpired:
        raise WifiError('%s did not answer within %d s' % (' '.join(args[:3]), timeout))


# ---------------------------------------------------------------- Windows

def _profile_xml(ssid, password):
    """A WLAN profile for netsh: WPA2-PSK/AES, or open when the password is
    empty, and manual connection so Windows never joins the blimp on its own."""
    name = escape(ssid)
    if password:
        security = ('<authEncryption><authentication>WPA2PSK</authentication>'
                    '<encryption>AES</encryption><useOneX>false</useOneX></authEncryption>'
                    '<sharedKey><keyType>passPhrase</keyType><protected>false</protected>'
                    '<keyMaterial>%s</keyMaterial></sharedKey>' % escape(password))
    else:
        security = ('<authEncryption><authentication>open</authentication>'
                    '<encryption>none</encryption><useOneX>false</useOneX></authEncryption>')
    return ('<?xml version="1.0"?>'
            '<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">'
            '<name>%s</name><SSIDConfig><SSID><name>%s</name></SSID></SSIDConfig>'
            '<connectionType>ESS</connectionType><connectionMode>manual</connectionMode>'
            '<MSM><security>%s</security></MSM></WLANProfile>' % (name, name, security))


class _Netsh:
    name = 'netsh'

    def profiles(self):
        """Stored profile names. Every profile line is "<label> : <name>",
        whatever the display language, indented under a heading."""
        out = _run(['netsh', 'wlan', 'show', 'profiles']).stdout
        names = []
        for line in out.splitlines():
            if line.startswith('    ') and ':' in line:
                name = line.partition(':')[2].strip()
                if name:
                    names.append(name)
        return names

    def scan(self):
        """(ssid, signal %) in range, or [] when Windows withholds the list
        (location services off): the caller falls back to the profiles."""
        try:
            out = _run(['netsh', 'wlan', 'show', 'networks', 'mode=bssid'], timeout=20).stdout
        except WifiError:
            return []
        best = {}
        ssid = None
        for line in out.splitlines():
            m = re.match(r'\s*SSID\s+\d+\s*:\s*(.*)$', line)
            if m:
                ssid = m.group(1).strip()
                continue
            m = re.search(r':\s*(\d+)%', line)
            if m and ssid:
                best[ssid] = max(best.get(ssid, 0), int(m.group(1)))
        return sorted(best.items(), key=lambda kv: -kv[1])

    def current_ssid(self):
        return None     # withheld without location services; the IP check stands in

    def connect(self, ssid, password):
        if ssid not in self.profiles():
            fd, path = tempfile.mkstemp(suffix='.xml', prefix='icarus-wlan-')
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                f.write(_profile_xml(ssid, password))
            try:
                r = _run(['netsh', 'wlan', 'add', 'profile', 'filename=' + path, 'user=current'])
            finally:
                os.unlink(path)
            if r.returncode != 0:
                raise WifiError('could not store the Wi-Fi profile: %s' % (r.stdout + r.stderr).strip())
        r = _run(['netsh', 'wlan', 'connect', 'name=' + ssid])
        if r.returncode != 0:
            raise WifiError('netsh could not start the join: %s' % (r.stdout + r.stderr).strip())

    def disconnect(self, ssid):
        _run(['netsh', 'wlan', 'disconnect'])


# ------------------------------------------------------------------ Linux

class _Nmcli:
    name = 'nmcli'

    def __init__(self):
        self.iface = None

    def _if(self):
        if self.iface is None:
            out = _run(['nmcli', '-t', '-f', 'DEVICE,TYPE', 'device']).stdout
            for line in out.splitlines():
                dev, _, typ = line.partition(':')
                if typ == 'wifi':
                    self.iface = dev
                    break
            if self.iface is None:
                raise WifiError('NetworkManager reports no Wi-Fi interface')
        return self.iface

    def profiles(self):
        out = _run(['nmcli', '-t', '-f', 'NAME', 'connection', 'show']).stdout
        return [l.strip() for l in out.splitlines() if l.strip()]

    def scan(self):
        out = _run(['nmcli', '-t', '-f', 'SSID,SIGNAL', 'dev', 'wifi', 'list', '--rescan', 'yes',
                    'ifname', self._if()], timeout=30).stdout
        best = {}
        for line in out.splitlines():
            ssid, _, sig = line.rpartition(':')
            if ssid and sig.isdigit():
                best[ssid] = max(best.get(ssid, 0), int(sig))
        return sorted(best.items(), key=lambda kv: -kv[1])

    def current_ssid(self):
        out = _run(['nmcli', '-t', '-f', 'ACTIVE,SSID', 'dev', 'wifi', 'list', 'ifname', self._if()]).stdout
        for line in out.splitlines():
            active, _, ssid = line.partition(':')
            if active == 'yes' and ssid:
                return ssid
        return None

    def connect(self, ssid, password):
        if ssid in self.profiles():
            r = _run(['nmcli', 'connection', 'up', 'id', ssid, 'ifname', self._if()], timeout=45)
        else:
            args = ['nmcli', 'dev', 'wifi', 'connect', ssid, 'ifname', self._if()]
            if password:
                args[5:5] = ['password', password]
            r = _run(args, timeout=45)
        if r.returncode != 0:
            raise WifiError((r.stderr or r.stdout).strip() or 'nmcli could not connect')
        # The blimp is not a network to come back to on its own.
        _run(['nmcli', 'connection', 'modify', 'id', ssid, 'connection.autoconnect', 'no'])

    def disconnect(self, ssid):
        if ssid:
            _run(['nmcli', 'connection', 'down', 'id', ssid])
        else:
            _run(['nmcli', 'dev', 'disconnect', self._if()])


def _backend():
    if sys.platform == 'win32':
        return _Netsh()
    if sys.platform.startswith('linux'):
        return _Nmcli()
    return None


# -------------------------------------------------------------- the manager

class Wifi:
    """One join at a time; status() is cheap and safe from any thread."""

    def __init__(self, prefix='ICARUS', password='icarusblimp', ssid=None, blimp_ip='192.168.4.1'):
        self.prefix = prefix
        self.password = password
        self.ssid_arg = ssid
        self.blimp_ip = blimp_ip
        self.net = blimp_ip.rsplit('.', 1)[0] + '.'
        self.backend = _backend()
        self.lock = threading.Lock()
        self.busy = False
        self.phase = 'idle'         # idle, joining, up, leaving, down, error
        self.error = None
        self.ours = None            # the SSID this process joined, if it did
        self.candidates = []        # SSIDs with the prefix: stored profiles, then scans
        if self.backend is not None:
            try:
                self.refresh_candidates(scan=False)
            except WifiError as e:
                self.error = str(e)

    def joined_ip(self):
        ip = local_ip_toward(self.blimp_ip)
        return ip if ip and ip.startswith(self.net) else None

    def status(self):
        ip = self.joined_ip()
        return {'backend': self.backend.name if self.backend else 'none',
                'joined': ip is not None, 'ip': ip,
                'ssid': self.ours if ip else None, 'ours': bool(self.ours) and ip is not None,
                'busy': self.busy, 'phase': self.phase, 'error': self.error,
                'candidates': list(self.candidates)}

    def refresh_candidates(self, scan=True):
        names = []
        if self.ssid_arg:
            names.append(self.ssid_arg)
        names += [p for p in self.backend.profiles() if p.startswith(self.prefix)]
        if scan:
            names += [s for s, _ in self.backend.scan() if s.startswith(self.prefix)]
        seen = []
        for n in names:
            if n not in seen:
                seen.append(n)
        self.candidates = seen
        return seen

    def _pick(self):
        if self.ssid_arg:
            return self.ssid_arg
        scanned = [s for s, _ in self.backend.scan() if s.startswith(self.prefix)]
        if scanned:
            return scanned[0]                   # strongest first
        stored = [p for p in self.backend.profiles() if p.startswith(self.prefix)]
        if len(stored) == 1:
            return stored[0]
        if stored:
            raise WifiError('several %s-* networks are stored (%s); start with --wifi-ssid to choose'
                            % (self.prefix, ', '.join(stored)))
        raise WifiError("no %s-* network is known: join it once by hand, or start with "
                        "--wifi-ssid ICARUS-XXXXXX (the suffix is in the blimp's boot log and "
                        "in its 'wifi' command)" % self.prefix)

    def connect(self, on_phase=None):
        """Blocking. Returns the SSID joined; raises WifiError."""
        if self.backend is None:
            raise WifiError('no Wi-Fi backend on this system: join the blimp network by hand')
        with self.lock:
            self.busy, self.phase, self.error = True, 'joining', None
            if on_phase:
                on_phase()
            try:
                if self.joined_ip():
                    self.phase = 'up'
                    return self.ours or '(already on the blimp network)'
                ssid = self._pick()
                self.backend.connect(ssid, self.password)
                t0 = time.monotonic()
                while time.monotonic() - t0 < JOIN_TIMEOUT_S:
                    if self.joined_ip():
                        self.ours, self.phase = ssid, 'up'
                        self.refresh_candidates(scan=False)
                        return ssid
                    time.sleep(0.5)
                raise WifiError('joined nothing within %.0f s: is the blimp on, and is the key %r the '
                                "one in the firmware's menuconfig?" % (JOIN_TIMEOUT_S, self.password))
            except WifiError as e:
                self.phase, self.error = 'error', str(e)
                raise
            finally:
                self.busy = False

    def disconnect(self, on_phase=None):
        if self.backend is None:
            raise WifiError('no Wi-Fi backend on this system')
        with self.lock:
            self.busy, self.phase, self.error = True, 'leaving', None
            if on_phase:
                on_phase()
            try:
                self.backend.disconnect(self.ours)
                t0 = time.monotonic()
                while time.monotonic() - t0 < LEAVE_TIMEOUT_S and self.joined_ip():
                    time.sleep(0.5)
                self.ours, self.phase = None, 'down'
            finally:
                self.busy = False


if __name__ == '__main__':     # python wifi.py [status|connect|disconnect]
    w = Wifi()
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'status'
    if cmd == 'connect':
        print('joined', w.connect())
    elif cmd == 'disconnect':
        w.disconnect()
    print(w.status())
