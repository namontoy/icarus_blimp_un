#!/usr/bin/env python3
"""Decode a capture of the blimp's control-loop records (the `.bin` that
`blimp_link.py capture download` or the ground station's Bench page saves).

    python software/blimp_log.py capture.bin                 # summary
    python software/blimp_log.py capture.bin --head 5        # plus the first rows
    python software/blimp_log.py capture.bin --csv out.csv   # every record, in units

Standard library only. The file is a 4-byte magic, the 52-byte header the
firmware sent in its LOG_INFO packet (the scales the counts need), then one
72-byte `log_record_t` per 20 ms tick, exactly as `ctl_types.h` lays it out.
The decoder applies the same scale factors the firmware's estimate stage
will: a bug there shows up here, instead of being baked into the log.

Also a module: the ground station imports `read_file`, `decode` and
`to_units` to plot a capture.
"""
import argparse
import csv
import math
import os
import struct
import sys

FILE_MAGIC = b'ICLG'            # "Icarus log"
FORMAT = 1                      # LINK_LOG_FORMAT: the record layout's version
RECORD_BYTES = 72               # LINK_LOG_RECORD_BYTES

# link_log_info_t without its link header and CRC: 52 bytes.
HEADER = struct.Struct('<BBBBHHIIIIHHfBBH16s')
HEADER_FIELDS = ('state', 'result', 'record_size', 'format', 'capture_id', 'period_us', 'count',
                 'capacity', 'wanted', 't_start_us', 'imu_accel_g', 'imu_gyro_dps', 'mag_lsb_per_gauss',
                 'mag_chip', 'baro_chip', 'max_seconds', 'fw')
assert HEADER.size == 52

# log_record_t, field for field (ctl_types.h); the two 24-bit barometer
# values are taken as 3-byte strings and assembled below.
RECORD = struct.Struct('<I3h3h3h3s3sHHBBBB' + 'BBHHH' * 2 + '4h4hHH')
assert RECORD.size == RECORD_BYTES

MOTORS = ('FL', 'RL', 'FR', 'RR')           # motor_id_t order, as on the wire
STATE = {0: 'idle', 1: 'running', 2: 'done'}
RESULT = {0: 'ok', 1: 'no buffer', 2: 'busy', 3: 'range', 4: 'bad command'}
MAG_CHIP = {0: 'none', 1: 'HMC5883L', 2: 'QMC5883L'}
BARO_CHIP = {0: 'none', 1: 'BMP180', 2: 'BMP280', 3: 'BME280'}
MODE = {0: 'MANUAL', 1: 'OPEN_LOOP', 2: 'CLOSED_LOOP', 3: 'KILLED'}
FRESH = ('imu', 'mag', 'baro', 'tof', 'cam0', 'cam1', 'bat', 'link')               # CTL_FRESH_* bits
FLAGS = ('link_lost', 'cmd_stale', 'killed', 'batt_low', 'suspended', 'clamp_total', 'overrun',
         'motor_fs')                                                             # CTL_FS_* bits

# What a CSV row holds, in order. Counts stay beside the units so nothing is lost.
COLUMNS = (['t_s', 'dt_ms'] +
           ['acc_%s_g' % a for a in 'xyz'] + ['gyro_%s_dps' % a for a in 'xyz'] +
           ['mag_%s_gauss' % a for a in 'xyz'] +
           ['pressure_pa', 'baro_temp_c', 'humidity_pct', 'tof_mm', 'tof_status', 'fresh', 'mode', 'flags'] +
           ['cam%d_%s' % (c, k) for c in (0, 1) for k in ('seq', 'detected', 'cx', 'cy', 'area')] +
           ['setpoint_%s' % m for m in MOTORS] + ['throttle_%s' % m for m in MOTORS] +
           ['bat_v', 'exec_us'] +
           ['acc_%s_raw' % a for a in 'xyz'] + ['gyro_%s_raw' % a for a in 'xyz'] +
           ['mag_%s_raw' % a for a in 'xyz'])


# ---------------------------------------------------------------- decoding

def decode_header(blob):
    """The 52-byte LOG_INFO payload as a dict."""
    d = dict(zip(HEADER_FIELDS, HEADER.unpack(blob[:HEADER.size])))
    d['fw'] = d['fw'].split(b'\0', 1)[0].decode('ascii', 'replace')
    return d


def encode_header(h):
    """The inverse, for the mock and the tests."""
    return HEADER.pack(*[h[k] if k != 'fw' else h[k].encode('ascii')[:16] for k in HEADER_FIELDS])


def u24(b):
    return b[0] | (b[1] << 8) | (b[2] << 16)


def s24(b):
    v = u24(b)
    return v - (1 << 24) if v & 0x800000 else v


# Index of each field in RECORD.unpack()'s tuple (a pair is a slice).
_IDX = {'t_us': 0, 'acc': (1, 4), 'gyro': (4, 7), 'mag': (7, 10), 'baro_p': 10, 'baro_t': 11,
        'baro_h': 12, 'tof_mm': 13, 'tof_status': 14, 'fresh': 15, 'mode': 16, 'fs_flags': 17,
        'cam': 18, 'setpoint': (28, 32), 'throttle': (32, 36), 'bat_mv': 36, 'exec_us': 37}


def decode_record(blob):
    """One 72-byte record as a dict of raw fields (counts and fixed point)."""
    f = RECORD.unpack(blob)
    cams = []
    for c in range(2):
        seq, flags, cx, cy, area = f[_IDX['cam'] + 5 * c:_IDX['cam'] + 5 * c + 5]
        cams.append({'seq': seq, 'detected': flags & 1, 'cx': cx, 'cy': cy, 'area': area})
    return {'t_us': f[_IDX['t_us']],
            'acc': f[slice(*_IDX['acc'])], 'gyro': f[slice(*_IDX['gyro'])], 'mag': f[slice(*_IDX['mag'])],
            'baro_p_dpa': u24(f[_IDX['baro_p']]), 'baro_t_cc': s24(f[_IDX['baro_t']]),
            'baro_h_cpct': f[_IDX['baro_h']], 'tof_mm': f[_IDX['tof_mm']], 'tof_status': f[_IDX['tof_status']],
            'fresh': f[_IDX['fresh']], 'mode': f[_IDX['mode']], 'fs_flags': f[_IDX['fs_flags']],
            'cam': cams, 'setpoint': f[slice(*_IDX['setpoint'])], 'throttle': f[slice(*_IDX['throttle'])],
            'bat_mv': f[_IDX['bat_mv']], 'exec_us': f[_IDX['exec_us']]}


def encode_record(r):
    """A raw record dict back to 72 bytes (the mock builds records this way)."""
    def p24(v):
        v &= 0xFFFFFF
        return bytes((v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF))
    cams = []
    for c in r['cam']:
        cams += [c['seq'] & 0xFF, 1 if c['detected'] else 0, c['cx'], c['cy'], c['area']]
    return RECORD.pack(r['t_us'] & 0xFFFFFFFF, *r['acc'], *r['gyro'], *r['mag'], p24(r['baro_p_dpa']),
                       p24(r['baro_t_cc']), r['baro_h_cpct'], r['tof_mm'], r['tof_status'], r['fresh'],
                       r['mode'], r['fs_flags'], *cams, *r['setpoint'], *r['throttle'], r['bat_mv'],
                       r['exec_us'])


def decode(header_blob, data):
    """(header, records) from the LOG_INFO payload and the concatenated records."""
    h = decode_header(header_blob)
    if h['format'] != FORMAT:
        raise ValueError('record format %d; this decoder knows %d' % (h['format'], FORMAT))
    if h['record_size'] != RECORD_BYTES:
        raise ValueError('record size %d, expected %d' % (h['record_size'], RECORD_BYTES))
    n = len(data) // RECORD_BYTES
    return h, [decode_record(data[i * RECORD_BYTES:(i + 1) * RECORD_BYTES]) for i in range(n)]


def scales(h):
    """Counts to units, from the header. A missing chip gives None, so a
    column of zeros is never passed off as a measurement."""
    acc = 32768.0 / h['imu_accel_g'] if h['imu_accel_g'] else None          # LSB per g
    gyro = 32768.0 / h['imu_gyro_dps'] if h['imu_gyro_dps'] else None       # LSB per deg/s
    mag = h['mag_lsb_per_gauss'] or None
    return acc, gyro, mag


def to_units(h, records):
    """Rows in physical units, one dict per record, keyed as COLUMNS.

    Time is seconds since the first record; dt_ms is the step from the previous
    record (the loop's period, 20 ms, unless a tick was missed). The clock is
    the blimp's 32-bit microsecond counter, which wraps every 71 minutes; a
    wrap inside a capture is unwrapped here."""
    acc_lsb, gyro_lsb, mag_lsb = scales(h)
    rows, t0, prev = [], None, None
    for r in records:
        t = r['t_us']
        if prev is not None and t < prev:
            t += 1 << 32
        if t0 is None:
            t0 = t
        row = {'t_s': (t - t0) / 1e6, 'dt_ms': (t - prev) / 1e3 if prev is not None else float('nan')}
        prev = t
        for i, a in enumerate('xyz'):
            row['acc_%s_g' % a] = r['acc'][i] / acc_lsb if acc_lsb else float('nan')
            row['gyro_%s_dps' % a] = r['gyro'][i] / gyro_lsb if gyro_lsb else float('nan')
            row['mag_%s_gauss' % a] = r['mag'][i] / mag_lsb if mag_lsb else float('nan')
            row['acc_%s_raw' % a] = r['acc'][i]
            row['gyro_%s_raw' % a] = r['gyro'][i]
            row['mag_%s_raw' % a] = r['mag'][i]
        have_baro = h['baro_chip'] != 0
        row['pressure_pa'] = r['baro_p_dpa'] / 10.0 if have_baro else float('nan')
        row['baro_temp_c'] = r['baro_t_cc'] / 100.0 if have_baro else float('nan')
        row['humidity_pct'] = r['baro_h_cpct'] / 100.0 if h['baro_chip'] == 3 else float('nan')
        row['tof_mm'] = r['tof_mm']
        row['tof_status'] = r['tof_status']
        row['fresh'] = r['fresh']
        row['mode'] = MODE.get(r['mode'], str(r['mode']))
        row['flags'] = r['fs_flags']
        for c in (0, 1):
            for k in ('seq', 'detected', 'cx', 'cy', 'area'):
                row['cam%d_%s' % (c, k)] = r['cam'][c][k]
        for i, m in enumerate(MOTORS):
            row['setpoint_%s' % m] = r['setpoint'][i] / 32767.0
            row['throttle_%s' % m] = r['throttle'][i] / 32767.0
        row['bat_v'] = r['bat_mv'] / 1000.0
        row['exec_us'] = r['exec_us']
        rows.append(row)
    return rows


def bits(value, names):
    return [n for i, n in enumerate(names) if value & (1 << i)]


# ------------------------------------------------------------------- files

def write_file(path, header_blob, data):
    with open(path, 'wb') as f:
        f.write(FILE_MAGIC + header_blob[:HEADER.size] + data)


def read_file(path):
    """(header, records) of a .bin written by write_file()."""
    with open(path, 'rb') as f:
        blob = f.read()
    if blob[:4] != FILE_MAGIC:
        raise ValueError('%s is not a capture file (magic %r)' % (path, blob[:4]))
    return decode(blob[4:4 + HEADER.size], blob[4 + HEADER.size:])


def write_csv(path, h, records):
    rows = to_units(h, records)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        for row in rows:
            w.writerow({k: ('%.6g' % v if isinstance(v, float) else v) for k, v in row.items()})
    return len(rows)


# ----------------------------------------------------------------- summary

def _stats(values):
    vals = [v for v in values if isinstance(v, (int, float)) and not math.isnan(v)]
    if not vals:
        return None
    n = len(vals)
    mean = sum(vals) / n
    std = math.sqrt(sum((v - mean) ** 2 for v in vals) / n) if n > 1 else 0.0
    return {'n': n, 'min': min(vals), 'max': max(vals), 'mean': mean, 'std': std}


def summary(h, records):
    """What a bench check wants to know: count, timing, which sensors were
    live and how often, the loop's execution time, and the flags seen."""
    rows = to_units(h, records)
    n = len(rows)
    out = {'records': n, 'header': h, 'duration_s': rows[-1]['t_s'] if rows else 0.0}
    dt = _stats([r['dt_ms'] for r in rows[1:]])
    out['dt_ms'] = dt
    period_ms = h['period_us'] / 1000.0
    out['gaps'] = sum(1 for r in rows[1:] if r['dt_ms'] > 1.5 * period_ms)
    out['fresh_hz'] = {}
    dur = out['duration_s'] or float('nan')
    for i, name in enumerate(FRESH):
        k = sum(1 for r in records if r['fresh'] & (1 << i))
        out['fresh_hz'][name] = k / dur if dur and k else 0.0
    out['exec_us'] = _stats([r['exec_us'] for r in records])
    out['overruns'] = sum(1 for r in records if r['fs_flags'] & (1 << 6))
    flags = 0
    for r in records:
        flags |= r['fs_flags']
    out['flags_seen'] = bits(flags, FLAGS)
    out['modes_seen'] = sorted({MODE.get(r['mode'], str(r['mode'])) for r in records})
    for key in ('acc_x_g', 'acc_y_g', 'acc_z_g', 'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps',
                'mag_x_gauss', 'mag_y_gauss', 'mag_z_gauss', 'pressure_pa', 'baro_temp_c', 'tof_mm'):
        out[key] = _stats([r[key] for r in rows])
    return out


def format_summary(s):
    h = s['header']
    lines = ['%d records, %.2f s at %.0f ms, capture %d, firmware %s' % (
        s['records'], s['duration_s'], h['period_us'] / 1000.0, h['capture_id'], h['fw'] or '?')]
    lines.append('scales: accel +/-%d g, gyro +/-%d deg/s, mag %s %.0f LSB/gauss, baro %s' % (
        h['imu_accel_g'], h['imu_gyro_dps'], MAG_CHIP.get(h['mag_chip'], '?'), h['mag_lsb_per_gauss'],
        BARO_CHIP.get(h['baro_chip'], '?')))
    dt = s['dt_ms']
    if dt:
        lines.append('step: %.3f / %.3f / %.3f ms (min / mean / max), std %.3f ms, %d gaps' % (
            dt['min'], dt['mean'], dt['max'], dt['std'], s['gaps']))
    ex = s['exec_us']
    if ex:
        lines.append('exec: %d / %.0f / %d us (min / mean / max), %d overruns' % (
            ex['min'], ex['mean'], ex['max'], s['overruns']))
    lines.append('fresh per second: ' + '  '.join('%s %.1f' % (k, v) for k, v in s['fresh_hz'].items() if v))
    if s['flags_seen']:
        lines.append('flags seen: ' + ', '.join(s['flags_seen']))
    lines.append('modes: ' + ', '.join(s['modes_seen']))
    for key, unit, d in (('acc_x_g', 'g', 3), ('acc_y_g', 'g', 3), ('acc_z_g', 'g', 3),
                         ('gyro_x_dps', 'deg/s', 2), ('gyro_y_dps', 'deg/s', 2), ('gyro_z_dps', 'deg/s', 2),
                         ('mag_x_gauss', 'G', 3), ('mag_y_gauss', 'G', 3), ('mag_z_gauss', 'G', 3),
                         ('pressure_pa', 'Pa', 1), ('baro_temp_c', 'C', 2), ('tof_mm', 'mm', 0)):
        st = s.get(key)
        if st:
            lines.append('%-12s mean %*.*f  std %.*f  min %.*f  max %.*f %s' % (
                key, 10, d, st['mean'], d, st['std'], d, st['min'], d, st['max'], unit))
    return '\n'.join(lines)


# --------------------------------------------------------------------- CLI

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('file', help='a .bin from blimp_link.py capture download')
    ap.add_argument('--csv', metavar='OUT', help='write every record in units to this CSV')
    ap.add_argument('--head', type=int, default=0, metavar='N', help='print the first N rows')
    args = ap.parse_args(argv)
    h, records = read_file(args.file)
    print(format_summary(summary(h, records)))
    if args.head:
        rows = to_units(h, records)
        keys = ('t_s', 'dt_ms', 'acc_x_g', 'acc_y_g', 'acc_z_g', 'gyro_z_dps', 'mag_x_gauss', 'pressure_pa',
                'tof_mm', 'fresh', 'throttle_FL', 'exec_us')
        print('  '.join('%12s' % k for k in keys))
        for row in rows[:args.head]:
            print('  '.join('%12.4g' % row[k] if isinstance(row[k], float) else '%12s' % row[k] for k in keys))
    if args.csv:
        n = write_csv(args.csv, h, records)
        print('%d rows written to %s' % (n, args.csv))


if __name__ == '__main__':
    main()
