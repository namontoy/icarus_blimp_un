"""ESP-Drone motor-path diagnostic.

Read-only by default. With --spin N (1-4) it also sets motorPowerSet.mN for
3 seconds and then turns it back off. REMOVE THE PROPELLER before using --spin.

Uses cflib's direct param writes, which avoids the cfclient Parameters tab
stalling on MISC_GET_DEFAULT_VALUE requests that this firmware never answers.

Run with the cfclient virtualenv, connected to the drone's Wi-Fi AP and with
cfclient closed:
    python software/motor_diag.py
    python software/motor_diag.py --spin 4
    python software/motor_diag.py --spin 3 --duty 25
"""
import argparse
import os
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

URI = 'udp://192.168.43.42:2390'
CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cfcache')


def read_logs(scf, variables, samples=5):
    conf = LogConfig(name='diag', period_in_ms=100)
    for name, typ in variables:
        conf.add_variable(name, typ)
    out = []
    with SyncLogger(scf, conf) as logger:
        for entry in logger:
            out.append(entry[1])
            if len(out) >= samples:
                break
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--spin', type=int, choices=[1, 2, 3, 4])
    level = ap.add_mutually_exclusive_group()
    level.add_argument('--value', type=int, default=10000,
                       help='raw motorPowerSet value, 0-65535 (default 10000)')
    level.add_argument('--duty', type=float,
                       help='PWM duty cycle in percent, 0-100')
    args = ap.parse_args()

    if args.duty is not None:
        if not 0.0 <= args.duty <= 100.0:
            ap.error('--duty must be between 0 and 100')
        args.value = round(args.duty / 100.0 * 65535)
    elif not 0 <= args.value <= 65535:
        ap.error('--value must be between 0 and 65535')

    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache=CACHE_DIR)) as scf:
        p = scf.cf.param
        print('--- Parameters ---')
        for name in ['system.selftestPassed', 'imu_tests.mpu6050',
                     'stabilizer.stop', 'system.forceArm',
                     'motorPowerSet.enable', 'motorPowerSet.m1',
                     'motorPowerSet.m2', 'motorPowerSet.m3',
                     'motorPowerSet.m4']:
            try:
                print(f'{name:24s} = {p.get_value(name)}')
            except Exception as e:
                print(f'{name:24s} = <error: {e}>')

        print('\n--- Live state (5 samples, 100 ms) ---')
        rows = read_logs(scf, [('sys.armed', 'int8_t'), ('sys.canfly', 'int8_t'),
                               ('acc.z', 'float'), ('stabilizer.roll', 'float'),
                               ('stabilizer.rtStab', 'float')])
        for r in rows:
            print(r)

        if args.spin:
            m = args.spin
            # motors.c keeps only the top 8 bits: 8-bit LEDC duty at 15 kHz
            duty_bits = args.value >> 8
            print(f'\n--- Spinning M{m} at {args.value} '
                  f'(~{duty_bits / 256 * 100:.1f}% duty, {duty_bits}/255) for 3 s ---')
            p.set_value(f'motorPowerSet.m{m}', str(args.value))
            p.set_value('motorPowerSet.enable', '1')
            time.sleep(0.5)
            for r in read_logs(scf, [(f'pwm.m{m}_pwm', 'uint32_t')], samples=5):
                print(r)
            time.sleep(2.5)
            p.set_value(f'motorPowerSet.m{m}', '0')
            p.set_value('motorPowerSet.enable', '0')
            print('Stopped.')


if __name__ == '__main__':
    main()
