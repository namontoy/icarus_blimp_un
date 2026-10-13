"""Open-loop motion tendency of the ESP-Drone from its four motor duties.

Used by drone_gui.py to draw, over the top view, where the airframe would tend to
move in the XY plane and how it would tend to yaw. This is the bare airframe
response to thrust imbalance: in real flight the stabilizer counteracts it, so it
is a didactic indicator, not a flight prediction. Vertical motion (Z) is ignored.

Model:
  * thrust per motor T = (duty / 100) ** THRUST_EXPONENT (props: thrust ~ rpm^2,
    and for brushed motors rpm is roughly proportional to duty)
  * planar: differential thrust tilts the drone away from the stronger side and the
    tilted thrust pushes it that way, so motion ~ -sum(p_i * T_i), p_i = motor position
  * yaw: a CW propeller pushes the body CCW, so CCW tendency ~ (T_cw - T_ccw)

Axes (top view, front up): x + = right, y + = rear (screen-down), so the GUI can draw
the returned vector directly.
"""
import math

THRUST_EXPONENT = 2

# motor -> (x, y, propeller rotation seen from above)
MOTOR_LAYOUT = {1: (-1, +1, 'CW'),    # rear-left
                2: (-1, -1, 'CCW'),   # front-left
                3: (+1, -1, 'CW'),    # front-right
                4: (+1, +1, 'CCW')}   # rear-right

MAX_PLANAR = 2.0    # |sum(p_i * T_i)| with one side at full thrust and the other off
MAX_YAW = 2.0       # both CW motors at full thrust, both CCW off (or vice versa)


def thrust(duty_pct):
    """Normalised thrust 0..1 for a duty in percent."""
    return (min(max(duty_pct, 0.0), 100.0) / 100.0) ** THRUST_EXPONENT


def planar_tendency(duties):
    """Return (dx, dy, magnitude): unit direction (x right, y rear) and 0..1 size.

    duties: {motor: duty %}; missing motors count as 0 %.
    """
    sx = sy = 0.0
    for m, (px, py, _) in MOTOR_LAYOUT.items():
        t = thrust(duties.get(m, 0.0))
        sx -= px * t
        sy -= py * t
    norm = math.hypot(sx, sy)
    if norm < 1e-9:
        return 0.0, 0.0, 0.0
    return sx / norm, sy / norm, min(norm / MAX_PLANAR, 1.0)


def yaw_tendency(duties):
    """Return -1..+1: + = body tends to rotate CCW, - = CW (seen from above)."""
    total = 0.0
    for m, (_, _, rot) in MOTOR_LAYOUT.items():
        t = thrust(duties.get(m, 0.0))
        total += t if rot == 'CW' else -t
    return max(-1.0, min(total / MAX_YAW, 1.0))


def max_tendency(duty_limit_pct):
    """Largest planar magnitude (and |yaw|) reachable when no motor exceeds the limit.

    One side (or one rotation pair) at the limit and the other off gives
    2 * thrust(limit), which normalised by MAX_PLANAR / MAX_YAW is thrust(limit).
    """
    return thrust(duty_limit_pct)


_SECTORS = ('derecha', 'adelante-derecha', 'adelante', 'adelante-izquierda',
            'izquierda', 'atras-izquierda', 'atras', 'atras-derecha')


def describe(dx, dy):
    """Spanish label for a direction (x right, y rear), in 8 sectors of 45 deg."""
    if dx == 0 and dy == 0:
        return 'equilibrado'
    angle = math.degrees(math.atan2(-dy, dx)) % 360   # 0 = right, 90 = front
    return _SECTORS[int((angle + 22.5) // 45) % 8]


if __name__ == '__main__':
    def close(a, b):
        return abs(a - b) < 1e-9

    dx, dy, mag = planar_tendency({m: 40 for m in MOTOR_LAYOUT})
    assert mag == 0 and yaw_tendency({m: 40 for m in MOTOR_LAYOUT}) == 0

    dx, dy, mag = planar_tendency({1: 100, 2: 100})            # left pair only
    assert close(dx, 1) and close(dy, 0) and close(mag, 1)
    assert describe(dx, dy) == 'derecha'

    dx, dy, mag = planar_tendency({1: 100, 4: 100})            # rear pair only
    assert close(dx, 0) and close(dy, -1) and describe(dx, dy) == 'adelante'

    dx, dy, mag = planar_tendency({1: 100})                    # M1 (rear-left) alone
    assert describe(dx, dy) == 'adelante-derecha' and close(mag, math.sqrt(2) / 2)

    assert close(yaw_tendency({1: 100, 3: 100}), 1)            # both CW -> body CCW
    assert planar_tendency({1: 100, 3: 100})[2] == 0
    assert yaw_tendency({2: 50}) < 0                            # CCW prop -> body CW

    dx, dy, mag = planar_tendency({3: 30})                     # M3 (front-right) alone
    assert describe(dx, dy) == 'atras-izquierda' and yaw_tendency({3: 30}) > 0

    assert close(planar_tendency({1: 30, 2: 30})[2], max_tendency(30))
    assert close(yaw_tendency({1: 30, 3: 30}), max_tendency(30))
    print('drone_dynamics self-check OK')
