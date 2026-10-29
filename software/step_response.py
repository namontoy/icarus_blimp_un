"""Fit the step responses in a capture: gain, time constant and delay per
step, the plant numbers they imply, and the gains the tuning rules give.

    python software/step_response.py captures/step-yaw-<date>.bin --axis yaw --all
    python software/step_response.py captures/step-heave-<date>.bin --axis heave --tau-c 0.6
    python software/step_response.py captures/step-yaw-<date>.bin --axis yaw --all \\
        --thrust-map captures/thrust-map.json --half-span 0.10
    python software/step_response.py --selftest

Reads the .bin that `blimp_link.py capture download`, the Bench page or
`step_test.py` writes, finds every change of the stimulus that is held for
MIN_HOLD_S, and fits the response after each one:

  axis   stimulus u (throttle readback, per unit duty)   response y            model
  yaw    FL - FR, the front differential                  gyro_z_dps           rate: first order plus delay
  heave  (RL + RR) / 2, the rear pair                      tof_mm / 1000, m     position: the integral of that lag

Both are one plant seen through two sensors: a velocity (the yaw rate, the
climb rate) that answers a thrust step as a first-order lag behind a delay,
K (1 - exp(-(t - th) / tau)); the gyro reads the velocity, the range sensor
its integral. K is per unit of duty. The baseline before a step is a constant
for the rate model and a straight line for the position model (the vehicle
sinks steadily before the rear pair steps: that is the protocol of plan H1),
so a down-step, the spin-down of Y2 and the descent of H2 fit with the same
code. K is solved by least squares for every (tau, th) of a grid, so no
starting guess is needed. Two quality figures: MATLAB's NRMSE fit percentage
(docs/07 section 8 asks for 90 %) and the residual over the baseline's own
noise, which is the one to trust when the hold is long (a flat tail adds
noise and no spread, and the percentage drops although the model is right).

Physics. Yaw: I_z w' + D_yaw w = F l_f, with F the differential thrust, so
K = F l_f / (D_yaw u) and tau = I_z / D_yaw. Heave: m_eff v' + D_z v = F_pair,
so K = F_pair / (D_z u) and tau = m_eff / D_z. Given the thrust per unit duty
(thrust_map.py's JSON, or --gf-per-unit) and l_f, the tool prints D and I_z
or m_eff, and from them the gains three ways:

  - params.c's placement, kp = J wn^2 and kd = 2 zeta wn J with J = I_z or
    m_eff, plus the kd that gives the same damping once the measured drag is
    counted (the formula ignores drag, so on a draggy vehicle it over-damps);
  - SIMC (Skogestad 2003; Skogestad and Grimholt, "The SIMC method for smooth
    PID controller tuning", table 1, verified 2026-10-28): the "integrating
    with lag" rule for the plant k' e^(-th s) / (s (tau s + 1)) from thrust
    to heading or height, Kc = 1 / (k' (tau_c + th)), tau_I = 4 (tau_c + th),
    tau_D = tau, in the series form, converted to the parallel form
    blimp_pid_step uses: kp = Kc (1 + tau_D / tau_I), kd = Kc tau_D,
    ki = Kc / tau_I. ki stays zero by decision (docs/07 section 4.3), so kp
    and kd are what a bench session takes; tau_c is the one knob, 0.5 s by
    default (Skogestad's "tight" tau_c = th would ask for a bandwidth the
    100 gf budget cannot deliver);
  - the delay check, wn th under 0.5 rad (section 1 of the plan).

Gains come out in core/'s units: N m/rad and N m s/rad for yaw, N/m and
N s/m for heave. Standard library only.
"""
import argparse
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blimp_log  # noqa: E402

MIN_HOLD_S = 0.5        # a stimulus change shorter than this is not a step
STEP_MIN = 0.02         # per unit duty: below this the readback is noise or a trim
TAU_GRID = [0.02 * 1.12 ** i for i in range(60)]      # 20 ms .. 11 s, geometric
THETA_MAX_S = 0.6       # command-to-motion delays beyond this are a wiring problem, not a plant
PRE_S = 1.0             # baseline window before a step, s (shorter if the previous step ends later)
GF_TO_N = 0.00980665    # BLIMP_GF_TO_N

AXES = {
    'yaw': {'u': lambda r: r['throttle_FL'] - r['throttle_FR'], 'y': lambda r: r['gyro_z_dps'],
            'model': 'rate', 'y_unit': 'deg/s', 'u_name': 'FL-FR', 'motors': ('FL', 'FR'),
            'J': 'I_z', 'J_unit': 'kg m^2', 'D': 'D_yaw', 'D_unit': 'N m s/rad',
            'gain_units': ('N m/rad', 'N m s/rad'), 'loop': 'yaw', 'drive_unit': 'N m'},
    'heave': {'u': lambda r: (r['throttle_RL'] + r['throttle_RR']) / 2, 'y': lambda r: r['tof_mm'] / 1000.0,
              'model': 'position', 'y_unit': 'm', 'u_name': '(RL+RR)/2', 'motors': ('RL', 'RR'),
              'J': 'm_eff', 'J_unit': 'kg', 'D': 'D_z', 'D_unit': 'N s/m',
              'gain_units': ('N/m', 'N s/m'), 'loop': 'alt', 'drive_unit': 'N'},
}


# ------------------------------------------------------------------ models

def rate_shape(t, tau, th):
    return 0.0 if t < th else 1.0 - math.exp(-(t - th) / tau)


def position_shape(t, tau, th):
    if t < th:
        return 0.0
    x = t - th
    return x - tau * (1.0 - math.exp(-x / tau))


SHAPES = {'rate': rate_shape, 'position': position_shape}


def fit(t, y, model):
    """Least-squares (K, tau, th) over the grid, plus the NRMSE fit in percent.
    t starts at the step edge (t[0] == 0) and is sampled uniformly: the delay
    is searched in whole samples by shifting each tau's shape, which is what
    makes the grid affordable in plain Python. y has its baseline removed."""
    shape = SHAPES[model]
    n = len(t)
    mean = sum(y) / n
    spread = math.sqrt(sum((v - mean) ** 2 for v in y) / n) or 1e-9
    dt = (t[-1] - t[0]) / (n - 1) if n > 1 else 0.02
    kmax = max(0, min(n - 2, int(THETA_MAX_S / dt)))
    best = None
    for tau in TAU_GRID:
        f0 = [shape(tk, tau, 0.0) for tk in t]
        for k in range(kmax + 1):
            f = [0.0] * k + f0[:n - k]
            ff = sum(v * v for v in f)
            if ff <= 0:
                continue
            gain = sum(a * b for a, b in zip(y, f)) / ff
            rmse = math.sqrt(sum((a - gain * b) ** 2 for a, b in zip(y, f)) / n)
            if best is None or rmse < best[0]:
                best = (rmse, gain, tau, k * dt)
    rmse, gain, tau, th = best
    return {'K': gain, 'tau': tau, 'theta': th, 'rmse': rmse, 'fit_pct': 100.0 * (1.0 - rmse / spread)}


def response_times(m):
    """Rise time 10-90 % and the time to within 5 % of the final value, from
    the fitted lag (ln 9 tau, and 3 tau after the delay): the measured
    samples are too noisy for threshold crossings to mean much."""
    return math.log(9.0) * m['tau'], m['theta'] + 3.0 * m['tau']


# ------------------------------------------------------------------- steps

def find_steps(t, u):
    """[(i, j)] for every change of the stimulus held for MIN_HOLD_S: i is
    the first sample at the new level, j the first sample that leaves it
    (or len(u)). A shorter change is a glitch and is skipped."""
    n = len(u)
    out = []
    level = u[0]
    i = 1
    while i < n:
        if abs(u[i] - level) >= STEP_MIN:
            j = i + 1
            while j < n and abs(u[j] - u[i]) < STEP_MIN:
                j += 1
            if t[min(j, n - 1)] - t[i] >= MIN_HOLD_S:
                out.append((i, j))
            level = u[i]
            i = j
        else:
            i += 1
    return out


def find_step(t, u):
    """The first step, as (i, j), or (None, None)."""
    steps = find_steps(t, u)
    return steps[0] if steps else (None, None)


def baseline(t, y, i, model, start, dt):
    """What the response was doing before the step: its mean for the rate
    model, a straight line for the position model (a steady sink or climb),
    over PRE_S or since the previous step ended. Returns (base(tk), noise,
    v0, n): the baseline as a function of time, the residual's standard
    deviation, the slope (position model only) and the samples used."""
    lo = max(start, i - int(round(PRE_S / dt)), 0)
    pre = [(t[k], y[k]) for k in range(lo, i) if y[k] == y[k]]
    n = len(pre)
    if n == 0:
        return (lambda tk: 0.0), float('nan'), float('nan'), 0
    ti = t[i]
    if model == 'position' and n >= 4:
        xs = [p[0] - ti for p in pre]
        ys = [p[1] for p in pre]
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        v0 = sum((x - mx) * (yv - my) for x, yv in zip(xs, ys)) / sxx if sxx > 0 else 0.0
        y0 = my - v0 * mx
        resid = [yv - (y0 + v0 * x) for x, yv in zip(xs, ys)]
        noise = math.sqrt(sum(r * r for r in resid) / n) if n > 4 else float('nan')
        return (lambda tk: y0 + v0 * (tk - ti)), noise, v0, n
    m = sum(p[1] for p in pre) / n
    noise = math.sqrt(sum((p[1] - m) ** 2 for p in pre) / n) if n > 3 else float('nan')
    return (lambda tk: m), noise, float('nan'), n


def _dt(t):
    d = sorted(b - a for a, b in zip(t, t[1:]) if b > a)
    return d[len(d) // 2] if d else 0.02


def analyse_step(rows, axis, i, j, prev_end, tau_c):
    """Fit the step at rows[i:j]; prev_end bounds the baseline window."""
    a = AXES[axis]
    t = [r['t_s'] for r in rows]
    u = [a['u'](r) for r in rows]
    y = [a['y'](r) for r in rows]
    dt = _dt(t)
    base, noise, v0, npre = baseline(t, y, i, a['model'], prev_end, dt)
    seg = [(tk - t[i], v - base(tk)) for tk, v in zip(t[i:j], y[i:j]) if v == v]     # drop NaN rows
    seg_t, seg_y = [p[0] for p in seg], [p[1] for p in seg]
    if len(seg_t) < 5:
        return None
    du = u[i] - u[i - 1]
    m = fit(seg_t, seg_y, a['model'])
    # The residual against the baseline's own noise: about 1 means the model
    # explains everything the sensor lets it.
    m['resid_vs_noise'] = m['rmse'] / noise if noise and noise == noise else float('nan')
    rise, settle = response_times(m)
    per_unit = dict(m, K=m['K'] / du)
    return {'axis': axis, 'model': a['model'], 'step_at_s': t[i], 'hold_s': seg_t[-1], 'du': du, 'level': u[i],
            'baseline': base(t[i]), 'v0': v0, 'baseline_n': npre, 'noise': noise, 'fit': m, 'per_unit': per_unit,
            'rise_s': rise, 'settle_s': settle, 'n': len(seg_t), 'simc': simc(per_unit, tau_c), 'tau_c': tau_c,
            'u_name': a['u_name'], 'y_unit': a['y_unit'], 'index': (i, j)}


def analyse_all(rows, axis, tau_c):
    """Every step in the capture, in order."""
    a = AXES[axis]
    t = [r['t_s'] for r in rows]
    u = [a['u'](r) for r in rows]
    out = []
    prev_end = 0
    for i, j in find_steps(t, u):
        r = analyse_step(rows, axis, i, j, prev_end, tau_c)
        if r:
            out.append(r)
        prev_end = i
    return out


def analyse(rows, axis, tau_c):
    """The first step only (what the 2026-10-28 tool did)."""
    r = analyse_all(rows, axis, tau_c)
    return r[0] if r else None


# ------------------------------------------------------------------- gains

def simc(m, tau_c):
    """SIMC for the plant the fit describes, per unit of DUTY: thrust to
    heading or height is integrating with a lag, k' e^(-th s) / (s (tau s +
    1)), k' = K the velocity per unit, so (table 1 of Skogestad and Grimholt)
    Kc = 1 / (k' (tau_c + th)), tau_I = 4 (tau_c + th), tau_D = tau in the
    series form; then the parallel form core/'s blimp_pid_step uses."""
    k, tau, th = m['K'], m['tau'], m['theta']
    kc = 1.0 / (k * (tau_c + th)) if k else float('nan')
    ti = 4.0 * (tau_c + th)
    td = tau
    return {'Kc': kc, 'tau_I': ti, 'tau_D': td, 'kp': kc * (1.0 + td / ti), 'kd': kc * td, 'ki': kc / ti,
            'bandwidth_rads': 1.0 / (tau_c + th)}


def physical(r, n_per_unit, half_span, wn, zeta):
    """Plant numbers and gains in core/'s units, from a fitted step and the
    thrust per unit of its stimulus (N per unit duty: one motor's for yaw,
    the pair's for heave)."""
    axis = r['axis']
    p = r['per_unit']
    tau, th = p['tau'], p['theta']
    if axis == 'yaw':
        k_vel = p['K'] * math.pi / 180.0            # rad/s per unit duty
        drive = n_per_unit * half_span               # N m per unit duty
        sign_ok = p['K'] < 0                         # +FL-FR turns the nose right: clockwise, negative yaw in core/
    else:
        k_vel = p['K']                               # m/s per unit duty
        drive = n_per_unit                           # N per unit duty
        sign_ok = p['K'] > 0                         # the rear pair pushes up and the range grows
    kv = abs(k_vel)
    damp = drive / kv if kv else float('nan')        # D: the steady state, drive = D * velocity
    inertia = tau * damp                             # J = tau * D
    kp_place = inertia * wn * wn
    kd_place = 2.0 * zeta * wn * inertia
    kd_drag = max(0.0, kd_place - damp)              # the drag already damps; the loop adds the rest
    zeta_formula = (kd_place + damp) / (2.0 * math.sqrt(kp_place * inertia)) if kp_place > 0 else float('nan')
    kprime = kv / drive if drive else float('nan')   # velocity per unit of core/'s output
    kc = 1.0 / (kprime * (r['tau_c'] + th)) if kprime else float('nan')
    ti = 4.0 * (r['tau_c'] + th)
    td = tau
    return {'drive_per_unit': drive, 'thrust_n_per_unit': n_per_unit, 'half_span_m': half_span, 'sign_ok': sign_ok,
            'D': damp, 'J': inertia, 'wn': wn, 'zeta': zeta, 'kp_place': kp_place, 'kd_place': kd_place,
            'kd_drag': kd_drag, 'zeta_formula': zeta_formula, 'wn_theta': wn * th,
            'simc': {'Kc': kc, 'tau_I': ti, 'tau_D': td, 'kp': kc * (1.0 + td / ti), 'kd': kc * td, 'ki': kc / ti,
                     'bandwidth_rads': 1.0 / (r['tau_c'] + th)}}


def thrust_per_unit(axis, du, gf_per_unit=None, tmap=None):
    """N per unit of the axis's stimulus at the amplitude of this step: one
    motor's thrust for yaw (the differential), both motors' for heave. From
    the thrust map's curves at |du| (the step is from zero, so F(u) / u is
    the gain that acted), or from a flat --gf-per-unit."""
    a = AXES[axis]
    amp = abs(du)
    if tmap is not None:
        import thrust_map
        per_motor = [thrust_map.thrust_gf(tmap, m, amp) / amp for m in a['motors']]
        gf = sum(per_motor) / len(per_motor) if axis == 'yaw' else sum(per_motor)
    elif gf_per_unit is not None:
        gf = gf_per_unit if axis == 'yaw' else 2.0 * gf_per_unit
    else:
        return None
    return gf * GF_TO_N


# ------------------------------------------------------------------ report

def report(r, phys=None):
    f, p, s = r['fit'], r['per_unit'], r['simc']
    a = AXES[r['axis']]
    unit_v = r['y_unit'] if r['model'] == 'rate' else r['y_unit'] + '/s'
    lines = [
        '%s: step of %+.3f in %s to %+.3f at t = %.2f s, held %.2f s (%d samples); baseline %.3f %s%s'
        % (r['axis'], r['du'], r['u_name'], r['level'], r['step_at_s'], r['hold_s'], r['n'], r['baseline'],
           r['y_unit'], (' moving at %+.3f %s' % (r['v0'], unit_v)) if r['v0'] == r['v0'] else ''),
        '  %s model: K = %+.4g %s per unit duty, tau = %.3f s, delay = %.3f s, fit %.1f %% (residual %.2f x the '
        'baseline noise)' % (r['model'], p['K'], unit_v, f['tau'], f['theta'], f['fit_pct'], f['resid_vs_noise']),
        '  from the fit: rise 10-90 %% %.2f s, within 5 %% of the final value %.2f s after the step%s'
        % (r['rise_s'], r['settle_s'],
           ('; delay + tau = %.2f s is what the range pins down, the split moves by a few samples'
            % (f['theta'] + f['tau'])) if r['model'] == 'position' else ''),
        '  SIMC per unit duty, tau_c = %.2f s: Kc = %.4g, tau_I = %.2f s, tau_D = %.3f s; parallel kp = %.4g, '
        'kd = %.4g duty per %s' % (r['tau_c'], s['Kc'], s['tau_I'], s['tau_D'], s['kp'], s['kd'], r['y_unit']),
    ]
    if f['fit_pct'] < 90 and not f['resid_vs_noise'] < 1.5:
        lines.append('  fit below 90 % and the residual well above the noise: the model does not describe this '
                     'response (a bigger step, a quieter rig, or a second time constant)')
    if f['theta'] >= THETA_MAX_S - 1e-9:
        lines.append('  delay at the search limit: the response did not start within %.1f s' % THETA_MAX_S)
    if phys is None:
        lines.append('  per unit duty: give --thrust-map or --gf-per-unit%s for %s, %s and the gains in core/ units'
                     % (' and --half-span' if r['axis'] == 'yaw' else '', a['D'], a['J']))
        return '\n'.join(lines)
    ku, kdu = a['gain_units']
    ps = phys['simc']
    lines += [
        '  with %.4g N per unit duty%s: %s = %.4g %s, %s = %.4g %s%s'
        % (phys['thrust_n_per_unit'], (' and l_f = %.3f m' % phys['half_span_m']) if r['axis'] == 'yaw' else '',
           a['D'], phys['D'], a['D_unit'], a['J'], phys['J'], a['J_unit'],
           '' if phys['sign_ok'] else '  (SIGN: K has the wrong sign for core/\'s frame; check the motor order or '
           'the gyro axis)'),
        '  params.c placement, wn = %.2f rad/s, zeta = %.2f: %s.kp = %.4g %s, %s.kd = %.4g %s; with the measured '
        'drag the formula gives zeta %.2f, and kd = %.4g keeps zeta %.2f'
        % (phys['wn'], phys['zeta'], a['loop'], phys['kp_place'], ku, a['loop'], phys['kd_place'], kdu,
           phys['zeta_formula'], phys['kd_drag'], phys['zeta']),
        '  SIMC integrating with lag, tau_c = %.2f s: Kc = %.4g %s, tau_I = %.2f s, tau_D = %.3f s; parallel '
        '%s.kp = %.4g %s, %s.kd = %.4g %s (ki would be %.4g, zero by decision); bandwidth %.2f rad/s'
        % (r['tau_c'], ps['Kc'], ku, ps['tau_I'], ps['tau_D'], a['loop'], ps['kp'], ku, a['loop'], ps['kd'], kdu,
           ps['ki'], ps['bandwidth_rads']),
        '  delay check: wn theta = %.2f rad (%s)' % (phys['wn_theta'], 'ok, under 0.5' if phys['wn_theta'] < 0.5
                                                     else 'too much phase lag at this wn: lower it'),
    ]
    return '\n'.join(lines)


def summarise(results):
    """The plan's pass criteria over the steps of one capture: fit quality
    per step, K within 20 % across steps, tau up against tau down (Y2),
    theta under 0.1 s."""
    if not results:
        return 'no step found'
    good = [r for r in results if r['fit']['fit_pct'] >= 90 or r['fit']['resid_vs_noise'] < 1.5]
    ks = [r['per_unit']['K'] for r in results]
    k_spread = (max(ks) - min(ks)) / abs(sum(ks) / len(ks)) if sum(ks) else float('nan')
    ups = [r['fit']['tau'] for r in results if r['du'] > 0]
    downs = [r['fit']['tau'] for r in results if r['du'] < 0]
    th = max(r['fit']['theta'] for r in results)
    # yaw: the tick and the motor's spin-up; heave: plus the ToF's timing budget and a poll (plan L1)
    th_max = 0.1 if results[0]['axis'] == 'yaw' else 0.25
    lines = ['summary of %d steps: %d fit the model (90 %% or residual under 1.5x noise)' % (len(results), len(good)),
             '  K per unit: %s; spread %.0f %% of the mean (plan asks under 20 %%)'
             % (', '.join('%+.3g' % k for k in ks), 100 * k_spread)]
    if ups and downs:
        tu, td = sum(ups) / len(ups), sum(downs) / len(downs)
        lines.append('  tau: steps up %.3f s, steps down %.3f s (Y2/H2 ask them within 20 %%: %s)'
                     % (tu, td, 'ok' if abs(tu - td) <= 0.2 * max(tu, td) else 'no'))
    lines.append('  delay: at most %.3f s (%s %.2f s)%s'
                 % (th, 'ok, under' if th < th_max else 'over', th_max,
                    '; the position model trades delay against tau at the onset, so L1 takes the delay from the gyro'
                    if results[0]['model'] == 'position' else ''))
    return '\n'.join(lines)


def serialisable(r, phys=None):
    """The result as plain numbers for a .json sidecar."""
    out = {k: v for k, v in r.items() if k not in ('index',)}
    out['index'] = list(r['index'])
    if phys:
        out['physical'] = phys
    return out


# ---------------------------------------------------------------- selftest

def _simulate(dt, n, u_of_t, k, tau, th, model, y0=0.0, v0=0.0):
    """Discrete first-order lag behind a delay, exact per sample; the
    position model integrates it (with a constant drift v0)."""
    out = []
    vel = 0.0
    pos = y0
    a = 1.0 - math.exp(-dt / tau)
    lag = int(round(th / dt))
    hist = [0.0] * (lag + 1)
    for i in range(n):
        t = i * dt
        hist.append(u_of_t(t))
        ud = hist[-1 - lag]
        out.append(vel if model == 'rate' else pos)
        new = vel + a * (k * ud - vel)
        pos += (v0 + 0.5 * (vel + new)) * dt      # trapezoid: no extra sample of delay on the position
        vel = new
    return out


def selftest():
    rnd = random.Random(1)
    dt = 0.02
    ok = True
    # yaw: K = -40 deg/s per unit, tau 0.35 s, delay 0.06 s, 0.4 deg/s noise;
    # FL 0.25 for 2-6 s (up and down steps), then FR 0.25 for 10-14 s (the mirror).
    fl = lambda t: 0.25 if 2.0 <= t < 6.0 else 0.0
    fr = lambda t: 0.25 if 10.0 <= t < 14.0 else 0.0
    y = _simulate(dt, 900, lambda t: fl(t) - fr(t), -40.0, 0.35, 0.06, 'rate')
    rows = [{'t_s': i * dt, 'throttle_FL': fl(i * dt), 'throttle_FR': fr(i * dt), 'throttle_RL': 0.0,
             'throttle_RR': 0.0, 'gyro_z_dps': y[i] + 0.4 * rnd.gauss(0, 1), 'tof_mm': 1000} for i in range(900)]
    res = analyse_all(rows, 'yaw', 0.5)
    print('yaw: %d steps found (4 expected)' % len(res))
    for r in res:
        p = r['per_unit']
        good = (abs(p['K'] + 40) < 2.5 and abs(r['fit']['tau'] - 0.35) < 0.06 and abs(r['fit']['theta'] - 0.06) < 0.03
                and r['fit']['resid_vs_noise'] < 1.3)
        ok = ok and good
        print(report(r))
        print('  selftest: %s' % ('ok' if good else 'FAIL'))
    ok = ok and len(res) == 4
    print(summarise(res))
    # the physical conversion, with 15 gf per unit duty and l_f = 0.10 m: the plant that made
    # the data has D_yaw = drive / |K| and I_z = tau * D_yaw
    drive = 15.0 * GF_TO_N * 0.10
    want_d = drive / (40.0 * math.pi / 180.0)
    ph = physical(res[0], 15.0 * GF_TO_N, 0.10, 2.0, 1.0)
    good = abs(ph['D'] - want_d) < 0.07 * want_d and abs(ph['J'] - 0.35 * want_d) < 0.2 * 0.35 * want_d and ph['sign_ok']
    ok = ok and good
    print(report(res[0], ph))
    print('  selftest physical: %s (want D_yaw %.4g, I_z %.4g)' % ('ok' if good else 'FAIL', want_d, 0.35 * want_d))
    # heave: a range that integrates a first-order climb rate, K = 0.6 m/s per unit, tau 0.8 s,
    # delay 0.1 s, 8 mm noise; the vehicle sinks at 0.05 m/s all along (the trim), as plan H1 wants
    u = lambda t: 0.3 if 3.0 <= t < 9.0 else 0.0
    y = _simulate(dt, 800, u, 0.6, 0.8, 0.1, 'position', y0=1.6, v0=-0.05)
    rows = [{'t_s': i * dt, 'throttle_FL': 0.0, 'throttle_FR': 0.0, 'throttle_RL': u(i * dt), 'throttle_RR': u(i * dt),
             'gyro_z_dps': 0.0, 'tof_mm': 1000 * y[i] + 8 * rnd.gauss(0, 1)} for i in range(800)]
    res = analyse_all(rows, 'heave', 0.6)
    print('heave: %d steps found (2 expected)' % len(res))
    for r in res:
        p = r['per_unit']
        # the range's smooth onset trades the delay against tau: their sum is what the fit pins down
        good = (abs(p['K'] - 0.6) < 0.06 and abs(r['fit']['tau'] - 0.8) < 0.2 and r['fit']['resid_vs_noise'] < 1.5
                and abs(r['fit']['theta'] - 0.1) < 0.08 and abs(r['fit']['theta'] + r['fit']['tau'] - 0.9) < 0.1)
        ok = ok and good
        print(report(r))
        print('  selftest: %s' % ('ok' if good else 'FAIL'))
    ok = ok and len(res) == 2 and abs(res[0]['v0'] + 0.05) < 0.02
    print(summarise(res))
    print('selftest: %s' % ('ok' if ok else 'FAIL'))
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('file', nargs='?', help='a .bin from blimp_link.py capture download, the Bench page or step_test.py')
    ap.add_argument('--axis', choices=sorted(AXES), default='yaw')
    ap.add_argument('--tau-c', type=float, default=0.5, help='SIMC closed-loop time constant, s (default %(default)s)')
    ap.add_argument('--all', action='store_true', help='fit every step, not only the first, and summarise')
    ap.add_argument('--step', type=int, help='fit only step N (1-based)')
    ap.add_argument('--thrust-map', help='thrust_map.py fit JSON: converts the fit to core/ units')
    ap.add_argument('--gf-per-unit', type=float, help='one motor\'s thrust per unit duty, gf, instead of the map')
    ap.add_argument('--half-span', type=float, default=0.10,
                    help='l_f, m: the front motors\' half span (default params.c\'s %(default)s placeholder)')
    ap.add_argument('--wn', type=float, default=2.0, help='placement natural frequency, rad/s (default %(default)s)')
    ap.add_argument('--zeta', type=float, default=1.0, help='placement damping ratio (default %(default)s)')
    ap.add_argument('--json', help='write the results to this file')
    ap.add_argument('--selftest', action='store_true', help='fit synthetic responses and check the numbers')
    args = ap.parse_args(argv)
    if args.selftest:
        return 0 if selftest() else 1
    if not args.file:
        ap.error('a capture file, or --selftest')
    tmap = None
    if args.thrust_map:
        with open(args.thrust_map) as f:
            tmap = json.load(f)
    h, records = blimp_log.read_file(args.file)
    rows = blimp_log.to_units(h, records)
    results = analyse_all(rows, args.axis, args.tau_c)
    if not results:
        print('no step found in %s: the %s readback never changes by %.2f for %.1f s'
              % (args.file, AXES[args.axis]['u_name'], STEP_MIN, MIN_HOLD_S))
        return 1
    if args.step:
        if not 1 <= args.step <= len(results):
            print('%d steps in the capture' % len(results))
            return 1
        results = [results[args.step - 1]]
    elif not args.all:
        results = results[:1]
    out = []
    for k, r in enumerate(results, 1):
        n_per_unit = thrust_per_unit(args.axis, r['du'], args.gf_per_unit, tmap)
        ph = physical(r, n_per_unit, args.half_span, args.wn, args.zeta) if n_per_unit else None
        print('[%d] %s' % (k, report(r, ph)))
        out.append(serialisable(r, ph))
    if len(results) > 1:
        print(summarise(results))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(out, f, indent=2)
    return 0


if __name__ == '__main__':
    sys.exit(main())
