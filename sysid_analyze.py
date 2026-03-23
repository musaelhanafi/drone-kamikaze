#!/usr/bin/env python3
"""
sysid_analyze.py — Fit first-order flight dynamics from sysid_logger.py CSV.

For each axis (roll, pitch, yaw) and for throttle→airspeed, it:
  1. Finds step inputs in the control surface deflection
  2. Extracts the aircraft's rate response from X-Plane data
  3. Fits: rate(t) = gain * input * (1 - exp(-t / tau))
  4. Maps gain + tau to suggested ArduPlane / SITL parameters

Then compares X-Plane response vs MAVLink (SITL) response to show the gap
the operator needs to close by tuning SITL parameters.

Usage
─────
  python3 sysid_analyze.py sysid_log.csv
  python3 sysid_analyze.py sysid_log.csv --plot          # requires matplotlib
  python3 sysid_analyze.py sysid_log.csv --min-step 0.2  # ignore small inputs
"""

import argparse
import csv
import math
import sys
from pathlib import Path

# ── CSV loading ───────────────────────────────────────────────────────────────

def load_csv(path: Path) -> dict:
    """Return dict of column_name → list[float]."""
    data = {}
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k, v in row.items():
                data.setdefault(k, []).append(float(v))
    return data


# ── step detection ────────────────────────────────────────────────────────────

def find_steps(signal: list, times: list, min_step: float = 0.15,
               min_hold: float = 0.5, dt: float = 0.02):
    """
    Find indices where the signal makes a sustained step of >= min_step.
    Returns list of (start_idx, end_idx, delta) tuples.
    """
    steps = []
    n = len(signal)
    i = 0
    while i < n - 1:
        delta = signal[i + 1] - signal[i]
        if abs(delta) >= min_step:
            # Check it holds for min_hold seconds
            hold_samples = int(min_hold / dt)
            end = min(i + 1 + hold_samples, n - 1)
            if all(abs(signal[j] - signal[i + 1]) < min_step * 0.5
                   for j in range(i + 1, end)):
                steps.append((i, end, delta))
                i = end
                continue
        i += 1
    return steps


# ── first-order fit ───────────────────────────────────────────────────────────

def fit_first_order(times: list, response: list, t_step: float):
    """
    Fit y(t) = y0 + gain * (1 - exp(-(t - t_step) / tau)) to the response
    after a step at t_step.
    Uses least-squares on the linearised form: log(1 - (y - y0) / y_inf) = -t/tau.

    Returns (gain, tau_s) or (None, None) if fit fails.
    """
    # Offset-correct to the value just before step
    y0 = response[0]
    y  = [v - y0 for v in response]

    # Estimate steady-state as mean of last 25%
    tail = y[int(len(y) * 0.75):]
    y_inf = sum(tail) / len(tail) if tail else None
    if y_inf is None or abs(y_inf) < 1e-6:
        return None, None

    # Linearise: log(1 - y/y_inf) = -t/tau  →  linear regression
    ts, ys = [], []
    for i, (t, yi) in enumerate(zip(times, y)):
        ratio = 1.0 - yi / y_inf
        if ratio <= 0.02:   # saturated — skip
            break
        ts.append(t - t_step)
        ys.append(math.log(ratio))

    if len(ts) < 3:
        return None, None

    # Least-squares: ys = -1/tau * ts
    num = sum(t * yv for t, yv in zip(ts, ys))
    den = sum(t * t for t in ts)
    if abs(den) < 1e-12:
        return None, None

    slope = num / den         # = -1 / tau
    tau   = -1.0 / slope if slope < 0 else None
    if tau is None or tau <= 0 or tau > 30:
        return None, None

    return y_inf, tau


# ── axis analysis ─────────────────────────────────────────────────────────────

def analyse_axis(data: dict, input_col: str, xp_rate_col: str, mav_rate_col: str,
                 axis_name: str, min_step: float = 0.15) -> dict:
    """Analyse one control axis. Returns dict of identified parameters."""
    times  = data['time_s']
    inputs = data[input_col]
    xp_r   = data[xp_rate_col]
    mav_r  = data.get(mav_rate_col, [0.0] * len(times))

    dt = times[1] - times[0] if len(times) > 1 else 0.02

    steps = find_steps(inputs, times, min_step=min_step, dt=dt)
    if not steps:
        return {'axis': axis_name, 'status': 'no steps found',
                'gain': None, 'tau_s': None}

    gains_xp, taus_xp = [], []
    gains_mav, taus_mav = [], []

    for (i0, i1, delta) in steps:
        window = slice(i0, min(i1, i0 + int(5.0 / dt)))   # max 5 s window
        t_win  = times[i0:window.stop]
        xp_win = xp_r[i0:window.stop]
        mav_win = mav_r[i0:window.stop]

        t_step = times[i0]

        g, tau = fit_first_order(t_win, xp_win, t_step)
        if g is not None:
            gains_xp.append(g / delta)   # gain per unit input
            taus_xp.append(tau)

        g, tau = fit_first_order(t_win, mav_win, t_step)
        if g is not None:
            gains_mav.append(g / delta)
            taus_mav.append(tau)

    def _avg(lst): return sum(lst) / len(lst) if lst else None

    return {
        'axis':      axis_name,
        'status':    f'{len(steps)} step(s) found',
        'n_steps':   len(steps),
        'gain_xp':   _avg(gains_xp),    # deg/s per unit deflection (X-Plane)
        'tau_xp':    _avg(taus_xp),     # time constant s (X-Plane)
        'gain_mav':  _avg(gains_mav),   # same from SITL/MAVLink
        'tau_mav':   _avg(taus_mav),
    }


def analyse_throttle_airspeed(data: dict) -> dict:
    """Simple gain: Δairspeed_kts / Δthrottle in steady state."""
    thr = data['thr']
    ias = data['xp_ias_kts']
    times = data['time_s']
    dt = times[1] - times[0] if len(times) > 1 else 0.02

    steps = find_steps(thr, times, min_step=0.1, min_hold=3.0, dt=dt)
    gains = []
    for (i0, i1, delta) in steps:
        # Use last quarter of hold period as steady state
        mid  = (i0 + i1) // 2
        ias0 = sum(ias[i0 - 10:i0]) / 10 if i0 >= 10 else ias[i0]
        ias1 = sum(ias[mid:i1]) / max(1, i1 - mid)
        gains.append((ias1 - ias0) / delta)

    avg_gain = sum(gains) / len(gains) if gains else None
    return {
        'axis':    'throttle→airspeed',
        'status':  f'{len(steps)} step(s)',
        'gain_xp': avg_gain,    # kts per unit throttle
        'tau_xp':  None,
    }


# ── parameter mapping ─────────────────────────────────────────────────────────

def suggest_params(results: list, cruise_ias_kts: float = 60.0) -> dict:
    """
    Map identified dynamics to ArduPlane parameters.

    Roll axis:
      RLL_RATE_P   ∝  1 / (gain_xp * tau_xp)  — proportional gain
      RLL_RATE_D   ∝  tau_xp / 4               — derivative (damps oscillation)
      RLL_2SRV_RMAX ≈  gain_xp * 1.0           — max roll rate deg/s at full deflection

    Pitch axis:
      PTCH_RATE_P, PTCH_RATE_D, PTCH_2SRV_RMAX_UP/_DN  — same logic as roll

    Gap ratio:
      gain_mav / gain_xp  — how far SITL is from X-Plane truth per axis
    """
    params = {}
    for r in results:
        axis = r['axis']
        g    = r.get('gain_xp')
        tau  = r.get('tau_xp')

        if axis == 'roll' and g and tau:
            params['RLL_RATE_P']    = round(1.0 / max(abs(g) * tau, 0.1), 3)
            params['RLL_RATE_D']    = round(tau / 4.0, 3)
            params['RLL_RATE_I']    = round(params['RLL_RATE_P'] * 0.1, 4)
            params['RLL_2SRV_RMAX'] = round(abs(g), 1)

        elif axis == 'pitch' and g and tau:
            params['PTCH_RATE_P']       = round(1.0 / max(abs(g) * tau, 0.1), 3)
            params['PTCH_RATE_D']       = round(tau / 4.0, 3)
            params['PTCH_RATE_I']       = round(params['PTCH_RATE_P'] * 0.1, 4)
            params['PTCH_2SRV_RMAX_UP'] = round(abs(g), 1)
            params['PTCH_2SRV_RMAX_DN'] = round(abs(g), 1)

        elif axis == 'yaw' and g and tau:
            params['YAW_2SRV_RLL'] = round(1.0 / max(abs(g) * tau, 0.5), 3)

        elif axis == 'throttle→airspeed' and g:
            # TECS: speed → throttle response; map via cruise IAS
            params['TECS_SPDWEIGHT'] = round(min(2.0, 60.0 / max(abs(g), 1.0)), 2)

    return params


# ── reporting ─────────────────────────────────────────────────────────────────

def _fmt(v): return f'{v:.3f}' if v is not None else 'n/a'


def print_report(results: list, params: dict):
    W = 72
    print('\n' + '=' * W)
    print(' SYSTEM IDENTIFICATION RESULTS')
    print('=' * W)

    for r in results:
        axis = r['axis'].upper()
        print(f'\n  [{axis}]  {r["status"]}')
        g_xp  = r.get('gain_xp')
        t_xp  = r.get('tau_xp')
        g_mav = r.get('gain_mav')
        t_mav = r.get('tau_mav')

        if g_xp is not None:
            print(f'    X-Plane  gain={_fmt(g_xp)} °/s/unit   tau={_fmt(t_xp)} s')
        if g_mav is not None:
            print(f'    SITL/MAV gain={_fmt(g_mav)} °/s/unit   tau={_fmt(t_mav)} s')
            if g_xp and abs(g_xp) > 1e-6:
                ratio = g_mav / g_xp
                print(f'    Gap ratio (SITL/XP): {ratio:.2f}  '
                      f'{"✓ matched" if 0.8 < ratio < 1.2 else "⚠ needs tuning"}')
        else:
            print(f'    (SITL data not available for this axis)')

    print('\n' + '-' * W)
    print('  SUGGESTED ARDUPLANE PARAMETERS')
    print('-' * W)
    if params:
        for k, v in params.items():
            print(f'    {k:<20} = {v}')
    else:
        print('    (insufficient data to suggest parameters)')

    print('\n  NOTE: These are starting points from first-order fits.')
    print('  Validate with AutoTune or in-flight testing before operations.')
    print('=' * W + '\n')


# ── optional plot ─────────────────────────────────────────────────────────────

def plot_comparison(data: dict, results: list):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('[PLOT] matplotlib not installed — pip3 install matplotlib')
        return

    axes_map = [
        ('ail_r',  'xp_p_dps',  'mav_p_dps',  'Roll (left elevon → P rate)'),
        ('elev_r', 'xp_q_dps',  'mav_q_dps',  'Pitch (right elevon → Q rate)'),
        ('thr',    'xp_ias_kts', None,         'Throttle → IAS'),
    ]

    fig, axs = plt.subplots(len(axes_map), 1, figsize=(12, 3 * len(axes_map)),
                            sharex=True)
    t = data['time_s']

    for ax, (inp_col, xp_col, mav_col, title) in zip(axs, axes_map):
        inp = data[inp_col]
        ax2 = ax.twinx()
        ax.plot(t, inp, color='gray', alpha=0.5, label='input')
        ax.set_ylabel('input [-1,1]', color='gray')
        ax2.plot(t, data[xp_col], color='royalblue', label='X-Plane')
        if mav_col and mav_col in data:
            ax2.plot(t, data[mav_col], color='tomato', linestyle='--', label='SITL')
        ax2.set_ylabel(xp_col)
        ax.set_title(title)
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)

    axs[-1].set_xlabel('time (s)')
    fig.suptitle('X-Plane vs SITL Flight Dynamics', fontsize=13)
    plt.tight_layout()
    plt.show()


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='Analyse sysid_logger.py CSV and suggest ArduPlane parameters'
    )
    ap.add_argument('csv', help='CSV file from sysid_logger.py')
    ap.add_argument('--min-step',  type=float, default=0.15,
                    help='Minimum control step size to analyse (default: 0.15)')
    ap.add_argument('--cruise-ias', type=float, default=60.0,
                    help='Cruise IAS kts for TECS mapping (default: 60)')
    ap.add_argument('--plot', action='store_true',
                    help='Show time-series comparison plot (requires matplotlib)')
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print(f'ERROR: {path} not found')
        sys.exit(1)

    print(f'[SID] loading {path} …')
    data = load_csv(path)
    n = len(data.get('time_s', []))
    duration = data['time_s'][-1] - data['time_s'][0] if n > 1 else 0
    print(f'[SID] {n} rows  duration={duration:.1f} s')

    if n < 50:
        print('ERROR: too few rows — need at least 1 s of flight data')
        sys.exit(1)

    # Analyse each axis
    results = [
        analyse_axis(data, 'ail_r',  'xp_p_dps', 'mav_p_dps',
                     'roll',  min_step=args.min_step),
        analyse_axis(data, 'elev_r', 'xp_q_dps', 'mav_q_dps',
                     'pitch', min_step=args.min_step),
        analyse_throttle_airspeed(data),
    ]

    params = suggest_params(results, cruise_ias_kts=args.cruise_ias)
    print_report(results, params)

    if args.plot:
        plot_comparison(data, results)


if __name__ == '__main__':
    main()
