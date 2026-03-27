#!/usr/bin/env python3
"""
sysid_analyze.py — Fit first-order flight dynamics from sysid_logger.py CSV.

For each axis (roll, pitch) and for throttle→airspeed, it:
  1. Finds step inputs in the control surface deflection
  2. Extracts the aircraft rate response from MAVLink data
  3. Fits: rate(t) = gain * input * (1 - exp(-t / tau))
  4. Maps gain + tau to suggested ArduPlane / SITL parameters

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

    Returns (gain, tau_s) or (None, None) if fit fails.
    """
    y0 = response[0]
    y  = [v - y0 for v in response]

    tail = y[int(len(y) * 0.75):]
    y_inf = sum(tail) / len(tail) if tail else None
    if y_inf is None or abs(y_inf) < 1e-6:
        return None, None

    ts, ys = [], []
    for t, yi in zip(times, y):
        ratio = 1.0 - yi / y_inf
        if ratio <= 0.02:
            break
        ts.append(t - t_step)
        ys.append(math.log(ratio))

    if len(ts) < 3:
        return None, None

    num = sum(t * yv for t, yv in zip(ts, ys))
    den = sum(t * t for t in ts)
    if abs(den) < 1e-12:
        return None, None

    slope = num / den
    tau   = -1.0 / slope if slope < 0 else None
    if tau is None or tau <= 0 or tau > 30:
        return None, None

    return y_inf, tau


# ── axis analysis ─────────────────────────────────────────────────────────────

def analyse_axis(data: dict, input_col: str, rate_col: str,
                 axis_name: str, min_step: float = 0.15) -> dict:
    """Analyse one control axis. Returns dict of identified parameters."""
    times  = data['time_s']
    inputs = data[input_col]
    rates  = data.get(rate_col)

    if rates is None:
        return {'axis': axis_name, 'status': f'column {rate_col!r} not found',
                'gain': None, 'tau_s': None}

    dt = times[1] - times[0] if len(times) > 1 else 0.02

    steps = find_steps(inputs, times, min_step=min_step, dt=dt)
    if not steps:
        return {'axis': axis_name, 'status': 'no steps found',
                'gain': None, 'tau_s': None}

    gains, taus = [], []

    for (i0, i1, delta) in steps:
        stop  = min(i1, i0 + int(5.0 / dt))
        t_win = times[i0:stop]
        r_win = rates[i0:stop]
        t_step = times[i0]

        g, tau = fit_first_order(t_win, r_win, t_step)
        if g is not None:
            gains.append(g / delta)
            taus.append(tau)

    def _avg(lst): return sum(lst) / len(lst) if lst else None

    return {
        'axis':    axis_name,
        'status':  f'{len(steps)} step(s) found, {len(gains)} fitted',
        'n_steps': len(steps),
        'gain':    _avg(gains),
        'tau_s':   _avg(taus),
    }


def analyse_throttle_airspeed(data: dict) -> dict:
    """Simple steady-state gain: Δairspeed_ms / Δthrottle."""
    thr   = data['thr']
    ias   = data['airspeed_ms']
    times = data['time_s']
    dt    = times[1] - times[0] if len(times) > 1 else 0.02

    steps = find_steps(thr, times, min_step=0.1, min_hold=3.0, dt=dt)
    gains = []
    for (i0, i1, delta) in steps:
        mid  = (i0 + i1) // 2
        ias0 = sum(ias[i0 - 10:i0]) / 10 if i0 >= 10 else ias[i0]
        ias1 = sum(ias[mid:i1]) / max(1, i1 - mid)
        gains.append((ias1 - ias0) / delta)

    avg_gain = sum(gains) / len(gains) if gains else None
    return {
        'axis':   'throttle→airspeed',
        'status': f'{len(steps)} step(s)',
        'gain':   avg_gain,    # m/s per unit throttle
        'tau_s':  None,
    }


# ── parameter mapping ─────────────────────────────────────────────────────────

def suggest_params(results: list, cruise_ias_ms: float = 30.0) -> dict:
    """Map identified dynamics to ArduPlane parameters."""
    params = {}
    for r in results:
        axis = r['axis']
        g    = r.get('gain')
        tau  = r.get('tau_s')

        if axis == 'roll' and g and tau:
            params['RLL_RATE_P']    = round(1.0 / max(abs(g) * tau, 0.1), 3)
            params['RLL_RATE_D']    = round(tau / 4.0, 3)
            params['RLL_RATE_I']    = round(params['RLL_RATE_P'] * 0.1, 4)
            params['RLL_2SRV_RMAX'] = round(abs(g), 1)

        elif axis == 'pitch' and g and tau:
            params['PTCH_RATE_P']       = round(1.0 / max(abs(g) * tau, 0.1), 3)
            params['PTCH_RATE_D']       = round(tau / 4.0, 3)
            params['PTCH_RATE_I']       = round(params['PTCH_RATE_P'] * 0.1, 4)
            rmax = round(min(abs(g), 100.0), 1)   # clamped to param range 0-100 deg/s
            params['PTCH_2SRV_RMAX_UP'] = rmax
            params['PTCH_2SRV_RMAX_DN'] = rmax

        elif axis == 'throttle→airspeed' and g:
            params['TECS_SPDWEIGHT'] = round(min(2.0, cruise_ias_ms / max(abs(g), 1.0)), 2)

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
        g   = r.get('gain')
        tau = r.get('tau_s')
        if g is not None:
            unit = 'm/s/unit' if 'airspeed' in r['axis'] else '°/s/unit'
            print(f'    gain={_fmt(g)} {unit}   tau={_fmt(tau)} s')

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

def plot_comparison(data: dict):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('[PLOT] matplotlib not installed — pip3 install matplotlib')
        return

    axes_map = [
        ('ail_r',  'p_dps',      'Roll (CH1 elevon → P rate)'),
        ('elev_r', 'q_dps',      'Pitch (CH2 elevon → Q rate)'),
        ('thr',    'airspeed_ms', 'Throttle → airspeed (m/s)'),
    ]

    fig, axs = plt.subplots(len(axes_map), 1, figsize=(12, 3 * len(axes_map)),
                            sharex=True)
    t = data['time_s']

    for ax, (inp_col, resp_col, title) in zip(axs, axes_map):
        ax2 = ax.twinx()
        ax.plot(t, data[inp_col], color='gray', alpha=0.5, label='input')
        ax.set_ylabel('input', color='gray')
        if resp_col in data:
            ax2.plot(t, data[resp_col], color='royalblue', label=resp_col)
        ax2.set_ylabel(resp_col)
        ax.set_title(title)
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)

    axs[-1].set_xlabel('time (s)')
    fig.suptitle('Flight Dynamics — MAVLink data', fontsize=13)
    plt.tight_layout()
    plt.show()


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='Analyse sysid_logger.py CSV and suggest ArduPlane parameters'
    )
    ap.add_argument('csv', help='CSV file from sysid_logger.py')
    ap.add_argument('--min-step',   type=float, default=0.15,
                    help='Minimum control step size to analyse (default: 0.15)')
    ap.add_argument('--cruise-ias', type=float, default=30.0,
                    help='Cruise airspeed m/s for TECS mapping (default: 30)')
    ap.add_argument('--plot', action='store_true',
                    help='Show time-series plot (requires matplotlib)')
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

    results = [
        analyse_axis(data, 'ail_r',  'p_dps', 'roll',  min_step=args.min_step),
        analyse_axis(data, 'elev_r', 'q_dps', 'pitch', min_step=args.min_step),
        analyse_throttle_airspeed(data),
    ]

    params = suggest_params(results, cruise_ias_ms=args.cruise_ias)
    print_report(results, params)

    if args.plot:
        plot_comparison(data)


if __name__ == '__main__':
    main()
