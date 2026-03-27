#!/usr/bin/env python3
"""
test_xpc_send.py — XPC test script: sweep each control surface with fixed
                   values to verify X-Plane responds without MAVLink.

Uses the same DREFs as test_control_plane.py:
  wing1l_ail1def / wing1r_ail1def   aileron  (differential)
  hstab1_elv1def / hstab2_elv1def  elevator
  vstab1_rud1def                    rudder
  ENGN_thro_use[0]                  throttle

Sequence (hold HOLD_S seconds each step):
  neutral → +aileron → -aileron → neutral
  → +elevator → -elevator → neutral
  → +rudder → -rudder → neutral
  → mid-throttle → full-throttle → neutral
  → combined → neutral

Usage:
  python3 test_xpc_send.py
  python3 test_xpc_send.py --host 192.168.1.10 --deg 20 --thr 0.8 --hold 2.0
"""

import argparse
import time

try:
    import xpc
except ImportError:
    print("ERROR: xpc not installed — install XPlaneConnect:")
    print("  pip3 install xplaneconnect   OR   clone github.com/nasa/XPlaneConnect")
    raise SystemExit(1)

HOLD_S   = 1.5   # seconds to hold each step
TEST_DEG = 20.0  # surface deflection in degrees
TEST_THR = 0.75  # throttle test level (0–1)


def set_surfaces(client, ail=0.0, elev=0.0, rud=0.0, thr=0.0):
    """Send all surface + throttle DREFs in one XPC call."""
    drefs = [
        'sim/flightmodel/controls/wing1l_ail1def',
        'sim/flightmodel/controls/wing1r_ail1def',
        'sim/flightmodel/controls/hstab1_elv1def',
        'sim/flightmodel/controls/hstab2_elv1def',
        'sim/flightmodel/controls/vstab1_rud1def',
        'sim/flightmodel/engine/ENGN_thro_use[0]',
    ]
    values = [-ail, ail, elev, elev, rud, thr]
    client.sendDREFs(drefs, values)


def step(client, label, hold=HOLD_S, **kwargs):
    print(f"  {label}")
    set_surfaces(client, **kwargs)
    time.sleep(hold)


def main():
    ap = argparse.ArgumentParser(
        description='XPC fixed-value surface sweep to verify X-Plane DREFs'
    )
    ap.add_argument('--host', default='127.0.0.1',
                    help='X-Plane host (default: 127.0.0.1)')
    ap.add_argument('--port', type=int, default=49009,
                    help='XPC port (default: 49009)')
    ap.add_argument('--deg',  type=float, default=TEST_DEG,
                    help=f'Surface deflection degrees (default: {TEST_DEG})')
    ap.add_argument('--thr',  type=float, default=TEST_THR,
                    help=f'Throttle test level 0-1 (default: {TEST_THR})')
    ap.add_argument('--hold', type=float, default=HOLD_S,
                    help=f'Hold time per step seconds (default: {HOLD_S})')
    args = ap.parse_args()

    hold     = args.hold
    deg      = args.deg
    thr_test = args.thr

    print(f'[XPC] connecting to {args.host}:{args.port}')
    with xpc.XPlaneConnect(xpHost=args.host, xpPort=args.port) as client:

        # enable overrides
        client.sendDREFs(
            ['sim/operation/override/override_control_surfaces',
             'sim/operation/override/override_throttles'],
            [1.0, 1.0],
        )
        print('[XPC] overrides SET (override_control_surfaces + override_throttles)\n')

        print('=== surface sweep ===')

        step(client, 'neutral (all zero)',              hold=hold, ail=0,    elev=0,    rud=0,    thr=0)
        step(client, f'+aileron  ({+deg:.0f} deg)',     hold=hold, ail=+deg, elev=0,    rud=0,    thr=0)
        step(client, f'-aileron  ({-deg:.0f} deg)',     hold=hold, ail=-deg, elev=0,    rud=0,    thr=0)
        step(client, 'neutral',                         hold=hold, ail=0,    elev=0,    rud=0,    thr=0)

        step(client, f'+elevator ({+deg:.0f} deg)',     hold=hold, ail=0,    elev=+deg, rud=0,    thr=0)
        step(client, f'-elevator ({-deg:.0f} deg)',     hold=hold, ail=0,    elev=-deg, rud=0,    thr=0)
        step(client, 'neutral',                         hold=hold, ail=0,    elev=0,    rud=0,    thr=0)

        step(client, f'+rudder   ({+deg:.0f} deg)',     hold=hold, ail=0,    elev=0,    rud=+deg, thr=0)
        step(client, f'-rudder   ({-deg:.0f} deg)',     hold=hold, ail=0,    elev=0,    rud=-deg, thr=0)
        step(client, 'neutral',                         hold=hold, ail=0,    elev=0,    rud=0,    thr=0)

        step(client, f'throttle  ({thr_test:.2f})',     hold=hold, ail=0,    elev=0,    rud=0,    thr=thr_test)
        step(client, 'throttle  (1.0)',                 hold=hold, ail=0,    elev=0,    rud=0,    thr=1.0)
        step(client, 'throttle  (0.0)',                 hold=hold, ail=0,    elev=0,    rud=0,    thr=0)

        step(client,
             f'combined  ail={+deg:.0f} elev={+deg:.0f} rud={+deg:.0f} thr={thr_test:.2f}',
             hold=hold, ail=+deg, elev=+deg, rud=+deg, thr=thr_test)
        step(client, 'neutral (all zero)',              hold=hold, ail=0,    elev=0,    rud=0,    thr=0)

        print('\n[done] all surfaces returned to neutral')


if __name__ == '__main__':
    main()
