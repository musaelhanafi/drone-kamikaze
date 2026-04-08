#!/usr/bin/env python3
"""
test_xpc_posi.py — Quick XPlaneConnect POSI test.

Usage:
  python3 test_xpc_posi.py
  python3 test_xpc_posi.py --host 127.0.0.1 --port 49009
  python3 test_xpc_posi.py --set-lat -6.9003 --set-lon 107.5752 --set-alt 800
"""

import argparse
import time

import xpc


def main():
    ap = argparse.ArgumentParser(description='XPlaneConnect POSI test')
    ap.add_argument('--host',    default='127.0.0.1')
    ap.add_argument('--port',    type=int, default=49000)
    ap.add_argument('--set-lat', type=float, default=None,
                    help='Override latitude to send (degrees)')
    ap.add_argument('--set-lon', type=float, default=None,
                    help='Override longitude to send (degrees)')
    ap.add_argument('--set-alt', type=float, default=None,
                    help='Override altitude to send (m MSL)')
    ap.add_argument('--interval', type=float, default=1.0,
                    help='Monitor interval in seconds (default: 1.0)')
    args = ap.parse_args()

    print(f'Connecting to XPlaneConnect at {args.host}:{args.port} …')
    client = xpc.XPlaneConnect(xpHost=args.host, xpPort=args.port)

    # ── 1. read current POSI ──────────────────────────────────────────────────
    try:
        posi = client.getPOSI()
    except Exception as e:
        print(f'ERROR: getPOSI failed — {e}')
        print('Is the XPlaneConnect plugin installed and X-Plane unpaused?')
        return

    lat, lon, alt, pitch, roll, hdg, gear = posi
    print(f'\nCurrent POSI from X-Plane:')
    print(f'  lat={lat:.6f}  lon={lon:.6f}  alt={alt:.1f} m')
    print(f'  pitch={pitch:+.2f}°  roll={roll:+.2f}°  hdg={hdg:.1f}°  gear={gear}')

    # ── 2. build target POSI (use overrides if given, else current) ───────────
    send_lat = args.set_lat if args.set_lat is not None else lat
    send_lon = args.set_lon if args.set_lon is not None else lon
    send_alt = args.set_alt if args.set_alt is not None else alt
    target   = [send_lat, send_lon, send_alt, pitch, roll, hdg, -1]

    if args.set_lat or args.set_lon or args.set_alt:
        print(f'\nSending custom POSI:')
        print(f'  lat={send_lat:.6f}  lon={send_lon:.6f}  alt={send_alt:.1f} m')

    # ── 3. send + monitor loop ────────────────────────────────────────────────
    print(f'\nMonitoring getPOSI every {args.interval} s — Ctrl+C to stop\n')
    print(f'{"lat":>12}  {"lon":>12}  {"alt(m)":>8}  '
          f'{"pitch°":>8}  {"roll°":>8}  {"hdg°":>7}')
    print('-' * 70)

    try:
        while True:
            client.sendPOSI(target)
            p = client.getPOSI()
            la, lo, al, pi, ro, hd, _ = p
            print(f'{la:12.6f}  {lo:12.6f}  {al:8.1f}  '
                  f'{pi:+8.2f}  {ro:+8.2f}  {hd:7.1f}')
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print('\nDone.')


if __name__ == '__main__':
    main()
