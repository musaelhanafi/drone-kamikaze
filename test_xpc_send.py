#!/usr/bin/env python3
"""
test_xpc_send.py — Verify every DREF used by sitl_xpc_xplane.py via XPlaneConnect.

Tests all write paths in sitl_xpc_xplane.py:
  • sendPOSI   — lat/lon/alt/pitch/roll/hdg
  • Velocity   — local_vx/vy/vz, angular rates Prad/Qrad/Rrad
  • Controls   — joystick yoke ratios, throttle actuator
  • Instruments — h_ind (altitude), vh_ind_fpm (VSI), IAS (read-only probe)
  • Gyro gauges — vacuum ADI pitch/roll
  • GPS         — wp_lat/wp_lon → GNS430 DIS/BRG (the key XPC advantage)

Usage:
  python3 test_xpc_send.py
  python3 test_xpc_send.py --port 49009
  python3 test_xpc_send.py --pitch 5 --roll 15 --hdg 108
  python3 test_xpc_send.py --repeat      # animate to confirm instruments respond
"""

import argparse
import math
import time

import xpc

R_EARTH = 6_371_000.0


# ── group helpers ─────────────────────────────────────────────────────────────

def posi_group(client, lat, lon, alt_m, pitch_deg, roll_deg, hdg_deg, tol=0.0001):
    """Test sendPOSI round-trip."""
    print('\n[POSI — position + attitude]')
    print(f'  {"field":<12}  {"sent":>12}  {"recv":>12}  {"ok?":>6}')
    print('  ' + '-' * 50)

    client.sendPOSI([lat, lon, alt_m, pitch_deg, roll_deg, hdg_deg, -1])
    time.sleep(0.05)
    p = client.getPOSI()
    recv_lat, recv_lon, recv_alt, recv_pitch, recv_roll, recv_hdg = p[:6]

    fields = [
        ('lat(°)',    lat,       recv_lat,   1e-4),
        ('lon(°)',    lon,       recv_lon,   1e-4),
        ('alt(m)',    alt_m,     recv_alt,   1.0),
        ('pitch(°)',  pitch_deg, recv_pitch, 0.5),
        ('roll(°)',   roll_deg,  recv_roll,  0.5),
        ('hdg(°)',    hdg_deg,   recv_hdg,   0.5),
    ]

    all_ok = True
    for label, sent, recv, field_tol in fields:
        ok = abs(recv - sent) <= field_tol
        if not ok:
            all_ok = False
        print(f'  {label:<12}  {sent:>12.4f}  {recv:>12.4f}  {"OK" if ok else "FAIL":>6}')
    return all_ok


def dref_group(client, title, drefs, tol=0.01):
    """Send a batch of (dref, label, value) via sendDREFs then read back with getDREF.
    value=None means read-only probe: skip send, just report what X-Plane returns."""
    print(f'\n[{title}]')
    print(f'  {"label":<20}  {"sent":>10}  {"recv":>10}  {"ok?":>6}')
    print('  ' + '-' * 54)

    writable = [(d, l, v) for d, l, v in drefs if v is not None]
    if writable:
        client.sendDREFs([d for d, _, _ in writable],
                         [v for _, _, v in writable])
    time.sleep(0.05)

    all_ok = True
    for dref, label, sent in drefs:
        try:
            result = client.getDREF(dref)
            recv = result[0] if result else None
        except Exception:
            recv = None

        if recv is None:
            # XPC can't read back array DREFs or some cockpit DREFs — treat as send-only
            if sent is None:
                print(f'  {label:<20}  {"(read)":>10}  {"NOREAD":>10}    INFO')
            else:
                print(f'  {label:<20}  {sent:>10.4f}  {"NOREAD":>10}    SENT')
            continue

        if sent is None:
            print(f'  {label:<20}  {"(read)":>10}  {recv:>10.4f}    INFO')
        else:
            ok = abs(recv - sent) <= tol
            if not ok:
                all_ok = False
            print(f'  {label:<20}  {sent:>10.4f}  {recv:>10.4f}  {"OK" if ok else "FAIL":>6}')
    return all_ok


# ── DREF group definitions ────────────────────────────────────────────────────

def velocity_drefs(vx, vy, vz, p, q, r):
    # NED → X-Plane local (+x east, +y up, +z south)
    return [
        ('sim/flightmodel/position/local_vx', 'vx(m/s)',  vx),
        ('sim/flightmodel/position/local_vy', 'vy(m/s)',  vy),
        ('sim/flightmodel/position/local_vz', 'vz(m/s)',  vz),
        ('sim/flightmodel/position/Prad',     'P(rad/s)', p),
        ('sim/flightmodel/position/Qrad',     'Q(rad/s)', q),
        ('sim/flightmodel/position/Rrad',     'R(rad/s)', r),
    ]


def control_drefs(ail_r, elev_r, rud_r, throttle):
    return [
        ('sim/joystick/yoke_roll_ratio',                        'aileron[-1,1]',  ail_r),
        ('sim/joystick/yoke_pitch_ratio',                       'elevator[-1,1]', -elev_r),
        ('sim/joystick/yoke_heading_ratio',                     'rudder[-1,1]',   rud_r),
        ('sim/cockpit2/engine/actuators/throttle_ratio[0]',     'throttle[0,1]',  throttle),
    ]


def instrument_drefs(alt_ft, vsi_fpm):
    return [
        ('sim/flightmodel/misc/h_ind',                      'alt(ft)',   alt_ft),
        ('sim/flightmodel/position/vh_ind_fpm',             'VSI(fpm)', vsi_fpm),
        ('sim/flightmodel/position/indicated_airspeed',     'IAS(kts)',  None),   # read-only
    ]


def gyro_drefs(pitch_deg, roll_deg):
    return [
        ('sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot', 'vac-pitch',  pitch_deg),
        ('sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot',  'vac-roll',   roll_deg),
        ('sim/cockpit/gyros/the_vac_ind_deg',                     'gyro-pitch', pitch_deg),
        ('sim/cockpit/gyros/phi_vac_ind_deg',                     'gyro-roll',  roll_deg),
    ]


def gps_drefs(wp_lat, wp_lon):
    return [
        ('sim/cockpit/gps/wp_lat',           'wp_lat(°)',  wp_lat),
        ('sim/cockpit/gps/wp_lon',           'wp_lon(°)',  wp_lon),
        # read-back — X-Plane computes these from wp_lat/wp_lon vs aircraft position
        ('sim/cockpit/gps/course_degtm',     'DTK(°)',     None),
        ('sim/cockpit/gps/wp_distance',      'DIS(nm)',    None),
        ('sim/cockpit/gps/wp_bearing_degtm', 'BRG(°)',     None),
    ]


# ── run ───────────────────────────────────────────────────────────────────────

def run_all(client, args):
    # velocity: NED → XP local frame
    vx_xp =  args.vy_ned   # east
    vy_xp = -args.vz_ned   # up
    vz_xp = -args.vx_ned   # south

    groups = []

    groups.append(('POSI', lambda: posi_group(
        client,
        args.lat, args.lon, args.alt_m,
        args.pitch, args.roll, args.hdg,
    )))
    groups.append(('Velocity + angular rates', lambda: dref_group(
        client, 'Velocity + angular rates',
        velocity_drefs(vx_xp, vy_xp, vz_xp, args.p, args.q_rate, args.r),
    )))
    groups.append(('Controls + throttle', lambda: dref_group(
        client, 'Controls + throttle',
        control_drefs(
            args.aileron / 20.0,
            args.elevator / 20.0,
            args.rudder / 25.0,
            args.throttle,
        ),
    )))
    groups.append(('Instruments', lambda: dref_group(
        client, 'Instruments',
        instrument_drefs(args.alt_m * 3.28084, args.vsi_fpm),
        tol=1.0,
    )))
    groups.append(('Gyro gauges (vacuum ADI)', lambda: dref_group(
        client, 'Gyro gauges (vacuum ADI)',
        gyro_drefs(args.pitch, args.roll),
        tol=0.5,
    )))
    groups.append(('GNS430 GPS waypoint', lambda: dref_group(
        client, 'GNS430 GPS waypoint',
        gps_drefs(args.wp_lat, args.wp_lon),
        tol=0.0001,
    )))

    results = []
    for title, fn in groups:
        ok = fn()
        results.append((title, ok))

    print('\n' + '=' * 54)
    for title, ok in results:
        print(f'  [{"OK  " if ok else "FAIL"}]  {title}')
    print('=' * 54)
    return all(ok for _, ok in results)


def main():
    ap = argparse.ArgumentParser(
        description='XPlaneConnect DREF send/verify — mirrors sitl_xpc_xplane.py write paths'
    )
    ap.add_argument('--host',     default='127.0.0.1')
    ap.add_argument('--port',     type=int, default=49009, help='XPC plugin port (default: 49009)')
    # attitude / position
    ap.add_argument('--lat',      type=float, default=-6.9003,  help='Latitude deg')
    ap.add_argument('--lon',      type=float, default=107.5752, help='Longitude deg')
    ap.add_argument('--alt-m',    type=float, default=800.0,    help='Altitude m MSL')
    ap.add_argument('--pitch',    type=float, default=5.0,      help='Pitch deg (nose up)')
    ap.add_argument('--roll',     type=float, default=15.0,     help='Roll deg (right wing down)')
    ap.add_argument('--hdg',      type=float, default=108.0,    help='Heading deg true')
    # velocity (NED, m/s)
    ap.add_argument('--vx-ned',   type=float, default=40.0,     help='NED north m/s')
    ap.add_argument('--vy-ned',   type=float, default=0.0,      help='NED east m/s')
    ap.add_argument('--vz-ned',   type=float, default=-5.0,     help='NED down m/s')
    # angular rates
    ap.add_argument('--p',        type=float, default=0.0,      help='Roll rate rad/s')
    ap.add_argument('--q-rate',   type=float, default=0.0,      help='Pitch rate rad/s')
    ap.add_argument('--r',        type=float, default=0.0,      help='Yaw rate rad/s')
    # controls
    ap.add_argument('--aileron',  type=float, default=10.0,     help='Aileron deg')
    ap.add_argument('--elevator', type=float, default=5.0,      help='Elevator deg')
    ap.add_argument('--rudder',   type=float, default=3.0,      help='Rudder deg')
    ap.add_argument('--throttle', type=float, default=0.75,     help='Throttle 0-1')
    # instruments
    ap.add_argument('--vsi-fpm',  type=float, default=0.0,      help='VSI ft/min')
    # GPS waypoint
    ap.add_argument('--wp-lat',   type=float, default=-6.8899,  help='Waypoint latitude deg')
    ap.add_argument('--wp-lon',   type=float, default=107.5734, help='Waypoint longitude deg')
    # mode
    ap.add_argument('--repeat',   action='store_true',
                    help='Repeat every 1 s sweeping attitude to animate instruments')
    args = ap.parse_args()

    print(f'XPlaneConnect : {args.host}:{args.port}')
    client = xpc.XPlaneConnect(xpHost=args.host, xpPort=args.port)

    # Verify connection via getPOSI (simplest round-trip check)
    print('\n[connection check] getPOSI …')
    try:
        posi = client.getPOSI()
        print(f'  lat={posi[0]:.4f}  lon={posi[1]:.4f}  alt={posi[2]:.1f} m  OK')
    except Exception as e:
        print(f'  ERROR — {e}')
        print('  Is XPlaneConnect plugin installed and X-Plane unpaused?')
        return

    # Enable physics override (send-only — array DREFs are not readable via XPC getDREF)
    print('[override_planepath] setting …')
    client.sendDREF('sim/operation/override/override_planepath[0]', 1.0)
    print('  sent')

    # Zero winds so IAS is computed from velocity vectors alone
    for layer in range(3):
        client.sendDREF(f'sim/weather/wind_speed_kt[{layer}]', 0.0)

    if not args.repeat:
        run_all(client, args)
    else:
        print('\nSweeping attitude every 1 s — Ctrl+C to stop')
        t = 0
        try:
            while True:
                args.roll  = 30.0 * math.sin(math.radians(t * 18))
                args.pitch = 10.0 * math.sin(math.radians(t * 9))
                print(f'\n=== sweep t={t}  pitch={args.pitch:.1f}°  '
                      f'roll={args.roll:.1f}°  hdg={args.hdg:.1f}° ===')
                run_all(client, args)
                t += 1
                time.sleep(1.0)
        except KeyboardInterrupt:
            print('\nDone.')


if __name__ == '__main__':
    main()
