#!/usr/bin/env python3
"""
test_xpc_dref.py — Verify X-Plane native UDP DREF send/read round-trip.

Tests ALL DREFs used by sitl_xplane.py:
  • Attitude    — Euler theta/phi/psi (q[] is computed by X-Plane, not writable)
  • Instruments — h_ind (altitude), vh_ind_fpm (VSI)
                  (indicated_airspeed is read-only; X-Plane derives it from velocity)
  • Control surfaces — joystick yoke ratios (surface def DREFs are computed outputs)
  • Engine      — throttle_ratio actuator
  • Velocity    — local_vx/vy/vz, angular rates Prad/Qrad/Rrad

Usage:
  python3 test_xpc_dref.py
  python3 test_xpc_dref.py --port 49010
  python3 test_xpc_dref.py --pitch 5 --roll 15 --hdg 108
  python3 test_xpc_dref.py --repeat      # animate to confirm HUD responds
"""

import argparse
import math
import socket
import struct
import time

try:
    import xpc as xpclib
    _XPC = True
except ImportError:
    _XPC = False

XP_RECV_PORT = 49010
BIND_PORT    = 49021
XPC_PORT     = 49009

OVERRIDE_DREF = 'sim/operation/override/override_planepath[0]'


# ── DREF groups ───────────────────────────────────────────────────────────────

def attitude_drefs(pitch_deg, roll_deg, hdg_deg):
    # q[] is computed by X-Plane from theta/phi/psi — writes are ignored
    return [
        ('sim/flightmodel/position/theta', 'theta(°)', pitch_deg),
        ('sim/flightmodel/position/phi',   'phi(°)',   roll_deg),
        ('sim/flightmodel/position/psi',   'psi(°)',   hdg_deg),
    ]


def adi_drefs(pitch_deg, roll_deg, hdg_deg):
    """Probe attitude + heading indicator DREFs.
    If writable, these directly set the gauge reading independent of theta/phi/psi."""
    return [
        ('sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot',   'vac-pitch-P',  pitch_deg),
        ('sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot',    'vac-roll-P',   roll_deg),
        ('sim/cockpit2/gauges/indicators/pitch_electric_deg_pilot', 'elec-pitch-P', pitch_deg),
        ('sim/cockpit2/gauges/indicators/roll_electric_deg_pilot',  'elec-roll-P',  roll_deg),
        # heading indicator (DI) candidates — vacuum gyro has its own state
        ('sim/cockpit2/gauges/indicators/heading_vacuum_deg_mag_pilot', 'DI-vac2',  hdg_deg),
        ('sim/cockpit/gyros/psi_vac_ind_deg',                           'DI-psi',   hdg_deg),
        # legacy namespace (XP10 style)
        ('sim/cockpit/gyros/the_vac_ind_deg', 'gyro-pitch', pitch_deg),
        ('sim/cockpit/gyros/phi_vac_ind_deg', 'gyro-roll',  roll_deg),
    ]


def instrument_drefs(alt_ft, vsi_fpm):
    # indicated_airspeed is read-only — X-Plane computes it from velocity vectors
    return [
        ('sim/flightmodel/misc/h_ind',                          'alt(ft)',   alt_ft),
        ('sim/flightmodel/position/vh_ind_fpm',                 'VSI(fpm)', vsi_fpm),
        # read-back only — shows what X-Plane computed from velocity
        ('sim/flightmodel/position/indicated_airspeed',         'IAS(kts)',  None),
    ]


def surface_drefs(ail_r, elev_r, rud_r):
    """Test both control paths:
    - joystick yoke ratios (known writable, go through dead-zone/curves)
    - flightmodel control ratios (more direct, bypasses joystick processing)
    laildef/raildef/hstabdef/ldruddr are computed outputs — NOT writable."""
    return [
        # joystick path (confirmed writable from previous tests)
        ('sim/joystick/yoke_roll_ratio',           'yoke-roll',   ail_r),
        ('sim/joystick/yoke_pitch_ratio',          'yoke-pitch',  -elev_r),
        ('sim/joystick/yoke_heading_ratio',        'yoke-yaw',    rud_r),
        # direct flightmodel control ratios (writability unknown — testing now)
        ('sim/flightmodel/controls/ailn_rat',      'ailn_rat',    ail_r),
        ('sim/flightmodel/controls/elv_rat',       'elv_rat',     -elev_r),
        ('sim/flightmodel/controls/ruddr_rat',     'ruddr_rat',   rud_r),
        # actual surface deflections — expected READ-ONLY (computed outputs)
        ('sim/flightmodel/controls/laildef',       'laildef(°)',  None),
        ('sim/flightmodel/controls/raildef',       'raildef(°)',  None),
        ('sim/flightmodel/controls/hstabdef',      'hstabdef(°)', None),
        ('sim/flightmodel/controls/ldruddr',       'ldruddr(°)',  None),
    ]


def engine_drefs(throttle):
    # ENGN_thro_use is a computed output — use the actuator ratio instead
    return [
        ('sim/cockpit2/engine/actuators/throttle_ratio[0]', 'throttle', throttle),
    ]


def xpc_gps_test(xpc_client, wp_lat, wp_lon, tol=0.0001):
    """Write GPS waypoint DREFs via XPlaneConnect and read them back.
    Tests both direct GPS DREFs and FMS entry DREFs."""
    candidates = [
        ('sim/cockpit/gps/wp_lat',       'gps_wp_lat', wp_lat),
        ('sim/cockpit/gps/wp_lon',       'gps_wp_lon', wp_lon),
        ('sim/cockpit2/fms/entry_lat[0]','fms0_lat',   wp_lat),
        ('sim/cockpit2/fms/entry_lon[0]','fms0_lon',   wp_lon),
    ]
    read_only = [
        ('sim/cockpit/gps/course_degtm',     'DTK(°)'),
        ('sim/cockpit/gps/wp_distance',      'DIS(nm)'),
        ('sim/cockpit/gps/wp_bearing_degtm', 'BRG(°)'),
    ]

    print('\n[GNS430 via XPlaneConnect]')
    print(f'  {"label":<14}  {"sent":>10}  {"recv":>10}  {"ok?":>6}')
    print('  ' + '-' * 48)

    all_ok = True
    for dref, label, sent in candidates:
        xpc_client.sendDREF(dref, sent)
    time.sleep(0.1)

    for dref, label, sent in candidates:
        try:
            recv = xpc_client.getDREF(dref)[0]
        except Exception as e:
            print(f'  {label:<14}  {sent:>10.4f}  {"ERROR":>10}    FAIL  ({e})')
            all_ok = False
            continue
        ok = abs(recv - sent) <= tol
        if not ok:
            all_ok = False
        print(f'  {label:<14}  {sent:>10.4f}  {recv:>10.4f}  {"OK" if ok else "FAIL":>6}')

    for dref, label in read_only:
        try:
            val = xpc_client.getDREF(dref)[0]
            print(f'  {label:<14}  {"(read)":>10}  {val:>10.4f}    INFO')
        except Exception as e:
            print(f'  {label:<14}  {"(read)":>10}  {"ERROR":>10}    INFO  ({e})')

    return all_ok


def gps_drefs(wp_lat, wp_lon):
    """Probe GNS430 waypoint DREFs — test all candidate write paths.
    If writable, X-Plane will compute BRG and DIS from the waypoint position."""
    return [
        # direct GPS wp DREFs (often read-only)
        ('sim/cockpit/gps/wp_lat',               'gps_wp_lat',  wp_lat),
        ('sim/cockpit/gps/wp_lon',               'gps_wp_lon',  wp_lon),
        # FMS entry 0 — active waypoint slot (more likely to be writable)
        ('sim/cockpit2/fms/entry_lat[0]',        'fms0_lat',    wp_lat),
        ('sim/cockpit2/fms/entry_lon[0]',        'fms0_lon',    wp_lon),
        # read-back only — shows what X-Plane computed
        ('sim/cockpit/gps/course_degtm',         'DTK(°)',      None),
        ('sim/cockpit/gps/wp_distance',          'DIS(nm)',     None),
        ('sim/cockpit/gps/wp_bearing_degtm',     'BRG(°)',      None),
    ]


def velocity_drefs(vx, vy, vz, p, q, r):
    return [
        ('sim/flightmodel/position/local_vx', 'vx(m/s)', vx),
        ('sim/flightmodel/position/local_vy', 'vy(m/s)', vy),
        ('sim/flightmodel/position/local_vz', 'vz(m/s)', vz),
        ('sim/flightmodel/position/Prad',     'P(r/s)',  p),
        ('sim/flightmodel/position/Qrad',     'Q(r/s)',  q),
        ('sim/flightmodel/position/Rrad',     'R(r/s)',  r),
    ]


# ── raw UDP helpers ───────────────────────────────────────────────────────────

def send_dref(sock, addr, name: str, value: float):
    name_b = name.encode()
    name_padded = name_b + b'\x00' * (500 - len(name_b))
    sock.sendto(b'DREF\x00' + struct.pack('<f', value) + name_padded, addr)


def subscribe_rref(sock, addr, freq: int, index: int, name: str):
    name_b = name.encode()
    name_padded = name_b + b'\x00' * (400 - len(name_b))
    sock.sendto(b'RREF\x00' + struct.pack('<ii', freq, index) + name_padded, addr)


def unsubscribe_rref(sock, addr, index: int, name: str):
    subscribe_rref(sock, addr, 0, index, name)


def read_rref(sock, index: int, timeout: float = 1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            data, _ = sock.recvfrom(4096)
        except (BlockingIOError, OSError):
            time.sleep(0.01)
            continue
        if len(data) < 5 or data[:4] != b'RREF':
            continue
        offset = 5
        while offset + 8 <= len(data):
            idx, val = struct.unpack_from('<if', data, offset)
            if idx == index:
                return val
            offset += 8
    return None


# ── test logic ────────────────────────────────────────────────────────────────

def test_group(send_sock, recv_sock, xp_addr, title, drefs, tol=0.5):
    """Send each (dref, label, value) then read back and print pass/fail.
    value=None means read-only probe: skip send, just show what X-Plane reports."""
    print(f'\n[{title}]')
    print(f'  {"label":<14}  {"sent":>8}  {"recv":>8}  {"ok?":>6}')
    print(f'  ' + '-' * 46)

    # Send writable entries only
    for dref, _label, value in drefs:
        if value is not None:
            send_dref(send_sock, xp_addr, dref, value)
    time.sleep(0.05)

    # Subscribe all
    for i, (dref, _label, _value) in enumerate(drefs):
        subscribe_rref(recv_sock, xp_addr, 4, i, dref)
    time.sleep(0.05)

    all_ok = True
    for i, (dref, label, sent) in enumerate(drefs):
        received = read_rref(recv_sock, i, timeout=1.0)
        unsubscribe_rref(recv_sock, xp_addr, i, dref)
        if sent is None:
            # read-only probe — just report what X-Plane says
            val_str = f'{received:>8.4f}' if received is not None else '  TIMEOUT'
            print(f'  {label:<14}  {"(read)":>8}  {val_str}  {"INFO":>6}')
        elif received is None:
            print(f'  {label:<14}  {sent:>8.4f}  {"TIMEOUT":>8}  {"FAIL":>6}')
            all_ok = False
        else:
            ok = abs(received - sent) <= tol
            status = 'OK' if ok else 'FAIL'
            if not ok:
                all_ok = False
            print(f'  {label:<14}  {sent:>8.4f}  {received:>8.4f}  {status:>6}')

    return all_ok


def run_all(send_sock, recv_sock, xp_addr, args):
    send_dref(send_sock, xp_addr, OVERRIDE_DREF, 1.0)

    groups = [
        ('Attitude Euler',
         attitude_drefs(args.pitch, args.roll, args.hdg)),
        ('ADI gauge DREFs (probe writability)',
         adi_drefs(args.pitch, args.roll, args.hdg)),
        ('Instruments',
         instrument_drefs(args.alt_ft, args.vsi)),
        ('Control surfaces',
         surface_drefs(args.aileron / 20.0, args.elevator / 20.0, args.rudder / 25.0)),
        ('Engine',
         engine_drefs(args.throttle)),
        ('Velocity + angular rates',
         velocity_drefs(args.vx, args.vy, args.vz, args.p, args.q_rate, args.r)),
        ('GNS430 waypoint (DIS / BRG)',
         gps_drefs(args.wp_lat, args.wp_lon)),
    ]

    results = []
    for title, drefs in groups:
        ok = test_group(send_sock, recv_sock, xp_addr, title, drefs)
        results.append((title, ok))

    print('\n' + '=' * 52)
    for title, ok in results:
        status = 'OK  ' if ok else 'FAIL'
        print(f'  [{status}]  {title}')
    print('=' * 52)
    return all(ok for _, ok in results)


def main():
    ap = argparse.ArgumentParser(description='X-Plane native DREF send/verify (all sitl_xplane DREFs)')
    ap.add_argument('--host',     default='127.0.0.1')
    ap.add_argument('--port',     type=int, default=XP_RECV_PORT)
    ap.add_argument('--bind',     type=int, default=BIND_PORT)
    # attitude
    ap.add_argument('--pitch',    type=float, default=5.0,   help='Pitch deg (nose up)')
    ap.add_argument('--roll',     type=float, default=15.0,  help='Roll deg (right wing down)')
    ap.add_argument('--hdg',      type=float, default=108.0, help='Heading deg true')
    # instruments
    ap.add_argument('--airspeed', type=float, default=100.0, help='IAS knots')
    ap.add_argument('--alt-ft',   type=float, default=2500.0,help='Altitude ft MSL')
    ap.add_argument('--vsi',      type=float, default=0.0,   help='VSI ft/min')
    # surfaces
    ap.add_argument('--aileron',  type=float, default=10.0,  help='Aileron deg')
    ap.add_argument('--elevator', type=float, default=5.0,   help='Elevator deg')
    ap.add_argument('--rudder',   type=float, default=3.0,   help='Rudder deg')
    # engine
    ap.add_argument('--throttle', type=float, default=0.75,  help='Throttle 0-1')
    # GPS waypoint (for GNS430 DIS/BRG test)
    ap.add_argument('--wp-lat',   type=float, default=-6.8899, help='Waypoint latitude deg')
    ap.add_argument('--wp-lon',   type=float, default=107.5734, help='Waypoint longitude deg')
    # XPlaneConnect (optional — for GPS DREFs not writable via native UDP)
    ap.add_argument('--xpc',      action='store_true', help='Also test GPS DREFs via XPlaneConnect plugin')
    ap.add_argument('--xpc-port', type=int, default=XPC_PORT, help=f'XPC plugin port (default: {XPC_PORT})')
    # velocity / rates
    ap.add_argument('--vx',       type=float, default=40.0,  help='local_vx m/s (east)')
    ap.add_argument('--vy',       type=float, default=0.0,   help='local_vy m/s (up)')
    ap.add_argument('--vz',       type=float, default=-50.0, help='local_vz m/s (south)')
    ap.add_argument('--p',        type=float, default=0.0,   help='Roll rate rad/s')
    ap.add_argument('--q-rate',   type=float, default=0.0,   help='Pitch rate rad/s')
    ap.add_argument('--r',        type=float, default=0.0,   help='Yaw rate rad/s')
    # mode
    ap.add_argument('--repeat',   action='store_true',
                    help='Repeat every 1 s and sweep attitude to animate HUD')
    args = ap.parse_args()

    xp_addr  = (args.host, args.port)
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(('', args.bind))
    recv_sock.setblocking(False)

    print(f'X-Plane : {args.host}:{args.port}')
    print(f'RREF    : :{args.bind}')

    # Verify X-Plane is reachable
    print(f'\n[override_planepath] setting …')
    send_dref(send_sock, xp_addr, OVERRIDE_DREF, 1.0)
    subscribe_rref(recv_sock, xp_addr, 4, 99, OVERRIDE_DREF)
    time.sleep(0.1)
    val = read_rref(recv_sock, 99, timeout=2.0)
    unsubscribe_rref(recv_sock, xp_addr, 99, OVERRIDE_DREF)
    if val is None:
        print('  TIMEOUT — is X-Plane running and accepting UDP on this port?')
        return
    print(f'  read back {val:.1f}  {"OK" if val >= 1.0 else "FAIL"}')

    # ── optional XPlaneConnect GPS test ───────────────────────────────────────
    xpc_client = None
    if args.xpc:
        if not _XPC:
            print('[XPC] xpc not installed — run: pip3 install xpc')
        else:
            try:
                xpc_client = xpclib.XPlaneConnect(xpHost=args.host, xpPort=args.xpc_port)
                print(f'[XPC] connected → {args.host}:{args.xpc_port}')
            except Exception as e:
                print(f'[XPC] connect failed: {e}')

    if not args.repeat:
        run_all(send_sock, recv_sock, xp_addr, args)
        if xpc_client:
            xpc_gps_test(xpc_client, args.wp_lat, args.wp_lon)
    else:
        print('\nSweeping attitude every 1 s — Ctrl+C to stop')
        t = 0
        try:
            while True:
                # Animate roll ±30° and pitch ±10° so HUD visibly responds
                args.roll  = 30.0  * math.sin(math.radians(t * 18))
                args.pitch = 10.0  * math.sin(math.radians(t * 9))
                print(f'\n=== sweep t={t}  pitch={args.pitch:.1f}°  roll={args.roll:.1f}°  hdg={args.hdg:.1f}° ===')
                run_all(send_sock, recv_sock, xp_addr, args)
                if xpc_client:
                    xpc_gps_test(xpc_client, args.wp_lat, args.wp_lon)
                t += 1
                time.sleep(1.0)
        except KeyboardInterrupt:
            print('\nDone.')


if __name__ == '__main__':
    main()
