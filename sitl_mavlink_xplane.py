#!/usr/bin/env python3
"""
sitl_mavlink_xplane.py — X-Plane ↔ ArduPilot SITL bridge (--model JSON).

Architecture
────────────
  X-Plane sends DATA@ rows  ──UDP:49005──▶  this script
  This script sends JSON state ──UDP:9003──▶  SITL (--model JSON)
  SITL sends binary servo PWM  ──UDP:9002──▶  this script
  This script sends DREFs  ──UDP:49000──▶  X-Plane
  This script sends RC override ──MAVLink──▶  SITL (joystick)

SITL startup
────────────
  python3 Tools/autotest/sim_vehicle.py \\
    --vehicle Plane --model JSON --location WICC \\
    --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14560

Usage
─────
  python3 sitl_mavlink_xplane.py
  python3 sitl_mavlink_xplane.py --joystick --joy-index 0 --debug
"""

import argparse
import math
import select
import socket
import struct
import time

from pymavlink import mavutil

try:
    import pygame
    _PYGAME = True
except ImportError:
    _PYGAME = False

# ── constants ─────────────────────────────────────────────────────────────────
KNOTS_TO_MS = 0.514444
FEET_TO_M   = 0.3048
GRAVITY_MSS = 9.80665
DEG_TO_RAD  = math.pi / 180.0
UINT16_MAX  = 65535

# ── X-Plane DATA@ row codes ───────────────────────────────────────────────────
ROW_TIMES          = 1
ROW_SPEED          = 3
ROW_GLOAD          = 4
ROW_ANG_VEL        = 16
ROW_PITCH_ROLL_HDG = 17
ROW_LAT_LON_ALT    = 20
ROW_LOC_VEL_DIST   = 21
REQUIRED_ROWS = [
    ROW_TIMES, ROW_SPEED, ROW_GLOAD,
    ROW_ANG_VEL, ROW_PITCH_ROLL_HDG,
    ROW_LAT_LON_ALT, ROW_LOC_VEL_DIST,
]
RREF_VERSION = 1

# ── X-Plane DREF servo mapping ────────────────────────────────────────────────
def _angle(pwm, r=1.0): return r * (pwm - 1500) / 500.0
def _range(pwm, r=1.0): return r * (pwm - 1000) / 1000.0

SERVO_DREFS = [
    ('sim/joystick/yoke_roll_ratio',             0, _angle),
    ('sim/joystick/yoke_pitch_ratio',            1, _angle),
    ('sim/flightmodel/engine/ENGN_thro_use[0]',  2, _range),
    ('sim/flightmodel/engine/ENGN_thro_use[1]',  2, _range),
    ('sim/flightmodel/engine/ENGN_thro_use[2]',  2, _range),
    ('sim/flightmodel/engine/ENGN_thro_use[3]',  2, _range),
    ('sim/joystick/yoke_heading_ratio',           3, _angle),
    ('sim/cockpit2/controls/flap_ratio',          4, _range),
]


# ── X-Plane UDP helpers ───────────────────────────────────────────────────────

def xp_dsel(sock, addr, rows):
    padded = (list(rows) + [0] * 8)[:8]
    sock.sendto(b'DSEL\x00' + struct.pack('<8I', *padded), addr)


def xp_dref(sock, addr, name: str, value: float):
    name_b = name.encode() + b'\x00' * (500 - len(name))
    sock.sendto(b'DREF\x00' + struct.pack('<f', value) + name_b, addr)


def xp_rref(sock, addr, name: str, code: int, rate_hz: int):
    name_b = name.encode() + b'\x00' * (400 - len(name))
    sock.sendto(b'RREF\x00' + struct.pack('<II', rate_hz, code) + name_b, addr)


def xp_parse(pkt: bytes) -> dict:
    """Parse DATA@ packet → {row_code: (v0…v7)}."""
    if len(pkt) < 5 or pkt[:4] != b'DATA':
        return {}
    rows = {}
    off = 5
    while off + 36 <= len(pkt):
        code   = struct.unpack_from('<I', pkt, off)[0]
        values = struct.unpack_from('<8f', pkt, off + 4)
        rows[code] = values
        off += 36
    return rows


# ── joystick helpers ──────────────────────────────────────────────────────────

def _axis_pwm(v: float, invert: bool = False) -> int:
    if invert:
        v = -v
    return int(max(1000, min(2000, 1500 + v * 500)))


def _thr_pwm(v: float, invert: bool = False) -> int:
    if invert:
        v = -v
    return int(max(1000, min(2000, 1000 + (v + 1.0) * 500)))


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='X-Plane ↔ ArduPilot SITL bridge (--model JSON)'
    )
    ap.add_argument('--sitl',        default='tcp:127.0.0.1:5760',
                    help='MAVLink connection to SITL (default: tcp:127.0.0.1:5760)')
    ap.add_argument('--xplane-host', default='127.0.0.1')
    ap.add_argument('--xplane-port', type=int, default=49000,
                    help='X-Plane DREF receive port (default: 49000)')
    ap.add_argument('--bind-port',   type=int, default=49005,
                    help='Local port for X-Plane DATA@ (default: 49005)')
    ap.add_argument('--json-port',   type=int, default=9003,
                    help='SITL JSON state receive port (default: 9003)')
    ap.add_argument('--servo-port',  type=int, default=9002,
                    help='Local port for SITL servo packets (default: 9002)')
    # ── joystick ──────────────────────────────────────────────────────────────
    ap.add_argument('--joystick',         action='store_true')
    ap.add_argument('--joy-index',        type=int, default=0)
    ap.add_argument('--joy-roll-axis',    type=int, default=0)
    ap.add_argument('--joy-pitch-axis',   type=int, default=1)
    ap.add_argument('--joy-thr-axis',     type=int, default=2)
    ap.add_argument('--joy-yaw-axis',     type=int, default=3)
    ap.add_argument('--joy-fltmode-axis', type=int, default=-1,
                    help='Axis for CH5 / mode switch (-1 = disabled)')
    ap.add_argument('--joy-thr-invert',   action='store_true')
    ap.add_argument('--joy-rate',         type=int, default=50)
    ap.add_argument('--debug',            action='store_true')
    args = ap.parse_args()

    xp_addr   = (args.xplane_host, args.xplane_port)
    joy_ivl   = 1.0 / args.joy_rate

    # ── joystick init ─────────────────────────────────────────────────────────
    joystick = None
    if args.joystick:
        if not _PYGAME:
            print('[JOY] pygame not installed — run: pip3 install pygame')
        else:
            pygame.init()
            pygame.joystick.init()
            count = pygame.joystick.get_count()
            if count == 0:
                print('[JOY] No joystick detected')
            elif args.joy_index >= count:
                print(f'[JOY] Index {args.joy_index} out of range ({count} found)')
            else:
                joystick = pygame.joystick.Joystick(args.joy_index)
                joystick.init()
                print(f'[JOY] {joystick.get_name()}  '
                      f'axes={joystick.get_numaxes()}  '
                      f'buttons={joystick.get_numbuttons()}')

    # ── MAVLink to SITL ───────────────────────────────────────────────────────
    print(f'[MAV] Connecting to {args.sitl} …')
    mav = mavutil.mavlink_connection(args.sitl, source_system=255)
    mav.wait_heartbeat()
    print(f'[MAV] Heartbeat  sysid={mav.target_system}  compid={mav.target_component}')

    # ── X-Plane socket ────────────────────────────────────────────────────────
    xp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    xp_sock.bind(('0.0.0.0', args.bind_port))
    xp_sock.setblocking(False)
    print(f'[XP]  Listening on :{args.bind_port}  →  {xp_addr}')

    xp_dsel(xp_sock, xp_addr, REQUIRED_ROWS)
    xp_rref(xp_sock, xp_addr, 'sim/version/xplane_internal_version', RREF_VERSION, 1)
    xp_dref(xp_sock, xp_addr, 'sim/operation/override/override_joystick',  1.0)
    xp_dref(xp_sock, xp_addr, 'sim/operation/override/override_throttles', 1.0)
    xp_dref(xp_sock, xp_addr, 'sim/flightmodel/controls/parkbrake', 1.0)
    print('[XP]  Parking brake SET')
    print(f'[XP]  DSEL rows {REQUIRED_ROWS}')

    # ── JSON state socket (send to SITL) ──────────────────────────────────────
    json_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ── Servo socket (receive from SITL) ──────────────────────────────────────
    srv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv_sock.bind(('0.0.0.0', args.servo_port))
    srv_sock.setblocking(False)
    print(f'[JSON] State → 127.0.0.1:{args.json_port}   Servos ← :{args.servo_port}')

    # ── state ─────────────────────────────────────────────────────────────────
    xp_time    = 0.0
    lat        = 0.0
    lon        = 0.0
    alt_m      = 0.0
    agl_m      = 0.0
    ground_elev = 0.0
    roll_r     = 0.0
    pitch_r    = 0.0
    yaw_r      = 0.0
    gyro       = [0.0, 0.0, 0.0]        # rad/s body frame
    accel_body = [0.0, 0.0, -GRAVITY_MSS]  # m/s² body frame
    vel_ned    = [0.0, 0.0, 0.0]        # m/s NED

    xp_version       = 0
    pos_valid        = False
    att_valid        = False
    xp_sensor_updated = False

    # NED origin — set from first X-Plane fix
    home_lat  = None
    home_lon  = None
    home_alt  = None

    # status counters
    srv_count   = 0
    servos      = [0] * 8
    dref_sent   = False
    last_dsel   = time.monotonic()
    last_srv    = time.monotonic()
    last_debug  = time.monotonic()
    last_joy    = 0.0
    prev_joy    = [None] * 5

    print('\nRunning — Ctrl+C to stop\n')

    while True:
        now = time.monotonic()

        # ── refresh X-Plane subscriptions every 5 s ───────────────────────────
        if now - last_dsel >= 5.0:
            xp_dsel(xp_sock, xp_addr, REQUIRED_ROWS)
            xp_dref(xp_sock, xp_addr,
                    'sim/operation/override/override_joystick',  1.0)
            xp_dref(xp_sock, xp_addr,
                    'sim/operation/override/override_throttles', 1.0)
            last_dsel = now

        # ── receive X-Plane DATA@ ─────────────────────────────────────────────
        timeout = min(0.02, joy_ivl if joystick else 0.02)
        readable, _, _ = select.select([xp_sock], [], [], timeout)

        xp_sensor_updated = False
        try:
            while readable:
                pkt, _ = xp_sock.recvfrom(65536)

                if pkt[:4] == b'RREF' and len(pkt) >= 13:
                    code, val = struct.unpack_from('<If', pkt, 5)
                    if code == RREF_VERSION and xp_version == 0:
                        xp_version = int(val)
                        print(f'[XP]  X-Plane {xp_version // 10000} '
                              f'(build {xp_version})')
                    continue

                rows = xp_parse(pkt)
                if not rows:
                    continue

                is_xp12 = (xp_version // 10000) >= 12

                if ROW_TIMES in rows:
                    xp_time = rows[ROW_TIMES][2]

                if ROW_SPEED in rows:
                    pass  # airspeed not needed for JSON backend

                if ROW_GLOAD in rows:
                    d = rows[ROW_GLOAD]
                    accel_body = [
                         d[5] * GRAVITY_MSS,
                         d[6] * GRAVITY_MSS,
                        -d[4] * GRAVITY_MSS,
                    ]
                    xp_sensor_updated = True

                if ROW_ANG_VEL in rows:
                    d = rows[ROW_ANG_VEL]
                    if is_xp12:
                        gyro = [d[0] * DEG_TO_RAD,
                                d[1] * DEG_TO_RAD,
                                d[2] * DEG_TO_RAD]
                    else:
                        gyro = [d[1], d[0], d[2]]
                    xp_sensor_updated = True

                if ROW_PITCH_ROLL_HDG in rows:
                    d = rows[ROW_PITCH_ROLL_HDG]
                    pitch_r = d[0] * DEG_TO_RAD
                    roll_r  = d[1] * DEG_TO_RAD
                    yaw_r   = d[2] * DEG_TO_RAD
                    if not att_valid:
                        att_valid = True
                        print(f'[ATT] First attitude  hdg={math.degrees(yaw_r):.1f}°')
                    xp_sensor_updated = True

                if ROW_LAT_LON_ALT in rows:
                    d = rows[ROW_LAT_LON_ALT]
                    lat       = d[0]
                    lon       = d[1]
                    alt_m     = d[2] * FEET_TO_M
                    agl_m     = d[3] * FEET_TO_M
                    ground_elev = alt_m - agl_m
                    pos_valid = True

                if ROW_LOC_VEL_DIST in rows:
                    d = rows[ROW_LOC_VEL_DIST]
                    vel_ned = [-d[5], d[3], -d[4]]

        except BlockingIOError:
            pass

        # ── joystick → RC_CHANNELS_OVERRIDE ───────────────────────────────────
        if joystick:
            pygame.event.pump()
            ch1 = _axis_pwm(joystick.get_axis(args.joy_roll_axis))
            ch2 = _axis_pwm(joystick.get_axis(args.joy_pitch_axis), invert=True)
            ch3 = _thr_pwm( joystick.get_axis(args.joy_thr_axis),
                            invert=args.joy_thr_invert)
            ch4 = _axis_pwm(joystick.get_axis(args.joy_yaw_axis))
            ch5 = (_axis_pwm(joystick.get_axis(args.joy_fltmode_axis))
                   if args.joy_fltmode_axis >= 0 else UINT16_MAX)

            cur = [ch1, ch2, ch3, ch4, ch5]
            if cur != prev_joy:
                prev_joy = cur
                last_joy = now
                mav.mav.rc_channels_override_send(
                    mav.target_system, mav.target_component,
                    ch1, ch2, ch3, ch4,
                    ch5, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                )
                if args.debug:
                    print(f'[JOY] CH1={ch1} CH2={ch2} CH3={ch3} '
                          f'CH4={ch4} CH5={ch5}')

        if not pos_valid:
            mav.recv_match(blocking=False)
            if args.debug and now - last_debug >= 5.0:
                last_debug = now
                print('[wait] No X-Plane DATA@ — is X-Plane running and unpaused?')
            continue

        # ── set NED origin from first fix ─────────────────────────────────────
        if home_lat is None:
            home_lat = lat
            home_lon = lon
            home_alt = alt_m
            print(f'[JSON] NED origin  lat={lat:.6f}  lon={lon:.6f}  '
                  f'alt={alt_m:.1f} m')

        # ── send JSON state to SITL ───────────────────────────────────────────
        # SIM_JSON.cpp field layout:
        #   timestamp (double), imu.gyro (vec3f), imu.accel_body (vec3f),
        #   position (vec3d NED m from home), attitude (vec3f rad), velocity (vec3f m/s NED)
        if xp_sensor_updated:
            _DLAT = 111320.0
            pos_n = (lat - home_lat) * _DLAT
            pos_e = (lon - home_lon) * _DLAT * math.cos(math.radians(home_lat))
            pos_d = -(alt_m - home_alt)

            state = (
                '{"timestamp":' + f'{xp_time:.6f},'
                '"imu":{'
                f'"gyro":[{gyro[0]:.6f},{gyro[1]:.6f},{gyro[2]:.6f}],'
                f'"accel_body":[{accel_body[0]:.6f},{accel_body[1]:.6f},{accel_body[2]:.6f}]'
                '},'
                f'"position":[{pos_n:.4f},{pos_e:.4f},{pos_d:.4f}],'
                f'"attitude":[{roll_r:.6f},{pitch_r:.6f},{yaw_r:.6f}],'
                f'"velocity":[{vel_ned[0]:.4f},{vel_ned[1]:.4f},{vel_ned[2]:.4f}]'
                '}\n'
            )
            json_sock.sendto(state.encode(), ('127.0.0.1', args.json_port))

        # ── receive servo packet from SITL → X-Plane DREFs ───────────────────
        # SIM_JSON servo_packet_16: uint16 frame_rate, uint32 frame_count, uint16[16] pwm
        try:
            pkt, _ = srv_sock.recvfrom(256)
            if len(pkt) >= 6 + 16 * 2:
                pwm = struct.unpack_from('<16H', pkt, 6)
                servos = list(pwm[:8])
                srv_count += 1
                if not dref_sent:
                    dref_sent = True
                    print(f'[XP]  First DREF → {xp_addr}')
                for dref_name, ch, conv in SERVO_DREFS:
                    xp_dref(xp_sock, xp_addr, dref_name, conv(servos[ch]))
                if args.debug:
                    labels = ['ail', 'elev', 'thr', 'rud', 'ch5', 'ch6', 'ch7', 'ch8']
                    parts  = [f'{l}={v}' for l, v in zip(labels, servos) if v > 0]
                    print(f'[SRV] {" ".join(parts)}')
        except BlockingIOError:
            pass

        # ── periodic 1 s status ───────────────────────────────────────────────
        if now - last_srv >= 1.0:
            last_srv = now
            hdg = math.degrees(yaw_r) % 360.0
            print(f'[GPS] lat={lat:.6f}  lon={lon:.6f}  alt={alt_m:.1f} m  '
                  f'agl={agl_m:.1f} m  hdg={hdg:.1f}°')
            if not args.debug:
                labels = ['ail', 'elev', 'thr', 'rud', 'ch5', 'ch6', 'ch7', 'ch8']
                parts  = [f'{l}={v}' for l, v in zip(labels, servos) if v > 0]
                print(f'[SRV] {" ".join(parts) or "(none)"}  ({srv_count} pkt/s)')
            srv_count = 0

        # ── debug ─────────────────────────────────────────────────────────────
        if args.debug and now - last_debug >= 5.0:
            last_debug = now
            hdg = math.degrees(yaw_r) % 360
            print(f'[xp]   lat={lat:.5f}  lon={lon:.5f}  '
                  f'alt={alt_m:.1f} m  terrain={ground_elev:.1f} m')
            print(f'[att]  pitch={math.degrees(pitch_r):+.1f}°  '
                  f'roll={math.degrees(roll_r):+.1f}°  hdg={hdg:.1f}°')
            print(f'[vel]  N={vel_ned[0]:.1f}  E={vel_ned[1]:.1f}  '
                  f'D={vel_ned[2]:.1f} m/s')
            print(f'[gyro] {[round(g, 3) for g in gyro]} rad/s')
            print(f'[acc]  {[round(a, 2) for a in accel_body]} m/s²')
            print()


if __name__ == '__main__':
    main()
