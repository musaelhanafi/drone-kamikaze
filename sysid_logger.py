#!/usr/bin/env python3
"""
sysid_logger.py — Flight dynamics data logger for SITL ↔ X-Plane system identification.

Scenario
────────
  1. ArduPilot SITL runs in AUTO mode, OR a joystick drives the aircraft via
     RC_CHANNELS_OVERRIDE (use --joystick to enable manual control).
  2. This script reads servo outputs (SERVO_OUTPUT_RAW) from MAVLink and
     sends them to X-Plane as control surface DREFs.
  3. X-Plane responds with its own physics — attitude, angular rates, airspeed.
     This script reads X-Plane sensor data from port 49005 (DATA@ format).
  4. MAVLink attitude/rate data is read in parallel from SITL.
  5. Both streams are logged to CSV with synchronized timestamps.

The CSV is then analysed by sysid_analyze.py to produce suggested SITL
parameter values that match X-Plane's flight dynamics.

X-Plane setup
─────────────
  Settings → Net Connections → Data:
    "Send network data output" → to 127.0.0.1:49005
    Enable rows: 3 (speed), 4 (Gload), 16 (angular vel), 17 (pitch/roll/hdg),
                 20 (lat/lon/alt), 21 (local vel)

Joystick / RC override
──────────────────────
  Use --joystick 0  to enable the first joystick found by pygame.
  Five axes are mapped to RC channels 1-5 (aileron, elevator, throttle,
  rudder, aux/ch5).  Default axis order: 0 1 2 3 4.  Override with
  --joy-axes.  Invert individual axes with --joy-invert (e.g. "1 3" to
  invert elevator and rudder).

  Throttle (CH3) is mapped from the full axis range [-1, 1] to PWM
  [1000, 2000].  All other axes are centred: 0.0 → 1500 µs.

  RC_CHANNELS_OVERRIDE is sent every loop tick; set SITL to FBWA or
  MANUAL mode so ArduPilot passes the overrides through.

Usage
─────
  python3 sysid_logger.py                             # defaults (AUTO, no joystick)
  python3 sysid_logger.py --joystick 0                # joystick on device 0
  python3 sysid_logger.py --joystick 0 --joy-axes 0 1 3 2 4 --joy-invert 1
  python3 sysid_logger.py --out flight01.csv --debug
  python3 sysid_logger.py --xplane-port 49000 --sitl tcp:127.0.0.1:5760
"""

import argparse
import csv
import math
import select
import socket
import struct
import time
from pathlib import Path

# ── physical constants ────────────────────────────────────────────────────────
KNOTS_TO_MS      = 0.514444
FEET_TO_M        = 0.3048
GRAVITY_MSS      = 9.80665
DEG_TO_RAD       = math.pi / 180.0
SEA_PRESSURE_HPA = 1013.25
LAPSE_RATE       = 0.0065
T0_K             = 288.15

# ── HIL_SENSOR fields_updated bitmask ────────────────────────────────────────
_HIL_ACCEL = 0x007
_HIL_GYRO  = 0x038
_HIL_MAG   = 0x1C0
_HIL_BARO  = 0xE00
_HIL_TEMP  = 0x1000
_HIL_FIELDS = _HIL_ACCEL | _HIL_GYRO | _HIL_MAG | _HIL_BARO | _HIL_TEMP

_GPS_EPOCH_UNIX  = 315964800
_GPS_LEAP_SECONDS = 18


def _alt_to_pressure(alt_m: float) -> float:
    return SEA_PRESSURE_HPA * ((1.0 - LAPSE_RATE * alt_m / T0_K) ** 5.2561)


def _airspeed_to_diff_pressure(airspeed_ms: float) -> float:
    return 0.5 * 1.225 * airspeed_ms * airspeed_ms / 100.0


def _gps_time():
    t = time.time() + _GPS_LEAP_SECONDS - _GPS_EPOCH_UNIX
    week   = int(t / 604800)
    tow_ms = int((t % 604800) * 1000)
    return week, tow_ms


def _igrf_dipole_ned_gauss(lat_deg: float, lon_deg: float) -> list:
    g10, g11, h11 = -29351.0, -1410.0, 4545.0
    lat    = math.radians(lat_deg)
    lon    = math.radians(lon_deg)
    colat  = math.pi / 2.0 - lat
    sc, cc = math.sin(colat), math.cos(colat)
    sl, cl = math.sin(lon),   math.cos(lon)
    gsl    = g11 * cl + h11 * sl
    bn = -(g10 * sc - gsl * cc) * 1e-5
    be =  (g11 * sl - h11 * cl) * 1e-5
    bd = -2.0 * (g10 * cc + gsl * sc) * 1e-5
    return [bn, be, bd]


def _ned_to_body(vec_ned: list, roll_r: float, pitch_r: float, yaw_r: float) -> list:
    sr, cr = math.sin(roll_r),  math.cos(roll_r)
    sp, cp = math.sin(pitch_r), math.cos(pitch_r)
    sy, cy = math.sin(yaw_r),   math.cos(yaw_r)
    n, e, d = vec_ned
    bx = cp * cy * n + cp * sy * e - sp * d
    by = (sr * sp * cy - cr * sy) * n + (sr * sp * sy + cr * cy) * e + sr * cp * d
    bz = (cr * sp * cy + sr * sy) * n + (cr * sp * sy - sr * cy) * e + cr * cp * d
    return [bx, by, bz]

try:
    import pygame
    _PYGAME = True
except ImportError:
    _PYGAME = False

from pymavlink import mavutil

UINT16_MAX = 65535

# ── joystick helpers ──────────────────────────────────────────────────────────

_FLTMODE_CENTRES = [1165, 1295, 1425, 1555, 1685, 1815]

def _axis_pwm(v, invert=False):
    if invert:
        v = -v
    return int(max(1000, min(2000, 1500 + v * 500)))

def _thr_pwm(v, invert=False):
    if invert:
        v = -v
    return int(max(1000, min(2000, 1000 + (v + 1.0) * 500)))

def _fltmode_pwm(v):
    raw = int(max(1000, min(2000, 1500 + v * 500)))
    return min(_FLTMODE_CENTRES, key=lambda c: abs(c - raw))


# ── X-Plane DATA@ row codes (mirrors SIM_XPlane.cpp / mavlink_xplane.py) ─────
ROW_SPEED          = 3    # [0]=vind_kts  [4]=vtrue_kts  [7]=vgnd_kts
ROW_GLOAD          = 4    # [0]=G_total   [2]=vvi_fpm  [4]=Gz [5]=Gx [6]=Gy
ROW_ANG_VEL        = 16   # [0]=P deg/s   [1]=Q deg/s    [2]=R deg/s
ROW_PITCH_ROLL_HDG = 17   # [0]=pitch_deg [1]=roll_deg   [2]=hdg_true_deg
ROW_LAT_LON_ALT    = 20   # [0]=lat       [1]=lon        [2]=alt_ft_msl  [3]=alt_ft_agl
ROW_LOC_VEL_DIST   = 21   # [3]=vE_ms  [4]=vD_ms  [5]=vN_ms (sign-flipped)

XPLANE_DATA_ROWS = [ROW_SPEED, ROW_GLOAD, ROW_ANG_VEL,
                    ROW_PITCH_ROLL_HDG, ROW_LAT_LON_ALT, ROW_LOC_VEL_DIST]

# ── CSV column layout ─────────────────────────────────────────────────────────
CSV_FIELDS = [
    'time_s',
    # Control inputs (from SERVO_OUTPUT_RAW, normalised)
    'ail_r',       # left elevon   CH1  [-1, 1]
    'elev_r',      # right elevon  CH2  [-1, 1]
    'thr',         # throttle      CH3  [ 0, 1]
    'ch5_r',       # aux/CH5            [-1, 1]  (joystick axis 5; 0 when no joystick)
    # X-Plane state (from DATA@ port 49005)
    'xp_roll_deg',
    'xp_pitch_deg',
    'xp_hdg_deg',
    'xp_p_dps',    # roll rate  deg/s
    'xp_q_dps',    # pitch rate deg/s
    'xp_r_dps',    # yaw rate   deg/s
    'xp_ias_kts',
    'xp_alt_ft',
    'xp_vvi_fpm',
    # MAVLink state (from SIMSTATE / VFR_HUD)
    'mav_roll_deg',
    'mav_pitch_deg',
    'mav_hdg_deg',
    'mav_p_dps',
    'mav_q_dps',
    'mav_r_dps',
    'mav_airspeed_ms',
    'mav_climb_ms',
    'mav_alt_m',
]


# ── X-Plane UDP helpers ───────────────────────────────────────────────────────

def xp_parse_data(pkt: bytes) -> dict:
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


def xp_send_dref(sock, addr, name: str, value: float):
    name_b = name.encode()
    name_padded = name_b + b'\x00' * (500 - len(name_b))
    sock.sendto(b'DREF\x00' + struct.pack('<f', value) + name_padded, addr)


def xp_dsel(sock, addr, rows):
    """Ask X-Plane to start sending specific DATA@ row codes."""
    padded = (list(rows) + [0] * 8)[:8]
    sock.sendto(b'DSEL\x00' + struct.pack('<8I', *padded), addr)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='Log SITL servo outputs + X-Plane/MAVLink responses for sysid'
    )
    ap.add_argument('--sitl',         default='udp:127.0.0.1:14560',
                    help='MAVLink connection (default: udp:127.0.0.1:14560)')
    ap.add_argument('--xplane-host',  default='127.0.0.1')
    ap.add_argument('--xplane-port',  type=int, default=49000,
                    help='X-Plane DREF receive port (default: 49000)')
    ap.add_argument('--xplane-data',  type=int, default=49005,
                    help='X-Plane DATA@ output port this script listens on (default: 49005)')
    ap.add_argument('--out',          default='sysid_log.csv',
                    help='Output CSV file (default: sysid_log.csv)')
    ap.add_argument('--rate',         type=int, default=50,
                    help='MAVLink message rate Hz (default: 50)')
    ap.add_argument('--hil-rate',     type=int, default=100,
                    help='Maximum HIL_SENSOR send rate Hz (default: 100)')
    ap.add_argument('--gps-rate',     type=int, default=5,
                    help='GPS_INPUT send rate Hz (default: 5)')
    # ── joystick ──────────────────────────────────────────────────────────────
    ap.add_argument('--joystick',         action='store_true')
    ap.add_argument('--joy-index',        type=int, default=0)
    ap.add_argument('--joy-roll-axis',    type=int, default=0)
    ap.add_argument('--joy-pitch-axis',   type=int, default=1)
    ap.add_argument('--joy-thr-axis',     type=int, default=2)
    ap.add_argument('--joy-yaw-axis',     type=int, default=3)
    ap.add_argument('--joy-fltmode-axis', type=int, default=4)
    ap.add_argument('--joy-thr-invert',   action='store_true')
    ap.add_argument('--debug',            action='store_true')
    args = ap.parse_args()

    xp_addr = (args.xplane_host, args.xplane_port)

    # ── joystick init ─────────────────────────────────────────────────────────
    joystick = None
    if args.joystick:
        if not _PYGAME:
            print('[JOY] pygame not installed — run: pip3 install pygame')
            return
        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if count == 0:
            print('[JOY] No joystick detected — exiting')
            return
        if args.joy_index >= count:
            print(f'[JOY] Index {args.joy_index} out of range ({count} found) — exiting')
            return
        joystick = pygame.joystick.Joystick(args.joy_index)
        joystick.init()
        print(f'[JOY] {joystick.get_name()}  '
              f'axes={joystick.get_numaxes()}  '
              f'buttons={joystick.get_numbuttons()}')

    # ── sockets ───────────────────────────────────────────────────────────────
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_sock.bind(('', args.xplane_data))
    data_sock.setblocking(False)
    print(f'[XP]  listening for DATA@ on :{args.xplane_data}')

    # Ask X-Plane to send the rows we need
    xp_dsel(send_sock, xp_addr, XPLANE_DATA_ROWS)
    print(f'[XP]  DSEL sent for rows {XPLANE_DATA_ROWS}')

    # Take ownership of joystick, throttles and control surfaces so X-Plane
    # does not fight our DREF writes with its own flight model.
    xp_send_dref(send_sock, xp_addr, 'sim/operation/override/override_joystick',         1.0)
    xp_send_dref(send_sock, xp_addr, 'sim/operation/override/override_throttles',        1.0)
    xp_send_dref(send_sock, xp_addr, 'sim/operation/override/override_control_surfaces', 1.0)
    print('[XP]  overrides SET (joystick + throttles + control surfaces)')

    # ── MAVLink ───────────────────────────────────────────────────────────────
    print(f'[MAV] connecting to {args.sitl} …')
    mav = mavutil.mavlink_connection(args.sitl, source_system=255)
    mav.wait_heartbeat()
    print(f'[MAV] heartbeat  sysid={mav.target_system}')

    # Enable HIL mode so ArduPilot fuses HIL_SENSOR / GPS_INPUT
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED |
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0,
    )
    print('[MAV] HIL mode enabled')

    def _req(msg_id, hz):
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0,
        )

    _req(164,                                                    args.rate)  # SIMSTATE
    _req(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,        args.rate)
    _req(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,                 args.rate)
    _req(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,     args.rate)
    print(f'[MAV] subscribed at {args.rate} Hz')

    # ── state ─────────────────────────────────────────────────────────────────
    # MAVLink
    ail_r = elev_r = thr = ch5_r = 0.0
    mav_roll = mav_pitch = mav_hdg = 0.0
    mav_p = mav_q = mav_r = 0.0
    mav_airspeed = mav_climb = mav_alt = 0.0
    # X-Plane
    xp_roll = xp_pitch = xp_hdg = 0.0
    xp_p = xp_q = xp_r = 0.0
    xp_ias = xp_alt = xp_vvi = 0.0
    # HIL state (SI units)
    lat = lon = 0.0
    alt_m = agl_m = 0.0
    roll_r = pitch_r = yaw_r = 0.0
    gyro        = [0.0, 0.0, 0.0]
    accel_body  = [0.0, 0.0, -GRAVITY_MSS]
    vel_ned     = [0.0, 0.0, 0.0]
    airspeed_ms = 0.0
    xp_sensor_updated = False

    hil_min_ivl = 1.0 / args.hil_rate
    gps_ivl     = 1.0 / args.gps_rate
    last_hil    = 0.0
    last_gps    = 0.0
    last_rc     = 0.0
    rc_ivl      = 1.0 / args.rate   # RC heartbeat at same rate as logging

    xp_valid   = False
    mav_valid  = False
    home_set   = False
    prev_joy   = [None] * 5
    t0 = time.monotonic()

    # ── helper commands ───────────────────────────────────────────────────────
    def _set_home(lat_deg, lon_deg, alt_m):
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,
            0,          # param1: 0 = use specified location
            0, 0, 0,
            lat_deg, lon_deg, alt_m,
        )
        print(f'[MAV] home set  lat={lat_deg:.6f}  lon={lon_deg:.6f}  alt={alt_m:.1f} m')

    def _arm():
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,          # 1 = arm
            0, 0, 0, 0, 0, 0,
        )
        print('[MAV] arm sent')

    def _set_mode_auto():
        # Request AUTO mode via MAV_CMD_DO_SET_MODE
        # Mode number for ArduPlane AUTO = 10
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            10,         # ArduPlane AUTO mode number
            0, 0, 0, 0, 0,
        )
        print('[MAV] mode AUTO sent')

    # ── CSV ───────────────────────────────────────────────────────────────────
    out_path = Path(args.out)
    print(f'[LOG] writing to {out_path}')
    csv_file = out_path.open('w', newline='')
    writer   = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
    writer.writeheader()

    last_log   = time.monotonic()
    last_dsel  = time.monotonic()
    log_ivl    = 1.0 / args.rate
    row_count  = 0

    print('\nWaiting for first X-Plane DATA@ packet to set home …\n')

    try:
        while True:
            now = time.monotonic()

            # ── re-subscribe every 5 s (X-Plane resets overrides on reload) ──
            if now - last_dsel >= 5.0:
                xp_dsel(send_sock, xp_addr, XPLANE_DATA_ROWS)
                xp_send_dref(send_sock, xp_addr,
                             'sim/operation/override/override_joystick',         1.0)
                xp_send_dref(send_sock, xp_addr,
                             'sim/operation/override/override_throttles',        1.0)
                xp_send_dref(send_sock, xp_addr,
                             'sim/operation/override/override_control_surfaces', 1.0)
                last_dsel = now

            # ── drain X-Plane DATA@ ───────────────────────────────────────────
            while True:
                try:
                    pkt, _ = data_sock.recvfrom(4096)
                except (BlockingIOError, OSError):
                    break
                rows = xp_parse_data(pkt)
                if not rows:
                    continue
                if ROW_PITCH_ROLL_HDG in rows:
                    v = rows[ROW_PITCH_ROLL_HDG]
                    xp_pitch, xp_roll, xp_hdg = v[0], v[1], v[2]
                    pitch_r = xp_pitch * DEG_TO_RAD
                    roll_r  = xp_roll  * DEG_TO_RAD
                    yaw_r   = xp_hdg   * DEG_TO_RAD
                    xp_sensor_updated = True
                if ROW_ANG_VEL in rows:
                    v = rows[ROW_ANG_VEL]
                    xp_p, xp_q, xp_r = v[0], v[1], v[2]
                    # XP12 reports deg/s; convert to rad/s for HIL_SENSOR
                    gyro = [v[0] * DEG_TO_RAD, v[1] * DEG_TO_RAD, v[2] * DEG_TO_RAD]
                    xp_sensor_updated = True
                if ROW_SPEED in rows:
                    xp_ias      = rows[ROW_SPEED][0]
                    airspeed_ms = xp_ias * KNOTS_TO_MS
                if ROW_LAT_LON_ALT in rows:
                    v = rows[ROW_LAT_LON_ALT]
                    lat   = v[0]
                    lon   = v[1]
                    alt_m = v[2] * FEET_TO_M
                    agl_m = v[3] * FEET_TO_M
                    xp_alt = v[2]                        # ft MSL (CSV)
                    if not home_set and abs(v[0]) > 0.001:
                        _set_home(lat, lon, alt_m - agl_m)
                        _arm()
                        xp_send_dref(send_sock, xp_addr,
                                     'sim/flightmodel/controls/parkbrake', 0.0)
                        print('[XP]  parking brake released')
                        # _set_mode_auto()
                        home_set = True
                        print('\n[LOG] Logging — Ctrl+C to stop\n')
                if ROW_GLOAD in rows:
                    d = rows[ROW_GLOAD]
                    xp_vvi     = d[2]                    # ft/min (CSV)
                    accel_body = [
                         d[5] * GRAVITY_MSS,
                         d[6] * GRAVITY_MSS,
                        -d[4] * GRAVITY_MSS,
                    ]
                    xp_sensor_updated = True
                if ROW_LOC_VEL_DIST in rows:
                    d = rows[ROW_LOC_VEL_DIST]
                    vel_ned = [-d[5], d[3], -d[4]]       # [N, E, D] m/s
                xp_valid = True

            # ── HIL_SENSOR ────────────────────────────────────────────────────
            if xp_sensor_updated and (now - last_hil) >= hil_min_ivl:
                last_hil = now
                xp_sensor_updated = False
                abs_p  = _alt_to_pressure(alt_m)
                diff_p = _airspeed_to_diff_pressure(airspeed_ms)
                temp_c = 15.0 - 0.0065 * alt_m
                mag_ned  = _igrf_dipole_ned_gauss(lat, lon)
                mag_body = _ned_to_body(mag_ned, roll_r, pitch_r, yaw_r)
                mav.mav.hil_sensor_send(
                    int(now * 1e6),
                    accel_body[0], accel_body[1], accel_body[2],
                    gyro[0],       gyro[1],       gyro[2],
                    mag_body[0],   mag_body[1],   mag_body[2],
                    abs_p, diff_p, agl_m, temp_c,
                    _HIL_FIELDS, 0,
                )

            # ── GPS_INPUT ─────────────────────────────────────────────────────
            if xp_valid and (now - last_gps) >= gps_ivl:
                last_gps = now
                gps_week, gps_tow_ms = _gps_time()
                raw    = int((yaw_r % (2 * math.pi)) * 18000 / math.pi)
                hdg_cd = raw if raw != 0 else 36000
                mav.mav.gps_input_send(
                    int(now * 1e6),
                    0, 0,
                    gps_tow_ms, gps_week,
                    3,
                    int(lat * 1e7), int(lon * 1e7), alt_m,
                    1.0, 1.0,
                    vel_ned[0], vel_ned[1], vel_ned[2],
                    0.2, 0.3, 0.5,
                    10,
                    hdg_cd,
                )

            # ── drain MAVLink ─────────────────────────────────────────────────
            while True:
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break
                t = msg.get_type()

                if t == 'SERVO_OUTPUT_RAW':
                    ail_r  = (msg.servo1_raw - 1500) / 500.0
                    elev_r = (msg.servo2_raw - 1500) / 500.0
                    thr    = (msg.servo3_raw - 1000) / 1000.0
                    # Flying-wing elevon: ArduPilot already mixes aileron+elevator
                    # into per-elevon PWM on CH1/CH2.  Write directly to wing
                    # surface deflection DREFs (degrees) so X-Plane applies no
                    # additional mixing.  20° matches Plane Maker control surface
                    # deflection limits.
                    # CH1 = SERVO1 = ELEVON_LEFT  → wing1l
                    # CH2 = SERVO2 = ELEVON_RIGHT → wing1r
                    xp_send_dref(send_sock, xp_addr,
                                 'sim/flightmodel/controls/wing1l_ail1def',
                                 20.0 * (msg.servo1_raw - 1500) / 500.0)
                    xp_send_dref(send_sock, xp_addr,
                                 'sim/flightmodel/controls/wing1r_ail1def',
                                 20.0 * (msg.servo2_raw - 1500) / 500.0)
                    xp_send_dref(send_sock, xp_addr,
                                 'sim/cockpit2/engine/actuators/throttle_ratio[0]',
                                 max(0.0, min(1.0, thr)))

                elif t == 'SIMSTATE':
                    mav_roll  = math.degrees(msg.roll)
                    mav_pitch = math.degrees(msg.pitch)
                    mav_hdg   = math.degrees(msg.yaw) % 360.0
                    mav_p     = math.degrees(msg.xgyro)
                    mav_q     = math.degrees(msg.ygyro)
                    mav_r     = math.degrees(msg.zgyro)
                    mav_valid = True

                elif t == 'VFR_HUD':
                    mav_airspeed = msg.airspeed
                    mav_climb    = msg.climb

                elif t == 'GLOBAL_POSITION_INT':
                    mav_alt = msg.alt * 1e-3

            # ── joystick → RC_CHANNELS_OVERRIDE ──────────────────────────────
            if joystick:
                pygame.event.pump()
                ch1 = _axis_pwm(joystick.get_axis(args.joy_roll_axis))
                ch2 = _axis_pwm(joystick.get_axis(args.joy_pitch_axis), invert=True)
                ch3 = _thr_pwm( joystick.get_axis(args.joy_thr_axis),
                                invert=args.joy_thr_invert)
                ch4 = _axis_pwm(joystick.get_axis(args.joy_yaw_axis))
                ch5 = (_fltmode_pwm(joystick.get_axis(args.joy_fltmode_axis))
                       if args.joy_fltmode_axis >= 0 else UINT16_MAX)
                cur = [ch1, ch2, ch3, ch4, ch5]
                changed = cur != prev_joy
                if changed:
                    prev_joy = cur
                # Send on change OR as periodic heartbeat to prevent RC failsafe
                if changed or (now - last_rc) >= rc_ivl:
                    last_rc = now
                    mav.mav.rc_channels_override_send(
                        mav.target_system, mav.target_component,
                        ch1, ch2, ch3, ch4,
                        ch5, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                    )
                    if args.debug and changed:
                        print(f'[JOY] CH1={ch1} CH2={ch2} CH3={ch3} '
                              f'CH4={ch4} CH5={ch5}')
                ch5_r = (ch5 - 1500) / 500.0 if ch5 != UINT16_MAX else 0.0

            # ── log row ───────────────────────────────────────────────────────
            if mav_valid and (now - last_log) >= log_ivl:
                last_log = now
                row = {
                    'time_s':        round(now - t0, 4),
                    'ail_r':         round(ail_r,  4),
                    'elev_r':        round(elev_r, 4),
                    'thr':           round(thr,    4),

                    'ch5_r':         round(ch5_r,  4),
                    'xp_roll_deg':   round(xp_roll,  3),
                    'xp_pitch_deg':  round(xp_pitch, 3),
                    'xp_hdg_deg':    round(xp_hdg,   3),
                    'xp_p_dps':      round(xp_p,  3),
                    'xp_q_dps':      round(xp_q,  3),
                    'xp_r_dps':      round(xp_r,  3),
                    'xp_ias_kts':    round(xp_ias, 3),
                    'xp_alt_ft':     round(xp_alt, 1),
                    'xp_vvi_fpm':    round(xp_vvi, 1),
                    'mav_roll_deg':  round(mav_roll,  3),
                    'mav_pitch_deg': round(mav_pitch, 3),
                    'mav_hdg_deg':   round(mav_hdg,   3),
                    'mav_p_dps':     round(mav_p,  3),
                    'mav_q_dps':     round(mav_q,  3),
                    'mav_r_dps':     round(mav_r,  3),
                    'mav_airspeed_ms': round(mav_airspeed, 3),
                    'mav_climb_ms':    round(mav_climb, 3),
                    'mav_alt_m':       round(mav_alt, 2),
                }
                writer.writerow(row)
                row_count += 1

                if args.debug:
                    print(f'  t={row["time_s"]:7.2f}  '
                          f'ail={ail_r:+.2f}  elev={elev_r:+.2f}  '
                          f'xp_roll={xp_roll:+6.1f}°  xp_p={xp_p:+6.1f}°/s  '
                          f'mav_roll={mav_roll:+6.1f}°')
                elif row_count % (args.rate * 5) == 0:
                    # Status every 5 s
                    xp_str = f'xp_ok' if xp_valid else 'xp_wait'
                    print(f'[LOG] {row["time_s"]:6.1f} s  {row_count} rows  '
                          f'ail={ail_r:+.2f}  thr={thr:.2f}  {xp_str}')

            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        csv_file.flush()
        csv_file.close()
        print(f'\n[LOG] {row_count} rows → {out_path}')


if __name__ == '__main__':
    main()
