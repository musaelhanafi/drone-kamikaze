#!/usr/bin/env python3
"""
sitl_xplane.py — ArduPilot SITL → X-Plane visualizer (raw UDP, no plugin needed).

ArduPilot SITL runs its own aerodynamic model (SIM_Plane).  This script reads
aircraft state from SITL via MAVLink and writes position/attitude to X-Plane
using X-Plane's native UDP DREF protocol.  No XPlaneConnect plugin required.

Architecture
────────────
  ArduPilot SITL ──MAVLink──▶  sitl_xplane.py ──UDP DREF──▶  X-Plane 11
  SIMSTATE + GLOBAL_POSITION_INT  →  lat/lon/alt/pitch/roll/hdg DREFs
                                     (override_planepath=1, physics bypassed)

X-Plane setup
─────────────
  Settings → Net Connections → Data:
    Accept network data input: port 49010 (or default 49000)
  No other settings needed.

Usage
─────
  # Terminal 1 — SITL
  python3 Tools/autotest/sim_vehicle.py --vehicle Plane --location WICC

  # Terminal 2 — mirror to X-Plane
  python3 sitl_xplane.py
  python3 sitl_xplane.py --xplane-port 49010 --debug
"""

import argparse
import math
import socket
import struct
import time

from pymavlink import mavutil

try:
    import pygame
    _PYGAME = True
except ImportError:
    _PYGAME = False

UINT16_MAX = 65535
R_EARTH    = 6_371_000.0   # metres

# ── X-Plane native UDP helpers ────────────────────────────────────────────────

def xp_send_dref(sock, addr, name: str, value: float):
    """Send a single DREF write packet (X-Plane native format, 509 bytes)."""
    name_b = name.encode()
    name_padded = name_b + b'\x00' * (500 - len(name_b))
    sock.sendto(b'DREF\x00' + struct.pack('<f', value) + name_padded, addr)


def xp_send_rref(sock, addr, freq: int, index: int, name: str):
    """Subscribe to a DREF via RREF.  X-Plane replies to the source port.
    Set freq=0 to unsubscribe."""
    name_b = name.encode()
    name_padded = name_b + b'\x00' * (400 - len(name_b))
    sock.sendto(b'RREF\x00' + struct.pack('<ii', freq, index) + name_padded, addr)


def xp_parse_rref(data: bytes):
    """Parse an RREF response; return list of (index, value) pairs."""
    if len(data) < 5 or data[:4] != b'RREF':
        return []
    results = []
    offset = 5
    while offset + 8 <= len(data):
        idx, val = struct.unpack_from('<if', data, offset)
        results.append((idx, val))
        offset += 8
    return results



def latlon_to_local(lat, lon, alt, lat0, lon0, alt0, lx0, ly0, lz0):
    """Flat-Earth lat/lon/alt → X-Plane local_x/y/z.
    X-Plane local frame: +x east, +y up, +z south.
    """
    dx =  math.radians(lon - lon0) * math.cos(math.radians(lat0)) * R_EARTH
    dy =  alt - alt0
    dz = -math.radians(lat - lat0) * R_EARTH
    return lx0 + dx, ly0 + dy, lz0 + dz


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


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='Mirror ArduPilot SITL state to X-Plane via native UDP DREFs'
    )
    ap.add_argument('--sitl',         default='udp:127.0.0.1:14560',
                    help='MAVLink connection to SITL (default: udp:127.0.0.1:14560)')
    ap.add_argument('--xplane-host',  default='127.0.0.1')
    ap.add_argument('--xplane-port',  type=int, default=49000,
                    help='X-Plane UDP receive port (default: 49000)')
    ap.add_argument('--bind-port',    type=int, default=49020,
                    help='Local port to receive RREF replies (default: 49020)')
    ap.add_argument('--update-rate',  type=int, default=100,
                    help='Position DREF update rate Hz (default: 100)')
    # ── joystick ──────────────────────────────────────────────────────────────
    ap.add_argument('--joystick',         action='store_true')
    ap.add_argument('--joy-index',        type=int, default=0)
    ap.add_argument('--joy-roll-axis',    type=int, default=0)
    ap.add_argument('--joy-pitch-axis',   type=int, default=1)
    ap.add_argument('--joy-thr-axis',     type=int, default=2)
    ap.add_argument('--joy-yaw-axis',     type=int, default=3)
    ap.add_argument('--joy-fltmode-axis', type=int, default=-1)
    ap.add_argument('--joy-thr-invert',   action='store_true')
    ap.add_argument('--debug',            action='store_true')
    args = ap.parse_args()

    xp_addr  = (args.xplane_host, args.xplane_port)
    upd_ivl  = 1.0 / args.update_rate

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

    # ── MAVLink ───────────────────────────────────────────────────────────────
    print(f'[MAV] Connecting to {args.sitl} …')
    mav = mavutil.mavlink_connection(args.sitl, source_system=255)
    mav.wait_heartbeat()
    print(f'[MAV] Heartbeat  sysid={mav.target_system}  compid={mav.target_component}')

    def subscribe_rref(sock, addr, freq: int, index: int, name: str):
        """Subscribe to a DREF via RREF.  X-Plane replies to sock's bound port."""
        name_b = name.encode()
        name_padded = name_b + b'\x00' * (400 - len(name_b))
        sock.sendto(b'RREF\x00' + struct.pack('<ii', freq, index) + name_padded, addr)
    def unsubscribe_rref(sock, addr, index: int, name: str):
        subscribe_rref(sock, addr, 0, index, name)

    def _req_interval(msg_id, hz):
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0,
        )

    def _set_param(name, value):
        mav.mav.param_set_send(
            mav.target_system, mav.target_component,
            name.encode(), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

    # Params for SITL (software) or fmuv3-hil hardware SITL.
    # These match defaults.parm embedded in the fmuv3-hil firmware.
    _set_param('GPS1_TYPE',      1)    # SITL internal GPS
    _set_param('EK3_SRC1_YAW',   1)    # compass yaw
    _set_param('COMPASS_USE',    1)    # SITL compass
    _set_param('ARSPD_TYPE',     100)  # SITL airspeed backend
    _set_param('ARMING_REQUIRE', 0)
    _set_param('SIM_SPEEDUP',    1)
    print('[MAV] params set')

    _req_interval(164, args.update_rate)   # SIMSTATE (attitude + lat/lon + angular rates)
    _req_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, args.update_rate)  # alt + NED velocity
    _req_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, args.update_rate)     # servos → control surfaces + throttle
    _req_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, args.update_rate)              # climb rate for VSI

    # ── UDP sockets ───────────────────────────────────────────────────────────
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(('', args.bind_port))
    recv_sock.setblocking(False)
    print(f'[XP]  send → {args.xplane_host}:{args.xplane_port}  '
          f'recv ← :{args.bind_port}')

    # Disable physics so the aircraft follows DREF position without crashing.
    xp_send_dref(send_sock, xp_addr, 'sim/operation/override/override_planepath[0]', 1.0)
    xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/controls/parkbrake', 0.0)
    # Zero X-Plane winds so IAS is computed from our velocity vectors alone.
    for layer in range(3):
        xp_send_dref(send_sock, xp_addr, f'sim/weather/wind_speed_kt[{layer}]', 0.0)
    print('[XP]  override_planepath=1  parkbrake=0  wind=0')

    # ── get X-Plane local origin + position via RREF ──────────────────────────
    # Indices 1-3: local_x/y/z — writable position DREFs with override_planepath=1
    # Indices 4-6: lat/lon/elevation — used to set FC home position
    subscribe_rref(recv_sock, xp_addr, 4, 1, 'sim/flightmodel/position/local_x')
    subscribe_rref(recv_sock, xp_addr, 4, 2, 'sim/flightmodel/position/local_y')
    subscribe_rref(recv_sock, xp_addr, 4, 3, 'sim/flightmodel/position/local_z')
    subscribe_rref(recv_sock, xp_addr, 4, 4, 'sim/flightmodel/position/latitude')
    subscribe_rref(recv_sock, xp_addr, 4, 5, 'sim/flightmodel/position/longitude')
    subscribe_rref(recv_sock, xp_addr, 4, 6, 'sim/flightmodel/position/elevation')
    print('[XP]  Waiting for RREF local_x/y/z + lat/lon/alt …')

    lx0 = ly0 = lz0 = None
    home_lat = home_lon = home_alt = None
    rref_deadline = time.monotonic() + 10.0
    while time.monotonic() < rref_deadline:
        try:
            data, _ = recv_sock.recvfrom(4096)
            for idx, val in xp_parse_rref(data):
                if   idx == 1: lx0      = val
                elif idx == 2: ly0      = val
                elif idx == 3: lz0      = val
                elif idx == 4: home_lat = val
                elif idx == 5: home_lon = val
                elif idx == 6: home_alt = val
        except (BlockingIOError, OSError):
            time.sleep(0.05)
        if all(v is not None for v in (lx0, ly0, lz0, home_lat, home_lon, home_alt)):
            break

    if lx0 is None:
        print('[XP]  ERROR: no RREF reply — is X-Plane running and unpaused?')
        return

    # Unsubscribe all RREF channels
    unsubscribe_rref(recv_sock, xp_addr, 1, 'sim/flightmodel/position/local_x')
    unsubscribe_rref(recv_sock, xp_addr, 2, 'sim/flightmodel/position/local_y')
    unsubscribe_rref(recv_sock, xp_addr, 3, 'sim/flightmodel/position/local_z')
    unsubscribe_rref(recv_sock, xp_addr, 4, 'sim/flightmodel/position/latitude')
    unsubscribe_rref(recv_sock, xp_addr, 5, 'sim/flightmodel/position/longitude')
    unsubscribe_rref(recv_sock, xp_addr, 6, 'sim/flightmodel/position/elevation')
    print(f'[XP]  Origin  lx0={lx0:.1f}  ly0={ly0:.1f}  lz0={lz0:.1f}')
    print(f'[XP]  Home    lat={home_lat:.6f}  lon={home_lon:.6f}  alt={home_alt:.1f} m')

    # ── set FC home position ───────────────────────────────────────────────────
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,          # confirmation
        0,          # param1: 0 = use specified location (not current)
        0, 0, 0,    # param2-4: unused
        home_lat,   # param5: latitude  (degrees)
        home_lon,   # param6: longitude (degrees)
        home_alt,   # param7: altitude  (m MSL)
    )
    print(f'[MAV] Home set  lat={home_lat:.6f}  lon={home_lon:.6f}  alt={home_alt:.1f} m')

    # ── state ─────────────────────────────────────────────────────────────────
    lat     = 0.0
    lon     = 0.0
    alt_m   = 0.0
    roll_r  = 0.0
    pitch_r = 0.0
    yaw_r   = 0.0

    att_valid   = False
    alt_valid   = False
    state_valid = False
    # Use the RREF-derived home as the flat-Earth reference — it was captured
    # at the same instant as lx0/ly0/lz0, so the mapping is exact.
    ref_lat, ref_lon, ref_alt = home_lat, home_lon, home_alt

    # velocity (NED, m/s) and angular rates (rad/s) — set from MAVLink, used for smooth visuals
    vx_ned = vy_ned = vz_ned = 0.0   # north / east / down
    p_rad  = q_rad  = r_rad  = 0.0   # roll / pitch / yaw rate
    throttle    = 0.0                # 0..1 for engine animation
    aileron_r   = 0.0                # -1..1  (CH1: roll)
    elevator_r  = 0.0                # -1..1  (CH2: pitch)
    rudder_r    = 0.0                # -1..1  (CH4: yaw)
    climb_ms    = 0.0                # climb rate, m/s (up positive)

    # Dead-reckoning: truth anchor updated on each SIMSTATE, then extrapolated
    # at the X-Plane update rate using NED velocity.
    snap_lx   = lx0
    snap_ly   = ly0
    snap_lz   = lz0
    snap_time = time.monotonic()

    prev_joy      = [None] * 5
    last_upd      = 0.0
    last_srv      = time.monotonic()
    last_override = time.monotonic()

    print('\nRunning — Ctrl+C to stop\n')

    while True:
        now = time.monotonic()

        # ── joystick → RC_CHANNELS_OVERRIDE ───────────────────────────────────
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
            if cur != prev_joy:
                prev_joy = cur
                mav.mav.rc_channels_override_send(
                    mav.target_system, mav.target_component,
                    ch1, ch2, ch3, ch4,
                    ch5, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                )
                if args.debug:
                    print(f'[JOY] CH1={ch1} CH2={ch2} CH3={ch3} '
                          f'CH4={ch4} CH5={ch5}')

        # ── drain MAVLink ──────────────────────────────────────────────────────
        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()

            if t == 'SIMSTATE':
                roll_r  = msg.roll
                pitch_r = msg.pitch
                yaw_r   = msg.yaw
                lat     = msg.lat * 1e-7
                lon     = msg.lng * 1e-7
                p_rad   = msg.xgyro
                q_rad   = msg.ygyro
                r_rad   = msg.zgyro
                att_valid = True
                # Anchor dead-reckoning to this truth position
                if alt_valid:
                    snap_lx, snap_ly, snap_lz = latlon_to_local(
                        lat, lon, alt_m,
                        ref_lat, ref_lon, ref_alt,
                        lx0, ly0, lz0,
                    )
                    snap_time = now

            elif t == 'GLOBAL_POSITION_INT':
                alt_m   = msg.alt * 1e-3        # mm → m MSL
                vx_ned  = msg.vx * 0.01         # cm/s → m/s  (north)
                vy_ned  = msg.vy * 0.01         # cm/s → m/s  (east)
                vz_ned  = msg.vz * 0.01         # cm/s → m/s  (down)
                alt_valid = True

            elif t == 'SERVO_OUTPUT_RAW':
                aileron_r  = (msg.servo1_raw - 1500) / 500.0
                elevator_r = (msg.servo2_raw - 1500) / 500.0
                throttle   = (msg.servo3_raw - 1000) / 1000.0
                rudder_r   = (msg.servo4_raw - 1500) / 500.0

            elif t == 'VFR_HUD':
                climb_ms = msg.climb         # m/s climb rate (up positive)

            if att_valid and alt_valid and not state_valid:
                state_valid = True
                print(f'[SIM] First state  lat={lat:.6f}  lon={lon:.6f}  '
                      f'alt={alt_m:.1f} m  hdg={math.degrees(yaw_r) % 360:.1f}°')

        # ── refresh override_planepath every 5 s ──────────────────────────────
        if now - last_override >= 5.0:
            last_override = now
            xp_send_dref(send_sock, xp_addr,
                         'sim/operation/override/override_planepath[0]', 1.0)

        # ── write position + attitude DREFs → X-Plane ─────────────────────────
        if state_valid and (now - last_upd) >= upd_ivl:
            last_upd = now

            pitch_deg = math.degrees(pitch_r)
            roll_deg  = math.degrees(roll_r)
            hdg_deg   = math.degrees(yaw_r) % 360.0

            # Dead-reckon from last truth anchor using NED velocity.
            # X-Plane local frame: +x east, +y up, +z south.
            dt = now - snap_time
            lx = snap_lx + vy_ned  * dt
            ly = snap_ly - vz_ned  * dt
            lz = snap_lz - vx_ned  * dt

            # position + attitude — theta/phi/psi are writable; X-Plane derives q[] itself
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/local_x', lx)
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/local_y', ly)
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/local_z', lz)
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/theta',   pitch_deg)
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/phi',     roll_deg)
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/psi',     hdg_deg)

            # velocity — NED → X-Plane local (+x east, +y up, +z south)
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/local_vx',  vy_ned)
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/local_vy', -vz_ned)
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/local_vz', -vx_ned)

            # angular rates (body frame, rad/s)
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/Prad', p_rad)
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/Qrad', q_rad)
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/Rrad', r_rad)

            # control surfaces — joystick ratios are writable; X-Plane converts to deflection
            xp_send_dref(send_sock, xp_addr, 'sim/joystick/yoke_roll_ratio',    aileron_r)
            xp_send_dref(send_sock, xp_addr, 'sim/joystick/yoke_pitch_ratio',  -elevator_r)
            xp_send_dref(send_sock, xp_addr, 'sim/joystick/yoke_heading_ratio', rudder_r)

            # throttle — actuator ratio is writable (ENGN_thro_use is read-only)
            xp_send_dref(send_sock, xp_addr, 'sim/cockpit2/engine/actuators/throttle_ratio[0]',
                         max(0.0, min(1.0, throttle)))

            # gyro gauges — vacuum ADI reads pitch_vacuum/roll_vacuum directly
            # (heading DI has internal gyro state; psi_vac_ind_deg is not writable)
            xp_send_dref(send_sock, xp_addr,
                         'sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot', pitch_deg)
            xp_send_dref(send_sock, xp_addr,
                         'sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot',  roll_deg)
            xp_send_dref(send_sock, xp_addr,
                         'sim/cockpit/gyros/the_vac_ind_deg', pitch_deg)
            xp_send_dref(send_sock, xp_addr,
                         'sim/cockpit/gyros/phi_vac_ind_deg', roll_deg)

            # instruments — h_ind and vh_ind_fpm are writable; indicated_airspeed is read-only
            # (X-Plane computes IAS from the velocity vectors we already set above)
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/misc/h_ind',
                         alt_m * 3.28084)                           # m MSL → ft
            xp_send_dref(send_sock, xp_addr, 'sim/flightmodel/position/vh_ind_fpm',
                         climb_ms * 196.85)                         # m/s → ft/min


            if args.debug:
                print(f'[XP]  lat={lat:.6f}  lon={lon:.6f}  alt={alt_m:.1f} m  '
                      f'pitch={pitch_deg:+.1f}°  roll={roll_deg:+.1f}°  '
                      f'hdg={hdg_deg:.1f}°')

        # ── 1 s status ────────────────────────────────────────────────────────
        if now - last_srv >= 1.0:
            last_srv = now
            if state_valid:
                hdg = math.degrees(yaw_r) % 360.0
                print(f'[SIM] lat={lat:.6f}  lon={lon:.6f}  '
                      f'alt={alt_m:.1f} m  hdg={hdg:.1f}°')
            else:
                print('[SIM] waiting for SITL state …')

        time.sleep(0.001)


if __name__ == '__main__':
    main()
