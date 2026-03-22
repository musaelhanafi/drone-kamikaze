#!/usr/bin/env python3
"""
sitl_xpc_xplane.py — ArduPilot SITL → X-Plane visualizer via XPlaneConnect plugin.

Same mechanism as sitl_xplane.py (override_planepath=1, physics bypassed) but uses
the XPlaneConnect (xpc) plugin instead of raw native UDP DREFs.  Benefits over the
native-UDP version:
  • sendPOSI sets lat/lon/alt directly — no local-coordinate transform required
  • GPS waypoint DREFs (sim/cockpit/gps/wp_*) ARE writable via XPC → GNS430 DIS/BRG
  • sendDREFs batches multiple writes in one UDP packet

Prerequisites
─────────────
  pip3 install xpc
  Install XPlaneConnect plugin in X-Plane (copy plugin folder to X-Plane/Resources/plugins/)
  Plugin download: https://github.com/nasa/XPlaneConnect/releases

Architecture
────────────
  ArduPilot SITL ──MAVLink──▶  sitl_xpc_xplane.py ──XPC──▶  X-Plane 11
  SIMSTATE + GLOBAL_POSITION_INT  →  sendPOSI (lat/lon/alt/pitch/roll/hdg)
  SERVO_OUTPUT_RAW                →  sendDREFs (controls + throttle)
  VFR_HUD                         →  sendDREFs (instruments)
  NAV_CONTROLLER_OUTPUT           →  sendDREFs (GPS wp_lat/wp_lon → GNS430 DIS/BRG)

X-Plane setup
─────────────
  Install XPlaneConnect plugin (plugin listens on port 49009 by default).
  No other settings needed.

Usage
─────
  # Terminal 1 — SITL
  python3 Tools/autotest/sim_vehicle.py --vehicle Plane --location WICC

  # Terminal 2 — mirror to X-Plane
  python3 sitl_xpc_xplane.py
  python3 sitl_xpc_xplane.py --xpc-port 49009 --debug
"""

import argparse
import math
import time

from pymavlink import mavutil
import xpc

try:
    import pygame
    _PYGAME = True
except ImportError:
    _PYGAME = False

R_EARTH   = 6_371_000.0   # metres
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


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='Mirror ArduPilot SITL state to X-Plane via XPlaneConnect plugin'
    )
    ap.add_argument('--sitl',         default='udp:127.0.0.1:14560',
                    help='MAVLink connection to SITL (default: udp:127.0.0.1:14560)')
    ap.add_argument('--xpc-host',     default='127.0.0.1')
    ap.add_argument('--xpc-port',     type=int, default=49009,
                    help='XPlaneConnect plugin port (default: 49009)')
    ap.add_argument('--update-rate',  type=int, default=50,
                    help='Position update rate Hz (default: 50)')
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

    upd_ivl = 1.0 / args.update_rate

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

    # ── XPlaneConnect ─────────────────────────────────────────────────────────
    print(f'[XPC] Connecting to XPlaneConnect at {args.xpc_host}:{args.xpc_port} …')
    client = xpc.XPlaneConnect(xpHost=args.xpc_host, xpPort=args.xpc_port)

    try:
        posi = client.getPOSI()
    except Exception as e:
        print(f'[XPC] ERROR: getPOSI failed — {e}')
        print('      Is XPlaneConnect plugin installed and X-Plane unpaused?')
        return

    home_lat, home_lon, home_alt_m = posi[0], posi[1], posi[2]
    print(f'[XPC] Connected  lat={home_lat:.6f}  lon={home_lon:.6f}  alt={home_alt_m:.1f} m')

    # Bypass X-Plane physics so the aircraft follows our DREF position.
    client.sendDREF('sim/operation/override/override_planepath[0]', 1.0)
    client.sendDREF('sim/flightmodel/controls/parkbrake', 0.0)
    # Zero winds so IAS is computed from our velocity vectors alone.
    for layer in range(3):
        client.sendDREF(f'sim/weather/wind_speed_kt[{layer}]', 0.0)
    print('[XPC] override_planepath=1  parkbrake=0  wind=0')

    # ── MAVLink ───────────────────────────────────────────────────────────────
    print(f'[MAV] Connecting to {args.sitl} …')
    mav = mavutil.mavlink_connection(args.sitl, source_system=255)
    mav.wait_heartbeat()
    print(f'[MAV] Heartbeat  sysid={mav.target_system}  compid={mav.target_component}')

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

    _set_param('GPS1_TYPE',      1)
    _set_param('EK3_SRC1_YAW',   1)
    _set_param('COMPASS_USE',    1)
    _set_param('ARSPD_TYPE',     100)
    _set_param('ARMING_REQUIRE', 0)
    _set_param('SIM_SPEEDUP',    1)
    print('[MAV] params set')

    _req_interval(164, args.update_rate)  # SIMSTATE
    _req_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, args.update_rate)
    _req_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,    args.update_rate)
    _req_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,             args.update_rate)
    _req_interval(mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, 10)  # GPS waypoint

    # ── set FC home to X-Plane's current aircraft position ────────────────────
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0, 0, 0, 0, 0,
        home_lat, home_lon, home_alt_m,
    )
    print(f'[MAV] Home set  lat={home_lat:.6f}  lon={home_lon:.6f}  alt={home_alt_m:.1f} m')

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

    vx_ned = vy_ned = vz_ned = 0.0   # NED m/s
    p_rad  = q_rad  = r_rad  = 0.0   # body angular rates rad/s
    aileron_r   = 0.0
    elevator_r  = 0.0
    rudder_r    = 0.0
    throttle    = 0.0
    climb_ms    = 0.0
    wp_dist_m          = 0.0
    target_bearing_deg = 0.0

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

            elif t == 'GLOBAL_POSITION_INT':
                alt_m  = msg.alt * 1e-3
                vx_ned = msg.vx * 0.01
                vy_ned = msg.vy * 0.01
                vz_ned = msg.vz * 0.01
                alt_valid = True

            elif t == 'SERVO_OUTPUT_RAW':
                aileron_r  = (msg.servo1_raw - 1500) / 500.0
                elevator_r = (msg.servo2_raw - 1500) / 500.0
                throttle   = (msg.servo3_raw - 1000) / 1000.0
                rudder_r   = (msg.servo4_raw - 1500) / 500.0

            elif t == 'VFR_HUD':
                climb_ms = msg.climb

            elif t == 'NAV_CONTROLLER_OUTPUT':
                wp_dist_m          = msg.wp_dist
                target_bearing_deg = msg.target_bearing

            if att_valid and alt_valid and not state_valid:
                state_valid = True
                print(f'[SIM] First state  lat={lat:.6f}  lon={lon:.6f}  '
                      f'alt={alt_m:.1f} m  hdg={math.degrees(yaw_r) % 360:.1f}°')

        # ── refresh override_planepath every 2 s ──────────────────────────────
        # List syntax [1.0] avoids the [0] array-index ambiguity in XPC getDREF.
        if now - last_override >= 2.0:
            last_override = now
            try:
                client.sendDREF('sim/operation/override/override_planepath', [1.0])
            except Exception:
                pass

        # ── write state → X-Plane via XPC ─────────────────────────────────────
        if state_valid and (now - last_upd) >= upd_ivl:
            last_upd = now

            pitch_deg = math.degrees(pitch_r)
            roll_deg  = math.degrees(roll_r)
            hdg_deg   = math.degrees(yaw_r) % 360.0

            try:
                # position + attitude — sendPOSI sets lat/lon/alt directly,
                # no local-coordinate transform needed
                client.sendPOSI([lat, lon, alt_m, pitch_deg, roll_deg, hdg_deg, -1])

                # velocity + angular rates + controls + instruments in one batch
                client.sendDREFs(
                    [
                        # velocity — NED → X-Plane local (+x east, +y up, +z south)
                        'sim/flightmodel/position/local_vx',
                        'sim/flightmodel/position/local_vy',
                        'sim/flightmodel/position/local_vz',
                        # angular rates (body frame, rad/s)
                        'sim/flightmodel/position/Prad',
                        'sim/flightmodel/position/Qrad',
                        'sim/flightmodel/position/Rrad',
                        # control surfaces — joystick ratios are writable
                        'sim/joystick/yoke_roll_ratio',
                        'sim/joystick/yoke_pitch_ratio',
                        'sim/joystick/yoke_heading_ratio',
                        # throttle
                        'sim/cockpit2/engine/actuators/throttle_ratio[0]',
                        # vacuum ADI gauges — read pitch/roll directly from these
                        'sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot',
                        'sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot',
                        'sim/cockpit/gyros/the_vac_ind_deg',
                        'sim/cockpit/gyros/phi_vac_ind_deg',
                        # altimeter + VSI
                        'sim/flightmodel/misc/h_ind',           # ft MSL
                        'sim/flightmodel/position/vh_ind_fpm',  # ft/min
                    ],
                    [
                        vy_ned, -vz_ned, -vx_ned,
                        p_rad, q_rad, r_rad,
                        aileron_r, -elevator_r, rudder_r,
                        max(0.0, min(1.0, throttle)),
                        pitch_deg, roll_deg,
                        pitch_deg, roll_deg,
                        alt_m * 3.28084,
                        climb_ms * 196.85,
                    ],
                )
            except Exception as e:
                if args.debug:
                    print(f'[XPC] send error: {e}')

            # GNS430 DIS/BRG — isolated try/except: GPS DREFs may not be readable
            # via XPC getDREF but sendDREFs still writes them; failure must not
            # interrupt the main position/attitude update above
            if wp_dist_m > 0.0:
                try:
                    bear_r = math.radians(target_bearing_deg)
                    lat_r  = math.radians(lat)
                    wp_lat = lat + math.degrees(wp_dist_m * math.cos(bear_r) / R_EARTH)
                    wp_lon = lon + math.degrees(wp_dist_m * math.sin(bear_r)
                                                / (R_EARTH * math.cos(lat_r)))
                    client.sendDREFs(
                        ['sim/cockpit/gps/wp_lat', 'sim/cockpit/gps/wp_lon'],
                        [wp_lat, wp_lon],
                    )
                except Exception as e:
                    if args.debug:
                        print(f'[XPC] GPS send error: {e}')

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
