#!/usr/bin/env python3
"""
sysid_logger.py — Read-only MAVLink data logger for system identification.

Connects to ArduPilot (SITL or hardware) and logs flight state and servo
outputs to CSV for parameter analysis with sysid_analyze.py.

Usage
─────
  python3 sysid_logger.py
  python3 sysid_logger.py --out flight01.csv --debug
  python3 sysid_logger.py --sitl udp:127.0.0.1:14550 --rate 50
"""

import argparse
import csv
import math
import time
from pathlib import Path

from pymavlink import mavutil

# ── CSV column layout ─────────────────────────────────────────────────────────
CSV_FIELDS = [
    'time_s',
    # Raw servo PWM (µs)
    'servo1_us',
    'servo2_us',
    'servo3_us',
    # Normalised control inputs
    'ail_r',       # CH1 elevon  [-1, 1]
    'elev_r',      # CH2 elevon  [-1, 1]
    'thr',         # CH3 throttle [0, 1]
    # Attitude (from ATTITUDE)
    'roll_deg',
    'pitch_deg',
    'hdg_deg',
    # Angular rates deg/s (from ATTITUDE)
    'p_dps',
    'q_dps',
    'r_dps',
    # Airspeed / altitude (from VFR_HUD)
    'airspeed_ms',
    'climb_ms',
    'alt_m',
    # Position (from GLOBAL_POSITION_INT)
    'lat_deg',
    'lon_deg',
    'rel_alt_m',
]


def main():
    ap = argparse.ArgumentParser(
        description='Read-only MAVLink logger for sysid parameter analysis'
    )
    ap.add_argument('--sitl',  default='udp:127.0.0.1:14560',
                    help='MAVLink connection string (default: udp:127.0.0.1:14560)')
    ap.add_argument('--out',   default='sysid_log.csv',
                    help='Output CSV file (default: sysid_log.csv)')
    ap.add_argument('--rate',  type=int, default=50,
                    help='MAVLink message rate Hz (default: 50)')
    ap.add_argument('--debug', action='store_true')
    args = ap.parse_args()

    # ── MAVLink ───────────────────────────────────────────────────────────────
    print(f'[MAV] connecting to {args.sitl} …')
    mav = mavutil.mavlink_connection(args.sitl, source_system=255)
    mav.wait_heartbeat()
    print(f'[MAV] heartbeat  sysid={mav.target_system}')

    def _req(msg_id, hz):
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0,
        )

    _req(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,            args.rate)
    _req(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,    args.rate)
    _req(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,             args.rate)
    _req(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, args.rate)
    print(f'[MAV] subscribed at {args.rate} Hz')

    # ── state ─────────────────────────────────────────────────────────────────
    servo1 = servo2 = servo3 = 1500
    ail_r = elev_r = thr = 0.0
    roll = pitch = hdg = 0.0
    p = q = r = 0.0
    airspeed = climb = alt = 0.0
    lat = lon = rel_alt = 0.0
    mav_valid = False
    t0 = time.monotonic()

    # ── CSV ───────────────────────────────────────────────────────────────────
    out_path = Path(args.out)
    print(f'[LOG] writing to {out_path}')
    csv_file = out_path.open('w', newline='')
    writer   = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
    writer.writeheader()

    last_log  = time.monotonic()
    log_ivl   = 1.0 / args.rate
    row_count = 0

    print('\n[LOG] Logging — Ctrl+C to stop\n')

    try:
        while True:
            now = time.monotonic()

            # ── drain MAVLink ─────────────────────────────────────────────────
            while True:
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break
                t = msg.get_type()

                if t == 'SERVO_OUTPUT_RAW':
                    servo1 = msg.servo1_raw
                    servo2 = msg.servo2_raw
                    servo3 = msg.servo3_raw
                    ail_r  = (servo1 - 1500) / 500.0
                    elev_r = (servo2 - 1500) / 500.0
                    thr    = max(0.0, min(1.0, (servo3 - 1000) / 1000.0))

                elif t == 'ATTITUDE':
                    roll  = math.degrees(msg.roll)
                    pitch = math.degrees(msg.pitch)
                    hdg   = math.degrees(msg.yaw) % 360.0
                    p     = math.degrees(msg.rollspeed)
                    q     = math.degrees(msg.pitchspeed)
                    r     = math.degrees(msg.yawspeed)
                    mav_valid = True

                elif t == 'VFR_HUD':
                    airspeed = msg.airspeed
                    climb    = msg.climb
                    alt      = msg.alt

                elif t == 'GLOBAL_POSITION_INT':
                    lat     = msg.lat * 1e-7
                    lon     = msg.lon * 1e-7
                    rel_alt = msg.relative_alt * 1e-3

            # ── log row ───────────────────────────────────────────────────────
            if mav_valid and (now - last_log) >= log_ivl:
                last_log = now
                row = {
                    'time_s':    round(now - t0, 4),
                    'servo1_us': servo1,
                    'servo2_us': servo2,
                    'servo3_us': servo3,
                    'ail_r':     round(ail_r,  4),
                    'elev_r':    round(elev_r, 4),
                    'thr':       round(thr,    4),
                    'roll_deg':  round(roll,   3),
                    'pitch_deg': round(pitch,  3),
                    'hdg_deg':   round(hdg,    3),
                    'p_dps':     round(p,      3),
                    'q_dps':     round(q,      3),
                    'r_dps':     round(r,      3),
                    'airspeed_ms': round(airspeed, 3),
                    'climb_ms':    round(climb,    3),
                    'alt_m':       round(alt,      2),
                    'lat_deg':     round(lat,      7),
                    'lon_deg':     round(lon,      7),
                    'rel_alt_m':   round(rel_alt,  2),
                }
                writer.writerow(row)
                row_count += 1

                if args.debug:
                    print(f'  t={row["time_s"]:7.2f}  '
                          f'roll={roll:+7.2f}°  pitch={pitch:+7.2f}°  hdg={hdg:6.1f}°  '
                          f'ias={airspeed:.1f} m/s  alt={alt:.1f} m')
                elif row_count % (args.rate * 5) == 0:
                    print(f'[LOG] {row["time_s"]:6.1f} s  {row_count} rows  '
                          f'roll={roll:+.1f}°  pitch={pitch:+.1f}°  '
                          f'ias={airspeed:.1f} m/s  thr={thr:.2f}')

            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        csv_file.flush()
        csv_file.close()
        print(f'\n[LOG] {row_count} rows → {out_path}')


if __name__ == '__main__':
    main()
