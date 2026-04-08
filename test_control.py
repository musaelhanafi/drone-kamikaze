#!/usr/bin/env python3
"""
test_control.py — Read SERVO_OUTPUT_RAW from MAVLink and send elevon + throttle
                  DREFs to X-Plane via native UDP.

ArduPilot SITL / hardware  ──SERVO_OUTPUT_RAW──►  test_control.py  ──DREF──►  X-Plane

CH1 → wing1l_ail1def  (left elevon,  degrees)
CH2 → wing1r_ail1def  (right elevon, degrees)
CH3 → ENGN_thro_use   (throttle,     0-1)

Usage:
  python3 test_control.py
  python3 test_control.py --pixhawk /dev/tty.usbmodem1101
  python3 test_control.py --pixhawk tcp:127.0.0.1:5760 --debug
"""

import argparse
import socket
import struct
import time

from pymavlink import mavutil

ELEVON_MAX_DEG = 20.0   # matches Plane Maker control surface deflection limit


# ── X-Plane native UDP helpers ────────────────────────────────────────────────

def xp_dref(sock, addr, name: str, value: float):
    name_b = name.encode()
    sock.sendto(
        b'DREF\x00' + struct.pack('<f', value) +
        name_b + b'\x00' * (500 - len(name_b)),
        addr,
    )


def xp_dsel(sock, addr, rows):
    padded = (list(rows) + [0] * 8)[:8]
    sock.sendto(b'DSEL\x00' + struct.pack('<8I', *padded), addr)


# ── PWM converters ────────────────────────────────────────────────────────────

def pwm_to_deg(pwm: int) -> float:
    """PWM [1000-2000] → surface deflection [-ELEVON_MAX_DEG, +ELEVON_MAX_DEG]."""
    return ELEVON_MAX_DEG * (pwm - 1500) / 500.0


def pwm_to_thr(pwm: int) -> float:
    """PWM [1000-2000] → throttle ratio [0.0, 1.0]."""
    return max(0.0, min(1.0, (pwm - 1000) / 1000.0))


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    global ELEVON_MAX_DEG
    ap = argparse.ArgumentParser(
        description='SERVO_OUTPUT_RAW → X-Plane elevon + throttle DREFs'
    )
    ap.add_argument('--pixhawk',     default='udpin:0.0.0.0:14560',
                    help='MAVLink connection (default: udpin:0.0.0.0:14560)')
    ap.add_argument('--xplane-host', default='127.0.0.1')
    ap.add_argument('--xplane-port', type=int, default=49000,
                    help='X-Plane DREF port (default: 49000)')
    ap.add_argument('--max-deg',     type=float, default=ELEVON_MAX_DEG,
                    help=f'Elevon max deflection degrees (default: {ELEVON_MAX_DEG})')
    ap.add_argument('--debug',       action='store_true')
    args = ap.parse_args()

    ELEVON_MAX_DEG = args.max_deg

    xp_addr = (args.xplane_host, args.xplane_port)

    # ── X-Plane socket ────────────────────────────────────────────────────────
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    xp_dref(sock, xp_addr, 'sim/operation/override/override_joystick',         1.0)
    xp_dref(sock, xp_addr, 'sim/operation/override/override_throttles',        1.0)
    xp_dref(sock, xp_addr, 'sim/operation/override/override_control_surfaces', 1.0)
    print(f'[XP]  overrides SET → {xp_addr}')

    # ── MAVLink ───────────────────────────────────────────────────────────────
    print(f'[MAV] connecting to {args.pixhawk} …')
    mav = mavutil.mavlink_connection(args.pixhawk, source_system=255)
    mav.wait_heartbeat()
    print(f'[MAV] heartbeat  sysid={mav.target_system}')

    # request SERVO_OUTPUT_RAW at 50 Hz
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        20000,   # 50 Hz
        0, 0, 0, 0, 0,
    )
    print('[MAV] SERVO_OUTPUT_RAW requested at 50 Hz')

    # ── state ────────────────────────────────────────────────────────────────
    last_dsel  = time.monotonic()
    last_print = time.monotonic()
    srv_count  = 0
    ch1 = ch2 = 1500
    ch3 = 1000

    print('\nRunning — Ctrl+C to stop\n')
    print(f'  {"ail1_def (deg)":>16}  {"ail2_def (deg)":>16}  {"thr":>6}  '
          f'{"CH1 pwm":>8}  {"CH2 pwm":>8}  {"CH3 pwm":>8}')
    print('  ' + '─' * 80)

    try:
        while True:
            now = time.monotonic()

            # re-send overrides every 5 s (X-Plane resets on scene reload)
            if now - last_dsel >= 5.0:
                xp_dref(sock, xp_addr,
                         'sim/operation/override/override_joystick',         1.0)
                xp_dref(sock, xp_addr,
                         'sim/operation/override/override_throttles',        1.0)
                xp_dref(sock, xp_addr,
                         'sim/operation/override/override_control_surfaces', 1.0)
                last_dsel = now

            # drain MAVLink
            while True:
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() != 'SERVO_OUTPUT_RAW':
                    continue

                ch1 = msg.servo1_raw
                ch2 = msg.servo2_raw
                ch3 = msg.servo3_raw

                ail1 = pwm_to_deg(ch1)
                ail2 = pwm_to_deg(ch2)
                thr  = pwm_to_thr(ch3)

                # X-Plane sign convention: positive = trailing edge DOWN.
                # ArduPlane elevon convention: positive PWM = trailing edge UP.
                # Swap channels and negate to correct pitch & roll simultaneously:
                #   wing1l <- -ail2,  wing1r <- -ail1
                xp_dref(sock, xp_addr,
                         'sim/flightmodel/controls/wing1l_ail1def', ail1)
                xp_dref(sock, xp_addr,
                         'sim/flightmodel/controls/wing1r_ail1def', ail2)
                xp_dref(sock, xp_addr,
                         'sim/flightmodel/engine/ENGN_thro_use[0]', thr)

                srv_count += 1

                if args.debug:
                    print(f'  ail1={ail1:+7.2f}°  ail2={ail2:+7.2f}°  '
                          f'thr={thr:.3f}  '
                          f'CH1={ch1}  CH2={ch2}  CH3={ch3}')

            # status line every 1 s
            if not args.debug and now - last_print >= 1.0:
                last_print = now
                ail1 = pwm_to_deg(ch1)
                ail2 = pwm_to_deg(ch2)
                thr  = pwm_to_thr(ch3)
                print(f'\r  {ail1:+16.2f}  {ail2:+16.2f}  {thr:>6.3f}  '
                      f'{ch1:>8}  {ch2:>8}  {ch3:>8}  '
                      f'({srv_count} msg/s)    ',
                      end='', flush=True)
                srv_count = 0

            time.sleep(0.002)

    except KeyboardInterrupt:
        print('\n[done]')
    finally:
        # zero surfaces on exit
        xp_dref(sock, xp_addr, 'sim/flightmodel/controls/wing1l_ail1def', 0.0)
        xp_dref(sock, xp_addr, 'sim/flightmodel/controls/wing1r_ail1def', 0.0)
        xp_dref(sock, xp_addr, 'sim/flightmodel/engine/ENGN_thro_use[0]', 0.0)
        sock.close()


if __name__ == '__main__':
    main()
