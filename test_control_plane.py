#!/usr/bin/env python3
"""
test_control_plane.py — Read SERVO_OUTPUT_RAW from MAVLink and send flying-wing
                        joystick DREFs to X-Plane via native UDP.

ArduPilot SITL / hardware  ──SERVO_OUTPUT_RAW──►  test_control_plane.py  ──DREF──►  X-Plane

Flying-wing elevon demix (same as xplane_elevon.json / SIM_XPlane.cpp):
  CH1 = right elevon PWM (pre-mixed by ArduPilot)
  CH2 = left  elevon PWM (pre-mixed by ArduPilot)
  CH3 = throttle

  roll  = (CH1 - CH2) / 1000          → yoke_roll_ratio    (-1 … +1)
  pitch = -(CH1 + CH2 - 3000) / 1000  → yoke_pitch_ratio   (-1 … +1, negated for X-Plane)
  thr   = (CH3 - 1000) / 1000         → ENGN_thro_use[0]   ( 0 … +1)

Usage:
  python3 test_control_plane.py
  python3 test_control_plane.py --pixhawk /dev/tty.usbmodem1101
  python3 test_control_plane.py --xplane-host 192.168.1.10
  python3 test_control_plane.py --debug
"""

import argparse
import socket
import struct
import time

from pymavlink import mavutil

# ── X-Plane native UDP helpers ────────────────────────────────────────────────

def xp_dref(sock, addr, name: str, value: float):
    name_b = name.encode()
    sock.sendto(
        b'DREF\x00' + struct.pack('<f', value) +
        name_b + b'\x00' * (500 - len(name_b)),
        addr,
    )


def send_overrides(sock, addr):
    xp_dref(sock, addr, 'sim/operation/override/override_joystick',  1.0)
    xp_dref(sock, addr, 'sim/operation/override/override_throttles', 1.0)


# ── PWM helpers ───────────────────────────────────────────────────────────────

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def pwm_to_thr(pwm: int) -> float:
    return clamp((pwm - 1000) / 1000.0, 0.0, 1.0)


def valid_pwm(v: int, neutral: int = 1500) -> int:
    return v if 900 <= v <= 2100 else neutral


# ── Flying-wing elevon demix ──────────────────────────────────────────────────
# Mirrors xplane_elevon.json / SIM_XPlane.cpp ELEVON_ROLL / ELEVON_PITCH_NEG

def elevon_demix(ch1: int, ch2: int):
    """Return (roll_ratio, pitch_ratio) in -1…+1 from raw PWM.
    Exact match to SIM_XPlane.cpp ELEVON_ROLL / ELEVON_PITCH_NEG (range=1):
      ELEVON_ROLL:      v =  (ch1 - ch2) / 700
      ELEVON_PITCH_NEG: v = -(ch1 + ch2 - 2700) / 1000 + 0.5
    """
    roll  =  clamp((ch1 - ch2)        / 700.0,  -1.0, 1.0)
    pitch = -clamp((ch1 + ch2 - 2700) / 1000.0, -1.0, 1.0) + 0.5
    pitch =  clamp(pitch, -1.0, 1.0)
    return roll, pitch


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='SERVO_OUTPUT_RAW → X-Plane control surfaces via native UDP'
    )
    ap.add_argument('--pixhawk',     default='/dev/tty.usbmodem14101',
                    help='MAVLink connection (default: /dev/tty.usbmodem14101)')
    ap.add_argument('--xplane-host', default='127.0.0.1')
    ap.add_argument('--xplane-port', type=int, default=49000,
                    help='X-Plane DREF UDP port (default: 49000)')
    ap.add_argument('--debug',       action='store_true')
    args = ap.parse_args()

    xp_addr  = (args.xplane_host, args.xplane_port)

    # ── X-Plane socket ────────────────────────────────────────────────────────
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    send_overrides(sock, xp_addr)
    print(f'[XP]  overrides SET (override_joystick + override_throttles) → {xp_addr}')

    # ── MAVLink ───────────────────────────────────────────────────────────────
    print(f'[MAV] connecting to {args.pixhawk} …')
    mav = mavutil.mavlink_connection(args.pixhawk, source_system=255)
    mav.wait_heartbeat()
    print(f'[MAV] heartbeat  sysid={mav.target_system}')

    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        20000,   # 50 Hz
        0, 0, 0, 0, 0,
    )
    print('[MAV] SERVO_OUTPUT_RAW requested at 50 Hz\n')

    # ── state ─────────────────────────────────────────────────────────────────
    last_override = time.monotonic()
    last_print    = time.monotonic()
    srv_count     = 0
    ch1 = ch2 = 1500
    ch3 = 1000

    print(f'  {"ROLL":>8}  {"PITCH":>8}  {"THR":>6}  {"CH1":>6}  {"CH2":>6}  {"CH3":>6}')
    print('  ' + '─' * 56)

    try:
        while True:
            now = time.monotonic()

            # re-send overrides every 5 s (X-Plane resets on scene reload)
            if now - last_override >= 5.0:
                send_overrides(sock, xp_addr)
                last_override = now
                print('\n[XP]  overrides re-sent')

            # drain MAVLink
            while True:
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() != 'SERVO_OUTPUT_RAW':
                    continue

                ch1 = valid_pwm(msg.servo1_raw)
                ch2 = valid_pwm(msg.servo2_raw)
                ch3 = valid_pwm(msg.servo3_raw, neutral=1000)

                roll, pitch = elevon_demix(ch1, ch2)
                thr         = pwm_to_thr(ch3)

                # same DREFs as xplane_elevon.json SITL map (override_joystick=1)
                xp_dref(sock, xp_addr, 'sim/joystick/yoke_roll_ratio',  roll)
                xp_dref(sock, xp_addr, 'sim/joystick/yoke_pitch_ratio', pitch)
                xp_dref(sock, xp_addr, 'sim/flightmodel/engine/ENGN_thro_use[0]', thr)

                srv_count += 1

                if args.debug:
                    print(f'  roll={roll:+.3f}  pitch={pitch:+.3f}  thr={thr:.3f}'
                          f'  CH1={ch1}  CH2={ch2}  CH3={ch3}')

            # status line every 1 s
            if not args.debug and now - last_print >= 1.0:
                last_print = now
                roll, pitch = elevon_demix(ch1, ch2)
                thr = pwm_to_thr(ch3)
                print(f'\r  {roll:+8.3f}  {pitch:+8.3f}  {thr:>6.3f}'
                      f'  {ch1:>6}  {ch2:>6}  {ch3:>6}'
                      f'  ({srv_count} msg/s)    ',
                      end='', flush=True)
                srv_count = 0

            time.sleep(0.002)

    except KeyboardInterrupt:
        print('\n[done]')
    finally:
        xp_dref(sock, xp_addr, 'sim/joystick/yoke_roll_ratio',  0.0)
        xp_dref(sock, xp_addr, 'sim/joystick/yoke_pitch_ratio', 0.0)
        xp_dref(sock, xp_addr, 'sim/flightmodel/engine/ENGN_thro_use[0]', 0.0)
        sock.close()


if __name__ == '__main__':
    main()
