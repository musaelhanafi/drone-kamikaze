#!/usr/bin/env python3
"""
test_joystick.py — Joystick → MAVLink RC_CHANNELS_OVERRIDE test bridge.

Reads a physical joystick via pygame and forwards axes as MAVLink
RC_CHANNELS_OVERRIDE so ArduPilot SITL (or real hardware) receives stick
input.  QGroundControl can monitor the RC channels in the Radio page while
this script runs.

Axis mapping (default, override with --axes):
  Axis 0 → CH1 aileron   (centred)
  Axis 1 → CH2 elevator  (centred)
  Axis 2 → CH3 throttle  (full-range 1000-2000)
  Axis 3 → CH4 rudder    (centred)
  Axis 4 → CH5 mode      (snapped to ArduPilot flight-mode bands)

CH6 (tracking arm):
  --ch6-button N  hold button N → 1800 µs (armed), released → 1000 µs
  --ch6-axis   N  axis N mapped to full-range 1000-2000 µs

Usage:
  python3 test_joystick.py                          # list joysticks then run
  python3 test_joystick.py --joy-index 1            # use second joystick
  python3 test_joystick.py --sitl tcp:127.0.0.1:5760
  python3 test_joystick.py --axes 0 1 3 2 4 --invert 1
  python3 test_joystick.py --ch6-button 0           # button 0 arms tracking
  python3 test_joystick.py --ch6-axis 5             # axis 5 drives CH6
  python3 test_joystick.py --manual-control         # also send MANUAL_CONTROL
  python3 test_joystick.py --xplane                 # also send aileron/elevator DREFs to X-Plane
"""

import argparse
import socket
import struct
import time

try:
    import pygame
    _PYGAME = True
except ImportError:
    _PYGAME = False

from pymavlink import mavutil

UINT16_MAX     = 65535

_FLTMODE_CENTRES = [1165, 1295, 1425, 1555, 1685, 1815]


def _axis_pwm(v: float, invert: bool = False) -> int:
    if invert:
        v = -v
    return int(max(1000, min(2000, 1500 + v * 500)))


def _thr_pwm(v: float, invert: bool = False) -> int:
    if invert:
        v = -v
    return int(max(1000, min(2000, 1000 + (v + 1.0) * 500)))


def _fltmode_pwm(v: float) -> int:
    raw = int(max(1000, min(2000, 1500 + v * 500)))
    return min(_FLTMODE_CENTRES, key=lambda c: abs(c - raw))


def _pwm_bar(pwm: int, width: int = 20) -> str:
    """Render a PWM value as an ASCII bar centred at 1500."""
    pos = int((pwm - 1000) / 1000.0 * width)
    mid = width // 2
    bar = ['-'] * width
    bar[mid] = '|'
    bar[max(0, min(width - 1, pos))] = '#'
    return ''.join(bar)


def main():
    ap = argparse.ArgumentParser(
        description='Joystick → MAVLink RC_CHANNELS_OVERRIDE test bridge'
    )
    ap.add_argument('--sitl',          default='udp:127.0.0.1:14560',
                    help='MAVLink connection string (default: udp:127.0.0.1:14560)')
    ap.add_argument('--joy-index',     type=int, default=0,
                    help='Joystick device index (default: 0)')
    ap.add_argument('--axes',          type=int, nargs=6,
                    default=[0, 1, 2, 3, 4,5],
                    metavar=('ROLL', 'PITCH', 'THR', 'YAW', 'MODE','CH6'),
                    help='Axis indices for roll pitch thr yaw mode (default: 0 1 2 3 4 5)')
    ap.add_argument('--ch6-button',   type=int, default=None, metavar='BTN',
                    help='Button index: held=1800 µs (tracking armed), released=1000 µs')
    ap.add_argument('--ch6-axis',     type=int, default=None, metavar='AXIS',
                    help='Axis index mapped to CH6 full-range 1000-2000 µs')
    ap.add_argument('--invert',        type=int, nargs='*', default=[],
                    metavar='AXIS',
                    help='Axis indices to invert (e.g. --invert 1 3)')
    ap.add_argument('--thr-invert',    action='store_true',
                    help='Invert throttle axis')
    ap.add_argument('--rate',          type=int, default=50,
                    help='Send rate Hz (default: 50)')
    ap.add_argument('--manual-control', action='store_true',
                    help='Also send MANUAL_CONTROL (QGC joystick protocol)')
    ap.add_argument('--xplane',        action='store_true',
                    help='Also send control surface DREFs to X-Plane via native UDP')
    ap.add_argument('--xplane-host',   default='127.0.0.1')
    ap.add_argument('--xplane-port',   type=int, default=49000,
                    help='X-Plane DREF UDP port (default: 49000)')
    ap.add_argument('--list',          action='store_true',
                    help='List joysticks and exit')
    ap.add_argument('--no-mavlink',    action='store_true',
                    help='Run without MAVLink (joystick display only)')
    args = ap.parse_args()

    if not _PYGAME:
        print('ERROR: pygame not installed — run: pip3 install pygame')
        return

    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()

    print(f'[JOY] {count} joystick(s) detected:')
    for i in range(count):
        j = pygame.joystick.Joystick(i)
        j.init()
        print(f'  [{i}] {j.get_name()}  '
              f'axes={j.get_numaxes()}  buttons={j.get_numbuttons()}')
        j.quit()

    if args.list or count == 0:
        return

    if args.joy_index >= count:
        print(f'ERROR: index {args.joy_index} out of range')
        return

    joy = pygame.joystick.Joystick(args.joy_index)
    joy.init()
    print(f'\n[JOY] Using [{args.joy_index}] {joy.get_name()}')

    roll_ax, pitch_ax, thr_ax, yaw_ax, mode_ax, axes_ch6_ax = args.axes
    invert_set = set(args.invert)

    # ── X-Plane native UDP socket ─────────────────────────────────────────────
    xp_sock = None
    xp_addr = (args.xplane_host, args.xplane_port)
    if args.xplane:
        xp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for dref, val in [
            ('sim/operation/override/override_control_surfaces', 1.0),
            ('sim/operation/override/override_throttles',        1.0),
        ]:
            name_b = dref.encode()
            xp_sock.sendto(
                b'DREF\x00' + struct.pack('<f', val) +
                name_b + b'\x00' * (500 - len(name_b)),
                xp_addr,
            )
        print(f'[XP]  overrides SET (control_surfaces + throttles) → {xp_addr}')

    # ── MAVLink ───────────────────────────────────────────────────────────────
    mav = None
    if not args.no_mavlink:
        print(f'[MAV] Connecting to {args.sitl} …')
        mav = mavutil.mavlink_connection(args.sitl, source_system=255)
        mav.wait_heartbeat()
        print(f'[MAV] Heartbeat  sysid={mav.target_system}  '
              f'compid={mav.target_component}\n')

    ivl = 1.0 / args.rate
    last_send  = 0.0
    last_print = 0.0
    prev_ch    = [None] * 6

    print('Running — Ctrl+C to stop\n')
    print(f'  {"CH1 AIL":<22} {"CH2 ELEV":<22} {"CH3 THR":<22} '
          f'{"CH4 RUD":<22} CH5       CH6')
    print('  ' + '─' * 125)

    try:
        while True:
            now = time.monotonic()
            pygame.event.pump()

            ch1 = _axis_pwm(joy.get_axis(roll_ax),   roll_ax  in invert_set)
            ch2 = _axis_pwm(joy.get_axis(pitch_ax),  pitch_ax in invert_set)
            ch3 = _thr_pwm( joy.get_axis(thr_ax),    thr_ax   in invert_set or args.thr_invert)
            ch4 = _axis_pwm(joy.get_axis(yaw_ax),    yaw_ax   in invert_set)

            n_axes    = joy.get_numaxes()
            n_buttons = joy.get_numbuttons()

            if mode_ax < n_axes:
                ch5 = _fltmode_pwm(joy.get_axis(mode_ax))
            else:
                ch5 = 1165   # mode 0 default

            if args.ch6_button is not None and args.ch6_button < n_buttons:
                ch6 = 1800 if joy.get_button(args.ch6_button) else 1000
            elif args.ch6_axis is not None and args.ch6_axis < n_axes:
                ch6 = _thr_pwm(joy.get_axis(args.ch6_axis))
            elif axes_ch6_ax < n_axes:
                ch6 = _thr_pwm(joy.get_axis(axes_ch6_ax))
            else:
                ch6 = UINT16_MAX   # pass-through (no override)

            cur = [ch1, ch2, ch3, ch4, ch5, ch6]
            changed = cur != prev_ch

            if mav and (changed or now - last_send >= ivl):
                last_send = now
                prev_ch   = cur

                mav.mav.rc_channels_override_send(
                    mav.target_system, mav.target_component,
                    ch1, ch2, ch3, ch4,
                    ch5, ch6, UINT16_MAX, UINT16_MAX,
                )

                if args.manual_control:
                    # MANUAL_CONTROL uses [-1000, 1000] normalised values
                    # QGC monitors this message in the joystick status page
                    x = int((ch2 - 1500) / 500.0 * 1000)   # pitch
                    y = int((ch1 - 1500) / 500.0 * 1000)   # roll
                    z = int((ch3 - 1000) / 1000.0 * 1000)  # throttle 0-1000
                    r = int((ch4 - 1500) / 500.0 * 1000)   # yaw
                    mav.mav.manual_control_send(
                        mav.target_system,
                        x, y, z, r,
                        0,   # buttons
                    )

            # ── X-Plane control surfaces via native UDP ───────────────────────
            if xp_sock and changed:
                def _dref(name, value):
                    nb = name.encode()
                    xp_sock.sendto(
                        b'DREF\x00' + struct.pack('<f', value) +
                        nb + b'\x00' * (500 - len(nb)),
                        xp_addr,
                    )
                ail_deg  = 20.0 * (ch1 - 1500) / 500.0
                elev_deg = 20.0 * (ch2 - 1500) / 500.0
                thr      = max(0.0, min(1.0, (ch3 - 1000) / 1000.0))
                rud_deg  = 20.0 * (ch4 - 1500) / 500.0
                _dref('sim/flightmodel/controls/wing1l_ail1def',  ail_deg)
                _dref('sim/flightmodel/controls/wing1r_ail1def', -ail_deg)
                _dref('sim/flightmodel/controls/hstab1_elv1def',  elev_deg)
                _dref('sim/flightmodel/controls/hstab2_elv1def',  elev_deg)
                _dref('sim/flightmodel/engine/ENGN_thro[0]',      thr)
                _dref('sim/flightmodel/controls/vstab1_rudd1def', rud_deg)

            # ── terminal display ──────────────────────────────────────────────
            if now - last_print >= 0.1:
                last_print = now
                bars = '  '
                for pwm in [ch1, ch2, ch3, ch4]:
                    bars += f'{pwm:4d} {_pwm_bar(pwm)}  '
                mode_idx = _FLTMODE_CENTRES.index(ch5) if ch5 in _FLTMODE_CENTRES else -1
                bars += f'CH5={ch5} (mode {mode_idx})  '
                if ch6 == UINT16_MAX:
                    bars += 'CH6=--'
                else:
                    bars += f'CH6={ch6} {"[ARMED]" if ch6 >= 1700 else ""}'
                print(f'\r{bars}', end='', flush=True)

    except KeyboardInterrupt:
        print('\n[JOY] stopped')
    finally:
        if mav:
            # Release RC override by sending 0 on all channels
            mav.mav.rc_channels_override_send(
                mav.target_system, mav.target_component,
                0, 0, 0, 0, 0, 0, 0, 0,
            )
            print('[MAV] RC override released')


if __name__ == '__main__':
    main()
