#!/usr/bin/env python3
"""
joy_test.py — print joystick axes and buttons in real time.

Usage:
    python3 joy_test.py            # use joystick index 0
    python3 joy_test.py --index 1  # use joystick index 1
"""

import argparse
import sys
import time

try:
    import pygame
except ImportError:
    sys.exit("pygame not installed.  Run: pip3 install pygame")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--index', type=int, default=0,
                    help='Joystick device index (default: 0)')
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()

    count = pygame.joystick.get_count()
    if count == 0:
        sys.exit("[JOY] No joystick detected")

    print(f"Found {count} joystick(s):")
    for i in range(count):
        j = pygame.joystick.Joystick(i)
        print(f"  [{i}] {j.get_name()}")

    joy = pygame.joystick.Joystick(args.index)
    joy.init()
    print(f"\nUsing [{args.index}] {joy.get_name()}")
    print(f"  axes={joy.get_numaxes()}  buttons={joy.get_numbuttons()}  hats={joy.get_numhats()}")
    print("\nMove axes / press buttons (Ctrl+C to quit)\n")

    try:
        while True:
            pygame.event.pump()

            axes    = [round(joy.get_axis(i), 3) for i in range(joy.get_numaxes())]
            buttons = [joy.get_button(i)          for i in range(joy.get_numbuttons())]
            hats    = [joy.get_hat(i)             for i in range(joy.get_numhats())]

            axis_str   = "  ".join(f"A{i}={v:+.3f}" for i, v in enumerate(axes))
            btn_str    = "  ".join(f"B{i}={v}"       for i, v in enumerate(buttons) if v)
            hat_str    = "  ".join(f"H{i}={v}"       for i, v in enumerate(hats)    if any(v))

            line = axis_str
            if btn_str: line += "   " + btn_str
            if hat_str: line += "   " + hat_str

            print(f"\r{line:<120}", end="", flush=True)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nDone.")


if __name__ == "__main__":
    main()
