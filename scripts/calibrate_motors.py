#!/usr/bin/env python3
"""Measure the MC 4.0 motor constants WITHOUT encoders.

Stop ROS first (only one program can hold the serial port).

  1) Dead-band: ramps PWM up slowly; note the PWM at which the robot starts rolling.
       python3 scripts/calibrate_motors.py deadband --port /dev/mc4_base
  2) Loaded RPM: drives at full PWM for N seconds; measure the distance with a tape.
       python3 scripts/calibrate_motors.py rpm --port /dev/mc4_base --seconds 3
  3) Straight-line trim: same as rpm, but watch how far the robot drifts sideways.

Results go into firmware/mc4_base_firmware/mc4_base_firmware.ino:
  MIN_PWM_LEFT / MIN_PWM_RIGHT, LOADED_RPM, TRIM_LEFT / TRIM_RIGHT.
"""
import argparse
import math
import time

import serial

WHEEL_DIAMETER_M = 0.065


def open_port(port):
    s = serial.Serial(port, 115200, timeout=0.05)
    time.sleep(2.0)            # ESP32 resets when the port opens
    s.reset_input_buffer()
    return s


def hold(s, left, right, seconds):
    """Send P every 100 ms (the firmware watchdog stops after 500 ms)."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        s.write(f'P {left} {right}\n'.encode())
        time.sleep(0.1)


def deadband(s, side):
    print(f'Ramping {side} side(s) from PWM 0 to 255 on the FLOOR. Ctrl+C when it moves.')
    try:
        for pwm in range(0, 256, 5):
            left = pwm if side in ('both', 'left') else 0
            right = pwm if side in ('both', 'right') else 0
            print(f'  PWM {pwm}', flush=True)
            hold(s, left, right, 1.0)
    except KeyboardInterrupt:
        pass
    finally:
        s.write(b'P 0 0\n')
    print('-> MIN_PWM = the first value where the robot kept moving (add ~5 for margin).')


def rpm(s, seconds, pwm):
    input(f'Robot on the floor with {seconds:.0f}+ m of space. Press Enter to drive at PWM {pwm}...')
    hold(s, pwm, pwm, seconds)
    s.write(b'P 0 0\n')
    d = float(input('Distance travelled in metres (tape measure): '))
    wheel_revs = d / (math.pi * WHEEL_DIAMETER_M)
    loaded_rpm = wheel_revs / seconds * 60.0
    print(f'-> LOADED_RPM = {loaded_rpm:.0f}   (max speed {d / seconds:.2f} m/s)')
    print('   If the robot curved, lower TRIM_ on the faster side (e.g. 0.95) and repeat.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=['deadband', 'rpm'])
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--pwm', type=int, default=255)
    ap.add_argument('--side', choices=['both', 'left', 'right'], default='both')
    a = ap.parse_args()

    s = open_port(a.port)
    try:
        if a.mode == 'deadband':
            deadband(s, a.side)
        else:
            rpm(s, a.seconds, a.pwm)
    finally:
        s.write(b'P 0 0\n')
        s.close()


if __name__ == '__main__':
    main()
