"""Heading from the IMU, using the compass calibration from calibration.py.

Boat frame: x forward (bow), y to starboard (right), z down (through the deck).
Heading psi: 0 = North, +pi/2 = East (clockwise seen from above), in [-pi, pi].

Usage in other scripts:
    from imu import heading
    psi = heading()
Run directly to print the raw data and the heading:
    python3 imu.py
"""
import sys
import os
import time

import numpy as np

# access to the drivers
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

import imu9_driver_v2 as imudrv
from calibration import load_calibration, calibrate

_imu = None
_A, _b = None, None


def _init():
    """Create the IMU object and load the calibration on first use."""
    global _imu, _A, _b
    if _imu is None:
        _imu = imudrv.Imu9IO()
        _A, _b = load_calibration()
    return _imu


def tilt(accel):
    """Roll and pitch (rad) from the accelerometer.

    At rest a level boat should read (0, 0, -g): check the sign and the axes of
    read_accel_raw() on the boat before trusting this (see __main__ below).
    """
    fx, fy, fz = np.asarray(accel, dtype=float)
    phi = np.arctan2(-fy, -fz)                # roll
    theta = np.arctan2(fx, np.hypot(fy, fz))  # pitch
    return phi, theta


def heading(tilt_compensation=True):
    """Heading of the boat (rad), 0 = North, +pi/2 = East."""
    imu = _init()
    yx, yy, yz = calibrate(imu.read_mag_raw(), _A, _b)

    if tilt_compensation:
        phi, theta = tilt(imu.read_accel_raw())
        # rotate the field back to the horizontal plane
        mx = yx*np.cos(theta) + (yy*np.sin(phi) + yz*np.cos(phi))*np.sin(theta)
        my = yy*np.cos(phi) - yz*np.sin(phi)
        return np.arctan2(-my, mx)

    return np.arctan2(-yy, yx)


if __name__ == '__main__':
    imu = _init()
    while True:
        # raw data (raw = not calibrated !)
        mag = imu.read_mag_raw()
        accel = imu.read_accel_raw()
        gyro = imu.read_gyro_raw()
        phi, theta = tilt(accel)
        print('mag=%s accel=%s gyro=%s | roll=%6.1f pitch=%6.1f | heading=%6.1f tilt-comp=%6.1f deg'
              % (mag, accel, gyro, np.degrees(phi), np.degrees(theta),
                 np.degrees(heading()), np.degrees(heading(tilt_compensation=True))))
        time.sleep(0.1)
