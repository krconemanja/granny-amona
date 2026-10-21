"""Compass calibration for the DDBoat (4-pose method).

The raw magnetometer reading x_mag is modelled as
    x_mag = A . y - b
where y is the true magnetic field expressed in the boat frame, so the
calibrated field is
    y = A^-1 . (x_mag + b)

Boat frame: x forward (bow), y to starboard (right), z down (through the deck).
Field in the boat frame for each pose (beta = 1 so the calibrated field
has unit norm, I = magnetic inclination, field points down in France):
    North : bow North, deck up               y_N = ( cos I,  0,      sin I)
    South : bow South, upside down (deck down)  y_S = -y_N
    West  : bow West, deck up                y_W = ( 0,      cos I,  sin I)
    Up    : bow pointing to the sky, deck facing North
                                             y_U = (-sin I,  0,     -cos I)

From these:
    b = -(x_N + x_S) / 2
    A = [x_N + b, x_W + b, x_U + b] . [y_N, y_W, y_U]^-1

Usage on the boat:
    python3 calibration.py            # run the calibration, save to calib.npz
Usage in other scripts:
    from calibration import load_calibration, calibrate
    A, b = load_calibration()
    x, y, z = calibrate(imu.read_mag_raw(), A, b)
"""
import sys
import os
import time

import numpy as np

# access to the drivers
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calib.npz')
INCLINATION_DEG = 64.0  # magnetic inclination in Brest
N_SAMPLES = 50          # measures averaged per pose
SAMPLE_PERIOD = 0.1     # s (mag sensor runs at 10 Hz)

POSES = [
    ('North', 'bow pointing North, deck up'),
    ('South', 'bow pointing South, boat UPSIDE DOWN'),
    ('West', 'bow pointing West, deck up'),
    ('Up', 'bow pointing to the sky, deck facing North'),
]


def beep(n):
    """Beep code: n beeps (terminal bell) for pose number n."""
    for _ in range(n):
        sys.stdout.write('\a')
        sys.stdout.flush()
        time.sleep(0.3)


def measure(imu, n=N_SAMPLES, period=SAMPLE_PERIOD):
    """Average n raw magnetometer measures to reduce the noise."""
    samples = []
    for _ in range(n):
        samples.append(imu.read_mag_raw())
        time.sleep(period)
    samples = np.array(samples, dtype=float)
    return samples.mean(axis=0), samples.std(axis=0)


def reference_fields(inclination_deg=INCLINATION_DEG):
    """True field (unit norm) in the boat frame for the North, West and Up poses."""
    I = np.radians(inclination_deg)
    y_N = np.array([np.cos(I), 0.0, np.sin(I)])
    y_W = np.array([0.0, np.cos(I), np.sin(I)])
    y_U = np.array([-np.sin(I), 0.0, -np.cos(I)])
    return y_N, y_W, y_U


def compute_calibration(x_N, x_S, x_W, x_U, inclination_deg=INCLINATION_DEG):
    """Compute A and b from the mean raw measures of the 4 poses."""
    x_N, x_S, x_W, x_U = (np.asarray(v, dtype=float) for v in (x_N, x_S, x_W, x_U))
    b = -(x_N + x_S) / 2.0
    X = np.column_stack((x_N + b, x_W + b, x_U + b))
    Y = np.column_stack(reference_fields(inclination_deg))
    A = X @ np.linalg.inv(Y)
    return A, b


def calibrate(mag_raw, A, b):
    """[x_calib, y_calib, z_calib] = A^-1 . (mag_raw + b)"""
    return np.linalg.solve(A, np.asarray(mag_raw, dtype=float) + b)


def save_calibration(A, b, path=CALIB_FILE):
    np.savez(path, A=A, b=b)


def load_calibration(path=CALIB_FILE):
    data = np.load(path)
    return data['A'], data['b']


def run_calibration():
    import imu9_driver_v2 as imudrv
    imu = imudrv.Imu9IO()

    means = {}
    for k, (name, description) in enumerate(POSES, start=1):
        beep(k)
        input('[%d/4] %s pose: %s. Press Enter when ready...' % (k, name, description))
        print('  measuring, keep the boat still...')
        mean, std = measure(imu)
        means[name] = mean
        print('  mean = %s  std = %s' % (np.round(mean, 1), np.round(std, 1)))
    beep(5)

    A, b = compute_calibration(means['North'], means['South'], means['West'], means['Up'])
    save_calibration(A, b)

    np.set_printoptions(precision=4, suppress=True)
    print('\nA =\n%s\nb = %s' % (A, b))
    print('Saved to %s' % CALIB_FILE)

    # sanity check: each calibrated pose should have norm ~1
    for name, mean in means.items():
        y = calibrate(mean, A, b)
        print('  %-5s calibrated = %s  |y| = %.3f' % (name, y, np.linalg.norm(y)))


if __name__ == '__main__':
    run_calibration()
