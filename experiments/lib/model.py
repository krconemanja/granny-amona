"""Tank model of the boat (course slides) and its digital twin.

State x = [x, y, theta, v]: position in metres (x East, y North), heading in
rad (counter-clockwise from East), speed in m/s.
Input u = [u1, u2]: right and left motor, fraction of full power (-1..1).
"""
import numpy as np

K_SPEED = 0.6  # speed (m/s) = K_SPEED * (u1 + u2)        measure on the water
K_TURN = 1.0   # turn rate (rad/s) = K_TURN * (u1 - u2)   measure on the water
K_POS = 0.3    # twin: trust in a GPS fix (0 = model only, 1 = GPS only)
K_HEAD = 0.5   # twin: trust in the compass


def sawtooth(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def f(x, u, k_speed=K_SPEED, k_turn=K_TURN):
    theta, v = x[2], x[3]
    u1, u2 = u
    return np.array([v * np.cos(theta), v * np.sin(theta),
                     k_turn * (u1 - u2), k_speed * (u1 + u2) - v])


class Twin(object):
    """The model running next to the boat, pulled towards the GPS and the compass."""

    def __init__(self, theta):
        self.x = np.array([0.0, 0.0, theta, 0.0])

    def predict(self, u, dt):
        self.x = self.x + dt * f(self.x, u)

    def correct(self, theta, gps_xy=None):
        self.x[2] += K_HEAD * sawtooth(theta - self.x[2])
        if gps_xy is not None:
            self.x[:2] += K_POS * (np.array(gps_xy) - self.x[:2])
