"""Tank model of the boat (course slides), used by the simulated boat.

State x = [x, y, theta, v]: position in metres (x East, y North), heading in
rad (counter-clockwise from East), speed in m/s.
Input u = [u1, u2]: right and left motor, fraction of full power (-1..1).
"""
import numpy as np

K_SPEED = 0.5  # speed (m/s) = K_SPEED * (u1 + u2)
K_TURN = 1.3   # turn rate (rad/s) = K_TURN * (u1 - u2)


def sawtooth(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def f(x, u):
    theta, v = x[2], x[3]
    u1, u2 = u
    return np.array([v * np.cos(theta), v * np.sin(theta),
                     K_TURN * (u1 - u2), K_SPEED * (u1 + u2) - v])
