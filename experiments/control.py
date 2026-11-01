"""Proportional heading control towards a waypoint (course slides)."""
import numpy as np

from model import sawtooth

SPEED = 0.6   # cruise command, fraction of full power
K = 0.5       # heading gain
RADIUS = 5.0  # m, a waypoint is reached closer than this (GPS noise ~2-3 m)


def distance(x, goal):
    return np.hypot(goal[0] - x[0], goal[1] - x[1])


def control(x, goal):
    """Motor commands (u1 right, u2 left) to steer the boat in state x to goal."""
    e = sawtooth(np.arctan2(goal[1] - x[1], goal[0] - x[0]) - x[2])
    u1 = np.clip(SPEED + K * e, -1, 1)
    u2 = np.clip(SPEED - K * e, -1, 1)
    return u1, u2
