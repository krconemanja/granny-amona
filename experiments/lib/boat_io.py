"""The boat for mission.py: real hardware or a simulation, with the same functions:
    read_gps()        (lat, lon) when a new fix arrived, else None
    read_heading()    theta in rad (counter-clockwise from East)
    set_motors(l, r)  motor commands in -255..255
    step(dt)          move the simulated boat (nothing on the real boat)
"""
import os
import random
import sys

import numpy as np

from geo import nmea_to_decimal, to_latlon
from model import f, sawtooth

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'drivers-ddboat-v2'))


class RealBoat(object):
    def __init__(self):
        import arduino_driver_v2
        import gps_driver_v2
        from imu import heading
        self.heading = heading
        self.ard = arduino_driver_v2.ArduinoIO()  # its Ctrl+C handler stops the motors
        self.gps = gps_driver_v2.GpsIO()
        self.gps.set_filter_speed("0")

    def read_gps(self):
        fix = None
        while True:  # read all waiting NMEA lines, so the fix is not late
            ok, val = self.gps.read_gll_non_blocking()
            if ok and val[0] != 0.0:
                fix = (nmea_to_decimal(val[0], val[1]), nmea_to_decimal(val[2], val[3]))
            if not self.gps.ser.in_waiting:
                return fix

    def read_heading(self):
        return sawtooth(np.pi / 2 - self.heading())  # compass: 0 = North, clockwise

    def set_motors(self, left, right):
        self.ard.send_arduino_cmd_motor(int(left), int(right))

    def step(self, dt):
        pass


class SimBoat(object):
    """The model plus what the twin does not know: a water current, a slower
    and faster turning boat, GPS noise (1 fix/s) and compass noise."""

    def __init__(self, home_lat, home_lon):
        self.home = (home_lat, home_lon)
        self.x = np.array([0.0, 0.0, 1.0, 0.0])
        self.u = (0.0, 0.0)
        self.t = 0.0
        self.next_fix = 0.0

    def read_gps(self):
        if self.t < self.next_fix:
            return None
        self.next_fix += 1.0
        return to_latlon(self.x[0] + random.gauss(0, 1.5), self.x[1] + random.gauss(0, 1.5),
                         self.home[0], self.home[1])

    def read_heading(self):
        return sawtooth(self.x[2] + random.gauss(0, 0.05))

    def set_motors(self, left, right):
        self.u = (right / 255.0, left / 255.0)

    def step(self, dt):
        self.x = self.x + dt * f(self.x, self.u, k_speed=0.5, k_turn=1.3)
        self.x[:2] += dt * np.array([0.15, 0.05])  # water current, m/s
        self.t += dt
