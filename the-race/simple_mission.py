"""Simple test mission: go to ONE goal, then come back home.

No twin, no route: GPS -> local frame -> control() -> motors.
Home is where the boat is when R is pressed the FIRST time (so it is in the
water, not on the dock). The local frame is centred on home: x = East, y = North (m).

Driven from the laptop with client.py (same keys as mission.py):
    R         start / resume: AUTO (to the goal) or BACK_HOME, whichever was running
    P         MANUAL: drive with W A S D. If the laptop is silent for LINK_TIMEOUT,
              it resumes AUTO / BACK_HOME (STOP if R was never pressed)
    SPACE, Q  STOP: motors off
AUTO -> BACK_HOME at the goal, BACK_HOME -> STOP at home. R again does another
round trip (same home).

Usage:
    python3 simple_mission.py 48.19950 -3.01450               goal in lat lon (degrees)
    python3 simple_mission.py 20 30 --local                   goal 20 m East, 30 m North of home
    python3 simple_mission.py 20 30 --local --sim             on the laptop, simulated boat
    python3 simple_mission.py 20 30 --local --sim --autostart AUTO at the first fix, quit at home

Ctrl+C stops the motors.
"""
import argparse
import os
import socket
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lib'))
from boat_io import RealBoat, SimBoat
from geo import to_local
from model import sawtooth

DT = 0.1           # s, loop period
MAX_PWM = 255
RADIUS = 4.0       # m, goal reached (GPS noise is ~2-3 m)
GPS_TIMEOUT = 10.0 # s without a fix -> motors off
SIM_HOME = (48.19915, -3.01488)
PORT = 5000
LINK_TIMEOUT = 2.0   # s without a packet in MANUAL -> resume AUTO / BACK_HOME
STATUS_PERIOD = 0.5  # s between two status packets to the laptop

STOP, AUTO, MANUAL, BACK_HOME = 'STOP', 'AUTO', 'MANUAL', 'BACK_HOME'


class Link(object):
    """UDP socket to client.py. Never blocks."""

    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)
        self.addr = None     # laptop address, learnt from its packets
        self.last_rx = None  # time of the last packet

    def receive(self):
        """All the packets waiting, each as a list of words."""
        msgs = []
        while True:
            try:
                data, self.addr = self.sock.recvfrom(1024)
            except socket.error:  # nothing more waiting
                return msgs
            self.last_rx = time.time()
            words = data.decode('ascii', 'ignore').split()
            if words:
                msgs.append(words)

    def send(self, text):
        if self.addr:
            try:
                self.sock.sendto(text.encode('ascii'), self.addr)
            except socket.error:
                pass

    def lost(self):
        return self.last_rx is None or time.time() - self.last_rx > LINK_TIMEOUT


def clamp(v):
    return max(-MAX_PWM, min(MAX_PWM, int(float(v))))


def control (x, goal, speed):
    xr,yr,theta = x.flatten()

    goal_angle = np.arctan2(goal[1]-x[1], goal[0]-x[0])

    e_angle = sawtooth(goal_angle - theta)

    distance = (goal[0] - x[0])**2 + (goal[1] - x[1])**2
    if distance < 0.15:
        distance = 0
    u1 = (speed + e_angle) * min(distance, 1)
    u2 = (speed - e_angle) * min(distance, 1)
    print(u1, " ", u2, " ", distance)
    return np.array([[u1],[u2]])


def main():
    ap = argparse.ArgumentParser(description='Go to one goal with control(), then back home')
    ap.add_argument('a', type=float, help='goal latitude (or x in m with --local)')
    ap.add_argument('b', type=float, help='goal longitude (or y in m with --local)')
    ap.add_argument('--local', action='store_true', help='goal in metres from home')
    ap.add_argument('--speed', type=float, default=0.6, help='speed passed to control() - must be between 0 and 1')
    ap.add_argument('--sim', action='store_true', help='simulated boat, no hardware')
    ap.add_argument('--port', type=int, default=PORT)
    ap.add_argument('--autostart', action='store_true',
                    help='AUTO at the first GPS fix (home = there), quit when back home')
    args = ap.parse_args()

    boat = SimBoat(*SIM_HOME) if args.sim else RealBoat()
    link = Link(args.port)
    print('Listening on UDP port %d%s' % (args.port, ' (SIMULATION)' if args.sim else ''))
    latlon = None             # last GPS fix (lat, lon)
    home, goal = None, None   # home (lat, lon), goal (x, y) in m: set at the first R
    xy = None                 # position (x, y) in m from home
    gps_t = time.time()
    mode = STOP
    leg = AUTO                # the automatic mode R / link loss resumes: AUTO or BACK_HOME
    start = args.autostart    # set home and go at the next fix
    manual = (0, 0)           # last WASD command (left, right)
    status_t = 0.0

    def set_mode(new, why):
        if new != mode:
            print('%s -> %s (%s)' % (mode, new, why))
        return new

    try:
        while True:
            now = time.time()
            boat.step(DT)

            # commands from the laptop
            for words in link.receive():
                if words[0] == 'M' and len(words) == 3:
                    try:
                        manual = (clamp(words[1]), clamp(words[2]))
                    except ValueError:
                        pass
                elif words[0] == 'MODE' and len(words) == 2:
                    if words[1] == AUTO:
                        if home is None:
                            start = True  # home is set below, from the GPS
                        else:
                            mode = set_mode(leg, 'laptop')
                    elif words[1] in (STOP, MANUAL):
                        if words[1] == MANUAL and mode != MANUAL:
                            manual = (0, 0)
                        start = False
                        mode = set_mode(words[1], 'laptop')
            if mode == MANUAL and link.lost():
                mode = set_mode(leg if home else STOP, 'link lost')

            # GPS -> local frame (origin = home)
            fix = boat.read_gps()
            if fix:
                latlon, gps_t = fix, now
            if start and latlon and now - gps_t < GPS_TIMEOUT:
                start = False
                home = latlon
                goal = (args.a, args.b) if args.local else to_local(args.a, args.b, *home)
                print('Home %.6f %.6f  goal x=%.1f y=%.1f m' % (home + goal))
                mode = set_mode(AUTO, 'start')
            if home:
                xy = to_local(latlon[0], latlon[1], *home)
            theta = boat.read_heading()

            target = goal if leg == AUTO else (0.0, 0.0)
            dist = np.hypot(target[0] - xy[0], target[1] - xy[1]) if xy else None
            left = right = 0
            if mode == MANUAL:
                left, right = manual
            elif mode == STOP:
                pass
            elif now - gps_t > GPS_TIMEOUT:
                print('no GPS for %.0f s, motors off' % GPS_TIMEOUT)
            elif dist < RADIUS and mode == AUTO:
                leg = BACK_HOME
                mode = set_mode(BACK_HOME, 'goal reached')
            elif dist < RADIUS and mode == BACK_HOME:
                leg = AUTO  # R again = another round trip
                mode = set_mode(STOP, 'home reached')
                if args.autostart:
                    break
            else:
                x = np.array([[xy[0]], [xy[1]], [theta]])
                u1, u2 = control(x, target, args.speed).flatten()  # u1 right, u2 left
                right = int(np.clip(u1 * MAX_PWM, -MAX_PWM, MAX_PWM))
                left = int(np.clip(u2 * MAX_PWM, -MAX_PWM, MAX_PWM))
            boat.set_motors(left, right)

            # status to the laptop
            if now - status_t >= STATUS_PERIOD:
                status_t = now
                if latlon is None:
                    link.send('%s waiting for GPS' % mode)
                elif home is None:
                    link.send('%s GPS ok, R sets home' % mode)
                else:
                    link.send('%s %s %.0f m  gps %.0f s  L=%d R=%d'
                              % (mode, 'goal' if leg == AUTO else 'home', dist,
                                 now - gps_t, left, right))

            time.sleep(max(0.0, DT - (time.time() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        boat.set_motors(0, 0)

if __name__ == '__main__':
    main()
