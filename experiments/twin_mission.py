"""DDBoat mission -- run this ON THE BOAT (or on the laptop with --sim).

Follows the waypoints of a Google Earth mission and comes back to where it
started. Driven from the laptop with client.py over UDP.

Modes:
    STOP    motors off. At startup, after SPACE / Q, when home is reached,
            and when the GPS is lost for GPS_TIMEOUT in AUTO. Only R leaves it.
    AUTO    heading control to the waypoints (R starts and resumes)
    MANUAL  WASD from the laptop (P). If the laptop is silent for
            LINK_TIMEOUT, the boat goes back to AUTO.

Every loop (DT = 0.1 s):
    1. read the laptop commands   2. read GPS + compass
    3. twin: predict, then correct with the sensors
    4. motors from the mode       5. status to the laptop + one log line

Usage:
    python3 twin_mission.py missions/lake.kml                  on the boat
    python3 twin_mission.py missions/sim_square.kml --sim      on the laptop
    python3 twin_mission.py missions/sim_square.kml --sim --autostart --speed 20
"""
import argparse
import os
import socket
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lib'))
from boat_io import MAX_PWM, RealBoat, SimBoat
from control import control, distance, next_waypoint
from geo import build_route, to_local
from model import sawtooth, Twin

PORT = 5000
DT = 0.1             # s, control period (the compass runs at 10 Hz)
LINK_TIMEOUT = 2.0   # s without a packet in MANUAL -> back to AUTO
GPS_TIMEOUT = 10.0   # s without a GPS fix in AUTO -> STOP
HOME_FIXES = 10      # GPS fixes averaged to get the home position
STATUS_PERIOD = 0.5  # s between two status packets to the laptop
SIM_HOME = (48.19915, -3.01488)

STOP, AUTO, MANUAL = 'STOP', 'AUTO', 'MANUAL'


def clamp(v):
    return max(-MAX_PWM, min(MAX_PWM, int(float(v))))


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


def main():
    ap = argparse.ArgumentParser(description='DDBoat waypoint mission')
    ap.add_argument('mission', help='Google Earth .kml / .kmz file')
    ap.add_argument('--port', type=int, default=PORT)
    ap.add_argument('--k-pos', type=float, default=0.3, help='twin GPS gain (0..1)')
    ap.add_argument('--k-head', type=float, default=0.5, help='twin compass gain (0..1)')
    ap.add_argument('--sim', action='store_true', help='simulated boat, no hardware')
    ap.add_argument('--speed', type=float, default=1.0, help='simulation speed-up')
    ap.add_argument('--autostart', action='store_true',
                    help='start AUTO as soon as home is known, quit when back home')
    args = ap.parse_args()

    boat = SimBoat(*SIM_HOME) if args.sim else RealBoat()
    link = Link(args.port)
    print('Listening on UDP port %d%s' % (args.port, ' (SIMULATION)' if args.sim else ''))

    mode = STOP
    manual = (0, 0)              # last WASD command (left, right)
    fixes, home, route = [], None, None
    twin, i = None, 0            # twin and index of the current waypoint
    u = (0.0, 0.0)               # last command (u1 right, u2 left)
    t, gps_t, status_t = 0.0, 0.0, 0.0
    log = None
    last = time.time()

    def set_mode(new, why):
        if new != mode:
            print('%7.1f s  %s -> %s (%s)' % (t, mode, new, why))
        return new

    try:
        while True:
            now = time.time()
            dt = DT if args.sim else now - last
            last = now
            boat.step(dt)
            t += dt

            # 1. commands from the laptop
            for words in link.receive():
                if words[0] == 'M' and len(words) == 3:
                    try:
                        manual = (clamp(words[1]), clamp(words[2]))
                    except ValueError:
                        pass
                elif words[0] == 'MODE' and len(words) == 2:
                    if words[1] == AUTO and route is None:
                        continue  # home not known yet
                    if words[1] == MANUAL and mode != MANUAL:
                        manual = (0, 0)
                    if words[1] in (STOP, AUTO, MANUAL):
                        mode = set_mode(words[1], 'laptop')
            if mode == MANUAL and link.lost():
                mode = set_mode(AUTO if route else STOP, 'link lost')

            # 2. sensors
            fix = boat.read_gps()
            theta = sawtooth(boat.read_heading())
            gps_xy = None
            if fix and home is None:
                fixes.append(fix)
                if len(fixes) >= HOME_FIXES:
                    home = tuple(np.mean(fixes, axis=0))
                    route = build_route(args.mission, home[0], home[1])
                    twin = Twin(theta, args.k_pos, args.k_head)
                    print('Home %.6f %.6f, route: %s' % (home[0], home[1], ', '.join(
                        '(%.0f, %.0f)' % p for p in route)))
                    log = open('mission_log_%d.csv' % time.time(), 'w')
                    log.write('# home %.7f %.7f mission %s\n' % (home[0], home[1], args.mission))
                    log.write('t,mode,wp,gps_x,gps_y,theta_meas,twin_x,twin_y,twin_theta,'
                              'twin_v,left,right,true_x,true_y\n')
                    gps_t = t
                    if args.autostart:
                        mode = set_mode(AUTO, 'autostart')
            elif fix:
                gps_xy = to_local(fix[0], fix[1], home[0], home[1])
                gps_t = t

            # 3. twin: model prediction, then correction by the sensors
            if twin:
                twin.predict(u, dt)
                twin.correct(gps_xy, theta)

            if mode == AUTO and t - gps_t > GPS_TIMEOUT:
                mode = set_mode(STOP, 'no GPS for %.0f s' % GPS_TIMEOUT)

            # 4. motors
            left = right = 0
            if mode == AUTO:
                i = next_waypoint(twin.x, route, i)
                if i == len(route):
                    mode = set_mode(STOP, 'home reached')
                    if args.autostart:
                        break
                else:
                    u1, u2 = control(twin.x, route[i], last=(i == len(route) - 1))
                    left, right = u2 * MAX_PWM, u1 * MAX_PWM
            elif mode == MANUAL:
                left, right = manual
            boat.set_motors(left, right)
            u = (right / float(MAX_PWM), left / float(MAX_PWM))

            # 5. status to the laptop, log
            if t - status_t >= STATUS_PERIOD:
                status_t = t
                if route is None:
                    link.send('%s waiting for GPS %d/%d' % (mode, len(fixes), HOME_FIXES))
                else:
                    wp = min(i, len(route) - 1)
                    link.send('%s wp %d/%d %.0f m  gps %.0f s  L=%d R=%d'
                              % (mode, i + 1, len(route), distance(twin.x, route[wp]),
                                 t - gps_t, left, right))
            if log:
                txy = boat.true_xy()
                log.write('%.2f,%s,%d,%s,%s,%.3f,%.2f,%.2f,%.3f,%.2f,%d,%d,%s,%s\n' % (
                    t, mode, i,
                    '%.2f' % gps_xy[0] if gps_xy else '', '%.2f' % gps_xy[1] if gps_xy else '',
                    theta, twin.x[0], twin.x[1], twin.x[2], twin.x[3], left, right,
                    '%.2f' % txy[0] if txy else '', '%.2f' % txy[1] if txy else ''))

            if args.sim:
                time.sleep(DT / args.speed)
            else:
                time.sleep(max(0.0, DT - (time.time() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        boat.set_motors(0, 0)
        if log:
            log.close()
            print('Log saved to %s' % log.name)


if __name__ == '__main__':
    main()