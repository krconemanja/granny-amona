"""Plot a mission log, save the GPS track for Google Earth, and measure the
model constants from MANUAL runs (README, "Tuning on the water").

    python3 plot_log.py mission_log_123.csv   -> plot + mission_log_123.kml
"""
import csv
import os
import sys
from itertools import groupby

import matplotlib.pyplot as plt
import numpy as np

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lib'))
from geo import build_route, to_latlon

name = sys.argv[1]
with open(name) as fp:
    lat0, lon0, mission = fp.readline().split(None, 2)
    rows = list(csv.DictReader(fp))
lat0, lon0, mission = float(lat0), float(lon0), mission.strip()
gps = np.array([(float(r['gps_x']), float(r['gps_y'])) for r in rows if r['gps_x']])
twin = np.array([(float(r['twin_x']), float(r['twin_y'])) for r in rows])

# GPS track for Google Earth
coords = ' '.join('%.7f,%.7f' % to_latlon(x, y, lat0, lon0)[::-1] for x, y in gps)
with open(name.replace('.csv', '.kml'), 'w') as fp:
    fp.write('<kml xmlns="http://www.opengis.net/kml/2.2"><Placemark><name>%s</name>'
             '<LineString><coordinates>%s</coordinates></LineString></Placemark></kml>\n'
             % (name, coords))

# model constants from MANUAL runs at a constant command
for (mode, left, right), run in groupby(rows, lambda r: (r['mode'], int(r['left']), int(r['right']))):
    run = list(run)[30:]  # skip the first 3 s, the boat accelerates
    if mode != 'MANUAL' or len(run) < 50:
        continue
    if left == right != 0:  # straight: speed from a line through the GPS fixes
        t, x, y = np.array([(float(r['t']), float(r['gps_x']), float(r['gps_y']))
                            for r in run if r['gps_x']]).T
        v = np.hypot(np.polyfit(t, x, 1)[0], np.polyfit(t, y, 1)[0])
        print('straight at %d: v = %.2f m/s -> K_SPEED = %.2f' % (left, v, v / (2 * left / 255.0)))
    elif left == -right:  # spin in place: turn rate from the compass
        t = [float(r['t']) for r in run]
        w = np.polyfit(t, np.unwrap([float(r['theta_meas']) for r in run]), 1)[0]
        print('spin at %d: w = %.2f rad/s -> K_TURN = %.2f' % (right, w, w / (2 * right / 255.0)))

if os.path.exists(mission):
    route = np.array([(0.0, 0.0)] + build_route(mission, lat0, lon0))
    plt.plot(route[:, 0], route[:, 1], 'o--', color='gray', label='route')
plt.plot(gps[:, 0], gps[:, 1], '.', color='red', label='GPS')
plt.plot(twin[:, 0], twin[:, 1], color='blue', label='twin')
plt.plot(0, 0, 'r*', markersize=15, label='home')
plt.axis('equal')
plt.xlabel('x East (m)')
plt.ylabel('y North (m)')
plt.legend()
plt.show()
