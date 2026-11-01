"""Mission waypoints and GPS coordinates.

Positions are in metres in a flat frame centred on home (where the boat
starts): x = East, y = North.

Missions are drawn in Google Earth with "Add Path" and saved as .kml.
Check one with:  python3 lib/geo.py missions/lake.kml
"""
import math
import sys
import xml.etree.ElementTree as ET

EARTH_R = 6371000.0  # m
SKIP_RADIUS = 10.0   # m, path points this close to home (the dock) are dropped
KML = '{http://www.opengis.net/kml/2.2}'


def nmea_to_decimal(value, direction):
    """GPS DDmm.mmmm + 'N'/'S'/'E'/'W' -> decimal degrees."""
    deg = int(value / 100)
    dec = deg + (value - deg * 100) / 60.0
    return -dec if direction in ('S', 'W') else dec


def to_local(lat, lon, lat0, lon0):
    """(lat, lon) in degrees -> (x, y) in metres from (lat0, lon0)."""
    x = EARTH_R * math.cos(math.radians(lat0)) * math.radians(lon - lon0)
    y = EARTH_R * math.radians(lat - lat0)
    return x, y


def to_latlon(x, y, lat0, lon0):
    """(x, y) in metres from (lat0, lon0) -> (lat, lon) in degrees."""
    lat = lat0 + math.degrees(y / EARTH_R)
    lon = lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon


def load_kml(path):
    """(lat, lon) points of the path in a Google Earth .kml file."""
    coords = ET.parse(path).find('.//%sLineString/%scoordinates' % (KML, KML)).text
    return [(float(c.split(',')[1]), float(c.split(',')[0])) for c in coords.split()]


def build_route(path, home_lat, home_lon):
    """Waypoints in metres: the path without the points near home, then home."""
    route = [to_local(lat, lon, home_lat, home_lon) for lat, lon in load_kml(path)]
    route = [p for p in route if math.hypot(p[0], p[1]) > SKIP_RADIUS]
    return route + [(0.0, 0.0)]


if __name__ == '__main__':
    lat0, lon0 = load_kml(sys.argv[1])[0]  # home = start of the path
    for x, y in build_route(sys.argv[1], lat0, lon0):
        print('x = %6.1f m   y = %6.1f m' % (x, y))
