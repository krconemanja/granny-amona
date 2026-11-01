"""GPS coordinates and the local frame.

Positions are in metres in a flat frame centred on home (where the boat
starts): x = East, y = North.
"""
import math

EARTH_R = 6371000.0  # m


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
