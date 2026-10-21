import math
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from controls import mix, shape, accel_roll_deg, accel_pitch_deg, RollFilter, TiltFilter, compute_command, ControlState
from link import parse_line, parse_host
from map_widget import lonlat_to_px, px_to_lonlat, haversine, bearing, nice_scale
from server_telemetry import nmea_to_decimal
from telemetry import Tracker


def test_mix_matches_client():
    assert mix(0, 0, 200) == (0, 0)
    assert mix(100, 0, 200) == (100, 100)
    assert mix(100, 50, 200) == (150, 50)
    l, r = mix(200, 100, 200)            # saturates -> ratio kept
    assert l == 200 and r == 67


def test_shape_deadzone_and_range():
    assert shape(0.05) == 0.0
    assert shape(1.0) == 1.0 and shape(-1.0) == -1.0
    assert 0 < shape(0.5) < 0.5          # expo softens the middle
    assert shape(2.0) == 1.0


def test_accel_roll_sign():
    g = 9.81
    assert abs(accel_roll_deg(0, g, 0)) < 1e-9
    # right side down: X axis points down-right -> ax negative
    assert accel_roll_deg(-g * math.sin(math.radians(30)), g * math.cos(math.radians(30)), 0) > 29
    # pitch does not change roll
    assert abs(accel_roll_deg(0, g * 0.7, g * 0.7)) < 1e-9


def _accel(roll_deg=0.0, pitch_deg=0.0, g=9.81):
    """Gravity reaction for a Deck rolled right by roll and tilted forward by pitch."""
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    return (-g * math.sin(r) * math.cos(p), g * math.cos(r) * math.cos(p), g * math.sin(p))


def test_accel_pitch_sign():
    assert abs(accel_pitch_deg(*_accel(0, 0))) < 1e-9
    assert abs(accel_pitch_deg(*_accel(0, 25)) - 25) < 1e-6      # forward = positive
    assert abs(accel_pitch_deg(*_accel(0, -25)) + 25) < 1e-6
    assert abs(accel_roll_deg(*_accel(20, 40)) - 20) < 0.5       # roll unaffected by pitch


def test_tilt_filter_converges_and_recenters():
    f = TiltFilter()
    for _ in range(400):
        f.update(_accel(20, 50), (0, 0, 0), 0.02)              # held reclined 50 deg
    roll, pitch = f.value
    assert abs(roll - 20) < 0.5 and abs(pitch - 50) < 0.5
    f.recenter()                                               # 50 deg becomes neutral
    for _ in range(400):
        f.update(_accel(20, 70), (0, 0, 0), 0.02)              # tilt 20 deg forward
    roll, pitch = f.value
    assert abs(roll) < 0.5 and abs(pitch - 20) < 0.5
    assert RollFilter is TiltFilter


def test_compute_command_modes():
    cfg = {}
    st = ControlState(ly=1.0, lx=0.0)
    _, _, l, r = compute_command(st, "Joystick", 200, cfg)
    assert (l, r) == (200, 200)
    st = ControlState(ly=1.0, ry=-1.0)
    _, _, l, r = compute_command(st, "Tank", 100, cfg)
    assert (l, r) == (100, -100)


def test_gyro_tilt_throttle_and_deadman():
    cfg = {}
    # full forward tilt + right roll, paddle held -> moves, turning right
    st = ControlState(pitch=30.0, roll=20.0, has_gyro=True, deadman=True)
    thr, turn, l, r = compute_command(st, "Gyro", 200, cfg)
    assert thr == 1.0 and turn > 0 and l > r > 0
    # same pose, paddle released -> nothing
    st.deadman = False
    assert compute_command(st, "Gyro", 200, cfg) == (0.0, 0.0, 0, 0)
    # inside the 5 deg dead zone -> no throttle
    st = ControlState(pitch=4.0, has_gyro=True, deadman=True)
    assert compute_command(st, "Gyro", 200, cfg)[0] == 0.0
    # tilt back -> reverse
    st = ControlState(pitch=-30.0, has_gyro=True, deadman=True)
    _, _, l, r = compute_command(st, "Gyro", 200, cfg)
    assert l == r == -200
    # triggers are ignored in gyro mode
    st = ControlState(rt=1.0, has_gyro=True, deadman=True)
    assert compute_command(st, "Gyro", 200, cfg)[2:] == (0, 0)
    # Steam Input fallback (no raw gyro): right stick Y throttle, X turn
    st = ControlState(ry=1.0, has_gyro=False, deadman=True)
    assert compute_command(st, "Gyro", 200, cfg)[2:] == (200, 200)


def test_parse_line():
    assert parse_line('T {"lat": 1.5}') == ("T", {"lat": 1.5})
    assert parse_line("O 1234") == ("O", 1234.0)
    assert parse_line("T notjson") == (None, None)
    assert parse_line("garbage") == (None, None)
    assert parse_host("ue32@172.20.25.205") == ("172.20.25.205", None)
    assert parse_host("10.0.0.1:6000") == ("10.0.0.1", 6000)


def test_geo():
    x, y = lonlat_to_px(-4.4741, 48.4183, 17)
    lon, lat = px_to_lonlat(x, y, 17)
    assert abs(lon + 4.4741) < 1e-9 and abs(lat - 48.4183) < 1e-9
    d = haversine(48.0, -4.0, 48.001, -4.0)
    assert abs(d - 111.2) < 0.5
    assert abs(bearing(48, -4, 48.001, -4)) < 1e-6
    assert abs(bearing(48, -4, 48, -3.999) - 90) < 0.1
    assert nice_scale(37) == 20 and nice_scale(120) == 100


def test_nmea():
    assert abs(nmea_to_decimal(4825.098, "N") - 48.4183) < 1e-4
    assert nmea_to_decimal(428.446, "W") < 0


def test_tracker_route_speed():
    tr = Tracker()
    t = 0.0
    for i in range(20):
        tr.ingest({"lat": 48.0 + i * 1e-5, "lon": -4.0, "fix": 1}, now=t)
        t += 1.0
    assert len(tr.route) == 20
    assert abs(tr.distance - 19 * 1.112) < 0.1
    assert abs(tr.speed - 1.112) < 0.05
    assert abs(tr.cog) < 1 or abs(tr.cog - 360) < 1
    tr.ingest({"lat": 0, "lon": 0, "fix": 0}, now=t)   # no fix ignored
    assert len(tr.route) == 20


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_sim_server_end_to_end():
    port = _free_port()
    srv = subprocess.Popen([sys.executable, os.path.join(os.path.dirname(HERE), "server_telemetry.py"),
                            "--sim", "--host", "127.0.0.1", "--port", str(port)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        time.sleep(0.8)
        s = socket.create_connection(("127.0.0.1", port), timeout=3)
        f = s.makefile("rb")
        s.sendall(b"P 42\n")
        t_end = time.time() + 3
        got_t = got_o = None
        while time.time() < t_end and not (got_t and got_o):
            s.sendall(b"M 150 100\n")
            kind, payload = parse_line(f.readline().decode())
            if kind == "T" and payload.get("cmd") == [150, 100]:
                got_t = payload
            elif kind == "O":
                got_o = payload
        assert got_o == 42.0
        assert got_t and "lat" in got_t and got_t["cmd"] == [150, 100]
        s.close()
    finally:
        srv.terminate()
        srv.wait()


def test_legacy_server_accepts_gui_protocol():
    """The original server.py must still work with 'P <t>' pings."""
    legacy = os.environ.get("LEGACY_SERVER")
    if not legacy:
        return
    port = _free_port()
    srv = subprocess.Popen([sys.executable, legacy, "--dry-run", "--host", "127.0.0.1", "--port", str(port)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        time.sleep(0.8)
        s = socket.create_connection(("127.0.0.1", port), timeout=3)
        s.sendall(b"P 123\nM 80 -80\nS\n")
        time.sleep(0.3)
        s.close()
        time.sleep(0.3)
    finally:
        srv.terminate()
        out = srv.communicate()[0].decode()
    assert "Unknown command" not in out
    assert "L=  80 R= -80" in out
