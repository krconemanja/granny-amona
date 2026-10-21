"""DDBoat motor server WITH telemetry -- run this ON THE BOAT.

Drop-in replacement for server.py: same commands, same watchdog, plus a
telemetry stream back to the pilot on the same TCP socket.

Pilot -> boat (newline-terminated ASCII):
    M <left> <right>    set motor speeds, each in [-MAX_SPEED, MAX_SPEED]
    S                   stop both motors
    P [t]               ping / heartbeat; if t is given the boat answers "O t"

Boat -> pilot:
    T {json}            telemetry at TELEMETRY_HZ; every key optional:
                        t, lat, lon, fix, hdg, roll, pitch, gz, cmd, act, batt, rc, status
    O <t>               pong

Usage (copy this file next to server.py, in the folder that holds drivers-ddboat-v2/):
    python3 server_telemetry.py [--port 5000]               real boat
    python3 server_telemetry.py --sim                       simulated boat, no hardware
    python3 server_telemetry.py --mag-calib calib.json      calibrated heading
"""
from __future__ import print_function
import argparse
import json
import math
import os
import random
import socket
import sys
import threading
import time

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drivers-ddboat-v2'))

MAX_SPEED = 255
WATCHDOG_TIMEOUT = 1.0
TELEMETRY_HZ = 5.0
EARTH_R = 6371000.0


def clamp(v):
    return max(-MAX_SPEED, min(MAX_SPEED, int(v)))


def nmea_to_decimal(value, direction):
    """DDmm.mmmm (or DDDmm.mmmm) + N/S/E/W -> signed decimal degrees."""
    deg = int(value / 100)
    dec = deg + (value - deg * 100) / 60.0
    return -dec if direction in ('S', 'W') else dec


# ---------------------------------------------------------------------------
# boats: each provides send_arduino_cmd_motor(l, r) and snapshot() -> dict
# ---------------------------------------------------------------------------
class SimBoat(object):
    """Unicycle model driven by the motor commands. No hardware needed."""

    def __init__(self, lat=48.4183, lon=-4.4741):
        self.lat0, self.lon0 = lat, lon
        self.x = self.y = 0.0          # metres east / north
        self.hdg = 30.0                # deg, 0 = north, clockwise
        self.v = 0.0
        self.cmd = (0, 0)
        self.lock = threading.Lock()
        self.t0 = time.time()
        t = threading.Thread(target=self._run)
        t.daemon = True
        t.start()

    def send_arduino_cmd_motor(self, l, r):
        with self.lock:
            self.cmd = (clamp(l), clamp(r))

    def _run(self):
        last = time.time()
        while True:
            time.sleep(0.05)
            now = time.time()
            dt, last = now - last, now
            with self.lock:
                l, r = self.cmd
                target_v = 2.0 * (l + r) / (2.0 * MAX_SPEED)      # 2 m/s at full
                self.v += (target_v - self.v) * min(1.0, dt * 1.5)  # inertia
                omega = 60.0 * (l - r) / (2.0 * MAX_SPEED)          # deg/s
                self.gz = omega
                self.hdg = (self.hdg + omega * dt) % 360
                self.x += self.v * math.sin(math.radians(self.hdg)) * dt
                self.y += self.v * math.cos(math.radians(self.hdg)) * dt

    def snapshot(self):
        with self.lock:
            lat = self.lat0 + math.degrees(self.y / EARTH_R) + random.gauss(0, 2e-7)
            lon = self.lon0 + math.degrees(self.x / (EARTH_R * math.cos(math.radians(self.lat0)))) \
                + random.gauss(0, 2e-7)
            l, r = self.cmd
            return {"t": round(time.time() - self.t0, 2), "lat": round(lat, 7), "lon": round(lon, 7),
                    "fix": 1, "hdg": round(self.hdg, 1), "roll": round(random.gauss(0, 0.8), 1),
                    "pitch": round(random.gauss(0, 0.5), 1), "gz": round(getattr(self, "gz", 0.0), 1),
                    "act": [int(l * 0.97), int(r * 0.97)], "batt": round(12.4 - (time.time() - self.t0) / 3600, 2),
                    "rc": 0, "sim": 1}


class RealBoat(object):
    """Real DDBoat: Arduino for motors, GPS + IMU read in a background thread."""

    def __init__(self, mag_calib=None, heading_offset=0.0, heading_sign=1.0, arduino_status=False):
        import arduino_driver_v2 as arddrv
        self.ard = arddrv.ArduinoIO()
        self.ser_lock = threading.Lock()   # Arduino serial line is shared
        self.state = {}
        self.state_lock = threading.Lock()
        self.t0 = time.time()
        self.calib = None
        if mag_calib:
            with open(mag_calib) as f:
                self.calib = json.load(f)  # {"invA": 3x3, "xbar": [3]}
        self.heading_offset = heading_offset
        self.heading_sign = heading_sign
        self.arduino_status = arduino_status
        self.gps = self.imu = None
        try:
            import gps_driver_v2 as gpddrv
            self.gps = gpddrv.GpsIO()
            self.gps.set_filter_speed("0")
        except Exception as e:
            print("GPS unavailable: %s" % e)
        try:
            import imu9_driver_v2 as imudrv
            self.imu = imudrv.Imu9IO()
        except Exception as e:
            print("IMU unavailable: %s" % e)
        t = threading.Thread(target=self._run)
        t.daemon = True
        t.start()

    def send_arduino_cmd_motor(self, l, r):
        with self.ser_lock:
            self.ard.send_arduino_cmd_motor(l, r)

    def _heading(self, mag):
        if self.calib:
            A, xb = self.calib["invA"], self.calib["xbar"]
            d = [mag[i] - xb[i] for i in range(3)]
            y = [sum(A[r][c] * d[c] for c in range(3)) for r in range(3)]
            cap = -math.atan2(y[1], y[0])     # same formula as control_pot.py
        else:
            cap = math.atan2(mag[1], mag[0])  # uncalibrated, as imu9 heading_raw
        return (self.heading_sign * math.degrees(cap) + self.heading_offset) % 360

    def _run(self):
        last_imu = 0.0
        last_status = 0.0
        while True:
            upd = {}
            if self.gps:
                try:
                    ok, val = self.gps.read_gll_non_blocking()
                    if ok and val[0] != 0:
                        upd["lat"] = round(nmea_to_decimal(val[0], val[1]), 7)
                        upd["lon"] = round(nmea_to_decimal(val[2], val[3]), 7)
                        upd["fix"] = 1
                        upd["gps_t"] = time.time()
                except Exception as e:
                    upd["status"] = "GPS error: %s" % e
            now = time.time()
            if self.imu and now - last_imu > 0.1:
                last_imu = now
                try:
                    mag = self.imu.read_mag_raw()
                    acc = self.imu.read_accel_raw()
                    gyr = self.imu.read_gyro_raw()
                    upd["hdg"] = round(self._heading(mag), 1)
                    upd["roll"] = round(math.degrees(math.atan2(acc[1], acc[2])), 1)
                    upd["pitch"] = round(math.degrees(math.atan2(-acc[0], math.hypot(acc[1], acc[2]))), 1)
                    upd["gz"] = round(gyr[2] * 0.07, 1)   # 2000 dps full scale: 70 mdps/LSB
                except Exception as e:
                    upd["status"] = "IMU error: %s" % e
            if self.arduino_status and now - last_status > 2.0:
                last_status = now
                try:
                    with self.ser_lock:
                        upd["status"] = self.ard.get_arduino_status(timeout=0.2)
                except Exception:
                    pass
            if upd:
                with self.state_lock:
                    self.state.update(upd)
            if not self.gps:
                time.sleep(0.02)

    def snapshot(self):
        with self.state_lock:
            d = dict(self.state)
        gps_t = d.pop("gps_t", None)
        if gps_t is None or time.time() - gps_t > 3.0:
            d["fix"] = 0
        d["t"] = round(time.time() - self.t0, 2)
        return d


# ---------------------------------------------------------------------------
# client handling
# ---------------------------------------------------------------------------
def telemetry_loop(conn, boat, state, send_lock, alive):
    period = 1.0 / TELEMETRY_HZ
    while alive.is_set():
        d = boat.snapshot()
        d["cmd"] = list(state["cmd"])
        line = "T " + json.dumps(d, separators=(",", ":")) + "\n"
        try:
            with send_lock:
                conn.sendall(line.encode("ascii"))
        except (socket.error, OSError):
            break
        time.sleep(period)


def handle_client(conn, addr, boat):
    print("Client connected: %s" % (addr,))
    conn.settimeout(WATCHDOG_TIMEOUT)
    buf = b""
    state = {"cmd": (0, 0)}
    send_lock = threading.Lock()
    alive = threading.Event()
    alive.set()
    tt = threading.Thread(target=telemetry_loop, args=(conn, boat, state, send_lock, alive))
    tt.daemon = True
    tt.start()
    try:
        while True:
            try:
                data = conn.recv(1024)
            except socket.timeout:
                if state["cmd"] != (0, 0):
                    print("Watchdog timeout -> stopping motors")
                    boat.send_arduino_cmd_motor(0, 0)
                    state["cmd"] = (0, 0)
                continue
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                parts = line.decode("ascii", "ignore").split()
                if not parts:
                    continue
                cmd = parts[0].upper()
                if cmd == "M" and len(parts) == 3:
                    try:
                        new = (clamp(parts[1]), clamp(parts[2]))
                    except ValueError:
                        continue
                elif cmd == "S":
                    new = (0, 0)
                elif cmd == "P":
                    if len(parts) > 1:
                        with send_lock:
                            conn.sendall(("O %s\n" % parts[1]).encode("ascii"))
                    continue
                else:
                    print("Unknown command: %r" % (line,))
                    continue
                if new != state["cmd"]:
                    boat.send_arduino_cmd_motor(*new)
                    state["cmd"] = new
                    print("L=%4d R=%4d" % new)
    except (socket.error, OSError):
        pass
    finally:
        alive.clear()
        boat.send_arduino_cmd_motor(0, 0)
        conn.close()
        print("Client disconnected: %s -> motors stopped" % (addr,))


def main():
    ap = argparse.ArgumentParser(description="DDBoat motor + telemetry server")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--sim", action="store_true", help="simulated boat, no hardware")
    ap.add_argument("--sim-lat", type=float, default=48.4183)
    ap.add_argument("--sim-lon", type=float, default=-4.4741)
    ap.add_argument("--mag-calib", help='JSON {"invA": [[...]x3], "xbar": [x, y, z]}')
    ap.add_argument("--heading-offset", type=float, default=0.0, help="deg added to heading")
    ap.add_argument("--heading-sign", type=float, default=1.0, help="-1 to flip heading direction")
    ap.add_argument("--arduino-status", action="store_true",
                    help="also poll the Arduino status line every 2 s")
    args = ap.parse_args()

    if args.sim:
        boat = SimBoat(args.sim_lat, args.sim_lon)
    else:
        boat = RealBoat(args.mag_calib, args.heading_offset, args.heading_sign, args.arduino_status)
    boat.send_arduino_cmd_motor(0, 0)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print("Listening on %s:%d%s" % (args.host, args.port, " (SIMULATION)" if args.sim else ""))
    try:
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            handle_client(conn, addr, boat)
    except KeyboardInterrupt:
        print("\nShutting down")
    finally:
        boat.send_arduino_cmd_motor(0, 0)
        srv.close()


if __name__ == "__main__":
    main()
