"""DDBoat motor server -- run this ON THE BOAT.

Listens on a TCP socket for motor commands from client.py and forwards
them to the Arduino propeller driver.

Protocol (newline-terminated ASCII lines):
    M <left> <right>    set motor speeds, each in [-MAX_SPEED, MAX_SPEED]
    S                   stop both motors
    P                   ping / heartbeat (keeps the watchdog alive)

Safety: motors are stopped if the client disconnects or if no message
arrives within WATCHDOG_TIMEOUT seconds.

Usage:
    python3 server.py [--host 0.0.0.0] [--port 5000] [--dry-run]
"""
from __future__ import print_function
import sys
import os
import time
import socket
import argparse

# access to the drivers
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

MAX_SPEED = 255
WATCHDOG_TIMEOUT = 1.0  # seconds without a message before motors are stopped


class DryRunMotors:
    """Stand-in for the Arduino when testing without the boat."""
    def send_arduino_cmd_motor(self, left, right):
        print("[dry-run] motors L=%4d R=%4d" % (left, right))


def make_motors(dry_run):
    if dry_run:
        return DryRunMotors()
    import arduino_driver_v2 as arddrv
    return arddrv.ArduinoIO()


def clamp(v):
    return max(-MAX_SPEED, min(MAX_SPEED, int(v)))


def handle_client(conn, addr, ard):
    print("Client connected: %s" % (addr,))
    conn.settimeout(WATCHDOG_TIMEOUT)
    buf = b""
    current = (0, 0)
    try:
        while True:
            try:
                data = conn.recv(1024)
            except socket.timeout:
                if current != (0, 0):
                    print("Watchdog timeout -> stopping motors")
                    ard.send_arduino_cmd_motor(0, 0)
                    current = (0, 0)
                continue
            if not data:
                break  # client closed the connection
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
                    continue
                else:
                    print("Unknown command: %r" % (line,))
                    continue
                if new != current:
                    ard.send_arduino_cmd_motor(*new)
                    current = new
                    print("L=%4d R=%4d" % new)
    except socket.error:  # connection reset / broken pipe
        pass
    finally:
        ard.send_arduino_cmd_motor(0, 0)
        conn.close()
        print("Client disconnected: %s -> motors stopped" % (addr,))


def main():
    parser = argparse.ArgumentParser(description="DDBoat motor server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--dry-run", action="store_true",
                        help="print motor commands instead of driving the Arduino")
    args = parser.parse_args()

    ard = make_motors(args.dry_run)
    ard.send_arduino_cmd_motor(0, 0)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print("Listening on %s:%d" % (args.host, args.port))

    try:
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            handle_client(conn, addr, ard)  # one pilot at a time
    except KeyboardInterrupt:
        print("\nShutting down")
    finally:
        ard.send_arduino_cmd_motor(0, 0)
        srv.close()


if __name__ == "__main__":
    main()
