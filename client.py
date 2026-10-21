"""DDBoat WASD remote control -- run this ON YOUR LAPTOP.

Connects to server.py on the boat and drives the two propellers
differentially (left/right).

Controls (terminal, no extra libraries needed):
    W / S     increase / decrease forward throttle
    A / D     turn left / right (more presses = sharper turn)
    C         center the steering (turn = 0)
    SPACE     stop everything
    Q         quit (motors are stopped)

Terminals don't report key releases, so throttle and turn are "sticky":
each press changes them by a step and they stay until changed again.

Usage:
    python3 client.py <boat-ip> [--port 5000] [--step 25] [--max 200]
"""
from __future__ import print_function
import sys
import time
import socket
import select
import termios
import tty
import argparse

SEND_PERIOD = 0.1  # seconds; also serves as heartbeat for the server watchdog


def mix(throttle, turn, max_speed):
    """Differential drive: turn > 0 means turn right (left prop faster)."""
    left = throttle + turn
    right = throttle - turn
    # scale down both sides if one saturates, so the turn ratio is preserved
    biggest = max(abs(left), abs(right), max_speed)
    left = int(left * max_speed / biggest)
    right = int(right * max_speed / biggest)
    return left, right


def main():
    parser = argparse.ArgumentParser(description="DDBoat WASD client")
    parser.add_argument("host", help="IP address of the DDBoat")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--step", type=int, default=25, help="change per key press")
    parser.add_argument("--max", type=int, default=200, help="max motor speed (<=255)")
    args = parser.parse_args()
    args.host = args.host.split("@")[-1]  # accept ssh-style user@host too

    sock = socket.create_connection((args.host, args.port), timeout=5)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    throttle = 0
    turn = 0
    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    print("Connected to %s:%d" % (args.host, args.port))
    print("W/S throttle, A/D turn, C center, SPACE stop, Q quit\r")

    try:
        tty.setcbreak(fd)
        last_send = 0.0
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], SEND_PERIOD)
            if ready:
                key = sys.stdin.read(1).lower()
                if key == "w":
                    throttle += args.step
                elif key == "s":
                    throttle -= args.step
                elif key == "d":
                    turn += args.step
                elif key == "a":
                    turn -= args.step
                elif key == "c":
                    turn = 0
                elif key == " ":
                    throttle = turn = 0
                elif key == "q":
                    break
                throttle = max(-args.max, min(args.max, throttle))
                turn = max(-args.max, min(args.max, turn))

            now = time.time()
            if ready or now - last_send >= SEND_PERIOD:
                left, right = mix(throttle, turn, args.max)
                sock.sendall(("M %d %d\n" % (left, right)).encode("ascii"))
                last_send = now
                sys.stdout.write(
                    "\rthrottle=%4d turn=%4d  ->  L=%4d R=%4d   " % (throttle, turn, left, right))
                sys.stdout.flush()
    except socket.error:  # connection reset / broken pipe
        print("\nConnection lost")
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        try:
            sock.sendall(b"S\n")
        except socket.error:
            pass
        sock.close()
        print("\nStopped and disconnected")


if __name__ == "__main__":
    main()
