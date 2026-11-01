"""Remote control for mission.py -- run ON THE LAPTOP.

    R      run / resume the mission       P      pause and drive with W A S D
    C      center the steering            SPACE  stop
    Q      stop and quit

W/S and A/D change throttle and turn by a step per press; they stay until
changed again (terminals don't report key releases).

    python3 client.py 12          DDBoat 12 = 172.20.25.212
    python3 client.py localhost   mission.py --sim on this laptop
"""
import select
import socket
import sys
import termios
import time
import tty

STEP = 25  # throttle / turn change per key press
MAX = 200  # max motor command
MODES = {'r': 'AUTO', 'p': 'MANUAL', ' ': 'STOP'}

boat = sys.argv[1]
if boat.isdigit():
    boat = '172.20.25.2%02d' % int(boat)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send(text):
    try:
        sock.sendto(text.encode(), (boat, 5000))
    except socket.error:
        pass


throttle = turn = 0
status, last_status = '', 0.0
old_terminal = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin)
try:
    while True:
        ready = select.select([sys.stdin, sock], [], [], 0.1)[0]
        if sock in ready:
            try:
                status, last_status = sock.recv(100).decode(), time.time()
            except socket.error:  # boat not listening (yet)
                pass
        if sys.stdin in ready:
            key = sys.stdin.read(1).lower()
            if key == 'q':
                break
            elif key in MODES:
                throttle = turn = 0
                for _ in range(3):  # UDP may lose a packet
                    send('MODE ' + MODES[key])
            elif key == 'w':
                throttle = min(MAX, throttle + STEP)
            elif key == 's':
                throttle = max(-MAX, throttle - STEP)
            elif key == 'd':
                turn = min(MAX, turn + STEP)
            elif key == 'a':
                turn = max(-MAX, turn - STEP)
            elif key == 'c':
                turn = 0
        left = max(-MAX, min(MAX, throttle + turn))
        right = max(-MAX, min(MAX, throttle - turn))
        send('M %d %d' % (left, right))  # also tells the boat the laptop is there
        if time.time() - last_status > 2.0:
            status = 'NO LINK'
        sys.stdout.write('\r%-35s  you: L=%4d R=%4d  ' % (status, left, right))
        sys.stdout.flush()
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
    for _ in range(3):
        send('MODE STOP')
    print('\nstopped')
