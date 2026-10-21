"""TCP link to the boat (server.py or server_telemetry.py).

A background thread owns the socket so the UI never blocks. The GUI calls
set_command(left, right) at its own rate; the thread sends the latest value
every SEND_PERIOD. If the GUI stops refreshing the command (frozen UI), the
thread sends S so the boat stops.

Protocol (newline-terminated ASCII):
  GUI -> boat:  M <l> <r> | S | P <t_ms>
  boat -> GUI:  T {json}  | O <t_ms>
"""
import json
import select
import socket
import threading
import time

from PySide6.QtCore import QObject, Signal

SEND_PERIOD = 0.1       # s, also the heartbeat for the boat watchdog
PING_PERIOD = 1.0       # s
STALE_CMD = 0.5         # s without set_command() -> send S instead of M
TELEMETRY_TIMEOUT = 2.0 # s without T (once seen) -> link Lost
RETRY_PERIOD = 2.0      # s between reconnect attempts


def parse_host(text):
    """Accept '1.2.3.4', 'user@1.2.3.4', '1.2.3.4:5000' -> (host, port|None)."""
    text = text.strip().split("@")[-1]
    port = None
    if text.count(":") == 1:
        text, p = text.split(":")
        try:
            port = int(p)
        except ValueError:
            port = None
    return text, port


def parse_line(line):
    """Parse one boat->GUI line. Returns (kind, payload) or (None, None)."""
    line = line.strip()
    if not line:
        return None, None
    if line.startswith("T "):
        try:
            data = json.loads(line[2:])
        except ValueError:
            return None, None
        return ("T", data) if isinstance(data, dict) else (None, None)
    if line.startswith("O "):
        try:
            return "O", float(line[2:].split()[0])
        except (ValueError, IndexError):
            return None, None
    return None, None


class Link(QObject):
    # state: Disconnected | Connecting | Connected | Lost ; detail: free text
    status = Signal(str, str)
    telemetry = Signal(dict)
    rtt = Signal(float)              # ms
    sent = Signal(int, int)          # what actually went out

    def __init__(self):
        super().__init__()
        self._lock = threading.Lock()
        self._cmd = (0, 0)
        self._cmd_t = 0.0
        self._thread = None
        self._stop = threading.Event()
        self._estop = threading.Event()
        self.state = "Disconnected"
        self.rx_count = 0
        self.lost_count = 0

    # ---- called from the GUI thread ---------------------------------------
    def connect_to(self, host, port):
        self.disconnect()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(host, port), daemon=True)
        self._thread.start()

    def disconnect(self):
        if self._thread:
            self._stop.set()
            self._thread.join(timeout=3)
            self._thread = None

    def emergency_stop(self):
        """Send S right away (also zeroes the command)."""
        self.set_command(0, 0)
        self._estop.set()

    def set_command(self, left, right):
        with self._lock:
            self._cmd = (int(left), int(right))
            self._cmd_t = time.monotonic()

    # ---- background thread -------------------------------------------------
    def _set_state(self, state, detail=""):
        self.state = state
        self.status.emit(state, detail)

    def _run(self, host, port):
        first = True
        while not self._stop.is_set():
            self._set_state("Connecting" if first else "Lost",
                            "%s:%d" % (host, port) if first else "retrying %s:%d" % (host, port))
            try:
                sock = socket.create_connection((host, port), timeout=5)
            except OSError as e:
                if first:
                    self._set_state("Disconnected", "connect failed: %s" % e)
                    return
                if self._stop.wait(RETRY_PERIOD):
                    break
                continue
            first = False
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._set_state("Connected", "%s:%d" % (host, port))
            reason = self._session(sock)
            try:
                sock.sendall(b"S\n")
            except OSError:
                pass
            sock.close()
            if self._stop.is_set():
                break
            self.lost_count += 1
            self._set_state("Lost", reason)
            if self._stop.wait(RETRY_PERIOD):
                break
        self._set_state("Disconnected", "")

    def _session(self, sock):
        """Runs until stop or link failure. Returns a reason string."""
        buf = b""
        last_send = 0.0
        last_ping = 0.0
        last_rx_t = None
        sock.setblocking(False)
        while not self._stop.is_set():
            now = time.monotonic()
            try:
                if self._estop.is_set():
                    self._estop.clear()
                    sock.sendall(b"S\n")
                    self.sent.emit(0, 0)
                    last_send = now
                if now - last_send >= SEND_PERIOD:
                    with self._lock:
                        cmd, cmd_t = self._cmd, self._cmd_t
                    if now - cmd_t > STALE_CMD:
                        sock.sendall(b"S\n")
                        self.sent.emit(0, 0)
                    else:
                        sock.sendall(("M %d %d\n" % cmd).encode("ascii"))
                        self.sent.emit(*cmd)
                    last_send = now
                if now - last_ping >= PING_PERIOD:
                    sock.sendall(("P %d\n" % int(time.time() * 1000)).encode("ascii"))
                    last_ping = now
            except OSError as e:
                return "send failed: %s" % e

            r, _, _ = select.select([sock], [], [], 0.02)
            if r:
                try:
                    data = sock.recv(4096)
                except BlockingIOError:
                    data = None
                except OSError as e:
                    return "receive failed: %s" % e
                if data == b"":
                    return "boat closed the connection"
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        kind, payload = parse_line(line.decode("utf-8", "ignore"))
                        if kind == "T":
                            last_rx_t = time.monotonic()
                            self.rx_count += 1
                            self.telemetry.emit(payload)
                        elif kind == "O":
                            last_rx_t = time.monotonic()
                            self.rtt.emit(time.time() * 1000 - payload)
                    if len(buf) > 65536:
                        buf = b""
            if last_rx_t is not None and time.monotonic() - last_rx_t > TELEMETRY_TIMEOUT:
                return "no telemetry for %.0f s" % TELEMETRY_TIMEOUT
        return "disconnected"
