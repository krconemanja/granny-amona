"""Telemetry model (Tracker) and the right-hand telemetry panel."""
import csv
import math
import os
import time
from collections import deque

from PySide6.QtCore import Qt, QPointF, QRectF
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPainterPath
from PySide6.QtWidgets import (QFrame, QGridLayout, QLabel, QVBoxLayout, QWidget,
                               QHBoxLayout)

from map_widget import haversine, bearing

DASH = "—"
KNOWN_KEYS = {"t", "lat", "lon", "fix", "sats", "hdg", "roll", "pitch", "gz",
              "cmd", "act", "batt", "rc", "status", "uptime", "sim"}


class Tracker:
    """Turns raw T packets into derived values: speed, COG, distance, route."""

    MIN_STEP_M = 1.0
    MAX_GAP_S = 5.0
    MAX_POINTS = 20000

    def __init__(self, log_dir=None):
        self.last = {}            # last raw packet
        self.last_rx = None       # monotonic time of last packet
        self.fix_t = None         # monotonic time of last valid fix
        self.route = []           # [(lat, lon)]
        self.route_t = []         # wall times for GPX
        self.home = None
        self.distance = 0.0
        self.speed = None         # m/s
        self.cog = None           # deg
        self._prev_fix = None     # (t, lat, lon)
        self._route_last_t = 0.0
        self._cog_anchor = None
        self.pos = None           # last valid (lat, lon)
        self.speed_hist = deque(maxlen=300)   # (t, v) ~60 s at 5 Hz
        self.cmd_hist = deque(maxlen=300)     # (t, l, r)
        self.rx_times = deque(maxlen=50)
        self.log_dir = log_dir
        self._log = None
        self._log_writer = None

    # ---- ingest ------------------------------------------------------------
    def ingest(self, d, now=None):
        now = time.monotonic() if now is None else now
        self.last = d
        self.last_rx = now
        self.rx_times.append(now)
        lat, lon = d.get("lat"), d.get("lon")
        valid = (isinstance(lat, (int, float)) and isinstance(lon, (int, float))
                 and d.get("fix", 1) and not (lat == 0 and lon == 0))
        if valid:
            self.fix_t = now
            self.pos = (lat, lon)
            if self.home is None:
                self.home = (lat, lon)
            if self._prev_fix:
                t0, la0, lo0 = self._prev_fix
                dt = now - t0
                dist = haversine(la0, lo0, lat, lon)
                if dt > 0.15:
                    v = dist / dt
                    self.speed = v if self.speed is None else 0.6 * self.speed + 0.4 * v
                    self._prev_fix = (now, lat, lon)
            else:
                self._prev_fix = (now, lat, lon)
            if self._cog_anchor is None:
                self._cog_anchor = (lat, lon)
            elif haversine(self._cog_anchor[0], self._cog_anchor[1], lat, lon) >= 1.0:
                self.cog = bearing(self._cog_anchor[0], self._cog_anchor[1], lat, lon)
                self._cog_anchor = (lat, lon)
            self._add_route(lat, lon, now)
        if self.speed is not None:
            self.speed_hist.append((now, self.speed))
        cmd = d.get("cmd")
        if isinstance(cmd, (list, tuple)) and len(cmd) == 2:
            self.cmd_hist.append((now, cmd[0], cmd[1]))
        self._write_log(d)

    def add_command(self, left, right, now=None):
        """Commanded L/R as the GUI sent them (used when the boat has no telemetry)."""
        if "cmd" not in self.last:
            now = time.monotonic() if now is None else now
            self.cmd_hist.append((now, left, right))

    def _add_route(self, lat, lon, now):
        if self.route:
            la, lo = self.route[-1]
            step = haversine(la, lo, lat, lon)
            if step < self.MIN_STEP_M and now - self._route_last_t < self.MAX_GAP_S:
                return
            self.distance += step
        self.route.append((lat, lon))
        self.route_t.append(time.time())
        self._route_last_t = now
        if len(self.route) > self.MAX_POINTS:
            del self.route[0]
            del self.route_t[0]

    def clear_route(self):
        self.route = self.route[-1:]
        self.route_t = self.route_t[-1:]
        self.distance = 0.0

    # ---- derived -----------------------------------------------------------
    @property
    def position(self):
        return self.pos

    @property
    def heading(self):
        h = self.last.get("hdg")
        return h if isinstance(h, (int, float)) else self.cog

    def rate_hz(self, now=None):
        now = time.monotonic() if now is None else now
        recent = [t for t in self.rx_times if now - t < 2.0]
        return len(recent) / 2.0

    def home_vector(self):
        p = self.position
        if not (p and self.home):
            return None
        return haversine(p[0], p[1], *self.home), bearing(p[0], p[1], *self.home)

    # ---- logging / export --------------------------------------------------
    def _write_log(self, d):
        if not self.log_dir:
            return
        if self._log is None:
            os.makedirs(self.log_dir, exist_ok=True)
            name = time.strftime("session_%Y%m%d_%H%M%S.csv")
            self._log = open(os.path.join(self.log_dir, name), "w", newline="")
            self._log_writer = csv.writer(self._log)
            self._log_writer.writerow(["time", "lat", "lon", "hdg", "speed", "cmd_l", "cmd_r"])
        cmd = d.get("cmd") or [None, None]
        self._log_writer.writerow([round(time.time(), 3), d.get("lat"), d.get("lon"),
                                   d.get("hdg"), None if self.speed is None else round(self.speed, 3),
                                   cmd[0], cmd[1]])
        self._log.flush()

    def export_gpx(self, path):
        with open(path, "w") as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n'
                    '<gpx version="1.1" creator="ddboat-steamdeck-gui" '
                    'xmlns="http://www.topografix.com/GPX/1/1">\n<trk><name>DDBoat route</name><trkseg>\n')
            for (lat, lon), t in zip(self.route, self.route_t):
                f.write('<trkpt lat="%.7f" lon="%.7f"><time>%s</time></trkpt>\n'
                        % (lat, lon, time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t))))
            f.write("</trkseg></trk>\n</gpx>\n")
        return len(self.route)


# ---- widgets ---------------------------------------------------------------
class Sparkline(QWidget):
    def __init__(self, color="#3ea6ff", span=60.0, parent=None):
        super().__init__(parent)
        self.color = QColor(color)
        self.span = span
        self.series = []          # list of deques/lists of (t, v)
        self.colors = []
        self.sym = False          # symmetric around zero
        self.setMinimumHeight(34)

    def set_data(self, series, colors, sym=False):
        self.series, self.colors, self.sym = series, colors, sym
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.setPen(QPen(QColor(255, 255, 255, 40), 1))
        mid = h / 2 if self.sym else h - 1
        p.drawLine(QPointF(0, mid), QPointF(w, mid))
        now = time.monotonic()
        vals = [v for s in self.series for _, v in s]
        if not vals:
            p.end()
            return
        m = max(1e-6, max(abs(v) for v in vals))
        for s, c in zip(self.series, self.colors):
            path = QPainterPath()
            first = True
            for t, v in s:
                x = w - (now - t) / self.span * w
                if x < 0:
                    continue
                y = mid - (v / m) * (h / 2 - 2 if self.sym else h - 4)
                if first:
                    path.moveTo(x, y)
                    first = False
                else:
                    path.lineTo(x, y)
            p.setPen(QPen(QColor(c), 1.8))
            p.drawPath(path)
        p.end()


class PropBars(QWidget):
    """Two vertical bars, centre = 0, up = forward. Ghost marker = actual."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.cmd = (None, None)
        self.act = (None, None)
        self.max = 255
        self.setMinimumSize(110, 120)

    def set_values(self, cmd, act):
        self.cmd, self.act = cmd, act
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        bw = 30
        top, bot = 6, h - 26
        mid = (top + bot) / 2
        f = QFont(self.font())
        f.setPointSizeF(10)
        p.setFont(f)
        for i, (name, c, a) in enumerate((("L", self.cmd[0], self.act[0]), ("R", self.cmd[1], self.act[1]))):
            x = w / 2 - bw - 10 + i * (bw + 20)
            p.setPen(QPen(QColor(255, 255, 255, 90), 1))
            p.setBrush(Qt.NoBrush)
            p.drawRoundedRect(QRectF(x, top, bw, bot - top), 4, 4)
            p.drawLine(QPointF(x - 4, mid), QPointF(x + bw + 4, mid))
            if c is not None:
                v = max(-1, min(1, c / self.max))
                y = mid - v * (mid - top)
                p.setPen(Qt.NoPen)
                p.setBrush(QColor("#3ea6ff") if c >= 0 else QColor("#ef5350"))
                p.drawRoundedRect(QRectF(x + 2, min(y, mid), bw - 4, abs(y - mid)), 3, 3)
            if a is not None:
                v = max(-1, min(1, a / self.max))
                y = mid - v * (mid - top)
                p.setPen(QPen(QColor("#ffffff"), 2.5))
                p.drawLine(QPointF(x - 3, y), QPointF(x + bw + 3, y))
            p.setPen(QColor("white"))
            p.drawText(QRectF(x - 12, bot + 4, bw + 24, 18), Qt.AlignCenter,
                       "%s %s" % (name, DASH if c is None else "%d" % c))
        p.end()


class TelemetryPanel(QFrame):
    ROWS = [
        ("Position", [("lat", "Lat"), ("lon", "Lon"), ("fix", "Fix age")]),
        ("Motion", [("speed", "Speed"), ("hdg", "Heading"), ("cog", "Course (GPS)"),
                    ("dist", "Distance"), ("home", "Home")]),
        ("Attitude", [("roll", "Roll"), ("pitch", "Pitch"), ("gz", "Yaw rate")]),
        ("System", [("batt", "Battery"), ("rc", "RC override"), ("status", "Status")]),
        ("Link & input", [("rtt", "RTT"), ("rate", "Telemetry"), ("lost", "Link drops"),
                          ("input", "Input"), ("groll", "Deck roll / tilt"), ("paddle", "Dead-man paddle")]),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("telemetry")
        outer = QVBoxLayout(self)
        outer.setContentsMargins(12, 10, 12, 10)
        outer.setSpacing(2)
        self.values = {}
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(1)
        r = 0
        for gname, fields in self.ROWS:
            if gname == "Attitude":
                # propellers block goes between Motion and Attitude
                outer.addLayout(grid)
                outer.addWidget(self._header("Propellers  (bar = command, line = actual)"))
                row = QHBoxLayout()
                self.props = PropBars()
                row.addWidget(self.props, 0)
                col = QVBoxLayout()
                self.spark_cmd = Sparkline()
                self.spark_speed = Sparkline()
                col.addWidget(self._small("L / R command, 60 s"))
                col.addWidget(self.spark_cmd)
                col.addWidget(self._small("Speed, 60 s"))
                col.addWidget(self.spark_speed)
                row.addLayout(col, 1)
                outer.addLayout(row)
                grid = QGridLayout()
                grid.setHorizontalSpacing(10)
                grid.setVerticalSpacing(1)
                r = 0
            grid.addWidget(self._header(gname), r, 0, 1, 2)
            r += 1
            for key, label in fields:
                lab = QLabel(label)
                lab.setObjectName("tlabel")
                val = QLabel(DASH)
                val.setObjectName("tvalue")
                val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                grid.addWidget(lab, r, 0)
                grid.addWidget(val, r, 1)
                self.values[key] = val
                r += 1
        self.raw = QLabel("")
        self.raw.setObjectName("tlabel")
        self.raw.setWordWrap(True)
        outer.addLayout(grid)
        outer.addWidget(self.raw)
        outer.addStretch(1)

    def _header(self, t):
        l = QLabel(t)
        l.setObjectName("theader")
        return l

    def _small(self, t):
        l = QLabel(t)
        l.setObjectName("tlabel")
        return l

    def set(self, key, text):
        self.values[key].setText(DASH if text is None else text)

    def refresh(self, tr, pilot, link):
        d = tr.last
        now = time.monotonic()
        f = lambda k, fmt: fmt % d[k] if isinstance(d.get(k), (int, float)) else None
        self.set("lat", f("lat", "%.6f°"))
        self.set("lon", f("lon", "%.6f°"))
        self.set("fix", None if tr.fix_t is None else "%.1f s" % (now - tr.fix_t))
        self.set("speed", None if tr.speed is None else "%.2f m/s  %.1f kn" % (tr.speed, tr.speed * 1.94384))
        self.set("hdg", f("hdg", "%05.1f°"))
        self.set("cog", None if tr.cog is None else "%05.1f°" % tr.cog)
        self.set("dist", "%.0f m" % tr.distance if tr.route else None)
        hv = tr.home_vector()
        self.set("home", None if hv is None else "%.0f m @ %03.0f°" % hv)
        self.set("roll", f("roll", "%.1f°"))
        self.set("pitch", f("pitch", "%.1f°"))
        self.set("gz", f("gz", "%.1f°/s"))
        self.set("batt", f("batt", "%.2f V"))
        rc = d.get("rc")
        self.set("rc", None if rc is None else ("ON" if rc else "off"))
        st = d.get("status")
        self.set("status", None if st is None else str(st)[:28])
        self.set("rtt", None if link.get("rtt") is None else "%.0f ms" % link["rtt"])
        self.set("rate", "%.1f Hz" % tr.rate_hz(now) if tr.last_rx else None)
        self.set("lost", str(link.get("lost", 0)))
        self.set("groll", "%+.1f° / %+.1f°" % (pilot["roll"], pilot["pitch"])
                 if pilot["has_gyro"] else "no gyro")
        self.set("paddle", "held" if pilot.get("deadman") else "released")
        self.set("input", pilot["input"])

        cmd = d.get("cmd") if isinstance(d.get("cmd"), list) else pilot["sent"]
        act = d.get("act") if isinstance(d.get("act"), list) else (None, None)
        self.props.set_values(tuple(cmd), tuple(act))
        self.spark_cmd.set_data([[(t, l) for t, l, _ in tr.cmd_hist], [(t, r) for t, _, r in tr.cmd_hist]],
                                ["#3ea6ff", "#ffb74d"], sym=True)
        self.spark_speed.set_data([tr.speed_hist], ["#81c784"])
        extra = {k: v for k, v in d.items() if k not in KNOWN_KEYS}
        self.raw.setText("Raw: " + ", ".join("%s=%s" % kv for kv in extra.items()) if extra else "")
