"""Slippy map widget: OSM tiles + boat + route, no browser engine.

Tiles are fetched in the background and cached on disk
(~/.cache/boat-gui/tiles/z/x/y.png). With no network and no cache, a metric
grid is drawn so the route and boat are still readable.
"""
import math
import os
import time

from PySide6.QtCore import Qt, QPointF, QRectF, QUrl, Signal, QEvent
from PySide6.QtGui import (QColor, QFont, QPainter, QPainterPath, QPen, QPixmap,
                           QPolygonF, QBrush)
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply
from PySide6.QtWidgets import QWidget, QPinchGesture

TILE = 256
TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT = b"ddboat-steamdeck-gui/1.0 (student robotics project)"
CACHE_DIR = os.path.expanduser("~/.cache/boat-gui/tiles")
MIN_Z, MAX_Z = 3, 19
EARTH_R = 6371000.0


# ---- geo helpers (unit tested) ---------------------------------------------
def lonlat_to_px(lon, lat, z):
    """World pixel coordinates at zoom z (Web Mercator)."""
    n = TILE * (2 ** z)
    lat = max(-85.0511, min(85.0511, lat))
    x = (lon + 180.0) / 360.0 * n
    r = math.radians(lat)
    y = (1.0 - math.log(math.tan(r) + 1.0 / math.cos(r)) / math.pi) / 2.0 * n
    return x, y


def px_to_lonlat(x, y, z):
    n = TILE * (2 ** z)
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lon, lat


def meters_per_px(lat, z):
    return 156543.03392 * math.cos(math.radians(lat)) / (2 ** z)


def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(min(1.0, math.sqrt(a)))


def bearing(lat1, lon1, lat2, lon2):
    """Initial bearing in degrees, 0 = north, clockwise."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def nice_scale(meters):
    """Largest 1/2/5 x 10^k not above `meters`."""
    if meters <= 0:
        return 1
    e = 10 ** math.floor(math.log10(meters))
    for m in (5, 2, 1):
        if m * e <= meters:
            return m * e
    return e


# ---- widget ----------------------------------------------------------------
class MapWidget(QWidget):
    followChanged = Signal(bool)

    def __init__(self, parent=None, center=(48.4183, -4.4741), zoom=18):
        super().__init__(parent)
        self.setMinimumSize(300, 240)
        self.setAttribute(Qt.WA_AcceptTouchEvents, True)
        self.grabGesture(Qt.PinchGesture)
        self.lat, self.lon = center
        self.z = zoom
        self.follow = True
        self.boat = None          # (lat, lon)
        self.heading = None       # deg, 0 = north
        self.route = []           # [(lat, lon)]
        self.home = None
        self.info_lines = []      # overlay text, bottom left
        self.online = True
        self._tiles = {}          # (z,x,y) -> QPixmap
        self._pending = set()
        self._failed = {}         # key -> time of failure
        self._drag = None
        self._pinch_acc = 1.0
        self._net = QNetworkAccessManager(self)
        self._net.finished.connect(self._on_tile)

    # ---- public API --------------------------------------------------------
    def set_boat(self, lat, lon, heading=None):
        self.boat = (lat, lon)
        self.heading = heading
        if self.follow:
            self.lat, self.lon = lat, lon
        self.update()

    def set_route(self, pts):
        self.route = pts
        self.update()

    def set_home(self, latlon):
        self.home = latlon
        self.update()

    def center_on_boat(self):
        self.follow = True
        self.followChanged.emit(True)
        if self.boat:
            self.lat, self.lon = self.boat
        self.update()

    def zoom_by(self, dz):
        self.z = max(MIN_Z, min(MAX_Z, self.z + dz))
        self.update()

    # ---- tiles -------------------------------------------------------------
    def _tile(self, z, x, y):
        n = 2 ** z
        if y < 0 or y >= n:
            return None
        x %= n
        key = (z, x, y)
        if key in self._tiles:
            return self._tiles[key]
        path = os.path.join(CACHE_DIR, str(z), str(x), "%d.png" % y)
        if os.path.exists(path):
            pm = QPixmap(path)
            if not pm.isNull():
                self._tiles[key] = pm
                if len(self._tiles) > 600:
                    self._tiles.pop(next(iter(self._tiles)))
                return pm
        if key not in self._pending and time.time() - self._failed.get(key, 0) > 30:
            if len(self._pending) < 16:
                self._pending.add(key)
                req = QNetworkRequest(QUrl(TILE_URL.format(z=z, x=x, y=y)))
                req.setRawHeader(b"User-Agent", USER_AGENT)
                req.setAttribute(QNetworkRequest.User, key)
                self._net.get(req)
        return None

    def _on_tile(self, reply):
        key = reply.request().attribute(QNetworkRequest.User)
        self._pending.discard(key)
        if reply.error() == QNetworkReply.NoError:
            data = reply.readAll().data()
            pm = QPixmap()
            if pm.loadFromData(data):
                self._tiles[key] = pm
                z, x, y = key
                d = os.path.join(CACHE_DIR, str(z), str(x))
                try:
                    os.makedirs(d, exist_ok=True)
                    with open(os.path.join(d, "%d.png" % y), "wb") as f:
                        f.write(data)
                except OSError:
                    pass
                self.online = True
                self.update()
        else:
            self._failed[key] = time.time()
            self.online = False
        reply.deleteLater()

    # ---- painting ----------------------------------------------------------
    def _to_screen(self, lat, lon, cx, cy):
        x, y = lonlat_to_px(lon, lat, self.z)
        return QPointF(x - cx + self.width() / 2, y - cy + self.height() / 2)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor("#1d2329"))
        cx, cy = lonlat_to_px(self.lon, self.lat, self.z)
        self._draw_grid(p, cx, cy)

        x0 = int(math.floor((cx - w / 2) / TILE))
        x1 = int(math.floor((cx + w / 2) / TILE))
        y0 = int(math.floor((cy - h / 2) / TILE))
        y1 = int(math.floor((cy + h / 2) / TILE))
        for tx in range(x0, x1 + 1):
            for ty in range(y0, y1 + 1):
                pm = self._tile(self.z, tx, ty)
                if pm:
                    p.drawPixmap(int(tx * TILE - cx + w / 2), int(ty * TILE - cy + h / 2), pm)
        # dim tiles slightly so overlays read on the dark theme
        p.fillRect(self.rect(), QColor(0, 0, 0, 40))

        if len(self.route) > 1:
            path = QPainterPath()
            path.moveTo(self._to_screen(*self.route[0], cx, cy))
            for pt in self.route[1:]:
                path.lineTo(self._to_screen(*pt, cx, cy))
            p.setPen(QPen(QColor(0, 0, 0, 120), 6, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            p.drawPath(path)
            p.setPen(QPen(QColor("#3ea6ff"), 3.5, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            p.drawPath(path)

        if self.home:
            s = self._to_screen(*self.home, cx, cy)
            p.setPen(QPen(QColor("white"), 2.5))
            p.setBrush(QColor("#2e7d32"))
            p.drawEllipse(s, 7, 7)

        if self.boat:
            s = self._to_screen(*self.boat, cx, cy)
            hd = self.heading if self.heading is not None else 0.0
            p.save()
            p.translate(s)
            p.rotate(hd)
            poly = QPolygonF([QPointF(0, -18), QPointF(11, 12), QPointF(0, 6), QPointF(-11, 12)])
            p.setPen(QPen(QColor("white"), 2))
            p.setBrush(QColor("#ff9800") if self.heading is not None else QColor("#9e9e9e"))
            p.drawPolygon(poly)
            p.restore()

        self._draw_overlays(p, w, h)
        p.end()

    def _draw_grid(self, p, cx, cy):
        mpp = meters_per_px(self.lat, self.z)
        step_m = nice_scale(mpp * 120)
        step_px = step_m / mpp
        w, h = self.width(), self.height()
        p.setPen(QPen(QColor(255, 255, 255, 28), 1))
        ox = -((cx - w / 2) % step_px)
        oy = -((cy - h / 2) % step_px)
        x = ox
        while x < w:
            p.drawLine(QPointF(x, 0), QPointF(x, h))
            x += step_px
        y = oy
        while y < h:
            p.drawLine(QPointF(0, y), QPointF(w, y))
            y += step_px

    def _draw_overlays(self, p, w, h):
        f = QFont(self.font())
        f.setPointSizeF(10.5)
        p.setFont(f)
        # scale bar
        mpp = meters_per_px(self.lat, self.z)
        m = nice_scale(mpp * 120)
        L = m / mpp
        label = "%d m" % m if m < 1000 else "%g km" % (m / 1000)
        self._panel(p, QRectF(10, h - 40, L + 70, 30))
        p.setPen(QPen(QColor("white"), 3))
        p.drawLine(QPointF(20, h - 25), QPointF(20 + L, h - 25))
        p.drawText(QPointF(28 + L, h - 20), label)
        # north arrow
        c = QPointF(w - 34, 34)
        self._panel(p, QRectF(c.x() - 22, c.y() - 22, 44, 44), radius=22)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#ef5350"))
        p.drawPolygon(QPolygonF([c + QPointF(0, -16), c + QPointF(7, 2), c + QPointF(-7, 2)]))
        p.setPen(QColor("white"))
        p.drawText(QRectF(c.x() - 10, c.y() + 1, 20, 18), Qt.AlignCenter, "N")
        # zoom buttons
        for i, t in enumerate(("+", "−")):
            r = self._zoom_rect(i)
            self._panel(p, r)
            p.setPen(QColor("white"))
            ff = QFont(f)
            ff.setPointSizeF(16)
            p.setFont(ff)
            p.drawText(r, Qt.AlignCenter, t)
        p.setFont(f)
        # follow button
        r = self._follow_rect()
        self._panel(p, r, active=self.follow)
        p.setPen(QColor("white"))
        p.drawText(r, Qt.AlignCenter, "Follow" if self.follow else "Centre (X)")
        # info lines
        if self.info_lines:
            lh = 18
            box = QRectF(10, 10, 250, 10 + lh * len(self.info_lines))
            self._panel(p, box)
            p.setPen(QColor("white"))
            for i, t in enumerate(self.info_lines):
                p.drawText(QPointF(18, 28 + i * lh), t)
        if not self.online:
            p.setPen(QColor("#ffcc80"))
            p.drawText(QPointF(w - 230, h - 64), "offline: cached tiles + grid")

    def _panel(self, p, r, radius=6, active=False):
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(30, 136, 229, 200) if active else QColor(0, 0, 0, 150))
        p.drawRoundedRect(r, radius, radius)

    def _zoom_rect(self, i):
        return QRectF(self.width() - 56, 70 + i * 52, 44, 44)

    def _follow_rect(self):
        return QRectF(self.width() - 112, self.height() - 52, 100, 40)

    # ---- interaction -------------------------------------------------------
    def mousePressEvent(self, e):
        pos = e.position()
        if self._zoom_rect(0).contains(pos):
            self.zoom_by(1)
            return
        if self._zoom_rect(1).contains(pos):
            self.zoom_by(-1)
            return
        if self._follow_rect().contains(pos):
            self.center_on_boat()
            return
        self._drag = pos

    def mouseMoveEvent(self, e):
        if self._drag is None:
            return
        d = e.position() - self._drag
        self._drag = e.position()
        if d.manhattanLength() > 0:
            if self.follow:
                self.follow = False
                self.followChanged.emit(False)
            cx, cy = lonlat_to_px(self.lon, self.lat, self.z)
            self.lon, self.lat = px_to_lonlat(cx - d.x(), cy - d.y(), self.z)
            self.update()

    def mouseReleaseEvent(self, e):
        self._drag = None

    def wheelEvent(self, e):
        self.zoom_by(1 if e.angleDelta().y() > 0 else -1)

    def event(self, e):
        if e.type() == QEvent.Gesture:
            g = e.gesture(Qt.PinchGesture)
            if g:
                self._pinch_acc *= g.scaleFactor()
                if self._pinch_acc > 1.6:
                    self.zoom_by(1)
                    self._pinch_acc = 1.0
                elif self._pinch_acc < 0.6:
                    self.zoom_by(-1)
                    self._pinch_acc = 1.0
                if g.state() == Qt.GestureFinished:
                    self._pinch_acc = 1.0
                return True
        return super().event(e)
