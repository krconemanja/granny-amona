#!/usr/bin/env python3
"""DDBoat Steam Deck controller -- main window.

Usage:
    python3 boat_gui.py [--host 172.20.25.205] [--port 5000] [--fullscreen]
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtCore import Qt, QTimer, QSettings, QPointF, QRectF
from PySide6.QtGui import QColor, QPainter, QPen, QKeySequence, QShortcut
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDialog,
                               QDialogButtonBox, QDoubleSpinBox, QFileDialog,
                               QFormLayout, QHBoxLayout, QLabel, QMainWindow,
                               QPushButton, QScrollArea, QSizePolicy, QSpinBox, QVBoxLayout,
                               QWidget, QFrame)

from controls import MODES, Controller, KeyboardInput, ControlState, compute_command
from link import Link, parse_host
from map_widget import MapWidget
from telemetry import TelemetryPanel, Tracker

LOG_DIR = os.path.expanduser("~/boat-gui-logs")
STATUS_COLORS = {"Disconnected": "#757575", "Connecting": "#f9a825",
                 "Connected": "#2e7d32", "Lost": "#c62828"}

QSS = """
QWidget { background: #14181c; color: #e8eaed; font-size: 15px; }
QLabel { background: transparent; }
QFrame#bar { background: #1b2026; border: none; }
QFrame#telemetry { background: #1b2026; border-radius: 8px; }
QLabel#theader { color: #90caf9; font-weight: 600; padding-top: 4px; font-size: 14px; }
QLabel#tlabel { color: #9aa0a6; font-size: 13px; }
QLabel#tvalue { font-family: monospace; font-size: 13px; }
QLabel#chip { border-radius: 14px; padding: 4px 14px; font-weight: 600; }
QLabel#msg { color: #ffcc80; }
QComboBox, QSpinBox, QDoubleSpinBox { background: #262c33; border: 1px solid #3c444d;
    border-radius: 6px; padding: 6px 8px; min-height: 28px; }
QPushButton { background: #262c33; border: 1px solid #3c444d; border-radius: 8px;
    padding: 8px 16px; min-height: 28px; font-weight: 600; }
QPushButton:pressed { background: #33404d; }
QPushButton#connect { background: #1565c0; border-color: #1e88e5; }
QPushButton#connect[connected="true"] { background: #37474f; border-color: #607d8b; }
QPushButton#stop { background: #b71c1c; border: 2px solid #ef5350; font-size: 18px;
    min-width: 130px; min-height: 40px; }
QScrollArea { border: none; }
"""


class AxisBar(QWidget):
    """Small horizontal gauge for a value in [-1, 1] (or [0, 1])."""

    def __init__(self, signed=True, parent=None):
        super().__init__(parent)
        self.v = 0.0
        self.signed = signed
        self.setFixedSize(140, 18)

    def set_value(self, v):
        self.v = v
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.setPen(QPen(QColor("#3c444d"), 1))
        p.setBrush(QColor("#262c33"))
        p.drawRoundedRect(QRectF(0.5, 0.5, w - 1, h - 1), h / 2, h / 2)
        mid = w / 2
        x = mid + self.v * (w / 2 - 3)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#3ea6ff") if self.v >= 0 else QColor("#ef5350"))
        p.drawRoundedRect(QRectF(min(x, mid), 3, abs(x - mid), h - 6), 4, 4)
        p.setPen(QPen(QColor(255, 255, 255, 120), 1))
        p.drawLine(QPointF(mid, 2), QPointF(mid, h - 2))
        p.end()


class SettingsDialog(QDialog):
    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Settings")
        f = QFormLayout(self)
        self.max_cap = QSpinBox(); self.max_cap.setRange(25, 255); self.max_cap.setValue(cfg["max_cap"])
        self.dz = QDoubleSpinBox(); self.dz.setRange(0, 0.5); self.dz.setSingleStep(0.01); self.dz.setValue(cfg["deadzone"])
        self.expo = QDoubleSpinBox(); self.expo.setRange(0, 1); self.expo.setSingleStep(0.05); self.expo.setValue(cfg["expo"])
        self.full = QDoubleSpinBox(); self.full.setRange(10, 90); self.full.setValue(cfg["gyro_full_deg"])
        self.gdz = QDoubleSpinBox(); self.gdz.setRange(0, 20); self.gdz.setValue(cfg["gyro_deadzone_deg"])
        self.inv = QCheckBox(); self.inv.setChecked(cfg["gyro_invert"])
        self.tfull = QDoubleSpinBox(); self.tfull.setRange(10, 60); self.tfull.setValue(cfg["tilt_full_deg"])
        self.tdz = QDoubleSpinBox(); self.tdz.setRange(0, 20); self.tdz.setValue(cfg["tilt_deadzone_deg"])
        self.tinv = QCheckBox(); self.tinv.setChecked(cfg["tilt_invert"])
        self.tank = QCheckBox(); self.tank.setChecked(cfg["tank_enabled"])
        f.addRow("Max speed cap (0-255)", self.max_cap)
        f.addRow("Stick dead zone", self.dz)
        f.addRow("Expo", self.expo)
        f.addRow("Gyro: tilt for full turn (°)", self.full)
        f.addRow("Gyro dead zone (°)", self.gdz)
        f.addRow("Gyro: invert steering", self.inv)
        f.addRow("Gyro: tilt for full throttle (°)", self.tfull)
        f.addRow("Gyro: throttle dead zone (°)", self.tdz)
        f.addRow("Gyro: invert throttle tilt", self.tinv)
        f.addRow("Allow Tank mode (Y cycles 3 modes)", self.tank)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        f.addRow(bb)

    def values(self):
        return {"max_cap": self.max_cap.value(), "deadzone": self.dz.value(),
                "expo": self.expo.value(), "gyro_full_deg": self.full.value(),
                "gyro_deadzone_deg": self.gdz.value(), "gyro_invert": self.inv.isChecked(),
                "tilt_full_deg": self.tfull.value(), "tilt_deadzone_deg": self.tdz.value(),
                "tilt_invert": self.tinv.isChecked(),
                "tank_enabled": self.tank.isChecked()}


class MainWindow(QMainWindow):
    DEFAULTS = {"max_cap": 255, "deadzone": 0.08, "expo": 0.3, "gyro_full_deg": 45.0,
                "gyro_deadzone_deg": 3.0, "gyro_invert": False, "tilt_full_deg": 30.0,
                "tilt_deadzone_deg": 5.0, "tilt_invert": False, "tank_enabled": False}

    def __init__(self, args):
        super().__init__()
        self.setWindowTitle("DDBoat — Steam Deck control")
        self.settings = QSettings("ddboat", "steamdeck-gui")
        self.cfg = {}
        for k, v in self.DEFAULTS.items():
            val = self.settings.value(k, v)
            if isinstance(v, bool):
                val = val in (True, "true", "1", 1)
            self.cfg[k] = type(v)(val)
        self.max_speed = min(int(self.settings.value("max_speed", 200)), self.cfg["max_cap"])
        self.mode = "Joystick"
        self.armed = False
        self.throttle = self.turn = 0.0
        self.sent = (0, 0)
        self.rtt = None
        self.link_state = "Disconnected"
        self._msg_until = 0
        self._had_pad = False

        self.controller = Controller()
        self.keyboard = KeyboardInput()
        self.tracker = Tracker(log_dir=LOG_DIR)
        self.link = Link()
        self.link.status.connect(self.on_status)
        self.link.telemetry.connect(self.on_telemetry)
        self.link.rtt.connect(self.on_rtt)
        self.link.sent.connect(self.on_sent)

        self._build_ui(args)
        self.on_status("Disconnected", "")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)          # 50 Hz input
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh)
        self.ui_timer.start(100)      # 10 Hz display

        if not self.controller.ok:
            self.flash(self.controller.error + " — keyboard only", 8)
        QApplication.instance().applicationStateChanged.connect(self.on_app_state)

    # ---- UI ----------------------------------------------------------------
    def _build_ui(self, args):
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        top = QFrame(); top.setObjectName("bar")
        tl = QHBoxLayout(top)
        tl.setContentsMargins(10, 8, 10, 8)
        self.host = QComboBox()
        self.host.setEditable(True)
        self.host.setMinimumWidth(190)
        hist = self.settings.value("history", []) or []
        if isinstance(hist, str):
            hist = [hist]
        self.host.addItems(hist)
        self.host.setEditText(args.host or (hist[0] if hist else "172.20.25.2"))
        self.port = QSpinBox()
        self.port.setRange(1, 65535)
        self.port.setValue(args.port or int(self.settings.value("port", 5000)))
        self.port.setMinimumWidth(90)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setObjectName("connect")
        self.btn_connect.clicked.connect(self.toggle_connect)
        self.chip_status = self._chip()
        self.chip_mode = self._chip()
        self.chip_arm = self._chip()
        self.chip_arm.mousePressEvent = lambda e: self.toggle_arm()
        self.chip_mode.mousePressEvent = lambda e: self.cycle_mode()
        btn_gpx = QPushButton("GPX")
        btn_gpx.clicked.connect(self.export_gpx)
        btn_clear = QPushButton("Clear route")
        btn_clear.clicked.connect(self.clear_route)
        btn_set = QPushButton("⚙")
        btn_set.clicked.connect(self.open_settings)
        tl.addWidget(QLabel("IP"))
        tl.addWidget(self.host)
        tl.addWidget(QLabel("Port"))
        tl.addWidget(self.port)
        tl.addWidget(self.btn_connect)
        tl.addWidget(self.chip_status, 1)
        tl.addWidget(self.chip_mode)
        tl.addWidget(self.chip_arm)
        tl.addWidget(btn_clear)
        tl.addWidget(btn_gpx)
        tl.addWidget(btn_set)
        root.addWidget(top)

        mid = QHBoxLayout()
        mid.setContentsMargins(8, 8, 8, 8)
        mid.setSpacing(8)
        self.map = MapWidget()
        mid.addWidget(self.map, 1)
        self.panel = TelemetryPanel()
        sc = QScrollArea()
        sc.setWidget(self.panel)
        sc.setWidgetResizable(True)
        sc.setFixedWidth(400)
        sc.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        mid.addWidget(sc)
        root.addLayout(mid, 1)

        bot = QFrame(); bot.setObjectName("bar")
        bl = QHBoxLayout(bot)
        bl.setContentsMargins(12, 6, 12, 6)
        bl.addWidget(QLabel("Throttle"))
        self.bar_thr = AxisBar()
        bl.addWidget(self.bar_thr)
        bl.addSpacing(10)
        bl.addWidget(QLabel("Turn"))
        self.bar_turn = AxisBar()
        bl.addWidget(self.bar_turn)
        bl.addSpacing(10)
        self.lbl_max = QLabel()
        bl.addWidget(self.lbl_max)
        bl.addSpacing(10)
        self.lbl_msg = QLabel()
        self.lbl_msg.setObjectName("msg")
        # long messages must never widen the window past the Deck's 1280 px
        self.lbl_msg.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.lbl_msg.setMinimumWidth(120)
        bl.addWidget(self.lbl_msg, 1)
        hint = QLabel("A arm · B stop · Y mode · X centre · L1 zero · paddle = dead-man")
        hint.setStyleSheet("color:#9aa0a6; font-size:13px;")
        bl.addWidget(hint)
        self.btn_stop = QPushButton("STOP (B)")
        self.btn_stop.setObjectName("stop")
        self.btn_stop.clicked.connect(self.emergency_stop)
        bl.addWidget(self.btn_stop)
        root.addWidget(bot)

        self.setCentralWidget(central)
        self.resize(1280, 800)
        QShortcut(QKeySequence("F11"), self, activated=self.toggle_fullscreen)

    def _chip(self):
        l = QLabel()
        l.setObjectName("chip")
        l.setAlignment(Qt.AlignCenter)
        return l

    def _paint_chip(self, chip, text, color):
        chip.setText(text)
        chip.setStyleSheet("background:%s; color:white;" % color)

    def toggle_fullscreen(self):
        self.showNormal() if self.isFullScreen() else self.showFullScreen()

    def flash(self, text, seconds=3):
        self.lbl_msg.setText(text)
        self._msg_until = time.monotonic() + seconds

    # ---- connection --------------------------------------------------------
    def toggle_connect(self):
        if self.link_state == "Disconnected":
            host, port = parse_host(self.host.currentText())
            if port:
                self.port.setValue(port)
            port = self.port.value()
            if not host:
                self.flash("Enter the boat IP")
                return
            hist = [host] + [h for h in (self.settings.value("history", []) or []) if h != host]
            self.settings.setValue("history", hist[:5])
            self.settings.setValue("port", port)
            self.host.clear()
            self.host.addItems(hist[:5])
            self.host.setEditText(host)
            self.disarm("connecting")
            self.link.connect_to(host, port)
        else:
            self.disarm()
            self.link.disconnect()

    def on_status(self, state, detail):
        prev = self.link_state
        self.link_state = state
        label = state
        if state == "Connected" and self.rtt is not None:
            label = "Connected · %.0f ms" % self.rtt
        elif detail and state != "Connected":
            label = "%s · %s" % (state, detail)
        self._paint_chip(self.chip_status, label, STATUS_COLORS.get(state, "#757575"))
        connected = state != "Disconnected"
        self.btn_connect.setText("Disconnect" if connected else "Connect")
        self.btn_connect.setProperty("connected", "true" if connected else "false")
        self.btn_connect.style().unpolish(self.btn_connect)
        self.btn_connect.style().polish(self.btn_connect)
        self.host.setEnabled(not connected)
        self.port.setEnabled(not connected)
        if state == "Lost" and prev != "Lost":
            self.disarm("link lost")
            self.controller.rumble(500)
        if state == "Connected" and prev != "Connected":
            self.rtt = None

    def on_rtt(self, ms):
        self.rtt = ms
        if self.link_state == "Connected":
            self._paint_chip(self.chip_status, "Connected · %.0f ms" % ms, STATUS_COLORS["Connected"])

    def on_sent(self, l, r):
        self.sent = (l, r)
        self.tracker.add_command(l, r)

    def on_telemetry(self, d):
        self.tracker.ingest(d)

    # ---- pilot actions -----------------------------------------------------
    def toggle_arm(self):
        if self.armed:
            self.disarm()
        elif abs(self.throttle) > 0.02 or abs(self.turn) > 0.02:
            self.flash("Centre the sticks / release triggers to arm")
        elif self.link_state != "Connected":
            self.flash("Connect first")
        else:
            self.armed = True
            self.flash("ARMED", 1.5)

    def disarm(self, why=""):
        if self.armed and why:
            self.flash("Disarmed: " + why)
        self.armed = False

    def emergency_stop(self):
        self.disarm("STOP")
        self.link.emergency_stop()

    def cycle_mode(self):
        if abs(self.throttle) > 0.05:
            self.flash("Release throttle to change mode")
            return
        modes = list(MODES) if self.cfg["tank_enabled"] else ["Joystick", "Gyro"]
        i = modes.index(self.mode) if self.mode in modes else -1
        self.mode = modes[(i + 1) % len(modes)]
        if self.mode == "Gyro":
            self.controller.recenter_gyro()
            self.keyboard.recenter()
            if not self.controller.has_gyro and self.controller.pad:
                self.flash("No raw gyro: using right stick (map Steam Input gyro → right stick)", 6)
            else:
                self.flash("Gyro: hold a back paddle, tilt forward to go, roll to steer", 4)

    def clear_route(self):
        self.tracker.clear_route()

    def export_gpx(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        default = os.path.join(LOG_DIR, time.strftime("route_%Y%m%d_%H%M%S.gpx"))
        path, _ = QFileDialog.getSaveFileName(self, "Export route", default, "GPX (*.gpx)")
        if path:
            n = self.tracker.export_gpx(path)
            self.flash("Saved %d points to %s" % (n, os.path.basename(path)))

    def open_settings(self):
        dlg = SettingsDialog(self.cfg, self)
        if dlg.exec():
            self.cfg.update(dlg.values())
            for k, v in self.cfg.items():
                self.settings.setValue(k, v)
            self.max_speed = min(self.max_speed, self.cfg["max_cap"])
            if self.mode == "Tank" and not self.cfg["tank_enabled"]:
                self.mode = "Joystick"

    # ---- input loop (50 Hz) ------------------------------------------------
    def tick(self):
        st = self.controller.poll()
        if self._had_pad and not st.connected:
            self.disarm("controller disconnected")
        self._had_pad = st.connected
        kb = self.keyboard.poll(0.02)
        if self.keyboard.held or not st.connected:
            pressed = st.pressed | kb.pressed
            st = kb if (self.keyboard.held or not st.connected) else st
            st.pressed = pressed
            self.input_name = "Keyboard" + ("" if not self.controller.pad else " (+pad)")
        else:
            st.pressed |= kb.pressed
            self.input_name = st.name + (" + gyro" if st.has_gyro else "")
        self.state = st

        for b in st.pressed:
            if b == "A":
                self.toggle_arm()
            elif b == "B":
                self.emergency_stop()
            elif b == "Y":
                self.cycle_mode()
            elif b == "X":
                self.map.center_on_boat()
            elif b == "L1":
                self.controller.recenter_gyro()
                self.keyboard.recenter()
                self.flash("Gyro re-centred", 1.5)
            elif b == "UP":
                self.max_speed = min(self.cfg["max_cap"], self.max_speed + 25)
                self.settings.setValue("max_speed", self.max_speed)
            elif b == "DOWN":
                self.max_speed = max(25, self.max_speed - 25)
                self.settings.setValue("max_speed", self.max_speed)
            elif b == "BACK":
                self.clear_route()

        self.throttle, self.turn, left, right = compute_command(st, self.mode, self.max_speed, self.cfg)
        if not self.armed:
            left = right = 0
        self.link.set_command(left, right)

    testing = False

    def on_app_state(self, state):
        if self.testing:
            return
        if state != Qt.ApplicationActive:
            self.keyboard.clear()
            self.disarm("window lost focus")

    def keyPressEvent(self, e):
        if not e.isAutoRepeat():
            self.keyboard.press(QKeySequence(e.key()).toString())
        super().keyPressEvent(e)

    def keyReleaseEvent(self, e):
        if not e.isAutoRepeat():
            self.keyboard.release(QKeySequence(e.key()).toString())
        super().keyReleaseEvent(e)

    # ---- display loop (10 Hz) ----------------------------------------------
    def refresh(self):
        st = getattr(self, "state", ControlState())
        if self.mode == "Gyro":
            held = getattr(st, "deadman", False)
            self._paint_chip(self.chip_mode, "Gyro · " + ("paddle held" if held else "hold paddle"),
                             "#00695c" if held else "#37474f")
        else:
            self._paint_chip(self.chip_mode, "Mode: " + self.mode, "#37474f")
        self._paint_chip(self.chip_arm, "ARMED" if self.armed else "Disarmed",
                         "#e65100" if self.armed else "#455a64")
        self.bar_thr.set_value(self.throttle)
        self.bar_turn.set_value(self.turn)
        self.lbl_max.setText("Max speed %d" % self.max_speed)
        if self._msg_until and time.monotonic() > self._msg_until:
            self.lbl_msg.setText("")
            self._msg_until = 0

        tr = self.tracker
        if tr.position:
            self.map.set_boat(tr.position[0], tr.position[1], tr.heading)
        self.map.route = tr.route
        self.map.home = tr.home
        lines = []
        if tr.position:
            lines.append("%.6f, %.6f" % tr.position)
            lines.append("Travelled %.0f m" % tr.distance)
            hv = tr.home_vector()
            if hv:
                lines.append("Home %.0f m @ %03.0f°" % hv)
        else:
            lines.append("No GPS fix yet")
        self.map.info_lines = lines
        self.map.update()

        pilot = {"mode": self.mode, "throttle": self.throttle, "turn": self.turn,
                 "roll": st.roll, "pitch": st.pitch, "deadman": st.deadman,
                 "has_gyro": st.has_gyro,
                 "input": getattr(self, "input_name", "") or "none", "sent": self.sent}
        self.panel.refresh(tr, pilot, {"rtt": self.rtt, "lost": self.link.lost_count})

    def closeEvent(self, e):
        self.link.emergency_stop()
        time.sleep(0.15)
        self.link.disconnect()
        super().closeEvent(e)


def main():
    ap = argparse.ArgumentParser(description="DDBoat Steam Deck GUI")
    ap.add_argument("--host", help="boat IP (default: last used)")
    ap.add_argument("--port", type=int, help="boat port (default: last used or 5000)")
    ap.add_argument("--fullscreen", action="store_true")
    ap.add_argument("--connect", action="store_true", help="connect on start")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    args = ap.parse_args()

    app = QApplication(sys.argv)
    from PySide6.QtNetwork import QNetworkProxyFactory
    QNetworkProxyFactory.setUseSystemConfiguration(True)   # honour http(s)_proxy if set
    app.setApplicationName("DDBoat Control")
    app.setStyleSheet(QSS)
    w = MainWindow(args)
    if args.fullscreen:
        w.showFullScreen()
    else:
        w.show()
    if args.connect:
        QTimer.singleShot(200, w.toggle_connect)
    if args.screenshot:
        # test hook: arm and hold W+D (forward + right) to drive the --sim boat
        w.testing = True
        QTimer.singleShot(1500, w.toggle_arm)
        if os.environ.get("TEST_GYRO"):
            QTimer.singleShot(1000, w.cycle_mode)
            QTimer.singleShot(1800, lambda: w.keyboard.held.update({"W", "E", "Shift"}))
            QTimer.singleShot(2600, lambda: w.keyboard.held.discard("E"))
        else:
            QTimer.singleShot(1800, lambda: w.keyboard.held.update({"W"}))
        QTimer.singleShot(6000, lambda: w.keyboard.held.add("D"))
        QTimer.singleShot(8500, lambda: w.keyboard.held.discard("D"))
        def shot():
            w.grab().save(args.screenshot)
            app.quit()
        QTimer.singleShot(int(os.environ.get("SHOT_DELAY_MS", "8000")), shot)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
