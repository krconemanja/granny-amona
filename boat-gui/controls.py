"""Controller input for the DDBoat GUI.

* Pure helpers (dead zone, expo, differential mix, gyro filter) -- unit tested.
* Controller: reads the Steam Deck (or any SDL game controller) through
  PySDL2, including the built-in gyro + accelerometer when SDL exposes them.
* KeyboardInput: fallback for testing on a laptop (W/S/A/D, held keys).
"""
import math
import os
import time
from dataclasses import dataclass, field

MODES = ("Joystick", "Gyro", "Tank")


# ----------------------------------------------------------------------------
# pure helpers
# ----------------------------------------------------------------------------
def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


def shape(v, deadzone=0.08, expo=0.3):
    """Dead zone + expo curve on a value in [-1, 1]; output in [-1, 1]."""
    v = clamp(v)
    a = abs(v)
    if a <= deadzone:
        return 0.0
    a = (a - deadzone) / (1.0 - deadzone)          # rescale so output starts at 0
    a = (1.0 - expo) * a + expo * a ** 3            # expo: finer near centre
    return math.copysign(a, v)


def mix(throttle, turn, max_speed):
    """Differential drive, same as client.py: turn > 0 = turn right.

    throttle and turn are already scaled to motor units (e.g. -200..200).
    If one side saturates, both are scaled so the turn ratio is preserved.
    """
    left = throttle + turn
    right = throttle - turn
    biggest = max(abs(left), abs(right), max_speed)
    if biggest == 0:
        return 0, 0
    left = int(round(left * max_speed / biggest))
    right = int(round(right * max_speed / biggest))
    return left, right


def accel_roll_deg(ax, ay, az):
    """Steering-wheel roll from SDL accelerometer (m/s^2).

    SDL axes for a handheld: +X right, +Y up, +Z toward the player.
    Positive result = Deck rotated clockwise (right side down) = turn right.
    The wheel turns in the screen plane, so Z (out of the screen) is left
    out: the result does not change with how far the Deck is reclined.
    """
    return math.degrees(math.atan2(-ax, ay))


def accel_pitch_deg(ax, ay, az):
    """Forward tilt from SDL accelerometer (m/s^2).

    Positive = top of the Deck tilted away from the player (push forward).
    Measured from upright; the neutral angle you hold it at is removed by
    TiltFilter.recenter().
    """
    return math.degrees(math.atan2(az, math.hypot(ax, ay)))


class TiltFilter:
    """Complementary filter for roll (steering) and pitch (throttle).

    Gyro for smoothness, accelerometer to cancel drift.
    """

    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.roll = None      # degrees, absolute
        self.pitch = None
        self.roll0 = 0.0      # neutral pose, set by recenter()
        self.pitch0 = 0.0

    def update(self, accel, gyro, dt):
        """accel in m/s^2 (x, y, z), gyro in rad/s (x, y, z), dt in s."""
        acc_roll = accel_roll_deg(*accel)
        acc_pitch = accel_pitch_deg(*accel)
        if self.roll is None or dt <= 0 or dt > 0.5:
            self.roll, self.pitch = acc_roll, acc_pitch
        else:
            # rotation about +Z (toward player) is counter-clockwise -> negative roll
            roll_rate = -math.degrees(gyro[2])
            # rotation about +X (right) brings the top toward the player -> negative pitch
            pitch_rate = -math.degrees(gyro[0])
            a = self.alpha
            self.roll = a * (self.roll + roll_rate * dt) + (1 - a) * acc_roll
            self.pitch = a * (self.pitch + pitch_rate * dt) + (1 - a) * acc_pitch
        return self.value

    @property
    def value(self):
        """(roll, pitch) relative to the neutral pose, degrees."""
        if self.roll is None:
            return 0.0, 0.0
        return self.roll - self.roll0, self.pitch - self.pitch0

    def recenter(self):
        if self.roll is not None:
            self.roll0, self.pitch0 = self.roll, self.pitch


RollFilter = TiltFilter   # backward-compatible name


# ----------------------------------------------------------------------------
# state handed to the GUI each tick
# ----------------------------------------------------------------------------
@dataclass
class ControlState:
    connected: bool = False
    name: str = ""
    lx: float = 0.0          # sticks in [-1, 1], +y = up
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    lt: float = 0.0          # triggers in [0, 1]
    rt: float = 0.0
    roll: float = 0.0        # degrees, from the gyro filter (+ = right side down)
    pitch: float = 0.0       # degrees, from the gyro filter (+ = tilted forward)
    deadman: bool = False    # a back paddle (L4/L5/R4/R5) is held
    has_gyro: bool = False
    pressed: set = field(default_factory=set)   # buttons pressed THIS tick (edges)


BUTTONS = ("A", "B", "X", "Y", "BACK", "L1", "R1", "UP", "DOWN")


def compute_command(state, mode, max_speed, cfg):
    """ControlState -> (throttle, turn, left, right). Throttle/turn in [-1, 1]."""
    dz = cfg.get("deadzone", 0.08)
    expo = cfg.get("expo", 0.3)
    trig = state.rt - state.lt
    if mode == "Tank":
        l = shape(state.ly, dz, expo)
        r = shape(state.ry, dz, expo)
        left = int(round(l * max_speed))
        right = int(round(r * max_speed))
        return (l + r) / 2, (l - r) / 2, left, right

    if mode == "Gyro":
        # dead-man: nothing moves unless a back paddle is held
        if not state.deadman:
            return 0.0, 0.0, 0, 0
        if state.has_gyro:
            full = cfg.get("gyro_full_deg", 45.0)
            gdz = cfg.get("gyro_deadzone_deg", 3.0) / full
            sign = -1.0 if cfg.get("gyro_invert", False) else 1.0
            turn = shape(sign * state.roll / full, gdz, expo)
            tfull = cfg.get("tilt_full_deg", 30.0)
            tdz = cfg.get("tilt_deadzone_deg", 5.0) / tfull
            tsign = -1.0 if cfg.get("tilt_invert", False) else 1.0
            throttle = shape(tsign * state.pitch / tfull, tdz, expo)
        else:
            # Steam Input fallback: "Gyro to joystick" mapped on the right stick
            turn = shape(state.rx, dz, expo)
            throttle = shape(state.ry, dz, expo)
    else:  # Joystick
        stick = shape(state.ly, dz, expo)
        throttle = trig if abs(trig) > abs(stick) else stick
        turn = shape(state.lx, dz, expo)

    throttle = clamp(throttle)
    turn = clamp(turn)
    left, right = mix(throttle * max_speed, turn * max_speed, max_speed)
    return throttle, turn, left, right


# ----------------------------------------------------------------------------
# SDL controller
# ----------------------------------------------------------------------------
class Controller:
    """Polls the first SDL game controller. Safe to use when SDL is missing."""

    def __init__(self):
        self.ok = False
        self.error = ""
        self.pad = None
        self.has_gyro = False
        self.filter = TiltFilter()
        self._prev = set()
        self._last_t = None
        try:
            import sdl2
            self.sdl = sdl2
            sdl2.SDL_SetHint(b"SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", b"1")
            sdl2.SDL_SetHint(b"SDL_GAMECONTROLLER_USE_BUTTON_LABELS", b"1")
            if sdl2.SDL_Init(sdl2.SDL_INIT_GAMECONTROLLER | sdl2.SDL_INIT_SENSOR) != 0:
                raise RuntimeError(sdl2.SDL_GetError().decode())
            self.ok = True
            self._open_first()
        except Exception as e:  # pragma: no cover - depends on host
            self.error = "SDL unavailable: %s" % e

    def _open_first(self):
        sdl2 = self.sdl
        for i in range(sdl2.SDL_NumJoysticks()):
            if sdl2.SDL_IsGameController(i):
                pad = sdl2.SDL_GameControllerOpen(i)
                if pad:
                    self.pad = pad
                    self.has_gyro = False
                    if (sdl2.SDL_GameControllerHasSensor(pad, sdl2.SDL_SENSOR_GYRO)
                            and sdl2.SDL_GameControllerHasSensor(pad, sdl2.SDL_SENSOR_ACCEL)):
                        a = sdl2.SDL_GameControllerSetSensorEnabled(pad, sdl2.SDL_SENSOR_GYRO, True)
                        b = sdl2.SDL_GameControllerSetSensorEnabled(pad, sdl2.SDL_SENSOR_ACCEL, True)
                        self.has_gyro = (a == 0 and b == 0)
                    self.filter = TiltFilter()
                    return

    def _close(self):
        if self.pad:
            self.sdl.SDL_GameControllerClose(self.pad)
        self.pad = None
        self.has_gyro = False

    @property
    def name(self):
        if not self.pad:
            return ""
        n = self.sdl.SDL_GameControllerName(self.pad)
        return n.decode(errors="ignore") if n else "Controller"

    def rumble(self, ms=300):
        if self.pad:
            try:
                self.sdl.SDL_GameControllerRumble(self.pad, 0xC000, 0xC000, ms)
            except Exception:
                pass

    def recenter_gyro(self):
        self.filter.recenter()

    def poll(self):
        st = ControlState()
        if not self.ok:
            return st
        sdl2 = self.sdl
        ev = sdl2.SDL_Event()
        while sdl2.SDL_PollEvent(ev):
            if ev.type == sdl2.SDL_CONTROLLERDEVICEADDED and not self.pad:
                self._open_first()
            elif ev.type == sdl2.SDL_CONTROLLERDEVICEREMOVED:
                if self.pad and not sdl2.SDL_GameControllerGetAttached(self.pad):
                    self._close()
        sdl2.SDL_GameControllerUpdate()
        if not self.pad:
            return st
        pad = self.pad
        ax = lambda a: sdl2.SDL_GameControllerGetAxis(pad, a) / 32767.0
        st.connected = True
        st.name = self.name
        st.lx = clamp(ax(sdl2.SDL_CONTROLLER_AXIS_LEFTX))
        st.ly = clamp(-ax(sdl2.SDL_CONTROLLER_AXIS_LEFTY))
        st.rx = clamp(ax(sdl2.SDL_CONTROLLER_AXIS_RIGHTX))
        st.ry = clamp(-ax(sdl2.SDL_CONTROLLER_AXIS_RIGHTY))
        st.lt = clamp(ax(sdl2.SDL_CONTROLLER_AXIS_TRIGGERLEFT), 0, 1)
        st.rt = clamp(ax(sdl2.SDL_CONTROLLER_AXIS_TRIGGERRIGHT), 0, 1)

        b = sdl2
        mapping = {
            "A": b.SDL_CONTROLLER_BUTTON_A, "B": b.SDL_CONTROLLER_BUTTON_B,
            "X": b.SDL_CONTROLLER_BUTTON_X, "Y": b.SDL_CONTROLLER_BUTTON_Y,
            "BACK": b.SDL_CONTROLLER_BUTTON_BACK,
            "L1": b.SDL_CONTROLLER_BUTTON_LEFTSHOULDER,
            "R1": b.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER,
            "UP": b.SDL_CONTROLLER_BUTTON_DPAD_UP, "DOWN": b.SDL_CONTROLLER_BUTTON_DPAD_DOWN,
        }
        down = {k for k, v in mapping.items() if sdl2.SDL_GameControllerGetButton(pad, v)}
        # dead-man: any back paddle (Deck L4/R4/L5/R5). R3 (right stick click)
        # also counts, for Steam Input layouts that map a paddle onto it.
        paddles = (b.SDL_CONTROLLER_BUTTON_PADDLE1, b.SDL_CONTROLLER_BUTTON_PADDLE2,
                   b.SDL_CONTROLLER_BUTTON_PADDLE3, b.SDL_CONTROLLER_BUTTON_PADDLE4,
                   b.SDL_CONTROLLER_BUTTON_RIGHTSTICK)
        st.deadman = any(sdl2.SDL_GameControllerGetButton(pad, p) for p in paddles)
        st.pressed = down - self._prev
        self._prev = down

        if self.has_gyro:
            import ctypes
            g = (ctypes.c_float * 3)()
            a = (ctypes.c_float * 3)()
            okg = sdl2.SDL_GameControllerGetSensorData(pad, sdl2.SDL_SENSOR_GYRO, g, 3) == 0
            oka = sdl2.SDL_GameControllerGetSensorData(pad, sdl2.SDL_SENSOR_ACCEL, a, 3) == 0
            now = time.monotonic()
            dt = 0 if self._last_t is None else now - self._last_t
            self._last_t = now
            if okg and oka and any(a):
                self.filter.update(tuple(a), tuple(g), dt)
            st.roll, st.pitch = self.filter.value
            st.has_gyro = True
        return st


class KeyboardInput:
    """Held-key fallback (Qt reports releases, unlike a terminal).

    W/S throttle, A/D turn. Gyro mode: Q/E roll left/right, W/S tilt
    forward/back, hold Shift as the back-paddle dead-man.
    Space = B (stop), Enter = A (arm), M = Y (mode), C = X (centre map).
    """
    KEYMAP = {"Space": "B", "Return": "A", "Enter": "A", "M": "Y", "C": "X",
              "R": "L1", "Backspace": "BACK", "Up": "UP", "Down": "DOWN"}

    def __init__(self):
        self.held = set()
        self.edges = set()
        self.roll = 0.0
        self.pitch = 0.0

    def press(self, name):
        if name in self.KEYMAP and name not in self.held:
            self.edges.add(self.KEYMAP[name])
        self.held.add(name)

    def release(self, name):
        self.held.discard(name)

    def clear(self):
        self.held.clear()

    @property
    def active(self):
        return bool(self.held or self.edges)

    def poll(self, dt=0.02):
        st = ControlState(connected=False, name="Keyboard")
        st.ly = (1.0 if "W" in self.held else 0) - (1.0 if "S" in self.held else 0)
        st.lx = (1.0 if "D" in self.held else 0) - (1.0 if "A" in self.held else 0)
        st.ry = st.ly
        if "E" in self.held:
            self.roll = min(60.0, self.roll + 90 * dt)
        elif "Q" in self.held:
            self.roll = max(-60.0, self.roll - 90 * dt)
        st.roll = self.roll
        if "W" in self.held:
            self.pitch = min(45.0, self.pitch + 60 * dt)
        elif "S" in self.held:
            self.pitch = max(-45.0, self.pitch - 60 * dt)
        else:
            self.pitch *= max(0.0, 1 - 4 * dt)          # spring back to neutral
        st.pitch = self.pitch
        st.deadman = "Shift" in self.held
        st.has_gyro = True
        st.rt = 1.0 if "W" in self.held else 0.0
        st.lt = 1.0 if "S" in self.held else 0.0
        st.pressed = set(self.edges)
        self.edges.clear()
        return st

    def recenter(self):
        self.roll = 0.0
        self.pitch = 0.0
