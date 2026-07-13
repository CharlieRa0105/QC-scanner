"""Backends for the linear slider (floor track) driven by a Roboteq BLDC controller.

Same interface pattern as the arm backends, but the slider is a single
PRISMATIC axis measured in METRES.

  * MockSlider     -- simulation (default).
  * RoboteqSlider  -- talks to a Roboteq controller over serial (pyserial).

Interface:
    connect() / disconnect()
    get_position() -> float          # metres
    get_status() -> str
    is_moving() -> bool
    update(dt)
    move(position_m, speed_pct)
    set_power(on) -> bool
    set_drag(on) -> bool             # free-wheel / backdrive for hand teaching
    stop()
"""

import math
import time


# ---------------------------------------------------------------------------
# Mock (simulation) backend
# ---------------------------------------------------------------------------
class MockSlider:
    def __init__(self, track_len_m=3.0, max_speed_mps=0.5, log=print):
        self.track_len = track_len_m
        self.max_speed = max_speed_mps      # m/s at 100 %
        self.log = log
        self.pos = 0.0
        self._target = 0.0
        self._speed_mps = max_speed_mps      # current commanded linear speed (m/s)
        self._moving = False
        self.powered = False
        self.drag = False
        self._t = 0.0
        self._drag_base = 0.0
        self._drag_cyc = -1
        self._cyc_from = 0.0
        self._cyc_to = 0.0

    def connect(self):
        self.log("[MOCK] slider backend ready (no hardware).")

    def disconnect(self):
        self._moving = False

    def get_position(self):
        return self.pos

    def is_moving(self):
        return self._moving

    def get_status(self):
        if self.drag:
            return "drag"
        if not self.powered:
            return "off"
        return "moving" if self._moving else "idle"

    def _step_toward(self, target, speed_mps, dt):
        step = speed_mps * dt
        d = target - self.pos
        if abs(d) <= step:
            self.pos = target
            return True
        self.pos += math.copysign(step, d)
        return False

    def update(self, dt):
        self._t += dt
        if self.drag:
            # move-then-settle (mirrors the arm) so capture-on-settle can trigger
            cyc_len = 2.6
            k = int(self._t / cyc_len)
            if k != self._drag_cyc:
                self._drag_cyc = k
                self._cyc_from = self.pos
                self._cyc_to = self._clamp(self._drag_base + 0.3 * math.sin(1.3 * k))
            phase = (self._t - k * cyc_len) / cyc_len
            if phase < 0.5:
                f = phase / 0.5
                self.pos = self._cyc_from + (self._cyc_to - self._cyc_from) * f
            else:
                self.pos = self._cyc_to
            return
        if not self._moving:
            return
        if self._step_toward(self._target, self._speed_mps, dt):
            self._moving = False

    def _clamp(self, x):
        return max(0.0, min(self.track_len, x))

    def move(self, position_m, speed_mms):
        if not self.powered:
            self.log("[MOCK] slider move rejected: motors off.")
            return False
        if self.drag:
            self.log("[MOCK] slider move rejected: in drag mode.")
            return False
        self._target = self._clamp(position_m)
        # speed arrives as mm/s (shared linear-speed field); slider uses m/s
        self._speed_mps = max(0.02, min(self.max_speed, speed_mms / 1000.0))
        self._moving = True
        return True

    def set_power(self, on):
        self.powered = bool(on)
        if not on:
            self._moving = False
        return self.powered

    def set_drag(self, on):
        if on:
            self.powered = False
            self.drag = True
            self._moving = False
            self._drag_base = self.pos
        else:
            self.drag = False
            self.powered = True
        return self.drag

    def stop(self):
        self._moving = False
        self._target = self.pos


# ---------------------------------------------------------------------------
# Real backend -- Roboteq controller over serial
# ---------------------------------------------------------------------------
class RoboteqSlider:
    """Minimal Roboteq serial driver. Uses closed-loop position mode.

    Roboteq ASCII protocol (see the controller's user manual):
      !P  cc nn   -> go to absolute position (counts) on channel cc
      !G  cc nn   -> set motor command (open loop / speed)
      !MS cc      -> motor stop
      ?C  cc      -> query encoder counter (counts)
    Position <-> metres uses counts_per_metre (set from your mechanics).
    """

    def __init__(self, port="/dev/ttyUSB0", baud=115200, counts_per_m=100000.0,
                 channel=1, track_len_m=3.0, log=print):
        self.port = port
        self.baud = baud
        self.cpm = counts_per_m
        self.ch = channel
        self.track_len = track_len_m
        self.log = log
        self.ser = None
        self._pos = 0.0
        self._status = "off"
        self.powered = False
        self.drag = False

    def connect(self):
        import serial  # pyserial; only needed for the real backend
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        self._status = "idle"
        self.log(f"[ROBOTEQ] opened {self.port} @ {self.baud}")

    def disconnect(self):
        if self.ser:
            try:
                self._send(f"!MS {self.ch}")
            except Exception:  # noqa: BLE001
                pass
            self.ser.close()

    def _send(self, cmd):
        self.ser.write((cmd + "\r").encode("ascii"))

    def _query(self, cmd):
        self.ser.reset_input_buffer()
        self._send(cmd)
        return self.ser.readline().decode("ascii", "ignore").strip()

    def get_position(self):
        return self._pos

    def is_moving(self):
        return self._status == "moving"

    def get_status(self):
        return self._status

    def update(self, dt):
        if not self.ser:
            return
        try:
            resp = self._query(f"?C {self.ch}")     # e.g. 'C=123456'
            if "=" in resp:
                counts = float(resp.split("=")[-1])
                self._pos = counts / self.cpm
        except Exception as e:  # noqa: BLE001
            self._status = f"error:{e}"

    def move(self, position_m, speed_mms):
        pos = max(0.0, min(self.track_len, position_m))
        counts = int(pos * self.cpm)
        try:
            self._send(f"!P {self.ch} {counts}")
            self._status = "moving"
            return True
        except Exception as e:  # noqa: BLE001
            self._status = f"error:{e}"
            return False

    def set_power(self, on):
        # Roboteq is enabled by default; 'off' issues a motor stop.
        if not on:
            self._send(f"!MS {self.ch}")
        self.powered = bool(on)
        self._status = "idle" if on else "off"
        return self.powered

    def set_drag(self, on):
        # Free-wheel: put the controller in open loop at zero command so the
        # axis can be backdriven by hand (only if the drive/mechanics allow it).
        if on:
            self._send(f"!G {self.ch} 0")
            self.drag = True
            self._status = "drag"
        else:
            self.drag = False
            self._status = "idle"
        return self.drag

    def stop(self):
        try:
            self._send(f"!MS {self.ch}")
        except Exception:  # noqa: BLE001
            pass
        self._status = "idle"
