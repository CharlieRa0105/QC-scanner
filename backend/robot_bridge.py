"""
robot_bridge.py

Connection layer between the operator console and the ROKAE xMate SR5.

The robot-driver code is taken from the project's ROS 2 arm driver package
(`ros2_ws/src/sr5_arm_driver`): its backend classes are pure-Python (no rclpy),
so the console reuses the SAME driver implementation that the ROS 2 `ArmDriver`
node uses — one source of truth for talking to the arm:

  * real: `sr5_arm_driver.backends.RokaeArm`  -> xCore SDK (Release/linux/*.so)
  * mock: `sr5_arm_driver.backends.MockArm`   -> pure-Python simulation

This bridge is a thin adapter: it selects the backend, owns the single SDK
session (the SDK allows only one TCP session to the controller), serialises
access, and formats status/telemetry for the HTTP API.

Motion:
  The bridge exposes the arm's motion commands (power, drag/teach, jog, stop,
  clear-alarm) so the operator console can drive the physical SR5 directly.
  Every motion call is gated on an open connection AND the master motion
  switch QC_ALLOW_MOTION (default on) -- set it to 0 to make the whole bridge
  read-only again without touching the UI. Jog moves are absolute joint
  targets; there is no continuous jog, so a dropped request can't leave the
  arm driving.

Env:
  QC_ROBOT_IP     default 192.168.2.160   SR5 address
  QC_SDK_PATH     default ~/rokae_sdk     Linux xCore SDK root (contains Release/linux/)
  QC_ALLOW_MOTION default 1               master motion switch (0 => read-only)
  QC_JOG_SPEED    default 60              default jog end-effector speed, mm/s
"""

import math
import os
import subprocess
import sys
import threading
from pathlib import Path

DEFAULT_IP = os.environ.get("QC_ROBOT_IP", "192.168.2.160")
SDK_ROOT = os.environ.get("QC_SDK_PATH") or os.path.expanduser("~/rokae_sdk")
# Master motion switch. Any value other than 0/false/no leaves motion enabled.
ALLOW_MOTION = os.environ.get("QC_ALLOW_MOTION", "1").lower() not in ("0", "false", "no", "")
DEFAULT_JOG_SPEED = float(os.environ.get("QC_JOG_SPEED", "60"))  # mm/s end-effector

# Make the ROS 2 arm-driver package importable (its backends are plain Python).
_ARM_PKG = Path(__file__).resolve().parent.parent / "ros2_ws" / "src" / "sr5_arm_driver"
if str(_ARM_PKG) not in sys.path:
    sys.path.insert(0, str(_ARM_PKG))
from sr5_arm_driver.backends import RokaeArm  # noqa: E402  (project driver code)

# SR5 joint labels for the UI (6 revolute joints; the rail is a separate axis,
# driven by the RailDriver in ros2_ws/src/rail_driver).
JOINT_NAMES = ["J1 · base", "J2 · shoulder", "J3 · elbow",
               "J4 · wrist 1", "J5 · wrist 2", "J6 · wrist 3"]


def _ping(ip, timeout_s=1):
    """True if the host answers a single ICMP echo within timeout_s."""
    try:
        r = subprocess.run(
            ["ping", "-c", "1", "-W", str(int(timeout_s)), ip],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=timeout_s + 1,
        )
        return r.returncode == 0
    except Exception:
        return False


class RobotBridge:
    """Owns a single SR5 connection (via the project's arm driver backend) and
    serialises access. Read-only w.r.t. motion."""

    def __init__(self):
        self._lock = threading.Lock()
        self._arm = None
        self._kind = None          # 'real' | 'mock' | None
        self._ip = DEFAULT_IP
        self._connected = False
        self._note = ""

    # -- backend logging sink (capture the last driver message for the UI note) --
    def _blog(self, msg):
        self._note = str(msg)

    def connect(self, ip=None):
        """Open a connection to the REAL SR5. There is no mock — if the arm
        isn't reachable / the SDK can't connect, we stay honestly disconnected.
        A fresh connect replaces any existing session."""
        with self._lock:
            self._ip = ip or self._ip or DEFAULT_IP
            self._teardown_locked()

            # Cheap reachability gate first so a missing arm fails fast (no long
            # SDK timeout) and reads as plainly disconnected.
            if not _ping(self._ip):
                self._kind, self._connected = None, False
                self._note = f"arm not reachable at {self._ip}"
                return self._status_locked()

            try:
                arm = RokaeArm(ip=self._ip, sdk_root=SDK_ROOT, log=self._blog)
                arm.connect()                           # imports SDK + opens session
                self._arm, self._kind, self._connected = arm, "real", True
                self._note = ""
            except Exception as e:  # noqa: BLE001
                self._arm, self._kind, self._connected = None, None, False
                self._note = f"connect failed: {e}"
            return self._status_locked()

    def disconnect(self):
        with self._lock:
            self._teardown_locked()
            return self._status_locked()

    def _teardown_locked(self):
        if self._arm is not None:
            try:
                self._arm.disconnect()
            except Exception:
                pass
        self._arm = None
        self._connected = False

    def status(self):
        with self._lock:
            return self._status_locked()

    def _status_locked(self):
        s = {"connected": self._connected, "kind": self._kind,
             "ip": self._ip, "note": self._note}
        if self._connected and self._arm is not None:
            try:
                info = self._arm.device_info()
                s["info"] = {"id": info.get("id"), "type": info.get("type"),
                             "version": info.get("version"), "jointNum": info.get("joint_num")}
                s["power"] = info.get("power")
                s["mode"] = info.get("mode")
                s["sdkVersion"] = info.get("sdk_version")
            except Exception as e:  # noqa: BLE001
                s["note"] = (self._note + "; " if self._note else "") + f"status read error: {e}"
        return s

    def joints(self):
        """Live joint state, angles in DEGREES for the UI. Read-only."""
        with self._lock:
            if not (self._connected and self._arm is not None):
                return {"connected": False, "joints": []}
            try:
                self._arm.update(0.0)                   # refresh cache (real polls live)
                pos = self._arm.get_joints()
                vel = self._arm.get_velocities()
                tq = self._arm.get_torques()
            except Exception as e:  # noqa: BLE001
                return {"connected": True, "joints": [], "error": str(e)}
            out = []
            for i, p in enumerate(pos):
                out.append({
                    "name": JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"J{i+1}",
                    "deg": round(math.degrees(p), 2),
                    "vel": round(vel[i], 3) if i < len(vel) else 0.0,
                    "torque": round(tq[i], 3) if i < len(tq) else 0.0,
                })
            return {"connected": True, "kind": self._kind, "joints": out}

    # ------------------------------------------------------------------
    # Motion commands (drive the physical SR5).
    #
    # Every command runs through _motion_locked(), which enforces the two
    # safety gates -- master switch + live connection -- before delegating to
    # the backend, then returns the fresh status dict (with ok/action/error)
    # so the caller updates its UI from the arm's real state.
    # ------------------------------------------------------------------
    def _motion_locked(self, action, fn):
        """Run motion callable `fn` under the gates, tagging the returned
        status. Assumes self._lock is held."""
        if not ALLOW_MOTION:
            s = self._status_locked()
            s.update({"ok": False, "action": action,
                      "error": "motion disabled (QC_ALLOW_MOTION=0)"})
            return s
        if not (self._connected and self._arm is not None):
            s = self._status_locked()
            s.update({"ok": False, "action": action, "error": "not connected"})
            return s
        try:
            fn(self._arm)
            self._note = ""
            s = self._status_locked()
            s.update({"ok": True, "action": action})
            return s
        except Exception as e:  # noqa: BLE001
            self._note = f"{action} failed: {e}"
            s = self._status_locked()
            s.update({"ok": False, "action": action, "error": str(e)})
            return s

    def set_power(self, on):
        """Energise (True) or de-energise (False) the motors."""
        with self._lock:
            return self._motion_locked("power_on" if on else "power_off",
                                       lambda arm: arm.set_power(bool(on)))

    def set_drag(self, on):
        """Enter (True) / leave (False) hand-guiding / teach mode. Entering drag
        relaxes the motors; leaving it re-energises."""
        with self._lock:
            return self._motion_locked("drag_on" if on else "drag_off",
                                       lambda arm: arm.set_drag(bool(on)))

    def stop(self):
        """Controlled (soft) stop -- halts motion, motors stay energised."""
        with self._lock:
            return self._motion_locked("stop", lambda arm: arm.stop())

    def estop(self):
        """Emergency stop from the console: soft-stop then cut motor power.
        NOTE: this is a software stop (SDK stop2) plus power-off, NOT a
        substitute for the physical E-stop button, which must remain the
        primary safety device."""
        with self._lock:
            def _do(arm):
                arm.stop()
                arm.set_power(False)
            return self._motion_locked("estop", _do)

    def clear_alarm(self):
        """Recover controller state and clear a servo alarm / released e-stop."""
        with self._lock:
            return self._motion_locked("clear_alarm", lambda arm: arm.clear_alarm())

    def move_joints(self, joints_deg, speed_mms=None):
        """Jog to ABSOLUTE joint targets (degrees, one per revolute joint) at
        the given end-effector speed (mm/s). A single point-to-point move --
        not a continuous jog."""
        speed = float(speed_mms) if speed_mms else DEFAULT_JOG_SPEED
        try:
            targets_rad = [math.radians(float(a)) for a in joints_deg]
        except (TypeError, ValueError) as e:
            with self._lock:
                s = self._status_locked()
                s.update({"ok": False, "action": "move", "error": f"bad joint targets: {e}"})
                return s
        with self._lock:
            if self._arm is not None and len(targets_rad) != getattr(self._arm, "n", len(targets_rad)):
                s = self._status_locked()
                s.update({"ok": False, "action": "move",
                          "error": f"expected {self._arm.n} joint targets, got {len(targets_rad)}"})
                return s
            def _do(arm):
                # RokaeArm.move() reports rejection by returning False (and
                # stashing the reason in its status) rather than raising, so
                # turn that into an exception the gate can surface.
                if not arm.move(targets_rad, speed):
                    raise RuntimeError(arm.get_status() or "move rejected")
            return self._motion_locked("move", _do)


# Module-level singleton the server shares across requests.
BRIDGE = RobotBridge()
