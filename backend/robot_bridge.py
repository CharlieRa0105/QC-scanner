"""
robot_bridge.py

Connection layer between the operator console and the ROKAE xMate SR5.

The robot-driver code has a single home: the ROS 2 ArmDriver package at
`ros2_ws/src/sr5_arm_driver/sr5_arm_driver/backends.py`. Its backend classes are
pure-Python (no rclpy), so the console borrows them directly and there is one
source of truth for talking to the arm:

  * `sr5_arm_driver.backends.RokaeArm`  -> xCore SDK (Release/linux/*.so)

There is NO mock backend: if the arm is unreachable, or the SDK cannot load,
the bridge stays honestly disconnected rather than faking a session.

This bridge is a thin adapter: it owns the single SDK session (the SDK allows
only one TCP session to the controller), serialises access, and formats
status/telemetry for the HTTP API.

PYTHON VERSION (important): the xCore SDK ships CPython builds for 3.8-3.12
ONLY. Under 3.13+ the `from Release.linux import xCoreSDK_python` import falls
through to an empty namespace directory and connect fails with "No known robot
class in SDK build". Run the console under Python 3.12 (see scripts/).

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
  QC_SDK_PATH     auto (~/rokaeProject or ~/rokae_sdk)  xCore SDK root (contains Release/linux/)
  QC_ALLOW_MOTION default 1               master motion switch (0 => read-only)
  QC_JOG_SPEED    default 60              default jog end-effector speed, mm/s
"""

import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

DEFAULT_IP = os.environ.get("QC_ROBOT_IP", "192.168.2.160")


def _resolve_sdk_root():
    """Locate the xCore SDK root (the dir containing Release/linux/*.so).
    Honour QC_SDK_PATH if set; otherwise probe the known install locations so a
    bare `python3 backend/server.py` still finds it."""
    env = os.environ.get("QC_SDK_PATH")
    if env:
        return os.path.expanduser(env)
    for cand in ("~/rokaeProject", "~/rokae_sdk"):
        p = os.path.expanduser(cand)
        if os.path.isdir(os.path.join(p, "Release", "linux")):
            return p
    return os.path.expanduser("~/rokae_sdk")


SDK_ROOT = _resolve_sdk_root()
# Master motion switch. Any value other than 0/false/no leaves motion enabled.
ALLOW_MOTION = os.environ.get("QC_ALLOW_MOTION", "1").lower() not in ("0", "false", "no", "")
DEFAULT_JOG_SPEED = float(os.environ.get("QC_JOG_SPEED", "60"))  # mm/s end-effector
# Cartesian scan-path tracing (follow_path -> MoveL) uses the still-UNVALIDATED
# pose units/rpy convention, so it is DISABLED on the real arm by default. Prove
# move_pose at low speed with Ra + the E-stop first, then set QC_ALLOW_SCAN_TRACE=1
# to enable. This is separate from QC_ALLOW_MOTION (which gates jog/home/etc.).
ALLOW_SCAN_TRACE = os.environ.get("QC_ALLOW_SCAN_TRACE", "0").lower() not in ("0", "false", "no", "")
# TEST-ONLY mock backend (QC_ARM_MOCK=1): used to self-verify motion flows
# without hardware (the refactor rule: prove against the mock first). Default
# OFF -- a normal session always talks to the real SR5 and never fakes a
# connection. Re-added deliberately for the debug/verification ladder.
USE_MOCK = os.environ.get("QC_ARM_MOCK", "0").lower() not in ("0", "false", "no", "")

# The arm driver backend (RokaeArm) lives in the ROS 2 ArmDriver package
# at ros2_ws/src/sr5_arm_driver/. It is pure-Python (imports only math/time -- no
# rclpy), so the console borrows it directly rather than keeping a second copy.
# This borrow is temporary: once the ROS 2 mission graph owns motion (see
# docs/refactor-guide.md 2.2) the console talks to it over rosbridge and this
# direct import retires. Add the package dir to sys.path so `sr5_arm_driver.backends`
# resolves regardless of how the server is launched.
_ARM_PKG_DIR = Path(__file__).resolve().parent.parent / "ros2_ws" / "src" / "sr5_arm_driver"
if str(_ARM_PKG_DIR) not in sys.path:
    sys.path.insert(0, str(_ARM_PKG_DIR))
from sr5_arm_driver.backends import MockArm, RokaeArm  # noqa: E402  (project driver code)

# SR5 joint labels for the UI (6 revolute joints; the rail is a separate axis,
# not commanded from this console).
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


def _mount_height_m():
    """Arm-base height above the table, from config (fallback: the cell's known
    1.2 m). Read lazily so the stdlib server still runs without PyYAML/config."""
    try:
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
        from libs.qc_config import load_config
        return float(load_config()["workspace"]["mount"]["base_xyz_mm"][2]) / 1000.0
    except Exception:  # noqa: BLE001
        return 1.2


MOUNT_H = _mount_height_m()

# NOTE (removed): a geometric "reach sphere" preflight used to refuse poses
# farther than 919 mm from the base ORIGIN. That model is WRONG -- the SR5's
# 919 mm reach is its envelope radius about the SHOULDER, and the base structure
# itself adds length, so the flange sits ~1259 mm from the base origin even at
# home (all joints 0). The check falsely rejected valid points. Reachability is
# now left to the CONTROLLER's own IK, which rejects genuinely-unreachable poses
# on the move (surfaced as an error) -- the authoritative source, not a sphere.


def _table_to_arm_base(position, quaternion):
    """Convert a pose from the TABLE frame (origin on the table under the arm,
    Z up — what the planner/viewer use) into the ARM BASE frame the controller
    actually understands (base overhead at height H, rolled 180 deg about X).

        p_arm = R_x(pi) . (p_table - [0,0,H])  =  ( x, -y, H - z )
        q_arm = q_flip * q_table,  q_flip = rot(pi about X) = (1,0,0,0)

    This was THE missing piece in the first real-arm validation: poses were sent
    raw, so the controller placed them ~H too low and Y-mirrored vs the sim.
    """
    x, y, z = (float(v) for v in position)
    p_arm = [x, -y, MOUNT_H - z]
    qx, qy, qz, qw = (float(v) for v in quaternion)
    # quaternion product q_flip(1,0,0,0) * q  (Hamilton, [x,y,z,w]):
    #   x' = qw,  y' = -qz,  z' = qy,  w' = -qx
    q_arm = [qw, -qz, qy, -qx]
    return p_arm, q_arm


class RobotBridge:
    """Owns a single SR5 connection (via the project's arm driver backend) and
    serialises access. Read-only w.r.t. motion."""

    def __init__(self):
        self._lock = threading.Lock()
        self._arm = None
        self._kind = None          # 'real' | None  (console connects to the real SR5 only)
        self._ip = DEFAULT_IP
        self._connected = False
        self._note = ""
        self._unreach = 0          # consecutive failed liveness pings (see status())
        self._last_tick = time.monotonic()   # for mock sim time (see joints())
        # Set to interrupt an in-flight follow_path (by stop/estop/abort). It's
        # a lock-free flag so an abort lands even while follow_path holds nothing.
        self._abort = threading.Event()
        # Progress of an async path trace (start_follow_path), read by the UI via
        # follow_status(). Updated under self._lock.
        self._follow = {"running": False, "completed": 0, "total": 0,
                        "aborted": False, "error": None, "ok": None}
        self._follow_thread = None

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

            # TEST-ONLY mock (QC_ARM_MOCK=1): simulated SR5 for verification
            # ladders; never active in a normal session.
            if USE_MOCK:
                arm = MockArm(log=self._blog)
                arm.connect()
                self._arm, self._kind, self._connected = arm, "mock", True
                self._note = "MOCK backend (QC_ARM_MOCK=1) -- no hardware"
                return self._status_locked()

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
                self._unreach = 0
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
            # Liveness check: if we believe we're connected but the arm has gone
            # unreachable (Ethernet unplugged, controller powered off), drop the
            # session HONESTLY instead of reporting a stale "connected". Tolerate
            # a single dropped ping so a transient blip doesn't bounce us offline.
            if self._connected and self._arm is not None and self._kind == "real":
                if _ping(self._ip):
                    self._unreach = 0
                else:
                    self._unreach += 1
                    if self._unreach >= 2:
                        self._teardown_locked()
                        self._note = f"arm unreachable at {self._ip} (check cable / power)"
            return self._status_locked()

    def _status_locked(self):
        s = {"connected": self._connected, "kind": self._kind,
             "ip": self._ip, "note": self._note,
             # gate introspection (read-only) so the UI / a curl can confirm
             # whether cartesian tracing is enabled WITHOUT sending motion
             "motionEnabled": ALLOW_MOTION, "scanTraceEnabled": ALLOW_SCAN_TRACE}
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
                # Advance/refresh the backend by the real elapsed time since the
                # last poll: the REAL arm ignores dt (it reads live state), but
                # the MOCK integrates its simulated motion with it — without this
                # a mock jog would never visibly move between polls.
                now = time.monotonic()
                dt = min(now - self._last_tick, 0.5)
                self._last_tick = now
                self._arm.update(dt)
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
            # The controller's OWN flange-pose report (SDK cartPosture), passed
            # through untouched — the debug UI shows this as ground truth to
            # compare against the software's frame math. None if unsupported.
            tcp = None
            try:
                tcp = self._arm.get_pose_raw()
            except Exception:  # noqa: BLE001
                pass
            return {"connected": True, "kind": self._kind, "joints": out, "tcp": tcp}

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
        """Controlled (soft) stop -- halts motion, motors stay energised. Also
        interrupts an in-flight follow_path."""
        self._abort.set()
        with self._lock:
            return self._motion_locked("stop", lambda arm: arm.stop())

    def estop(self):
        """Emergency stop from the console: soft-stop then cut motor power.
        NOTE: this is a software stop (SDK stop2) plus power-off, NOT a
        substitute for the physical E-stop button, which must remain the
        primary safety device."""
        self._abort.set()
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

    def move_pose_once(self, position, quaternion, speed_mms=None):
        """One cartesian move to a probe pose (metres, quat [x,y,z,w], arm/table
        frame). Used by the debug viewport's camera-follow / re-aim. Shares the
        scan-trace gate: on the REAL arm this is refused unless
        QC_ALLOW_SCAN_TRACE=1 (unvalidated MoveL); the mock is always allowed."""
        if (not isinstance(position, (list, tuple)) or len(position) != 3 or
                not isinstance(quaternion, (list, tuple)) or len(quaternion) != 4):
            with self._lock:
                s = self._status_locked()
                s.update({"ok": False, "action": "move_pose",
                          "error": "need position[3] + quaternion[4]"})
                return s
        speed = float(speed_mms) if speed_mms else DEFAULT_JOG_SPEED
        with self._lock:
            if self._kind == "real" and not ALLOW_SCAN_TRACE:
                s = self._status_locked()
                s.update({"ok": False, "action": "move_pose",
                          "error": "cartesian moves are DISABLED on the real arm "
                                   "(unvalidated MoveL); set QC_ALLOW_SCAN_TRACE=1 "
                                   "after validating at low speed"})
                return s
            # table frame in; real controller wants its base frame (see helper)
            p_cmd, q_cmd = ((_table_to_arm_base(position, quaternion))
                            if self._kind == "real"
                            else ([float(v) for v in position], [float(v) for v in quaternion]))
            def _do(arm):
                if not arm.move_pose(p_cmd, q_cmd, speed):
                    raise RuntimeError(arm.get_status() or "move_pose rejected")
            return self._motion_locked("move_pose", _do)

    def request_abort(self):
        """Ask an in-flight follow_path to stop at the next check. (stop()/estop()
        also do this, plus halt the arm.)"""
        self._abort.set()

    def start_follow_path(self, waypoints, speed_mms=None, settle_s=0.3):
        """Start follow_path() in a background thread and return immediately, so
        the single-threaded HTTP server stays responsive (the UI polls progress
        via follow_status() and can abort via stop()). Rejects if a trace is
        already running, motion is disabled, or the arm isn't connected.

        Returns {ok, started, total, error?}.
        """
        with self._lock:
            if self._follow["running"]:
                return {"ok": False, "started": False, "error": "a path is already running"}
            if not ALLOW_MOTION:
                return {"ok": False, "started": False,
                        "error": "motion disabled (QC_ALLOW_MOTION=0)"}
            if not (self._connected and self._arm is not None):
                return {"ok": False, "started": False, "error": "not connected"}
            # Hard safety gate: never drive the REAL arm with the unvalidated
            # cartesian trace unless explicitly opted in (QC_ALLOW_SCAN_TRACE=1).
            if self._kind == "real" and not ALLOW_SCAN_TRACE:
                return {"ok": False, "started": False,
                        "error": "scan-path tracing is DISABLED on the real arm "
                                 "(unvalidated cartesian MoveL). Validate move_pose at "
                                 "low speed with the E-stop in reach, then set "
                                 "QC_ALLOW_SCAN_TRACE=1 to enable."}
            # (reachability is judged by the controller's IK, not a sphere here)
            self._follow = {"running": True, "completed": 0, "total": len(waypoints),
                            "aborted": False, "error": None, "ok": None}

        def _run():
            def _progress(completed, total):
                with self._lock:
                    self._follow["completed"] = completed
                    self._follow["total"] = total
            rep = self.follow_path(waypoints, speed_mms=speed_mms, settle_s=settle_s,
                                   on_progress=_progress)
            with self._lock:
                self._follow = {"running": False, "completed": rep["completed"],
                                "total": rep["total"], "aborted": rep["aborted"],
                                "error": rep["error"], "ok": rep["ok"]}

        self._follow_thread = threading.Thread(target=_run, daemon=True)
        self._follow_thread.start()
        return {"ok": True, "started": True, "total": len(waypoints)}

    def follow_status(self):
        """Snapshot of the async path trace for the UI."""
        with self._lock:
            return dict(self._follow)

    def follow_path(self, waypoints, speed_mms=None, settle_s=0.3,
                    reach_timeout_s=15.0, tick_s=0.05, on_progress=None):
        """Trace a sequence of CARTESIAN probe poses (arm base frame).

        Each waypoint is a dict with 'position' [x,y,z] in metres and
        'quaternion' [x,y,z,w] -- i.e. the arm-frame ScanPath schema produced by
        scripts/scanpath_convert.py. The arm is moved to each pose in turn; the
        method waits until it settles (no longer moving), dwells `settle_s`, then
        advances. This is the "sequential poses, settle between each" motion the
        demo slice needs; the arm resolves each pose to joints via the SDK's
        cartesian move (MoveL) internally -- no host-side IK.

        Gated on QC_ALLOW_MOTION + a live connection + powered motors. Abortable
        between and within moves via stop()/estop()/request_abort(). Blocking --
        the caller runs it (T6 wraps it in a worker thread so the UI stays live).

        The lock is taken PER STEP (issue move, tick, read state), never across
        the whole path, so an abort or a status poll is never blocked for long.

        Returns {ok, completed, total, aborted, error}.
        """
        speed = float(speed_mms) if speed_mms else DEFAULT_JOG_SPEED
        total = len(waypoints)
        self._abort.clear()

        def _report(ok, completed, aborted=False, error=None):
            return {"ok": ok, "completed": completed, "total": total,
                    "aborted": aborted, "error": error}

        # Pre-flight gate (one locked check before we start moving).
        with self._lock:
            if not ALLOW_MOTION:
                return _report(False, 0, error="motion disabled (QC_ALLOW_MOTION=0)")
            if not (self._connected and self._arm is not None):
                return _report(False, 0, error="not connected")
            # Defense-in-depth: the unvalidated cartesian trace never runs on the
            # real arm without an explicit opt-in (see start_follow_path).
            if self._kind == "real" and not ALLOW_SCAN_TRACE:
                return _report(False, 0, error="scan-path tracing disabled on the real "
                                               "arm (set QC_ALLOW_SCAN_TRACE=1 to enable)")

        completed = 0
        for i, wp in enumerate(waypoints):
            if self._abort.is_set():
                return _report(False, completed, aborted=True, error="aborted")
            try:
                pos, quat = wp["position"], wp["quaternion"]
            except (KeyError, TypeError) as e:
                return _report(False, completed, error=f"waypoint {i}: bad shape ({e})")

            # Issue the cartesian move. Waypoints arrive in the TABLE frame; the
            # real controller wants its own BASE frame (overhead, rolled), so
            # convert for the real arm. The mock simulates in the table frame.
            with self._lock:
                arm = self._arm
                if arm is None:
                    return _report(False, completed, error="connection lost")
                p_cmd, q_cmd = (_table_to_arm_base(pos, quat)
                                if self._kind == "real" else (pos, quat))
                if not arm.move_pose(p_cmd, q_cmd, speed):
                    return _report(False, completed, error=f"waypoint {i}: {arm.get_status()}")

            # Wait for the arm to reach + settle. A brief grace tick first so the
            # real controller has time to flip into 'moving' before we test it.
            time.sleep(tick_s)
            deadline = time.time() + reach_timeout_s
            last = time.time()
            while True:
                if self._abort.is_set():
                    with self._lock:
                        if self._arm is not None:
                            self._arm.stop()
                    return _report(False, completed, aborted=True, error="aborted")
                now = time.time()
                dt, last = now - last, now
                with self._lock:
                    arm = self._arm
                    if arm is None:
                        return _report(False, completed, error="connection lost")
                    arm.update(dt)            # refresh live joint / operation state
                    moving, st = arm.is_moving(), arm.get_status()
                if isinstance(st, str) and st.startswith("error"):
                    return _report(False, completed, error=st)
                if not moving:
                    break
                if now > deadline:
                    return _report(False, completed, error=f"waypoint {i}: reach timeout")
                time.sleep(tick_s)

            time.sleep(settle_s)             # dwell so a scan capture could settle
            completed += 1
            if on_progress is not None:
                try:
                    on_progress(completed, total)
                except Exception:  # noqa: BLE001
                    pass

        return _report(True, completed)


# Module-level singleton the server shares across requests.
BRIDGE = RobotBridge()
