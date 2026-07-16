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


def _call_with_timeout(fn, timeout_s, default):
    """Run fn() on a daemon thread and return its result, or `default` if it
    doesn't finish within timeout_s. Used to bound SDK calls that can BLOCK on a
    dead link (a hung read must never freeze the status poll / HTTP handler). A
    timed-out thread is abandoned (daemon) rather than killed."""
    box = {"v": default}
    def _run():
        try:
            box["v"] = fn()
        except Exception:  # noqa: BLE001
            box["v"] = default
    t = threading.Thread(target=_run, daemon=True)
    t.start()
    t.join(timeout_s)
    return box["v"]


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


# --- tiny 3x3 rigid-transform helpers (stdlib only) -------------------------
def _rpy_to_R(rpy):
    """(roll,pitch,yaw) rad -> 3x3 rotation, XYZ convention R = Rz@Ry@Rx (the SDK's)."""
    rx, ry, rz = (float(a) for a in rpy)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]]
    Ry = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]
    Rz = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]
    return _mm3(_mm3(Rz, Ry), Rx)


def _mm3(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mT3(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def _mv3(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def _solve_frame_transform(pose_from, pose_to):
    """Fixed rigid transform (R,t) mapping a POSITION in the `from` frame to the
    `to` frame, derived from ONE pose expressed in BOTH frames. Both describe the
    same physical flange, so R = R_to @ R_from^T and t = p_to - R @ p_from. Used to
    turn our flangeInBase math into the endInRef frame the controller commands in."""
    R_from = _rpy_to_R(pose_from["rpy"])
    R_to = _rpy_to_R(pose_to["rpy"])
    R = _mm3(R_to, _mT3(R_from))
    Rp = _mv3(R, [float(v) for v in pose_from["trans"]])
    t = [float(pose_to["trans"][i]) - Rp[i] for i in range(3)]
    return R, t


def _quat_to_R(q):
    """Unit quaternion [x,y,z,w] -> 3x3 rotation matrix."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ]


def _R_to_rpy(R):
    """3x3 rotation -> (rx,ry,rz) rad, XYZ convention (inverse of _rpy_to_R)."""
    sy = -R[2][0]
    sy = max(-1.0, min(1.0, sy))
    ry = math.asin(sy)
    if abs(R[2][0]) < 0.99999:
        rx = math.atan2(R[2][1], R[2][2])
        rz = math.atan2(R[1][0], R[0][0])
    else:  # gimbal lock: fix rz=0, fold into rx
        rx = math.atan2(-R[1][2], R[1][1])
        rz = 0.0
    return [rx, ry, rz]


def _axis_angle_R(axis, ang):
    """Rotation matrix for `ang` rad about unit-ish `axis` (Rodrigues)."""
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s, C = math.cos(ang), math.sin(ang), 1 - math.cos(ang)
    return [
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def _vcross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _vnorm(a):
    n = math.sqrt(sum(c * c for c in a)) or 1.0
    return [c / n for c in a]


def _R_to_quat(R):
    """3x3 rotation -> quaternion [x,y,z,w]."""
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2][1] - R[1][2]) / s
        y = (R[0][2] - R[2][0]) / s
        z = (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s
    return [x, y, z, w]


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
        self._fb2er = None         # (R,t) flangeInBase->endInRef, self-calibrated at connect
        self._logbuf = []          # bounded ring buffer of backend log lines (see _blog)
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

    # -- backend logging sink -------------------------------------------------
    # Every driver message is (a) PRINTED to the server terminal in full, so the
    # complete move/diagnostic sequence is visible ("know everything"), and (b)
    # kept as the last-message UI note + a bounded ring buffer the console can
    # drain via /api/robot/log. Without (a) only the final line survived.
    def _blog(self, msg):
        msg = str(msg)
        self._note = msg
        try:
            line = f"[{time.strftime('%H:%M:%S')}] {msg}"
            print(line, flush=True)
            buf = self._logbuf
            buf.append(line)
            if len(buf) > 500:
                del buf[:-500]
        except Exception:  # noqa: BLE001
            pass

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
                self._fb2er = None       # mock: base == ref, no transform needed
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
                # Self-calibrate the flangeInBase->endInRef transform from the arm's
                # CURRENT pose read in both frames. Our table->arm math yields
                # flangeInBase, but the controller COMMANDS in endInRef -- without
                # this every path waypoint is sent in the wrong frame (positive z)
                # and rejected as unreachable. Derived once; it's a fixed transform.
                self._fb2er = None
                try:
                    er = arm.get_pose_raw()          # endInRef
                    fb = arm.get_pose_raw_base()     # flangeInBase
                    if er and fb and er.get("trans") is not None and fb.get("trans") is not None:
                        self._fb2er = _solve_frame_transform(fb, er)
                        fbt = [round(v * 1000, 1) for v in fb["trans"]]
                        ert = [round(v * 1000, 1) for v in er["trans"]]
                        self._blog(f"[bridge] frame self-cal: flangeInBase {fbt} mm -> endInRef {ert} mm; "
                                   f"transform derived (t_mm={[round(v*1000,1) for v in self._fb2er[1]]})")
                    else:
                        self._blog("[bridge] frame self-cal SKIPPED (could not read both frames) -- "
                                   "path tracing may send the wrong frame")
                except Exception as e:  # noqa: BLE001
                    self._blog(f"[bridge] frame self-cal failed: {e}")
            except Exception as e:  # noqa: BLE001
                self._arm, self._kind, self._connected = None, None, False
                self._note = f"connect failed: {e}"
            return self._status_locked()

    def disconnect(self):
        with self._lock:
            self._teardown_locked()
            return self._status_locked()

    def _teardown_locked(self):
        # Drop the session reference FIRST so a concurrent status() poll can't
        # re-probe a half-torn-down arm, then close the SDK session under a
        # timeout: disconnect() can itself BLOCK on a dead link, and it must not
        # freeze the caller (which holds self._lock).
        arm = self._arm
        self._arm = None
        self._connected = False
        if arm is not None:
            _call_with_timeout(arm.disconnect, 3.0, None)

    def _session_alive_locked(self):
        """Is the real SR5 session genuinely still up? ICMP ping is only a cheap
        fast-fail (a pulled host cable / powered-off controller) -- it does NOT
        prove the SDK session is alive, because the controller's NIC keeps
        answering ping after the arm session has died. So on a successful ping we
        also do a real, timeout-bounded SDK read (is_alive). Assumes the lock is
        held; the SDK read is serialised with all other arm access by that lock."""
        if not _ping(self._ip):
            return False
        return bool(_call_with_timeout(self._arm.is_alive, 2.0, False))

    def status(self):
        with self._lock:
            # Liveness check: if we believe we're connected but the SESSION has
            # died (Ethernet unplugged, controller powered off, or the SDK link
            # dropped while the controller's NIC still answers ping), drop the
            # session HONESTLY instead of reporting a stale "connected". Probe
            # the real SDK session -- not just ICMP ping, which stays green after
            # the session is dead. Tolerate a single failed probe so a transient
            # blip doesn't bounce us offline.
            if self._connected and self._arm is not None and self._kind == "real":
                if self._session_alive_locked():
                    self._unreach = 0
                else:
                    self._unreach += 1
                    if self._unreach >= 2:
                        self._teardown_locked()
                        self._note = f"arm session lost at {self._ip} (check cable / power)"
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

    def move_pose_once(self, position, quaternion, speed_mms=None, keep_orientation=False):
        """One cartesian move to a probe pose (metres, quat [x,y,z,w], arm/table
        frame). Used by the debug viewport's camera-follow / re-aim, and by
        "Go to point" (keep_orientation=True). Shares the scan-trace gate: on the
        REAL arm this is refused unless QC_ALLOW_SCAN_TRACE=1 (unvalidated MoveL);
        the mock is always allowed.

        keep_orientation: TRANSLATE ONLY -- hold the arm's CURRENT orientation and
        move to the typed position. This is what "Go to point" wants: a fixed
        target orientation (the old identity quat) forces the wrist to a specific
        pose, and "tool straight down" (rpy 0) is the WRIST SINGULARITY at/near
        home (J5=0) -- the controller's IK then rejects it as "exceeds range of
        motion". Preserving the current orientation avoids driving the wrist into
        that degenerate config (jog J5 off zero first so "current" is non-singular).
        When set, `quaternion` is ignored."""
        if (not isinstance(position, (list, tuple)) or len(position) != 3 or
                (not keep_orientation and
                 (not isinstance(quaternion, (list, tuple)) or len(quaternion) != 4))):
            with self._lock:
                s = self._status_locked()
                s.update({"ok": False, "action": "move_pose",
                          "error": "need position[3]" + ("" if keep_orientation else " + quaternion[4]")})
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
            if self._arm is None:
                s = self._status_locked()
                s.update({"ok": False, "action": "move_pose", "error": "arm not connected"})
                return s
            # "Go to point" (keep_orientation): the operator types the target in
            # the controller's OWN endInRef frame (metres here), matching the raw
            # readout -- so we command it DIRECTLY, no table<->base conversion and
            # no calibration frame math. Orientation is held from the current
            # endInRef pose. This mirrors Ra's working xCore debugger exactly.
            # (Camera-follow, keep_orientation=False, still uses the table->base
            # helper for the viewer's table-frame poses.)
            if keep_orientation:
                p_cmd = [float(v) for v in position]
                _, q_cmd = self._arm.get_pose()          # current endInRef orientation
            elif self._kind == "real":
                p_cmd, q_cmd = _table_to_arm_base(position, quaternion)
            else:
                p_cmd = [float(v) for v in position]
                q_cmd = [float(v) for v in quaternion]
            def _do(arm):
                # "Go to point" is a debug probe -- reach the POSITION no matter
                # what: search_orientation finds a reachable tool angle if the held
                # one has no IK solution there; allow_joint_fallback drops MoveL->
                # MoveJ when the straight path is singular. So it moves to any point
                # in the feasible region. Scanning (follow_path) keeps strict MoveL.
                if not arm.move_pose(p_cmd, q_cmd, speed, linear=True, seed_conf=True,
                                     allow_joint_fallback=True, search_orientation=True):
                    raise RuntimeError(arm.get_status() or "move_pose rejected")
            return self._motion_locked("move_pose", _do)

    def request_abort(self):
        """Ask an in-flight follow_path to stop at the next check. (stop()/estop()
        also do this, plus halt the arm.)"""
        self._abort.set()

    def start_follow_path(self, waypoints, speed_mms=None, settle_s=0.3, position_only=False):
        """Start follow_path() in a background thread and return immediately, so
        the single-threaded HTTP server stays responsive (the UI polls progress
        via follow_status() and can abort via stop()). Rejects if a trace is
        already running, motion is disabled, or the arm isn't connected.

        position_only: trace the waypoint POSITIONS only, ignoring their
        orientation -- each waypoint uses the go-to machinery (orientation search
        + MoveJ fallback) so the tool tip follows the path even where a fixed
        orientation would be singular/unreachable. Used by "follow the wireframe".

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
                                   on_progress=_progress, position_only=position_only)
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

    # ---- oriented continuous scan (head aims at the part within a cone) -------
    def _wp_to_endinref_pose(self, pos, quat, incidence_deg):
        """Convert a TABLE-frame waypoint (position + aim quaternion) into a
        reachable endInRef pose (trans_m, rpy_rad). The aim points the tool at the
        surface; if the EXACT aim has no IK solution, tilt the tool up to
        incidence_deg off it (staying within the acceptable incidence cone) until a
        reachable orientation is found -- smallest tilt first. Returns (trans, rpy)
        or None if the whole cone is unreachable. On the mock (calc_ik_ok True) the
        exact aim is always used."""
        p_fb, q_fb = _table_to_arm_base(pos, quat)      # table -> flangeInBase
        if self._fb2er is not None:                     # flangeInBase -> endInRef
            R, t = self._fb2er
            p_er = [_mv3(R, p_fb)[k] + t[k] for k in range(3)]
            R_er = _mm3(R, _quat_to_R(q_fb))
        else:                                           # mock: command frame == fb
            p_er, R_er = list(p_fb), _quat_to_R(q_fb)
        aim_rpy = _R_to_rpy(R_er)
        arm = self._arm
        ok = arm.calc_ik_ok(p_er, aim_rpy)
        if ok or ok is None:                            # reachable, or calcIk n/a
            return p_er, aim_rpy
        # cone search: tilt the tool's forward axis within +/- incidence_deg
        fwd = _vnorm(_mv3(R_er, [0, 0, 1]))
        ref = [0, 0, 1] if abs(fwd[2]) < 0.9 else [1, 0, 0]
        u = _vnorm(_vcross(fwd, ref))
        v = _vnorm(_vcross(fwd, u))
        for a_deg in (incidence_deg * 0.5, incidence_deg):
            if a_deg <= 0:
                continue
            a = math.radians(a_deg)
            for az_deg in (0, 90, 180, 270, 45, 135, 225, 315):
                az = math.radians(az_deg)
                axis = [u[i] * math.cos(az) + v[i] * math.sin(az) for i in range(3)]
                rpy = _R_to_rpy(_mm3(_axis_angle_R(axis, a), R_er))
                if arm.calc_ik_ok(p_er, rpy):
                    return p_er, rpy
        return None

    def start_scan_trace(self, waypoints, incidence_deg=10.0, speed_mms=None):
        """Background wrapper for scan_trace(), mirroring start_follow_path so the
        UI polls follow_status() and can abort. Rejects if a trace is running,
        motion is disabled, or the arm isn't connected."""
        with self._lock:
            if self._follow["running"]:
                return {"ok": False, "started": False, "error": "a path is already running"}
            if not ALLOW_MOTION:
                return {"ok": False, "started": False, "error": "motion disabled (QC_ALLOW_MOTION=0)"}
            if not (self._connected and self._arm is not None):
                return {"ok": False, "started": False, "error": "not connected"}
            if self._kind == "real" and not ALLOW_SCAN_TRACE:
                return {"ok": False, "started": False,
                        "error": "scan tracing DISABLED on the real arm (set QC_ALLOW_SCAN_TRACE=1)"}
            self._follow = {"running": True, "completed": 0, "total": len(waypoints),
                            "aborted": False, "error": None, "ok": None}

        def _run():
            def _progress(done, total):
                with self._lock:
                    self._follow["completed"] = done
                    self._follow["total"] = total
            rep = self.scan_trace(waypoints, incidence_deg=incidence_deg,
                                  speed_mms=speed_mms, on_progress=_progress)
            with self._lock:
                self._follow = {"running": False, "completed": rep["completed"],
                                "total": rep["total"], "aborted": rep["aborted"],
                                "error": rep["error"], "ok": rep["ok"]}

        self._follow_thread = threading.Thread(target=_run, daemon=True)
        self._follow_thread.start()
        return {"ok": True, "started": True, "total": len(waypoints)}

    def scan_trace(self, waypoints, incidence_deg=10.0, speed_mms=None,
                   reach_timeout_s=60.0, tick_s=0.1, on_progress=None):
        """CONTINUOUS oriented scan. Each scan LINE (grouped by line_id) is run as
        ONE blended MoveL sweep -- the head flows through it without stopping, the
        tool aiming at the part within +/- incidence_deg the whole way. Between
        lines the arm repositions with a MoveJ. Returns {ok, completed, total,
        aborted, error}. Abortable between lines and via stop()/estop()."""
        speed = float(speed_mms) if speed_mms else DEFAULT_JOG_SPEED
        total = len(waypoints)
        self._abort.clear()

        def _report(ok, completed, aborted=False, error=None):
            return {"ok": ok, "completed": completed, "total": total,
                    "aborted": aborted, "error": error}

        with self._lock:
            if not ALLOW_MOTION:
                return _report(False, 0, error="motion disabled (QC_ALLOW_MOTION=0)")
            if not (self._connected and self._arm is not None):
                return _report(False, 0, error="not connected")
            if self._kind == "real" and not ALLOW_SCAN_TRACE:
                return _report(False, 0, error="scan tracing disabled (QC_ALLOW_SCAN_TRACE=1)")

        # group consecutive waypoints by line_id, preserving order
        lines = []
        for wp in waypoints:
            lid = wp.get("line_id", 0)
            if not lines or lines[-1][0] != lid:
                lines.append((lid, []))
            lines[-1][1].append(wp)
        self._blog(f"[bridge] scan_trace: {total} waypoints in {len(lines)} line(s), "
                   f"incidence +/-{incidence_deg} deg, speed {speed} mm/s")

        completed = 0
        for lid, wps in lines:
            if self._abort.is_set():
                return _report(False, completed, aborted=True, error="aborted")
            # Resolve every pose in this line to a reachable endInRef pose first
            # (a continuous MoveL can't bail mid-stroke, so validate the whole line).
            poses = []
            for wp in wps:
                if self._abort.is_set():
                    return _report(False, completed, aborted=True, error="aborted")
                with self._lock:
                    if self._arm is None:
                        return _report(False, completed, error="connection lost")
                    res = self._wp_to_endinref_pose(wp.get("position"), wp.get("quaternion"),
                                                    incidence_deg)
                if res is None:
                    return _report(False, completed,
                                   error=f"line {lid}: a waypoint is unreachable within "
                                         f"+/-{incidence_deg} deg of the surface normal")
                poses.append(res)

            # Reposition to the line start (MoveJ, arrive at the scan orientation),
            # then run the whole line as one continuous blended MoveL sweep.
            t0, rpy0 = poses[0]
            q0 = _R_to_quat(_rpy_to_R(rpy0))
            with self._lock:
                arm = self._arm
                if arm is None:
                    return _report(False, completed, error="connection lost")
                if not arm.move_pose(t0, q0, speed, linear=False, seed_conf=True):
                    return _report(False, completed, error=f"line {lid}: reposition failed ({arm.get_status()})")
            if not self._wait_settle(reach_timeout_s):
                return _report(False, completed, aborted=self._abort.is_set(),
                               error=f"line {lid}: reposition did not settle")

            with self._lock:
                arm = self._arm
                if arm is None:
                    return _report(False, completed, error="connection lost")
                if not arm.move_pose_list(poses, speed):
                    return _report(False, completed, error=f"line {lid}: sweep rejected ({arm.get_status()})")
            if not self._wait_settle(reach_timeout_s):
                return _report(False, completed, aborted=self._abort.is_set(),
                               error=f"line {lid}: sweep did not complete")

            completed += len(wps)
            if on_progress:
                on_progress(completed, total)

        return _report(True, completed)

    def _wait_settle(self, timeout_s, tick_s=0.1):
        """Wait until the arm stops moving (or abort/timeout). Returns True if it
        settled cleanly, False on abort / error / timeout."""
        time.sleep(tick_s)   # grace so the controller flips into 'moving' first
        deadline = time.time() + timeout_s
        last = time.time()
        while time.time() < deadline:
            if self._abort.is_set():
                with self._lock:
                    if self._arm is not None:
                        self._arm.stop()
                return False
            now = time.time()
            dt, last = now - last, now
            with self._lock:
                arm = self._arm
                if arm is None:
                    return False
                arm.update(dt)
                moving, st = arm.is_moving(), arm.get_status()
            if isinstance(st, str) and st.startswith("error"):
                return False
            if not moving:
                return True
            time.sleep(tick_s)
        return False

    def follow_path(self, waypoints, speed_mms=None, settle_s=0.3,
                    reach_timeout_s=15.0, tick_s=0.05, on_progress=None,
                    position_only=False):
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
                # TABLE frame -> the controller's COMMAND frame. For the real arm:
                # table -> flangeInBase (_table_to_arm_base) -> endInRef (the
                # self-calibrated transform), because the controller commands in
                # endInRef. Skipping the second hop sends positive-z targets that
                # calcIk rejects -> "nothing moves". Mock is frameless.
                if self._kind == "real":
                    p_cmd, q_cmd = _table_to_arm_base(pos, quat)
                    if self._fb2er is not None:
                        R, t = self._fb2er
                        p_cmd = [_mv3(R, p_cmd)[k] + t[k] for k in range(3)]
                else:
                    p_cmd, q_cmd = (pos, quat)
                # position_only: follow the tip through the waypoint POSITIONS,
                # ignoring their orientation -- use the go-to machinery (search a
                # reachable orientation + MoveJ fallback) so a singular fixed
                # orientation never stalls the trace. Otherwise strict MoveL.
                if position_only:
                    okmove = arm.move_pose(p_cmd, q_cmd, speed, seed_conf=True,
                                           allow_joint_fallback=True, search_orientation=True)
                else:
                    okmove = arm.move_pose(p_cmd, q_cmd, speed)
                if not okmove:
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
