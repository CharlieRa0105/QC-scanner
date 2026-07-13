"""
robot_bridge.py

Real connection layer between the operator console and the ROKAE xMate SR5,
via the vendor xCore Python SDK.

The SDK API (see the "Rokae xCore Python SDK Cheat Sheet"):
    robot = xCoreSDK_python.xMateRobot(ip)
    ec = {}
    robot.connectToRobot(ec)          # ec["message"] == "" on success
    robot.robotInfo(ec)               # .id .version .type .joint_num .mac
    robot.powerState(ec) / operateMode(ec)
    robot.jointPos(ec)                # 6 joint angles, radians
    robot.jointVel(ec) / jointTorque(ec)
    robot.disconnectFromRobot(ec)

Two SDK sources, identical API (the vendor ships the real one; Ra built a mock
that registers under the same `Release.linux` import path so the same code runs
with no arm attached):
  * real: a compiled xCoreSDK-Python build (Release/linux/xCoreSDK_python.*.so)
  * mock: a pure-Python stand-in (mock_sdk/xCoreSDK_python.py)

Mode selection (env QC_ROBOT_MODE = auto | real | mock, default auto):
  * real  -> only the real SDK; error if it can't load / arm unreachable
  * mock  -> only the mock (safe anywhere, no network)
  * auto  -> use the real SDK iff it imports AND the arm pings within a short
             timeout; otherwise fall back to the mock. `kind` in the status
             always says which actually loaded, so the UI is never misleading.

Everything is guarded by a lock: the SDK holds one TCP session to the
controller and must not be called concurrently from multiple request threads.
"""

import os
import subprocess
import sys
import threading
import types
from pathlib import Path

DEFAULT_IP = os.environ.get("QC_ROBOT_IP", "192.168.2.160")
MODE = os.environ.get("QC_ROBOT_MODE", "auto").lower()

# Candidate real-SDK repo roots (each expected to contain Release/linux/).
_REAL_SDK_PATHS = [
    os.environ.get("QC_SDK_PATH"),
    "~/rokaeProject/xCoreSDK-Python",
    "~/Documents/arm test/xCoreSDK-Python",
]
# Directory containing the `mock_sdk` package.
_MOCK_SDK_DIR = os.environ.get("QC_MOCK_SDK_DIR", "~/Documents/arm test")

# SR5 joint labels for the UI (6 revolute joints; the rail is a separate axis
# the SDK doesn't expose, handled elsewhere).
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


def _import_real_sdk():
    """Try to import the real compiled SDK. Returns the module or None.

    Note: the compiled .so targets a specific Python ABI; on a mismatched
    interpreter this import fails and we fall back to the mock (auto mode).
    """
    for p in _REAL_SDK_PATHS:
        if not p:
            continue
        repo = Path(os.path.expanduser(p))
        if not (repo / "Release" / "linux").exists():
            continue
        sys.path.insert(0, str(repo))
        try:
            from Release.linux import xCoreSDK_python as sdk  # type: ignore
            return sdk
        except Exception:
            continue
    return None


def _import_mock_sdk():
    """Register the mock under `Release.linux.xCoreSDK_python` and return it.

    Mirrors run_sim.py's install_mock(): build the fake Release -> Release.linux
    package chain in sys.modules so `from Release.linux import xCoreSDK_python`
    resolves to the mock.
    """
    mock_dir = Path(os.path.expanduser(_MOCK_SDK_DIR))
    if not (mock_dir / "mock_sdk").is_dir():
        return None
    sys.path.insert(0, str(mock_dir))
    try:
        from mock_sdk import xCoreSDK_python as mock  # type: ignore
    except Exception:
        return None
    release = types.ModuleType("Release")
    release.__path__ = []
    linux = types.ModuleType("Release.linux")
    linux.__path__ = []
    linux.xCoreSDK_python = mock
    sys.modules.setdefault("Release", release)
    sys.modules["Release.linux"] = linux
    sys.modules["Release.linux.xCoreSDK_python"] = mock
    return mock


def _select_sdk(ip):
    """Pick and import an SDK per MODE. Returns (module, kind, note)."""
    if MODE == "mock":
        m = _import_mock_sdk()
        return (m, "mock", "" if m else "mock SDK not found")
    if MODE == "real":
        m = _import_real_sdk()
        if not m:
            return (None, None, "real SDK could not be imported")
        return (m, "real", "")
    # auto: prefer real iff importable AND arm reachable, else mock.
    real = _import_real_sdk()
    if real is not None and _ping(ip):
        return (real, "real", "")
    m = _import_mock_sdk()
    if m:
        note = "arm unreachable" if real is not None else "real SDK unavailable on this Python"
        return (m, "mock", f"auto: {note}, using mock")
    if real is not None:
        return (real, "real", "arm unreachable but mock unavailable")
    return (None, None, "no SDK available (real import failed, mock not found)")


class RobotBridge:
    """Owns a single SR5 connection and serialises SDK access."""

    def __init__(self):
        self._lock = threading.Lock()
        self._sdk = None
        self._robot = None
        self._kind = None          # 'real' | 'mock' | None
        self._ip = DEFAULT_IP
        self._connected = False
        self._note = ""

    def connect(self, ip=None):
        """Load an SDK (per MODE) and open a connection. Idempotent-ish:
        a fresh connect replaces any existing one."""
        with self._lock:
            self._ip = ip or self._ip or DEFAULT_IP
            self._disconnect_locked()  # drop any stale session first

            sdk, kind, note = _select_sdk(self._ip)
            self._sdk, self._kind, self._note = sdk, kind, note
            if sdk is None:
                self._connected = False
                return self._status_locked()

            ec = {}
            self._robot = sdk.xMateRobot(self._ip)
            self._robot.connectToRobot(ec)
            msg = ec.get("message", "") if isinstance(ec, dict) else ""
            self._connected = (msg == "")
            if not self._connected:
                self._note = (self._note + "; " if self._note else "") + f"connect failed: {msg}"
            return self._status_locked()

    def disconnect(self):
        with self._lock:
            self._disconnect_locked()
            return self._status_locked()

    def _disconnect_locked(self):
        if self._robot is not None and self._connected:
            try:
                self._robot.disconnectFromRobot({})
            except Exception:
                pass
        self._robot = None
        self._connected = False

    def status(self):
        with self._lock:
            return self._status_locked()

    def _status_locked(self):
        s = {
            "connected": self._connected,
            "kind": self._kind,          # which SDK loaded: real / mock / None
            "ip": self._ip,
            "note": self._note,
        }
        if self._connected and self._robot is not None:
            ec = {}
            try:
                s["sdkVersion"] = str(self._robot.sdkVersion())
                info = self._robot.robotInfo(ec)
                s["info"] = {"id": getattr(info, "id", None),
                             "type": getattr(info, "type", None),
                             "version": getattr(info, "version", None),
                             "jointNum": getattr(info, "joint_num", None)}
                s["power"] = str(self._robot.powerState(ec))
                s["mode"] = str(self._robot.operateMode(ec))
            except Exception as e:  # noqa: BLE001
                s["note"] = (self._note + "; " if self._note else "") + f"status read error: {e}"
        return s

    def joints(self):
        """Return live joint state, angles in DEGREES for the UI.

        Only ever reads (jointPos/Vel/Torque) — never commands motion.
        """
        import math
        with self._lock:
            if not (self._connected and self._robot is not None):
                return {"connected": False, "joints": []}
            ec = {}
            try:
                pos = list(self._robot.jointPos(ec))
            except Exception as e:  # noqa: BLE001
                return {"connected": True, "joints": [], "error": str(e)}
            try:
                vel = list(self._robot.jointVel(ec))
            except Exception:
                vel = [0.0] * len(pos)
            try:
                tq = list(self._robot.jointTorque(ec))
            except Exception:
                tq = [0.0] * len(pos)
            out = []
            for i, p in enumerate(pos):
                out.append({
                    "name": JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"J{i+1}",
                    "deg": round(math.degrees(p), 2),
                    "vel": round(vel[i], 3) if i < len(vel) else 0.0,
                    "torque": round(tq[i], 3) if i < len(tq) else 0.0,
                })
            return {"connected": True, "kind": self._kind, "joints": out}


# Module-level singleton the server shares across requests.
BRIDGE = RobotBridge()
