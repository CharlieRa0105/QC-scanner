"""Backends for the SR5 arm driver.

Two interchangeable implementations behind one interface so the ROS node code
never changes when you swap simulation for the real robot:

  * MockArm   -- pure-Python simulation (default). No hardware needed.
  * RokaeArm  -- wraps the Rokae xCore Python SDK (real SR5 over Ethernet).

Interface (all joint values are RADIANS, the ROS convention):
    connect() / disconnect()
    get_joints() -> list[float]      # length = n_joints
    get_status() -> str              # 'idle' | 'moving' | 'drag' | 'off' | 'error:...'
    is_moving() -> bool
    update(dt)                       # advance state by dt seconds (mock physics / poll real)
    move(target_rad, speed_pct)      # queue a joint move
    move_pose(pos_m, quat_xyzw, speed_mms)  # queue a CARTESIAN move (probe pose)
    get_pose() -> (pos_m, quat_xyzw) # current end-effector pose, arm base frame
    set_power(on) -> bool
    set_drag(on) -> bool
    clear_alarm()
    stop()

Cartesian poses (move_pose / get_pose) are in the ARM BASE frame: position in
METRES, orientation as a quaternion [x, y, z, w]. The real arm resolves the pose
to joints internally (the xCore controller does the IK for a MoveL); the mock has
no kinematics, so it simulates a moving tool-centre-point pose directly. This is
what lets a scan path (a list of probe poses) be traced without host-side IK.
"""

import math
import time


def _quat_to_rpy(quat_xyzw):
    """Quaternion [x, y, z, w] -> (roll, pitch, yaw) radians, XYZ intrinsic
    convention (R = Rz @ Ry @ Rx) -- the same convention libs/path_planning
    /frame_transform.py uses, and what the xCore CartesianPosition expects.
    math-only so the backend stays numpy-free."""
    x, y, z, w = quat_xyzw
    # roll (about X)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # pitch (about Y), clamped through the gimbal singularity
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    # yaw (about Z)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _rpy_to_quat(roll, pitch, yaw):
    """(roll, pitch, yaw) radians -> quaternion [x, y, z, w], the inverse of
    _quat_to_rpy (XYZ intrinsic). Used to report the real arm's pose as a quat."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    ]


# ---------------------------------------------------------------------------
# Mock (simulation) backend
# ---------------------------------------------------------------------------
class MockArm:
    """Simulates a 6-joint arm. Moves interpolate toward the target at a speed
    proportional to speed_pct; 'drag' mode gently wanders each joint so that a
    teach-and-capture workflow produces varied waypoints with no hardware."""

    def __init__(self, n_joints=6, max_speed_dps=45.0, log=print):
        self.n = n_joints
        self.max_speed = math.radians(max_speed_dps)   # rad/s at 100%
        self.log = log
        self.joints = [0.0] * n_joints
        self._target = list(self.joints)
        self._speed_pct = 100.0
        self._moving = False
        self.powered = False
        self.drag = False
        self.alarm = False
        self.hold_to_drag = True     # interface parity with RokaeArm (unused in sim)
        self._t = 0.0
        self._drag_base = list(self.joints)
        self._drag_cyc = -1          # drag wander cycle bookkeeping (move-then-settle)
        self._cyc_from = list(self.joints)
        self._cyc_to = list(self.joints)
        self._button = False         # simulated end-effector drag button
        self._sim_index = 0          # which keypad index the sim button maps to

        # Cartesian tool-centre-point pose (arm base frame): position in metres,
        # orientation as a quaternion [x, y, z, w]. The mock has no kinematics, so
        # this pose is tracked independently of the joints -- move_pose() drives it
        # directly so a scan path (a list of poses) can be simulated end to end.
        self._pose_pos = [0.0, 0.0, 0.0]
        self._pose_quat = [0.0, 0.0, 0.0, 1.0]
        self._pose_target_pos = None
        self._pose_target_quat = None
        self._pose_speed_mms = 0.0   # cartesian speed for the active pose move
        self._pose_moving = False

    # -- lifecycle --
    def connect(self):
        self.log("[MOCK] arm backend ready (no hardware).")

    def disconnect(self):
        self._moving = False

    # -- reads --
    def get_joints(self):
        return list(self.joints)

    def is_moving(self):
        return self._moving or self._pose_moving

    def get_status(self):
        if self.alarm:
            return "error:estop"
        if self.drag:
            return "drag"
        if not self.powered:
            return "off"
        return "moving" if (self._moving or self._pose_moving) else "idle"

    def _step_toward(self, target, speed_pct, dt):
        """Move joints toward target at speed. Returns True when reached."""
        step = self.max_speed * (speed_pct / 100.0) * dt
        done = True
        for i in range(self.n):
            d = target[i] - self.joints[i]
            if abs(d) <= step:
                self.joints[i] = target[i]
            else:
                self.joints[i] += math.copysign(step, d)
                done = False
        return done

    # -- physics tick --
    def update(self, dt):
        self._t += dt
        if self.drag:
            # simulate hand-guiding: glide to a new offset then HOLD still, repeat.
            # The hold phase lets capture-on-settle trigger (like pausing while teaching).
            cyc_len = 2.6
            k = int(self._t / cyc_len)
            if k != self._drag_cyc:
                self._drag_cyc = k
                self._cyc_from = list(self.joints)
                self._cyc_to = [self._drag_base[i] + 0.25 * math.sin(1.7 * k + i * 1.1)
                                for i in range(self.n)]
            phase = (self._t - k * cyc_len) / cyc_len
            if phase < 0.5:                        # move ~1.3 s
                f = phase / 0.5
                for i in range(self.n):
                    self.joints[i] = self._cyc_from[i] + (self._cyc_to[i] - self._cyc_from[i]) * f
            else:                                  # HOLD ~1.3 s (> settle dwell)
                self.joints = list(self._cyc_to)
            return
        if self._pose_moving:
            # Cartesian move: glide the TCP position toward the target at the
            # commanded linear speed (mm/s -> m/s); snap the orientation on
            # arrival (the mock doesn't slerp -- it only needs to demonstrate
            # move-then-settle sequencing, not smooth rotation).
            step_m = (self._pose_speed_mms / 1000.0) * dt
            done = True
            for i in range(3):
                d = self._pose_target_pos[i] - self._pose_pos[i]
                if abs(d) <= step_m or step_m <= 0.0:
                    self._pose_pos[i] = self._pose_target_pos[i]
                else:
                    self._pose_pos[i] += math.copysign(step_m, d)
                    done = False
            if done:
                self._pose_quat = list(self._pose_target_quat)
                self._pose_moving = False
            return
        if not self._moving:
            return
        if self._step_toward(self._target, self._speed_pct, dt):
            self._moving = False

    @staticmethod
    def _mms_to_pct(mms):
        """Map end-effector mm/s to the controller's joint-speed band (mirrors real SR5)."""
        if mms < 100:
            return 10.0
        if mms < 200:
            return 30.0
        if mms < 500:
            return 50.0
        if mms < 800:
            return 80.0
        return 100.0

    # -- commands --
    def move(self, target_rad, speed_mms):
        if self.alarm:
            self.log("[MOCK] move rejected: alarm/e-stop active.")
            return False
        if not self.powered:
            self.log("[MOCK] move rejected: motors off.")
            return False
        if self.drag:
            self.log("[MOCK] move rejected: in drag mode.")
            return False
        if len(target_rad) != self.n:
            self.log(f"[MOCK] move rejected: expected {self.n} joints, got {len(target_rad)}.")
            return False
        self._target = list(target_rad)
        self._speed_pct = self._mms_to_pct(speed_mms)   # mm/s -> joint-speed band
        self._moving = True
        return True

    def move_pose(self, pos_m, quat_xyzw, speed_mms):
        """Queue a cartesian move to a probe pose (position in metres, orientation
        quaternion [x,y,z,w], arm base frame). Same gates as a joint move."""
        if self.alarm:
            self.log("[MOCK] move_pose rejected: alarm/e-stop active.")
            return False
        if not self.powered:
            self.log("[MOCK] move_pose rejected: motors off.")
            return False
        if self.drag:
            self.log("[MOCK] move_pose rejected: in drag mode.")
            return False
        if len(pos_m) != 3 or len(quat_xyzw) != 4:
            self.log("[MOCK] move_pose rejected: need 3 position + 4 quaternion values.")
            return False
        self._pose_target_pos = [float(v) for v in pos_m]
        self._pose_target_quat = [float(v) for v in quat_xyzw]
        self._pose_speed_mms = float(speed_mms)
        self._pose_moving = True
        return True

    def get_pose(self):
        """Current TCP pose: (position [x,y,z] metres, quaternion [x,y,z,w])."""
        return list(self._pose_pos), list(self._pose_quat)

    def get_pose_raw(self):
        """Mock equivalent of the controller's own pose report: the simulated
        TCP as trans (m) + rpy (rad). For the mock, the sim IS the controller."""
        x, y, z, w = self._pose_quat
        # quat -> rpy (XYZ), mirror of _quat_to_rpy
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        sinp = 2.0 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return {"trans": list(self._pose_pos), "rpy": [roll, pitch, yaw], "frame": "mock (sim TCP)"}

    def set_power(self, on):
        if on and self.alarm:
            self.log("[MOCK] cannot power on: clear the alarm first.")
            return False
        self.powered = bool(on)
        if not on:
            self._moving = False
            self._pose_moving = False
        return self.powered

    def set_drag(self, on):
        if on:
            self.powered = False        # drag = hand-guide, motors relaxed (mirrors real SR5)
            self.drag = True
            self._moving = False
            self._drag_base = list(self.joints)
        else:
            self.drag = False
            self.powered = True         # re-energise so it's ready to run
        return self.drag

    def clear_alarm(self):
        self.alarm = False
        self.log("[MOCK] alarm cleared.")

    def stop(self):
        self._moving = False
        self._target = list(self.joints)
        self._pose_moving = False
        self._pose_target_pos = list(self._pose_pos)

    # -- end-effector keypad (simulated) --
    def get_keypad(self):
        ks = [False] * 7
        ks[self._sim_index] = self._button
        return ks

    def set_button(self, on):
        self._button = bool(on)

    # -- telemetry for the operator console (read-only) --
    def get_velocities(self):
        return [0.0] * self.n

    def get_torques(self):
        return [0.0] * self.n

    def device_info(self):
        return {"id": "MOCK-SR5", "type": "XMS5-R800 (mock)", "version": "mock",
                "joint_num": self.n, "power": "on" if self.powered else "off",
                "mode": "drag" if self.drag else "automatic", "sdk_version": "mock"}


# ---------------------------------------------------------------------------
# Real backend -- Rokae xCore Python SDK
# ---------------------------------------------------------------------------
class RokaeArm:
    """Wraps the xCore SDK for a real SR5. Mirrors the proven control sequence
    from the standalone Dexory Teach app (automatic mode, NrtCommandMode,
    MoveAbsJ replay). Only imported/instantiated when backend:=rokae.

    NOTE: requires the Linux SDK build (Release/linux) importable, Python
    3.8-3.12, and network reachability to the robot IP from inside WSL."""

    def __init__(self, ip, sdk_root, n_joints=6, log=print):
        self.ip = ip
        self._sdk_root = sdk_root
        self.n = n_joints
        self.log = log
        self.ec = {}
        self.robot = None
        self.sdk = None
        self._joints = [0.0] * n_joints
        self._status = "off"
        self.hold_to_drag = True     # True => must hold the end button to drag
        self._keypad_ok = True       # set False if the model doesn't support getKeypadState

    def _import_sdk(self):
        import sys
        import platform
        sys.path.insert(0, self._sdk_root)
        try:
            if platform.system() == "Windows":
                from Release.windows import xCoreSDK_python as sdk
            else:
                from Release.linux import xCoreSDK_python as sdk
            return sdk
        except Exception as e:  # noqa: BLE001
            raise RuntimeError(f"Could not import xCore SDK from {self._sdk_root}: {e}")

    def connect(self):
        self.sdk = self._import_sdk()
        sdk = self.sdk
        for name in ("Cobot_6", "xMateRobot", "xMateErProRobot"):
            cls = getattr(sdk, name, None)
            if cls is not None:
                self.robot = cls(self.ip)
                self.log(f"[ROKAE] using robot class {name}")
                break
        else:
            raise RuntimeError("No known robot class in SDK build.")
        self.robot.connectToRobot(self.ec)
        self.robot.setToolset("tool0", "wobj0", self.ec)
        self._status = "idle"
        self.log(f"[ROKAE] connected to {self.ip}")

    def disconnect(self):
        if self.robot:
            try:
                self.robot.disableDrag(self.ec)
            except Exception:  # noqa: BLE001
                pass
            self.robot.disconnectFromRobot(self.ec)

    def get_joints(self):
        return list(self._joints)

    def is_moving(self):
        return self._status == "moving"

    def get_status(self):
        return self._status

    def update(self, dt):
        # poll the robot for live joint angles + operation state
        if not self.robot:
            return
        sdk = self.sdk
        try:
            # jointPos() returns a padded array (e.g. 12 entries) -- keep the first n
            self._joints = list(self.robot.jointPos(self.ec))[:self.n]
            st = self.robot.operationState(self.ec)
            if st in (sdk.OperationState.moving, sdk.OperationState.jogging):
                self._status = "moving"
            elif st == sdk.OperationState.drag:
                self._status = "drag"
            else:
                self._status = "idle"
        except Exception as e:  # noqa: BLE001
            self._status = f"error:{e}"

    def _wait_power_on(self, timeout=8.0):
        """Energise motors (automatic mode) and WAIT until they actually report ON.
        Without this, moveStart is issued before power settles and the arm never moves."""
        sdk, r = self.sdk, self.robot
        r.setOperateMode(sdk.OperateMode.automatic, self.ec)
        r.setPowerState(True, self.ec)
        deadline = time.time() + timeout
        while time.time() < deadline:
            ps = r.powerState(self.ec)
            if ps == sdk.PowerState.on:
                return True
            if ps in (sdk.PowerState.estop, sdk.PowerState.gstop):
                self.log("[ROKAE] cannot power on — e-stop/safety active. Release e-stop "
                         "and Clear alarm.")
                return False
            time.sleep(0.1)
        self.log("[ROKAE] timed out waiting for motors to power on.")
        return False

    def _prep(self):
        sdk, r = self.sdk, self.robot
        # If the arm was left in drag/teach mode (manual + motors relaxed), a move
        # can't run: exit drag FIRST, otherwise powering on + moveStart fight the
        # hand-guide state and the arm never moves. Best-effort (no-op if not in
        # drag). This is why "Home"/"Jog" did nothing after using "Drag".
        try:
            r.disableDrag(self.ec)
        except Exception:  # noqa: BLE001
            pass
        if not self._wait_power_on():
            raise RuntimeError("motors not powered")
        r.setMotionControlMode(sdk.MotionControlMode.NrtCommandMode, self.ec)
        r.moveReset(self.ec)
        r.setToolset("tool0", "wobj0", self.ec)
        for f in (lambda: r.setDefaultConfOpt(False, self.ec),   # nearest-solution IK
                  lambda: r.setDefaultZone(10, self.ec),
                  lambda: r.setDefaultSpeed(100, self.ec)):
            try:
                f()
            except Exception:  # noqa: BLE001
                pass

    def move(self, target_rad, speed_mms):
        sdk, r = self.sdk, self.robot
        try:
            self._prep()
            # speed is the end-effector LINEAR speed in mm/s (SDK-native), as in the
            # standalone app. The controller maps it to a joint-speed band:
            #   <100:10%  100-200:30%  200-500:50%  500-800:80%  >800:100%
            cmd = sdk.MoveAbsJCommand(sdk.JointPosition(list(target_rad)), float(speed_mms))
            r.moveAppend([cmd], sdk.PyString(), self.ec)
            r.moveStart(self.ec)
            self._status = "moving"
            return True
        except Exception as e:  # noqa: BLE001
            self._status = f"error:{e}"
            self.log(f"[ROKAE] move failed: {e}")
            return False

    def move_pose(self, pos_m, quat_xyzw, speed_mms):
        """Queue a cartesian linear move to a probe pose (MoveL). The xCore
        controller resolves the pose to joints internally (nearest-solution IK,
        set in _prep via setDefaultConfOpt(False)) -- there is no separate IK
        call in this SDK.

        Pose is arm base frame: position in METRES, orientation quaternion
        [x,y,z,w]. CartesianPosition takes (trans[3], rpy[3]).

        UNIT/CONVENTION CAVEAT: this follows the SDK signature (trans metres, rpy
        radians, XYZ), but the exact units + rpy convention have NOT been verified
        against a real SR5 yet. Validate at low speed with Ra + the E-stop before
        trusting on hardware. Proven so far only against MockArm."""
        sdk, r = self.sdk, self.robot
        try:
            self._prep()
            roll, pitch, yaw = _quat_to_rpy(quat_xyzw)
            target = sdk.CartesianPosition([float(v) for v in pos_m], [roll, pitch, yaw])
            cmd = sdk.MoveLCommand(target, float(speed_mms))
            r.moveAppend([cmd], sdk.PyString(), self.ec)
            r.moveStart(self.ec)
            self._status = "moving"
            return True
        except Exception as e:  # noqa: BLE001
            self._status = f"error:{e}"
            self.log(f"[ROKAE] move_pose failed: {e}")
            return False

    def get_pose_raw(self):
        """The controller's OWN report of the flange pose, untouched: cartPosture
        (flange in the BASE frame). Returns {'trans':[x,y,z] m, 'rpy':[rx,ry,rz]
        rad, 'frame':'flangeInBase'} exactly as the SDK gives it (no frame math
        of ours), or None if unavailable. NOTE: cartPosture REQUIRES the
        CoordinateType argument — calling it without one raises (a latent bug in
        the earlier get_pose, which silently returned zeros)."""
        if not self.robot:
            return None
        try:
            posture = self.robot.cartPosture(self.sdk.CoordinateType.flangeInBase, self.ec)
            return {"trans": list(posture.trans)[:3], "rpy": list(posture.rpy)[:3],
                    "frame": "flangeInBase"}
        except Exception:  # noqa: BLE001
            return None

    def get_pose(self):
        """Current TCP pose (position [x,y,z] metres, quaternion [x,y,z,w]) from
        the controller's own report; zero pose if unavailable."""
        raw = self.get_pose_raw()
        if raw is None:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
        return raw["trans"], _rpy_to_quat(*raw["rpy"])

    def set_power(self, on):
        sdk, r = self.sdk, self.robot
        r.setOperateMode(sdk.OperateMode.automatic, self.ec)
        r.setPowerState(bool(on), self.ec)
        return on

    def set_drag(self, on):
        sdk, r = self.sdk, self.robot
        if on:
            r.setPowerState(False, self.ec)
            r.setOperateMode(sdk.OperateMode.manual, self.ec)
            r.moveReset(self.ec)
            # enable_drag_button=False  -> must HOLD the end-effector button to drag
            #   (so release is a meaningful edge for auto-capture)
            # enable_drag_button=True   -> drag freely without holding any button
            enable_drag_button = not self.hold_to_drag
            # space=cartesianSpace(1), type=freely(2)
            r.enableDrag(1, 2, self.ec, enable_drag_button)
            self._status = "drag"
        else:
            r.disableDrag(self.ec)
            r.setOperateMode(sdk.OperateMode.automatic, self.ec)
            r.setPowerState(True, self.ec)
            self._status = "idle"
        return on

    def get_keypad(self):
        """Return the 7 end-effector button states [key1..key7] as bools.
        Returns all-False if the model doesn't support it (checked once)."""
        if not self.robot or not self._keypad_ok:
            return [False] * 7
        try:
            ks = self.robot.getKeypadState(self.ec)
            return [bool(getattr(ks, f"key{i}_state")) for i in range(1, 8)]
        except Exception as e:  # noqa: BLE001
            self._keypad_ok = False
            self.log(f"[ROKAE] getKeypadState unsupported/failed ({e}); "
                     "auto-capture-on-release disabled.")
            return [False] * 7

    # -- telemetry for the operator console (read-only) --
    def get_velocities(self):
        try:
            return list(self.robot.jointVel(self.ec))[:self.n]
        except Exception:  # noqa: BLE001
            return [0.0] * self.n

    def get_torques(self):
        try:
            return list(self.robot.jointTorque(self.ec))[:self.n]
        except Exception:  # noqa: BLE001
            return [0.0] * self.n

    def device_info(self):
        d = {"id": None, "type": None, "version": None, "joint_num": None,
             "power": None, "mode": None, "sdk_version": None}
        if not self.robot:
            return d
        try:
            info = self.robot.robotInfo(self.ec)
            d.update({"id": getattr(info, "id", None), "type": getattr(info, "type", None),
                      "version": getattr(info, "version", None),
                      "joint_num": getattr(info, "joint_num", None)})
        except Exception:  # noqa: BLE001
            pass
        try:
            d["power"] = str(self.robot.powerState(self.ec))
        except Exception:  # noqa: BLE001
            pass
        try:
            d["mode"] = str(self.robot.operateMode(self.ec))
        except Exception:  # noqa: BLE001
            pass
        try:
            d["sdk_version"] = str(self.robot.sdkVersion())
        except Exception:  # noqa: BLE001
            pass
        return d

    def clear_alarm(self):
        try:
            self.robot.recoverState(1, self.ec)
        except Exception:  # noqa: BLE001
            pass
        self.robot.clearServoAlarm(self.ec)

    def stop(self):
        try:
            self.robot.stop(self.ec)   # soft stop (stop2)
        except Exception:  # noqa: BLE001
            pass
        self._status = "idle"
