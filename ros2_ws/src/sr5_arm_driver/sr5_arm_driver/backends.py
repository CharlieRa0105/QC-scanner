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
import os
import time

# Cartesian-move tuning (from Ra's working xCore debugger).
DEFAULT_ZONE_MM = 10.0      # turning zone for MoveL/MoveJ point-to-point
MOVE_START_TIMEOUT_S = 3.0  # if motion hasn't started by now, the controller rejected it

# Singularity avoidance for MoveL (opt-in). setAvoidSingularity is only on the
# xMateRobot class (NOT Cobot_6), and only helps SINGULAR paths (checkPath -50102),
# not genuinely-unreachable ones (-50002). The `wrist` method slips through by
# DEVIATING orientation up to the tolerance -- fine for repositioning, but it
# degrades scan precision, so keep it off unless you need it.
#   QC_ROBOT_CLASS       force a robot class (e.g. "xMateRobot"); blank = auto
#   QC_AVOID_SINGULARITY "1" to enable setAvoidSingularity in _prep
#   QC_AVOID_SING_TOL_RAD orientation slack (rad) for the wrist method (default 0.2 ~= 11 deg)
ROBOT_CLASS_PREF = os.environ.get("QC_ROBOT_CLASS", "").strip()
AVOID_SINGULARITY = os.environ.get("QC_AVOID_SINGULARITY", "0") == "1"
AVOID_SING_TOL_RAD = float(os.environ.get("QC_AVOID_SING_TOL_RAD", "0.2"))


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

    def is_alive(self):
        """Liveness probe (mirrors RokaeArm.is_alive). The mock has no hardware
        to lose, so a mock session is always alive once constructed."""
        return True

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

    def move_pose(self, pos_m, quat_xyzw, speed_mms, linear=True, seed_conf=False,
                  allow_joint_fallback=False, search_orientation=False):
        """Queue a cartesian move to a probe pose (position in metres, orientation
        quaternion [x,y,z,w], arm base frame). Same gates as a joint move. The
        mock has no kinematics, so `linear`, `seed_conf`, and `allow_joint_fallback`
        are accepted for signature parity and ignored."""
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

    def calc_ik_ok(self, trans_m, rpy_rad):
        """Mock has no kinematics -- every pose is 'reachable' for preview."""
        return True

    def move_pose_list(self, poses, speed_mms, zone_mm=None, linear=True):
        """Mock continuous sweep: jump to the final pose (no per-point physics)."""
        if not self.powered or self.alarm or self.drag or not poses:
            self.log("[MOCK] scan sweep rejected: not ready / empty.")
            return False
        t, rpy = poses[-1]
        self._pose_target_pos = [float(v) for v in t]
        self._pose_speed_mms = float(speed_mms)
        self._pose_moving = True
        self.log(f"[MOCK] scan sweep: {len(poses)} poses (jumps to last)")
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

    def get_pose_raw_base(self):
        """Mock has one frame, so base == ref (the derived transform is identity)."""
        return self.get_pose_raw()

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
        self._avoid_on = False   # set by _prep when singularity avoidance is enabled
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
        # QC_ROBOT_CLASS forces a class (e.g. "xMateRobot", which -- unlike the
        # default Cobot_6 -- exposes setAvoidSingularity for the SR). Otherwise
        # keep the proven order (Cobot_6 first).
        order = ([ROBOT_CLASS_PREF] if ROBOT_CLASS_PREF else []) + \
                ["Cobot_6", "xMateRobot", "xMateErProRobot"]
        for name in order:
            cls = getattr(sdk, name, None)
            if cls is not None:
                self.robot = cls(self.ip)
                has_avoid = hasattr(self.robot, "setAvoidSingularity")
                self.log(f"[ROKAE] using robot class {name} "
                         f"(setAvoidSingularity {'available' if has_avoid else 'NOT available'})")
                break
        else:
            raise RuntimeError("No known robot class in SDK build.")
        self.robot.connectToRobot(self.ec)
        self.robot.setToolset("tool0", "wobj0", self.ec)
        # Log the ACTIVE tool/work-object frames once: if either is non-identity,
        # the endInRef pose is offset from the base frame and calcIk MUST use this
        # toolset (see _preflight) or far targets false-fail. This tells us whether
        # tool0/wobj0 are zero (then endInRef == flangeInBase) or not.
        try:
            ts = self.robot.toolset(self.ec)
            def _frame(f):
                t = list(getattr(f, "trans", []) or [])
                rp = list(getattr(f, "rpy", []) or [])
                return f"trans={[round(v,4) for v in t]} rpy={[round(v,4) for v in rp]}"
            self.log(f"[ROKAE] active toolset  end(tool0): {_frame(ts.end)}")
            self.log(f"[ROKAE] active toolset  ref(wobj0): {_frame(ts.ref)}")
        except Exception as e:  # noqa: BLE001
            self.log(f"[ROKAE] (could not read active toolset: {e})")
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
        # setDefaultConfOpt(False): nearest-solution IK. The target still carries
        # a copied confData (see move_pose) as a soft bias toward the current arm
        # configuration -- this is the proven recipe from Ra's working debugger.
        # (Do NOT force it True: that makes the controller ACCEPT the command but
        # silently not execute when the forced conf plan is degenerate.)
        setters = [lambda: r.setDefaultConfOpt(False, self.ec),
                   lambda: r.setDefaultZone(DEFAULT_ZONE_MM, self.ec),
                   lambda: r.setDefaultSpeed(100, self.ec)]
        for f in setters:
            try:
                f()
            except Exception:  # noqa: BLE001
                pass
        # Opt-in MoveL wrist singularity avoidance (QC_AVOID_SINGULARITY=1). Only
        # on classes that expose it (xMateRobot, not Cobot_6). `wrist` sacrifices
        # orientation up to AVOID_SING_TOL_RAD to slip through a singularity.
        self._avoid_on = False
        if AVOID_SINGULARITY:
            if hasattr(r, "setAvoidSingularity") and hasattr(sdk, "AvoidSingularityMethod"):
                ec_av = {}
                try:
                    r.setAvoidSingularity(sdk.AvoidSingularityMethod.wrist, True,
                                          AVOID_SING_TOL_RAD, ec_av)
                    err = self._ec_str(ec_av)
                    if err:
                        self.log(f"[ROKAE] setAvoidSingularity(wrist) FAILED: {err}")
                    else:
                        self._avoid_on = True
                        self.log(f"[ROKAE] singularity avoidance ON (wrist, tol="
                                 f"{AVOID_SING_TOL_RAD} rad ~= {math.degrees(AVOID_SING_TOL_RAD):.0f} deg)")
                except Exception as e:  # noqa: BLE001
                    self.log(f"[ROKAE] setAvoidSingularity raised: {e}")
            else:
                self.log("[ROKAE] QC_AVOID_SINGULARITY=1 but this robot class lacks "
                         "setAvoidSingularity -- set QC_ROBOT_CLASS=xMateRobot")

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

    # Known xCore move error codes -> plain-English meaning + what to do. Extend
    # as we hit more. Surfaced verbatim in the log so nothing is hidden.
    _MOVE_ERR = {
        50002: ("MoveL target is a CARTESIAN SINGULARITY (or reach/limit boundary)",
                "MoveL forces a straight-line, orientation-held path -- near a wrist/shoulder/elbow "
                "singularity the required joint speeds blow up and it is refused. Use MoveJ "
                "(joint-interpolated), nudge the target orientation off the singular alignment, "
                "or move via joints (MoveAbsJ)."),
        50023: ("MoveJ target picked a SINGULAR IK branch (confData empty/incorrect)",
                "No confData was given so nearest-solution IK landed on a singular branch. "
                "Seed the target's confData from the current pose (seed_conf) OR park the arm "
                "away from the singularity first."),
        50021: ("MoveJ target has NO SOLUTION in the specified confData",
                "The forced conf (copied from the current pose) has no IK solution at the target "
                "-- the move crosses a configuration/singularity boundary. The arm is likely "
                "parked NEAR a Cartesian singularity: re-park it in a dexterous mid-workspace "
                "pose (elbow + wrist clearly bent) via joint jogging, then retry."),
    }

    @staticmethod
    def _ec_code(ec):
        """Integer error code from an SDK ec dict (0 == success)."""
        if isinstance(ec, dict):
            for key in ("ec", "error", "code", "value"):
                v = ec.get(key)
                if v is not None:
                    return getattr(v, "value", v)
        return 0

    @staticmethod
    def _ec_str(ec):
        """Render an SDK ec (error-code) dict/object as a short string, or '' if
        it signals no error. The SDK writes into the dict/obj we pass; shape is
        version-dependent so we probe the usual fields."""
        if not ec:
            return ""
        # The xCore python binding fills ec as {'ec': <int code>, 'message': <str>}
        # (0 == success). Older shapes used 'error'/'code'/'value'; handle both.
        for key in ("ec", "error", "code", "value"):
            v = ec.get(key) if isinstance(ec, dict) else getattr(ec, key, None)
            if v is None:
                continue
            code = getattr(v, "value", v)
            if isinstance(code, int) and code == 0:
                return ""  # success
            msg = getattr(v, "message", "") or (ec.get("message", "") if isinstance(ec, dict) else "")
            return f"code={code} {msg}".strip()
        return str(ec)

    def _dump_cart(self, tag, cp):
        """Log a CartesianPosition in full: trans (mm), rpy (deg), and the
        confData / elbow / external the controller cares about for IK."""
        try:
            trans = [round(v * 1000, 2) for v in list(cp.trans)]
            rpy = [round(math.degrees(v), 2) for v in list(cp.rpy)]
            conf = list(cp.confData) if getattr(cp, "confData", None) is not None else None
            self.log(f"[ROKAE]   {tag}: trans_mm={trans} rpy_deg={rpy} "
                     f"confData={conf} hasElbow={getattr(cp, 'hasElbow', None)} "
                     f"elbow={getattr(cp, 'elbow', None)}")
        except Exception as e:  # noqa: BLE001
            self.log(f"[ROKAE]   {tag}: <dump failed: {e}>")

    def _tcp_trans(self):
        """Current flange translation (metres) from the controller, or None."""
        try:
            return list(self.robot.cartPosture(self.sdk.CoordinateType.endInRef, {}).trans)
        except Exception:  # noqa: BLE001
            return None

    def _move_error(self):
        """Return a decoded moveExecution Error string if the controller has
        rejected/aborted the current move, else None. Logs the raw event when it
        carries an error. (moveStart's own ec stays clean -- IK/limit/singularity
        failures surface only through this event.)"""
        sdk, r = self.sdk, self.robot
        K = sdk.EventInfoKey.MoveExecution
        try:
            info = r.queryEventInfo(sdk.Event.moveExecution, {})
        except Exception:  # noqa: BLE001
            return None
        err = info.get(K.Error)
        code = getattr(err, "value", 0) or 0
        if not code:
            return None
        msg = (getattr(err, "message", "") or "").strip()
        remark = (info.get(K.Remark) or "").strip()
        self.log(f"[ROKAE]   moveExecution: cmdID={info.get(K.ID)} errCode={code} "
                 f"errMsg='{msg}' remark='{remark}'")
        decoded = self._MOVE_ERR.get(code)
        extra = f" -> {decoded[0]}. FIX: {decoded[1]}" if decoded else ""
        return f"controller rejected (code {code}): {msg}" + (f" [{remark}]" if remark else "") + extra

    def _move_outcome(self):
        """Confirm a just-started NRT move by its OPERATION STATE, the way Ra's
        working debugger does -- not by the moveExecution event alone, which
        showed errCode=0 while the arm sat still.

        Two phases:
          1) wait up to MOVE_START_TIMEOUT_S for operationState -> moving/jogging.
             If a moveExecution error appears -> rejected (return it). If neither
             happens (state stays idle) -> the controller silently refused it.
          2) once moving, return (True, 'in motion'): we do NOT block for
             completion (a long move must not freeze the status poll / lock).
             The status endpoint reflects moving->idle afterwards."""
        sdk, r = self.sdk, self.robot
        moving = (sdk.OperationState.moving, sdk.OperationState.jogging)
        t0 = time.time()
        while time.time() - t0 < MOVE_START_TIMEOUT_S:
            err = self._move_error()
            if err:
                return False, err
            try:
                st = r.operationState({})
            except Exception:  # noqa: BLE001
                st = None
            if st in moving:
                self.log(f"[ROKAE]   operationState={st} -> motion STARTED")
                return True, "in motion"
            time.sleep(0.05)
        # Didn't start and no error: the classic silent no-move. Return an explicit
        # failure tuple -- never fall through to None (that crashed / mis-reported
        # callers that unpack the result).
        err = self._move_error()
        if err:
            return False, err
        return False, "no motion within start timeout (controller accepted but arm stayed idle)"

    def _calcik_ok(self, trans_m, rpy_rad, model, toolset):
        """True if the controller's offline IK finds a solution for this exact
        POSE (position + orientation). No motion."""
        sdk = self.sdk
        pose = sdk.CartesianPosition([float(v) for v in trans_m],
                                     [float(a) for a in rpy_rad])
        ec = {}
        joints = model.calcIk(pose, toolset, ec)
        return self._ec_code(ec) == 0 and bool(joints)

    def _search_orientation(self, trans_m, preferred_rpy):
        """Find an orientation for which the target POSITION is reachable ("just
        get the head to the point, any orientation" -- Ra). A Cartesian pose is
        position+orientation; a point reachable at one tool angle is often NOT at
        another, so holding the current angle false-fails far points. We try the
        preferred (held) angle first -- smooth when it works -- then sweep angles
        and return the first that calcIk accepts. Returns rpy (rad) or None if the
        POSITION is unreachable at every angle tried (genuinely out of the region).

        Returns preferred_rpy unchanged if calcIk is unavailable (can't search)."""
        sdk, r = self.sdk, self.robot
        try:
            model = r.model()
            try:
                toolset = r.toolset(self.ec)
            except Exception:  # noqa: BLE001
                toolset = sdk.Toolset()
        except Exception as e:  # noqa: BLE001
            self.log(f"[ROKAE]   orientation search: calcIk unavailable ({e}) -- keeping held angle")
            return preferred_rpy

        if self._calcik_ok(trans_m, preferred_rpy, model, toolset):
            return preferred_rpy
        self.log("[ROKAE]   held orientation infeasible at target -> searching for a reachable one...")
        # Coarse sweep. Prefer small tilt from 'tool along the arm' first by
        # ordering pitch/roll near 0; yaw is cheap to vary. ~320 fast offline IK
        # calls worst case.
        rolls = [0, 45, -45, 90, -90, 135, -135, 180]
        pitches = [0, -45, 45, -90, 90]
        yaws = [0, 45, 90, 135, 180, -45, -90, -135]
        for p in pitches:
            for rr in rolls:
                for y in yaws:
                    rpy = [math.radians(rr), math.radians(p), math.radians(y)]
                    if self._calcik_ok(trans_m, rpy, model, toolset):
                        self.log(f"[ROKAE]   found reachable orientation rpy_deg=[{rr}, {p}, {y}]")
                        return rpy
        return None

    def _preflight(self, target, start_pose, start_joints, linear):
        """Check reachability BEFORE moving (NO motion), the way Ra's debugger
        does -- because the controller rejects singular MoveJ targets AFTER
        moveStart via the pendant/planner, too late for our post-move poll.

          A) offline IK (model().calcIk): is the ENDPOINT reachable at all?
             MoveJ needs only this.
          B) checkPath: is the straight-line MoveL path clean? MoveL needs this.

        Returns (ik_ok, path_ok, detail):
          ik_ok   -- endpoint reachable (calcIk). None-safe: True if calcIk is
                     unavailable, so a missing API never blocks a valid move.
          path_ok -- straight-line MoveL path clean. True/False, or None when not
                     checked (non-linear, or checkPath unavailable).
          detail  -- human message for the failing case.
        The caller decides: MoveJ needs only ik_ok; MoveL needs path_ok; go-to
        falls back MoveL->MoveJ when path_ok is False but ik_ok is True."""
        sdk, r = self.sdk, self.robot
        # A) offline inverse kinematics -- endpoint reachable?
        try:
            ik_ec = {}
            # Use the ARM's ACTIVE toolset (tool0/wobj0), not a default zero one:
            # our target is in endInRef (end-in-work-object), so calcIk must resolve
            # against that same reference frame. A zero Toolset assumes ref=base,
            # which -- if wobj0 != base -- mis-locates the target (fine near the
            # current pose, but tips FAR targets to a false -32). Fall back to the
            # zero toolset only if the getter is unavailable.
            try:
                toolset = r.toolset(self.ec)
            except Exception:  # noqa: BLE001
                toolset = sdk.Toolset()
            joints = r.model().calcIk(target, toolset, ik_ec)
            code = self._ec_code(ik_ec)
            if code == 0 and joints:
                js = ", ".join(f"{math.degrees(j):.1f}" for j in list(joints)[:6])
                self.log(f"[ROKAE]   preflight calcIk: OK  joints_deg=[{js}]")
            else:
                self.log(f"[ROKAE]   preflight calcIk: FAIL {self._ec_str(ik_ec) or f'code {code}'}")
                return False, None, ("target endpoint has NO IK solution -- unreachable, or SINGULAR "
                                     "at this orientation. Change the orientation, or jog there.")
        except Exception as e:  # noqa: BLE001
            self.log(f"[ROKAE]   preflight calcIk: unavailable ({e}) -- skipping preflight")
            return True, None, "preflight skipped (calcIk unavailable)"

        # B) controller straight-line path check -- MoveL only.
        if linear:
            try:
                cp_ec = {}
                r.checkPath(start_pose, [float(j) for j in start_joints], target, cp_ec)
                code = self._ec_code(cp_ec)
                if code == 0:
                    self.log("[ROKAE]   preflight checkPath: OK (straight-line MoveL path clean)")
                    return True, True, "reachable, path clean"
                if code == -50102 and getattr(self, "_avoid_on", False):
                    self.log("[ROKAE]   preflight checkPath: singular (-50102) but avoidance ON "
                             "-> allowing MoveL (orientation may deviate near the singularity)")
                    return True, True, "reachable, singular path allowed via avoidance"
                self.log(f"[ROKAE]   preflight checkPath: FAIL {self._ec_str(cp_ec) or f'code {code}'}")
                why = ("straight-line path is SINGULAR" if code == -50102
                       else "straight-line path leaves the workspace")
                return True, False, f"endpoint reachable but {why} ({self._ec_str(cp_ec) or code})"
            except Exception as e:  # noqa: BLE001
                self.log(f"[ROKAE]   preflight checkPath: unavailable ({e})")
        return True, None, "reachable"

    def move_pose(self, pos_m, quat_xyzw, speed_mms, linear=True, seed_conf=False,
                  allow_joint_fallback=False, search_orientation=False):
        """Queue a cartesian move to a probe pose and report the controller's REAL
        outcome (see _move_outcome), logging every step. The xCore controller
        resolves the pose to joints internally -- there is no separate IK call.

        linear=True  -> MoveL: straight-line TCP path, orientation held along the
                        whole line. Use for SCANNING trajectories (the path matters);
                        the entire line must be reachable AND non-singular.
        linear=False -> MoveJ: joint-interpolated to the same cartesian target.
                        Use for POINT-TO-POINT positioning ("go to point"): only
                        the endpoint must be reachable; the controller picks the
                        joint path. This is the controller's own advice for a
                        MoveL singularity ("move the robot by using its joints").

        allow_joint_fallback: if a MoveL is requested but its straight-line path
                        fails the pre-flight while the ENDPOINT is still reachable,
                        automatically fall back to MoveJ so the arm still gets
                        there. This is what makes "go to point" (a debug probe)
                        just work; scanning leaves it False (straight lines only).

        seed_conf: copy the CURRENT pose's confData/elbow onto the target and
                   force the controller to honour it (setDefaultConfOpt(True)).
                   A CartesianPosition built from (trans, rpy) alone has EMPTY
                   confData, and MoveJ's IK then picks a singular/wrong branch
                   (error 50023 "target point conf information is incorrect").
                   For a small move from a valid current pose, the current conf
                   is the right branch -- this is the fix the controller asks for
                   ("modify the target point conf"). Leave False for multi-waypoint
                   scanning, where each far waypoint may need a different conf and
                   nearest-solution IK (setDefaultConfOpt(False)) is wanted.

        Pose is arm base frame: position in METRES, orientation quaternion
        [x,y,z,w]. CartesianPosition takes (trans[3], rpy[3]). Speed is the
        SDK-native end-effector speed (mm/s) for MoveL; MoveJ maps it likewise."""
        sdk, r = self.sdk, self.robot
        move = "MoveL" if linear else "MoveJ"
        try:
            self._prep()
            roll, pitch, yaw = _quat_to_rpy(quat_xyzw)

            self.log(f"[ROKAE] {move} speed={speed_mms} mm/s  (linear={linear} seed_conf={seed_conf} "
                     f"search_orientation={search_orientation})")

            # "Just get the head to the point, any orientation" (debug go-to): if
            # the requested tool angle has no IK solution at this position, sweep
            # for one that does. A straight-line MoveL can't hold an arbitrary
            # found orientation, so force MoveJ when we had to change it.
            if search_orientation:
                found = self._search_orientation(pos_m, [roll, pitch, yaw])
                if found is None:
                    self._status = "error:position unreachable at any orientation"
                    self.log(f"[ROKAE] {move} PRE-FLIGHT FAILED (no motion sent): POSITION "
                             f"{[round(v*1000,1) for v in pos_m]} mm is unreachable at EVERY orientation "
                             "tried -- it is outside the feasible region.")
                    return False
                if [round(a, 6) for a in found] != [round(roll, 6), round(pitch, 6), round(yaw, 6)]:
                    roll, pitch, yaw = found
                    linear = False   # arbitrary new orientation -> joint move
                    move = "MoveJ"
            target = sdk.CartesianPosition([float(v) for v in pos_m], [roll, pitch, yaw])
            try:  # ground truth: where the flange IS right now, for comparison
                cur = r.cartPosture(sdk.CoordinateType.endInRef, self.ec)
                self._dump_cart("CURRENT endInRef", cur)
            except Exception as e:  # noqa: BLE001
                cur = None
                self.log(f"[ROKAE]   (could not read current pose: {e})")
            try:  # NRT SDK moves only execute in AUTOMATIC mode with motors on
                self.log(f"[ROKAE]   state: power={r.powerState(self.ec)} "
                         f"operateMode={r.operateMode(self.ec)} "
                         f"motionMode={r.operationState(self.ec)}")
            except Exception as e:  # noqa: BLE001
                self.log(f"[ROKAE]   (could not read power/mode: {e})")

            self._dump_cart("TARGET  (bare)", target)

            # Pre-flight reachability (NO motion) on the BARE target -- NO confData.
            # calcIk MUST be free to find ANY arm configuration: constraining it
            # with the current pose's confData made a target on the far side of the
            # workspace (which needs a DIFFERENT config) falsely report -32 "no IK
            # solution" even though it is reachable (just in another config). This
            # was the bug -- the copied conf broke the check while doing nothing for
            # the move (setDefaultConfOpt(False) ignores confData anyway).
            try:
                start_joints = list(r.jointPos(self.ec))[:6]
            except Exception:  # noqa: BLE001
                start_joints = []
            ik_ok, path_ok, pf_detail = self._preflight(target, cur, start_joints, linear)
            if not ik_ok:
                self._status = f"error:{pf_detail}"
                self.log(f"[ROKAE] {move} PRE-FLIGHT FAILED (no motion sent): {pf_detail}")
                return False
            if linear and path_ok is False:
                if allow_joint_fallback:
                    self.log(f"[ROKAE]   MoveL path unusable ({pf_detail}) -> FALLING BACK to MoveJ "
                             "(endpoint reachable; joint-interpolated).")
                    linear = False
                    move = "MoveJ"
                else:
                    self._status = f"error:{pf_detail}"
                    self.log(f"[ROKAE] MoveL PRE-FLIGHT FAILED (no motion sent): {pf_detail} "
                             "-- use MoveJ / a cleaner path.")
                    return False

            # confData soft-bias for the command ONLY (after the check). With
            # setDefaultConfOpt(False) the controller ignores it and picks the
            # nearest solution to the current joints -- kept for parity, and never
            # on the target during calcIk (see above).
            if seed_conf and cur is not None:
                try:
                    target.confData = list(cur.confData) if cur.confData is not None else []
                    if getattr(cur, "hasElbow", False):
                        target.hasElbow = True
                        target.elbow = cur.elbow
                except Exception as e:  # noqa: BLE001
                    self.log(f"[ROKAE]   (confData seed failed: {e})")

            cmd = (sdk.MoveLCommand if linear else sdk.MoveJCommand)(
                target, float(speed_mms), DEFAULT_ZONE_MM)

            ec_append = {}
            r.moveAppend([cmd], sdk.PyString(), ec_append)   # empty path id (as MoveAbsJ)
            ea = self._ec_str(ec_append)
            self.log(f"[ROKAE]   moveAppend: {ea or 'ok'}")
            ec_start = {}
            r.moveStart(ec_start)
            es = self._ec_str(ec_start)
            self.log(f"[ROKAE]   moveStart: {es or 'ok'}")

            ok, detail = self._move_outcome()
            if not ok:
                self._status = f"error:{detail}"
                self.log(f"[ROKAE] {move} REJECTED: {detail}")
                return False
            self._status = "moving"
            self.log(f"[ROKAE] {move} accepted -> {detail}")
            return True
        except Exception as e:  # noqa: BLE001
            self._status = f"error:{e}"
            self.log(f"[ROKAE] {move} failed (exception): {e}")
            return False

    def calc_ik_ok(self, trans_m, rpy_rad):
        """True if the controller's offline IK (calcIk) finds a solution for this
        endInRef pose. NO motion. Returns None if calcIk is unavailable (unknown).
        Used by the oriented-scan cone search to test candidate orientations before
        committing a continuous sweep. Uses the ACTIVE toolset (see _preflight)."""
        sdk, r = self.sdk, self.robot
        try:
            try:
                ts = r.toolset(self.ec)
            except Exception:  # noqa: BLE001
                ts = sdk.Toolset()
            pose = sdk.CartesianPosition([float(v) for v in trans_m], [float(a) for a in rpy_rad])
            ec = {}
            joints = r.model().calcIk(pose, ts, ec)
            return self._ec_code(ec) == 0 and bool(joints)
        except Exception:  # noqa: BLE001
            return None

    def move_pose_list(self, poses, speed_mms, zone_mm=None, linear=True):
        """CONTINUOUS scan sweep: append EVERY pose with a turning zone and moveStart
        ONCE, so the controller blends through them without stopping. poses =
        [(trans_m[3], rpy_rad[3]), ...] in the endInRef frame, pre-checked reachable.

        linear=True  -> MoveL: straight-line TCP path (the whole path must be
                        reachable AND non-singular; strict).
        linear=False -> MoveJ: joint-interpolated blend through the poses -- only each
                        pose must be reachable, not the straight-line path between
                        them. Needed for the dome (inward-facing orientations whose
                        straight-line MoveL path is singular/unreachable).

        Confirms motion STARTED; the caller waits for completion. Returns True if it
        started."""
        sdk, r = self.sdk, self.robot
        z = float(zone_mm if zone_mm is not None else DEFAULT_ZONE_MM)
        try:
            self._prep()
            CmdCls = sdk.MoveLCommand if linear else sdk.MoveJCommand
            cmds = [CmdCls(sdk.CartesianPosition([float(v) for v in t], [float(a) for a in rpy]),
                           float(speed_mms), z)
                    for (t, rpy) in poses]
            self.log(f"[ROKAE] scan sweep: {len(cmds)} blended {'MoveL' if linear else 'MoveJ'} "
                     f"poses, speed={speed_mms} mm/s zone={z} mm")
            # The controller rejects a single moveAppend with too many commands
            # (observed: 76 ok, 107 -> code=259 'argument invalid'). Append in
            # chunks UNDER that cap but moveStart ONCE, so the whole line still runs
            # as one continuous blended sweep -- the queue accumulates across appends.
            MAX_APPEND = 50
            for i in range(0, len(cmds), MAX_APPEND):
                chunk = cmds[i:i + MAX_APPEND]
                ec_a = {}
                r.moveAppend(chunk, sdk.PyString(), ec_a)
                ea = self._ec_str(ec_a)
                self.log(f"[ROKAE]   moveAppend({len(chunk)}): {ea or 'ok'}")
                if ea:                     # a rejected append -> stop cleanly, don't moveStart an empty/partial cache
                    self._status = f"error:{ea}"
                    self.log(f"[ROKAE] scan sweep REJECTED at moveAppend: {ea}")
                    return False
            ec_s = {}
            r.moveStart(ec_s)
            self.log(f"[ROKAE]   moveStart: {self._ec_str(ec_s) or 'ok'}")
            outcome = self._move_outcome()
            ok, detail = outcome if outcome else (False, "no motion outcome")
            if not ok:
                self._status = f"error:{detail}"
                self.log(f"[ROKAE] scan sweep REJECTED: {detail}")
                return False
            self._status = "moving"
            self.log(f"[ROKAE] scan sweep started -> {detail}")
            return True
        except Exception as e:  # noqa: BLE001
            self._status = f"error:{e}"
            self.log(f"[ROKAE] scan sweep failed (exception): {e}")
            return False

    def get_pose_raw(self):
        """The controller's OWN report of the TCP pose, untouched: cartPosture in
        the endInRef frame (end-effector in the reference/work-object frame) --
        the SAME frame Ra's working debugger reads, commands, and runs calcIk in.
        Returns {'trans':[x,y,z] m, 'rpy':[rx,ry,rz] rad, 'frame':'endInRef'} as
        the SDK gives it (no frame math of ours), or None if unavailable. NOTE:
        cartPosture REQUIRES the CoordinateType argument."""
        if not self.robot:
            return None
        try:
            posture = self.robot.cartPosture(self.sdk.CoordinateType.endInRef, self.ec)
            return {"trans": list(posture.trans)[:3], "rpy": list(posture.rpy)[:3],
                    "frame": "endInRef"}
        except Exception:  # noqa: BLE001
            return None

    def get_pose_raw_base(self):
        """The current flange pose in the BASE frame (cartPosture flangeInBase).
        Used ONCE at connect (with get_pose_raw's endInRef) to derive the fixed
        flangeInBase->endInRef transform, since the controller COMMANDS in endInRef
        but our table->arm math produces flangeInBase. Same shape as get_pose_raw."""
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

    def is_alive(self):
        """Return True only if the controller session genuinely answers a live
        read. Unlike device_info() -- which swallows every SDK error and returns
        a dict of Nones -- this is an unambiguous boolean probe used by the
        console to detect a dead session (controller off / cable pulled / SDK
        session dropped) that ICMP ping alone cannot see: the controller's NIC
        (or a switch) keeps answering ping after the arm session has died.

        powerState() is a cheap read that requires a live session; any exception
        (or a None result) means the session is not usable. May BLOCK if the SDK
        call hangs on a dead link -- callers must bound it with a timeout."""
        if not self.robot:
            return False
        try:
            return self.robot.powerState(self.ec) is not None
        except Exception:  # noqa: BLE001
            return False

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
