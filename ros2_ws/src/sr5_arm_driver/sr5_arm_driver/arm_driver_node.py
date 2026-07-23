"""ROS 2 driver node for the Rokae SR5 arm.

Wire contract (all standard message types -- no custom interfaces):

  Publishes:
    /arm/joint_states   sensor_msgs/JointState    live joint angles (rad), ~20 Hz
    /arm/status         std_msgs/String           'idle'|'moving'|'drag'|'off'|'error:..'

  Subscribes:
    /arm/command    std_msgs/Float64MultiArray  data = [j1..jN, speed_pct]
                                                     (radians; speed_pct optional, default 100)

  Services:
    /arm/set_power      std_srvs/SetBool    energise / de-energise motors
    /arm/set_drag       std_srvs/SetBool    enable / disable hand-guide (drag) mode
    /arm/clear_alarm    std_srvs/Trigger    recover e-stop / clear servo alarm
    /arm/stop           std_srvs/Trigger    soft stop (halts current motion)
    /arm/home           std_srvs/Trigger    move to the configured home pose

Parameters:
    backend        'mock' (default) | 'rokae'
    robot_ip       '192.168.2.160'
    sdk_root       path to xCoreSDK-Python (rokae backend only)
    joint_names    ['joint1'..'joint6']
    home_deg       [0,0,0,0,0,0]
    max_speed_dps  45.0     joint speed at 100 %
    rate_hz        20.0
"""

import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray, Bool
from std_srvs.srv import SetBool, Trigger

from sr5_arm_driver.backends import MockArm, RokaeArm


class ArmDriver(Node):
    def __init__(self):
        super().__init__("ArmDriver")

        self.declare_parameter("backend", "mock")
        self.declare_parameter("robot_ip", "192.168.2.160")
        self.declare_parameter("sdk_root", os.path.expanduser("~/rokae_sdk"))
        self.declare_parameter("joint_names",
                               ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self.declare_parameter("home_deg", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("max_speed_dps", 45.0)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("hold_to_drag", True)   # True = hold end button to drag (matches standalone)
        self.declare_parameter("drag_button_key", 5)   # SR5 handle: CR5 (verified) -> capture trigger
        self.declare_parameter("read_keypad", False)   # OFF by default: polling the keypad can stall drag
        self.declare_parameter("keypad_rate_hz", 3.0)  # low rate -> don't contend with drag control

        self.backend_name = self.get_parameter("backend").value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.home_deg = list(self.get_parameter("home_deg").value)
        self.drag_button_key = int(self.get_parameter("drag_button_key").value)
        self._read_keypad = bool(self.get_parameter("read_keypad").value)
        self._kp_period = 1.0 / max(0.5, float(self.get_parameter("keypad_rate_hz").value))
        self._kp_accum = 0.0
        self._last_btn = False
        # config kept so we can build/swap backends on demand (GUI Connect button)
        self._n = len(self.joint_names)
        self._sdk_root = self.get_parameter("sdk_root").value
        self._max_speed_dps = self.get_parameter("max_speed_dps").value
        self._hold_to_drag = bool(self.get_parameter("hold_to_drag").value)
        self._robot_ip = self.get_parameter("robot_ip").value

        if self.backend_name == "rokae":
            self._connect_rokae(self._robot_ip)
        else:
            self._go_mock()

        self._prev_keypad = [False] * 7

        # publishers
        self.pub_js = self.create_publisher(JointState, "/arm/joint_states", 10)
        self.pub_status = self.create_publisher(String, "/arm/status", 10)
        self.pub_button = self.create_publisher(Bool, "/arm/drag_button", 10)
        self.pub_backend = self.create_publisher(String, "/arm/backend", 10)

        # command + connection subscribers
        self.create_subscription(Float64MultiArray, "/arm/command", self._on_move, 10)
        # payload = robot IP to connect the real arm; empty string = disconnect -> mock
        self.create_subscription(String, "/arm/connect", self._on_connect, 10)

        # services
        self.create_service(SetBool, "/arm/set_power", self._srv_power)
        self.create_service(SetBool, "/arm/set_drag", self._srv_drag)
        self.create_service(Trigger, "/arm/clear_alarm", self._srv_clear)
        self.create_service(Trigger, "/arm/stop", self._srv_stop)
        self.create_service(Trigger, "/arm/home", self._srv_home)
        self.create_service(SetBool, "/arm/sim_drag_button", self._srv_sim_button)

        rate = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"SR5 arm driver up | backend={self.backend_name} | {self._n} joints")

    # -- backend lifecycle (mock <-> real, swappable at runtime) --
    def _go_mock(self):
        self.arm = MockArm(n_joints=self._n, max_speed_dps=self._max_speed_dps,
                           log=self.get_logger().info)
        self.arm.hold_to_drag = self._hold_to_drag
        self.arm._sim_index = max(0, min(6, self.drag_button_key - 1))  # sim button -> CR<key>
        self.arm.connect()
        # Start the mock at HOME (not the zero pose) so telemetry + the sim begin at
        # the parked pose; the mock then only leaves HOME on an explicit /arm/command.
        home_rad = [math.radians(float(d)) for d in self.home_deg[:self._n]]
        self.arm.joints = list(home_rad)
        self.arm._target = list(home_rad)
        self.backend_name = "mock"
        self._target = ""

    def _connect_rokae(self, ip):
        """(Re)connect the real arm at ip. Falls back to mock on failure."""
        ip = (ip or "").strip()
        if not ip:
            return self._go_mock()
        try:
            arm = RokaeArm(ip=ip, sdk_root=self._sdk_root, n_joints=self._n,
                           log=self.get_logger().info)
            arm.hold_to_drag = self._hold_to_drag
            arm.connect()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"connect to {ip} failed: {e} — staying on mock")
            return self._go_mock()
        try:
            self.arm.disconnect()
        except Exception:  # noqa: BLE001
            pass
        self.arm = arm
        self.backend_name = "rokae"
        self._target = ip
        self.get_logger().info(f"arm connected: rokae @ {ip}")

    def _on_connect(self, msg: String):
        ip = msg.data.strip()
        if ip:
            self.get_logger().info(f"connect request -> {ip}")
            self._connect_rokae(ip)
        else:
            self.get_logger().info("disconnect request -> mock")
            try:
                self.arm.disconnect()
            except Exception:  # noqa: BLE001
                pass
            self._go_mock()

    # -- periodic --
    def _tick(self):
        self.arm.update(self._dt)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [float(x) for x in self.arm.get_joints()]
        self.pub_js.publish(js)
        self.pub_status.publish(String(data=self.arm.get_status()))
        desc = self.backend_name + (f" {self._target}" if self._target else "")
        self.pub_backend.publish(String(data=desc))

        # end-effector keypad -> publish the selected button state.
        # Poll at a LOW rate and only while dragging, so reading the keypad does not
        # contend with the drag control loop (which was stalling hand-guiding).
        if self._read_keypad and self.arm.get_status() == "drag":
            self._kp_accum += self._dt
            if self._kp_accum >= self._kp_period:
                self._kp_accum = 0.0
                keypad = self.arm.get_keypad()
                if keypad != self._prev_keypad:
                    pressed = [i + 1 for i, v in enumerate(keypad) if v]
                    self.get_logger().info(f"keypad pressed keys: {pressed or 'none'}")
                    self._prev_keypad = keypad
                idx = self.drag_button_key - 1
                self._last_btn = keypad[idx] if 0 <= idx < len(keypad) else False
        else:
            self._last_btn = False
            self._kp_accum = 0.0
        self.pub_button.publish(Bool(data=bool(self._last_btn)))

    # -- command --
    def _on_move(self, msg: Float64MultiArray):
        data = list(msg.data)
        n = len(self.joint_names)
        if len(data) < n:
            self.get_logger().warn(f"/arm/command needs >= {n} values")
            return
        target = data[:n]
        speed = data[n] if len(data) > n else 100.0
        self.arm.move(target, speed)

    # -- services --
    def _srv_power(self, req, resp):
        ok = self.arm.set_power(req.data)
        resp.success = (ok == req.data) or bool(ok)
        resp.message = f"power={'on' if self.arm.powered else 'off'}" \
            if hasattr(self.arm, "powered") else "ok"
        return resp

    def _srv_drag(self, req, resp):
        self.arm.set_drag(req.data)
        resp.success = True
        resp.message = f"drag={'on' if req.data else 'off'}"
        return resp

    def _srv_clear(self, req, resp):
        self.arm.clear_alarm()
        resp.success = True
        resp.message = "alarm cleared"
        return resp

    def _srv_stop(self, req, resp):
        self.arm.stop()
        resp.success = True
        resp.message = "stopped"
        return resp

    def _srv_home(self, req, resp):
        target = [math.radians(a) for a in self.home_deg]
        ok = self.arm.move(target, 100.0)
        resp.success = bool(ok)
        resp.message = "homing" if ok else "home rejected (check power/drag/alarm)"
        return resp

    def _srv_sim_button(self, req, resp):
        # test hook: simulate the end-effector drag button on the mock backend
        if hasattr(self.arm, "set_button"):
            self.arm.set_button(req.data)
            resp.success = True
            resp.message = f"sim button={'pressed' if req.data else 'released'}"
        else:
            resp.success = False
            resp.message = "sim button only available on the mock backend"
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = ArmDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.arm.disconnect()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
