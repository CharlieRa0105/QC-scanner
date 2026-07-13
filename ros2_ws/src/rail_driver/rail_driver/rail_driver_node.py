"""ROS 2 driver node for the linear slider (floor track, Roboteq BLDC).

Wire contract (standard messages only):

  Publishes:
    /railPos  sensor_msgs/JointState   1 prismatic joint 'rail' (metres)
    /rail/status        std_msgs/String          'idle'|'moving'|'drag'|'off'|'error:..'

  Subscribes:
    /railCMD       std_msgs/Float64MultiArray  data = [position_m, speed_pct]

  Services:
    /rail/set_power     std_srvs/SetBool
    /rail/set_drag      std_srvs/SetBool     free-wheel for hand teaching
    /rail/stop          std_srvs/Trigger
    /rail/home          std_srvs/Trigger     go to home_m

Parameters:
    backend        'mock' (default) | 'roboteq'
    joint_name     'slider'
    track_len_m    3.0
    max_speed_mps  0.5
    home_m         0.0
    port           '/dev/ttyUSB0'   (roboteq)
    baud           115200           (roboteq)
    counts_per_m   100000.0         (roboteq)
    rate_hz        20.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from rail_driver.backends import MockSlider, RoboteqSlider


class SliderDriver(Node):
    def __init__(self):
        super().__init__("RailDriver")

        self.declare_parameter("backend", "mock")
        self.declare_parameter("joint_name", "rail")
        self.declare_parameter("track_len_m", 3.0)
        self.declare_parameter("max_speed_mps", 0.5)
        self.declare_parameter("home_m", 0.0)
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("counts_per_m", 100000.0)
        self.declare_parameter("rate_hz", 20.0)

        self.backend_name = self.get_parameter("backend").value
        self.joint_name = self.get_parameter("joint_name").value
        self.home_m = float(self.get_parameter("home_m").value)
        # config kept so we can build/swap backends on demand (GUI Connect button)
        self._track = float(self.get_parameter("track_len_m").value)
        self._max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self._baud = int(self.get_parameter("baud").value)
        self._cpm = float(self.get_parameter("counts_per_m").value)
        self._port = self.get_parameter("port").value

        if self.backend_name == "roboteq":
            self._connect_roboteq(self._port)
        else:
            self._go_mock()

        self.pub_js = self.create_publisher(JointState, "/railPos", 10)
        self.pub_status = self.create_publisher(String, "/rail/status", 10)
        self.pub_backend = self.create_publisher(String, "/rail/backend", 10)
        self.create_subscription(Float64MultiArray, "/railCMD", self._on_move, 10)
        # payload = serial device to connect the real slider; empty = disconnect -> mock
        self.create_subscription(String, "/rail/connect", self._on_connect, 10)

        self.create_service(SetBool, "/rail/set_power", self._srv_power)
        self.create_service(SetBool, "/rail/set_drag", self._srv_drag)
        self.create_service(Trigger, "/rail/stop", self._srv_stop)
        self.create_service(Trigger, "/rail/home", self._srv_home)

        rate = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"Slider driver up | backend={self.backend_name} | track={self._track} m")

    # -- backend lifecycle (mock <-> real, swappable at runtime) --
    def _go_mock(self):
        self.slider = MockSlider(track_len_m=self._track, max_speed_mps=self._max_speed_mps,
                                 log=self.get_logger().info)
        self.slider.connect()
        self.backend_name = "mock"
        self._target = ""

    def _connect_roboteq(self, port):
        """(Re)connect the real slider on serial `port`. Falls back to mock on failure."""
        port = (port or "").strip()
        if not port:
            return self._go_mock()
        try:
            sl = RoboteqSlider(port=port, baud=self._baud, counts_per_m=self._cpm,
                               track_len_m=self._track, log=self.get_logger().info)
            sl.connect()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"connect to {port} failed: {e} — staying on mock")
            return self._go_mock()
        try:
            self.slider.disconnect()
        except Exception:  # noqa: BLE001
            pass
        self.slider = sl
        self.backend_name = "roboteq"
        self._target = port
        self.get_logger().info(f"slider connected: roboteq @ {port}")

    def _on_connect(self, msg: String):
        port = msg.data.strip()
        if port:
            self.get_logger().info(f"connect request -> {port}")
            self._connect_roboteq(port)
        else:
            self.get_logger().info("disconnect request -> mock")
            try:
                self.slider.disconnect()
            except Exception:  # noqa: BLE001
                pass
            self._go_mock()

    def _tick(self):
        self.slider.update(self._dt)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        js.position = [float(self.slider.get_position())]
        self.pub_js.publish(js)
        desc = self.backend_name + (f" {self._target}" if self._target else "")
        self.pub_backend.publish(String(data=desc))
        self.pub_status.publish(String(data=self.slider.get_status()))

    def _on_move(self, msg: Float64MultiArray):
        data = list(msg.data)
        if not data:
            self.get_logger().warn("/railCMD needs [position_m, speed_pct]")
            return
        pos = data[0]
        speed = data[1] if len(data) > 1 else 100.0
        self.slider.move(pos, speed)

    def _srv_power(self, req, resp):
        self.slider.set_power(req.data)
        resp.success = True
        resp.message = f"power={'on' if self.slider.powered else 'off'}"
        return resp

    def _srv_drag(self, req, resp):
        self.slider.set_drag(req.data)
        resp.success = True
        resp.message = f"drag={'on' if req.data else 'off'}"
        return resp

    def _srv_stop(self, req, resp):
        self.slider.stop()
        resp.success = True
        resp.message = "stopped"
        return resp

    def _srv_home(self, req, resp):
        ok = self.slider.move(self.home_m, 100.0)
        resp.success = bool(ok)
        resp.message = "homing" if ok else "home rejected (check power/drag)"
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = SliderDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.slider.disconnect()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
