"""
scanner_driver_node.py

ScanningDriver (architecture.md §4.2): the MIRACO Plus bridge.

  * /scan/start  (service, std_srvs/Trigger)  -- begin continuous capture
  * /scan/stop   (service, std_srvs/Trigger)  -- end capture, export the cloud
  * /scan/state  (topic, latched, qc_msgs/ScanState)  -- idle|scanning|done|error

INTERFACE ONLY (hardware-blocked): the Revopoint MIRACO Plus isn't connected and
has no SDK here, so this driver does NOT capture anything. It honestly reports
state transitions (idle -> scanning -> done) with an EMPTY cloud_path and a detail
saying capture is not implemented. It never fabricates a point cloud. Wire the
real capture + export here once the scanner SDK lands.
"""

import rclpy
from qc_msgs.msg import ScanState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger

_LATCHED = QoSProfile(depth=1)
_LATCHED.durability = DurabilityPolicy.TRANSIENT_LOCAL

_NOT_IMPL = "scanner capture not implemented (no MIRACO Plus hardware/SDK yet)"


class ScannerDriverNode(Node):
    def __init__(self):
        super().__init__("scanner_driver")
        self._state_pub = self.create_publisher(ScanState, "/scan/state", _LATCHED)
        self.create_service(Trigger, "/scan/start", self._on_start)
        self.create_service(Trigger, "/scan/stop", self._on_stop)
        self._publish("idle", detail="ready (interface only)")
        self.get_logger().info("ScanningDriver ready — /scan/start, /scan/stop, /scan/state (INTERFACE ONLY)")

    def _publish(self, state, cloud_path="", detail=""):
        msg = ScanState()
        msg.state, msg.cloud_path, msg.detail = state, cloud_path, detail
        self._state_pub.publish(msg)

    def _on_start(self, request, response):
        # Real driver would start continuous capture here.
        self._publish("scanning", detail=_NOT_IMPL)
        response.success = True
        response.message = "capture 'started' — " + _NOT_IMPL
        return response

    def _on_stop(self, request, response):
        # Real driver would stop + export the cloud, setting cloud_path.
        self._publish("done", cloud_path="", detail=_NOT_IMPL)
        response.success = True
        response.message = "capture 'stopped' — " + _NOT_IMPL + " (no cloud exported)"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ScannerDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
