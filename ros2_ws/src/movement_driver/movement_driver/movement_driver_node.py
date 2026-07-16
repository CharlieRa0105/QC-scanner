"""
movement_driver_node.py

MovementDriver (architecture.md §4.2): EXECUTION ONLY -- no planning.

Serves the /execute_path action (qc_msgs/ExecutePath). Given a planned
moveit_msgs/RobotTrajectory it plays the trajectory to ArmDriver point by point:
publish each joint target on /arm/command, wait until /arm/joint_states confirms
the point is reached, then advance -- emitting (point_index, total) feedback and
a /movement/state string (idle|moving|reached|error). Cancellable.

Joint feedback arrives on a reentrant callback group under a MultiThreadedExecutor
so /arm/joint_states keeps updating while the goal callback is running.

RUNTIME NOTE (build-to-interface state): a real run needs ArmDriver publishing
/arm/joint_states and a non-empty trajectory (from PathPlanner + MoveIt). The
node builds and registers /execute_path regardless; with an empty trajectory it
returns a clear error rather than doing nothing.
"""

import time

import rclpy
from qc_msgs.action import ExecutePath
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

_DEFAULT_SPEED_MMS = 30.0   # conservative end-effector speed for playback
_REACH_TOL_RAD = 0.02       # per-joint "reached" tolerance
_POINT_TIMEOUT_S = 15.0     # max wait for a single point to be reached


class MovementDriverNode(Node):
    def __init__(self):
        super().__init__("movement_driver")
        self._cbg = ReentrantCallbackGroup()
        self._cmd_pub = self.create_publisher(Float64MultiArray, "/arm/command", 10)
        self._state_pub = self.create_publisher(String, "/movement/state", 10)
        self._joints = None
        self.create_subscription(
            JointState, "/arm/joint_states", self._on_joints, 10, callback_group=self._cbg
        )
        self._server = ActionServer(
            self, ExecutePath, "/execute_path",
            execute_callback=self._execute,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self._cbg,
        )
        self._set_state("idle")
        self.get_logger().info("MovementDriver ready — action /execute_path")

    def _on_joints(self, msg):
        self._joints = list(msg.position)

    def _set_state(self, state):
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)

    def _wait_reached(self, target):
        deadline = time.time() + _POINT_TIMEOUT_S
        n = len(target)
        while time.time() < deadline:
            j = self._joints
            if j and len(j) >= n and all(abs(j[k] - target[k]) <= _REACH_TOL_RAD for k in range(n)):
                return True
            time.sleep(0.02)
        return False

    def _execute(self, goal_handle):
        result = ExecutePath.Result()
        points = goal_handle.request.trajectory.joint_trajectory.points
        total = len(points)
        if total == 0:
            self._set_state("error")
            result.success, result.message = False, "empty trajectory (nothing to execute)"
            goal_handle.abort()
            return result

        self._set_state("moving")
        for i, pt in enumerate(points):
            if goal_handle.is_cancel_requested:
                self._set_state("idle")
                result.success, result.message = False, "canceled"
                goal_handle.canceled()
                return result
            target = [float(x) for x in pt.positions]
            cmd = Float64MultiArray()
            cmd.data = target + [_DEFAULT_SPEED_MMS]   # [j1..jN, speed] per ArmDriver §4.2
            self._cmd_pub.publish(cmd)
            if not self._wait_reached(target):
                self._set_state("error")
                result.success = False
                result.message = f"point {i} not reached within {_POINT_TIMEOUT_S:.0f}s"
                goal_handle.abort()
                return result
            fb = ExecutePath.Feedback()
            fb.point_index, fb.total = i + 1, total
            goal_handle.publish_feedback(fb)

        self._set_state("reached")
        result.success, result.message = True, f"executed {total} trajectory points"
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MovementDriverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
