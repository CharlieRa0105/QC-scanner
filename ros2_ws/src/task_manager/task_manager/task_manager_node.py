"""
task_manager_node.py

TaskManager (architecture.md §3, §4.2): the mission orchestrator. Owns the
mission lifecycle + /mission/state, and sequences one scan mission:

    plan  ->  (operator confirm)  ->  execute  ->  scan  ->  inspect

Interfaces:
  * /mission/plan     (service, qc_msgs/StartMission)  -- plan for a part (no motion)
  * /mission/execute  (service, std_srvs/Trigger)      -- operator confirm; run the mission
  * /mission/abort    (service, std_srvs/Trigger)      -- cancel + stop
  * /mission/state    (topic, latched, qc_msgs/MissionState)
  * /plan_path, /execute_path, /inspect  (action clients)
  * /scan/start, /scan/stop  (service clients, std_srvs/Trigger)
  * /scan/state       (topic sub, qc_msgs/ScanState)

Rescan loop (decision 8): if inspection reports a quality fail, re-plan/re-execute
up to `max_rescans` (a ROS param, default 2), then flag for a human.

RUNTIME NOTE (build-to-interface state): a full run needs PathPlanner,
MovementDriver, ScanningDriver and InspectionNode all up (and, downstream, MoveIt
+ the arm). The node builds and registers every interface above; the sequencing
is real, but end-to-end execution can't be exercised until those are running.
"""

import threading
import time

import rclpy
from qc_msgs.action import ExecutePath, Inspect, PlanPath
from qc_msgs.msg import MissionState, ScanState
from qc_msgs.srv import StartMission
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger

_LATCHED = QoSProfile(depth=1)
_LATCHED.durability = DurabilityPolicy.TRANSIENT_LOCAL

# mission phases (mirror MissionState.msg)
IDLE, PLANNING, PLANNED = "idle", "planning", "planned"
EXECUTING, SCANNING, INSPECTING = "executing", "scanning", "inspecting"
COMPLETE, RESCANNING, ABORTED, ERROR = "complete", "rescanning", "aborted", "error"


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager")
        self.declare_parameter("max_rescans", 2)
        self._cbg = ReentrantCallbackGroup()

        self._state_pub = self.create_publisher(MissionState, "/mission/state", _LATCHED)
        self._part_id = ""
        self._attempt = 0
        self._phase = IDLE
        self._plan_result = None
        self._scan_state = None
        self._abort = threading.Event()
        self._busy = threading.Lock()

        # servers
        self.create_service(StartMission, "/mission/plan", self._on_plan, callback_group=self._cbg)
        self.create_service(Trigger, "/mission/execute", self._on_execute, callback_group=self._cbg)
        self.create_service(Trigger, "/mission/abort", self._on_abort, callback_group=self._cbg)
        # clients
        self._plan_cli = ActionClient(self, PlanPath, "/plan_path", callback_group=self._cbg)
        self._exec_cli = ActionClient(self, ExecutePath, "/execute_path", callback_group=self._cbg)
        self._inspect_cli = ActionClient(self, Inspect, "/inspect", callback_group=self._cbg)
        self._scan_start = self.create_client(Trigger, "/scan/start", callback_group=self._cbg)
        self._scan_stop = self.create_client(Trigger, "/scan/stop", callback_group=self._cbg)
        self.create_subscription(ScanState, "/scan/state", self._on_scan_state, 10, callback_group=self._cbg)

        self._publish_state(IDLE, detail="ready")
        self.get_logger().info("TaskManager ready — /mission/plan, /mission/execute, /mission/abort")

    # ---- state ----
    def _publish_state(self, phase, detail=""):
        self._phase = phase
        msg = MissionState()
        msg.phase, msg.part_id, msg.attempt, msg.detail = phase, self._part_id, self._attempt, detail
        self._state_pub.publish(msg)

    def _on_scan_state(self, msg):
        self._scan_state = msg

    # ---- helpers: block on a future while the executor keeps spinning ----
    def _await(self, future, timeout=120.0):
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline and not self._abort.is_set():
            time.sleep(0.05)
        return future.result() if future.done() else None

    def _await_action(self, client, goal, timeout=300.0):
        """Send an action goal, wait for the result. Returns (result, goal_handle)."""
        if not client.wait_for_server(timeout_sec=5.0):
            return None, None
        gh_future = client.send_goal_async(goal)
        goal_handle = self._await(gh_future, timeout=10.0)
        if goal_handle is None or not goal_handle.accepted:
            return None, goal_handle
        res_future = goal_handle.get_result_async()
        wrapped = self._await(res_future, timeout=timeout)
        return (wrapped.result if wrapped else None), goal_handle

    # ---- /mission/plan ----
    def _on_plan(self, request, response):
        if self._phase not in (IDLE, COMPLETE, ABORTED, ERROR):
            response.accepted, response.message = False, f"busy (phase={self._phase})"
            return response
        self._abort.clear()
        self._part_id, self._attempt = request.part_id, 1
        self._publish_state(PLANNING, detail=f"planning {request.part_id}")
        goal = PlanPath.Goal()
        goal.part_id = request.part_id
        result, _ = self._await_action(self._plan_cli, goal, timeout=300.0)
        if result and result.success:
            self._plan_result = result
            self._publish_state(PLANNED, detail=result.message)
            response.accepted, response.message = True, result.message
        else:
            self._plan_result = None
            self._publish_state(ERROR, detail="plan failed")
            response.accepted, response.message = False, (result.message if result else "PathPlanner unavailable")
        return response

    # ---- /mission/execute (operator confirm) ----
    def _on_execute(self, request, response):
        if self._phase != PLANNED or self._plan_result is None:
            response.success, response.message = False, f"nothing planned (phase={self._phase})"
            return response
        threading.Thread(target=self._run_mission, daemon=True).start()
        response.success, response.message = True, "mission started"
        return response

    def _run_mission(self):
        if not self._busy.acquire(blocking=False):
            return
        try:
            max_rescans = self.get_parameter("max_rescans").value
            while not self._abort.is_set():
                # scan/start -> execute_path -> scan/stop -> inspect
                self._publish_state(EXECUTING, detail=f"attempt {self._attempt}")
                self._call_scan(self._scan_start)
                self._publish_state(SCANNING)
                exec_goal = ExecutePath.Goal()
                exec_goal.trajectory = self._plan_result.trajectory
                exec_res, _ = self._await_action(self._exec_cli, exec_goal, timeout=600.0)
                self._call_scan(self._scan_stop)
                if self._abort.is_set():
                    break
                if not (exec_res and exec_res.success):
                    self._publish_state(ERROR, detail=(exec_res.message if exec_res else "execute failed"))
                    return
                self._publish_state(INSPECTING)
                cloud = self._scan_state.cloud_path if self._scan_state else ""
                insp_goal = Inspect.Goal()
                insp_goal.cloud_path, insp_goal.part_id = cloud, self._part_id
                insp_res, _ = self._await_action(self._inspect_cli, insp_goal, timeout=300.0)
                if not insp_res:
                    self._publish_state(ERROR, detail="inspection unavailable")
                    return
                r = insp_res.result
                if r.quality_pass or not r.rescan_requested:
                    self._publish_state(COMPLETE, detail=f"report: {r.report_path}")
                    return
                if self._attempt > max_rescans:
                    self._publish_state(ERROR, detail=f"quality fail after {max_rescans} rescans — needs a human")
                    return
                self._attempt += 1
                self._publish_state(RESCANNING, detail=f"quality fail — rescan {self._attempt}")
                # re-plan for the next attempt
                plan_goal = PlanPath.Goal()
                plan_goal.part_id = self._part_id
                plan_res, _ = self._await_action(self._plan_cli, plan_goal, timeout=300.0)
                if not (plan_res and plan_res.success):
                    self._publish_state(ERROR, detail="re-plan failed")
                    return
                self._plan_result = plan_res
            self._publish_state(ABORTED, detail="aborted")
        finally:
            self._busy.release()

    def _call_scan(self, client):
        if client.wait_for_service(timeout_sec=2.0):
            self._await(client.call_async(Trigger.Request()), timeout=10.0)

    # ---- /mission/abort ----
    def _on_abort(self, request, response):
        self._abort.set()
        # best-effort: stop capture (arm stop is ArmDriver's own /arm/stop service)
        self._call_scan(self._scan_stop)
        self._publish_state(ABORTED, detail="abort requested")
        response.success, response.message = True, "aborting"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
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
