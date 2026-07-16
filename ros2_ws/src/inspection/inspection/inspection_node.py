"""
inspection_node.py

InspectionNode (Phase 2; architecture.md §4.2): a thin ROS wrapper over the
pure-Python inspection library (libs/inspection).

Serves the /inspect action (qc_msgs/Inspect): given a captured cloud + part_id it
runs clean -> quality gate -> register -> deviation and returns a
qc_msgs/InspectionResult (quality pass/fail + deviation stats). The quality gate
is the ONE automatic pass/fail (scan quality only, decision 4); the part verdict
is a human call.

RUNTIME NOTE (build-to-interface state): the pipeline stages aren't implemented
yet (they need Open3D + TEASER++ + real scans -- see docs/point_cloud_processing.md),
so a real run returns an honest "not computed" result: quality_pass False,
rescan_requested False, NaN deviation metrics, and a message saying inspection is
not implemented. It never fabricates a pass or a number. The lib is imported
lazily from QC_REPO_ROOT (default /repo) so the node starts + registers /inspect
regardless.
"""

import math
import os
import sys

import rclpy
from qc_msgs.action import Inspect
from qc_msgs.msg import InspectionResult
from rclpy.action import ActionServer
from rclpy.node import Node


class InspectionNode(Node):
    def __init__(self):
        super().__init__("inspection")
        self._server = ActionServer(self, Inspect, "/inspect", self._execute)
        self.get_logger().info("InspectionNode ready — action /inspect")

    def _execute(self, goal_handle):
        req = goal_handle.request
        result = Inspect.Result()
        res = InspectionResult()

        def fb(stage, frac):
            msg = Inspect.Feedback()
            msg.stage, msg.progress = stage, float(frac)
            goal_handle.publish_feedback(msg)

        try:
            repo = os.environ.get("QC_REPO_ROOT", "/repo")
            if repo not in sys.path:
                sys.path.insert(0, repo)
            from libs.inspection.pipeline import inspect as run_inspect

            try:
                from libs.qc_config import load_config
                cfg = load_config()
            except Exception:  # noqa: BLE001
                cfg = {}

            outcome = run_inspect(req.cloud_path, req.part_id, config=cfg, progress=fb)

            res.quality_pass = bool(outcome.quality_pass)
            res.rescan_requested = bool(outcome.rescan_requested)
            res.attempt = 0
            res.report_path = outcome.report_path or ""
            res.mean_dev_mm = float(outcome.mean_dev_mm) if outcome.mean_dev_mm is not None else math.nan
            res.rmse_mm = float(outcome.rmse_mm) if outcome.rmse_mm is not None else math.nan
            res.coverage_pct = float(outcome.coverage_pct) if outcome.coverage_pct is not None else math.nan
            result.result = res
            self.get_logger().info(outcome.message)
            goal_handle.succeed()
        except Exception as e:  # noqa: BLE001
            res.quality_pass = False
            res.rescan_requested = False
            res.mean_dev_mm = res.rmse_mm = res.coverage_pct = math.nan
            result.result = res
            self.get_logger().error(f"inspect failed: {e}")
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = InspectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
