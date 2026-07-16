"""
path_planner_node.py

PathPlanner (architecture.md §3, §4.2): owns the WHOLE plan for a part.

Serves the /plan_path action (qc_msgs/PlanPath). On a goal it:
  1. loads the part CAD from config/cad/<part_id>,
  2. runs the pure-Python coverage planner (libs/path_planning) -> waypoints,
  3. applies the marked-corner frame transform -> arm base frame,
  4. builds a qc_msgs/ScanPath and publishes it (latched) on /plan/scanpath,
  5. runs MoveIt to produce a collision-free trajectory on /plan/trajectory,
and returns trajectory + scanpath in the result. Plan-fully-then-execute
(decision 13): all planning here, execution is MovementDriver's job.

RUNTIME NOTES (build-to-interface state):
  * The coverage planner needs gmsh + numpy + PyYAML and the repo's libs/ +
    config/ trees. Set QC_REPO_ROOT to the mounted repo (default /repo). These
    are imported LAZILY (inside the goal callback), so the node starts and
    registers /plan_path even where they're absent -- only actual planning
    needs them.
  * MoveIt planning needs the SR5 moveit_config (URDF etc.), not wired yet. Until
    it is, the node DEGRADES GRACEFULLY: it still computes + publishes the
    ScanPath, returns an empty trajectory, and says MoveIt is pending. Marked
    TODO, never faked.
"""

import os
import sys

import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from qc_msgs.action import PlanPath
from qc_msgs.msg import ScanPath, ScanWaypoint
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from trajectory_msgs.msg import JointTrajectory

# Latched (transient-local) so a late web subscriber still gets the last plan.
_LATCHED = QoSProfile(depth=1)
_LATCHED.durability = DurabilityPolicy.TRANSIENT_LOCAL


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner")
        self._cbg = ReentrantCallbackGroup()
        self._scanpath_pub = self.create_publisher(ScanPath, "/plan/scanpath", _LATCHED)
        self._traj_pub = self.create_publisher(JointTrajectory, "/plan/trajectory", _LATCHED)
        # move_group clients (PathPlanner is a client of MoveIt, per §4.2).
        self._cart_cli = self.create_client(
            GetCartesianPath, "/compute_cartesian_path", callback_group=self._cbg)
        self._scene_cli = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene", callback_group=self._cbg)
        self._server = ActionServer(
            self, PlanPath, "/plan_path", self._execute, callback_group=self._cbg)
        self.get_logger().info(
            "PathPlanner ready — action /plan_path, topics /plan/scanpath + /plan/trajectory"
        )

    def _repo_root(self):
        """Make the repo's libs/ + config/ importable (mounted at QC_REPO_ROOT)."""
        repo = os.environ.get("QC_REPO_ROOT", "/repo")
        if repo not in sys.path:
            sys.path.insert(0, repo)
        return repo

    def _find_cad(self, repo, part_id):
        cad_dir = os.path.join(repo, "config", "cad")
        for fn in sorted(os.listdir(cad_dir)):
            stem, ext = os.path.splitext(fn)
            if stem == part_id and ext.lower() in (".step", ".stp", ".stl", ".obj"):
                return os.path.join(cad_dir, fn)
        raise FileNotFoundError(f"no CAD for part_id {part_id!r} in {cad_dir}")

    def _execute(self, goal_handle):
        part_id = goal_handle.request.part_id
        self.get_logger().info(f"planning part_id={part_id!r}")
        result = PlanPath.Result()

        def fb(stage, progress):
            msg = PlanPath.Feedback()
            msg.stage, msg.progress = stage, float(progress)
            goal_handle.publish_feedback(msg)

        try:
            repo = self._repo_root()
            fb("loading cad", 0.1)
            # Lazy imports: heavy deps (gmsh/numpy) only needed to actually plan.
            import numpy as np

            from libs.path_planning.cad_loader import load_cad
            from libs.path_planning.frame_transform import FrameTransform, matrix_to_quaternion
            from libs.path_planning.incidence_cone_modifier import apply_incidence_cone_relaxation
            from libs.path_planning.normal_estimation import sample_surface
            from libs.path_planning.waypoint_generator import (
                generate_raster_waypoints,
                raster_spacing_from_fov,
            )
            from libs.qc_config import load_config

            cfg = load_config()
            p = cfg.get("planner", {})
            cad_path = self._find_cad(repo, part_id)

            fb("coverage planning", 0.35)
            verts, faces = load_cad(cad_path, mesh_size=p.get("mesh_size_mm", 5.0))
            pts, normals = sample_surface(verts, faces, n_samples=p.get("samples", 20000))
            spacing = raster_spacing_from_fov(
                p.get("standoff_mm", 250), p.get("fov_deg", 40), p.get("overlap", 0.3)
            )
            wps = generate_raster_waypoints(
                pts, normals,
                standoff_mm=p.get("standoff_mm", 250),
                raster_spacing_mm=spacing,
                along_track_mm=p.get("along_track_mm", 10),
                face_angle_tol_deg=p.get("face_angle_tol_deg", 30),
            )
            relaxed = apply_incidence_cone_relaxation(
                wps, max_incidence_angle_deg=p.get("max_incidence_deg", 25)
            )

            fb("frame transform", 0.6)
            ft = FrameTransform.from_config(cfg)
            standoff_mm = float(p.get("standoff_mm", 250))
            scanpath = ScanPath()
            scanpath.part_id = part_id
            scanpath.standoff_mm = standoff_mm
            for wp, r in zip(wps, relaxed):
                pos = ft.apply_point(r["position"])
                m = np.column_stack([r["x_axis"], r["y_axis"], r["z_axis"]])
                q = matrix_to_quaternion(ft.rotation @ m)
                tgt = ft.apply_point(wp.position - wp.normal * standoff_mm)
                sw = ScanWaypoint()
                sw.pose = Pose()
                sw.pose.position.x, sw.pose.position.y, sw.pose.position.z = (float(v) for v in pos)
                (sw.pose.orientation.x, sw.pose.orientation.y,
                 sw.pose.orientation.z, sw.pose.orientation.w) = (float(v) for v in q)
                sw.target = Point()
                sw.target.x, sw.target.y, sw.target.z = (float(v) for v in tgt)
                sw.incidence_deg = float(r["incidence_angle_deg"])
                sw.line_id = int(wp.line_id)
                scanpath.waypoints.append(sw)
            self._scanpath_pub.publish(scanpath)

            fb("moveit", 0.8)
            # MoveIt: load the table + part as collision objects and plan each
            # raster line cartesian (avoid_collisions=True). Needs move_group up
            # (qc_moveit_config); if it isn't, this degrades to an empty
            # trajectory + a clear message, but the ScanPath is still returned.
            from path_planner import moveit_planner as mp

            part_points_m = [ft.apply_point(v) for v in verts]  # part->table frame (m)
            trajectory, fraction, mv_msg = mp.plan_scanpath(
                self, self._cart_cli, self._scene_cli,
                scanpath, part_points_m, faces, cfg,
            )
            self._traj_pub.publish(trajectory.joint_trajectory)

            result.success = True
            result.scanpath = scanpath
            result.trajectory = trajectory
            result.message = f"planned {len(scanpath.waypoints)} waypoints; {mv_msg}"
            fb("done", 1.0)
            goal_handle.succeed()
        except Exception as e:  # noqa: BLE001
            result.success = False
            result.message = f"plan failed: {e}"
            self.get_logger().error(result.message)
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    # MultiThreadedExecutor + reentrant group: the /plan_path callback blocks on
    # MoveIt service futures, which another executor thread must complete.
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
