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

import json
import os
import sys

import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPositionIK
from qc_msgs.action import PlanPath
from qc_msgs.msg import ScanPath, ScanWaypoint
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

# Latched (transient-local) so a late web subscriber still gets the last plan.
_LATCHED = QoSProfile(depth=1)
_LATCHED.durability = DurabilityPolicy.TRANSIENT_LOCAL


def _lookat_R(pos, tgt):
    """Rotation whose +Z (the tool approach axis / flange +Z) points from `pos` to
    `tgt`, with a STABLE up-reference so consecutive scan poses share one smooth
    posture. Replaces per-face surface-normal alignment, whose orientation flipped
    between faces and made the IK jump between arm branches (base swinging ±320°).
    Columns are the tool x,y,z axes expressed in the parent (table) frame."""
    import numpy as np
    z = np.asarray(tgt, float) - np.asarray(pos, float)
    nz = np.linalg.norm(z)
    if nz < 1e-9:
        return np.eye(3)
    z = z / nz
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(z, up))) > 0.95:    # aim ~vertical -> use world +X as up-ref
        up = np.array([1.0, 0.0, 0.0])
    x = np.cross(up, z); x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner")
        self._cbg = ReentrantCallbackGroup()
        self._scanpath_pub = self.create_publisher(ScanPath, "/plan/scanpath", _LATCHED)
        self._traj_pub = self.create_publisher(JointTrajectory, "/plan/trajectory", _LATCHED)
        # Diagnostic: per-waypoint pose<->KDL-IK-joints pairing, as a JSON string so
        # the console can read it over rosbridge (no custom msg, no qc_msgs rebuild)
        # and print it to its Log stream. Latched so a late web subscriber gets the
        # last plan's IK.
        self._ik_pub = self.create_publisher(String, "/plan/ik", _LATCHED)
        # move_group clients (PathPlanner is a client of MoveIt, per §4.2).
        self._cart_cli = self.create_client(
            GetCartesianPath, "/compute_cartesian_path", callback_group=self._cbg)
        self._scene_cli = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene", callback_group=self._cbg)
        # IK client to seed each scan line's cartesian start_state (so the trace
        # starts at the line's first waypoint, not the arm's current straight pose).
        self._ik_cli = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self._cbg)
        # Free-space (OMPL) planning for the moves BETWEEN scan lines.
        self._plan_cli = self.create_client(
            GetMotionPlan, "/plan_kinematic_path", callback_group=self._cbg)
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
        # Same CAD source as the console/host backend: QC_PARTS_DIR if set (the
        # operator's part library, e.g. ~/Desktop/Parts mounted into the container),
        # else the repo's config/cad. WITHOUT this the planner looked only in
        # config/cad and missed parts the console lists -> "no CAD for part_id".
        cad_dir = os.environ.get("QC_PARTS_DIR") or os.path.join(repo, "config", "cad")
        for fn in sorted(os.listdir(cad_dir)):
            stem, ext = os.path.splitext(fn)
            if stem == part_id and ext.lower() in (".step", ".stp", ".stl", ".obj"):
                return os.path.join(cad_dir, fn)
        raise FileNotFoundError(f"no CAD for part_id {part_id!r} in {cad_dir}")

    def _reachable_shift(self, part_points_m, planner_cfg):
        """Translation (dx,dy,dz) to move the part footprint centre onto a reachable
        anchor (off the base's singular column) and raise its lowest point to a
        pedestal height in the arm's reach sweet-spot. Config (planner section):
        `scan_anchor_m` = [x,y] (default [0.30, 0.0]); `pedestal_z_m` (default 0.40).
        Placeholder until the marked-corner transform + fixture height are measured."""
        anchor = (planner_cfg.get("scan_anchor_m") or [0.30, 0.0])
        ax, ay = float(anchor[0]), float(anchor[1])
        pedestal = float(planner_cfg.get("pedestal_z_m", 0.40))
        xs = [v[0] for v in part_points_m]
        ys = [v[1] for v in part_points_m]
        zs = [v[2] for v in part_points_m]
        cx = (min(xs) + max(xs)) / 2.0
        cy = (min(ys) + max(ys)) / 2.0
        return (ax - cx, ay - cy, pedestal - min(zs))

    def _topdown_waypoints(self, part_points_m, p):
        """Overhead raster: a boustrophedon grid of DOWN-aiming poses on a plane
        `standoff` above the part's top, covering its X/Y footprint. Every pose aims
        straight down, so the arm holds ONE stable posture (no branch flips) — unlike
        surface-normal coverage, which made an overhead arm reconfigure through
        singularities between faces (±300° joint swings). Validated offline: joint
        spans <15°, max step ~10°. Returns [(pos, target, line_id)] in the (already
        placed) table frame. `planner.scan_mode='surface'` restores full coverage."""
        import numpy as np
        pts = np.asarray(part_points_m, float)
        lo = pts.min(axis=0); hi = pts.max(axis=0)
        standoff = float(p.get("standoff_mm", 250)) / 1000.0
        try:
            from libs.path_planning.waypoint_generator import raster_spacing_from_fov
            pitch = raster_spacing_from_fov(
                p.get("standoff_mm", 250), p.get("fov_deg", 40), p.get("overlap", 0.3)) / 1000.0
        except Exception:  # noqa: BLE001
            pitch = float(p.get("raster_pitch_mm", 50)) / 1000.0
        pitch = max(0.01, float(pitch))
        top = float(hi[2]); z = top + standoff

        def grid(a, b):
            out = []; v = float(a)
            while v <= float(b) + 1e-9:
                out.append(v); v += pitch
            if not out or out[-1] < float(b) - 1e-9:
                out.append(float(b))
            return out

        gx = grid(lo[0], hi[0]); gy = grid(lo[1], hi[1])
        wps = []; line = 0
        for i, x in enumerate(gx):
            row = gy if i % 2 == 0 else list(reversed(gy))
            for y in row:
                wps.append(([x, y, z], [x, y, top], line))
            line += 1
        return wps

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
            from libs.qc_config import load_config

            cfg = load_config()
            p = cfg.get("planner", {})
            cad_path = self._find_cad(repo, part_id)

            fb("coverage planning", 0.35)
            verts, faces = load_cad(cad_path, mesh_size=p.get("mesh_size_mm", 5.0))

            fb("frame transform", 0.6)
            ft = FrameTransform.from_config(cfg)
            standoff_mm = float(p.get("standoff_mm", 250))
            # Part -> table frame, then REACHABLE PLACEMENT: shift so the footprint
            # centre sits off the base's singular column and the part is raised into
            # the reach sweet-spot (placeholder for the measured corner transform +
            # fixture height — decision 5 / session-log 2026-07-22 §2).
            part_points_m = [ft.apply_point(v) for v in verts]
            dx, dy, dz = self._reachable_shift(part_points_m, p)
            part_points_m = [[v[0] + dx, v[1] + dy, v[2] + dz] for v in part_points_m]

            # TOP-DOWN scan poses over the placed part (see _topdown_waypoints). Every
            # pose aims straight down -> ONE stable arm posture -> continuous joints
            # (surface-normal coverage forced ±300° branch flips on this overhead arm).
            scanpath = ScanPath()
            scanpath.part_id = part_id
            scanpath.standoff_mm = standoff_mm
            for pos, tgt, line_id in self._topdown_waypoints(part_points_m, p):
                q = matrix_to_quaternion(_lookat_R(pos, tgt))
                sw = ScanWaypoint()
                sw.pose = Pose()
                sw.pose.position.x, sw.pose.position.y, sw.pose.position.z = (float(v) for v in pos)
                (sw.pose.orientation.x, sw.pose.orientation.y,
                 sw.pose.orientation.z, sw.pose.orientation.w) = (float(v) for v in q)
                sw.target = Point()
                sw.target.x, sw.target.y, sw.target.z = (float(v) for v in tgt)
                sw.incidence_deg = 0.0
                sw.line_id = int(line_id)
                scanpath.waypoints.append(sw)
            self._scanpath_pub.publish(scanpath)

            fb("moveit", 0.8)
            # MoveIt: load the table + part as collision objects and plan each
            # raster line cartesian (avoid_collisions=True). Needs move_group up
            # (qc_moveit_config); if it isn't, this degrades to an empty
            # trajectory + a clear message, but the ScanPath is still returned.
            from path_planner import moveit_planner as mp

            trajectory, fraction, mv_msg = mp.plan_scanpath(
                self, self._cart_cli, self._scene_cli,
                scanpath, part_points_m, faces, cfg,
                ik_client=self._ik_cli, plan_client=self._plan_cli,
            )
            self._traj_pub.publish(trajectory.joint_trajectory)

            # Per-waypoint IK (pose <-> joints) for the console Log stream. On by
            # default; QC_LOG_WAYPOINT_IK=0 disables it (N IK service calls, so it
            # adds plan time on parts with many waypoints).
            if os.environ.get("QC_LOG_WAYPOINT_IK", "1") != "0":
                ik_report = mp.compute_waypoint_ik(self, self._ik_cli, scanpath)
                self._ik_pub.publish(String(data=json.dumps(ik_report)))
                self.get_logger().info(
                    f"per-waypoint IK: {ik_report['solved']}/{ik_report['count']} "
                    "waypoints solved (published on /plan/ik)"
                )

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
