"""
moveit_planner.py

The MoveIt half of PathPlanner (architecture.md §4.2): given a scan path in the
cell's `table` frame, drive move_group to produce a COLLISION-FREE joint
trajectory. Pure rclpy + moveit_msgs service clients -- no moveit_py needed.

It does two things:
  1. Loads the scene: a table box + the part CAD mesh as MoveIt collision
     objects (architecture decision 12) via /apply_planning_scene.
  2. Plans motion: for each raster line, calls /compute_cartesian_path (with
     avoid_collisions=True) to get a straight-line-in-cartesian trajectory that
     MoveIt has checked against the arm itself (SRDF) AND the scene (table +
     part). Free-space moves BETWEEN lines are left to a future MoveGroup-action
     step -- marked TODO.

Because avoid_collisions=True, the returned fraction drops below 1.0 exactly
where a segment would collide -- that is MoveIt refusing to drive the arm through
itself or the part.

Planning frame is the model root `table`; the arm group is `rokae_arm`, tip
`xMateSR5_link6`.
"""

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive

GROUP = "rokae_arm"
TIP_LINK = "xMateSR5_link6"
PLANNING_FRAME = "table"


def make_table_object(config):
    """A box for the table surface, top at table z=0 (architecture decision 12).
    Size from config.frames or a sane default (1.5 x 0.7 m, 0.1 m thick)."""
    cell = (config or {}).get("cell", {})
    length = float(cell.get("table_length_m", 1.5))
    width = float(cell.get("table_width_m", 0.7))
    thick = float(cell.get("table_thickness_m", 0.1))
    co = CollisionObject()
    co.header.frame_id = PLANNING_FRAME
    co.id = "table"
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [length, width, thick]
    pose = Pose()
    pose.position.z = -thick / 2.0   # top face at z = 0
    pose.orientation.w = 1.0
    co.primitives = [box]
    co.primitive_poses = [pose]
    co.operation = CollisionObject.ADD
    return co


def make_part_object(points_m, triangles):
    """The part CAD as a MoveIt mesh collision object, in the `table` frame.
    `points_m` are (N,3) vertices already transformed into the table frame
    (metres); `triangles` are (M,3) vertex-index triples."""
    co = CollisionObject()
    co.header.frame_id = PLANNING_FRAME
    co.id = "part"
    mesh = Mesh()
    for tri in triangles:
        mt = MeshTriangle()
        mt.vertex_indices = [int(tri[0]), int(tri[1]), int(tri[2])]
        mesh.triangles.append(mt)
    for p in points_m:
        pt = Pose().position.__class__()   # geometry_msgs/Point
        pt.x, pt.y, pt.z = float(p[0]), float(p[1]), float(p[2])
        mesh.vertices.append(pt)
    identity = Pose()
    identity.orientation.w = 1.0
    co.meshes = [mesh]
    co.mesh_poses = [identity]
    co.operation = CollisionObject.ADD
    return co


def apply_scene(node, apply_client, objects, timeout=10.0):
    """Add/replace collision objects in the planning scene (diff)."""
    if not apply_client.wait_for_service(timeout_sec=5.0):
        return False, "planning-scene service unavailable"
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = list(objects)
    fut = apply_client.call_async(ApplyPlanningScene.Request(scene=scene))
    _spin(node, fut, timeout)
    r = fut.result()
    return (bool(r and r.success), "scene applied" if r and r.success else "scene apply failed")


def compute_ik(node, ik_client, pose, timeout=5.0):
    """IK for a single Pose (table frame) -> a RobotState at that pose, or None.
    Used to SEED a cartesian path's start_state so the trace begins at the line's
    first waypoint (otherwise compute_cartesian_path starts from the robot's
    current state -- arm straight down -- and the first straight-line segment to
    the scan pose is infeasible, giving fraction 0)."""
    if ik_client is None or not ik_client.wait_for_service(timeout_sec=2.0):
        return None
    req = GetPositionIK.Request()
    r = req.ik_request
    r.group_name = GROUP
    r.pose_stamped = PoseStamped()
    r.pose_stamped.header.frame_id = PLANNING_FRAME
    r.pose_stamped.pose = pose
    r.timeout.sec = 1
    r.avoid_collisions = True
    fut = ik_client.call_async(req)
    _spin(node, fut, timeout)
    res = fut.result()
    if res is not None and res.error_code.val == 1:   # 1 = SUCCESS
        return res.solution
    return None


def plan_line(node, cart_client, poses, max_step=0.01, timeout=30.0, ik_client=None):
    """Compute a collision-checked cartesian path through `poses` (Pose[] in the
    table frame). Returns (RobotTrajectory, fraction). fraction < 1.0 means MoveIt
    could only trace that portion collision-free.

    If `ik_client` is given, the path's start_state is seeded with the IK of
    poses[0] so the trace begins AT the line's first waypoint (not the robot's
    current straight-down state)."""
    if not cart_client.wait_for_service(timeout_sec=5.0):
        return RobotTrajectory(), 0.0
    req = GetCartesianPath.Request()
    req.header.frame_id = PLANNING_FRAME
    req.group_name = GROUP
    req.link_name = TIP_LINK
    seed = compute_ik(node, ik_client, poses[0]) if poses else None
    if seed is not None:
        req.start_state = seed
    req.waypoints = list(poses)
    req.max_step = max_step
    req.jump_threshold = 0.0
    req.avoid_collisions = True          # <-- MoveIt refuses self/scene collisions
    fut = cart_client.call_async(req)
    _spin(node, fut, timeout)
    r = fut.result()
    if r is None:
        return RobotTrajectory(), 0.0
    return r.solution, float(r.fraction)


def plan_scanpath(node, cart_client, apply_client, scanpath, part_points_m, part_triangles, config,
                  ik_client=None):
    """Full MoveIt plan for a scan path: load table+part into the scene, then plan
    each raster line cartesian. Returns (RobotTrajectory, min_fraction, message).
    `ik_client` (optional) seeds each line's cartesian start_state with the IK of
    its first waypoint."""
    ok, msg = apply_scene(
        node, apply_client, [make_table_object(config), make_part_object(part_points_m, part_triangles)]
    )
    if not ok:
        return RobotTrajectory(), 0.0, msg

    # group waypoints by raster line, preserving order
    lines = {}
    for wp in scanpath.waypoints:
        lines.setdefault(wp.line_id, []).append(wp.pose)

    combined = RobotTrajectory()
    fractions = []
    for line_id in sorted(lines):
        traj, frac = plan_line(node, cart_client, lines[line_id], ik_client=ik_client)
        fractions.append(frac)
        _append_trajectory(combined, traj)
    min_frac = min(fractions) if fractions else 0.0
    return combined, min_frac, (
        f"planned {len(lines)} lines; min cartesian fraction {min_frac:.2f} "
        "(collision-checked vs self + table + part)"
    )


def _append_trajectory(dst, src):
    """Concatenate src's joint trajectory points onto dst (keeps joint_names)."""
    sjt = src.joint_trajectory
    if not sjt.points:
        return
    if not dst.joint_trajectory.joint_names:
        dst.joint_trajectory.joint_names = list(sjt.joint_names)
    dst.joint_trajectory.points.extend(sjt.points)


def _spin(node, future, timeout):
    # Poll the future instead of spin_until_future_complete: this runs INSIDE the
    # PathPlanner action callback, so the executor is already spinning elsewhere
    # (MultiThreadedExecutor + reentrant group). Another executor thread completes
    # the service future while we wait here.
    import time
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        time.sleep(0.02)
