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
from moveit_msgs.msg import (
    CollisionObject, Constraints, JointConstraint, LinkPadding, PlanningScene, RobotTrajectory,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPositionIK
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive

GROUP = "rokae_arm"
TIP_LINK = "xMateSR5_link6"
PLANNING_FRAME = "table"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Optional robot-link padding = a uniform clearance buffer from the part, table,
# gantry AND self. NOTE: default is 0 because the SR5 is bolted ~1 cm below the
# mounting plate, so a large uniform pad makes its own shoulder "collide" with the
# plate it hangs from and folded scan poses self-collide (measured: 5 cm dropped the
# achievable scan from fraction 1.00 to 0.14). True geometry already plans
# collision-free; the scan clears the part by the 250 mm standoff. Set a small pad
# via QC_COLLISION_PADDING_M (e.g. 0.02) if you want extra margin and can accept a
# lower fraction. A buffer for MANUAL JOG is a separate feature (collision-checked
# jog), since jog bypasses MoveIt entirely.
ARM_LINKS = ["xMateSR5_base", "xMateSR5_link1", "xMateSR5_link2", "xMateSR5_link3",
             "xMateSR5_link4", "xMateSR5_link5", "xMateSR5_link6"]
COLLISION_PADDING_M = 0.0
import os as _os
try:
    COLLISION_PADDING_M = float(_os.environ.get("QC_COLLISION_PADDING_M", COLLISION_PADDING_M))
except ValueError:
    pass


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
    # 5 cm buffer: pad every arm link so planned motion keeps clearance from the
    # part/table/gantry and from itself.
    scene.link_padding = [LinkPadding(link_name=L, padding=COLLISION_PADDING_M) for L in ARM_LINKS]
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


def _joints_in_order(robot_state):
    """Extract JOINT_NAMES-ordered positions from a RobotState, or None."""
    if robot_state is None:
        return None
    js = robot_state.joint_state
    m = dict(zip(js.name, js.position))
    if all(n in m for n in JOINT_NAMES):
        return [float(m[n]) for n in JOINT_NAMES]
    return None


def plan_line(node, cart_client, poses, max_step=0.01, timeout=30.0, seed=None):
    """Compute a collision-checked cartesian path through `poses` (Pose[] in the
    table frame). Returns (RobotTrajectory, fraction). fraction < 1.0 means MoveIt
    could only trace that portion collision-free.

    `seed` (RobotState, optional) sets the path start_state so the trace begins AT
    the line's first waypoint (not the robot's current straight-down state)."""
    if not cart_client.wait_for_service(timeout_sec=5.0):
        return RobotTrajectory(), 0.0
    req = GetCartesianPath.Request()
    req.header.frame_id = PLANNING_FRAME
    req.group_name = GROUP
    req.link_name = TIP_LINK
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


def plan_freespace(node, plan_client, start_joints, goal_joints, timeout=20.0):
    """OMPL joint-space plan from start_joints to goal_joints, collision-free
    (self + scene). Bridges the gap BETWEEN scan lines so the whole trajectory is
    continuously executable. Returns a RobotTrajectory (empty if it fails/absent)."""
    if plan_client is None or not plan_client.wait_for_service(timeout_sec=5.0):
        return RobotTrajectory()
    req = GetMotionPlan.Request()
    mpr = req.motion_plan_request
    mpr.group_name = GROUP
    mpr.num_planning_attempts = 5
    mpr.allowed_planning_time = 5.0
    mpr.start_state.joint_state.name = list(JOINT_NAMES)
    mpr.start_state.joint_state.position = [float(v) for v in start_joints]
    c = Constraints()
    for n, v in zip(JOINT_NAMES, goal_joints):
        jc = JointConstraint()
        jc.joint_name = n
        jc.position = float(v)
        jc.tolerance_above = jc.tolerance_below = 0.01
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    mpr.goal_constraints = [c]
    fut = plan_client.call_async(req)
    _spin(node, fut, timeout)
    r = fut.result()
    if r is None or r.motion_plan_response.error_code.val != 1:   # 1 = SUCCESS
        return RobotTrajectory()
    return r.motion_plan_response.trajectory


def plan_scanpath(node, cart_client, apply_client, scanpath, part_points_m, part_triangles, config,
                  ik_client=None, plan_client=None):
    """Full MoveIt plan for a scan path: load table+part into the scene, then plan
    each raster line cartesian, with collision-free OMPL bridges BETWEEN lines.
    Returns (RobotTrajectory, min_fraction, message).
      * `ik_client`   — seeds each line's cartesian start_state (IK of waypoint 0).
      * `plan_client` — /plan_kinematic_path, for the free-space between-line moves.
    """
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
    bridges = 0
    prev_end = None   # JOINT_NAMES-ordered joints at the end of the last line
    for line_id in sorted(lines):
        poses = lines[line_id]
        seed = compute_ik(node, ik_client, poses[0]) if (ik_client and poses) else None
        start_joints = _joints_in_order(seed)
        # free-space bridge: move from where the last line ended to this line's start
        if prev_end is not None and start_joints is not None:
            bridge = plan_freespace(node, plan_client, prev_end, start_joints)
            if bridge.joint_trajectory.points:
                _append_trajectory(combined, bridge)
                bridges += 1
        traj, frac = plan_line(node, cart_client, poses, seed=seed)
        fractions.append(frac)
        _append_trajectory(combined, traj)
        if traj.joint_trajectory.points:
            prev_end = list(traj.joint_trajectory.points[-1].positions)
    min_frac = min(fractions) if fractions else 0.0
    return combined, min_frac, (
        f"{bridges} free-space bridge(s); " +
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
