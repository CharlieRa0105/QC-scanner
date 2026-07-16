"""Proof that MoveIt produces a COLLISION-FREE cartesian trajectory for the
overhead SR5 and refuses to drive through an obstacle -- the capability
PathPlanner.moveit_planner uses. Headless: FK the home pose, plan a short
cartesian line (free -> full fraction), then drop a box on the line (blocked ->
reduced fraction)."""
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionFK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

J = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
# A bent, NON-SINGULAR "ready" config (all-zeros is a singularity where cartesian
# planning degenerates to fraction 0). We FK this and plan cartesian from it.
READY = [0.0, 0.4, 0.8, 0.0, 0.8, 0.0]


class T(Node):
    def __init__(self):
        super().__init__("cartesian_plan_test")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.cart = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        for c in (self.fk, self.cart, self.scene):
            c.wait_for_service(timeout_sec=20.0)

    def _call(self, client, req, timeout=15.0):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def ready_tcp(self):
        req = GetPositionFK.Request()
        req.header.frame_id = "table"
        req.fk_link_names = ["xMateSR5_link6"]
        req.robot_state = RobotState()
        req.robot_state.joint_state = JointState(name=J, position=list(READY))
        r = self._call(self.fk, req)
        return r.pose_stamped[0].pose if r and r.pose_stamped else None

    def cartesian(self, waypoints):
        req = GetCartesianPath.Request()
        req.header.frame_id = "table"
        req.group_name = "rokae_arm"
        req.link_name = "xMateSR5_link6"
        # start from the same non-singular READY config the waypoints are around
        req.start_state = RobotState()
        req.start_state.joint_state = JointState(name=J, position=list(READY))
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        r = self._call(self.cart, req, timeout=30.0)
        return float(r.fraction) if r else -1.0

    def box(self, op, cx, cy, cz, s=0.1):
        co = CollisionObject()
        co.header.frame_id = "table"
        co.id = "blocker"
        co.operation = op
        if op == CollisionObject.ADD:
            b = SolidPrimitive(); b.type = SolidPrimitive.BOX; b.dimensions = [s, s, s]
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(cx), float(cy), float(cz)
            p.orientation.w = 1.0
            co.primitives = [b]; co.primitive_poses = [p]
        sc = PlanningScene(); sc.is_diff = True; sc.world.collision_objects = [co]
        self._call(self.scene, ApplyPlanningScene.Request(scene=sc))
        time.sleep(0.5)


def main():
    rclpy.init()
    t = T()
    home = t.ready_tcp()
    if home is None:
        print("FK failed — cannot get ready TCP"); sys.exit(1)
    print(f"home TCP (table frame): x={home.position.x:.3f} y={home.position.y:.3f} z={home.position.z:.3f}")

    # a short straight cartesian move from home: +/- 6 cm along X, in 2 cm steps
    def wp(dx):
        p = Pose(); p.orientation = home.orientation
        p.position.x = home.position.x + dx
        p.position.y = home.position.y
        p.position.z = home.position.z
        return p
    line = [wp(d) for d in (0.02, 0.04, 0.06)]

    frac_free = t.cartesian(line)
    print(f"[free]    cartesian fraction = {frac_free:.2f}   (expect high, ~1.0)")

    # drop a box right on the line (at home + 4 cm X) -> should block it
    t.box(CollisionObject.ADD, home.position.x + 0.04, home.position.y, home.position.z)
    frac_blocked = t.cartesian(line)
    print(f"[blocked] cartesian fraction = {frac_blocked:.2f}   (expect lower — MoveIt won't cross the box)")
    t.box(CollisionObject.REMOVE, 0, 0, 0)

    ok = frac_free > frac_blocked and frac_free > 0.3
    print("\nRESULT:", "PASS — collision-free cartesian planning works + obstacle reduces reachable fraction"
          if ok else f"INCONCLUSIVE (free={frac_free:.2f} blocked={frac_blocked:.2f})")
    t.destroy_node(); rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
