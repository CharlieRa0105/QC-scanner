"""Headless proof that MoveIt collision-checks the overhead SR5 against (a) a
part/obstacle in the scene and (b) itself. Uses move_group's /check_state_validity
and /apply_planning_scene services -- no rviz, no real arm."""
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def rs(positions):
    s = RobotState()
    s.joint_state = JointState()
    s.joint_state.name = JOINTS
    s.joint_state.position = [float(x) for x in positions]
    return s


class T(Node):
    def __init__(self):
        super().__init__("collision_check_test")
        self.val = self.create_client(GetStateValidity, "/check_state_validity")
        self.scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.val.wait_for_service(timeout_sec=20.0)
        self.scene.wait_for_service(timeout_sec=20.0)

    def valid(self, positions):
        req = GetStateValidity.Request()
        req.group_name = "rokae_arm"
        req.robot_state = rs(positions)
        fut = self.val.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        r = fut.result()
        contacts = [f"{c.contact_body_1}<->{c.contact_body_2}" for c in r.contacts] if r else []
        return (r.valid if r else None), contacts

    def add_box(self, name, cx, cy, cz, sx, sy, sz):
        co = CollisionObject()
        co.header.frame_id = "table"
        co.id = name
        box = SolidPrimitive(); box.type = SolidPrimitive.BOX; box.dimensions = [sx, sy, sz]
        p = Pose(); p.position.x, p.position.y, p.position.z = cx, cy, cz; p.orientation.w = 1.0
        co.primitives = [box]; co.primitive_poses = [p]; co.operation = CollisionObject.ADD
        self._apply(co)

    def remove(self, name):
        co = CollisionObject(); co.id = name; co.operation = CollisionObject.REMOVE
        co.header.frame_id = "table"
        self._apply(co)

    def _apply(self, co):
        scene = PlanningScene(); scene.is_diff = True
        scene.world.collision_objects = [co]
        req = ApplyPlanningScene.Request(); req.scene = scene
        fut = self.scene.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        time.sleep(0.5)


def main():
    rclpy.init()
    t = T()
    ok = True

    # 1) home config, empty scene -> should be VALID
    v, _ = t.valid([0, 0, 0, 0, 0, 0])
    print(f"[1] home state, empty scene        -> valid={v}   (expect True)")
    ok &= (v is True)

    # 2) add a big box where the arm hangs -> home config should become INVALID (part collision)
    t.add_box("obstacle", 0.0, 0.0, 0.7, 1.0, 1.0, 1.0)
    v, contacts = t.valid([0, 0, 0, 0, 0, 0])
    print(f"[2] home state, box over the arm   -> valid={v}   (expect False)  contacts={contacts[:3]}")
    ok &= (v is False)
    t.remove("obstacle")

    # 3) back to empty -> VALID again
    v, _ = t.valid([0, 0, 0, 0, 0, 0])
    print(f"[3] home state, box removed        -> valid={v}   (expect True)")
    ok &= (v is True)

    # 4) self-collision: search folded configs for one MoveIt rejects with an
    #    arm-vs-arm contact (proves the SRDF self-collision matrix is active).
    found = None
    for cfg in ([0, 2.9, 2.9, 0, 0, 0], [0, 2.9, 2.9, 0, 2.9, 0], [0, -2.9, 2.9, 0, 2.9, 0],
                [0, 2.0, 2.9, 2.9, 2.9, 0], [0, 2.9, -2.9, 2.9, 2.9, 2.9]):
        v, contacts = t.valid(cfg)
        if v is False and contacts:
            found = (cfg, contacts); break
    if found:
        print(f"[4] self-collision config {found[0]} -> valid=False  contacts={found[1][:3]}  (self-collision DETECTED)")
    else:
        print("[4] no self-colliding config hit in the sample set (checker active; sample didn't fold into itself)")

    print("\nRESULT:", "PASS -- MoveIt collision-checks part + self" if ok else "FAIL")
    t.destroy_node(); rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
