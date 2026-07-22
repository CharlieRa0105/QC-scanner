"""Headless proof that MoveIt now collision-checks the SR5 against the GANTRY
(the overhead mounting plate + posts modeled in qc_cell.urdf.xacro) -- i.e. the
"J2 into the rail" collision we can now plan around.

Needs a running move_group on the qc_cell model:
    ros2 launch qc_moveit_config qc_move_group.launch.py

Then:
    python3 gantry_collision_proof.py

Asserts (exit 0 = pass):
  1. the zero pose (arm hanging straight down) is collision-FREE -- proves the
     SRDF gantry exceptions are right (plate<->base overlap is NOT a false hit);
  2. at least one J2 value drives an arm link INTO a gantry_* link (valid=False
     with a gantry contact) -- proves MoveIt sees the rail.
"""
import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def rs(positions):
    s = RobotState()
    s.joint_state = JointState()
    s.joint_state.name = JOINTS
    s.joint_state.position = [float(x) for x in positions]
    return s


class Proof(Node):
    def __init__(self):
        super().__init__("gantry_collision_proof")
        self.cli = self.create_client(GetStateValidity, "/check_state_validity")
        if not self.cli.wait_for_service(timeout_sec=30.0):
            print("FAIL: /check_state_validity never came up (move_group not running?)")
            sys.exit(2)

    def check(self, positions):
        req = GetStateValidity.Request()
        req.group_name = "rokae_arm"
        req.robot_state = rs(positions)
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        r = fut.result()
        contacts = [f"{c.contact_body_1}<->{c.contact_body_2}" for c in (r.contacts if r else [])]
        gantry = [c for c in contacts if "gantry" in c]
        return (r.valid if r else None), contacts, gantry


def main():
    rclpy.init()
    p = Proof()
    ok = True

    # 1) zero pose: arm hangs straight down, clear of the overhead gantry.
    v, contacts, gantry = p.check([0, 0, 0, 0, 0, 0])
    print(f"[1] zero pose            -> valid={v}  (expect True)  contacts={contacts[:4]}")
    ok &= (v is True)

    # 2) sweep J2 (fold the shoulder up toward the overhead plate) until a gantry hit.
    hit = None
    for j2 in [x / 10.0 for x in range(0, 30)]:          # 0 .. 2.9 rad
        v, contacts, gantry = p.check([0, j2, 0, 0, 0, 0])
        if gantry:
            hit = (round(j2, 2), v, gantry[:3])
            break
    if hit:
        print(f"[2] J2={hit[0]} rad          -> valid={hit[1]}  GANTRY CONTACT {hit[2]}  (expect collision)")
        ok &= (hit[1] is False)
    else:
        print("[2] swept J2 0..2.9 rad   -> NO gantry contact found (unexpected)")
        ok = False

    print("RESULT:", "PASS" if ok else "FAIL")
    p.destroy_node(); rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
