"""
qc_move_group.launch.py

Bring up MoveIt's move_group for the overhead-mounted SR5 (planning only -- no
trajectory execution controllers, which is what PathPlanner needs to plan
collision-free trajectories via the compute_cartesian_path service).

It reuses the vendor SR5 MoveIt config (kinematics, OMPL) but overrides the
robot_description with qc_cell.urdf.xacro (arm mounted 1.0 m above the table,
pointing down, + the gantry collision boxes) and the semantic model with our
qc_cell.srdf (vendor self-collision matrix + the gantry collision exceptions).

We publish the model with robot_state_publisher and feed a joint_state_publisher
(zeros) so move_group's PlanningSceneMonitor always has a current robot state --
required for planning to work headless (no real arm / rviz).

    ros2 launch qc_moveit_config qc_move_group.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    cell_xacro = os.path.join(
        get_package_share_directory("qc_moveit_config"), "config", "qc_cell.urdf.xacro"
    )

    # Vendor config for the semantics/kinematics/planners; our xacro for the URDF.
    moveit_config = (
        MoveItConfigsBuilder("xMateSR5", package_name="rokae_xMateSR5_moveit_config")
        .robot_description(file_path=cell_xacro)
        # OUR semantic model (gantry collision exceptions), not the vendor SRDF.
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("qc_moveit_config"), "config", "qc_cell.srdf"))
        # our kinematics.yaml (longer IK timeout than the vendor's 5 ms)
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory("qc_moveit_config"), "config", "kinematics.yaml"))
        .joint_limits(file_path="config/joint_limits.yaml")
        # Pin the controllers file explicitly: the vendor package ships two
        # candidates (simple_moveit_controllers + ros2_controllers), so the
        # auto-guess can't choose. We plan only, but move_group still wants this.
        .trajectory_execution(file_path="config/simple_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            output="screen", parameters=[moveit_config.robot_description],
        ),
        # Zeros for the 6 joints -> a known current state for the scene monitor.
        Node(
            package="joint_state_publisher", executable="joint_state_publisher",
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group", executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"publish_robot_description_semantic": True},
                # planning-only: don't try to fake a controller manager
                {"use_sim_time": False},
            ],
        ),
    ])
