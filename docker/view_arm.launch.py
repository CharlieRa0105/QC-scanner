#!/usr/bin/env python3
"""
view_arm.launch.py  (runs INSIDE the Humble container)

Display the exact SR5 model in RViz and move it with the real
joint_state_publisher_gui — no rail, no MoveIt. Uses the full Humble toolchain
(xacro + package:// resolution via the colcon-built rokae_description), so no
host-side symlink/URDF-rewrite hacks are needed.

  robot_state_publisher  — SR5 URDF (xacro-processed) -> /robot_description + TF
  joint_state_publisher_gui — real slider window for the 6 joints
  rviz2                  — /view_arm.rviz (RobotModel + TF, fixed frame 'world')

Launched by docker/run_arm.sh after `colcon build --packages-select rokae_description`.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory("rokae_description")
    xacro_file = os.path.join(desc_share, "urdf", "xMateSR5.urdf.xacro")
    # Process the xacro to a URDF string. package://rokae_description/meshes/...
    # resolves because rokae_description is colcon-built + sourced.
    robot_description = subprocess.check_output(["xacro", xacro_file]).decode()

    rviz_config = "/view_arm.rviz"  # bind-mounted in by run_arm.sh

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
