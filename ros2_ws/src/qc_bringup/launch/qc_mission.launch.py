"""
qc_mission.launch.py

Top-level launch for the whole QC Scanner mission graph (architecture.md §8).
Brings up rosbridge + every mission node with one command:

    ros2 launch qc_bringup qc_mission.launch.py

This grows as the nodes land. Today it starts:
  * rosbridge            -- web <-> ROS 2 link (T8)

To add (each its own package + node, wired here as it's built):
  * arm_driver           -- SR5 joint driver (built; T12 renames its topics)
  * path_planner         -- /plan_path (T9)
  * movement_driver      -- /execute_path (T10)
  * task_manager         -- /mission/* (T11)
  * scanner_driver       -- /scan/* (T13, hardware-blocked)
  * inspection           -- /inspect (T14)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("qc_bringup")

    # Arm backend selection (Phase B). DEFAULT 'mock' — the whole mission runs on the
    # simulated arm and NEVER touches hardware. Switch to the real SR5 ONLY for a
    # supervised bring-up, with the physical E-stop in reach:
    #   ros2 launch qc_bringup qc_mission.launch.py arm_backend:=rokae
    # robot_ip / sdk_root are forwarded to the ArmDriver's own parameters.
    arm_backend = LaunchConfiguration("arm_backend")
    robot_ip = LaunchConfiguration("robot_ip")
    sdk_root = LaunchConfiguration("sdk_root")
    launch_args = [
        DeclareLaunchArgument("arm_backend", default_value="mock",
                              description="ArmDriver backend: 'mock' (default) or 'rokae' (real SR5)"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.2.160",
                              description="Real SR5 controller IP (used only when arm_backend:=rokae)"),
        DeclareLaunchArgument("sdk_root", default_value=os.path.expanduser("~/rokae_sdk"),
                              description="xCore SDK root importable in the container (arm_backend:=rokae)"),
    ]

    rosbridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "rosbridge.launch.py"))
    )

    # MoveIt move_group for the overhead-mounted SR5 (PathPlanner plans against it).
    from ament_index_python.packages import get_package_share_directory as _pkg
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_pkg("qc_moveit_config"), "launch", "qc_move_group.launch.py"))
    )

    # Each mission node (architecture.md §3). arm_driver takes its backend from the
    # launch args above (mock by default); ScanningDriver is interface-only until
    # the scanner lands.
    mission_nodes = [
        Node(package="sr5_arm_driver", executable="arm_driver", name="arm_driver", output="screen",
             parameters=[{"backend": arm_backend, "robot_ip": robot_ip, "sdk_root": sdk_root,
                          # /arm/home target — the SAFE, non-singular parked pose (deg), matching
                          # the console's HOME_JOINTS_DEG. WITHOUT this, home_deg defaults to
                          # all-zeros (the singular column) and "Home" would drive the arm there.
                          "home_deg": [-71.38, 93.33, 127.11, 157.33, -61.23, -77.06]}]),
        Node(package="path_planner", executable="path_planner", name="path_planner", output="screen"),
        Node(package="movement_driver", executable="movement_driver", name="movement_driver", output="screen"),
        Node(package="task_manager", executable="task_manager", name="task_manager", output="screen"),
        Node(package="scanner_driver", executable="scanner_driver", name="scanner_driver", output="screen"),
        Node(package="inspection", executable="inspection", name="inspection", output="screen"),
    ]

    return LaunchDescription([*launch_args, rosbridge, move_group, *mission_nodes])
