"""Launch the PathPlanner node on its own: ros2 launch path_planner path_planner.launch.py"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="path_planner", executable="path_planner",
             name="path_planner", output="screen"),
    ])
