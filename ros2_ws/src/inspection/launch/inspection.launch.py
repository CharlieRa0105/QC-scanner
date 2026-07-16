"""Launch the inspection node: ros2 launch inspection inspection.launch.py"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="inspection", executable="inspection", name="inspection", output="screen"),
    ])
