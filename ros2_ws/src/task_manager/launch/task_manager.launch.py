"""Launch the task_manager node: ros2 launch task_manager task_manager.launch.py"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="task_manager", executable="task_manager", name="task_manager", output="screen"),
    ])
