"""Launch the movement_driver node: ros2 launch movement_driver movement_driver.launch.py"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="movement_driver", executable="movement_driver", name="movement_driver", output="screen"),
    ])
