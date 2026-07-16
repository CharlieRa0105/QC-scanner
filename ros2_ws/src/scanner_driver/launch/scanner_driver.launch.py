"""Launch the scanner_driver node: ros2 launch scanner_driver scanner_driver.launch.py"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="scanner_driver", executable="scanner_driver", name="scanner_driver", output="screen"),
    ])
