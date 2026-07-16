"""
rosbridge.launch.py

Bring up rosbridge -- the WebSocket bridge that exposes the ROS 2 graph to the
web console at ws://localhost:9090 (architecture.md decision 6 / §4). The host
has no ROS 2, so this is the ONLY path from the browser/backend to the graph.

We include rosbridge_suite's own stock launch (rosbridge_websocket_launch.xml)
rather than re-declaring the node, so we inherit its defaults (port 9090) and
stay compatible across rosbridge versions.

    ros2 launch qc_bringup rosbridge.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    rosbridge_launch = os.path.join(
        get_package_share_directory("rosbridge_server"),
        "launch",
        "rosbridge_websocket_launch.xml",
    )
    return LaunchDescription([
        IncludeLaunchDescription(AnyLaunchDescriptionSource(rosbridge_launch)),
    ])
