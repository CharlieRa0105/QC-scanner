"""Launch the whole teach system: arm driver + slider driver + teach GUI.

Usage:
    ros2 launch dexory_teach_ros teach.launch.py                 # all mock
    ros2 launch dexory_teach_ros teach.launch.py arm_backend:=rokae robot_ip:=192.168.2.160
    ros2 launch dexory_teach_ros teach.launch.py rail_backend:=roboteq port:=/dev/ttyUSB0
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_backend = LaunchConfiguration("arm_backend")
    rail_backend = LaunchConfiguration("rail_backend")
    robot_ip = LaunchConfiguration("robot_ip")
    sdk_root = LaunchConfiguration("sdk_root")
    drag_button_key = LaunchConfiguration("drag_button_key")
    read_keypad = LaunchConfiguration("read_keypad")
    hold_to_drag = LaunchConfiguration("hold_to_drag")
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument("arm_backend", default_value="mock",
                              description="mock | rokae"),
        DeclareLaunchArgument("rail_backend", default_value="mock",
                              description="mock | roboteq"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.2.160"),
        DeclareLaunchArgument("sdk_root", default_value=os.path.expanduser("~/rokae_sdk"),
                              description="path to the Linux xCore SDK (rokae backend)"),
        DeclareLaunchArgument("drag_button_key", default_value="5",
                              description="which end-handle key (1..7) triggers capture; SR5=CR5"),
        DeclareLaunchArgument("read_keypad", default_value="false",
                              description="poll the handle keypad for CR5 auto-capture; OFF by default "
                                          "because polling can stall hand-guiding"),
        DeclareLaunchArgument("hold_to_drag", default_value="true",
                              description="true = hold end button to drag (matches standalone); "
                                          "false = free-drag"),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),

        Node(package="sr5_arm_driver", executable="arm_driver", name="ArmDriver",
             output="screen",
             parameters=[{"backend": arm_backend, "robot_ip": robot_ip, "sdk_root": sdk_root,
                          "drag_button_key": drag_button_key, "read_keypad": read_keypad,
                          "hold_to_drag": hold_to_drag}]),

        Node(package="rail_driver", executable="rail_driver", name="RailDriver",
             output="screen",
             parameters=[{"backend": rail_backend, "port": port}]),

        Node(package="dexory_teach_ros", executable="teach", name="dexory_teach_gui",
             output="screen"),
    ])
