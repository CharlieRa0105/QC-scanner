#!/usr/bin/env python3
"""
move_group.launch.py  (runs INSIDE the Humble container)

A FIXED move_group bringup for the SR5, working around two bugs in the vendor
rokae_xMateSR5_moveit_config on Humble / MoveIt 2.5.9:

  1. `trajectory_execution: Unable to guess which parameter file to load` — the
     config ships both simple_moveit_controllers.yaml and ros2_controllers.yaml
     and the auto-glob can't choose. Fixed by naming the moveit-controllers file
     explicitly.
  2. Wrapped kinematics.yaml (`/**: ros__parameters:`) → KDL solver never loads →
     NO_IK_SOLUTION. Fixed by loading the de-wrapped /kinematics_fixed.yaml
     (bind-mounted in by run_preview.sh).

This is planning-only (no real controllers needed) — enough for
compute_cartesian_path + RViz playback previews.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("xMateSR5", package_name="rokae_xMateSR5_moveit_config")
        .robot_description(file_path="config/xMateSR5.urdf.xacro")
        .robot_description_semantic(file_path="config/xMateSR5.srdf")
        .robot_description_kinematics(file_path="/kinematics_fixed.yaml")
        .trajectory_execution(file_path="config/simple_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
