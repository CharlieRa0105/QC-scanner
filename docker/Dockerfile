# QC Scanner — ROS2 Humble environment for the SR5 (arm viz now; MoveIt/rail later).
#
# The ROKAE SDK + rokae_ros2 only support ROS2 Humble, so all ROS2 work runs in
# this container rather than on the host. Base image already has rviz2, xacro,
# colcon; we just add joint_state_publisher_gui (missing from desktop-full) and
# a couple of conveniences.
#
# The workspace (ros2_ws with the cloned rokae_ros2) is bind-mounted at runtime
# at /ros2_ws — NOT copied — so edits on the host take effect without rebuilding
# the image, and /ros2_ws has no space in its path (unlike the host repo dir),
# which lets package:// mesh URIs resolve after a normal colcon build.
FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-humble-joint-state-publisher-gui \
        ros-humble-joint-state-publisher \
        ros-humble-moveit \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*
# IK solver: MoveIt's built-in KDL plugin (ships with ros-humble-moveit) is used
# for now. TracIK is NOT in the Humble apt repo (no ros-humble-trac-ik-* package),
# so adding it means a from-source colcon build of trac_ik_kinematics_plugin --
# deferred until/unless KDL's reliability proves insufficient.

# Source ROS2 for every interactive shell.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["bash"]
