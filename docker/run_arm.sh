#!/usr/bin/env bash
# Show the SR5 arm in RViz (movable via joint_state_publisher_gui), running in
# the Humble container with GUI forwarded to the host X display. No rail.
#
# Builds rokae_description inside the container on first run (cached in a named
# volume so later runs are fast), then launches robot_state_publisher +
# joint_state_publisher_gui + rviz2.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! docker image inspect qc-humble >/dev/null 2>&1; then
  echo "image qc-humble not found — building it first"; "$REPO/docker/build.sh"
fi

# Allow the container to talk to the host X server (revoked on exit).
xhost +local:root >/dev/null 2>&1 || true
trap 'xhost -local:root >/dev/null 2>&1 || true' EXIT

docker run --rm -it \
  --net=host \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$REPO/ros2_ws:/ros2_ws" \
  -v "$REPO/docker/view_arm.launch.py:/view_arm.launch.py:ro" \
  -v "$REPO/docker/view_arm.rviz:/view_arm.rviz:ro" \
  -v qc_humble_build:/ros2_ws/build \
  -v qc_humble_install:/ros2_ws/install \
  qc-humble \
  bash -lc 'source /opt/ros/humble/setup.bash && \
            cd /ros2_ws && \
            colcon build --packages-select rokae_description >/dev/null && \
            source install/setup.bash && \
            ros2 launch /view_arm.launch.py'
