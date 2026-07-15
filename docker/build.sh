#!/usr/bin/env bash
# Build the QC Scanner Humble ROS2 image (tag: qc-humble).
# Also clones the vendor rokae_ros2 into ros2_ws/src if missing (SR5 URDF+meshes).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ ! -d "$REPO/ros2_ws/src/rokae_ros2" ]; then
  echo "cloning RokaeRobot/rokae_ros2 (SR5 description + meshes)…"
  git clone --depth 1 https://github.com/RokaeRobot/rokae_ros2.git \
    "$REPO/ros2_ws/src/rokae_ros2"
fi

echo "building image qc-humble…"
docker build -t qc-humble "$REPO/docker"
echo "done. run the arm view with:  docker/run_arm.sh"
