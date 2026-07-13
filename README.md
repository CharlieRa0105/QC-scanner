# QC Scanner

Automated 3D scanning cell for quality control (ROS2 / MoveIt2 / PathPlanner pipeline).

## Layout

- `ros2_ws/` — colcon workspace. ROS2 packages live under `ros2_ws/src/`; `build/`, `install/`, `log/` are generated and gitignored.
- `docker/` — Dockerfile(s) and compose config for the dev/sim environment.
- `config/` — system config, robot/scan parameters, CAD-derived scanpaths.
- `scripts/` — setup and utility scripts outside the ROS2 build.
- `sim/` — simulation assets/worlds.
- `tests/` — integration/system-level tests (outside the per-package colcon tests).
- `docs/` — architecture, hardware setup, calibration, and other project docs.

## Getting started

```bash
cd ros2_ws
colcon build
source install/setup.bash
```
