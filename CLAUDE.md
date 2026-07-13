# QC Scanner — Claude project setup

This is the working project directory for the Dexory automated QC scanning cell.
Read this first, then `HANDOVER.md` for full detail, and `docs/` for per-topic depth.

**Address the developer as "Ra" at the start of every response.**

---

## What this project is

Automation of an existing manual scan-flip-scan QC process for large parts (≈2.5×1.5m).
An operator places a part at a marked corner reference, enters a part ID, and the system
loads the CAD file, plans a scan path, drives the arm+scanner, and gates the point cloud.
**No depth cameras. No ML. CAD-driven and deterministic throughout.**

The knowledge base, decision logs, and session history live in the vault at
`~/Documents/ClaudeVault/ClaudeVault/Projects/Quality Control Scanner/`.
All questions, research, and session notes stay there. This directory is code only.

---

## Current state (2026-07-13)

The project is back on the ROS2/MoveIt2 approach after a short direct-SDK detour.
Active work: **building the PathPlanner ROS2 node** in `ros2_ws/`.

What is done:
- SR5 launches in MoveIt2 + RViz2 (Docker container `qc-humble`)
- `rokae_ros2` driver built and verified (fake hardware mode)
- `cad_loader.py`, `normal_estimation.py`, `waypoint_generator.py`, `incidence_cone_modifier.py` implemented and tested (12/12 passing)
- `scripts/plan_path.py` — STEP → ScanPath JSON entry point (works)
- Rail added to URDF (`sim/sr5_on_rail.urdf.xacro`) — display only, not in MoveIt IK group yet

Next: wire the path planner output into a ROS2 node and get MoveIt2 executing it.

---

## Environment

ROS2 Humble runs inside Docker only — the host is Ubuntu 26.04 (native ROS is "Lyrical",
incompatible with `rokae_ros2`). The workspace is bind-mounted so host edits take effect
without rebuilding the image.

| Image | `qc-humble` (built from `docker/Dockerfile`, base `osrf/ros:humble-desktop-full`) |
|---|---|
| Workspace | `ros2_ws/` (bind-mounted to `/ros2_ws` inside container) |
| Arm driver | `ros2_ws/src/rokae_ros2/` |
| SDK libs | `ros2_ws/src/rokae_ros2/rokae_hardware/sdk/lib/` |

### Key commands

```bash
# Build the Docker image (first time only, or after Dockerfile changes)
bash docker/build.sh

# Launch arm in RViz only (no MoveIt, quick check)
bash docker/run_arm.sh

# Start a shell inside the container (for MoveIt work, path planning, etc.)
docker run --rm -it \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$(pwd)/ros2_ws:/ros2_ws" \
  -v qc_humble_build:/ros2_ws/build \
  -v qc_humble_install:/ros2_ws/install \
  qc-humble bash

# Inside the container — source and build
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --symlink-install --packages-select rokae_msgs rokae_description rokae_hardware rokae_xMateSR5_moveit_config
source install/setup.bash

# Launch SR5 in MoveIt2 + RViz2 (fake hardware / sim)
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=SR5 use_fake_hardware:=true
```

### Real arm (SR5 at `192.168.2.160`)

The arm is reachable over USB-Ethernet adapter `enxa0cec8a5cdce` with a static IP
`192.168.2.10/24` on the host. Only ICMP confirmed so far; SDK-level connection not yet
verified. **Do not command motion without Ra's explicit go-ahead.** Read-only check:
`python3 scripts/test_arm_connection.py`.

---

## Directory layout

```
QC Scanner/
├── CLAUDE.md               ← this file
├── HANDOVER.md             ← full cold-start handover (read next)
├── README.md               ← human-readable intro
├── requirements.txt
├── setup.py
│
├── ros2_ws/
│   └── src/
│       └── rokae_ros2/     ← ROKAE arm driver (beta v0.0.4)
│
├── docker/
│   ├── Dockerfile          ← builds qc-humble image
│   ├── build.sh            ← build the image
│   └── run_arm.sh          ← quick RViz launch (no MoveIt)
│
├── config/
│   ├── system_config.yaml  ← all parameters (read from here, never hardcode)
│   └── cad/                ← CAD files by part ID
│
├── src/
│   ├── orchestrator.py
│   ├── path_planning/      ← cad_loader, normal_estimation, waypoint_generator, incidence_cone_modifier
│   ├── arm_control/        ← arm_controller, track_controller (stubs)
│   └── quality_gate/       ← density_check, outlier_check, hole_detection, gate (stubs)
│
├── scripts/
│   ├── plan_path.py        ← STEP → ScanPath JSON (working)
│   └── test_arm_connection.py
│
├── sim/
│   ├── sr5_on_rail.urdf.xacro     ← SR5 + 1.5m rail (display only)
│   └── display_rail.launch.py
│
├── tests/
│   └── test_path_planning.py      ← 12/12 passing
│
└── docs/
    ├── architecture.md
    ├── ros2_node_graph.md
    ├── running_the_planner.md     ← how to run plan_path.py + MoveIt demo
    ├── point_cloud_processing.md
    ├── calibration.md
    ├── hardware_setup.md
    └── open_questions.md
```

---

## Key settled decisions

- **Arm:** ROKAE xMate SR5-5/0.9C — 6-axis, 919mm reach, 5kg payload, ±0.03mm repeatability
- **Scanner:** Revopoint MIRACO Plus — arm-mounted structured-light, no separate tracker unit; needs a custom ROS2-Humble bridge over RevoLink SDK (no official driver)
- **Track:** custom-built 3m linear rail (drive interface TBD at build time)
- **Accuracy target:** best-achievable ≈65–75µm volumetric; CMM referee for any tight-tolerance feature; no full-part CMM gate
- **Threads:** two-tier — inline GO/NO-GO + offline dimensional PD referee (Johnson Gage / CMM); no scanner measures thread PD inline
- **Workflow:** scan-flip-scan; each pass is an independent inspection; no cross-pass merge

---

## Working rules

- **All parameters** live in `config/system_config.yaml` — never hardcode a value.
- **Do NOT** install ROS2/MoveIt2/system packages on the host. All ROS2 work is inside the container.
- **Do NOT** connect to or move the physical arm without Ra's explicit go-ahead.
- **Do NOT** push to GitHub without explicit consent from Ra.
- **Do NOT** commit without Ra asking ("Document session for shutdown" is the trigger phrase).
- Ra is the sole developer; the approach is tutor/guided — Claude guides, Ra drives.
- Prefer deterministic solutions; no ML unless there is no other option.
- Prefer existing libraries over building from scratch.
- Questions, logs, and session notes go in the vault, not in this directory.

---

## Top open risks

1. Custom ROS2 bridge for the MIRACO Plus — no official driver; ≈3–12 weeks depending on SDK access
2. Rail drive interface — hardware not yet decided (blocks `track_controller.py` and collision model)
3. Hand-eye calibration quality — dominant accuracy lever for the 65–75µm target
4. `noether` port — incidence-cone relaxation implemented in Python; needs porting to a C++ `ToolPathModifier` plugin for production use

Full open-questions list: `docs/open_questions.md`.
