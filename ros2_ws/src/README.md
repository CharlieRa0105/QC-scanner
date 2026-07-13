# ros2_ws/src — ROS 2 packages (robot drivers + teach GUI)

The robot-driver layer for the QC scanner. Built with `colcon`. Targets
**ROS 2 Humble / Ubuntu 22.04** (the on-site stack); uses only
`rclpy` + `std_msgs`/`sensor_msgs`/`std_srvs`, so it builds unchanged on Humble.
Developed and smoke-tested on Jazzy in WSL via `../build_local.sh`.

## Packages

| Package | Node | Node-graph role ([docs/ros2_node_graph.md](../../docs/ros2_node_graph.md)) |
|---|---|---|
| `sr5_arm_driver` | **ArmDriver** | thin SR5 layer — sub `armCMD`, pub `armPos` |
| `rail_driver` | **RailDriver** | thin 3 m track layer — sub `railCMD`, pub `railPos` |
| `dexory_teach_ros` | `dexory_teach_gui` | hand-teach / jog GUI (not part of the scan runtime graph) |

Each driver has two interchangeable backends selected by the `backend` parameter:
`mock` (pure-Python simulation, default) and the real driver (`rokae` = xCore SDK
for the arm; `roboteq` = serial for the track).

## Shared with the operator console

The driver **backends are plain Python** (`sr5_arm_driver/backends.py`,
`rail_driver/backends.py` — no `rclpy`). The console backend
(`../../backend/robot_bridge.py`) imports `sr5_arm_driver.backends` directly and
reuses `RokaeArm`/`MockArm` for its read-only status + joint telemetry, so there is
**one robot-driver implementation** shared by the ROS 2 node and the console.
The SDK allows only one TCP session — don't run the console's real connection and
the ROS 2 `ArmDriver` against the same arm simultaneously.

## Node-graph interface (spec topics)

| Topic | Type | Dir | Meaning |
|---|---|---|---|
| `armCMD` | `std_msgs/Float64MultiArray` `[j1..j6, speed_mm/s]` | → ArmDriver | joint target + EE speed (mm/s) |
| `armPos` | `sensor_msgs/JointState` (6 joints, rad) | ArmDriver → | live joint feedback |
| `railCMD` | `std_msgs/Float64MultiArray` `[position_m, speed_mm/s]` | → RailDriver | track target |
| `railPos` | `sensor_msgs/JointState` (1 prismatic, m) | RailDriver → | live track position |

Types were unspecified in the node-graph doc — these are the implemented choices.
Aux setup/teach topics (`{/arm,/rail}/status`, `/set_power`, `/set_drag`, `/stop`,
`/home`, `/clear_alarm`, `/connect`, `/backend`, `/arm/drag_button`) support
hand-teaching/bring-up and sit outside the scan-runtime graph.

## Build & run

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch dexory_teach_ros teach.launch.py                              # all simulated
ros2 launch dexory_teach_ros teach.launch.py arm_backend:=rokae robot_ip:=<ip>
ros2 launch dexory_teach_ros teach.launch.py rail_backend:=roboteq port:=/dev/ttyUSB0
```

Status: builds + self-test PASS on Jazzy/WSL (mock); real `rokae` arm backend has
connected to an SR5. Not yet built on the Humble on-site machine.
