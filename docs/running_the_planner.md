# Running the PathPlanner (STEP → path → RViz)

> Part of [[Quality Control Scanner]]

How to take a STEP file, generate a scan path, and see it in RViz — and how
to add the 1.5 m rail to the sim. Every command here is run by **you**; none
of it is auto-run.

There are two halves:

1. **On the host** — turn a CAD file into a `ScanPath` JSON. Pure Python, no
   ROS/Docker.
2. **Inside the `qc_sim` container** — see that path in RViz, both as markers
   and by driving the arm through it, plus the arm on its rail.

---

## 0. One-time setup

On the **host** (the planner needs gmsh + numpy):

```bash
cd "Projects/Quality Control Scanner"
pip install -r requirements.txt          # gmsh, numpy, open3d, ...
```

`gmsh` is what reads STEP and tessellates it; `open3d` is only needed later
for Phase 2 point-cloud work, not for planning.

---

## 1. STEP → ScanPath JSON  (host)

```bash
python3 scripts/plan_path.py my_part.step
```

That prints a 4-stage progress log and writes `my_part_scanpath.json`. The
important knobs (all optional):

| Flag | Default | Meaning |
| --- | --- | --- |
| `--standoff-mm` | 300 | probe distance from the surface (config value) |
| `--fov-deg` | 40 | scanner field of view — **placeholder, confirm the MIRACO Plus spec** |
| `--overlap` | 0.3 | required overlap between adjacent scan lines |
| `--raster-spacing-mm` | *(from FOV)* | set line spacing directly instead of deriving it |
| `--along-track-mm` | 10 | waypoint spacing along each line |
| `--max-incidence-deg` | 25 | incidence-cone limit (the relaxation constraint) |
| `--step-axis` / `--travel-axis` | 1 / 0 | which axes lines stack along / travel along |
| `--samples` | 20000 | surface sample count |
| `--mesh-size-mm` | 5 | STEP tessellation edge length |

If the raster lines come out along the wrong axis for your part, swap
`--step-axis`/`--travel-axis` (0=X, 1=Y, 2=Z).

The output uses the same schema as `scanpath_example.json`, so every existing
tool (`convert_scanpath.py`, `run_replay.sh`) accepts it unchanged.

**Pipeline inside `plan_path.py`:**
`cad_loader` (STEP→mesh) → `normal_estimation` (surface points + outward
normals) → `waypoint_generator` (raster lines at the FOV-derived spacing) →
`incidence_cone_modifier` (orientation relaxation, line by line).

---

## 2. See the path in RViz — markers  (inside `qc_sim`)

The container only has `/ros2_ws` mounted, so copy the scripts + your JSON in
(same trick `run_replay.sh` uses):

```bash
# from the host
docker cp scripts/scanpath_convert.py qc_sim:/tmp/
docker cp scripts/visualize_path.py   qc_sim:/tmp/
docker cp my_part_scanpath.json       qc_sim:/tmp/
```

Then inside the container:

```bash
docker exec -it qc_sim bash
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
cd /tmp && python3 visualize_path.py my_part_scanpath.json
```

It publishes a **latched** `MarkerArray` on `/scan_path_markers` (path line +
target surface points + decimated aim arrows) in the `xMateSR5_base` frame.
Add a **MarkerArray** display in your open RViz pointed at that topic — or use
the rail display below, which already includes it.

---

## 3. See the path in RViz — drive the arm through it  (inside `qc_sim`)

**Verified working 2026-07-09** on the location-pin STEP: 122/132 waypoints
solved, trajectory executed `error_code=0`, arm swept the full raster in RViz.
Getting there needed fixes the stock image/config don't have — do these once
per fresh container:

**3a. Container packages** (the base `osrf/ros:humble-desktop-full` has neither
MoveIt nor ros2_control):

```bash
docker exec qc_sim bash -c "apt-get update && DEBIAN_FRONTEND=noninteractive \
  apt-get install -y ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers"
```

**3b. Use the fixed launch, NOT the vendor `demo.launch.py`.** On MoveIt 2.5.9
the vendor demo crashes two ways: (1) it can't pick between
`simple_moveit_controllers.yaml` and `ros2_controllers.yaml`, and (2) the
vendor `kinematics.yaml` is wrapped in `/**: ros__parameters:`, so the KDL IK
solver never loads for `rokae_arm` and every `/compute_ik` returns
NO_IK_SOLUTION. Both are handled by `sim/qc_moveit_demo.launch.py` +
`sim/kinematics_fixed.yaml`:

```bash
# from the host — copy the sim bundle into the mounted workspace
docker cp sim qc_sim:/ros2_ws/qc_sim
# inside the container
docker exec -it qc_sim bash
  source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
  cp /ros2_ws/qc_sim/kinematics_fixed.yaml /tmp/

  # If you've launched before in this same container, kill any leftover
  # nodes first -- relaunching on top of a stale move_group/rviz silently
  # does nothing visible (rviz can die from a prior run without you noticing).
  pkill -9 -f rviz; pkill -9 -f move_group; pkill -9 -f ros2_control; pkill -9 -f spawner
  sleep 2

  ros2 launch /ros2_ws/qc_sim/qc_moveit_demo.launch.py     # RViz opens
```

If nothing appears on screen after ~15s, from the host check the RViz process is actually alive (not `<defunct>`):
`docker exec qc_sim bash -c "ps aux | grep rviz2 | grep -v grep"` — a real PID with CPU/memory usage means it's running (an X11/DISPLAY issue, not ROS); no matching line or only `<defunct>` means it crashed and needs the kill+relaunch above.

Sanity check IK is alive before replaying:
`ros2 param get /move_group robot_description_kinematics.rokae_arm.kinematics_solver`
should return the KDL plugin (not "Parameter not set"), and
`ros2 control list_controllers` should show **both** `joint_state_broadcaster`
and `position_joint_trajectory_controller` **active**.

**3c. Replay** (from the host, once RViz is up):

```bash
bash scripts/run_replay.sh my_part_scanpath.json
```

It solves IK per waypoint (seeded for joint-space continuity), sends one
trajectory, and the arm sweeps the path. Unreachable waypoints are skipped,
not fatal.

---

## 4. Add the 1.5 m rail to the sim

The rail lives in `sim/`. Copy the whole folder into the mounted workspace so
it persists and `package://rokae_description` meshes resolve:

```bash
# from the host
docker cp sim qc_sim:/ros2_ws/qc_sim
# (or: cp -r sim ~/ros2_ws/qc_sim  — /ros2_ws is bind-mounted)
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
ros2 launch /ros2_ws/qc_sim/display_rail.launch.py
```

You get RViz (preloaded `scan_path.rviz`) + a **joint_state_publisher_gui**
window with a slider per joint, including **`rail_joint` (0–1.5 m)** — drag it
and the whole arm slides along the rail. Run `visualize_path.py` alongside to
see the scan path in the same scene.

### Caveats / what's NOT done yet

- The rail is a **movable display joint**, not yet a MoveIt-planned coordinated
  axis. Making MoveIt *plan* the rail needs edits to the SR5 moveit_config
  (`config/xMateSR5.srdf` group, `xMateSR5.ros2_control.xacro`,
  `ros2_controllers.yaml`). Flagged as the next step. This matches the real
  cell, which is **index-and-shoot** — the rail is positioned independently of
  the arm's IK.
- Rail length is **1.5 m** here (your call, 2026-07-09). `config/system_config.yaml`
  and the hardware note still say the physical track is **3 m** — reconcile
  before this drives hardware.
- `plan_path.py` is the **R&D pure-Python planner** (binning raster + the
  incidence-cone relaxation). The production planner is noether's raster slicer
  + the C++ `ToolPathModifier` port. The binning approximation is best for
  roughly prismatic parts; deep concavities can mis-orient normals (star-convex
  outward-flip heuristic in `normal_estimation.py`).
- `--fov-deg` is a placeholder until the MIRACO Plus FOV is confirmed.
