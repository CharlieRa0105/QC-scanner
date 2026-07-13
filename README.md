# Dexory automated QC scanner

This is an automated 3D scanning cell for quality control. An operator
places a part on a table, enters a part ID, and presses go. The system
works out the shape of the part on its own, plans a scan path around it,
drives a high-resolution probe over every surface, and then checks that
the resulting scan is complete before passing it downstream.

Parts vary widely — base plates, brackets, belt clamps, and so on. Rather
than measure the part live, the operator aligns it to a marked corner
reference on the table and enters its part ID; the scan path is generated
fresh each time from that part's pre-made CAD file. After scanning, the
same CAD model is reused as the reference to confirm the scan didn't miss
anything, and later to compute the pass/fail deviation.

This repository is the R&D codebase for that system. It is structured for
a clean handoff: the layout and documentation matter as much as the code.
At this stage the modules are scaffolding with documented intent — the
implementation is filled in as hardware arrives and decisions are
settled.

## How it works

A scan job runs end to end like this:

1. The operator places the part at the marked corner reference and enters
   its part ID.
2. The part's CAD file is loaded — its position and orientation are known
   from the corner reference, no live measurement needed.
3. Surface normals are estimated across the CAD mesh.
4. A coverage path is generated — probe held at 300mm from each surface,
   facing it square on.
5. The path is validated against a collision model.
6. The arm and track run the path while the scanner records.
7. The scan is exported as a point cloud.
8. A quality gate scores the scan for density, noise, and missing
   coverage (against the CAD-derived expected coverage).
9. If it passes, the scan goes downstream. If not, it rescans, up to two
   attempts, then flags the part for a human.

The full pipeline and module breakdown are in
[docs/architecture.md](docs/architecture.md).

## Hardware

The cell is built around four pieces of hardware:

- **Revopoint MIRACO Plus** — arm-mounted structured-light 3D scanner. Uses
  its own photogrammetry for pose tracking, so there is no separate tracker
  unit — the arm's repeatability enters the measurement chain instead.
  Accuracy target ~65–75µm volumetric (scan-vs-CAD); optimal standoff 300mm.
  Controlled via the Revolink SDK over a custom ROS2-Humble bridge.
- **ROKAE xMate SR5-5/0.9C** 6-axis force-controlled cobot (5kg payload,
  919mm reach, ±0.03mm repeatability), which carries the scanner. Controlled
  via the xCore SDK and the `rokae_ros2` ROS2 Humble driver.
- **Custom-built 3m linear floor track** the arm rides on, running along the
  table so the arm's short reach still covers the whole part
  (index-and-shoot — the track is not a coordinated joint).
- **~2.5×2.5m table** the part (up to ~2.5×1.5m) sits on, aligned to a marked
  corner reference. The part does not move during a pass; an operator flips
  it between the two passes.

A tool changer also carries a **tracker-referenced contact thread probe** for
dimensional thread gauging. How these connect and mount is covered in
[docs/hardware_setup.md](docs/hardware_setup.md). Several physical details
(track product, mounting heights, max part height) are still open — see
[docs/open_questions.md](docs/open_questions.md).

## Software and installation

The system targets Ubuntu 22.04 with ROS2 Humble, MoveIt2, and Python
3.10+. The Python dependencies are in
[requirements.txt](requirements.txt):

```
pip install -r requirements.txt
pip install -e .
```

ROS2, MoveIt2, and the `rokae_ros2` driver are installed separately on the
on-site Ubuntu machine following their own instructions — they are not pip
packages and are not installed by the steps above.

Site-specific settings (real IP addresses and so on) go in
`config/local_config.yaml`, which is git-ignored. The committed defaults
and placeholders live in
[config/system_config.yaml](config/system_config.yaml).

## Running a scan

```
python scripts/run_scan.py
```

This prompts for a part ID and runs the full pipeline. Before the first
run, check arm connectivity:

```
python scripts/test_arm_connection.py   # confirm arm + track reachable
```

## Layout

```
config/                  system parameters and thresholds
  system_config.yaml

src/
  path_planning/
    cad_loader.py              STEP/STL/OBJ -> triangle mesh (via gmsh)
    normal_estimation.py       surface sampling + outward normals from the CAD mesh
    waypoint_generator.py      raster coverage: probe poses at standoff along normals
    incidence_cone_modifier.py PathPlanner's custom orientation-relaxation algorithm
    collision_check.py         MoveIt2 collision validation wrapper
  arm_control/
    arm_controller.py      SR5 control via xCore SDK / rokae_ros2 — connect, move, home, status
    track_controller.py    custom 3m track control (index-and-shoot)
  quality_gate/
    density_check.py      voxel-grid density validation
    outlier_check.py      statistical outlier removal and ratio check
    hole_detection.py     coverage comparison against the CAD-derived reference
    gate.py               runs all checks, returns pass/fail + score
  orchestrator.py         top-level pipeline tying the modules together

scripts/
  plan_path.py            STEP/CAD -> ScanPath JSON (the PathPlanner entry point)
  visualize_path.py       publish a ScanPath to RViz as markers (in-container)
  replay_scanpath.py      drive the SR5 through a ScanPath in the sim (in-container)
  run_replay.sh           copy scripts + path into qc_sim and replay
  scanpath_convert.py     shared mm->m / Y-up->Z-up frame remap
  run_scan.py             CLI entry point for a full scan (Phase 1 goal)
  test_arm_connection.py  quick connectivity check for arm + track

sim/
  sr5_on_rail.urdf.xacro  SR5 on a 1.5m movable prismatic rail
  display_rail.launch.py  RViz + joint sliders (incl. the rail) for that model
  scan_path.rviz          RViz layout: RobotModel + TF + scan-path markers
  qc_moveit_demo.launch.py MoveIt demo that works on MoveIt 2.5.9 (vendor demo.launch.py doesn't)
  kinematics_fixed.yaml   de-wrapped kinematics so KDL IK loads for rokae_arm

tests/                    test suite, with sample clouds in fixtures/

docs/                     detailed documentation (see below)
```

## Documentation

- [docs/running_the_planner.md](docs/running_the_planner.md) — STEP → path →
  RViz, and adding the rail to the sim (step-by-step commands)
- [docs/architecture.md](docs/architecture.md) — system overview,
  pipeline, and module map
- [docs/hardware_setup.md](docs/hardware_setup.md) — physical setup of
  arm, track, and scanner
- [docs/calibration.md](docs/calibration.md) — the hand-eye and
  corner-reference calibration chain
- [docs/adding_new_tools.md](docs/adding_new_tools.md) — adding a new end
  effector via the tool changer
- [docs/open_questions.md](docs/open_questions.md) — unresolved decisions
  and what each one blocks
