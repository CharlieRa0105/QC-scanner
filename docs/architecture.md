# Architecture

> Part of [[Quality Control Scanner]]

This document describes how the QC scanning system is organised and how a
single scan job flows through it.

## Overview

The system drives a **Revopoint MIRACO Plus** arm-mounted structured-light 3D
scanner, mounted on a **ROKAE xMate SR5-5/0.9C**
6-axis cobot (5kg payload, 919mm reach) that rides a **custom-built 3m linear
floor track** around a ~2.5×2.5m table, to scan parts up to ~2.5×1.5m.

The MIRACO Plus is **not externally referenced** — its position is tracked by its
own photogrammetry, so the arm's repeatability (±0.03mm on the SR5) does enter
the measurement chain. The cobot carries and aims the scanner.

The part is placed at a **marked corner reference** on the table (fixed position
and orientation). The scan path is generated from the **pre-made CAD file** for
that part, loaded by part ID. No depth cameras are used — part geometry is
known from CAD, not measured live.

## Workflow — scan-flip-scan

This automates an existing manual process and mirrors it exactly: scan the
up-facing surfaces → compare to CAD → an operator flips the part → scan the
other side → compare to CAD again. The two passes are **independent, complete
inspections** — there is **no cross-pass registration or cloud merge**. Each
pose is scanned and gated on its own. Because the arm only ever works the
upper hemisphere, it never has to reach under the part.

## Pipeline

A scan job runs in this order. The orchestrator
([src/orchestrator.py](../src/orchestrator.py)) owns this sequence.

1. **Operator input** — operator places the part at the marked corner, enters a
   part ID, and presses go. Entry point is [scripts/run_scan.py](../scripts/run_scan.py).

2. **CAD load** — the CAD file for the part ID is loaded. The part's position and
   orientation are known from the marked corner — no live measurement needed.
   See [src/path_planning/cad_loader.py](../src/path_planning/cad_loader.py).

3. **Normal estimation** — surface normals are computed from the CAD mesh with
   Open3D. Each normal tells the probe which way to face at that point. See
   [src/path_planning/normal_estimation.py](../src/path_planning/normal_estimation.py).

4. **Waypoint generation** — for each surface point, a probe pose is
   placed at the 300mm standoff distance along the normal, perpendicular
   to the surface. Arm and track positions are derived from these poses.
   See [src/path_planning/waypoint_generator.py](../src/path_planning/waypoint_generator.py).

5. **Collision check** — the path is validated against the collision
   model (table, track limits, self-collision) via
   MoveIt2. See [src/path_planning/collision_check.py](../src/path_planning/collision_check.py).

6. **Execution** — the track indexes to a station, the arm poses the
   scanner, it settles, captures, and confirms before advancing
   (**index-and-shoot**; the track is not a coordinated joint). Cornering is
   done as straight segments with an index-at-corner stop. See
   [src/arm_control/arm_controller.py](../src/arm_control/arm_controller.py)
   and [src/arm_control/track_controller.py](../src/arm_control/track_controller.py).

7. **Capture + export** — the scanner is triggered via the Revopoint SDK;
   the scan is exported as a point cloud (.ply). Revopoint does not ship an
   official ROS2 driver — a custom ROS2-Humble bridge over the RevoLink SDK
   is required.

8. **Quality gate** — the exported scan is validated. See below.

9. **Decision** — if the gate passes, the scan moves downstream. If it
   fails, the job rescans, up to a maximum of two attempts, then flags
   the part for human review.

## Quality gate

The gate ([src/quality_gate/gate.py](../src/quality_gate/gate.py)) runs
three independent checks and combines them into a single 0–1 score:

- **Density** — a voxel grid over the scan's bounding volume; each
  occupied cell must hold a minimum point count.
  [density_check.py](../src/quality_gate/density_check.py)
- **Outliers** — statistical outlier removal; a high removed-ratio
  signals a noisy scan.
  [outlier_check.py](../src/quality_gate/outlier_check.py)
- **Holes** — scan coverage compared against the expected coverage derived
  from the CAD model; missing regions are flagged as holes.
  [hole_detection.py](../src/quality_gate/hole_detection.py)

If the combined score clears the configured threshold, the scan passes.

## Registration & deviation analysis

Once a scan clears the quality gate, it is registered to the CAD model and
compared against it to produce the actual pass/fail measurement — this is
distinct from the gate above, which only judges whether the *scan itself* is
good enough to analyse. TEASER++ (global registration) + Point-to-Plane ICP
(refinement, Open3D) align the scan onto the CAD; a Cloud-to-Mesh
signed-distance query (Open3D `RaycastingScene`, BVH-accelerated) then gives
per-point deviation, from which mean/RMSE/std and a colour heatmap are
generated. Deterministic algorithms only — see
[point_cloud_processing.md](point_cloud_processing.md) for the full design
and library choices.

## Configuration

All tunable values — standoff distance, motion speeds, gate thresholds,
CAD file paths — live in
[config/system_config.yaml](../config/system_config.yaml). Modules read
from this file rather than hard-coding constants. Site-specific overrides
with real IP addresses go in `config/local_config.yaml`, which is
git-ignored.

## Coordinate frames

The marked corner on the table defines the working coordinate frame for path
planning. The CAD model is registered to this frame at setup — the part is
always placed at the corner, so the part-to-arm transform is fixed. Establishing
the relationship between the corner frame and the arm's base frame is part of
on-site calibration — see [calibration.md](calibration.md). Getting these frames
aligned is the single most important thing to get right; a small frame error puts
the probe at the wrong standoff everywhere.

## ROS 2 node graph

The runtime is a set of ROS 2 nodes exchanging messages over topics — the
pub/sub realisation of the pipeline above (TaskManager → PathPlanner →
MovementDriver → arm/rail/scanner drivers, with a feedback loop on
`movementState`). The full node/topic map, control loop and open questions are
in [ros2_node_graph.md](ros2_node_graph.md).

## Module map

| Area | Module | Role |
|------|--------|------|
| Path planning | cad_loader.py | Load CAD mesh for part ID |
| Path planning | normal_estimation.py | Surface normals from CAD mesh |
| Path planning | waypoint_generator.py | Probe poses at standoff |
| Path planning | collision_check.py | MoveIt2 validation |
| Arm control | arm_controller.py | SR5 motion via xCore SDK / `rokae_ros2` |
| Arm control | track_controller.py | 7th-axis track motion |
| Quality gate | density_check.py | Voxel density |
| Quality gate | outlier_check.py | Outlier ratio |
| Quality gate | hole_detection.py | Coverage vs reference |
| Quality gate | roughness_check.py | PCA surface-noise check *(planned)* |
| Quality gate | gate.py | Combined score + decision |
| Registration | align_to_cad.py | TEASER++ global alignment + ICP refine *(planned)* |
| Inspection | deviation.py | C2M signed distance, stats, heatmap *(planned)* |
| Top level | orchestrator.py | Runs the full pipeline |
