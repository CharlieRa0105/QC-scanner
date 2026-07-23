# QC Scanner — Architecture (of record)

This is the **settled architecture** the code conforms to — complete and
self-contained. Every node, and every interface between nodes (topic / service /
action, with message type and direction), is defined here. There should be no
"TBD" left in the runtime contract; where a value is a tuning default it is stated
as one. If code and this doc disagree, this doc wins — or update it deliberately.

The reasoning, alternatives, and decision history live in Ra's vault (home
machine). This file is what a fresh session on any machine reads.

> Status tags: **built** · **partial** (some code, not wired in) · **to build**.

> **Status refreshed 2026-07-23.** The §3 node table reflects the current build:
> the full mission graph is now scaffolded (all six nodes register their
> interfaces), and MoveIt, rosbridge, the `qc-humble` Docker container, and a
> browser three.js viewer are wired. The §4 interface contract was the build
> target and is largely met. ScanningDriver and Phase-2 Inspection remain
> **interface-only** (hardware / algorithm-blocked). The marked-corner
> calibration (decision 5) is still a placeholder shift, not a measured transform.

---

## 1. What the system does

An automated 3D scanning cell for quality control. An operator places a part
(≤ ~1500 × 700 mm) at a **marked corner reference** on a ~2000 × 800 mm table,
selects the part by its DEX number, previews the planned scan, confirms, and the
**ROKAE xMate SR5** arm (scanner on the flange) traces the path while the scanner
captures. A Phase-2 pipeline then registers the cloud to CAD and produces a
deviation analysis a human QC operator reads to pass/fail the part.

**CAD-driven, deterministic, no depth cameras, no ML.** The part pose is known from
the marked corner (calibrated once), so the path is generated from the CAD mesh.

**Hardware**

| Component | Detail |
| --- | --- |
| Arm | ROKAE xMate SR5-5/0.9C — 6-axis, 919 mm reach, ±0.03 mm repeatability, 5 kg payload, at `192.168.2.160` |
| Scanner | Revopoint MIRACO Plus — arm-mounted structured-light, self-tracking, 20–30 cm standoff (capture not integrated yet) |
| Table | ~2000 × 800 mm; parts ≤ 1500 × 700 mm; **no rail** — the arm reaches the whole part from a fixed base |
| Host | Ubuntu; ROS 2 Humble runs in Docker only (the host has no ROS 2) |

---

## 2. The two systems

Two halves meet at one signal (`/mission/state`):

- **Control (the ROS 2 mission graph)** — gets the arm + scanner to every waypoint
  and captures a cloud.
- **Inspection (Phase 2)** — turns the cloud into a verdict aid. Five stages:
  **extract → clean → quality-check → align → analyse.** The quality-check is the
  *one automatic pass/fail*; a fail triggers a rescan, a pass yields the deviation
  analysis for a human to judge.

---

## 3. Nodes

Nine runtime participants (six ROS 2 nodes + the arm/rail drivers + rosbridge +
the web app + the Phase-2 node). The path-planning library runs *inside*
PathPlanner — it is not a separate node.

| Node | Role | Status |
| --- | --- | --- |
| **TaskManager** | Mission orchestrator. Owns the mission lifecycle and `/mission/state`; sequences plan → (operator confirm) → execute → scan → inspect; owns the rescan loop and **mission abort**. | **built** (scaffold; sequencing real, end-to-end untested) |
| **PathPlanner** | Owns the **whole plan**: generate coverage waypoints from CAD, apply the marked-corner transform, load part + table into MoveIt as collision objects, run MoveIt to produce the collision-free **trajectory**. Publishes the scan path + trajectory for preview. Plan-fully, then hand off. | **built** (coverage + frame transform + MoveIt cartesian/free-space + per-waypoint IK; degrades to scanpath-only when move_group is down) |
| **MovementDriver** | **Executes only.** Plays the planned trajectory to ArmDriver, watches joint feedback, confirms progress. No planning. | **built** (scaffold; playback logic real, needs a live trajectory + arm to exercise) |
| **ArmDriver** | Thin SR5 layer: joint command in, joint state + status out; power/drag/stop/home/clear-alarm services. Mock + real (xCore SDK) backends. | built |
| **RailDriver** | Dormant stub — there is no rail (kept for a possible future larger part). | out of scope |
| **ScanningDriver** | MIRACO Plus bridge. start/stop from TaskManager; reports capture state + the cloud file path. Continuous capture during the sweep. | **interface only** (node built; hardware-blocked — no MIRACO SDK, never fabricates a cloud) |
| **InspectionNode (Phase 2)** | Thin ROS wrapper over a pure-Python inspection library (extract → clean → quality → register → deviation). Runs the quality gate + deviation analysis; requests a rescan on quality fail. | **interface only** (node built; pipeline stages need Open3D/TEASER++ + real scans) |
| **rosbridge** | `rosbridge_suite` in the container — a WebSocket (`ws://localhost:9090`) exposing the graph to the web app. The host has no ROS 2, so this is the only web ↔ ROS 2 path. | **built** (`qc_bringup/launch/rosbridge.launch.py`) |
| **Web app** | The operator console (browser + backend). Calls mission services, subscribes to state + telemetry, renders the preview. | **built**; a browser three.js viewer (`gui/viewer/`) now animates MoveIt's planned trajectory live over rosbridge |

---

## 4. The interface contract (complete)

**Conventions (decided):** topic/service/action names are **snake_case** (ROS 2
standard). One-shot commands are **services**; long-running operations with
progress + cancel are **actions**; continuous state/telemetry are **topics**.
Custom message/service/action types live in a **`qc_msgs`** package (to build).

> The two *built* drivers currently use camelCase names (`/armCMD`, `/armPos`,
> `/railCMD`, `/railPos`). The canonical names below are snake_case; renaming them
> is a one-line-per-topic cleanup (tracked in `refactor-guide.md`). Canonical names
> are used throughout this doc.

### 4.1 Custom interfaces — `qc_msgs` (built)

**Messages**
- `MissionState.msg` — `string phase` · `string part_id` · `uint32 attempt` · `string detail`
  - `phase ∈ { idle, planning, planned, executing, scanning, inspecting, complete, rescanning, aborted, error }`
- `ScanWaypoint.msg` — `geometry_msgs/Pose pose` (probe pose, arm frame) · `geometry_msgs/Point target` (surface aim point) · `float32 incidence_deg` · `uint32 line_id`
- `ScanPath.msg` — `string part_id` · `float32 standoff_mm` · `ScanWaypoint[] waypoints`
- `ScanState.msg` — `string state` (`idle|scanning|done|error`) · `string cloud_path` · `string detail`
- `InspectionResult.msg` — `bool quality_pass` · `bool rescan_requested` · `uint32 attempt` · `string report_path` · `float32 mean_dev_mm` · `float32 rmse_mm` · `float32 coverage_pct`

**Services**
- `StartMission.srv` — request `string part_id` → response `bool accepted, string message`

**Actions**
- `PlanPath.action` — goal `string part_id` → result `bool success, moveit_msgs/RobotTrajectory trajectory, ScanPath scanpath, string message` — feedback `string stage, float32 progress`
- `ExecutePath.action` — goal `moveit_msgs/RobotTrajectory trajectory` → result `bool success, string message` — feedback `uint32 point_index, uint32 total`
- `Inspect.action` — goal `string cloud_path, string part_id` → result `InspectionResult result` — feedback `string stage, float32 progress`

### 4.2 Per-node interfaces

**TaskManager**
| Interface | Kind | Type | Meaning |
| --- | --- | --- | --- |
| `/mission/plan` | service (server) | `qc_msgs/StartMission` | Plan a mission for a part (no motion). Triggers PathPlanner. |
| `/mission/execute` | service (server) | `std_srvs/Trigger` | Operator confirm — run execute → scan → inspect. |
| `/mission/abort` | service (server) | `std_srvs/Trigger` | Abort the current mission; cancels active actions. |
| `/mission/state` | topic (pub, latched) | `qc_msgs/MissionState` | Current mission phase — the UI + Phase-2 read this. |
| `/plan_path` | action (client) | `qc_msgs/PlanPath` | → PathPlanner. |
| `/execute_path` | action (client) | `qc_msgs/ExecutePath` | → MovementDriver. |
| `/inspect` | action (client) | `qc_msgs/Inspect` | → InspectionNode. |
| `/scan/start`, `/scan/stop` | service (client) | `std_srvs/Trigger` | → ScanningDriver. |
| `/scan/state` | topic (sub) | `qc_msgs/ScanState` | Capture state from ScanningDriver. |

**PathPlanner**
| Interface | Kind | Type | Meaning |
| --- | --- | --- | --- |
| `/plan_path` | action (server) | `qc_msgs/PlanPath` | Generate coverage waypoints → arm-frame transform → load part+table as MoveIt collision objects → run MoveIt → trajectory + scanpath. |
| `/plan/scanpath` | topic (pub, latched) | `qc_msgs/ScanPath` | The scan path, for the web viewer to draw. |
| `/plan/trajectory` | topic (pub, latched) | `trajectory_msgs/JointTrajectory` | The planned joint trajectory, for the viewer's arm **preview** animation. |
| (uses MoveIt) | — | `move_group`, planning scene | IK + collision-free planning; cartesian (`compute_cartesian_path`) *along* each scan line, free-space *between* lines. |

**MovementDriver**
| Interface | Kind | Type | Meaning |
| --- | --- | --- | --- |
| `/execute_path` | action (server) | `qc_msgs/ExecutePath` | Play the planned trajectory to the arm, confirming each point via joint feedback. |
| `/arm/command` | topic (pub) | `std_msgs/Float64MultiArray` `[j1..j6, speed]` | Joint target to ArmDriver. |
| `/arm/joint_states` | topic (sub) | `sensor_msgs/JointState` | Actual joint feedback — used to confirm "reached". |
| `/movement/state` | topic (pub) | `std_msgs/String` (`idle\|moving\|reached\|error`) | Execution state (also surfaced via action feedback). |

**ArmDriver** (built — canonical names shown; see camelCase note above)
| Interface | Kind | Type | Meaning |
| --- | --- | --- | --- |
| `/arm/command` | topic (sub) | `std_msgs/Float64MultiArray` `[j1..jN, speed_pct?]` | Move to joint target. |
| `/arm/connect` | topic (sub) | `std_msgs/String` | IP to connect the real arm; empty = mock. |
| `/arm/joint_states` | topic (pub, ~20 Hz) | `sensor_msgs/JointState` | Live joint positions (rad). |
| `/arm/status` | topic (pub) | `std_msgs/String` | `idle\|moving\|drag\|off\|error:…` |
| `/arm/drag_button` | topic (pub) | `std_msgs/Bool` | End-effector capture button (teach mode). |
| `/arm/backend` | topic (pub) | `std_msgs/String` | Active backend (`mock`/`rokae ip`). |
| `/arm/set_power` | service | `std_srvs/SetBool` | Energise / de-energise motors. |
| `/arm/set_drag` | service | `std_srvs/SetBool` | Enter / leave hand-guide (drag) mode. |
| `/arm/stop` | service | `std_srvs/Trigger` | Soft stop. |
| `/arm/home` | service | `std_srvs/Trigger` | Move to home pose. |
| `/arm/clear_alarm` | service | `std_srvs/Trigger` | Clear servo alarm / released e-stop. |

**ScanningDriver** (interface only — hardware-blocked)
| Interface | Kind | Type | Meaning |
| --- | --- | --- | --- |
| `/scan/start` | service (server) | `std_srvs/Trigger` | Begin continuous capture. |
| `/scan/stop` | service (server) | `std_srvs/Trigger` | End capture; export the cloud. |
| `/scan/state` | topic (pub) | `qc_msgs/ScanState` | `idle\|scanning\|done\|error` + the exported cloud path. |

**InspectionNode / Phase 2** (interface only — algorithm-blocked)
| Interface | Kind | Type | Meaning |
| --- | --- | --- | --- |
| `/inspect` | action (server) | `qc_msgs/Inspect` | Run extract → clean → quality → register → deviation on a cloud. Result carries `quality_pass`, `rescan_requested`, and the deviation report. |

**RailDriver** (dormant — defined for completeness, not in the mission flow)
| Interface | Kind | Type | Meaning |
| --- | --- | --- | --- |
| `/rail/command` | topic (sub) | `std_msgs/Float64MultiArray` `[position_m, speed]` | Move to rail position. |
| `/rail/joint_states` | topic (pub) | `sensor_msgs/JointState` | Rail position (1 prismatic joint). |

**rosbridge** — exposes all of the above at `ws://localhost:9090`. The web app uses:
call `/mission/plan`, `/mission/execute`, `/mission/abort`; subscribe `/mission/state`,
`/arm/joint_states` (live telemetry), `/plan/scanpath` + `/plan/trajectory` (preview).

### 4.3 Mapping to the original vision names

`missionState → /mission/state` · `pathState →` PlanPath feedback + `/mission/state`
· `scanState → /scan/state` · `passFail → InspectionResult.quality_pass` ·
`setScanner → /scan/start,/scan/stop` · `CAD →` `part_id` (the mesh is loaded from
`config/cad/` by id, never put on the wire) · `armCMD/armPos → /arm/command,
/arm/joint_states`. **`nextWaypoint` is retired:** because PathPlanner plans the
*whole* trajectory (decision 13), the plan is handed to MovementDriver once via the
`ExecutePath` action, not streamed a waypoint at a time.

---

## 5. The settled decisions

1. **PathPlanner is the single source of the whole plan** (coverage + MoveIt motion). The web viewer only displays it.
2. **No rail** — fixed base; `rail_driver` dormant. *(Verify reach in sim: 919 mm vs a 1500 mm part + 300 mm standoff is tight.)*
3. **Continuous scanner capture** — ScanningDriver is start/stop/done.
4. **One automatic pass/fail = scan quality only.** The *part* verdict is a human call; Phase 2 informs, it does not decide.
5. **Part-frame → arm-frame = marked-corner calibration**, measured once, applied before MoveIt. *(Blocked on doing the calibration.)*
6. **rosbridge** is the web ↔ ROS 2 link (host has no ROS 2).
7. **One mission, no flip logic** — the operator flips the part and re-runs.
8. **The scan-quality gate lives inside Phase 2** (stage 3). `/mission/state = complete`… actually the Inspect action runs it; a fail sets `rescan_requested`, and TaskManager re-plans/re-executes up to **`max_rescans = 2`** (config), then flags for a human.
9. **Standoff 20–30 cm** (`standoff_mm`, default 250). Hand-eye calibration deferred (Phase-2 registration recovers scan↔CAD alignment by geometry).
10. **TaskManager owns mission abort** — `/mission/abort` cancels the active action (PlanPath/ExecutePath/Inspect), stops the arm (ArmDriver `/arm/stop`), stops capture, sets `phase = aborted`.
11. **The operator is the last safety switch** — `/mission/execute` (confirm) authorises motion; a separate physical E-stop is primary; attended operation assumed. `QC_ALLOW_MOTION=0` forces read-only.
12. **Part CAD mesh + a table box are MoveIt collision objects**, loaded by PathPlanner; part pose fixed by the marked corner.
13. **MoveIt runs in PathPlanner** — plan fully, then execute; MovementDriver only moves.

---

## 6. Frames, units, config

- **Part/CAD frame:** millimetres, Y-up (the STEP file's own frame).
- **Arm base frame:** metres, Z-up. PathPlanner converts part→arm (mm→m, Y-up→Z-up,
  **plus the marked-corner offset**) before MoveIt. The old `scanpath_convert`
  logic did the axis/unit half; the corner offset is the calibrated half (decision 5).
- **Tunables** live in `config/system_config.yaml` (read, never hard-code):
  `standoff_mm` (250), `max_incidence_deg` (25), FOV/overlap (FOV placeholder until
  the MIRACO spec is confirmed), quality-gate thresholds, `max_rescans` (2),
  motion speeds. Site-specific values (robot IP, the corner transform) go in a
  git-ignored `config/local_config.yaml`.

---

## 7. One mission, end to end

**Plan (no motion).** Operator selects DEX part → web calls `/mission/plan`
(part_id) → TaskManager calls PathPlanner `/plan_path` → PathPlanner: load CAD →
coverage waypoints → arm-frame transform → part+table into the MoveIt planning
scene → MoveIt plans (cartesian along scan lines, free-space between) → returns
`trajectory` + `scanpath`, and publishes `/plan/scanpath` + `/plan/trajectory`.
`/mission/state → planned`. The web viewer draws the path and, on **Preview**,
animates the arm through `/plan/trajectory`.

**Execute (operator confirm — decision 11).** Operator presses **Send to robot** →
web calls `/mission/execute` → TaskManager: `/scan/start` (ScanningDriver begins
continuous capture) → `/execute_path` (MovementDriver plays the trajectory to
ArmDriver, confirming each point via `/arm/joint_states`) → on completion,
`/scan/stop` → `/scan/state = done` with the cloud path. `/mission/state` walks
`executing → scanning`.

**Inspect (Phase 2).** TaskManager calls `/inspect` (cloud_path, part_id) →
InspectionNode runs extract → clean → **quality gate** → register → deviation. If
`quality_pass` is false → `rescan_requested`; TaskManager re-plans/re-executes (up
to `max_rescans`, then flags a human). If true → the deviation report is produced;
`/mission/state → complete`; the **human QC operator** reads the report and makes
the part verdict.

**Abort / failure (decision 10).** Any MoveIt-plan failure, arm fault, or
`/mission/abort` → TaskManager cancels the active action, stops the arm and
capture, sets `phase = aborted`/`error`, and surfaces it on `/mission/state`.

---

## 8. Repo layout the architecture implies

Everything lives in **this one repo**:

```
backend/            web console backend (HTTP + robot bridge + scan stub) — built
gui/                operator console front-end + scanpath-visualizer — built
libs/path_planning/ pure-Python coverage planner (CAD → waypoints) — recovered
scripts/            CLI entry points (plan_path.py, run_console.sh)
ros2_ws/src/
  sr5_arm_driver/   ArmDriver node + shared pure-Python backend — built
  rail_driver/      dormant
  dexory_teach_ros/ teach/jog app
  qc_msgs/          custom messages/services/actions (§4.1) — TO BUILD
  task_manager/ path_planner/ movement_driver/ scanner_driver/ inspection/ — TO BUILD
  rokae_ros2/       vendor clone (own git; re-cloned by docker/build.sh — not committed)
docker/             the Humble container that runs the ROS 2 graph
config/cad/         CAD files backing the part catalogue
```

**Launch convention:** each node is its own package with its own launch file; one
top-level `qc_mission.launch.py` brings up the whole graph (all six mission nodes +
the drivers + rosbridge) at once.
