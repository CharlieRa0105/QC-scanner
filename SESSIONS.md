# Sessions — change log

A running log of the changes made to the QC Scanner, newest session first. Each
entry says **what** changed and **why**, with file references, so anyone (or a
fresh Claude session) can see how the code got to its current state.

> The canonical decision log / session history lives in Ra's vault
> (`~/Documents/ClaudeVault/…`, not in this repo). This file is a lightweight
> in-repo summary of concrete code changes, kept because it's useful on the
> machine where the work happens.

---

## Current state — resume here (as of 2026-07-16, on top of `2eb3499`)

**Real-arm Cartesian motion works** ("Go to point"). The whole fix + the working
recipe + the error-code cheat sheet are in `docs/session-logs/2026-07-16.md` —
read that first. TL;DR of what makes Cartesian moves go on the SR5:
- Read/command in **`endInRef`** (not `flangeInBase`); operator types
  controller-frame mm matching the readout, no table↔base conversion for go-to.
- **Pre-flight** every target with `model().calcIk` + `checkPath` (NO motion);
  reject singular/unreachable before dispatch.
- `setDefaultConfOpt(False)` + copy current `confData` as a SOFT bias (never force
  `True` → accept-but-don't-move). Confirm motion via `operationState`.
- MoveJ = point-to-point; MoveL = straight line (stricter, needs clean path).
- **Path tracing now works end to end.** `follow_path(position_only=True)` traces
  waypoint POSITIONS via the go-to machinery (orientation searched per point +
  MoveJ fallback). Wired into all send paths (debug viewport, "Send to robot",
  "Confirm & start scan"). The frame gap is closed: at connect the backend
  self-calibrates **flangeInBase→endInRef** (reads current pose in both frames),
  and `follow_path` does table→flangeInBase→endInRef before commanding — before
  this, positive-z flangeInBase targets were rejected and "nothing moved".
- **CAD scan = overhead X-raster.** `scan_trace` builds an X-raster grid over the
  part footprint (part centred at the FIXED origin 0,0,0 = arm base — no drift),
  scanner straight DOWN from 250 mm, each row a continuous blended MoveL sweep,
  unreachable points skipped. Debug viewport: **90° flip buttons → Generate path
  (preview+reachability) → Scan → arm**; camera-follow always on. Endpoints
  `/api/robot/{scan_trace,scan_preview}`. Tunable: `QC_RASTER_STEP_M`,
  `QC_STANDOFF_M`. Full detail: `docs/session-logs/2026-07-16.md` "Continued (2)".
- **Open (next):** surface-normal aiming (currently straight-down for a clean
  raster — add as a toggle); real part registration (part assumed at origin;
  jog-to-corner needed for parts elsewhere); confirm the raster lands on the
  physical part; viewport polish.
- MoveL singularity avoidance (`setAvoidSingularity`) is opt-in via
  `QC_ROBOT_CLASS=xMateRobot QC_AVOID_SINGULARITY=1` (off by default).

**Everything below is UNCOMMITTED working-tree changes.** Nothing has been
committed or pushed (that's Ra's call). `git status` shows the full surface:
deleted `backend/sr5_arm_driver/` + `gui/scanpath-visualizer/`; new
`libs/{qc_config.py,path_planning/frame_transform.py,inspection/}`,
`gui/viewer/`, `config/*.yaml`, and the ROS packages
`ros2_ws/src/{qc_msgs,qc_bringup,qc_moveit_config,path_planner,movement_driver,task_manager,scanner_driver,inspection}`;
plus the vendor `ros2_ws/src/rokae_ros2/` clone (git-ignored).

**What works (verified):**
- Refactor queue T1–T14 complete (Part 2 ROS graph = build-to-interface; all
  packages build in `qc-humble`, interfaces register).
- MoveIt for the overhead SR5: IK + collision-checking + cartesian planning all
  work (`qc_moveit_config`). The earlier "IK broken" was a config-structure bug,
  since fixed.
- Console: real-arm only + honest disconnect + Connect button; the new light 3D
  viewer + the Debug popup (shape outlines, gizmo, trace preview, camera-follow),
  "Go to point", and the raw controller-TCP readout — all verified on the MOCK
  via playwright (installed in `.venv312`).
- Frame fix: table-frame poses are converted to the controller's base frame for
  the real arm (`_table_to_arm_base` in `robot_bridge.py`).

**Open / next:**
1. **T9.3 [Ra checkpoint]:** validate `move_pose` on the real arm at low speed via
   "Go to point" (commanded vs the controller-TCP readout — same frame). Launch:
   `QC_ALLOW_SCAN_TRACE=1 QC_JOG_SPEED=30 scripts/run_console.sh`.
2. **Cell geometry:** homed flange reads (0,135,1259) mm from the base, but the
   mount config says 1200 mm above the table → the true base-to-table height is
   > 1259 mm. Need Ra's real measurement to set `workspace.mount.base_xyz_mm[2]`
   so the viewer's table/part/path geometry matches (does NOT affect go-to).
3. **Reach-aware planning:** the demo scan path had out-of-reach waypoints; once
   the cell geometry is fixed, add reachability filtering in the planner.
4. **Still stubbed:** inspection stages (Open3D/TEASER++), scanner hardware, the
   console↔rosbridge wiring for the real mission graph.

**Gates / env:** `QC_ALLOW_MOTION` (default on) · `QC_ALLOW_SCAN_TRACE` (default
OFF — cartesian trace/shape-trace/camera-follow/go-to on the real arm) ·
`QC_ARM_MOCK=1` (TEST-ONLY simulated arm). Reachability is the controller's IK
call, not a software sphere (a wrong sphere check was removed this session).

**Verification note:** playwright test scripts were written under the ephemeral
job tmp dir (not committed); the kept ROS checks are
`ros2_ws/src/qc_moveit_config/test/{collision_check_test,cartesian_plan_test}.py`.

---

## 2026-07-22 — Cell geometry + motion safety (full log: `docs/session-logs/2026-07-22.md`)

- **Table height** set to **1000 mm** below the arm base (`config/local_config.yaml`
  `workspace` override). Reach check: parts on the table are below the scanning
  sweet-spot (~300–700 mm up) → need a raised fixture (design decision, open).
- **Sim part centred** on the table: `export_viewer_bundle.py` now shifts X,Y (not
  just Z) so the footprint sits under the base; mesh + path + dome/box move together.
- **Home pose** defined in **joint space** — `HOME_JOINTS_DEG =
  [-71.38, 93.33, 127.11, 157.33, -61.23, -77.06]` (TCP −88.1, 21.4, −553.5 mm /
  87.2, −0.7, 1.2 deg). New `go_home()` (MoveJ + settle) runs at **scan start and
  end** (skipped after abort/E-stop). **Home button** fixed — was hardcoded to
  all-zeros; now `POST /api/robot/home` → same `HOME_JOINTS_DEG`.
- **Hardcoded J2 rail guard**: `move_joints` refuses any J2 target outside
  **[−120°, 115°]** (arm was swinging J2 into its mounting rail); UI J2 max 120→115.
  Guards joint-space commands only — Cartesian scans still route J2 via the controller.
- **Jog-to-targets fixed**: replaced the fragile 4 s hold timer with a per-joint
  `edited` hold; then split into **two columns** (live position vs commanded target).
- **ROS 2 / MoveIt — BUILT + proven on the mock** (Docker `qc-humble`; real arm never
  commanded): MoveIt models the **gantry** (plate + posts) and refuses to hit it;
  PathPlanner produces a **collision-free scan trajectory (fraction 1.00, ~1196
  joint points)** after reachable placement + IK-seeded cartesian; MovementDriver
  executes on the mock, TaskManager orchestrates `/mission/plan`; console previews
  `/plan/trajectory` over rosbridge; **collision-checked jog** refuses self/gantry
  jog targets. IK = KDL. Fixed en route: gmsh off-thread init, MovementDriver
  power-on, `.gitignore` swallowing the MoveIt config.
- **Open:** browser-verify the console preview/jog/gantry; free-space (OMPL) bridges
  between scan lines (coded, failing 0/8); placement is a placeholder for the
  measured corner transform + fixture; TracIK deferred. Full detail + resume anchor:
  `docs/session-logs/2026-07-22.md`.

---

## 2026-07-15 — Refactor Part 1 (demo slice) + console fixes

Working through `REFACTOR_INSTRUCTIONS.md`, one task per checkpoint. **Nothing
committed yet** — all changes below are uncommitted working-tree edits on top of
`73ec017`.

### Cleanup

- **T1 — One arm backend.** Removed the duplicate `RokaeArm`/`MockArm` backend;
  the single home is now the ROS 2 package. The console borrows it via `sys.path`.
  - `backend/sr5_arm_driver/{backends.py,__init__.py}` — **deleted** (was a byte
    identical copy).
  - `backend/robot_bridge.py` — import repointed to
    `ros2_ws/src/sr5_arm_driver/`; docstring/comments updated.
  - `backend/README.md` — stale backend-path references fixed.

- **T2 — Verify recovered pieces.** Confirmed `docker/build.sh` builds `qc-humble`
  and `scripts/plan_path.py` runs on the sample STEP. Found + fixed environment
  gap: `gmsh` needs `libGLU.so.1` (installed `libglu1-mesa`). No code change.

### Part 1 — get a scan working (demo slice)

- **T3 — Face-grouped raster planner.** Rewrote `generate_raster_waypoints` to
  cluster surface samples by normal (angular clustering) *before* rastering, and
  raster each face in its own plane. Fixes the collapse-to-one-line / mixed-face
  bug (DEX05120563: 5 wp/1 line → 128 wp/28 lines, one face per line).
  - `libs/path_planning/waypoint_generator.py` — new `_group_by_normal`,
    `_inplane_axes`; `step_axis`/`travel_axis` params replaced by
    `face_angle_tol_deg`.
  - `scripts/plan_path.py` — CLI flags updated (`--face-angle-tol-deg`).

- **T4 — Frame transform to config.** Part-frame (mm, Y-up) → arm-frame
  (m, Z-up + marked-corner offset). Config-driven; the corner calibration is a
  measured value (still the **identity placeholder** — decision 5, not yet
  measured).
  - `libs/path_planning/frame_transform.py` — **new**, pure-numpy transform.
  - `libs/qc_config.py` — **new**, loads + merges the YAML config.
  - `scripts/scanpath_convert.py` — **new** CLI: part-frame JSON → arm-frame JSON.
  - `config/system_config.yaml` — **new** (shared tunables, tracked).
  - `config/local_config.example.yaml` — **new** template (+ `config/local_config.yaml`,
    git-ignored, holds the corner transform).
  - Installed **PyYAML** into `.venv312` (not yet captured in `setup.sh`).

- **T5 — Pose → arm motion.** SDK finding: no standalone IK — cartesian `MoveL`
  resolves the pose to joints internally. Added a `move_pose`/`get_pose`
  primitive to both backends and a `follow_path` (sequential poses, settle
  between, abortable) to the console bridge. Proven end-to-end against the mock.
  - `ros2_ws/src/sr5_arm_driver/sr5_arm_driver/backends.py` — `move_pose`,
    `get_pose`, `_quat_to_rpy`/`_rpy_to_quat`; MockArm gets a synthetic-TCP sim.
  - `backend/robot_bridge.py` — `follow_path` + abort.
  - ⚠️ `RokaeArm.move_pose` units/rpy are **unvalidated on hardware**.

- **T6 — "Send to robot" (demo).** Console button streams the arm-frame path to
  the arm, pose by pose, with live progress. Gated on connection +
  `QC_ALLOW_MOTION` + operator confirm.
  - `backend/robot_bridge.py` — `start_follow_path` (worker thread) +
    `follow_status`.
  - `backend/server.py` — `POST /api/robot/follow_path`, `GET /api/robot/follow_status`;
    path source is `data/scanpath_arm.json` (planning stays out of the backend,
    per refactor-guide §2.4).
  - `data/scanpath_arm.json` — **new**, pre-generated demo path (128 wp, arm frame).
  - `gui/Scan Cell Console.dc.html` — "Scan path" card (Send to robot / Stop /
    progress bar).

### Console fixes (raised during T6 testing)

- **Removed the mock from the console.** The console now connects only to the
  real SR5 (`MockArm` kept in the shared backend for the ROS `ArmDriver`).
  `backend/robot_bridge.py`.
- **Honest disconnect.** `robot_bridge.status()` pings the arm each poll and drops
  to Offline after 2 misses (fixes "unplugged but still Connected").
- **Connect / Disconnect buttons** added to the console header.
  `gui/Scan Cell Console.dc.html`.
- **No-cache dev server.** `server.py` sends `Cache-Control: no-store` so browser
  caching stops masking live GUI edits.
- **Shared motion speed.** One `motionSpeed` (mm/s) field drives all Debug motion
  — jog, home, and the scan trace. `gui/Scan Cell Console.dc.html`.
- **Safety gate + incident.** A verification call accidentally ran `follow_path`
  on the **live** arm (moved 35/128 waypoints before abort). Arm was halted +
  de-energised. Added `QC_ALLOW_SCAN_TRACE` (default off): the unvalidated
  cartesian trace is now **disabled on the real arm** unless explicitly enabled.
  `backend/robot_bridge.py`.

### Part 2 — ROS 2 graph (in progress)

Built to `docs/architecture.md` §4 in **build-to-interface** mode (Ra's call):
each node's package builds in `qc-humble` and registers its §4 interfaces; the
existing pure-Python logic is wired in; MoveIt planning, real execution, and
scanner capture are structured but marked TODO (need the SR5 moveit_config /
hardware) rather than faked. All 8 packages build; all interfaces verified via
`ros2 action/service/topic list` with the nodes running.

- **T7 — `qc_msgs`.** Custom interfaces §4.1: msgs (MissionState, ScanWaypoint,
  ScanPath, ScanState, InspectionResult), srv (StartMission), actions (PlanPath,
  ExecutePath, Inspect). `ros2_ws/src/qc_msgs/`.
- **T8 — rosbridge.** Added `ros-humble-rosbridge-suite` to `docker/Dockerfile`;
  new `qc_bringup` package with `rosbridge.launch.py` (binds ws://:9090) and the
  top-level `qc_mission.launch.py` (rosbridge + all mission nodes).
- **T9 — PathPlanner.** `ros2_ws/src/path_planner/` — /plan_path action;
  coverage (libs/path_planning) + frame transform → qc_msgs/ScanPath on
  /plan/scanpath; MoveIt trajectory on /plan/trajectory = TODO (graceful-degrade
  to empty trajectory). Planner libs imported lazily from QC_REPO_ROOT.
- **T10 — MovementDriver.** `ros2_ws/src/movement_driver/` — /execute_path action;
  plays trajectory to /arm/command, confirms each point via /arm/joint_states,
  publishes /movement/state. Execution only.
- **T11 — TaskManager.** `ros2_ws/src/task_manager/` — /mission/{plan,execute,abort}
  services, /mission/state (latched), action clients to plan/execute/inspect +
  scan start/stop; rescan loop (`max_rescans` param). Orchestrates the mission.
- **T12 — ArmDriver rename.** `/armPos`→`/arm/joint_states`, `/armCMD`→`/arm/command`
  in `arm_driver_node.py` (matches MovementDriver + §4.2).
- **T13 — ScanningDriver (interface only).** `ros2_ws/src/scanner_driver/` —
  /scan/{start,stop} + /scan/state. Hardware-blocked: honest state transitions,
  empty cloud_path, never fabricates a capture.
- **T14 — InspectionNode.** `ros2_ws/src/inspection/` + `libs/inspection/` — /inspect
  action wrapping the pure-Python pipeline (clean→quality→register→deviation per
  docs/point_cloud_processing.md). Stages are documented skeletons that return an
  honest "not computed" result (NaN metrics), never a fake pass.

**Part 2 runtime gaps (not yet runnable end-to-end):** SR5 moveit_config (MoveIt
planning), the real arm, and the scanner hardware/SDK; plus the planner's runtime
deps (gmsh/numpy/PyYAML) + a repo mount inside the mission container. The graph
*structure* is complete and interface-verified.

### Console Run-flow: preview + arm wiring

Two reported gaps: no 3D preview after loading a part, and the Run flow never
moved the arm.

- **Preview (added then REMOVED).** Briefly embedded the standalone
  `gui/scanpath-visualizer/` as an iframe viewport. Then Ra had the whole
  visualiser **deleted** (`gui/scanpath-visualizer/`, `/api/scanpath`,
  `data/scanpath.json`, the iframe viewport, the layout change all reverted) — it
  is being **remade from scratch**. The rendered path was also flagged messy
  (self-clipping, dipping under the table) → planner-quality issues to fix for the
  remake: (a) inconsistent surface-normal orientation pushing some standoff poses
  *into/under* the part; (b) over-fragmented face groups (28 lines for the small
  pin: the cylinder splits into many ~30° bands) with un-ordered group traversal.
- **Arm wiring.** The Run flow's "Confirm & start scan" (`sendToRobot`) called the
  scan stub only. Rewired it to SEND TO ROBOT: operator confirm →
  `/api/robot/follow_path` (arm traces the path) → progress drives the scanning
  ring → record the scan → Result.
- **Still gated:** on the real arm the trace needs `QC_ALLOW_SCAN_TRACE=1`
  (unvalidated `move_pose`), so "Send to robot" is refused with a clear message
  until that's deliberately enabled. Browser click-through not run here (no
  browser); plumbing verified over HTTP.

### New 3D scan-path viewer (phase 1)

Built from scratch (no code from the deleted scanpath-visualizer), themed to the
console (charcoal + lime). `gui/viewer/`:
- vendored fresh three.js r128 + OrbitControls + STLLoader (`vendor/`); SR5 link
  meshes copied from `rokae_description` (`assets/arm/`).
- `scripts/export_viewer_bundle.py` → `data/viewer_bundle.json`: the PART mesh +
  the SCAN PATH in the `table` frame (m), via the project's own planner + frame
  transform. Served at `GET /api/viewer_bundle`.
- `index.html` + `viewer.js`: Z-up scene, orbit navigation, part mesh, path
  polyline + waypoint dots + aim rays, a scanner gizmo, **play/scrub**, a
  **Scanner-POV** camera toggle, and layer toggles. Embedded in the Run tab
  viewport (iframe). Confirmed: bundle + all assets serve; part sits correctly on
  the table (z≥0), so the old "under the table" was the old viewer's seating bug.
- **Phase 2 (next):** animate the ROBOT ARM through the real `/plan/trajectory`
  (now possible since MoveIt IK works) for the collision-preview playback.
- **Fixes (Ra feedback):** viewer set to **light mode** (bg/panels/grid/path
  colours) to match the console; viewport was tiny/in-corner → per-frame
  `maybeResize()` so the canvas fills the iframe even when laid out late.

### Debug viewport + shape tracing + command path (Ra's task list T0–T9)

All verified against the MOCK backend (API ladder + two playwright suites, all
green); real-arm enablement is the explicit Ra-driven checkpoint (T9.3).

- **T0 config:** `workspace:` block in `config/system_config.yaml` (2000×750×1200 mm,
  arm base top-centre upside-down at (1000,375,1200), table Z=0 — reconciled with
  `qc_cell.urdf.xacro`) + `debug_shapes:` defaults. Served at `GET /api/config`.
- **T1 box:** translucent workspace box + overhead arm rendered in BOTH viewports
  from the config (no hard-coded dims).
- **T2 refactor + popup:** `gui/viewer/viewer3d.js` = ONE reusable scene module
  (cell/box/part/path/arm FK + preview-only CCD IK/playback/dispose) consumed by
  the main embed (`viewer.js`) AND the new debug popup (`debug.html`/`debug.js`,
  opened from a "Debugging" card in the Debug tab as a modal iframe; closing
  releases the WebGL context — 5× open/close verified clean). Shared theme in
  `viewer.css`.
- **T3 shapes:** circle/square/triangle outline primitives, vendored
  TransformControls gizmo (move/rotate), live pose readout in ARM-BASE frame
  mm/deg, chips select, delete. Defaults from config.
- **T4 trace:** outline → ordered standoff waypoints (sub-table + out-of-box poses
  DROPPED and counted) → preview always plays (IK-posed arm) + the same gated
  backend path as scan tracing (`/api/robot/follow_path` with inline waypoints).
  Mock traced 5/5.
- **T5 camera-follow (D2):** end-effector re-aims at the orbit camera's look-at
  point (≤20 Hz preview; verified 0.2–0.4° aim error), optional send-to-arm ≤10 Hz
  with dedup (3 reqs in 2 s — no flooding), auto-disables on gate refusal; a trace
  suspends follow and it resumes (D2).
- **T6 planner:** `generate_raster_waypoints` now DROPS waypoints below the table
  (up-axis < table height) and logs the count — never silently kept. Unit-checked.
- **T7:** "Recorded scans" history panel removed from Analytics (store +
  `/api/scans` intact).
- **T8:** wizard Back buttons (Place→Load, Identify→Place); state preserved
  round-trip (verified).
- **T9.1/9.2:** full command path verified on mock (connect/power/jog/home/
  follow_path/move_pose — a real bug found+fixed: `joints()` polled the backend
  with dt=0 so the mock never advanced; now passes real elapsed time). Gates:
  `QC_ALLOW_MOTION` + `QC_ALLOW_SCAN_TRACE` shared by scan-trace, shape-trace and
  camera-follow; default OFF on the real arm (in-process gate test + live refusal
  on :8000 confirmed). New: `POST /api/robot/move_pose`, `GET /api/config`;
  `QC_ARM_MOCK=1` re-added as an explicit TEST-ONLY opt-in.
- **T9.3 [HUMAN CHECKPOINT]** — first live run FAILED as designed to be caught:
  the physical arm didn't match the sim. **Root cause found by inspection: poses
  were commanded in the TABLE frame but the controller interprets its own BASE
  frame** (overhead at 1.2 m, rolled 180°) — nothing converted, so real motion
  was ~1.2 m off in Z and Y-mirrored vs the preview. Fixed in
  `backend/robot_bridge.py`: `_table_to_arm_base()` (p→(x,−y,H−z), q→flip⊗q,
  H from the workspace config) applied at BOTH real-arm command sites
  (`follow_path`, `move_pose_once`); the mock still simulates in the table frame.
  Verified numerically (landmarks + 200 random quaternions vs a rotation-matrix
  reference + mock passthrough). REMAINING RISK for the re-test: the SDK's rpy
  convention in `_quat_to_rpy` (XYZ intrinsic) is still unvalidated — if position
  now tracks but orientation is twisted, that's the next (last) suspect. Re-run
  the low-speed ladder before trusting.

### "Out of range" investigation — format confirmed, REACH was the real problem

- **Wire format verified against the SDK's own docs:** `CartesianPosition.trans`
  = metres, `.rpy` = radians `[Rx,Ry,Rz]` — exactly what we send. No unit bug.
- **The real issue is geometry:** SR5 reach 919 mm, base 1200 mm above the table
  → the arm can NEVER touch the table (bubble bottoms out ~281 mm above it).
  Audit: **16/21 waypoints of the DEX scan path are beyond reach** (up to
  1242 mm). The debug circle at defaults (364 mm) is comfortably reachable.
  (architecture.md decision 2 warned exactly this: "verify reach … is tight".)
- **New reach guard** (`backend/robot_bridge.py` + `workspace.arm_reach_mm: 919`
  in config): every cartesian entry point (`start_follow_path`, `follow_path`,
  `move_pose_once`) now computes each pose's arm-frame distance and REFUSES
  up front with a count ("16/21 beyond 919 mm, worst 1242 mm, first #0") instead
  of letting the controller fault mid-motion. Verified in-process: unreachable
  path refused, reachable circle accepted + mock-traced 9/9.
- **OPEN CELL-DESIGN QUESTION for Ra:** with mount 1.2 m + standoff 250–300 mm,
  scanning a table-seated part is at/beyond the reach limit. Options: lower the
  mount, raise the part (pedestal), shorten the standoff, or accept top-face-only
  coverage. The planner should also gain a reach-aware filter once decided.

### Reach check REMOVED (was wrong) + go-to frame aligned to the controller

Ra's ground truth: homed flange (all joints 0) reads (0, 135, 1259.4) mm in the
controller base frame. My geometric reach check ("distance from base origin ≤
919 mm") was therefore WRONG — 919 mm is the envelope radius about the shoulder,
and the base structure adds length, so the homed flange is already ~1259 mm from
the origin. The check falsely refused valid points. Removed it from all three
cartesian paths (`start_follow_path`, `follow_path`, `move_pose_once`); the
`_reach_check` helper + `ARM_REACH` are gone. Reachability is now the
CONTROLLER's IK call (it errors on genuinely-unreachable poses) — the authority,
not a sphere. `arm_reach_mm` kept in config as informational only.

Also aligned "Go to point" to the controller base frame: the operator now types
coords in the SAME frame the readout strip shows (verified the arm↔table
conversions cancel, so typed (0,135,1259.4) is commanded exactly as
(0,135,1259.4) mm base-frame). Default moved to a near-home reachable point.

### "Go to point" (Debug tab)

New card in the console's Debug tab: type an XYZ target in the ARM-BASE frame
(mm), press Go — translation only (tool aims straight down). Converts to the
table frame (mount height from /api/config) and POSTs `/api/robot/move_pose`,
so it inherits the full gate chain (connection + confirm + QC_ALLOW_MOTION +
QC_ALLOW_SCAN_TRACE on real) AND the reach guard (out-of-reach points refused
with the measured distance, e.g. "pose is 1297 mm from the base — beyond 919 mm").
Verified on the mock in-browser: conversion (arm [100,−50,400] mm → table
[0.1, 0.05, 0.8] m), request sent, action logged, refusal path exercised.

### "Robot head — controller-reported" strip (Debug tab)

Live readout of the arm's OWN pose report, raw from the SDK (`cartPosture` with
`CoordinateType.flangeInBase`) — deliberately NOT our frame math, so it serves
as ground truth to compare against the software. Shown as XYZ mm + RPY deg in
the Debug tab, updating with the 1 Hz joint poll. Along the way this fixed a
latent bug: the old `RokaeArm.get_pose()` called `cartPosture` WITHOUT the
required CoordinateType arg, so it always silently returned zeros. Mock provides
the equivalent (`get_pose_raw` = sim TCP). Verified in-browser on the mock: strip
renders, payload carries the raw pose, readout live-tracked a go-to move
(0 → 480 → 780 mm toward the 800 mm target). Mock-only artifact noted: with the
1 Hz debug poll the mock sim advances at ~half real-time (dt clamp); the real
arm's report is unaffected by poll rate.

### Viewer round 2 (Ra feedback) — all verified in-browser (playwright)

- **SR5 arm added to the viewport** from the URDF meshes: extracted the kinematic
  chain (`gui/viewer/assets/arm/chain.json`), load the 7 STLs + build FK, mount it
  overhead (base 1.2 m, roll π). "Robot arm" layer toggle. Verified: 7 meshes +
  chain load, no errors.
- **Default part removed:** the console no longer auto-selects the DEX bearing —
  operator must pick/upload (removed all `parts[0]`/`PARTS[0]` fallbacks).
- **Import/pick now regenerates the path:** `chooseCad`/`onFile` → `POST /api/plan`
  (backend subprocesses plan_path.py → scanpath_convert.py → export_viewer_bundle.py;
  no planner code in the stdlib backend) → viewer iframe reloads via a nonce.
  Upload saves the STEP to config/cad via `POST /api/parts/upload`. Verified:
  picking a part fires /api/plan and reloads the viewer.
- **Send-to-robot still gated** by `QC_ALLOW_SCAN_TRACE` (unvalidated cartesian
  MoveL). To trace on the real arm: launch with `QC_ALLOW_SCAN_TRACE=1` + low
  speed + E-stop. (The proper path is the MoveIt trajectory now that IK works —
  Phase 2 / ROS wiring.)

### Viewer + arm-control: verified with a real headless browser (playwright)

Installed playwright + chromium and actually drove the console (instead of
asserting). Findings:
- **Viewport was tiny (canvas 300×150) despite the iframe being 1040×790:**
  `position:fixed;inset:0` does NOT stretch a `<canvas>`, so it kept its
  intrinsic size. Fixed: `maybeResize()` now sizes to `window.innerWidth/Height`
  with `setSize(w,h)` (updateStyle on) + `#canvas{width:100vw;height:100vh}`.
  Re-verified in-browser: canvas now 1040×790 (buffer + CSS).
- **Jog/Home DO fire + log:** the confirm dialog appears, `POST /api/robot/move`
  is sent, and the log stream shows the result (proven with `QC_ALLOW_MOTION=0`:
  "Jog → […]° refused — motion disabled"). So the handlers/logging work. Two real
  gotchas, not bugs: (a) **Jog targets the *current* joint position** (sliders
  track live), so with nothing edited it's a no-op — edit a slider first; (b) the
  **log stream is Debug-tab only**, so pressing **Home** (Run dock) shows no
  visible feedback there.
- Also added `RokaeArm._prep()` best-effort `disableDrag()` (safety: a move exits
  drag first) — good hygiene, but it was NOT the cause of the "no log".
- NOT verified (won't command the live arm): whether the physical arm actually
  moves on Home with motion enabled — plumbing is proven up to the SDK call.

### MoveIt integration (overhead SR5)

- Cloned vendor `rokae_ros2` (SR5 URDF + collision meshes + `rokae_xMateSR5_moveit_config`).
- **`qc_moveit_config`** (new): `config/qc_cell.urdf.xacro` mounts the SR5 OVERHEAD
  (base 1.2 m above the `table` root, rpy π so it points down) by wrapping the
  vendor description; `launch/qc_move_group.launch.py` runs a planning-only
  move_group (reuses vendor SRDF/OMPL, our kinematics.yaml). Robot name set to
  `xMate_robot` so the SRDF binds. Added to `qc_mission.launch.py`.
- **PathPlanner ↔ MoveIt** (`path_planner/moveit_planner.py`): loads the table box
  + part CAD mesh as collision objects (`/apply_planning_scene`) and plans each
  raster line via `/compute_cartesian_path` (`avoid_collisions=True`). Builds;
  node runs under a MultiThreadedExecutor for the service calls.
- **VERIFIED — collision avoidance (the explicit ask):** headless
  `check_state_validity` proves MoveIt flags (a) arm-vs-obstacle (part) and
  (b) arm-vs-self collisions; valid again when cleared. See
  `qc_moveit_config/test/collision_check_test.py`.
- **IK + cartesian planning now WORK (resolved).** Earlier I wrongly blamed a
  vendor/KDL defect; the real cause was a config bug in
  `qc_moveit_config/config/kinematics.yaml` — it used the `/**: ros__parameters:`
  wrapper, but `MoveItConfigsBuilder.robot_description_kinematics()` wants the
  bare `rokae_arm:` mapping, so the solver never loaded (`No kinematics plugins
  defined`). The "vendor-stock also fails" test was misleading (same wrapper).
  Fixed the structure (no TRAC-IK, no rebuild): `/compute_ik` returns SUCCESS,
  and `/compute_cartesian_path` gives fraction 1.0 in free space and 0.0 through
  an obstacle. See `qc_moveit_config/test/cartesian_plan_test.py`.
