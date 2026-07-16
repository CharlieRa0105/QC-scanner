# Code vs. Architecture — Audit

**What this file is:** a point-in-time comparison of the actual code against the
settled contract in [`architecture.md`](architecture.md). It records where the two
agree, where the code has moved ahead of the doc, and where the code is genuinely
behind the doc's intent. Read it alongside `architecture.md` (the of-record
contract) and `refactor-guide.md` (what to build next).

**Audit date:** 2026-07-15. Read-only; no code was changed producing this.
**Method:** every node/interface was checked file-by-file against §4 of
`architecture.md`. Findings below are verified against source, with file/line refs.

---

## Headline

**`architecture.md`'s status tags are stale — in the code's favour.** The doc was
written when only the ArmDriver + planner libs existed, and tags nearly everything
"to build". Since then the **entire ROS 2 mission graph has been scaffolded and
wired**: `qc_msgs`, TaskManager, PathPlanner, MovementDriver, ScanningDriver,
InspectionNode, `qc_moveit_config`, and `qc_bringup` all exist, launch from one
top-level file, and conform to the §4 interface contract almost exactly.

What is **not** done is the leaf-level real work (scanner capture, inspection
algorithms, free-space MoveIt planning, the physical calibration, the web↔ROS 2
link). Those are **honestly stubbed** (explicit `NotImplementedError` / "not
implemented" messages), not faked. So the gap is mostly *"doc says TO BUILD, code
says BUILT (skeleton) / partial (guts)."*

---

## Status: doc tag vs. reality

| Node / area | Doc tag | Reality |
| --- | --- | --- |
| `qc_msgs` | to build | **Built — 100% match to §4.1**, zero field differences |
| TaskManager | to build | **Built** — real state machine + rescan loop; abort partial |
| PathPlanner | partial | **Built + real MoveIt** — cartesian-along done; free-space-between is TODO |
| MovementDriver | to build | **Built** — execute-only, fully compliant |
| ScanningDriver | to build | **Built interface, stub capture** (no MIRACO SDK) |
| InspectionNode | to build | **Built interface, stub pipeline** (4 of 5 stages, all `NotImplementedError`) |
| ArmDriver | built | **Built** — already renamed to snake_case (doc still says camelCase) |
| `qc_moveit_config` | (untagged) | **Built** — real overhead-mount URDF + kinematics fix; depends on vendor SR5 config |
| rosbridge / bringup | to build | **Built** — one `qc_mission.launch.py` brings up the whole graph on `:9090` |
| Web app ROS wiring | to build | **Not built** — still direct xCore SDK over HTTP (as documented) |

---

## Contract conformance (§4) — where code matches the doc

- **`qc_msgs` is a perfect match.** Every message, service, and action field matches
  §4.1 exactly. `package.xml`/CMake deps are correct. Nothing missing, nothing extra.
- **All ROS interfaces match the contract**, by name/kind/type/direction:
  TaskManager 10/10, MovementDriver 4/4, PathPlanner 4/4 (+ the MoveIt service
  clients §4 calls for), ScanningDriver 3/3, InspectionNode 1/1. No renamed or
  missing interfaces, no rogue extras.
- **Latched QoS** (`TRANSIENT_LOCAL`, depth 1) is used correctly on the topics the
  doc marks latched: `/mission/state`, `/plan/scanpath`, `/plan/trajectory`, and
  `/scan/state`.
- **MovementDriver does no planning** (decision 13) — it plays back a pre-planned
  trajectory point-by-point, confirming each via `/arm/joint_states`. Compliant.
- **PathPlanner owns the whole plan** (decisions 1/12/13): real coverage-planner
  calls into `libs/path_planning`, real marked-corner transform applied per waypoint,
  real MoveIt collision objects (table box + part mesh) via `/apply_planning_scene`,
  real cartesian planning along each scan line via `/compute_cartesian_path`.
- **`frame_transform.py` math is real and correctly ordered** — mm→m, Y-up→Z-up, and
  the marked-corner offset composed as `R_total = R_corner @ R_axis`. `qc_config.py`
  loads + deep-merges system + local config as the doc describes.

---

## Where the doc is stale (fix the doc)

1. **§4 camelCase note is out of date.** The doc says the two built drivers "currently
   use camelCase (`/armCMD`, `/armPos`, `/railCMD`, `/railPos`)."
   - **ArmDriver is fully snake_case** — `/armCMD`/`/armPos` do not exist anywhere in
     `sr5_arm_driver/arm_driver_node.py`. The rename is done.
   - **RailDriver is only half-renamed**: `/railCMD` + `/railPos` remain camelCase;
     its status/backend/connect/services are already snake_case.
2. **Status tags** (table above) — most "to build" nodes are built skeletons; update
   PathPlanner from "partial" toward "built (planning done; free-space + chaining TODO)".
3. **`qc_mission.launch.py` docstring is stale** — it claims "today it starts:
   rosbridge" with everything else "to add", but the code below already launches the
   full graph.

---

## Real gaps (code behind the doc's intent — the actual TODOs)

Grouped by area, most structural first.

### Web app ↔ ROS 2 wiring absent (biggest divergence, as documented)
- Browser → HTTP/JSON → `http.server` → **direct xCore SDK** (`robot_bridge.py`).
  No rosbridge, no roslibjs, no `:9090` client anywhere — only three source
  *comments* pointing at the future path (`robot_bridge.py:80`, `server.py:65`,
  console line 820).
- Preview reads a **static `data/viewer_bundle.json`** (generated offline by
  `scripts/export_viewer_bundle.py` ← `scripts/scanpath_convert.py`), not live
  `/plan/*` topics. Arm-preview animation is unbuilt ("phase 2").
- Telemetry/state come from **HTTP polling** (`/api/robot/status|joints|follow_status`),
  not `/mission/state` + `/arm/joint_states` subscriptions.
- `POST /api/plan` shells out to `plan_path.py` → `scanpath_convert.py` →
  `export_viewer_bundle.py` as subprocesses — a demo-slice stand-in for the
  PathPlanner node.

### Inspection (Phase 2) — interface built, guts stubbed
- `libs/inspection/pipeline.py` has 4 stages (`clean → quality → register →
  deviation`), each raising `NotImplementedError`. The doc's **`extract` stage (§2)
  is missing entirely** — 4 of 5 stages present.
- No Open3D / TEASER++ (named in docstrings only).
- The `quality_pass → rescan_requested` decision (decisions 4/8) is **plumbed but
  never sets `True`** — the field always returns `False`.
- `libs/inspection` is wired in via lazy `sys.path` injection from `QC_REPO_ROOT`,
  not as a declared package dependency.

### ScanningDriver — never captures
- Honest stub: no MIRACO/Revopoint SDK, `/scan/stop` always returns `cloud_path=""`,
  never publishes the `error` state.

### PathPlanner — planning gaps
- **Free-space moves between raster lines not implemented** (TODO); per-line plans are
  just concatenated.
- `plan_line()` omits `req.start_state`, so per-line cartesian plans aren't
  kinematically chained (the config's own `cartesian_plan_test.py` *does* set it).
- A failed MoveIt plan degrades to `success=True` with an empty trajectory (masks
  failure).
- Frame-label inconsistency: `ScanPath.msg`/`frame_transform.py` say "arm base", but
  `moveit_planner.py` labels the same poses `table` (arm base is 1.2 m above `table`
  in the URDF). Confirm it's intentional relabelling.

### TaskManager — abort is cooperative, not active (decision 10)
- Sets an `_abort` flag + stops capture + sets `phase=aborted`, but does **not**
  cancel the in-flight action goal and does **not** call `/arm/stop` (deferred in a
  comment at line 194). Decision 10 wants active cancel + arm stop.

### Calibration — still identity (decision 5, blocked)
- `frame_transform.py` math is correct, but `corner_transform` in `local_config.yaml`
  is `[0,0,0]/[0,0,0]` marked `# TODO: MEASURE`. The physical calibration is unmeasured,
  so the part is currently positioned as if its corner sits on the arm base.

### Config — one missing block
- `config/system_config.yaml` is **missing the quality-gate thresholds** block (§6).
  All other tunables present (`standoff_mm 250`, `max_incidence_deg 25`,
  `max_rescans 2`, motion speeds; FOV is a self-flagged placeholder value).

---

## Minor / worth noting

- **`sr5_arm_driver` has no per-node launch file** — violates §8 ("each node its own
  launch file"); only startable via `qc_mission.launch.py`.
- **No startup ordering** in `qc_mission.launch.py` — everything launches flat;
  `path_planner` depends on `move_group` and mission nodes call each other's services,
  but nothing sequences them (usually settles via ROS discovery).
- **`.gitignore` blanket-ignores `config/`** — shared files survive only as
  force-tracked exceptions; a fresh `system_config.yaml` would be ignored by default.
- **`rail_driver` exceeds its "dormant" scope** — full mock + real (Roboteq serial)
  backend + service suite, where the doc treats rail as out of scope.
- **`/arm/command` speed unit** — doc calls the trailing value `speed_pct`; the backend
  interprets it as mm/s. Latent unit mismatch (topic shape matches).
- **`qc_moveit_config` is a functional overlay, not a standalone generated config** —
  it hard-depends on the vendor `rokae_xMateSR5_moveit_config` package for the SRDF,
  joint limits, and controllers.

---

## Bottom line

The architecture is being followed **faithfully at the contract level** — interfaces,
message types, node responsibilities, and the plan-fully-then-execute split all match.
The divergences are (a) the doc's status tags + camelCase note lagging the code, and
(b) the expected unfinished leaf work: scanner/inspection algorithms, free-space
planning, the calibration value, and the whole web↔rosbridge layer.
