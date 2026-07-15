# QC Scanner — Refactor & build guide

Read this after `../CLAUDE.md` and `architecture.md`. It is the **actionable
picture**: what the repo is *right now*, what to clean up, and what to build next —
in priority order — to reach the target architecture. Written 2026-07-15 after a
"big changes" cleanup that stripped the repo down to the operator console.

> House rule (Ra's): **every file you write or change must be self-explanatory** —
> a plain-language header saying what it does and how it fits, plus comments on
> anything non-obvious. Match the style already in `libs/path_planning/*.py`. A
> file without that is not done. Work in **small, reviewable steps** (one coherent
> change at a time). Ra drives, you guide.

---

## 1. Where the repo is right now (accurate as of 2026-07-15)

The repo is currently **the operator console only**. A cleanup commit removed a lot
of older/half-built material; some of it is coming back properly, some is gone for
good.

**Built and working (console):**
- `backend/server.py` — stdlib HTTP server. Serves `gui/` + a JSON API:
  `/api/parts`, `/api/robot/{status,joints,connect,disconnect,power,drag,stop,estop,clear_alarm,move}`,
  `/api/scan/{start,stop}`, `/api/scans`. **No `/api/plan`** — path planning was
  removed from the backend in the cleanup (it comes back inside PathPlanner, see §3).
- `backend/robot_bridge.py` — owns the **single** xCore SDK session; live telemetry
  + motion, gated on connection **and** `QC_ALLOW_MOTION`.
- `backend/sr5_arm_driver/backends.py` — the pure-Python arm backend
  (`RokaeArm` / `MockArm`) the bridge uses.
- `backend/scan_pipeline.py` — honest stub scan lifecycle + JSON results store
  (records `incomplete`; no capture/QC yet). Does **not** command motion.
- `gui/` — the dc-runtime console (Run / Analytics / Debug tabs). The in-app 3D
  scan-path visualiser was removed in the cleanup.

**Just recovered (was wrongly deleted — restored from git `ff8de15`):**
- `libs/path_planning/` — the pure-Python coverage planner: `cad_loader.py`
  (STEP→mesh via gmsh), `normal_estimation.py` (area-weighted surface sampling +
  outward normals), `waypoint_generator.py` (raster planner — **has a known design
  flaw, see §2.3**), `incidence_cone_modifier.py` (orientation smoothing + SLERP
  clamp into the incidence cone).
- `scripts/plan_path.py` — CLI: CAD file → ScanPath JSON. Chains the four modules.

**On disk but untracked (not committed here):**
- `ros2_ws/src/` — `sr5_arm_driver` (the ROS 2 ArmDriver node), `rail_driver`
  (dormant), and `rokae_ros2` (the **vendor clone — has its own `.git`**; treat as
  a dependency to re-clone, don't commit it). These need bringing back into the
  build story (see §3).

**Gone (intentionally):** the old `docs/` (superseded by this set), `HANDOVER.md`,
the `gui/scanpath-visualizer/` three.js app, `docker/` (the Humble container — will
be rebuilt when the ROS 2 graph is stood up).

---

## 2. Cleanup / refactor tasks

### 2.1 One arm backend, not two
The arm backend (`RokaeArm`/`MockArm`) now exists at `backend/sr5_arm_driver/` **and**
(untracked) at `ros2_ws/src/sr5_arm_driver/sr5_arm_driver/`. That's two copies of
the one thing. Decide a **single shared home** and have both the console
(`robot_bridge.py`) and the ROS 2 `ArmDriver` node import it from there. Do not let
them drift.

### 2.2 The two rival paths to the arm
The console drives the arm **directly** via `robot_bridge.py` (SDK). The ROS 2
`ArmDriver` node wants the **same single SDK session**. They cannot both hold it at
once. Target (per architecture decision 13/6): ROS 2 owns motion during a mission;
the console talks to the graph over rosbridge; `robot_bridge`'s direct control
becomes **manual-jog-only when no mission is running**, or retires. Until the graph
exists, the console's direct path is the working one — keep it, but don't build new
mission logic on it beyond the demo slice (§3.1).

### 2.3 Rewrite the raster planner (`waypoint_generator.py`)
The recovered `waypoint_generator.py` works but has a **design limit**: it buckets
surface points into raster lines by a single coordinate and never separates them by
**normal direction**. On non-prismatic parts this (a) collapses everything to one
line when the FOV-derived spacing exceeds the part size, and (b) mixes points from
different faces into one line. Confirmed on DEX05120563 (5 waypoints, one line,
targets on four different faces — un-executable). **Fix = face-group the samples by
dominant normal *before* rastering**, raster within each face group. This is the
"remake it" Ra asked for — the other three modules are correct and can stay as-is.

### 2.4 Planning comes back inside PathPlanner, not the backend
The old backend had `/api/plan` running the planner in-process (and a duplicate of
the pipeline + a quaternion helper imported from a CLI script). Do **not** rebuild
that. Per the architecture, PathPlanner (ROS 2 node) owns planning; the console
requests a plan over rosbridge. `scripts/plan_path.py` stays as the CLI/dev entry
point and as the reference the PathPlanner node wraps.

---

## 3. Build tasks (add the new components)

**Immediate goal:** select a part → path generated → the **SR5 physically moves**
around the part, tracing the path (no scanner — the motion *is* the demo).

### 3.1 The demo slice (fastest path — reuse what works, then migrate)
Recommended first, before standing up the full ROS 2 graph:
1. **Rough frame transform** (architecture decision 5) — measure the marked-corner
   → arm-base transform, put it in config (not hard-coded). Convert a generated
   path from part-frame (mm, Y-up) into arm-frame (m, Z-up + corner offset).
2. **Pose → arm motion (IK)** — confirm whether the xCore SDK exposes IK / a
   cartesian move (`moveL`) or only joint moves; add a "follow this path" method
   (sequential poses, settle between each) to the arm control.
3. **"Send to robot"** in the console — take the generated path → convert to arm
   frame → stream poses to the arm, gated on connection + `QC_ALLOW_MOTION` +
   operator confirm.
4. **Dry-run in mock first**, then real on the SR5 at low speed, with Ra present and
   the physical E-stop in reach.

This slice drives the arm host-side via the SDK (the console path). It is
**temporary** — mark anything built this way so it's easy to replace when the graph
lands.

### 3.2 The full ROS 2 graph (the target — migrate the slice into it)
Build toward `architecture.md` §3, one node/package at a time, each with its own
launch file. **The full interface contract (every topic/service/action + message
type) is defined in `architecture.md` §4 — build to that.**
- **qc_msgs** — the custom messages/services/actions (`architecture.md` §4.1).
  Build this **first**; every node below depends on these interface types.
- **docker/** — rebuild the Humble container (the ROS 2 stack only runs there).
  Recovered under `docker/`; confirm it still builds (`docker/build.sh`).
- **rosbridge** — add `rosbridge_suite` to the container; wire the console to
  `ws://localhost:9090` (browser → telemetry; backend → plan/mission calls).
- **PathPlanner** — wrap `libs/path_planning` + apply the frame transform + load
  part/table as MoveIt collision objects + run MoveIt (cartesian along scan lines)
  → finished trajectory. Publishes it for the viewer + feeds MovementDriver.
- **MovementDriver** — execute the planned trajectory on ArmDriver, position-check,
  request next. No planning.
- **TaskManager** — mission start/abort, owns `mission_state`/`path_state`/
  `scan_state`, triggers ScanningDriver, hands off to Phase 2.
- **ScanningDriver** — MIRACO Plus bridge (start/stop/`scan_state`). Blocked on the
  scanner hardware/SDK.
- **Phase 2** — pure-Python inspection lib (extract → clean → quality gate →
  register → deviation) wrapped by a thin ROS node that does the TaskManager
  handshake (rescan-on-quality-fail; hand the analysis to the operator).

---

## 4. Gotchas that will bite you

- **Python 3.12 only.** The xCore SDK ships CPython 3.8–3.12 builds; the host
  `python3` may be newer and silently fail to load the SDK (arm stays "Offline").
  Run under `.venv312` (created by `./setup.sh`); `run_console.sh` handles it.
- **SDK location** is `~/rokaeProject` (auto-detected; override `QC_SDK_PATH`).
- **One SDK session.** Don't run the console's real connection and the ROS 2
  `ArmDriver` against the arm at the same time.
- **ROS 2 is Docker-only** on this machine (host ROS 2 is the wrong distro).
- **`rokae_ros2` is a vendor clone** with its own `.git` — a dependency, not our
  code; don't commit it into this repo.
- **Motion safety:** never command the physical arm without Ra's explicit
  go-ahead; `QC_ALLOW_MOTION=0` forces read-only; the physical E-stop is primary.
- **`config/` is gitignored** now (site-specific) — but keep at least one CAD file
  available for the part catalogue during dev.

---

## 5. Where the deep context lives

The reasoning behind every decision, the session history, and cross-project notes
live in **Ra's vault** (`~/Documents/ClaudeVault/...`) on his home machine — not on
this one. This repo is self-contained: `CLAUDE.md` (orientation), `docs/architecture.md`
(what to build), this guide (how/what-next), and `backend/README.md` (API). If you
need the *why* behind a decision and it isn't here, ask Ra.
