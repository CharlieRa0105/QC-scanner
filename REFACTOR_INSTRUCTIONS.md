# Refactor instructions — start here (for a fresh Claude session)

You are picking up the **Dexory QC Scanner** to refactor the existing code and
build it out toward the target architecture. This file is your operating procedure
and your task queue. Read it fully before touching anything.

## 0. Read these first, in order

1. `CLAUDE.md` — what the project is, how to run it, the gotchas.
2. `docs/architecture.md` — **the architecture of record.** Every node and every
   interface (topic / service / action + message type) is defined there. Build to it.
3. `docs/refactor-guide.md` — the current state, what's messy, and what to add.
4. `backend/README.md` — the console backend + API.

The *reasoning* behind the decisions and the session history live in Ra's vault on
his home machine — not in this repo. If you need a *why* that isn't written down
here, ask Ra rather than guessing.

## 1. The plan (where this fits)

- **Audit (done)** — the read-only pass of the code against the architecture. Its
  output is `docs/refactor-guide.md`. You don't need to repeat it; trust it, but
  verify a claim before acting on it if something looks off.
- **Part 1 — get a scan working.** The first milestone: select a part → a path is
  generated → the **SR5 physically traces the path** (no scanner hardware yet, so
  the motion *is* the demo). This is allowed to be a pragmatic slice that reuses
  what works, then gets migrated.
- **Part 2 — refactor/build the components, one at a time.** Turn the slice into
  the real ROS 2 mission graph from `docs/architecture.md` — each node/package as
  its own scoped task.

## 2. How to work (rules — do not break these)

- **One task per prompt.** Do a single, coherent change, then stop and let Ra
  review. Do not chain multiple tasks in one go. The task queue below is sized for
  this.
- **Every file you write or change must be self-explanatory** — a plain-language
  header saying what it does and how it fits, plus comments on anything non-obvious.
  Match the style already in `libs/path_planning/*.py`. A file without that isn't done.
- **Ra drives; you guide.** Prefer explaining the reasoning and proposing the
  change over silently producing a finished file — especially for anything Ra is
  learning. Ask before making a non-obvious design choice.
- **Never move the physical arm without Ra's explicit go-ahead.** `QC_ALLOW_MOTION=0`
  forces read-only; the physical E-stop is the real safety device. Prove any motion
  flow against the **mock backend first**.
- **Do not commit or push** — that's Ra's call (his trigger, his push).
- **Prefer deterministic solutions; reuse existing libraries** over building from
  scratch. Read `config/system_config.yaml` for tunables; never hard-code them.

## 3. The task queue

Work top to bottom. Each task lists its **goal**, **scope**, and **done when**.
Cross-reference `docs/architecture.md` §4 for the exact interfaces.

### Cleanup (small, safe — do these first)

- **T1 — One arm backend.** The `RokaeArm`/`MockArm` backend exists twice:
  `backend/sr5_arm_driver/backends.py` and
  `ros2_ws/src/sr5_arm_driver/sr5_arm_driver/backends.py`. Pick one shared home and
  have both the console (`robot_bridge.py`) and the ROS 2 `ArmDriver` import it from
  there. *Done when:* one copy, both importers use it, console still connects.
- **T2 — Verify the recovered pieces build/run.** Confirm `docker/build.sh` still
  builds the `qc-humble` image, and `scripts/plan_path.py` runs on the sample STEP
  in `config/cad/`. *Done when:* both run; note any breakage in a short report.

### Part 1 — get a scan working (the demo slice)

- **T3 — Fix the raster planner.** Rewrite `libs/path_planning/waypoint_generator.py`
  to **face-group samples by dominant normal before rastering** (see
  `refactor-guide.md` §2.3). The other three planner modules are correct — leave them.
  *Done when:* a non-prismatic part produces a multi-line path whose waypoints stay
  on one face per line. **Check with Ra before writing — he may want to drive this.**
- **T4 — Frame transform.** Add the marked-corner part→arm transform to config (not
  hard-coded) and apply it so a generated path is in arm-base frame (m, Z-up).
  *Done when:* a path exports in arm coordinates; the offset is a config value.
- **T5 — Pose → arm motion.** Determine whether the xCore SDK exposes IK / a
  cartesian move or only joint moves; add a "follow this path" method (sequential
  poses, settle between each) to the arm control. *Done when:* it drives the **mock**
  through a path end-to-end.
- **T6 — "Send to robot".** Wire a console button: generated path → arm-frame →
  stream poses to the arm, gated on connection + `QC_ALLOW_MOTION` + operator
  confirm. *Done when:* mock traces the path from the UI; then, **with Ra + E-stop**,
  the real SR5 traces it at low speed. **This is the Part-1 milestone.**

> The slice (T5/T6) drives the arm host-side via the SDK — mark anything built this
> way as **temporary**; Part 2 replaces it with the ROS 2 graph.

### Part 2 — build the ROS 2 components (one task each)

Build to the interfaces in `docs/architecture.md` §4. Each is its own package with
its own launch file; a top-level `qc_mission.launch.py` brings up the whole graph.

- **T7 — `qc_msgs`.** The custom messages/services/actions (`architecture.md` §4.1).
  **Build this first — every node below depends on it.** *Done when:* the package
  builds and the types are importable.
- **T8 — rosbridge.** Add `rosbridge_suite` to the container; wire the console to
  `ws://localhost:9090` (browser → telemetry; backend → mission calls).
- **T9 — PathPlanner node.** Wrap `libs/path_planning` + the frame transform + load
  part/table as MoveIt collision objects + run MoveIt → trajectory. Serves the
  `/plan_path` action; publishes `/plan/scanpath` + `/plan/trajectory` for preview.
- **T10 — MovementDriver node.** Serves `/execute_path`; plays the trajectory to
  ArmDriver; confirms each point via joint feedback. Execution only, no planning.
- **T11 — TaskManager node.** `/mission/plan`, `/mission/execute`, `/mission/abort`;
  owns `/mission/state`, the rescan loop, and abort. Sequences plan → confirm →
  execute → scan → inspect.
- **T12 — ArmDriver rename.** Rename the camelCase topics (`/armCMD`, `/armPos`, …)
  to the canonical snake_case in `architecture.md` §4.2 (one line per topic).
- **T13 — ScanningDriver node.** MIRACO Plus bridge (`/scan/start`, `/scan/stop`,
  `/scan/state`). *Blocked on the scanner hardware/SDK — interface only until then.*
- **T14 — Phase 2 InspectionNode.** Pure-Python inspection lib (extract → clean →
  quality gate → register → deviation) wrapped by a thin node serving `/inspect`.
  See `docs/point_cloud_processing.md` for the algorithm design.

## 4. When you finish a task

Summarise what changed and why, in plain language, and hand back to Ra for review —
do not move on to the next task on your own. If a task turns out bigger or different
than described here, say so and propose an updated task rather than forcing it.
