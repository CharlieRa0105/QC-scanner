# Project handover — Dexory automated QC scanning system

This is a single-file handover for a fresh Claude Code session (or a new
engineer) picking up this project cold. It captures what the project is,
what currently exists in the repo, the decisions that are settled, and what
to do next. Read this first, then `README.md` and the files under `docs/`.
For the full options/recommendation reasoning see the vault sub-notes
[[Quality Control Scanner - Options Report]] and
[[Quality Control Scanner - Hardware Research]].

- **Stage:** R&D, exploratory. Folder structure and documentation exist;
  **no implementation code has been written yet** — every Python file is
  an intentional docstring-only stub.
- **Owner:** Dexory R&D (solo project, structured for clean handoff).
- **Developer profile:** strong software programmer, minimal hardware
  experience. Prefer existing libraries/algorithms over building from
  scratch. Prefer deterministic solutions over ML.
- **Nature of the project:** this is the **automation of an existing,
  working manual QC process** — not a greenfield cell and not a redesign of
  the QC department.

---

## 1. What the system does

An automated 3D scanning cell for quality control. An operator places a
part (up to ~2.5×1.5m) at a marked corner reference on a ~2.5×2.5m table,
enters a part ID, and presses go. The system loads the pre-made CAD file for
that part, generates a scan path from the CAD mesh, drives the scanner over
every up-facing surface, and checks the scan is complete before passing it
downstream.

The key idea: **the part's geometry is known from the CAD file**. Parts vary
widely, but each has a CAD file stored by part ID. The operator places the part
at the marked corner (fixed position + orientation), and the scan path is
generated deterministically from that CAD mesh every time. No depth cameras.

**Workflow — scan-flip-scan (mirrors the existing manual process):** scan
the up-facing surfaces → compare to CAD → operator flips the part → scan
the other side → compare to CAD again. The two passes are **independent,
complete inspections — there is no cross-pass registration or cloud merge.**
Each pose is scanned and gated on its own.

This repo is the codebase for that cell. At this stage it is scaffolding
with documented intent; implementation gets filled in as hardware arrives
and decisions are settled.

---

## 2. System pipeline (the core flow)

The orchestrator (`src/orchestrator.py`) owns this sequence:

1. Operator places part at the marked corner reference, enters part ID, presses go (`scripts/run_scan.py`).
2. CAD file for the part ID is loaded; part position + orientation are known from the corner.
3. Surface normals are estimated from the CAD mesh (Open3D).
4. A coverage path planner generates arm + track waypoints — scanner at
   ~300mm standoff, perpendicular to each CAD surface normal.
5. MoveIt2 validates the path against the collision model (table, track limits, self-collision).
6. The track indexes to a station; the arm poses the scanner; it settles,
   triggers, and confirms capture before advancing (**index-and-shoot** —
   the track is not a coordinated joint).
7. Scan is captured via the scanner's hardware trigger + status handshake
   and exported as a point cloud (`.ply`).
8. Quality gate runs:
   - **Density** — voxel grid over bounding volume, minimum point count per cell.
   - **Outliers** — Open3D `statistical_outlier_removal()`, flag if removed ratio too high.
   - **Holes** — compare scan coverage against expected CAD surface coverage.
   - Combine into a 0–1 score: above threshold → pass, below → rescan.
9. Max 2 rescan attempts before flagging for human review.
10. On pass: export scan data for the downstream QC comparison pipeline
    (a future phase, not built here).

A second tool on the changer — a **tracker-referenced contact thread
probe** — gauges threads dimensionally (PD/pitch to tolerance) in the same
clamped cycle, with its path generated from the scan. See §5.

---

## 3. Software stack

- OS: Ubuntu 22.04 LTS
- Robot framework: ROS2 Humble
- Motion planning: MoveIt2
- Arm driver: `rokae_ros2` (official ROS 2 Humble; ships SR5 URDF + MoveIt 2 config + Gazebo; **beta — v0.0.4**)
- Arm control API: **xCore SDK** (C++ primary / Python binding; up to 1 kHz; position/impedance/torque modes)
- Point cloud processing: Open3D (also used for CAD mesh loading + normal estimation)
- Scanner: custom ROS2-Humble bridge over the **RevoLink SDK** (Revopoint MIRACO Plus — no official ROS2 driver)
- Path planning: surface_coverage_planning (ROS2 package) or equivalent
- Language: Python 3.10+

ROS2, MoveIt2, and the arm driver are installed on the on-site Ubuntu
machine following their own instructions — they are **not** pip packages
and are intentionally not installed by this repo's setup. Python deps are
in `requirements.txt`; install the package itself with `pip install -e .`.

---

## 4. Current repo state — what exists

Everything below exists. **All `.py` files contain only a module
docstring** describing intent; there is no implementation yet. This was a
deliberate scaffolding-only first session.

```
dexory-qc-scanner/
├── README.md                     # human-readable project intro
├── HANDOVER.md                   # this file
├── .gitignore                    # Python + ROS2 artifacts + point-cloud data
├── requirements.txt              # open3d, pyrealsense2, ur-rtde, numpy, pyyaml, pytest
├── setup.py                      # editable-install packaging stub
│
├── config/
│   └── system_config.yaml        # all known params + TBD placeholders
│
├── src/
│   ├── __init__.py
│   ├── orchestrator.py           # top-level pipeline (ties modules together)
│   ├── sensors/
│   │   ├── realsense_capture.py  # capture + merge clouds from both cameras
│   │   └── calibration.py        # one-time stereo calibration between cameras
│   ├── path_planning/
│   │   ├── normal_estimation.py  # surface normals from merged cloud
│   │   ├── waypoint_generator.py # scanner poses at standoff from normals
│   │   └── collision_check.py    # MoveIt2 collision validation wrapper
│   ├── arm_control/
│   │   ├── ur_controller.py      # UR20 control over RTDE
│   │   └── track_controller.py   # 7th-axis track control
│   └── quality_gate/
│       ├── density_check.py      # voxel-grid density validation
│       ├── outlier_check.py      # statistical outlier removal + ratio check
│       ├── hole_detection.py     # coverage vs reference cloud
│       └── gate.py               # runs all checks, returns pass/fail + score
│
├── tests/
│   ├── test_sensors.py
│   ├── test_path_planning.py
│   ├── test_quality_gate.py
│   └── fixtures/                 # sample point clouds for testing (empty)
│
├── scripts/
│   ├── calibrate_cameras.py      # run once to calibrate stereo camera setup
│   ├── run_scan.py               # CLI entry point for a scan
│   └── test_arm_connection.py    # quick connectivity check for arm + track
│
└── docs/
    ├── architecture.md           # system overview, pipeline, module map
    ├── hardware_setup.md         # physical setup guide
    ├── calibration.md            # camera + frame calibration
    ├── adding_new_tools.md       # how to add a new end effector
    └── open_questions.md         # unresolved decisions + what each blocks
```

All tunable values live in `config/system_config.yaml` — modules should
read from it rather than hard-coding constants. Site-specific overrides
with real IPs go in `config/local_config.yaml` (git-ignored).

### Known config values (and the important TBDs)

- Scanner standoff: optimal 300mm, range 200–400mm, output `.ply`.
- Accuracy bar: **best-achievable ≈ 65–75µm volumetric** (RESOLVED 2026-07-01 —
  "as accurate as achievable within constraints"; 50µm = current *manual
  touch-probe* figure). Scope = **whole-part scan-vs-CAD (volumetric), not
  per-feature**; CMM referee for guaranteed-tight features + threads.
- Arm: **ROKAE xMate SR5-5/0.9C**, reach **919mm**, payload **5kg**, repeatability **±0.03mm**, conservative speed 0.1 m/s, accel 0.05.
- Track: **custom-built 3m linear rail**; drive interface TBD at build time; IP/port TBD on site.
- No depth cameras. Part placed at **marked corner reference**; CAD files in `config/cad/`.
- Quality gate: voxel 5.0mm, min 10 pts/voxel, outlier std ratio 2.0 /
  20 neighbours, max hole fraction 0.05, min coverage score 0.85, max 2
  rescans.
- Table: ~2500 x 2500mm; largest part ~2500 x 1500mm; **working height TBD**.

---

## 5. Settled decisions & hardware direction

Full reasoning and the candidate shortlist live in the recommendation
sub-notes ([[Quality Control Scanner - Options Report]],
[[Quality Control Scanner - Hardware Research]]). Summary of what is
decided vs open:

- **Accuracy bar = RESOLVED (owner, 2026-07-01): "as accurate as achievable
  within constraints."** Not a hard number. The current scanner does ~50µm over
  the whole part (a *manual touch-probe* figure); the automated whole-part
  **scan-vs-CAD** target is **best-achievable ≈ 65–75µm volumetric**. A **CMM
  referee** covers any guaranteed-tighter feature + the threads; no full-part CMM
  gate. Scope = whole-part scan-vs-CAD (volumetric), not per-feature.
- **Part ≈ 2.5×1.5m**, table ≈ 2.5×2.5m. Room is **climate-stable**, so
  thermal expansion is largely controlled.
- **Carrier = ROKAE xMate SR5-5/0.9C 6-DOF cobot** (5 kg payload, 919 mm reach, ±0.03 mm, on-hand — confirmed 2026-07-02, supersedes earlier UR20 placeholder) on a floor U/perimeter track, index-and-shoot, scan-flip-scan workflow.
- **Scanner = Revopoint MIRACO Plus** (confirmed in project brief — arm-mounted structured-light; no separate camera tracker unit; ~940g payload; the arm's ±0.03mm repeatability enters the measurement chain; RevoLink SDK; custom ROS2-Humble bridge required). WM-6000 rejected (manual handheld). Camera-based optical-tracker RFQ shortlist (Scantech TrackScan-Sharp / Creaform / ZG-Track) superseded.
- **Thread inspection is in scope** — threads (external + internal) carry a
  dimensional PD tolerance. No scanner probe gauges thread PD and no automated
  inline dimensional PD exists (internal threads hardest) → **two-tier: inline
  GO/NO-GO gauging + offline dimensional-PD referee** (Johnson Gage / Gagemaker;
  CMM escalation).
- **Depth cameras = 2× RealSense D455** corner-mounted (Orbbec Gemini 335
  longevity hedge) — path-planning + coverage reference only, never metrology.

**Open (Phase 0 — gates procurement):**
- **Accuracy bar RESOLVED** (best-achievable ~65–75µm; see above) — no numeric
  confirmation outstanding.
- MIRACO Plus integration: verify certified end-to-end accuracy at our 2.5×1.5m geometry; build custom ROS2-Humble bridge over RevoLink SDK.
- Thread: scope the two-tier station (inline GO/NO-GO + offline Johnson
  Gage/Gagemaker referee; CMM escalation). Internal threads get a true PD only
  offline.
- Track product selection; max part height; clamp/fixture design.

---

## 6. Biggest risks (ranked)

1. **Accuracy target at our geometry** — best-achievable ~65–75µm volumetric over 2.5m. Certify the MIRACO Plus end-to-end at our geometry. The cell may need a CMM verification gate for any feature needing guaranteed-tighter check.
2. **Arm-mounted dimensional thread gauging to tolerance** — classically a CMM task; whether a contact probe does it at gauge accuracy is unverified. Fallback: CMM-referee for flagged/sampled features.
3. **Coordinate-frame / hand-eye calibration** is the dominant accuracy lever (a 0.01° error ≈ 52µm at 300mm standoff). Unlike external-tracker architectures, the MIRACO Plus is arm-mounted so the SR5's ±0.03mm repeatability is now in the chain — calibration quality is critical.
4. **Custom ROS2 bridge for the MIRACO Plus** — Revopoint does not ship an official ROS2 driver; a bespoke ROS2-Humble bridge over the RevoLink SDK is required.
5. **7th-axis track is not a native coordinated joint** — index-and-shoot
   baseline, or custom `ros2_control` + URDF.
6. **Depth-camera accuracy/range** over a 2.5×2.5m table (D455 degrades past
   ~1m at the corners) — path-planning only, so non-critical.

---

## 7. Suggested next steps for the picking-up session

In rough priority order — confirm with the project owner before building:

1. **Phase 0 vendor confirmations** (gates procurement): the
   volumetric-vs-local 50µm question; scanner selection + certified accuracy
   at our volume + trigger/handshake + quotes; thread-probe accuracy.
2. **Stand up a simulation environment** (Gazebo + ROS2 Humble + MoveIt2 via `rokae_ros2`)
   so path planning and collision checking can be developed without
   hardware. The developer is software-strong, so simulation-first is the
   natural path. Budget the custom MIRACO Plus ROS2 bridge (RevoLink SDK).
3. **Implement the quality gate first** — the most hardware-independent
   part; develop/test against sample clouds in `tests/fixtures/`:
   `density_check.py`, `outlier_check.py`, `hole_detection.py`, `gate.py`.
4. **Then sensors → path planning → arm control**, integrating against
   simulation, leaving real-hardware bring-up for the on-site machine.

### Working constraints / conventions
- Prefer existing libraries and algorithms over building from scratch.
- Prefer deterministic solutions over ML.
- Read parameters from `config/system_config.yaml`; don't hard-code.
- Keep each module understandable cold — documentation matters as much as
  function. Match the existing docstring style.
- Hardware-dependent tests should be mocked/skipped when hardware is
  absent; cloud-processing tests run against fixtures.

### Things this stage should NOT do (until owner says otherwise)
- Don't install ROS2/MoveIt2/system deps locally — that's the on-site
  Ubuntu machine.
- Don't connect to hardware.
- Don't create Notion pages — project management is handled separately.

---

## 8. Pointers

- `README.md` — human-readable intro and run instructions.
- `docs/architecture.md` — pipeline + module map.
- `docs/hardware_setup.md` — physical setup, connections, setup order.
- `docs/calibration.md` — camera and frame calibration; why frame
  alignment is the dominant accuracy lever.
- `docs/adding_new_tools.md` — adding end effectors via the tool changer.
- `docs/open_questions.md` — the canonical open-questions list.
- `config/system_config.yaml` — all parameters.
- `Quality Control Scanner - Options Report` / `- Hardware Research` (vault
  sub-notes) — full options, candidate shortlist, and recommendation.
