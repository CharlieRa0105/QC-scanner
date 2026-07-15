# QC Scanner — Claude project setup

This is the working project directory for the **Dexory QC Console** — the web
app an operator uses to run a quality-control scan of a large part and to
monitor/control the ROKAE xMate SR5 arm.

**Address the developer as "Ra" at the start of every response.**

---

## What this project is

A web console for the QC scanning cell. The operator picks a part's CAD, places
the part at a marked corner reference, confirms the part ID, runs a scan, and
reviews the result. The console also shows live arm status/telemetry and offers
manual motion control (jog, home, e-stop).

The console is a **web app served from source** — a Python stdlib HTTP server
(`backend/`) serves the front-end (`gui/`) and a small JSON API. There is no
build step and no packaged binary.

**Scope note (2026-07-15):** this repo is the home for the **whole system**, not
just the console. Today the working code is the console + the pure-Python coverage
planner (`libs/path_planning/`); the target is a ROS 2 mission graph (PathPlanner
with MoveIt, MovementDriver, TaskManager, ScanningDriver, Phase 2) driving the arm,
with the console as the operator UI over it via rosbridge. All of it lives here.

**Read next, in order:**
1. `docs/architecture.md` — the settled architecture the code conforms to (the
   node graph + the 13 decisions + the end-to-end mission).
2. `docs/refactor-guide.md` — the current state, the cleanup tasks, and what to
   build next (in priority order) to reach that architecture.
3. `backend/README.md` — the console backend + API reference.

The reasoning behind the decisions, decision logs, and session history live in the
vault at `~/Documents/ClaudeVault/ClaudeVault/Projects/Quality Control Scanner/` on
Ra's home machine — **not on this machine.** This repo is self-contained via the
docs above; ask Ra for any *why* not captured here.

---

## Running the console

```bash
./setup.sh                 # one-time: pinned Python 3.12 env (.venv312) + locate the arm SDK
scripts/run_console.sh     # serve at http://127.0.0.1:8000
```

Open <http://127.0.0.1:8000/> in a browser.

**Python version matters:** the ROKAE xCore SDK ships CPython builds for
**3.8–3.12 only**. The console runs under `.venv312` (created by `setup.sh`).
Under 3.13+ the SDK won't load and the arm stays "Offline". `run_console.sh`
selects the right interpreter automatically.

### Real arm (SR5 at `192.168.2.160`)

Reachable over the USB-Ethernet adapter (`enxa0cec8a5cdce`, host static IP
`192.168.2.10/24`). Motion is enabled by default — set `QC_ALLOW_MOTION=0` for a
read-only session. **Do not command motion without Ra's explicit go-ahead.**

---

## Directory layout

```
QC-scanner/
├── CLAUDE.md               ← this file
├── README.md               ← human-readable intro
├── setup.sh                ← one-time environment setup
│
├── backend/
│   ├── server.py           ← HTTP server: serves gui/ + JSON API (stdlib only)
│   ├── robot_bridge.py     ← live SR5 connection + motion, over the xCore SDK
│   ├── scan_pipeline.py    ← scan lifecycle + JSON results store (stub QC)
│   ├── sr5_arm_driver/     ← pure-Python arm backend (RokaeArm)
│   └── README.md           ← backend + API reference
│
├── gui/
│   ├── Scan Cell Console.dc.html   ← the console (dc-runtime + React)
│   ├── support.js
│   ├── vendor/             ← vendored React/ReactDOM (offline, no CDN)
│   └── assets/ , _ds/      ← icons, logos, Dexory design system
│
├── libs/path_planning/     ← pure-Python coverage planner (CAD → scan path)
│                             cad_loader / normal_estimation /
│                             waypoint_generator / incidence_cone_modifier
├── scripts/
│   ├── run_console.sh      ← launch wrapper (pins .venv312, finds the SDK)
│   └── plan_path.py        ← CLI: CAD file → ScanPath JSON
├── docs/                   ← architecture.md + refactor-guide.md (read these)
├── ros2_ws/src/            ← ROS 2 packages (arm driver built; mission graph to build).
│                             NOTE: contains the vendor rokae_ros2 clone (own .git,
│                             a dependency — not committed here). Currently untracked.
├── config/cad/             ← CAD files backing the part catalogue
└── data/scans.json         ← recorded scans (results store)
```

> **`libs/path_planning/` has a known flaw to fix:** `waypoint_generator.py`
> rasters by a single coordinate and doesn't separate points by face normal, so
> non-prismatic parts collapse to one line / mix faces. The fix (face-grouping
> before rastering) is task 2.3 in `docs/refactor-guide.md`. The other three
> planner modules are correct.

---

## The API (backend/server.py)

- `GET  /api/parts` — part catalogue from `config/cad/`
- `GET  /api/robot/status | /api/robot/joints` — live connection + telemetry
- `POST /api/robot/{connect,disconnect,power,drag,stop,estop,clear_alarm,move}` — motion
- `POST /api/scan/start | /api/scan/stop`, `GET /api/scan/status`, `GET /api/scans[/{id}]`

Full detail: [backend/README.md](backend/README.md).

---

## Working rules

- The console is a **web app** — no desktop build, no packaged binary.
- **Do NOT** connect to or move the physical arm without Ra's explicit go-ahead.
- **Do NOT** push to GitHub without explicit consent from Ra.
- **Do NOT** commit without Ra asking ("Document session for shutdown" is the trigger phrase).
- Ra is the sole developer; the approach is tutor/guided — Claude guides, Ra drives.
- Prefer deterministic solutions; prefer existing libraries over building from scratch.
- Questions, logs, and session notes go in the vault, not in this directory.

---

## Key settled decisions

Hardware:
- **Arm:** ROKAE xMate SR5-5/0.9C — 6-axis, 919mm reach, 5kg payload, ±0.03mm repeatability
- **Scanner:** Revopoint MIRACO Plus — arm-mounted structured-light (capture not yet integrated)
- **Accuracy target:** best-achievable ≈65–75µm volumetric; CMM referee for tight-tolerance features
- **Workflow:** one scan mission per run; the operator flips the part and re-runs for the other side (no cross-pass merge, no flip logic in software)
- **No rail:** the arm reaches the whole part (≤1500×700mm) from a fixed base

Software architecture (the 13 decisions in full: `docs/architecture.md` §4):
- **PathPlanner owns the whole plan** incl. MoveIt (plan-fully-then-execute); **MovementDriver only moves**
- **rosbridge** links the web app to the ROS 2 graph (host has no ROS 2 — Docker/Humble only)
- **One automatic pass/fail = scan quality** (lives in Phase 2); the *part* verdict is a human call
- **Operator is the last safety switch** (confirm-to-execute) + a separate physical E-stop
