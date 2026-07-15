# QC Scanner — Operator Console backend

Serves the operator GUI (`gui/`) and exposes the JSON API it drives: the live
arm connection, the part catalogue, and the scan lifecycle / results store.
Built on the Python standard library only (`http.server`) — no Flask/FastAPI,
zero pip installs for the server itself.

## Run

The console is a **web app** served from source — no build step:

```bash
scripts/run_console.sh               # http://127.0.0.1:8000 (uses .venv312 + finds the SDK)
scripts/run_console.sh --port 9000   # different port
scripts/run_console.sh --host 0.0.0.0  # expose on the LAN (careful)
```

Then open <http://127.0.0.1:8000/> in a browser.

> **Python version — read this first.** The ROKAE xCore SDK ships CPython
> builds for **3.8–3.12 only**. The console must run under one of those or it
> **cannot connect to the arm** — under 3.13+ the SDK import silently falls
> through to an empty namespace dir and the arm stays "Offline". This machine's
> system `python3` is 3.14, so the repo keeps a pinned **3.12** env at
> `.venv312`. `./setup.sh` creates it with `uv` (a standalone 3.12, no system
> changes).

`run_console.sh` runs under `.venv312` and auto-sets `QC_SDK_PATH`, so the arm
connects. Running `python3 backend/server.py` directly uses the system 3.14 and
will leave the arm Offline — use the wrapper (or `.venv312/bin/python`).

> The GUI **must** be served over http (not opened as a `file://`) — the
> dc-runtime front-end self-fetches its own page to parse its template, which
> only works over http.

## What is real vs stub

Everything the console shows is either wired to real code or an honest "not
implemented yet" state. There is **no fabricated data** in the front-end.

| GUI area | Backed by |
| --- | --- |
| **Part catalogue** (Load CAD, recent parts) | **REAL** — `GET /api/parts` lists the actual CAD files in `config/cad/`. A part exists only if its STEP/STL is on disk. |
| **Robot connection** (header status dot + IP) | **REAL** — `robot_bridge.py` connects via the arm driver (`backend/sr5_arm_driver` → `RokaeArm`) on a real xCore SDK session (`/api/robot/connect`, `/api/robot/status`). No mock: if the arm is unreachable / the SDK can't load, it stays honestly **Offline**. |
| **Debug joint telemetry** | **REAL** — `/api/robot/joints` streams live `jointPos`/`jointVel`/`jointTorque`. Temperature isn't exposed by the driver → shown as `—`. |
| **Motion control** (safety bar + Debug "Jog to targets") | **REAL** — power/drag/stop/e-stop/clear-alarm/jog drive the physical SR5 via `/api/robot/{power,drag,stop,estop,clear_alarm,move}`. Jog is an absolute point-to-point move, confirmed each time. Gated on a live connection **and** the `QC_ALLOW_MOTION` master switch. |
| **Scan run** (Part ID → Start scan → Result) | **STUB but wired** — `POST /api/scan/start` records a real scan to the results store. Scanner capture + QC don't exist, so the record is honestly `incomplete` (null metrics, notes naming the missing subsystems). No fabricated pass/fail. |
| **Analytics** | **REAL (empty)** — reads the results store (`GET /api/scans`); shows recorded scans + an honest "QC metrics not yet computed" banner. |

### Robot connection + motion (`robot_bridge.py`)

The pure-Python arm backend lives in `backend/sr5_arm_driver/backends.py`
(`RokaeArm`); no `rclpy` needed. The bridge owns the single SDK session,
serialises access, and exposes both reads and motion commands. Env vars:

| Var | Default | Meaning |
| --- | --- | --- |
| `QC_ROBOT_IP` | `192.168.2.160` | SR5 address |
| `QC_SDK_PATH` | auto | Linux xCore SDK root (contains `Release/linux/`). Auto-detected: probes `~/rokaeProject` then `~/rokae_sdk`; set to override. |
| `QC_ALLOW_MOTION` | `1` | Master motion switch. `0` → bridge is read-only (all motion refused). |
| `QC_JOG_SPEED` | `60` | Default jog end-effector speed, mm/s. |

Endpoints:
- Reads: `GET /api/robot/status`, `GET /api/robot/joints`
- Session: `POST /api/robot/connect` `{ip?}`, `POST /api/robot/disconnect`
- Motion: `POST /api/robot/power` `{on}`, `/api/robot/drag` `{on}`, `/api/robot/stop`, `/api/robot/estop`, `/api/robot/clear_alarm`, `/api/robot/move` `{joints:[deg…], speedMms?}`

Each motion call returns the fresh status dict tagged `{ok, action, error?}`.
The console e-stop is a **software** stop (SDK stop2 + power-off), not a
substitute for the physical E-stop. The SDK allows only **one** session at a
time. A scan does **not** drive the arm — the console's motion controls are for
direct operator jogging only.

### Scan lifecycle + results store (`scan_pipeline.py`)

`POST /api/scan/start` `{partId}` runs the scan state machine and persists a
record to `data/scans.json` (`QC_DATA_DIR` to relocate). Also:
`GET /api/scan/status`, `GET /api/scans`, `GET /api/scans/{id}`,
`POST /api/scan/stop`. Scanner capture / registration / QC gate are stubs
marked `not_implemented`; wire them in `scan_pipeline.py` and the same UI +
store keep working.

## Offline

The GUI is fully offline-capable: React/ReactDOM are vendored in `gui/vendor/`
and preloaded before `support.js` (which then skips its unpkg CDN fetch), and
the only remote font `@import` (Roboto Mono) is disabled in the design-system
CSS in favour of the system-monospace fallback.
