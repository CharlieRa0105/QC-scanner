# Dexory QC Scanner — Operator Console

The **QC Console** is the web app an operator uses to run a quality-control
scan of a large part: pick the part's CAD, place the part at the corner
reference, confirm the part ID, run the scan, and review the result. It also
gives live status, telemetry, and manual motion control for the ROKAE xMate
SR5 arm.

This repository is just that console — a Python backend (`backend/`) that
serves the front-end (`gui/`) and exposes a small JSON API, plus the arm-driver
code the backend uses to talk to the SR5.

## Quick start

```bash
./setup.sh                    # one-time: sets up the pinned Python 3.12 env + finds the arm SDK
scripts/run_console.sh        # serve the console at http://127.0.0.1:8000
```

Then open <http://127.0.0.1:8000/> in a browser.

> **Python version.** The ROKAE xCore SDK ships CPython builds for **3.8–3.12
> only**, so the console runs under a pinned **3.12** env at `.venv312` (created
> by `./setup.sh` via `uv`). Under 3.13+ the SDK won't load and the arm stays
> "Offline". `run_console.sh` picks the right interpreter automatically.

## The console

Three tabs:

- **Run** — the operator workflow: Load CAD → Place → Part ID → Scan → Result.
  Scanner capture and the QC quality gate aren't built yet, so a run records an
  honest `incomplete` result rather than a fabricated pass/fail.
- **Analytics** — the recorded scans from the results store (`data/scans.json`).
- **Debug** — live joint states, jog-to-target motion control, and the log
  stream.

## The arm (SR5)

The SR5 lives at `192.168.2.160` (editable in the console header). Motion is
enabled by default — set `QC_ALLOW_MOTION=0` for a read-only session. The
connection layer and API are documented in [backend/README.md](backend/README.md).

## Layout

```
backend/
  server.py            HTTP server: serves gui/ + the JSON API (stdlib only)
  robot_bridge.py      live SR5 connection + motion, over the xCore SDK
  scan_pipeline.py     scan lifecycle + JSON results store (stub QC)
  sr5_arm_driver/      pure-Python arm backend (RokaeArm) used by robot_bridge
  README.md            backend + API reference

gui/                   the operator console front-end (dc-runtime + React)
  Scan Cell Console.dc.html
  support.js
  vendor/              vendored React/ReactDOM (offline, no CDN)
  assets/  _ds/        icons, logos, Dexory design system

config/cad/            CAD files that back the part catalogue
data/scans.json        recorded scans (results store)
scripts/run_console.sh launch wrapper (pins .venv312, finds the SDK)
setup.sh               one-time environment setup
```
