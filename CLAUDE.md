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

The knowledge base, decision logs, and session history live in the vault at
`~/Documents/ClaudeVault/ClaudeVault/Projects/Quality Control Scanner/`.
This directory is code only.

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
├── config/cad/             ← CAD files backing the part catalogue
├── data/scans.json         ← recorded scans (results store)
└── scripts/run_console.sh  ← launch wrapper (pins .venv312, finds the SDK)
```

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

- **Arm:** ROKAE xMate SR5-5/0.9C — 6-axis, 919mm reach, 5kg payload, ±0.03mm repeatability
- **Scanner:** Revopoint MIRACO Plus — arm-mounted structured-light (capture not yet integrated)
- **Accuracy target:** best-achievable ≈65–75µm volumetric; CMM referee for tight-tolerance features
- **Workflow:** scan-flip-scan; each pass is an independent inspection; no cross-pass merge
