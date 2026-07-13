# QC Scanner — Operator Console backend

Serves the operator GUI (`gui/`) and exposes the one endpoint that is wired to
real code today: **`POST /api/plan`**, which runs the actual path-planning
pipeline on a part's STEP file.

## Run (two ways)

**A. Desktop app (double-click)** — a single-file executable that opens a
native window (falls back to the default browser if no webview backend):

```bash
./dist/qc-console                    # native window
QC_HEADLESS=1 ./dist/qc-console      # server only, no window (for testing)
```

Build it (Linux) with PyInstaller — see `qc_console.spec` / `scripts/build_console.sh`:

```bash
python3 -m venv --system-site-packages .venv     # sees system numpy + gmsh
.venv/bin/pip install pywebview pyinstaller
scripts/build_console.sh                          # -> dist/qc-console
```

The desktop entry point is `app.py` (starts the backend on a free localhost
port in a daemon thread, then opens the pywebview window). `qc-console.desktop`
is a file-manager launcher template.

> **Windows .exe:** PyInstaller cannot cross-compile. To get a `.exe`, run the
> same `qc_console.spec` on a Windows machine.

**B. Served website** — no build step, stdlib only:

```bash
python3 backend/server.py            # http://127.0.0.1:8000
scripts/run_console.sh               # same, convenience wrapper
```

Then open <http://127.0.0.1:8000/> in a browser.

> The GUI **must** be served over http (not opened as a `file://`) — the
> dc-runtime front-end self-fetches its own page to parse its template, which
> only works over http.

## What is real vs mock

This is the **first** integration slice. Only path generation is real.

| GUI area | Backed by |
| --- | --- |
| **Generate scan path** (Load CAD → Preview) | **REAL** — `POST /api/plan` runs `cad_loader → normal_estimation → waypoint_generator → incidence_cone_modifier` and returns the real waypoint count, line count, bbox, mesh stats, incidence, and a decimated waypoint preview. |
| **Robot connection** (header status dot + IP) | **REAL** — `robot_bridge.py` connects via the **project's arm driver** (`ros2_ws/src/sr5_arm_driver` → `RokaeArm`) on a real xCore SDK session (`/api/robot/connect`, `/api/robot/status`). Falls back to the driver's `MockArm` when the arm is unreachable / SDK unavailable (label shows "Connected · mock"). Offline → honestly shows "Offline". |
| **Debug joint telemetry** | **REAL** — `/api/robot/joints` streams live `jointPos`/`jointVel`/`jointTorque` from the connected arm (real or mock). The slider/field set a local target; motion command is **not** wired (safety). |
| **RViz launch** | **REAL** — `/api/rviz/launch` runs the SR5 arm view in the **Humble Docker container** (`qc-humble`), GUI forwarded to the host X display: the exact SR5 model + a joint-slider window (drag to move). No rail / no scan-path overlay yet. Build once with `docker/build.sh`. |
| Analytics / scan history / heatmap | **Empty state** — no QC results store exists, so it shows "No scan data yet" instead of fabricated dashboards. |
| Live scan progress, ROS2 graph, logs, Open3D | Mock (front-end) — no MovementDriver / scanner capture / QC engine yet |

### Robot connection (`robot_bridge.py`)

The robot-driver code is **taken from the project's ROS 2 arm driver**
(`ros2_ws/src/sr5_arm_driver`): the bridge wraps its pure-Python backend classes
(`RokaeArm` real / `MockArm` mock), so the console and the ROS 2 `ArmDriver` node
share one driver implementation. No `rclpy` needed here — the backends are plain
Python. Env vars:

| Var | Default | Meaning |
| --- | --- | --- |
| `QC_ROBOT_MODE` | `auto` | `auto` (real iff arm pings + SDK connects, else mock) / `real` / `mock` |
| `QC_ROBOT_IP` | `192.168.2.160` | SR5 address |
| `QC_SDK_PATH` | `~/rokae_sdk` | Linux xCore SDK root (contains `Release/linux/`) |

Endpoints: `GET /api/robot/status`, `GET /api/robot/joints`, `POST /api/robot/connect` `{ip?}`, `POST /api/robot/disconnect`. All read-only w.r.t. motion — motion is driven through the ROS 2 `ArmDriver` / teach GUI, not the console. Note the SDK allows only **one** session, so don't run the console's real connection and the ROS 2 `ArmDriver` against the same arm at once.

As those subsystems get built, add endpoints here and wire the matching GUI
handler (they currently live in the `<script data-dc-script>` block of
`gui/Scan Cell Console.dc.html`).

## `POST /api/plan`

Request:

```json
{ "partId": "DEX05120563 B18 - Bearing press tool - Location pin small_Rev 0",
  "params": { "raster_spacing_mm": 8, "mesh_size_mm": 3 } }
```

- `partId` — mapped to a CAD file in `config/cad/` (exact stem match, then
  DEX-code prefix). A part that exists only as GUI mock data (no STEP on disk)
  returns a clean `400` explaining that.
- `params` — any of the planner knobs; omitted keys use the defaults in
  `DEFAULT_PARAMS` (mirrors `scripts/plan_path.py`). `params: {}` uses all
  defaults.

Response (200):

```json
{ "ok": true, "waypointCount": 56, "lineCount": 13, "spacingMm": 8.0,
  "maxIncidenceDeg": 25.0, "bbox": {...}, "meshVerts": 2191, "meshFaces": 4378,
  "params": {...}, "previewWaypoints": [ {"position":[...],"quaternion":[...],"lineId":0}, ... ] }
```

### Placeholder-FOV caveat

The GUI's "Generate scan path" button sends `params: {}` (all defaults),
including the **unconfirmed 40° placeholder FOV**. On a small part that derives
a large line spacing (~153 mm), collapsing the part to a single raster line
(e.g. the 45 mm location-pin test part → 1 line, 5 waypoints). The result is
correct, just coarse. To get a demo-quality multi-line raster, pass an explicit
`raster_spacing_mm` in the `generatePath` request body (front-end) or when
calling the endpoint directly. The real fix is pinning the MIRACO Plus FOV
(tracked open item).

## ROS2 / RViz (Humble, in Docker)

The ROKAE SDK + `rokae_ros2` only support **ROS2 Humble**, so all ROS2 work
runs in the `qc-humble` Docker container (base `osrf/ros:humble-desktop-full`
+ `joint_state_publisher_gui`), not on the host. See `docker/`.

```bash
docker/build.sh     # build qc-humble + clone rokae_ros2 into ros2_ws/src
docker/run_arm.sh   # SR5 in RViz + joint sliders (X11 forwarded to the host)
```

`run_arm.sh` bind-mounts `ros2_ws` at `/ros2_ws` (space-free path, so
`package://` resolves after a `colcon build`), forwards X11, and launches
`docker/view_arm.launch.py` (robot_state_publisher + joint_state_publisher_gui
+ rviz2). The console's **Open in RViz** button runs the same thing via
`/api/rviz/launch`.

The host no longer needs a ROS2 install. To remove the old `lyrical`:
`sudo apt remove 'ros-lyrical-*' && sudo apt autoremove`.

## Offline

The GUI is fully offline-capable: React/ReactDOM and THREE (r128) are vendored
in `gui/vendor/` and preloaded before `support.js` (which then skips its unpkg
CDN fetch), and the only remote font `@import` (Roboto Mono) is disabled in the
design-system CSS in favour of the system-monospace fallback.
