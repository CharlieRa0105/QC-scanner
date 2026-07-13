#!/usr/bin/env python3
"""
server.py

Backend for the 3D Scan QC Operator Interface (the `gui/` console).

Purpose / scope (2026-07-13):
    This is the FIRST real integration point between the operator GUI and the
    QC Scanner codebase. It does exactly two jobs:

      1. Serve the GUI as static files (so the dc-runtime front-end can run in a
         browser over http -- it self-fetches its own page, which only works
         when served, not opened as a file://).

      2. Expose ONE real endpoint, POST /api/plan, which runs the actual
         path-planning pipeline (cad_loader -> normal_estimation ->
         waypoint_generator -> incidence_cone_modifier) on a part's STEP file
         and returns real waypoint data.

    Everything else the GUI shows (live scan progress, joint telemetry, QC
    pass/fail, scan history, RViz/Open3D launch) is still MOCK data baked into
    the front-end -- those subsystems don't exist in the codebase yet, so there
    is nothing real to serve for them. As MovementDriver / scanner capture / QC
    are built, add endpoints here and wire the corresponding GUI handlers.

    Built on the Python standard library only (http.server) -- no Flask/FastAPI
    -- so it runs with zero pip installs, which also suits an offline shop-floor
    machine.

Run:
    python3 backend/server.py               # serves on http://127.0.0.1:8000
    python3 backend/server.py --port 9000
    python3 backend/server.py --host 0.0.0.0 # expose on the LAN (careful)

Then open http://127.0.0.1:8000/ in a browser.
"""

import argparse
import json
import os
import signal
import sys
import threading
import traceback
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# gmsh threading shim
#
# gmsh.initialize() installs a SIGINT handler (for Ctrl-C interruptibility)
# via signal.signal(), which Python only permits on the main thread. When the
# console runs as a desktop app (app.py), the HTTP server -- and therefore the
# planning pipeline that calls gmsh -- runs in a background thread, so that
# call would raise "signal only works in main thread of the main interpreter".
#
# We don't need gmsh's Ctrl-C handling here, so make signal.signal a no-op when
# it's called off the main thread. On the main thread it behaves normally, so
# the plain `python backend/server.py` CLI path is unaffected.
# ---------------------------------------------------------------------------
_orig_signal = signal.signal


def _safe_signal(sig, handler):
    if threading.current_thread() is threading.main_thread():
        return _orig_signal(sig, handler)
    return None  # off main thread: skip (gmsh loses Ctrl-C interrupt, which is fine)


signal.signal = _safe_signal

# ---------------------------------------------------------------------------
# Path wiring
#
# This file lives in <repo>/backend/. The GUI lives in <repo>/gui/, the CAD
# files in <repo>/config/cad/, the planner library in <repo>/libs/, and the
# CLI driver (whose quaternion helper we reuse) in <repo>/scripts/.
# ---------------------------------------------------------------------------
# Base dir holding gui/, config/, libs/, scripts/. Normally the repo root
# (this file's parent's parent). The desktop app (app.py) / a PyInstaller
# build overrides it via QC_BASE_DIR, because when frozen the data tree is
# unpacked somewhere else (sys._MEIPASS) and this file's path no longer points
# at the repo.
REPO_ROOT = Path(os.environ.get("QC_BASE_DIR") or Path(__file__).resolve().parent.parent)
GUI_DIR = REPO_ROOT / "gui"
CAD_DIR = REPO_ROOT / "config" / "cad"

# Make `libs.path_planning.*` and the scripts/ driver importable.
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from libs.path_planning.cad_loader import load_cad
from libs.path_planning.normal_estimation import sample_surface
from libs.path_planning.waypoint_generator import (
    generate_raster_waypoints,
    raster_spacing_from_fov,
)
from libs.path_planning.incidence_cone_modifier import apply_incidence_cone_relaxation

# Reuse the CLI's rotation-matrix -> quaternion conversion rather than
# duplicating it. Importing plan_path is side-effect-free (its argparse/main
# only runs under `if __name__ == "__main__"`).
from plan_path import rotation_matrix_to_quaternion

# Real SR5 connection layer (xCore SDK, real or mock). Lives in this backend
# dir; robot_bridge.py owns the single connection + serialises SDK access.
from robot_bridge import BRIDGE as ROBOT

# Planner defaults -- kept in sync with scripts/plan_path.py's argparse
# defaults and docs/running_the_planner.md. A request's "params" object
# overrides any of these per-call.
DEFAULT_PARAMS = {
    "standoff_mm": 300.0,
    "fov_deg": 40.0,          # PLACEHOLDER until the MIRACO Plus FOV is confirmed
    "overlap": 0.3,
    "raster_spacing_mm": None,  # None -> derive from fov+overlap
    "along_track_mm": 10.0,
    "max_incidence_deg": 25.0,
    "window": 2,
    "samples": 20000,
    "mesh_size_mm": 5.0,
    "step_axis": 1,
    "travel_axis": 0,
    "seed": 0,
}

# Cap how many waypoints we ship back to the browser for the path preview.
# The full count is always reported separately; this only bounds payload size.
MAX_PREVIEW_WAYPOINTS = 800


def resolve_cad_file(part_id):
    """
    Map a GUI part identifier to an actual CAD file in config/cad/.

    The GUI's part IDs are the full part names (e.g.
    "DEX05120563 B18 - Bearing press tool - Location pin small_Rev 0"), and the
    real STEP file is that same name plus a CAD extension. Matching strategy,
    most specific first:

      1. exact stem match  (filename without extension == part_id)
      2. DEX-code prefix   (filename starts with the part_id's first token,
                            e.g. "DEX05120563") -- tolerant of minor naming drift

    Returns a Path, or None if no CAD file matches (e.g. one of the GUI's
    mock-only parts that has no real STEP).
    """
    if not CAD_DIR.is_dir():
        return None

    cad_files = [
        p for p in CAD_DIR.iterdir()
        if p.suffix.lower() in (".step", ".stp", ".stl", ".obj")
    ]

    part_id = (part_id or "").strip()
    if not part_id:
        return None

    # 1. exact stem match
    for p in cad_files:
        if p.stem == part_id:
            return p

    # 2. DEX-code (first whitespace-separated token) prefix match
    code = part_id.split()[0] if part_id.split() else part_id
    for p in cad_files:
        if p.name.startswith(code):
            return p

    return None


def run_plan(part_id, params):
    """
    Run the full planning pipeline for one part and return a JSON-serialisable
    result dict. Raises ValueError with a human-readable message on any input
    problem (no CAD file, bad params) so the handler can return a clean 400.
    """
    cad_file = resolve_cad_file(part_id)
    if cad_file is None:
        raise ValueError(
            f"No CAD file in config/cad/ for part '{part_id}'. "
            "This part may exist only as mock data in the GUI -- add its STEP "
            "file to config/cad/ to plan against it."
        )

    # Merge caller overrides over the defaults.
    p = dict(DEFAULT_PARAMS)
    if isinstance(params, dict):
        p.update({k: v for k, v in params.items() if v is not None})

    # --- Pipeline (mirrors scripts/plan_path.py) ---
    vertices, faces = load_cad(str(cad_file), mesh_size=p["mesh_size_mm"])
    bbox_min = vertices.min(axis=0)
    bbox_max = vertices.max(axis=0)

    points, normals = sample_surface(
        vertices, faces, n_samples=int(p["samples"]), seed=int(p["seed"])
    )

    if p["raster_spacing_mm"] is not None:
        spacing = float(p["raster_spacing_mm"])
    else:
        spacing = raster_spacing_from_fov(p["standoff_mm"], p["fov_deg"], p["overlap"])

    waypoints = generate_raster_waypoints(
        points, normals,
        standoff_mm=p["standoff_mm"],
        raster_spacing_mm=spacing,
        along_track_mm=p["along_track_mm"],
        step_axis=int(p["step_axis"]),
        travel_axis=int(p["travel_axis"]),
    )

    results = apply_incidence_cone_relaxation(
        waypoints,
        max_incidence_angle_deg=p["max_incidence_deg"],
        window=int(p["window"]),
    )

    # Build the preview waypoint list (position + orientation quaternion),
    # decimated to MAX_PREVIEW_WAYPOINTS so the payload stays small.
    n = len(waypoints)
    stride = max(1, n // MAX_PREVIEW_WAYPOINTS)
    preview = []
    max_incidence = 0.0
    for i, (wp, r) in enumerate(zip(waypoints, results)):
        max_incidence = max(max_incidence, float(r["incidence_angle_deg"]))
        if i % stride != 0:
            continue
        qx, qy, qz, qw = rotation_matrix_to_quaternion(
            r["x_axis"], r["y_axis"], r["z_axis"]
        )
        preview.append({
            "position": [round(float(v), 4) for v in r["position"]],
            "quaternion": [round(q, 6) for q in (qx, qy, qz, qw)],
            "lineId": int(wp.line_id),
        })

    line_count = 1 + max((wp.line_id for wp in waypoints), default=-1)

    return {
        "ok": True,
        "partId": part_id,
        "cadFile": cad_file.name,
        "waypointCount": n,
        "lineCount": line_count,
        "spacingMm": round(spacing, 3),
        "maxIncidenceDeg": round(max_incidence, 3),
        "bbox": {
            "min": [round(float(v), 3) for v in bbox_min],
            "max": [round(float(v), 3) for v in bbox_max],
            "sizeMm": [round(float(v), 3) for v in (bbox_max - bbox_min)],
        },
        "meshVerts": int(len(vertices)),
        "meshFaces": int(len(faces)),
        "params": {k: p[k] for k in DEFAULT_PARAMS},
        "previewWaypoints": preview,
        "previewStride": stride,
    }


def launch_rviz():
    """Launch the SR5 arm view in RViz via the Humble Docker container.

    All ROS2 work runs in the `qc-humble` container (the ROKAE stack only
    supports Humble), with the GUI forwarded to the host X display. Runs
    detached so it outlives the request. Returns a status dict for the UI.

    Repo dir (holding docker/ + ros2_ws/) is REPO_ROOT for a source run; set
    QC_REPO_DIR when running the packaged app so the container can mount the
    real workspace.
    """
    import shutil
    import shlex
    import subprocess
    import time

    repo = Path(os.environ.get("QC_REPO_DIR") or REPO_ROOT)

    if shutil.which("docker") is None:
        return {"launched": False, "reason": "docker not found on the host"}
    if not (repo / "ros2_ws" / "src" / "rokae_ros2").is_dir():
        return {"launched": False,
                "reason": f"rokae_ros2 workspace not found under {repo}/ros2_ws — run docker/build.sh"}
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        return {"launched": False, "reason": "no display available (headless session)"}
    if subprocess.run(["docker", "image", "inspect", "qc-humble"],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode != 0:
        return {"launched": False, "reason": "qc-humble image not built — run docker/build.sh"}

    # Remove any prior arm-view container (avoids a volume/X collision) and let
    # the container reach the host X server. NOTE: no --rm on the run below, so
    # if it dies on startup we can still read its logs to report why.
    subprocess.run(["bash", "-lc",
                    "docker rm -f qc_arm_viz >/dev/null 2>&1; "
                    "xhost +local:root >/dev/null 2>&1 || true"])

    inner = ("source /opt/ros/humble/setup.bash && cd /ros2_ws && "
             "colcon build --packages-select rokae_description && "
             "source install/setup.bash && ros2 launch /view_arm.launch.py")
    run = (
        'docker run -d --name qc_arm_viz --net=host '
        '-e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 '
        '-e LIBGL_ALWAYS_SOFTWARE=1 -e XDG_RUNTIME_DIR=/tmp/runtime-root '
        '-v /tmp/.X11-unix:/tmp/.X11-unix '
        f'-v "{repo}/ros2_ws:/ros2_ws" '
        f'-v "{repo}/docker/view_arm.launch.py:/view_arm.launch.py:ro" '
        f'-v "{repo}/docker/view_arm.rviz:/view_arm.rviz:ro" '
        '-v qc_humble_build:/ros2_ws/build -v qc_humble_install:/ros2_ws/install '
        f'qc-humble bash -lc {shlex.quote(inner)}'
    )
    # -d returns quickly with the container id (or a non-zero + error). Capture
    # it instead of firing blind, so real launch failures reach the UI.
    p = subprocess.run(["bash", "-lc", run], capture_output=True, text=True)
    if p.returncode != 0:
        return {"launched": False,
                "reason": ("docker run failed: " + (p.stderr.strip() or p.stdout.strip()))[:400]}

    # Give it a moment; if the container exited (bad X access, build error, …),
    # surface its logs rather than claiming success.
    time.sleep(3)
    alive = subprocess.run(["docker", "ps", "-q", "--filter", "name=qc_arm_viz"],
                           capture_output=True, text=True).stdout.strip()
    if not alive:
        logs = subprocess.run(["bash", "-lc", "docker logs qc_arm_viz 2>&1 | tail -20"],
                              capture_output=True, text=True).stdout
        return {"launched": False,
                "reason": "RViz container exited on startup — see log",
                "log": logs[-1600:]}

    return {
        "launched": True,
        "note": ("Launching the SR5 in RViz via the Humble container, with a "
                 "joint-slider window — drag to move the arm. The first launch "
                 "builds the workspace, so RViz can take a moment to appear."),
    }


class QCRequestHandler(SimpleHTTPRequestHandler):
    """
    Serves the GUI as static files (inherited from SimpleHTTPRequestHandler,
    rooted at gui/) and adds the POST /api/plan JSON endpoint.
    """

    def __init__(self, *args, **kwargs):
        # Serve static files out of the GUI directory.
        super().__init__(*args, directory=str(GUI_DIR), **kwargs)

    def _send_json(self, status, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        # Same-origin in normal use, but allow CORS so the endpoint can be
        # probed from a separate dev tool without a preflight headache.
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        route = self.path.split("?")[0]
        # ---- robot state (read-only, never commands motion) ----
        if route == "/api/robot/status":
            self._send_json(200, ROBOT.status())
            return
        if route == "/api/robot/joints":
            self._send_json(200, ROBOT.joints())
            return
        # "/" -> the console entry point. SimpleHTTPRequestHandler would look
        # for index.html (which doesn't exist); point it at the app file.
        if self.path == "/" or self.path == "":
            self.path = "/Scan Cell Console.dc.html"
        return super().do_GET()

    def do_POST(self):
        route = self.path.split("?")[0]
        # Read the JSON body once (all POST routes take one, possibly empty).
        try:
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length else b"{}"
            payload = json.loads(raw or b"{}")
        except (ValueError, json.JSONDecodeError) as e:
            self._send_json(400, {"ok": False, "error": f"bad request body: {e}"})
            return

        # ---- robot connect / disconnect ----
        if route == "/api/robot/connect":
            try:
                self._send_json(200, ROBOT.connect(ip=payload.get("ip")))
            except Exception as e:  # noqa: BLE001
                traceback.print_exc()
                self._send_json(500, {"connected": False, "error": f"connect failed: {e}"})
            return
        if route == "/api/robot/disconnect":
            self._send_json(200, ROBOT.disconnect())
            return

        # ---- launch RViz2 (real, host ROS2) ----
        if route == "/api/rviz/launch":
            try:
                self._send_json(200, launch_rviz())
            except Exception as e:  # noqa: BLE001
                traceback.print_exc()
                self._send_json(500, {"launched": False, "reason": str(e)})
            return

        # ---- path planning ----
        if route != "/api/plan":
            self._send_json(404, {"ok": False, "error": "unknown endpoint"})
            return

        part_id = payload.get("partId", "")
        params = payload.get("params", {})

        try:
            result = run_plan(part_id, params)
            self._send_json(200, result)
        except ValueError as e:
            # Expected input problems (no CAD file, bad params) -> 400.
            self._send_json(400, {"ok": False, "error": str(e)})
        except Exception as e:  # noqa: BLE001 -- unexpected: log + return 500
            traceback.print_exc()
            self._send_json(500, {"ok": False, "error": f"planning failed: {e}"})

    def log_message(self, fmt, *args):
        # Terse one-line access log to stderr (default is noisy).
        sys.stderr.write("[qc-backend] %s\n" % (fmt % args))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address (default 127.0.0.1; use 0.0.0.0 to expose on LAN)")
    ap.add_argument("--port", type=int, default=8000, help="port (default 8000)")
    args = ap.parse_args()

    httpd = HTTPServer((args.host, args.port), QCRequestHandler)
    print(f"QC Scanner console backend serving {GUI_DIR}")
    print(f"  open  http://{args.host}:{args.port}/")
    print(f"  plan  POST http://{args.host}:{args.port}/api/plan")
    print(f"  CAD   {CAD_DIR}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
        httpd.server_close()


if __name__ == "__main__":
    main()
