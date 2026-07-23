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

      2. Expose the real API the GUI drives:
           /api/robot/*                      SR5 connection + live telemetry +
                                             motion, via robot_bridge (real)
           /api/parts                        part catalogue from config/cad (real)
           /api/scan/*, /api/scans           scan lifecycle + results store
                                             (real plumbing; capture/QC still
                                             stubbed -> honest `incomplete`)

    There is NO fabricated data served here: anything a subsystem can't yet
    produce (e.g. QC metrics) is returned as an explicit empty/`incomplete`
    state, never invented.

    Built on the Python standard library only (http.server) -- no Flask/FastAPI
    -- so it runs with zero pip installs, which also suits an offline shop-floor
    machine. NOTE: must run under Python 3.8-3.12 for the ROKAE SDK to load
    (see robot_bridge.py); 3.13+ leaves the arm Offline.

Run:
    python3 backend/server.py               # serves on http://127.0.0.1:8000
    python3 backend/server.py --port 9000
    python3 backend/server.py --host 0.0.0.0 # expose on the LAN (careful)

Then open http://127.0.0.1:8000/ in a browser.
"""

import argparse
import json
import math
import os
import sys
import traceback
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

# ---------------------------------------------------------------------------
# Path wiring
#
# This file lives in <repo>/backend/. The GUI lives in <repo>/gui/ and the CAD
# files (for the part catalogue) in <repo>/config/cad/.
# ---------------------------------------------------------------------------
# Base dir holding gui/ and config/. Normally the repo root (this file's
# parent's parent). Override with QC_BASE_DIR if the data tree ever lives
# somewhere other than alongside this file.
REPO_ROOT = Path(os.environ.get("QC_BASE_DIR") or Path(__file__).resolve().parent.parent)
GUI_DIR = REPO_ROOT / "gui"
# Part catalogue = the CAD database folder: a part number selects a file from here.
# Overridable via QC_PARTS_DIR; defaults to the operator's Parts library.
CAD_DIR = Path(os.environ.get("QC_PARTS_DIR") or "/home/frank/Desktop/Parts")
DATA_DIR = REPO_ROOT / "data"
# Pre-generated ARM-FRAME scan path streamed by "Send to robot" when the request
# carries no waypoints of its own. Produced offline by the CLI (plan_path.py ->
# scanpath_convert.py) -- planning is NOT done in this backend (refactor-guide
# §2.4; it returns inside the PathPlanner ROS 2 node). This is a demo-slice
# stand-in until the console requests a plan over rosbridge.
SCANPATH_FILE = DATA_DIR / "scanpath_arm.json"
# Bundle (part mesh + scan path + mount, table frame) for the 3D viewer
# (gui/viewer/), produced by scripts/export_viewer_bundle.py. Served at
# GET /api/viewer_bundle.
VIEWER_BUNDLE_FILE = DATA_DIR / "viewer_bundle.json"

# Last part planned + the orientation and fit primitive it was planned at, so the
# viewport can re-plan (re-fit the shape) when the operator reorients the part or
# switches the fit primitive.
LAST_PLAN = {"part_id": None, "orient_deg": None, "planner": None, "standoff_mm": None}


def _load_qc_config():
    """Merged YAML config for the UI (workspace box, planner defaults, debug
    shapes). Uses libs/qc_config (PyYAML) when available; the backend itself
    stays servable without it -- on any failure we return {} and the viewers
    fall back to their built-in defaults rather than dying."""
    try:
        sys.path.insert(0, str(REPO_ROOT))
        from libs.qc_config import load_config
        return load_config()
    except Exception as e:  # noqa: BLE001
        print(f"[qc-backend] config unavailable ({e}); serving empty config", file=sys.stderr)
        return {}

# Real SR5 connection layer (xCore SDK). Lives in this backend dir;
# robot_bridge.py owns the single connection + serialises SDK access, and
# exposes the arm's motion commands (power/drag/jog/stop/clear-alarm).
from robot_bridge import BRIDGE as ROBOT, HOME_JOINTS_DEG

# Scan lifecycle + results store (stub pipeline: real plumbing, honest empty
# data until scanner capture / QC exist).
from scan_pipeline import MANAGER as SCANS


def list_parts():
    """Enumerate the real CAD files in config/cad/ as the console's part list.

    A part exists iff its STEP/STL/etc. is actually on disk. `id` is the file
    stem; `short` is a tidied display name for the operator.
    """
    parts = []
    if not CAD_DIR.is_dir():
        return parts
    for p in sorted(CAD_DIR.iterdir()):
        if p.suffix.lower() not in (".step", ".stp", ".stl", ".obj"):
            continue
        stem = p.stem
        # Display name: drop a trailing "_Rev N" and collapse a leading DEX code
        # + dash into the human part of the name where present.
        short = stem
        if " - " in stem:
            short = stem.split(" - ", 1)[1]
        short = short.replace("_Rev ", " · Rev ").replace("_", " ").strip()
        parts.append({
            "id": stem,
            "short": short or stem,
            "file": p.name,
            "ext": p.suffix.lower().lstrip("."),
        })
    return parts


class QCRequestHandler(SimpleHTTPRequestHandler):
    """
    Serves the GUI as static files (inherited from SimpleHTTPRequestHandler,
    rooted at gui/) and adds the JSON API the console drives (robot, parts,
    scans).
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
        try:
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            # The browser polls status/joints continuously and routinely closes a
            # connection before we finish writing. That's benign -- swallow it so
            # the terminal isn't buried in tracebacks (which hid the arm logs).
            pass

    def do_GET(self):
        route = self.path.split("?")[0]
        # ---- robot state (read-only reads) ----
        if route == "/api/robot/status":
            self._send_json(200, ROBOT.status())
            return
        if route == "/api/robot/joints":
            self._send_json(200, ROBOT.joints())
            return
        if route == "/api/robot/log":
            # Live arm-comms feed for the debug monitor: every command + response
            # that went through the bridge log, since ?since=<seq>.
            from urllib.parse import parse_qs, urlparse
            since = (parse_qs(urlparse(self.path).query).get("since") or ["0"])[0]
            self._send_json(200, ROBOT.log_since(since))
            return

        if route == "/api/robot/follow_status":
            self._send_json(200, ROBOT.follow_status())
            return
        # Merged system+local config (workspace box, planner + debug defaults).
        if route == "/api/config":
            # Report the EFFECTIVE arm-base-to-table gap so a fresh viewer matches
            # the backend even after a runtime table-height change / env override.
            cfg = _load_qc_config()
            try:
                cfg.setdefault("workspace", {}).setdefault("mount", {})["height_mm"] = \
                    ROBOT.mount_height_mm()
            except Exception:  # noqa: BLE001
                pass
            self._send_json(200, {"ok": True, "config": cfg})
            return
        # Part mesh + scan path + mount (table frame) for the 3D viewer.
        if route == "/api/viewer_bundle":
            try:
                with open(VIEWER_BUNDLE_FILE) as f:
                    self._send_json(200, json.load(f))
            except FileNotFoundError:
                self._send_json(404, {"ok": False,
                                      "error": f"no {VIEWER_BUNDLE_FILE.name} — run scripts/export_viewer_bundle.py"})
            return
        # Last plan requested through the console (part_id + orient/planner/standoff).
        # Lets the Debug viewport trigger the ROS mission (/mission/plan) by part id.
        if route == "/api/plan/last":
            self._send_json(200, {"ok": bool(LAST_PLAN.get("part_id")), **LAST_PLAN})
            return
        # ---- part catalogue (real CAD files on disk) ----
        if route == "/api/parts":
            self._send_json(200, {"ok": True, "parts": list_parts()})
            return
        # ---- scan lifecycle + results store ----
        if route == "/api/scan/status":
            self._send_json(200, SCANS.status())
            return
        if route == "/api/scans":
            self._send_json(200, {"ok": True, "scans": SCANS.store.list()})
            return
        if route.startswith("/api/scans/"):
            scan_id = route[len("/api/scans/"):]
            rec = SCANS.store.get(scan_id)
            if rec is None:
                self._send_json(404, {"ok": False, "error": f"no scan {scan_id}"})
            else:
                self._send_json(200, {"ok": True, "scan": rec})
            return
        # "/" -> the console entry point. SimpleHTTPRequestHandler would look
        # for index.html (which doesn't exist); point it at the app file.
        if self.path == "/" or self.path == "":
            self.path = "/Scan Cell Console.dc.html"
        return super().do_GET()

    def _handle_upload(self, raw):
        """Save an uploaded CAD file (raw bytes) into config/cad/ so it joins the
        part catalogue. Filename comes from the ?name= query param."""
        from urllib.parse import parse_qs, urlparse
        name = (parse_qs(urlparse(self.path).query).get("name") or [""])[0]
        fn = os.path.basename(name)
        ext = os.path.splitext(fn)[1].lower()
        if ext not in (".step", ".stp", ".stl", ".obj"):
            self._send_json(400, {"ok": False, "error": f"unsupported CAD type {ext!r}"})
            return
        if not raw:
            self._send_json(400, {"ok": False, "error": "empty upload"})
            return
        CAD_DIR.mkdir(parents=True, exist_ok=True)
        with open(CAD_DIR / fn, "wb") as f:
            f.write(raw)
        self._send_json(200, {"ok": True, "file": fn})

    def _handle_plan(self, part_id, orient_deg=None, planner_key=None, standoff_mm=None):
        """Regenerate the scan path + viewer bundle for a part by running the
        existing planner CLIs as subprocesses (no planner code in this backend;
        planning belongs in the PathPlanner ROS node -- this is the demo slice).

        orient_deg: optional [rx,ry,rz] the operator reoriented the part by -- the
        planner re-fits for that pose, so rotating the part yields a new path.
        standoff_mm: optional scanner standoff (mm) for the shell planners -- for the
        hemisphere it sets the waypoint-to-centre distance (radius + standoff), for
        the box the per-face offset. None = the planner's own default (80mm)."""
        if not part_id:
            self._send_json(400, {"ok": False, "error": "partId required"})
            return
        LAST_PLAN["part_id"] = part_id           # remember for re-plan-on-reorient
        LAST_PLAN["orient_deg"] = orient_deg
        LAST_PLAN["planner"] = planner_key
        LAST_PLAN["standoff_mm"] = standoff_mm
        cad = next((p for p in sorted(CAD_DIR.iterdir())
                    if p.stem == part_id and p.suffix.lower() in (".step", ".stp", ".stl", ".obj")), None)
        if cad is None:
            self._send_json(404, {"ok": False, "error": f"no CAD for part {part_id!r}"})
            return
        import subprocess
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        part_json = DATA_DIR / "_plan_part.json"
        # Scale raster density to the part's surface area so small parts stay
        # densely covered (see plan_path.py --target-waypoints). Configurable via
        # QC_TARGET_WAYPOINTS; default 400. Set 0 to fall back to fixed FOV spacing.
        target_wp = os.environ.get("QC_TARGET_WAYPOINTS", "400")
        # Planner selection (QC_PLANNER): default 'dome' = the enclosing-hemisphere
        # dome raster (plan_hemisphere.py) -- scanner sweeps a dome around the part.
        # Others: 'surround' = multi-view lawnmower (all sides, plan_surround.py);
        # 'lawn' = top-surface only; 'contour' = plane-slice rings; 'grid' = old
        # face-group raster. All emit the same ScanPath JSON.
        # planner key: the request wins (viewport primitive toggle), else the
        # QC_PLANNER env default. 'dome' -> hemisphere; 'box' -> minimum-volume
        # oriented box with a per-face raster (plan_box.py).
        key = planner_key or os.environ.get("QC_PLANNER", "dome")
        planner = {"grid": "plan_path.py", "contour": "plan_contour.py",
                   "lawn": "plan_lawn.py", "surround": "plan_surround.py",
                   "box": "plan_box.py"}.get(key, "plan_hemisphere.py")
        plan_cmd = [sys.executable, f"scripts/{planner}", str(cad), str(part_json)]
        # --target-waypoints only exists on the area-scaled planners; the shell
        # planners (dome hemisphere, min-volume box) derive spacing from the
        # scanner footprint instead, so they must NOT receive it.
        SHELL_PLANNERS = ("plan_hemisphere.py", "plan_box.py")
        if target_wp and target_wp != "0" and planner not in SHELL_PLANNERS:
            plan_cmd += ["--target-waypoints", target_wp]
        # operator reorientation -> re-fit the shell for the new pose (both shell
        # planners accept --orient-deg).
        if orient_deg and planner in SHELL_PLANNERS:
            plan_cmd += ["--orient-deg", ",".join(str(float(a)) for a in orient_deg[:3])]
        # operator standoff (debug slider) -> waypoint distance from the fit shape;
        # only the shell planners expose --standoff-mm.
        if standoff_mm is not None and planner in SHELL_PLANNERS:
            plan_cmd += ["--standoff-mm", str(float(standoff_mm))]
        steps = [
            plan_cmd,
            [sys.executable, "scripts/scanpath_convert.py", str(part_json), str(SCANPATH_FILE)],
            [sys.executable, "scripts/export_viewer_bundle.py", str(cad), str(SCANPATH_FILE), str(VIEWER_BUNDLE_FILE)],
        ]
        for cmd in steps:
            try:
                r = subprocess.run(cmd, cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=180)
            except Exception as e:  # noqa: BLE001
                self._send_json(500, {"ok": False, "error": f"planner error: {e}"})
                return
            if r.returncode != 0:
                tail = (r.stderr or r.stdout or "").strip()[-300:]
                self._send_json(500, {"ok": False, "error": f"{os.path.basename(cmd[1])} failed: {tail}"})
                return
        try:
            with open(VIEWER_BUNDLE_FILE) as f:
                n = len(json.load(f).get("waypoints", []))
        except Exception:  # noqa: BLE001
            n = 0
        self._send_json(200, {"ok": True, "partId": part_id, "waypoints": n})

    def do_POST(self):
        route = self.path.split("?")[0]
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b""

        # ---- CAD upload (binary body, handled before JSON parsing) ----
        if route == "/api/parts/upload":
            self._handle_upload(raw)
            return

        # All other POST routes take a JSON body (possibly empty).
        try:
            payload = json.loads(raw or b"{}")
        except (ValueError, json.JSONDecodeError) as e:
            self._send_json(400, {"ok": False, "error": f"bad request body: {e}"})
            return

        # ---- (re)generate a scan path for a part (demo-slice: subprocess the
        # existing planner CLIs; no planner code runs in this stdlib backend) ----
        if route == "/api/plan":
            self._handle_plan(payload.get("partId", ""), payload.get("orientRpyDeg"),
                              payload.get("planner"), payload.get("standoffMm"))
            return

        # ---- re-plan the CURRENT part at a new orientation (operator reoriented
        # the part in the viewport -> regenerate the path/fit for that pose) ----
        if route == "/api/plan/reorient":
            pid = LAST_PLAN.get("part_id")
            if not pid:
                self._send_json(400, {"ok": False, "error": "no part planned yet"})
                return
            # preserve the current fit primitive + standoff across a reorient
            self._handle_plan(pid, payload.get("orientRpyDeg"), LAST_PLAN.get("planner"),
                              LAST_PLAN.get("standoff_mm"))
            return

        # ---- switch the FIT PRIMITIVE (hemisphere <-> rectangle) and re-plan the
        # current part on it, preserving orientation ----
        if route == "/api/plan/primitive":
            pid = LAST_PLAN.get("part_id")
            if not pid:
                self._send_json(400, {"ok": False, "error": "no part planned yet"})
                return
            prim = (payload.get("primitive") or "").lower()
            key = {"dome": "dome", "hemisphere": "dome",
                   "box": "box", "rectangle": "box"}.get(prim)
            if key is None:
                self._send_json(400, {"ok": False, "error": f"unknown primitive {prim!r}"})
                return
            self._handle_plan(pid, LAST_PLAN.get("orient_deg"), key,
                              LAST_PLAN.get("standoff_mm"))
            return

        # ---- adjust the scanner STANDOFF (debug slider: waypoint distance from the
        # fit shape) and re-plan the current part on it, preserving orient + primitive ----
        if route == "/api/plan/standoff":
            pid = LAST_PLAN.get("part_id")
            if not pid:
                self._send_json(400, {"ok": False, "error": "no part planned yet"})
                return
            try:
                standoff = float(payload.get("standoffMm"))
            except (TypeError, ValueError):
                self._send_json(400, {"ok": False, "error": "standoffMm (number) required"})
                return
            if not (0 < standoff <= 1000):
                self._send_json(400, {"ok": False, "error": "standoffMm out of range (0, 1000]"})
                return
            self._handle_plan(pid, LAST_PLAN.get("orient_deg"), LAST_PLAN.get("planner"), standoff)
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

        # ---- robot MOTION (drives the physical SR5) ----
        # Each returns the fresh status dict tagged {ok, action, error?}.
        if route in ("/api/robot/power", "/api/robot/drag", "/api/robot/stop",
                     "/api/robot/estop", "/api/robot/clear_alarm", "/api/robot/move",
                     "/api/robot/home"):
            try:
                if route == "/api/robot/power":
                    result = ROBOT.set_power(bool(payload.get("on", True)))
                elif route == "/api/robot/drag":
                    result = ROBOT.set_drag(bool(payload.get("on", True)))
                elif route == "/api/robot/stop":
                    result = ROBOT.stop()
                elif route == "/api/robot/estop":
                    result = ROBOT.estop()
                elif route == "/api/robot/clear_alarm":
                    result = ROBOT.clear_alarm()
                elif route == "/api/robot/home":
                    # MoveJ to the SINGLE source of truth for home (HOME_JOINTS_DEG),
                    # same one the scan start/end uses. Non-blocking (returns once
                    # motion starts) so the single-threaded server keeps serving the
                    # UI's joint polling while the arm travels home.
                    result = ROBOT.move_joints(HOME_JOINTS_DEG, payload.get("speedMms"))
                else:  # /api/robot/move
                    result = ROBOT.move_joints(payload.get("joints", []),
                                               payload.get("speedMms"))
                self._send_json(200, result)
            except Exception as e:  # noqa: BLE001
                traceback.print_exc()
                self._send_json(500, {"ok": False, "action": route.rsplit("/", 1)[-1],
                                      "error": str(e)})
            return

        # ---- single cartesian pose (debug viewport: camera-follow / re-aim) ----
        # Same gating as path tracing: connection + QC_ALLOW_MOTION, and on the
        # REAL arm also QC_ALLOW_SCAN_TRACE (unvalidated MoveL). Mock is free.
        if route == "/api/robot/move_pose":
            try:
                result = ROBOT.move_pose_once(
                    payload.get("position"), payload.get("quaternion"),
                    payload.get("speedMms"),
                    keep_orientation=bool(payload.get("keepOrientation")))
                self._send_json(200, result)
            except Exception as e:  # noqa: BLE001
                traceback.print_exc()
                self._send_json(500, {"ok": False, "action": "move_pose", "error": str(e)})
            return

        # ---- "Send to robot": stream a scan path to the arm (demo slice) ----
        # Traces a sequence of cartesian probe poses (ARM frame). Gated on an
        # explicit operator confirm here, plus connection + QC_ALLOW_MOTION in
        # the bridge. Runs in a worker thread; the UI polls /follow_status and
        # can abort via /stop. Waypoints come from the body, or (if absent) the
        # pre-generated SCANPATH_FILE.
        if route == "/api/robot/follow_path":
            if not payload.get("confirm"):
                self._send_json(400, {"ok": False,
                                      "error": "operator confirm required (confirm=true)"})
                return
            waypoints = payload.get("waypoints")
            source = "request body"
            if not waypoints:
                try:
                    with open(SCANPATH_FILE) as f:
                        sp = json.load(f)
                    waypoints = sp.get("waypoints", [])
                    source = SCANPATH_FILE.name
                    if sp.get("units") != "m":
                        # Poses must already be in arm units (metres). Refuse a
                        # part-frame path rather than drive the arm in millimetres.
                        self._send_json(400, {"ok": False,
                            "error": f"{SCANPATH_FILE.name} is not in arm frame "
                                     f"(units={sp.get('units')!r}); run scanpath_convert.py first"})
                        return
                except FileNotFoundError:
                    self._send_json(400, {"ok": False,
                        "error": f"no waypoints in body and no {SCANPATH_FILE.name} on server "
                                 "(generate one with plan_path.py + scanpath_convert.py)"})
                    return
            if not waypoints:
                self._send_json(400, {"ok": False, "error": "no waypoints to follow"})
                return
            result = ROBOT.start_follow_path(
                waypoints,
                speed_mms=payload.get("speedMms"),
                settle_s=payload.get("settleS", 0.3),
                position_only=bool(payload.get("positionOnly")),
            )
            result["source"] = source
            self._send_json(200, result)
            return

        # ---- oriented CONTINUOUS scan: head aims at the part (within a cone),
        # each scan line traced as one blended sweep. Same waypoint source +
        # gating as follow_path; uses the planner's aim quaternions + line_id.
        if route == "/api/robot/scan_trace":
            if not payload.get("confirm"):
                self._send_json(400, {"ok": False, "error": "operator confirm required (confirm=true)"})
                return
            waypoints = payload.get("waypoints")
            source = "request body"
            if not waypoints:
                try:
                    with open(SCANPATH_FILE) as f:
                        sp = json.load(f)
                    waypoints = sp.get("waypoints", [])
                    source = SCANPATH_FILE.name
                    if sp.get("units") != "m":
                        self._send_json(400, {"ok": False,
                            "error": f"{SCANPATH_FILE.name} is not in arm frame "
                                     f"(units={sp.get('units')!r}); run scanpath_convert.py first"})
                        return
                except FileNotFoundError:
                    self._send_json(400, {"ok": False,
                        "error": f"no waypoints in body and no {SCANPATH_FILE.name} on server"})
                    return
            if not waypoints:
                self._send_json(400, {"ok": False, "error": "no waypoints to scan"})
                return
            # orient soft-bias: the request wins; else fall back to the orientation
            # the current path was planned at (so the console, which doesn't track
            # it, still biases the IK to match the plan).
            orient_deg = payload.get("orientRpyDeg") or LAST_PLAN.get("orient_deg") or [0, 0, 0]
            orient_rpy = [math.radians(float(a)) for a in orient_deg[:3]]
            result = ROBOT.start_scan_trace(
                waypoints,
                incidence_deg=float(payload.get("incidenceDeg", 10.0)),
                speed_mms=payload.get("speedMms"),
                orient_rpy=orient_rpy,
            )
            result["source"] = source
            self._send_json(200, result)
            return

        # ---- NO-MOTION preview: register + reachability-check the scan path for
        # a given orientation, returned in the viewer frame ("Generate path").
        if route == "/api/robot/table_height":
            # NO MOTION: set the arm-base-to-table gap H (mm) -- the table-height
            # knob the debug viewport drives. Reachability preview uses it live.
            result = ROBOT.set_mount_height_mm(payload.get("mm"))
            self._send_json(200 if result.get("ok") else 400, result)
            return

        if route == "/api/robot/scan_preview":
            # The viewer sends the planner path it is ALREADY drawing (probe
            # positions + surface targets, in the viewer/table frame); the bridge
            # only reachability-tags those exact poses. This keeps preview and the
            # drawn path a single source of truth -- no separate overhead raster.
            poses = payload.get("poses") or []
            if not poses:
                self._send_json(400, {"ok": False,
                                      "error": "no poses to preview (the viewer sends the drawn path)"})
                return
            try:
                result = ROBOT.scan_preview(
                    poses, incidence_deg=float(payload.get("incidenceDeg", 10.0)))
                self._send_json(200, result)
            except Exception as e:  # noqa: BLE001
                traceback.print_exc()
                self._send_json(500, {"ok": False, "error": str(e)})
            return

        # ---- scan lifecycle ----
        if route == "/api/scan/start":
            self._send_json(200, SCANS.start(payload.get("partId", "")))
            return
        if route == "/api/scan/stop":
            self._send_json(200, SCANS.stop())
            return

        # Unknown POST endpoint.
        self._send_json(404, {"ok": False, "error": "unknown endpoint"})

    def end_headers(self):
        # Dev server: never let the browser cache the GUI files. Stale caches
        # were masking live edits during development (an old broken build kept
        # rendering after the source was fixed). Applies to static + API alike.
        self.send_header("Cache-Control", "no-store, must-revalidate")
        super().end_headers()

    def log_message(self, fmt, *args):
        # Terse one-line access log to stderr (default is noisy). Skip the
        # high-frequency polling reads (status/joints/follow_status) so real
        # events -- moves, errors, the [ROKAE] arm logs -- stay visible.
        msg = fmt % args
        if any(p in msg for p in ("/api/robot/joints", "/api/robot/status",
                                  "/api/robot/follow_status")):
            return
        sys.stderr.write("[qc-backend] %s\n" % msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address (default 127.0.0.1; use 0.0.0.0 to expose on LAN)")
    ap.add_argument("--port", type=int, default=8000, help="port (default 8000)")
    args = ap.parse_args()

    httpd = HTTPServer((args.host, args.port), QCRequestHandler)
    base = f"http://{args.host}:{args.port}"
    print(f"QC Scanner console backend serving {GUI_DIR}")
    print(f"  open   {base}/")
    print(f"  parts  GET  {base}/api/parts")
    print(f"  robot  GET  {base}/api/robot/status | /api/robot/joints")
    print(f"  motion POST {base}/api/robot/{{power,drag,stop,estop,clear_alarm,move}}")
    print(f"  scan   POST {base}/api/scan/start | /api/scan/stop   GET /api/scan/status")
    print(f"  scans  GET  {base}/api/scans")
    print(f"  CAD    {CAD_DIR}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
        httpd.server_close()


if __name__ == "__main__":
    main()
