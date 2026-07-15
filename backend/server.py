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
CAD_DIR = REPO_ROOT / "config" / "cad"

# Real SR5 connection layer (xCore SDK). Lives in this backend dir;
# robot_bridge.py owns the single connection + serialises SDK access, and
# exposes the arm's motion commands (power/drag/jog/stop/clear-alarm).
from robot_bridge import BRIDGE as ROBOT

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
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        route = self.path.split("?")[0]
        # ---- robot state (read-only reads) ----
        if route == "/api/robot/status":
            self._send_json(200, ROBOT.status())
            return
        if route == "/api/robot/joints":
            self._send_json(200, ROBOT.joints())
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

        # ---- robot MOTION (drives the physical SR5) ----
        # Each returns the fresh status dict tagged {ok, action, error?}.
        if route in ("/api/robot/power", "/api/robot/drag", "/api/robot/stop",
                     "/api/robot/estop", "/api/robot/clear_alarm", "/api/robot/move"):
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
                else:  # /api/robot/move
                    result = ROBOT.move_joints(payload.get("joints", []),
                                               payload.get("speedMms"))
                self._send_json(200, result)
            except Exception as e:  # noqa: BLE001
                traceback.print_exc()
                self._send_json(500, {"ok": False, "action": route.rsplit("/", 1)[-1],
                                      "error": str(e)})
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
