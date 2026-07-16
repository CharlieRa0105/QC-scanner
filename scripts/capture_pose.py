#!/usr/bin/env python3
"""Capture ONE calibration pose (read-only) for the sim<->arm frame alignment.

Reads the running console's /api/robot/joints — which returns, under a single
lock, both the live joint angles AND the controller's own cartPosture
(flangeInBase) — and appends the paired sample to data/calibration_poses.json.

This commands NO motion. You jog / hand-guide the arm to a pose, then run:

    scripts/capture_pose.py home
    scripts/capture_pose.py pose1
    ... etc

Each call snapshots the CURRENT pose. Aim for ~5-6 poses spread across the
workspace with VARIED wrist orientations (see the checklist printed at the end).

Env:
    QC_CONSOLE_URL   console base URL (default http://127.0.0.1:8000)
"""
import json
import os
import sys
import urllib.request
from datetime import datetime
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "data" / "calibration_poses.json"
URL = os.environ.get("QC_CONSOLE_URL", "http://127.0.0.1:8000").rstrip("/")


def _get(path):
    with urllib.request.urlopen(URL + path, timeout=5) as r:
        return json.loads(r.read().decode())


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else f"pose{_count() + 1}"

    try:
        d = _get("/api/robot/joints")
    except Exception as e:  # noqa: BLE001
        sys.exit(f"ERROR: could not reach the console at {URL} — is it running? ({e})")

    if not d.get("connected"):
        sys.exit("ERROR: arm not connected — connect in the console first (read-only, no motion).")
    joints = d.get("joints") or []
    tcp = d.get("tcp")
    if not joints:
        sys.exit("ERROR: no joint angles reported.")
    if not tcp or tcp.get("trans") is None:
        sys.exit("ERROR: no controller cartPosture (tcp) reported — cannot calibrate without ground truth.\n"
                 "       (Check the SDK build / that cartPosture is supported on this arm.)")

    sample = {
        "label": label,
        "captured_at": datetime.now().isoformat(timespec="seconds"),
        "joints_deg": [j.get("deg") for j in joints],
        "joint_names": [j.get("name") for j in joints],
        "tcp": tcp,  # {'trans':[x,y,z] m, 'rpy':[rx,ry,rz] rad, 'frame':'flangeInBase'}
    }

    data = _load()
    data.append(sample)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(data, indent=2))

    t = tcp["trans"]
    print(f"captured '{label}'  (#{len(data)})")
    print(f"  joints (deg): {[round(x, 2) if isinstance(x,(int,float)) else x for x in sample['joints_deg']]}")
    print(f"  flange trans (mm): [{t[0]*1000:.1f}, {t[1]*1000:.1f}, {t[2]*1000:.1f}]  frame={tcp.get('frame')}")
    print(f"  flange rpy  (deg): {[round(__import__('math').degrees(a),2) for a in tcp['rpy']]}")
    print(f"  -> {OUT}")
    _checklist(data)


def _load():
    if OUT.exists():
        try:
            return json.loads(OUT.read_text())
        except Exception:  # noqa: BLE001
            return []
    return []


def _count():
    return len(_load())


def _checklist(data):
    n = len(data)
    print(f"\n{n} pose(s) captured so far: {', '.join(s['label'] for s in data)}")
    if n < 5:
        print(f"Capture ~{max(0,5-n)} more. For a good solve, span the workspace:")
        print("  - one at HOME (all joints ~0)")
        print("  - spread targets across X, Y and Z (near + far, low + high)")
        print("  - VARY the wrist orientation between poses (not all flange-down)")
        print("  - avoid near-identical or collinear configurations")
    else:
        print("Enough for a solve. Tell me and I'll run the alignment + kinematics check.")


if __name__ == "__main__":
    main()
