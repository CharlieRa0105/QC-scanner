#!/usr/bin/env python3
"""Verify the sim's arm FK matches the controller's cartPosture.

Runs the SAME kinematic chain the viewer uses (gui/viewer/assets/arm/chain.json)
on every captured calibration pose (data/calibration_poses.json) and reports the
flange-position residual against the controller's own cartPosture(flangeInBase).

This is the authoritative check that "sim coordinates == arm coordinates": if the
max residual is small, a joint configuration renders the flange where the real
arm reports it. Pure Python (no browser, no hardware).

    scripts/verify_frame_match.py
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CHAIN = ROOT / "gui" / "viewer" / "assets" / "arm" / "chain.json"
POSES = ROOT / "data" / "calibration_poses.json"
TOL_MM = 3.0  # pass threshold


def rot(axis, a):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return [
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def apply(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def fk_flange(links, q_rad):
    """Flange position in the base frame (matches viewer's nested-pivot FK)."""
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    p = [0.0, 0.0, 0.0]
    qi = 0
    for lk in links:
        off = apply(R, lk["xyz"])
        p = [p[i] + off[i] for i in range(3)]
        if lk["type"] == "revolute":
            R = matmul(R, rot(lk["axis"], q_rad[qi]))
            qi += 1
    return p


def main():
    links = json.loads(CHAIN.read_text())["links"]
    poses = json.loads(POSES.read_text())
    print(f"chain: {CHAIN.relative_to(ROOT)}   poses: {len(poses)}\n")
    print(f"{'pose':8s} {'|err| mm':>9s}")
    errs = []
    for pz in poses:
        q = [math.radians(a) for a in pz["joints_deg"]]
        pred = fk_flange(links, q)
        ctrl = pz["tcp"]["trans"]
        e = math.sqrt(sum((pred[i] - ctrl[i]) ** 2 for i in range(3))) * 1000
        errs.append(e)
        print(f"{pz['label']:8s} {e:9.2f}")
    mx = max(errs)
    print(f"\nmax {mx:.2f} | mean {sum(errs)/len(errs):.2f} | rms {math.sqrt(sum(e*e for e in errs)/len(errs)):.2f} mm")
    ok = mx < TOL_MM
    print(f"VERDICT: {'PASS' if ok else 'FAIL'} (max < {TOL_MM} mm)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
