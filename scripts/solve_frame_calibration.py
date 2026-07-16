#!/usr/bin/env python3
"""Solve the sim<->arm frame calibration from captured pose pairs.

Reads data/calibration_poses.json (from capture_pose.py) and:
  1. Runs the vendor-URDF FK (gui/viewer/assets/arm/chain.json) on each pose's
     joint angles to get the link6-origin pose in the arm base frame.
  2. Solves the CONSTANT tool transform (link6 origin -> flange) that best maps
     FK to the controller's cartPosture(flangeInBase). A single rigid tool
     transform explaining every pose == the sim kinematics ARE the controller's.
  3. Reports per-pose residuals (position mm, orientation deg). Small + constant
     => kinematics agree; large/non-constant => a joint sign/offset or DH
     mismatch that no frame transform can hide.

Nothing here touches hardware; it's pure post-processing of the captured data.
"""
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
CHAIN = ROOT / "gui" / "viewer" / "assets" / "arm" / "chain.json"
POSES = ROOT / "data" / "calibration_poses.json"


def rot_axis(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    c, s, C = math.cos(ang), math.sin(ang), 1 - math.cos(ang)
    return np.array([
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def T(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def rpy_to_R(rpy, order="XYZ"):
    rx, ry, rz = rpy
    Rx = rot_axis([1, 0, 0], rx)
    Ry = rot_axis([0, 1, 0], ry)
    Rz = rot_axis([0, 0, 1], rz)
    if order == "XYZ":      # R = Rz@Ry@Rx (fixed-axis XYZ / intrinsic ZYX)
        return Rz @ Ry @ Rx
    return Rx @ Ry @ Rz     # ZYX alt


def fk_link6(chain, q_rad):
    """Transform base_link -> link6 origin, for joint angles q_rad (revolute)."""
    M = np.eye(4)
    qi = 0
    for link in chain["links"]:
        M = M @ T(np.eye(3), link["xyz"])
        if link["type"] == "revolute":
            M = M @ T(rot_axis(link["axis"], q_rad[qi]), [0, 0, 0])
            qi += 1
    return M


def main():
    chain = json.loads(CHAIN.read_text())
    poses = json.loads(POSES.read_text())
    print(f"{len(poses)} poses; chain from {CHAIN.name}\n")

    fk6, ctrl_p, ctrl_rpy, labels = [], [], [], []
    for p in poses:
        q = [math.radians(a) for a in p["joints_deg"]]
        fk6.append(fk_link6(chain, q))
        ctrl_p.append(np.asarray(p["tcp"]["trans"], float))
        ctrl_rpy.append(p["tcp"]["rpy"])
        labels.append(p["label"])

    # --- solve tool TRANSLATION (position only, no rpy needed) ---------------
    # flange_ctrl = link6_pos + R_link6 @ t_tool   ->  t_tool per pose, then avg
    t_tools = []
    for M, pc in zip(fk6, ctrl_p):
        R6, p6 = M[:3, :3], M[:3, 3]
        t_tools.append(R6.T @ (pc - p6))
    t_tools = np.array(t_tools)
    t_tool = t_tools.mean(axis=0)
    print("Per-pose tool translation (link6->flange), mm:")
    for lab, tt in zip(labels, t_tools):
        print(f"  {lab:7s} [{tt[0]*1000:8.2f} {tt[1]*1000:8.2f} {tt[2]*1000:8.2f}]")
    print(f"  MEAN    [{t_tool[0]*1000:8.2f} {t_tool[1]*1000:8.2f} {t_tool[2]*1000:8.2f}]")
    print(f"  stddev  [{t_tools.std(0)[0]*1000:8.2f} {t_tools.std(0)[1]*1000:8.2f} {t_tools.std(0)[2]*1000:8.2f}]  "
          f"(small stddev => kinematics consistent)\n")

    # --- position residuals with the averaged tool translation --------------
    print("Position residual (predicted flange vs controller), mm:")
    errs = []
    for lab, M, pc in zip(labels, fk6, ctrl_p):
        pred = M[:3, 3] + M[:3, :3] @ t_tool
        e = np.linalg.norm(pred - pc) * 1000
        errs.append(e)
        print(f"  {lab:7s} {e:7.2f}")
    errs = np.array(errs)
    print(f"  max {errs.max():.2f} | mean {errs.mean():.2f} | rms {math.sqrt((errs**2).mean()):.2f}\n")

    # --- orientation consistency check (which rpy convention?) ---------------
    for order in ("XYZ", "ZYX"):
        R_tools = []
        for M, rpy in zip(fk6, ctrl_rpy):
            R_tools.append(M[:3, :3].T @ rpy_to_R(rpy, order))
        # spread of R_tool across poses: angle between each and the mean-ish first
        R0 = R_tools[0]
        spread = []
        for R in R_tools:
            dR = R0.T @ R
            ang = math.degrees(math.acos(max(-1, min(1, (np.trace(dR) - 1) / 2))))
            spread.append(ang)
        spread = np.array(spread)
        print(f"orientation: rpy order {order} -> tool-rotation spread max {spread.max():.2f} deg "
              f"(small => this is the right convention & orientation matches)")

    verdict = "MATCH (kinematics agree)" if errs.max() < 2.0 else "MISMATCH — investigate joint signs/DH"
    print(f"\nVERDICT: position max {errs.max():.2f} mm -> {verdict}")


if __name__ == "__main__":
    main()
