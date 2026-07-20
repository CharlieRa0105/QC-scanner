#!/usr/bin/env python3
"""
plan_box.py

PathPlanner entry point using the TABLE-ALIGNED BOX planner (box_raster.py): wrap
the part in the smallest box that sits flat on the table (one face on the table,
vertical sides; footprint yaw minimised) and raster a lawnmower path over EACH of
its six faces, scanner aimed square-on inward. Same CLI shape + ScanPath JSON as
the other planners, plus a "box" record (centre + orientation + half-extents in
the placed frame) so export_viewer_bundle can draw the box in the viewport.

Usage:
    python3 scripts/plan_box.py part.step [out.json] \\
        --standoff-mm 80 --fov-deg 40 --overlap 0.3 --along-track-mm 6
"""

import argparse
import datetime
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPTS.parent))
sys.path.insert(0, str(_SCRIPTS))

from libs.path_planning.box_raster import generate_box_waypoints
from libs.path_planning.cad_loader import load_cad
from libs.path_planning.frame_transform import matrix_to_quaternion, rotation_from_rpy_deg
from libs.path_planning.incidence_cone_modifier import apply_incidence_cone_relaxation
from libs.path_planning.placement import apply_rotation, resting_rotation
from libs.path_planning.waypoint_generator import raster_spacing_from_fov
from plan_path import rotation_matrix_to_quaternion


def build_arg_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("input")
    p.add_argument("output", nargs="?", default=None)
    p.add_argument("--standoff-mm", type=float, default=80.0,
                   help="how far outside each box face the scanner sits")
    p.add_argument("--fov-deg", type=float, default=40.0,
                   help="scanner FOV, full angle -- PLACEHOLDER, confirm MIRACO Plus spec")
    p.add_argument("--overlap", type=float, default=0.3)
    p.add_argument("--line-spacing-mm", type=float, default=None,
                   help="override raster-row spacing directly instead of FOV+overlap")
    p.add_argument("--along-track-mm", type=float, default=6.0)
    p.add_argument("--max-incidence-deg", type=float, default=0.0,
                   help="orientation relaxation cone; 0 = keep every pose aimed EXACTLY "
                        "square-on at its face")
    p.add_argument("--window", type=int, default=2)
    p.add_argument("--mesh-size-mm", type=float, default=5.0)
    p.add_argument("--min-clearance-mm", type=float,
                   default=float(os.environ.get("QC_BOX_CLEARANCE_MM", "150")),
                   help="raise any waypoint whose scanner sits below this height above "
                        "the table UP to it (kept aiming at the part), so the head clears "
                        "the table. 0 disables. Default 150 (15 cm).")
    p.add_argument("--orient-deg", default="0,0,0",
                   help="operator rotation 'rx,ry,rz' (deg) applied ON TOP of the resting "
                        "placement -- re-fit the box for a part the operator reoriented")
    return p


def main():
    args = build_arg_parser().parse_args()
    out_path = args.output or str(Path(args.input).with_suffix("")) + "_scanpath.json"

    print(f"[1/3] Loading CAD: {args.input}")
    raw_vertices, faces = load_cad(args.input, mesh_size=args.mesh_size_mm)
    # placement = operator reorientation composed on the automatic resting placement,
    # so re-planning after the operator rotates the part re-fits the box to the pose.
    rx, ry, rz = (float(a) for a in args.orient_deg.split(","))
    placement_R = np.asarray(rotation_from_rpy_deg(rx, ry, rz)) @ resting_rotation(raw_vertices)
    vertices = apply_rotation(raw_vertices, placement_R)

    if args.line_spacing_mm is not None:
        spacing = args.line_spacing_mm
    else:
        spacing = raster_spacing_from_fov(args.standoff_mm, args.fov_deg, args.overlap)
    print(f"[2/3] Face raster spacing = {spacing:.2f}mm, along-track = {args.along_track_mm:.2f}mm")

    waypoints, centre, R, half = generate_box_waypoints(
        vertices,
        standoff_mm=args.standoff_mm,
        line_spacing_mm=spacing,
        along_track_mm=args.along_track_mm,
    )
    n_lines = 1 + max((wp.line_id for wp in waypoints), default=-1)

    print(f"[3/3] Incidence-cone relaxation (limit {args.max_incidence_deg} deg)")
    results = apply_incidence_cone_relaxation(
        waypoints, max_incidence_angle_deg=args.max_incidence_deg, window=args.window)

    out_waypoints = []
    for i, (wp, result) in enumerate(zip(waypoints, results)):
        qx, qy, qz, qw = rotation_matrix_to_quaternion(
            result["x_axis"], result["y_axis"], result["z_axis"])
        target = wp.position + wp.normal * args.standoff_mm   # the point on the face
        out_waypoints.append({
            "i": i,
            "position": [round(float(v), 4) for v in result["position"]],
            "quaternion": [qx, qy, qz, qw],
            "target": [round(float(v), 4) for v in target],
            "line_id": int(wp.line_id),
            "incidence_angle_deg": round(float(result["incidence_angle_deg"]), 3),
        })

    # Table clearance: raise any waypoint whose scanner sits below the clearance up
    # to it, then re-aim from the lifted position at the SAME face point -- so the
    # head clears the table while still scanning the part. Placed frame is +Y up,
    # table = the part's lowest Y; a box has side/bottom-face waypoints near/below
    # the table, which this lifts.
    if args.min_clearance_mm and args.min_clearance_mm > 0:
        floor = float(vertices[:, 1].min()) + args.min_clearance_mm
        n_lifted = 0
        for w in out_waypoints:
            if w["position"][1] >= floor:
                continue
            w["position"][1] = round(floor, 4)
            pos = np.asarray(w["position"], dtype=float)
            look = np.asarray(w["target"], dtype=float) - pos
            nz = float(np.linalg.norm(look))
            if nz > 1e-9:
                z = look / nz
                ref = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
                x = np.cross(ref, z); x /= np.linalg.norm(x)
                y = np.cross(z, x)
                w["quaternion"] = list(rotation_matrix_to_quaternion(x, y, z))
            n_lifted += 1
        print(f"Table clearance: raised {n_lifted}/{len(out_waypoints)} waypoint(s) to "
              f"{args.min_clearance_mm:.0f}mm above the table")

    # Fitted box in the PLACED frame (mm): centre, orientation (quaternion, box
    # axes = R columns), half-extents. export_viewer_bundle transforms it into the
    # table frame and the viewport draws the oriented box.
    bqx, bqy, bqz, bqw = matrix_to_quaternion(R)
    out_data = {
        "generator": "plan_box.py (table-aligned box face raster + incidence-cone relaxation)",
        "units": "mm",
        "frame": "part local (CAD units, assumed mm; scanpath_convert.py remaps to arm frame)",
        "standoff_mm": args.standoff_mm,
        "density": {"raster_spacing_mm": round(spacing, 3), "along_track_mm": round(args.along_track_mm, 3)},
        "placement_R": [[round(float(c), 8) for c in row] for row in placement_R],
        # placed-frame table plane (+Y up); read by scanpath_convert's clearance guard.
        "table_up_mm": round(float(vertices[:, 1].min()), 4),
        "box": {"center_mm": [round(float(c), 4) for c in centre],
                "half_dims_mm": [round(float(h), 4) for h in half],
                "quaternion": [float(bqx), float(bqy), float(bqz), float(bqw)]},
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "source_cad": str(args.input),
        "params": {
            "fov_deg": args.fov_deg, "overlap": args.overlap,
            "max_incidence_deg": args.max_incidence_deg, "window": args.window,
            "mesh_size_mm": args.mesh_size_mm,
        },
        "waypoints": out_waypoints,
    }
    with open(out_path, "w") as f:
        json.dump(out_data, f, indent=2)

    print(f"\nWrote {len(out_waypoints)} waypoints across {n_lines} rows (6 faces) -> {out_path}")
    print(f"Table-aligned box: half-extents {np.round(half, 1)} mm, volume {8 * float(np.prod(half)) / 1e3:.1f} cm^3")


if __name__ == "__main__":
    main()
