#!/usr/bin/env python3
"""
plan_hemisphere.py

PathPlanner entry point using the HEMISPHERE DOME planner (hemisphere_raster.py):
wrap the part in the smallest enclosing hemisphere (flat face on the table) and
raster the scan path over that dome, scanner aimed inward. Same CLI shape + ScanPath
JSON as the other planners, plus a "dome" record (centre + radius in the placed
frame) so export_viewer_bundle can draw the hemisphere in the debug viewport.

Usage:
    python3 scripts/plan_hemisphere.py part.step [out.json] \\
        --standoff-mm 80 --fov-deg 40 --overlap 0.3 --along-track-mm 6
"""

import argparse
import datetime
import json
import math
import sys
from pathlib import Path

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPTS.parent))
sys.path.insert(0, str(_SCRIPTS))

from libs.path_planning.cad_loader import load_cad
from libs.path_planning.hemisphere_raster import generate_hemisphere_waypoints
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
                   help="how far outside the containing dome the scanner sits")
    p.add_argument("--fov-deg", type=float, default=40.0,
                   help="scanner FOV, full angle -- PLACEHOLDER, confirm MIRACO Plus spec")
    p.add_argument("--overlap", type=float, default=0.3)
    p.add_argument("--line-spacing-mm", type=float, default=None,
                   help="override ring spacing directly instead of FOV+overlap")
    p.add_argument("--along-track-mm", type=float, default=6.0)
    p.add_argument("--max-incidence-deg", type=float, default=0.0,
                   help="orientation relaxation cone; 0 = keep every pose aimed EXACTLY at "
                        "the part centroid (the head always points at the part)")
    p.add_argument("--window", type=int, default=2)
    p.add_argument("--mesh-size-mm", type=float, default=5.0)
    p.add_argument("--orient-deg", default="0,0,0",
                   help="operator rotation 'rx,ry,rz' (deg) applied ON TOP of the resting "
                        "placement -- re-plan the dome for a part the operator reoriented")
    return p


def main():
    args = build_arg_parser().parse_args()
    out_path = args.output or str(Path(args.input).with_suffix("")) + "_scanpath.json"

    print(f"[1/3] Loading CAD: {args.input}")
    raw_vertices, faces = load_cad(args.input, mesh_size=args.mesh_size_mm)
    # placement = operator reorientation (rx,ry,rz) composed on top of the automatic
    # resting placement, so re-planning after the operator rotates the part re-fits
    # the dome to the new pose.
    from libs.path_planning.frame_transform import rotation_from_rpy_deg
    rx, ry, rz = (float(a) for a in args.orient_deg.split(","))
    placement_R = np.asarray(rotation_from_rpy_deg(rx, ry, rz)) @ resting_rotation(raw_vertices)
    vertices = apply_rotation(raw_vertices, placement_R)

    # ring spacing from the scanner footprint (arc). Dome standoff is fixed, so a
    # single FOV footprint sets both ring + along-track spacing unless overridden.
    if args.line_spacing_mm is not None:
        spacing = args.line_spacing_mm
    else:
        spacing = raster_spacing_from_fov(args.standoff_mm, args.fov_deg, args.overlap)
    print(f"[2/3] Dome raster spacing = {spacing:.2f}mm, along-track = {args.along_track_mm:.2f}mm")

    waypoints, centre, radius = generate_hemisphere_waypoints(
        vertices,
        standoff_mm=args.standoff_mm,
        line_spacing_mm=spacing,
        along_track_mm=args.along_track_mm,
        up_axis=1,
    )
    n_lines = 1 + max((wp.line_id for wp in waypoints), default=-1)

    print(f"[3/3] Incidence-cone relaxation (limit {args.max_incidence_deg} deg)")
    results = apply_incidence_cone_relaxation(
        waypoints, max_incidence_angle_deg=args.max_incidence_deg, window=args.window)

    out_waypoints = []
    for i, (wp, result) in enumerate(zip(waypoints, results)):
        qx, qy, qz, qw = rotation_matrix_to_quaternion(
            result["x_axis"], result["y_axis"], result["z_axis"])
        target = centre     # scanner looks at the hemisphere centre (the part)
        out_waypoints.append({
            "i": i,
            "position": [round(float(v), 4) for v in result["position"]],
            "quaternion": [qx, qy, qz, qw],
            "target": [round(float(v), 4) for v in target],
            "line_id": int(wp.line_id),
            "incidence_angle_deg": round(float(result["incidence_angle_deg"]), 3),
        })

    out_data = {
        "generator": "plan_hemisphere.py (enclosing-hemisphere dome raster + incidence-cone relaxation)",
        "units": "mm",
        "frame": "part local (CAD units, assumed mm; scanpath_convert.py remaps to arm frame)",
        "standoff_mm": args.standoff_mm,
        "density": {"raster_spacing_mm": round(spacing, 3), "along_track_mm": round(args.along_track_mm, 3)},
        "placement_R": [[round(float(c), 8) for c in row] for row in placement_R],
        # containing hemisphere in the PLACED frame (mm): export_viewer_bundle
        # transforms it into the table frame and the debug viewport draws it.
        "dome": {"center_mm": [round(float(c), 4) for c in centre],
                 "radius_mm": round(float(radius), 4), "up_axis": 1},
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

    print(f"\nWrote {len(out_waypoints)} waypoints across {n_lines} rings -> {out_path}")
    print(f"Containing hemisphere: centre {np.round(centre, 1)} mm, radius {radius:.1f} mm")


if __name__ == "__main__":
    main()
