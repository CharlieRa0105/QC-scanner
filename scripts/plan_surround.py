#!/usr/bin/env python3
"""
plan_surround.py

PathPlanner entry point using the MULTI-VIEW lawnmower (multiview_lawnmower.py) --
full-surround coverage: lawnmower passes on EVERY side of the part, not just the top.
Same CLI shape and ScanPath JSON output as the other planners, so
scanpath_convert.py and export_viewer_bundle.py consume it unchanged.

    cad_loader.load_cad                              CAD -> triangle mesh (gmsh)
    placement.resting_rotation                       lay part in a consistent frame
    multiview_lawnmower.generate_multiview_waypoints raster all sides -> Waypoints
    incidence_cone_modifier.apply_...                relax orientations, pass by pass
    -> write ScanPath JSON (part frame, mm)

Reachability is deliberately ignored (a separate concern): this planner covers the
whole surface; which waypoints the arm can actually reach is decided downstream.

Usage:
    python3 scripts/plan_surround.py part.step [out.json] \\
        --standoff-mm 80 --fov-deg 40 --overlap 0.3 --along-track-mm 3 \\
        --target-waypoints 800 --n-views 6
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
from libs.path_planning.incidence_cone_modifier import apply_incidence_cone_relaxation
from libs.path_planning.multiview_lawnmower import generate_multiview_waypoints
from libs.path_planning.normal_estimation import surface_area
from libs.path_planning.placement import apply_rotation, resting_rotation
from libs.path_planning.waypoint_generator import raster_spacing_from_fov
from plan_path import rotation_matrix_to_quaternion


def build_arg_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("input", help="CAD file: .step/.stp/.stl/.obj")
    p.add_argument("output", nargs="?", default=None,
                   help="output ScanPath JSON (default: <input>_scanpath.json)")
    p.add_argument("--standoff-mm", type=float, default=80.0)
    p.add_argument("--fov-deg", type=float, default=40.0,
                   help="scanner FOV, full angle -- PLACEHOLDER, confirm MIRACO Plus spec")
    p.add_argument("--overlap", type=float, default=0.3)
    p.add_argument("--line-spacing-mm", type=float, default=None,
                   help="override pass spacing directly instead of FOV+overlap")
    p.add_argument("--along-track-mm", type=float, default=3.0)
    p.add_argument("--target-waypoints", type=int, default=None,
                   help="scale density to surface area (spacing = sqrt(area/target), used "
                        "where finer than FOV/along-track). Full-surround needs more than a "
                        "single-side sweep -- try ~800+.")
    p.add_argument("--min-spacing-mm", type=float, default=2.0)
    p.add_argument("--n-views", type=int, default=6,
                   help="number of view directions (6 = axis-aligned, best for boxy parts; "
                        "more = better square-on coverage of rounded parts)")
    p.add_argument("--max-incidence-deg", type=float, default=25.0)
    p.add_argument("--window", type=int, default=2)
    p.add_argument("--mesh-size-mm", type=float, default=3.0)
    return p


def main():
    args = build_arg_parser().parse_args()
    out_path = args.output or str(Path(args.input).with_suffix("")) + "_scanpath.json"

    print(f"[1/3] Loading CAD: {args.input}")
    raw_vertices, faces = load_cad(args.input, mesh_size=args.mesh_size_mm)
    placement_R = resting_rotation(raw_vertices)
    vertices = apply_rotation(raw_vertices, placement_R)
    bbox = np.round(vertices.max(axis=0) - vertices.min(axis=0), 1)
    print(f"      {len(vertices)} verts, {len(faces)} faces; placed bbox = {bbox} mm")

    if args.line_spacing_mm is not None:
        spacing, src = args.line_spacing_mm, "override"
    else:
        spacing = raster_spacing_from_fov(args.standoff_mm, args.fov_deg, args.overlap)
        src = f"standoff={args.standoff_mm}, fov={args.fov_deg}, overlap={args.overlap}"
    along = args.along_track_mm
    if args.target_waypoints:
        area = surface_area(vertices, faces)
        s_area = max(args.min_spacing_mm, math.sqrt(area / args.target_waypoints))
        spacing, along = min(spacing, s_area), min(along, s_area)
        src += f"; area={area:.0f}mm^2 target={args.target_waypoints} -> {s_area:.2f}mm"
    print(f"[2/3] Line spacing = {spacing:.2f}mm, along-track = {along:.2f}mm ({src})")

    waypoints = generate_multiview_waypoints(
        vertices, faces,
        standoff_mm=args.standoff_mm,
        line_spacing_mm=spacing,
        along_track_mm=along,
        n_views=args.n_views,
    )
    n_lines = 1 + max((wp.line_id for wp in waypoints), default=-1)

    print(f"[3/3] Incidence-cone relaxation (limit {args.max_incidence_deg} deg)")
    results = apply_incidence_cone_relaxation(
        waypoints, max_incidence_angle_deg=args.max_incidence_deg, window=args.window)

    out_waypoints = []
    for i, (wp, result) in enumerate(zip(waypoints, results)):
        qx, qy, qz, qw = rotation_matrix_to_quaternion(
            result["x_axis"], result["y_axis"], result["z_axis"])
        target = wp.position - wp.normal * args.standoff_mm
        out_waypoints.append({
            "i": i,
            "position": [round(float(v), 4) for v in result["position"]],
            "quaternion": [qx, qy, qz, qw],
            "target": [round(float(v), 4) for v in target],
            "line_id": int(wp.line_id),
            "incidence_angle_deg": round(float(result["incidence_angle_deg"]), 3),
        })

    out_data = {
        "generator": "plan_surround.py (multi-view lawnmower, full-surround + incidence-cone relaxation)",
        "units": "mm",
        "frame": "part local (CAD units, assumed mm; scanpath_convert.py remaps to arm frame)",
        "standoff_mm": args.standoff_mm,
        "density": {"raster_spacing_mm": round(spacing, 3), "along_track_mm": round(along, 3)},
        "placement_R": [[round(float(c), 8) for c in row] for row in placement_R],
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "source_cad": str(args.input),
        "params": {
            "fov_deg": args.fov_deg, "overlap": args.overlap, "n_views": args.n_views,
            "max_incidence_deg": args.max_incidence_deg, "window": args.window,
            "target_waypoints": args.target_waypoints, "mesh_size_mm": args.mesh_size_mm,
        },
        "waypoints": out_waypoints,
    }
    with open(out_path, "w") as f:
        json.dump(out_data, f, indent=2)

    max_inc = max((w["incidence_angle_deg"] for w in out_waypoints), default=0.0)
    print(f"\nWrote {len(out_waypoints)} waypoints across {n_lines} passes -> {out_path}")
    print(f"Max incidence angle used: {max_inc:.2f} deg (limit {args.max_incidence_deg})")


if __name__ == "__main__":
    main()
