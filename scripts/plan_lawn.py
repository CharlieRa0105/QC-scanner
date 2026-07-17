#!/usr/bin/env python3
"""
plan_lawn.py

PathPlanner entry point using the 2.5D LAWNMOWER top-surface raster
(lawnmower_raster.py) -- the continuous back-and-forth sweep over the presented
face (refPath.jpg). Drop-in alternative to plan_path.py / plan_contour.py: same
CLI shape and same ScanPath JSON output, so scanpath_convert.py and
export_viewer_bundle.py consume it unchanged.

    cad_loader.load_cad                          CAD -> triangle mesh (via gmsh)
    lawnmower_raster.generate_lawnmower_waypoints down-raycast lawnmower Waypoints
    incidence_cone_modifier.apply_...            relax orientations, pass by pass
    -> write ScanPath JSON (part frame, mm)

Line spacing comes from scanner FOV + overlap (raster_spacing_from_fov) and can be
tightened to the part's surface area (--target-waypoints), exactly like the other
planners, so small parts stay densely covered.

Usage:
    python3 scripts/plan_lawn.py part.step [out.json] \\
        --standoff-mm 80 --fov-deg 40 --overlap 0.3 --along-track-mm 3 \\
        --target-waypoints 400 --scan-dir-deg 0
"""

import argparse
import datetime
import json
import math
import sys
from pathlib import Path

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPTS.parent))   # repo root, for libs.*
sys.path.insert(0, str(_SCRIPTS))          # scripts/, to reuse plan_path helpers

from libs.path_planning.cad_loader import load_cad
from libs.path_planning.incidence_cone_modifier import apply_incidence_cone_relaxation
from libs.path_planning.lawnmower_raster import generate_lawnmower_waypoints
from libs.path_planning.normal_estimation import surface_area
from libs.path_planning.placement import apply_rotation, resting_rotation
from libs.path_planning.waypoint_generator import raster_spacing_from_fov
from plan_path import rotation_matrix_to_quaternion   # reuse Shepperd's method


def build_arg_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("input", help="CAD file: .step/.stp/.stl/.obj")
    p.add_argument("output", nargs="?", default=None,
                   help="output ScanPath JSON (default: <input>_scanpath.json)")
    p.add_argument("--standoff-mm", type=float, default=80.0)
    p.add_argument("--fov-deg", type=float, default=40.0,
                   help="scanner FOV, full angle -- PLACEHOLDER, confirm MIRACO Plus spec")
    p.add_argument("--overlap", type=float, default=0.3,
                   help="required overlap between adjacent passes (0..1)")
    p.add_argument("--line-spacing-mm", type=float, default=None,
                   help="override pass spacing directly instead of FOV+overlap")
    p.add_argument("--along-track-mm", type=float, default=3.0,
                   help="waypoint spacing along each pass")
    p.add_argument("--target-waypoints", type=int, default=None,
                   help="scale density to surface area: spacing = sqrt(area/target), used "
                        "where finer than the FOV/along-track spacing (densifies small parts)")
    p.add_argument("--min-spacing-mm", type=float, default=2.0,
                   help="floor on the area-derived spacing (only with --target-waypoints)")
    p.add_argument("--scan-dir-deg", type=float, default=0.0,
                   help="pass direction in the table plane (0 = along +X, stepping +Y)")
    p.add_argument("--hole-drop-frac", type=float, default=0.3,
                   help="bridge over samples deviating more than this fraction of part height "
                        "from their neighbours (holes/edges); 0 disables")
    p.add_argument("--max-surface-tilt-deg", type=float, default=60.0,
                   help="drop points whose surface normal tilts more than this from up "
                        "(near-vertical faces the overhead scanner can't see -> flip pass)")
    p.add_argument("--max-incidence-deg", type=float, default=25.0)
    p.add_argument("--window", type=int, default=2)
    p.add_argument("--mesh-size-mm", type=float, default=3.0,
                   help="STEP tessellation target edge length (finer = smoother surface)")
    return p


def main():
    args = build_arg_parser().parse_args()
    out_path = args.output or str(Path(args.input).with_suffix("")) + "_scanpath.json"

    print(f"[1/3] Loading CAD: {args.input}")
    raw_vertices, faces = load_cad(args.input, mesh_size=args.mesh_size_mm)
    bbox = np.round(raw_vertices.max(axis=0) - raw_vertices.min(axis=0), 1)
    # Lay the part on its largest face (smallest-extent axis -> +Y) so the scanner
    # sweeps the broad up-facing surface, not an end. The SAME rotation is recorded
    # and re-applied to the mesh in export_viewer_bundle so path and mesh stay aligned.
    placement_R = resting_rotation(raw_vertices)
    vertices = apply_rotation(raw_vertices, placement_R)
    print(f"      {len(vertices)} verts, {len(faces)} faces; raw bbox = {bbox} mm; "
          f"placed bbox = {np.round(vertices.max(axis=0) - vertices.min(axis=0), 1)} mm")

    if args.line_spacing_mm is not None:
        spacing = args.line_spacing_mm
        src = "override"
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

    waypoints = generate_lawnmower_waypoints(
        vertices, faces,
        standoff_mm=args.standoff_mm,
        line_spacing_mm=spacing,
        along_track_mm=along,
        scan_dir_deg=args.scan_dir_deg,
        up_axis=1,                     # planned in the placed frame: +Y is up
        hole_drop_frac=args.hole_drop_frac,
        max_surface_tilt_deg=args.max_surface_tilt_deg,
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
        "generator": "plan_lawn.py (2.5D lawnmower top-surface raster + incidence-cone relaxation)",
        "units": "mm",
        "frame": "part local (CAD units, assumed mm; scanpath_convert.py remaps to arm frame)",
        "standoff_mm": args.standoff_mm,
        "density": {"raster_spacing_mm": round(spacing, 3), "along_track_mm": round(along, 3)},
        # resting placement applied to the mesh before planning; export_viewer_bundle
        # re-applies it so the displayed mesh matches the path (identity if absent).
        "placement_R": [[round(float(c), 8) for c in row] for row in placement_R],
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "source_cad": str(args.input),
        "params": {
            "fov_deg": args.fov_deg, "overlap": args.overlap,
            "max_incidence_deg": args.max_incidence_deg, "window": args.window,
            "target_waypoints": args.target_waypoints, "scan_dir_deg": args.scan_dir_deg,
            "hole_drop_frac": args.hole_drop_frac, "mesh_size_mm": args.mesh_size_mm,
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
