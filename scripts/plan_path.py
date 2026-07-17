#!/usr/bin/env python3
"""
plan_path.py

PathPlanner entry point (the R&D pure-Python stand-in -- see
docs/running_the_planner.md and the Coverage Path Planning design doc for
the full production-vs-R&D picture). Takes a CAD file in, writes a
ScanPath JSON out.

Full pipeline, all pure Python, runs on the host -- no ROS2/Docker/sim
needed for this step:

    cad_loader.load_cad                    CAD file -> triangle mesh (via gmsh)
    normal_estimation.sample_surface       mesh -> surface points + outward normals
    waypoint_generator.raster_spacing_from_fov
                                            FOV + overlap -> raster line spacing
    waypoint_generator.generate_raster_waypoints
                                            points -> ordered raster Waypoints
    incidence_cone_modifier.apply_incidence_cone_relaxation
                                            relax orientations, line by line, into
                                            the incidence cone
    -> write ScanPath JSON

The output JSON is in the CAD file's own units (mm, per the standard CAD
convention) and the CAD file's own local frame -- scanpath_convert.py is
the separate tool responsible for remapping mm -> m and Y-up -> Z-up into
the arm's frame for RViz / the sim, same as it already does for the
existing scanpath_example.json.

Dependencies: gmsh, numpy   (pip install gmsh numpy)

Usage:
    python3 scripts/plan_path.py part.step
    python3 scripts/plan_path.py part.step out.json \\
        --standoff-mm 300 --fov-deg 40 --overlap 0.3 \\
        --along-track-mm 10 --max-incidence-deg 25 \\
        --step-axis 1 --travel-axis 0 --samples 20000 --mesh-size-mm 5
"""

import argparse
import datetime
import json
import math
import sys
from pathlib import Path

import numpy as np

# libs/ isn't installed as a package (no setup.py yet), so make it
# importable by adding the project root to sys.path before importing --
# this file lives in scripts/, so the project root is one level up.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from libs.path_planning.cad_loader import load_cad
from libs.path_planning.incidence_cone_modifier import apply_incidence_cone_relaxation
from libs.path_planning.normal_estimation import sample_surface, surface_area
from libs.path_planning.waypoint_generator import (
    generate_raster_waypoints,
    raster_spacing_from_fov,
)


def rotation_matrix_to_quaternion(x_axis, y_axis, z_axis):
    """
    Convert a right-handed orientation frame (three orthonormal column
    vectors) into a quaternion (qx, qy, qz, qw).

    Uses Shepperd's method (branch on which diagonal term of the rotation
    matrix is largest, to avoid dividing by a near-zero term) -- a
    standard closed-form rotation-matrix-to-quaternion conversion that
    needs no external dependency (no scipy).
    """
    m = np.column_stack([x_axis, y_axis, z_axis])
    trace = np.trace(m)

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (m[2, 1] - m[1, 2]) * s
        qy = (m[0, 2] - m[2, 0]) * s
        qz = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s

    return (float(qx), float(qy), float(qz), float(qw))


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("input", help="CAD file: .step/.stp/.stl/.obj")
    parser.add_argument(
        "output",
        nargs="?",
        default=None,
        help="output ScanPath JSON (default: <input>_scanpath.json)",
    )
    parser.add_argument(
        "--standoff-mm", type=float, default=300.0,
        help="probe standoff from surface (config default: 300)",
    )
    parser.add_argument(
        "--fov-deg", type=float, default=40.0,
        help="scanner field of view, full angle -- PLACEHOLDER, confirm MIRACO Plus spec",
    )
    parser.add_argument(
        "--overlap", type=float, default=0.3,
        help="required overlap between adjacent raster lines (0..1)",
    )
    parser.add_argument(
        "--raster-spacing-mm", type=float, default=None,
        help="override raster line spacing directly instead of deriving it from FOV+overlap",
    )
    parser.add_argument(
        "--along-track-mm", type=float, default=10.0,
        help="waypoint spacing along each raster line (upper bound; --target-waypoints "
        "may make it finer for a small part)",
    )
    parser.add_argument(
        "--target-waypoints", type=int, default=None,
        help="scale raster density to the part's SURFACE AREA: derive a base spacing "
        "of sqrt(area / target) and use it wherever it is FINER than the FOV/along-track "
        "spacing. Densifies small parts (so they don't collapse to one line/face) without "
        "ever coarsening a large part or violating scan overlap. Unset = fixed spacing.",
    )
    parser.add_argument(
        "--min-spacing-mm", type=float, default=2.0,
        help="floor on the area-derived spacing, so a tiny part can't demand an absurd "
        "waypoint count (only used with --target-waypoints)",
    )
    parser.add_argument(
        "--max-incidence-deg", type=float, default=25.0,
        help="incidence-cone half-angle limit (the relaxation constraint)",
    )
    parser.add_argument(
        "--window", type=int, default=2,
        help="normal-smoothing sliding-window half-width",
    )
    parser.add_argument(
        "--samples", type=int, default=20000,
        help="surface sample count (density of the source point cloud)",
    )
    parser.add_argument(
        "--mesh-size-mm", type=float, default=5.0,
        help="STEP tessellation target edge length -- finer (smaller) gives a more "
        "accurate mesh at the cost of more triangles to process",
    )
    parser.add_argument(
        "--face-angle-tol-deg", type=float, default=30.0,
        help="normal-clustering tolerance: samples whose normals fall within this "
        "angle share a face group (smaller = more, tighter faces)",
    )
    parser.add_argument(
        "--min-group-frac", type=float, default=0.02,
        help="minimum face-group size as a fraction of total samples; smaller "
        "groups (fillet/chamfer fragments) are merged into the nearest face "
        "(0 disables merging)",
    )
    parser.add_argument(
        "--min-line-points", type=int, default=2,
        help="drop raster lines with fewer than this many points (isolated dots, "
        "not sweeps)",
    )
    parser.add_argument("--seed", type=int, default=0, help="surface-sampling RNG seed")
    return parser


def main():
    args = build_arg_parser().parse_args()

    out_path = args.output or str(Path(args.input).with_suffix("")) + "_scanpath.json"

    print(f"[1/4] Loading CAD: {args.input}")
    vertices, faces = load_cad(args.input, mesh_size=args.mesh_size_mm)
    bbox_min = vertices.min(axis=0)
    bbox_max = vertices.max(axis=0)
    print(
        f"      {len(vertices)} verts, {len(faces)} faces; "
        f"bbox size = {np.round(bbox_max - bbox_min, 1)} (mm)"
    )

    print(f"[2/4] Sampling {args.samples} surface points + normals")
    points, normals = sample_surface(vertices, faces, n_samples=args.samples, seed=args.seed)

    if args.raster_spacing_mm is not None:
        spacing = args.raster_spacing_mm
        print(f"[3/4] Raster spacing = {spacing:.2f}mm (override)")
    else:
        spacing = raster_spacing_from_fov(args.standoff_mm, args.fov_deg, args.overlap)
        print(
            f"[3/4] Raster spacing = {spacing:.2f}mm "
            f"(from standoff={args.standoff_mm}, fov={args.fov_deg}, overlap={args.overlap})"
        )

    # Surface-area-based density: derive a base spacing from the part's own area so
    # small parts stay densely covered instead of collapsing to one line per face.
    # Only ever makes spacing FINER (min against the coverage/along-track spacing),
    # so a large part is unchanged and scan overlap is never reduced.
    along_track = args.along_track_mm
    if args.target_waypoints:
        area = surface_area(vertices, faces)
        s_area = max(args.min_spacing_mm, math.sqrt(area / args.target_waypoints))
        new_spacing = min(spacing, s_area)
        new_along = min(along_track, s_area)
        print(
            f"      area={area:.0f}mm^2, target={args.target_waypoints} wp -> "
            f"area spacing {s_area:.2f}mm; using line={new_spacing:.2f}mm "
            f"(was {spacing:.2f}), along-track={new_along:.2f}mm (was {along_track:.2f})"
        )
        spacing, along_track = new_spacing, new_along

    waypoints = generate_raster_waypoints(
        points,
        normals,
        standoff_mm=args.standoff_mm,
        raster_spacing_mm=spacing,
        along_track_mm=along_track,
        face_angle_tol_deg=args.face_angle_tol_deg,
        min_group_frac=args.min_group_frac,
        min_line_points=args.min_line_points,
    )
    n_lines = 1 + max((wp.line_id for wp in waypoints), default=-1)
    print(f"      {len(waypoints)} waypoints across {n_lines} raster lines")

    print(f"[4/4] Incidence-cone relaxation (limit {args.max_incidence_deg} deg, window {args.window})")
    results = apply_incidence_cone_relaxation(
        waypoints, max_incidence_angle_deg=args.max_incidence_deg, window=args.window
    )

    out_waypoints = []
    for i, (wp, result) in enumerate(zip(waypoints, results)):
        qx, qy, qz, qw = rotation_matrix_to_quaternion(
            result["x_axis"], result["y_axis"], result["z_axis"]
        )
        # The scan target is the original surface point, recovered from
        # the (pre-relaxation) probe position and true normal -- not the
        # relaxed orientation, since relaxation only ever changes where
        # the probe is FACING, never where it physically sits.
        target = wp.position - wp.normal * args.standoff_mm
        out_waypoints.append(
            {
                "i": i,
                "position": [round(float(v), 4) for v in result["position"]],
                "quaternion": [qx, qy, qz, qw],
                "target": [round(float(v), 4) for v in target],
                "line_id": int(wp.line_id),
                "incidence_angle_deg": round(float(result["incidence_angle_deg"]), 3),
            }
        )

    max_incidence_used = max((w["incidence_angle_deg"] for w in out_waypoints), default=0.0)
    out_data = {
        "generator": "plan_path.py (pure-Python raster + incidence-cone relaxation)",
        "units": "mm",
        "frame": "part local (CAD units, assumed mm; scanpath_convert.py remaps to arm frame)",
        "standoff_mm": args.standoff_mm,
        "density": {
            "raster_spacing_mm": round(spacing, 3),
            "along_track_mm": round(along_track, 3),
        },
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "source_cad": str(args.input),
        "params": {
            "fov_deg": args.fov_deg,
            "overlap": args.overlap,
            "max_incidence_deg": args.max_incidence_deg,
            "window": args.window,
            "face_angle_tol_deg": args.face_angle_tol_deg,
            "min_group_frac": args.min_group_frac,
            "min_line_points": args.min_line_points,
            "target_waypoints": args.target_waypoints,
            "samples": args.samples,
            "mesh_size_mm": args.mesh_size_mm,
            "seed": args.seed,
        },
        "waypoints": out_waypoints,
    }

    with open(out_path, "w") as f:
        json.dump(out_data, f, indent=2)

    print()
    print(f"Wrote {len(out_waypoints)} waypoints -> {out_path}")
    print(f"Max incidence angle used: {max_incidence_used:.2f} deg (limit {args.max_incidence_deg})")
    print()
    print("Next: visualise it -> python3 scripts/visualize_path.py " + out_path + "   (inside the sim container)")


if __name__ == "__main__":
    main()
