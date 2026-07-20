#!/usr/bin/env python3
"""
scanpath_convert.py

Remap a scan path from the PART frame into the ARM BASE frame.

plan_path.py produces a ScanPath JSON in the part/CAD frame (millimetres, Y-up).
The arm needs it in its base frame (metres, Z-up, positioned by the marked-corner
calibration). This CLI is the demo-slice tool that does that conversion: it reads
the part-frame JSON, applies the transform built from config, and writes an
arm-frame JSON with the same schema (so the console / RViz / the arm consume the
result the same way they would any ScanPath).

    scripts/scanpath_convert.py IN_partframe.json [OUT_armframe.json]

The transform (mm->m + Y-up->Z-up + the measured corner offset) lives in
libs/path_planning/frame_transform.py; the corner offset comes from
config/local_config.yaml (see architecture.md §6, decision 5). This is the
host-side stand-in for the arm-frame transform the PathPlanner ROS 2 node will
run before MoveIt -- it reuses the very same transform code, so it is TEMPORARY
in the sense of "the CLI wrapper", not the maths.

All pure Python (numpy + PyYAML), runs on the host -- no ROS2/Docker needed.
"""

import argparse
import json
import os
import sys
from pathlib import Path

# libs/ isn't an installed package, so add the repo root to sys.path first
# (same launch shim as plan_path.py).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from libs.path_planning.frame_transform import FrameTransform, transform_scanpath  # noqa: E402
from libs.path_planning.table_clearance import apply_table_clearance  # noqa: E402
from libs.qc_config import load_config  # noqa: E402


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("input", help="part-frame ScanPath JSON (from plan_path.py)")
    parser.add_argument(
        "output",
        nargs="?",
        default=None,
        help="output arm-frame ScanPath JSON (default: <input>_arm.json)",
    )
    parser.add_argument(
        "--config-dir",
        default=None,
        help="config directory (default: the repo's config/)",
    )
    parser.add_argument(
        "--table-clearance-mm",
        type=float,
        default=float(os.environ.get("QC_TABLE_CLEARANCE_MM", "0")),
        help="if > 0, waypoints whose scanner sits within this distance of the "
        "table are DELETED. Default 0 (disabled -- keep all waypoints); set via "
        "QC_TABLE_CLEARANCE_MM or this flag to enable.",
    )
    return parser


def main():
    args = build_arg_parser().parse_args()
    out_path = args.output or str(Path(args.input).with_suffix("")) + "_arm.json"

    with open(args.input) as f:
        data = json.load(f)

    config = load_config(args.config_dir)
    transform = FrameTransform.from_config(config)

    corner = config.get("corner_transform", {})
    if corner.get("translation_m") in (None, [0.0, 0.0, 0.0], [0, 0, 0]) and \
       corner.get("rotation_deg") in (None, [0.0, 0.0, 0.0], [0, 0, 0]):
        print(
            "WARNING: corner_transform is the identity placeholder -- the "
            "marked-corner calibration (decision 5) has not been measured. The "
            "output is in arm units/axes but positioned as if the part corner "
            "sits on the arm base.",
            file=sys.stderr,
        )

    # Table-collision floor (placed frame, before the arm remap): delete any
    # waypoint whose scanner sits within the clearance of the table, so none
    # survive inside it. Flows into both the arm path and (via
    # export_viewer_bundle) the viewer.
    if args.table_clearance_mm and args.table_clearance_mm > 0:
        n_before = len(data.get("waypoints", []))
        data, n_removed = apply_table_clearance(data, clearance_mm=args.table_clearance_mm)
        if n_removed:
            n_after = n_before - n_removed
            print(f"Table-clearance floor: removed {n_removed}/{n_before} waypoint(s) "
                  f"within {args.table_clearance_mm:.0f}mm of the table ({n_after} left)",
                  file=sys.stderr)
            if n_after == 0:
                print("  WARNING: no waypoints left -- the whole scan is within the "
                      "clearance (part shorter than it). Lower QC_TABLE_CLEARANCE_MM.",
                      file=sys.stderr)

    out_data = transform_scanpath(data, transform)

    with open(out_path, "w") as f:
        json.dump(out_data, f, indent=2)

    n = len(out_data.get("waypoints", []))
    print(f"Converted {n} waypoints: {data.get('units')} {data.get('frame', 'part frame')}")
    print(f"                     ->  {out_data['units']} arm base frame")
    print(f"Wrote -> {out_path}")


if __name__ == "__main__":
    main()
