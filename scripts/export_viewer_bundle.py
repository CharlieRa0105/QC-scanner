#!/usr/bin/env python3
"""
export_viewer_bundle.py

Produce data/viewer_bundle.json for the 3D scan-path viewer (gui/viewer/): the
PART MESH + the SCAN PATH, both in the arm cell's `table` frame (metres, Z-up),
plus the overhead-mount geometry. Everything reuses the project's own planner
(libs/path_planning) + frame transform -- no viewer-specific path maths.

    scripts/export_viewer_bundle.py <cad> <scanpath_arm.json> [out.json]

  <cad>              the part CAD (config/cad/*.STEP)
  <scanpath_arm.json> arm-frame ScanPath from scanpath_convert.py
  out.json           default data/viewer_bundle.json
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from libs.path_planning.cad_loader import load_cad          # noqa: E402
from libs.path_planning.frame_transform import FrameTransform  # noqa: E402
from libs.qc_config import load_config                       # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cad")
    ap.add_argument("scanpath_arm", help="arm-frame ScanPath JSON (scanpath_convert.py output)")
    ap.add_argument("output", nargs="?", default="data/viewer_bundle.json")
    ap.add_argument("--mesh-size-mm", type=float, default=5.0)
    args = ap.parse_args()

    cfg = load_config()
    ft = FrameTransform.from_config(cfg)

    # Part mesh in the part frame (mm), transformed into the table frame (m) with
    # the SAME transform PathPlanner uses -- so the mesh and the path line up.
    verts, faces = load_cad(args.cad, mesh_size=args.mesh_size_mm)
    part_vertices = [[round(float(c), 6) for c in ft.apply_point(v)] for v in verts]
    triangles = [[int(t[0]), int(t[1]), int(t[2])] for t in faces]

    sp = json.load(open(args.scanpath_arm))
    if sp.get("units") != "m":
        raise SystemExit(f"{args.scanpath_arm} is not in arm frame (units={sp.get('units')!r}); "
                         "run scanpath_convert.py first")

    bundle = {
        "units": "m",
        "frame": "table",
        "mount": {"base_height_m": 1.2,
                  "note": "SR5 overhead: base 1.2 m above the table, pointing down"},
        "standoff_m": sp.get("standoff_m") or round(sp.get("standoff_mm", 250) / 1000.0, 6),
        "part": {"vertices": part_vertices, "triangles": triangles},
        "waypoints": sp.get("waypoints", []),
    }
    with open(args.output, "w") as f:
        json.dump(bundle, f)
    print(f"wrote {args.output}: part {len(part_vertices)} verts / {len(triangles)} tris, "
          f"{len(bundle['waypoints'])} waypoints")


if __name__ == "__main__":
    main()
