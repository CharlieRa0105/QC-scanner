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

    import numpy as np

    cfg = load_config()
    ft = FrameTransform.from_config(cfg)

    sp = json.load(open(args.scanpath_arm))
    if sp.get("units") != "m":
        raise SystemExit(f"{args.scanpath_arm} is not in arm frame (units={sp.get('units')!r}); "
                         "run scanpath_convert.py first")

    # Part mesh in the part frame (mm) -> table frame (m). If the planner recorded a
    # resting placement (placement_R, e.g. the lawnmower laying the part on its
    # largest face), apply the SAME rotation to the mesh FIRST, then the shared
    # part->arm transform -- otherwise the displayed mesh and the path would drift.
    verts, faces = load_cad(args.cad, mesh_size=args.mesh_size_mm)
    R = np.asarray(sp.get("placement_R", [[1, 0, 0], [0, 1, 0], [0, 0, 1]]), dtype=float)
    verts = verts @ R.T
    part_vertices = [[round(float(c), 6) for c in ft.apply_point(v)] for v in verts]
    triangles = [[int(t[0]), int(t[1]), int(t[2])] for t in faces]

    # Containing hemisphere (dome planner): the scanpath records it in the placed
    # frame (mm); transform the centre into the table frame and scale the radius so
    # the debug viewport can draw it aligned with the mesh + path.
    dome = None
    sp_dome = sp.get("dome")
    if sp_dome:
        c = ft.apply_point(sp_dome["center_mm"])
        dome = {"center": [round(float(x), 6) for x in c],
                "radius": round(float(sp_dome["radius_mm"]) * ft.scale, 6),
                "up_axis": sp_dome.get("up_axis", 1)}

    waypoints = sp.get("waypoints", [])

    # Ground the whole scene on the table: the marked-corner calibration is an
    # identity placeholder, so ft leaves the part sitting at an arbitrary Z. Shift
    # everything (mesh, waypoints, dome) so the part's lowest point is exactly z=0.
    # This is a VIEWER-only shift (scanpath_arm, which the arm runs, is untouched),
    # and it lets the viewer draw the path/dome in a FIXED table frame that already
    # sits on the table -- no per-frame regrounding, so a part flip can rotate only
    # the part without dragging the path around.
    z_shift = min((v[2] for v in part_vertices), default=0.0)
    if z_shift:
        for v in part_vertices:
            v[2] = round(v[2] - z_shift, 6)
        for w in waypoints:
            if "position" in w:
                w["position"][2] = round(w["position"][2] - z_shift, 6)
            if "target" in w:
                w["target"][2] = round(w["target"][2] - z_shift, 6)
        if dome:
            dome["center"][2] = round(dome["center"][2] - z_shift, 6)

    bundle = {
        "units": "m",
        "frame": "table",
        "mount": {"base_height_m": 1.2,
                  "note": "SR5 overhead: base 1.2 m above the table, pointing down"},
        "standoff_m": sp.get("standoff_m") or round(sp.get("standoff_mm", 250) / 1000.0, 6),
        "part": {"vertices": part_vertices, "triangles": triangles},
        "dome": dome,
        "waypoints": waypoints,
    }
    with open(args.output, "w") as f:
        json.dump(bundle, f)
    print(f"wrote {args.output}: part {len(part_vertices)} verts / {len(triangles)} tris, "
          f"{len(bundle['waypoints'])} waypoints")


if __name__ == "__main__":
    main()
