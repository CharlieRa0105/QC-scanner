"""
box_raster.py

Table-aligned bounding box coverage planner: wrap the part in the smallest box
that sits FLAT ON THE TABLE (one face flush on the table plane, vertical sides)
and raster a lawnmower path over EACH of the box's six faces -- scanner standoff
OUTSIDE the face, aimed square-on inward. Geometry-agnostic: the path depends only
on the fitted box, so it behaves the same for a pin, a bracket, or a casting.

  1. FIT -- box flat on the table. The up axis is fixed to the table normal (the
     placed frame's +Y), so the box never tilts: bottom face on the table plane
     (the part's lowest point), top face at the part's highest point. Only the
     rotation ABOUT the vertical (the footprint yaw) is optimised -- the minimum-
     AREA rectangle of the part's footprint, via rotating calipers over the 2-D
     convex-hull edges. Minimises the box footprint while staying flat.
  2. RASTER -- boustrophedon grid over each of the 6 faces: rows spaced by
     line_spacing, points along a row spaced by along_track. Each waypoint sits
     standoff OUTSIDE its face; its normal points INWARD (so the scanner at the
     position looks in along +normal at the face, square-on).
  3. The fitted box (centre, rotation, half-extents) is returned too, so the CLI
     can record it and the viewport can draw the box.

Reachability is not considered here (a separate concern). Output is the same
incidence_cone_modifier.Waypoint list as every planner, so downstream is unchanged.

Dependencies: numpy, scipy (ConvexHull).
"""

import math

import numpy as np

from .incidence_cone_modifier import Waypoint


def _min_area_rect(points_2d):
    """
    Minimum-area bounding rectangle of a 2-D point set (rotating calipers over the
    convex-hull edges: the optimal rectangle has one edge collinear with a hull
    edge). Returns (area, axes_2x2, half_extents_2, centre_2), axes columns = the
    rectangle's local axes in the input frame.
    """
    from scipy.spatial import ConvexHull

    p = np.asarray(points_2d, dtype=float)
    try:
        hull = ConvexHull(p)
        hp = p[hull.vertices]
    except Exception:                      # collinear / degenerate -> axis-aligned
        hp = p
    best = None
    n = len(hp)
    for i in range(n):
        edge = hp[(i + 1) % n] - hp[i]
        length = np.linalg.norm(edge)
        if length < 1e-9:
            continue
        ex = edge / length
        ey = np.array([-ex[1], ex[0]])
        xs, ys = hp @ ex, hp @ ey
        w, h = xs.max() - xs.min(), ys.max() - ys.min()
        area = w * h
        if best is None or area < best[0]:
            centre = ((xs.max() + xs.min()) / 2) * ex + ((ys.max() + ys.min()) / 2) * ey
            best = (area, np.column_stack([ex, ey]), np.array([w / 2, h / 2]), centre)
    if best is None:                       # single point
        return 0.0, np.eye(2), np.zeros(2), hp.mean(axis=0)
    return best


def fit_table_box(vertices, up_axis=1):
    """
    Smallest box CONTAINING `vertices` (Nx3, mm) that sits flat on the table: the
    up axis is fixed to `up_axis` (the table normal), so the box stays vertical.
    Only the footprint yaw is optimised (minimum-area footprint rectangle).

    Returns (centre, R, half): centre is the box centre (3,), R a proper rotation
    (3x3, COLUMNS = the box's local axes; column 2 = the vertical/up axis), half
    the box half-extents along those axes (3,). det(R) = +1.
    """
    v = np.asarray(vertices, dtype=float)
    lat = [i for i in range(3) if i != up_axis]     # the two table-plane axes

    ground = float(v[:, up_axis].min())
    top = float(v[:, up_axis].max())
    half_up = (top - ground) / 2.0

    # minimum-area rectangle of the footprint (projection onto the table plane)
    _, r2, ext2, c2 = _min_area_rect(v[:, lat])

    e0 = np.zeros(3); e0[lat[0]] = 1.0
    e1 = np.zeros(3); e1[lat[1]] = 1.0
    up = np.zeros(3); up[up_axis] = 1.0
    ax0 = r2[0, 0] * e0 + r2[1, 0] * e1              # footprint axes lifted to 3-D
    ax1 = r2[0, 1] * e0 + r2[1, 1] * e1
    R = np.column_stack([ax0, ax1, up])
    if np.linalg.det(R) < 0:                         # proper rotation (box is symmetric)
        R[:, 1] = -R[:, 1]

    centre = c2[0] * e0 + c2[1] * e1 + (ground + half_up) * up
    half = np.array([ext2[0], ext2[1], half_up])
    return centre, R, half


def generate_box_waypoints(
    vertices,
    standoff_mm,
    line_spacing_mm,
    along_track_mm,
    up_axis=1,
    log=print,
):
    """
    Boustrophedon coverage path over EACH face of the part's table-aligned box.

    Args:
        vertices: part mesh vertices (placed frame, CAD units mm).
        standoff_mm: how far OUTSIDE each face the scanner sits.
        line_spacing_mm: gap between raster rows on a face.
        along_track_mm: gap between points along a row.
        up_axis: which axis is the table normal (default 1 = +Y, pipeline convention).
        log: progress sink.

    Returns:
        (waypoints, centre, R, half): waypoints is the Waypoint list (normal points
        INWARD so the scanner looks square-on at the face); centre/R/half describe
        the fitted box (for recording + drawing).
    """
    centre, R, half = fit_table_box(vertices, up_axis=up_axis)
    axes = [R[:, 0], R[:, 1], R[:, 2]]

    waypoints = []
    line_id = 0
    for f in range(3):                     # 3 axis pairs -> 2 faces each = 6 faces
        na, hf = axes[f], half[f]
        i1, i2 = (f + 1) % 3, (f + 2) % 3
        u, w = axes[i1], axes[i2]
        hu, hw = half[i1], half[i2]
        n_rows = max(1, int(math.ceil((2 * hu) / max(line_spacing_mm, 1e-6))))
        n_cols = max(1, int(math.ceil((2 * hw) / max(along_track_mm, 1e-6))))
        us = np.linspace(-hu, hu, n_rows + 1)
        ws = np.linspace(-hw, hw, n_cols + 1)
        for sign in (1.0, -1.0):
            outward = sign * na
            # skip the DOWN-facing face: it rests on the table (unscannable), and
            # its waypoints sit below the table -- lifting them for table clearance
            # would push them straight up INTO a tall part. (axes[2] is the box's
            # vertical/up axis.)
            if float(np.dot(outward, axes[2])) < -0.5:
                continue
            inward = -outward              # scanner looks along +normal, so normal = inward
            face_centre = centre + outward * hf
            reverse = False
            for a in us:
                row = ws[::-1] if reverse else ws
                travel = -w if reverse else w
                for b in row:
                    face_pt = face_centre + u * a + w * b
                    position = face_pt + outward * standoff_mm
                    waypoints.append(Waypoint(position=position, normal=inward,
                                              travel_direction=travel, line_id=line_id))
                line_id += 1
                reverse = not reverse

    log(f"[box_raster] OBB half-extents {np.round(half, 1)}mm, {line_id} rows, "
        f"{len(waypoints)} waypoints (line {line_spacing_mm:.2f}mm, along {along_track_mm:.2f}mm)")
    return waypoints, centre, R, half
