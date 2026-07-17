"""
contour_raster.py

Plane-slice coverage planner: the "zigzag that curves around the shape".

This REPLACES the face-grouped planar raster in waypoint_generator.py. Instead of
clustering surface samples by normal and laying an independent flat grid on each
face -- which reads as disjoint "weird grids" on any curved part -- it slices the
mesh with a stack of parallel planes and follows the CONTOUR each plane cuts
across the surface. Every contour curves around the true 3D shape (slice a cylinder
and the cut wraps around it); walking one contour, stepping to the next slice, and
reversing direction each time (boustrophedon) gives ONE continuous path that hugs
the surface -- a snake, not a patchwork.

This is the same primitive as noether's PlaneSlicerRasterPlanner (the documented
production target), done in pure Python on top of trimesh's robust mesh sectioning.

Pipeline position: takes the (vertices, faces) mesh from cad_loader.py and returns
an ordered list of incidence_cone_modifier.Waypoint -- the SAME type the old raster
emitted -- so the incidence-cone relaxation and everything downstream are unchanged.

  1. Build a trimesh, orient its normals outward (fix_normals).
  2. Slice with planes perpendicular to a chosen axis (default: the part's LONGEST
     bbox axis, so contours wrap the shorter cross-section), spaced at the raster
     line spacing.
  3. Each slice yields one or more contours (trimesh.section -> discrete polylines).
     Resample each along its arc length at the along-track spacing.
  4. Boustrophedon-reverse alternate contours so the path is continuous.
  5. Each point becomes a Waypoint: probe = surface point + outward normal *
     standoff; travel_direction points along the contour; line_id is unique per
     contour, so downstream smoothing never crosses a contour boundary.

Dependencies: numpy, trimesh (+ its scipy/networkx deps).
"""

import numpy as np
import trimesh

from .incidence_cone_modifier import Waypoint


def _resample_polyline(poly, step_mm):
    """Resample a polyline (n,3) to points ~step_mm apart along its arc length.

    The raw contour from mesh sectioning has vertices wherever the slice plane
    crossed a triangle edge -- irregular spacing. Resampling by cumulative arc
    length gives the even along-track spacing the scan wants, and works the same
    for open and closed (looped) contours.
    """
    poly = np.asarray(poly, dtype=float)
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    total = arc[-1]
    if total < 1e-9:
        return poly[:1]
    n = max(2, int(round(total / step_mm)) + 1)
    want = np.linspace(0.0, total, n)
    return np.column_stack([np.interp(want, arc, poly[:, k]) for k in range(3)])


def generate_contour_waypoints(
    vertices,
    faces,
    standoff_mm,
    slice_spacing_mm,
    along_track_mm,
    slice_axis=None,
    min_contour_points=2,
    log=print,
):
    """
    Build an ordered boustrophedon coverage path by slicing the mesh with parallel
    planes and following each surface contour.

    Args:
        vertices, faces: the triangle mesh (from cad_loader.load_cad), CAD units (mm).
        standoff_mm: probe distance from the surface along each point's outward normal.
        slice_spacing_mm: gap between adjacent slice planes = the raster line spacing
            (from raster_spacing_from_fov, or area-scaled by the caller).
        along_track_mm: target spacing between waypoints along a contour.
        slice_axis: 0/1/2 to slice perpendicular to X/Y/Z; None = the part's longest
            bbox axis (so passes wrap the shorter cross-section -- usually what you
            want). Exposed so the operator can flip the sweep direction.
        min_contour_points: contours that resample to fewer than this are dropped
            (a 1-point contour is a tangent touch, not a scan pass).
        log: progress sink (default print; a ROS node passes its logger).

    Returns:
        List of Waypoint. Consecutive waypoints within a contour form a continuous
        pass; line_id is unique per contour across the whole list.
    """
    mesh = trimesh.Trimesh(
        vertices=np.asarray(vertices, dtype=float),
        faces=np.asarray(faces),
        process=True,
    )
    mesh.fix_normals()  # consistent outward winding, so face normals point out

    extents = np.asarray(mesh.extents, dtype=float)
    axis = int(np.argmax(extents)) if slice_axis is None else int(slice_axis)
    plane_normal = np.eye(3)[axis]
    lo, hi = float(mesh.bounds[0][axis]), float(mesh.bounds[1][axis])

    # Slice planes centred within the span so the first/last sit half a step in
    # from the ends (avoids a degenerate tangent slice exactly at the extreme).
    heights = np.arange(lo + slice_spacing_mm * 0.5, hi, slice_spacing_mm)

    waypoints = []
    line_id = 0
    flip = False   # boustrophedon: reverse every other contour
    dropped = 0
    origin = np.asarray(mesh.centroid, dtype=float).copy()

    for h in heights:
        origin[axis] = h
        section = mesh.section(plane_origin=origin, plane_normal=plane_normal)
        if section is None:
            continue
        # section.discrete = one (n,3) polyline per connected contour in this slice;
        # longest first so the main loop leads and stubs follow.
        contours = sorted((np.asarray(p, dtype=float) for p in section.discrete),
                          key=len, reverse=True)
        for contour in contours:
            if len(contour) < 2:
                continue
            pts = _resample_polyline(contour, along_track_mm)
            if len(pts) < min_contour_points:
                dropped += len(pts)
                continue
            if flip:
                pts = pts[::-1]
            flip = not flip

            # Outward normal at each point = the nearest surface face's normal.
            _, _, face_ids = mesh.nearest.on_surface(pts)
            face_normals = mesh.face_normals[face_ids]

            k = len(pts)
            for i in range(k):
                normal = face_normals[i] / (np.linalg.norm(face_normals[i]) + 1e-12)
                probe_position = pts[i] + normal * standoff_mm
                if i + 1 < k:
                    travel = pts[i + 1] - pts[i]
                elif i > 0:
                    travel = pts[i] - pts[i - 1]
                else:
                    travel = np.array([1.0, 0.0, 0.0])
                if np.linalg.norm(travel) < 1e-9:
                    travel = np.array([1.0, 0.0, 0.0])
                waypoints.append(
                    Waypoint(
                        position=probe_position,
                        normal=normal,
                        travel_direction=travel / np.linalg.norm(travel),
                        line_id=line_id,
                    )
                )
            line_id += 1

    log(f"[contour_raster] {line_id} contours, {len(waypoints)} waypoints "
        f"(slice axis {axis}, line spacing {slice_spacing_mm:.2f}mm, "
        f"along-track {along_track_mm:.2f}mm)"
        + (f"; dropped {dropped} pt(s) in sub-{min_contour_points}-point contours" if dropped else ""))
    return waypoints
