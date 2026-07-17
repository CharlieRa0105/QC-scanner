"""
lawnmower_raster.py

2.5D lawnmower (boustrophedon) coverage planner for the part's PRESENTED (top)
surface -- the path shape in refPath.jpg: straight parallel passes, U-turns at the
ends, one continuous snake draping over the surface.

This is the planner the QC cell actually wants for an OVERHEAD scanner: the arm
sweeps the up-facing surface in a lawnmower pattern (the underside/sides are the
operator's flip pass, per the settled workflow). It supersedes both the face-group
grid (waypoint_generator.py, "weird grids") and the plane-slice contour raster
(contour_raster.py, rings that wrapped the whole 3D shape).

How it works ("2.5D" = a 2D XY raster lifted onto the surface in Z):

  1. Lay a straight raster in the table plane: passes run along a chosen direction
     (scan_dir_deg), stepped sideways at the line spacing, points along each pass
     at the along-track spacing. Alternate passes reverse (boustrophedon) so the
     whole thing is one continuous snake.
  2. For each raster point, cast a ray straight DOWN onto the mesh and take the
     HIGHEST hit -- that's the top surface at that (x, y). Its face gives the
     outward normal. The probe sits standoff above along that normal.
  3. HOLE BRIDGING: a screw hole / pocket shows up as a sample whose surface drops
     sharply below its neighbours (or a ray that misses through a through-hole).
     Those are bridged -- the pass keeps a smooth top-surface height across the
     hole instead of diving into it -- so small features don't wreck the sweep.
     (A CAD pre-simplify would do the same upstream; this needs no CAD edit.)
  4. Points off the part silhouette (ray misses at a pass's ends) are trimmed, so a
     pass spans only where the part actually is.

Each pass is one line_id, so downstream incidence-cone smoothing never crosses a
U-turn. Output is the same incidence_cone_modifier.Waypoint list every planner
emits, so the rest of the pipeline is unchanged.

Dependencies: numpy, trimesh (+ its scipy/networkx/rtree deps for ray casting).
"""

import math

import numpy as np
import trimesh

from .incidence_cone_modifier import Waypoint


def _pass_axes(scan_dir_deg, up_axis):
    """In-plane unit axes (u = pass direction, v = step direction) spanning the
    ground plane perpendicular to `up_axis`, with `scan_dir_deg` rotating them
    within that plane so the operator can pick which way the passes run."""
    others = [i for i in range(3) if i != up_axis]
    e0, e1 = np.eye(3)[others[0]], np.eye(3)[others[1]]
    a = math.radians(scan_dir_deg)
    u = math.cos(a) * e0 + math.sin(a) * e1
    v = -math.sin(a) * e0 + math.cos(a) * e1
    return u, v


def generate_lawnmower_waypoints(
    vertices,
    faces,
    standoff_mm,
    line_spacing_mm,
    along_track_mm,
    scan_dir_deg=0.0,
    up_axis=1,
    hole_drop_frac=0.3,
    max_surface_tilt_deg=60.0,
    min_pass_points=2,
    log=print,
):
    """
    Build a continuous top-surface lawnmower coverage path.

    Args:
        vertices, faces: triangle mesh (from cad_loader.load_cad), CAD units (mm),
            already in its resting placement (see placement.resting_rotation).
        standoff_mm: probe distance above the surface along the local normal.
        line_spacing_mm: gap between adjacent passes (from raster_spacing_from_fov,
            or area-scaled by the caller).
        along_track_mm: spacing between points along a pass.
        scan_dir_deg: pass direction within the ground plane (0 = along the first
            non-up axis, stepping along the second).
        up_axis: which axis points up in this frame (default 1 = +Y, the CAD/pipeline
            convention). Rays are cast down along -up_axis to find the top surface.
        hole_drop_frac: a sample whose surface sits more than this fraction of the
            part's height from its local neighbours (dip OR spike) is treated as a
            hole/edge artifact and bridged over. 0 disables bridging.
        max_surface_tilt_deg: drop points whose surface normal tilts more than this
            from up -- near-vertical faces (part sides, end caps) can't be seen by an
            overhead scanner and belong to the operator's flip pass. None disables.
        min_pass_points: passes shorter than this (after trimming) are dropped.
        log: progress sink (default print; a ROS node passes its logger).

    Returns:
        List of Waypoint, one continuous boustrophedon; line_id unique per pass.
    """
    mesh = trimesh.Trimesh(
        vertices=np.asarray(vertices, dtype=float),
        faces=np.asarray(faces),
        process=True,
    )
    mesh.fix_normals()

    UP = np.eye(3)[up_axis]
    u, v = _pass_axes(scan_dir_deg, up_axis)
    u_coord = mesh.vertices @ u
    v_coord = mesh.vertices @ v
    umin, umax = float(u_coord.min()), float(u_coord.max())
    vmin, vmax = float(v_coord.min()), float(v_coord.max())
    top = float(mesh.bounds[1][up_axis])
    hole_drop = hole_drop_frac * float(mesh.extents[up_axis])

    steps = np.arange(vmin, vmax + 1e-9, line_spacing_mm)
    waypoints = []
    line_id = 0
    flip = False
    bridged = 0

    for t in steps:
        s_vals = np.arange(umin, umax + 1e-9, along_track_mm)
        if len(s_vals) < 2:
            continue
        if flip:
            s_vals = s_vals[::-1]
        flip = not flip

        # ground point per sample (in-plane, up-coord 0), then lift above the part
        # and cast a ray straight DOWN (-UP) to find the top surface.
        ground = np.array([s * u + t * v for s in s_vals])
        origins = ground + UP * (top + 10.0)
        dirs = np.tile(-UP, (len(s_vals), 1))
        locs, ray_i, tri_i = mesh.ray.intersects_location(origins, dirs, multiple_hits=True)

        z = np.full(len(s_vals), np.nan)          # surface height along UP
        normals = np.tile(UP, (len(s_vals), 1))
        for r in range(len(s_vals)):
            hits = np.where(ray_i == r)[0]
            if len(hits) == 0:
                continue                          # ray missed the part here
            k = hits[np.argmax(locs[hits, up_axis])]   # highest hit = top surface
            z[r] = locs[k, up_axis]
            normals[r] = mesh.face_normals[tri_i[k]]

        valid = ~np.isnan(z)
        # outlier detection: a hit that deviates > hole_drop from its local
        # neighbourhood median is a hole/pocket (dip) or a grazing edge hit (spike)
        # -- either way not the real top surface here, so bridge over it.
        if hole_drop > 0:
            for i in range(len(s_vals)):
                if not valid[i]:
                    continue
                a, b = max(0, i - 2), min(len(s_vals), i + 3)
                neigh = z[a:b][valid[a:b]]
                if neigh.size and abs(z[i] - np.median(neigh)) > hole_drop:
                    valid[i] = False           # dip or spike -> bridge

        # overhead-visibility: drop near-vertical faces (sides/end caps) the scanner
        # can't see from above -- they belong to the flip pass, not this sweep.
        if max_surface_tilt_deg is not None:
            cos_tilt = math.cos(math.radians(max_surface_tilt_deg))
            for i in range(len(s_vals)):
                if not valid[i]:
                    continue
                n = normals[i] / (np.linalg.norm(normals[i]) + 1e-12)
                if float(n @ UP) < cos_tilt:
                    valid[i] = False

        kept = np.where(valid)[0]
        if len(kept) < min_pass_points:
            continue
        lo, hi = kept[0], kept[-1]             # trim off-silhouette ends
        span = np.arange(lo, hi + 1)
        span_valid = valid[span]

        # bridge interior holes: interpolate surface Z across invalid samples
        z_span = z[span].copy()
        if not span_valid.all():
            z_span = np.interp(span, span[span_valid], z[span][span_valid])
            bridged += int((~span_valid).sum())

        # surface point = ground position + surface height along UP
        surf = ground[span] + np.outer(z_span, UP)
        norm_span = normals[span].copy()
        norm_span[~span_valid] = UP                # bridged points aim straight up

        n = len(span)
        for i in range(n):
            normal = norm_span[i] / (np.linalg.norm(norm_span[i]) + 1e-12)
            probe = surf[i] + normal * standoff_mm
            if i + 1 < n:
                travel = surf[i + 1] - surf[i]
            elif i > 0:
                travel = surf[i] - surf[i - 1]
            else:
                travel = u
            if np.linalg.norm(travel) < 1e-9:
                travel = u
            waypoints.append(
                Waypoint(
                    position=probe,
                    normal=normal,
                    travel_direction=travel / np.linalg.norm(travel),
                    line_id=line_id,
                )
            )
        line_id += 1

    log(f"[lawnmower_raster] {line_id} passes, {len(waypoints)} waypoints "
        f"(dir {scan_dir_deg:.0f} deg, line {line_spacing_mm:.2f}mm, along {along_track_mm:.2f}mm)"
        + (f"; bridged {bridged} hole/pocket sample(s)" if bridged else ""))
    return waypoints
