"""
multiview_lawnmower.py

Full-surround coverage planner: lawnmower passes on EVERY side of the part.

The single-view lawnmower (lawnmower_raster.py) casts rays straight down and keeps
the highest hit, so it only ever covers the top-most surface. This planner removes
that limit by running the same lawnmower from SEVERAL view directions and merging
them, so every face is swept by the view that faces it most square-on.

  1. VIEW DIRECTIONS -- default the 6 axis directions (+/-X, +/-Y, +/-Z); more (a
     Fibonacci sphere) for rounded parts so oblique faces get a square-on view.
  2. FACE ASSIGNMENT -- each mesh face is assigned to the ONE view whose direction
     best faces it (max normal.view_dir). This partitions the surface: every face
     belongs to exactly one view, so coverage is complete and no face is scanned
     twice.
  3. PER-VIEW LAWNMOWER -- for each view direction d: lay a boustrophedon raster in
     the plane perpendicular to d, cast rays along -d, and keep only hits on faces
     ASSIGNED to this view (taking the OUTERMOST such hit along d). That gives clean
     lawnmower passes over just this view's share of the surface.
  4. MERGE -- concatenate all views into one path; line_id is unique per pass across
     every view. Reachability is deliberately NOT considered here (a separate
     concern) -- this planner's job is full-surface coverage.
  5. COVERAGE REPORT -- fraction of surface area actually swept, so any gaps (deep
     pockets a straight ray can't reach) are explicit, never silent.

Output is the same incidence_cone_modifier.Waypoint list as every other planner, so
the downstream pipeline (incidence-cone relaxation, frame transform) is unchanged.

Dependencies: numpy, trimesh (+ scipy/networkx/rtree for ray casting).
"""

import math

import numpy as np
import trimesh

from .incidence_cone_modifier import Waypoint


def view_directions(n_views):
    """`n_views` roughly-uniform unit directions on the sphere. n_views == 6 gives
    the exact axis directions (best for boxy parts); otherwise a Fibonacci sphere
    (good for rounded parts -- more views = more square-on coverage)."""
    if n_views == 6:
        return np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0],
                         [0, -1, 0], [0, 0, 1], [0, 0, -1]], dtype=float)
    golden = math.pi * (3.0 - math.sqrt(5.0))
    dirs = []
    for i in range(n_views):
        y = 1.0 - 2.0 * (i + 0.5) / n_views
        r = math.sqrt(max(0.0, 1.0 - y * y))
        theta = golden * i
        dirs.append([math.cos(theta) * r, y, math.sin(theta) * r])
    return np.array(dirs, dtype=float)


def _plane_axes(d):
    """Two orthonormal axes spanning the plane perpendicular to view direction d."""
    d = d / (np.linalg.norm(d) + 1e-12)
    seed = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = seed - np.dot(seed, d) * d
    u /= np.linalg.norm(u) + 1e-12
    w = np.cross(d, u)
    w /= np.linalg.norm(w) + 1e-12
    return u, w


def _ensure_outward(mesh):
    """Make face normals point OUTWARD. fix_normals() gets winding consistent; for a
    watertight solid a negative signed volume means the whole thing is inside-out, so
    flip it. Guards against a dirty/inverted mesh sending every probe into the part."""
    mesh.fix_normals()
    try:
        if mesh.is_watertight and mesh.volume < 0:
            mesh.invert()
    except Exception:  # noqa: BLE001
        pass
    return mesh


def generate_multiview_waypoints(
    vertices,
    faces,
    standoff_mm,
    line_spacing_mm,
    along_track_mm,
    n_views=6,
    min_pass_points=2,
    max_bridge_gap=4,
    log=print,
):
    """
    Build a full-surround lawnmower coverage path (waypoints on all sides).

    Args:
        vertices, faces: triangle mesh (from cad_loader.load_cad), CAD units (mm).
        standoff_mm: probe distance from the surface along each point's normal.
        line_spacing_mm: gap between passes within a view.
        along_track_mm: spacing between points along a pass.
        n_views: number of view directions (6 = axis-aligned; more for rounded parts).
        min_pass_points: passes shorter than this (after trimming) are dropped.
        max_bridge_gap: bridge over at most this many consecutive missing samples in a
            pass (a small hole); a longer gap splits the pass instead of interpolating
            across empty space.
        log: progress sink.

    Returns:
        List of Waypoint; line_id unique per pass across all views.
    """
    mesh = trimesh.Trimesh(vertices=np.asarray(vertices, dtype=float),
                           faces=np.asarray(faces), process=True)
    _ensure_outward(mesh)

    face_normals = mesh.face_normals
    views = view_directions(n_views)
    # assign each face to the view whose direction it faces most squarely
    assign = np.argmax(face_normals @ views.T, axis=1)

    waypoints = []
    line_id = 0
    covered_faces = set()

    for vi, d in enumerate(views):
        face_mask = assign == vi
        if not face_mask.any():
            continue
        view_faces = set(np.where(face_mask)[0].tolist())

        u, w = _plane_axes(d)
        vids = np.unique(mesh.faces[face_mask].ravel())
        P = mesh.vertices[vids]
        uc, wc = P @ u, P @ w
        umin, umax, wmin, wmax = float(uc.min()), float(uc.max()), float(wc.min()), float(wc.max())
        launch = float((mesh.vertices @ d).max()) + 10.0   # ray start beyond the part along d

        line_id = _raster_view(
            mesh, d, u, w, umin, umax, wmin, wmax, launch, view_faces,
            standoff_mm, line_spacing_mm, along_track_mm,
            min_pass_points, max_bridge_gap, waypoints, line_id, covered_faces)

    # Footprint coverage: a face is covered if its centroid lies within one line
    # spacing of a scanned surface point (adjacent footprints overlap by design, so
    # anything within a line spacing of a waypoint is inside the swept band). This is
    # the honest metric -- counting only the single face each zero-width ray hits
    # would cap at (#waypoints / #faces) regardless of real coverage.
    total_area = float(mesh.area)
    pct = 0.0
    if waypoints and total_area > 0:
        from scipy.spatial import cKDTree
        targets = np.array([wp.position - wp.normal * standoff_mm for wp in waypoints])
        dist, _ = cKDTree(targets).query(mesh.triangles_center)
        covered = dist <= line_spacing_mm
        pct = 100.0 * float(mesh.area_faces[covered].sum()) / total_area
    log(f"[multiview_lawnmower] {n_views} views, {line_id} passes, {len(waypoints)} waypoints; "
        f"surface coverage {pct:.1f}% (within one line spacing of a scan point)")
    return waypoints


def _raster_view(mesh, d, u, w, umin, umax, wmin, wmax, launch, view_faces,
                 standoff_mm, line_spacing_mm, along_track_mm,
                 min_pass_points, max_bridge_gap, waypoints, line_id, covered_faces):
    """Lawnmower-raster one view's assigned faces; append Waypoints; return next line_id."""
    steps = np.arange(wmin, wmax + 1e-9, line_spacing_mm)
    flip = False
    for t in steps:
        s_vals = np.arange(umin, umax + 1e-9, along_track_mm)
        if len(s_vals) < 2:
            continue
        if flip:
            s_vals = s_vals[::-1]
        flip = not flip

        ground = np.outer(s_vals, u) + t * w              # in-plane sample positions
        origins = ground + d * launch
        dirs = np.tile(-d, (len(s_vals), 1))
        locs, ray_i, tri_i = mesh.ray.intersects_location(origins, dirs, multiple_hits=True)

        depth = np.full(len(s_vals), np.nan)              # signed distance along d
        normals = np.tile(d, (len(s_vals), 1))
        hit_face = np.full(len(s_vals), -1, dtype=int)
        for r in range(len(s_vals)):
            idx = np.where(ray_i == r)[0]
            # keep only hits on faces assigned to THIS view
            idx = [h for h in idx if int(tri_i[h]) in view_faces]
            if not idx:
                continue
            k = idx[int(np.argmax(locs[idx] @ d))]        # outermost assigned hit along d
            depth[r] = float(locs[k] @ d)
            normals[r] = mesh.face_normals[tri_i[k]]
            hit_face[r] = int(tri_i[k])

        valid = ~np.isnan(depth)
        # split this raster row into runs of valid samples, bridging only SMALL gaps
        line_id = _emit_runs(ground, depth, normals, hit_face, valid, d,
                             standoff_mm, min_pass_points, max_bridge_gap,
                             waypoints, line_id, covered_faces)
    return line_id


def _emit_runs(ground, depth, normals, hit_face, valid, d, standoff_mm,
               min_pass_points, max_bridge_gap, waypoints, line_id, covered_faces):
    """Turn one raster row into passes: bridge gaps up to max_bridge_gap, split on
    larger gaps (so we never interpolate a pass across empty space)."""
    n = len(valid)
    i = 0
    while i < n:
        if not valid[i]:
            i += 1
            continue
        j = i
        while j + 1 < n:
            if valid[j + 1]:
                j += 1
                continue
            # look ahead: bridge only if the gap of invalids is small AND closes again
            gap = 1
            while j + 1 + gap < n and not valid[j + gap]:
                gap += 1
            nxt = j + gap
            if nxt < n and valid[nxt] and gap <= max_bridge_gap:
                j = nxt                     # absorb the small gap into this run
            else:
                break
        run = np.arange(i, j + 1)
        run_valid = valid[run]
        if run_valid.sum() >= min_pass_points:
            depth_run = depth[run].copy()
            if not run_valid.all():          # bridge the small interior gaps
                depth_run = np.interp(run, run[run_valid], depth[run][run_valid])
            surf = ground[run] + np.outer(depth_run, d)
            norms = normals[run].copy()
            norms[~run_valid] = d            # bridged points aim along the view dir
            m = len(run)
            for a in range(m):
                normal = norms[a] / (np.linalg.norm(norms[a]) + 1e-12)
                probe = surf[a] + normal * standoff_mm
                if a + 1 < m:
                    travel = surf[a + 1] - surf[a]
                elif a > 0:
                    travel = surf[a] - surf[a - 1]
                else:
                    travel = surf[a] * 0.0 + np.cross(d, normal)
                if np.linalg.norm(travel) < 1e-9:
                    travel = np.cross(d, normal)
                    if np.linalg.norm(travel) < 1e-9:
                        travel = _plane_axes(d)[0]
                waypoints.append(Waypoint(position=probe, normal=normal,
                                          travel_direction=travel / np.linalg.norm(travel),
                                          line_id=line_id))
                if run_valid[a] and hit_face[run[a]] >= 0:
                    covered_faces.add(hit_face[run[a]])
            line_id += 1
        i = j + 1
    return line_id
