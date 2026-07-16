"""
waypoint_generator.py

Stages 3-5 of the PathPlanner pipeline: raster spacing, raster generation,
and travel-direction orientation.

Turns a sampled surface (points + outward normals from normal_estimation.py)
into an ORDERED raster coverage path of probe waypoints. Each waypoint sits
at the configured standoff distance along its surface normal, facing the
surface.

This is a "face-grouped binning" raster planner -- deliberately simple and
dependency-light (numpy only), used as the R&D stand-in for the eventual
production planner (noether's PlaneSlicerRasterPlanner). The approach:

  1. Raster LINE SPACING comes from the scanner's field of view (FOV) and
     the required overlap between adjacent lines (raster_spacing_from_fov)
     -- "how far apart can two scan lines be and still overlap enough to
     register cleanly".
  2. FACE GROUPING (the key step): surface samples are clustered by their
     outward NORMAL, so each group is one roughly-planar face patch whose
     normals all point the same way (within face_angle_tol_deg). This is
     what stops points from different faces -- a top and a side, say --
     from being mixed into one un-traceable line. A curved surface simply
     splits into a few angular bands. See _group_by_normal.
  3. Within each group we raster IN THAT FACE'S OWN PLANE: the two raster
     axes are derived from the group's mean normal (_inplane_axes), not a
     fixed world axis. Lines run along the group's longer in-plane extent
     (fewer, longer lines) and are stacked across the shorter one at the
     step-1 spacing. Points within a line are sorted, decimated to the
     along-track spacing, and alternate lines reversed (BOUSTROPHEDON --
     "as the ox plows", back-and-forth) so the probe snakes continuously.
  4. Each surviving point becomes a Waypoint: probe position = surface
     point + its own normal * standoff; travel_direction points toward the
     next point in the same line; line_id is a running counter UNIQUE
     across all groups, so incidence_cone_modifier only ever smooths within
     one line -- never across a face boundary or a zig-zag turn.

The resulting Waypoint list feeds directly into
incidence_cone_modifier.apply_incidence_cone_relaxation, the next stage.

CAVEAT -- this is still an approximation, not an exact geometric slice:
grouping by normal then binning in a plane is not a true geodesic mesh
slice, so highly curved or organic parts are only covered approximately.
It is, however, correct for the prismatic / faceted parts this cell scans,
and gives a real, viewable path today while the noether-based production
planner is still being built. See the Coverage Path Planning design doc
for the full production-vs-R&D comparison.

Dependencies: numpy.
"""

import math

import numpy as np

from .incidence_cone_modifier import Waypoint


def raster_spacing_from_fov(standoff_mm, fov_deg, overlap_fraction):
    """
    Compute the spacing between adjacent raster lines so their scan
    footprints overlap by the required fraction.

    The scanner's field of view is a cone; at a given standoff distance
    it illuminates a footprint of width `2 * standoff * tan(fov/2)` on a
    flat surface facing it square-on. Stepping the next line over by the
    FULL footprint width would leave zero overlap (any misalignment could
    leave a gap); stepping by less than the footprint leaves the required
    overlap for the two adjacent scans to register against each other
    reliably.

    Args:
        standoff_mm: probe distance from the surface.
        fov_deg: scanner field of view, full angle (not half-angle).
        overlap_fraction: required overlap between neighbouring lines, in
            [0, 1) -- e.g. 0.3 means adjacent footprints must share 30% of
            their width.

    Returns:
        Line spacing in mm.
    """
    footprint_width = 2.0 * standoff_mm * math.tan(math.radians(fov_deg) / 2.0)
    spacing = footprint_width * (1.0 - overlap_fraction)
    if spacing <= 0:
        raise ValueError(
            "computed raster spacing <= 0 -- check standoff/fov/overlap "
            f"(got standoff={standoff_mm}, fov_deg={fov_deg}, overlap={overlap_fraction})"
        )
    return spacing


def _decimate_along(coord, spacing):
    """
    Pick indices into a coordinate-sorted array that keep consecutive kept
    points at least `spacing` apart.

    Greedy left-to-right walk: always keep the first point, then keep the
    next point whose coordinate is at least `spacing` past the last kept
    one. This is simple and deterministic, and (unlike naive fixed-step
    slicing) never needs the points to be evenly spaced to begin with --
    the input here is randomly sampled, not a regular grid.
    """
    keep = [0]
    last = coord[0]
    for i in range(1, len(coord)):
        if coord[i] - last >= spacing:
            keep.append(i)
            last = coord[i]
    return keep


def _group_by_normal(normals, angle_tol_deg):
    """
    Cluster sample indices into face groups by outward-normal direction.

    Greedy single pass: each sample joins the existing group whose running
    mean normal is closest to its own AND within `angle_tol_deg`; if none
    qualifies it starts a new group. This separates a top face from a side
    face (normals ~90 deg apart) BEFORE any rastering, so a raster line can
    never span two faces. A curved surface -- whose normal sweeps
    continuously -- falls into several adjacent bands of width ~angle_tol_deg,
    which is what we want (each band rasters cleanly on its own).

    Order-dependent, but the caller's samples come from a seeded RNG, so the
    grouping is deterministic run to run.

    Args:
        normals: (N, 3) outward normals (need not be pre-normalised).
        angle_tol_deg: max angle between a sample normal and its group's mean.

    Returns:
        (groups, means): `groups` is a list of index lists (into `normals`),
        one per face group in first-seen order; `means` is the matching list
        of unit mean normals.
    """
    normals = np.asarray(normals, dtype=float)
    cos_tol = math.cos(math.radians(angle_tol_deg))
    sums = []    # running vector sum of unit normals, per group
    means = []   # unit mean normal per group (sums, renormalised)
    groups = []  # member indices per group
    for i in range(len(normals)):
        n = normals[i] / (np.linalg.norm(normals[i]) + 1e-12)
        best, best_dot = -1, cos_tol
        for g, mean in enumerate(means):
            d = float(np.dot(n, mean))
            if d >= best_dot:  # closer than tol, and better than any group so far
                best, best_dot = g, d
        if best < 0:
            sums.append(n.copy())
            means.append(n.copy())
            groups.append([i])
        else:
            sums[best] += n
            means[best] = sums[best] / (np.linalg.norm(sums[best]) + 1e-12)
            groups[best].append(i)
    return groups, means


def _inplane_axes(normal):
    """
    Two orthonormal axes (u, v) spanning the plane perpendicular to `normal`.

    Lets a face be rastered in its OWN plane rather than a fixed world plane.
    Seeded from the world axis least aligned with the normal (so the
    Gram-Schmidt subtraction below can't collapse to zero), then orthonormalised
    with a cross product. Deterministic for a given normal.
    """
    n = np.asarray(normal, dtype=float)
    n = n / (np.linalg.norm(n) + 1e-12)
    seed = np.eye(3)[int(np.argmin(np.abs(n)))]  # world axis least parallel to n
    u = seed - np.dot(seed, n) * n
    u = u / (np.linalg.norm(u) + 1e-12)
    v = np.cross(n, u)
    v = v / (np.linalg.norm(v) + 1e-12)
    return u, v


def generate_raster_waypoints(
    points,
    normals,
    standoff_mm,
    raster_spacing_mm,
    along_track_mm,
    face_angle_tol_deg=30.0,
    table_up_mm=0.0,
    up_axis=1,
    log=print,
):
    """
    Build the ordered list of probe Waypoints for a raster coverage path.

    Samples are first grouped into faces by normal direction
    (_group_by_normal); each group is then rastered independently in its own
    plane. Waypoints are emitted group by group, so the returned list is a
    concatenation of per-face boustrophedon paths.

    Args:
        points: (N, 3) surface sample points (from normal_estimation.sample_surface).
        normals: (N, 3) outward normals at each point (need not be unit length).
        standoff_mm: probe distance from the surface along each normal.
        raster_spacing_mm: gap between raster lines (from raster_spacing_from_fov,
            or overridden directly by the caller).
        along_track_mm: target spacing between waypoints along a single line.
        face_angle_tol_deg: normal-clustering tolerance -- samples whose normals
            fall within this angle share a face group. Smaller = more, tighter
            faces; larger = fewer, coarser ones. (Belongs in config once
            system_config.yaml exists; passed in explicitly for now.)
        table_up_mm: the table-top height along the part frame's UP axis. Probe
            waypoints BELOW this are physically unreachable (they'd be inside /
            under the table), so they are DROPPED -- never silently kept, never
            clipped onto the surface (a clipped pose would break the standoff).
            The dropped count is reported via `log`. With the part seated on the
            table (the marked-corner convention) the table top is up=0.
        up_axis: which part-frame axis points up (default 1 = Y, the CAD
            convention; the Y-up->Z-up remap makes this the arm frame's Z).
        log: sink for the dropped-waypoint report (default print; the ROS node
            passes its own logger).

    Returns:
        List of Waypoint objects. Within each face group consecutive waypoints
        form a continuous boustrophedon path; line_id is unique across the whole
        list, so downstream smoothing never crosses a face boundary or a turn.
    """
    points = np.asarray(points, dtype=float)
    normals = np.asarray(normals, dtype=float)

    groups, group_means = _group_by_normal(normals, face_angle_tol_deg)

    waypoints = []
    line_id = 0    # running, unique across every face group
    dropped = 0    # probe poses below the table (unreachable) -- dropped, counted

    for members, mean_normal in zip(groups, group_means):
        idx = np.asarray(members, dtype=int)
        group_points = points[idx]
        group_normals = normals[idx]

        # Raster in the face's OWN plane. u/v span that plane; let lines run
        # along whichever in-plane axis the face is LONGER on (fewer, longer
        # lines -> fewer turns) and stack them across the shorter one.
        u, v = _inplane_axes(mean_normal)
        u_coord = group_points @ u
        v_coord = group_points @ v
        if np.ptp(u_coord) >= np.ptp(v_coord):
            travel_coord, step_coord, travel_axis_vec = u_coord, v_coord, u
        else:
            travel_coord, step_coord, travel_axis_vec = v_coord, u_coord, v

        # Bucket this group's points into raster lines by their step-axis
        # coordinate -- same fixed-width binning as before, but now confined
        # to one face and measured along an in-plane axis.
        origin = step_coord.min()
        bucket = np.floor((step_coord - origin) / raster_spacing_mm).astype(int)

        # sorted(set(...)) so lines are visited in ascending step order
        # regardless of how floor() happened to order the bucket indices.
        for local_line, bucket_value in enumerate(sorted(set(bucket.tolist()))):
            mask = bucket == bucket_value
            line_points = group_points[mask]
            line_normals = group_normals[mask]
            line_travel = travel_coord[mask]

            # Order along the travel axis; boustrophedon-reverse alternate
            # lines WITHIN this face so the probe snakes continuously instead
            # of flying back to a fixed start each line.
            order = np.argsort(line_travel)
            if local_line % 2 == 1:
                order = order[::-1]
            line_points = line_points[order]
            line_normals = line_normals[order]
            line_travel = line_travel[order]

            # Decimate along cumulative arc length (not raw coordinate) so a
            # reversed line decimates identically to a forward one -- arc
            # length is non-negative and monotone regardless of direction.
            arc_length = np.concatenate([[0.0], np.cumsum(np.abs(np.diff(line_travel)))])
            kept_indices = _decimate_along(arc_length, along_track_mm)

            kept_points = line_points[kept_indices]
            kept_normals = line_normals[kept_indices]
            n = len(kept_points)

            for i in range(n):
                normal = kept_normals[i] / (np.linalg.norm(kept_normals[i]) + 1e-12)
                # Offset along the point's OWN normal (more accurate than the
                # group mean on curved bands). The scan TARGET stays
                # recoverable as position - normal * standoff, so it isn't
                # stored redundantly -- Waypoint uses __slots__ to stay lean.
                probe_position = kept_points[i] + normal * standoff_mm

                # Table constraint: a probe pose below the table top cannot be
                # reached (the arm would collide with / pass under the table).
                # DROP it and count -- see the docstring for the policy.
                if probe_position[up_axis] < table_up_mm:
                    dropped += 1
                    continue

                # Travel direction: toward the next kept point, else continue
                # the previous direction, else fall back to the in-plane travel
                # axis -- never left undefined, since build_orientation_frame()
                # downstream needs a real vector to build X/Y from.
                if i + 1 < n:
                    travel = kept_points[i + 1] - kept_points[i]
                elif i > 0:
                    travel = kept_points[i] - kept_points[i - 1]
                else:
                    travel = travel_axis_vec
                if np.linalg.norm(travel) < 1e-9:
                    travel = travel_axis_vec

                waypoints.append(
                    Waypoint(
                        position=probe_position,
                        normal=normal,
                        travel_direction=travel / np.linalg.norm(travel),
                        line_id=line_id,
                    )
                )
            line_id += 1

    if dropped:
        log(f"[waypoint_generator] dropped {dropped} waypoint(s) below the table "
            f"(up axis {up_axis} < {table_up_mm} mm) -- unreachable, not kept")

    return waypoints
