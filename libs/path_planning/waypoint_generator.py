"""
waypoint_generator.py

Stages 3-5 of the PathPlanner pipeline: raster spacing, raster generation,
and travel-direction orientation.

Turns a sampled surface (points + outward normals from normal_estimation.py)
into an ORDERED raster coverage path of probe waypoints. Each waypoint sits
at the configured standoff distance along its surface normal, facing the
surface.

This is a "binning" raster planner -- deliberately simple and
dependency-light (numpy only), used as the R&D stand-in for the eventual
production planner (noether's PlaneSlicerRasterPlanner). The approach:

  1. Raster LINE SPACING comes from the scanner's field of view (FOV) and
     the required overlap between adjacent lines (raster_spacing_from_fov)
     -- this is the "how far apart can two scan lines be and still overlap
     enough to register cleanly" calculation.
  2. Surface points are bucketed into parallel lines by their coordinate
     along the chosen STEP axis (bucket width = the raster spacing from
     step 1).
  3. Within each line, points are sorted along the TRAVEL axis and
     decimated down to the along-track spacing (so waypoints along a line
     aren't needlessly dense). Alternate lines are reversed
     (BOUSTROPHEDON pattern -- "as the ox plows", i.e. back-and-forth) so
     the probe snakes continuously between lines instead of flying back to
     the start of every line.
  4. Each surviving point becomes a Waypoint: probe position = surface
     point + normal * standoff; travel_direction points toward the next
     point in the same line; line_id records which raster line it belongs
     to (needed downstream so incidence_cone_modifier only smooths within
     a line, never across a zig-zag turn).

The resulting Waypoint list feeds directly into
incidence_cone_modifier.apply_incidence_cone_relaxation, which is the next
pipeline stage.

CAVEAT -- this is an approximation, not an exact geometric slice: bucketing
by a single coordinate is not a true geodesic mesh slice through curved
surfaces, so it's best suited to roughly prismatic parts (and to getting a
real, viewable path today while the noether-based production planner is
still being built). See the Coverage Path Planning design doc for the
full production-vs-R&D comparison.

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


def generate_raster_waypoints(
    points,
    normals,
    standoff_mm,
    raster_spacing_mm,
    along_track_mm,
    step_axis=1,
    travel_axis=0,
):
    """
    Build the ordered list of probe Waypoints for a raster coverage path.

    Args:
        points: (N, 3) surface sample points (from normal_estimation.sample_surface).
        normals: (N, 3) outward unit normals at each point.
        standoff_mm: probe distance from the surface along each normal.
        raster_spacing_mm: gap between raster lines (from raster_spacing_from_fov,
            or overridden directly by the caller).
        along_track_mm: target spacing between waypoints along a single line.
        step_axis: axis index (0=X, 1=Y, 2=Z) that raster lines are stacked
            along -- i.e. which axis distinguishes one line from the next.
        travel_axis: axis index the probe travels along WITHIN a line.

    Returns:
        List of Waypoint objects, ordered so that consecutive waypoints
        form a continuous boustrophedon path (line 0 forward, line 1
        backward, line 2 forward, ...).
    """
    points = np.asarray(points, dtype=float)
    normals = np.asarray(normals, dtype=float)

    # Bucket every sample point into a raster line purely by its
    # step_axis coordinate -- points within the same raster_spacing_mm-wide
    # band belong to the same line.
    step_coord = points[:, step_axis]
    origin = step_coord.min()
    bucket = np.floor((step_coord - origin) / raster_spacing_mm).astype(int)

    waypoints = []
    # sorted(set(...)) so lines are visited in ascending step_axis order
    # regardless of how bucket indices happened to come out of floor().
    for line_id, bucket_value in enumerate(sorted(set(bucket.tolist()))):
        mask = bucket == bucket_value
        line_points = points[mask]
        line_normals = normals[mask]

        # Order points along the line by their travel_axis coordinate.
        order = np.argsort(line_points[:, travel_axis])
        # Boustrophedon: reverse every other line so the path travels
        # continuously from the end of one line into the start of the
        # next, instead of snapping back to a fixed start each time.
        if line_id % 2 == 1:
            order = order[::-1]
        line_points = line_points[order]
        line_normals = line_normals[order]

        # Decimate along cumulative arc length (not raw coordinate value)
        # so a reversed (line_id odd) line decimates identically to a
        # forward one -- arc length is always non-negative and
        # monotonically increasing regardless of travel direction.
        travel_coord = line_points[:, travel_axis]
        arc_length = np.concatenate([[0.0], np.cumsum(np.abs(np.diff(travel_coord)))])
        kept_indices = _decimate_along(arc_length, along_track_mm)

        kept_points = line_points[kept_indices]
        kept_normals = line_normals[kept_indices]
        n = len(kept_points)

        for i in range(n):
            surface_point = kept_points[i]
            normal = kept_normals[i] / (np.linalg.norm(kept_normals[i]) + 1e-12)
            # The probe waypoint sits offset from the surface along the
            # normal, at the configured standoff distance.
            probe_position = surface_point + normal * standoff_mm

            # Travel direction: toward the next point in the line, or (for
            # the last point) continuing the previous direction, or (for a
            # single-point line) an arbitrary fallback -- never left
            # undefined, since build_orientation_frame() downstream needs
            # a real vector to build X/Y from.
            if i + 1 < n:
                travel = kept_points[i + 1] - kept_points[i]
            elif i > 0:
                travel = kept_points[i] - kept_points[i - 1]
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
            # Note: the scan TARGET (the surface point itself, before the
            # incidence-cone step potentially relaxes the orientation) is
            # always recoverable later as position - normal * standoff_mm,
            # so it isn't stored redundantly here -- Waypoint uses
            # __slots__ deliberately to keep this a lean data container.

    return waypoints
