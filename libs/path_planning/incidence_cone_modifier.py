"""
incidence_cone_modifier.py

Stage 6 of the PathPlanner pipeline: incidence-cone relaxation (see
docs/architecture.md and the Coverage Path Planning design notes).

The problem this solves: a raster planner that locks every waypoint's
orientation to the EXACT surface normal (0 degree incidence, always) forces
the arm's wrist to keep re-orienting to chase every small ripple in the
surface normal from one waypoint to the next. That's needless wrist motion
(risking singularities) and, on some finishes, exact 0 degree incidence can
cause specular glare on a structured-light scanner. This module smooths
orientation across neighbouring waypoints to cut that wrist motion, then
clamps the smoothed result back into a `max_incidence_angle` cone around the
TRUE surface normal, so optical validity (the scanner still sees the surface
close enough to square-on) is never violated even after smoothing.

This is a standalone numpy implementation, deliberately independent of
noether/ROS2, so the closed-form math (sliding-window average -> SLERP
clamp) can be verified on its own before it's ported into a real
noether_tpp ToolPathModifier C++ plugin for production use.

Smoothing runs PER RASTER LINE only -- a zig-zag turn between lines is a
real physical discontinuity in travel direction, not something that should
be smoothed through. Callers must therefore group waypoints by line_id
before calling apply_incidence_cone_relaxation (generate_raster_waypoints
in waypoint_generator.py already assigns line_id per waypoint).

Dependencies: numpy.
"""

import numpy as np


class Waypoint:
    """
    One probe waypoint before incidence-cone relaxation.

    Uses __slots__ (no ordinary instance __dict__) since many thousands of
    these get created per scan path -- keeps per-object memory overhead
    down for what is otherwise a plain data container.
    """

    __slots__ = ("position", "normal", "travel_direction", "line_id")

    def __init__(self, position, normal, travel_direction, line_id):
        self.position = np.asarray(position, dtype=float)
        # Normalize on construction so every consumer downstream can assume
        # unit length without re-checking.
        self.normal = _normalize(np.asarray(normal, dtype=float))
        self.travel_direction = _normalize(np.asarray(travel_direction, dtype=float))
        self.line_id = line_id


def _normalize(v):
    """Unit-length copy of v; raises rather than silently dividing by ~0."""
    norm = np.linalg.norm(v)
    if norm < 1e-12:
        raise ValueError("cannot normalize a zero-length vector")
    return v / norm


def _slerp(a, b, t):
    """
    Spherical linear interpolation between two unit vectors, a -> b, at
    parameter t in [0, 1]. Used (rather than a plain linear blend) because
    linearly blending two unit vectors and re-normalizing does not move at
    a constant angular rate and drifts off the great-circle arc between
    them -- SLERP stays exactly on the sphere and moves at constant angular
    speed, which is what "land exactly on the cone boundary" requires.
    """
    dot = np.clip(np.dot(a, b), -1.0, 1.0)
    theta = np.arccos(dot)
    if theta < 1e-9:
        # a and b already coincide -- interpolation is undefined (0/0 below)
        # but any point "between" two identical vectors is just that vector.
        return a
    sin_theta = np.sin(theta)
    return (np.sin((1 - t) * theta) / sin_theta) * a + (np.sin(t * theta) / sin_theta) * b


def smooth_normals(normals, window=2):
    """
    Sliding-window average of one raster line's sequence of normals, each
    result re-normalized back to unit length.

    Args:
        normals: ordered list/array of unit normals along a single raster
            line (must already be in travel order).
        window: half-width of the averaging window in each direction, so
            waypoint i averages normals[i-window : i+window+1]. Larger
            window = smoother orientation change = less wrist motion, but
            more aggressive deviation from the true local normal (which is
            exactly what the incidence-cone clamp below exists to bound).

    Returns:
        List of smoothed unit normals, same length and order as the input.

    Edge waypoints (near the start/end of the line) use a shrinking window
    instead of padding or wrapping the array -- a raster line is not
    periodic, and wrapping would incorrectly blend the end of one line's
    geometry into the start of it.
    """
    n = len(normals)
    smoothed = []
    for i in range(n):
        lo = max(0, i - window)
        hi = min(n, i + window + 1)
        avg = np.mean(normals[lo:hi], axis=0)
        smoothed.append(_normalize(avg))
    return smoothed


def clamp_to_incidence_cone(smoothed, true_normal, max_incidence_angle_rad):
    """
    Enforce the incidence-angle constraint on one waypoint's orientation.

    If the smoothed orientation is already within max_incidence_angle of
    the true surface normal, it's kept as-is -- the smoothing benefit is
    free in that case. If it has drifted further than that, it gets
    SLERP'd from the true normal toward the smoothed orientation just far
    enough to land EXACTLY on the cone boundary -- this is a clamp, not a
    discard, so as much of the smoothing benefit is kept as the constraint
    allows, rather than snapping all the way back to the raw normal.

    Args:
        smoothed: candidate smoothed unit normal for this waypoint.
        true_normal: this waypoint's original (pre-smoothing) unit normal
            -- the cone is always measured from the true surface geometry,
            never from another smoothed value, so the constraint reflects
            actual optical validity.
        max_incidence_angle_rad: half-angle of the allowed cone, in
            radians.

    Returns:
        (orientation, incidence_angle_rad) -- the final unit normal to use
        for this waypoint, and the incidence angle it ends up at (equal to
        max_incidence_angle_rad if it was clamped, otherwise the smoothed
        angle).
    """
    dot = np.clip(np.dot(smoothed, true_normal), -1.0, 1.0)
    angle = np.arccos(dot)

    if angle <= max_incidence_angle_rad or angle < 1e-9:
        return smoothed, angle

    # t is the fraction of the way from true_normal to smoothed that lands
    # exactly at max_incidence_angle_rad off true_normal (angle scales
    # linearly with t along a SLERP arc).
    t = max_incidence_angle_rad / angle
    clamped = _slerp(true_normal, smoothed, t)
    return clamped, max_incidence_angle_rad


def build_orientation_frame(z_axis, travel_direction):
    """
    Build a right-handed orientation frame for a waypoint from its
    (possibly relaxed) normal and the direction of travel.

    Z = z_axis (the relaxed surface normal -- the scanner points along
        -Z or +Z into the surface, depending on convention downstream).
    X = travel_direction, projected to be orthogonal to Z (so the frame
        stays right-angled even though the raw travel direction generally
        isn't exactly perpendicular to the normal).
    Y = Z cross X, completing a right-handed frame.

    This mirrors noether's own DirectionOfTravelOrientationModifier
    convention for the X/Y axes, so the production noether port and this
    R&D stand-in agree on what "orientation" means for a waypoint.
    """
    z = _normalize(z_axis)
    x_raw = travel_direction - np.dot(travel_direction, z) * z
    if np.linalg.norm(x_raw) < 1e-9:
        # Travel direction and normal are parallel -- there's no unique
        # orthogonal projection to build X from. Shouldn't happen for a
        # probe scanning a surface it's moving across, so this is treated
        # as a data problem to surface, not silently worked around.
        raise ValueError("travel direction is parallel to the surface normal")
    x = _normalize(x_raw)
    y = np.cross(z, x)
    return x, y, z


def apply_incidence_cone_relaxation(waypoints, max_incidence_angle_deg, window=2):
    """
    Run the full incidence-cone relaxation pass over a list of waypoints.

    Groups waypoints by line_id, smooths normals within each line
    independently, clamps each smoothed result back into the incidence
    cone around that waypoint's own true normal, and rebuilds a
    right-handed orientation frame from the result. Waypoint POSITION is
    never modified by this step -- only orientation.

    Args:
        waypoints: list of Waypoint objects (see class above), in any
            order -- they get grouped by line_id internally for smoothing,
            so waypoints from different lines may be interleaved in the
            input without affecting the result.
        max_incidence_angle_deg: incidence-cone half-angle limit, degrees.
        window: sliding-window half-width passed through to
            smooth_normals().

    Returns:
        List of dicts, one per input waypoint, in EXACTLY the same order as
        the input `waypoints` list (result[k] corresponds to waypoints[k]):
            {"position", "x_axis", "y_axis", "z_axis", "incidence_angle_deg"}

        Order-preservation matters: callers (e.g. plan_path.py) pair each
        input Waypoint with its result positionally and read fields from
        BOTH in the same output row (position/orientation from the result,
        target/line_id from the Waypoint). Grouping by line for smoothing
        must therefore never leak into the returned order, or those fields
        would be spliced across different waypoints. This is enforced below
        by scattering each line's results back into their original input
        slots rather than appending in grouped order.
    """
    max_angle_rad = np.radians(max_incidence_angle_deg)

    # Group the ORIGINAL input indices by line_id (not the Waypoint objects
    # themselves), so each computed result can be written straight back to
    # the slot its waypoint came from. dict preserves insertion order, but
    # we no longer rely on that for correctness -- the scatter-by-index
    # below makes the output order independent of grouping order.
    line_to_indices = {}
    for idx, wp in enumerate(waypoints):
        line_to_indices.setdefault(wp.line_id, []).append(idx)

    # Pre-size the output so results can be placed at their original index.
    results = [None] * len(waypoints)

    for indices in line_to_indices.values():
        line_waypoints = [waypoints[i] for i in indices]
        true_normals = [wp.normal for wp in line_waypoints]
        smoothed_normals = smooth_normals(true_normals, window=window)

        for idx, wp, smoothed_normal, true_normal in zip(
            indices, line_waypoints, smoothed_normals, true_normals
        ):
            clamped_z, angle_rad = clamp_to_incidence_cone(
                smoothed_normal, true_normal, max_angle_rad
            )
            x_axis, y_axis, z_axis = build_orientation_frame(clamped_z, wp.travel_direction)
            results[idx] = {
                "position": wp.position,
                "x_axis": x_axis,
                "y_axis": y_axis,
                "z_axis": z_axis,
                "incidence_angle_deg": np.degrees(angle_rad),
            }

    return results
