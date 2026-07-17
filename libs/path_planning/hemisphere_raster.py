"""
hemisphere_raster.py

Dome coverage planner: instead of rastering the part's own (arbitrarily complex)
surface, wrap the part in the smallest enclosing HEMISPHERE with its flat face on
the table, and raster the scan path over THAT dome -- scanner on the dome surface,
always aimed inward at the part. Geometry-agnostic: the path depends only on the
part's bounding dome, so it behaves identically for a pin, a bracket, or a casting.

  1. FIT -- smallest hemisphere (centre on the table plane, dome opening upward)
     that contains every part point. With the centre constrained to the table
     plane, this is a small 2-D min-max ("smallest enclosing sphere, centre on a
     plane") solved numerically.
  2. RASTER -- lay a boustrophedon over the dome: elevation rings from the pole
     down to the equator (table), points spaced along each ring by arc length.
     Each waypoint sits standoff OUTSIDE the containing dome and its normal points
     radially OUTWARD (so the scanner, at position, looks inward along -normal at
     the part centre).
  3. The fitted (centre, radius) is returned too, so the CLI can record it and the
     debug viewport can draw the dome.

Reachability is not considered here (a separate concern). Output is the same
incidence_cone_modifier.Waypoint list as every planner, so downstream is unchanged.

Dependencies: numpy, scipy (fit).
"""

import math

import numpy as np

from .incidence_cone_modifier import Waypoint


def fit_containing_hemisphere(vertices, up_axis=1):
    """
    Smallest hemisphere containing all `vertices`, flat face on the table (the
    plane at the parts' minimum up-coordinate), dome opening along +up_axis.

    Returns (centre, radius): centre is a 3-vector on the table plane, radius the
    dome radius. Minimises max-point-distance over centres constrained to the
    plane -- a convex 2-D min-max, solved with Nelder-Mead from the footprint
    centroid (few vertices, converges fast and deterministically).
    """
    from scipy.optimize import minimize

    v = np.asarray(vertices, dtype=float)
    others = [i for i in range(3) if i != up_axis]
    ground = float(v[:, up_axis].min())
    foot = v[:, others]                      # projection onto the table plane
    height_sq = (v[:, up_axis] - ground) ** 2

    def max_r2(c):
        return float(np.max((foot[:, 0] - c[0]) ** 2 + (foot[:, 1] - c[1]) ** 2 + height_sq))

    c0 = foot.mean(axis=0)
    res = minimize(max_r2, c0, method="Nelder-Mead",
                   options={"xatol": 1e-4, "fatol": 1e-4, "maxiter": 2000})
    cx, cy = res.x
    radius = math.sqrt(max_r2(res.x))
    centre = np.zeros(3)
    centre[others[0]] = cx
    centre[others[1]] = cy
    centre[up_axis] = ground
    return centre, radius


def generate_hemisphere_waypoints(
    vertices,
    standoff_mm,
    line_spacing_mm,
    along_track_mm,
    up_axis=1,
    log=print,
):
    """
    Build a boustrophedon coverage path over the part's containing hemisphere.

    Args:
        vertices: part mesh vertices (placed frame, CAD units mm).
        standoff_mm: how far OUTSIDE the containing dome the scanner sits.
        line_spacing_mm: arc gap between elevation rings.
        along_track_mm: arc gap between points along a ring.
        up_axis: which axis is up (default 1 = +Y, the pipeline convention).
        log: progress sink.

    Returns:
        (waypoints, centre, radius): waypoints is the Waypoint list (normal points
        radially outward; scanner at position looks inward); centre/radius describe
        the fitted containing hemisphere (for recording + drawing).
    """
    centre, radius = fit_containing_hemisphere(vertices, up_axis)
    r_scan = radius + standoff_mm            # scanner rides this dome, standoff outside

    UP = np.eye(3)[up_axis]
    others = [i for i in range(3) if i != up_axis]
    e0, e1 = np.eye(3)[others[0]], np.eye(3)[others[1]]

    # The flange normal (tool +Z = scanner look axis) must point AT the hemisphere
    # centre from every dome position, so the flange sits on the dome looking IN at
    # the part with its body OUTSIDE the dome. normal therefore points INWARD, from
    # the pose toward the centre; the resulting waypoint orientation puts +Z on the
    # centre.
    def aim(position):
        n = centre - position
        norm = np.linalg.norm(n)
        return n / norm if norm > 1e-9 else -UP

    waypoints = []
    line_id = 0
    flip = False

    # elevation rings: theta 0 at the pole (straight up) -> pi/2 at the equator (table)
    d_theta = line_spacing_mm / r_scan
    n_theta = max(1, int(round((math.pi / 2) / d_theta)))
    for it in range(n_theta + 1):
        theta = (math.pi / 2) * it / n_theta
        ring_radius = r_scan * math.sin(theta)
        if ring_radius < 1e-6:               # pole -> a single point straight up
            pos = centre + r_scan * UP
            waypoints.append(Waypoint(position=pos, normal=aim(pos),
                                      travel_direction=e0, line_id=line_id))
            line_id += 1
            continue

        d_phi = along_track_mm / ring_radius
        n_phi = max(3, int(round(2 * math.pi / d_phi)))
        phis = [2 * math.pi * k / n_phi for k in range(n_phi)]
        if flip:
            phis = phis[::-1]
        flip = not flip

        def dir_at(phi):
            d = math.sin(theta) * (math.cos(phi) * e0 + math.sin(phi) * e1) + math.cos(theta) * UP
            return d / np.linalg.norm(d)

        for k, phi in enumerate(phis):
            direction = dir_at(phi)
            position = centre + r_scan * direction
            nxt = phis[(k + 1) % n_phi]
            travel = (centre + r_scan * dir_at(nxt)) - position
            if np.linalg.norm(travel) < 1e-9:
                travel = e0
            waypoints.append(Waypoint(position=position, normal=aim(position),
                                      travel_direction=travel, line_id=line_id))
        line_id += 1

    log(f"[hemisphere_raster] dome radius {radius:.1f}mm (scan {r_scan:.1f}mm), "
        f"{line_id} rings, {len(waypoints)} waypoints "
        f"(line {line_spacing_mm:.2f}mm, along {along_track_mm:.2f}mm)")
    return waypoints, centre, radius
