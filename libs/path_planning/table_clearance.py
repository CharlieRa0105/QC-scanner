"""
table_clearance.py

Table-collision floor: guarantee no scan waypoint's scanner sits within a set
clearance of the table. Any waypoint below the clearance is DELETED, so nothing
that survives is closer than the clearance to the table.

Operates in the PLACED part frame (the frame every planner emits: mm, +Y up), so
a waypoint's height above the table is (position.y - table_up), where `table_up`
is the placed-frame Y of the table plane (the part's lowest point, recorded by the
planners as `table_up_mm`). Applying it here -- before scanpath_convert remaps to
the arm frame -- means the cull flows into BOTH the arm-frame path the robot runs
and the viewer bundle, with no frame maths repeated.

Note: for a part whose entire scan envelope is below the clearance, this removes
every waypoint (empty path). That is the intended, literal behaviour -- lower the
clearance (QC_TABLE_CLEARANCE_MM) for such parts.
"""


def apply_table_clearance(data, clearance_mm=150.0, up_axis=1):
    """
    Delete (in place) every waypoint whose scanner sits within `clearance_mm` of
    the table; renumber the survivors' `i`. Returns (data, n_removed).

    Needs `data["table_up_mm"]` (placed-frame table plane); if absent, does nothing
    (older planners that don't record it are left untouched).
    """
    ground = data.get("table_up_mm")
    if ground is None:
        return data, 0

    floor = ground + clearance_mm
    waypoints = data.get("waypoints", [])
    kept = [wp for wp in waypoints if wp["position"][up_axis] >= floor]
    removed = len(waypoints) - len(kept)
    for i, wp in enumerate(kept):
        wp["i"] = i
    data["waypoints"] = kept
    return data, removed
