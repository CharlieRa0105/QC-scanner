"""
frame_transform.py

Part/CAD frame -> arm-base frame conversion for a scan path (architecture.md §6,
decision 5).

A scan path comes out of the planner in the PART frame: millimetres, Y-up (the
STEP file's own axes). The arm wants poses in its BASE frame: metres, Z-up, and
positioned relative to where the part actually sits in the cell. That last part
-- WHERE the part sits -- is the "marked-corner" calibration: a rigid transform,
measured once per cell, from the arm base to the reference corner the operator
places the part against.

So the full part -> arm transform is three things composed, in order:

  1. axis remap   Y-up -> Z-up        (fixed; a +90 deg rotation about X)
  2. unit scale   mm -> m             (fixed; x1/1000)
  3. corner pose  R_corner, t_corner  (MEASURED; from local_config.yaml)

which collapses to, for a part-frame point p (mm):

      p_arm = scale * (R_total @ p) + t_corner        R_total = R_corner @ R_axis

Orientations (the probe-pose quaternions) are rotated by R_total only -- scale
and translation don't affect a direction. Surface aim points and probe positions
are full points (all three steps).

This module is PURE math (numpy only) -- no file IO, no YAML. The config dict is
loaded elsewhere (libs/qc_config.py) and handed to `FrameTransform.from_config`.
That keeps it reusable by both the scanpath_convert.py CLI (the demo slice) and
the future PathPlanner ROS 2 node, which apply the same transform before MoveIt.

Dependencies: numpy.
"""

from dataclasses import dataclass

import numpy as np

# How many part-frame length units make one arm-frame unit, per named unit.
# (Everything is expressed relative to a metre so mm->m is just a lookup.)
_UNITS_PER_METRE = {"m": 1.0, "mm": 1000.0, "cm": 100.0}


def y_up_to_z_up_matrix():
    """
    Rotation mapping Y-up coordinates to Z-up coordinates: R_x(+90 deg).

    Sends the part's up-axis (+Y) onto the arm's up-axis (+Z):
        (x, y, z) -> (x, -z, y).
    It is a proper rotation (det = +1), so it preserves handedness.
    """
    return np.array(
        [[1.0, 0.0, 0.0],
         [0.0, 0.0, -1.0],
         [0.0, 1.0, 0.0]]
    )


def rotation_from_rpy_deg(roll_deg, pitch_deg, yaw_deg):
    """
    Build a rotation matrix from roll/pitch/yaw in degrees (R = Rz @ Ry @ Rx),
    i.e. intrinsic rotations about X then Y then Z of the arm base axes -- the
    common robotics convention. Used for the measured corner orientation.
    """
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def quaternion_to_matrix(q):
    """Rotation matrix from a unit quaternion [x, y, z, w]."""
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w) + 1e-12
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(m):
    """
    Quaternion [x, y, z, w] from a rotation matrix, via Shepperd's method
    (branch on the largest diagonal term to avoid dividing by ~0). Mirrors the
    generator's rotation_matrix_to_quaternion (scripts/plan_path.py) but takes a
    matrix rather than three axis columns.
    """
    trace = np.trace(m)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


@dataclass
class FrameTransform:
    """
    A part -> arm rigid transform with a uniform scale.

    Fields:
        rotation:    3x3 rotation R_total = R_corner @ R_axis (applied to both
                     points and orientations).
        scale:       part-unit -> arm-unit factor (mm -> m = 0.001).
        translation: (3,) corner position in the arm base frame, metres.
    """

    rotation: np.ndarray
    scale: float
    translation: np.ndarray

    @classmethod
    def from_config(cls, config):
        """
        Build the transform from a merged config dict (see libs/qc_config.py).

        Reads `frames` (units + up-axes) and `corner_transform` (the measured
        translation_m / rotation_deg). Only the documented Y-up -> Z-up, X->m
        style remap is implemented; anything else raises rather than silently
        producing a wrong frame.
        """
        frames = config.get("frames", {})
        part_up = str(frames.get("part_up_axis", "y")).lower()
        arm_up = str(frames.get("arm_up_axis", "z")).lower()
        if (part_up, arm_up) != ("y", "z"):
            raise NotImplementedError(
                f"only a Y-up part -> Z-up arm remap is implemented "
                f"(got part_up={part_up!r}, arm_up={arm_up!r})"
            )
        r_axis = y_up_to_z_up_matrix()

        part_units = str(frames.get("part_units", "mm")).lower()
        arm_units = str(frames.get("arm_units", "m")).lower()
        if part_units not in _UNITS_PER_METRE or arm_units not in _UNITS_PER_METRE:
            raise ValueError(f"unknown units: part={part_units!r}, arm={arm_units!r}")
        # e.g. mm -> m: (1/1000) / (1/1) = 0.001
        scale = _UNITS_PER_METRE[arm_units] / _UNITS_PER_METRE[part_units]

        corner = config.get("corner_transform", {})
        translation = np.asarray(corner.get("translation_m", [0.0, 0.0, 0.0]), dtype=float)
        roll, pitch, yaw = corner.get("rotation_deg", [0.0, 0.0, 0.0])
        r_corner = rotation_from_rpy_deg(roll, pitch, yaw)

        return cls(rotation=r_corner @ r_axis, scale=scale, translation=translation)

    def apply_point(self, p):
        """Transform a part-frame point (length units) to an arm-frame point (m)."""
        p = np.asarray(p, dtype=float)
        return self.scale * (self.rotation @ p) + self.translation

    def apply_quaternion(self, q):
        """
        Rotate a probe-orientation quaternion [x, y, z, w] from part to arm
        frame (rotation only -- scale/translation don't affect orientation).
        """
        m_arm = self.rotation @ quaternion_to_matrix(q)
        return matrix_to_quaternion(m_arm)


def transform_scanpath(data, transform):
    """
    Return a copy of a ScanPath dict (the plan_path.py JSON schema) with every
    waypoint moved into the arm base frame.

    Positions and targets go through the full point transform (rotate, scale,
    offset); quaternions are rotated; incidence_deg / line_id are geometry-frame
    invariant and copied. The `units` / `frame` header fields and a record of the
    transform that was applied are updated so the output is self-describing.
    """
    out = dict(data)  # shallow copy; we replace the fields we change below

    new_waypoints = []
    for wp in data.get("waypoints", []):
        new_wp = dict(wp)
        new_wp["position"] = [round(float(v), 6) for v in transform.apply_point(wp["position"])]
        new_wp["target"] = [round(float(v), 6) for v in transform.apply_point(wp["target"])]
        q = transform.apply_quaternion(wp["quaternion"])
        new_wp["quaternion"] = [float(v) for v in q]
        new_waypoints.append(new_wp)
    out["waypoints"] = new_waypoints

    out["units"] = "m"
    out["frame"] = "arm base (metres, Z-up; mm->m + Y-up->Z-up + marked-corner offset applied)"
    if "standoff_mm" in data:
        out["standoff_m"] = round(float(data["standoff_mm"]) * transform.scale, 6)
    out["frame_transform"] = {
        "scale": transform.scale,
        "translation_m": [float(v) for v in transform.translation],
        "rotation": [[float(v) for v in row] for row in transform.rotation],
        "note": "part->arm rigid transform (R_corner @ R_axis); see libs/path_planning/frame_transform.py",
    }
    return out
