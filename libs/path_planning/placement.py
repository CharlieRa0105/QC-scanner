"""
placement.py

Resting placement: how the part sits on the table before scanning.

The rest of the pipeline uses the CAD "Y-up" convention -- scanpath_convert rotates
Y-up -> Z-up for the arm. But CAD files aren't authored with the scan-up face on +Y
(these sample parts have their LONGEST axis on Y, so a naive "Y is up" would stand
them on end). This module picks a sensible resting orientation instead: lay the part
on its LARGEST face by rotating its smallest-extent axis onto +Y. The scanner then
sweeps the broad up-facing surface -- which is what the operator would actually
present to an overhead scanner.

The SAME rotation must be applied to the mesh and to the planned path, or they drift
apart in the viewer. So the planner records it in the ScanPath JSON ("placement_R")
and export_viewer_bundle applies the identical rotation to the mesh. Planners that
don't set it (the older grid/contour planners) default to identity, unchanged.

Deterministic (from the bounding box), pure numpy.
"""

import numpy as np


def resting_rotation(vertices):
    """
    3x3 rotation R mapping raw CAD coords -> resting placement, chosen so the
    part's smallest-extent axis points along +Y (the pipeline's up axis) -- i.e.
    the part lies on its largest face.

    Proper rotations (det +1), so handedness and the mesh winding are preserved.
    """
    v = np.asarray(vertices, dtype=float)
    extent = v.max(axis=0) - v.min(axis=0)
    up = int(np.argmin(extent))
    if up == 1:                      # already Y-up
        return np.eye(3)
    if up == 0:                      # X -> Y : +90 deg about Z
        return np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    return np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])  # Z -> Y : -90 about X


def apply_rotation(points, R):
    """Rotate an (N,3) array of points by R (maps each p -> R @ p)."""
    return np.asarray(points, dtype=float) @ np.asarray(R, dtype=float).T
