"""
normal_estimation.py

Stage 2 of the PathPlanner pipeline: surface sampling + normal estimation.

Takes the (vertices, faces) triangle mesh from cad_loader.py and samples
points across its surface, each with an outward-facing normal. Those
normals are what the rest of the pipeline uses to place the scanner probe:
the probe sits along the normal at the configured standoff distance,
facing the surface square on.

Sampling method: AREA-WEIGHTED random sampling. Each triangle gets a
probability proportional to its own surface area, so large flat faces
receive proportionally more sample points than small ones -- this keeps
point density roughly uniform across the surface regardless of how
unevenly the CAD tessellation happened to divide it up.

Outward orientation: gmsh usually emits consistently-oriented faces for a
solid, but this code does NOT rely on that assumption holding. Instead,
every face normal is flipped if it points back toward the mesh's own
centroid instead of away from it. This is a "star-convex" heuristic --
correct for plates, brackets, and most machined parts (anything roughly
convex as seen from its own centre), but it can mis-orient normals inside
deep concavities where a line from the centroid to the surface point
doesn't cleanly point "outward". Flagged here so it isn't mistaken for an
exact method.

Dependencies: numpy.
"""

import numpy as np


def _face_geometry(vertices, faces):
    """
    Compute per-face unit normal, centroid, and area for every triangle.

    Args:
        vertices: (N, 3) array of mesh vertex positions.
        faces: (M, 3) array of vertex indices, one row per triangle.

    Returns:
        (unit_normal, centroid, area, v0, v1, v2) -- each of the first
        three is (M, 3) or (M,); v0/v1/v2 are the (M, 3) arrays of each
        triangle's three corner points (handed back so callers doing
        barycentric sampling don't have to re-index vertices/faces again).
    """
    v0 = vertices[faces[:, 0]]
    v1 = vertices[faces[:, 1]]
    v2 = vertices[faces[:, 2]]

    # The cross product of two edges gives a vector perpendicular to the
    # triangle, with magnitude = 2 * triangle area.
    cross = np.cross(v1 - v0, v2 - v0)
    cross_norm = np.linalg.norm(cross, axis=1)
    area = 0.5 * cross_norm

    # Guard against degenerate (zero-area, e.g. sliver) triangles before
    # dividing by their norm -- avoids a 0/0 producing NaN normals that
    # would silently poison every downstream calculation.
    safe_norm = np.where(cross_norm < 1e-12, 1.0, cross_norm)
    unit_normal = cross / safe_norm[:, None]

    centroid = (v0 + v1 + v2) / 3.0

    return unit_normal, centroid, area, v0, v1, v2


def surface_area(vertices, faces):
    """
    Total surface area of the mesh, in the CAD's own units squared (mm^2).

    Sum of every triangle's area. Used to scale the raster density to the
    part: a small part has a small area, so deriving waypoint spacing from
    area (spacing ~ sqrt(area / target_count)) keeps even a tiny part from
    collapsing to one line per face -- see plan_path.py's --target-waypoints.
    """
    _, _, area, _, _, _ = _face_geometry(
        np.asarray(vertices, dtype=float), np.asarray(faces, dtype=np.int64)
    )
    return float(area.sum())


def sample_surface(vertices, faces, n_samples, seed=0):
    """
    Draw an area-weighted random sample of points across the mesh surface,
    each with an outward-facing unit normal.

    Args:
        vertices: (N, 3) mesh vertex array (from cad_loader.load_cad).
        faces: (M, 3) mesh face array (from cad_loader.load_cad).
        n_samples: how many surface points to generate. More points give
            finer downstream raster coverage at the cost of more waypoints
            to process; this is the main density knob for the whole
            pipeline.
        seed: RNG seed, so a run is exactly reproducible given the same
            mesh and sample count.

    Returns:
        (points, normals): both (n_samples, 3) numpy arrays. `points` are
        surface locations; `normals` are outward unit normals at each
        point.
    """
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=np.int64)

    unit_normal, centroid, area, v0, v1, v2 = _face_geometry(vertices, faces)

    total_area = area.sum()
    if total_area <= 0:
        raise ValueError("mesh has zero total surface area -- check the input CAD file")

    # Probability of choosing each face is proportional to its own area,
    # so a face twice as big is twice as likely to be sampled -- this is
    # what makes the resulting point cloud density-uniform.
    face_probs = area / total_area

    rng = np.random.default_rng(seed)
    face_idx = rng.choice(len(faces), size=n_samples, p=face_probs)

    # Uniform random point within a triangle via barycentric coordinates.
    # Drawing (r1, r2) uniformly on the unit square and reflecting the
    # part that falls outside the triangle back in (the "over" fold) is
    # the standard trick for an unbiased uniform sample inside a triangle.
    r1 = rng.random(n_samples)
    r2 = rng.random(n_samples)
    over = (r1 + r2) > 1.0
    r1[over] = 1.0 - r1[over]
    r2[over] = 1.0 - r2[over]

    a = v0[face_idx]
    b = v1[face_idx]
    c = v2[face_idx]
    points = a + r1[:, None] * (b - a) + r2[:, None] * (c - a)
    normals = unit_normal[face_idx].copy()

    # Outward-flip heuristic: use the mesh's own AREA-WEIGHTED centroid
    # (not a plain vertex average, which would be skewed by however
    # densely gmsh happened to tessellate different regions) as a stand-in
    # for "the inside" of the part. Any normal pointing back toward that
    # centroid instead of away from it gets flipped.
    mesh_centroid = np.average(centroid, axis=0, weights=area)
    outward_direction = points - mesh_centroid
    points_inward = np.einsum("ij,ij->i", normals, outward_direction) < 0.0
    normals[points_inward] *= -1.0

    return points, normals
