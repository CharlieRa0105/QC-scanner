"""
cad_loader.py

Stage 1 of the PathPlanner pipeline: CAD ingestion.

Takes a CAD file path (STEP, STL, or OBJ) and returns an in-memory
triangle mesh (vertices + faces) for the rest of the pipeline
(normal_estimation.py, waypoint_generator.py, incidence_cone_modifier.py)
to operate on.

Why gmsh instead of Open3D:
Open3D can only read formats that are ALREADY tessellated meshes
(STL/OBJ/PLY). STEP is a B-rep (boundary representation) format -- it
describes exact mathematical surfaces (NURBS patches, trimmed curves),
not triangles. There is no vertex/face list to "just read" until
something approximates those exact surfaces with flat triangles at a
chosen resolution. gmsh understands B-rep and does that tessellation,
so it's used here for all inputs (STEP and already-meshed formats
alike) to keep a single code path instead of branching per file type.
"""

import os

import gmsh
import numpy as np

# gmsh's element-type code for a straight 3-node (linear) triangle.
# Surface meshing (dim=2) with default settings only ever emits these,
# but we check the type explicitly below rather than assuming it.
GMSH_TRIANGLE_ELEMENT_TYPE = 2


def load_cad(filepath, mesh_size=None):
    """
    Load a CAD file and tessellate it into a triangle mesh.

    Args:
        filepath: path to a .step/.stp/.stl/.obj file.
        mesh_size: target element size, in the CAD file's native units
            (STEP files from CAD tools are typically authored in mm).
            Smaller -> finer mesh -> more triangles -> more accurate
            surface representation but slower downstream normal/raster
            computation. Larger -> coarser and faster, but risks
            missing small features (holes, fillets) that matter for
            coverage planning. If None, gmsh chooses its own default
            sizing from the model's bounding box.

    Returns:
        (vertices, faces):
            vertices - (N, 3) float64 numpy array of XYZ points.
            faces    - (M, 3) int64 numpy array of vertex indices,
                       one row per triangle.
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"CAD file not found: {filepath}")

    gmsh.initialize()
    try:
        # Silence gmsh's own stdout logging -- otherwise every call
        # spams the terminal with meshing progress lines.
        gmsh.option.setNumber("General.Terminal", 0)

        if mesh_size is not None:
            # Setting both Max and Min (Min as a fraction of Max) keeps
            # the mesh roughly uniform instead of letting gmsh mix
            # wildly different triangle sizes across the part.
            gmsh.option.setNumber("Mesh.MeshSizeMax", mesh_size)
            gmsh.option.setNumber("Mesh.MeshSizeMin", mesh_size * 0.1)

        gmsh.open(filepath)

        # Generate a surface (2D) mesh only. A volume (3D) mesh isn't
        # needed -- the scanner only ever sees the part's outer skin,
        # never its interior.
        gmsh.model.mesh.generate(2)

        # --- Vertices ---
        # getNodes() returns parallel arrays: gmsh's own node tags, and
        # a flat [x0,y0,z0, x1,y1,z1, ...] coordinate array.
        node_tags, node_coords, _ = gmsh.model.mesh.getNodes()
        vertices = node_coords.reshape(-1, 3)

        # gmsh node tags are not guaranteed to be a dense 0..N-1 range
        # (they can have gaps, e.g. after internal remeshing), so build
        # an explicit tag -> row-index lookup before reading element
        # connectivity below. Indexing into `vertices` with a raw gmsh
        # tag would silently point at the wrong row otherwise.
        tag_to_index = {tag: i for i, tag in enumerate(node_tags)}

        # --- Faces ---
        # getElements(dim=2) returns parallel arrays across all element
        # *types* present (triangles, possibly quads): element type
        # codes, element tags, and each type's flattened node-tag list.
        elem_types, _elem_tags, elem_node_tags = gmsh.model.mesh.getElements(dim=2)

        faces = []
        for elem_type, node_tags_for_type in zip(elem_types, elem_node_tags):
            if elem_type != GMSH_TRIANGLE_ELEMENT_TYPE:
                # Skip any non-triangular elements (e.g. quads). Default
                # surface meshing shouldn't produce these, but guarding
                # here avoids silently misreading the connectivity array
                # with the wrong stride if it ever does.
                continue

            # Each triangle contributes 3 consecutive node tags to the
            # flat array -- reshape to one row per triangle.
            triangles = node_tags_for_type.reshape(-1, 3)
            for triangle in triangles:
                faces.append([tag_to_index[tag] for tag in triangle])

        faces = np.array(faces, dtype=np.int64)

    finally:
        # Always finalize, even if something above raised -- otherwise
        # gmsh's global state leaks into the next load_cad() call made
        # in the same process.
        gmsh.finalize()

    if len(faces) == 0:
        raise ValueError(
            f"No surface triangles were generated from {filepath}. "
            "Check that the file contains valid surface geometry."
        )

    return vertices, faces


if __name__ == "__main__":
    # Quick manual smoke test: point this at a real CAD file and confirm
    # a sane-looking mesh comes back before wiring it into the pipeline.
    import sys

    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <path_to_cad_file> [mesh_size]")
        sys.exit(1)

    test_filepath = sys.argv[1]
    test_mesh_size = float(sys.argv[2]) if len(sys.argv) > 2 else None

    verts, tris = load_cad(test_filepath, mesh_size=test_mesh_size)
    print(f"Loaded {test_filepath}")
    print(f"  vertices: {verts.shape}")
    print(f"  faces:    {tris.shape}")
    print(f"  bounding box min: {verts.min(axis=0)}")
    print(f"  bounding box max: {verts.max(axis=0)}")
