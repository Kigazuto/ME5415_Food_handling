"""
Food item factory functions for the Food Tray Assembly scene.

Two construction modes are exposed:

* `add_food_deformable(...)` — full FEM body (volume tet mesh +
  TetrahedronFEMForceField + LinearSolverConstraintCorrection +
  Barycentric collision/visual). This is used for the food the
  gripper actively interacts with, because FEM-on-FEM contact under
  Lagrangian friction is what gives a stable, compliance-rich grasp.

* `add_food_visual(...)` — a lightweight static prop: just an
  OglModel of the STL mesh placed in world. Used for the eight
  food categories that are *displayed* on the Source Table to show
  scene coverage but are not actively grasped in the demo.

Material parameters per food are taken from the reference project
`ME5415_Food_Gripper/PickObjects.py` (literature values for cooked
versions of each food).
"""

# (E [MPa], nu, scale, mass_kg)
# Hardness ordering for the four active foods:
#   bean (softest) ≪ sausage (medium) ≪ meatball ≈ broccoli (effectively rigid)
# Meatball and broccoli are "hard solid" food in real life so we
# crank their E to 500 MPa — well above the gripper's silicone
# (590) — so they don't visibly deform under the grasp.
FOOD_PARAMS = {
    # name        E      nu    scale  mass
    "meatball":  (1500.0, 0.30, 1.0,  0.020),  # near-rigid, tiny contact patch
    "broccoli":  (1500.0, 0.30, 1.0,  0.020),  # near-rigid
    "sausage":   (30.0,  0.49, 0.5,   0.020),  # firm (stable at high E)
    "bean":      (0.6,   0.49, 1.0,   0.0001), # cooked, very soft
    "carrot":    (3.0,   0.40, 0.6,   0.100),
    "spaghetti": (0.5,   0.49, 1.0,   0.001),
    "cookie":    (50.0,  0.30, 1.0,   0.010),
    "eggs":      (0.8,   0.49, 1.0,   0.005),
    "cup":       (2000.0, 0.35, 1.0,  0.300),
}

# Mesh centroid + axis-aligned bounding box in the native
# (unscaled) mesh frame, computed once from the VTK point lists.
# Used by add_food_visual() to place each food so its *bottom*
# (mesh-X minimum) sits at a chosen world X regardless of where
# the mesh is defined relative to its own origin.
FOOD_CENTROIDS = {
    "meatball":  (-0.6,  15.1,  0.2),
    "sausage":   (110.4, 145.4, 15.3),
    "broccoli":  (-0.2,   0.1, 21.4),
    "carrot":    (-0.2,  83.3, 24.4),
    "bean":      (15.4,  25.5,  9.2),
    "spaghetti": ( 1.0,  20.0,  1.0),
    "cookie":    (34.0,  -0.3,  7.5),
    "eggs":      (13.9,  16.0,  4.9),
    "cup":       (-0.1,  61.6,  0.0),
}
# (xmin, xmax) — the mesh-X extent, used to locate the bottom face.
FOOD_X_BOUNDS = {
    "meatball":  (-15.0,  15.0),
    "sausage":   ( 95.2, 125.2),
    "broccoli":  (-15.0,  15.0),
    "carrot":    (-24.5,  24.4),
    "bean":      ( -2.9,  29.8),
    "spaghetti": (  0.0,   2.0),
    "cookie":    (  0.0,  70.0),
    "eggs":      ( -6.0,  41.4),
    "cup":       (-44.8,  44.8),
}

FOOD_COLOURS = {
    "meatball":  [0.55, 0.30, 0.18, 1.0],
    "sausage":   [0.75, 0.35, 0.25, 1.0],
    "broccoli":  [0.20, 0.55, 0.25, 1.0],
    "carrot":    [0.92, 0.55, 0.15, 1.0],
    "bean":      [0.30, 0.65, 0.20, 1.0],
    "spaghetti": [0.95, 0.85, 0.50, 1.0],
    "cookie":    [0.85, 0.65, 0.35, 1.0],
    "eggs":      [0.97, 0.93, 0.70, 1.0],
    "cup":       [0.95, 0.55, 0.10, 0.85],
}


def add_food_deformable(root, name, mesh_basename, world_centroid,
                        rotation=(0, 0, 0)):
    """Add a deformable FEM food item.

    `world_centroid` is the world-space position where the mesh
    centroid should sit. The loader translation is computed from
    `world_centroid - scale * mesh_centroid` so different meshes
    line up regardless of their native origin.
    """
    young, poisson, scale, mass = FOOD_PARAMS[mesh_basename]
    color = FOOD_COLOURS[mesh_basename]
    cx, cy, cz = FOOD_CENTROIDS[mesh_basename]
    position = [
        world_centroid[0] - scale * cx,
        world_centroid[1] - scale * cy,
        world_centroid[2] - scale * cz,
    ]

    node = root.addChild(name)
    node.addObject("EulerImplicitSolver", name="odesolver",
                   rayleighStiffness=0.1, rayleighMass=0.1)
    node.addObject("SparseLDLSolver", name="preconditioner",
                   template="CompressedRowSparseMatrixd")
    node.addObject("MeshVTKLoader", name="loader",
                   filename=f"object/mesh/{mesh_basename}.vtk",
                   translation=list(position), scale=scale,
                   rotation=list(rotation))
    node.addObject("MeshTopology", src="@loader", name="container")
    node.addObject("MechanicalObject", name="tetras", template="Vec3d")
    node.addObject("UniformMass", totalMass=mass)
    node.addObject("ParallelTetrahedronFEMForceField", template="Vec3d", name="FEM",
                   method="large", poissonRatio=poisson, youngModulus=young)
    node.addObject("LinearSolverConstraintCorrection")

    col = node.addChild("Collision")
    col.addObject("MeshSTLLoader", name="loader",
                  filename=f"object/mesh/{mesh_basename}.stl",
                  translation=list(position), scale=scale,
                  rotation=list(rotation))
    col.addObject("MeshTopology", src="@loader", name="topo")
    col.addObject("MechanicalObject", name="collisMech")
    col.addObject("TriangleCollisionModel", selfCollision=False)
    col.addObject("LineCollisionModel", selfCollision=False)
    col.addObject("PointCollisionModel", selfCollision=False)
    col.addObject("BarycentricMapping")

    vis = node.addChild("Visu")
    vis.addObject("MeshSTLLoader", name="loader",
                  filename=f"object/mesh/{mesh_basename}.stl",
                  translation=list(position), scale=scale,
                  rotation=list(rotation))
    vis.addObject("OglModel", src="@loader", color=color)
    vis.addObject("BarycentricMapping")
    return node


def add_food_visual(root, name, mesh_basename, world_bottom,
                    rotation=(0, 0, 0)):
    """Add a static visual-only food prop.

    `world_bottom` is `(X, Y, Z)`: X is the world height where the
    food's bottom face should sit (so heights line up regardless of
    each mesh's vertical extent), Y/Z are the horizontal centroid.
    """
    _, _, scale, _ = FOOD_PARAMS[mesh_basename]
    color = FOOD_COLOURS[mesh_basename]
    cx, cy, cz = FOOD_CENTROIDS[mesh_basename]
    xmin, _ = FOOD_X_BOUNDS[mesh_basename]
    bx, by, bz = world_bottom
    translation = [
        bx - scale * xmin,    # bottom of mesh-X aligns with bx
        by - scale * cy,      # Y centroid at by
        bz - scale * cz,      # Z centroid at bz
    ]

    node = root.addChild(name)
    node.addObject("MeshSTLLoader", name="loader",
                   filename=f"object/mesh/{mesh_basename}.stl",
                   translation=translation, scale=scale,
                   rotation=list(rotation))
    node.addObject("OglModel", name="Visual", src="@loader", color=color)
    return node
