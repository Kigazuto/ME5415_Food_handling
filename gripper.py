"""
3-finger PneuNet soft gripper for the Food Tray Assembly scene.

This is a port of the gripper from the `ME5415_Food_Gripper`
reference project: it uses the custom finger_rot.vtk / cavitywhole_rot.vtk
volume meshes and a `finger_hooked_rot.stl` collision shell with a
hooked fingertip (visualised separately as a yellow `fingernail`).
The mesh data lives in `object/mesh/` (real copies, not symlinks).

Compared with the original SoftRobots PneunetGripper tutorial this
gripper:

* uses a stiffer silicone (E = 590, stiff layer 5500)
* clamps the base with `FixedProjectiveConstraint` (instead of a
  rest-shape spring) and a `LinearSolverConstraintCorrection`
* has a hooked fingertip — the inward kink at the tip helps the
  three fingers keep a round food (meatball, sausage end, etc.) in
  place once they close around it
"""

import math

# --- Material parameters (from reference) ---
YOUNG_FINGER = 590
YOUNG_STIFF_LAYER = 5500

# --- Radial layout: N fingers evenly distributed (2π/N) around the
#     gripper's central axis (which is parallel to gravity, -X).
#     The reference project switched from 3 to 6 fingers and uses a
#     uniform `2π i / N` layout instead of the original 3-only
#     formula — we follow suit. ---
NUM_FINGERS = 6
RADIUS = 35
ANGLES = [2 * math.pi * i / NUM_FINGERS for i in range(NUM_FINGERS)]

# Where the gripper is anchored in world space.
GRIPPER_BASE = [50.0, 0.0, -300.0]


def _finger_translation(idx):
    """Place each finger's base on a circle of radius RADIUS in the
    YZ plane (perpendicular to gravity), centred on the gripper's
    central axis. Uses the reference project's layout formula so
    the local frame is consistent with how the cavity-side angle
    rotation is applied later."""
    a = ANGLES[idx]
    local_y = RADIUS + RADIUS * math.sin(a - math.pi / 2)
    local_z = RADIUS * math.cos(a - math.pi / 2)
    return [GRIPPER_BASE[0],
            GRIPPER_BASE[1] + local_y,
            GRIPPER_BASE[2] + local_z]


def _finger_rotation(idx):
    """The mesh has been pre-rotated (`*_rot.vtk` / `*_rot.stl`) so
    its length axis points along world -X. All that remains is
    rotation around the central X axis to spread the N fingers
    radially. The angle is negated so each finger's cavity faces
    the central axis (curl direction inward).
    """
    angle_deg = -ANGLES[idx] * 180.0 / math.pi
    return [angle_deg, 0.0, 0.0]


def add_gripper(root, initial_pressure=0.0):
    """Build the N-finger gripper under `root`.

    `initial_pressure` lets the fingers start slightly pre-curled.
    """
    box_refs = {}
    for i in range(NUM_FINGERS):
        _add_finger(root, i, box_refs, initial_pressure)


def _add_finger(root, idx, box_refs, initial_pressure):
    rotation = _finger_rotation(idx)
    translation = _finger_translation(idx)

    finger = root.addChild(f"Finger{idx + 1}")
    finger.addObject("EulerImplicitSolver", name="odesolver",
                     rayleighStiffness=0.1, rayleighMass=0.1)
    finger.addObject("SparseLDLSolver", name="preconditioner",
                     template="CompressedRowSparseMatrixd")
    finger.addObject("MeshVTKLoader", name="loader",
                     filename="object/mesh/finger_rot.vtk",
                     rotation=rotation, translation=translation)
    finger.addObject("MeshTopology", src="@loader", name="container")
    finger.addObject("MechanicalObject", name="tetras", template="Vec3d")
    finger.addObject("UniformMass", totalMass=0.0001)
    finger.addObject("ParallelTetrahedronFEMForceField", template="Vec3d", name="FEM",
                     method="large", poissonRatio=0.3,
                     youngModulus=YOUNG_FINGER)

    # BoxROIs are created on finger 1 only and reused by 2/3 — same
    # mesh topology means the same vertex indices represent the
    # base / backbone region in every finger.
    if idx == 0:
        bx, by, bz = GRIPPER_BASE
        # Finger 1 (no Rx rotation) occupies, in world coords,
        # X∈[bx-77, bx], Y∈[by-20, by], Z∈[bz-10, bz+10].
        # The "base" is the high-X end of the finger.
        boxROI = finger.addObject(
            "BoxROI", name="boxROI",
            box=[bx - 2, by - 25, bz - 15, bx + 5, by + 5, bz + 15],
            doUpdate=False,
        )
        # The stiff backbone strip runs the entire length of the
        # finger on the side opposite the cavity. A wide box that
        # encloses the whole finger volume catches it.
        boxROISubTopo = finger.addObject(
            "BoxROI", name="boxROISubTopo",
            box=[bx - 80, by - 25, bz - 15, bx + 5, by + 5, bz + 15],
            strict=False,
        )
        box_refs["box"] = boxROI
        box_refs["sub"] = boxROISubTopo
    boxROI = box_refs["box"]
    boxROISubTopo = box_refs["sub"]

    finger.addObject("RestShapeSpringsForceField",
                     points=boxROI.indices.linkpath,
                     stiffness=1e12, angularStiffness=1e12)
    finger.addObject("LinearSolverConstraintCorrection")

    # Stiff backbone strip — added on EVERY finger so all three
    # have the same composite stiffness. The BoxROI was computed
    # once on finger 1, but the same vertex indices identify the
    # backbone on every finger because all three share the mesh
    # topology.
    sub = finger.addChild("SubTopology")
    sub.addObject("TetrahedronSetTopologyContainer",
                  position="@../loader.position",
                  tetrahedra=boxROISubTopo.tetrahedraInROI.linkpath,
                  name="container")
    sub.addObject("ParallelTetrahedronFEMForceField", template="Vec3d", name="FEM",
                  method="large", poissonRatio=0.3,
                  youngModulus=YOUNG_STIFF_LAYER - YOUNG_FINGER)

    # Pneumatic cavity (`cavitywhole_rot.vtk` is a closed surface)
    cavity = finger.addChild("Cavity")
    cavity.addObject("MeshVTKLoader", name="loader",
                     filename="object/mesh/cavitywhole_rot.vtk",
                     translation=translation, rotation=rotation)
    cavity.addObject("MeshTopology", src="@loader", name="topo")
    cavity.addObject("MechanicalObject", name="cavity")
    cavity.addObject("SurfacePressureConstraint",
                     name="SurfacePressureConstraint", template="Vec3",
                     value=initial_pressure, triangles="@topo.triangles",
                     valueType="pressure")
    cavity.addObject("BarycentricMapping", name="mapping",
                     mapForces=False, mapMasses=False)

    # Collision skin (with hooked fingertip)
    col = finger.addChild("Collision")
    col.addObject("MeshSTLLoader", name="loader",
                  filename="object/mesh/finger_hooked_rot.stl",
                  translation=translation, rotation=rotation)
    col.addObject("MeshTopology", src="@loader", name="topo")
    col.addObject("MechanicalObject", name="collisMech")
    col.addObject("TriangleCollisionModel", selfCollision=False)
    col.addObject("LineCollisionModel", selfCollision=False)
    col.addObject("PointCollisionModel", selfCollision=False)
    col.addObject("BarycentricMapping")

    # Visual: finger body
    vis = finger.addChild("Visu")
    vis.addObject("MeshVTKLoader", name="loader",
                  filename="object/mesh/finger_rot.vtk",
                  translation=translation, rotation=rotation)
    vis.addObject("OglModel", src="@loader", color=[0.4, 0.4, 0.45, 0.7])
    vis.addObject("BarycentricMapping")

    # Visual: yellow hooked fingernail (purely cosmetic, makes the
    # tip stand out in the recording)
    nail = finger.addChild("Fingernail")
    nail.addObject("MeshVTKLoader", name="loader",
                   filename="object/mesh/fingernail_v3_rot.vtk",
                   translation=translation, rotation=rotation)
    nail.addObject("OglModel", src="@loader", color=[1.0, 0.4, 0.7, 1.0])
    nail.addObject("BarycentricMapping")
