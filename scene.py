"""
ME5415 — Manipulation challenge: Food Tray Assembly
====================================================

Top-level scene file for the food-tray assembly demo. The scene is
split into modules:

    food.py      — FEM/visual food helpers + per-food parameters
    gripper.py   — 3-finger PneuNet soft gripper
    tables.py    — Source / Assembly tables, containers, trays
    gripper_controller.py
                 — keyboard controller (Ctrl + Space/Minus/Arrows/W/S)

Run with:
    ./run.sh         (or:  runSofa scene.py  with SofaPython3)

Coordinate convention (inherited from the SoftRobots PneunetGripper
tutorial): +X is up, gravity acts along -X. Y, Z form the horizontal
plane.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from food import add_food_deformable
from gripper import add_gripper
from tables import add_source_table, add_assembly_table
from gripper_controller import WholeGripperController


PLUGINS = [
    "SoftRobots", "SofaPython3", "MultiThreading",
    "Sofa.Component.AnimationLoop",
    "Sofa.Component.Collision.Detection.Algorithm",
    "Sofa.Component.Collision.Detection.Intersection",
    "Sofa.Component.Collision.Geometry",
    "Sofa.Component.Collision.Response.Contact",
    "Sofa.Component.Constraint.Lagrangian.Correction",
    "Sofa.Component.Constraint.Lagrangian.Solver",
    "Sofa.Component.Constraint.Projective",
    "Sofa.Component.Engine.Select",
    "Sofa.Component.IO.Mesh",
    "Sofa.Component.LinearSolver.Direct",
    "Sofa.Component.LinearSolver.Iterative",
    "Sofa.Component.Mapping.Linear",
    "Sofa.Component.Mapping.NonLinear",
    "Sofa.Component.Mass",
    "Sofa.Component.ODESolver.Backward",
    "Sofa.Component.Setting",
    "Sofa.Component.SolidMechanics.FEM.Elastic",
    "Sofa.Component.SolidMechanics.Spring",
    "Sofa.Component.StateContainer",
    "Sofa.Component.Topology.Container.Constant",
    "Sofa.Component.Topology.Container.Dynamic",
    "Sofa.Component.Visual",
    "Sofa.GL.Component.Rendering3D",
]


def createScene(root):
    for p in PLUGINS:
        root.addObject("RequiredPlugin", name=p)

    root.addObject("VisualStyle",
                   displayFlags="showVisualModels hideBehaviorModels "
                                "hideCollisionModels hideBoundingCollisionModels "
                                "hideForceFields showInteractionForceFields "
                                "hideWireframe")

    root.gravity.value = [-9810, 0, 0]
    root.dt.value = 0.01

    # Constraint pipeline. `computeConstraintForces=True` lets the
    # controller read per-step contact forces for the CSV log.
    root.addObject("FreeMotionAnimationLoop")
    root.addObject("GenericConstraintSolver", name="GCS",
                   tolerance=1e-7, maxIterations=1000,
                   computeConstraintForces=True)
    root.addObject("CollisionPipeline")
    root.addObject("ParallelBruteForceBroadPhase")
    root.addObject("ParallelBVHNarrowPhase")
    root.addObject("CollisionResponse",
                   response="FrictionContactConstraint",
                   responseParams="mu=1.2")
    root.addObject("LocalMinDistance", name="Proximity",
                   alarmDistance=15, contactDistance=3)

    root.addObject("BackgroundSetting", color=[0.94, 0.92, 0.86, 1.0])
    root.addObject("OglSceneFrame", style="Arrows", alignment="TopRight")

    # ---- Tables, containers, trays, visual food props ----
    add_source_table(root)
    add_assembly_table(root)

    # ---- Active food: 4 deformable FEM bodies, one per container.
    #      Each is spawned ~30 mm above the table so it falls
    #      cleanly into its bowl.
    add_food_deformable(root, "Meatball",  "meatball",
                        world_centroid=(-90, -30, -400))
    add_food_deformable(root, "Sausage",   "sausage",
                        world_centroid=(-90, 170, -400))
    add_food_deformable(root, "Broccoli",  "broccoli",
                        world_centroid=(-90, -30, -200))
    add_food_deformable(root, "Bean",      "bean",
                        world_centroid=(-90, 170, -200))

    # ---- Soft gripper ----
    # Small initial cavity pressure pre-curls the fingers slightly,
    # giving them a "ready-to-grip" pose without needing hooked
    # fingertip meshes.
    add_gripper(root, initial_pressure=0.05)

    ctrl = WholeGripperController(name="GripperController", node=root)
    root.addObject(ctrl)
    root.gripper_ctrl = ctrl  # keep the Python ref alive
    return root
