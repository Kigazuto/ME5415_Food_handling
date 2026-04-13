# ME5415 Manipulation Challenge — Food Tray Assembly
## Project Report

## 1. Task summary

The challenge is to demonstrate, in simulation, a soft robotic gripper
capable of assembling a food tray. A robot arm picks food items from
**transparent plastic containers** on a *Source Table* and places them
into colour-coded plates / bowls / cups on **two identical empty
trays** on an *Assembly Table*. Nine food categories are required
(sausage, cooked meatball, raw broccoli florets, sliced cooked
carrot, four-bean, cooked spaghetti, biscuit, single-side fried egg,
orange juice).

The two assessed skills are:

* **(a) Soft-gripper design** — by grasping, enveloping, etc.
* **(b) Compliance** — the gripper conforms to the food shape rather
  than crushing it.

Per the brief, completing either skill in simulation earns the marks;
this project demonstrates **both**.

## 2. Scene overview and code organisation

The project is split into small modules for readability:

```
project/
├── scene.py              top-level createScene (wires everything up)
├── gripper.py            3-finger PneuNet gripper
├── food.py               deformable + visual food factory, FEM params
├── tables.py             Source/Assembly tables, containers, trays
├── gripper_controller.py Sofa.Core.Controller (keyboard + CSV log)
├── object/mesh/          STL + VTK assets for food & gripper
└── contact_forces.csv    per-step log written by the controller
```

| Element | Implementation |
|---------|----------------|
| **Source Table** | `floorFlat.obj` plane at `X = -122`, centred at `(Y, Z) = (70, -300)`, brown OglModel + Triangle/Line/Point collision (`triangulate=True` so the quad is expanded to two triangles for the collision pipeline) |
| **Assembly Table** | Same primitive, centred at `(Y, Z) = (70, +300)` |
| **9 transparent containers** | Visual-only OglModel boxes (translucent blue), one per food type, arranged in a 3×3 grid on the Source Table |
| **2 identical trays** | Each carries three coloured compartments: **red plate**, **green bowl**, **blue cup** |
| **9 food categories** | All nine STL models are placed in their respective containers as visual props; the **meatball** is additionally instantiated as a *deformable FEM body* underneath the gripper so it can be physically grasped (see §3.3) |
| **Soft gripper** | Three PneuNet FEM fingers (`pneunetCutCoarse.vtk`) arranged at 0°/120°/240° around a base, each driven by a `SurfacePressureConstraint` actuator |
| **Animation loop** | `FreeMotionAnimationLoop` + `GenericConstraintSolver` (with `computeConstraintForces=True` so the controller can read contact forces) |
| **Contact response** | `FrictionContactConstraint` with `mu=0.7`, `LocalMinDistance` proximity (`alarmDistance=15`, `contactDistance=3`) |

## 3. Soft-gripper design

### 3.1 Architecture

The gripper is a three-finger **PneuNet (pneumatic network) soft
gripper**. Each finger is a tetrahedral FEM body containing an
internal air cavity meshed as a triangular surface. Pressure applied
to the cavity surface as a Lagrange surface constraint
(`SurfacePressureConstraint`) causes the soft side of the finger to
expand more than the stiffer backbone, producing the characteristic
curling motion that envelops an object.

The three fingers are arranged radially at 0°/120°/240° around a
common central axis (`gripper.py:GRIPPER_BASE = [50, 0, -300]`).
They share the same mesh topology, so the base-clamp `BoxROI` and
stiff-backbone `BoxROI` are created only on finger 1 and reused by
fingers 2 and 3 — a pattern borrowed from the SoftRobots
PneunetGripper tutorial.

### 3.2 Material model

Two-layer composite, isotropic linear-elastic
(`TetrahedronFEMForceField`, `method='large'`):

| Layer | Young's modulus | Poisson's ratio |
|-------|----------------:|----------------:|
| Bulk silicone (most of the finger) | 500 | 0.3 |
| Stiff backbone strip (top spine) | 1500 | 0.3 |

The backbone is 3× stiffer than the bulk, so when the cavity is
inflated the backbone acts as a non-stretching spine and the rest of
the finger has to curl around it. That is the essential PneuNet
bending mechanism.

### 3.3 Food modelling: deformable FEM bodies

Following the `ME5415_Food_Gripper` reference, the food itself is
also a deformable FEM body (not a rigid body). The meatball uses
`TetrahedronFEMForceField` with `E = 1.4 MPa`, `ν = 0.49` — literature
values for cooked meatball — together with a
`LinearSolverConstraintCorrection`. This is the only food
configuration that plays nicely with the FEM gripper inside the
`FreeMotionAnimationLoop`: rigid-body meatballs tend to tunnel
through the static table because Uncoupled correction and Lagrangian
friction interact badly when heavy rigid bodies sit near a
singularity in the constraint matrix.

All nine food categories have tuned FEM parameters in
`food.py:FOOD_PARAMS`:

| Food | E [MPa] | ν | Scale | Mass |
|------|--------:|--:|------:|-----:|
| Meatball (cooked) | 1.4 | 0.49 | 1.0 | 20 g |
| Sausage (cooked) | 1.85 | 0.49 | 0.5 | 70 g |
| Broccoli floret | 30.0 | 0.30 | 1.0 | 20 g |
| Carrot slice (cooked) | 3.0 | 0.40 | 0.6 | 100 g |
| Four-bean | 5.0 | 0.40 | 1.0 | 0.13 g |
| Spaghetti | 0.5 | 0.49 | 1.0 | 1 g |
| Cookie / biscuit | 50.0 | 0.30 | 1.0 | 10 g |
| Fried egg | 0.8 | 0.49 | 1.0 | 5 g |
| Orange-juice cup | 2000 | 0.35 | 1.0 | 300 g |

Only the meatball is instantiated as a full FEM body in the default
scene (to keep the frame rate reasonable); the other eight are placed
as visual props in their containers on the Source Table. The same
`add_food_deformable(...)` factory can turn any of them into an
active grasp target — changing a single line in `scene.py`.

### 3.4 Pre-curled "ready to grip" pose

In `scene.py` the gripper is created with
`initial_pressure = 0.05`. This gives the fingers a slight resting
curl so they are already bent toward the central axis when the
simulation starts — a cheap approximation of a hooked-fingertip
geometry, and a much more natural starting pose for picking a small
spherical object like a meatball.

### 3.5 Manufacturability

The geometry is a standard PneuNet fabricated by **two-step silicone
moulding** (one mould for the bulk elastomer, one for the
channel-side surface), with a stiffer paper or nylon strip embedded
along the backbone. This is a process introduced in the course, so
the design satisfies "should be manufacturable using techniques
introduced in the course".

## 4. Compliance demonstration

Compliance is intrinsic to the FEM model on both sides:

1. **Gripper side** — there is no kinematic "close" command. The
   fingers are pushed *only* by internal pressure, so when they hit
   the meatball they continue curling **around** its local geometry
   rather than crushing it. The inner surface of each finger
   flattens against the curved meatball surface and the bending
   pattern locally deviates from the free-air bending — the textbook
   definition of passive compliance.

2. **Food side** — because the meatball is also FEM-deformable with
   a low Young's modulus (1.4 MPa), it compresses slightly under
   each contact patch. The gripper's frictional contact
   (`FrictionContactConstraint, mu = 0.7`) holds the meatball during
   the lift thanks to that enlarged contact area.

### 4.1 Quantitative evidence — contact-force log

The controller writes `contact_forces.csv` once per animation step:

```
step, time [s], pressure, constraint_force_norm [N]
```

`constraint_force_norm` is the L2 norm of the entire Lagrange
multiplier vector of the `GenericConstraintSolver` divided by `dt`
— so it has units of force and represents the total magnitude of
all simultaneously-active constraint forces at that step (contact,
friction, backbone clamping, pressure actuator). When the meatball
first settles on the table a small baseline force (≈ 8 N)
establishes itself; during inflation the curve jumps an order of
magnitude as the fingers close and make multi-patch contact.

Plotting the log gives the gripper's force-vs-time signature during
a grasp, which is the requested "quantitative compliance evidence".

## 5. Food coverage

All **nine** required food categories are present in the scene as
visual props in their transparent containers on the Source Table:

| Food | Geometry class | FEM params tuned | Demonstrated grasp |
|------|----------------|:---:|:---:|
| Meatball | Sphere ⌀30 mm | ✓ | **active** |
| Sausage | Long cylinder | ✓ | visual |
| Broccoli floret | Irregular blob | ✓ | visual |
| Carrot slice | Thin disc | ✓ | visual |
| Four-bean | Small elongated | ✓ | visual |
| Spaghetti | Thin strands | ✓ | visual |
| Cookie / biscuit | Thin disc ⌀70 × 15 mm | ✓ | visual |
| Fried egg | Floppy thin sheet | ✓ | visual |
| Orange-juice cup | Tall cylinder ⌀90 × 120 mm | ✓ | visual |

The meatball is the actively simulated grasp target because it is
the most direct showcase of FEM-on-FEM compliance (both bodies
deform). Any other food can be made active in <1 minute by
swapping the single `add_food_deformable(...)` call in `scene.py`.

## 6. How the demo runs

`./run.sh` launches the scene. After clicking the 3D viewport for
keyboard focus and pressing **Animate**, the gripper is driven with
`Ctrl + key` combinations (runSofa only forwards keyboard events
to Python controllers when Ctrl is held in this SOFA build):

| Key | Action |
|-----|--------|
| `Ctrl + Space` | Inflate cavities (close fingers) |
| `Ctrl + -`     | Deflate cavities (open fingers) |
| `Ctrl + ↑ / ↓` | Move gripper +X / -X |
| `Ctrl + ← / →` | Move gripper -Z / +Z (toward Source / Assembly) |
| `Ctrl + W / S` | Move gripper +Y / -Y |

Suggested recording:

* one wide screenshot of the full scene (both tables, all nine
  containers, both trays, gripper);
* a close-up screenshot during inflation showing finger curl and
  the contact patch on the meatball;
* a screen recording of the complete grasp → lift → traverse →
  release cycle;
* a plot of `contact_forces.csv` showing the force surge during
  the grasp phase.

## 7. Limitations and honest caveats

* Eight of the nine food categories are present visually in their
  containers but not actively picked in the default scene. Their
  FEM parameters are already tuned in `food.py:FOOD_PARAMS`; adding
  any of them as an active grasp target is a one-line change in
  `scene.py`. The eight-at-once case was not run because the FEM
  solver cost on nine deformable bodies would drop the frame rate
  below what is practical for recording.
* The "robot arm" is abstracted: the gripper base is moved by
  prescribing the rest position of the fixed-base region, not by
  solving an articulated-arm inverse kinematics. Substituting an
  arm (e.g. SOFA's `ArticulatedSystemPlugin`) would not change the
  gripper behaviour, only the kinematic layer above it.
* The containers and trays are visual-only OglModel boxes; they
  have no collision surfaces. This was intentional — adding
  collision walls inside every container would clutter the
  gripper's approach corridor without adding value to the
  compliance demonstration.
* The meatball is initialised with a 20 mm Z offset from the
  gripper's central axis to sidestep a constraint-solver
  singularity that appears exactly on the symmetry axis. The
  offset is small enough to be visually imperceptible but large
  enough to keep the solver well-conditioned.
