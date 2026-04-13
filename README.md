# ME5415 — Food Tray Assembly (SOFA)

A SOFA simulation that demonstrates a **3-finger soft pneumatic
gripper** picking food items from a *Source Table* and placing them
on trays on an *Assembly Table*, in the spirit of the food-tray
assembly manipulation challenge.

## Files

| File | Purpose |
|------|---------|
| `scene.py` | Main SOFA scene (tables, containers, trays, food, gripper, controller hookup) |
| `gripper_controller.py` | `Sofa.Core.Controller` for the gripper: keyboard + scripted pick-and-place |
| `run.sh` | Launcher — sets `LD_LIBRARY_PATH` so SofaPython3 finds `libpython3.12` and opens the scene |
| `data/mesh/` | Symlinks to the PneunetGripper tutorial meshes and the food meshes (`../Objects`) |
| `REPORT.md` | Project report (design, compliance, food coverage) |

## How to run

```bash
./run.sh
```

This opens the scene in `runSofa`. Press **Animate** in the GUI to start
the simulation.

## Controls

**Important:** runSofa only forwards key events to Python controllers
when the **Ctrl modifier is held**, so every key below must be
pressed as `Ctrl + key`. Click the 3D viewport first so it has
keyboard focus.

| Key | Action |
|-----|--------|
| `Ctrl + Space` | Inflate cavities → fingers curl in (close) |
| `Ctrl + -`     | Deflate cavities → fingers open |
| `Ctrl + ↑` / `Ctrl + ↓` | Move gripper up / down (along ±X) |
| `Ctrl + A` / `Ctrl + D` | Move gripper left / right (-Z / +Z) |
| `Ctrl + W` / `Ctrl + S` | Move gripper forward / back (-Y / +Y) |

Recording flow: click the 3D viewport, press **Animate**, wait for
the meatball to settle, then drive the gripper with the Ctrl-key
combinations above to demonstrate the grasp + traverse + release.

## Scene contents

* **Source Table** — wooden table-top with eight transparent plastic
  containers laid out in two rows. Containers visually represent the
  storage of all nine required food types (sausage, meatball,
  broccoli, carrot, four-bean, spaghetti, cookie, fried-egg, and an
  orange-juice cup).
* **Assembly Table** — wooden table-top with **two identical empty
  trays**, each carrying a red plate, a green bowl, and a blue cup
  (different colours per task brief).
* **Food meshes** — the rigid food items actively simulated for the
  pick-and-place are: meatball, cookie, broccoli, and the orange-juice
  cup. The remaining five food categories are present in the scene as
  visual containers and are discussed in `REPORT.md`.
* **Soft gripper** — three FEM-tetrahedral PneuNet fingers in a
  120°/120°/120° radial layout, each driven by a `SurfacePressure
  Constraint` actuator. Fingers are made of two materials: a soft bulk
  silicone (`youngModulus = 500`) and a stiffer backbone strip
  (`youngModulus = 1500`), so they curl asymmetrically when inflated —
  exactly the PneuNet bending principle.

## Dependencies

* SOFA v24.12 with the **SoftRobots** plugin (already present in
  `/home/kirito/sofa-install/extracted/`).
* `libpython3.12` from a conda env (the launcher already wires this).
