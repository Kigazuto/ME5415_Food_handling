"""
Gripper controller for the food-tray assembly scene.

IMPORTANT: runSofa only forwards keyboard events to Python
controllers when the **Ctrl modifier is held**. So all keys below
must be pressed as Ctrl+key.

Keyboard (hold Ctrl while pressing):
    Ctrl + Space          inflate cavities  (close soft fingers)
    Ctrl + Minus          deflate cavities  (open soft fingers)
    Ctrl + Up arrow       move gripper UP   (+X)
    Ctrl + Down arrow     move gripper DOWN (-X)
    Ctrl + A / Ctrl + D   move gripper -Z / +Z  (left / right)
    Ctrl + W / Ctrl + S   move gripper -Y / +Y  (forward / back)
"""

import math
import os
import csv
import numpy as np
import Sofa.Core
from Sofa.constants import Key


def move_rest(rest_pos, dx, dy, dz):
    return [[p[0] + dx, p[1] + dy, p[2] + dz] for p in rest_pos]


class WholeGripperController(Sofa.Core.Controller):
    def __init__(self, node, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = node
        self.dofs = []
        self.constraints = []
        # Discover all `FingerN` children (N starts at 1) so this
        # controller works for any number of fingers in the gripper.
        i = 1
        while True:
            f = self.node.getChild(f"Finger{i}")
            if f is None:
                break
            self.dofs.append(f.getMechanicalState())
            self.constraints.append(f.Cavity.SurfacePressureConstraint)
            i += 1
        self.num_fingers = i - 1

        # Scripted demo state
        self.auto = False
        self.step = 0
        self.tick = 0
        self.total_dx = 0.0
        self.total_dz = 0.0

        # ---- Contact-force CSV log ----
        # Writes one row per animation step with both the global
        # constraint-force norm and a per-finger XYZ breakdown
        # computed by walking each finger's collision sparse
        # constraint matrix and folding it with the Lagrange
        # multipliers from GenericConstraintSolver.constraintForces.
        log_dir = os.path.dirname(os.path.abspath(__file__))
        self.log_path = os.path.join(log_dir, "contact_forces.csv")
        self.log_file = open(self.log_path, "w", newline="", buffering=1)
        self.log_writer = csv.writer(self.log_file)
        header = ["step", "time", "pressure", "force_norm"]
        for i in range(1, self.num_fingers + 1):
            header += [f"F{i}x", f"F{i}y", f"F{i}z"]
        self.log_writer.writerow(header)
        self.log_file.flush()
        self._gcs = self.node.getObject("GCS")
        self._finger_collision_mos = []
        for i in range(1, self.num_fingers + 1):
            f = self.node.getChild(f"Finger{i}")
            self._finger_collision_mos.append(
                f.getChild("Collision").getObject("collisMech"))

    # -------- helpers --------
    def translate(self, dx, dy, dz):
        for d in self.dofs:
            d.rest_position.value = move_rest(d.rest_position.value, dx, dy, dz)
        self.total_dx += dx
        self.total_dz += dz

    def set_pressure(self, p):
        p = max(0.0, min(5.0, p))
        for c in self.constraints:
            c.value = [p]

    def add_pressure(self, dp):
        self.set_pressure(self.constraints[0].value.value[0] + dp)

    def _finger_force(self, mo, lambdas, dt):
        """Sum the contact force on one finger's collision MO into
        a single (Fx, Fy, Fz) world-space vector, by walking the
        sparse constraint Jacobian and folding it with the Lagrange
        multiplier vector from the constraint solver.
        """
        force = np.zeros(3, dtype=float)
        try:
            cmat = mo.constraint.value
            shape = cmat.get_shape()
        except Exception:
            return force
        n_constraints = shape[0]
        for ci in range(n_constraints):
            row = cmat[ci]
            cols = row.nonzero()[1]
            if len(cols) == 0:
                continue
            lam = float(lambdas[ci])
            for col in cols:
                axis = col % 3
                force[axis] += float(row[0, col]) * lam
        return force / max(dt, 1e-9)

    def _log_forces(self):
        try:
            lambdas = self._gcs.constraintForces.value
        except Exception:
            return
        dt = float(self.node.dt.value)
        norm = float(np.linalg.norm(lambdas)) / max(dt, 1e-9)
        per_finger = [self._finger_force(mo, lambdas, dt)
                      for mo in self._finger_collision_mos]
        pressure = float(self.constraints[0].value.value[0])
        sim_time = float(self.node.time.value)
        row = [self._animate_ticks, f"{sim_time:.4f}",
               f"{pressure:.4f}", f"{norm:.4f}"]
        for f in per_finger:
            row += [f"{f[0]:.4f}", f"{f[1]:.4f}", f"{f[2]:.4f}"]
        self.log_writer.writerow(row)
        if self._animate_ticks % 50 == 0:
            self.log_file.flush()

    # -------- input --------
    def onKeyPressedEvent(self, e):
        self.onKeypressedEvent(e)

    def onKeypressedEvent(self, e):
        k = e["key"]
        step = 10.0
        if k == Key.space:                 # Ctrl+Space → inflate
            self.add_pressure(0.05)
        elif k == Key.minus:               # Ctrl+- → deflate
            self.add_pressure(-0.05)
        elif k == Key.uparrow:             # Ctrl+↑ → up
            self.translate(step, 0, 0)
        elif k == Key.downarrow:           # Ctrl+↓ → down
            self.translate(-step, 0, 0)
        elif k == Key.A:                   # Ctrl+A → -Z (left)
            self.translate(0, 0, -step)
        elif k == Key.D:                   # Ctrl+D → +Z (right)
            self.translate(0, 0, step)
        elif k == Key.W:                   # Ctrl+W → -Y (forward)
            self.translate(0, -step, 0)
        elif k == Key.S:                   # Ctrl+S → +Y (back)
            self.translate(0, step, 0)

    # -------- scripted pick-and-place --------
    _animate_ticks = 0

    def onAnimateBeginEvent(self, _):
        self._animate_ticks += 1
        self._log_forces()
        if not self.auto:
            return
        self.tick += 1
        # Each phase runs for N animation steps
        if self.step == -1:          # descend onto food
            self.translate(-2.0, 0, 0)
            if self.tick > 25:
                self.step, self.tick = 0, 0
        elif self.step == 0:         # close fingers around food
            self.add_pressure(0.02)
            if self.tick > 60:
                self.step, self.tick = 1, 0
        elif self.step == 1:         # lift up
            self.translate(4.0, 0, 0)
            if self.tick > 30:
                self.step, self.tick = 2, 0
        elif self.step == 2:         # traverse toward Assembly Table
            self.translate(0, 0, 10.0)
            if self.tick > 60:
                self.step, self.tick = 3, 0
        elif self.step == 3:         # descend onto tray
            self.translate(-4.0, 0, 0)
            if self.tick > 25:
                self.step, self.tick = 4, 0
        elif self.step == 4:         # release (deflate)
            self.add_pressure(-0.02)
            if self.tick > 60:
                self.step, self.tick = 5, 0
        elif self.step == 5:         # retreat upward
            self.translate(4.0, 0, 0)
            if self.tick > 30:
                self.auto = False
