"""
Tables, transparent containers, and assembly trays for the
Food Tray Assembly scene.

Coordinate convention: gravity is in -X (so +X is "up"). Y/Z form
the horizontal plane. The Source Table is at Z = -300 and the
Assembly Table at Z = +300.
"""

import math

from food import add_food_visual

PLASTIC = [0.75, 0.85, 0.95, 0.35]
TRAY    = [0.85, 0.80, 0.75, 1.0]
PLATE   = [0.85, 0.20, 0.15, 1.0]   # red
BOWL    = [0.20, 0.70, 0.25, 1.0]   # green
CUP     = [0.20, 0.35, 0.85, 1.0]   # blue
SHELF   = [0.50, 0.30, 0.18, 1.0]


def _cylinder_mesh(center, radius, height, segments=24,
                   wall_thickness=None):
    """Build vertex + triangle lists for a vertical cylinder
    (axis along +X, since +X is "up" in this scene).

    `center` is the centre of the cylinder's bottom face. If
    `wall_thickness` is given, the returned mesh is a hollow
    cup/bowl: outer wall, inner wall, top rim, inner bottom.
    Otherwise it is a solid capped cylinder (used for plates).
    """
    cx, cy, cz = center
    pts = []
    tris = []

    def ring(x, r):
        """Append `segments` points at height `x` and radius `r`,
        return the index of the first point of the ring."""
        base = len(pts)
        for k in range(segments):
            theta = 2 * math.pi * k / segments
            pts.append([x,
                        cy + r * math.cos(theta),
                        cz + r * math.sin(theta)])
        return base

    def side(top_base, bot_base, flip=False):
        """Add a quad strip between two rings as two triangles each."""
        for k in range(segments):
            kp = (k + 1) % segments
            t0 = top_base + k
            t1 = top_base + kp
            b0 = bot_base + k
            b1 = bot_base + kp
            if flip:
                tris.append([t0, b0, b1])
                tris.append([t0, b1, t1])
            else:
                tris.append([t0, t1, b1])
                tris.append([t0, b1, b0])

    def cap(ring_base, centre_pt, flip=False):
        c = len(pts)
        pts.append(centre_pt)
        for k in range(segments):
            kp = (k + 1) % segments
            a = ring_base + k
            b = ring_base + kp
            if flip:
                tris.append([c, b, a])
            else:
                tris.append([c, a, b])

    outer_top = ring(cx + height, radius)
    outer_bot = ring(cx,          radius)
    side(outer_top, outer_bot, flip=False)

    if wall_thickness is None:
        # Solid cylinder: cap top and bottom
        cap(outer_top, [cx + height, cy, cz], flip=False)
        cap(outer_bot, [cx,          cy, cz], flip=True)
    else:
        # Hollow cup/bowl: build the inner wall, the top rim
        # annulus, and a closed inner bottom slightly above the
        # outer bottom.
        inner_r = max(radius - wall_thickness, 0.5)
        bottom_thickness = max(wall_thickness * 0.6, 1.0)
        inner_top = ring(cx + height, inner_r)
        inner_bot = ring(cx + bottom_thickness, inner_r)
        # Inner wall (normals point inward → flip)
        side(inner_top, inner_bot, flip=True)
        # Top rim: annulus between outer_top and inner_top
        for k in range(segments):
            kp = (k + 1) % segments
            a = outer_top + k
            b = outer_top + kp
            c = inner_top + kp
            d = inner_top + k
            tris.append([a, b, c])
            tris.append([a, c, d])
        # Inner bottom cap (sits inside the cup, normal up)
        cap(inner_bot, [cx + bottom_thickness, cy, cz], flip=False)
        # Outer bottom cap (normal down)
        cap(outer_bot, [cx, cy, cz], flip=True)

    return pts, tris


def add_visual_cylinder(root, name, center, radius, height, color,
                        wall_thickness=None, segments=24):
    pts, tris = _cylinder_mesh(center, radius, height,
                               segments=segments,
                               wall_thickness=wall_thickness)
    n = root.addChild(name)
    n.addObject("MechanicalObject", position=pts)
    n.addObject("OglModel", name="Visual", position=pts,
                triangles=tris, color=color)
    return n


def add_table(root, name, center_yz, color):
    """A flat table-top with collision at X = -122."""
    cy, cz = center_yz
    table = root.addChild(name)
    table.addObject("MeshOBJLoader", name="loader",
                    filename="object/mesh/floorFlat.obj",
                    rotation=[0, 0, 270], scale=10,
                    translation=[-122, cy, cz],
                    triangulate=True)
    table.addObject("MeshTopology", src="@loader")
    table.addObject("MechanicalObject", src="@loader")
    table.addObject("TriangleCollisionModel")
    table.addObject("LineCollisionModel")
    table.addObject("PointCollisionModel")
    table.addObject("OglModel", name="Visual", src="@loader", color=color)
    return table


def add_visual_box(root, name, center, size, color):
    """A static visual-only box (used for trays / containers)."""
    cx, cy, cz = center
    sx, sy, sz = size
    n = root.addChild(name)
    pts = []
    for ix in (-0.5, 0.5):
        for iy in (-0.5, 0.5):
            for iz in (-0.5, 0.5):
                pts.append([cx + ix * sx, cy + iy * sy, cz + iz * sz])
    quads = [
        [0, 1, 3, 2], [4, 6, 7, 5],
        [0, 4, 5, 1], [2, 3, 7, 6],
        [0, 2, 6, 4], [1, 5, 7, 3],
    ]
    n.addObject("MechanicalObject", position=pts)
    n.addObject("OglModel", name="Visual", position=pts,
                quads=quads, color=color)
    return n


# Source Table layout
# -------------------
# Four round containers on the table top, one food per container.
# (Meatball's container is empty here — the active deformable meatball
# is placed directly under the gripper by scene.py.)
TABLE_TOP_X = -122           # the table-top plane
TABLE_BOTTOM_X = -300        # how far down legs extend
BOWL_HEIGHT = 35
BOWL_RADIUS = 75             # wide enough for the gripper (R = 70) to enter

# (food_name, (Y, Z))
# 2×2 grid of bowls, centres 200 mm apart so radius-75 bowls don't
# overlap and the gripper can descend cleanly into any of them.
SOURCE_LAYOUT = [
    ("meatball", (-30, -400)),
    ("sausage",  (170, -400)),
    ("broccoli", (-30, -200)),
    ("bean",     (170, -200)),
]


def _add_table_legs(root, prefix, table_center_yz, half_y=180, half_z=180):
    """Four box legs hanging beneath a table top at TABLE_TOP_X."""
    cy, cz = table_center_yz
    leg_x = (TABLE_TOP_X + TABLE_BOTTOM_X) * 0.5
    leg_h = TABLE_TOP_X - TABLE_BOTTOM_X
    leg_size = (leg_h, 12, 12)
    leg_color = [0.40, 0.25, 0.12, 1.0]
    for sy in (-1, 1):
        for sz in (-1, 1):
            add_visual_box(
                root, f"{prefix}_Leg_{sy}_{sz}",
                center=(leg_x, cy + sy * half_y, cz + sz * half_z),
                size=leg_size, color=leg_color)


def add_source_table(root):
    """Source Table: wooden top + 4 box legs + 4 round plastic
    containers, each holding one food category (meatball container
    is intentionally empty since the active FEM meatball lives next
    to the gripper).
    """
    add_table(root, "SourceTable", center_yz=(70, -300),
              color=[0.55, 0.35, 0.20, 1.0])
    _add_table_legs(root, "SourceTable", table_center_yz=(70, -300))

    # Only the bowls are added here — the food *itself* is created
    # in scene.py as an active deformable FEM body, so it falls
    # into the bowl with proper physics.
    for food_name, (cy, cz) in SOURCE_LAYOUT:
        bowl_center = (TABLE_TOP_X, cy, cz)
        add_visual_cylinder(root, f"Container_{food_name}",
                            center=bowl_center,
                            radius=BOWL_RADIUS,
                            height=BOWL_HEIGHT,
                            wall_thickness=4,
                            color=PLASTIC)


def add_assembly_table(root):
    """Assembly Table: wooden top + 2 identical empty trays. Each
    tray is a shallow box with a raised rim, holding a circular red
    plate, a green bowl, and a blue cup (all built procedurally as
    cylinders so they actually look like dishware).
    """
    add_table(root, "AssemblyTable", center_yz=(70, 300),
              color=[0.45, 0.30, 0.15, 1.0])
    _add_table_legs(root, "AssemblyTable", table_center_yz=(70, 300))

    tray_top_x = TABLE_TOP_X + 8     # tray sits on the table
    for k, ty in enumerate((0, 140)):
        # Tray base
        add_visual_box(root, f"Tray{k+1}_Base",
                       center=(tray_top_x - 4, ty, 300),
                       size=(8, 130, 290), color=TRAY)
        # Tray rim (4 thin walls around the base)
        add_visual_box(root, f"Tray{k+1}_RimA",
                       center=(tray_top_x + 4, ty - 65, 300),
                       size=(8, 4, 290), color=TRAY)
        add_visual_box(root, f"Tray{k+1}_RimB",
                       center=(tray_top_x + 4, ty + 65, 300),
                       size=(8, 4, 290), color=TRAY)
        add_visual_box(root, f"Tray{k+1}_RimC",
                       center=(tray_top_x + 4, ty, 300 - 145),
                       size=(8, 130, 4), color=TRAY)
        add_visual_box(root, f"Tray{k+1}_RimD",
                       center=(tray_top_x + 4, ty, 300 + 145),
                       size=(8, 130, 4), color=TRAY)

        dish_x = tray_top_x + 4
        # Round red plate (very flat cylinder)
        add_visual_cylinder(root, f"Tray{k+1}_Plate",
                            center=(dish_x, ty, 220),
                            radius=45, height=6,
                            wall_thickness=4, color=PLATE)
        # Green bowl (medium cylinder with cavity)
        add_visual_cylinder(root, f"Tray{k+1}_Bowl",
                            center=(dish_x, ty, 320),
                            radius=40, height=22,
                            wall_thickness=4, color=BOWL)
        # Blue cup (tall narrow cylinder)
        add_visual_cylinder(root, f"Tray{k+1}_Cup",
                            center=(dish_x, ty, 400),
                            radius=28, height=55,
                            wall_thickness=3, color=CUP)
