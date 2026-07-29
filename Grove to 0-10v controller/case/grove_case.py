#!/usr/bin/env python3
"""
Grove to 0-10V controller 3D-printed case generator
===================================================
Generates Grove10V-Case-Base.stl and Grove10V-Case-Lid.stl for the
57.531 x 55.626 mm "MainBoard" PCB (Altium project 025-Grove0to10VESP).

The board has NO mounting holes (the two 3 mm drills are the RJ11 jack's
snap-in posts), so the case works as a drop-in cradle: the board sits on a
perimeter shelf, is registered laterally by wall ribs/curbs (0.35 mm play),
and is clamped down by four pillars on the lid.

Coordinate system: PCB lower-left corner = (0,0); z = 0 at the inside floor.
Board data extracted from the fab outputs (gerbers, NC drill, pick&place,
silkscreen):

  - board outline: 57.531 x 55.626, rectangular
  - U3 RJ11 jack (DS1133-S60BPX): body x 0.93..13.93, y 12.67..24.67,
    port faces the LEFT board edge, ~13.5 mm tall, TH pins below board
  - CN1 Grove socket (ZX-HY2.0-4PWT): near left edge, y ~35.5..48.5,
    port faces LEFT, ~6 mm tall
  - SW1/2/4/5/6/7 slide switches (MST23D19G2) in a column near x=46,
    y = 5.38, 14.01, 22.82, 31.41, 40.47, 49.28 - lid access windows
  - SW3 DIP switch (DSHP03TS-S) at (29.2, 7.4) - lid access window
  - everything else is low-profile SMD on the top side only

Requires:  pip install manifold3d numpy matplotlib contourpy
"""
import os
import struct
import numpy as np
import manifold3d as m3
from manifold3d import Manifold, CrossSection, JoinType

m3.set_circular_segments(64)

# --------------------------------------------------------------------------
# Parameters
# --------------------------------------------------------------------------
PCB_W, PCB_L, PCB_T = 57.531, 55.626, 1.6

GAP_L, GAP_R = 1.2, 1.2    # board edge -> inner wall (ports nearly flush)
GAP_B, GAP_T = 7.0, 7.0    # room for the lid-screw corner posts
WALL = 2.5
FLOOR_T = 2.5
LID_T = 2.5
CAVITY_H = 22.5            # inside floor -> wall top (RJ11 top is ~19.1)
OUTER_R = 3.0

SEAT_H = 4.0               # board seat height (RJ11 pins need ~3.5 below)
SHELF_W = 2.6              # left/right under-board shelves
CURB_LEDGE = 2.2           # top/bottom curbs reach this far under the board
BOARD_CLR = 0.35           # lateral play left after registration ribs/curbs
RIB_W, RIB_H = 4.0, 5.4    # registration ribs on left/right walls

POST_R = 8.5               # quarter-round lid-screw posts, cavity corners
POST_PILOT_DEPTH = 12.0
POST_SCREW_INSET = 2.5

# M3 knurled brass heat-set inserts: 4 mm long, 4.5 mm OD.
INSERT_OD = 4.5
INSERT_HOLE_D = 4.2        # heat-set hole; tune 4.0-4.3 to your inserts
INSERT_LEN = 4.0
INSERT_DEPTH = 5.5         # pocket depth = insert length + displaced-plastic slop
SCREW_CLR_D = 3.4          # M3 screw clearance below the insert
KEEPOUT_CLR = 0.4          # post clearance to the PCB outline

LIP_T, LIP_H, LIP_CLR = 1.8, 2.5, 0.35
CBORE_D, CBORE_DEPTH, LID_HOLE_D = 6.8, 1.6, 3.4

PILLAR_D = 6.0             # lid hold-down pillars onto the board corners
PILLAR_GAP = 0.2           # nominal gap pillar -> board top (clamp slack)
PILLARS = [(2.5, 3.0), (55.0, 3.0), (2.5, 52.5), (55.0, 52.5)]

RJ_YC, RJ_W, RJ_Z0, RJ_Z1 = 18.68, 14.0, 5.2, 19.0    # RJ11 port, left wall
CN_YC, CN_W, CN_Z0, CN_Z1 = 42.04, 13.0, 5.2, 14.2    # Grove port, left wall

SW_WINDOWS = True          # lid windows over the config switches
SW_XY = [(46.387, 5.378), (46.292, 14.014), (46.291, 22.816),
         (46.164, 31.413), (45.752, 40.469), (45.593, 49.276)]
SW_WIN_W, SW_WIN_H = 11.5, 5.5
DIP_XY, DIP_WIN_W, DIP_WIN_H = (28.75, 7.37), 10.0, 11.0

MOUNT_EARS = True
EAR_W, EAR_OUT, EAR_T, EAR_HOLE_D = 18.0, 12.0, 4.0, 4.5

LOGO_FILE = "chill_logo.png"   # Chill Division logo, debossed into the lid
LOGO_W, LOGO_DEPTH = 32.0, 0.4
LOGO_XY = (20.5, 33.0)         # centre, board coordinates

# derived
IX0, IX1 = -GAP_L, PCB_W + GAP_R
IY0, IY1 = -GAP_B, PCB_L + GAP_T
OX0, OX1 = IX0 - WALL, IX1 + WALL
OY0, OY1 = IY0 - WALL, IY1 + WALL
PCB_TOP = SEAT_H + PCB_T
CORNERS = [(IX0, IY0), (IX1, IY0), (IX0, IY1), (IX1, IY1)]
SCREWS = [(x + POST_SCREW_INSET * (1 if x == IX0 else -1),
           y + POST_SCREW_INSET * (1 if y == IY0 else -1)) for x, y in CORNERS]


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def rrect(w, l, r):
    return CrossSection.square([w - 2 * r, l - 2 * r], True).offset(
        r, JoinType.Round)


def rbox(w, l, h, r, cx=0.0, cy=0.0, z0=0.0):
    return rrect(w, l, r).extrude(h).translate([cx, cy, z0])


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl(d, h, x=0.0, y=0.0, z0=0.0):
    return Manifold.cylinder(h, d / 2).translate([x, y, z0])


def insert_pocket(x, y, z_top, depth):
    """Heat-set insert pocket drilled down from z_top: INSERT_DEPTH of
    INSERT_HOLE_D for the brass, then SCREW_CLR_D clearance below."""
    p = cyl(INSERT_HOLE_D, INSERT_DEPTH + 0.5, x, y, z_top - INSERT_DEPTH)
    if depth > INSERT_DEPTH:
        p += cyl(SCREW_CLR_D, depth - INSERT_DEPTH + 0.1, x, y, z_top - depth)
    return p


def pcb_keepout(z0):
    """Volume the PCB drops through - posts must stay out of it above z0."""
    return box(-KEEPOUT_CLR, PCB_W + KEEPOUT_CLR, -KEEPOUT_CLR,
               PCB_L + KEEPOUT_CLR, z0, CAVITY_H + 1)


def port_x(w, h, r, y, zc, x_wall):
    """Rounded-rect cutter through a left/right wall (axis along x)."""
    p = rrect(h, w, r).extrude(WALL + 2).translate([0, 0, -(WALL + 2) / 2])
    return p.rotate([0, 90, 0]).translate([x_wall + WALL / 2, y, zc])


def logo_deboss(cx, cy, z_top):
    """Trace LOGO_FILE (light-on-dark bitmap) and deboss it LOGO_DEPTH deep
    into the lid top surface, LOGO_W wide, centred at (cx, cy)."""
    import matplotlib.image as mpimg
    from contourpy import contour_generator
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), LOGO_FILE)
    if not os.path.exists(path):
        print(f"note: {LOGO_FILE} not found - skipping logo deboss")
        return None
    img = mpimg.imread(path)
    g = img[..., :3].mean(-1) if img.ndim == 3 else img
    mask = (g > 0.5).astype(float)
    rows, cols = np.any(mask > .5, 1), np.any(mask > .5, 0)
    r0, r1 = np.argmax(rows), len(rows) - np.argmax(rows[::-1])
    c0, c1 = np.argmax(cols), len(cols) - np.argmax(cols[::-1])
    sub = mask[max(0, r0 - 2):r1 + 2, max(0, c0 - 2):c1 + 2]
    s = LOGO_W / (c1 - c0)
    polys = []
    for lp in contour_generator(z=sub).lines(0.5):
        lp = np.asarray(lp)
        if len(lp) >= 3:
            polys.append(np.column_stack([
                (lp[:, 0] - (sub.shape[1] - 1) / 2) * s,
                ((sub.shape[0] - 1) / 2 - lp[:, 1]) * s]).astype(np.float32))
    cs = CrossSection(polys, m3.FillRule.EvenOdd).simplify(0.02)
    return cs.extrude(LOGO_DEPTH + 0.1).translate([cx, cy, z_top - LOGO_DEPTH])


def write_stl(man, path):
    mesh = man.to_mesh()
    v = np.asarray(mesh.vert_properties, dtype=np.float32)[:, :3]
    t = np.asarray(mesh.tri_verts, dtype=np.int64)
    tv = v[t]
    n = np.cross(tv[:, 1] - tv[:, 0], tv[:, 2] - tv[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    rec = np.zeros(len(t), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)),
                                           ('attr', '<u2')]))
    rec['n'], rec['v'] = n, tv
    with open(path, 'wb') as f:
        f.write(b'Grove 0-10V case'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(t)))
        f.write(rec.tobytes())
    print(f"wrote {path}: {len(t)} tris, "
          f"vol {man.volume() / 1000:.1f} cm3, status {man.status()}")


# --------------------------------------------------------------------------
# Base
# --------------------------------------------------------------------------
def build_base():
    ow, ol = OX1 - OX0, OY1 - OY0
    cw, cl = IX1 - IX0, IY1 - IY0
    ocx, ocy = (OX0 + OX1) / 2, (OY0 + OY1) / 2
    icx, icy = (IX0 + IX1) / 2, (IY0 + IY1) / 2

    shell = rbox(ow, ol, FLOOR_T + CAVITY_H, OUTER_R, ocx, ocy, -FLOOR_T)
    base = shell - rbox(cw, cl, CAVITY_H + 1, 1.0, icx, icy, 0)

    # under-board shelves along left/right walls
    base += box(IX0, IX0 + SHELF_W, IY0, IY1, 0, SEAT_H)
    base += box(IX1 - SHELF_W, IX1, IY0, IY1, 0, SEAT_H)

    # curbs along bottom/top walls: registration face + under-board ledge
    base += box(IX0, IX1, -5.0, -BOARD_CLR, 0, 8.0)
    base += box(IX0, IX1, -BOARD_CLR, CURB_LEDGE, 0, SEAT_H)
    base += box(IX0, IX1, PCB_L + BOARD_CLR, PCB_L + 5.0, 0, 8.0)
    base += box(IX0, IX1, PCB_L - CURB_LEDGE, PCB_L + BOARD_CLR, 0, SEAT_H)

    # registration ribs (leave BOARD_CLR to the board edge)
    for y in (8.0, 30.5, 51.5):
        base += box(IX0, -BOARD_CLR, y - RIB_W / 2, y + RIB_W / 2, 0, RIB_H)
    for y in (8.0, 30.0, 51.0):
        base += box(PCB_W + BOARD_CLR, IX1, y - RIB_W / 2, y + RIB_W / 2,
                    0, RIB_H)

    # lid-screw corner posts
    # posts are fat enough to wrap a 4.5 mm insert, so the board's drop-in
    # volume is trimmed back off them
    cav_clip = rbox(cw, cl, CAVITY_H, 1.0, icx, icy, 0)
    keepout = pcb_keepout(SEAT_H)
    for x, y in CORNERS:
        base += (cyl(POST_R * 2, CAVITY_H, x, y) ^ cav_clip) - keepout

    # ports + pilot holes
    cuts = [
        port_x(RJ_W, RJ_Z1 - RJ_Z0, 2.0, RJ_YC, (RJ_Z0 + RJ_Z1) / 2,
               IX0 - WALL),
        port_x(CN_W, CN_Z1 - CN_Z0, 2.0, CN_YC, (CN_Z0 + CN_Z1) / 2,
               IX0 - WALL),
    ]
    for x, y in SCREWS:
        cuts.append(insert_pocket(x, y, CAVITY_H, POST_PILOT_DEPTH))
    base -= Manifold.batch_boolean(cuts, m3.OpType.Add)

    if MOUNT_EARS:
        for yc, s in ((OY0, -1), (OY1, 1)):
            ear = rbox(EAR_W, EAR_OUT + 2, EAR_T, 3,
                       ocx, yc + s * (EAR_OUT - 2) / 2, -FLOOR_T)
            ear -= cyl(EAR_HOLE_D, EAR_T + 1, ocx,
                       yc + s * (EAR_OUT / 2 + 0.5), -FLOOR_T - 0.5)
            base += ear - shell
    return base


# --------------------------------------------------------------------------
# Lid (modelled in assembly position)
# --------------------------------------------------------------------------
def build_lid():
    ow, ol = OX1 - OX0, OY1 - OY0
    cw, cl = IX1 - IX0, IY1 - IY0
    ocx, ocy = (OX0 + OX1) / 2, (OY0 + OY1) / 2
    icx, icy = (IX0 + IX1) / 2, (IY0 + IY1) / 2

    lid = rbox(ow, ol, LID_T, OUTER_R, ocx, ocy, CAVITY_H)

    # inner lip, cleared at posts and over the RJ11 body (jack is lip-height)
    lw, ll = cw - 2 * LIP_CLR, cl - 2 * LIP_CLR
    lip = rbox(lw, ll, LIP_H, 0.8, icx, icy, CAVITY_H - LIP_H) - \
        rbox(lw - 2 * LIP_T, ll - 2 * LIP_T, LIP_H + 1, 0.8, icx, icy,
             CAVITY_H - LIP_H - 0.5)
    for x, y in CORNERS:
        lip -= cyl((POST_R + 1.5) * 2, LIP_H + 1, x, y, CAVITY_H - LIP_H - 0.5)
    lip -= box(IX0 - 1, 2.0, RJ_YC - RJ_W / 2 - 1, RJ_YC + RJ_W / 2 + 1,
               CAVITY_H - LIP_H - 0.5, CAVITY_H + 1)
    lid += lip

    # board hold-down pillars
    for x, y in PILLARS:
        lid += cyl(PILLAR_D, CAVITY_H - PCB_TOP - PILLAR_GAP, x, y,
                   PCB_TOP + PILLAR_GAP)

    # screw holes, counterbores, switch windows
    cuts = []
    for x, y in SCREWS:
        cuts.append(cyl(LID_HOLE_D, LID_T + LIP_H + 1, x, y,
                        CAVITY_H - LIP_H - 0.5))
        cuts.append(cyl(CBORE_D, CBORE_DEPTH + 0.5, x, y,
                        CAVITY_H + LID_T - CBORE_DEPTH))
    if SW_WINDOWS:
        for x, y in SW_XY:
            cuts.append(rbox(SW_WIN_W, SW_WIN_H, LID_T + 1, 2.0, x, y,
                             CAVITY_H - 0.5))
        cuts.append(rbox(DIP_WIN_W, DIP_WIN_H, LID_T + 1, 2.0,
                         DIP_XY[0], DIP_XY[1], CAVITY_H - 0.5))
    lid -= Manifold.batch_boolean(cuts, m3.OpType.Add)
    logo = logo_deboss(LOGO_XY[0], LOGO_XY[1], CAVITY_H + LID_T)
    if logo is not None:
        lid -= logo
    return lid


# --------------------------------------------------------------------------
# Interference checks
# --------------------------------------------------------------------------
def check(base, lid):
    bt = PCB_TOP
    mocks = {
        "PCB": box(0, PCB_W, 0, PCB_L, SEAT_H, bt),
        "RJ11 body": box(0.9, 13.9, 12.2, 25.2, bt, bt + 13.5),
        "RJ11 pins": box(5.9, 12.9, 11.9, 25.4, 0.5, SEAT_H),
        "CN1 Grove": box(0.3, 13.3, 35.5, 48.5, bt, bt + 6.0),
        "SW column": box(41.0, 51.0, 3.0, 52.0, bt, bt + 4.2),
        "DIP SW3": box(24.4, 33.4, 2.6, 12.2, bt, bt + 3.8),
        "SMD blanket": box(6.0, 51.0, 6.0, 49.5, bt, bt + 3.2),
    }
    ok = True
    for a, am, b, bm in [("base", base, "lid", lid)] + \
            [("base", base, k, v) for k, v in mocks.items()] + \
            [("lid", lid, k, v) for k, v in mocks.items()]:
        vol = (am ^ bm).volume()
        flag = "OK " if vol < 1e-6 else "COLLISION"
        if vol >= 1e-6:
            ok = False
        print(f"  {flag} {a} vs {b}: intersection {vol:.3f} mm3")
    seat = (base ^ mocks["PCB"].translate([0, 0, -0.1])).volume()
    print(f"  board seat contact {seat:.1f} mm3 (shelves + curb ledges)")
    clamp = (lid ^ mocks["PCB"].translate([0, 0, 0.3])).volume()
    print(f"  pillar clamp check (board raised 0.3): {clamp:.1f} mm3 "
          f"(>0 means pillars engage)")
    ok &= insert_walls(base)
    return ok


def insert_walls(base, min_wall=1.8):
    """Verify each insert pocket keeps >= min_wall of solid plastic wrapped
    around the brass over its full length."""
    good = True
    for x, y in SCREWS:
        z0 = CAVITY_H - INSERT_LEN
        ring = cyl(INSERT_OD + 2 * min_wall, INSERT_LEN, x, y, z0) - \
            cyl(INSERT_OD, INSERT_LEN + 1, x, y, z0 - 0.5)
        void, full = (ring - base).volume(), ring.volume()
        if void > 0.02 * full:
            good = False
        print(f"  {'OK ' if void <= 0.02 * full else 'THIN WALL'} post insert "
              f"@({x:.1f},{y:.1f}): {100 * (1 - void / full):.0f}% of a "
              f"{min_wall} mm collar is solid")
    return good


# --------------------------------------------------------------------------
if __name__ == "__main__":
    base, lid = build_base(), build_lid()
    print("interference checks:")
    good = check(base, lid)
    print("all clear" if good else "!! collisions found")

    ext = base.bounding_box()
    print(f"base bbox: x {ext[0]:.1f}..{ext[3]:.1f}  y {ext[1]:.1f}..{ext[4]:.1f}"
          f"  z {ext[2]:.1f}..{ext[5]:.1f}")

    ear_off = EAR_OUT if MOUNT_EARS else 0
    write_stl(base.translate([-OX0, -OY0 + ear_off, FLOOR_T]),
              "Grove10V-Case-Base.stl")
    write_stl(lid.rotate([180, 0, 0]).translate(
        [-OX0, OY1, CAVITY_H + LID_T]), "Grove10V-Case-Lid.stl")
