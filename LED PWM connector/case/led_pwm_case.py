#!/usr/bin/env python3
"""
LED PWM connector (RJ11 -> Grove) 3D-printed case generator
===========================================================
Generates LEDPWM-Case-Base.stl and LEDPWM-Case-Lid.stl for the 50 x 28 mm
PoESP_ConnectorBoard PCB.

Board data extracted from PoESP_ConnectorBoard.PcbDoc (no gerbers were in the
project, so outline / components / pads were parsed straight from the Altium
binary):

  - board outline: 50.0 x 28.0, rectangular
  - 4x M3 mounting holes (3.0 mm PTH, 4.5 mm pads) at (4,4) (46,4) (46,24)
    (4,24) - note: the board DOES have mounting holes
  - J1 Grove socket (Seeed 114020164) centred (5.30, 14.00), port faces the
    LEFT board edge, ~5.5 mm tall SMD
  - J2 RJ11 jack (Molex 0955012661) centred (42.04, 14.00), port faces the
    RIGHT board edge, ~11.5 mm tall; TH pins x 33.1..35.7 and two 3.25 mm
    snap posts at (42.04, 8.92) / (42.04, 19.08), up to ~3.2 mm below board
  - T1 SOT-23 + two 0603 resistors mid-board (LED1/R2 not populated)

Coordinate system: PCB lower-left corner = (0,0); z = 0 at the inside floor.

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
PCB_W, PCB_L, PCB_T = 50.0, 28.0, 1.6
HOLES = [(4.0, 4.0), (46.0, 4.0), (46.0, 24.0), (4.0, 24.0)]

GAP_L, GAP_R = 1.5, 1.5    # connector ports nearly flush with these walls
GAP_B, GAP_T = 7.0, 7.0    # room for the lid-screw corner posts
WALL = 2.5
FLOOR_T = 2.5
LID_T = 2.5
CAVITY_H = 20.0            # inside floor -> wall top (RJ11 top is ~17.6)
OUTER_R = 3.0

STANDOFF_H = 4.5           # RJ11 pins/posts reach ~3.2 mm below the board
BOSS_D = 10.0             # sized for a 4.5 mm insert + ~2.75 mm wall
BOSS_PILOT_DEPTH = 6.0     # continues ~1.5 mm into the floor

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

LIP_T, LIP_H, LIP_CLR = 1.8, 2.5, 0.3
CBORE_D, CBORE_DEPTH, LID_HOLE_D = 6.8, 1.6, 3.4

GR_YC, GR_W, GR_Z0, GR_Z1 = 14.0, 13.0, 5.7, 14.7    # Grove port, left wall
RJ_YC, RJ_W, RJ_Z0, RJ_Z1 = 14.0, 14.0, 5.7, 18.1    # RJ11 port, right wall

MOUNT_EARS = True
EAR_W, EAR_OUT, EAR_T, EAR_HOLE_D = 18.0, 12.0, 4.0, 4.5

LOGO_FILE = "chill_logo.png"   # Chill Division logo, debossed into the lid
LOGO_W, LOGO_DEPTH = 30.0, 0.4
LOGO_XY = (25.0, 14.0)         # centre, board coordinates

# derived
IX0, IX1 = -GAP_L, PCB_W + GAP_R
IY0, IY1 = -GAP_B, PCB_L + GAP_T
OX0, OX1 = IX0 - WALL, IX1 + WALL
OY0, OY1 = IY0 - WALL, IY1 + WALL
PCB_TOP = STANDOFF_H + PCB_T
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
        f.write(b'LED PWM connector case'.ljust(80, b'\0'))
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

    # PCB bosses. The right-side Ø10 bosses would graze the RJ11's snap-post
    # stubs that poke through the board, so scallop clearance for those.
    rj_posts = Manifold.batch_boolean(
        [cyl(4.4, STANDOFF_H + 1, px, py, -0.5)
         for px, py in ((42.037, 8.92), (42.037, 19.08))], m3.OpType.Add)
    for x, y in HOLES:
        base += cyl(BOSS_D, STANDOFF_H, x, y) - rj_posts

    # lid-screw corner posts
    # posts wrap a 4.5 mm insert, so trim the board's drop-in volume off them
    cav_clip = rbox(cw, cl, CAVITY_H, 1.0, icx, icy, 0)
    keepout = pcb_keepout(STANDOFF_H)
    for x, y in CORNERS:
        base += (cyl(POST_R * 2, CAVITY_H, x, y) ^ cav_clip) - keepout

    # ports + pilot holes
    cuts = [
        port_x(GR_W, GR_Z1 - GR_Z0, 2.0, GR_YC, (GR_Z0 + GR_Z1) / 2,
               IX0 - WALL),
        port_x(RJ_W, RJ_Z1 - RJ_Z0, 2.0, RJ_YC, (RJ_Z0 + RJ_Z1) / 2, IX1),
    ]
    for x, y in HOLES:
        cuts.append(insert_pocket(x, y, STANDOFF_H, BOSS_PILOT_DEPTH))
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

    # inner lip, cleared at posts and over the RJ11 (jack is lip-height)
    lw, ll = cw - 2 * LIP_CLR, cl - 2 * LIP_CLR
    lip = rbox(lw, ll, LIP_H, 0.8, icx, icy, CAVITY_H - LIP_H) - \
        rbox(lw - 2 * LIP_T, ll - 2 * LIP_T, LIP_H + 1, 0.8, icx, icy,
             CAVITY_H - LIP_H - 0.5)
    for x, y in CORNERS:
        lip -= cyl((POST_R + 1.5) * 2, LIP_H + 1, x, y, CAVITY_H - LIP_H - 0.5)
    lip -= box(PCB_W - 2.0, IX1 + 1, RJ_YC - RJ_W / 2 - 1, RJ_YC + RJ_W / 2 + 1,
               CAVITY_H - LIP_H - 0.5, CAVITY_H + 1)
    lid += lip

    # screw holes + counterbores
    cuts = []
    for x, y in SCREWS:
        cuts.append(cyl(LID_HOLE_D, LID_T + LIP_H + 1, x, y,
                        CAVITY_H - LIP_H - 0.5))
        cuts.append(cyl(CBORE_D, CBORE_DEPTH + 0.5, x, y,
                        CAVITY_H + LID_T - CBORE_DEPTH))
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
        "PCB": box(0, PCB_W, 0, PCB_L, STANDOFF_H, bt),
        "J1 Grove": box(-1.0, 10.6, 7.4, 20.6, bt, bt + 6.0),
        "J2 RJ11": box(36.6, 51.4, 7.3, 20.7, bt, bt + 11.6),
        "RJ11 pins": box(32.4, 36.4, 10.0, 18.0, 1.3, STANDOFF_H),
        "RJ11 post A": cyl(3.4, STANDOFF_H - 1.4, 42.037, 8.92, 1.4),
        "RJ11 post B": cyl(3.4, STANDOFF_H - 1.4, 42.037, 19.08, 1.4),
        "T1/R blanket": box(18.0, 30.0, 12.0, 17.0, bt, bt + 1.5),
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
    thin = mocks["PCB"].translate([0, 0, -0.1])
    for x, y in HOLES:
        a = ((base ^ thin) ^ cyl(BOSS_D + 1, 1.0, x, y, STANDOFF_H - 0.5)
             ).volume() / 0.1
        if a < 10.0:
            ok = False
        print(f"  {'OK ' if a >= 10.0 else 'NO SEAT'} boss @({x:.1f},{y:.1f}) "
              f"seats {a:.1f} mm2 under the board")
    ok &= insert_walls(base)
    return ok


def insert_walls(base, min_wall=1.8):
    """Verify each insert pocket keeps >= min_wall of solid plastic around
    the brass over its full length."""
    good = True
    sites = [("boss", x, y, STANDOFF_H) for x, y in HOLES] + \
            [("post", x, y, CAVITY_H) for x, y in SCREWS]
    for label, x, y, z_top in sites:
        z0 = z_top - INSERT_LEN
        ring = cyl(INSERT_OD + 2 * min_wall, INSERT_LEN, x, y, z0) - \
            cyl(INSERT_OD, INSERT_LEN + 1, x, y, z0 - 0.5)
        void, full = (ring - base).volume(), ring.volume()
        if void > 0.02 * full:
            good = False
        print(f"  {'OK ' if void <= 0.02 * full else 'THIN WALL'} {label} "
              f"insert @({x:.1f},{y:.1f}): {100 * (1 - void / full):.0f}% of a "
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
              "LEDPWM-Case-Base.stl")
    write_stl(lid.rotate([180, 0, 0]).translate(
        [-OX0, OY1, CAVITY_H + LID_T]), "LEDPWM-Case-Lid.stl")
