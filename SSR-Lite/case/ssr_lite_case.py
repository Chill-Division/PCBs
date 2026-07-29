#!/usr/bin/env python3
"""
SSR-Lite 3D-printed case generator
==================================
Generates SSR-Lite-Case-Base.stl and SSR-Lite-Case-Lid.stl for the 45 x 60 mm
SSR-Lite PCB (Altium project "Board2").

Coordinate system: PCB lower-left corner = (0,0); z = 0 at the inside floor of
the case.  All dimensions in mm.  Board data extracted from the project's
gerber/NC-drill outputs and the Board2.obj 3D export:

  - board outline: 45 x 60
  - 4x M3 mounting holes (3.0 mm PTH) at (2.667, 2.667) (41.910, 2.667)
    (2.667, 56.896) (41.910, 56.896)
  - 3x 9.5 mm-pitch screw terminal blocks (C8442), bodies 30.6 x 16.5 x 18.5 mm
    that OVERHANG the board edge by up to 6.2 mm:
      T1 left edge,  wire pins at y = 20.61, 30.11, 39.61
      T2 bottom edge, wire pins at x = 11.94, 21.44, 30.94
      T3 top edge,   wire pins at x = 11.96, 21.46, 30.96
  - J1 4P 2 mm control header on right edge, body y 26.9..36.9, 8 mm tall
  - deepest bottom-side part (fuse clips) 8.0 mm below board top surface

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
PCB_W, PCB_L, PCB_T = 45.0, 60.0, 1.6
HOLES = [(2.667, 2.667), (41.910, 2.667), (2.667, 56.896), (41.910, 56.896)]

GAP_L, GAP_R, GAP_B, GAP_T = 7.5, 4.0, 7.5, 7.5   # board edge -> inner wall
WALL = 2.5
FLOOR_T = 2.5
LID_T = 2.5
CAVITY_H = 32.0            # inside floor -> wall top (block top is at 29.1)
OUTER_R = 3.0              # outer corner radius

STANDOFF_H = 9.0           # boss height (fuse clips need >= 8 below board)
BOSS_D = 10.0              # sized for a 4.5 mm insert + ~2.75 mm wall
BOSS_PILOT_DEPTH = 8.0

POST_R = 8.5               # lid screw posts: quarter-round in cavity corners
POST_PILOT_DEPTH = 12.0
POST_SCREW_INSET = 2.5     # screw centre inset from both inner walls

# M3 knurled brass heat-set inserts: 4 mm long, 4.5 mm OD.
INSERT_OD = 4.5
INSERT_HOLE_D = 4.2        # heat-set hole; tune 4.0-4.3 to your inserts
INSERT_LEN = 4.0
INSERT_DEPTH = 5.5         # pocket depth = insert length + displaced-plastic slop
SCREW_CLR_D = 3.4          # M3 screw clearance below the insert
KEEPOUT_CLR = 0.4          # post clearance to the PCB outline

LIP_T, LIP_H, LIP_CLR = 1.8, 2.5, 0.3   # lid inner lip
CBORE_D, CBORE_DEPTH, LID_HOLE_D = 6.8, 1.6, 3.4

WIRE_W, WIRE_H, WIRE_R = 8.0, 9.0, 2.0  # terminal wire ports (per position)
WIRE_Z0 = 8.5              # port bottom above board top (raised 5.5 mm from
                           # the first print to line up with the terminal
                           # wire-entry holes; keep <= 9.0 to clear the lid lip)
J1_W, J1_H, J1_R = 14.0, 10.5, 2.0      # control-cable port, right wall
J1_Z0 = 4.5                # port bottom above board top (raised 4 mm)

VENTS = True               # vent slots in lid
VENT_W, VENT_L, VENT_PITCH, VENT_N = 2.5, 28.0, 5.5, 6

MOUNT_EARS = True          # external wall-mount ears w/ 4.5 mm holes
EAR_W, EAR_OUT, EAR_T, EAR_HOLE_D = 18.0, 12.0, 4.0, 4.5

LOGO_FILE = "chill_logo.png"   # Chill Division logo, debossed into the lid
LOGO_W, LOGO_DEPTH = 34.0, 0.4
LOGO_XY = (20.75, 55.0)        # centre, board coordinates (above the vents)

# board-derived positions
LEFT_YS = [20.61, 30.11, 39.61]
BOT_XS = [11.94, 21.44, 30.94]
TOP_XS = [11.96, 21.46, 30.96]
J1_YC = 29.45              # centred on J1's drilled pins (0.85 mm holes at
                           # y 26.45..32.45); the library 3D body sat 2.45 mm
                           # off the pins, which skewed the first print

# derived
IX0, IX1 = -GAP_L, PCB_W + GAP_R          # inner cavity extents
IY0, IY1 = -GAP_B, PCB_L + GAP_T
OX0, OX1 = IX0 - WALL, IX1 + WALL         # outer extents
OY0, OY1 = IY0 - WALL, IY1 + WALL
PCB_TOP = STANDOFF_H + PCB_T
CORNERS = [(IX0, IY0), (IX1, IY0), (IX0, IY1), (IX1, IY1)]
SCREWS = [(x + POST_SCREW_INSET * (1 if x == IX0 else -1),
           y + POST_SCREW_INSET * (1 if y == IY0 else -1)) for x, y in CORNERS]


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def rrect(w, l, r):
    """Rounded rectangle CrossSection centred at origin."""
    return CrossSection.square([w - 2 * r, l - 2 * r], True).offset(
        r, JoinType.Round)


def rbox(w, l, h, r, cx=0.0, cy=0.0, z0=0.0):
    """Rounded-corner box, footprint centred at (cx, cy), from z0 to z0+h."""
    return rrect(w, l, r).extrude(h).translate([cx, cy, z0])


def cyl(d, h, x=0.0, y=0.0, z0=0.0):
    return Manifold.cylinder(h, d / 2).translate([x, y, z0])


def insert_pocket(x, y, z_top, depth):
    """Heat-set insert pocket drilled down from z_top: INSERT_DEPTH of
    INSERT_HOLE_D for the brass, then SCREW_CLR_D clearance for the screw
    tip below it."""
    p = cyl(INSERT_HOLE_D, INSERT_DEPTH + 0.5, x, y, z_top - INSERT_DEPTH)
    if depth > INSERT_DEPTH:
        p += cyl(SCREW_CLR_D, depth - INSERT_DEPTH + 0.1, x, y, z_top - depth)
    return p


def pcb_keepout(z0):
    """Volume the PCB sweeps through when it is lowered onto the bosses -
    corner posts must stay out of it from z0 (board underside) upward."""
    return Manifold.cube([PCB_W + 2 * KEEPOUT_CLR, PCB_L + 2 * KEEPOUT_CLR,
                          CAVITY_H + 1 - z0]).translate(
        [-KEEPOUT_CLR, -KEEPOUT_CLR, z0])


def port_x(w, h, r, y, zc, x_wall):
    """Rounded-rect cutter through a wall parallel to the y axis (left/right).
    w along y, h along z, centred at (y, zc), piercing wall at x_wall."""
    p = rrect(h, w, r).extrude(WALL + 2).translate([0, 0, -(WALL + 2) / 2])
    return p.rotate([0, 90, 0]).translate([x_wall + WALL / 2, y, zc])


def port_y(w, h, r, x, zc, y_wall):
    """Same, through a wall parallel to the x axis (top/bottom)."""
    p = rrect(w, h, r).extrude(WALL + 2).translate([0, 0, -(WALL + 2) / 2])
    return p.rotate([90, 0, 0]).translate([x, y_wall + WALL / 2, zc])


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
    tv = v[t]                                     # (M, 3, 3)
    n = np.cross(tv[:, 1] - tv[:, 0], tv[:, 2] - tv[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    rec = np.zeros(len(t), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)),
                                           ('attr', '<u2')]))
    rec['n'], rec['v'] = n, tv
    with open(path, 'wb') as f:
        f.write(b'SSR-Lite case'.ljust(80, b'\0'))
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
    cavity = rbox(cw, cl, CAVITY_H + 1, 1.0, icx, icy, 0)
    base = shell - cavity

    # PCB bosses
    for x, y in HOLES:
        base += cyl(BOSS_D, STANDOFF_H, x, y)

    # lid-screw corner posts (quarter-round: cylinder clipped by the cavity).
    # They are fat enough to wrap a 4.5 mm insert, so they would foul the PCB
    # corners - the board's drop-in volume is trimmed back off them.
    cav_clip = rbox(cw, cl, CAVITY_H, 1.0, icx, icy, 0)
    keepout = pcb_keepout(STANDOFF_H)
    for x, y in CORNERS:
        base += (cyl(POST_R * 2, CAVITY_H, x, y) ^ cav_clip) - keepout

    # wire ports
    wz = PCB_TOP + WIRE_Z0 + WIRE_H / 2
    cuts = []
    for y in LEFT_YS:
        cuts.append(port_x(WIRE_W, WIRE_H, WIRE_R, y, wz, IX0 - WALL))
    for x in BOT_XS:
        cuts.append(port_y(WIRE_W, WIRE_H, WIRE_R, x, wz, IY0 - WALL))
    for x in TOP_XS:
        cuts.append(port_y(WIRE_W, WIRE_H, WIRE_R, x, wz, IY1))
    # control-cable port
    cuts.append(port_x(J1_W, J1_H, J1_R, J1_YC,
                       PCB_TOP + J1_Z0 + J1_H / 2, IX1))
    # heat-set insert pockets
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
# Lid  (modelled in assembly position: plate from CAVITY_H up)
# --------------------------------------------------------------------------
def build_lid():
    ow, ol = OX1 - OX0, OY1 - OY0
    cw, cl = IX1 - IX0, IY1 - IY0
    ocx, ocy = (OX0 + OX1) / 2, (OY0 + OY1) / 2
    icx, icy = (IX0 + IX1) / 2, (IY0 + IY1) / 2

    lid = rbox(ow, ol, LID_T, OUTER_R, ocx, ocy, CAVITY_H)

    # inner lip (cleared around the corner posts)
    lw, ll = cw - 2 * LIP_CLR, cl - 2 * LIP_CLR
    lip = rbox(lw, ll, LIP_H, 0.8, icx, icy, CAVITY_H - LIP_H) - \
        rbox(lw - 2 * LIP_T, ll - 2 * LIP_T, LIP_H + 1, 0.8, icx, icy,
             CAVITY_H - LIP_H - 0.5)
    for x, y in CORNERS:
        lip -= cyl((POST_R + 1.5) * 2, LIP_H + 1, x, y, CAVITY_H - LIP_H - 0.5)
    lid += lip

    # screw holes + counterbores
    cuts = []
    for x, y in SCREWS:
        cuts.append(cyl(LID_HOLE_D, LID_T + LIP_H + 1, x, y,
                        CAVITY_H - LIP_H - 0.5))
        cuts.append(cyl(CBORE_D, CBORE_DEPTH + 0.5, x, y,
                        CAVITY_H + LID_T - CBORE_DEPTH))
    # vents
    if VENTS:
        x0 = icx - (VENT_N - 1) * VENT_PITCH / 2
        for i in range(VENT_N):
            cuts.append(rbox(VENT_W, VENT_L, LID_T + 1, VENT_W / 2 - 0.05,
                             x0 + i * VENT_PITCH, icy - 5, CAVITY_H - 0.5))
    lid -= Manifold.batch_boolean(cuts, m3.OpType.Add)
    logo = logo_deboss(LOGO_XY[0], LOGO_XY[1], CAVITY_H + LID_T)
    if logo is not None:
        lid -= logo
    return lid


# --------------------------------------------------------------------------
# Interference checks against board-derived envelopes
# --------------------------------------------------------------------------
def check(base, lid):
    def box(x0, x1, y0, y1, z0, z1):
        return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate(
            [x0, y0, z0])

    bt = STANDOFF_H + PCB_T          # board top abs z
    mocks = {
        "PCB": box(0, PCB_W, 0, PCB_L, STANDOFF_H, bt),
        "T1 block": box(-5.98, 10.52, 16.42, 46.99, bt, bt + 18.45),
        "T2 block": box(6.38, 36.96, -1.41, 15.09, bt, bt + 18.45),
        "T3 block": box(4.96, 35.54, 49.69, 66.19, bt, bt + 18.45),
        "J1 header": box(38.97, 44.07, 24.2, 34.7, bt, bt + 8.0),
        "FH1 fuse": box(20.48, 31.09, 15.50, 20.53, bt - 9.7, STANDOFF_H),
        "FH2 fuse": box(20.48, 31.09, 44.46, 49.49, bt - 9.7, STANDOFF_H),
        "U1/U2/Q1/Q2": box(7.0, 37.0, 20.0, 42.0, bt - 8.1, STANDOFF_H),
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
    # PCB must rest on all four bosses (Ø10 bosses overhang the board edge,
    # so only the part of each annulus under the board actually seats)
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
    """Verify every insert pocket has >= min_wall of solid plastic wrapped
    around the brass over its full length: the annulus from the melted-in
    insert OD out to OD + 2*min_wall must be entirely inside the part."""
    good = True
    sites = [("boss", x, y, STANDOFF_H) for x, y in HOLES] + \
            [("post", x, y, CAVITY_H) for x, y in SCREWS]
    for label, x, y, z_top in sites:
        z0 = z_top - INSERT_LEN
        ring = cyl(INSERT_OD + 2 * min_wall, INSERT_LEN, x, y, z0) - \
            cyl(INSERT_OD, INSERT_LEN + 1, x, y, z0 - 0.5)
        void = (ring - base).volume()
        full = ring.volume()
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

    # export: base floor on the bed; lid flipped so its outer face is on the bed
    ear_off = EAR_OUT if MOUNT_EARS else 0
    write_stl(base.translate([-OX0, -OY0 + ear_off, FLOOR_T]),
              "SSR-Lite-Case-Base.stl")
    write_stl(lid.rotate([180, 0, 0]).translate(
        [-OX0, OY1, CAVITY_H + LID_T]), "SSR-Lite-Case-Lid.stl")
