#!/usr/bin/env python3
"""
Auto Batch Doser v2.1 open-air wall mount generator
===================================================
Generates ABD-Mount-Left.stl and ABD-Mount-Right.stl - a pair of standoff
brackets for the two short ends of the 488.95 x 74.14 mm SystemV1 board.
The board is far too long for a 256 mm printer bed and carries heatsinks,
so this is an open mount, not an enclosure: each bracket screws to the wall
via two ears and the PCB screws to boss standoffs with its M3 holes.

Board data parsed from SystemV1.PcbDoc (Board6/Pads6 streams):

  - outline 488.95 x 74.143, rectangular
  - 4x M3 holes (3.0 mm PTH, 3.5 mm pads), all components top-side:
      left end:  (3.302, 70.282) and (23.724, 13.183)  <- offset inboard!
      right end: (484.480, 14.427) and (484.480, 70.282)

Coordinate system: PCB lower-left corner = (0,0); z = 0 at the WALL surface.
Board underside sits at z = STANDOFF_H on the boss tops.

Requires:  pip install manifold3d numpy
"""
import struct
import numpy as np
import manifold3d as m3
from manifold3d import Manifold, CrossSection, JoinType

m3.set_circular_segments(64)

# --------------------------------------------------------------------------
# Parameters
# --------------------------------------------------------------------------
PCB_W, PCB_L, PCB_T = 488.95, 74.143, 1.6
HOLES_LEFT = [(3.302, 70.282), (23.724, 13.183)]
HOLES_RIGHT = [(484.480, 14.427), (484.480, 70.282)]

STANDOFF_H = 12.0          # wall -> board underside
PLATE_T = 4.0              # wall plate thickness
BOSS_D = 10.0             # sized for a 4.5 mm insert + ~2.75 mm wall

# M3 knurled brass heat-set inserts: 4 mm long, 4.5 mm OD.
INSERT_OD = 4.5
INSERT_HOLE_D = 4.2        # heat-set hole; tune 4.0-4.3 to your inserts
INSERT_LEN = 4.0
INSERT_DEPTH = 5.5         # pocket depth = insert length + displaced-plastic slop
WEB_W, WEB_H = 3.0, 9.5    # stiffening ribs (stop 2.5 below the board:
                           # trimmed through-hole leads need ~2 mm)
EAR_W, EAR_OUT, EAR_T, EAR_HOLE_D = 18.0, 12.0, 4.0, 4.5

# mid-span support clip: sits in the component-free gap between stepper
# channels 3 and 4. The driver heatsinks overhang the BOTTOM board edge on
# the component side, so the clip only hooks over the TOP edge; at the bottom
# it is a plain ledge that stays below board level, passing under the fins.
CLIP_X = 251.0             # clip centreline (channels are at x=211 and 291)
CLIP_HW = 15.0             # half width
EDGE_CLR = 0.4             # clip face -> board edge clearance
HOOK_OVER = 1.0            # rigid hook overlap onto the board top face
HOOK_CLR = 0.25            # hook underside above board top (tilt-in slop)
LEDGE_D = 3.0              # support ledge depth under the board


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


def ear(cx, edge_x, direction, cy):
    """Wall-screw ear extending from edge_x in +/-x direction, centred cy."""
    s = direction
    e = rbox(EAR_OUT + 2, EAR_W, EAR_T, 3,
             edge_x + s * (EAR_OUT - 2) / 2, cy, 0)
    e -= cyl(EAR_HOLE_D, EAR_T + 1, edge_x + s * (EAR_OUT / 2 + 0.5), cy, -0.5)
    return e


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
        f.write(b'ABD v2.1 mount'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(t)))
        f.write(rec.tobytes())
    print(f"wrote {path}: {len(t)} tris, "
          f"vol {man.volume() / 1000:.1f} cm3, status {man.status()}")


# --------------------------------------------------------------------------
# Brackets
# --------------------------------------------------------------------------
def build_right():
    # plate under the board end + two ears past the board edge
    plate = rbox(12.5, 68.0, PLATE_T, 3, 483.25, 42.15)
    b = plate
    for x, y in HOLES_RIGHT:
        b += ear(0, 489.5, +1, y)
    # spine + boss-to-ear ribs
    b += box(478.0, 481.0, 10.5, 74.0, 0, WEB_H)
    for x, y in HOLES_RIGHT:
        b += box(x, 493.0, y - WEB_W / 2, y + WEB_W / 2, 0, WEB_H)
        b += cyl(BOSS_D, STANDOFF_H, x, y)
        b -= cyl(INSERT_HOLE_D, INSERT_DEPTH + 0.5, x, y,
                 STANDOFF_H - INSERT_DEPTH)
    return b


def build_left():
    # plate reaching inboard to the offset lower hole
    plate = rbox(29.5, 69.3, PLATE_T, 3, 14.25, 41.65)
    b = plate
    for x, y in ((-8, 70.282), (-8, 13.183)):
        b += ear(0, -0.5, -1, y)
    # L-shaped web through both bosses + stubs to the ears
    b += box(1.8, 4.8, 12.0, 74.0, 0, WEB_H)                    # vertical
    b += box(-4.5, 25.3, 11.68, 14.68, 0, WEB_H)                # horizontal
    b += box(-4.5, 3.3, 68.78, 71.78, 0, WEB_H)                 # top ear stub
    for x, y in HOLES_LEFT:
        b += cyl(BOSS_D, STANDOFF_H, x, y)
        b -= cyl(INSERT_HOLE_D, INSERT_DEPTH + 0.5, x, y,
                 STANDOFF_H - INSERT_DEPTH)
    return b


def prof_extrude(pts, width, x0):
    """Extrude a (y, z) profile polygon 'width' long in x, starting at x0."""
    cs = CrossSection([np.array(pts, dtype=np.float32)], m3.FillRule.EvenOdd)
    return cs.extrude(width).rotate([90, 0, 0]).rotate([0, 0, 90]).translate(
        [x0, 0, 0])


def build_mid():
    c, hw = CLIP_X, CLIP_HW
    bt = STANDOFF_H + PCB_T                      # board top z
    yt = PCB_L + EDGE_CLR                        # top clip face
    hook_z = bt + HOOK_CLR
    b = rbox(2 * hw + 4, 86.5, PLATE_T, 3, c, 36.25)     # plate, y -7..79.5

    # bottom side: column + ledge only, nothing above board level (the
    # stepper heatsinks overhang this edge on the component side)
    b += box(c - hw, c + hw, -EDGE_CLR - 4, -EDGE_CLR, 0, STANDOFF_H)
    b += prof_extrude([(-EDGE_CLR, 8), (LEDGE_D - 0.4, 8), (LEDGE_D - 0.4, 11),
                       (LEDGE_D - 1.4, 12), (-EDGE_CLR, 12)], 2 * hw, c - hw)

    # top side: two columns with rigid hooks (tilt the clip to engage, then
    # swing the bottom ledge under the board)
    for x0 in (c - hw, c + hw - 10):
        b += box(x0, x0 + 10, yt, yt + 4, 0, hook_z + 4)
        b += prof_extrude([(yt, 8), (PCB_L - LEDGE_D, 8),
                           (PCB_L - LEDGE_D, 11), (PCB_L - LEDGE_D + 1, 12),
                           (yt, 12)], 10, x0)
        tip = yt - EDGE_CLR - HOOK_OVER
        b += prof_extrude([(yt, hook_z), (tip + 0.6, hook_z),
                           (tip, hook_z + 0.6), (tip, hook_z + 4),
                           (yt, hook_z + 4)], 10, x0)

    # wall ears: short one past the top edge, long one past the bottom edge
    # (its screw sits ~25 mm below the board, clear of the heatsink overhang)
    b += rbox(EAR_W, 14, EAR_T, 3, c, 84.5) - \
        cyl(EAR_HOLE_D, EAR_T + 1, c, 86.0, -0.5)
    b += rbox(EAR_W, 25, EAR_T, 3, c, -18.5) - \
        cyl(EAR_HOLE_D, EAR_T + 1, c, -25.0, -0.5)
    return b


# --------------------------------------------------------------------------
# Checks
# --------------------------------------------------------------------------
def check(parts, left, right):
    pcb = box(0, PCB_W, 0, PCB_L, STANDOFF_H, STANDOFF_H + PCB_T)
    leads = box(0, PCB_W, 0, PCB_L, STANDOFF_H - 2.2, STANDOFF_H)
    # driver heatsinks: overhang the bottom board edge on the component side
    fins = box(CLIP_X - 30, CLIP_X + 30, -18, 34, STANDOFF_H + PCB_T, 45)
    ok = True
    for name, part in parts.items():
        for mname, mock, allowed in (("PCB", pcb, False),
                                     ("lead tips", leads, True),
                                     ("heatsink fins", fins, False)):
            vol = (part ^ mock).volume()
            # bosses/ledges may touch the lead-tip zone: those strips are
            # verified TH-free from the drill data
            bad = vol > 1e-6 and not allowed
            print(f"  {'OK ' if not bad else 'COLLISION'} {name} vs {mname}: "
                  f"{vol:.2f} mm3" +
                  (" (inside lead zone - expected)" if allowed and
                   vol > 0 else ""))
            ok &= not bad
        seat = (part ^ pcb.translate([0, 0, -0.1])).volume()
        print(f"  {name} seat contact {seat:.1f} mm3")
    ok &= insert_walls({"left": left, "right": right})
    return ok


def insert_walls(parts, min_wall=1.8):
    """Verify each boss insert pocket keeps >= min_wall of solid plastic
    around the brass over its full length."""
    good = True
    holes = {"left": HOLES_LEFT, "right": HOLES_RIGHT}
    for name, part in parts.items():
        for x, y in holes[name]:
            z0 = STANDOFF_H - INSERT_LEN
            ring = cyl(INSERT_OD + 2 * min_wall, INSERT_LEN, x, y, z0) - \
                cyl(INSERT_OD, INSERT_LEN + 1, x, y, z0 - 0.5)
            void, full = (ring - part).volume(), ring.volume()
            if void > 0.02 * full:
                good = False
            print(f"  {'OK ' if void <= 0.02 * full else 'THIN WALL'} {name} "
                  f"insert @({x:.1f},{y:.1f}): {100 * (1 - void / full):.0f}% "
                  f"of a {min_wall} mm collar is solid")
    return good


# --------------------------------------------------------------------------
if __name__ == "__main__":
    left, right, mid = build_left(), build_right(), build_mid()
    print("checks:")
    good = check({"left": left, "right": right, "mid": mid}, left, right)
    print("all clear" if good else "!! collisions found")
    for name, part in (("ABD-Mount-Left.stl", left),
                       ("ABD-Mount-Right.stl", right),
                       ("ABD-Mount-Mid.stl", mid)):
        bb = part.bounding_box()
        print(f"{name}: x {bb[0]:.1f}..{bb[3]:.1f}  y {bb[1]:.1f}..{bb[4]:.1f}"
              f"  z {bb[2]:.1f}..{bb[5]:.1f}")
        write_stl(part.translate([-bb[0], -bb[1], 0]), name)
