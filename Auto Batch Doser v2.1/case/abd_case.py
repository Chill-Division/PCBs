#!/usr/bin/env python3
"""
Auto Batch Doser v2.1 ENCLOSED case generator (sectioned)
=========================================================
A second, properly-enclosed case for the 488.95 x 74.14 mm SystemV1 board -
distinct from the open standoff mount in ../mount. The board is enclosed on
all sides EXCEPT a full-width open channel over the stepper-driver row, where
the finned heatsink protrudes up and overhangs the front long edge.

Because the board is 489 mm long and the P1S bed is ~256 mm, the base and lid
each tile into 3 sections (~163 mm) that bolt together:
  - floor tongue-and-groove self-aligns each seam (shear + coplanarity)
  - a bolt-on back-wall splice plate clamps each seam (4x M3 inserts)

Board data parsed from SystemV1.PcbDoc:
  - outline 488.95 x 74.143, all components top-side
  - 4x M3 holes: (3.302,70.282) (23.724,13.183) (484.480,14.427) (484.480,70.282)
  - external connectors clustered at the LEFT end:
      F1 power polar header (4.11, 3.76), P1 2P screw term (7.70, 28.80),
      J5 4P header (4.70, 54.60);  S1 programming 2x3 (65.02, 64.39)

Coordinate system: PCB lower-left = (0,0); z = 0 at the inside floor surface,
board underside rests at z = STANDOFF_H.  Front long edge = y0 side (heatsink
overhang); back long edge = y = PCB_L side.

  >>> HEATSINK / CLEARANCE NUMBERS ARE PLACEHOLDERS <<<
  Confirm HS_X0/HS_X1 (channel span), HS_FRONT_OPEN (how far the channel opens
  past the front edge for the overhang), and LID_CLEAR (interior height over the
  tallest non-heatsink part) against the real hardware, then regenerate.

Requires:  pip install manifold3d numpy
"""
import struct
import numpy as np
import manifold3d as m3
from manifold3d import Manifold, CrossSection, JoinType

m3.set_circular_segments(48)

# --------------------------------------------------------------------------
# Parameters
# --------------------------------------------------------------------------
PCB_W, PCB_L, PCB_T = 488.95, 74.143, 1.6
HOLES = [(3.302, 70.282), (23.724, 13.183), (484.480, 14.427), (484.480, 70.282)]

FLOOR_T = 3.0
WALL = 3.0
STANDOFF_H = 4.5           # inside floor -> board underside (lead clearance)
LID_CLEAR = 18.0           # board top -> lid inner face (tallest non-HS part) *
LID_T = 3.0
GAP = 1.6                  # board edge -> inner wall
LEDGE_D = 3.0              # board-support ledge reach under the board
OUTER_R = 3.0

# Heatsink open channel (component-side driver row).  *placeholders*
HS_X0, HS_X1 = 36.0, 474.0     # channel span along the board
HS_BACK_Y = 34.0               # channel extends from the front edge to this y
HS_FRONT_OPEN = 16.0           # how far the channel opens PAST the front edge
                               # (front wall removed here for the fin overhang)

# Left-end cable ports (y centre, width, z0 above board, height)
PORTS = [(3.76, 10.0, 0.0, 9.0),    # F1 power
         (28.80, 12.0, 0.0, 9.0),   # P1 screw terminal
         (54.60, 9.0, 0.0, 8.0)]    # J5 header

# Sectioning: base seams and lid seams are STAGGERED (brickwork) so each lid
# section bridges a base seam and bolts to both base sections through the
# external flanges - the lid is the top splice. The bottom of each base seam
# is clamped by two lap tongues screwed from the outside floor face.
SEAMS = [PCB_W / 3.0, 2.0 * PCB_W / 3.0]   # base seams: 162.98, 325.97
LID_SEAMS = [100.0, 280.0]                 # lid seams, offset from base seams

# floor lap joints (2 per base seam, at TH-free spots verified from Pads6)
LAP_L = 14.0               # tongue reach past the seam
LAP_ROOT = 10.0            # tongue doubler reach back into its own section
LAP_W = 12.0               # tongue width (y)
LAP_T = 2.0                # tongue thickness
RECESS = 1.0               # tongue sits this deep in the next floor's top
LAP_CLR = 0.25             # tongue-to-recess fit clearance
LAP_YS = (8.0, 66.0)       # y centres (clear of ledges and lead tips)
CSK_D = 6.6                # countersink for flush M3 flat-head from outside

# M3 knurled brass heat-set inserts: 4 mm long, 4.5 mm OD.
INSERT_OD = 4.5
INSERT_HOLE_D = 4.2
INSERT_LEN = 4.0
INSERT_DEPTH = 5.5
SCREW_CLR_D = 3.4
BOSS_D = 10.0

# lid fixing: EXTERNAL bolt flanges on the back edge. Each base flange is a
# solid block hanging below the wall top with a vertical M3 insert; the lid
# flange sits on top and is screwed straight down from outside. No internal
# posts or lip.
CBORE_D, CBORE_DEPTH, LID_HOLE_D = 6.8, 2.5, 3.4
LID_FLANGE_OUT = 13.0      # how far the flange sticks out past the back wall
LID_FLANGE_W = 16.0        # flange width along x
LID_FLANGE_H = 8.0         # base flange block height below the wall top

# wall-mount ears (on the back long edge, at floor level)
EAR_W, EAR_OUT, EAR_T, EAR_HOLE_D = 20.0, 14.0, FLOOR_T + 3.0, 4.5
EARS_PER_SECTION_X = (0.28, 0.72)     # fractional x within each section

# derived
IY0, IY1 = -GAP, PCB_L + GAP               # inner wall faces (front, back)
OY0, OY1 = IY0 - WALL, IY1 + WALL
IX0, IX1 = -GAP, PCB_W + GAP
OX0, OX1 = IX0 - WALL, IX1 + WALL
BOARD_TOP = STANDOFF_H + PCB_T
WALL_H = STANDOFF_H + PCB_T + LID_CLEAR     # inside floor -> wall top
BOUND_X = [OX0] + SEAMS + [OX1]             # base section x boundaries
LID_BOUND_X = [OX0] + LID_SEAMS + [OX1]     # lid section x boundaries
# external lid-flange x positions (global: staggering means lid sections must
# catch flanges on BOTH sides of every base seam)
FLANGE_XS = [15.0, 75.0, 140.0, 190.0, 255.0, 305.0, 370.0, 460.0]


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


def insert_pocket_z(x, y, z_top, depth):
    """Vertical heat-set pocket drilled down from z_top."""
    p = cyl(INSERT_HOLE_D, INSERT_DEPTH + 0.5, x, y, z_top - INSERT_DEPTH)
    if depth > INSERT_DEPTH:
        p += cyl(SCREW_CLR_D, depth - INSERT_DEPTH + 0.1, x, y, z_top - depth)
    return p


def prof_extrude(pts, width, x0):
    """Extrude a (y, z) profile polygon 'width' long in x, starting at x0."""
    cs = CrossSection([np.array(pts, dtype=np.float32)], m3.FillRule.EvenOdd)
    return cs.extrude(width).rotate([90, 0, 0]).rotate([0, 0, 90]).translate(
        [x0, 0, 0])


def csk_hole(x, y, z_bottom):
    """M3 clearance hole + flush countersink cut upward from an outside face."""
    h = cyl(LID_HOLE_D, FLOOR_T + RECESS + 1, x, y, z_bottom - 0.5)
    cone = Manifold.cylinder((CSK_D - LID_HOLE_D) / 2, CSK_D / 2,
                             LID_HOLE_D / 2).translate([x, y, z_bottom - 0.05])
    return h + cone


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
        f.write(b'ABD v2.1 enclosed case'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(t)))
        f.write(rec.tobytes())
    print(f"wrote {path}: {len(t)} tris, vol {man.volume() / 1000:.0f} cm3, "
          f"status {man.status()}")


# --------------------------------------------------------------------------
# Full base (before sectioning)
# --------------------------------------------------------------------------
def in_channel(x):
    return HS_X0 <= x <= HS_X1


def build_full_base():
    # outer shell + cavity
    shell = box(OX0, OX1, OY0, OY1, -FLOOR_T, WALL_H)
    cav = box(IX0, IX1, IY0, IY1, 0, WALL_H + 1)
    base = shell - cav

    # front wall cut down to just above the board over the heatsink channel,
    # and opened outward by HS_FRONT_OPEN for the fin overhang
    base -= box(HS_X0, HS_X1, OY0 - HS_FRONT_OPEN, IY0, BOARD_TOP, WALL_H + 1)
    base -= box(HS_X0, HS_X1, OY0 - HS_FRONT_OPEN, OY0, -FLOOR_T - 1, WALL_H + 1)

    # board-support ledges (top at board underside)
    base += box(IY_ledge0(), IX1, IY1 - LEDGE_D, IY1, 0, STANDOFF_H)  # back
    for x0, x1 in front_ledge_spans():
        base += box(x0, x1, IY0, IY0 + LEDGE_D, 0, STANDOFF_H)

    # PCB bosses (M3 inserts)
    for x, y in HOLES:
        base += cyl(BOSS_D, STANDOFF_H, x, y)

    # external lid-fixing flanges on the back edge (block hangs below the wall
    # top so it can host a vertical insert), each with a 45-degree gusset
    # underneath so it prints support-free
    flanges = lid_flanges()
    for x, yf in flanges:
        base += box(x - LID_FLANGE_W / 2, x + LID_FLANGE_W / 2,
                    IY1, OY1 + LID_FLANGE_OUT, WALL_H - LID_FLANGE_H, WALL_H)
        zb = WALL_H - LID_FLANGE_H
        base += prof_extrude([(OY1, zb), (OY1 + LID_FLANGE_OUT, zb),
                              (OY1, zb - LID_FLANGE_OUT)],
                             LID_FLANGE_W, x - LID_FLANGE_W / 2)

    # ---- cut-outs ----
    cuts = []
    # left-end cable ports
    for yc, w, z0, h in PORTS:
        cuts.append(box(OX0 - 1, IX0 + 0.5, yc - w / 2, yc + w / 2,
                        BOARD_TOP + z0, BOARD_TOP + z0 + h))
    # PCB boss inserts
    for x, y in HOLES:
        cuts.append(insert_pocket_z(x, y, STANDOFF_H, INSERT_DEPTH + 1))
    # lid-flange inserts (vertical, from the wall top down into the block)
    for x, yf in flanges:
        cuts.append(insert_pocket_z(x, yf, WALL_H, INSERT_DEPTH + 0.5))
    base -= Manifold.batch_boolean(cuts, m3.OpType.Add)

    # wall-mount ears on the back edge
    for i in range(3):
        xa, xb = section_x(i)
        for f in EARS_PER_SECTION_X:
            xc = xa + (xb - xa) * f
            ear = rbox(EAR_W, EAR_OUT + 2, EAR_T, 3, xc,
                       OY1 + (EAR_OUT - 2) / 2, -FLOOR_T)
            ear -= cyl(EAR_HOLE_D, EAR_T + 1, xc, OY1 + EAR_OUT / 2 + 1,
                       -FLOOR_T - 0.5)
            base += ear
    return base


def KEEP():
    return 0.4


def IY_ledge0():
    return IX0     # back ledge runs the whole length


def front_ledge_spans():
    # front ledge everywhere except the open heatsink channel
    return [(IX0, HS_X0), (HS_X1, IX1)]


def lid_flanges():
    """External back-edge lid-fixing points: (x, insert-y) pairs. Global x
    list, placed so no flange straddles a base or lid seam and every lid
    section catches flanges on both sides of the base seam it bridges."""
    yf = OY1 + LID_FLANGE_OUT * 0.55
    return [(x, yf) for x in FLANGE_XS]


def section_x(i):
    return BOUND_X[i], BOUND_X[i + 1]


# --------------------------------------------------------------------------
# Full lid
# --------------------------------------------------------------------------
def build_full_lid():
    lid = box(OX0, OX1, OY0, OY1, WALL_H, WALL_H + LID_T)
    # open channel over the heatsink (full width past the front edge)
    lid -= box(HS_X0, HS_X1, OY0 - HS_FRONT_OPEN, HS_BACK_Y,
               WALL_H - 1, WALL_H + LID_T + 1)
    # external bolt flanges (sit on top of the base flanges)
    for x, yf in lid_flanges():
        lid += box(x - LID_FLANGE_W / 2, x + LID_FLANGE_W / 2,
                   IY1, OY1 + LID_FLANGE_OUT, WALL_H, WALL_H + LID_T)
    # screw clearance holes + counterbores through the flanges
    cuts = []
    for x, yf in lid_flanges():
        cuts.append(cyl(LID_HOLE_D, LID_T + 2, x, yf, WALL_H - 1))
        cuts.append(cyl(CBORE_D, CBORE_DEPTH + 0.5, x, yf,
                        WALL_H + LID_T - CBORE_DEPTH))
    return lid - Manifold.batch_boolean(cuts, m3.OpType.Add)


# --------------------------------------------------------------------------
# Sectioning
# --------------------------------------------------------------------------
def slab(bounds, i):
    x0 = bounds[i] - (0.0 if i > 0 else 1.0)
    x1 = bounds[i + 1] + (0.0 if i < len(bounds) - 2 else 1.0)
    return box(x0, x1, OY0 - HS_FRONT_OPEN - 5, OY1 + LID_FLANGE_OUT + EAR_OUT + 5,
               -FLOOR_T - 5, WALL_H + LID_T + 5)


def lap_screws(s):
    """Bottom lap screw centres for base seam s."""
    return [(s + LAP_L / 2, y) for y in LAP_YS]


def section_base(full, i):
    """Base section i. At its RIGHT seam it carries two lap tongues (2 mm
    thick, sitting in a 1 mm recess of the next floor) topped by Ø10 insert
    bosses; the next section is clamped onto them by countersunk M3s driven
    up from the outside floor face. Tongue tops stay >= 1.5 mm below the
    board so trimmed lead tips clear; spots verified TH-free from Pads6."""
    part = full ^ slab(BOUND_X, i)
    if i < 2:                       # male side: tongues + bosses + inserts
        s = SEAMS[i]
        add, cut = [], []
        for x, y in lap_screws(s):
            add.append(box(s - LAP_ROOT, s + LAP_L, y - LAP_W / 2,
                           y + LAP_W / 2, -RECESS, -RECESS + LAP_T))
            add.append(cyl(BOSS_D, STANDOFF_H - 0.2 - (-RECESS + LAP_T),
                           x, y, -RECESS + LAP_T))
            cut.append(cyl(INSERT_HOLE_D, INSERT_LEN + 1.0, x, y,
                           -RECESS - 0.5))
        part += Manifold.batch_boolean(add, m3.OpType.Add)
        part -= Manifold.batch_boolean(cut, m3.OpType.Add)
    if i > 0:                       # female side: recess + flush csk holes
        s = SEAMS[i - 1]
        cut = []
        for x, y in lap_screws(s):
            cut.append(box(s - LAP_CLR, s + LAP_L + LAP_CLR,
                           y - LAP_W / 2 - LAP_CLR, y + LAP_W / 2 + LAP_CLR,
                           -RECESS, 0.5))
            cut.append(csk_hole(x, y, -FLOOR_T))
        part -= Manifold.batch_boolean(cut, m3.OpType.Add)
    return part


def section_lid(full, i):
    return full ^ slab(LID_BOUND_X, i)


# --------------------------------------------------------------------------
# Checks
# --------------------------------------------------------------------------
def check(base_secs, lid_secs):
    ok = True
    pcb = box(0, PCB_W, 0, PCB_L, STANDOFF_H, BOARD_TOP)
    leads = box(0, PCB_W, 0, PCB_L, -0.1, STANDOFF_H)
    # heatsink mock: sits over the driver row, rises above the board and
    # overhangs the front edge - must live entirely in the open channel
    hs = box(HS_X0 + 2, HS_X1 - 2, -HS_FRONT_OPEN + 2, HS_BACK_Y - 2,
             BOARD_TOP, WALL_H + LID_T + 20)
    allb = Manifold.batch_boolean(base_secs, m3.OpType.Add)
    alll = Manifold.batch_boolean(lid_secs, m3.OpType.Add)
    for nm, part in (("base", allb), ("lid", alll)):
        for mn, mock, allow in (("PCB", pcb, False), ("leads", leads, True),
                                ("heatsink", hs, False)):
            v = (part ^ mock).volume()
            bad = v > 1.0 and not allow
            ok &= not bad
            print(f"  {'OK ' if not bad else 'COLLISION'} {nm} vs {mn}: "
                  f"{v:.1f} mm3" + (" (leads below board - expected)"
                                    if allow and v > 1 else ""))
    seat = (allb ^ pcb.translate([0, 0, -0.1])).volume()
    print(f"  board seat contact {seat:.0f} mm3 (ledges + bosses)")
    # assembled sections must mate without interference (tongue in recess)
    for i in (0, 1):
        v = (base_secs[i] ^ base_secs[i + 1]).volume()
        ok &= v < 0.01
        print(f"  {'OK ' if v < 0.01 else 'INTERFERES'} base seam {i + 1}: "
              f"section overlap {v:.3f} mm3")
    # every insert (lid flanges + floor laps) keeps a >=1.8 mm solid collar
    mw = 1.8
    sites = [("flange", x, yf, WALL_H - INSERT_LEN) for x, yf in lid_flanges()]
    sites += [("lap", x, y, -RECESS)
              for s in SEAMS for x, y in lap_screws(s)]
    thin = 0
    for lab, x, y, z0 in sites:
        ring = cyl(INSERT_OD + 2 * mw, INSERT_LEN, x, y, z0) - \
            cyl(INSERT_OD, INSERT_LEN + 1, x, y, z0 - 0.5)
        void = (ring - allb).volume()
        full = ring.volume()
        if void > 0.02 * full:
            ok = False
            thin += 1
            print(f"  THIN WALL {lab} insert @({x:.0f},{y:.0f}): "
                  f"{100 * (1 - void / full):.0f}% collar")
    print(f"  insert collars: {len(sites)} checked, "
          f"{len(sites) - thin} solid, {thin} thin")
    # every lid section must bridge with >=2 flanges, incl. both sides of any
    # base seam it spans
    for i in range(3):
        la, lb = LID_BOUND_X[i], LID_BOUND_X[i + 1]
        fx = [x for x in FLANGE_XS if la < x < lb]
        spans = [s for s in SEAMS if la < s < lb]
        bridged = all(any(x < s for x in fx) and any(x > s for x in fx)
                      for s in spans)
        good_n = len(fx) >= 2 and bridged
        ok &= good_n
        print(f"  lid section {i + 1}: flanges at {fx}, bridges base seams "
              f"{spans if spans else 'none'} {'OK' if good_n else 'BAD'}")
    # sections must fit the bed
    for lab, bounds in (("base", BOUND_X), ("lid", LID_BOUND_X)):
        for i in range(3):
            x0, x1 = max(bounds[i], OX0), min(bounds[i + 1], OX1)
            L = x1 - x0 + (LAP_L if (lab == "base" and i < 2) else 0)
            print(f"  {lab} section {i + 1}: x {x0:.0f}..{x1:.0f}, "
                  f"print length {L:.0f} mm "
                  f"{'OK' if L < 250 else 'TOO LONG'}")
            ok &= L < 250
    return ok


# --------------------------------------------------------------------------
if __name__ == "__main__":
    fb, fl = build_full_base(), build_full_lid()
    base_secs = [section_base(fb, i) for i in range(3)]
    lid_secs = [section_lid(fl, i) for i in range(3)]
    print("checks:")
    good = check(base_secs, lid_secs)
    print("all clear" if good else "!! problems found")

    for i, part in enumerate(base_secs):
        bb = part.bounding_box()
        write_stl(part.translate([-bb[0], -bb[1], FLOOR_T]),
                  f"ABD-Case-Base-{i + 1}.stl")
    for i, part in enumerate(lid_secs):
        bb = part.bounding_box()
        write_stl(part.rotate([180, 0, 0]).translate(
            [-bb[0], bb[4], (WALL_H + LID_T)]), f"ABD-Case-Lid-{i + 1}.stl")
