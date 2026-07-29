#!/usr/bin/env python3
"""Preview of the enclosed, sectioned ABD v2.1 case (true assembly coords)."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import abd_case as C

HERE = os.path.dirname(os.path.abspath(__file__))
SEC_COLS = [(0.35, 0.62, 0.85), (0.45, 0.75, 0.55), (0.85, 0.6, 0.35)]


def tris(man):
    m = man.to_mesh()
    v = np.asarray(m.vert_properties, dtype=float)[:, :3]
    t = np.asarray(m.tri_verts)
    tv = v[t]
    n = np.cross(tv[:, 1] - tv[:, 0], tv[:, 2] - tv[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    return tv, np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)


def add(ax, man, rgb, light=(0.3, -0.5, 0.8), alpha=1.0):
    tv, n = tris(man)
    lv = np.array(light) / np.linalg.norm(light)
    lum = 0.5 + 0.5 * np.clip(n @ lv, 0, 1)
    cols = np.concatenate([np.clip(lum[:, None] * np.array(rgb), 0, 1),
                           np.full((len(tv), 1), alpha)], 1)
    ax.add_collection3d(Poly3DCollection(tv, facecolors=cols, edgecolors="none"))


def frame(ax, mans, elev=32, azim=-62, zoom=None):
    allv = np.concatenate([np.asarray(m.to_mesh().vert_properties)[:, :3]
                           for m in mans])
    lo, hi = allv.min(0), allv.max(0)
    c = (lo + hi) / 2
    r = (hi - lo).max() / 2 if zoom is None else zoom
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_proj_type("ortho")
    ax.axis("off")


fb, fl = C.build_full_base(), C.build_full_lid()
bsec = [C.section_base(fb, i) for i in range(3)]
lsec = [C.section_lid(fl, i) for i in range(3)]
# heatsink mock (representative), board mock
hs = C.box(C.HS_X0 + 2, C.HS_X1 - 2, -C.HS_FRONT_OPEN + 3, C.HS_BACK_Y - 3,
           C.BOARD_TOP, C.WALL_H + 8)
pcb = C.box(0, C.PCB_W, 0, C.PCB_L, C.STANDOFF_H, C.BOARD_TOP)

fig = plt.figure(figsize=(15, 9), dpi=100)

ax = fig.add_subplot(2, 2, 1, projection="3d")
for i, s in enumerate(bsec):
    add(ax, s, SEC_COLS[i])
add(ax, pcb, (0.2, 0.5, 0.3), alpha=0.55)
add(ax, hs, (0.5, 0.5, 0.55), alpha=0.9)
frame(ax, bsec, elev=42, azim=-70)
ax.set_title("3 base sections (colours) + PCB + heatsink in the open channel")

ax = fig.add_subplot(2, 2, 2, projection="3d")
for i, s in enumerate(bsec):
    add(ax, s, SEC_COLS[i])
for i, s in enumerate(lsec):
    add(ax, s.translate([0, 0, 26]), SEC_COLS[i], alpha=0.85)
frame(ax, bsec, elev=30, azim=-62)
ax.set_title("lids lifted (external back flanges bolt lid to base)")

# close-up of one seam pulled apart: lap tongues + insert bosses
ax = fig.add_subplot(2, 2, 3, projection="3d")
add(ax, bsec[0], SEC_COLS[0])
add(ax, bsec[1].translate([26, 0, 0]), SEC_COLS[1], alpha=0.9)
frame(ax, [bsec[0]], elev=28, azim=-55, zoom=52)
ax.set_xlim(C.SEAMS[0] - 45, C.SEAMS[0] + 60)
ax.set_title("base seam apart: lap tongues w/ insert bosses -> csk M3 from below")

# close-up of an external lid flange: base flange + lid flange lifted
ax = fig.add_subplot(2, 2, 4, projection="3d")
fx = C.lid_flanges()[1][0]      # a mid flange on section 0
add(ax, bsec[0], SEC_COLS[0])
add(ax, lsec[0].translate([0, 0, 14]), SEC_COLS[0], alpha=0.85)
frame(ax, [bsec[0]], elev=18, azim=-52, zoom=34)
ax.set_xlim(fx - 34, fx + 34)
ax.set_ylim(C.OY1 - 40, C.OY1 + 28)
ax.set_title("external lid flange: screw down into base insert (no lip)")

fig.tight_layout()
fig.savefig(f"{HERE}/case_preview.png", bbox_inches="tight")
print("wrote case_preview.png")
