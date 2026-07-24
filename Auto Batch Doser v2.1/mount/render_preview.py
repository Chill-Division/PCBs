#!/usr/bin/env python3
"""Render preview images of the ABD v2.1 mount STLs + alignment drawing."""
import os
import struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
# must match grove_case.py


def read_stl(path):
    with open(path, "rb") as f:
        f.seek(80)
        n = struct.unpack("<I", f.read(4))[0]
        rec = np.frombuffer(f.read(), dtype=np.dtype(
            [("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n)
    return rec["v"].astype(float), rec["n"].astype(float)


def shade(ax, tris, normals, base_rgb, light=(0.4, -0.6, 0.8)):
    lv = np.array(light) / np.linalg.norm(light)
    lum = 0.45 + 0.55 * np.clip(normals @ lv, 0, 1)
    cols = np.clip(lum[:, None] * np.array(base_rgb)[None, :], 0, 1)
    ax.add_collection3d(Poly3DCollection(tris, facecolors=cols,
                                         edgecolors="none"))


def view(ax, parts, elev=28, azim=-55):
    allv = np.concatenate([t.reshape(-1, 3) for t, _, _ in parts])
    lo, hi = allv.min(0), allv.max(0)
    c, r = (lo + hi) / 2, (hi - lo).max() / 2
    for tris, normals, rgb in parts:
        shade(ax, tris, normals, rgb)
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_proj_type("ortho")
    ax.axis("off")



lt, ln = read_stl(f"{HERE}/ABD-Mount-Left.stl")
rt, rn = read_stl(f"{HERE}/ABD-Mount-Right.stl")
mt, mn = read_stl(f"{HERE}/ABD-Mount-Mid.stl")
BLUE, ORANGE = (0.35, 0.62, 0.85), (0.9, 0.55, 0.25)

fig = plt.figure(figsize=(14, 10), dpi=110)
ax = fig.add_subplot(2, 2, 1, projection="3d")
rt_off = rt + np.array([55.0, 0, 0])
view(ax, [(lt, ln, BLUE), (rt_off, rn, ORANGE)], elev=35, azim=-60)
ax.set_title("both brackets (left blue, right orange - not to true spacing)")
ax = fig.add_subplot(2, 2, 2, projection="3d")
view(ax, [(lt, ln, BLUE)], elev=40, azim=-115)
ax.set_title("left bracket - offset inboard boss")
ax = fig.add_subplot(2, 2, 3, projection="3d")
view(ax, [(rt, rn, ORANGE)], elev=40, azim=-65)
ax.set_title("right bracket")
ax = fig.add_subplot(2, 2, 4, projection="3d")
view(ax, [(mt, mn, (0.45, 0.75, 0.45))], elev=30, azim=-140)
ax.set_title("mid clip - top hooks, bottom ledge (heatsink side), long ear")
fig.tight_layout()
fig.savefig(f"{HERE}/mount_preview.png", bbox_inches="tight")
print("wrote mount_preview.png")

fig2, ax2 = plt.subplots(figsize=(12, 3.2), dpi=110)
def rect(x0, y0, w, h, **kw):
    ax2.add_patch(Rectangle((x0, y0), w, h, fill=False, **kw))
rect(0, 0, 488.95, 74.143, ec="tab:green", lw=1.5)
rect(16, -15, 478, 45, ec="dimgray", ls=":", lw=1)
ax2.text(150, -11, "heatsink fins overhang bottom edge", color="dimgray",
         fontsize=7)
rect(234, -7, 34, 86.5, ec="tab:olive")
rect(242, -31, 18, 25, ec="tab:olive")
rect(242, 77.5, 18, 14, ec="tab:olive")
for cy in (-25.0, 86.0):
    ax2.add_patch(Circle((251, cy), 2.25, fill=False, ec="purple"))
ax2.text(251, 40, "mid clip", ha="center", color="tab:olive", fontsize=8)
rect(-0.5, 7.0, 29.5, 69.3, ec="k")
rect(477.0, 8.15, 12.5, 68.0, ec="k")
for cx, cy in [(-7, 13.183), (-7, 70.282), (496, 14.427), (496, 70.282)]:
    rect(cx - 7, cy - 9, 14, 18, ec="k")
    ax2.add_patch(Circle((cx, cy), 2.25, fill=False, ec="purple"))
for x, y in [(3.302, 70.282), (23.724, 13.183), (484.48, 14.427),
             (484.48, 70.282)]:
    ax2.add_patch(Circle((x, y), 4, fill=False, ec="tab:blue"))
    ax2.add_patch(Circle((x, y), 1.5, fill=False, ec="tab:blue"))
ax2.set_xlim(-25, 515)
ax2.set_ylim(-34, 94)
ax2.set_aspect("equal")
ax2.set_title("plan: PCB (green), brackets/ears (black), M3 bosses (blue), "
              "wall screws (purple)")
fig2.savefig(f"{HERE}/mount_alignment.png", bbox_inches="tight")
print("wrote mount_alignment.png")
