#!/usr/bin/env python3
"""Render preview images of the generated case STLs + a 2D alignment drawing."""
import os
import struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

SCRATCH = os.path.dirname(os.path.abspath(__file__))


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
    pc = Poly3DCollection(tris, facecolors=cols, edgecolors="none")
    ax.add_collection3d(pc)


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


bt, bn = read_stl(f"{SCRATCH}/SSR-Lite-Case-Base.stl")
lt, ln = read_stl(f"{SCRATCH}/SSR-Lite-Case-Lid.stl")

# lid back into assembly orientation, hovering 22 mm above the base
lr = lt.copy()
lr[:, :, 1] = 92.0 - lr[:, :, 1]          # un-mirror; +12 = base ear offset
lr[:, :, 2] = (34.5 + 2.5) - lr[:, :, 2] + 22.0
lnr = ln.copy() * np.array([1, -1, -1])
lr = lr[:, ::-1, :]                       # restore winding

fig = plt.figure(figsize=(14, 10), dpi=110)
ax = fig.add_subplot(2, 2, 1, projection="3d")
view(ax, [(bt, bn, (0.35, 0.62, 0.85)), (lr, lnr, (0.9, 0.55, 0.25))])
ax.set_title("assembly (lid lifted)")
ax = fig.add_subplot(2, 2, 2, projection="3d")
view(ax, [(bt, bn, (0.35, 0.62, 0.85))], elev=35, azim=-125)
ax.set_title("base — from top/left")
ax = fig.add_subplot(2, 2, 3, projection="3d")
view(ax, [(bt, bn, (0.35, 0.62, 0.85))], elev=18, azim=55)
ax.set_title("base — right side (control cable port)")
ax = fig.add_subplot(2, 2, 4, projection="3d")
view(ax, [(lr, lnr, (0.9, 0.55, 0.25))], elev=32, azim=-55)
ax.set_title("lid (top)")
fig.tight_layout()
fig.savefig(f"{SCRATCH}/case_preview.png", bbox_inches="tight")
print("wrote case_preview.png")

# ---------------- 2D alignment drawing (board coords, base shifted +10) ----
fig2, ax2 = plt.subplots(figsize=(8, 10), dpi=110)
S = 10.0  # STL offset: board (0,0) sits at (10,10)


def rect(x0, y0, w, h, **kw):
    ax2.add_patch(Rectangle((x0 + S, y0 + S), w, h, fill=False, **kw))


rect(-10, -10, 61.5, 80, ec="k", lw=2)                    # outer wall
rect(-7.5, -7.5, 56.5, 75, ec="k", lw=1, ls=":")          # inner wall
rect(0, 0, 45, 60, ec="tab:green", lw=1.5)                # PCB
for x, y in [(2.667, 2.667), (41.91, 2.667), (2.667, 56.896), (41.91, 56.896)]:
    ax2.add_patch(Circle((x + S, y + S), 4, fill=False, ec="tab:blue"))
    ax2.add_patch(Circle((x + S, y + S), 1.5, fill=False, ec="tab:blue"))
for x0, y0, w, h, lab in [(-5.98, 16.42, 16.5, 30.57, "T1"),
                          (6.38, -1.41, 30.58, 16.5, "T2"),
                          (4.96, 49.69, 30.58, 16.5, "T3"),
                          (38.97, 24.2, 5.1, 10.5, "J1")]:
    rect(x0, y0, w, h, ec="tab:red", lw=1.2)
    ax2.text(x0 + w / 2 + S, y0 + h / 2 + S, lab, ha="center", va="center",
             color="tab:red")
for y in [20.61, 30.11, 39.61]:                            # wire ports left
    ax2.plot([S - 10, S - 7.5], [y + S, y + S], lw=6, c="tab:orange")
for x in [11.94, 21.44, 30.94]:
    ax2.plot([x + S, x + S], [S - 10, S - 7.5], lw=6, c="tab:orange")
for x in [11.96, 21.46, 30.96]:
    ax2.plot([x + S, x + S], [S + 67.5, S + 70], lw=6, c="tab:orange")
ax2.plot([S + 49, S + 51.5], [29.45 + S, 29.45 + S], lw=10, c="tab:orange")
for x, y in [(-4.5, -4.5), (46, -4.5), (-4.5, 64.5), (46, 64.5)]:
    ax2.add_patch(Circle((x + S, y + S), 1.3, fill=False, ec="purple"))
ax2.set_xlim(-3, 65)
ax2.set_ylim(-3, 83)
ax2.set_aspect("equal")
ax2.set_title("plan: walls (black), PCB (green), bosses (blue),\n"
              "connectors (red), ports (orange), lid screws (purple)")
fig2.savefig(f"{SCRATCH}/case_alignment.png", bbox_inches="tight")
print("wrote case_alignment.png")
