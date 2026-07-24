#!/usr/bin/env python3
"""Render preview images of the Grove 0-10V case STLs + alignment drawing."""
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
OX0, OX1, OY0, OY1 = -3.7, 61.231, -9.5, 65.126
CAVITY_H, LID_T, EAR_OUT = 22.5, 2.5, 12.0
BASE_SHIFT_Y = -OY0 + EAR_OUT


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


bt, bn = read_stl(f"{HERE}/Grove10V-Case-Base.stl")
lt, ln = read_stl(f"{HERE}/Grove10V-Case-Lid.stl")

# lid back to assembly pose above the base STL, hovering 20 mm up
lr = lt.copy()
lr[:, :, 1] = (OY1 + BASE_SHIFT_Y) - lr[:, :, 1]
lr[:, :, 2] = (CAVITY_H + LID_T + 2.5) - lr[:, :, 2] + 20.0
lnr = ln.copy() * np.array([1, -1, -1])
lr = lr[:, ::-1, :]

fig = plt.figure(figsize=(14, 10), dpi=110)
ax = fig.add_subplot(2, 2, 1, projection="3d")
view(ax, [(bt, bn, (0.35, 0.62, 0.85)), (lr, lnr, (0.9, 0.55, 0.25))])
ax.set_title("assembly (lid lifted)")
ax = fig.add_subplot(2, 2, 2, projection="3d")
view(ax, [(bt, bn, (0.35, 0.62, 0.85))], elev=35, azim=-125)
ax.set_title("base — cable ports (RJ11 + Grove, left wall)")
ax = fig.add_subplot(2, 2, 3, projection="3d")
view(ax, [(bt, bn, (0.35, 0.62, 0.85))], elev=55, azim=-55)
ax.set_title("base — board cradle")
ax = fig.add_subplot(2, 2, 4, projection="3d")
view(ax, [(lr, lnr, (0.9, 0.55, 0.25))], elev=32, azim=-55)
ax.set_title("lid — switch windows + hold-down pillars")
fig.tight_layout()
fig.savefig(f"{HERE}/case_preview.png", bbox_inches="tight")
print("wrote case_preview.png")

# ---------------- 2D alignment drawing (board coordinates) -----------------
fig2, ax2 = plt.subplots(figsize=(8, 9), dpi=110)


def rect(x0, y0, w, h, **kw):
    ax2.add_patch(Rectangle((x0, y0), w, h, fill=False, **kw))


rect(OX0, OY0, OX1 - OX0, OY1 - OY0, ec="k", lw=2)
rect(-1.2, -7, 59.93, 69.63, ec="k", lw=1, ls=":")
rect(0, 0, 57.531, 55.626, ec="tab:green", lw=1.5)
for x0, y0, w, h, lab in [(0.9, 12.2, 13.0, 13.0, "RJ11"),
                          (0.3, 35.5, 13.0, 13.0, "CN1"),
                          (24.4, 2.6, 9.0, 9.6, "SW3")]:
    rect(x0, y0, w, h, ec="tab:red", lw=1.2)
    ax2.text(x0 + w / 2, y0 + h / 2, lab, ha="center", va="center",
             color="tab:red", fontsize=8)
for x, y in [(46.387, 5.378), (46.292, 14.014), (46.291, 22.816),
             (46.164, 31.413), (45.752, 40.469), (45.593, 49.276)]:
    rect(x - 5.75, y - 2.75, 11.5, 5.5, ec="tab:purple", ls="--")
ax2.plot([-3.7, -1.2], [18.68, 18.68], lw=8, c="tab:orange")
ax2.plot([-3.7, -1.2], [42.04, 42.04], lw=8, c="tab:orange")
for x, y in [(1.8, -4), (55.73, -4), (1.8, 59.63), (55.73, 59.63)]:
    ax2.add_patch(Circle((x, y), 1.3, fill=False, ec="purple"))
for x, y in [(2.5, 3.0), (55.0, 3.0), (2.5, 52.5), (55.0, 52.5)]:
    ax2.add_patch(Circle((x, y), 3.0, fill=False, ec="tab:blue"))
ax2.set_xlim(-8, 66)
ax2.set_ylim(-14, 70)
ax2.set_aspect("equal")
ax2.set_title("plan: walls (black), PCB (green), connectors (red),\n"
              "ports (orange), lid pillars (blue), lid screws (purple),\n"
              "switch windows (dashed)")
fig2.savefig(f"{HERE}/case_alignment.png", bbox_inches="tight")
print("wrote case_alignment.png")
