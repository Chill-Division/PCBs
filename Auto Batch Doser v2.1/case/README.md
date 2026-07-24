# Auto Batch Doser v2.1 — enclosed case (sectioned)

A **second, fully-enclosed** case for the 488.95 x 74.14 mm SystemV1 board, as
an alternative to the open standoff mount in `../mount`. Enclosed on all sides
**except a full-width open channel** over the stepper-driver row, where the
finned heatsink protrudes up and overhangs the front long edge.

The board is 489 mm long, so base and lid each **tile into 3 sections** that
bolt together — everything fits the P1S 256 mm bed. Base seams (163 / 326)
and lid seams (100 / 280) are **staggered like brickwork**, so each lid
section bridges a base seam and bolts to both base sections: the lid IS the
top splice. No separate splice parts.

## Files

| File | What |
|---|---|
| `ABD-Case-Base-1/2/3.stl` | Base sections, left→right |
| `ABD-Case-Lid-1/2/3.stl`  | Lid sections (already flipped for printing) |
| `abd_case.py`             | Parametric generator (`pip install manifold3d numpy`) |
| `render_preview.py`       | Rebuilds the preview |

## ⚠ Confirm these before printing

The heatsink and clearance numbers are **placeholders** — I don't have the
physical heatsink dimensions. Check them against the hardware and re-run:

| Param | Now | Means |
|---|---|---|
| `HS_X0`, `HS_X1` | 36, 474 | channel start/end along the board |
| `HS_BACK_Y` | 34 | how far the channel reaches back from the front edge |
| `HS_FRONT_OPEN` | 16 | how far the channel opens **past** the front edge for the fin overhang |
| `LID_CLEAR` | 18 | interior height over the tallest **enclosed** part (back half) |
| `STANDOFF_H` | 4.5 | gap under the board for trimmed lead tips |

Note: opening the whole front band means the tall front-edge electrolytics
(y≈12–20) sit in the open channel too — unavoidable with a front-opening
scheme, and the price of not enclosing the heatsink.

## Design notes

- Board rests on a full-length ledge along the **back** edge, ledges on the
  **front** edge at the two ends (clear of the channel), and four Ø10 M3
  **insert** bosses at the mounting holes.
- Front (heatsink-side) wall is cut to board level over the channel; the outer
  front wall is fully opened over `HS_FRONT_OPEN` for the overhang.
- **Seams, bottom**: each base seam has two 2 mm lap tongues (at TH-free
  spots verified from the drill data) that sit in 1 mm recesses of the next
  floor, each topped by a Ø10 boss holding a downward-facing M3 insert. Two
  countersunk M3 x 8 driven up from the outside floor face clamp the joint -
  heads sit flush, so wall-mounting is unaffected.
- **Seams, top**: handled by the staggered lid - each lid section screws into
  external flanges on both sides of the base seam it spans.
- Every external flange has a 45° gusset underneath: **no supports anywhere**.
- **Lid** is an L-section cover over the back ~40 mm of the board. It bolts on
  through **external flanges on the back edge** (no internal lip or posts):
  each base flange is a block hanging just below the wall top with a vertical
  M3 insert, and the matching lid flange sits on top with a counterbored screw
  driven straight down from outside — 8 flanges along the length, placed so
  no flange lands on a seam. The front stays open as the channel.
- **Cable exits**: motor wires leave through the open channel; F1 / P1 / J5
  power & control exit ports in the left end wall. (S1 programming header is
  enclosed — add a lid hole if you want it accessible.)
- Wall-mount ears (Ø4.5, M4) on the back edge, two per section.

## Hardware (per case)

- 4x M3 inserts + screws — PCB bosses (board hold-down)
- 8x M3 inserts + screws (M3 x 8) — external back-edge lid flanges
- 4x M3 inserts + 4x M3 x 8 **countersunk** screws — floor lap joints (2 per seam)
- 6x M4 wall screws — back-edge ears (2 per section)
- Melt all inserts in with a soldering iron before assembly.

## Printing

- All sections print flat, no supports. PETG for the warm environment,
  4 perimeters.
- Base sections ≈ 168–184 mm long × ~102 mm (with ears) — fit a 256 bed.
- Lid sections print inner-face-up (STL already flipped).

## Assembly

1. Heat-set all inserts: PCB bosses, lid flanges (from the wall top), and the
   four lap bosses (from the tongue underside, part flipped).
2. Lay the three base sections end-to-end: each pair of lap tongues drops into
   the next section's floor recesses; drive the two countersunk M3s up from
   the outside floor face at each seam.
3. Drop the PCB in (heatsink toward the open channel), screw it to the four
   boss inserts.
4. Wire everything; motor leads out the channel, power/control out the left end.
5. Fit the three lid sections (their seams sit mid-way along the base
   sections), screw down through the back-edge flanges — each lid section
   stitches across a base seam as it goes on.
