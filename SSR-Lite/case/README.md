# SSR-Lite 3D-printed case

Two-part case for the SSR-Lite 45 x 60 mm PCB. External size **61.5 x 80 x 37 mm**.
Generated from the board's gerber/NC-drill data and the Altium `Board2.obj` 3D
export by `ssr_lite_case.py`.

## Files

| File | What |
|---|---|
| `SSR-Lite-Case-Base.stl` | Tray: PCB bosses, corner posts, all cable ports |
| `SSR-Lite-Case-Lid.stl`  | Lid: inner lip, vent slots, 4 screw holes (already flipped for printing) |
| `ssr_lite_case.py`       | Parametric generator (`pip install manifold3d numpy`, then `python3 ssr_lite_case.py`) |

## Design notes

- The three 9.5 mm terminal blocks **overhang the PCB edge by up to 6.2 mm**, so
  the walls stand 7.5 mm off the board on those sides; each wire position gets
  its own 8 x 9 mm port aligned to the drill-file pin coordinates (3 per side on
  left / top / bottom). Ports sit 3–12 mm above the board face, matching the
  wire-entry cavity of the blocks.
- Right wall has a 14 x 10.5 mm port for the J1 control plug (2 mm 4P header).
- PCB sits on four Ø10 bosses, **9 mm tall** — the bottom-side fuse clips hang
  ~7.5 mm below the board, and the terminal pins ~4.5 mm, so don't reduce
  `STANDOFF_H` below 8.5.
- **Every threaded point takes an M3 knurled brass heat-set insert** (4 mm
  long, 4.5 mm OD): four in the PCB bosses, four in the lid corner posts. The
  pockets are Ø4.2 x 5.5 mm deep with M3 screw clearance below; the generator
  verifies each keeps a solid ≥1.8 mm plastic collar. Tune `INSERT_HOLE_D`
  (4.0–4.3) to your inserts. Bosses/posts were fattened to Ø10 / R8.5 to wrap
  the brass.
- Lid screws into quarter-round posts in the cavity corners; a 2.5 mm inner lip
  registers it. Vent slots (2.5 mm wide) sit over the triac/fuse area for a bit
  of convection cooling — set `VENTS = False` for a solid lid.
- Wall-mount ears (18 mm wide, 4 mm thick, Ø4.5 holes for M4 or #8 screws)
  stick out from the two short ends at floor level, clear of the wire ports
  above them. Set `MOUNT_EARS = False` for a flush case.

- The Chill Division logo is debossed 0.4 mm into the lid top (34 mm wide, centred above the vent slots; `LOGO_W` / `LOGO_DEPTH` / `LOGO_XY` to adjust, needs `chill_logo.png` beside the script). The lid prints face-down so it comes out crisp.

## Hardware

- 8x M3 knurled brass heat-set inserts (4 mm, 4.5 mm OD) — 4 bosses + 4 posts
- 4x M3 machine screws — PCB to bosses (M3 x 6–8)
- 4x M3 machine screws — lid to corner posts (M3 x 10–12)
- 2x M4 (or #8) screws + anchors to suit the wall, through the mounting ears

## Printing

- **Base**: prints as oriented, no supports (wire ports are short bridges).
- **Lid**: STL is already flipped — outer face on the bed, lip up, no supports.
- PETG or ASA recommended (mains device, some heat from the triacs); PLA is fine
  for bench use. 3+ perimeters, ~25 % infill.
- Melt the eight inserts in with a soldering iron before assembly (brass flush
  or a hair proud of the boss/post face).
- Tolerances assume a reasonably tuned printer; the lid lip has 0.3 mm
  clearance (`LIP_CLR`), wire ports have >= 1.5 mm slack to the terminal
  positions.

## Assembly

1. Heat-set all eight brass inserts (4 bosses, 4 corner posts).
2. Drop the PCB onto the bosses (terminal blocks toward the tall walls) and fix
   with four M3 screws through the board's mounting holes.
3. Wire the terminals through the side ports, plug the control cable through the
   right-hand port.
4. Fit the lid and drive four M3 screws into the corner-post inserts.
