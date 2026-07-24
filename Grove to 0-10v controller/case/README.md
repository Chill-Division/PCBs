# Grove to 0-10V controller 3D-printed case

Two-part case for the 57.5 x 55.6 mm MainBoard. External size
**64.9 x 74.6 x 27.5 mm** plus wall-mount ears. Generated from the fab outputs
(gerbers, NC drill, pick & place, silkscreen) by `grove_case.py`.

## Files

| File | What |
|---|---|
| `Grove10V-Case-Base.stl` | Tray: board cradle, corner posts, cable ports, ears |
| `Grove10V-Case-Lid.stl`  | Lid: switch windows, hold-down pillars, 4 screw holes (already flipped for printing) |
| `grove_case.py`          | Parametric generator (`pip install manifold3d numpy`, then `python3 grove_case.py`) |
| `render_preview.py`      | Regenerates the preview PNGs from the STLs |

## Design notes

- **This board has no mounting holes** — the two 3 mm drills are the RJ11
  jack's snap-in posts. The case therefore works as a drop-in cradle: the board
  rests on a 4 mm perimeter shelf, side ribs and end curbs register it to
  ±0.35 mm, and four Ø6 pillars on the lid clamp it down when the lid is
  screwed on (0.2 mm nominal preload gap, `PILLAR_GAP`).
- Both cable ports are on the **left wall**, which sits close to the board so
  plugs reach and latch: a 14 x 13.8 mm opening for the RJ11 plug and a
  13 x 9 mm opening for the Grove HY2.0-4P plug.
- The lid has access windows over the six MST23D19G2 slide switches and the
  SW3 DIP switch so you can set them with a small screwdriver without opening
  the case (`SW_WINDOWS = False` for a solid lid). They double as vents.
- Lid screws into quarter-round posts in the cavity corners (the top/bottom
  walls stand 7 mm off the board to make room for them; left/right walls hug
  the board at 1.2 mm).
- Wall-mount ears (Ø4.5 holes for M4 or #8 screws) on the two port-free ends;
  `MOUNT_EARS = False` removes them.
- PCB thickness assumed 1.6 mm (`PCB_T`); RJ11 height assumed 13.5 mm — if
  your jack is taller than ~14.5 mm, bump `CAVITY_H`.

- The Chill Division logo is debossed 0.4 mm into the lid top (32 mm wide, centred left of the switch windows; `LOGO_W` / `LOGO_DEPTH` / `LOGO_XY` to adjust, needs `chill_logo.png` beside the script). The lid prints face-down so it comes out crisp.

## Hardware

- 4x M3 knurled brass heat-set inserts (4 mm, 4.5 mm OD) — lid corner posts
- 4x M3 machine screws (M3 x 10–12) — lid to corner posts (Ø2.6 pilots; use
  `PILOT_D = 4.0` for M3 heat-set inserts)
- 2x M4 (or #8) screws + anchors to suit the wall, through the mounting ears

## Printing

- **Base**: prints as oriented, no supports (ports are short bridges).
- **Lid**: STL is already flipped — outer face on the bed, pillars up, no
  supports needed (pillars are plain Ø6 columns).
- PETG or PLA both fine (low-voltage board, negligible heat).
  3 perimeters, ~20 % infill.

## Assembly

1. Drop the board into the cradle, RJ11/Grove sockets toward the ported wall;
   the ribs center it automatically.
2. Fit the lid (the cut-away lip section goes over the RJ11) and drive the
   four M3 screws into the corner posts — the pillars clamp the board.
3. Cables plug straight through the wall ports and latch into the sockets.
