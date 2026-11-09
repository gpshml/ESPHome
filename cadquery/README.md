# CadQuery STEP assembly — cabin bed

Parametric solids from locked `DIM` (`cabin_bed_model.js` / `LOCKED-IN-SPEC.md`), per `COMPONENT-3D.md`.

## Setup

```bash
pip install -r cadquery/requirements.txt
```

## Build

```bash
python3 cadquery/build_assembly.py
# → cadquery/out/cabin_bed_assembly.step
# → cadquery/out/cabin_bed_assembly.stl   (for Blender)
```

Named parts follow `PARTS-LIST.md` zones: front fascia studs/track/PB, foot extension, rear/head shelves, timber lift, bed/drawers (context).

C-stud / U-track use real-ish hollow profiles (`STUD_W` 70, flange 45, track web 72, leg 25, gauge 0.7 mm).

## Review on GitHub (no install)

GitHub renders the mesh in the browser when you open the file:

- Coloured: [`cabin_bed_assembly.obj`](out/cabin_bed_assembly.obj) (keep [`.mtl`](out/cabin_bed_assembly.mtl) alongside)
- Fallback: [`cabin_bed_assembly.stl`](out/cabin_bed_assembly.stl)

OBJ/STL are exported in **metres** (~2.3 × 1.2 × 2.4). STEP stays in **mm** for CAD.

STEP does not preview on GitHub. **Marker-FOOT** left, **Marker-HEAD** right (review-only); narrow pier at the foot.

## Review in Blender

OBJ/STL are **metres**, **Y-up**, and **X-mirrored for front view** so that when you look at the front opening from the room, **FOOT is left / HEAD is right** (locked elevation). STEP stays in construction coordinates (foot at X=0).

1. Download `cabin_bed_assembly.obj` **and** `.mtl` together  
2. **File → Import → Wavefront (.obj)** — defaults OK; do **not** scale by 0.001  
3. Clip End ≥ 100 m if anything still clips  
4. Face the **front opening** — **Marker-FOOT** on the left, **Marker-HEAD** on the right; narrow pier at the foot  

Opening reveals = 12.5mm PB, drywall screws to studs/track, stop bead. See [`RENDER.md`](RENDER.md) and [`../BUILD-SEQUENCE.md`](../BUILD-SEQUENCE.md).

## Coords

- **STEP / construction CAD:** origin back-left-floor; X foot→head (foot at 0), Y back→front, Z up (mm).  
- **OBJ/STL viewers:** same model with X mirrored so a room-side front view matches the locked front aspect (otherwise +X maps to screen-left and the plan looks flipped). Do **not** flip the STEP/room SoT to chase a viewer.
