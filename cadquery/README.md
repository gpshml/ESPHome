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

OBJ/STL are **metres** and **Y-up** (so the model stands upright with Blender defaults — not on its side).

1. Download `cabin_bed_assembly.obj` **and** `.mtl` together  
2. **File → Import → Wavefront (.obj)** — defaults OK; do **not** scale by 0.001  
3. Clip End ≥ 100 m if anything still clips  
4. Looking into the alcove: **FOOT left**, **HEAD right**; narrow pier at the foot (left) — must match `top_plan_FINAL.png`  

Opening reveals = 12.5mm PB, drywall screws to studs/track, stop bead. See [`RENDER.md`](RENDER.md) and [`../BUILD-SEQUENCE.md`](../BUILD-SEQUENCE.md).

## Coords

CAD: origin back-left-floor; X foot→head, Y back→front, Z up (mm). OBJ export converts to Blender Y-up. If the preview looks mirrored, fix the camera/view — do **not** invert the room SoT.
