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

## Review in Blender (no STEP importer needed)

Blender does not open STEP natively. Use the STL mesh:

1. Download `cadquery/out/cabin_bed_assembly.stl`
2. Blender → **File → Import → STL**
3. Scene units are **millimetres**. Either:
   - **Scene Properties → Units → Unit Scale 0.001** (and Length = Millimetres), or
   - Scale the imported object by **0.001** if you keep Blender’s default metres
4. Press **Home** / **Frame All** — envelope is ~2.3 m × 1.2 m × 2.4 m

For editable solids / named STEP parts, FreeCAD is the usual free download; Blender is enough for a visual check.

## Coords

Origin back-left-floor; X foot→head, Y back→front, Z up (mm).
