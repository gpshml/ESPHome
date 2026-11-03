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

Blender does not open STEP natively. Prefer the **coloured OBJ**:

1. Download `cadquery/out/cabin_bed_assembly.obj` **and** `cabin_bed_assembly.mtl` (same folder)
2. Blender → **File → Import → Wavefront (.obj)**
   - Transform: **Forward = Y Forward**, **Up = Z Up** (matches this model)
3. Scene units are **millimetres**. Either:
   - **Scene Properties → Units → Unit Scale 0.001** (Length = Millimetres), or
   - Scale by **0.001** if you keep Blender’s default metres
4. **View → Viewpoint → Front** — you should be looking into the alcove from the room
5. **Marker-FOOT** (olive block) must be on the **left**; **Marker-HEAD** on the **right**. If FOOT is on the right, you are viewing from the back wall — flip to Front / View opposite.

Materials: painted PB = autumnal olive; studs/track = galvanised; decks/ledgers/lift = timber.

`cabin_bed_assembly.stl` is an uncoloured fallback mesh.

## Coords

Origin back-left-floor; X foot→head, Y back→front, Z up (mm).
