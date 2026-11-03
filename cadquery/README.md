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

STEP does not preview on GitHub. In the GitHub 3D view, **Marker-HEAD** = left, **Marker-FOOT** = right when looking in from the room.

## Review in Blender

The “bizarre slicing” across the model was from importing **mm as Blender units** (~2.4 km tall) — the viewport **clip end** cuts through it. These meshes are now in **metres**.

1. Download `cabin_bed_assembly.obj` **and** `cabin_bed_assembly.mtl` together
2. **File → Import → Wavefront (.obj)**
   - Forward = **Y Forward**, Up = **Z Up**
   - **Do not** scale by 0.001 again
3. Scene units: **Metres** (Blender default is fine) — envelope ~2.3 m wide
4. If anything still looks sliced: **Sidebar (N) → View → Clip End** ≥ `100` m
5. **View → Viewpoint → Front** — **Marker-HEAD** left, **Marker-FOOT** right

Materials: painted PB = autumnal olive; studs/track/noggins = galvanised; decks/ledgers/lift = timber.

## Coords

Origin back-left-floor; X **head→foot**, Y back→front, Z up (mm). HEAD left, FOOT right.
