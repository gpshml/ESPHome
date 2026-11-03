# Cabin bed — component 3D model

## Verdict

**Yes — a component assembly 3D model is in this repo now.**

| Deliverable | Status |
|-------------|--------|
| Interactive component assembly (studs, track, ledgers, PB, decks, lift, bed, drawers) | `cabin_bed_model.js` + `cabin-bed-3d-viewer.html` |
| Static captures from that model | `3d_view_FINAL.png`, `front_elevation_FINAL.png`, `top_plan_FINAL.png`, `side_elevation_FINAL.png` |
| CadQuery STEP assembly (named parts, C-stud / U-track profiles) | `cadquery/` → `cadquery/out/cabin_bed_assembly.step` |
| Coloured OBJ (GitHub 3D preview + Blender) | `cadquery/out/cabin_bed_assembly.obj` + `.mtl` |
| STL mesh fallback (also GitHub-previewable) | `cadquery/out/cabin_bed_assembly.stl` |
| Serve locally | `python3 -m http.server 8877` → open `/cabin-bed-3d-viewer.html` |

Open the viewer, use **3D / Front / Top / Side**, and **Toggle front** to inspect framing behind the fascia.

## What this model is (and is not)

**It is:** a millimetre-accurate *assembly* of the locked volumes — each major buy/build part as a coloured box so you can see how studs, track, PB faces, wall-supported shelves, the 170 mm lift, and the owned bed sit together.

**Three.js is not:**

- True C-stud / U-track *profile* geometry (flanges are thickened for readability)
- Screw schedules, bog/jointing, or curtain track detail
- Structural analysis or cut-list nesting for CNC

**CadQuery STEP (`cadquery/out/cabin_bed_assembly.step`) is:** a parametric assembly with hollow 70 mm C-stud / 72 mm U-track profiles and named parts aligned to `PARTS-LIST.md`. Still not IFC/Revit BIM, screw schedules, or CNC nesting.

```bash
pip install -r cadquery/requirements.txt
python3 cadquery/build_assembly.py
```

## Source of truth

Same as `LOCKED-IN-SPEC.md`: origin back-left-floor; X **head→foot**, Y back→front, Z up.

```bash
# After editing cabin_bed_model.js:
node sync_viewer.mjs
python3 -m http.server 8877
node capture_static_views.mjs
python3 create_final_report.py
```

## Brief for a higher-fidelity CAD/BIM AI (if needed)

Copy this into another agent or CAD assistant:

> **Project:** UK metal-stud + plasterboard alcove cabin/daybed, Cheltenham GL53.  
> **Alcove:** 2344 × 1242 × 2380 mm (W×D×H). Origin = back-left-floor. X **head→foot**, Y back→front, Z up.  
> **Head (left):** 326 mm fascia; ledgers on existing head wall; PB on exposed faces; studs inside shelf.  
> **Foot (right):** existing 98 mm wall + 237 mm front extension (full height); PB flush to studs.  
> **Rear:** 210 mm shelf, top 1200 mm; ledger on existing back wall; PB front fascia; shelf width = INTERNAL_W − HEAD_W (stops before foot).  
> **Front:** 98 mm fascia both faces; access 1300 × 1440 @ X=626, Z=640–2080; **noggins** between pier studs at 600/1200/1800 (+ band noggins).  
> **Bed:** owned; frame 360 mm; lift-only 170 mm → platform 530; mattress 1900×900 (+10 mm); pillow toward head (left).  
> **Stud buy:** **13×** 3 m C-stud (incl. noggins) + 4× track — see `PARTS-LIST.md` / `PARTS-PURCHASE-CROSSREF.md`.  
> **Ask:** parametric model with real 70 mm C-stud / track / noggin profiles, named parts matching the BOQ, STEP export. Do not invent dimensions — use `LOCKED-IN-SPEC.md` and `DIM`.

## Recommendation — which AI / tool for CAD-grade work

| Need | Use |
|------|-----|
| Explore / check assembly (done here) | This repo’s Three.js viewer |
| Parametric solids + STEP for fabricators | **CadQuery** or **Build123d** in Cursor / Claude Code (scriptable, diffable, export STEP) |
| Interactive CAD with AI assist | **Zoo.dev** (Text-to-CAD) or **Adam** / Fusion 360 generative — good for profiles; still verify every locked dim |
| Architectural BIM / IFC | **BlenderBIM** or Revit + Dynamo; feed the brief + `DIM` table — not a mesh AI |
| Pretty mesh from photos | Meshy / Tripo etc. — **avoid** for construction accuracy |

**Practical pick:** keep this Three.js model as the living SoT for dimensions; if a fabricator needs STEP, hand the brief + `DIM` to a **CadQuery/Build123d** agent (Cursor or Claude Code) rather than a image-to-3D model. Do not use Midjourney/Meshy-style generators for studwork.
