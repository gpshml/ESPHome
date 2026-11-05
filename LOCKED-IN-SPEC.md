# Cabin Bed — Locked-In Spec (do not invent)

Last updated **3 Nov 2026** — orientation restored to match locked drawings (`top_plan_FINAL.png` / front aspect). See `BUILD-SEQUENCE.md`.

## Orientation (single source of truth)

| Direction | What it is |
|-----------|------------|
| **LEFT** | **FOOT** — existing wall + full-height foot wall (98mm) + **237mm extension at the front** into the room |
| **RIGHT** | **HEAD** — 326mm solid fascia / head wall (carried on existing head wall; PB on **exposed** faces only) |
| **BACK / REAR** | Rear wall + **210mm rear shelf**, wall structure stops at **1200mm** shelf top |
| **FRONT** | Into the room — access opening with curtains, front fascia **98mm** thick |

Looking into the alcove from the front: **foot LEFT, head RIGHT**. Access opening starts **300mm from the left of internal** → **narrow pier at the foot**, wider pier at the head.

### Root cause of the head/foot flip (do not repeat)

In Nov 2026 3D review, Blender / GitHub axis conventions made the mesh *look* mirrored relative to the locked plan. Feedback such as “foot where head should be” was wrongly treated as a **room layout** correction and the model/docs were flipped to HEAD-left.

**That was incorrect.** The room and the locked image never changed: looking in from the front, foot is left and head is right. Viewer/export confusion must be fixed in **how the mesh is viewed** (markers, camera, Y-up bake), never by inverting the SoT away from `top_plan_FINAL.png`.

## Coordinate system (3D master → all 2D slices)

```
Origin (0,0,0) = BACK-LEFT-FLOOR corner (rear wall ∩ foot wall ∩ floor)
X: 0 → 2344mm   FOOT (left) → HEAD (right)
Y: 0 → 1242mm   BACK → FRONT (into room)
Z: 0 → 2380mm   FLOOR → CEILING
```

- Foot extension: same X footprint as existing foot wall (**0..98**); front face at **Y=D** flush with the front wall’s room face; inner face at **X=FOOT** flush with the left end of the front fascia.
- Shelf **ledgers** sit so their **top meets the deck underside** (no floating gap).
- Rear (and head-front) fascia PB is **flush with the shelf deck front** — not proud into the bay, not recessed. Router-ease the nosing for a smooth lean.
- Head bed-face PB covers the **exposed bed bay only** (from rear-shelf front forward) — not a hidden sheet back to the rear wall.
- Blender OBJ/STL are exported **Y-up** (metres) so the model stands upright with default import settings.
- `Marker-FOOT` / `Marker-HEAD` are **review-only** cubes (not build parts).

## Width is fixed — depth may grow

**Alcove clear width `INTERNAL_W = 2246mm` is fixed** (original plan). It does **not** expand for bedding tolerances, build clearance, or PB thickness.

| Axis | Rule |
|------|------|
| **Width (X)** | **Fixed at 2246mm** clear (foot-wall inner face → existing head wall). External envelope is 2246 + 98 foot wall = **2344mm**. |
| **Depth (Y)** | **May grow into the room** — mainly by the **+237mm foot-wall extension** at the front (overall depth **1242mm**). |

**Width budget (must always sum to 2246):**

```
INTERNAL_W = MATT_W + 2×TOL + HEAD_W
    2246   =  1900  +  20   +  326
```

- Mattress bay (incl. 10mm tolerance each end) takes **1920mm**.
- Whatever remains is the **head shelf / fascia depth from bed face → head wall** = **`HEAD_W`**.
- If tolerances (or finishes) need more width, **`HEAD_W` shrinks** — never grow `INTERNAL_W`.
- Head PB and studs sit **inside** that 326mm (finished bed face at `W − HEAD_W`); they must not steal bay width.

See also `CORRECTED-TOLERANCE.md` (historic correction when width was wrongly grown to 2266).

## Dimensions that are locked

| Item | Value | Notes |
|------|-------|-------|
| Internal width | **2246mm** | **FIXED** — clear alcove; does **not** include foot wall |
| Foot wall thickness | 98mm | 70mm stud + plasterboard (outside the 2246) |
| Total external width | 2344mm | 2246 + 98 |
| Overall depth | 1242mm | Expanded into the room (foot extension) |
| Foot wall extension | **237mm at FRONT** | Depth growth into the room; flush with existing wall + front fascia end |
| Overall height | 2380mm | Full height |
| Shelf top | 1200mm | Rear wall stops here |
| Mattress / bed | 1900 × 900 × 220mm | Already purchased |
| Side tolerance | 10mm each end | Absorbed by reducing head shelf, not by widening alcove |
| Lift / platform | 170mm → platform **530mm** AFF | Bed frame 360mm owned |
| Head wall / fascia | **326mm** × 1144mm deep | Residual: `2246 − 1900 − 20`; on the **right** |
| Rear shelf | 1920 × 210mm | INTERNAL_W − HEAD_W; stops before head |
| Access opening | 1300 × 1440mm | 640–2080 AFF; starts **300mm from left of internal (foot side)** → ACCESS_X = 398 |
| Front fascia | 98mm thick | Both faces + opening reveals boarded |
| Noggins | Required | Full-height PB support — see `PARTS-LIST.md` |

## Opening reveal facings (inside the access opening)

| Item | Spec |
|------|------|
| Material | **12.5mm tapered-edge plasterboard** (same as other boarded faces) |
| Where | Left & right jamb reveals, sill top, lintel soffit — wrapping the 98mm wall thickness |
| Fixing | **Drywall screws** into C-stud jambs, noggins, and sill/lintel track (typically ~300mm centres) |
| Edges | **Metal stop bead / corner bead** on external corners of the opening, then tape + joint compound |
| Finish | Primer + emulsion (autumnal olive on visible faces) |

## What is NOT true (past mistakes — do not reintroduce)

1. Do **not** put a 200mm “cut-in” under the foot of the bed.
2. Do **not** omit equal front/rear bed tuck.
3. Do **not** put the 237mm extension at the **rear** of the foot wall.
4. Do **not** swap foot/head vs the locked front aspect (foot stays **left**).
5. Do **not** stop the foot wall at 1200mm.
6. Do **not** omit noggins on full-height boarded stud walls.
7. Do **not** leave the foot extension proud of / short of the existing wall or front fascia room face.
8. Do **not** leave wall ledgers below the shelf decks (gap).
9. Do **not** run head bed-face PB through to the rear wall (hidden behind rear shelf).
10. Do **not** treat a mirrored 3D preview as authority over `top_plan_FINAL.png`.
11. Do **not** expand alcove clear width past **2246mm** — grow depth (foot extension) or shrink **HEAD_W** instead.

## Source of truth & files

| File | Role |
|------|------|
| `top_plan_FINAL.png` / front aspect | Locked left/right (FOOT left, HEAD right) |
| `BUILD-SEQUENCE.md` | Real-world build order + first-principles checks |
| `cabin_bed_model.js` | Geometry + `DIM` |
| `LOCKED-IN-SPEC.md` | This file |
| `PARTS-LIST.md` / `PARTS-PURCHASE-CROSSREF.md` | BOM + purchase |
| `cadquery/` | STEP (mm) + OBJ/STL (metres, Blender Y-up) |
| `cadquery/RENDER.md` | Materials / render notes |
