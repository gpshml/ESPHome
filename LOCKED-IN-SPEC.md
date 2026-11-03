# Cabin Bed — Locked-In Spec (do not invent)

Last updated **3 Nov 2026** — orientation locked to **user-confirmed** review (HEAD left / FOOT right) with narrow pier at the foot. See `BUILD-SEQUENCE.md`.

## Orientation (3D assembly + build sequence)

| Direction | What it is |
|-----------|------------|
| **LEFT** | **HEAD** — 326mm solid fascia / head wall (carried on existing head wall; PB on **exposed** faces only) |
| **RIGHT** | **FOOT** — existing wall + full-height foot wall (98mm) + **237mm extension at the front** into the room |
| **BACK / REAR** | Rear wall + **210mm rear shelf**, wall structure stops at **1200mm** shelf top |
| **FRONT** | Into the room — access opening with curtains, front fascia **98mm** thick |

Looking into the alcove: **head LEFT, foot RIGHT**. Access opening is set out leaving **300mm at the foot (right)** → **narrow pier at the foot**, wider pier at the head.

> Note: early `top_plan_FINAL.png` labels read foot-left; live review repeatedly confirmed the **opposite** head/foot placement for the room. The 3D SoT follows the confirmed layout. Pier width rule (narrow at foot) is unchanged.

## Coordinate system (3D master → all 2D slices)

```
Origin (0,0,0) = BACK-LEFT-FLOOR corner (rear wall ∩ head wall ∩ floor)
X: 0 → 2344mm   HEAD (left) → FOOT (right)
Y: 0 → 1242mm   BACK → FRONT (into room)
Z: 0 → 2380mm   FLOOR → CEILING
```

- Foot extension: same X footprint as existing foot wall (**FOOT_X0..W**); front face at **Y=D** flush with the front wall’s room face; inner face at **X=FOOT_X0** flush with the right end of the front fascia.
- Shelf **ledgers** sit so their **top meets the deck underside** (no floating gap).
- Head bed-face PB covers the **exposed bed bay only** (from rear-shelf front forward) — not a hidden sheet back to the rear wall.
- Blender OBJ/STL are exported **Y-up** (metres) so the model stands upright with default import settings.

## Dimensions that are locked

| Item | Value | Notes |
|------|-------|-------|
| Internal width | 2246mm | Does **not** include foot wall |
| Foot wall thickness | 98mm | 70mm stud + plasterboard |
| Total external width | 2344mm | 2246 + 98 |
| Overall depth | 1242mm | Target alcove depth |
| Foot wall extension | **237mm at FRONT** | Into the room; flush with existing wall + front fascia end |
| Overall height | 2380mm | Full height |
| Shelf top | 1200mm | Rear wall stops here |
| Mattress / bed | 1900 × 900 × 220mm | Already purchased |
| Lift / platform | 170mm → platform **530mm** AFF | Bed frame 360mm owned |
| Head wall / fascia | 326mm × 1144mm deep | On the **left** |
| Rear shelf | 1920 × 210mm | INTERNAL_W − HEAD_W; starts after head |
| Access opening | 1300 × 1440mm | 640–2080 AFF; **300mm pier at foot (right)** → ACCESS_X = 646 |
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
4. Do **not** put the 326mm head fascia on the foot side (head stays **left**).
5. Do **not** stop the foot wall at 1200mm.
6. Do **not** omit noggins on full-height boarded stud walls.
7. Do **not** leave the foot extension proud of / short of the existing wall or front fascia room face.
8. Do **not** leave wall ledgers below the shelf decks (gap).
9. Do **not** run head bed-face PB through to the rear wall (hidden behind rear shelf).

## Source of truth & files

| File | Role |
|------|------|
| `BUILD-SEQUENCE.md` | Real-world build order + first-principles checks |
| `cabin_bed_model.js` | Geometry + `DIM` |
| `LOCKED-IN-SPEC.md` | This file |
| `PARTS-LIST.md` / `PARTS-PURCHASE-CROSSREF.md` | BOM + purchase |
| `cadquery/` | STEP (mm) + OBJ/STL (metres, Blender Y-up) |
| `cadquery/RENDER.md` | Materials / render notes |
