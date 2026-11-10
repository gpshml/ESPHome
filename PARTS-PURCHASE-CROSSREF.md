# Parts list ↔ purchase plan cross-reference

Reconciles **CadQuery model** → `MODEL-BOM.md` → `PARTS-LIST.md` → `COST-ESTIMATE-GL53.md` → `SOURCING-AND-DELIVERY.md`.

**Orientation:** foot **left**, head **right**. Alcove clear width **2246mm FIXED**.  
**Corner post** at rear/head fascia L (wall-fixed).  
**No noggins.**

---

## Verdict

| Check | Status |
|-------|--------|
| C-stud vs model uprights | **Aligned — buy 12** (10 packed + 2 spare) |
| Noggins | **None** — removed from model + buy |
| U-track vs model runs | **Aligned — buy 5** |
| Plasterboard vs 18 modeled faces | **Aligned — buy 8** |
| Lift timber vs `Lift-*` solids | **Aligned — 3×920 + 2×1900 @ 170** |
| Softwood ledgers | **Aligned — 1920 + 1144 from 2× 45×70×2.4m** |
| Ply 2×18 + 1×12 | **Aligned** — drawers + decks / bases |
| Electronics / runners | **Aligned** — online parcels |

---

## Model → buy (authoritative)

| Model | Count / length | Buy |
|-------|----------------|-----|
| C-stud uprights | **13** pcs / 22.55m | **12×** 3.0m |
| Noggins | **0** | — |
| U-track | **8** runs / 12.31m | **5×** 3.0m |
| PB faces | **18** / ~12.6 m² | **8×** 2400×1200 |
| Lift timber | 3×920 + 2×1900 @170 | **5×** 47×150×2.4m |
| Ledgers | Rear 1920 + Head 1144 softwood | **2×** 45×70×2.4m |
| Decks + drawers | 18mm / 12mm ply | **2×** 18mm + **1×** 12mm 8×4 |

Full named list: [`MODEL-BOM.md`](MODEL-BOM.md).

---

## Why noggins were dropped

Standard metal C-stud partitions board onto studs + floor/ceiling/sill/lintel track. Pier clear spans here are ~110mm (foot) and ~456mm (head) — both well under the usual ≤600mm stud centres — so horizontal noggins add cost without structural need. Tablet mounts on a **timber block on the existing rear wall**, not on stud PB.

---

## Purchase actions

1. **One Bence/crane drop:** **12×** stud, **5×** track, 8× PB, beads/compound/tape/screws, 5× lift timber, **2× softwood 45×70 ledgers**, **2× 18mm ply + 1× 12mm ply** (8×4), grab adhesive. Script in `COST-ESTIMATE-GL53.md`.  
2. **Online parcels:** 900mm 100kg runners ×2, ESP32 7", LED + PSU, curtain track if not on drop.  
3. **Electrician:** mains to PSU only.
