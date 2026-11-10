# Parts list ↔ purchase plan cross-reference

Reconciles **CadQuery model** → `MODEL-BOM.md` → `PARTS-LIST.md` → `COST-ESTIMATE-GL53.md` → `SOURCING-AND-DELIVERY.md`.

**Orientation:** foot **left**, head **right**. Alcove clear width **2246mm FIXED**.  
**Corner post** at rear/head fascia L (wall-fixed).

---

## Verdict

| Check | Status |
|-------|--------|
| C-stud vs model uprights + noggins | **Aligned — buy 13** (11 packed + 2 spare) |
| U-track vs model runs | **Aligned — buy 5** (was 4; short vs packing) |
| Plasterboard vs 18 modeled faces | **Aligned — buy 8** |
| Lift timber vs `Lift-*` solids | **Aligned — 3×920 + 2×1900 @ 170** |
| Drawer sheet 2×18 + 1×12 | **Aligned** |
| Ledgers / decks / tablet block | **Aligned** to named model parts |
| Electronics / runners | **Aligned** — online parcels, not Bence drop |

---

## Model → buy (authoritative)

| Model | Count / length | Buy |
|-------|----------------|-----|
| C-stud uprights | **13** pcs / 22.55m | with noggins → **13×** 3.0m |
| Noggins | **21** pcs / 7.55m | (same C-stud stock) |
| U-track | **8** runs / 12.31m | **5×** 3.0m |
| PB faces | **18** / ~12.6 m² | **8×** 2400×1200 |
| Lift timber | 3×920 + 2×1900 @170 | **5×** 47×150×2.4m |
| Ledgers | Rear 1920 + Head 1144 | **2×** lengths |
| Decks | Rear 1920×210 + Head 326×1144 ×18 | from 18mm sheets |

Full named list: [`MODEL-BOM.md`](MODEL-BOM.md).

---

## Why track went 4 → 5

Four front rails @ **2246mm** cannot share a 3.0m stick with the **1920** rear fascia rail (2246+1920 > 3000). Packing:

1. 2246 + 237 + 237 (one front + both foot shorts)  
2–4. three remaining front rails  
5. 1920 rear + 934 head  

---

## Why stud stays 13

Uprights modeled: **4** front jambs + **2** foot + **3** rear fascia + **3** head fascia + **1** corner = **13**.  
Noggins (21) nest into leftover and **11** total packed sticks; buy **13** leaves **2** spare for kerf/mistakes.

---

## Purchase actions

1. **One Bence/crane drop:** 13× stud, **5×** track, 8× PB, beads/compound/tape/screws, 5× lift timber, **2× softwood 45×70 ledgers**, **2× 18mm ply + 1× 12mm ply** (8×4), grab adhesive. Script in `COST-ESTIMATE-GL53.md`.
2. **Online parcels:** 900mm 100kg runners ×2, ESP32 7", LED + PSU, curtain track if not on drop.  
3. **Electrician:** mains to PSU only.
