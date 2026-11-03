# Parts list ↔ purchase plan cross-reference

Reconciles `PARTS-LIST.md` (zone BOM + cutting summary) with `COST-ESTIMATE-GL53.md` (priced BOQ) and `SOURCING-AND-DELIVERY.md` (where to buy / how to deliver).

**Orientation:** head **left**, foot **right** when looking into the alcove (`LOCKED-IN-SPEC.md`, Nov 2026).

---

## Verdict

| Check | Status |
|-------|--------|
| Metal stud qty vs framing + **noggins** | **Aligned at 13× 3.0m** (was 10 — short for PB cross-members) |
| U-track qty | **Aligned at 4× 3.0m** |
| Plasterboard 7–8 / cost 8 | **Aligned** (cost uses upper bound + spare) |
| Lift timber 5× | **Aligned** |
| Drawer sheet 2×18 + 1×12 | **Aligned** |
| Ledgers / beads / finish | **Aligned** (lump lines in cost) |
| Electronics / runners (parcels) | **Aligned** — online, not merchant stud drop |
| Sourcing script (Bence) | **Updated** to 13× stud + noggins wording |

No orphan parts in the zone list without a buy path; no purchase-plan lines that the parts list dropped.

---

## Line-by-line

| Parts-list need | Qty (list) | Cost estimate | Sourcing path | Notes |
|-----------------|------------|---------------|---------------|-------|
| 70mm C-stud 3.0m | **13** | **13** @ £11.85 | Merchant drop #1 (Bence / Wickes / TP) | 4 jambs + 2 foot + ~2 shelf fascia + **~3 noggins** + 1 spare |
| 72mm U-track 3.0m | **4** | **4** | Same drop | Front F/C/sill/lintel + foot ext + shelf rails |
| 12.5mm PB 2400×1200 | **7–8** | **8** | Same drop | Front **both faces**, foot ext, rear/head fascia faces, spare |
| Corner/stop bead ~12m | Yes | In beads lump £85 | Same drop | Opening + foot + shelf corners |
| Joint compound + tape | 1 set | In £85 lump | Same drop | |
| Drywall / wafer screws | as req. | In £85 lump | Same drop | |
| Softwood lift 47×150×2.4m | **5** | **5** ~£50 | Same drop | Rip/pack to **170mm** |
| Ledger 45×70 | ~3–4m (≈2 lengths) | **2** ~£12 | Same drop | Rear + head wall supports |
| 18mm sheet | **2** | In ply £104 | Same drop | Drawers; shelf decks from offcuts if possible |
| 12mm sheet | **1** | In ply £104 | Same drop | Drawer bases |
| Grab adhesive + short screws | Yes | In £120 misc | Same drop / Screwfix | Floor ≤25–30mm; cable-detect |
| Primer + emulsion | as req. | In £120 misc | Merchant decorating / Wickes | Olive / chosen finish |
| Curtain track ≥1300 | 1 | In £120 misc | Wickes / Dunelm / Amazon | Parcel OK |
| Drawer handles ×2 | 2 | In £120 misc | Screwfix / Amazon | |
| 900mm 100kg runners ×2 pairs | 2 pairs | Parcels £70–£90 | **Online** (not light kitchen runners) | |
| ESP32 7" + LED ~5m + 5V 10A + light/speaker/starfield | Kit | Parcels £200–£350 | **Online** (Amazon / Pi Hut) | Not on Bence stud drop |
| Wall plugs / resin for ledgers | as req. | Misc / on site | Merchant / Screwfix | Into **existing** back/head walls |
| Tablet mount block | 1 offcut | Softwood offcut | Same timber drop | ~975 AFF on rear wall |

---

## Noggin add (why stud count rose)

Full-height PB on the **front piers** and **foot extension** needs horizontal members between studs. The old list only had “optional cripples”; the model and buy list now treat these as **required**:

| Location | Heights (AFF) | Approx. clear span |
|----------|---------------|--------------------|
| Front left pier (head side) | 320, 600, 1200, 1800, 2230 | ~180–250mm |
| Front right pier (foot side) | same | ~500–550mm |
| Foot extension | 600, 1200, 1800 | ~100–150mm |
| Shelf fascia (light) | ~400, 800 | from upright offcuts |

≈ **8–9m** of noggin stock → **3× 3.0m** C-stud lengths in the buy list.

---

## Purchase actions (unchanged split)

1. **One merchant/crane drop:** 13× stud, 4× track, 8× PB, beads/compound/tape/screws, 5× lift timber, ledgers, 2×18 + 1×12 sheet, grab adhesive.  
2. **Online parcels:** 900mm 100kg runners ×2, ESP32 7", LED + PSU, curtain track if not on drop, optional starfield/speaker.  
3. **Electrician:** mains to PSU only.

Use the Bence script in `COST-ESTIMATE-GL53.md` (already says **13× stud** + noggins).
