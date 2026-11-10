# Parts list ↔ purchase plan cross-reference

Reconciles `PARTS-LIST.md` (zone BOM + cutting summary) with `COST-ESTIMATE-GL53.md` (priced BOQ) and `SOURCING-AND-DELIVERY.md` (where to buy / how to deliver).

**Orientation:** foot **left**, head **right**. Alcove clear width **2246mm FIXED**.  
**Latest structural add:** **corner post** at rear/head fascia L (wall-fixed pick-up).

---

## Verdict

| Check | Status |
|-------|--------|
| Metal stud qty vs framing + noggins + **corner post** | **Aligned at 13× 3.0m** |
| U-track qty | **Aligned at 4× 3.0m** |
| Plasterboard | **Aligned at 8** (incl. reveals + spare) |
| Lift timber 5× | **Aligned** |
| Drawer sheet 2×18 + 1×12 | **Aligned** |
| Ledgers / beads / finish | **Aligned** |
| Electronics / runners (parcels) | **Aligned** — online, not Bence stud drop |
| Bence script | **Updated** — corner post + fixed width called out |

No orphan parts; no purchase lines the parts list dropped.

---

## Line-by-line

| Parts-list need | Qty (list) | Cost estimate | Sourcing path | Notes |
|-----------------|------------|---------------|---------------|-------|
| 70mm C-stud 3.0m | **13** | **13** @ ~£11.85 | **Bence** drop #1 | 4 jambs + 2 foot + ~3 shelf/corner + ~3 noggins + 1 spare |
| 72mm U-track 3.0m | **4** | **4** | Same drop | Front F/C/sill/lintel + foot ext + shelf rails |
| 12.5mm TE PB 2400×1200 | **8** | **8** | Same drop | Front both faces + **reveals**, foot, rear/head fascias, spare |
| Corner/stop bead ~12m | Yes | In beads lump £85 | Same drop | Opening + foot + shelf + **rear/head L** |
| Joint compound + tape | 1 set | In £85 lump | Same drop | |
| Drywall / wafer screws | as req. | In £85 lump | Same drop | |
| Softwood lift 47×150×2.4m | **5** | **5** ~£50 | Same drop | Rip/pack to **170mm** |
| Ledger 45×70 | ~3–4m (**2** lengths) | **2** ~£12 | Same drop | Rear + head wall supports |
| 18mm sheet | **2** | In ply £104 | Same drop | Drawers; shelf decks from offcuts if possible |
| 12mm sheet | **1** | In ply £104 | Same drop | Drawer bases |
| Grab adhesive + short screws | Yes | In £120 misc | Same drop / Screwfix | Floor ≤25–30mm; cable-detect |
| Primer + emulsion | as req. | In £120 misc | Merchant decorating | Olive / chosen finish |
| Curtain track ≥1300 | 1 | In £120 misc | Wickes / Dunelm / Amazon | Parcel OK |
| Drawer handles ×2 | 2 | In £120 misc | Screwfix / Amazon | |
| 900mm 100kg runners ×2 pairs | 2 pairs | Parcels £70–£90 | **Online** | Not light kitchen runners |
| ESP32 7" + LED ~5m + 5V 10A + light/speaker/starfield | Kit | Parcels £200–£350 | **Online** | Not on Bence stud drop |
| Wall plugs / resin | as req. | Misc | Merchant / Screwfix | Ledgers **+ corner post** into existing walls |
| Tablet mount block | 1 offcut | Softwood offcut | Same timber drop | ~975 AFF on rear wall |

---

## Noggin + corner cut (why 13 studs)

| Location | Heights (AFF) | Approx. clear span |
|----------|---------------|--------------------|
| Front **foot/left** pier (narrow 300mm) | 320, 600, 1200, 1800, 2230 | ~**110mm** |
| Front **head/right** pier (wide ~646mm) | same | ~**456mm** (critical for PB) |
| Foot extension | 600, 1200, 1800 | ~100–150mm |
| Shelf fascia (light) | ~400, 800 | from upright offcuts |
| **Corner post** rear/head L | full fascia height ~1182 | 1 upright — wall-fixed |

≈ **7–9m** noggin stock → **3× 3.0m** in the buy list. Shelf fascia + corner ≈ **3× 3.0m**. Full-height jambs/foot = **6× 3.0m**. Spare **1×** → **13**.

---

## Purchase actions

1. **One Bence/crane drop:** 13× stud, 4× track, 8× PB, beads/compound/tape/screws, 5× lift timber, 2× ledgers, 2×18 + 1×12 sheet, grab adhesive. Use the script in `COST-ESTIMATE-GL53.md`.  
2. **Online parcels:** 900mm 100kg runners ×2, ESP32 7", LED + PSU, curtain track if not on drop, optional starfield/speaker.  
3. **Electrician:** mains to PSU only.
