# Cabin Bed — Parts & Materials List

Derived from locked dimensions (`LOCKED-IN-SPEC.md` / `cabin_bed_model.js`).  
Overall envelope: **2344mm W × 1242mm D × 2380mm H**.

**Bed status:** the bed (frame + mattress) is **already purchased**. Buy only what is needed to **raise** it (plinth / risers / legs), plus the alcove structure, drawers, and finishes below. Do **not** order a new bed frame or mattress support rails.

**Buying / delivery:** see [`SOURCING-AND-DELIVERY.md`](SOURCING-AND-DELIVERY.md) (Bence vs Wickes/TP vs trade drylining + electronics parcels).

---

## 1. Metal stud framing (70mm C-stud system)

| Item | Spec | Qty / length | Notes |
|------|------|--------------|-------|
| Floor track | 70mm | ~7.2m total | Perimeter: foot wall 1242, rear 1920 (stops at head), head 1144 (back→front fascia), front 2246 |
| Head / ceiling track | 70mm | ~7.2m total | Match floor where walls go to full height |
| Vertical studs (full height) | 70mm × 2380mm | 10–12 | Foot wall, front jambs, head end above shelf as needed; @ ~400mm centres |
| Vertical studs (to shelf) | 70mm × 1200mm | 6–8 | Rear wall only (stops at shelf); width to head fascia |
| Vertical studs (head fascia) | 70mm × 1200mm | 3–4 | Head wall 326mm wide × 1144mm deep (back wall to front fascia) |
| Noggins / horizontals | 70mm | ~12m | At 530mm (platform), ~975mm (tablet), 1200mm (shelf), 640 & 2080 (opening) |
| Foot wall extension framing | 70mm | +237mm depth | Extend existing foot wall **into the room** |

**Plasterboard join tip:** strip ~200mm of existing plasterboard back from the foot-wall end before tying new studs in.

---

## 2. Plasterboard & finish

| Item | Spec | Qty | Notes |
|------|------|-----|-------|
| Plasterboard | 12.5mm | ~8 sheets (2400×1200) | Walls both faces where exposed; allow waste |
| Tape & joint compound | — | 1 set | All seams |
| Corner beads | metal/PVC | ~12m | External corners, opening reveals |
| Primer + emulsion | — | as required | Inside alcove + fascia |

---

## 3. Timber — lift only (bed already owned)

Raise the **existing** bed (**360mm** measured frame height) on a **170mm** lift so platform top = **530mm** AFF (170 + 360).

| Item | Size | Qty | Notes |
|------|------|-----|-------|
| Base plinth / risers | ~45 × **170mm** × ~1900mm | 3 | Longitudinal bearers under the existing bed; short screws + grab adhesive (cables under floor) |
| Cross bearers | ~45 × **170mm** × ~900mm | 2–3 | Tie risers; stop racking |
| Support legs / packs | to finish at **170mm** | 6 | Corners + mid-span; pack 47×150 (~145–150mm) on edge up to 170mm if not ripping full-depth stock |
| Shelf fascia (rear) | 20×40–60×1920mm | 1 | Optional decorative front edge of rear shelf (flush, not proud) |
| Head fascia trim | 20×40–60×326mm | 1 | Optional finish on head wall |

**Practical timber:** 47×150 C16 on edge + ~20–25mm packs to **170mm**, or have Bence rip softwood to 170mm deep. Avoid relying on thin CLS alone.

**Not required (already owned):** bed frame rails, mattress, mattress base / slats, new bed carcass.

**Floor fixing:** max screw penetration **25–30mm**; prefer grab adhesive; cable-detect before drilling.  
**Fit check:** centre the lifted bed in the bay with equal front/rear tuck (~17mm to mattress) and ~10mm at foot/head.

---

## 4. Drawers × 2 (under raised bed)

| Component | Size (each) | Material | Qty |
|-----------|-------------|----------|-----|
| Front / back | 880 × 500 × 18mm | Plywood / MDF | 2+2 |
| Sides | 900 × 500 × 18mm | Plywood / MDF | 2+2 |
| Base | 880 × 900 × 12mm | Plywood | 2 |
| Floor runners | 900mm heavy-duty | Metal slides | 4 per drawer (8 total) |
| Handles | — | — | 2 |

Clearances: ~20mm bottom (runners), ~10mm top under the raised bed. Void height ≈ 500mm usable under the platform.

---

## 5. Front access opening

| Item | Spec | Notes |
|------|------|-------|
| Opening | 1300 × 1440mm | From 640mm AFF to 2080mm AFF; starts 300mm from left of **internal** width |
| Curtain track | 1300mm+ | Inside reveal or face-fix |
| Curtain fabric | pair | Floor- or sill-length as preferred |
| Reveal lining | 12.5mm PB or timber | Finish opening edges |

Front fascia thickness: **98mm** (stud + plasterboard), not the 237mm wall extension depth.

---

## 6. Electronics & lighting

| Item | Spec | Qty |
|------|------|-----|
| ESP32 board | DevKit or equivalent | 1 |
| 7" touchscreen | ESP32-compatible | 1 |
| Mount noggin | at ~975mm AFF, rear shelf centre | 1 |
| Ambient LED | WS2812B RGBW, 5V, ~5m | 1 |
| Reading light | 4000K, CRI>90, dimmable ~1m | 1 |
| Starfield | Fibre optic kit and/or WS2812B points | 1 set |
| PSU | 5V 10A (50–60W) | 1 |
| Speaker | Compact BT/Wi‑Fi, head corner | 1 |
| Cable | 5V LED + signal, mains to PSU | as required |

Route cables in stud cavities **before** boarding. Qualified electrician for mains.

---

## 7. Consumables & hardware

| Item | Notes |
|------|-------|
| Grab adhesive | Primary floor / plinth fixing under the existing bed |
| Screws | Short for floor (≤30mm); stud & board screws as normal |
| Wall plugs | If fixing into masonry head wall |
| Cable detector | Mandatory before floor fixings |
| Expanding foam / sealant | Gaps at wall junctions |

---

## 8. Cutting summary (quick reference)

| Material | Cut list (approx.) |
|----------|-------------------|
| 70mm track | 2×2344, 2×1242, 2×2246, extras for opening & head |
| 70mm stud | 10–12×2380, 8–10×1200, noggins ~12m |
| 12.5mm PB | ~8 sheets |
| Softwood (lift only) | 3×~1900 + 2–3×~900 @ **170mm** deep (or 47×150 + packs) |
| 18mm sheet | Drawer boxes (~2.5 sheets) |
| 12mm sheet | Drawer bases (1 sheet) |

---

## 9. Safety checklist

- [ ] Cable-detect timber floor before any screws  
- [ ] Screws into floor ≤25–30mm  
- [ ] Risers finished at **170mm** under **360mm** bed → platform top **530mm** AFF  
- [ ] Fire-safe finishes in enclosed sleeping space  
- [ ] Mains work by qualified electrician  
- [ ] Opening edges finished (no raw plasterboard)  
