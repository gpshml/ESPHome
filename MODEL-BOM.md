# Model BOM — exact CadQuery parts

Generated from `cadquery/build_assembly.py` (same assembly as STEP/OBJ). Purchase docs (`PARTS-LIST.md`, `COST-ESTIMATE-GL53.md`) must match the **buy** column below — not hand-waved zone estimates.

**Envelope:** 2344 × 1242 × 2380 mm · INTERNAL_W **2246** · HEAD_W **326** · foot/left pier **300** · head/right pier **646**.

Excluded from buy: `Existing-*`, `Marker-*`, owned `Bed-Frame` / `Mattress` / drawer solids (sheet buy still listed).

## Buy from model packing (3.0m metal stock)

| Stock | Model need | Pack sticks | **Buy** | Notes |
|-------|------------|-------------|---------|-------|
| 70mm C-stud × 3.0m | 13 uprights (22.55m) + 21 noggins (7.55m) = **30.10m** | **11** | **13** | Uprights: 4 front jambs + 2 foot + 3 rear fascia + 3 head fascia + 1 corner post |
| 72mm U-track × 3.0m | 8 runs (12.31m) | **5** | **5** | 4× front @ 2246 cannot share a stick with shelf rails |
| 12.5mm TE PB 2400×1200 | 18 faces (12.60 m²) | — | **8** | Picture-frame nesting; 8 sheets ≈ 23 m² |

### Metal packing detail

**C-stud sticks (uprights + noggins):**
- Stick 1: 2380 + 456 + 110 → rem 54mm
- Stick 2: 2380 + 456 + 110 → rem 54mm
- Stick 3: 2380 + 456 + 110 → rem 54mm
- Stick 4: 2380 + 456 + 110 → rem 54mm
- Stick 5: 2380 + 456 + 110 → rem 54mm
- Stick 6: 2380 + 427 + 177 → rem 16mm
- Stick 7: 1182 + 1182 + 427 + 177 → rem 32mm
- Stick 8: 1182 + 1182 + 70 + 70 + 70 → rem 426mm
- Stick 9: 1182 + 1182 → rem 636mm
- Stick 10: 1182 + 825 + 825 → rem 168mm
- Stick 11: 825 + 825 → rem 1350mm

**U-track sticks:**
- Stick 1: 2246 + 237 + 237 → rem 280mm
- Stick 2: 2246 → rem 754mm
- Stick 3: 2246 → rem 754mm
- Stick 4: 2246 → rem 754mm
- Stick 5: 1920 + 934 → rem 146mm

## C-stud uprights (13)

| Part | Size (mm) | Cut |
|------|-----------|-----|
| `Corner-Post-RearHead` | 70 × 45 × 1182 | **1182** |
| `FootExt-Stud-Front` | 70 × 45 × 2380 | **2380** |
| `FootExt-Stud-Rear` | 70 × 45 × 2380 | **2380** |
| `Front-Jamb-L1` | 70 × 45 × 2380 | **2380** |
| `Front-Jamb-L2` | 70 × 45 × 2380 | **2380** |
| `Front-Jamb-R1` | 70 × 45 × 2380 | **2380** |
| `Front-Jamb-R2` | 70 × 45 × 2380 | **2380** |
| `Head-Fascia-Stud-1` | 45 × 70 × 1182 | **1182** |
| `Head-Fascia-Stud-2` | 45 × 70 × 1182 | **1182** |
| `Head-Fascia-Stud-3` | 45 × 70 × 1182 | **1182** |
| `Rear-Fascia-Stud-1` | 70 × 45 × 1182 | **1182** |
| `Rear-Fascia-Stud-2` | 70 × 45 × 1182 | **1182** |
| `Rear-Fascia-Stud-3` | 70 × 45 × 1182 | **1182** |

## Noggins (from C-stud stock) (21)

| Part | Size (mm) | Cut |
|------|-----------|-----|
| `FootExt-Noggin-1` | 70 × 67 × 40 | **70** |
| `FootExt-Noggin-2` | 70 × 67 × 40 | **70** |
| `FootExt-Noggin-3` | 70 × 67 × 40 | **70** |
| `Front-Noggin-L-1` | 110 × 45 × 40 | **110** |
| `Front-Noggin-L-2` | 110 × 45 × 40 | **110** |
| `Front-Noggin-L-3` | 110 × 45 × 40 | **110** |
| `Front-Noggin-L-Band-1` | 110 × 45 × 40 | **110** |
| `Front-Noggin-L-Band-2` | 110 × 45 × 40 | **110** |
| `Front-Noggin-R-1` | 456 × 45 × 40 | **456** |
| `Front-Noggin-R-2` | 456 × 45 × 40 | **456** |
| `Front-Noggin-R-3` | 456 × 45 × 40 | **456** |
| `Front-Noggin-R-Band-1` | 456 × 45 × 40 | **456** |
| `Front-Noggin-R-Band-2` | 456 × 45 × 40 | **456** |
| `Head-Fascia-Noggin-1-1` | 45 × 177 × 40 | **177** |
| `Head-Fascia-Noggin-1-2` | 45 × 177 × 40 | **177** |
| `Head-Fascia-Noggin-2-1` | 45 × 427 × 40 | **427** |
| `Head-Fascia-Noggin-2-2` | 45 × 427 × 40 | **427** |
| `Rear-Fascia-Noggin-1-1` | 825 × 45 × 40 | **825** |
| `Rear-Fascia-Noggin-1-2` | 825 × 45 × 40 | **825** |
| `Rear-Fascia-Noggin-2-1` | 825 × 45 × 40 | **825** |
| `Rear-Fascia-Noggin-2-2` | 825 × 45 × 40 | **825** |

## U-track (8)

| Part | Size (mm) | Cut |
|------|-----------|-----|
| `FootExt-Track-Ceiling` | 72 × 237 × 25 | **237** |
| `FootExt-Track-Floor` | 72 × 237 × 25 | **237** |
| `Front-Track-Ceiling` | 2246 × 72 × 25 | **2246** |
| `Front-Track-Floor` | 2246 × 72 × 25 | **2246** |
| `Front-Track-Lintel` | 2246 × 72 × 25 | **2246** |
| `Front-Track-Sill` | 2246 × 72 × 25 | **2246** |
| `Head-Fascia-Track` | 72 × 934 × 25 | **934** |
| `Rear-Fascia-Track` | 1920 × 72 × 25 | **1920** |

## Plasterboard faces (18)

| Part | Size (mm) | Cut |
|------|-----------|-----|
| `FootExt-PB-Front` | 98 × 12 × 2380 | **2380** |
| `FootExt-PB-Inner` | 12 × 237 × 2380 | **2380** |
| `FootExt-PB-Outer` | 12 × 237 × 2380 | **2380** |
| `Front-PB-Alcove-Above` | 2246 × 12 × 300 | **2246** |
| `Front-PB-Alcove-Below` | 2246 × 12 × 640 | **2246** |
| `Front-PB-Alcove-LeftPier` | 300 × 12 × 1440 | **1440** |
| `Front-PB-Alcove-RightPier` | 646 × 12 × 1440 | **1440** |
| `Front-PB-Room-Above` | 2246 × 12 × 300 | **2246** |
| `Front-PB-Room-Below` | 2246 × 12 × 640 | **2246** |
| `Front-PB-Room-LeftPier` | 300 × 12 × 1440 | **1440** |
| `Front-PB-Room-RightPier` | 646 × 12 × 1440 | **1440** |
| `Front-Reveal-Left` | 12 × 98 × 1440 | **1440** |
| `Front-Reveal-Lintel` | 1300 × 98 × 12 | **1300** |
| `Front-Reveal-Right` | 12 × 98 × 1440 | **1440** |
| `Front-Reveal-Sill` | 1300 × 98 × 12 | **1300** |
| `Head-PB-BedFace` | 12 × 934 × 1182 | **1182** |
| `Head-PB-FrontFace` | 326 × 12 × 1182 | **1182** |
| `Rear-Fascia-PB` | 1920 × 12 × 1182 | **1920** |

## Timber (ledgers / decks / lift / tablet) (10)

| Part | Size (mm) | Cut |
|------|-----------|-----|
| `Head-Deck` | 326 × 1144 × 18 | **1144** |
| `Head-Ledger` | 70 × 1144 × 45 | **1144** |
| `Lift-Cross-Front` | 1900 × 45 × 170 | **1900** |
| `Lift-Cross-Rear` | 1900 × 45 × 170 | **1900** |
| `Lift-Riser-Long-1` | 45 × 920 × 170 | **920** |
| `Lift-Riser-Long-2` | 45 × 920 × 170 | **920** |
| `Lift-Riser-Long-3` | 45 × 920 × 170 | **920** |
| `Rear-Deck` | 1920 × 210 × 18 | **1920** |
| `Rear-Ledger` | 1920 × 70 × 45 | **1920** |
| `Rear-Tablet-Block` | 300 × 40 × 80 | **300** |

## Softwood lift — exact model pieces

| Part | Size (mm) | Role |
|------|-----------|------|
| `Lift-Cross-Front` | 1900 × 45 × 170 | 170mm plinth |
| `Lift-Cross-Rear` | 1900 × 45 × 170 | 170mm plinth |
| `Lift-Riser-Long-1` | 45 × 920 × 170 | 170mm plinth |
| `Lift-Riser-Long-2` | 45 × 920 × 170 | 170mm plinth |
| `Lift-Riser-Long-3` | 45 × 920 × 170 | 170mm plinth |

Buy **5×** 47×150×2.4m and rip/pack to **170mm** height. Model: **3×** long risers along depth (~920) + **2×** crosses along mattress width (1900).

## Ledgers + decks

**Ledgers = softwood** ~45×70 PSE/CLS (not sheet). **Decks = 18mm plywood** cut from the drawer sheets.

- `Head-Deck` (18mm ply deck): 326 × 1144 × 18
- `Head-Ledger` (softwood ledger): 70 × 1144 × 45
- `Rear-Deck` (18mm ply deck): 1920 × 210 × 18
- `Rear-Ledger` (softwood ledger): 1920 × 70 × 45

**Sheet buy:** 2× 18mm plywood 8×4 (drawer boxes + both decks) · 1× 12mm plywood 8×4 (drawer bases only).

---

*Regenerate: `python3 cadquery/build_assembly.py` (writes this file).*
