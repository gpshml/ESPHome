# Cabin Bed — Sourcing & Delivery Research

Researched for UK supply against `PARTS-LIST.md` (bed already owned; buy **lift + alcove + drawers + electronics**).  
Prices are indicative web list prices (inc. or ex. VAT as noted) and will move; treat as order-of-magnitude.

**Site-specific priced BOQ:** [`COST-ESTIMATE-GL53.md`](COST-ESTIMATE-GL53.md) for **GL53 0DF** (Bence + Wickes paths, ~£1.0–1.5k materials all-in).

---

## Verdict (short)

| Basket | Best delivery path | How easy? |
|--------|--------------------|-----------|
| **Bulk drylining + timber + plasterboard** | **George Bence** if you are in their delivery radius; otherwise **Wickes / Travis Perkins** one-drop, or a **trade drylining specialist** for better stud pricing | Easy — all stocked as standard |
| **Drawer sheet + heavy runners** | Merchant for sheet; specialist/online for **900mm 100kg** runners | Easy with a second small order |
| **Curtains / paint / beads / grab adhesive** | Same merchant drop or Screwfix next-day | Easy |
| **ESP32 7" + LEDs + PSU** | Amazon / The Pi Hut / specialist LED shops | Easy — **not** from a builders merchant |

**Recommended split:** one **builders-merchant drop** for everything heavy/long, one **online parcel** for electronics and (if needed) 900mm drawer runners.

---

## 1. George Bence ([bence.co.uk](https://www.bence.co.uk/))

Gloucestershire independent merchant (since 1854). Branches: **Cheltenham, Ledbury, Swindon, Bourton-on-the-Water** (+ roofing / plumbing / doors elsewhere).

| Topic | Detail |
|-------|--------|
| Drylining | Marketed as a **leading local plasterboard & drylining** supplier — right category for 70mm stud/track, board, beads, compounds |
| Timber / sheet | Softwood, plywood, MDF stocked (incl. Swindon “heavy side”) — covers **lift risers** and **drawer boxes** |
| Delivery radius | ~**40 miles** from Cheltenham HQ; Swindon delivers Swindon + nearby (confirm if outside) |
| Free next-day | Trade/cash account, order **>£150** before **12:00**, subject to stock/location |
| Vehicles | Crane lorries (26t / 12t / small “Mizi”), flatbed van, curtain sider — good for **8× plasterboard + long studs** without you unloading by hand |

**Fit for this project:** Strong **if the site is in Glos / nearby Wilts / Herefordshire**. One phone quote with the cut list from `PARTS-LIST.md` should cover stud, track, PB, beads, timber lift, plywood, grab adhesive, drywall screws — crane offload in one visit.

**Call:** Cheltenham `01242 237241` · Swindon `01793 383311` · [Delivery info](https://www.bence.co.uk/delivery)

If you are **outside** that radius, Bence is not the best primary option for heavy goods (nationwide specialists / Wickes / TP are).

---

## 2. Nationwide / multi-branch alternatives (heavy goods)

### Wickes
- **70mm Knauf C-stud** in 2.4 / 2.7 / 3.0m — e.g. ~£9–£12 per length; **next-day delivery** listed on product pages; free Click & Collect widely.
- **12.5mm 2400×1200** board ~£13/sheet.
- Watch **Big & Bulky** surcharge when order weight **>375kg** (8 sheets alone ≈ 180–200kg; full stud+board kit may approach the threshold depending on extras).

**Pros:** Easy DIY account, predictable next-day, store collect if a van is easier.  
**Cons:** Stud unit price often higher than trade drylining specialists.

### Travis Perkins
- Full metal-stud + Gyproc/Knauf board range; trade accounts get **trade pricing** and branch delivery perks.
- Good if you already have (or will open) a trade account.

### Trade drylining specialists (often cheapest studs)
| Supplier | Why consider | Delivery notes |
|----------|--------------|----------------|
| [Insulation Warehouse Direct](https://www.insulationwarehousedirect.co.uk/collections/drylining) | Stud/track packs, board, beads | England delivery; **£400 ex VAT minimum**, free delivery on orders |
| [Judge Ceilings](https://judge-ceilings.co.uk/dry-lining-stud-and-track) | Stud & track, board, adhesives | UK-wide; **1–3 working day** dispatch common |
| Online insulation / Fast Build style shops | Pack pricing on 70mm × 3.0m | Free next-day often above ~£50–£500 thresholds; **bulk PB may need a phone quote** by postcode |

**Indicative stud pricing (web):** trade/online ~**£3–£5** per 3.0m 70mm C-stud vs Wickes ~**£10+** — on ~25–30 lengths that is a meaningful saving if you are happy ordering packs and meeting a minimum order.

---

## 3. Mapped to the parts list

### A. Metal stud + plasterboard + finish → **merchant drop #1**

| Need (from list) | Availability | Notes |
|------------------|--------------|-------|
| 70mm C-stud + matching track | Excellent | Use **3.0m** lengths (ceiling 2380mm); cut down. Order ~**25–30 studs** + ~**6–8 tracks** + allow for noggins (or cut studs for noggins) |
| 12.5mm PB × ~8 | Excellent | Prefer **tapered edge** for taped joints |
| Corner bead ~12m, tape, joint compound | Excellent | Same drop |
| Primer / emulsion | Excellent | Bence decorating centre or Wickes/Dulux |

**Rough materials ballpark (ex labour):** stud+track+board+beads+compound often lands around **£350–£700** retail depending on supplier — enough to hit Bence’s £150 free-delivery bar and often specialist free-delivery minima.

### B. Timber lift (bed owned) → **same merchant drop**

| Need | Availability |
|------|--------------|
| Softwood bearers ~45×**riser H** × 1900 / 900 | Standard CLS / carcassing — cut to measured riser height on site |
| Packs / 20×20 legs | Easy |
| Optional shelf/head trim | PSE softwood / MDF mouldings |

Measure the **existing bed frame height** before ordering riser depth (target platform top ≈ **530mm** AFF).

### C. Drawers → **sheet from merchant + runners online**

| Need | Source |
|------|--------|
| 18mm / 12mm plywood or MDF (~2.5 + 1 sheet) | Bence / Wickes / TP — include on heavy drop |
| **900mm heavy-duty runners** (100kg class), 2 pairs | Screwfix maxes out awkwardly at shorter lengths; better: specialist e.g. GTV H53 900mm ~**£40/pair** with UK next-day style dispatch, or Häfele trade |
| Handles | Screwfix / Amazon |

Under-bed drawers are deep and heavy when full — **do not** use light kitchen 35kg runners.

### D. Curtains / opening → **retail or merchant**

| Need | Source |
|------|--------|
| Curtain track ≥1300mm | Wickes / Dunelm / Amazon — small parcel |
| Fabric | Soft furnishings retailer |

### E. Electronics & lighting → **separate parcel order (not Bence)**

| Item | Easy UK sources |
|------|-----------------|
| ESP32 + **7" touch** | Waveshare-class boards on **Amazon UK** (~£45, in stock, tracked courier) |
| WS2812B / SK6812 RGB(W) ~5m | Amazon or **The Pi Hut** (cut-to-length, better docs) |
| 5V **10A** PSU | Amazon / Pi Hut — size for strip load; inject power if running full 5m bright |
| Reading light 4000K dimmable | Lighting retailers / Amazon |
| Fibre-optic starfield kit | Specialist ceiling/starlight suppliers or Amazon kits |
| Speaker | Consumer electronics |
| Cable detector, grab adhesive, short screws | Screwfix **next-day** or on the merchant drop |

Mains to the PSU: keep as a **qualified electrician** item (per parts list) — not a DIY web order of “any” LED brick into a permanent install without checking.

---

## 4. Suggested order plan

### Option A — Near Bence (preferred if in radius)
1. Email/call Bence with the cutting summary from `PARTS-LIST.md` §8.  
2. Ask for **one crane delivery**: stud, track, PB, beads, compounds, softwood lift, plywood/MDF, grab adhesive, drywall screws, corner bead.  
3. Online: ESP32 7", LED strip, PSU, 900mm runners, curtain track, optional starfield.

### Option B — Outside Bence radius
1. **Wickes** or **Travis Perkins** for the same heavy basket (Click & Collect if driveway access is tight).  
2. *Or* get a drylining specialist quote (IWD / Judge) for stud+board if you want lower metal prices, then timber/sheet from Wickes/TP.  
3. Same online electronics/runners parcel as above.

### Option C — Max price chase
1. Specialist metal stud packs online.  
2. Board from local merchant (board is heavy; local collect often wins).  
3. Accept **2–3 deliveries** instead of one.

---

## 5. Delivery practicalities

- **Plasterboard + 3m studs** want kerbside/crane or a large estate/van — not a car boot.  
- Confirm **access** (shared drive, flats, parking permits) before booking crane.  
- Order heavy goods **before** booking labour; specialists warn not to book trades until materials are on site.  
- Keep electronics indoors on arrival (moisture).

---

## 6. What is *not* hard to get

Nothing on the current parts list is exotic or special-order for a UK build:

- 70mm metal stud systems and 12.5mm board are commodity drylining.  
- Lift timber is ordinary carcassing.  
- Drawer boxes are sheet goods + catalogue runners.  
- ESP32 7" HMIs and WS2812 strips are mainstream hobbyist stock.

The only “friction” is **logistics of bulk**, not scarcity — which is why one merchant drop (Bence when local, else Wickes/TP) plus one electronics parcel is the simplest path.
