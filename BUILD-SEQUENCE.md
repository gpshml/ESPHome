# Cabin bed — construction sequence

Build order for the real alcove. Use this to check the 3D assembly: every solid should match a step below.

**Orientation (locked — matches `top_plan_FINAL.png` / front aspect):** looking into the alcove from the room — **FOOT left**, **HEAD right**. The **narrow (~300 mm) access pier is at the FOOT** (left). Origin for the model = back-left-floor (rear ∩ **foot** wall ∩ floor).

> **Do not flip head/foot.** Past 3D-viewer mirroring was a display/export confusion, not a room correction. See `LOCKED-IN-SPEC.md` § Orientation.

---

## 0. Set-out

1. Confirm alcove: **2344 × 1242 × 2380** mm.
2. Mark **foot** (existing 98 mm wall, **left**) and **head** (existing side wall, **right**).
3. Snap chalk lines for front wall alcove face (**Y = 1144**), shelf top (**Z = 1200**), access opening (**1300 × 1440**, sill 640 / lintel 2080).
4. Opening set-out from the **foot**: leave a **300 mm** pier next to the foot wall (left), then 1300 mm clear opening, remainder (~646 mm) at the head (right).

---

## 1. Foot wall extension (LEFT) — full height first

Existing foot wall stays. Only the **front +237 mm** into the room is new.

1. Strip ~200 mm of existing PB where the return ties in.
2. Fix short **floor + ceiling U-track** in the extension footprint (same X as existing wall: **0 … 98**).
3. Stand **2× C-studs** between the PB skins; flanges toward the cavity.
4. Fit **noggins** at 600 / 1200 / 1800 AFF between the uprights.
5. Board **outer**, **inner**, and **front end** with 12.5 mm PB so that:
   - outer face is flush with the existing foot wall outer face;
   - inner face is flush with the **left** end of the front fascia;
   - front face is flush with the front wall **room** face (**Y = 1242**).

---

## 2. Wall ledgers (carry the shelves)

Shelves are **wall-supported** — no stud row along the back of either shelf.

1. **Rear ledger** on the existing back wall: softwood ~45×70, length **1920 mm**, under the rear shelf only (stops before the head fascia).
2. **Head ledger(s)** on the existing **head** wall (**right**): run the depth of the head fascia (~1144 mm).
3. Fix into sound structure (plugs / resin).  
4. **Critical:** ledger **top face = underside of deck** (`SHELF − DECK_T`). Ledgers must not sit in a gap below the boards.

---

## 3. Rear shelf fascia (front of the 210 mm shelf)

1. Stand **3×** light uprights (stud or timber) at the **front** of the 210 mm shelf only — not against the back wall.
2. Fit light noggins / head rail so the front can take PB.
3. Drop the **1920 × 210 × 18** deck onto the rear ledger + fascia head; top at **1200 AFF**.
4. Board the **front face only** with 12.5 mm PB (alcove-facing). No PB trapped behind the shelf against the back wall.
5. Fix tablet mount block on the back wall under the shelf (~975 AFF) — packer for the 7″ ESP32 touchscreen.

---

## 4. Head fascia / shelf (RIGHT, 326 mm)

Carried on the existing head wall. This is a solid return, not a deep “pillow shelf”.

1. Stand **2–3** fascia uprights at the **bed-facing** line only (inside the 326 mm), starting forward of the rear shelf line — **not** a stud row against the head wall.
2. Seat the head deck on the head ledger + fascia; top at **1200 AFF**.
3. Board **exposed** faces only:
   - **Bed face** PB from the **rear shelf front** forward to the front wall (do **not** run a hidden sheet back to the rear wall through the rear-shelf junction).
   - **Front return** PB on the room-facing end of the 326 mm block.
4. Do **not** board the face hard against the existing head wall (hidden / useless).

---

## 5. Front fascia (main metal-stud job)

Picture-frame wall, **98 mm** thick, spanning **foot → head** (left end meets foot wall / extension; right end meets head fascia).

1. Fix **floor + ceiling U-track** full internal run (**X = FOOT … W**).
2. Stand **2 jamb studs** in the **foot-side (narrow 300 mm) pier** and **2** in the **head-side (wide ~646 mm) pier**.
3. Fit **sill track** at 640 and **lintel track** at 2080.
4. Fit **noggins** between pier studs at 600 / 1200 / 1800 plus bands ~320 and ~2230 (wide pier is critical for PB).
5. Board **room face** and **alcove face** (picture-frame around the opening).
6. Board **opening reveals** (jambs, sill, lintel soffit) with **12.5 mm TE plasterboard**, drywall-screwed into studs / noggins / track (~300 mm centres). Stop / corner bead, tape, joint, paint (olive).

---

## 6. Bed lift, bed, drawers

1. Build timber **170 mm** lift (3 long risers + cross bearers); floor screws ≤25–30 mm after cable-detect.
2. Place owned **360 mm** bed → platform **530 AFF**.
3. Mattress; pillow toward **HEAD (right)**.
4. Fit **2** under-bed drawers.

---

## 7. First-principles checks (use against the 3D model)

| Check | Expect |
|-------|--------|
| Foot wall + extension | **Left**, 98 mm, +237 mm front, flush rules above |
| Head block | **Right**, 326 mm |
| Narrow pier | **Foot / left**, 300 mm |
| Wide pier | **Head / right**, ~646 mm |
| Ledgers | Top touching deck underside |
| Head bed-face PB | Exposed bay only — not through to rear wall behind the rear shelf |
| Front wall | Meets foot wall; meets head; both faces + reveals boarded |
| Front aspect | Looking in: **FOOT left, HEAD right** (= locked image) |

---

## Mapping to code

| Step | CadQuery / JS builders |
|------|------------------------|
| 1 | `build_foot_extension` / foot group |
| 2–4 | `build_rear_shelf`, `build_head_shelf` |
| 5 | `build_front_fascia` |
| 6 | `build_bed_lift_and_bed` |
