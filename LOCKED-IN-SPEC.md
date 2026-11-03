# Cabin Bed — Locked-In Spec (do not invent)

Last updated from user confirmation through **3 Nov 2026** (head/foot orientation corrected).

## Orientation (matches top plan — single source of truth)

| Direction | What it is |
|-----------|------------|
| **LEFT** | **HEAD** — 326mm solid fascia / head wall (carried on existing head wall; PB on exposed faces) |
| **RIGHT** | **FOOT** — existing wall + full-height foot wall (98mm) + **237mm extension at the front** into the room |
| **BACK / REAR** | Rear wall + **210mm rear shelf**, wall structure stops at **1200mm** shelf top |
| **FRONT** | Into the room — access opening with curtains, front fascia **98mm** thick |

> **Nov 2026 correction:** earlier drafts had foot on the left. Looking into the alcove from the room, **head is left, foot is right**. Do not reintroduce the old left=foot layout.

## Coordinate system (3D master → all 2D slices)

```
Origin (0,0,0) = BACK-LEFT-FLOOR corner (rear wall ∩ head wall ∩ floor)
X: 0 → 2344mm   HEAD (left) → FOOT (right)
Y: 0 → 1242mm   BACK → FRONT (into room)
Z: 0 → 2380mm   FLOOR → CEILING
```

- 3D viewport must start at **0,0,0** (no negative axis padding / void behind walls).
- Looking **into** the alcove: front toward viewer, back in the far corner.
- On screen: **head LEFT, foot RIGHT** — pillow end toward the head fascia.
- Side elevation is **from the foot** (right side): rear shelf LEFT, front wall RIGHT in that section view.

## Dimensions that are locked

| Item | Value | Notes |
|------|-------|-------|
| Internal width | 2246mm | Does **not** include foot wall |
| Foot wall thickness | 98mm | 70mm stud + plasterboard |
| Total external width | 2344mm | 2246 + 98 |
| Overall depth | 1242mm | Target alcove depth |
| Foot wall extension | **237mm at FRONT** | Extends wall **into the room**, not at the rear; not a cut-in under the bed |
| Overall height | 2380mm | Full height |
| Shelf top | 1200mm | Rear wall stops here; front/side walls continue |
| Mattress / bed | 1900 × 900 × 220mm | **Already purchased** — model clearance 10mm → bay 1920 × 920 |
| Existing bed height | **360mm** | Measured — frame only (before lift) |
| Lift / plinth | **170mm** | 530 − 360; buy risers/packs only |
| Platform top | **530mm** | Top of existing bed after lift (AFF) |
| Head wall / fascia | 326mm wide × 1144mm deep | On the **left**; carried on **existing head wall**; **PB-faced** on exposed/front faces |
| Rear shelf | 1920 × 210mm | Ledger on **existing back wall**; **PB-faced front fascia**; width = INTERNAL_W − HEAD_W (stops before foot) |
| Bed tuck clearance | 7mm frame / 17mm mattress each end | Equal front & rear inside the bay |
| Access opening | 1300 × 1440mm | Bottom **640mm**; top **2080mm**; start **300mm from left of internal** (head side) |
| Front fascia thickness | 98mm | Opening surround depth |
| Foot wall height | **Full 2380mm** | It is a wall, not a shelf |
| Drawers ×2 | ~880 × 900 × 500mm | Full void under platform |
| **Noggins** | Required | Horizontal cross-members between studs for full-height PB — see `PARTS-LIST.md` |

## What is NOT true (past mistakes — do not reintroduce)

1. Do **not** put a 200mm “cut-in” block under the foot of the bed.
2. Do **not** shove the bed flush to the rear only — keep **equal** front/rear tuck clearance for covers (centred in the bay).
3. Do **not** measure access opening from below the shelf only — use full 2380mm height.
4. Do **not** put the 237mm extension at the **rear** of the foot wall — it is at the **front**.
5. Do **not** leave empty space behind the model (−200 limits) — butt to 0.
6. Do **not** put **foot on the left** — head is left, foot is right (Nov 2026).
7. Do **not** stop the foot wall at 1200mm — full height.
8. Do **not** make the front opening surround as deep as the 237mm extension — it is **98mm**.
9. Do **not** omit noggins on full-height boarded stud walls (front piers + foot extension).

## Source of truth & files

**Master model:** `cabin_bed_model.js` + `cabin-bed-3d-viewer.html` (component assembly — studs, track, noggins, ledgers, PB, lift, bed)

Fidelity limits and CAD/BIM handoff: see **`COMPONENT-3D.md`**.

```bash
node sync_viewer.mjs
python3 -m http.server 8877
node capture_static_views.mjs
python3 create_final_report.py
pip install -r cadquery/requirements.txt && python3 cadquery/build_assembly.py
```

| File | Role |
|------|------|
| `cabin_bed_model.js` | Component geometry (mm) — `DIM` SoT |
| `cabin-bed-3d-viewer.html` | Interactive viewer |
| `COMPONENT-3D.md` | Fidelity note + CAD brief |
| `cadquery/` | CadQuery STEP / OBJ exporter |
| `PARTS-LIST.md` | Zone BOM incl. noggins |
| `COST-ESTIMATE-GL53.md` | Priced BOQ |
| `SOURCING-AND-DELIVERY.md` | Purchase / delivery plan |
| `PARTS-PURCHASE-CROSSREF.md` | Parts list ↔ purchase plan reconciliation |
