# Cabin Bed — Locked-In Spec (do not invent)

Last updated from user confirmation through 22 Oct 2026.

## Orientation (matches top plan — single source of truth)

| Direction | What it is |
|-----------|------------|
| **LEFT** | **FOOT** — existing wall + orange/yellow full-height foot wall (98mm) + 237mm extension at the **front** into the room |
| **RIGHT** | **HEAD** — 326mm solid fascia / head wall (not a shelf sticking out into the pillow area) |
| **BACK / REAR** | Rear wall + **210mm rear shelf**, wall structure stops at **1200mm** shelf top |
| **FRONT** | Into the room — access opening with curtains, front fascia **98mm** thick |

## Coordinate system (3D master → all 2D slices)

```
Origin (0,0,0) = BACK-LEFT-FLOOR corner (rear wall ∩ foot wall ∩ floor)
X: 0 → 2344mm   FOOT (left) → HEAD (right)
Y: 0 → 1242mm   BACK → FRONT (into room)
Z: 0 → 2380mm   FLOOR → CEILING
```

- 3D viewport must start at **0,0,0** (no negative axis padding / void behind walls).
- Looking **into** the alcove: front toward viewer, back in the far corner.
- On screen: **foot LEFT (orange/yellow), head RIGHT** — same as top plan.

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
| Mattress | 1900 × 900 × 220mm | 10mm clearance → frame opening 1920 × 920 |
| Platform top | 530mm | 173mm plinth + 357mm frame |
| Head wall / fascia | 326mm | Solid wall at head end |
| Rear shelf depth | 210mm | At back |
| Access opening | 1300 × 1440mm | Bottom **640mm** (halfway up mattress); top **2080mm** (300mm below ceiling); start 300mm from left of **internal** space |
| Front fascia thickness | 98mm | Opening surround depth |
| Foot wall height | **Full 2380mm** | It is a wall, not a shelf |
| Drawers ×2 | ~880 × 900 × 500mm | Full void under platform |

## What is NOT true (past mistakes — do not reintroduce)

1. Do **not** put a 200mm “cut-in” block under the foot of the bed.
2. Do **not** offset the mattress/bed away from the rear shelf.
3. Do **not** measure access opening from below the shelf only — use full 2380mm height.
4. Do **not** put the 237mm extension at the **rear** of the foot wall — it is at the **front**.
5. Do **not** leave empty space behind the model (−200 limits) — butt to 0.
6. Do **not** swap foot/head relative to the top plan.
7. Do **not** stop the foot wall at 1200mm — full height.
8. Do **not** make the front opening surround as deep as the 237mm extension — it is **98mm**.

## Source of truth & files

**Master model:** `cabin_bed_model.js` + `cabin-bed-3d-viewer.html` (interactive Three.js)

Static views are **captured from that model** (not drawn separately):

```bash
python3 -m http.server 8765   # in repo root
node capture_static_views.mjs # writes *_FINAL.png from Front / Top / Side / 3D presets
python3 create_final_report.py
```

| File | Role |
|------|------|
| `cabin_bed_model.js` | Geometry (mm) |
| `cabin-bed-3d-viewer.html` | Interactive viewer + view presets |
| `capture_static_views.mjs` | Headless Chrome capture |
| `cabin-bed-FINAL.html` | Static report with embedded PNGs |
| `3d_view_FINAL.png` etc. | Captured from the Three.js model |
