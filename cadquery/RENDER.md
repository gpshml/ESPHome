# Materials & rendering (Blender)

Mesh: `out/cabin_bed_assembly.obj` + `.mtl` (**metres**). STEP stays mm for CAD.

## Suggested material mapping

| OBJ material | Real-world | Principled BSDF |
|--------------|------------|----------------|
| **PaintedOlive** | Emulsion on 12.5mm PB | Base ≈ `#5C663D`, Roughness **0.55**, Metallic **0**, Specular **0.3** |
| **GalvanisedStud / Track** | Zinc C-stud | Base light grey, Metallic **0.85**, Roughness **0.35**, clearcoat 0.1 optional |
| **Timber / TimberDeck / BedFrame** | Softwood / ply | Roughness **0.5–0.65**; add a subtle wood texture if you want |
| **Mattress** | Linen / cotton | Roughness **0.85**, Sheen **0.4** (fabric) |
| **Drawer** | Painted ply / oak | Roughness **0.55** |
| **ExistingWall** | Existing plaster | Roughness **0.7**, slightly desaturated, opacity ~0.4 if kept as ghost |

Paint direction: **autumnal olive** on all visible PB (alcove, room face, reveals, foot extension, shelf fascias). Keep metal unfinished galvanised until after boarding.

## Quick Cycles setup

1. Import OBJ (Forward **Y**, Up **Z**) — already metres; do not scale.
2. World: **Sky Texture** (Nishita) or HDRI of a small room; strength ~0.6–1.0.
3. Add a warm area light in the alcove (~3000–3500 K) behind the front opening, and a soft fill from the room.
4. Camera: inside the room looking into the alcove, FOV ~40–50° — **FOOT left / HEAD right** (locked front aspect).
5. Render: Cycles, 128–256 samples + denoising; enable ambient occlusion in the view for a cheap check.

## EEVEE (fast preview)

Material Preview / EEVEE is enough to verify boarding and reveals. Turn on **Screen Space Reflections** and a single sun + area light.

## Opening reveal build-up

Inside the access opening (jambs, sill, lintel soffit):

1. **12.5mm TE plasterboard** cut to the 98mm wall depth  
2. **Drywall screws** into C-stud jambs and sill/lintel track (~300mm centres) — no noggins  
3. **Stop bead / corner bead** on the room and alcove edges of the reveal  
4. Tape, joint compound, primer, emulsion (olive)

## What to check visually

- Face the **front opening**: **FOOT left / HEAD right** (OBJ is X-mirrored for this); narrow pier at the foot  
- Foot extension flush with existing wall and front fascia room face  
- Ledgers tight under shelf decks (no gap); shelf fascia PB flush with deck front  
- Head bed-face PB only on the exposed bay  
- Alcove PB + reveals cover stud fronts  
- Model upright (Y-up). STEP is not X-mirrored — only OBJ/STL.  
