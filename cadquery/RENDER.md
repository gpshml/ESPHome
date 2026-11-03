# Materials & rendering (Blender)

Mesh: `out/cabin_bed_assembly.obj` + `.mtl` (**metres**). STEP stays mm for CAD.

## Suggested material mapping

| OBJ material | Real-world | Principled BSDF |
|--------------|------------|----------------|
| **PaintedOlive** | Emulsion on 12.5mm PB | Base ≈ `#5C663D`, Roughness **0.55**, Metallic **0**, Specular **0.3** |
| **GalvanisedStud / Track / Noggin** | Zinc C-stud | Base light grey, Metallic **0.85**, Roughness **0.35**, clearcoat 0.1 optional |
| **Timber / TimberDeck / BedFrame** | Softwood / ply | Roughness **0.5–0.65**; add a subtle wood texture if you want |
| **Mattress** | Linen / cotton | Roughness **0.85**, Sheen **0.4** (fabric) |
| **Drawer** | Painted ply / oak | Roughness **0.55** |
| **ExistingWall** | Existing plaster | Roughness **0.7**, slightly desaturated, opacity ~0.4 if kept as ghost |

Paint direction: **autumnal olive** on all visible PB (alcove, room face, reveals, foot extension, shelf fascias). Keep metal unfinished galvanised until after boarding.

## Quick Cycles setup

1. Import OBJ (Forward **Y**, Up **Z**) — already metres; do not scale.
2. World: **Sky Texture** (Nishita) or HDRI of a small room; strength ~0.6–1.0.
3. Add a warm area light in the alcove (~3000–3500 K) behind the front opening, and a soft fill from the room.
4. Camera: inside the room looking into the alcove, FOV ~40–50°, head left / foot right.
5. Render: Cycles, 128–256 samples + denoising; enable ambient occlusion in the view for a cheap check.

## EEVEE (fast preview)

Material Preview / EEVEE is enough to verify boarding and reveals. Turn on **Screen Space Reflections** and a single sun + area light.

## What to check visually

- Front wall runs **X = 0 → 2.246 m** (full internal) — flush to head, stops at foot wall  
- Alcove-side PB + opening **reveals** cover stud fronts  
- Olive paint on boarded faces; grey metal only where still exposed framing  
