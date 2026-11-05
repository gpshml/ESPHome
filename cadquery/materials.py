"""Material colours for the cabin-bed assembly.

Painted plasterboard / visible wall faces → autumnal olive green.
Other parts are material-based (galvanised, timber, soft goods).

MTL extras (Ns / illum) are written by export_obj_mtl for Blender Principled tweaks.
"""

from __future__ import annotations

import cadquery as cq

# sRGB 0–1 — tuned for Cycles / EEVEE under warm alcove light
PAINTED_OLIVE = (0.36, 0.40, 0.24)  # autumnal olive, slightly deeper for paint
GALVANISED = (0.68, 0.70, 0.66)
TIMBER = (0.55, 0.38, 0.22)
TIMBER_DECK = (0.66, 0.52, 0.34)
BED_FRAME = (0.32, 0.20, 0.12)
MATTRESS = (0.82, 0.78, 0.70)  # warm linen
DRAWER = (0.42, 0.34, 0.26)
EXISTING = (0.50, 0.40, 0.30)

COL = {
    "painted": cq.Color(*PAINTED_OLIVE),
    "stud": cq.Color(*GALVANISED),
    "track": cq.Color(0.58, 0.60, 0.57),
    "timber": cq.Color(*TIMBER),
    "deck": cq.Color(*TIMBER_DECK),
    "riser": cq.Color(*TIMBER),
    "bed": cq.Color(*BED_FRAME),
    "matt": cq.Color(*MATTRESS),
    "drawer": cq.Color(*DRAWER),
    "existing": cq.Color(*EXISTING, 0.40),
}

COL["pb"] = COL["painted"]

# Blender / MTL hints: Kd already from COL; Ns = specular exponent; roughness ≈ guide
MTL_HINTS = {
    "PaintedOlive": {"Ns": 18, "Ni": 1.45, "roughness": 0.55, "metallic": 0.0},
    "GalvanisedStud": {"Ns": 90, "Ni": 1.5, "roughness": 0.35, "metallic": 0.85},
    "GalvanisedTrack": {"Ns": 90, "Ni": 1.5, "roughness": 0.35, "metallic": 0.85},
    "GalvanisedNoggin": {"Ns": 80, "Ni": 1.5, "roughness": 0.40, "metallic": 0.8},
    "Timber": {"Ns": 25, "Ni": 1.4, "roughness": 0.65, "metallic": 0.0},
    "TimberDeck": {"Ns": 30, "Ni": 1.4, "roughness": 0.55, "metallic": 0.0},
    "BedFrame": {"Ns": 35, "Ni": 1.4, "roughness": 0.50, "metallic": 0.0},
    "Mattress": {"Ns": 8, "Ni": 1.3, "roughness": 0.85, "metallic": 0.0},
    "Drawer": {"Ns": 28, "Ni": 1.4, "roughness": 0.55, "metallic": 0.0},
    "ExistingWall": {"Ns": 15, "Ni": 1.4, "roughness": 0.70, "metallic": 0.0},
}
