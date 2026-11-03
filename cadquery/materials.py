"""Material colours for the cabin-bed assembly.

Painted plasterboard / visible wall faces → autumnal olive green.
Other parts are material-based (galvanised, timber, soft goods).
"""

from __future__ import annotations

import cadquery as cq

# sRGB 0–1
PAINTED_OLIVE = (0.42, 0.47, 0.28)  # autumnal olive green
GALVANISED = (0.62, 0.64, 0.60)
TIMBER = (0.62, 0.45, 0.28)
TIMBER_DECK = (0.72, 0.58, 0.38)
BED_FRAME = (0.40, 0.26, 0.14)
MATTRESS = (0.78, 0.74, 0.65)
DRAWER = (0.45, 0.38, 0.30)
EXISTING = (0.55, 0.42, 0.28)

COL = {
    "painted": cq.Color(*PAINTED_OLIVE),
    "stud": cq.Color(*GALVANISED),
    "track": cq.Color(0.55, 0.57, 0.54),
    "timber": cq.Color(*TIMBER),
    "deck": cq.Color(*TIMBER_DECK),
    "riser": cq.Color(*TIMBER),
    "bed": cq.Color(*BED_FRAME),
    "matt": cq.Color(*MATTRESS),
    "drawer": cq.Color(*DRAWER),
    "existing": cq.Color(*EXISTING, 0.45),
}

# Alias used by older call sites
COL["pb"] = COL["painted"]
