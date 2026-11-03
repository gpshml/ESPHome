"""Real-ish UK metal stud / track profiles for CadQuery STEP export."""

from __future__ import annotations

import cadquery as cq


def c_stud(length: float, web: float = 70.0, flange: float = 45.0, t: float = 0.7) -> cq.Workplane:
    """C-stud extruded along +Z. Outer envelope is web × flange; opening faces −Y."""
    # Outer rectangle with inner cut → C channel (open on −Y side of local frame)
    outer = cq.Workplane("XY").rect(web, flange)
    # Inner void: leave web and two flanges of thickness t; open face has no lip return
    # Simpler robust approach: three plates fused (web + two flanges)
    web_p = cq.Workplane("XY").box(web, t, length, centered=(True, False, False))
    fl_l = (
        cq.Workplane("XY")
        .transformed(offset=(-web / 2 + t / 2, 0, 0))
        .box(t, flange, length, centered=(True, False, False))
    )
    fl_r = (
        cq.Workplane("XY")
        .transformed(offset=(web / 2 - t / 2, 0, 0))
        .box(t, flange, length, centered=(True, False, False))
    )
    return web_p.union(fl_l).union(fl_r)


def u_track(length: float, web: float = 72.0, leg: float = 25.0, t: float = 0.7) -> cq.Workplane:
    """U-track extruded along +X (length). Opening faces +Z for floor track orientation.

    Local frame before placement: length along X, web across Y, legs up Z.
    """
    base = cq.Workplane("XY").box(length, web, t, centered=(False, True, False))
    leg_a = (
        cq.Workplane("XY")
        .transformed(offset=(0, -web / 2 + t / 2, 0))
        .box(length, t, leg, centered=(False, True, False))
    )
    leg_b = (
        cq.Workplane("XY")
        .transformed(offset=(0, web / 2 - t / 2, 0))
        .box(length, t, leg, centered=(False, True, False))
    )
    return base.union(leg_a).union(leg_b)


def board(w: float, d: float, h: float) -> cq.Workplane:
    """Axis-aligned board with min corner at origin (X width, Y depth, Z height)."""
    return cq.Workplane("XY").box(w, d, h, centered=(False, False, False))


def place_min(solid: cq.Workplane, x: float, y: float, z: float) -> cq.Workplane:
    """Translate a min-corner-at-origin solid to world (x,y,z)."""
    return solid.translate((x, y, z))
