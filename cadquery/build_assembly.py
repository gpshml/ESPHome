#!/usr/bin/env python3
"""Build cabin-bed STEP assembly from COMPONENT-3D.md / DIM.

Usage:
  python3 cadquery/build_assembly.py
  python3 cadquery/build_assembly.py --out cadquery/out/cabin_bed_assembly.step

Named parts match PARTS-LIST.md zones (front fascia, foot ext, rear/head shelves, lift, bed).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cadquery as cq

# Allow `python3 cadquery/build_assembly.py` from repo root
sys.path.insert(0, str(Path(__file__).resolve().parent))

from dim import DIM  # noqa: E402
from profiles import board, c_stud, place_min, u_track  # noqa: E402


def add_box(
    assy: cq.Assembly,
    name: str,
    x: float,
    y: float,
    z: float,
    w: float,
    d: float,
    h: float,
    color: cq.Color | None = None,
) -> None:
    solid = place_min(board(w, d, h), x, y, z)
    assy.add(solid, name=name, color=color)


def add_vstud(
    assy: cq.Assembly,
    name: str,
    x: float,
    y: float,
    z: float,
    length: float,
    *,
    web_along_x: bool = True,
    color: cq.Color | None = None,
) -> None:
    """Vertical C-stud. web_along_x: web = STUD_W in X, flange in Y (front/foot/rear)."""
    D = DIM
    stud = c_stud(length, web=D.STUD_W, flange=D.STUD_FL, t=D.GAUGE)
    if web_along_x:
        # c_stud local: web X, flange +Y, length +Z — min of envelope at (-web/2, 0, 0)
        # Shift so min corner is (x,y,z)
        solid = stud.translate((x + D.STUD_W / 2, y, z))
    else:
        # Rotate 90° about Z so web along Y, flange along +X (head fascia)
        solid = stud.rotate((0, 0, 0), (0, 0, 1), 90).translate((x, y + D.STUD_W / 2, z))
    assy.add(solid, name=name, color=color)


def _flip_track_legs_down(tr: cq.Workplane) -> cq.Workplane:
    """Mirror floor U-track so base is at z=TRACK_LEG and legs point down to z=0."""
    D = DIM
    return tr.mirror(mirrorPlane="XY").translate((0, 0, D.TRACK_LEG))


def add_track_x(
    assy: cq.Assembly,
    name: str,
    x: float,
    y_center: float,
    z: float,
    length: float,
    *,
    invert_legs: bool = False,
    color: cq.Color | None = None,
) -> None:
    """U-track running along X; web centred on y_center.

    Floor orientation: base at z, legs up to z+TRACK_LEG.
    Ceiling (invert_legs): pass z = top_of_base; legs hang down to z-TRACK_LEG.
    """
    D = DIM
    tr = u_track(length, web=D.TRACK_WEB, leg=D.TRACK_LEG, t=D.GAUGE)
    if invert_legs:
        tr = _flip_track_legs_down(tr)
        # Local solid spans z∈[0, TRACK_LEG] with base at top; place base at `z`
        solid = tr.translate((x, y_center, z - D.TRACK_LEG))
    else:
        solid = tr.translate((x, y_center, z))
    assy.add(solid, name=name, color=color)


def add_track_y(
    assy: cq.Assembly,
    name: str,
    x_center: float,
    y: float,
    z: float,
    length: float,
    *,
    invert_legs: bool = False,
    color: cq.Color | None = None,
) -> None:
    """U-track running along Y (foot extension floor/ceiling)."""
    D = DIM
    tr = u_track(length, web=D.TRACK_WEB, leg=D.TRACK_LEG, t=D.GAUGE)
    # Rotate so length is along +Y (u_track length was +X)
    tr = tr.rotate((0, 0, 0), (0, 0, 1), 90)
    if invert_legs:
        tr = _flip_track_legs_down(tr)
        solid = tr.translate((x_center, y, z - D.TRACK_LEG))
    else:
        solid = tr.translate((x_center, y, z))
    assy.add(solid, name=name, color=color)


COL = {
    "stud": cq.Color(0.58, 0.65, 0.65),
    "track": cq.Color(0.50, 0.55, 0.55),
    "pb": cq.Color(0.93, 0.94, 0.95),
    "ledger": cq.Color(0.77, 0.65, 0.45),
    "deck": cq.Color(0.83, 0.72, 0.59),
    "riser": cq.Color(0.72, 0.58, 0.42),
    "bed": cq.Color(0.55, 0.27, 0.07),
    "matt": cq.Color(0.18, 0.80, 0.44),
    "drawer": cq.Color(0.36, 0.43, 0.49),
    "existing": cq.Color(0.90, 0.49, 0.13, 0.35),
}


def build_foot_extension(assy: cq.Assembly) -> None:
    D = DIM
    y0 = D.D - D.EXT
    # 2× full-height C-studs
    add_vstud(assy, "FootExt-Stud-Rear", (D.FOOT - D.STUD_W) / 2, y0 + 50, 0, D.H, color=COL["stud"])
    add_vstud(
        assy,
        "FootExt-Stud-Front",
        (D.FOOT - D.STUD_W) / 2,
        y0 + D.EXT - 90,
        0,
        D.H,
        color=COL["stud"],
    )
    # Floor + ceiling track (short runs along Y)
    xc = D.FOOT / 2
    add_track_y(assy, "FootExt-Track-Floor", xc, y0, 0, D.EXT, color=COL["track"])
    add_track_y(
        assy,
        "FootExt-Track-Ceiling",
        xc,
        y0,
        D.H,
        D.EXT,
        invert_legs=True,
        color=COL["track"],
    )
    # PB both faces
    add_box(assy, "FootExt-PB-Outer", -D.PB, y0, 0, D.PB, D.EXT, D.H, COL["pb"])
    add_box(assy, "FootExt-PB-Inner", D.FOOT, y0, 0, D.PB, D.EXT, D.H, COL["pb"])


def build_rear_shelf(assy: cq.Assembly) -> None:
    D = DIM
    sw = D.shelf_w
    # Wall ledger on existing back wall
    add_box(
        assy,
        "Rear-Ledger",
        D.FOOT,
        0,
        D.SHELF - 80,
        sw,
        D.LEDGER,
        D.LEDGER_H,
        COL["ledger"],
    )
    # Shelf deck
    add_box(assy, "Rear-Deck", D.FOOT, 0, D.SHELF - D.DECK_T, sw, D.REAR, D.DECK_T, COL["deck"])
    # Fascia studs at front of 210mm shelf
    fascia_y = D.REAR - D.STUD_FL
    xs = [D.FOOT + 30, D.FOOT + sw / 2 - D.STUD_W / 2, D.FOOT + sw - D.STUD_W - 30]
    for i, x in enumerate(xs, start=1):
        add_vstud(assy, f"Rear-Fascia-Stud-{i}", x, fascia_y, 0, D.SHELF, color=COL["stud"])
    # Head rail at fascia
    add_track_x(
        assy,
        "Rear-Fascia-Track",
        D.FOOT,
        fascia_y + D.STUD_FL / 2,
        D.SHELF,
        sw,
        invert_legs=True,
        color=COL["track"],
    )
    # PB front face
    add_box(assy, "Rear-Fascia-PB", D.FOOT, D.REAR, 0, sw, D.PB, D.SHELF, COL["pb"])
    # Tablet mount block ~975 AFF
    add_box(assy, "Rear-Tablet-Block", D.FOOT + sw / 2 - 150, 0, 935, 300, 40, 80, COL["ledger"])


def build_head_shelf(assy: cq.Assembly) -> None:
    D = DIM
    hx = D.W - D.HEAD_W
    # Ledger on existing head wall
    add_box(
        assy,
        "Head-Ledger",
        D.W - D.LEDGER,
        0,
        D.SHELF - 80,
        D.LEDGER,
        D.FRONT_Y,
        D.LEDGER_H,
        COL["ledger"],
    )
    add_box(assy, "Head-Deck", hx, 0, D.SHELF - D.DECK_T, D.HEAD_W, D.FRONT_Y, D.DECK_T, COL["deck"])
    # Fascia uprights — web along Y (rotated)
    ys = [D.REAR + 40, D.FRONT_Y / 2 - D.STUD_W / 2, D.FRONT_Y - D.STUD_W - 40]
    for i, y in enumerate(ys, start=1):
        add_vstud(
            assy,
            f"Head-Fascia-Stud-{i}",
            hx,
            y,
            0,
            D.SHELF,
            web_along_x=False,
            color=COL["stud"],
        )
    # PB toward bed + toward room front
    add_box(assy, "Head-PB-BedFace", hx - D.PB, 0, 0, D.PB, D.FRONT_Y, D.SHELF, COL["pb"])
    add_box(assy, "Head-PB-FrontFace", hx, D.FRONT_Y, 0, D.HEAD_W, D.PB, D.SHELF, COL["pb"])


def build_front_fascia(assy: cq.Assembly) -> None:
    D = DIM
    y = D.FRONT_Y
    ty = y + (D.FOOT - D.TRACK_WEB) / 2  # track web centre ≈ FOOT centre of fascia depth
    sy = y + (D.FOOT - D.STUD_FL) / 2
    left_w = D.ACCESS_X - D.FOOT
    right_x = D.ACCESS_X + D.ACCESS_W
    right_w = D.W - right_x
    y_track_c = ty + D.TRACK_WEB / 2

    # Floor, ceiling, sill, lintel tracks
    add_track_x(assy, "Front-Track-Floor", D.FOOT, y_track_c, 0, D.INTERNAL_W, color=COL["track"])
    add_track_x(
        assy,
        "Front-Track-Ceiling",
        D.FOOT,
        y_track_c,
        D.H,
        D.INTERNAL_W,
        invert_legs=True,
        color=COL["track"],
    )
    # Sill: track occupies [ACCESS_BOT-TRACK_LEG, ACCESS_BOT] — floor orientation
    add_track_x(
        assy,
        "Front-Track-Sill",
        D.FOOT,
        y_track_c,
        D.ACCESS_BOT - D.TRACK_LEG,
        D.INTERNAL_W,
        color=COL["track"],
    )
    add_track_x(
        assy,
        "Front-Track-Lintel",
        D.FOOT,
        y_track_c,
        D.ACCESS_TOP,
        D.INTERNAL_W,
        color=COL["track"],
    )

    # 4 full-height jamb studs
    for i, x in enumerate([D.FOOT + 25, D.ACCESS_X - D.STUD_W - 25], start=1):
        add_vstud(assy, f"Front-Jamb-L{i}", x, sy, 0, D.H, color=COL["stud"])
    for i, x in enumerate([right_x + 25, D.W - D.STUD_W - 25], start=1):
        add_vstud(assy, f"Front-Jamb-R{i}", x, sy, 0, D.H, color=COL["stud"])

    # PB picture-frame (room face)
    pb_y = y + D.FOOT - D.PB
    add_box(assy, "Front-PB-Below", D.FOOT, pb_y, 0, D.INTERNAL_W, D.PB, D.ACCESS_BOT, COL["pb"])
    add_box(
        assy,
        "Front-PB-LeftPier",
        D.FOOT,
        pb_y,
        D.ACCESS_BOT,
        left_w,
        D.PB,
        D.ACCESS_TOP - D.ACCESS_BOT,
        COL["pb"],
    )
    add_box(
        assy,
        "Front-PB-RightPier",
        right_x,
        pb_y,
        D.ACCESS_BOT,
        right_w,
        D.PB,
        D.ACCESS_TOP - D.ACCESS_BOT,
        COL["pb"],
    )
    add_box(
        assy,
        "Front-PB-Above",
        D.FOOT,
        pb_y,
        D.ACCESS_TOP,
        D.INTERNAL_W,
        D.PB,
        D.H - D.ACCESS_TOP,
        COL["pb"],
    )


def build_bed_lift_and_bed(assy: cq.Assembly) -> None:
    D = DIM
    mx = D.FOOT + D.TOL
    fy = D.REAR + D.FRAME_TUCK
    my = fy + D.TOL
    rw = 45.0
    gap = (D.MATT_W - 3 * rw) / 2
    for i in range(3):
        add_box(
            assy,
            f"Lift-Riser-Long-{i + 1}",
            mx + i * (rw + gap),
            fy,
            0,
            rw,
            D.FRAME_D,
            D.PLINTH,
            COL["riser"],
        )
    add_box(assy, "Lift-Cross-Front", mx, fy + 80, 0, D.MATT_W, 45, D.PLINTH, COL["riser"])
    add_box(
        assy,
        "Lift-Cross-Rear",
        mx,
        fy + D.FRAME_D - 125,
        0,
        D.MATT_W,
        45,
        D.PLINTH,
        COL["riser"],
    )
    # Owned bed frame (context)
    add_box(assy, "Bed-Frame", mx, fy, D.PLINTH, D.MATT_W, D.FRAME_D, D.FRAME_H, COL["bed"])
    add_box(assy, "Mattress", mx, my, D.PLATFORM, D.MATT_W, D.MATT_D, D.MATT_H, COL["matt"])
    dw = (D.MATT_W - 50) / 2
    add_box(
        assy,
        "Drawer-1",
        mx + 15,
        fy + 50,
        25,
        dw,
        D.FRAME_D - 100,
        D.PLINTH - 45,
        COL["drawer"],
    )
    add_box(
        assy,
        "Drawer-2",
        mx + dw + 35,
        fy + 50,
        25,
        dw,
        D.FRAME_D - 100,
        D.PLINTH - 45,
        COL["drawer"],
    )


def build_existing_ghost(assy: cq.Assembly) -> None:
    """Reference volumes for existing walls (not fabrication parts)."""
    D = DIM
    add_box(assy, "Existing-FootWall", 0, 0, 0, D.FOOT, D.D - D.EXT, D.H, COL["existing"])


def build_assembly() -> cq.Assembly:
    assy = cq.Assembly(name="CabinBed-GL53")
    build_existing_ghost(assy)
    build_foot_extension(assy)
    build_rear_shelf(assy)
    build_head_shelf(assy)
    build_front_fascia(assy)
    build_bed_lift_and_bed(assy)
    return assy


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent / "out" / "cabin_bed_assembly.step",
        help="Output STEP path",
    )
    parser.add_argument("--no-export", action="store_true", help="Build only; skip STEP write")
    args = parser.parse_args()

    print(f"DIM envelope: {DIM.W:.0f} × {DIM.D:.0f} × {DIM.H:.0f} mm")
    print(f"FRONT_Y={DIM.FRONT_Y:.0f}  shelf_w={DIM.shelf_w:.0f}  PLATFORM={DIM.PLATFORM:.0f}")
    print("Building assembly…")
    assy = build_assembly()
    children = list(assy.children) if hasattr(assy, "children") else []
    # CadQuery Assembly iterates named objects via traverse
    names = [name for name, _obj in assy.traverse()]
    print(f"Parts: {len(names)}")
    for n in names:
        print(f"  - {n}")

    if not args.no_export:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        print(f"Exporting STEP → {args.out}")
        # Assembly.export avoids deprecated Assembly.save
        assy.export(str(args.out))
        size = args.out.stat().st_size
        print(f"Wrote {args.out} ({size:,} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
