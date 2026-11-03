#!/usr/bin/env python3
"""Build cabin-bed STEP assembly from COMPONENT-3D.md / DIM.

Orientation: looking into the alcove, HEAD left, FOOT right (user-corrected Nov 2026).
Includes horizontal noggins for full-height plasterboard support.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cadquery as cq

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dim import DIM  # noqa: E402
from materials import COL  # noqa: E402
from profiles import board, c_stud, place_min, u_track  # noqa: E402

# Extra material key for noggins (galvanised-ish)
COL["noggin"] = cq.Color(0.50, 0.53, 0.50)


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
    flange_sign: int = 1,
    color: cq.Color | None = None,
) -> None:
    """Vertical C-stud with min-corner logic.

    web_along_x=True: web in X, flange in +Y (front / foot / rear).
    web_along_x=False: web in Y, flange in ±X (head). flange_sign=+1 → +X; -1 → −X.
    """
    D = DIM
    stud = c_stud(length, web=D.STUD_W, flange=D.STUD_FL, t=D.GAUGE)
    if web_along_x:
        solid = stud.translate((x + D.STUD_W / 2, y, z))
    elif flange_sign >= 0:
        solid = stud.rotate((0, 0, 0), (0, 0, 1), 90).translate(
            (x + D.STUD_FL, y + D.STUD_W / 2, z)
        )
    else:
        # Flanges in −X from bed face: after +90 envelope x∈[-FL,0]; place so xmax=x
        solid = stud.rotate((0, 0, 0), (0, 0, 1), 90).translate(
            (x, y + D.STUD_W / 2, z)
        )
    assy.add(solid, name=name, color=color)


def _flip_track_legs_down(tr: cq.Workplane) -> cq.Workplane:
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
    D = DIM
    tr = u_track(length, web=D.TRACK_WEB, leg=D.TRACK_LEG, t=D.GAUGE)
    if invert_legs:
        tr = _flip_track_legs_down(tr)
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
    D = DIM
    tr = u_track(length, web=D.TRACK_WEB, leg=D.TRACK_LEG, t=D.GAUGE)
    tr = tr.rotate((0, 0, 0), (0, 0, 1), 90)
    if invert_legs:
        tr = _flip_track_legs_down(tr)
        solid = tr.translate((x_center, y, z - D.TRACK_LEG))
    else:
        solid = tr.translate((x_center, y, z))
    assy.add(solid, name=name, color=color)


def add_noggin_x(
    assy: cq.Assembly,
    name: str,
    x0: float,
    x1: float,
    y: float,
    z_mid: float,
    *,
    height: float | None = None,
    color: cq.Color | None = None,
) -> None:
    """Horizontal noggin spanning X between stud faces (PB support)."""
    D = DIM
    span = x1 - x0
    if span < 20:
        return
    h = D.NOGGIN_H if height is None else height
    add_box(assy, name, x0, y, z_mid - h / 2, span, D.STUD_FL, h, color or COL["noggin"])


def build_foot_extension(assy: cq.Assembly) -> None:
    """Foot return on the RIGHT (+237 mm). Studs flush to outer face x=W."""
    D = DIM
    y0 = D.D - D.EXT
    stud_x = D.W - D.STUD_W
    add_vstud(assy, "FootExt-Stud-Rear", stud_x, y0 + 50, 0, D.H, color=COL["stud"])
    add_vstud(
        assy,
        "FootExt-Stud-Front",
        stud_x,
        y0 + D.EXT - D.STUD_FL - 50,
        0,
        D.H,
        color=COL["stud"],
    )
    xc = stud_x + D.STUD_W / 2
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
    # Noggins between uprights (along Y)
    y_a = y0 + 50 + D.STUD_FL
    y_b = y0 + D.EXT - D.STUD_FL - 50
    y_nog = min(y_a, y_b)
    nog_d = abs(y_b - y_a)
    for i, z in enumerate(D.NOGGIN_ZS, start=1):
        add_box(
            assy,
            f"FootExt-Noggin-{i}",
            stud_x,
            y_nog,
            z - D.NOGGIN_H / 2,
            D.STUD_W,
            nog_d,
            D.NOGGIN_H,
            COL["noggin"],
        )
    add_box(assy, "FootExt-PB-Outer", D.W, y0, 0, D.PB, D.EXT, D.H, COL["painted"])
    add_box(
        assy,
        "FootExt-PB-Inner",
        stud_x - D.PB,
        y0,
        0,
        D.PB,
        D.EXT,
        D.H,
        COL["painted"],
    )
    add_box(
        assy,
        "FootExt-PB-Front",
        stud_x,
        D.D - D.PB,
        0,
        D.STUD_W,
        D.PB,
        D.H,
        COL["painted"],
    )


def build_rear_shelf(assy: cq.Assembly) -> None:
    D = DIM
    sx = D.HEAD_W
    sw = D.shelf_w
    add_box(
        assy,
        "Rear-Ledger",
        sx,
        0,
        D.SHELF - 80,
        sw,
        D.LEDGER,
        D.LEDGER_H,
        COL["timber"],
    )
    add_box(assy, "Rear-Deck", sx, 0, D.SHELF - D.DECK_T, sw, D.REAR, D.DECK_T, COL["deck"])
    fascia_y = D.REAR - D.STUD_FL
    xs = [sx + 30, sx + sw / 2 - D.STUD_W / 2, sx + sw - D.STUD_W - 30]
    for i, x in enumerate(xs, start=1):
        add_vstud(assy, f"Rear-Fascia-Stud-{i}", x, fascia_y, 0, D.SHELF, color=COL["stud"])
    for i in range(len(xs) - 1):
        for j, z in enumerate((400.0, 800.0), start=1):
            add_noggin_x(
                assy,
                f"Rear-Fascia-Noggin-{i + 1}-{j}",
                xs[i] + D.STUD_W,
                xs[i + 1],
                fascia_y,
                z,
                height=40.0,
            )
    add_track_x(
        assy,
        "Rear-Fascia-Track",
        sx,
        fascia_y + D.STUD_FL / 2,
        D.SHELF,
        sw,
        invert_legs=True,
        color=COL["track"],
    )
    add_box(assy, "Rear-Fascia-PB", sx, D.REAR, 0, sw, D.PB, D.SHELF, COL["painted"])
    add_box(
        assy,
        "Rear-Tablet-Block",
        sx + sw / 2 - 150,
        0,
        935,
        300,
        40,
        80,
        COL["timber"],
    )


def build_head_shelf(assy: cq.Assembly) -> None:
    """Head fascia on LEFT (0..HEAD_W). Studs inside shelf, flanges toward −X from bed face."""
    D = DIM
    add_box(
        assy,
        "Head-Ledger",
        0,
        0,
        D.SHELF - 80,
        D.LEDGER,
        D.FRONT_Y,
        D.LEDGER_H,
        COL["timber"],
    )
    add_box(assy, "Head-Deck", 0, 0, D.SHELF - D.DECK_T, D.HEAD_W, D.FRONT_Y, D.DECK_T, COL["deck"])
    stud_x = D.HEAD_W - D.STUD_FL  # flanges fill [HEAD_W-STUD_FL, HEAD_W]
    ys = [D.REAR + 40, D.FRONT_Y / 2 - D.STUD_W / 2, D.FRONT_Y - D.STUD_W - 40]
    for i, y in enumerate(ys, start=1):
        add_vstud(
            assy,
            f"Head-Fascia-Stud-{i}",
            D.HEAD_W,  # bed face; flange_sign=-1 places into shelf
            y,
            0,
            D.SHELF,
            web_along_x=False,
            flange_sign=-1,
            color=COL["stud"],
        )
    for i in range(len(ys) - 1):
        y0 = ys[i] + D.STUD_W
        span = ys[i + 1] - y0
        if span < 20:
            continue
        for j, z in enumerate((400.0, 800.0), start=1):
            add_box(
                assy,
                f"Head-Fascia-Noggin-{i + 1}-{j}",
                stud_x,
                y0,
                z - 20,
                D.STUD_FL,
                span,
                40,
                COL["noggin"],
            )
    add_box(assy, "Head-PB-BedFace", D.HEAD_W, 0, 0, D.PB, D.FRONT_Y, D.SHELF, COL["painted"])
    add_box(assy, "Head-PB-FrontFace", 0, D.FRONT_Y, 0, D.HEAD_W, D.PB, D.SHELF, COL["painted"])


def _front_pb_picture_frame(assy: cq.Assembly, prefix: str, pb_y: float) -> None:
    D = DIM
    left_w = D.ACCESS_X - D.HEAD_W
    right_x = D.ACCESS_X + D.ACCESS_W
    right_w = D.FOOT_X0 - right_x
    add_box(assy, f"{prefix}-Below", D.HEAD_W, pb_y, 0, D.INTERNAL_W, D.PB, D.ACCESS_BOT, COL["painted"])
    add_box(
        assy,
        f"{prefix}-LeftPier",
        D.HEAD_W,
        pb_y,
        D.ACCESS_BOT,
        left_w,
        D.PB,
        D.ACCESS_TOP - D.ACCESS_BOT,
        COL["painted"],
    )
    add_box(
        assy,
        f"{prefix}-RightPier",
        right_x,
        pb_y,
        D.ACCESS_BOT,
        right_w,
        D.PB,
        D.ACCESS_TOP - D.ACCESS_BOT,
        COL["painted"],
    )
    add_box(
        assy,
        f"{prefix}-Above",
        D.HEAD_W,
        pb_y,
        D.ACCESS_TOP,
        D.INTERNAL_W,
        D.PB,
        D.H - D.ACCESS_TOP,
        COL["painted"],
    )


def build_front_fascia(assy: cq.Assembly) -> None:
    D = DIM
    y = D.FRONT_Y
    ty = y + (D.FOOT - D.TRACK_WEB) / 2
    sy = y + (D.FOOT - D.STUD_FL) / 2
    y_track_c = ty + D.TRACK_WEB / 2
    right_x = D.ACCESS_X + D.ACCESS_W

    add_track_x(assy, "Front-Track-Floor", D.HEAD_W, y_track_c, 0, D.INTERNAL_W, color=COL["track"])
    add_track_x(
        assy,
        "Front-Track-Ceiling",
        D.HEAD_W,
        y_track_c,
        D.H,
        D.INTERNAL_W,
        invert_legs=True,
        color=COL["track"],
    )
    add_track_x(
        assy,
        "Front-Track-Sill",
        D.HEAD_W,
        y_track_c,
        D.ACCESS_BOT - D.TRACK_LEG,
        D.INTERNAL_W,
        color=COL["track"],
    )
    add_track_x(
        assy,
        "Front-Track-Lintel",
        D.HEAD_W,
        y_track_c,
        D.ACCESS_TOP,
        D.INTERNAL_W,
        color=COL["track"],
    )

    left_studs = [D.HEAD_W + 25, D.ACCESS_X - D.STUD_W - 25]
    right_studs = [right_x + 25, D.FOOT_X0 - D.STUD_W - 25]
    for i, x in enumerate(left_studs, start=1):
        add_vstud(assy, f"Front-Jamb-L{i}", x, sy, 0, D.H, color=COL["stud"])
    for i, x in enumerate(right_studs, start=1):
        add_vstud(assy, f"Front-Jamb-R{i}", x, sy, 0, D.H, color=COL["stud"])

    # Full-height + band noggins between pier studs
    for label, pair in (("L", left_studs), ("R", right_studs)):
        x0, x1 = pair[0] + D.STUD_W, pair[1]
        for i, z in enumerate(D.NOGGIN_ZS, start=1):
            add_noggin_x(assy, f"Front-Noggin-{label}-{i}", x0, x1, sy, z)
        for i, z in enumerate((320.0, 2230.0), start=1):
            add_noggin_x(assy, f"Front-Noggin-{label}-Band-{i}", x0, x1, sy, z, height=40.0)

    _front_pb_picture_frame(assy, "Front-PB-Room", y + D.FOOT - D.PB)
    _front_pb_picture_frame(assy, "Front-PB-Alcove", y)


def build_bed_lift_and_bed(assy: cq.Assembly) -> None:
    D = DIM
    mx = D.HEAD_W + D.TOL  # pillow end toward head (left)
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
    add_box(assy, "Bed-Frame", mx, fy, D.PLINTH, D.MATT_W, D.FRAME_D, D.FRAME_H, COL["bed"])
    add_box(assy, "Mattress", mx, my, D.PLATFORM, D.MATT_W, D.MATT_D, D.MATT_H, COL["matt"])
    dw = (D.MATT_W - 50) / 2
    add_box(assy, "Drawer-1", mx + 15, fy + 50, 25, dw, D.FRAME_D - 100, D.PLINTH - 45, COL["drawer"])
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
    D = DIM
    add_box(assy, "Existing-FootWall", D.FOOT_X0, 0, 0, D.FOOT, D.D - D.EXT, D.H, COL["existing"])


def build_orientation_markers(assy: cq.Assembly) -> None:
    D = DIM
    add_box(assy, "Marker-HEAD", -40, D.D - 60, D.H - 80, 40, 60, 80, COL["bed"])
    add_box(assy, "Marker-FOOT", D.W, D.D - 60, D.H - 80, 40, 60, 80, COL["painted"])


def build_assembly() -> cq.Assembly:
    assy = cq.Assembly(name="CabinBed-GL53")
    build_existing_ghost(assy)
    build_foot_extension(assy)
    build_rear_shelf(assy)
    build_head_shelf(assy)
    build_front_fascia(assy)
    build_bed_lift_and_bed(assy)
    build_orientation_markers(assy)
    return assy


def assembly_compound(assy: cq.Assembly) -> cq.Compound:
    shapes = []
    for name, obj in assy.traverse():
        if name == "CabinBed-GL53" or obj.obj is None:
            continue
        s = obj.obj
        shapes.append(s.val() if hasattr(s, "val") else s)
    return cq.Compound.makeCompound(shapes)


def iter_parts(assy: cq.Assembly):
    for name, obj in assy.traverse():
        if name == "CabinBed-GL53" or obj.obj is None:
            continue
        s = obj.obj
        shape = s.val() if hasattr(s, "val") else s
        yield name, shape, obj.color


def export_stl(assy: cq.Assembly, path: Path, tolerance: float = 0.5) -> None:
    comp = assembly_compound(assy)
    path.parent.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(
        cq.Workplane("XY").newObject([comp]),
        str(path),
        exportType="STL",
        tolerance=tolerance,
        angularTolerance=tolerance,
    )


def _mesh_shape(shape, tolerance: float = 0.5):
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_FACE, TopAbs_FORWARD
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    BRepMesh_IncrementalMesh(shape.wrapped, tolerance, False, tolerance, True).Perform()
    tris = []
    exp = TopExp_Explorer(shape.wrapped, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        loc = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation_s(face, loc)
        exp.Next()
        if triangulation is None:
            continue
        trsf = loc.Transformation()
        reverse = face.Orientation() != TopAbs_FORWARD
        for i in range(1, triangulation.NbTriangles() + 1):
            tri = triangulation.Triangle(i)
            n1, n2, n3 = tri.Get()
            if reverse:
                n1, n2 = n2, n1
            pts = []
            for n in (n1, n2, n3):
                p = triangulation.Node(n).Transformed(trsf)
                pts.append((p.X(), p.Y(), p.Z()))
            tris.append(tuple(pts))
    return tris


def export_obj_mtl(assy: cq.Assembly, obj_path: Path, tolerance: float = 0.5) -> None:
    obj_path.parent.mkdir(parents=True, exist_ok=True)
    mtl_path = obj_path.with_suffix(".mtl")
    mat_entries: dict[str, tuple[float, float, float]] = {}
    objects: list[tuple[str, str, list]] = []

    def mat_key(color: cq.Color | None, name: str) -> str:
        if color is None:
            return "Default"
        lower = name.lower()
        if "noggin" in lower:
            key = "GalvanisedNoggin"
        elif "marker-foot" in lower or "pb" in lower or "painted" in lower:
            key = "PaintedOlive"
        elif "stud" in lower or "jamb" in lower:
            key = "GalvanisedStud"
        elif "track" in lower:
            key = "GalvanisedTrack"
        elif "deck" in lower:
            key = "TimberDeck"
        elif "ledger" in lower or "tablet" in lower or "riser" in lower or "lift" in lower:
            key = "Timber"
        elif "mattress" in lower or name == "Mattress":
            key = "Mattress"
        elif "drawer" in lower:
            key = "Drawer"
        elif "bed" in lower or "marker-head" in lower:
            key = "BedFrame"
        elif "existing" in lower:
            key = "ExistingWall"
        else:
            rgba = color.toTuple()
            key = f"Mat_{abs(hash(rgba[:3])) % 10**6}"
        rgba = color.toTuple()
        mat_entries[key] = (float(rgba[0]), float(rgba[1]), float(rgba[2]))
        return key

    for name, shape, color in iter_parts(assy):
        key = mat_key(color, name)
        tris = _mesh_shape(shape, tolerance=tolerance)
        objects.append((name, key, tris))

    with mtl_path.open("w", encoding="utf-8") as mf:
        mf.write("# Cabin bed materials\n")
        for key, (r, g, b) in sorted(mat_entries.items()):
            mf.write(f"newmtl {key}\nKd {r:.4f} {g:.4f} {b:.4f}\n")
            mf.write("Ka 0.05 0.05 0.05\nKs 0.08 0.08 0.08\nNs 20\nd 1.0\n\n")

    with obj_path.open("w", encoding="utf-8") as of:
        of.write("# Cabin bed (mm) — HEAD left (low X), FOOT right (high X)\n")
        of.write(f"mtllib {mtl_path.name}\n")
        v_index = 1
        for name, key, tris in objects:
            of.write(f"o {name}\nusemtl {key}\n")
            for tri in tris:
                for pt in tri:
                    of.write(f"v {pt[0]:.4f} {pt[1]:.4f} {pt[2]:.4f}\n")
                of.write(f"f {v_index} {v_index + 1} {v_index + 2}\n")
                v_index += 3


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    out_dir = Path(__file__).resolve().parent / "out"
    parser.add_argument("--out", type=Path, default=out_dir / "cabin_bed_assembly.step")
    parser.add_argument("--stl", type=Path, default=out_dir / "cabin_bed_assembly.stl")
    parser.add_argument("--obj", type=Path, default=out_dir / "cabin_bed_assembly.obj")
    parser.add_argument("--no-export", action="store_true")
    args = parser.parse_args()

    print(f"DIM envelope: {DIM.W:.0f} × {DIM.D:.0f} × {DIM.H:.0f} mm")
    print(f"Orientation: HEAD left (X=0), FOOT right (X={DIM.FOOT_X0:.0f}..{DIM.W:.0f})")
    print(f"ACCESS_X={DIM.ACCESS_X:.0f}  shelf_w={DIM.shelf_w:.0f}  PLATFORM={DIM.PLATFORM:.0f}")
    print("Building assembly…")
    assy = build_assembly()
    names = [name for name, _obj in assy.traverse()]
    print(f"Parts: {len(names)}")
    for n in names:
        print(f"  - {n}")

    # Sanity checks
    for name, shape, _ in iter_parts(assy):
        bb = shape.BoundingBox()
        if name.startswith("FootExt-Stud"):
            assert bb.xmax <= DIM.W + 0.1 and bb.xmin >= DIM.W - DIM.STUD_W - 0.1, name
        if name.startswith("Head-Fascia-Stud"):
            assert bb.xmax <= DIM.HEAD_W + 0.1, f"{name} leaks past bed face"
            assert bb.xmin >= DIM.HEAD_W - DIM.STUD_FL - 0.1, name
        if name.startswith("Marker-HEAD"):
            assert bb.xmax <= 0.1, name
        if name.startswith("Marker-FOOT"):
            assert bb.xmin >= DIM.W - 0.1, name

    if not args.no_export:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        print(f"Exporting STEP → {args.out}")
        assy.export(str(args.out))
        print(f"Wrote {args.out} ({args.out.stat().st_size:,} bytes)")
        if args.stl and str(args.stl):
            export_stl(assy, args.stl)
            print(f"Wrote {args.stl} ({args.stl.stat().st_size:,} bytes)")
        if args.obj and str(args.obj):
            export_obj_mtl(assy, args.obj)
            print(f"Wrote {args.obj} + {args.obj.with_suffix('.mtl').name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
