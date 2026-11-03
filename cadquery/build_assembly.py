#!/usr/bin/env python3
"""Build cabin-bed STEP assembly from COMPONENT-3D.md / DIM.

Usage:
  python3 cadquery/build_assembly.py
  python3 cadquery/build_assembly.py --out cadquery/out/cabin_bed_assembly.step

Named parts match PARTS-LIST.md zones (front fascia, foot ext, rear/head shelves, lift, bed).

Coords (LOCKED-IN-SPEC): origin back-left-floor; X foot→head; Y back→front; Z up.
Foot = LEFT (low X); head = RIGHT (high X).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cadquery as cq

# Allow `python3 cadquery/build_assembly.py` from repo root
sys.path.insert(0, str(Path(__file__).resolve().parent))

from dim import DIM  # noqa: E402
from materials import COL  # noqa: E402
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
    """Vertical C-stud with min-corner at (x, y, z).

    web_along_x=True  (front / foot / rear fascia): web = STUD_W in X, flange = STUD_FL in +Y
    web_along_x=False (head fascia): web = STUD_W in Y, flange = STUD_FL in +X (into head shelf)
    """
    D = DIM
    stud = c_stud(length, web=D.STUD_W, flange=D.STUD_FL, t=D.GAUGE)
    if web_along_x:
        # Local envelope: x∈[-STUD_W/2,STUD_W/2], y∈[0,STUD_FL] → shift to min (x,y,z)
        solid = stud.translate((x + D.STUD_W / 2, y, z))
    else:
        # +90° about Z: flange runs −X from origin; shift by STUD_FL so flange fills +X from x
        # Resulting envelope: x∈[x, x+STUD_FL], y∈[y, y+STUD_W]
        solid = stud.rotate((0, 0, 0), (0, 0, 1), 90).translate(
            (x + D.STUD_FL, y + D.STUD_W / 2, z)
        )
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
    """U-track running along X; web centred on y_center."""
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
    """U-track running along Y (foot extension floor/ceiling)."""
    D = DIM
    tr = u_track(length, web=D.TRACK_WEB, leg=D.TRACK_LEG, t=D.GAUGE)
    tr = tr.rotate((0, 0, 0), (0, 0, 1), 90)
    if invert_legs:
        tr = _flip_track_legs_down(tr)
        solid = tr.translate((x_center, y, z - D.TRACK_LEG))
    else:
        solid = tr.translate((x_center, y, z))
    assy.add(solid, name=name, color=color)


def build_foot_extension(assy: cq.Assembly) -> None:
    """Foot return +237 mm into the room. PB flush on stud faces (both sides + front end)."""
    D = DIM
    y0 = D.D - D.EXT
    # 70 mm studs flush to outer face (x=0); outer PB sits on that face
    stud_x = 0.0
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
    # Track centred on stud web
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
    # PB flush with studs: outer on x=0 face, inner on stud inner face (x=STUD_W)
    add_box(assy, "FootExt-PB-Outer", -D.PB, y0, 0, D.PB, D.EXT, D.H, COL["painted"])
    add_box(
        assy,
        "FootExt-PB-Inner",
        D.STUD_W,
        y0,
        0,
        D.PB,
        D.EXT,
        D.H,
        COL["painted"],
    )
    # Front end of extension — PB flush with stud framing at room face
    add_box(
        assy,
        "FootExt-PB-Front",
        0,
        D.D - D.PB,
        0,
        D.STUD_W,
        D.PB,
        D.H,
        COL["painted"],
    )


def build_rear_shelf(assy: cq.Assembly) -> None:
    D = DIM
    sw = D.shelf_w
    add_box(
        assy,
        "Rear-Ledger",
        D.FOOT,
        0,
        D.SHELF - 80,
        sw,
        D.LEDGER,
        D.LEDGER_H,
        COL["timber"],
    )
    add_box(assy, "Rear-Deck", D.FOOT, 0, D.SHELF - D.DECK_T, sw, D.REAR, D.DECK_T, COL["deck"])
    # Fascia studs behind PB (inside the 210 mm shelf)
    fascia_y = D.REAR - D.STUD_FL
    xs = [D.FOOT + 30, D.FOOT + sw / 2 - D.STUD_W / 2, D.FOOT + sw - D.STUD_W - 30]
    for i, x in enumerate(xs, start=1):
        add_vstud(assy, f"Rear-Fascia-Stud-{i}", x, fascia_y, 0, D.SHELF, color=COL["stud"])
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
    add_box(assy, "Rear-Fascia-PB", D.FOOT, D.REAR, 0, sw, D.PB, D.SHELF, COL["painted"])
    add_box(
        assy,
        "Rear-Tablet-Block",
        D.FOOT + sw / 2 - 150,
        0,
        935,
        300,
        40,
        80,
        COL["timber"],
    )


def build_head_shelf(assy: cq.Assembly) -> None:
    """Head fascia at RIGHT (high X). Studs inside shelf volume, behind bed-face PB."""
    D = DIM
    hx = D.W - D.HEAD_W
    add_box(
        assy,
        "Head-Ledger",
        D.W - D.LEDGER,
        0,
        D.SHELF - 80,
        D.LEDGER,
        D.FRONT_Y,
        D.LEDGER_H,
        COL["timber"],
    )
    add_box(assy, "Head-Deck", hx, 0, D.SHELF - D.DECK_T, D.HEAD_W, D.FRONT_Y, D.DECK_T, COL["deck"])
    # Uprights at bed face, flanges INTO the head shelf (+X) — not into the bed bay
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
    # PB on exposed faces: bed-facing (covers studs) + room-front return
    add_box(assy, "Head-PB-BedFace", hx - D.PB, 0, 0, D.PB, D.FRONT_Y, D.SHELF, COL["painted"])
    add_box(assy, "Head-PB-FrontFace", hx, D.FRONT_Y, 0, D.HEAD_W, D.PB, D.SHELF, COL["painted"])


def _front_pb_picture_frame(
    assy: cq.Assembly,
    prefix: str,
    pb_y: float,
) -> None:
    """Picture-frame PB around the access opening at a given Y (room face or alcove face)."""
    D = DIM
    left_w = D.ACCESS_X - D.FOOT
    right_x = D.ACCESS_X + D.ACCESS_W
    right_w = D.W - right_x
    add_box(assy, f"{prefix}-Below", D.FOOT, pb_y, 0, D.INTERNAL_W, D.PB, D.ACCESS_BOT, COL["painted"])
    add_box(
        assy,
        f"{prefix}-LeftPier",
        D.FOOT,
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
        D.FOOT,
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

    for i, x in enumerate([D.FOOT + 25, D.ACCESS_X - D.STUD_W - 25], start=1):
        add_vstud(assy, f"Front-Jamb-L{i}", x, sy, 0, D.H, color=COL["stud"])
    for i, x in enumerate([right_x + 25, D.W - D.STUD_W - 25], start=1):
        add_vstud(assy, f"Front-Jamb-R{i}", x, sy, 0, D.H, color=COL["stud"])

    # Room face (front) + alcove face (rear of front wall) — both picture-framed
    _front_pb_picture_frame(assy, "Front-PB-Room", y + D.FOOT - D.PB)
    _front_pb_picture_frame(assy, "Front-PB-Alcove", y)


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
    D = DIM
    add_box(assy, "Existing-FootWall", 0, 0, 0, D.FOOT, D.D - D.EXT, D.H, COL["existing"])


def build_orientation_markers(assy: cq.Assembly) -> None:
    """Small blocks so Blender view orientation is obvious: FOOT=left, HEAD=right."""
    D = DIM
    # Foot marker — low X, front corner
    add_box(assy, "Marker-FOOT", -40, D.D - 60, D.H - 80, 40, 60, 80, COL["painted"])
    # Head marker — high X, front corner
    add_box(assy, "Marker-HEAD", D.W, D.D - 60, D.H - 80, 40, 60, 80, COL["bed"])


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
    """Mesh the assembly for Blender / mesh viewers (mm)."""
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
    """Return list of triangles as ((x,y,z),)*3 in mm."""
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
    """Wavefront OBJ + MTL with material colours for Blender."""
    obj_path.parent.mkdir(parents=True, exist_ok=True)
    mtl_path = obj_path.with_suffix(".mtl")

    # Group parts by material name derived from assembly colour
    mat_entries: dict[str, tuple[float, float, float]] = {}
    objects: list[tuple[str, str, list]] = []

    def mat_key(color: cq.Color | None, name: str) -> str:
        if color is None:
            return "Default"
        # Prefer semantic names from part prefix
        lower = name.lower()
        if "marker-foot" in lower or "pb" in lower or "painted" in lower:
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
            mf.write(f"newmtl {key}\n")
            mf.write(f"Kd {r:.4f} {g:.4f} {b:.4f}\n")
            mf.write("Ka 0.05 0.05 0.05\n")
            mf.write("Ks 0.08 0.08 0.08\n")
            mf.write("Ns 20\n")
            mf.write("d 1.0\n\n")

    with obj_path.open("w", encoding="utf-8") as of:
        of.write("# Cabin bed assembly (mm) — FOOT at low X, HEAD at high X\n")
        of.write(f"mtllib {mtl_path.name}\n")
        v_index = 1
        for name, key, tris in objects:
            of.write(f"o {name}\n")
            of.write(f"usemtl {key}\n")
            for tri in tris:
                for pt in tri:
                    of.write(f"v {pt[0]:.4f} {pt[1]:.4f} {pt[2]:.4f}\n")
                of.write(f"f {v_index} {v_index + 1} {v_index + 2}\n")
                v_index += 3


def export_colored_stl_zip_style(assy: cq.Assembly, path: Path, tolerance: float = 0.5) -> None:
    """Binary STL of whole assembly (no colour) — kept for simple import."""
    export_stl(assy, path, tolerance=tolerance)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    out_dir = Path(__file__).resolve().parent / "out"
    parser.add_argument("--out", type=Path, default=out_dir / "cabin_bed_assembly.step")
    parser.add_argument("--stl", type=Path, default=out_dir / "cabin_bed_assembly.stl")
    parser.add_argument("--obj", type=Path, default=out_dir / "cabin_bed_assembly.obj")
    parser.add_argument("--no-export", action="store_true")
    args = parser.parse_args()

    print(f"DIM envelope: {DIM.W:.0f} × {DIM.D:.0f} × {DIM.H:.0f} mm")
    print(f"FRONT_Y={DIM.FRONT_Y:.0f}  shelf_w={DIM.shelf_w:.0f}  PLATFORM={DIM.PLATFORM:.0f}")
    print("Orientation: FOOT=left (X=0), HEAD=right (X=W)")
    print("Building assembly…")
    assy = build_assembly()
    names = [name for name, _obj in assy.traverse()]
    print(f"Parts: {len(names)}")
    for n in names:
        print(f"  - {n}")

    # Sanity: head studs must sit inside head shelf (x >= hx)
    hx = DIM.W - DIM.HEAD_W
    for name, shape, _ in iter_parts(assy):
        if not name.startswith("Head-Fascia-Stud"):
            continue
        bb = shape.BoundingBox()
        assert bb.xmin >= hx - 0.1, f"{name} protrudes into bed bay: xmin={bb.xmin} hx={hx}"
        assert bb.xmax <= hx + DIM.STUD_FL + 0.1, f"{name} oversize: xmax={bb.xmax}"
        print(f"OK {name} X[{bb.xmin:.1f},{bb.xmax:.1f}] (into head shelf)")

    if not args.no_export:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        print(f"Exporting STEP → {args.out}")
        assy.export(str(args.out))
        print(f"Wrote {args.out} ({args.out.stat().st_size:,} bytes)")
        if args.stl and str(args.stl):
            print(f"Exporting STL → {args.stl}")
            export_stl(assy, args.stl)
            print(f"Wrote {args.stl} ({args.stl.stat().st_size:,} bytes)")
        if args.obj and str(args.obj):
            print(f"Exporting OBJ+MTL → {args.obj}")
            export_obj_mtl(assy, args.obj)
            print(f"Wrote {args.obj} + {args.obj.with_suffix('.mtl').name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
