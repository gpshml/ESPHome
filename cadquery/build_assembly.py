#!/usr/bin/env python3
"""Build cabin-bed STEP assembly from COMPONENT-3D.md / DIM / BUILD-SEQUENCE.md.

Orientation = locked front aspect: FOOT left, HEAD right; narrow pier at foot (left).
OBJ/STL export is metres + Blender Y-up (so the model is not on its side).
Do NOT invert head/foot for viewer mirroring — fix the view, not the room.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cadquery as cq

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dim import DIM  # noqa: E402
from materials import COL, MTL_HINTS  # noqa: E402
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
    """Foot return +237 mm at FRONT (LEFT). Same X footprint as existing (0..FOOT).

    Outer face x=0 flush with existing outer wall; inner face x=FOOT flush with
    the left end of the front fascia; front face Y=D flush with front wall room face.
    """
    D = DIM
    y0 = D.D - D.EXT
    stud_x = D.PB  # between PB skins
    add_vstud(assy, "FootExt-Stud-Rear", stud_x, y0 + 40, 0, D.H, color=COL["stud"])
    add_vstud(
        assy,
        "FootExt-Stud-Front",
        stud_x,
        D.D - D.STUD_FL - 40,
        0,
        D.H,
        color=COL["stud"],
    )
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
    y_a = y0 + 40 + D.STUD_FL
    y_b = D.D - D.STUD_FL - 40
    for i, z in enumerate(D.NOGGIN_ZS, start=1):
        add_box(
            assy,
            f"FootExt-Noggin-{i}",
            stud_x,
            y_a,
            z - D.NOGGIN_H / 2,
            D.STUD_W,
            y_b - y_a,
            D.NOGGIN_H,
            COL["noggin"],
        )
    add_box(assy, "FootExt-PB-Outer", 0, y0, 0, D.PB, D.EXT, D.H, COL["painted"])
    add_box(assy, "FootExt-PB-Inner", D.FOOT - D.PB, y0, 0, D.PB, D.EXT, D.H, COL["painted"])
    add_box(assy, "FootExt-PB-Front", 0, D.D - D.PB, 0, D.FOOT, D.PB, D.H, COL["painted"])


def build_rear_shelf(assy: cq.Assembly) -> None:
    """Rear shelf on back wall; stops before head fascia (RIGHT)."""
    D = DIM
    sx = D.FOOT
    sw = D.shelf_w
    add_box(assy, "Rear-Ledger", sx, 0, D.ledger_z, sw, D.LEDGER, D.LEDGER_H, COL["timber"])
    add_box(assy, "Rear-Deck", sx, 0, D.SHELF - D.DECK_T, sw, D.REAR, D.DECK_T, COL["deck"])
    fascia_y = D.REAR - D.STUD_FL
    xs = [sx + 30, sx + sw / 2 - D.STUD_W / 2, sx + sw - D.STUD_W - 30]
    for i, x in enumerate(xs, start=1):
        add_vstud(assy, f"Rear-Fascia-Stud-{i}", x, fascia_y, 0, D.fascia_h, color=COL["stud"])
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
        D.fascia_h,
        sw,
        invert_legs=True,
        color=COL["track"],
    )
    add_box(assy, "Rear-Fascia-PB", sx, D.REAR, 0, sw, D.PB, D.fascia_h, COL["painted"])
    add_box(assy, "Rear-Tablet-Block", sx + sw / 2 - 150, 0, 935, 300, 40, 80, COL["timber"])


def build_head_shelf(assy: cq.Assembly) -> None:
    """Head fascia on RIGHT. Envelope W-HEAD_W..W is fixed residual of INTERNAL_W.

    Finished bed face at head_bed_face_x; PB + studs sit inside HEAD_W toward the wall
    (do not steal bay width from the fixed 2246 clear).
    """
    D = DIM
    hx = D.head_bed_face_x  # finished bed face; HEAD_W runs hx → W
    add_box(
        assy,
        "Head-Ledger",
        D.W - D.LEDGER,
        0,
        D.ledger_z,
        D.LEDGER,
        D.FRONT_Y,
        D.LEDGER_H,
        COL["timber"],
    )
    add_box(assy, "Head-Deck", hx, 0, D.SHELF - D.DECK_T, D.HEAD_W, D.FRONT_Y, D.DECK_T, COL["deck"])
    # Studs behind bed-face PB (inside the 326mm envelope)
    stud_x = hx + D.PB
    ys = [D.REAR + 40, D.FRONT_Y / 2 - D.STUD_W / 2, D.FRONT_Y - D.STUD_W - 40]
    for i, y in enumerate(ys, start=1):
        add_vstud(
            assy,
            f"Head-Fascia-Stud-{i}",
            stud_x,
            y,
            0,
            D.fascia_h,
            web_along_x=False,
            flange_sign=1,
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
    # Bed-face PB inside HEAD_W; exposed bay only (Y=REAR..FRONT_Y)
    add_box(
        assy,
        "Head-PB-BedFace",
        hx,
        D.REAR,
        0,
        D.PB,
        D.FRONT_Y - D.REAR,
        D.fascia_h,
        COL["painted"],
    )
    add_box(assy, "Head-PB-FrontFace", hx, D.FRONT_Y, 0, D.HEAD_W, D.PB, D.fascia_h, COL["painted"])


def _front_pb_picture_frame(assy: cq.Assembly, prefix: str, pb_y: float) -> None:
    """Picture-frame PB across full internal width X=FOOT..W."""
    D = DIM
    left_w = D.ACCESS_X - D.FOOT  # narrow pier at foot (left) = FOOT_PIER
    right_x = D.ACCESS_X + D.ACCESS_W
    right_w = D.W - right_x  # wide pier at head (right)
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
    """Front wall X=FOOT..W. Narrow FOOT_PIER at FOOT (left); wide pier at HEAD (right)."""
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

    left_studs = [D.FOOT + 25, D.ACCESS_X - D.STUD_W - 25]
    right_studs = [right_x + 25, D.W - D.STUD_W - 25]
    for i, x in enumerate(left_studs, start=1):
        add_vstud(assy, f"Front-Jamb-L{i}", x, sy, 0, D.H, color=COL["stud"])
    for i, x in enumerate(right_studs, start=1):
        add_vstud(assy, f"Front-Jamb-R{i}", x, sy, 0, D.H, color=COL["stud"])

    for label, pair in (("L", left_studs), ("R", right_studs)):
        x0, x1 = pair[0] + D.STUD_W, pair[1]
        for i, z in enumerate(D.NOGGIN_ZS, start=1):
            add_noggin_x(assy, f"Front-Noggin-{label}-{i}", x0, x1, sy, z)
        for i, z in enumerate((320.0, 2230.0), start=1):
            add_noggin_x(assy, f"Front-Noggin-{label}-Band-{i}", x0, x1, sy, z, height=40.0)

    _front_pb_picture_frame(assy, "Front-PB-Room", y + D.FOOT - D.PB)
    _front_pb_picture_frame(assy, "Front-PB-Alcove", y)

    oh = D.ACCESS_TOP - D.ACCESS_BOT
    add_box(assy, "Front-Reveal-Left", D.ACCESS_X - D.PB, y, D.ACCESS_BOT, D.PB, D.FOOT, oh, COL["painted"])
    add_box(assy, "Front-Reveal-Right", right_x, y, D.ACCESS_BOT, D.PB, D.FOOT, oh, COL["painted"])
    add_box(assy, "Front-Reveal-Sill", D.ACCESS_X, y, D.ACCESS_BOT - D.PB, D.ACCESS_W, D.FOOT, D.PB, COL["painted"])
    add_box(assy, "Front-Reveal-Lintel", D.ACCESS_X, y, D.ACCESS_TOP, D.ACCESS_W, D.FOOT, D.PB, COL["painted"])


def build_bed_lift_and_bed(assy: cq.Assembly) -> None:
    D = DIM
    mx = D.FOOT + D.TOL  # pillow toward head (right)
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
    add_box(assy, "Existing-FootWall", 0, 0, 0, D.FOOT, D.D - D.EXT, D.H, COL["existing"])


def build_orientation_markers(assy: cq.Assembly) -> None:
    """Review-only cubes (not build parts) — FOOT left, HEAD right."""
    D = DIM
    add_box(assy, "Marker-FOOT", -40, D.D - 60, D.H - 80, 40, 60, 80, COL["painted"])
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


def _to_blender_y_up(x: float, y: float, z: float) -> tuple[float, float, float]:
    """CAD Z-up, Y-forward → Blender Y-up, −Z-forward: (x, y, z) → (x, z, −y)."""
    return (x, z, -y)


def export_stl(
    assy: cq.Assembly,
    path: Path,
    tolerance: float = 0.5,
    *,
    unit_scale: float = 0.001,
) -> None:
    """Export STL in metres, rotated upright for Blender default axes."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir

    comp = assembly_compound(assy)
    # Rotate −90° about X so Z-up becomes Y-up, then scale to metres
    tr = gp_Trsf()
    tr.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), -3.141592653589793 / 2.0)
    rotated = cq.Shape.cast(BRepBuilderAPI_Transform(comp.wrapped, tr, True).Shape())
    if unit_scale != 1.0:
        rotated = rotated.scale(unit_scale)
    path.parent.mkdir(parents=True, exist_ok=True)
    shape = cq.Workplane("XY").newObject([rotated])
    cq.exporters.export(
        shape,
        str(path),
        exportType="STL",
        tolerance=max(tolerance * unit_scale, 1e-4),
        angularTolerance=0.2,
    )


def _mesh_shape(shape, tolerance: float = 0.5):
    """Return triangles as list of ((x,y,z)*3,) plus flat face normals."""
    import math

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
            # Face normal (for Blender shading; avoids weird dark slices)
            ax, ay, az = pts[0]
            bx, by, bz = pts[1]
            cx, cy, cz = pts[2]
            ux, uy, uz = bx - ax, by - ay, bz - az
            vx, vy, vz = cx - ax, cy - ay, cz - az
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            normal = (nx / ln, ny / ln, nz / ln)
            tris.append((tuple(pts), normal))
    return tris


def export_obj_mtl(
    assy: cq.Assembly,
    obj_path: Path,
    tolerance: float = 0.5,
    *,
    unit_scale: float = 0.001,
) -> None:
    """Wavefront OBJ + MTL.

    Default unit_scale=0.001 writes **metres** so Blender's default clip range
    does not slice through a ~2.4 km (mm-as-units) model.
    """
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

    mesh_tol = tolerance  # OCC meshing stays in mm space
    for name, shape, color in iter_parts(assy):
        key = mat_key(color, name)
        tris = _mesh_shape(shape, tolerance=mesh_tol)
        objects.append((name, key, tris))

    with mtl_path.open("w", encoding="utf-8") as mf:
        mf.write("# Cabin bed materials — map Kd/Ns to Principled BSDF in Blender (see RENDER.md)\n")
        for key, (r, g, b) in sorted(mat_entries.items()):
            hint = MTL_HINTS.get(key, {"Ns": 20, "Ni": 1.45})
            ns = hint.get("Ns", 20)
            ni = hint.get("Ni", 1.45)
            # Specular from Ns: higher → shinier metal/paint
            ks = 0.35 if ns >= 80 else 0.12 if ns >= 25 else 0.04
            mf.write(f"newmtl {key}\n")
            mf.write(f"Kd {r:.4f} {g:.4f} {b:.4f}\n")
            mf.write(f"Ka {r * 0.03:.4f} {g * 0.03:.4f} {b * 0.03:.4f}\n")
            mf.write(f"Ks {ks:.4f} {ks:.4f} {ks:.4f}\n")
            mf.write(f"Ns {ns}\nNi {ni}\nillum 2\nd 1.0\n")
            if "roughness" in hint:
                mf.write(f"# Blender Principled: Roughness={hint['roughness']} Metallic={hint.get('metallic', 0)}\n")
            mf.write("\n")

    unit_note = "metres" if abs(unit_scale - 0.001) < 1e-12 else f"mm×{unit_scale}"
    with obj_path.open("w", encoding="utf-8") as of:
        of.write("# Cabin bed — FOOT left, HEAD right (locked front aspect)\n")
        of.write(f"# Units: {unit_note}; axes: Blender Y-up (−Z forward) from CAD Z-up\n")
        of.write("# Blender import: defaults OK (Y up). Do NOT scale again if metres.\n")
        of.write(f"mtllib {mtl_path.name}\n")
        v_index = 1
        for name, key, tris in objects:
            of.write(f"o {name}\ng {name}\nusemtl {key}\ns off\n")
            for tri, normal in tris:
                # Transform positions and normals into Blender Y-up
                btri = [
                    _to_blender_y_up(p[0] * unit_scale, p[1] * unit_scale, p[2] * unit_scale)
                    for p in tri
                ]
                bn = _to_blender_y_up(*normal)
                # Renormalise (rotation only — length preserved, but be safe)
                import math

                ln = math.sqrt(bn[0] ** 2 + bn[1] ** 2 + bn[2] ** 2) or 1.0
                bn = (bn[0] / ln, bn[1] / ln, bn[2] / ln)
                for pt in btri:
                    of.write(f"v {pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f}\n")
                for _ in range(3):
                    of.write(f"vn {bn[0]:.6f} {bn[1]:.6f} {bn[2]:.6f}\n")
                of.write(
                    f"f {v_index}//{v_index} {v_index + 1}//{v_index + 1} {v_index + 2}//{v_index + 2}\n"
                )
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
    print(f"Orientation: FOOT left (X=0), HEAD right (X={DIM.W - DIM.HEAD_W:.0f}..{DIM.W:.0f})")
    print(f"ACCESS_X={DIM.ACCESS_X:.0f} (narrow pier at foot/left)  PLATFORM={DIM.PLATFORM:.0f}")
    print("Building assembly…")
    assy = build_assembly()
    names = [name for name, _obj in assy.traverse()]
    print(f"Parts: {len(names)}")
    for n in names:
        print(f"  - {n}")

    # Width budget: INTERNAL_W fixed; HEAD_W is residual after mattress + 2×TOL
    assert abs(DIM.INTERNAL_W - 2246.0) < 0.1, DIM.INTERNAL_W
    assert abs(DIM.HEAD_W - (DIM.INTERNAL_W - DIM.MATT_W - 2 * DIM.TOL)) < 0.1, DIM.HEAD_W
    print(
        f"Width budget: INTERNAL_W={DIM.INTERNAL_W:.0f} = "
        f"MATT {DIM.MATT_W:.0f} + 2×TOL {2 * DIM.TOL:.0f} + HEAD_W {DIM.HEAD_W:.0f}"
    )

    left_pier = DIM.ACCESS_X - DIM.FOOT  # foot side
    right_pier = DIM.W - (DIM.ACCESS_X + DIM.ACCESS_W)  # head side
    assert left_pier < right_pier, "narrow pier must be at foot (left)"
    assert abs(left_pier - DIM.FOOT_PIER) < 0.1, left_pier
    print(f"Piers: foot/left={left_pier:.0f}mm  head/right={right_pier:.0f}mm")

    for name, shape, _ in iter_parts(assy):
        bb = shape.BoundingBox()
        if name == "Front-Track-Floor":
            assert abs(bb.xmin - DIM.FOOT) < 1 and abs(bb.xmax - DIM.W) < 1, (bb.xmin, bb.xmax)
        if name.startswith("FootExt-PB-Outer"):
            assert abs(bb.xmin) < 0.1 and abs(bb.xmax - DIM.PB) < 0.1
        if name.startswith("FootExt-PB-Inner"):
            assert abs(bb.xmax - DIM.FOOT) < 0.1
        if name.startswith("FootExt-PB-Front"):
            assert abs(bb.ymax - DIM.D) < 0.1
        if name.startswith("Marker-FOOT"):
            assert bb.xmax <= 0.1, name
        if name.startswith("Marker-HEAD"):
            assert bb.xmin >= DIM.W - 0.1, name
        if name in ("Rear-Ledger", "Head-Ledger"):
            assert abs(bb.zmax - (DIM.SHELF - DIM.DECK_T)) < 0.1, (name, bb.zmax)
        if name == "Head-PB-BedFace":
            assert bb.ymin >= DIM.REAR - 0.1, (name, bb.ymin)
            hx = DIM.head_bed_face_x
            assert abs(bb.xmin - hx) < 0.1, (name, bb.xmin)
            assert bb.xmax <= hx + DIM.PB + 0.1, name  # inside HEAD_W, not into bay
        if name.startswith("Head-Fascia-Stud"):
            assert bb.xmin >= DIM.head_bed_face_x + DIM.PB - 0.1, name
            assert bb.xmax <= DIM.W + 0.1, name

    if not args.no_export:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        print(f"Exporting STEP → {args.out}")
        assy.export(str(args.out))
        print(f"Wrote {args.out} ({args.out.stat().st_size:,} bytes)")
        if args.stl and str(args.stl):
            # Metres — Blender default clip range will not slice the model
            export_stl(assy, args.stl, unit_scale=0.001)
            print(f"Wrote {args.stl} ({args.stl.stat().st_size:,} bytes) [metres]")
        if args.obj and str(args.obj):
            export_obj_mtl(assy, args.obj, unit_scale=0.001)
            print(f"Wrote {args.obj} + {args.obj.with_suffix('.mtl').name} [metres]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
