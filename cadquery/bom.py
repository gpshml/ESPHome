"""Derive buy quantities from the CadQuery assembly (SoT for PARTS-LIST / COST)."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

from dim import DIM


SKIP_PREFIXES = ("Existing-", "Marker-", "Mattress", "Bed-Frame", "Drawer-")


@dataclass(frozen=True)
class PartRow:
    name: str
    kind: str
    dx: float
    dy: float
    dz: float

    @property
    def cut_mm(self) -> float:
        return max(self.dx, self.dy, self.dz)

    @property
    def face_m2(self) -> float:
        dims = sorted((self.dx, self.dy, self.dz))
        return (dims[1] * dims[2]) / 1e6


def classify(name: str) -> str:
    n = name.lower()
    if name.startswith("Existing") or name.startswith("Marker"):
        return "ghost"
    if name.startswith("Mattress") or name.startswith("Bed-Frame") or name.startswith("Drawer-"):
        return "context"
    if "noggin" in n:
        return "noggin"
    if "jamb" in n or "stud" in n or "corner-post" in n:
        return "stud"
    if "track" in n:
        return "track"
    if "pb" in n or "reveal" in n:
        return "pb"
    if "ledger" in n or "tablet" in n or "riser" in n or "lift" in n or "deck" in n:
        return "timber"
    return "other"


def pack_stock(cuts: list[float], stock: float = 3000.0) -> list[dict]:
    """First-fit decreasing into fixed stock lengths."""
    pieces = sorted(cuts, reverse=True)
    bins: list[dict] = []
    for p in pieces:
        for b in bins:
            if b["rem"] >= p:
                b["cuts"].append(p)
                b["rem"] -= p
                break
        else:
            bins.append({"cuts": [p], "rem": stock - p})
    return bins


def collect_rows(assy) -> list[PartRow]:
    from build_assembly import iter_parts

    rows: list[PartRow] = []
    for name, shape, _ in iter_parts(assy):
        bb = shape.BoundingBox()
        rows.append(
            PartRow(
                name=name,
                kind=classify(name),
                dx=bb.xlen,
                dy=bb.ylen,
                dz=bb.zlen,
            )
        )
    return rows


def buy_summary(rows: list[PartRow]) -> dict:
    by = defaultdict(list)
    for r in rows:
        by[r.kind].append(r)

    stud_cuts = [r.cut_mm for r in by["stud"]]
    nog_cuts = [r.cut_mm for r in by["noggin"]]
    track_cuts = [r.cut_mm for r in by["track"]]
    metal_bins = pack_stock(stud_cuts + nog_cuts)
    track_bins = pack_stock(track_cuts)

    pb_area = sum(r.face_m2 for r in by["pb"])
    # 2400×1200 sheets — allow ~45% nesting/waste for picture-frame cuts
    pb_sheets = max(8, int(-(-pb_area // (2.4 * 1.2 * 0.55))))  # ceil with util factor; floor at 8

    lift = [r for r in by["timber"] if r.name.startswith("Lift-")]
    ledgers = [r for r in by["timber"] if "Ledger" in r.name]
    decks = [r for r in by["timber"] if "Deck" in r.name]

    return {
        "stud_count": len(by["stud"]),
        "noggin_count": len(by["noggin"]),
        "track_count": len(by["track"]),
        "pb_count": len(by["pb"]),
        "stud_m": sum(stud_cuts) / 1000,
        "noggin_m": sum(nog_cuts) / 1000,
        "track_m": sum(track_cuts) / 1000,
        "metal_sticks_needed": len(metal_bins),
        "track_sticks_needed": len(track_bins),
        "stud_buy": len(metal_bins) + 2,  # kerf / spare
        "track_buy": len(track_bins),  # already tight; no +spare unless rem large
        "pb_area_m2": pb_area,
        "pb_buy": 8 if pb_area < 14 else max(8, pb_sheets),
        "metal_bins": metal_bins,
        "track_bins": track_bins,
        "lift": lift,
        "ledgers": ledgers,
        "decks": decks,
        "by": by,
    }


def write_model_bom(assy, path: Path) -> dict:
    rows = collect_rows(assy)
    summary = buy_summary(rows)
    by = summary["by"]

    lines: list[str] = []
    lines.append("# Model BOM — exact CadQuery parts")
    lines.append("")
    lines.append(
        "Generated from `cadquery/build_assembly.py` (same assembly as STEP/OBJ). "
        "Purchase docs (`PARTS-LIST.md`, `COST-ESTIMATE-GL53.md`) must match the **buy** "
        "column below — not hand-waved zone estimates."
    )
    lines.append("")
    lines.append(
        f"**Envelope:** {DIM.W:.0f} × {DIM.D:.0f} × {DIM.H:.0f} mm · "
        f"INTERNAL_W **{DIM.INTERNAL_W:.0f}** · HEAD_W **{DIM.HEAD_W:.0f}** · "
        f"foot/left pier **{DIM.FOOT_PIER:.0f}** · head/right pier "
        f"**{DIM.W - (DIM.ACCESS_X + DIM.ACCESS_W):.0f}**."
    )
    lines.append("")
    lines.append("Excluded from buy: `Existing-*`, `Marker-*`, owned `Bed-Frame` / `Mattress` / drawer solids (sheet buy still listed).")
    lines.append("")
    lines.append("## Buy from model packing (3.0m metal stock)")
    lines.append("")
    lines.append("| Stock | Model need | Pack sticks | **Buy** | Notes |")
    lines.append("|-------|------------|-------------|---------|-------|")
    lines.append(
        f"| 70mm C-stud × 3.0m | {summary['stud_count']} uprights "
        f"({summary['stud_m']:.2f}m) + {summary['noggin_count']} noggins "
        f"({summary['noggin_m']:.2f}m) = **{summary['stud_m'] + summary['noggin_m']:.2f}m** | "
        f"**{summary['metal_sticks_needed']}** | **{summary['stud_buy']}** | "
        f"Uprights: 4 front jambs + 2 foot + 3 rear fascia + 3 head fascia + 1 corner post |"
    )
    lines.append(
        f"| 72mm U-track × 3.0m | {summary['track_count']} runs "
        f"({summary['track_m']:.2f}m) | **{summary['track_sticks_needed']}** | "
        f"**{summary['track_buy']}** | 4× front @ 2246 cannot share a stick with shelf rails |"
    )
    lines.append(
        f"| 12.5mm TE PB 2400×1200 | {summary['pb_count']} faces "
        f"({summary['pb_area_m2']:.2f} m²) | — | **{summary['pb_buy']}** | "
        f"Picture-frame nesting; 8 sheets ≈ 23 m² |"
    )
    lines.append("")
    lines.append("### Metal packing detail")
    lines.append("")
    lines.append("**C-stud sticks (uprights + noggins):**")
    for i, b in enumerate(summary["metal_bins"], 1):
        cuts = " + ".join(f"{c:.0f}" for c in b["cuts"])
        lines.append(f"- Stick {i}: {cuts} → rem {b['rem']:.0f}mm")
    lines.append("")
    lines.append("**U-track sticks:**")
    for i, b in enumerate(summary["track_bins"], 1):
        cuts = " + ".join(f"{c:.0f}" for c in b["cuts"])
        lines.append(f"- Stick {i}: {cuts} → rem {b['rem']:.0f}mm")
    lines.append("")

    def section(title: str, kind: str) -> None:
        items = sorted(by[kind], key=lambda r: r.name)
        lines.append(f"## {title} ({len(items)})")
        lines.append("")
        lines.append("| Part | Size (mm) | Cut |")
        lines.append("|------|-----------|-----|")
        for r in items:
            lines.append(
                f"| `{r.name}` | {r.dx:.0f} × {r.dy:.0f} × {r.dz:.0f} | **{r.cut_mm:.0f}** |"
            )
        lines.append("")

    section("C-stud uprights", "stud")
    section("Noggins (from C-stud stock)", "noggin")
    section("U-track", "track")
    section("Plasterboard faces", "pb")
    section("Timber (ledgers / decks / lift / tablet)", "timber")

    lines.append("## Softwood lift — exact model pieces")
    lines.append("")
    lines.append("| Part | Size (mm) | Role |")
    lines.append("|------|-----------|------|")
    for r in sorted(summary["lift"], key=lambda r: r.name):
        lines.append(
            f"| `{r.name}` | {r.dx:.0f} × {r.dy:.0f} × {r.dz:.0f} | 170mm plinth |"
        )
    lines.append("")
    lines.append(
        "Buy **5×** 47×150×2.4m and rip/pack to **170mm** height. "
        "Model: **3×** long risers along depth (~920) + **2×** crosses along mattress width (1900)."
    )
    lines.append("")
    lines.append("## Ledgers + decks")
    lines.append("")
    lines.append(
        "**Ledgers = softwood** ~45×70 PSE/CLS (not sheet). "
        "**Decks = 18mm plywood** cut from the drawer sheets."
    )
    lines.append("")
    for r in sorted(summary["ledgers"] + summary["decks"], key=lambda r: r.name):
        kind = "softwood ledger" if "Ledger" in r.name else "18mm ply deck"
        lines.append(f"- `{r.name}` ({kind}): {r.dx:.0f} × {r.dy:.0f} × {r.dz:.0f}")
    lines.append("")
    lines.append(
        "**Sheet buy:** 2× 18mm plywood 8×4 (drawer boxes + both decks) · "
        "1× 12mm plywood 8×4 (drawer bases only)."
    )
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("*Regenerate: `python3 cadquery/build_assembly.py` (writes this file).*")
    lines.append("")

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")
    return summary
