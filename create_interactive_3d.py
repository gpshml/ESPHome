#!/usr/bin/env python3
"""
Interactive rotatable 3D cabin bed model (Plotly).
Open the HTML in a browser — drag to rotate, scroll to zoom, right-drag to pan.
"""

import plotly.graph_objects as go

# --- Locked dimensions (mm) — same as generate_cabin_bed_FINAL_v2.py ---
MATTRESS_WIDTH = 1900
MATTRESS_DEPTH = 900
BED_TOLERANCE = 10
BED_FRAME_DEPTH = MATTRESS_DEPTH + (BED_TOLERANCE * 2)

FOOT_WALL_THICKNESS = 98
INTERNAL_WIDTH = 2246
OVERALL_WIDTH = INTERNAL_WIDTH + FOOT_WALL_THICKNESS
OVERALL_DEPTH = 1242
WALL_EXTENSION_DEPTH = 237
OVERALL_HEIGHT = 2380

PLATFORM_BASE_HEIGHT = 173
PLATFORM_FRAME_HEIGHT = 357
PLATFORM_TOP = PLATFORM_BASE_HEIGHT + PLATFORM_FRAME_HEIGHT
MATTRESS_THICKNESS = 220

REAR_SHELF_DEPTH = 210
HEAD_WALL_WIDTH = 326
SHELF_TOP_HEIGHT = 1200

ACCESS_WIDTH = 1300
ACCESS_START_FROM_LEFT = 300 + FOOT_WALL_THICKNESS
ACCESS_BOTTOM = 640
ACCESS_TOP = 2080

FRONT_WALL_Y = OVERALL_DEPTH - FOOT_WALL_THICKNESS


def box_mesh(x, y, z, w, d, h, color, opacity, name, showlegend=True):
    """Axis-aligned box as Mesh3d."""
    xs = [x, x + w, x + w, x, x, x + w, x + w, x]
    ys = [y, y, y + d, y + d, y, y, y + d, y + d]
    zs = [z, z, z, z, z + h, z + h, z + h, z + h]
    faces = [
        (0, 1, 2), (0, 2, 3),  # bottom
        (4, 5, 6), (4, 6, 7),  # top
        (0, 1, 5), (0, 5, 4),  # min-y
        (3, 2, 6), (3, 6, 7),  # max-y
        (0, 3, 7), (0, 7, 4),  # min-x
        (1, 2, 6), (1, 6, 5),  # max-x
    ]
    i, j, k = zip(*faces)
    return go.Mesh3d(
        x=xs, y=ys, z=zs,
        i=list(i), j=list(j), k=list(k),
        color=color,
        opacity=opacity,
        name=name,
        showlegend=showlegend,
        flatshading=True,
        lighting=dict(ambient=0.55, diffuse=0.8, specular=0.2),
        lightposition=dict(x=OVERALL_WIDTH, y=OVERALL_DEPTH * 2, z=OVERALL_HEIGHT * 2),
        hovertemplate=(
            f"<b>{name}</b><br>"
            f"X {x:.0f}–{x + w:.0f} mm<br>"
            f"Y {y:.0f}–{y + d:.0f} mm<br>"
            f"Z {z:.0f}–{z + h:.0f} mm"
            "<extra></extra>"
        ),
    )


def opening_outline():
    """Red line loop for access opening on front wall face."""
    x0 = ACCESS_START_FROM_LEFT
    x1 = ACCESS_START_FROM_LEFT + ACCESS_WIDTH
    y = FRONT_WALL_Y
    z0, z1 = ACCESS_BOTTOM, ACCESS_TOP
    xs = [x0, x1, x1, x0, x0]
    ys = [y, y, y, y, y]
    zs = [z0, z0, z1, z1, z0]
    return go.Scatter3d(
        x=xs, y=ys, z=zs,
        mode="lines",
        line=dict(color="red", width=6),
        name="Access opening",
        hoverinfo="name",
    )


def build_figure():
    traces = []

    # Foot wall (existing) + front extension — full height
    ext_y = OVERALL_DEPTH - WALL_EXTENSION_DEPTH
    traces.append(box_mesh(
        0, 0, 0, FOOT_WALL_THICKNESS, ext_y, OVERALL_HEIGHT,
        "#e67e22", 0.85, "Foot wall (existing)"))
    traces.append(box_mesh(
        0, ext_y, 0, FOOT_WALL_THICKNESS, WALL_EXTENSION_DEPTH, OVERALL_HEIGHT,
        "#f1c40f", 0.9, "Foot wall extension 237mm (front)"))

    # Rear shelf / wall
    traces.append(box_mesh(
        FOOT_WALL_THICKNESS, 0, 0, INTERNAL_WIDTH, REAR_SHELF_DEPTH, SHELF_TOP_HEIGHT,
        "#d4b896", 0.75, "Rear shelf / wall 210mm"))

    # Head wall — back to front wall plane
    head_x = OVERALL_WIDTH - HEAD_WALL_WIDTH
    traces.append(box_mesh(
        head_x, 0, 0, HEAD_WALL_WIDTH, FRONT_WALL_Y, SHELF_TOP_HEIGHT,
        "#bdc3c7", 0.7, "Head wall 326mm"))

    # Volume above shelf (very light)
    traces.append(box_mesh(
        FOOT_WALL_THICKNESS, 0, SHELF_TOP_HEIGHT, INTERNAL_WIDTH, OVERALL_DEPTH,
        OVERALL_HEIGHT - SHELF_TOP_HEIGHT,
        "#ecf0f1", 0.12, "Walls above shelf"))

    # Bed stack (behind front wall)
    frame_y = REAR_SHELF_DEPTH
    matt_y = REAR_SHELF_DEPTH + BED_TOLERANCE
    matt_x = FOOT_WALL_THICKNESS + BED_TOLERANCE
    traces.append(box_mesh(
        matt_x, frame_y, 0, MATTRESS_WIDTH, BED_FRAME_DEPTH, PLATFORM_BASE_HEIGHT,
        "#c4a574", 0.9, "Plinth / base 173mm"))
    traces.append(box_mesh(
        matt_x, frame_y, PLATFORM_BASE_HEIGHT, MATTRESS_WIDTH, BED_FRAME_DEPTH,
        PLATFORM_FRAME_HEIGHT,
        "#8B4513", 0.9, "Bed frame 357mm"))
    traces.append(box_mesh(
        matt_x, matt_y, PLATFORM_TOP, MATTRESS_WIDTH, MATTRESS_DEPTH, MATTRESS_THICKNESS,
        "#2ecc71", 0.95, "Mattress 1900×900×220"))

    # Front fascia — drawn conceptually "in front"; Plotly sorts by depth on rotate
    # Left / right / below / above opening
    left_w = ACCESS_START_FROM_LEFT - FOOT_WALL_THICKNESS
    right_x = ACCESS_START_FROM_LEFT + ACCESS_WIDTH
    right_w = OVERALL_WIDTH - right_x
    traces.append(box_mesh(
        FOOT_WALL_THICKNESS, FRONT_WALL_Y, 0, left_w, FOOT_WALL_THICKNESS, ACCESS_TOP,
        "#2980b9", 0.75, "Front fascia (foot side)"))
    traces.append(box_mesh(
        right_x, FRONT_WALL_Y, 0, right_w, FOOT_WALL_THICKNESS, ACCESS_TOP,
        "#2980b9", 0.75, "Front fascia (head side)"))
    traces.append(box_mesh(
        FOOT_WALL_THICKNESS, FRONT_WALL_Y, 0, INTERNAL_WIDTH, FOOT_WALL_THICKNESS,
        ACCESS_BOTTOM,
        "#3498db", 0.7, "Front fascia (below opening)", showlegend=False))
    traces.append(box_mesh(
        FOOT_WALL_THICKNESS, FRONT_WALL_Y, ACCESS_TOP, INTERNAL_WIDTH, FOOT_WALL_THICKNESS,
        OVERALL_HEIGHT - ACCESS_TOP,
        "#3498db", 0.7, "Front fascia (above opening)", showlegend=False))

    traces.append(opening_outline())

    fig = go.Figure(data=traces)

    # Camera: look into alcove from front-left so foot is on the left, head on the right
    # Plotly eye is in data coordinates (normalized-ish scene)
    fig.update_layout(
        title=dict(
            text=(
                "Cabin bed — interactive 3D<br>"
                "<sup>Drag to rotate · Scroll to zoom · Right-drag to pan · "
                "Double-click to reset · Click legend to show/hide parts</sup>"
            ),
            x=0.5,
            xanchor="center",
        ),
        scene=dict(
            xaxis=dict(title="Width (mm) — Foot → Head", range=[0, OVERALL_WIDTH],
                       backgroundcolor="rgb(245,245,245)", gridcolor="white"),
            yaxis=dict(title="Depth (mm) — Back → Front", range=[0, OVERALL_DEPTH],
                       backgroundcolor="rgb(245,245,245)", gridcolor="white"),
            zaxis=dict(title="Height (mm)", range=[0, OVERALL_HEIGHT],
                       backgroundcolor="rgb(245,245,245)", gridcolor="white"),
            aspectmode="data",  # true millimetre proportions
            camera=dict(
                eye=dict(x=-1.55, y=1.85, z=0.95),
                center=dict(x=0, y=0, z=0),
                up=dict(x=0, y=0, z=1),
            ),
        ),
        legend=dict(
            yanchor="top", y=0.98, xanchor="left", x=0.01,
            bgcolor="rgba(255,255,255,0.85)",
        ),
        margin=dict(l=0, r=0, t=80, b=0),
        height=820,
    )
    return fig


if __name__ == "__main__":
    fig = build_figure()
    import os

    # CDN version — small, opens well over http / htmlpreview
    out = "/workspace/cabin-bed-3d-interactive.html"
    fig.write_html(
        out,
        include_plotlyjs="cdn",
        full_html=True,
        config=dict(
            displayModeBar=True,
            scrollZoom=True,
            displaylogo=False,
            responsive=True,
            toImageButtonOptions=dict(
                format="png",
                filename="cabin-bed-3d",
                height=1080,
                width=1600,
                scale=2,
            ),
        ),
    )
    print(f"✓ Interactive 3D (CDN): {out} ({os.path.getsize(out)/1024:.1f} KB)")

    # Offline fallback (embedded Plotly)
    out_off = "/workspace/cabin-bed-3d-interactive-offline.html"
    fig.write_html(out_off, include_plotlyjs=True, full_html=True,
                   config=dict(displayModeBar=True, scrollZoom=True, displaylogo=False, responsive=True))
    print(f"✓ Offline bundle: {out_off} ({os.path.getsize(out_off)/1024/1024:.2f} MB)")
    print("  Prefer cabin-bed-3d-viewer.html (Three.js) or serve via http.server.")
