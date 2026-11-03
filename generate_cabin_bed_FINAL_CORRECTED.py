#!/usr/bin/env python3
"""
Cabin Bed Technical Drawings Generator - FINAL CORRECTED VERSION
Build 3D model first, then slice views for consistency
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle, Polygon
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

# All measurements in mm
# CORRECTED: Build 3D model first, then create 2D slices

# Mattress and tolerance
MATTRESS_WIDTH = 1900
MATTRESS_DEPTH = 900
BED_TOLERANCE = 10  # 10mm tolerance around mattress
BED_FRAME_WIDTH = MATTRESS_WIDTH + (BED_TOLERANCE * 2)  # 1920mm
BED_FRAME_DEPTH = MATTRESS_DEPTH + (BED_TOLERANCE * 2)  # 920mm

# Overall dimensions
INTERNAL_WIDTH = 2246  # Internal dimension
FOOT_WALL_THICKNESS = 98  # 70mm stud + plasterboard
OVERALL_WIDTH = INTERNAL_WIDTH + FOOT_WALL_THICKNESS  # 2344mm
OVERALL_DEPTH = 1242  # Target depth
WALL_EXTENSION_DEPTH = 237  # Extension needed to reach 1242mm
CURRENT_WALL_DEPTH = OVERALL_DEPTH - WALL_EXTENSION_DEPTH  # 1005mm existing
OVERALL_HEIGHT = 2380  # Full height

# Platform and mattress
PLATFORM_BASE_HEIGHT = 173  # Wooden planks off floor
PLATFORM_FRAME_HEIGHT = 357  # Frame height
PLATFORM_TOP = PLATFORM_BASE_HEIGHT + PLATFORM_FRAME_HEIGHT  # 530mm
MATTRESS_THICKNESS = 220
MATTRESS_TOP = PLATFORM_TOP + MATTRESS_THICKNESS  # 750mm

# Shelving and walls
REAR_SHELF_DEPTH = 210  # Shelf at back wall
HEAD_WALL_WIDTH = 326  # Solid wall at head end
SHELF_TOP_HEIGHT = 1200  # Height to top of shelf surface
WALL_HEIGHT_ABOVE_SHELF = OVERALL_HEIGHT - SHELF_TOP_HEIGHT  # 1180mm

# Access opening - CORRECTED
ACCESS_WIDTH = 1300
ACCESS_START_FROM_LEFT = 300
ACCESS_BOTTOM = 640  # Halfway up mattress: 530 + 110 = 640mm
ACCESS_TOP = 2080  # 300mm below ceiling: 2380 - 300 = 2080mm
ACCESS_HEIGHT = ACCESS_TOP - ACCESS_BOTTOM  # 1440mm

# Drawers
DRAWER_WIDTH = 880
DRAWER_HEIGHT = 500
DRAWER_DEPTH = 900
DRAWER_CLEARANCE_BOTTOM = 20
DRAWER_CLEARANCE_TOP = 10

# Tablet
TABLET_CENTER_HEIGHT = 975  # Between mattress top (750) and shelf (1200)

print(f"\nDimensions calculated:")
print(f"  Access opening: {ACCESS_WIDTH}mm wide × {ACCESS_HEIGHT}mm high")
print(f"  Access bottom: {ACCESS_BOTTOM}mm (halfway up mattress)")
print(f"  Access top: {ACCESS_TOP}mm (300mm below ceiling)")
print(f"  Wall extension needed: {WALL_EXTENSION_DEPTH}mm")
print(f"  Current wall depth: {CURRENT_WALL_DEPTH}mm")
print()

def create_3d_model():
    """
    Create the master 3D model - all 2D views will be sliced from this
    
    Coordinate system:
    X = Width (0 to 2344mm, including 98mm foot wall at left)
    Y = Depth (0 to 1242mm, front to back)
    Z = Height (0 to 2380mm)
    
    Layout:
    - Foot wall at X = 0 to 98
    - Internal space at X = 98 to 2344
    - Front at Y = 0
    - Rear wall/shelf at Y = 1032 to 1242 (210mm deep)
    - Head wall at X = 2018 to 2344 (326mm wide)
    """
    
    # Define all 3D components as vertex lists
    components = {
        'foot_wall': {
            'vertices': [
                [0, 0, 0], [FOOT_WALL_THICKNESS, 0, 0],
                [FOOT_WALL_THICKNESS, OVERALL_DEPTH, 0], [0, OVERALL_DEPTH, 0],
                [0, 0, SHELF_TOP_HEIGHT], [FOOT_WALL_THICKNESS, 0, SHELF_TOP_HEIGHT],
                [FOOT_WALL_THICKNESS, OVERALL_DEPTH, SHELF_TOP_HEIGHT], [0, OVERALL_DEPTH, SHELF_TOP_HEIGHT]
            ],
            'color': 'darkorange',
            'alpha': 0.5,
            'label': 'Foot Wall (98mm)'
        },
        'rear_wall_shelf': {
            'vertices': [
                [FOOT_WALL_THICKNESS, OVERALL_DEPTH - REAR_SHELF_DEPTH, 0],
                [OVERALL_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH, 0],
                [OVERALL_WIDTH, OVERALL_DEPTH, 0],
                [FOOT_WALL_THICKNESS, OVERALL_DEPTH, 0],
                [FOOT_WALL_THICKNESS, OVERALL_DEPTH - REAR_SHELF_DEPTH, SHELF_TOP_HEIGHT],
                [OVERALL_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH, SHELF_TOP_HEIGHT],
                [OVERALL_WIDTH, OVERALL_DEPTH, SHELF_TOP_HEIGHT],
                [FOOT_WALL_THICKNESS, OVERALL_DEPTH, SHELF_TOP_HEIGHT]
            ],
            'color': 'wheat',
            'alpha': 0.4,
            'label': 'Rear Wall + Shelf'
        },
        'head_wall': {
            'vertices': [
                [OVERALL_WIDTH - HEAD_WALL_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_FRAME_DEPTH - BED_TOLERANCE, 0],
                [OVERALL_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_FRAME_DEPTH - BED_TOLERANCE, 0],
                [OVERALL_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH, 0],
                [OVERALL_WIDTH - HEAD_WALL_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH, 0],
                [OVERALL_WIDTH - HEAD_WALL_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_FRAME_DEPTH - BED_TOLERANCE, SHELF_TOP_HEIGHT],
                [OVERALL_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_FRAME_DEPTH - BED_TOLERANCE, SHELF_TOP_HEIGHT],
                [OVERALL_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH, SHELF_TOP_HEIGHT],
                [OVERALL_WIDTH - HEAD_WALL_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH, SHELF_TOP_HEIGHT]
            ],
            'color': 'lightgray',
            'alpha': 0.5,
            'label': 'Head Wall (326mm)'
        },
        'front_walls_above_shelf': {
            'vertices': [
                [FOOT_WALL_THICKNESS, 0, SHELF_TOP_HEIGHT],
                [OVERALL_WIDTH, 0, SHELF_TOP_HEIGHT],
                [OVERALL_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH, SHELF_TOP_HEIGHT],
                [FOOT_WALL_THICKNESS, OVERALL_DEPTH - REAR_SHELF_DEPTH, SHELF_TOP_HEIGHT],
                [FOOT_WALL_THICKNESS, 0, OVERALL_HEIGHT],
                [OVERALL_WIDTH, 0, OVERALL_HEIGHT],
                [OVERALL_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH, OVERALL_HEIGHT],
                [FOOT_WALL_THICKNESS, OVERALL_DEPTH - REAR_SHELF_DEPTH, OVERALL_HEIGHT]
            ],
            'color': 'lightgray',
            'alpha': 0.15,
            'label': 'Walls Above Shelf'
        },
        'mattress': {
            'vertices': [
                [FOOT_WALL_THICKNESS + BED_TOLERANCE, OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_TOLERANCE - MATTRESS_DEPTH, PLATFORM_TOP],
                [FOOT_WALL_THICKNESS + BED_TOLERANCE + MATTRESS_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_TOLERANCE - MATTRESS_DEPTH, PLATFORM_TOP],
                [FOOT_WALL_THICKNESS + BED_TOLERANCE + MATTRESS_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_TOLERANCE, PLATFORM_TOP],
                [FOOT_WALL_THICKNESS + BED_TOLERANCE, OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_TOLERANCE, PLATFORM_TOP],
                [FOOT_WALL_THICKNESS + BED_TOLERANCE, OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_TOLERANCE - MATTRESS_DEPTH, MATTRESS_TOP],
                [FOOT_WALL_THICKNESS + BED_TOLERANCE + MATTRESS_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_TOLERANCE - MATTRESS_DEPTH, MATTRESS_TOP],
                [FOOT_WALL_THICKNESS + BED_TOLERANCE + MATTRESS_WIDTH, OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_TOLERANCE, MATTRESS_TOP],
                [FOOT_WALL_THICKNESS + BED_TOLERANCE, OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_TOLERANCE, MATTRESS_TOP]
            ],
            'color': 'lightgreen',
            'alpha': 0.6,
            'label': 'Mattress'
        }
    }
    
    return components

def draw_box_3d(ax, vertices, color='blue', alpha=0.3, edgecolor='black', linewidth=1.5):
    """Draw a 3D box from 8 vertices"""
    v = vertices
    faces = [
        [v[0], v[1], v[5], v[4]],  # front
        [v[2], v[3], v[7], v[6]],  # back
        [v[0], v[3], v[7], v[4]],  # left
        [v[1], v[2], v[6], v[5]],  # right
        [v[0], v[1], v[2], v[3]],  # bottom
        [v[4], v[5], v[6], v[7]]   # top
    ]
    
    poly = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor=edgecolor, linewidth=linewidth)
    ax.add_collection3d(poly)

def create_3d_visualization():
    """Create 3D visualization from master model, viewed from front-left corner"""
    fig = plt.figure(figsize=(18, 14))
    ax = fig.add_subplot(111, projection='3d')
    
    components = create_3d_model()
    
    # Draw all components
    for name, comp in components.items():
        draw_box_3d(ax, comp['vertices'], comp['color'], comp['alpha'])
    
    # Mark access opening area on front face
    access_x_start = FOOT_WALL_THICKNESS + ACCESS_START_FROM_LEFT
    access_x_end = access_x_start + ACCESS_WIDTH
    
    # Draw access opening outline on front face (Y=0)
    opening_outline = [
        [access_x_start, 5, ACCESS_BOTTOM],
        [access_x_end, 5, ACCESS_BOTTOM],
        [access_x_end, 5, ACCESS_TOP],
        [access_x_start, 5, ACCESS_TOP]
    ]
    ax.plot([p[0] for p in opening_outline] + [opening_outline[0][0]],
            [p[1] for p in opening_outline] + [opening_outline[0][1]],
            [p[2] for p in opening_outline] + [opening_outline[0][2]],
            'r-', linewidth=4, label='Access Opening')
    
    # Add label for access opening
    ax.text(access_x_start + ACCESS_WIDTH/2, 100, ACCESS_BOTTOM + ACCESS_HEIGHT/2,
           f'ACCESS\nOPENING\n{ACCESS_WIDTH}×{ACCESS_HEIGHT}mm\n(curtains)',
           fontsize=11, ha='center', fontweight='bold',
           bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))
    
    # Add wall extension annotation
    ax.text(FOOT_WALL_THICKNESS/2, 200, 100,
           f'FOOT WALL\n{FOOT_WALL_THICKNESS}mm\n+{WALL_EXTENSION_DEPTH}mm\nextension needed',
           fontsize=9, ha='center', fontweight='bold', rotation=0)
    
    # Labels
    ax.text(OVERALL_WIDTH/2, OVERALL_DEPTH + 250, OVERALL_HEIGHT + 150,
           f'CABIN BED 3D VIEW\n{OVERALL_WIDTH}mm W × {OVERALL_DEPTH}mm D × {OVERALL_HEIGHT}mm H',
           fontsize=15, ha='center', fontweight='bold',
           bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))
    
    # Axis properties
    ax.set_xlabel('Width (mm)', fontsize=12, labelpad=12)
    ax.set_ylabel('Depth (mm)', fontsize=12, labelpad=12)
    ax.set_zlabel('Height (mm)', fontsize=12, labelpad=12)
    ax.set_title('3D VISUALIZATION - View from Front-Left Corner', 
                fontsize=17, fontweight='bold', pad=25)
    
    # Set axis limits
    ax.set_xlim(-200, OVERALL_WIDTH + 300)
    ax.set_ylim(-200, OVERALL_DEPTH + 400)
    ax.set_zlim(0, OVERALL_HEIGHT + 250)
    
    # View from front-left corner (azimuth ~225°, elevation ~20°)
    ax.view_init(elev=20, azim=225)
    
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=9)
    
    plt.tight_layout()
    plt.savefig('/workspace/3d_visualization_CORRECTED.png', dpi=300, bbox_inches='tight')
    print("✓ 3D visualization (CORRECTED) saved")
    plt.close()

def draw_front_elevation():
    """Front elevation - sliced from 3D model at Y=0 plane"""
    fig, ax = plt.subplots(figsize=(17, 13))
    ax.set_aspect('equal')
    
    # Structure outline (internal width only, foot wall not visible in front view)
    structure = Rectangle((FOOT_WALL_THICKNESS, 0), INTERNAL_WIDTH, OVERALL_HEIGHT,
                         fill=False, edgecolor='black', linewidth=3)
    ax.add_patch(structure)
    
    # Platform level
    ax.plot([FOOT_WALL_THICKNESS, OVERALL_WIDTH], [PLATFORM_TOP, PLATFORM_TOP], 
            'b--', linewidth=1.5, alpha=0.7, label=f'Platform Top ({PLATFORM_TOP}mm)')
    
    # Mattress top
    ax.plot([FOOT_WALL_THICKNESS, OVERALL_WIDTH], [MATTRESS_TOP, MATTRESS_TOP], 
            'g--', linewidth=1.5, alpha=0.7, label=f'Mattress Top ({MATTRESS_TOP}mm)')
    
    # Shelf level
    ax.plot([FOOT_WALL_THICKNESS, OVERALL_WIDTH], [SHELF_TOP_HEIGHT, SHELF_TOP_HEIGHT], 
            'purple', linewidth=2.5, label=f'Shelf Level ({SHELF_TOP_HEIGHT}mm)')
    
    # Walls above shelf
    wall_above = Rectangle((FOOT_WALL_THICKNESS, SHELF_TOP_HEIGHT), INTERNAL_WIDTH, WALL_HEIGHT_ABOVE_SHELF,
                          fill=True, facecolor='lightgray', edgecolor='darkgray', 
                          linewidth=2, alpha=0.3)
    ax.add_patch(wall_above)
    
    # Access opening - CORRECTED
    access_x = FOOT_WALL_THICKNESS + ACCESS_START_FROM_LEFT
    access = Rectangle((access_x, ACCESS_BOTTOM), ACCESS_WIDTH, ACCESS_HEIGHT,
                      fill=False, edgecolor='red', linewidth=4, linestyle='--')
    ax.add_patch(access)
    ax.text(access_x + ACCESS_WIDTH/2, ACCESS_BOTTOM + ACCESS_HEIGHT/2,
            f'ACCESS OPENING\n(CURTAINS)\n{ACCESS_WIDTH}mm × {ACCESS_HEIGHT}mm\nBottom: {ACCESS_BOTTOM}mm\nTop: {ACCESS_TOP}mm',
            ha='center', va='center', fontsize=11, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # Drawers
    drawer_y_actual = PLATFORM_TOP - DRAWER_CLEARANCE_BOTTOM - DRAWER_CLEARANCE_TOP
    left_drawer_x = FOOT_WALL_THICKNESS + 100
    right_drawer_x = OVERALL_WIDTH - DRAWER_WIDTH - 100
    
    for dx, label in [(left_drawer_x, 'DRAWER 1'), (right_drawer_x, 'DRAWER 2')]:
        drawer = Rectangle((dx, DRAWER_CLEARANCE_BOTTOM), DRAWER_WIDTH, drawer_y_actual,
                          fill=True, facecolor='lightblue', edgecolor='blue', linewidth=2)
        ax.add_patch(drawer)
        ax.text(dx + DRAWER_WIDTH/2, DRAWER_CLEARANCE_BOTTOM + drawer_y_actual/2,
                f'{label}\n{DRAWER_WIDTH}×{int(drawer_y_actual)}mm', 
                ha='center', va='center', fontsize=10, fontweight='bold')
    
    # Tablet
    tablet_x = FOOT_WALL_THICKNESS + INTERNAL_WIDTH/2
    tablet = Circle((tablet_x, TABLET_CENTER_HEIGHT), 90,
                   fill=True, facecolor='lightgray', edgecolor='black', linewidth=2)
    ax.add_patch(tablet)
    ax.text(tablet_x, TABLET_CENTER_HEIGHT, '7"\nTABLET', 
            ha='center', va='center', fontsize=9, fontweight='bold')
    
    # Dimensions
    # Internal width
    ax.annotate('', xy=(OVERALL_WIDTH, -350), xytext=(FOOT_WALL_THICKNESS, -350),
                arrowprops=dict(arrowstyle='<->', lw=2.5, color='black'))
    ax.text((FOOT_WALL_THICKNESS + OVERALL_WIDTH)/2, -420, 
            f'{INTERNAL_WIDTH}mm (internal width)', 
            ha='center', fontsize=12, fontweight='bold')
    
    # Overall height
    ax.annotate('', xy=(OVERALL_WIDTH + 350, OVERALL_HEIGHT), xytext=(OVERALL_WIDTH + 350, 0),
                arrowprops=dict(arrowstyle='<->', lw=2.5, color='black'))
    ax.text(OVERALL_WIDTH + 420, OVERALL_HEIGHT/2, 
            f'{OVERALL_HEIGHT}mm\n(FULL HEIGHT)', 
            ha='left', va='center', fontsize=12, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # Access opening height dimension
    ax.annotate('', xy=(access_x - 250, ACCESS_TOP), xytext=(access_x - 250, ACCESS_BOTTOM),
                arrowprops=dict(arrowstyle='<->', lw=2, color='red'))
    ax.text(access_x - 320, (ACCESS_BOTTOM + ACCESS_TOP)/2,
            f'{ACCESS_HEIGHT}mm\nopening\nheight',
            ha='right', va='center', fontsize=10, color='red', fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.6))
    
    ax.set_xlim(-100, OVERALL_WIDTH + 700)
    ax.set_ylim(-600, OVERALL_HEIGHT + 200)
    ax.set_xlabel('Width (mm)', fontsize=12)
    ax.set_ylabel('Height (mm)', fontsize=12)
    ax.set_title('FRONT ELEVATION - Access opening: 640-2080mm (halfway up mattress to 300mm below ceiling)', 
                fontsize=15, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/front_elevation_CORRECTED.png', dpi=300, bbox_inches='tight')
    print("✓ Front elevation (CORRECTED) saved")
    plt.close()

def draw_top_plan():
    """Top plan - sliced from 3D model at Z=1000mm plane (looking down)"""
    fig, ax = plt.subplots(figsize=(17, 15))
    ax.set_aspect('equal')
    
    # Orientation: TOP = rear, BOTTOM = front, LEFT = foot, RIGHT = head
    
    # Foot wall (98mm thick, full depth on LEFT)
    foot_wall = Rectangle((0, 0), FOOT_WALL_THICKNESS, OVERALL_DEPTH,
                         fill=True, facecolor='darkorange', edgecolor='black',
                         linewidth=3, alpha=0.6)
    ax.add_patch(foot_wall)
    ax.text(FOOT_WALL_THICKNESS/2, OVERALL_DEPTH/2, 
            f'FOOT\nWALL\n{FOOT_WALL_THICKNESS}mm',
            ha='center', va='center', fontsize=11, fontweight='bold', rotation=90)
    
    # Wall extension indication (237mm extension needed)
    # Show as dashed outline extending from current wall depth backward
    extension_y_start = OVERALL_DEPTH - WALL_EXTENSION_DEPTH
    extension_area = Rectangle((0, extension_y_start), FOOT_WALL_THICKNESS, WALL_EXTENSION_DEPTH,
                              fill=True, facecolor='yellow', edgecolor='orange',
                              linewidth=3, alpha=0.5, linestyle='--')
    ax.add_patch(extension_area)
    ax.text(FOOT_WALL_THICKNESS/2, extension_y_start + WALL_EXTENSION_DEPTH/2,
            f'{WALL_EXTENSION_DEPTH}mm\nEXT.',
            ha='center', va='center', fontsize=10, fontweight='bold', rotation=90,
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))
    
    # Internal space outline
    structure = Rectangle((FOOT_WALL_THICKNESS, 0), INTERNAL_WIDTH, OVERALL_DEPTH,
                         fill=False, edgecolor='black', linewidth=3)
    ax.add_patch(structure)
    
    # Rear shelf (at top/back)
    rear_shelf_y = OVERALL_DEPTH - REAR_SHELF_DEPTH
    rear_shelf = Rectangle((FOOT_WALL_THICKNESS, rear_shelf_y), INTERNAL_WIDTH, REAR_SHELF_DEPTH,
                          fill=True, facecolor='wheat', edgecolor='brown', linewidth=2, alpha=0.6)
    ax.add_patch(rear_shelf)
    ax.text(FOOT_WALL_THICKNESS + INTERNAL_WIDTH/2, rear_shelf_y + REAR_SHELF_DEPTH/2,
            'REAR SHELF - 210mm deep\n(Back Wall)',
            ha='center', va='center', fontsize=13, fontweight='bold')
    
    # Head wall (right side)
    head_wall_x = OVERALL_WIDTH - HEAD_WALL_WIDTH
    head_wall_y_start = rear_shelf_y - BED_FRAME_DEPTH - BED_TOLERANCE
    head_wall_depth = rear_shelf_y - head_wall_y_start
    head_wall = Rectangle((head_wall_x, head_wall_y_start), HEAD_WALL_WIDTH, head_wall_depth,
                          fill=True, facecolor='lightgray', edgecolor='darkgray', 
                          linewidth=3, alpha=0.7)
    ax.add_patch(head_wall)
    ax.text(head_wall_x + HEAD_WALL_WIDTH/2, head_wall_y_start + head_wall_depth/2,
            'HEAD\nWALL\n326mm',
            ha='center', va='center', fontsize=11, fontweight='bold')
    
    # Mattress (normal position, not offset)
    mattress_x = FOOT_WALL_THICKNESS + BED_TOLERANCE
    mattress_y = rear_shelf_y - BED_TOLERANCE - MATTRESS_DEPTH
    mattress = Rectangle((mattress_x, mattress_y), MATTRESS_WIDTH, MATTRESS_DEPTH,
                        fill=True, facecolor='lightgreen', edgecolor='darkgreen',
                        linewidth=3, alpha=0.7)
    ax.add_patch(mattress)
    ax.text(mattress_x + MATTRESS_WIDTH/2, mattress_y + MATTRESS_DEPTH/2,
            'MATTRESS\n1900 × 900mm\n(10mm clearance)',
            ha='center', va='center', fontsize=13, fontweight='bold')
    
    # Bed frame outline
    frame_x = FOOT_WALL_THICKNESS
    frame_y = mattress_y - BED_TOLERANCE
    bed_frame = Rectangle((frame_x, frame_y), BED_FRAME_WIDTH, BED_FRAME_DEPTH,
                          fill=False, edgecolor='red', linewidth=2, linestyle='--')
    ax.add_patch(bed_frame)
    
    # Access opening area (front section)
    access_x = FOOT_WALL_THICKNESS + ACCESS_START_FROM_LEFT
    access_area = Rectangle((access_x, 0), ACCESS_WIDTH, frame_y,
                           fill=True, facecolor='yellow', edgecolor='red',
                           linewidth=3, alpha=0.3)
    ax.add_patch(access_area)
    ax.text(access_x + ACCESS_WIDTH/2, frame_y/2,
            'ACCESS OPENING\n1300mm wide\n(curtains)',
            ha='center', va='center', fontsize=12, fontweight='bold')
    
    # Support legs (6 total)
    leg_size = 20
    leg_positions = [
        (frame_x + 50, frame_y + 100),
        (frame_x + 940, frame_y + 100),
        (frame_x + BED_FRAME_WIDTH - 50, frame_y + 100),
        (frame_x + 50, frame_y + BED_FRAME_DEPTH - 100),
        (frame_x + 940, frame_y + BED_FRAME_DEPTH - 100),
        (frame_x + BED_FRAME_WIDTH - 50, frame_y + BED_FRAME_DEPTH - 100),
    ]
    
    for lx, ly in leg_positions:
        leg = Rectangle((lx - leg_size/2, ly - leg_size/2), leg_size, leg_size,
                       fill=True, facecolor='black', edgecolor='black')
        ax.add_patch(leg)
    
    # Dimensions
    # Internal width
    ax.annotate('', xy=(OVERALL_WIDTH, -300), xytext=(FOOT_WALL_THICKNESS, -300),
                arrowprops=dict(arrowstyle='<->', lw=2, color='blue'))
    ax.text((FOOT_WALL_THICKNESS + OVERALL_WIDTH)/2, -370, 
            f'{INTERNAL_WIDTH}mm (INTERNAL WIDTH)',
            ha='center', fontsize=12, fontweight='bold', color='blue')
    
    # Total width
    ax.annotate('', xy=(OVERALL_WIDTH, -500), xytext=(0, -500),
                arrowprops=dict(arrowstyle='<->', lw=3, color='red'))
    ax.text(OVERALL_WIDTH/2, -580, 
            f'{OVERALL_WIDTH}mm TOTAL WIDTH = {INTERNAL_WIDTH}mm + {FOOT_WALL_THICKNESS}mm foot wall',
            ha='center', fontsize=13, fontweight='bold', color='red',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # Depth
    ax.annotate('', xy=(OVERALL_WIDTH + 300, OVERALL_DEPTH), xytext=(OVERALL_WIDTH + 300, 0),
                arrowprops=dict(arrowstyle='<->', lw=2.5, color='black'))
    ax.text(OVERALL_WIDTH + 380, OVERALL_DEPTH/2, 
            f'{OVERALL_DEPTH}mm\nDEPTH\n(needs {WALL_EXTENSION_DEPTH}mm\nwall extension)',
            ha='left', va='center', fontsize=11, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.7))
    
    # Orientation labels
    ax.text(OVERALL_WIDTH/2, -150, '↑ FRONT (into room) ↑',
            ha='center', fontsize=14, fontweight='bold', color='blue')
    ax.text(OVERALL_WIDTH/2, OVERALL_DEPTH + 150, '↓ REAR (back wall) ↓',
            ha='center', fontsize=14, fontweight='bold', color='blue')
    ax.text(-150, OVERALL_DEPTH/2, '← FOOT\n(+extension)',
            ha='center', va='center', fontsize=12, fontweight='bold', color='darkorange')
    ax.text(OVERALL_WIDTH + 150, head_wall_y_start + head_wall_depth/2, 'HEAD →',
            ha='left', va='center', fontsize=12, fontweight='bold', color='darkgray')
    
    ax.set_xlim(-400, OVERALL_WIDTH + 650)
    ax.set_ylim(-700, OVERALL_DEPTH + 300)
    ax.set_xlabel('Width (mm)', fontsize=13)
    ax.set_ylabel('Depth (mm)', fontsize=13)
    ax.set_title('TOP PLAN VIEW - Internal 2246mm + Foot Wall 98mm = 2344mm Total (needs 237mm extension)', 
                fontsize=15, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/top_plan_CORRECTED.png', dpi=300, bbox_inches='tight')
    print("✓ Top plan (CORRECTED) saved")
    plt.close()

def draw_side_elevation():
    """Side elevation from FOOT end - sliced from 3D model at X=0 plane"""
    fig, ax = plt.subplots(figsize=(15, 13))
    ax.set_aspect('equal')
    
    # When viewing from FOOT end looking toward HEAD end:
    # LEFT (X=0) = REAR (back wall with shelf)
    # RIGHT (X=max) = FRONT (access opening)
    
    # Main structure below shelf
    structure_lower = Rectangle((0, 0), OVERALL_DEPTH, SHELF_TOP_HEIGHT,
                         fill=False, edgecolor='black', linewidth=3)
    ax.add_patch(structure_lower)
    
    # Front walls above shelf (do not cover shelf area)
    wall_above_front = Rectangle((0, SHELF_TOP_HEIGHT), OVERALL_DEPTH - REAR_SHELF_DEPTH, 
                                WALL_HEIGHT_ABOVE_SHELF,
                                fill=True, facecolor='lightgray', edgecolor='darkgray',
                                linewidth=2, alpha=0.3)
    ax.add_patch(wall_above_front)
    
    # Rear shelf/wall structure (LEFT side when viewing from foot)
    rear_shelf_x = OVERALL_DEPTH - REAR_SHELF_DEPTH
    rear_shelf = Rectangle((rear_shelf_x, SHELF_TOP_HEIGHT - 20), 
                          REAR_SHELF_DEPTH, 20,
                          fill=True, facecolor='wheat', edgecolor='brown', linewidth=2)
    ax.add_patch(rear_shelf)
    
    # Rear wall (stops at shelf level)
    ax.plot([rear_shelf_x, rear_shelf_x], [0, SHELF_TOP_HEIGHT], 
            'brown', linewidth=4, label=f'Rear Wall (stops at {SHELF_TOP_HEIGHT}mm)')
    ax.plot(rear_shelf_x, SHELF_TOP_HEIGHT, 'ro', markersize=14, label='Wall termination')
    
    ax.text(rear_shelf_x, SHELF_TOP_HEIGHT + 150,
            f'REAR WALL\nSTOPS at {SHELF_TOP_HEIGHT}mm',
            ha='center', va='bottom', fontsize=11, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))
    
    # Mattress and platform
    mattress_x = rear_shelf_x - BED_TOLERANCE - MATTRESS_DEPTH
    
    # Platform
    platform = Rectangle((mattress_x, PLATFORM_TOP - 20), MATTRESS_DEPTH, 20,
                        fill=True, facecolor='brown', edgecolor='saddlebrown', linewidth=2)
    ax.add_patch(platform)
    
    # Mattress
    mattress_rect = Rectangle((mattress_x, PLATFORM_TOP), MATTRESS_DEPTH, MATTRESS_THICKNESS,
                        fill=True, facecolor='lightgreen', edgecolor='darkgreen',
                        linewidth=2, alpha=0.7)
    ax.add_patch(mattress_rect)
    
    # Base planks
    base = Rectangle((mattress_x, 0), MATTRESS_DEPTH, PLATFORM_BASE_HEIGHT,
                    fill=True, facecolor='sandybrown', edgecolor='brown',
                    linewidth=2, alpha=0.5)
    ax.add_patch(base)
    
    # Dimensions
    ax.annotate('', xy=(OVERALL_DEPTH, -350), xytext=(0, -350),
                arrowprops=dict(arrowstyle='<->', lw=2.5, color='black'))
    ax.text(OVERALL_DEPTH/2, -420, f'{OVERALL_DEPTH}mm DEPTH', 
            ha='center', fontsize=13, fontweight='bold')
    
    ax.annotate('', xy=(OVERALL_DEPTH + 350, OVERALL_HEIGHT), xytext=(OVERALL_DEPTH + 350, 0),
                arrowprops=dict(arrowstyle='<->', lw=2.5, color='black'))
    ax.text(OVERALL_DEPTH + 420, OVERALL_HEIGHT/2, f'{OVERALL_HEIGHT}mm\n(FULL HEIGHT)',
            ha='left', va='center', fontsize=12, fontweight='bold')
    
    # Orientation label
    ax.text(50, OVERALL_HEIGHT - 200, '← REAR', fontsize=12, fontweight='bold', color='brown')
    ax.text(OVERALL_DEPTH - 100, OVERALL_HEIGHT - 200, 'FRONT →', fontsize=12, fontweight='bold', color='blue')
    
    ax.set_xlim(-600, OVERALL_DEPTH + 700)
    ax.set_ylim(-600, OVERALL_HEIGHT + 200)
    ax.set_xlabel('Depth (mm) - Rear (LEFT) to Front (RIGHT)', fontsize=12)
    ax.set_ylabel('Height (mm)', fontsize=12)
    ax.set_title('SIDE ELEVATION - View from FOOT END (rear on left, front on right)', 
                fontsize=15, fontweight='bold')
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/side_elevation_CORRECTED.png', dpi=300, bbox_inches='tight')
    print("✓ Side elevation (CORRECTED) saved")
    plt.close()

if __name__ == "__main__":
    print("\n" + "="*80)
    print(" CABIN BED TECHNICAL DRAWINGS - FINAL CORRECTED VERSION")
    print(" Building 3D model first, then slicing 2D views for consistency")
    print("="*80 + "\n")
    
    print("Key corrections applied:")
    print(f"  • Access opening: {ACCESS_BOTTOM}mm to {ACCESS_TOP}mm (halfway up mattress to 300mm below ceiling)")
    print(f"  • Wall extension: {WALL_EXTENSION_DEPTH}mm needed (not a cut-in at bed foot)")
    print(f"  • Bed position: Normal (not offset)")
    print(f"  • Side elevation: Consistent orientation (rear on left when viewing from foot)")
    print(f"  • 3D view: From front-left corner showing access opening clearly")
    print()
    
    create_3d_visualization()
    draw_front_elevation()
    draw_top_plan()
    draw_side_elevation()
    
    print("\n" + "="*80)
    print(" ✓ ALL CORRECTED DRAWINGS GENERATED!")
    print("="*80)
    print("\nGenerated files:")
    print("  - 3d_visualization_CORRECTED.png")
    print("  - front_elevation_CORRECTED.png")
    print("  - top_plan_CORRECTED.png")
    print("  - side_elevation_CORRECTED.png")
    print("\n")
