#!/usr/bin/env python3
"""
Cabin Bed Technical Drawings - CORRECTED COORDINATE SYSTEM
Origin at BACK-LEFT-FLOOR corner, everything extends OUT into room
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle, Circle
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

# All measurements in mm
# COORDINATE SYSTEM:
# Origin (0,0,0) = BACK-LEFT-FLOOR corner (where back wall meets foot wall at floor)
# X-axis: Width (0 to 2344mm) - foot wall (left) to head wall (right)
# Y-axis: Depth (0 to 1242mm) - back wall to front (extending into room)
# Z-axis: Height (0 to 2380mm) - floor to ceiling

# Mattress and tolerance
MATTRESS_WIDTH = 1900
MATTRESS_DEPTH = 900
BED_TOLERANCE = 10
BED_FRAME_WIDTH = MATTRESS_WIDTH + (BED_TOLERANCE * 2)  # 1920mm
BED_FRAME_DEPTH = MATTRESS_DEPTH + (BED_TOLERANCE * 2)  # 920mm

# Overall dimensions
FOOT_WALL_THICKNESS = 98  # 70mm stud + plasterboard
INTERNAL_WIDTH = 2246
OVERALL_WIDTH = INTERNAL_WIDTH + FOOT_WALL_THICKNESS  # 2344mm
OVERALL_DEPTH = 1242
WALL_EXTENSION_DEPTH = 237  # Extension at FRONT of foot wall
OVERALL_HEIGHT = 2380

# Platform and mattress
PLATFORM_BASE_HEIGHT = 173
PLATFORM_FRAME_HEIGHT = 357
PLATFORM_TOP = PLATFORM_BASE_HEIGHT + PLATFORM_FRAME_HEIGHT  # 530mm
MATTRESS_THICKNESS = 220
MATTRESS_TOP = PLATFORM_TOP + MATTRESS_THICKNESS  # 750mm

# Shelving and walls
REAR_SHELF_DEPTH = 210  # At Y=0 (back wall)
HEAD_WALL_WIDTH = 326
SHELF_TOP_HEIGHT = 1200
WALL_HEIGHT_ABOVE_SHELF = OVERALL_HEIGHT - SHELF_TOP_HEIGHT  # 1180mm

# Access opening
ACCESS_WIDTH = 1300
ACCESS_START_FROM_LEFT = 300 + FOOT_WALL_THICKNESS  # From left edge including foot wall
ACCESS_BOTTOM = 640  # Halfway up mattress
ACCESS_TOP = 2080  # 300mm below ceiling
ACCESS_HEIGHT = ACCESS_TOP - ACCESS_BOTTOM  # 1440mm

# Drawers
DRAWER_WIDTH = 880
DRAWER_HEIGHT = 500
DRAWER_DEPTH = 900

print(f"\n{'='*80}")
print(f" COORDINATE SYSTEM: Origin at BACK-LEFT-FLOOR corner")
print(f"{'='*80}")
print(f"  X: 0 to {OVERALL_WIDTH}mm (foot wall to head wall)")
print(f"  Y: 0 to {OVERALL_DEPTH}mm (back to front, into room)")
print(f"  Z: 0 to {OVERALL_HEIGHT}mm (floor to ceiling)")
print(f"\n  Wall extension: {WALL_EXTENSION_DEPTH}mm at FRONT of foot wall")
print(f"  Access opening: {ACCESS_WIDTH}×{ACCESS_HEIGHT}mm ({ACCESS_BOTTOM}-{ACCESS_TOP}mm)")
print(f"{'='*80}\n")

def create_3d_visualization():
    """3D visualization with origin at back-left-floor corner"""
    fig = plt.figure(figsize=(18, 14))
    ax = fig.add_subplot(111, projection='3d')
    
    def draw_box(x, y, z, w, d, h, color, alpha, label=None):
        """Draw box from corner (x,y,z) with dimensions w×d×h"""
        vertices = [
            [x, y, z], [x+w, y, z], [x+w, y+d, z], [x, y+d, z],
            [x, y, z+h], [x+w, y, z+h], [x+w, y+d, z+h], [x, y+d, z+h]
        ]
        faces = [
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # front (min Y)
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # back (max Y)
            [vertices[0], vertices[3], vertices[7], vertices[4]],  # left (min X)
            [vertices[1], vertices[2], vertices[6], vertices[5]],  # right (max X)
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # bottom
            [vertices[4], vertices[5], vertices[6], vertices[7]]   # top
        ]
        poly = Poly3DCollection(faces, alpha=alpha, facecolor=color, 
                               edgecolor='black', linewidth=1.5)
        ax.add_collection3d(poly)
    
    # 1. FOOT WALL (left side, X=0 to 98)
    draw_box(0, 0, 0, FOOT_WALL_THICKNESS, OVERALL_DEPTH - WALL_EXTENSION_DEPTH, 
             SHELF_TOP_HEIGHT, 'darkorange', 0.5)
    
    # 2. FOOT WALL EXTENSION (237mm at front, shown in yellow)
    extension_y_start = OVERALL_DEPTH - WALL_EXTENSION_DEPTH
    draw_box(0, extension_y_start, 0, FOOT_WALL_THICKNESS, WALL_EXTENSION_DEPTH,
             SHELF_TOP_HEIGHT, 'yellow', 0.7)
    ax.text(FOOT_WALL_THICKNESS/2, extension_y_start + WALL_EXTENSION_DEPTH/2, SHELF_TOP_HEIGHT/2,
           f'{WALL_EXTENSION_DEPTH}mm\nEXT', fontsize=10, ha='center', fontweight='bold')
    
    # 3. REAR WALL/SHELF (at Y=0, back wall)
    draw_box(FOOT_WALL_THICKNESS, 0, 0, INTERNAL_WIDTH, REAR_SHELF_DEPTH, 
             SHELF_TOP_HEIGHT, 'wheat', 0.4)
    
    # 4. HEAD WALL (right side, X = 2018 to 2344)
    head_wall_x = OVERALL_WIDTH - HEAD_WALL_WIDTH
    # Head wall spans from rear shelf to where mattress starts
    head_wall_depth = REAR_SHELF_DEPTH + BED_TOLERANCE + BED_FRAME_DEPTH + BED_TOLERANCE
    draw_box(head_wall_x, 0, 0, HEAD_WALL_WIDTH, head_wall_depth, 
             SHELF_TOP_HEIGHT, 'lightgray', 0.5)
    
    # 5. FRONT WALLS (left, right, below shelf level - up to where access opening is)
    # Left front wall (foot wall side)
    front_left_width = ACCESS_START_FROM_LEFT - FOOT_WALL_THICKNESS
    draw_box(FOOT_WALL_THICKNESS, extension_y_start, 0, front_left_width, 
             WALL_EXTENSION_DEPTH, ACCESS_TOP, 'lightblue', 0.4)
    
    # Right front wall (head wall side)
    front_right_x = ACCESS_START_FROM_LEFT + ACCESS_WIDTH
    front_right_width = OVERALL_WIDTH - front_right_x
    draw_box(front_right_x, extension_y_start, 0, front_right_width,
             WALL_EXTENSION_DEPTH, ACCESS_TOP, 'lightblue', 0.4)
    
    # Front wall below access opening (full width)
    draw_box(FOOT_WALL_THICKNESS, extension_y_start, 0, INTERNAL_WIDTH,
             WALL_EXTENSION_DEPTH, ACCESS_BOTTOM, 'lightblue', 0.3)
    
    # 6. WALLS ABOVE SHELF (transparent, extending back over rear shelf area)
    # This covers from front to back, above shelf level
    draw_box(FOOT_WALL_THICKNESS, 0, SHELF_TOP_HEIGHT, INTERNAL_WIDTH, OVERALL_DEPTH,
             WALL_HEIGHT_ABOVE_SHELF, 'lightgray', 0.15)
    
    # 7. BASE PLINTH (wooden planks raising bed)
    mattress_y_start = REAR_SHELF_DEPTH + BED_TOLERANCE
    plinth_x = FOOT_WALL_THICKNESS + BED_TOLERANCE
    draw_box(plinth_x, mattress_y_start, 0, MATTRESS_WIDTH, MATTRESS_DEPTH,
             PLATFORM_BASE_HEIGHT, 'sandybrown', 0.4)
    
    # 8. BED FRAME PLATFORM
    draw_box(plinth_x, mattress_y_start, PLATFORM_BASE_HEIGHT, MATTRESS_WIDTH, 
             MATTRESS_DEPTH, PLATFORM_FRAME_HEIGHT, 'brown', 0.4)
    
    # 9. MATTRESS
    draw_box(plinth_x, mattress_y_start, PLATFORM_TOP, MATTRESS_WIDTH, 
             MATTRESS_DEPTH, MATTRESS_THICKNESS, 'lightgreen', 0.7)
    
    # 10. ACCESS OPENING outline (at front wall)
    opening_y = extension_y_start
    opening_corners = [
        [ACCESS_START_FROM_LEFT, opening_y, ACCESS_BOTTOM],
        [ACCESS_START_FROM_LEFT + ACCESS_WIDTH, opening_y, ACCESS_BOTTOM],
        [ACCESS_START_FROM_LEFT + ACCESS_WIDTH, opening_y, ACCESS_TOP],
        [ACCESS_START_FROM_LEFT, opening_y, ACCESS_TOP],
        [ACCESS_START_FROM_LEFT, opening_y, ACCESS_BOTTOM]
    ]
    xs, ys, zs = zip(*opening_corners)
    ax.plot(xs, ys, zs, 'r-', linewidth=5, label='Access Opening')
    
    ax.text(ACCESS_START_FROM_LEFT + ACCESS_WIDTH/2, opening_y + 100, 
           ACCESS_BOTTOM + ACCESS_HEIGHT/2,
           f'ACCESS\nOPENING\n{ACCESS_WIDTH}×{ACCESS_HEIGHT}mm',
           fontsize=11, ha='center', fontweight='bold',
           bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.9))
    
    # Labels
    ax.text(OVERALL_WIDTH/2, OVERALL_DEPTH/2, OVERALL_HEIGHT + 200,
           f'CABIN BED 3D VIEW\n{OVERALL_WIDTH}mm W × {OVERALL_DEPTH}mm D × {OVERALL_HEIGHT}mm H\nOrigin at back-left corner',
           fontsize=14, ha='center', fontweight='bold',
           bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))
    
    # Axis properties
    ax.set_xlabel('Width (mm) →', fontsize=12, labelpad=12)
    ax.set_ylabel('Depth (mm) → INTO ROOM', fontsize=12, labelpad=12)
    ax.set_zlabel('Height (mm) ↑', fontsize=12, labelpad=12)
    ax.set_title('3D VIEW - From back-left corner (tight against walls)', 
                fontsize=16, fontweight='bold', pad=20)
    
    ax.set_xlim(-200, OVERALL_WIDTH + 300)
    ax.set_ylim(-200, OVERALL_DEPTH + 300)
    ax.set_zlim(0, OVERALL_HEIGHT + 300)
    
    # View from front-left corner looking back toward the alcove
    ax.view_init(elev=20, azim=225)
    
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/3d_view_FINAL.png', dpi=300, bbox_inches='tight')
    print("✓ 3D visualization saved")
    plt.close()

def draw_front_elevation():
    """Front elevation - looking at front wall from inside room"""
    fig, ax = plt.subplots(figsize=(14, 14))  # Square aspect for proper proportions
    ax.set_aspect('equal')
    
    # Full structure
    structure = Rectangle((FOOT_WALL_THICKNESS, 0), INTERNAL_WIDTH, OVERALL_HEIGHT,
                         fill=False, edgecolor='black', linewidth=3)
    ax.add_patch(structure)
    
    # Platform, mattress, shelf levels
    for y, label, color in [(PLATFORM_TOP, f'Platform {PLATFORM_TOP}mm', 'blue'),
                            (MATTRESS_TOP, f'Mattress Top {MATTRESS_TOP}mm', 'green'),
                            (SHELF_TOP_HEIGHT, f'Shelf {SHELF_TOP_HEIGHT}mm', 'purple')]:
        ax.plot([FOOT_WALL_THICKNESS, OVERALL_WIDTH], [y, y], 
                color=color, linewidth=2, linestyle='--', alpha=0.7, label=label)
    
    # Walls above shelf
    wall_above = Rectangle((FOOT_WALL_THICKNESS, SHELF_TOP_HEIGHT), INTERNAL_WIDTH, 
                          WALL_HEIGHT_ABOVE_SHELF,
                          fill=True, facecolor='lightgray', edgecolor='darkgray', 
                          linewidth=2, alpha=0.3)
    ax.add_patch(wall_above)
    
    # Access opening
    access = Rectangle((ACCESS_START_FROM_LEFT, ACCESS_BOTTOM), ACCESS_WIDTH, ACCESS_HEIGHT,
                      fill=False, edgecolor='red', linewidth=4, linestyle='--')
    ax.add_patch(access)
    ax.text(ACCESS_START_FROM_LEFT + ACCESS_WIDTH/2, ACCESS_BOTTOM + ACCESS_HEIGHT/2,
            f'ACCESS OPENING\n{ACCESS_WIDTH}×{ACCESS_HEIGHT}mm\n{ACCESS_BOTTOM}-{ACCESS_TOP}mm',
            ha='center', va='center', fontsize=11, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # Drawers
    for dx, label in [(FOOT_WALL_THICKNESS + 100, 'DRAWER 1'), 
                      (OVERALL_WIDTH - DRAWER_WIDTH - 100, 'DRAWER 2')]:
        drawer = Rectangle((dx, 20), DRAWER_WIDTH, DRAWER_HEIGHT,
                          fill=True, facecolor='lightblue', edgecolor='blue', linewidth=2)
        ax.add_patch(drawer)
        ax.text(dx + DRAWER_WIDTH/2, 20 + DRAWER_HEIGHT/2,
                f'{label}\n{DRAWER_WIDTH}×{DRAWER_HEIGHT}mm', 
                ha='center', va='center', fontsize=10, fontweight='bold')
    
    # Tablet
    tablet = Circle((FOOT_WALL_THICKNESS + INTERNAL_WIDTH/2, 975), 90,
                   fill=True, facecolor='lightgray', edgecolor='black', linewidth=2)
    ax.add_patch(tablet)
    ax.text(FOOT_WALL_THICKNESS + INTERNAL_WIDTH/2, 975, '7"\nTABLET', 
            ha='center', va='center', fontsize=9, fontweight='bold')
    
    # Dimensions
    # Width
    ax.annotate('', xy=(OVERALL_WIDTH, -300), xytext=(FOOT_WALL_THICKNESS, -300),
                arrowprops=dict(arrowstyle='<->', lw=2.5, color='black'))
    ax.text((FOOT_WALL_THICKNESS + OVERALL_WIDTH)/2, -380, 
            f'{INTERNAL_WIDTH}mm width', ha='center', fontsize=12, fontweight='bold')
    
    # Height
    ax.annotate('', xy=(OVERALL_WIDTH + 300, OVERALL_HEIGHT), 
                xytext=(OVERALL_WIDTH + 300, 0),
                arrowprops=dict(arrowstyle='<->', lw=2.5, color='black'))
    ax.text(OVERALL_WIDTH + 380, OVERALL_HEIGHT/2, 
            f'{OVERALL_HEIGHT}mm\nheight', 
            ha='left', va='center', fontsize=12, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    ax.set_xlim(-100, OVERALL_WIDTH + 650)
    ax.set_ylim(-500, OVERALL_HEIGHT + 200)
    ax.set_xlabel('Width (mm)', fontsize=12)
    ax.set_ylabel('Height (mm)', fontsize=12)
    ax.set_title(f'FRONT ELEVATION - {INTERNAL_WIDTH}mm × {OVERALL_HEIGHT}mm', 
                fontsize=16, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/front_elevation_FINAL.png', dpi=300, bbox_inches='tight')
    print("✓ Front elevation saved")
    plt.close()

def draw_top_plan():
    """Top plan - looking down from above"""
    fig, ax = plt.subplots(figsize=(16, 15))
    ax.set_aspect('equal')
    
    # Foot wall (full depth minus extension)
    foot_wall = Rectangle((0, 0), FOOT_WALL_THICKNESS, OVERALL_DEPTH - WALL_EXTENSION_DEPTH,
                         fill=True, facecolor='darkorange', edgecolor='black',
                         linewidth=3, alpha=0.6)
    ax.add_patch(foot_wall)
    ax.text(FOOT_WALL_THICKNESS/2, (OVERALL_DEPTH - WALL_EXTENSION_DEPTH)/2, 
            f'FOOT\nWALL\n{FOOT_WALL_THICKNESS}mm',
            ha='center', va='center', fontsize=10, fontweight='bold', rotation=90)
    
    # Wall extension at FRONT (Y = 1005 to 1242)
    extension_y = OVERALL_DEPTH - WALL_EXTENSION_DEPTH
    extension = Rectangle((0, extension_y), FOOT_WALL_THICKNESS, WALL_EXTENSION_DEPTH,
                         fill=True, facecolor='yellow', edgecolor='orange',
                         linewidth=3, alpha=0.8)
    ax.add_patch(extension)
    ax.text(FOOT_WALL_THICKNESS/2, extension_y + WALL_EXTENSION_DEPTH/2,
            f'{WALL_EXTENSION_DEPTH}mm\nEXT.\n(FRONT)',
            ha='center', va='center', fontsize=10, fontweight='bold', rotation=90,
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.9))
    
    # Internal space outline
    structure = Rectangle((FOOT_WALL_THICKNESS, 0), INTERNAL_WIDTH, OVERALL_DEPTH,
                         fill=False, edgecolor='black', linewidth=3)
    ax.add_patch(structure)
    
    # Rear shelf (at Y=0, back)
    rear_shelf = Rectangle((FOOT_WALL_THICKNESS, 0), INTERNAL_WIDTH, REAR_SHELF_DEPTH,
                          fill=True, facecolor='wheat', edgecolor='brown', 
                          linewidth=2, alpha=0.6)
    ax.add_patch(rear_shelf)
    ax.text(FOOT_WALL_THICKNESS + INTERNAL_WIDTH/2, REAR_SHELF_DEPTH/2,
            'REAR SHELF\n210mm deep',
            ha='center', va='center', fontsize=12, fontweight='bold')
    
    # Head wall (right side)
    head_wall_x = OVERALL_WIDTH - HEAD_WALL_WIDTH
    head_wall_depth = REAR_SHELF_DEPTH + BED_TOLERANCE + BED_FRAME_DEPTH + BED_TOLERANCE
    head_wall = Rectangle((head_wall_x, 0), HEAD_WALL_WIDTH, head_wall_depth,
                          fill=True, facecolor='lightgray', edgecolor='darkgray', 
                          linewidth=3, alpha=0.7)
    ax.add_patch(head_wall)
    ax.text(head_wall_x + HEAD_WALL_WIDTH/2, head_wall_depth/2,
            'HEAD\nWALL\n326mm',
            ha='center', va='center', fontsize=11, fontweight='bold')
    
    # Mattress
    mattress_x = FOOT_WALL_THICKNESS + BED_TOLERANCE
    mattress_y = REAR_SHELF_DEPTH + BED_TOLERANCE
    mattress = Rectangle((mattress_x, mattress_y), MATTRESS_WIDTH, MATTRESS_DEPTH,
                        fill=True, facecolor='lightgreen', edgecolor='darkgreen',
                        linewidth=3, alpha=0.7)
    ax.add_patch(mattress)
    ax.text(mattress_x + MATTRESS_WIDTH/2, mattress_y + MATTRESS_DEPTH/2,
            'MATTRESS\n1900×900mm',
            ha='center', va='center', fontsize=13, fontweight='bold')
    
    # Access opening area (at front)
    access_area = Rectangle((ACCESS_START_FROM_LEFT, extension_y), ACCESS_WIDTH, 
                           WALL_EXTENSION_DEPTH,
                           fill=True, facecolor='yellow', edgecolor='red',
                           linewidth=3, alpha=0.3)
    ax.add_patch(access_area)
    ax.text(ACCESS_START_FROM_LEFT + ACCESS_WIDTH/2, extension_y + WALL_EXTENSION_DEPTH/2,
            'ACCESS\nOPENING\n1300mm',
            ha='center', va='center', fontsize=11, fontweight='bold')
    
    # Support legs (6 total)
    leg_size = 20
    leg_y_front = mattress_y + 100
    leg_y_rear = mattress_y + MATTRESS_DEPTH - 100
    for lx in [mattress_x + 50, mattress_x + 940, mattress_x + MATTRESS_WIDTH - 50]:
        for ly in [leg_y_front, leg_y_rear]:
            leg = Rectangle((lx - leg_size/2, ly - leg_size/2), leg_size, leg_size,
                           fill=True, facecolor='black')
            ax.add_patch(leg)
    
    # Dimensions
    # Internal width
    ax.annotate('', xy=(OVERALL_WIDTH, -250), xytext=(FOOT_WALL_THICKNESS, -250),
                arrowprops=dict(arrowstyle='<->', lw=2, color='blue'))
    ax.text((FOOT_WALL_THICKNESS + OVERALL_WIDTH)/2, -330, 
            f'{INTERNAL_WIDTH}mm INTERNAL WIDTH',
            ha='center', fontsize=12, fontweight='bold', color='blue')
    
    # Total width
    ax.annotate('', xy=(OVERALL_WIDTH, -450), xytext=(0, -450),
                arrowprops=dict(arrowstyle='<->', lw=3, color='red'))
    ax.text(OVERALL_WIDTH/2, -530, 
            f'{OVERALL_WIDTH}mm TOTAL = {INTERNAL_WIDTH} + {FOOT_WALL_THICKNESS}',
            ha='center', fontsize=13, fontweight='bold', color='red',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # Depth with extension annotation
    ax.annotate('', xy=(OVERALL_WIDTH + 300, OVERALL_DEPTH), 
                xytext=(OVERALL_WIDTH + 300, 0),
                arrowprops=dict(arrowstyle='<->', lw=2.5, color='black'))
    ax.text(OVERALL_WIDTH + 380, OVERALL_DEPTH/2, 
            f'{OVERALL_DEPTH}mm\nDEPTH',
            ha='left', va='center', fontsize=11, fontweight='bold')
    
    # Extension arrow at front
    ax.annotate('', xy=(-150, OVERALL_DEPTH), xytext=(-150, extension_y),
                arrowprops=dict(arrowstyle='<->', lw=2.5, color='orange'))
    ax.text(-220, extension_y + WALL_EXTENSION_DEPTH/2, 
            f'{WALL_EXTENSION_DEPTH}mm\nextension\nat FRONT',
            ha='right', va='center', fontsize=10, fontweight='bold', color='orange',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))
    
    # Orientation labels
    ax.text(OVERALL_WIDTH/2, -100, '↑ BACK WALL (Y=0) ↑',
            ha='center', fontsize=13, fontweight='bold', color='brown')
    ax.text(OVERALL_WIDTH/2, OVERALL_DEPTH + 100, '↓ FRONT (into room) ↓',
            ha='center', fontsize=13, fontweight='bold', color='blue')
    
    ax.set_xlim(-450, OVERALL_WIDTH + 650)
    ax.set_ylim(-650, OVERALL_DEPTH + 250)
    ax.set_xlabel('Width (mm)', fontsize=13)
    ax.set_ylabel('Depth (mm) - INTO ROOM', fontsize=13)
    ax.set_title('TOP PLAN VIEW - 237mm extension at FRONT of foot wall', 
                fontsize=16, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/top_plan_FINAL.png', dpi=300, bbox_inches='tight')
    print("✓ Top plan saved")
    plt.close()

def draw_side_elevation():
    """Side elevation from FOOT end - flip so rear/shelf on RIGHT, front/void on LEFT"""
    fig, ax = plt.subplots(figsize=(15, 13))
    ax.set_aspect('equal')
    
    # Structure below shelf (flipped: 0=front, max=rear)
    structure_lower = Rectangle((0, 0), OVERALL_DEPTH, SHELF_TOP_HEIGHT,
                         fill=False, edgecolor='black', linewidth=3)
    ax.add_patch(structure_lower)
    
    # Front walls above shelf (LEFT side when flipped)
    extension_depth = OVERALL_DEPTH - WALL_EXTENSION_DEPTH
    wall_above_front = Rectangle((0, SHELF_TOP_HEIGHT), extension_depth, 
                                WALL_HEIGHT_ABOVE_SHELF,
                                fill=True, facecolor='lightgray', edgecolor='darkgray',
                                linewidth=2, alpha=0.3)
    ax.add_patch(wall_above_front)
    
    # Rear shelf (RIGHT side when flipped)
    rear_shelf = Rectangle((OVERALL_DEPTH - REAR_SHELF_DEPTH, SHELF_TOP_HEIGHT - 20), 
                          REAR_SHELF_DEPTH, 20,
                          fill=True, facecolor='wheat', edgecolor='brown', linewidth=2)
    ax.add_patch(rear_shelf)
    
    # Rear wall (stops at shelf - RIGHT side)
    rear_wall_x = OVERALL_DEPTH - REAR_SHELF_DEPTH
    ax.plot([rear_wall_x, rear_wall_x], [0, SHELF_TOP_HEIGHT], 
            'brown', linewidth=4, label='Rear wall stops at shelf')
    ax.plot(rear_wall_x, SHELF_TOP_HEIGHT, 'ro', markersize=14)
    
    # Mattress area (on right side)
    mattress_x_end = rear_wall_x - BED_TOLERANCE
    mattress_x_start = mattress_x_end - MATTRESS_DEPTH
    
    # Base plinth
    plinth = Rectangle((mattress_x_start, 0), MATTRESS_DEPTH, PLATFORM_BASE_HEIGHT,
                      fill=True, facecolor='sandybrown', edgecolor='brown',
                      linewidth=2, alpha=0.5)
    ax.add_patch(plinth)
    
    # Platform frame
    platform = Rectangle((mattress_x_start, PLATFORM_BASE_HEIGHT), MATTRESS_DEPTH, 
                        PLATFORM_FRAME_HEIGHT,
                        fill=True, facecolor='brown', edgecolor='saddlebrown', linewidth=2)
    ax.add_patch(platform)
    
    # Mattress
    mattress_rect = Rectangle((mattress_x_start, PLATFORM_TOP), MATTRESS_DEPTH, 
                             MATTRESS_THICKNESS,
                             fill=True, facecolor='lightgreen', edgecolor='darkgreen',
                             linewidth=2, alpha=0.7)
    ax.add_patch(mattress_rect)
    
    # Dimensions
    ax.annotate('', xy=(OVERALL_DEPTH, -350), xytext=(0, -350),
                arrowprops=dict(arrowstyle='<->', lw=2.5, color='black'))
    ax.text(OVERALL_DEPTH/2, -420, f'{OVERALL_DEPTH}mm DEPTH', 
            ha='center', fontsize=13, fontweight='bold')
    
    ax.annotate('', xy=(OVERALL_DEPTH + 350, OVERALL_HEIGHT), 
                xytext=(OVERALL_DEPTH + 350, 0),
                arrowprops=dict(arrowstyle='<->', lw=2.5, color='black'))
    ax.text(OVERALL_DEPTH + 420, OVERALL_HEIGHT/2, f'{OVERALL_HEIGHT}mm\nHEIGHT',
            ha='left', va='center', fontsize=12, fontweight='bold')
    
    ax.set_xlim(-600, OVERALL_DEPTH + 700)
    ax.set_ylim(-600, OVERALL_HEIGHT + 200)
    ax.set_xlabel('Depth (mm) - Front (left) to Rear (right)', fontsize=12)
    ax.set_ylabel('Height (mm)', fontsize=12)
    ax.set_title('SIDE ELEVATION from FOOT END - Front left, rear right', 
                fontsize=16, fontweight='bold')
    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/side_elevation_FINAL.png', dpi=300, bbox_inches='tight')
    print("✓ Side elevation saved")
    plt.close()

if __name__ == "__main__":
    print("\n" + "="*80)
    print(" CABIN BED TECHNICAL DRAWINGS - FINAL VERSION")
    print("="*80)
    
    create_3d_visualization()
    draw_front_elevation()
    draw_top_plan()
    draw_side_elevation()
    
    print("\n" + "="*80)
    print(" ✓ ALL DRAWINGS GENERATED")
    print("="*80)
    print("\nFiles:")
    print("  - 3d_view_FINAL.png")
    print("  - front_elevation_FINAL.png")
    print("  - top_plan_FINAL.png")
    print("  - side_elevation_FINAL.png\n")
