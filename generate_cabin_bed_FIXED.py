#!/usr/bin/env python3
"""
Cabin Bed Technical Drawings Generator - CORRECTED VERSION
Fixes orientation, height, and layout issues
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle
import numpy as np

# All measurements in mm
# CORRECTED: Proper layout understanding

# Mattress and tolerance
MATTRESS_WIDTH = 1900
MATTRESS_DEPTH = 900
BED_TOLERANCE = 10  # 10mm tolerance around mattress
BED_FRAME_WIDTH = MATTRESS_WIDTH + (BED_TOLERANCE * 2)  # 1920mm
BED_FRAME_DEPTH = MATTRESS_DEPTH + (BED_TOLERANCE * 2)  # 920mm

# Overall dimensions
INTERNAL_WIDTH = 2246  # Internal dimension (mattress + clearances + head wall)
FOOT_WALL_THICKNESS = 98  # 70mm stud + 2×12.5mm plasterboard = 95mm, rounded to 98mm
OVERALL_WIDTH = INTERNAL_WIDTH + FOOT_WALL_THICKNESS  # 2344mm TOTAL width
OVERALL_DEPTH = 1242  # Extends into room (+20mm from original 1222mm)
OVERALL_HEIGHT = 2380  # FULL HEIGHT including walls above shelf!

# Platform and mattress
PLATFORM_BASE_HEIGHT = 173  # Wooden planks off floor
PLATFORM_FRAME_HEIGHT = 357  # Frame height
PLATFORM_TOP = PLATFORM_BASE_HEIGHT + PLATFORM_FRAME_HEIGHT  # 530mm
MATTRESS_THICKNESS = 220
MATTRESS_TOP = PLATFORM_TOP + MATTRESS_THICKNESS  # 750mm

# Shelving and walls
REAR_SHELF_DEPTH = 210  # Shelf at back wall
HEAD_WALL_WIDTH = 326  # Solid wall at head end (reduced for tolerance)
SHELF_TOP_HEIGHT = 1200  # Height to top of shelf surface
WALL_HEIGHT_ABOVE_SHELF = OVERALL_HEIGHT - SHELF_TOP_HEIGHT  # Walls continue above!

# Access opening
ACCESS_WIDTH = 1300
ACCESS_HEIGHT = 580
ACCESS_START_FROM_LEFT = 300
ACCESS_BOTTOM = PLATFORM_TOP  # 530mm

# Drawers - USE FULL HEIGHT from floor to underside of bed frame!
DRAWER_WIDTH = 880
DRAWER_HEIGHT = 500  # Full void height (530mm - 20mm runners - 10mm clearance)
DRAWER_DEPTH = 900
DRAWER_CLEARANCE_BOTTOM = 20  # For floor-mounted runners
DRAWER_CLEARANCE_TOP = 10  # Top clearance

# Tablet
TABLET_CENTER_HEIGHT = 975  # Between mattress top (750) and shelf (1200)

def draw_front_elevation_corrected():
    """Draw front elevation - CORRECTED to show full height and access from full height"""
    fig, ax = plt.subplots(figsize=(16, 12))
    ax.set_aspect('equal')
    
    # MAIN STRUCTURE - FULL HEIGHT TO 2380mm
    # Structure shows INTERNAL width (foot wall is in side view)
    structure = Rectangle((0, 0), INTERNAL_WIDTH, OVERALL_HEIGHT,
                         fill=False, edgecolor='black', linewidth=3)
    ax.add_patch(structure)
    
    # Platform level line
    ax.plot([0, INTERNAL_WIDTH], [PLATFORM_TOP, PLATFORM_TOP], 
            'b--', linewidth=1.5, alpha=0.7, label=f'Platform Top ({PLATFORM_TOP}mm)')
    
    # Floor line (to show drawer void)
    ax.plot([0, INTERNAL_WIDTH], [0, 0], 
            'k-', linewidth=3, label='Floor Level')
    
    # Mattress top line
    ax.plot([0, INTERNAL_WIDTH], [MATTRESS_TOP, MATTRESS_TOP], 
            'g--', linewidth=1.5, alpha=0.7, label=f'Mattress Top ({MATTRESS_TOP}mm)')
    
    # Shelf level line
    ax.plot([0, INTERNAL_WIDTH], [SHELF_TOP_HEIGHT, SHELF_TOP_HEIGHT], 
            'purple', linewidth=2.5, label=f'Shelf Top ({SHELF_TOP_HEIGHT}mm)')
    
    # Walls above shelf (THIS WAS MISSING!)
    wall_above = Rectangle((0, SHELF_TOP_HEIGHT), INTERNAL_WIDTH, WALL_HEIGHT_ABOVE_SHELF,
                          fill=True, facecolor='lightgray', edgecolor='darkgray', 
                          linewidth=2, alpha=0.3)
    ax.add_patch(wall_above)
    ax.text(INTERNAL_WIDTH/2, SHELF_TOP_HEIGHT + WALL_HEIGHT_ABOVE_SHELF/2,
            f'WALLS CONTINUE\n{WALL_HEIGHT_ABOVE_SHELF}mm above shelf',
            ha='center', va='center', fontsize=11, fontweight='bold')
    
    # Access opening
    access = Rectangle((ACCESS_START_FROM_LEFT, ACCESS_BOTTOM), ACCESS_WIDTH, ACCESS_HEIGHT,
                      fill=False, edgecolor='red', linewidth=3, linestyle='--')
    ax.add_patch(access)
    ax.text(ACCESS_START_FROM_LEFT + ACCESS_WIDTH/2, ACCESS_BOTTOM + ACCESS_HEIGHT/2,
            'ACCESS OPENING\n(CURTAINS)\n1300 × 580mm',
            ha='center', va='center', fontsize=10, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))
    
    # Drawers - USE FULL HEIGHT from floor to underside of bed!
    drawer_y_bottom = DRAWER_CLEARANCE_BOTTOM  # Start above floor (runners)
    drawer_y_top = PLATFORM_TOP - DRAWER_CLEARANCE_TOP  # End below bed frame
    actual_drawer_height = drawer_y_top - drawer_y_bottom  # ~500mm
    
    # Left drawer (approximate position)
    left_drawer_x = 100
    left_drawer = Rectangle((left_drawer_x, drawer_y_bottom), DRAWER_WIDTH, actual_drawer_height,
                           fill=True, facecolor='lightblue', edgecolor='blue', linewidth=2)
    ax.add_patch(left_drawer)
    ax.text(left_drawer_x + DRAWER_WIDTH/2, drawer_y_bottom + actual_drawer_height/2,
            f'DRAWER 1\n880mm × {int(actual_drawer_height)}mm\n(FULL HEIGHT)', 
            ha='center', va='center', fontsize=9, fontweight='bold')
    
    # Right drawer
    right_drawer_x = INTERNAL_WIDTH - DRAWER_WIDTH - 100
    right_drawer = Rectangle((right_drawer_x, drawer_y_bottom), DRAWER_WIDTH, actual_drawer_height,
                            fill=True, facecolor='lightblue', edgecolor='blue', linewidth=2)
    ax.add_patch(right_drawer)
    ax.text(right_drawer_x + DRAWER_WIDTH/2, drawer_y_bottom + actual_drawer_height/2,
            f'DRAWER 2\n880mm × {int(actual_drawer_height)}mm\n(FULL HEIGHT)', 
            ha='center', va='center', fontsize=9, fontweight='bold')
    
    # Tablet position
    tablet_x = INTERNAL_WIDTH / 2
    tablet = Circle((tablet_x, TABLET_CENTER_HEIGHT), 90,
                   fill=True, facecolor='lightgray', edgecolor='black', linewidth=2)
    ax.add_patch(tablet)
    ax.text(tablet_x, TABLET_CENTER_HEIGHT, '7"\nTABLET', 
            ha='center', va='center', fontsize=8, fontweight='bold')
    
    # Dimensions
    # Internal width (front view)
    ax.annotate('', xy=(INTERNAL_WIDTH, -300), xytext=(0, -300),
                arrowprops=dict(arrowstyle='<->', lw=2, color='black'))
    ax.text(INTERNAL_WIDTH/2, -350, f'{INTERNAL_WIDTH}mm (internal width)', 
            ha='center', fontsize=11, fontweight='bold')
    
    # Access opening dimensions - MEASURED FROM FULL HEIGHT!
    ax.annotate('', xy=(ACCESS_START_FROM_LEFT + ACCESS_WIDTH/2 + 200, ACCESS_BOTTOM + ACCESS_HEIGHT), 
                xytext=(ACCESS_START_FROM_LEFT + ACCESS_WIDTH/2 + 200, ACCESS_BOTTOM),
                arrowprops=dict(arrowstyle='<->', lw=2, color='red'))
    ax.text(ACCESS_START_FROM_LEFT + ACCESS_WIDTH/2 + 280, ACCESS_BOTTOM + ACCESS_HEIGHT/2,
            f'{ACCESS_HEIGHT}mm\nfrom {ACCESS_BOTTOM}mm\nto {ACCESS_BOTTOM + ACCESS_HEIGHT}mm\n(within {OVERALL_HEIGHT}mm full height)',
            ha='left', va='center', fontsize=9, color='red',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))
    
    # Overall height
    ax.annotate('', xy=(INTERNAL_WIDTH + 300, OVERALL_HEIGHT), xytext=(INTERNAL_WIDTH + 300, 0),
                arrowprops=dict(arrowstyle='<->', lw=2, color='black'))
    ax.text(INTERNAL_WIDTH + 350, OVERALL_HEIGHT/2, f'{OVERALL_HEIGHT}mm\n(FULL HEIGHT)', 
            ha='left', va='center', fontsize=11, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # Shelf height
    ax.annotate('', xy=(INTERNAL_WIDTH + 600, SHELF_TOP_HEIGHT), xytext=(INTERNAL_WIDTH + 600, 0),
                arrowprops=dict(arrowstyle='<->', lw=1.5, color='purple'))
    ax.text(INTERNAL_WIDTH + 650, SHELF_TOP_HEIGHT/2, f'{SHELF_TOP_HEIGHT}mm\n(to shelf)', 
            ha='left', va='center', fontsize=9)
    
    # Drawer void height (floor to platform)
    ax.annotate('', xy=(-400, PLATFORM_TOP), xytext=(-400, 0),
                arrowprops=dict(arrowstyle='<->', lw=2, color='blue'))
    ax.text(-450, PLATFORM_TOP/2, f'{PLATFORM_TOP}mm\nDRAWER\nVOID', 
            ha='right', va='center', fontsize=10, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
    
    ax.set_xlim(-500, INTERNAL_WIDTH + 900)
    ax.set_ylim(-500, OVERALL_HEIGHT + 200)
    ax.set_xlabel('Width (mm)', fontsize=11)
    ax.set_ylabel('Height (mm)', fontsize=11)
    ax.set_title('FRONT ELEVATION - Cabin Bed (Access opening measured from full height)', 
                fontsize=16, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/front_elevation_FIXED.png', dpi=300, bbox_inches='tight')
    print("✓ Front elevation (FIXED) saved")
    plt.close()

def draw_top_plan_corrected():
    """Draw top plan view - CORRECTED with foot wall extension"""
    fig, ax = plt.subplots(figsize=(16, 14))
    ax.set_aspect('equal')
    
    # CORRECTED LAYOUT ORIENTATION:
    # TOP of drawing = REAR (back wall with shelf)
    # BOTTOM of drawing = FRONT (access opening, extends into room)
    # LEFT of drawing = FOOT END (against existing wall + 98mm extension)
    # RIGHT of drawing = HEAD END (solid wall/fascia, 326mm wide)
    
    # FOOT WALL EXTENSION (98mm) - this is OUTSIDE the internal measurement
    foot_wall_ext = Rectangle((-FOOT_WALL_THICKNESS, 0), FOOT_WALL_THICKNESS, OVERALL_DEPTH,
                             fill=True, facecolor='darkorange', edgecolor='black',
                             linewidth=3, alpha=0.6)
    ax.add_patch(foot_wall_ext)
    ax.text(-FOOT_WALL_THICKNESS/2, OVERALL_DEPTH/2, 
            f'FOOT WALL\nEXTENSION\n{FOOT_WALL_THICKNESS}mm\n(70mm stud\n+plasterboard)',
            ha='center', va='center', fontsize=10, fontweight='bold', rotation=90)
    
    # Overall internal structure (2246mm)
    structure = Rectangle((0, 0), INTERNAL_WIDTH, OVERALL_DEPTH,
                         fill=False, edgecolor='black', linewidth=3)
    ax.add_patch(structure)
    
    # REAR SHELF - at TOP (back wall) - spans full internal width
    rear_shelf_y_start = OVERALL_DEPTH - REAR_SHELF_DEPTH
    rear_shelf = Rectangle((0, rear_shelf_y_start), INTERNAL_WIDTH, REAR_SHELF_DEPTH,
                          fill=True, facecolor='wheat', edgecolor='brown', linewidth=2, alpha=0.6)
    ax.add_patch(rear_shelf)
    ax.text(INTERNAL_WIDTH/2, rear_shelf_y_start + REAR_SHELF_DEPTH/2,
            'REAR SHELF - 210mm deep\n(Back Wall)',
            ha='center', va='center', fontsize=12, fontweight='bold')
    
    # HEAD WALL - RIGHT SIDE (solid wall/fascia perpendicular to bed)
    # This runs from back to mattress area
    head_wall_y_start = rear_shelf_y_start - BED_FRAME_DEPTH - BED_TOLERANCE
    head_wall_height = BED_FRAME_DEPTH + BED_TOLERANCE  # Covers mattress depth area
    head_wall = Rectangle((INTERNAL_WIDTH - HEAD_WALL_WIDTH, head_wall_y_start), 
                          HEAD_WALL_WIDTH, head_wall_height,
                          fill=True, facecolor='lightgray', edgecolor='darkgray', 
                          linewidth=3, alpha=0.7)
    ax.add_patch(head_wall)
    ax.text(INTERNAL_WIDTH - HEAD_WALL_WIDTH/2, head_wall_y_start + head_wall_height/2,
            'HEAD WALL\n(Solid Fascia)\n326mm wide',
            ha='center', va='center', fontsize=11, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # EXISTING WALL CUT-IN (inside internal area at foot end)
    # This is where the existing wall is cut back 200mm
    wall_cutin_depth = 200  # Cutting into existing wall
    wall_cutin = Rectangle((0, head_wall_y_start), wall_cutin_depth, head_wall_height,
                          fill=True, facecolor='lightyellow', edgecolor='orange',
                          linewidth=2, alpha=0.5, linestyle='--')
    ax.add_patch(wall_cutin)
    ax.text(wall_cutin_depth/2, head_wall_y_start + head_wall_height/2,
            'Existing\nwall\ncut-in\n200mm',
            ha='center', va='center', fontsize=9, rotation=0)
    
    # MATTRESS area (with 10mm tolerance all around)
    # Positioned between rear shelf and access area, between foot and head walls
    mattress_x_start = wall_cutin_depth + BED_TOLERANCE
    mattress_y_start = rear_shelf_y_start - BED_TOLERANCE - MATTRESS_DEPTH
    mattress = Rectangle((mattress_x_start, mattress_y_start), MATTRESS_WIDTH, MATTRESS_DEPTH,
                        fill=True, facecolor='lightgreen', edgecolor='darkgreen',
                        linewidth=3, alpha=0.7)
    ax.add_patch(mattress)
    ax.text(mattress_x_start + MATTRESS_WIDTH/2, mattress_y_start + MATTRESS_DEPTH/2,
            'MATTRESS\n1900 × 900mm\n(10mm clearance all sides)',
            ha='center', va='center', fontsize=12, fontweight='bold')
    
    # BED FRAME outline (shows tolerance area)
    frame_x_start = wall_cutin_depth
    frame_y_start = mattress_y_start - BED_TOLERANCE
    bed_frame = Rectangle((frame_x_start, frame_y_start), BED_FRAME_WIDTH, BED_FRAME_DEPTH,
                          fill=False, edgecolor='red', linewidth=2, linestyle='--')
    ax.add_patch(bed_frame)
    ax.text(frame_x_start + BED_FRAME_WIDTH/2, frame_y_start - 50,
            'Bed Frame Opening: 1920 × 920mm',
            ha='center', fontsize=10, color='red', fontweight='bold')
    
    # ACCESS OPENING area (at BOTTOM/FRONT)
    access_y_end = frame_y_start
    access_y_start = 0
    access_height_in_plan = access_y_end - access_y_start
    access_area = Rectangle((ACCESS_START_FROM_LEFT, access_y_start), ACCESS_WIDTH, access_height_in_plan,
                           fill=True, facecolor='yellow', edgecolor='red',
                           linewidth=3, alpha=0.3)
    ax.add_patch(access_area)
    ax.text(ACCESS_START_FROM_LEFT + ACCESS_WIDTH/2, access_height_in_plan/2,
            'ACCESS OPENING\n1300mm wide\n(with curtains)',
            ha='center', va='center', fontsize=11, fontweight='bold')
    
    # Support legs (6 total)
    leg_size = 20
    leg_positions = [
        # Front 3 legs
        (frame_x_start + 50, frame_y_start + 100),  # Front left
        (frame_x_start + 940, frame_y_start + 100),  # Front middle
        (frame_x_start + BED_FRAME_WIDTH - 50, frame_y_start + 100),  # Front right
        # Rear 3 legs
        (frame_x_start + 50, frame_y_start + BED_FRAME_DEPTH - 100),  # Rear left
        (frame_x_start + 940, frame_y_start + BED_FRAME_DEPTH - 100),  # Rear middle
        (frame_x_start + BED_FRAME_WIDTH - 50, frame_y_start + BED_FRAME_DEPTH - 100),  # Rear right
    ]
    
    for i, (lx, ly) in enumerate(leg_positions):
        leg = Rectangle((lx - leg_size/2, ly - leg_size/2), leg_size, leg_size,
                       fill=True, facecolor='black', edgecolor='black')
        ax.add_patch(leg)
    
    ax.text(leg_positions[0][0] + 60, leg_positions[0][1], 'Support Legs\n20×20mm (6 total)',
            fontsize=8)
    
    # Dimensions
    # Internal width
    ax.annotate('', xy=(INTERNAL_WIDTH, -250), xytext=(0, -250),
                arrowprops=dict(arrowstyle='<->', lw=2, color='blue'))
    ax.text(INTERNAL_WIDTH/2, -300, f'{INTERNAL_WIDTH}mm (INTERNAL WIDTH)',
            ha='center', fontsize=11, fontweight='bold', color='blue')
    
    # Total external width (including foot wall)
    ax.annotate('', xy=(INTERNAL_WIDTH, -400), xytext=(-FOOT_WALL_THICKNESS, -400),
                arrowprops=dict(arrowstyle='<->', lw=3, color='red'))
    ax.text((-FOOT_WALL_THICKNESS + INTERNAL_WIDTH)/2, -460, 
            f'{OVERALL_WIDTH}mm (TOTAL EXTERNAL WIDTH)\n= {INTERNAL_WIDTH}mm internal + {FOOT_WALL_THICKNESS}mm foot wall',
            ha='center', fontsize=12, fontweight='bold', color='red',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # Overall depth
    ax.annotate('', xy=(INTERNAL_WIDTH + 250, OVERALL_DEPTH), xytext=(INTERNAL_WIDTH + 250, 0),
                arrowprops=dict(arrowstyle='<->', lw=2, color='black'))
    ax.text(INTERNAL_WIDTH + 300, OVERALL_DEPTH/2, f'{OVERALL_DEPTH}mm\n(OVERALL DEPTH)\n+20mm extends\ninto room',
            ha='left', va='center', fontsize=11, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
    
    # Head wall width
    ax.annotate('', xy=(INTERNAL_WIDTH, head_wall_y_start - 100), 
                xytext=(INTERNAL_WIDTH - HEAD_WALL_WIDTH, head_wall_y_start - 100),
                arrowprops=dict(arrowstyle='<->', lw=1.5, color='darkgray'))
    ax.text(INTERNAL_WIDTH - HEAD_WALL_WIDTH/2, head_wall_y_start - 150,
            f'{HEAD_WALL_WIDTH}mm', ha='center', fontsize=10, fontweight='bold')
    
    # Foot wall thickness
    ax.annotate('', xy=(0, OVERALL_DEPTH + 100), xytext=(-FOOT_WALL_THICKNESS, OVERALL_DEPTH + 100),
                arrowprops=dict(arrowstyle='<->', lw=2, color='darkorange'))
    ax.text(-FOOT_WALL_THICKNESS/2, OVERALL_DEPTH + 150, f'{FOOT_WALL_THICKNESS}mm\n(foot wall)',
            ha='center', fontsize=10, fontweight='bold', color='darkorange')
    
    # Orientation labels
    ax.text(INTERNAL_WIDTH/2, OVERALL_DEPTH + 250, '↓ FRONT (into room) ↓',
            ha='center', fontsize=13, fontweight='bold', color='blue')
    ax.text(INTERNAL_WIDTH/2, -150, '↑ REAR (back wall) ↑',
            ha='center', fontsize=13, fontweight='bold', color='blue')
    ax.text(-FOOT_WALL_THICKNESS - 200, OVERALL_DEPTH/2, '← FOOT END\n(+98mm wall extension)',
            ha='center', va='center', fontsize=11, fontweight='bold', color='darkorange')
    ax.text(INTERNAL_WIDTH + 100, head_wall_y_start + head_wall_height/2, 'HEAD END →\n(326mm wall)',
            ha='left', va='center', fontsize=11, fontweight='bold', color='darkgray')
    
    # North arrow
    ax.annotate('', xy=(INTERNAL_WIDTH - 200, OVERALL_DEPTH - 100),
                xytext=(INTERNAL_WIDTH - 200, OVERALL_DEPTH - 300),
                arrowprops=dict(arrowstyle='->', lw=3, color='red'))
    ax.text(INTERNAL_WIDTH - 150, OVERALL_DEPTH - 200, 'N', fontsize=16, fontweight='bold', color='red')
    
    ax.set_xlim(-FOOT_WALL_THICKNESS - 300, INTERNAL_WIDTH + 600)
    ax.set_ylim(-600, OVERALL_DEPTH + 300)
    ax.set_xlabel('Width (mm) - Internal 2246mm + Foot Wall 98mm = 2344mm Total', fontsize=12)
    ax.set_ylabel('Depth (mm)', fontsize=12)
    ax.set_title('TOP PLAN VIEW - Cabin Bed (Internal dimensions + 98mm foot wall extension)', 
                fontsize=16, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/top_plan_FIXED.png', dpi=300, bbox_inches='tight')
    print("✓ Top plan (FIXED) saved")
    plt.close()

def draw_side_elevation_corrected():
    """Draw side elevation from foot end - showing rear wall termination at shelf"""
    fig, ax = plt.subplots(figsize=(14, 12))
    ax.set_aspect('equal')
    
    # Main structure BELOW shelf level (up to 1200mm)
    structure_lower = Rectangle((0, 0), OVERALL_DEPTH, SHELF_TOP_HEIGHT,
                         fill=False, edgecolor='black', linewidth=3)
    ax.add_patch(structure_lower)
    
    # Walls above shelf (from front to rear shelf depth only, as rear wall stops at shelf)
    # Front section of wall continues above shelf
    wall_above_front = Rectangle((0, SHELF_TOP_HEIGHT), OVERALL_DEPTH - REAR_SHELF_DEPTH, 
                                WALL_HEIGHT_ABOVE_SHELF,
                                fill=True, facecolor='lightgray', edgecolor='darkgray',
                                linewidth=2, alpha=0.3)
    ax.add_patch(wall_above_front)
    ax.text((OVERALL_DEPTH - REAR_SHELF_DEPTH)/2, SHELF_TOP_HEIGHT + WALL_HEIGHT_ABOVE_SHELF/2,
            f'Front walls\ncontinue\n{WALL_HEIGHT_ABOVE_SHELF}mm\nabove shelf',
            ha='center', va='center', fontsize=10, fontweight='bold')
    
    # Rear shelf structure (at back) - THE REAR WALL STOPS HERE AT 1200mm
    rear_shelf = Rectangle((OVERALL_DEPTH - REAR_SHELF_DEPTH, SHELF_TOP_HEIGHT - 20), 
                          REAR_SHELF_DEPTH, 20,
                          fill=True, facecolor='wheat', edgecolor='brown', linewidth=2)
    ax.add_patch(rear_shelf)
    ax.text(OVERALL_DEPTH - REAR_SHELF_DEPTH/2, SHELF_TOP_HEIGHT + 100,
            f'REAR WALL\nSTOPS HERE\nat {SHELF_TOP_HEIGHT}mm\n(shelf top)',
            ha='center', va='bottom', fontsize=10, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))
    
    # Mattress and platform
    mattress_depth_start = OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_TOLERANCE - MATTRESS_DEPTH
    
    # Platform
    platform = Rectangle((mattress_depth_start, PLATFORM_TOP - 20), MATTRESS_DEPTH, 20,
                        fill=True, facecolor='brown', edgecolor='saddlebrown', linewidth=2)
    ax.add_patch(platform)
    
    # Mattress
    mattress = Rectangle((mattress_depth_start, PLATFORM_TOP), MATTRESS_DEPTH, MATTRESS_THICKNESS,
                        fill=True, facecolor='lightgreen', edgecolor='darkgreen',
                        linewidth=2, alpha=0.7)
    ax.add_patch(mattress)
    
    # Base planks
    base = Rectangle((mattress_depth_start, 0), MATTRESS_DEPTH, PLATFORM_BASE_HEIGHT,
                    fill=True, facecolor='sandybrown', edgecolor='brown',
                    linewidth=2, alpha=0.5)
    ax.add_patch(base)
    
    # Walls - REAR WALL STOPS AT SHELF HEIGHT (1200mm)!
    ax.plot([OVERALL_DEPTH - REAR_SHELF_DEPTH, OVERALL_DEPTH - REAR_SHELF_DEPTH],
            [0, SHELF_TOP_HEIGHT], 'brown', linewidth=4, label=f'Rear Wall (stops at {SHELF_TOP_HEIGHT}mm)')
    
    # Mark the termination point clearly
    ax.plot(OVERALL_DEPTH - REAR_SHELF_DEPTH, SHELF_TOP_HEIGHT, 'ro', markersize=12,
            label='Rear wall termination')
    
    # Dimensions
    ax.annotate('', xy=(OVERALL_DEPTH, -300), xytext=(0, -300),
                arrowprops=dict(arrowstyle='<->', lw=2, color='black'))
    ax.text(OVERALL_DEPTH/2, -350, f'{OVERALL_DEPTH}mm', ha='center', fontsize=12, fontweight='bold')
    
    ax.annotate('', xy=(OVERALL_DEPTH + 300, OVERALL_HEIGHT), xytext=(OVERALL_DEPTH + 300, 0),
                arrowprops=dict(arrowstyle='<->', lw=2, color='black'))
    ax.text(OVERALL_DEPTH + 350, OVERALL_HEIGHT/2, f'{OVERALL_HEIGHT}mm\n(FULL HEIGHT)',
            ha='left', va='center', fontsize=11, fontweight='bold')
    
    ax.set_xlim(-500, OVERALL_DEPTH + 600)
    ax.set_ylim(-500, OVERALL_HEIGHT + 200)
    ax.set_xlabel('Depth (mm)', fontsize=11)
    ax.set_ylabel('Height (mm)', fontsize=11)
    ax.set_title('SIDE ELEVATION (From Foot End) - Rear wall stops at shelf (1200mm)', 
                fontsize=16, fontweight='bold')
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/side_elevation_FIXED.png', dpi=300, bbox_inches='tight')
    print("✓ Side elevation (FIXED) saved")
    plt.close()

def create_3d_visualization():
    """Create 3D wireframe visualization of the cabin bed"""
    from mpl_toolkits.mplot3d import Axes3D
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    
    fig = plt.figure(figsize=(16, 12))
    ax = fig.add_subplot(111, projection='3d')
    
    # Helper function to draw a box
    def draw_box(ax, x, y, z, width, depth, height, color='blue', alpha=0.3, label=None):
        """Draw a 3D box from corner (x,y,z) with given dimensions"""
        vertices = [
            [x, y, z],
            [x + width, y, z],
            [x + width, y + depth, z],
            [x, y + depth, z],
            [x, y, z + height],
            [x + width, y, z + height],
            [x + width, y + depth, z + height],
            [x, y + depth, z + height]
        ]
        
        faces = [
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # front
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # back
            [vertices[0], vertices[3], vertices[7], vertices[4]],  # left
            [vertices[1], vertices[2], vertices[6], vertices[5]],  # right
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # bottom
            [vertices[4], vertices[5], vertices[6], vertices[7]]   # top
        ]
        
        poly = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor='black', linewidth=1.5)
        ax.add_collection3d(poly)
        return vertices
    
    # Coordinate system: X = width, Y = depth, Z = height
    # Foot wall is at X = -98 to 0
    # Internal space is X = 0 to 2246
    
    # 1. FOOT WALL (98mm thick, full depth and height up to 1200mm)
    draw_box(ax, -FOOT_WALL_THICKNESS, 0, 0, FOOT_WALL_THICKNESS, OVERALL_DEPTH, SHELF_TOP_HEIGHT,
            color='darkorange', alpha=0.5, label='Foot Wall Extension')
    
    # 2. REAR WALL (from foot to head, 210mm deep, up to shelf at 1200mm)
    rear_wall_y = OVERALL_DEPTH - REAR_SHELF_DEPTH
    draw_box(ax, 0, rear_wall_y, 0, INTERNAL_WIDTH, REAR_SHELF_DEPTH, SHELF_TOP_HEIGHT,
            color='wheat', alpha=0.4, label='Rear Wall + Shelf')
    
    # 3. HEAD WALL (326mm wide, from rear to mattress front, up to shelf)
    head_wall_x = INTERNAL_WIDTH - HEAD_WALL_WIDTH
    head_wall_y_start = rear_wall_y - BED_FRAME_DEPTH - BED_TOLERANCE
    draw_box(ax, head_wall_x, head_wall_y_start, 0, HEAD_WALL_WIDTH, 
            rear_wall_y - head_wall_y_start, SHELF_TOP_HEIGHT,
            color='lightgray', alpha=0.5, label='Head Wall')
    
    # 4. FRONT WALLS continue above shelf (but rear does not)
    draw_box(ax, 0, 0, SHELF_TOP_HEIGHT, INTERNAL_WIDTH, rear_wall_y, WALL_HEIGHT_ABOVE_SHELF,
            color='lightgray', alpha=0.2, label='Front Walls Above Shelf')
    
    # 5. BED PLATFORM (raised 173mm, frame height 357mm)
    platform_x = 200  # After wall cut-in
    platform_y = rear_wall_y - BED_TOLERANCE - BED_FRAME_DEPTH
    draw_box(ax, platform_x, platform_y, PLATFORM_BASE_HEIGHT, BED_FRAME_WIDTH, BED_FRAME_DEPTH, 
            PLATFORM_FRAME_HEIGHT, color='brown', alpha=0.3, label='Bed Platform')
    
    # 6. MATTRESS (on top of platform)
    mattress_x = platform_x + BED_TOLERANCE
    mattress_y = platform_y + BED_TOLERANCE
    draw_box(ax, mattress_x, mattress_y, PLATFORM_TOP, MATTRESS_WIDTH, MATTRESS_DEPTH,
            MATTRESS_THICKNESS, color='lightgreen', alpha=0.6, label='Mattress')
    
    # 7. DRAWERS (two, in the void below platform)
    drawer_height_actual = PLATFORM_TOP - DRAWER_CLEARANCE_BOTTOM - DRAWER_CLEARANCE_TOP
    # Left drawer
    draw_box(ax, 300, 100, DRAWER_CLEARANCE_BOTTOM, DRAWER_WIDTH, DRAWER_DEPTH,
            drawer_height_actual, color='lightblue', alpha=0.5, label='Drawer')
    # Right drawer
    draw_box(ax, INTERNAL_WIDTH - DRAWER_WIDTH - 300, 100, DRAWER_CLEARANCE_BOTTOM,
            DRAWER_WIDTH, DRAWER_DEPTH, drawer_height_actual, color='lightblue', alpha=0.5)
    
    # 8. ACCESS OPENING (marked as transparent/open area)
    # This is the front section between the drawers
    access_x = ACCESS_START_FROM_LEFT
    ax.text(access_x + ACCESS_WIDTH/2, 50, PLATFORM_TOP + ACCESS_HEIGHT/2,
           'ACCESS\nOPENING\n(curtains)', fontsize=12, ha='center', fontweight='bold',
           bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # Labels and annotations
    ax.text(INTERNAL_WIDTH/2, OVERALL_DEPTH + 200, OVERALL_HEIGHT + 100,
           f'CABIN BED 3D VIEW\nTotal: {OVERALL_WIDTH}mm W × {OVERALL_DEPTH}mm D × {OVERALL_HEIGHT}mm H',
           fontsize=14, ha='center', fontweight='bold',
           bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))
    
    # Set axis properties
    ax.set_xlabel('Width (mm)', fontsize=11, labelpad=10)
    ax.set_ylabel('Depth (mm)', fontsize=11, labelpad=10)
    ax.set_zlabel('Height (mm)', fontsize=11, labelpad=10)
    ax.set_title('3D VISUALIZATION - Cabin Bed with Foot Wall Extension', 
                fontsize=16, fontweight='bold', pad=20)
    
    # Set axis limits
    ax.set_xlim(-FOOT_WALL_THICKNESS - 200, INTERNAL_WIDTH + 200)
    ax.set_ylim(-200, OVERALL_DEPTH + 300)
    ax.set_zlim(0, OVERALL_HEIGHT + 200)
    
    # Set viewing angle for best visibility
    ax.view_init(elev=25, azim=135)
    
    # Add grid
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/3d_visualization_FIXED.png', dpi=300, bbox_inches='tight')
    print("✓ 3D visualization (FIXED) saved")
    plt.close()

if __name__ == "__main__":
    print("\n" + "="*70)
    print(" CABIN BED TECHNICAL DRAWINGS - CORRECTED VERSION")
    print("="*70 + "\n")
    
    print("Generating FIXED drawings with:")
    print(f"  • Full height: {OVERALL_HEIGHT}mm")
    print(f"  • Access opening measured from full {OVERALL_HEIGHT}mm height")
    print(f"  • Internal width: {INTERNAL_WIDTH}mm")
    print(f"  • Foot wall extension: +{FOOT_WALL_THICKNESS}mm")
    print(f"  • TOTAL external width: {OVERALL_WIDTH}mm ({INTERNAL_WIDTH} + {FOOT_WALL_THICKNESS})")
    print(f"  • Overall depth: {OVERALL_DEPTH}mm")
    print(f"  • Head wall: {HEAD_WALL_WIDTH}mm")
    print(f"  • Rear wall stops at {SHELF_TOP_HEIGHT}mm (shelf level)")
    print()
    
    draw_front_elevation_corrected()
    draw_top_plan_corrected()
    draw_side_elevation_corrected()
    create_3d_visualization()
    
    print("\n" + "="*70)
    print(" ✓ ALL CORRECTED DRAWINGS GENERATED!")
    print("="*70)
    print("\nGenerated files:")
    print("  - front_elevation_FIXED.png")
    print("  - top_plan_FIXED.png")
    print("  - side_elevation_FIXED.png")
    print("  - 3d_visualization_FIXED.png")
    print("\n")
