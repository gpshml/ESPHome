#!/usr/bin/env python3
"""
Cabin Bed Technical Drawings Generator
Generates 2D technical drawings with dimensions
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle
import numpy as np

# All measurements in mm
# Updated: 10mm tolerance around bed frame
# WIDTH stays same (2246mm) - head shelf REDUCES by 20mm
# DEPTH increases (extends into room) - overall depth increases by 20mm

MATTRESS_WIDTH = 1900
MATTRESS_DEPTH = 900
BED_TOLERANCE = 10  # 10mm tolerance around mattress
BED_WIDTH = MATTRESS_WIDTH + (BED_TOLERANCE * 2)  # 1920mm
BED_DEPTH = MATTRESS_DEPTH + (BED_TOLERANCE * 2)  # 920mm

OVERALL_WIDTH = 2246  # STAYS SAME - cannot change
OVERALL_DEPTH = 1222 + 20  # Can extend into room = 1242mm
OVERALL_HEIGHT = 2380

PLATFORM_HEIGHT = 530  # 173 + 357
MATTRESS_HEIGHT = 220
MATTRESS_TOP = 750  # 530 + 220

REAR_SHELF_DEPTH = 210
HEAD_SHELF_WIDTH = 346 - 20  # REDUCES by 20mm (10mm each end for tolerance) = 326mm
SHELF_HEIGHT = 1200

ACCESS_WIDTH = 1300
ACCESS_HEIGHT = 580
ACCESS_START = 300
ACCESS_BOTTOM = 530

DRAWER_WIDTH = 880  # Updated: 6 legs total (4 corners + 2 middle)
DRAWER_HEIGHT = 150
DRAWER_DEPTH = 900

TABLET_CENTER_HEIGHT = 975

def mm_to_m(mm):
    """Convert millimeters to meters for display"""
    return mm / 1000

def draw_dimension_line(ax, x1, y1, x2, y2, text, offset=50, fontsize=8):
    """Draw a dimension line with arrows and text"""
    # Draw main line
    ax.plot([x1, x2], [y1, y2], 'k-', linewidth=0.5)
    
    # Draw end arrows
    arrow_size = 20
    if x1 == x2:  # Vertical line
        ax.plot([x1-10, x1, x1+10], [y1, y1-arrow_size, y1], 'k-', linewidth=0.5)
        ax.plot([x2-10, x2, x2+10], [y2, y2+arrow_size, y2], 'k-', linewidth=0.5)
        # Text
        mid_y = (y1 + y2) / 2
        ax.text(x1 + offset, mid_y, text, fontsize=fontsize, ha='left', va='center',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='none'))
    else:  # Horizontal line
        ax.plot([x1, x1+arrow_size, x1], [y1-10, y1, y1+10], 'k-', linewidth=0.5)
        ax.plot([x2, x2-arrow_size, x2], [y2-10, y2, y2+10], 'k-', linewidth=0.5)
        # Text
        mid_x = (x1 + x2) / 2
        ax.text(mid_x, y1 + offset, text, fontsize=fontsize, ha='center', va='bottom',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='none'))

def draw_front_elevation():
    """Draw front elevation view"""
    fig, ax = plt.subplots(figsize=(14, 10))
    ax.set_aspect('equal')
    
    # Main structure outline
    structure = Rectangle((0, 0), OVERALL_WIDTH, SHELF_HEIGHT, 
                          fill=False, edgecolor='black', linewidth=2)
    ax.add_patch(structure)
    
    # Access opening
    access_opening = Rectangle((ACCESS_START, ACCESS_BOTTOM), ACCESS_WIDTH, ACCESS_HEIGHT,
                               fill=False, edgecolor='red', linewidth=2, linestyle='--')
    ax.add_patch(access_opening)
    
    # Add "CURTAINS" text
    ax.text(ACCESS_START + ACCESS_WIDTH/2, ACCESS_BOTTOM + ACCESS_HEIGHT/2, 
            'CURTAINS\n(Access Opening)', ha='center', va='center', fontsize=10,
            bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.7))
    
    # Platform level line
    ax.plot([0, OVERALL_WIDTH], [PLATFORM_HEIGHT, PLATFORM_HEIGHT], 
            'b--', linewidth=1, label='Platform Top (530mm)')
    
    # Mattress top line
    ax.plot([0, OVERALL_WIDTH], [MATTRESS_TOP, MATTRESS_TOP], 
            'g--', linewidth=1, label='Mattress Top (750mm)')
    
    # Shelf top line
    ax.plot([0, OVERALL_WIDTH], [SHELF_HEIGHT, SHELF_HEIGHT], 
            'purple', linewidth=2, label='Shelf Top (1200mm)')
    
    # Drawer locations (below platform)
    drawer_y = 173 / 2 - DRAWER_HEIGHT / 2
    
    # Left drawer
    left_drawer_x = 300 - DRAWER_WIDTH - 100  # Approximate position
    if left_drawer_x < 50:
        left_drawer_x = 50
    left_drawer = Rectangle((left_drawer_x, drawer_y), DRAWER_WIDTH, DRAWER_HEIGHT,
                            fill=True, facecolor='lightblue', edgecolor='blue', linewidth=1.5)
    ax.add_patch(left_drawer)
    ax.text(left_drawer_x + DRAWER_WIDTH/2, drawer_y + DRAWER_HEIGHT/2,
            'DRAWER 1', ha='center', va='center', fontsize=8)
    
    # Right drawer
    right_drawer_x = ACCESS_START + ACCESS_WIDTH + 100  # Approximate position
    if right_drawer_x + DRAWER_WIDTH > OVERALL_WIDTH - 50:
        right_drawer_x = OVERALL_WIDTH - DRAWER_WIDTH - 50
    right_drawer = Rectangle((right_drawer_x, drawer_y), DRAWER_WIDTH, DRAWER_HEIGHT,
                             fill=True, facecolor='lightblue', edgecolor='blue', linewidth=1.5)
    ax.add_patch(right_drawer)
    ax.text(right_drawer_x + DRAWER_WIDTH/2, drawer_y + DRAWER_HEIGHT/2,
            'DRAWER 2', ha='center', va='center', fontsize=8)
    
    # Tablet position (on rear wall - shown as circle)
    tablet_x = OVERALL_WIDTH / 2
    tablet_circle = Circle((tablet_x, TABLET_CENTER_HEIGHT), 80, 
                          fill=True, facecolor='lightgray', edgecolor='black', linewidth=1.5)
    ax.add_patch(tablet_circle)
    ax.text(tablet_x, TABLET_CENTER_HEIGHT, '7" TABLET', ha='center', va='center', fontsize=7)
    
    # Dimensions
    # Overall width
    draw_dimension_line(ax, 0, -200, OVERALL_WIDTH, -200, f'{OVERALL_WIDTH}mm', offset=30)
    
    # Overall height
    draw_dimension_line(ax, OVERALL_WIDTH + 200, 0, OVERALL_WIDTH + 200, SHELF_HEIGHT, 
                       f'{SHELF_HEIGHT}mm', offset=50)
    
    # Access opening width
    draw_dimension_line(ax, ACCESS_START, ACCESS_BOTTOM - 150, 
                       ACCESS_START + ACCESS_WIDTH, ACCESS_BOTTOM - 150,
                       f'{ACCESS_WIDTH}mm', offset=30)
    
    # Access from left
    draw_dimension_line(ax, 0, ACCESS_BOTTOM - 300, ACCESS_START, ACCESS_BOTTOM - 300,
                       f'{ACCESS_START}mm', offset=30)
    
    # Access opening height
    draw_dimension_line(ax, ACCESS_START - 150, ACCESS_BOTTOM, 
                       ACCESS_START - 150, ACCESS_BOTTOM + ACCESS_HEIGHT,
                       f'{ACCESS_HEIGHT}mm', offset=50)
    
    # Platform height
    draw_dimension_line(ax, -150, 0, -150, PLATFORM_HEIGHT, f'{PLATFORM_HEIGHT}mm', offset=50)
    
    ax.set_xlim(-400, OVERALL_WIDTH + 400)
    ax.set_ylim(-400, SHELF_HEIGHT + 200)
    ax.set_xlabel('Width (mm)', fontsize=10)
    ax.set_ylabel('Height (mm)', fontsize=10)
    ax.set_title('FRONT ELEVATION - Cabin Bed', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/front_elevation.png', dpi=300, bbox_inches='tight')
    print("✓ Front elevation saved: front_elevation.png")
    plt.close()

def draw_top_plan():
    """Draw top plan view - CORRECTED ORIENTATION"""
    fig, ax = plt.subplots(figsize=(14, 10))
    ax.set_aspect('equal')
    
    # CORRECTED LAYOUT:
    # Top = Rear wall (back of room, shelf)
    # Bottom = Front (access opening into room)
    # Left = Foot end (against existing wall)
    # Right = Head end (solid wall/fascia, 326mm)
    
    # Overall structure
    structure = Rectangle((0, 0), OVERALL_WIDTH, OVERALL_DEPTH,
                         fill=False, edgecolor='black', linewidth=2)
    ax.add_patch(structure)
    
    # Rear shelf area (at TOP of bed, back wall)
    rear_shelf = Rectangle((0, OVERALL_DEPTH - REAR_SHELF_DEPTH), OVERALL_WIDTH, REAR_SHELF_DEPTH,
                          fill=True, facecolor='wheat', edgecolor='brown', linewidth=1.5, alpha=0.5)
    ax.add_patch(rear_shelf)
    ax.text(OVERALL_WIDTH/2, OVERALL_DEPTH - REAR_SHELF_DEPTH/2, 'REAR SHELF (210mm) - Back Wall',
            ha='center', va='center', fontsize=10)
    
    # Head shelf area (RIGHT side - solid wall/fascia perpendicular to bed)
    # This is a WALL not a shelf - runs from rear to front across the head end
    head_wall_y_start = OVERALL_DEPTH - REAR_SHELF_DEPTH - BED_DEPTH - BED_TOLERANCE
    head_wall_y_end = OVERALL_DEPTH - REAR_SHELF_DEPTH
    head_wall = Rectangle((OVERALL_WIDTH - HEAD_SHELF_WIDTH, head_wall_y_start), 
                          HEAD_SHELF_WIDTH, head_wall_y_end - head_wall_y_start,
                          fill=True, facecolor='lightgray', edgecolor='darkgray', linewidth=2, alpha=0.7)
    ax.add_patch(head_wall)
    ax.text(OVERALL_WIDTH - HEAD_SHELF_WIDTH/2, (head_wall_y_start + head_wall_y_end)/2, 
            'HEAD\nWALL\n326mm',
            ha='center', va='center', fontsize=9, fontweight='bold')
    
    # Mattress area
    mattress_start_y = REAR_SHELF_DEPTH
    mattress_start_x = (OVERALL_WIDTH - BED_WIDTH) / 2  # Center the bed
    mattress = Rectangle((mattress_start_x, mattress_start_y), BED_WIDTH, BED_DEPTH,
                        fill=True, facecolor='lightgreen', edgecolor='darkgreen', 
                        linewidth=2, alpha=0.6)
    ax.add_patch(mattress)
    ax.text(mattress_start_x + BED_WIDTH/2, mattress_start_y + BED_DEPTH/2,
            'MATTRESS\n1900 × 900mm', ha='center', va='center', fontsize=11, fontweight='bold')
    
    # Access opening location (front)
    access_y = OVERALL_DEPTH
    access_start_x = ACCESS_START
    access_arrow = patches.FancyBboxPatch((access_start_x, access_y - 100), ACCESS_WIDTH, 100,
                                         boxstyle="round,pad=10", 
                                         fill=True, facecolor='yellow', 
                                         edgecolor='red', linewidth=2, alpha=0.4)
    ax.add_patch(access_arrow)
    ax.text(access_start_x + ACCESS_WIDTH/2, access_y - 50,
            'ACCESS OPENING\n(1300mm wide)', ha='center', va='center', fontsize=9,
            fontweight='bold')
    
    # Left wall extension area
    left_extension = Rectangle((0, OVERALL_DEPTH - 237), 200, 237,
                              fill=True, facecolor='orange', edgecolor='darkorange',
                              linewidth=1.5, alpha=0.4)
    ax.add_patch(left_extension)
    ax.text(100, OVERALL_DEPTH - 120, 'WALL\nEXT.', ha='center', va='center', fontsize=7)
    
    # Middle support legs (2 at 940mm from left)
    leg_x = 940
    leg_size = 20
    leg1 = Rectangle((mattress_start_x + leg_x - leg_size/2, mattress_start_y + 200), 
                     leg_size, leg_size,
                     fill=True, facecolor='black', edgecolor='black')
    ax.add_patch(leg1)
    
    leg2 = Rectangle((mattress_start_x + leg_x - leg_size/2, mattress_start_y + 600),
                     leg_size, leg_size,
                     fill=True, facecolor='black', edgecolor='black')
    ax.add_patch(leg2)
    
    ax.text(mattress_start_x + leg_x + 40, mattress_start_y + 400, 'Support\nLegs',
            ha='left', va='center', fontsize=7)
    
    # Dimensions
    # Overall width
    draw_dimension_line(ax, 0, -200, OVERALL_WIDTH, -200, f'{OVERALL_WIDTH}mm', offset=30)
    
    # Overall depth
    draw_dimension_line(ax, OVERALL_WIDTH + 200, 0, OVERALL_WIDTH + 200, OVERALL_DEPTH,
                       f'{OVERALL_DEPTH}mm', offset=50)
    
    # Rear shelf depth
    draw_dimension_line(ax, OVERALL_WIDTH + 400, 0, OVERALL_WIDTH + 400, REAR_SHELF_DEPTH,
                       f'{REAR_SHELF_DEPTH}mm', offset=50)
    
    # Mattress dimensions
    draw_dimension_line(ax, mattress_start_x, mattress_start_y - 100,
                       mattress_start_x + BED_WIDTH, mattress_start_y - 100,
                       f'{BED_WIDTH}mm', offset=30)
    
    draw_dimension_line(ax, mattress_start_x + BED_WIDTH + 100, mattress_start_y,
                       mattress_start_x + BED_WIDTH + 100, mattress_start_y + BED_DEPTH,
                       f'{BED_DEPTH}mm', offset=50)
    
    # Head shelf width
    draw_dimension_line(ax, OVERALL_WIDTH - HEAD_SHELF_WIDTH, -100,
                       OVERALL_WIDTH, -100, f'{HEAD_SHELF_WIDTH}mm', offset=30)
    
    ax.set_xlim(-400, OVERALL_WIDTH + 600)
    ax.set_ylim(-400, OVERALL_DEPTH + 200)
    ax.set_xlabel('Width (mm)', fontsize=10)
    ax.set_ylabel('Depth (mm)', fontsize=10)
    ax.set_title('TOP PLAN VIEW - Cabin Bed', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Add compass
    ax.annotate('', xy=(OVERALL_WIDTH - 200, OVERALL_DEPTH - 100), 
                xytext=(OVERALL_WIDTH - 200, OVERALL_DEPTH - 300),
                arrowprops=dict(arrowstyle='->', lw=2, color='red'))
    ax.text(OVERALL_WIDTH - 150, OVERALL_DEPTH - 200, 'N', fontsize=14, fontweight='bold', color='red')
    
    plt.tight_layout()
    plt.savefig('/workspace/top_plan.png', dpi=300, bbox_inches='tight')
    print("✓ Top plan saved: top_plan.png")
    plt.close()

def draw_side_elevation():
    """Draw side elevation (from left)"""
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.set_aspect('equal')
    
    # Main structure
    structure = Rectangle((0, 0), OVERALL_DEPTH, SHELF_HEIGHT,
                         fill=False, edgecolor='black', linewidth=2)
    ax.add_patch(structure)
    
    # Rear shelf
    rear_shelf = Rectangle((0, SHELF_HEIGHT - 20), REAR_SHELF_DEPTH, 20,
                          fill=True, facecolor='wheat', edgecolor='brown', linewidth=1.5)
    ax.add_patch(rear_shelf)
    ax.text(REAR_SHELF_DEPTH/2, SHELF_HEIGHT - 10, 'Rear Shelf',
            ha='center', va='center', fontsize=7)
    
    # Platform
    platform = Rectangle((REAR_SHELF_DEPTH, PLATFORM_HEIGHT - 20), BED_DEPTH, 20,
                        fill=True, facecolor='brown', edgecolor='saddlebrown', linewidth=1.5)
    ax.add_patch(platform)
    
    # Mattress
    mattress = Rectangle((REAR_SHELF_DEPTH, PLATFORM_HEIGHT), BED_DEPTH, MATTRESS_HEIGHT,
                        fill=True, facecolor='lightgreen', edgecolor='darkgreen',
                        linewidth=2, alpha=0.6)
    ax.add_patch(mattress)
    ax.text(REAR_SHELF_DEPTH + BED_DEPTH/2, PLATFORM_HEIGHT + MATTRESS_HEIGHT/2,
            'MATTRESS', ha='center', va='center', fontsize=10)
    
    # Base planks
    base = Rectangle((REAR_SHELF_DEPTH, 0), BED_DEPTH, 173,
                    fill=True, facecolor='sandybrown', edgecolor='brown',
                    linewidth=1.5, alpha=0.5)
    ax.add_patch(base)
    ax.text(REAR_SHELF_DEPTH + BED_DEPTH/2, 86.5, 'Base Planks\n(173mm)',
            ha='center', va='center', fontsize=8)
    
    # Rear wall (vertical)
    ax.plot([REAR_SHELF_DEPTH, REAR_SHELF_DEPTH], [0, SHELF_HEIGHT], 
            'brown', linewidth=3, label='Rear Wall')
    
    # Front wall (vertical) - with opening
    ax.plot([REAR_SHELF_DEPTH + BED_DEPTH, REAR_SHELF_DEPTH + BED_DEPTH], 
            [0, ACCESS_BOTTOM], 'brown', linewidth=3)
    ax.plot([REAR_SHELF_DEPTH + BED_DEPTH, REAR_SHELF_DEPTH + BED_DEPTH], 
            [ACCESS_BOTTOM + ACCESS_HEIGHT, SHELF_HEIGHT], 
            'brown', linewidth=3, label='Front Wall')
    
    # Access opening indicator
    ax.plot([REAR_SHELF_DEPTH + BED_DEPTH, REAR_SHELF_DEPTH + BED_DEPTH],
            [ACCESS_BOTTOM, ACCESS_BOTTOM + ACCESS_HEIGHT],
            'r--', linewidth=4, label='Access Opening')
    
    # Floor line
    ax.plot([0, OVERALL_DEPTH], [0, 0], 'k-', linewidth=3)
    
    # Dimensions
    # Overall depth
    draw_dimension_line(ax, 0, -200, OVERALL_DEPTH, -200, f'{OVERALL_DEPTH}mm', offset=30)
    
    # Rear shelf depth
    draw_dimension_line(ax, 0, -350, REAR_SHELF_DEPTH, -350, f'{REAR_SHELF_DEPTH}mm', offset=30)
    
    # Bed depth
    draw_dimension_line(ax, REAR_SHELF_DEPTH, -350, REAR_SHELF_DEPTH + BED_DEPTH, -350,
                       f'{BED_DEPTH}mm', offset=30)
    
    # Overall height
    draw_dimension_line(ax, OVERALL_DEPTH + 200, 0, OVERALL_DEPTH + 200, SHELF_HEIGHT,
                       f'{SHELF_HEIGHT}mm', offset=50)
    
    # Base height
    draw_dimension_line(ax, -150, 0, -150, 173, '173mm', offset=50)
    
    # Platform to mattress
    draw_dimension_line(ax, -150, PLATFORM_HEIGHT, -150, MATTRESS_TOP,
                       f'{MATTRESS_HEIGHT}mm', offset=50)
    
    # Tablet position line
    ax.plot([0, REAR_SHELF_DEPTH], [TABLET_CENTER_HEIGHT, TABLET_CENTER_HEIGHT],
            'gray', linewidth=1, linestyle=':', label=f'Tablet @ {TABLET_CENTER_HEIGHT}mm')
    
    ax.set_xlim(-400, OVERALL_DEPTH + 400)
    ax.set_ylim(-500, SHELF_HEIGHT + 200)
    ax.set_xlabel('Depth (mm)', fontsize=10)
    ax.set_ylabel('Height (mm)', fontsize=10)
    ax.set_title('SIDE ELEVATION (From Left) - Cabin Bed', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/workspace/side_elevation.png', dpi=300, bbox_inches='tight')
    print("✓ Side elevation saved: side_elevation.png")
    plt.close()

def draw_drawer_detail():
    """Draw drawer construction detail"""
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect('equal')
    
    # Drawer box - front view
    drawer_front = Rectangle((0, 0), DRAWER_WIDTH, DRAWER_HEIGHT,
                            fill=True, facecolor='lightblue', edgecolor='blue', linewidth=2)
    ax.add_patch(drawer_front)
    
    # Handle position
    handle_y = DRAWER_HEIGHT / 2
    handle_x = DRAWER_WIDTH / 2
    handle = Rectangle((handle_x - 100, handle_y - 15), 200, 30,
                      fill=True, facecolor='silver', edgecolor='black', linewidth=1.5)
    ax.add_patch(handle)
    ax.text(handle_x, handle_y, 'HANDLE', ha='center', va='center', fontsize=8)
    
    # Material callout
    ax.text(DRAWER_WIDTH/2, DRAWER_HEIGHT + 100, 
            'DRAWER FRONT\n18mm Plywood/MDF\nFinished face',
            ha='center', va='bottom', fontsize=10,
            bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow'))
    
    # Dimensions
    draw_dimension_line(ax, 0, -100, DRAWER_WIDTH, -100, f'{DRAWER_WIDTH}mm', offset=30)
    draw_dimension_line(ax, DRAWER_WIDTH + 100, 0, DRAWER_WIDTH + 100, DRAWER_HEIGHT,
                       f'{DRAWER_HEIGHT}mm', offset=50)
    
    ax.set_xlim(-200, DRAWER_WIDTH + 300)
    ax.set_ylim(-200, DRAWER_HEIGHT + 250)
    ax.set_title('DRAWER DETAIL (Front View)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.axis('off')
    
    plt.tight_layout()
    plt.savefig('/workspace/drawer_detail.png', dpi=300, bbox_inches='tight')
    print("✓ Drawer detail saved: drawer_detail.png")
    plt.close()
    
    # Drawer box - side view
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_aspect('equal')
    
    # Side view outline
    drawer_side = Rectangle((0, 0), DRAWER_DEPTH, DRAWER_HEIGHT - 20,
                           fill=True, facecolor='lightblue', edgecolor='blue', linewidth=2)
    ax.add_patch(drawer_side)
    
    # Base
    base = Rectangle((0, 0), DRAWER_DEPTH, 10,
                    fill=True, facecolor='lightgray', edgecolor='gray', linewidth=1)
    ax.add_patch(base)
    ax.text(DRAWER_DEPTH/2, 5, '6mm base', ha='center', va='center', fontsize=7)
    
    # Side label
    ax.text(DRAWER_DEPTH/2, DRAWER_HEIGHT/2, 
            'DRAWER SIDE\n12mm Plywood\nHeight: ~130mm internal',
            ha='center', va='center', fontsize=9)
    
    # Runner indication
    runner = Rectangle((-50, -30), DRAWER_DEPTH + 100, 20,
                      fill=True, facecolor='silver', edgecolor='black', 
                      linewidth=1.5, alpha=0.5)
    ax.add_patch(runner)
    ax.text(DRAWER_DEPTH/2, -20, 'Floor-Mounted Runner/Slider', 
            ha='center', va='center', fontsize=8)
    
    # Dimensions
    draw_dimension_line(ax, 0, DRAWER_HEIGHT + 50, DRAWER_DEPTH, DRAWER_HEIGHT + 50,
                       f'{DRAWER_DEPTH}mm depth', offset=30)
    
    ax.set_xlim(-150, DRAWER_DEPTH + 150)
    ax.set_ylim(-100, DRAWER_HEIGHT + 150)
    ax.set_title('DRAWER DETAIL (Side View)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.axis('off')
    
    plt.tight_layout()
    plt.savefig('/workspace/drawer_side_detail.png', dpi=300, bbox_inches='tight')
    print("✓ Drawer side detail saved: drawer_side_detail.png")
    plt.close()

def create_3d_visualization():
    """Create a simple 3D visualization"""
    try:
        from mpl_toolkits.mplot3d import Axes3D
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Helper function to create a box
        def create_box(x, y, z, dx, dy, dz, color='blue', alpha=0.3):
            # Define the 8 vertices of the box
            vertices = [
                [x, y, z],
                [x+dx, y, z],
                [x+dx, y+dy, z],
                [x, y+dy, z],
                [x, y, z+dz],
                [x+dx, y, z+dz],
                [x+dx, y+dy, z+dz],
                [x, y+dy, z+dz]
            ]
            
            # Define the 6 faces
            faces = [
                [vertices[0], vertices[1], vertices[5], vertices[4]],  # front
                [vertices[2], vertices[3], vertices[7], vertices[6]],  # back
                [vertices[0], vertices[3], vertices[7], vertices[4]],  # left
                [vertices[1], vertices[2], vertices[6], vertices[5]],  # right
                [vertices[0], vertices[1], vertices[2], vertices[3]],  # bottom
                [vertices[4], vertices[5], vertices[6], vertices[7]]   # top
            ]
            
            # Create the 3D polygon collection
            poly = Poly3DCollection(faces, alpha=alpha, facecolor=color, 
                                   edgecolor='black', linewidth=0.5)
            return poly
        
        # Convert to meters for better visualization
        scale = 0.001
        
        # Floor reference
        floor = create_box(0, 0, 0, OVERALL_WIDTH*scale, OVERALL_DEPTH*scale, 
                          10*scale, 'gray', 0.1)
        ax.add_collection3d(floor)
        
        # Base planks
        base = create_box(173*scale, REAR_SHELF_DEPTH*scale, 0, 
                         BED_WIDTH*scale, BED_DEPTH*scale, 173*scale, 'sandybrown', 0.4)
        ax.add_collection3d(base)
        
        # Bed frame
        bed_frame = create_box(173*scale, REAR_SHELF_DEPTH*scale, 173*scale,
                              BED_WIDTH*scale, BED_DEPTH*scale, 357*scale, 'brown', 0.5)
        ax.add_collection3d(bed_frame)
        
        # Mattress
        mattress = create_box(173*scale, REAR_SHELF_DEPTH*scale, PLATFORM_HEIGHT*scale,
                             BED_WIDTH*scale, BED_DEPTH*scale, MATTRESS_HEIGHT*scale, 
                             'lightgreen', 0.6)
        ax.add_collection3d(mattress)
        
        # Rear wall
        rear_wall = create_box(0, 0, 0, OVERALL_WIDTH*scale, REAR_SHELF_DEPTH*scale,
                              SHELF_HEIGHT*scale, 'wheat', 0.3)
        ax.add_collection3d(rear_wall)
        
        # Head wall
        head_wall = create_box(OVERALL_WIDTH*scale - HEAD_SHELF_WIDTH*scale, 0, 0,
                              HEAD_SHELF_WIDTH*scale, 900*scale, SHELF_HEIGHT*scale, 
                              'wheat', 0.3)
        ax.add_collection3d(head_wall)
        
        # Left wall (simplified - partial)
        left_wall = create_box(0, 0, 0, 200*scale, OVERALL_DEPTH*scale,
                              SHELF_HEIGHT*scale, 'lightblue', 0.3)
        ax.add_collection3d(left_wall)
        
        # Front wall (with opening gap)
        front_wall_left = create_box(0, OVERALL_DEPTH*scale - 100*scale, 0,
                                    ACCESS_START*scale, 100*scale, SHELF_HEIGHT*scale,
                                    'lightblue', 0.3)
        ax.add_collection3d(front_wall_left)
        
        front_wall_right = create_box((ACCESS_START + ACCESS_WIDTH)*scale, 
                                     OVERALL_DEPTH*scale - 100*scale, 0,
                                     (OVERALL_WIDTH - ACCESS_START - ACCESS_WIDTH)*scale,
                                     100*scale, SHELF_HEIGHT*scale, 'lightblue', 0.3)
        ax.add_collection3d(front_wall_right)
        
        # Set labels and limits
        ax.set_xlabel('Width (m)', fontsize=10)
        ax.set_ylabel('Depth (m)', fontsize=10)
        ax.set_zlabel('Height (m)', fontsize=10)
        ax.set_title('3D VISUALIZATION - Cabin Bed', fontsize=14, fontweight='bold')
        
        # Set equal aspect ratio
        max_dim = max(OVERALL_WIDTH, OVERALL_DEPTH, SHELF_HEIGHT) * scale
        ax.set_xlim(0, max_dim)
        ax.set_ylim(0, max_dim)
        ax.set_zlim(0, max_dim)
        
        # Set viewing angle
        ax.view_init(elev=20, azim=45)
        
        plt.tight_layout()
        plt.savefig('/workspace/3d_visualization.png', dpi=300, bbox_inches='tight')
        print("✓ 3D visualization saved: 3d_visualization.png")
        plt.close()
        
    except Exception as e:
        print(f"⚠ Could not create 3D visualization: {e}")

def generate_materials_list():
    """Generate a detailed materials list with calculations"""
    materials = []
    
    # Calculate plasterboard area
    rear_wall_area = (OVERALL_WIDTH * SHELF_HEIGHT) / 1_000_000  # m²
    head_wall_area = (HEAD_SHELF_WIDTH * SHELF_HEIGHT) / 1_000_000
    front_wall_area = rear_wall_area - ((ACCESS_WIDTH * ACCESS_HEIGHT) / 1_000_000)  # minus opening
    left_wall_area = (OVERALL_DEPTH * SHELF_HEIGHT) / 1_000_000
    
    total_plasterboard = rear_wall_area + head_wall_area + front_wall_area + left_wall_area
    sheets_needed = np.ceil(total_plasterboard / 2.88)  # 2.4m × 1.2m = 2.88m² per sheet
    
    # Calculate metal stud lengths
    perimeter = 2 * (OVERALL_WIDTH + OVERALL_DEPTH)
    stud_height = SHELF_HEIGHT - 12.5  # minus plasterboard thickness
    num_studs = np.ceil(perimeter / 400) + 10  # 400mm centers plus extras
    total_stud_length = num_studs * stud_height / 1000  # meters
    
    # Plywood calculations
    shelf_area = ((OVERALL_WIDTH * REAR_SHELF_DEPTH) + (HEAD_SHELF_WIDTH * 900)) / 1_000_000
    platform_area = (BED_WIDTH * BED_DEPTH) / 1_000_000
    drawer_area = 2 * ((DRAWER_WIDTH * DRAWER_DEPTH) + 2 * (DRAWER_DEPTH * DRAWER_HEIGHT) + 
                       2 * (DRAWER_WIDTH * DRAWER_HEIGHT)) / 1_000_000
    
    materials_text = f"""
# MATERIALS CALCULATION SUMMARY

## Plasterboard (12.5mm)
- Rear wall: {rear_wall_area:.2f} m²
- Head wall: {head_wall_area:.2f} m²
- Front wall: {front_wall_area:.2f} m² (minus opening)
- Left wall: {left_wall_area:.2f} m²
- **Total area: {total_plasterboard:.2f} m²**
- **Sheets needed (2.4m × 1.2m): {int(sheets_needed)} sheets**
- Add 10% wastage: {int(sheets_needed * 1.1)} sheets

## Metal Stud System (70mm)
- Perimeter length: {perimeter / 1000:.2f} m
- Estimated stud count: {int(num_studs)} pieces
- Stud height: {stud_height:.0f} mm
- Floor/head track: {perimeter / 1000 * 2:.2f} m total
- **Order: {int(np.ceil(total_stud_length / 3))} × 3m C-studs**
- **Order: {int(np.ceil(perimeter / 1500))} × 3m floor track**
- **Order: {int(np.ceil(perimeter / 1500))} × 3m head track**

## Plywood 18mm (2440mm × 1220mm sheets)
- Platform: {platform_area:.2f} m² (1 sheet)
- Shelves: {shelf_area:.2f} m² (1 sheet)
- Bed frame: ~0.5 m² (cut from shelves sheet)
- Drawer fronts: {(2 * DRAWER_WIDTH * DRAWER_HEIGHT) / 1_000_000:.2f} m²
- **Total 18mm plywood: 2-3 sheets**

## Plywood 12mm (drawer sides/backs)
- Drawer construction: {drawer_area:.2f} m²
- **Total 12mm plywood: 1 sheet**

## Plywood 6mm (drawer bases)
- 2 × {DRAWER_WIDTH}mm × {DRAWER_DEPTH}mm
- **Total 6mm plywood: 1 sheet** (will have excess)

## Hardware & Fixings
- Drywall screws (25mm): 1000 pack
- Self-drilling screws (metal): 100 pack
- Heavy-duty drawer runners: 2 pairs × 900mm (20kg+ rated)
- Drawer handles: 2 pieces
- Curtain rail: 1300mm length + brackets
- Tablet mount: 1 × 7" universal mount
- Wood screws: various sizes, 100 pack
- Floor fixings: 30 pieces (depends on floor type)

## Finishing Materials
- Jointing tape: 50m roll
- Jointing compound: 12kg tub
- Corner bead: 10m
- Primer: 2.5L
- Paint: 5L (depends on coverage)
- Wood edge tape: 20m roll
- Wood glue: 500ml

## Structural Timber
- Support legs: 20mm × 20mm × 2m (1 length - cut to size)
- Platform base: 18mm plywood (covered above)

## ESTIMATED TOTAL COST (rough guide, UK prices)
- Metal stud system: £80-120
- Plasterboard & finishing: £100-150
- Plywood (all types): £150-200
- Hardware & fixings: £80-120
- Finishing materials: £50-80
- Curtains & accessories: £40-80
- **Total estimated: £500-750**

*Note: Prices are approximate and vary by supplier and location*
"""
    
    with open('/workspace/materials_calculation.txt', 'w') as f:
        f.write(materials_text)
    
    print("✓ Materials calculation saved: materials_calculation.txt")
    return materials_text

if __name__ == "__main__":
    print("\n" + "="*60)
    print("CABIN BED TECHNICAL DRAWINGS GENERATOR")
    print("="*60 + "\n")
    
    print("Generating technical drawings...\n")
    
    draw_front_elevation()
    draw_top_plan()
    draw_side_elevation()
    draw_drawer_detail()
    create_3d_visualization()
    generate_materials_list()
    
    print("\n" + "="*60)
    print("✓ ALL DRAWINGS GENERATED SUCCESSFULLY!")
    print("="*60)
    print("\nGenerated files:")
    print("  - front_elevation.png")
    print("  - top_plan.png")
    print("  - side_elevation.png")
    print("  - drawer_detail.png")
    print("  - drawer_side_detail.png")
    print("  - 3d_visualization.png")
    print("  - materials_calculation.txt")
    print("  - cabin-bed-specifications.md")
    print("\n")
