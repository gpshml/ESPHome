#!/usr/bin/env python3
"""
Generate HTML report with all cabin bed drawings embedded as base64
"""

import base64
import os

def image_to_base64(image_path):
    """Convert image to base64 string"""
    with open(image_path, 'rb') as img_file:
        return base64.b64encode(img_file.read()).decode('utf-8')

# Read all images and convert to base64
print("Converting images to base64...")
front_elevation_b64 = image_to_base64('/workspace/front_elevation_FIXED.png')
top_plan_b64 = image_to_base64('/workspace/top_plan_FIXED.png')
side_elevation_b64 = image_to_base64('/workspace/side_elevation_FIXED.png')
viz_3d_b64 = image_to_base64('/workspace/3d_visualization_FIXED.png')

print("Generating HTML report...")

html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Cabin Bed - Complete Technical Drawings (All Corrections Applied)</title>
    <style>
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            max-width: 1400px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f5f5f5;
        }}
        h1 {{
            color: #2c3e50;
            text-align: center;
            border-bottom: 3px solid #3498db;
            padding-bottom: 10px;
        }}
        h2 {{
            color: #34495e;
            margin-top: 30px;
            border-left: 5px solid #3498db;
            padding-left: 10px;
        }}
        .drawing-section {{
            background: white;
            padding: 20px;
            margin: 20px 0;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }}
        .drawing-image {{
            width: 100%;
            height: auto;
            border: 1px solid #ddd;
            border-radius: 4px;
            margin: 10px 0;
        }}
        .corrections-applied {{
            background: #d4edda;
            border-left: 4px solid #28a745;
            padding: 15px;
            margin: 20px 0;
            border-radius: 4px;
        }}
        .corrections-applied h3 {{
            color: #155724;
            margin-top: 0;
        }}
        .corrections-applied ul {{
            margin: 10px 0;
            padding-left: 20px;
        }}
        .corrections-applied li {{
            margin: 5px 0;
            color: #155724;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
            background: white;
        }}
        th, td {{
            border: 1px solid #ddd;
            padding: 12px;
            text-align: left;
        }}
        th {{
            background-color: #3498db;
            color: white;
            font-weight: bold;
        }}
        tr:nth-child(even) {{
            background-color: #f9f9f9;
        }}
        .highlight {{
            background-color: #fff3cd;
            padding: 2px 5px;
            border-radius: 3px;
            font-weight: bold;
        }}
        .note {{
            background: #fff3cd;
            border-left: 4px solid #ffc107;
            padding: 15px;
            margin: 20px 0;
            border-radius: 4px;
        }}
    </style>
</head>
<body>
    <h1>🛏️ Cabin Bed - Complete Technical Drawings</h1>
    <p style="text-align: center; font-size: 18px; color: #666;">
        All corrections applied - Final version
    </p>

    <div class="corrections-applied">
        <h3>✅ Latest Corrections Applied (Message 14):</h3>
        <ul>
            <li><strong>Front Elevation:</strong> Access opening dimensions now measured from the full 2380mm height structure (not just below the 1200mm shelf)</li>
            <li><strong>Top Plan:</strong> The 2246mm dimension is internal. Added 98mm foot wall extension on the left side, making total external width <span class="highlight">2344mm</span> (2246mm internal + 98mm foot wall)</li>
            <li><strong>Side Elevation (Foot End):</strong> Corrected orientation - rear wall/shelf now properly shown stopping at 1200mm mark where the shelf ends</li>
            <li><strong>3D Visualization:</strong> Restored with all latest corrections including full height (2380mm), correct width (2344mm total), and proper wall terminations</li>
        </ul>
    </div>

    <h2>📐 Key Dimensions Summary</h2>
    <table>
        <tr>
            <th>Component</th>
            <th>Dimension</th>
            <th>Notes</th>
        </tr>
        <tr>
            <td><strong>Overall Width (External)</strong></td>
            <td><span class="highlight">2344mm</span></td>
            <td>2246mm internal + 98mm foot wall extension</td>
        </tr>
        <tr>
            <td><strong>Internal Width</strong></td>
            <td>2246mm</td>
            <td>Mattress + clearances + head wall</td>
        </tr>
        <tr>
            <td><strong>Foot Wall Thickness</strong></td>
            <td>98mm</td>
            <td>70mm metal stud + 2×12.5mm plasterboard (rounded)</td>
        </tr>
        <tr>
            <td><strong>Overall Depth</strong></td>
            <td>1242mm</td>
            <td>Extended into room (+20mm for tolerance)</td>
        </tr>
        <tr>
            <td><strong>Overall Height</strong></td>
            <td>2380mm</td>
            <td>Full height including walls above shelf</td>
        </tr>
        <tr>
            <td><strong>Shelf Top Height</strong></td>
            <td>1200mm</td>
            <td>Rear wall terminates here; front walls continue above</td>
        </tr>
        <tr>
            <td><strong>Mattress Size</strong></td>
            <td>1900mm × 900mm</td>
            <td>10mm tolerance all around (bed frame 1920 × 920mm)</td>
        </tr>
        <tr>
            <td><strong>Head Wall Width</strong></td>
            <td>326mm</td>
            <td>Solid fascia (reduced from 346mm for tolerance)</td>
        </tr>
        <tr>
            <td><strong>Platform Height</strong></td>
            <td>530mm</td>
            <td>173mm base planks + 357mm frame</td>
        </tr>
        <tr>
            <td><strong>Drawer Dimensions</strong></td>
            <td>880mm × 900mm × 500mm (H)</td>
            <td>Two drawers, full height utilization of void</td>
        </tr>
        <tr>
            <td><strong>Access Opening</strong></td>
            <td>1300mm × 580mm</td>
            <td>Starts 300mm from left, height from 530mm to 1110mm</td>
        </tr>
    </table>

    <div class="note">
        <strong>📏 Important:</strong> The 2246mm dimension represents the INTERNAL space. The foot wall extension (98mm) is added to the LEFT side (foot end against existing wall), making the total external width <strong>2344mm</strong>. This wall does NOT count in the internal measurement.
    </div>

    <div class="drawing-section">
        <h2>1. Front Elevation</h2>
        <p>Shows the front view of the cabin bed. Access opening dimensions are referenced from the full 2380mm height structure.</p>
        <img src="data:image/png;base64,{front_elevation_b64}" alt="Front Elevation" class="drawing-image">
    </div>

    <div class="drawing-section">
        <h2>2. Top Plan View</h2>
        <p>Overhead view showing layout. The 98mm foot wall extension is shown on the left (foot end). Total external width = 2344mm.</p>
        <img src="data:image/png;base64,{top_plan_b64}" alt="Top Plan View" class="drawing-image">
    </div>

    <div class="drawing-section">
        <h2>3. Side Elevation (From Foot End)</h2>
        <p>Side view from the foot end. Note that the rear wall stops at the 1200mm shelf level, while front walls continue to full 2380mm height.</p>
        <img src="data:image/png;base64,{side_elevation_b64}" alt="Side Elevation" class="drawing-image">
    </div>

    <div class="drawing-section">
        <h2>4. 3D Visualization</h2>
        <p>Three-dimensional wireframe showing the complete structure with all corrections applied.</p>
        <img src="data:image/png;base64,{viz_3d_b64}" alt="3D Visualization" class="drawing-image">
    </div>

    <h2>🔧 Construction Notes</h2>
    <div class="drawing-section">
        <h3>Metal Stud Frame Construction</h3>
        <ul>
            <li><strong>Studs:</strong> 70mm C-studs at 400mm centers</li>
            <li><strong>Plasterboard:</strong> 12.5mm on each side (25mm total coverage)</li>
            <li><strong>Foot Wall:</strong> 98mm total thickness (70mm stud + plasterboard)</li>
            <li><strong>Horizontal noggins:</strong> Required at shelf level (1200mm) and tablet mounting height (~975mm)</li>
            <li><strong>Wall termination:</strong> Rear wall structure ends at shelf top (1200mm); front/side walls continue to 2380mm</li>
        </ul>

        <h3>Flooring & Fixings</h3>
        <ul>
            <li><strong>Floor type:</strong> Timber joists with thin board topping</li>
            <li><strong>Electrical cables:</strong> Running below flooring</li>
            <li><strong>Screw length:</strong> Maximum 25-30mm to avoid cable damage</li>
            <li><strong>Recommendation:</strong> Use grab adhesive + short screws; consider cable detection before drilling</li>
        </ul>

        <h3>Support & Structure</h3>
        <ul>
            <li><strong>Bed support legs:</strong> 6 total (20mm × 20mm each)</li>
            <li>Corner legs: 4 (one at each corner of bed frame)</li>
            <li>Middle legs: 2 (positioned at 940mm from left edge - midpoint of 1900mm mattress + offset)</li>
            <li><strong>Drawer runners:</strong> Floor-mounted with 20mm clearance</li>
            <li><strong>Top clearance:</strong> 10mm between drawer tops and bed frame underside</li>
        </ul>
    </div>

    <h2>💡 Lighting & Electronics</h2>
    <div class="drawing-section">
        <h3>ESP32 Touchscreen (7")</h3>
        <ul>
            <li><strong>Location:</strong> Rear shelf, centered horizontally</li>
            <li><strong>Vertical position:</strong> Midpoint between mattress top (750mm) and shelf top (1200mm) = 975mm</li>
            <li><strong>Function:</strong> Music control interface for corner speaker</li>
            <li><strong>Mounting:</strong> Flush-mounted requiring noggin support behind plasterboard</li>
        </ul>

        <h3>Lighting Systems (3 Types Recommended)</h3>
        
        <h4>1. Ambient Lighting (Warm & Cosy)</h4>
        <ul>
            <li><strong>Type:</strong> WS2812B addressable LED strips</li>
            <li><strong>Location:</strong> Under shelves (rear and head)</li>
            <li><strong>Color:</strong> Warm white (2700K-3000K) with optional RGB for mood</li>
            <li><strong>Controller:</strong> ESP32 with WLED firmware</li>
            <li><strong>Features:</strong> Dimmable, multiple preset patterns</li>
        </ul>

        <h4>2. Reading Light (Neutral White)</h4>
        <ul>
            <li><strong>Type:</strong> Dimmable LED strip or spotlight</li>
            <li><strong>Location:</strong> Head wall or underside of rear shelf, directed at pillow area</li>
            <li><strong>Color Temperature:</strong> 4000K (neutral white, reduces eye strain)</li>
            <li><strong>Control:</strong> PWM dimming via ESP32</li>
            <li><strong>CRI:</strong> >90 for comfortable reading</li>
        </ul>

        <h4>3. Starfield Projection System (4 Options)</h4>
        
        <p><strong>Option A - Fiber Optic Starfield (Most Realistic):</strong></p>
        <ul>
            <li>200-400 fiber optic strands poked through black fabric/board mounted on ceiling area</li>
            <li>RGB LED light source (single ESP32-controlled unit)</li>
            <li>Random strand lengths create depth and twinkling variation</li>
            <li>ESP32 controls color cycling and twinkle patterns</li>
        </ul>

        <p><strong>Option B - WS2812B Individual Star LEDs:</strong></p>
        <ul>
            <li>50-100 individual addressable LEDs mounted on ceiling/walls</li>
            <li>ESP32 + WLED for independent color and brightness control per "star"</li>
            <li>Programmable twinkling patterns (random intensity variations)</li>
            <li>Can create constellations, shooting stars, color shifts</li>
        </ul>

        <p><strong>Option C - Sky Projector Enhancement:</strong></p>
        <ul>
            <li>Small projector unit (laser or LED-based) mounted near head wall</li>
            <li>ESP32-controlled rotation motor for slow movement</li>
            <li>Projects realistic nebula/star patterns</li>
            <li>Combine with fiber optics for layered depth effect</li>
        </ul>

        <p><strong>Option D - Mixed Reality (Fiber + Addressable LEDs):</strong></p>
        <ul>
            <li>100 fiber optic strands for background "stars"</li>
            <li>20-30 WS2812B LEDs for "planets" and prominent stars</li>
            <li>ESP32 coordinates both systems via WLED</li>
            <li>Best balance of realism and programmability</li>
        </ul>

        <h4>ESP32 Wiring & Control</h4>
        <ul>
            <li>Central ESP32 board mounted inside head wall cavity</li>
            <li>Power: 5V 10A supply (60W) for all LED systems</li>
            <li>Touchscreen: I2C/SPI connection to ESP32</li>
            <li>All lighting: Controlled via WLED web interface + touchscreen UI</li>
            <li>Wire routing: Through stud wall cavities before plasterboard installation</li>
        </ul>
    </div>

    <h2>📦 Materials Cutting List</h2>
    <div class="drawing-section">
        <h3>Metal Stud Components (70mm system)</h3>
        <table>
            <tr>
                <th>Component</th>
                <th>Quantity</th>
                <th>Length</th>
                <th>Notes</th>
            </tr>
            <tr>
                <td>Floor tracks</td>
                <td>3</td>
                <td>2344mm, 1242mm, 2246mm</td>
                <td>Foot wall, side walls, rear wall</td>
            </tr>
            <tr>
                <td>Head tracks (ceiling)</td>
                <td>3</td>
                <td>2344mm, 1242mm, 2246mm</td>
                <td>Match floor tracks</td>
            </tr>
            <tr>
                <td>Vertical studs (2380mm)</td>
                <td>12</td>
                <td>2380mm</td>
                <td>At 400mm centers around perimeter</td>
            </tr>
            <tr>
                <td>Vertical studs (1200mm)</td>
                <td>8</td>
                <td>1200mm</td>
                <td>Rear wall (ends at shelf level)</td>
            </tr>
            <tr>
                <td>Horizontal noggins (shelving support)</td>
                <td>6</td>
                <td>2246mm, 326mm, 2246mm</td>
                <td>At 1200mm (shelf), 975mm (tablet), 530mm (platform)</td>
            </tr>
        </table>

        <h3>Plasterboard (12.5mm)</h3>
        <table>
            <tr>
                <th>Surface</th>
                <th>Area (approx)</th>
                <th>Sheets (2400×1200mm)</th>
            </tr>
            <tr>
                <td>Walls (all faces)</td>
                <td>~15 m²</td>
                <td>7 sheets</td>
            </tr>
            <tr>
                <td>Shelf top surfaces</td>
                <td>~1.5 m²</td>
                <td>1 sheet (shared)</td>
            </tr>
        </table>

        <h3>Timber Components</h3>
        <table>
            <tr>
                <th>Item</th>
                <th>Dimensions</th>
                <th>Quantity</th>
                <th>Purpose</th>
            </tr>
            <tr>
                <td>Bed frame planks</td>
                <td>45mm × 70mm × 1920mm</td>
                <td>3</td>
                <td>Bed frame perimeter (front, sides)</td>
            </tr>
            <tr>
                <td>Base planks (risers)</td>
                <td>45mm × 173mm × 1900mm</td>
                <td>3</td>
                <td>Raise bed 173mm off floor</td>
            </tr>
            <tr>
                <td>Support legs</td>
                <td>20mm × 20mm × 530mm</td>
                <td>6</td>
                <td>Drawer void support</td>
            </tr>
            <tr>
                <td>Shelf fascia (wooden)</td>
                <td>20mm × 200mm × 2246mm</td>
                <td>1</td>
                <td>Rear shelf front edge (decorative)</td>
            </tr>
            <tr>
                <td>Shelf fascia (head)</td>
                <td>20mm × 200mm × 326mm</td>
                <td>1</td>
                <td>Head wall shelf front edge</td>
            </tr>
        </table>

        <h3>Drawer Components (×2)</h3>
        <table>
            <tr>
                <th>Component</th>
                <th>Dimensions per drawer</th>
                <th>Material</th>
            </tr>
            <tr>
                <td>Front/Back panels</td>
                <td>880mm × 500mm × 18mm</td>
                <td>Plywood or MDF</td>
            </tr>
            <tr>
                <td>Side panels</td>
                <td>900mm × 500mm × 18mm</td>
                <td>Plywood or MDF</td>
            </tr>
            <tr>
                <td>Base panel</td>
                <td>880mm × 900mm × 12mm</td>
                <td>Plywood</td>
            </tr>
            <tr>
                <td>Floor runners/sliders</td>
                <td>900mm length</td>
                <td>Heavy-duty drawer slides (4 per drawer)</td>
            </tr>
        </table>

        <h3>Electrical & Electronics</h3>
        <table>
            <tr>
                <th>Item</th>
                <th>Specification</th>
                <th>Qty</th>
            </tr>
            <tr>
                <td>ESP32 Dev Board</td>
                <td>ESP32-DevKitC or similar</td>
                <td>1</td>
            </tr>
            <tr>
                <td>7" Touchscreen Display</td>
                <td>ESP32-compatible (SPI/I2C)</td>
                <td>1</td>
            </tr>
            <tr>
                <td>WS2812B LED Strip (Ambient)</td>
                <td>60 LEDs/m, RGBW, 5V</td>
                <td>5m</td>
            </tr>
            <tr>
                <td>LED Strip (Reading Light)</td>
                <td>4000K neutral white, dimmable</td>
                <td>1m</td>
            </tr>
            <tr>
                <td>Fiber Optic Kit (Option A/D)</td>
                <td>200-400 strands, RGB light engine</td>
                <td>1 kit</td>
            </tr>
            <tr>
                <td>WS2812B Individual LEDs (Option B/D)</td>
                <td>5V addressable, with heatsinks</td>
                <td>50-100</td>
            </tr>
            <tr>
                <td>Power Supply</td>
                <td>5V 10A (50-60W)</td>
                <td>1</td>
            </tr>
            <tr>
                <td>Speaker (corner mount)</td>
                <td>Bluetooth/WiFi, compact</td>
                <td>1</td>
            </tr>
        </table>
    </div>

    <div class="note">
        <strong>⚠️ Safety Reminder:</strong>
        <ul>
            <li>Always use a cable detector before drilling or screwing into the floor</li>
            <li>Maximum screw penetration: 25-30mm</li>
            <li>Use grab adhesive as primary fixing method where possible</li>
            <li>All electrical work should be performed by a qualified electrician</li>
            <li>Ensure adequate fire safety compliance for enclosed sleeping spaces</li>
        </ul>
    </div>

    <footer style="margin-top: 40px; padding: 20px; text-align: center; color: #666; border-top: 2px solid #ddd;">
        <p><strong>Cabin Bed Technical Drawings</strong> - Generated: {os.path.basename(__file__)}</p>
        <p>All corrections applied through iterative design refinement</p>
        <p style="font-size: 12px; margin-top: 10px;">
            Version: Final with all Message 14 corrections<br>
            • Access opening measured from full 2380mm height<br>
            • Internal width 2246mm + Foot wall 98mm = Total 2344mm external width<br>
            • Rear wall stops at shelf (1200mm), front walls continue to 2380mm<br>
            • 3D visualization restored with all corrections
        </p>
    </footer>
</body>
</html>
"""

# Write HTML file
output_path = '/workspace/cabin-bed-FINAL-ALL-CORRECTIONS-v2.html'
with open(output_path, 'w', encoding='utf-8') as f:
    f.write(html_content)

print(f"✓ HTML report saved: {output_path}")
print(f"  File size: {os.path.getsize(output_path) / (1024*1024):.2f} MB")
