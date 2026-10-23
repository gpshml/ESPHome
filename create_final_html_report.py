#!/usr/bin/env python3
"""
Generate final HTML report with all corrected cabin bed drawings embedded as base64
"""

import base64
import os

def image_to_base64(image_path):
    """Convert image to base64 string"""
    with open(image_path, 'rb') as img_file:
        return base64.b64encode(img_file.read()).decode('utf-8')

# Read all images and convert to base64
print("Converting images to base64...")
viz_3d_b64 = image_to_base64('/workspace/3d_visualization_CORRECTED.png')
front_elevation_b64 = image_to_base64('/workspace/front_elevation_CORRECTED.png')
top_plan_b64 = image_to_base64('/workspace/top_plan_CORRECTED.png')
side_elevation_b64 = image_to_base64('/workspace/side_elevation_CORRECTED.png')

print("Generating HTML report...")

html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Cabin Bed - Final Corrected Technical Drawings</title>
    <style>
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            max-width: 1500px;
            margin: 0 auto;
            padding: 25px;
            background-color: #f5f5f5;
        }}
        h1 {{
            color: #2c3e50;
            text-align: center;
            border-bottom: 4px solid #3498db;
            padding-bottom: 15px;
            margin-bottom: 10px;
        }}
        h2 {{
            color: #34495e;
            margin-top: 35px;
            border-left: 6px solid #3498db;
            padding-left: 12px;
        }}
        .drawing-section {{
            background: white;
            padding: 25px;
            margin: 25px 0;
            border-radius: 10px;
            box-shadow: 0 3px 6px rgba(0,0,0,0.12);
        }}
        .drawing-image {{
            width: 100%;
            height: auto;
            border: 2px solid #ddd;
            border-radius: 6px;
            margin: 12px 0;
        }}
        .corrections-applied {{
            background: #d4edda;
            border-left: 5px solid #28a745;
            padding: 20px;
            margin: 25px 0;
            border-radius: 6px;
        }}
        .corrections-applied h3 {{
            color: #155724;
            margin-top: 0;
        }}
        .corrections-applied ul {{
            margin: 12px 0;
            padding-left: 22px;
        }}
        .corrections-applied li {{
            margin: 8px 0;
            color: #155724;
            line-height: 1.6;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin: 25px 0;
            background: white;
        }}
        th, td {{
            border: 1px solid #ddd;
            padding: 14px;
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
            padding: 3px 6px;
            border-radius: 4px;
            font-weight: bold;
        }}
        .note {{
            background: #fff3cd;
            border-left: 5px solid #ffc107;
            padding: 18px;
            margin: 25px 0;
            border-radius: 6px;
        }}
        .subtitle {{
            text-align: center;
            font-size: 17px;
            color: #666;
            margin-bottom: 30px;
        }}
    </style>
</head>
<body>
    <h1>🛏️ Cabin Bed - Final Corrected Technical Drawings</h1>
    <p class="subtitle">
        All feedback incorporated - Consistent 3D model with sliced 2D views
    </p>

    <div class="corrections-applied">
        <h3>✅ Final Corrections Applied:</h3>
        <ul>
            <li><strong>Top Plan:</strong> 
                <ul>
                    <li>Removed incorrect 200mm "cut-in" block at foot of bed</li>
                    <li>Bed position corrected - no longer offset, back to normal position</li>
                    <li>Foot wall (98mm) clearly shown on LEFT side</li>
                    <li>Wall extension (237mm) indicated - needed to reach full 1242mm depth</li>
                    <li>Internal width: 2246mm, Total external width: 2344mm (2246 + 98)</li>
                </ul>
            </li>
            <li><strong>Front Elevation:</strong> 
                <ul>
                    <li>Access opening CORRECTED: Bottom at 640mm (halfway up mattress: 530 + 110)</li>
                    <li>Top at 2080mm (300mm below 2380mm ceiling)</li>
                    <li>Opening height: <span class="highlight">1440mm</span> (640-2080mm)</li>
                    <li>Shows only the mattress area in the opening, as requested</li>
                </ul>
            </li>
            <li><strong>Side Elevation (from Foot End):</strong> 
                <ul>
                    <li>Orientation corrected: REAR (shelf) on LEFT, FRONT (access) on RIGHT</li>
                    <li>Rear wall clearly stops at 1200mm (shelf level)</li>
                    <li>Front walls continue to full 2380mm height</li>
                    <li>Consistent with other views</li>
                </ul>
            </li>
            <li><strong>3D Visualization:</strong> 
                <ul>
                    <li>Rotated to view from <strong>front-left corner</strong></li>
                    <li>Access opening clearly visible on front face</li>
                    <li>Shows bed in proper alcove configuration</li>
                    <li>Rear shelf (210mm) properly rendered</li>
                    <li>Transparent walls above shelf show the enclosure properly</li>
                </ul>
            </li>
        </ul>
    </div>

    <div class="drawing-section">
        <h2>1. 3D Visualization (Front-Left Corner View)</h2>
        <p>Master 3D model showing complete structure. All 2D views are consistent "slices" from this model.</p>
        <img src="data:image/png;base64,{viz_3d_b64}" alt="3D Visualization" class="drawing-image">
    </div>

    <div class="drawing-section">
        <h2>2. Front Elevation</h2>
        <p>Front view showing access opening from 640mm (halfway up mattress) to 2080mm (300mm below ceiling).</p>
        <img src="data:image/png;base64,{front_elevation_b64}" alt="Front Elevation" class="drawing-image">
    </div>

    <div class="drawing-section">
        <h2>3. Top Plan View</h2>
        <p>Overhead view with foot wall (98mm) on left, wall extension (237mm) indicated, bed in normal position.</p>
        <img src="data:image/png;base64,{top_plan_b64}" alt="Top Plan View" class="drawing-image">
    </div>

    <div class="drawing-section">
        <h2>4. Side Elevation (From Foot End)</h2>
        <p>Side view from foot end: rear/shelf on left, front/access on right. Rear wall stops at 1200mm.</p>
        <img src="data:image/png;base64,{side_elevation_b64}" alt="Side Elevation" class="drawing-image">
    </div>

    <h2>📐 Final Dimensions Summary</h2>
    <table>
        <tr>
            <th>Component</th>
            <th>Dimension</th>
            <th>Notes</th>
        </tr>
        <tr>
            <td><strong>Overall Width (External)</strong></td>
            <td><span class="highlight">2344mm</span></td>
            <td>2246mm internal + 98mm foot wall</td>
        </tr>
        <tr>
            <td><strong>Internal Width</strong></td>
            <td>2246mm</td>
            <td>Mattress + clearances + head wall</td>
        </tr>
        <tr>
            <td><strong>Foot Wall Thickness</strong></td>
            <td>98mm</td>
            <td>70mm metal stud + plasterboard</td>
        </tr>
        <tr>
            <td><strong>Wall Extension Needed</strong></td>
            <td>237mm</td>
            <td>Extend existing foot wall toward front to reach 1242mm depth</td>
        </tr>
        <tr>
            <td><strong>Overall Depth</strong></td>
            <td>1242mm</td>
            <td>Target depth (requires wall extension)</td>
        </tr>
        <tr>
            <td><strong>Overall Height</strong></td>
            <td>2380mm</td>
            <td>Full height including walls above shelf</td>
        </tr>
        <tr>
            <td><strong>Access Opening</strong></td>
            <td>1300mm × 1440mm</td>
            <td>From 640mm to 2080mm height (halfway up mattress to 300mm below ceiling)</td>
        </tr>
        <tr>
            <td><strong>Shelf Top Height</strong></td>
            <td>1200mm</td>
            <td>Rear wall terminates here; front walls continue to 2380mm</td>
        </tr>
        <tr>
            <td><strong>Mattress Size</strong></td>
            <td>1900mm × 900mm × 220mm</td>
            <td>10mm tolerance all around (bed frame 1920 × 920mm)</td>
        </tr>
        <tr>
            <td><strong>Platform Height</strong></td>
            <td>530mm</td>
            <td>173mm base planks + 357mm frame</td>
        </tr>
        <tr>
            <td><strong>Head Wall Width</strong></td>
            <td>326mm</td>
            <td>Solid fascia at head end</td>
        </tr>
        <tr>
            <td><strong>Rear Shelf Depth</strong></td>
            <td>210mm</td>
            <td>Back wall, runs full internal width</td>
        </tr>
        <tr>
            <td><strong>Drawer Dimensions</strong></td>
            <td>880mm × 900mm × 500mm (H)</td>
            <td>Two drawers in void below platform</td>
        </tr>
    </table>

    <div class="note">
        <strong>📏 Key Understanding:</strong>
        <ul>
            <li>The existing foot wall on the LEFT side is currently not long enough</li>
            <li>It needs to be extended <strong>toward the FRONT</strong> by 237mm to reach the full 1242mm depth</li>
            <li>This extension is along the depth dimension (toward access opening), not width</li>
            <li>The bed is in its normal position (not offset)</li>
            <li>Total external width remains 2344mm (2246mm internal + 98mm foot wall)</li>
        </ul>
    </div>

    <h2>🔧 Construction Notes</h2>
    <div class="drawing-section">
        <h3>Wall Extension Work</h3>
        <ul>
            <li><strong>Extension required:</strong> 237mm toward front (increases depth from ~1005mm to 1242mm)</li>
            <li><strong>Method:</strong> Remove existing plasterboard ~200mm back from end, expose studs</li>
            <li><strong>Construction:</strong> Extend metal stud frame forward by 237mm</li>
            <li><strong>Finish:</strong> Apply new plasterboard over extended frame for clean join</li>
            <li><strong>Result:</strong> Full 1242mm depth alcove for cabin bed</li>
        </ul>

        <h3>Metal Stud Frame (70mm system)</h3>
        <ul>
            <li>Vertical studs at 400mm centers</li>
            <li>Horizontal noggins at 530mm (platform), 975mm (tablet), 1200mm (shelf)</li>
            <li>Rear wall frame stops at 1200mm; front/side frames continue to 2380mm</li>
            <li>12.5mm plasterboard on visible faces</li>
        </ul>

        <h3>Electrical Safety (Timber Floor)</h3>
        <ul>
            <li><strong>Floor:</strong> Timber joists with thin board, cables running below</li>
            <li><strong>Max screw penetration:</strong> 25-30mm only</li>
            <li><strong>Primary fixing:</strong> Grab adhesive recommended</li>
            <li><strong>Before drilling:</strong> Always use cable detector</li>
        </ul>

        <h3>Support Structure</h3>
        <ul>
            <li><strong>Bed support legs:</strong> 6 total (20mm × 20mm timber)</li>
            <li>4 corner legs + 2 middle legs at 940mm from left edge</li>
            <li><strong>Drawer runners:</strong> Floor-mounted, 20mm clearance below, 10mm above</li>
            <li><strong>Platform:</strong> 173mm base height + 357mm frame = 530mm total</li>
        </ul>
    </div>

    <h2>💡 Lighting & ESP32 Integration</h2>
    <div class="drawing-section">
        <h3>7" ESP32 Touchscreen</h3>
        <ul>
            <li><strong>Location:</strong> Rear shelf center, at 975mm height (between mattress top 750mm and shelf 1200mm)</li>
            <li><strong>Mounting:</strong> Flush-mounted, requires noggin support at ~975mm</li>
            <li><strong>Function:</strong> Music control for corner speaker</li>
        </ul>

        <h3>Three Lighting Systems</h3>
        
        <h4>1. Ambient Lighting (Warm & Cosy)</h4>
        <ul>
            <li>WS2812B addressable LED strips under rear shelf and head wall shelf</li>
            <li>Warm white 2700-3000K with optional RGB mood settings</li>
            <li>ESP32 + WLED firmware for patterns and dimming</li>
        </ul>

        <h4>2. Reading Light (Neutral White)</h4>
        <ul>
            <li>Dimmable LED strip at head wall or rear shelf underside</li>
            <li>4000K neutral white, CRI >90 for comfortable reading</li>
            <li>PWM dimming via ESP32</li>
        </ul>

        <h4>3. Starfield Projection (4 Options)</h4>
        <p><strong>Option A - Fiber Optic (Most Realistic):</strong></p>
        <ul>
            <li>200-400 fiber strands through black fabric/board on ceiling area</li>
            <li>RGB LED light engine controlled by ESP32</li>
            <li>Random strand lengths create depth and twinkling</li>
        </ul>

        <p><strong>Option B - WS2812B Individual Stars:</strong></p>
        <ul>
            <li>50-100 addressable LEDs on ceiling/walls</li>
            <li>Independent control per "star" via WLED</li>
            <li>Programmable patterns (twinkling, shooting stars, constellations)</li>
        </ul>

        <p><strong>Option C - Sky Projector:</strong></p>
        <ul>
            <li>LED/laser projector with ESP32-controlled rotation motor</li>
            <li>Projects realistic nebula/star patterns</li>
            <li>Can combine with fiber optics for layered effect</li>
        </ul>

        <p><strong>Option D - Hybrid (Fiber + LEDs):</strong></p>
        <ul>
            <li>100 fiber optic strands for background stars</li>
            <li>20-30 WS2812B LEDs for planets and prominent stars</li>
            <li>Best balance of realism and programmability</li>
        </ul>
    </div>

    <h2>📦 Materials Summary</h2>
    <div class="drawing-section">
        <h3>Metal Framing (70mm system)</h3>
        <ul>
            <li>Floor & head tracks for perimeter (2344mm × 1242mm footprint)</li>
            <li>Vertical studs: ~12 at 2380mm height, ~8 at 1200mm height (rear wall)</li>
            <li>Horizontal noggins at 530mm, 975mm, 1200mm levels</li>
            <li><strong>Wall extension:</strong> Additional 237mm of framing along foot wall depth</li>
        </ul>

        <h3>Plasterboard (12.5mm)</h3>
        <ul>
            <li>~7-8 sheets (2400×1200mm) for all wall surfaces</li>
            <li>Additional for wall extension area</li>
        </ul>

        <h3>Timber Components</h3>
        <ul>
            <li>Bed frame: 3× planks 45mm × 70mm × 1920mm</li>
            <li>Base risers: 3× planks 45mm × 173mm × 1900mm</li>
            <li>Support legs: 6× 20mm × 20mm × 530mm</li>
            <li>Shelf fascia: Rear 2246mm, Head 326mm (20mm × 200mm timber)</li>
        </ul>

        <h3>Drawers (×2)</h3>
        <ul>
            <li>Front/back: 880mm × 500mm × 18mm plywood</li>
            <li>Sides: 900mm × 500mm × 18mm plywood</li>
            <li>Base: 880mm × 900mm × 12mm plywood</li>
            <li>Floor-mounted runners: 900mm length, heavy-duty (4 per drawer)</li>
        </ul>

        <h3>Electronics</h3>
        <ul>
            <li>ESP32 dev board × 1</li>
            <li>7" touchscreen (ESP32-compatible) × 1</li>
            <li>WS2812B LED strip (5m for ambient)</li>
            <li>Neutral white LED strip (1m for reading)</li>
            <li>Starfield components (fiber optic kit or WS2812B LEDs)</li>
            <li>5V 10A power supply</li>
            <li>Bluetooth/WiFi speaker for corner</li>
        </ul>
    </div>

    <div class="note">
        <strong>⚠️ Safety Reminders:</strong>
        <ul>
            <li>Always use cable detector before drilling into floor</li>
            <li>Maximum screw depth: 25-30mm in floor (cables below)</li>
            <li>Primary fixing method: Grab adhesive + short screws</li>
            <li>Qualified electrician for all electrical work</li>
            <li>Fire safety compliance for enclosed sleeping spaces</li>
        </ul>
    </div>

    <footer style="margin-top: 50px; padding: 25px; text-align: center; color: #666; border-top: 3px solid #ddd;">
        <p><strong>Cabin Bed - Final Corrected Technical Drawings</strong></p>
        <p>All feedback incorporated • Consistent 3D model with sliced 2D views</p>
        <p style="font-size: 13px; margin-top: 15px;">
            <strong>Key Corrections:</strong><br>
            Access opening: 640-2080mm (1440mm height) • Wall extension: 237mm toward front<br>
            Bed position: Normal (not offset) • Side elevation: Rear left, front right<br>
            3D view: Front-left corner perspective
        </p>
    </footer>
</body>
</html>
"""

# Write HTML file
output_path = '/workspace/cabin-bed-FINAL-CORRECTED.html'
with open(output_path, 'w', encoding='utf-8') as f:
    f.write(html_content)

print(f"✓ HTML report saved: {output_path}")
print(f"  File size: {os.path.getsize(output_path) / (1024*1024):.2f} MB")
