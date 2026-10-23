#!/usr/bin/env python3
import base64
import os

def img_to_b64(path):
    with open(path, 'rb') as f:
        return base64.b64encode(f.read()).decode('utf-8')

print("Converting images...")
viz_3d = img_to_b64('/workspace/3d_view_FINAL.png')
front = img_to_b64('/workspace/front_elevation_FINAL.png')
top = img_to_b64('/workspace/top_plan_FINAL.png')
side = img_to_b64('/workspace/side_elevation_FINAL.png')

html = f"""<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Cabin Bed - Final Technical Drawings</title>
<style>
body{{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;max-width:1500px;margin:0 auto;padding:25px;background:#f5f5f5}}
h1{{color:#2c3e50;text-align:center;border-bottom:4px solid #3498db;padding-bottom:15px}}
h2{{color:#34495e;margin-top:35px;border-left:6px solid #3498db;padding-left:12px}}
.drawing{{background:white;padding:25px;margin:25px 0;border-radius:10px;box-shadow:0 3px 6px rgba(0,0,0,0.12)}}
.drawing img{{width:100%;height:auto;border:2px solid #ddd;border-radius:6px;margin:12px 0}}
.fixes{{background:#d4edda;border-left:5px solid #28a745;padding:20px;margin:25px 0;border-radius:6px}}
.fixes h3{{color:#155724;margin-top:0}}
.fixes ul{{margin:12px 0;padding-left:22px}}
.fixes li{{margin:8px 0;color:#155724;line-height:1.6}}
.highlight{{background:#fff3cd;padding:3px 6px;border-radius:4px;font-weight:bold}}
table{{width:100%;border-collapse:collapse;margin:25px 0;background:white}}
th,td{{border:1px solid #ddd;padding:14px;text-align:left}}
th{{background:#3498db;color:white;font-weight:bold}}
tr:nth-child(even){{background:#f9f9f9}}
</style>
</head><body>

<h1>🛏️ Cabin Bed - Final Technical Drawings</h1>
<p style="text-align:center;font-size:17px;color:#666;margin-bottom:16px">
Coordinate system corrected - Origin at back corner
</p>
<p style="text-align:center;margin-bottom:30px">
<a href="cabin-bed-3d-viewer.html"
   style="display:inline-block;background:#2980b9;color:white;padding:12px 22px;border-radius:8px;text-decoration:none;font-weight:600">
   Open interactive 3D (rotate / zoom)
</a>
<br><span style="font-size:13px;color:#666">Serve with a local web server (see below) · left-drag rotate · scroll zoom · right-drag pan</span>
</p>

<div class="fixes">
<h3>✅ Final Corrections:</h3>
<ul>
<li><strong>Coordinate System:</strong> Origin (0,0,0) at <span class="highlight">BACK-LEFT-FLOOR corner</span> where walls meet. Everything extends OUT into room from there.</li>
<li><strong>3D View:</strong>
<ul>
<li>Tight against back wall (not floating with void behind)</li>
<li>237mm extension shown at <strong>FRONT</strong> of foot wall (yellow/orange, extending into room)</li>
<li>Front wall properly rendered with access opening visible</li>
<li>Base plinth and bed frame platform included</li>
<li>Transparent walls above shelf extend back over rear shelf area</li>
</ul>
</li>
<li><strong>Top Plan:</strong>
<ul>
<li>237mm extension shown at <strong>FRONT</strong> (Y=1005-1242mm, extending into room)</li>
<li>Back wall at Y=0, front at Y=1242</li>
<li>Clear orientation: back at top, front at bottom</li>
</ul>
</li>
<li><strong>Side Elevation:</strong>
<ul>
<li>Flipped: <strong>Front (void) on LEFT, Rear (shelf) on RIGHT</strong></li>
<li>No confusing labels - just shows structure</li>
<li>Rear wall stops at 1200mm shelf level</li>
</ul>
</li>
<li><strong>Front Elevation:</strong>
<ul>
<li>Square aspect ratio (14×14) for proper proportions</li>
<li>Should look almost square, not oblong/squashed</li>
<li>Access opening: 640-2080mm (1440mm height)</li>
</ul>
</li>
</ul>
</div>

<div class="drawing">
<h2>1. 3D Visualization</h2>
<p>View from front-left corner. Origin at back corner, tight against walls. 237mm extension at front (yellow).</p>
<img src="data:image/png;base64,{viz_3d}" alt="3D View">
</div>

<div class="drawing">
<h2>2. Front Elevation</h2>
<p>Front view - {2246}mm × {2380}mm. Access opening 640-2080mm (1440mm high).</p>
<img src="data:image/png;base64,{front}" alt="Front Elevation">
</div>

<div class="drawing">
<h2>3. Top Plan View</h2>
<p>Overhead view. 237mm extension at FRONT of foot wall (bottom). Back wall at top (Y=0).</p>
<img src="data:image/png;base64,{top}" alt="Top Plan">
</div>

<div class="drawing">
<h2>4. Side Elevation (from Foot End)</h2>
<p>Side view from foot end. Front/void on left, rear/shelf on right. Rear wall stops at 1200mm.</p>
<img src="data:image/png;base64,{side}" alt="Side Elevation">
</div>

<h2>📐 Final Dimensions</h2>
<table>
<tr><th>Component</th><th>Dimension</th><th>Notes</th></tr>
<tr><td><strong>Overall Width</strong></td><td><span class="highlight">2344mm</span></td><td>2246mm internal + 98mm foot wall</td></tr>
<tr><td><strong>Overall Depth</strong></td><td><span class="highlight">1242mm</span></td><td>Requires 237mm wall extension at front</td></tr>
<tr><td><strong>Overall Height</strong></td><td>2380mm</td><td>Full height to ceiling</td></tr>
<tr><td><strong>Access Opening</strong></td><td>1300mm × 1440mm</td><td>640-2080mm height</td></tr>
<tr><td><strong>Mattress</strong></td><td>1900 × 900 × 220mm</td><td>10mm tolerance around</td></tr>
<tr><td><strong>Platform Height</strong></td><td>530mm</td><td>173mm base + 357mm frame</td></tr>
<tr><td><strong>Shelf Level</strong></td><td>1200mm</td><td>Rear wall stops here</td></tr>
<tr><td><strong>Head Wall</strong></td><td>326mm wide</td><td>Solid fascia</td></tr>
<tr><td><strong>Rear Shelf</strong></td><td>210mm deep</td><td>At back wall (Y=0)</td></tr>
<tr><td><strong>Drawers (×2)</strong></td><td>880 × 900 × 500mm</td><td>Below platform</td></tr>
</table>

<h2>🔧 Key Construction Details</h2>
<div class="drawing">
<h3>Wall Extension Work</h3>
<ul>
<li><strong>Location:</strong> Front of foot wall (LEFT side when looking at top plan)</li>
<li><strong>Amount:</strong> 237mm extending into room</li>
<li><strong>Method:</strong> Remove plasterboard ~200mm back from current end, extend metal stud frame forward 237mm, re-plasterboard</li>
<li><strong>Result:</strong> Full 1242mm depth alcove</li>
</ul>

<h3>Coordinate System</h3>
<ul>
<li><strong>Origin (0,0,0):</strong> Back-left-floor corner where back wall meets foot wall at floor level</li>
<li><strong>X-axis:</strong> 0 to 2344mm (foot wall to head wall, left to right)</li>
<li><strong>Y-axis:</strong> 0 to 1242mm (back to front, extending INTO ROOM)</li>
<li><strong>Z-axis:</strong> 0 to 2380mm (floor to ceiling)</li>
</ul>

<h3>Structure</h3>
<ul>
<li>Metal stud frame: 70mm C-studs at 400mm centers</li>
<li>Plasterboard: 12.5mm on visible faces</li>
<li>Rear wall frame: Stops at 1200mm (shelf level)</li>
<li>Front/side frames: Continue to full 2380mm</li>
<li>6 support legs: 20×20mm at corners and midpoints</li>
</ul>

<h3>Electrical & Safety</h3>
<ul>
<li>7" ESP32 touchscreen at 975mm height (rear shelf center)</li>
<li>Floor: Timber joists with cables below - max 25-30mm screw depth</li>
<li>Primary fixing: Grab adhesive + short screws</li>
<li>Always use cable detector before drilling</li>
</ul>
</div>

<footer style="margin-top:50px;padding:25px;text-align:center;color:#666;border-top:3px solid #ddd">
<p><strong>Cabin Bed - Final Technical Drawings</strong></p>
<p>Coordinate system: Origin at back corner • 237mm extension at front • All views consistent</p>
</footer>

</body></html>"""

out = '/workspace/cabin-bed-FINAL.html'
with open(out, 'w', encoding='utf-8') as f:
    f.write(html)

print(f"✓ Saved: {out}")
print(f"  Size: {os.path.getsize(out)/(1024*1024):.2f} MB")
