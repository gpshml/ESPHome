#!/usr/bin/env python3
"""Build cabin-bed-FINAL.html from captured PNGs + PARTS-LIST.md."""
import base64
import os
import re

ROOT = "/workspace"


def img_to_b64(path):
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")


def md_table_to_html(md_text):
    """Convert markdown tables and simple structure to HTML fragments."""
    lines = md_text.splitlines()
    out = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith("|") and i + 1 < len(lines) and re.match(r"^\|[\s\-:|]+\|$", lines[i + 1]):
            headers = [c.strip() for c in line.strip("|").split("|")]
            i += 2
            rows = []
            while i < len(lines) and lines[i].startswith("|"):
                rows.append([c.strip() for c in lines[i].strip("|").split("|")])
                i += 1
            out.append("<table>")
            out.append("<tr>" + "".join(f"<th>{h}</th>" for h in headers) + "</tr>")
            for row in rows:
                cells = "".join(f"<td>{c}</td>" for c in row)
                out.append(f"<tr>{cells}</tr>")
            out.append("</table>")
            continue
        if line.startswith("## "):
            out.append(f"<h3>{line[3:].strip()}</h3>")
        elif line.startswith("### "):
            out.append(f"<h4>{line[4:].strip()}</h4>")
        elif line.startswith("- [ ] "):
            out.append(f"<li>☐ {inline_md(line[6:])}</li>")
        elif line.startswith("- "):
            out.append(f"<li>{inline_md(line[2:])}</li>")
        elif line.startswith("---"):
            out.append("<hr>")
        elif line.strip() == "":
            out.append("")
        else:
            out.append(f"<p>{inline_md(line)}</p>")
        i += 1

    # Wrap consecutive <li> in <ul>
    html = "\n".join(out)
    html = re.sub(
        r"(?:<li>.*?</li>\n?)+",
        lambda m: "<ul>\n" + m.group(0) + "</ul>\n",
        html,
        flags=re.DOTALL,
    )
    return html


def inline_md(text):
    text = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", text)
    text = re.sub(r"`([^`]+)`", r"<code>\1</code>", text)
    return text


print("Converting images...")
viz_3d = img_to_b64(f"{ROOT}/3d_view_FINAL.png")
front = img_to_b64(f"{ROOT}/front_elevation_FINAL.png")
top = img_to_b64(f"{ROOT}/top_plan_FINAL.png")
side = img_to_b64(f"{ROOT}/side_elevation_FINAL.png")

with open(f"{ROOT}/PARTS-LIST.md", encoding="utf-8") as f:
    parts_html = md_table_to_html(f.read())

html = f"""<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Cabin Bed - Final Technical Drawings</title>
<style>
body{{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;max-width:1500px;margin:0 auto;padding:25px;background:#f5f5f5}}
h1{{color:#2c3e50;text-align:center;border-bottom:4px solid #3498db;padding-bottom:15px}}
h2{{color:#34495e;margin-top:35px;border-left:6px solid #3498db;padding-left:12px}}
h3{{color:#2c3e50;margin-top:28px}}
h4{{color:#34495e;margin-top:18px}}
.drawing{{background:white;padding:25px;margin:25px 0;border-radius:10px;box-shadow:0 3px 6px rgba(0,0,0,0.12)}}
.drawing img{{width:100%;height:auto;border:2px solid #ddd;border-radius:6px;margin:12px 0}}
.fixes{{background:#d4edda;border-left:5px solid #28a745;padding:20px;margin:25px 0;border-radius:6px}}
.fixes h3{{color:#155724;margin-top:0}}
.fixes ul{{margin:12px 0;padding-left:22px}}
.fixes li{{margin:8px 0;color:#155724;line-height:1.6}}
.highlight{{background:#fff3cd;padding:3px 6px;border-radius:4px;font-weight:bold}}
.parts{{background:white;padding:25px;margin:25px 0;border-radius:10px;box-shadow:0 3px 6px rgba(0,0,0,0.12)}}
.parts p{{line-height:1.55;color:#333}}
.parts ul{{padding-left:22px}}
.parts li{{margin:6px 0;line-height:1.5}}
.parts code{{background:#f0f3f6;padding:1px 5px;border-radius:3px;font-size:0.92em}}
table{{width:100%;border-collapse:collapse;margin:18px 0;background:white}}
th,td{{border:1px solid #ddd;padding:12px;text-align:left;vertical-align:top}}
th{{background:#3498db;color:white;font-weight:bold}}
tr:nth-child(even){{background:#f9f9f9}}
hr{{border:none;border-top:1px solid #ddd;margin:24px 0}}
</style>
</head><body>

<h1>Cabin Bed — Final Technical Drawings</h1>
<p style="text-align:center;font-size:17px;color:#666;margin-bottom:16px">
Static views captured from the interactive Three.js source model
</p>
<p style="text-align:center;margin-bottom:30px">
<a href="cabin-bed-3d-viewer.html"
   style="display:inline-block;background:#2980b9;color:white;padding:12px 22px;border-radius:8px;text-decoration:none;font-weight:600">
   Open interactive 3D (rotate / zoom)
</a>
<br><span style="font-size:13px;color:#666">Serve locally on port 8877 · left-drag rotate · scroll zoom · right-drag pan</span>
</p>

<div class="fixes">
<h3>Final corrections (volume model)</h3>
<ul>
<li><strong>Coordinate system:</strong> Origin at <span class="highlight">back-left-floor</span>. X foot→head, Y back→front, Z up.</li>
<li><strong>Non-overlapping volumes:</strong>
<ul>
<li>Rear shelf width = internal width − head wall (1920mm) — does not run under the head fascia</li>
<li>Head fascia 326 × 1144mm from the back wall to the front fascia (no void at the head/front corner)</li>
<li>Bed centred in the 934mm bay: <span class="highlight">7mm frame / 17mm mattress tuck</span> equal front &amp; rear for covers</li>
<li>Front opening as a picture-frame: full-width sill, separate jambs, full-width lintel</li>
</ul>
</li>
<li><strong>Foot wall:</strong> 237mm extension at the <strong>front</strong> into the room (yellow)</li>
<li><strong>Access opening:</strong> 1300 × 1440mm (640–2080mm AFF)</li>
<li><strong>Side elevation:</strong> front on left, rear shelf on right; rear structure stops at 1200mm</li>
</ul>
</div>

<div class="drawing">
<h2>1. 3D Visualization</h2>
<p>View from front-left. Origin at back corner. Foot wall left (orange/yellow), head right (grey), rear shelf beige.</p>
<img src="data:image/png;base64,{viz_3d}" alt="3D View">
</div>

<div class="drawing">
<h2>2. Front Elevation</h2>
<p>Front view — 2344mm × 2380mm overall. Access opening 640–2080mm (1440mm high).</p>
<img src="data:image/png;base64,{front}" alt="Front Elevation">
</div>

<div class="drawing">
<h2>3. Top Plan View</h2>
<p>Overhead. Back wall at top (Y=0), front at bottom. Foot left (orange = existing, yellow = 237mm front extension). Head meets front fascia; bed tuck equal front/rear.</p>
<img src="data:image/png;base64,{top}" alt="Top Plan">
</div>

<div class="drawing">
<h2>4. Side Elevation (from Foot End)</h2>
<p>From foot end looking toward head: rear/shelf on <strong>left</strong>, front wall on <strong>right</strong>. Rear structure stops at 1200mm.</p>
<img src="data:image/png;base64,{side}" alt="Side Elevation">
</div>

<h2>Final Dimensions</h2>
<table>
<tr><th>Component</th><th>Dimension</th><th>Notes</th></tr>
<tr><td><strong>Overall Width</strong></td><td><span class="highlight">2344mm</span></td><td>2246mm internal + 98mm foot wall</td></tr>
<tr><td><strong>Overall Depth</strong></td><td><span class="highlight">1242mm</span></td><td>Requires 237mm wall extension at front</td></tr>
<tr><td><strong>Overall Height</strong></td><td>2380mm</td><td>Full height to ceiling</td></tr>
<tr><td><strong>Access Opening</strong></td><td>1300mm × 1440mm</td><td>640–2080mm height</td></tr>
<tr><td><strong>Mattress / bed</strong></td><td>1900 × 900 × 220mm</td><td><span class="highlight">Already purchased</span> — 10mm clearance in bay</td></tr>
<tr><td><strong>Platform Height</strong></td><td>530mm</td><td>Lift/plinth under existing bed (buy risers only)</td></tr>
<tr><td><strong>Shelf Level</strong></td><td>1200mm</td><td>Rear wall stops here</td></tr>
<tr><td><strong>Rear Shelf</strong></td><td>1920 × 210mm</td><td>Stops before 326mm head wall</td></tr>
<tr><td><strong>Head Wall</strong></td><td>326 × 1144mm</td><td>From back wall to front fascia</td></tr>
<tr><td><strong>Bed tuck (F/R)</strong></td><td>7mm frame / 17mm mattress</td><td>Equal each end — covers tuck evenly</td></tr>
<tr><td><strong>Drawers (×2)</strong></td><td>880 × 900 × 500mm</td><td>Below platform</td></tr>
</table>

<div class="parts">
<h2>Parts &amp; Materials List</h2>
<p>Also available as <a href="PARTS-LIST.md"><code>PARTS-LIST.md</code></a> in the repo.</p>
{parts_html}
</div>

<h2>Key Construction Details</h2>
<div class="drawing">
<h3>Wall Extension Work</h3>
<ul>
<li><strong>Location:</strong> Front of foot wall (LEFT on top plan)</li>
<li><strong>Amount:</strong> 237mm extending into room</li>
<li><strong>Method:</strong> Strip plasterboard ~200mm back from current end, extend metal stud frame forward 237mm, re-board</li>
</ul>

<h3>Coordinate System</h3>
<ul>
<li><strong>Origin (0,0,0):</strong> Back-left-floor corner</li>
<li><strong>X:</strong> 0 → 2344mm (foot → head)</li>
<li><strong>Y:</strong> 0 → 1242mm (back → front into room)</li>
<li><strong>Z:</strong> 0 → 2380mm (floor → ceiling)</li>
</ul>

<h3>Electrical &amp; Safety</h3>
<ul>
<li>7" ESP32 touchscreen at ~975mm AFF (rear shelf centre)</li>
<li>Floor: timber joists with cables — max 25–30mm screw depth</li>
<li>Primary fixing: grab adhesive + short screws; cable-detect before drilling</li>
</ul>
</div>

<footer style="margin-top:50px;padding:25px;text-align:center;color:#666;border-top:3px solid #ddd">
<p><strong>Cabin Bed — Final Technical Drawings</strong></p>
<p>Source: <code>cabin-bed-3d-viewer.html</code> / <code>cabin_bed_model.js</code> · Non-overlapping volumes · Parts from <code>PARTS-LIST.md</code></p>
</footer>

</body></html>"""

out = f"{ROOT}/cabin-bed-FINAL.html"
with open(out, "w", encoding="utf-8") as f:
    f.write(html)

print(f"✓ Saved: {out}")
print(f"  Size: {os.path.getsize(out)/(1024*1024):.2f} MB")
