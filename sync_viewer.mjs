#!/usr/bin/env node
/**
 * Inject cabin_bed_model.js into cabin-bed-3d-viewer.html (+ index.html).
 */
import { readFileSync, writeFileSync } from "fs";

const MODEL = "/workspace/cabin_bed_model.js";
const OUTS = ["/workspace/cabin-bed-3d-viewer.html", "/workspace/index.html"];

const modelSrc = readFileSync(MODEL, "utf8")
  .replace(/^export /gm, "")
  .trim();

const html = `<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Cabin Bed — Component 3D</title>
  <style>
    html, body { margin: 0; height: 100%; background: #f4f5f7; color: #1a1d23; font-family: system-ui, sans-serif; overflow: hidden; }
    #ui {
      position: absolute; z-index: 10; top: 12px; left: 12px; right: 12px;
      display: flex; flex-wrap: wrap; gap: 8px; align-items: center;
      pointer-events: none;
    }
    #ui > * { pointer-events: auto; }
    h1 { font-size: 15px; font-weight: 600; margin: 0; background: rgba(255,255,255,.9); padding: 8px 12px; border-radius: 8px; box-shadow: 0 1px 3px rgba(0,0,0,.12); }
    .hint { font-size: 12px; background: rgba(255,255,255,.85); padding: 6px 10px; border-radius: 8px; color: #5a6270; }
    button {
      background: #2980b9; color: #fff; border: 0; border-radius: 8px;
      padding: 8px 12px; font-size: 13px; cursor: pointer; font-weight: 600;
    }
    button.secondary { background: #5a6270; }
    button.active { outline: 2px solid #1a1d23; outline-offset: 1px; }
    #legend {
      position: absolute; z-index: 10; bottom: 14px; right: 14px;
      background: rgba(255,255,255,.94); padding: 10px 12px; border-radius: 8px;
      font-size: 11px; line-height: 1.55; box-shadow: 0 1px 3px rgba(0,0,0,.12); max-width: 220px;
    }
    #legend b { display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:6px; vertical-align:middle; }
    #canvas-wrap { width: 100%; height: 100%; }
    canvas { display: block; }
    #view-label {
      position: absolute; bottom: 14px; left: 14px; z-index: 10;
      background: rgba(255,255,255,.92); padding: 8px 12px; border-radius: 8px;
      font-size: 13px; font-weight: 600; box-shadow: 0 1px 3px rgba(0,0,0,.12);
    }
    #err {
      display: none; position: absolute; inset: 20% 10%; z-index: 20;
      background: #fff3cd; border: 1px solid #ffc107; border-radius: 10px;
      padding: 20px; font-size: 14px; line-height: 1.5;
    }
  </style>
  <script type="importmap">
  {
    "imports": {
      "three": "https://unpkg.com/three@0.170.0/build/three.module.js",
      "three/addons/": "https://unpkg.com/three@0.170.0/examples/jsm/"
    }
  }
  </script>
</head>
<body>
  <div id="ui">
    <h1>Cabin bed — component assembly</h1>
    <span class="hint">Studs · track · ledgers · PB · lift · bed · Left-drag rotate</span>
    <button type="button" class="secondary" data-view="3d">3D</button>
    <button type="button" class="secondary" data-view="front">Front</button>
    <button type="button" class="secondary" data-view="top">Top</button>
    <button type="button" class="secondary" data-view="side">Side (foot)</button>
    <button type="button" id="reset">Reset</button>
    <button type="button" class="secondary" id="toggleFront">Toggle front</button>
  </div>
  <div id="view-label">View: 3D</div>
  <div id="legend">
    <div><b style="background:#95a5a6"></b>C-stud / track</div>
    <div><b style="background:#ecf0f1;border:1px solid #bbb"></b>Plasterboard face</div>
    <div><b style="background:#c4a574"></b>Wall ledger</div>
    <div><b style="background:#d4b896"></b>Shelf deck</div>
    <div><b style="background:#b8956a"></b>170mm lift</div>
    <div><b style="background:#8B4513"></b>Existing bed</div>
    <div><b style="background:#2ecc71"></b>Mattress</div>
    <div><b style="background:#e67e22"></b>Existing foot (ghost)</div>
  </div>
  <div id="err"></div>
  <div id="canvas-wrap"></div>

  <script type="module">
    const errEl = document.getElementById("err");
    function showError(msg) {
      errEl.style.display = "block";
      errEl.innerHTML = "<strong>Could not load 3D viewer</strong><br>" + msg +
        "<br><br>Needs network for Three.js, or: <code>python3 -m http.server 8877</code>";
      console.error(msg);
    }

    let THREE, OrbitControls;
    try {
      THREE = await import("three");
      ({ OrbitControls } = await import("three/addons/controls/OrbitControls.js"));
    } catch (e) {
      showError(String(e && e.message ? e.message : e));
      throw e;
    }

${modelSrc}


    function buildCabinBedLocal() {
      return buildCabinBed(THREE);
    }

    const S = DIM.S, W = DIM.W, D = DIM.D, H = DIM.H;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf4f5f7);

    const persp = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.05, 50);
    const ortho = new THREE.OrthographicCamera(-2, 2, 2, -2, 0.05, 50);
    let camera = persp;

    const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.setSize(innerWidth, innerHeight);
    document.getElementById("canvas-wrap").appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.maxDistance = 12;
    controls.minDistance = 0.5;

    scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const dir = new THREE.DirectionalLight(0xffffff, 0.8);
    dir.position.set(-2, 4, 3);
    scene.add(dir);
    const fill = new THREE.DirectionalLight(0xffffff, 0.35);
    fill.position.set(2, 2, -1);
    scene.add(fill);

    const { root, frontGroup, footGroup, headGroup } = buildCabinBedLocal();
    scene.add(root);

    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(4, 4),
      new THREE.MeshBasicMaterial({ color: 0xe8eaed, transparent: true, opacity: 0.5 })
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.set(W * S * 0.5, -0.001, D * S * 0.5);
    scene.add(ground);

    function useOrtho(spanX, spanY, mirrorX = false) {
      camera = ortho;
      controls.object = ortho;
      controls.enabled = false;
      const aspect = innerWidth / innerHeight;
      let halfW = spanX / 2, halfH = spanY / 2;
      if (halfW / halfH > aspect) halfH = halfW / aspect;
      else halfW = halfH * aspect;
      const pad = 1.08;
      ortho.left = mirrorX ? halfW * pad : -halfW * pad;
      ortho.right = mirrorX ? -halfW * pad : halfW * pad;
      ortho.top = halfH * pad;
      ortho.bottom = -halfH * pad;
      ortho.near = 0.1;
      ortho.far = 20;
      ortho.updateProjectionMatrix();
    }

    const views = {
      "3d": () => {
        camera = persp;
        controls.object = persp;
        controls.enabled = true;
        controls.enableRotate = true;
        controls.enableDamping = true;
        footGroup.visible = true;
        headGroup.visible = true;
        frontGroup.visible = true;
        persp.position.set(-2.4, 2.8, 2.8);
        controls.target.set(W * S * 0.55, H * S * 0.35, D * S * 0.4);
        persp.up.set(0, 1, 0);
        controls.update();
      },
      front: () => {
        footGroup.visible = true;
        headGroup.visible = true;
        frontGroup.visible = true;
        useOrtho(W * S, H * S, false);
        ortho.position.set(W * S * 0.5, H * S * 0.5, D * S + 4);
        ortho.up.set(0, 1, 0);
        ortho.lookAt(W * S * 0.5, H * S * 0.5, 0);
        ortho.updateMatrixWorld(true);
      },
      top: () => {
        footGroup.visible = true;
        headGroup.visible = true;
        frontGroup.visible = true;
        useOrtho(W * S, D * S, false);
        ortho.position.set(W * S * 0.5, H * S + 4, D * S * 0.5);
        ortho.up.set(0, 0, -1);
        ortho.lookAt(W * S * 0.5, 0, D * S * 0.5);
        ortho.updateMatrixWorld(true);
      },
      side: () => {
        footGroup.visible = false;
        headGroup.visible = false;
        frontGroup.visible = true;
        useOrtho(D * S, H * S, false);
        ortho.position.set((DIM.FOOT + DIM.MATT_W * 0.5) * S, H * S * 0.5, D * S * 0.5);
        ortho.up.set(0, 1, 0);
        ortho.lookAt(W * S * 0.5, H * S * 0.5, D * S * 0.5);
        ortho.updateMatrixWorld(true);
      },
    };

    let currentView = "3d";
    const label = document.getElementById("view-label");
    const viewNames = {
      "3d": "3D assembly — components",
      front: "Front elevation",
      top: "Top plan",
      side: "Side section (from foot)",
    };

    function setView(name) {
      if (!views[name]) return false;
      footGroup.visible = true;
      headGroup.visible = true;
      frontGroup.visible = true;
      currentView = name;
      views[name]();
      label.textContent = "View: " + viewNames[name];
      document.querySelectorAll("[data-view]").forEach((b) => {
        b.classList.toggle("active", b.dataset.view === name);
      });
      renderer.render(scene, camera);
      return true;
    }

    document.querySelectorAll("[data-view]").forEach((btn) => {
      btn.addEventListener("click", () => setView(btn.dataset.view));
    });
    document.getElementById("reset").onclick = () => setView(currentView);
    document.getElementById("toggleFront").onclick = () => {
      frontGroup.visible = !frontGroup.visible;
      renderer.render(scene, camera);
    };
    addEventListener("resize", () => {
      persp.aspect = innerWidth / innerHeight;
      persp.updateProjectionMatrix();
      renderer.setSize(innerWidth, innerHeight);
      setView(currentView);
    });

    window.__cabinBed = {
      ready: false, DIM, setView,
      getView: () => currentView,
      render: () => renderer.render(scene, camera),
      canvas: () => renderer.domElement,
    };

    setView("3d");
    window.__cabinBed.ready = true;
    document.title = "Cabin Bed 3D — READY";

    (function animate() {
      requestAnimationFrame(animate);
      if (currentView === "3d" && controls.enabled) controls.update();
      renderer.render(scene, camera);
    })();
  </script>
</body>
</html>
`;

for (const out of OUTS) {
  writeFileSync(out, html);
  console.log("wrote", out, html.length, "bytes");
}
