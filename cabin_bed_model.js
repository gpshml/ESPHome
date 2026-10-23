/**
 * Cabin bed geometry — single source of truth (mm).
 * Used by the interactive Three.js viewer and static view capture.
 */

export const DIM = {
  FOOT: 98,
  INTERNAL_W: 2246,
  W: 2246 + 98, // 2344
  D: 1242,
  EXT: 237,
  H: 2380,
  REAR: 210,
  HEAD_W: 326,
  SHELF: 1200,
  PLINTH: 173,
  FRAME_H: 357,
  PLATFORM: 173 + 357, // 530
  MATT_W: 1900,
  MATT_D: 900,
  MATT_H: 220,
  TOL: 10,
  FRAME_D: 900 + 20, // 920
  ACCESS_W: 1300,
  ACCESS_X: 300 + 98, // 398
  ACCESS_BOT: 640,
  ACCESS_TOP: 2080,
  get FRONT_Y() { return this.D - this.FOOT; }, // 1144
  S: 0.001, // mm → scene units
};

/**
 * Build the cabin bed scene graph.
 * Coordinate map: model X→Three X, model depth Y→Three Z, model height Z→Three Y.
 */
export function buildCabinBed(THREE) {
  const {
    FOOT, INTERNAL_W, W, D, EXT, H, REAR, HEAD_W, SHELF,
    PLINTH, FRAME_H, PLATFORM, MATT_W, MATT_D, MATT_H, TOL, FRAME_D,
    ACCESS_W, ACCESS_X, ACCESS_BOT, ACCESS_TOP, S,
  } = DIM;
  const FRONT_Y = DIM.FRONT_Y;

  const root = new THREE.Group();
  const frontGroup = new THREE.Group();
  const footGroup = new THREE.Group();
  root.add(frontGroup);
  root.add(footGroup);

  function box(parent, x, y, z, w, d, h, color, opacity = 1, renderOrder = 0) {
    const geo = new THREE.BoxGeometry(w * S, h * S, d * S);
    const mat = new THREE.MeshStandardMaterial({
      color,
      transparent: opacity < 1,
      opacity,
      depthWrite: opacity >= 0.9,
      metalness: 0.05,
      roughness: 0.75,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set((x + w / 2) * S, (z + h / 2) * S, (y + d / 2) * S);
    mesh.renderOrder = renderOrder;
    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(geo),
      new THREE.LineBasicMaterial({ color: 0x222222, transparent: true, opacity: 0.4 })
    );
    mesh.add(edges);
    parent.add(mesh);
    return mesh;
  }

  // Foot wall + front extension (full height) — in footGroup so side section can hide it
  box(footGroup, 0, 0, 0, FOOT, D - EXT, H, 0xe67e22, 0.92);
  box(footGroup, 0, D - EXT, 0, FOOT, EXT, H, 0xf1c40f, 0.95);

  // Rear shelf / wall
  box(root, FOOT, 0, 0, INTERNAL_W, REAR, SHELF, 0xd4b896, 0.85);

  // Head wall to front plane
  box(root, W - HEAD_W, 0, 0, HEAD_W, FRONT_Y, SHELF, 0xbdc3c7, 0.8);

  // Above shelf (ghost volume)
  box(root, FOOT, 0, SHELF, INTERNAL_W, D, H - SHELF, 0xdde1e6, 0.14);

  // Bed (behind front wall)
  const mx = FOOT + TOL;
  const fy = REAR;
  const my = REAR + TOL;
  box(root, mx, fy, 0, MATT_W, FRAME_D, PLINTH, 0xc4a574, 0.95);
  box(root, mx, fy, PLINTH, MATT_W, FRAME_D, FRAME_H, 0x8B4513, 0.95);
  box(root, mx, my, PLATFORM, MATT_W, MATT_D, MATT_H, 0x2ecc71, 0.98);

  // Front fascia (after bed in tree; higher renderOrder)
  const leftW = ACCESS_X - FOOT;
  const rightX = ACCESS_X + ACCESS_W;
  const rightW = W - rightX;
  box(frontGroup, FOOT, FRONT_Y, 0, leftW, FOOT, ACCESS_TOP, 0x2980b9, 0.92, 2);
  box(frontGroup, rightX, FRONT_Y, 0, rightW, FOOT, ACCESS_TOP, 0x2980b9, 0.92, 2);
  box(frontGroup, FOOT, FRONT_Y, 0, INTERNAL_W, FOOT, ACCESS_BOT, 0x3498db, 0.88, 2);
  box(frontGroup, FOOT, FRONT_Y, ACCESS_TOP, INTERNAL_W, FOOT, H - ACCESS_TOP, 0x3498db, 0.88, 2);

  // Access opening outline
  const ox0 = ACCESS_X * S, ox1 = (ACCESS_X + ACCESS_W) * S;
  const oz0 = ACCESS_BOT * S, oz1 = ACCESS_TOP * S;
  const oy = (FRONT_Y + FOOT * 0.5) * S;
  const pts = [
    new THREE.Vector3(ox0, oz0, oy),
    new THREE.Vector3(ox1, oz0, oy),
    new THREE.Vector3(ox1, oz1, oy),
    new THREE.Vector3(ox0, oz1, oy),
    new THREE.Vector3(ox0, oz0, oy),
  ];
  const openLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(pts),
    new THREE.LineBasicMaterial({ color: 0xff2222 })
  );
  openLine.renderOrder = 3;
  root.add(openLine);

  return { root, frontGroup, footGroup, DIM };
}
