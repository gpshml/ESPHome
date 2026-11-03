/**
 * Cabin bed — component assembly model (mm).
 * Parts: studs, track, ledgers, PB faces, shelf decks, lift, bed, drawers.
 * Not BIM/CAD profile-accurate; stud flanges slightly exaggerated for readability.
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
  PLINTH: 170,
  FRAME_H: 360,
  PLATFORM: 170 + 360, // 530
  MATT_W: 1900,
  MATT_D: 900,
  MATT_H: 220,
  TOL: 10,
  FRAME_D: 900 + 20, // 920
  ACCESS_W: 1300,
  ACCESS_X: 300 + 98, // 398
  ACCESS_BOT: 640,
  ACCESS_TOP: 2080,
  STUD_W: 70,
  STUD_FL: 45,
  TRACK_LEG: 25,
  PB: 12.5,
  LEDGER: 70,
  get FRONT_Y() { return this.D - this.FOOT; },
  get BED_BAY() { return this.FRONT_Y - this.REAR; },
  get FRAME_TUCK() { return (this.BED_BAY - this.FRAME_D) / 2; },
  S: 0.001,
};

export const COL = {
  existing: 0xe67e22,
  footExt: 0xf1c40f,
  stud: 0x95a5a6,
  track: 0x7f8c8d,
  pb: 0xecf0f1,
  ledger: 0xc4a574,
  deck: 0xd4b896,
  riser: 0xb8956a,
  bed: 0x8B4513,
  matt: 0x2ecc71,
  ghost: 0xbdc3c7,
  drawer: 0x5d6d7e,
  open: 0xff2222,
};

/**
 * Build component assembly.
 * Model X→Three X, depth Y→Three Z, height Z→Three Y.
 */
export function buildCabinBed(THREE) {
  const {
    FOOT, INTERNAL_W, W, D, EXT, H, REAR, HEAD_W, SHELF,
    PLINTH, FRAME_H, PLATFORM, MATT_W, MATT_D, MATT_H, TOL, FRAME_D,
    ACCESS_W, ACCESS_X, ACCESS_BOT, ACCESS_TOP,
    STUD_W, STUD_FL, TRACK_LEG, PB, LEDGER, S,
  } = DIM;
  const FRONT_Y = DIM.FRONT_Y;
  const shelfW = INTERNAL_W - HEAD_W;
  const frameTuck = DIM.FRAME_TUCK;

  const root = new THREE.Group();
  const frontGroup = new THREE.Group();
  const footGroup = new THREE.Group();
  const headGroup = new THREE.Group();
  const rearGroup = new THREE.Group();
  root.add(frontGroup, footGroup, headGroup, rearGroup);

  function box(parent, x, y, z, w, d, h, color, opacity = 1, renderOrder = 0, metalness = 0.05) {
    const geo = new THREE.BoxGeometry(w * S, h * S, d * S);
    const mat = new THREE.MeshStandardMaterial({
      color,
      transparent: opacity < 1,
      opacity,
      depthWrite: opacity >= 0.85,
      metalness,
      roughness: metalness > 0.3 ? 0.35 : 0.75,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set((x + w / 2) * S, (z + h / 2) * S, (y + d / 2) * S);
    mesh.renderOrder = renderOrder;
    mesh.add(new THREE.LineSegments(
      new THREE.EdgesGeometry(geo),
      new THREE.LineBasicMaterial({ color: 0x222222, transparent: true, opacity: 0.35 })
    ));
    parent.add(mesh);
    return mesh;
  }

  // —— Existing structure (ghost) ——
  box(footGroup, 0, 0, 0, FOOT, D - EXT, H, COL.existing, 0.35);
  box(root, FOOT, -20, 0, INTERNAL_W, 20, H, COL.ghost, 0.18); // back wall
  box(headGroup, W, 0, 0, 20, FRONT_Y, H, COL.ghost, 0.18); // head wall

  // —— Foot extension (+237mm): 2 studs, tracks, PB ——
  {
    const y0 = D - EXT;
    box(footGroup, (FOOT - STUD_W) / 2, y0 + 50, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    box(footGroup, (FOOT - STUD_W) / 2, y0 + EXT - 90, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    box(footGroup, (FOOT - 72) / 2, y0, 0, 72, EXT, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(footGroup, (FOOT - 72) / 2, y0, H - TRACK_LEG, 72, EXT, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(footGroup, -PB, y0, 0, PB, EXT, H, COL.pb, 0.95, 2);
    box(footGroup, FOOT, y0, 0, PB, EXT, H, COL.pb, 0.95, 2);
    box(footGroup, 0, y0, 0, FOOT, EXT, H, COL.footExt, 0.12);
  }

  // —— Rear shelf: ledger on back wall, deck, front fascia studs + PB ——
  {
    const g = rearGroup;
    box(g, FOOT, 0, SHELF - 80, shelfW, LEDGER, 45, COL.ledger, 1, 1);
    box(g, FOOT, 0, SHELF - 18, shelfW, REAR, 18, COL.deck, 1, 2);
    const fasciaY = REAR - STUD_FL;
    for (const x of [FOOT + 30, FOOT + shelfW / 2 - STUD_W / 2, FOOT + shelfW - STUD_W - 30]) {
      box(g, x, fasciaY, 0, STUD_W, STUD_FL, SHELF, COL.stud, 1, 1, 0.55);
    }
    box(g, FOOT, fasciaY, SHELF - TRACK_LEG, shelfW, STUD_FL, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, FOOT, REAR, 0, shelfW, PB, SHELF, COL.pb, 0.92, 2);
    box(g, FOOT + shelfW / 2 - 150, 0, 935, 300, 40, 80, COL.ledger, 1, 1); // tablet block ~975
  }

  // —— Head shelf/fascia: on existing head wall; PB on exposed faces ——
  {
    const g = headGroup;
    const hx = W - HEAD_W;
    box(g, W - LEDGER, 0, SHELF - 80, LEDGER, FRONT_Y, 45, COL.ledger, 1, 1);
    box(g, hx, 0, SHELF - 18, HEAD_W, FRONT_Y, 18, COL.deck, 1, 2);
    box(g, hx, REAR + 40, 0, STUD_FL, STUD_W, SHELF, COL.stud, 1, 1, 0.55);
    box(g, hx, FRONT_Y / 2 - STUD_W / 2, 0, STUD_FL, STUD_W, SHELF, COL.stud, 1, 1, 0.55);
    box(g, hx, FRONT_Y - STUD_W - 40, 0, STUD_FL, STUD_W, SHELF, COL.stud, 1, 1, 0.55);
    box(g, hx - PB, 0, 0, PB, FRONT_Y, SHELF, COL.pb, 0.92, 2); // toward bed
    box(g, hx, FRONT_Y, 0, HEAD_W, PB, SHELF, COL.pb, 0.92, 2); // toward room front
  }

  // —— Bed lift + bed + mattress + drawer ghosts ——
  {
    const mx = FOOT + TOL;
    const fy = REAR + frameTuck;
    const my = fy + TOL;
    const rw = 45;
    const gap = (MATT_W - 3 * rw) / 2;
    for (let i = 0; i < 3; i++) {
      box(root, mx + i * (rw + gap), fy, 0, rw, FRAME_D, PLINTH, COL.riser, 1, 1);
    }
    box(root, mx, fy + 80, 0, MATT_W, 45, PLINTH, COL.riser, 1, 1);
    box(root, mx, fy + FRAME_D - 125, 0, MATT_W, 45, PLINTH, COL.riser, 1, 1);
    box(root, mx, fy, PLINTH, MATT_W, FRAME_D, FRAME_H, COL.bed, 0.95, 1);
    box(root, mx, my, PLATFORM, MATT_W, MATT_D, MATT_H, COL.matt, 0.98, 2);
    const dw = (MATT_W - 50) / 2;
    box(root, mx + 15, fy + 50, 25, dw, FRAME_D - 100, PLINTH - 45, COL.drawer, 0.4, 1);
    box(root, mx + dw + 35, fy + 50, 25, dw, FRAME_D - 100, PLINTH - 45, COL.drawer, 0.4, 1);
  }

  // —— Front fascia: tracks, 4 jamb studs, sill/lintel, PB ——
  {
    const g = frontGroup;
    const leftW = ACCESS_X - FOOT;
    const rightX = ACCESS_X + ACCESS_W;
    const rightW = W - rightX;
    const y = FRONT_Y;
    const ty = y + (FOOT - 72) / 2;
    const sy = y + (FOOT - STUD_FL) / 2;

    box(g, FOOT, ty, 0, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, FOOT, ty, H - TRACK_LEG, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, FOOT, ty, ACCESS_BOT - TRACK_LEG, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, FOOT, ty, ACCESS_TOP, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);

    for (const x of [FOOT + 25, ACCESS_X - STUD_W - 25]) {
      box(g, x, sy, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    }
    for (const x of [rightX + 25, W - STUD_W - 25]) {
      box(g, x, sy, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    }

    const pbY = y + FOOT - PB;
    box(g, FOOT, pbY, 0, INTERNAL_W, PB, ACCESS_BOT, COL.pb, 0.92, 2);
    box(g, FOOT, pbY, ACCESS_BOT, leftW, PB, ACCESS_TOP - ACCESS_BOT, COL.pb, 0.92, 2);
    box(g, rightX, pbY, ACCESS_BOT, rightW, PB, ACCESS_TOP - ACCESS_BOT, COL.pb, 0.92, 2);
    box(g, FOOT, pbY, ACCESS_TOP, INTERNAL_W, PB, H - ACCESS_TOP, COL.pb, 0.92, 2);

    const ox0 = ACCESS_X * S, ox1 = (ACCESS_X + ACCESS_W) * S;
    const oz0 = ACCESS_BOT * S, oz1 = ACCESS_TOP * S;
    const oy = (y + FOOT * 0.5) * S;
    const openLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(ox0, oz0, oy),
        new THREE.Vector3(ox1, oz0, oy),
        new THREE.Vector3(ox1, oz1, oy),
        new THREE.Vector3(ox0, oz1, oy),
        new THREE.Vector3(ox0, oz0, oy),
      ]),
      new THREE.LineBasicMaterial({ color: COL.open })
    );
    openLine.renderOrder = 4;
    root.add(openLine);
  }

  return { root, frontGroup, footGroup, headGroup, rearGroup, DIM, COL };
}
