/**
 * Cabin bed — component assembly model (mm).
 * Parts: studs, track, noggins, ledgers, PB faces, shelf decks, lift, bed, drawers.
 *
 * Orientation (user-corrected Nov 2026): looking into the alcove from the room,
 * HEAD is LEFT, FOOT is RIGHT. Origin = back-left-floor (rear ∩ head wall ∩ floor).
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
  // 300mm from left of internal (= after head fascia)
  get ACCESS_X() { return this.HEAD_W + 300; }, // 626
  ACCESS_BOT: 640,
  ACCESS_TOP: 2080,
  STUD_W: 70,
  STUD_FL: 45,
  TRACK_LEG: 25,
  PB: 12.5,
  LEDGER: 70,
  /** Horizontal noggin depths (AFF) for full-height PB support */
  NOGGIN_ZS: [600, 1200, 1800],
  get FRONT_Y() { return this.D - this.FOOT; },
  get BED_BAY() { return this.FRONT_Y - this.REAR; },
  get FRAME_TUCK() { return (this.BED_BAY - this.FRAME_D) / 2; },
  get FOOT_X0() { return this.W - this.FOOT; }, // 2246 — foot wall starts
  get INTERNAL_X0() { return this.HEAD_W; }, // 326
  S: 0.001,
};

export const COL = {
  existing: 0x8c6b47,
  footExt: 0x6b783f, // autumnal olive (painted)
  stud: 0x9ea39a, // galvanised
  track: 0x8c9189,
  noggin: 0x7f8680,
  pb: 0x6b783f, // painted plasterboard — autumnal olive
  ledger: 0x9e7347, // timber
  deck: 0xb89461,
  riser: 0x9e7347,
  bed: 0x664222,
  matt: 0xc7bda6,
  ghost: 0xbdc3c7,
  drawer: 0x73614d,
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
    ACCESS_W, ACCESS_BOT, ACCESS_TOP,
    STUD_W, STUD_FL, TRACK_LEG, PB, LEDGER, S,
  } = DIM;
  const FRONT_Y = DIM.FRONT_Y;
  const ACCESS_X = DIM.ACCESS_X;
  const FOOT_X0 = DIM.FOOT_X0;
  const shelfW = INTERNAL_W - HEAD_W;
  const frameTuck = DIM.FRAME_TUCK;
  const nogginZs = DIM.NOGGIN_ZS;

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

  /** Horizontal noggin between two stud faces (clear span), flange depth along +Y. */
  function nogginsBetween(parent, xL, xR, y, zs, thick = STUD_FL) {
    const span = xR - xL;
    if (span < 20) return;
    for (const z of zs) {
      box(parent, xL, y, z - thick / 2, span, STUD_FL, thick, COL.noggin, 1, 1, 0.55);
    }
  }

  // —— Existing structure (ghost) ——
  // Head wall on LEFT; foot wall on RIGHT (non-extension depth)
  box(headGroup, -20, 0, 0, 20, FRONT_Y, H, COL.ghost, 0.18);
  box(root, HEAD_W, -20, 0, INTERNAL_W, 20, H, COL.ghost, 0.18); // back wall
  box(footGroup, FOOT_X0, 0, 0, FOOT, D - EXT, H, COL.existing, 0.35);

  // —— Foot extension (+237mm) on RIGHT: studs flush to outer face (x=W) ——
  {
    const y0 = D - EXT;
    const studX = W - STUD_W; // outer flange at x=W
    box(footGroup, studX, y0 + 50, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    box(footGroup, studX, y0 + EXT - STUD_FL - 50, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    box(footGroup, studX + (STUD_W - 72) / 2, y0, 0, 72, EXT, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(footGroup, studX + (STUD_W - 72) / 2, y0, H - TRACK_LEG, 72, EXT, TRACK_LEG, COL.track, 1, 1, 0.55);
    // Noggins between the two uprights (full-height PB both faces)
    const yA = y0 + 50 + STUD_FL;
    const yB = y0 + EXT - STUD_FL - 50;
    const yNog = Math.min(yA, yB);
    const nogDepth = Math.abs(yB - yA);
    for (const z of nogginZs) {
      box(footGroup, studX, yNog, z - 20, STUD_W, nogDepth, 40, COL.noggin, 1, 1, 0.55);
    }
    box(footGroup, W, y0, 0, PB, EXT, H, COL.pb, 0.95, 2); // outer, flush
    box(footGroup, studX - PB, y0, 0, PB, EXT, H, COL.pb, 0.95, 2); // inner, flush
    box(footGroup, studX, D - PB, 0, STUD_W, PB, H, COL.pb, 0.95, 2); // front end
    box(footGroup, FOOT_X0, y0, 0, FOOT, EXT, H, COL.footExt, 0.08);
  }

  // —— Rear shelf: ledger on back wall; width stops before foot ——
  {
    const g = rearGroup;
    const sx = HEAD_W;
    box(g, sx, 0, SHELF - 80, shelfW, LEDGER, 45, COL.ledger, 1, 1);
    box(g, sx, 0, SHELF - 18, shelfW, REAR, 18, COL.deck, 1, 2);
    const fasciaY = REAR - STUD_FL;
    const uprightXs = [sx + 30, sx + shelfW / 2 - STUD_W / 2, sx + shelfW - STUD_W - 30];
    for (const x of uprightXs) {
      box(g, x, fasciaY, 0, STUD_W, STUD_FL, SHELF, COL.stud, 1, 1, 0.55);
    }
    // Light noggins between fascia uprights (PB front support)
    for (let i = 0; i < uprightXs.length - 1; i++) {
      nogginsBetween(g, uprightXs[i] + STUD_W, uprightXs[i + 1], fasciaY, [400, 800], 40);
    }
    box(g, sx, fasciaY, SHELF - TRACK_LEG, shelfW, STUD_FL, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, sx, REAR, 0, shelfW, PB, SHELF, COL.pb, 0.92, 2);
    box(g, sx + shelfW / 2 - 150, 0, 935, 300, 40, 80, COL.ledger, 1, 1);
  }

  // —— Head shelf/fascia on LEFT (0..HEAD_W); studs inside, behind bed-face PB ——
  {
    const g = headGroup;
    box(g, 0, 0, SHELF - 80, LEDGER, FRONT_Y, 45, COL.ledger, 1, 1);
    box(g, 0, 0, SHELF - 18, HEAD_W, FRONT_Y, 18, COL.deck, 1, 2);
    // Bed face at x=HEAD_W; flanges into shelf (−X)
    const studX = HEAD_W - STUD_FL;
    const ys = [REAR + 40, FRONT_Y / 2 - STUD_W / 2, FRONT_Y - STUD_W - 40];
    for (const y of ys) {
      box(g, studX, y, 0, STUD_FL, STUD_W, SHELF, COL.stud, 1, 1, 0.55);
    }
    for (let i = 0; i < ys.length - 1; i++) {
      const y0 = ys[i] + STUD_W;
      const span = ys[i + 1] - y0;
      if (span > 20) {
        for (const z of [400, 800]) {
          box(g, studX, y0, z - 20, STUD_FL, span, 40, COL.noggin, 1, 1, 0.55);
        }
      }
    }
    box(g, HEAD_W, 0, 0, PB, FRONT_Y, SHELF, COL.pb, 0.92, 2); // toward bed
    box(g, 0, FRONT_Y, 0, HEAD_W, PB, SHELF, COL.pb, 0.92, 2); // toward room front
  }

  // —— Bed lift + bed + mattress + drawers (pillow end toward HEAD / left) ——
  {
    const mx = HEAD_W + TOL;
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

  // —— Front fascia: tracks, 4 jambs, sill/lintel, noggins, PB both faces ——
  {
    const g = frontGroup;
    const leftW = ACCESS_X - HEAD_W; // pier beside head (~300)
    const rightX = ACCESS_X + ACCESS_W;
    const rightW = FOOT_X0 - rightX; // pier beside foot (~646)
    const y = FRONT_Y;
    const ty = y + (FOOT - 72) / 2;
    const sy = y + (FOOT - STUD_FL) / 2;

    box(g, HEAD_W, ty, 0, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, HEAD_W, ty, H - TRACK_LEG, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, HEAD_W, ty, ACCESS_BOT - TRACK_LEG, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, HEAD_W, ty, ACCESS_TOP, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);

    const leftStuds = [HEAD_W + 25, ACCESS_X - STUD_W - 25];
    const rightStuds = [rightX + 25, FOOT_X0 - STUD_W - 25];
    for (const x of leftStuds) {
      box(g, x, sy, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    }
    for (const x of rightStuds) {
      box(g, x, sy, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    }

    // Full-height noggins between pier studs (PB support)
    nogginsBetween(g, leftStuds[0] + STUD_W, leftStuds[1], sy, nogginZs);
    nogginsBetween(g, rightStuds[0] + STUD_W, rightStuds[1], sy, nogginZs);
    // Mid noggins in boarded bands above lintel / below sill
    nogginsBetween(g, leftStuds[0] + STUD_W, leftStuds[1], sy, [320, 2230], 40);
    nogginsBetween(g, rightStuds[0] + STUD_W, rightStuds[1], sy, [320, 2230], 40);

    for (const pbY of [y + FOOT - PB, y]) {
      box(g, HEAD_W, pbY, 0, INTERNAL_W, PB, ACCESS_BOT, COL.pb, 0.92, 2);
      box(g, HEAD_W, pbY, ACCESS_BOT, leftW, PB, ACCESS_TOP - ACCESS_BOT, COL.pb, 0.92, 2);
      box(g, rightX, pbY, ACCESS_BOT, rightW, PB, ACCESS_TOP - ACCESS_BOT, COL.pb, 0.92, 2);
      box(g, HEAD_W, pbY, ACCESS_TOP, INTERNAL_W, PB, H - ACCESS_TOP, COL.pb, 0.92, 2);
    }

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
