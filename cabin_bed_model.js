/**
 * Cabin bed — component assembly model (mm).
 *
 * Orientation (user-confirmed — BUILD-SEQUENCE.md):
 * Looking into the alcove from the room — HEAD LEFT, FOOT RIGHT.
 * Narrow access pier (~300mm) at the FOOT (right); wider pier at the HEAD.
 * Origin = back-left-floor (rear ∩ head wall ∩ floor).
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
  FOOT_PIER: 300,
  // Opening set out from foot: narrow pier at right
  get ACCESS_X() { return this.INTERNAL_W - this.ACCESS_W - this.FOOT_PIER; }, // 646
  ACCESS_BOT: 640,
  ACCESS_TOP: 2080,
  STUD_W: 70,
  STUD_FL: 45,
  TRACK_LEG: 25,
  PB: 12.5,
  LEDGER: 70,
  DECK_T: 18,
  LEDGER_H: 45,
  NOGGIN_ZS: [600, 1200, 1800],
  get FRONT_Y() { return this.D - this.FOOT; }, // 1144
  get FRONT_ROOM_Y() { return this.D; },
  get FOOT_X0() { return this.W - this.FOOT; }, // 2246
  get BED_BAY() { return this.FRONT_Y - this.REAR; },
  get FRAME_TUCK() { return (this.BED_BAY - this.FRAME_D) / 2; },
  get ledger_z() { return this.SHELF - this.DECK_T - this.LEDGER_H; },
  get fascia_h() { return this.SHELF - this.DECK_T; },
  S: 0.001,
};

export const COL = {
  existing: 0x8c6b47,
  footExt: 0x6b783f,
  stud: 0x9ea39a,
  track: 0x8c9189,
  noggin: 0x7f8680,
  pb: 0x5c663d, // autumnal olive
  ledger: 0x9e7347,
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
  const ledgerZ = DIM.ledger_z;
  const fasciaH = DIM.fascia_h;
  const deckT = DIM.DECK_T;

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

  function nogginsBetween(parent, xL, xR, y, zs, thick = STUD_FL) {
    const span = xR - xL;
    if (span < 20) return;
    for (const z of zs) {
      box(parent, xL, y, z - thick / 2, span, STUD_FL, thick, COL.noggin, 1, 1, 0.55);
    }
  }

  // —— Existing structure ——
  // Head wall LEFT ghost, foot wall RIGHT, back wall
  box(headGroup, -20, 0, 0, 20, FRONT_Y, H, COL.ghost, 0.18);
  box(footGroup, FOOT_X0, 0, 0, FOOT, D - EXT, H, COL.existing, 0.35);
  box(root, 0, -20, 0, INTERNAL_W, 20, H, COL.ghost, 0.18);

  // —— Foot extension +237mm at FRONT — RIGHT (FOOT_X0..W), flush rules ——
  {
    const y0 = D - EXT;
    const studX = W - PB - STUD_W;
    box(footGroup, studX, y0 + 40, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    box(footGroup, studX, D - STUD_FL - 40, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    box(footGroup, FOOT_X0 + (FOOT - 72) / 2, y0, 0, 72, EXT, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(footGroup, FOOT_X0 + (FOOT - 72) / 2, y0, H - TRACK_LEG, 72, EXT, TRACK_LEG, COL.track, 1, 1, 0.55);
    const yA = y0 + 40 + STUD_FL;
    const yB = D - STUD_FL - 40;
    for (const z of nogginZs) {
      box(footGroup, studX, yA, z - 20, STUD_W, yB - yA, 40, COL.noggin, 1, 1, 0.55);
    }
    box(footGroup, W - PB, y0, 0, PB, EXT, H, COL.pb, 0.98, 2); // outer
    box(footGroup, FOOT_X0, y0, 0, PB, EXT, H, COL.pb, 0.98, 2); // inner = front wall end
    box(footGroup, FOOT_X0, D - PB, 0, FOOT, PB, H, COL.pb, 0.98, 2); // front end
    box(footGroup, FOOT_X0, y0, 0, FOOT, EXT, H, COL.footExt, 0.06);
  }

  // —— Rear shelf (after head) ——
  {
    const g = rearGroup;
    const sx = HEAD_W;
    box(g, sx, 0, ledgerZ, shelfW, LEDGER, DIM.LEDGER_H, COL.ledger, 1, 1);
    box(g, sx, 0, SHELF - deckT, shelfW, REAR, deckT, COL.deck, 1, 2);
    const fasciaY = REAR - STUD_FL;
    const uprightXs = [sx + 30, sx + shelfW / 2 - STUD_W / 2, sx + shelfW - STUD_W - 30];
    for (const x of uprightXs) {
      box(g, x, fasciaY, 0, STUD_W, STUD_FL, fasciaH, COL.stud, 1, 1, 0.55);
    }
    for (let i = 0; i < uprightXs.length - 1; i++) {
      nogginsBetween(g, uprightXs[i] + STUD_W, uprightXs[i + 1], fasciaY, [400, 800], 40);
    }
    box(g, sx, fasciaY, fasciaH - TRACK_LEG, shelfW, STUD_FL, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, sx, REAR, 0, shelfW, PB, fasciaH, COL.pb, 0.98, 2);
    box(g, sx + shelfW / 2 - 150, 0, 935, 300, 40, 80, COL.ledger, 1, 1);
  }

  // —— Head shelf/fascia LEFT ——
  {
    const g = headGroup;
    box(g, 0, 0, ledgerZ, LEDGER, FRONT_Y, DIM.LEDGER_H, COL.ledger, 1, 1);
    box(g, 0, 0, SHELF - deckT, HEAD_W, FRONT_Y, deckT, COL.deck, 1, 2);
    const ys = [REAR + 40, FRONT_Y / 2 - STUD_W / 2, FRONT_Y - STUD_W - 40];
    for (const y of ys) {
      // Stud inside shelf: flanges toward −X from bed face at HEAD_W
      box(g, HEAD_W - STUD_FL, y, 0, STUD_FL, STUD_W, fasciaH, COL.stud, 1, 1, 0.55);
    }
    for (let i = 0; i < ys.length - 1; i++) {
      const y0 = ys[i] + STUD_W;
      const span = ys[i + 1] - y0;
      if (span > 20) {
        for (const z of [400, 800]) {
          box(g, HEAD_W - STUD_FL, y0, z - 20, STUD_FL, span, 40, COL.noggin, 1, 1, 0.55);
        }
      }
    }
    // Bed face — exposed bay only (not hidden back to rear wall)
    box(g, HEAD_W, REAR, 0, PB, FRONT_Y - REAR, fasciaH, COL.pb, 0.98, 2);
    box(g, 0, FRONT_Y, 0, HEAD_W, PB, fasciaH, COL.pb, 0.98, 2); // room front
  }

  // —— Bed (pillow toward HEAD / left) ——
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

  // —— Front fascia: X=0..INTERNAL_W; narrow pier at FOOT (right) ——
  {
    const g = frontGroup;
    const leftW = ACCESS_X; // ~646 at head
    const rightX = ACCESS_X + ACCESS_W;
    const rightW = INTERNAL_W - rightX; // 300 at foot
    const y = FRONT_Y;
    const ty = y + (FOOT - 72) / 2;
    const sy = y + (FOOT - STUD_FL) / 2;
    const roomY = y + FOOT - PB;
    const alcoveY = y;

    box(g, 0, ty, 0, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, 0, ty, H - TRACK_LEG, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, 0, ty, ACCESS_BOT - TRACK_LEG, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, 0, ty, ACCESS_TOP, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);

    const leftStuds = [25, ACCESS_X - STUD_W - 25];
    const rightStuds = [rightX + 25, INTERNAL_W - STUD_W - 25];
    for (const x of leftStuds) box(g, x, sy, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    for (const x of rightStuds) box(g, x, sy, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);

    nogginsBetween(g, leftStuds[0] + STUD_W, leftStuds[1], sy, nogginZs);
    nogginsBetween(g, rightStuds[0] + STUD_W, rightStuds[1], sy, nogginZs);
    nogginsBetween(g, leftStuds[0] + STUD_W, leftStuds[1], sy, [320, 2230], 40);
    nogginsBetween(g, rightStuds[0] + STUD_W, rightStuds[1], sy, [320, 2230], 40);

    for (const pbY of [roomY, alcoveY]) {
      box(g, 0, pbY, 0, INTERNAL_W, PB, ACCESS_BOT, COL.pb, 0.98, 2);
      box(g, 0, pbY, ACCESS_BOT, leftW, PB, ACCESS_TOP - ACCESS_BOT, COL.pb, 0.98, 2);
      box(g, rightX, pbY, ACCESS_BOT, rightW, PB, ACCESS_TOP - ACCESS_BOT, COL.pb, 0.98, 2);
      box(g, 0, pbY, ACCESS_TOP, INTERNAL_W, PB, H - ACCESS_TOP, COL.pb, 0.98, 2);
    }

    const oh = ACCESS_TOP - ACCESS_BOT;
    box(g, ACCESS_X - PB, y, ACCESS_BOT, PB, FOOT, oh, COL.pb, 0.98, 2);
    box(g, rightX, y, ACCESS_BOT, PB, FOOT, oh, COL.pb, 0.98, 2);
    box(g, ACCESS_X, y, ACCESS_BOT - PB, ACCESS_W, FOOT, PB, COL.pb, 0.98, 2);
    box(g, ACCESS_X, y, ACCESS_TOP, ACCESS_W, FOOT, PB, COL.pb, 0.98, 2);

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
