/**
 * Cabin bed — component assembly model (mm).
 *
 * Orientation = locked front aspect / top_plan_FINAL.png (LOCKED-IN-SPEC):
 * Looking into the alcove from the room — FOOT LEFT, HEAD RIGHT.
 * Narrow access pier (~300mm) at the FOOT (left); wider pier at the HEAD.
 * Origin = back-left-floor (rear ∩ foot wall ∩ floor).
 *
 * Width: INTERNAL_W = 2246 FIXED. Depth may grow (foot extension).
 * HEAD_W is residual after mattress + 2×TOL — never grow INTERNAL_W for clearances.
 * Do NOT invert head/foot for a mirrored 3D viewer — fix the view, not the room.
 */

export const DIM = {
  FOOT: 98,
  INTERNAL_W: 2246, // FIXED alcove clear width (original plan)
  get W() { return this.INTERNAL_W + this.FOOT; }, // 2344
  D: 1242, // depth into room (includes foot-wall extension)
  EXT: 237, // foot wall extension at FRONT — depth growth only
  H: 2380,
  REAR: 210,
  SHELF: 1200,
  PLINTH: 170,
  FRAME_H: 360,
  get PLATFORM() { return this.PLINTH + this.FRAME_H; }, // 530
  MATT_W: 1900,
  MATT_D: 900,
  MATT_H: 220,
  TOL: 10, // each end; absorbed by HEAD_W, not by growing INTERNAL_W
  get FRAME_D() { return this.MATT_D + 2 * this.TOL; }, // 920
  /** Bed-face → head wall; residual of fixed INTERNAL_W */
  get HEAD_W() { return this.INTERNAL_W - this.MATT_W - 2 * this.TOL; }, // 326
  ACCESS_W: 1300,
  FOOT_PIER: 300,
  get ACCESS_X() { return this.FOOT + this.FOOT_PIER; }, // 398
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
  get BED_BAY() { return this.FRONT_Y - this.REAR; },
  get FRAME_TUCK() { return (this.BED_BAY - this.FRAME_D) / 2; },
  get ledger_z() { return this.SHELF - this.DECK_T - this.LEDGER_H; },
  get fascia_h() { return this.SHELF - this.DECK_T; },
  get head_bed_face_x() { return this.W - this.HEAD_W; },
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
  // Foot wall LEFT (0..FOOT), head wall RIGHT ghost, back wall
  box(footGroup, 0, 0, 0, FOOT, D - EXT, H, COL.existing, 0.35);
  box(root, FOOT, -20, 0, INTERNAL_W, 20, H, COL.ghost, 0.18);
  box(headGroup, W, 0, 0, 20, FRONT_Y, H, COL.ghost, 0.18);

  // —— Foot extension +237mm at FRONT — LEFT (0..FOOT), flush rules ——
  {
    const y0 = D - EXT;
    const studX = PB;
    box(footGroup, studX, y0 + 40, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    box(footGroup, studX, D - STUD_FL - 40, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    box(footGroup, (FOOT - 72) / 2, y0, 0, 72, EXT, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(footGroup, (FOOT - 72) / 2, y0, H - TRACK_LEG, 72, EXT, TRACK_LEG, COL.track, 1, 1, 0.55);
    const yA = y0 + 40 + STUD_FL;
    const yB = D - STUD_FL - 40;
    for (const z of nogginZs) {
      box(footGroup, studX, yA, z - 20, STUD_W, yB - yA, 40, COL.noggin, 1, 1, 0.55);
    }
    box(footGroup, 0, y0, 0, PB, EXT, H, COL.pb, 0.98, 2);
    box(footGroup, FOOT - PB, y0, 0, PB, EXT, H, COL.pb, 0.98, 2);
    box(footGroup, 0, D - PB, 0, FOOT, PB, H, COL.pb, 0.98, 2);
    box(footGroup, 0, y0, 0, FOOT, EXT, H, COL.footExt, 0.06);
  }

  // —— Rear shelf (stops before head) ——
  {
    const g = rearGroup;
    const sx = FOOT;
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
    // PB flush with deck front (Y=REAR) — not proud; router-ease nosing for lean
    box(g, sx, REAR - PB, 0, shelfW, PB, fasciaH, COL.pb, 0.98, 2);
    box(g, sx + shelfW / 2 - 150, 0, 935, 300, 40, 80, COL.ledger, 1, 1);
  }

  // —— Head shelf/fascia RIGHT — envelope inside fixed HEAD_W (no bay theft) ——
  {
    const g = headGroup;
    const hx = DIM.head_bed_face_x; // finished bed face; HEAD_W = hx → W
    const studX = hx + PB;
    box(g, W - LEDGER, 0, ledgerZ, LEDGER, FRONT_Y, DIM.LEDGER_H, COL.ledger, 1, 1);
    box(g, hx, 0, SHELF - deckT, HEAD_W, FRONT_Y, deckT, COL.deck, 1, 2);
    // Corner post at rear/head L — solid pick-up fixed toward head wall
    box(g, studX, REAR - STUD_FL, 0, STUD_W, STUD_FL, fasciaH, COL.stud, 1, 1, 0.55);
    const ys = [REAR + 80, FRONT_Y / 2 - STUD_W / 2, FRONT_Y - STUD_W - 40];
    for (const y of ys) {
      box(g, studX, y, 0, STUD_FL, STUD_W, fasciaH, COL.stud, 1, 1, 0.55);
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
    // Bed face PB inside HEAD_W; exposed bay only
    box(g, hx, REAR, 0, PB, FRONT_Y - REAR, fasciaH, COL.pb, 0.98, 2);
    // Front return flush with deck front (not proud into front wall)
    box(g, hx, FRONT_Y - PB, 0, HEAD_W, PB, fasciaH, COL.pb, 0.98, 2);
  }

  // —— Bed (pillow toward HEAD / right) ——
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

  // —— Front fascia: X=FOOT..W; narrow pier at FOOT (left) ——
  {
    const g = frontGroup;
    const leftW = ACCESS_X - FOOT; // 300 at foot
    const rightX = ACCESS_X + ACCESS_W;
    const rightW = W - rightX; // ~646 at head
    const y = FRONT_Y;
    const ty = y + (FOOT - 72) / 2;
    const sy = y + (FOOT - STUD_FL) / 2;
    const roomY = y + FOOT - PB;
    const alcoveY = y;

    box(g, FOOT, ty, 0, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, FOOT, ty, H - TRACK_LEG, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, FOOT, ty, ACCESS_BOT - TRACK_LEG, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);
    box(g, FOOT, ty, ACCESS_TOP, INTERNAL_W, 72, TRACK_LEG, COL.track, 1, 1, 0.55);

    const leftStuds = [FOOT + 25, ACCESS_X - STUD_W - 25];
    const rightStuds = [rightX + 25, W - STUD_W - 25];
    for (const x of leftStuds) box(g, x, sy, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);
    for (const x of rightStuds) box(g, x, sy, 0, STUD_W, STUD_FL, H, COL.stud, 1, 1, 0.55);

    nogginsBetween(g, leftStuds[0] + STUD_W, leftStuds[1], sy, nogginZs);
    nogginsBetween(g, rightStuds[0] + STUD_W, rightStuds[1], sy, nogginZs);
    nogginsBetween(g, leftStuds[0] + STUD_W, leftStuds[1], sy, [320, 2230], 40);
    nogginsBetween(g, rightStuds[0] + STUD_W, rightStuds[1], sy, [320, 2230], 40);

    for (const pbY of [roomY, alcoveY]) {
      box(g, FOOT, pbY, 0, INTERNAL_W, PB, ACCESS_BOT, COL.pb, 0.98, 2);
      box(g, FOOT, pbY, ACCESS_BOT, leftW, PB, ACCESS_TOP - ACCESS_BOT, COL.pb, 0.98, 2);
      box(g, rightX, pbY, ACCESS_BOT, rightW, PB, ACCESS_TOP - ACCESS_BOT, COL.pb, 0.98, 2);
      box(g, FOOT, pbY, ACCESS_TOP, INTERNAL_W, PB, H - ACCESS_TOP, COL.pb, 0.98, 2);
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
