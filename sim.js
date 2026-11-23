/*
 * Traffic simulation core — NO drawing code in here.
 *
 * Everything is expressed as cars travelling along 1-D "segments" (lanes, intersection
 * connectors, car-park driveways, private paths inside car parks). A car is just
 * {segment, s (distance along it), v}. A renderer only needs carPose() -> (x, y, heading),
 * so the very same sim can drive a 2-D canvas, three.js, Unity, a GTA-style engine, etc.
 *
 * Every car follows the SAME rules (nobody is scripted):
 *   1. Car-following (Intelligent Driver Model): keep a safe gap, adapt speed.
 *   2. Obey signals: stop on red, stop on yellow if you can do so comfortably.
 *   3. Reserve your path through a junction: only enter if no one is using a crossing path.
 *   4. Never block the box: don't enter unless there is room on the far side.
 *   5. Right turn on red after a full stop, when clear (Florida rule).
 *   6. Pick the lane for your next turn; change lanes only into safe gaps.
 *   7. Merge out of a car park only into a safe gap.
 * Physics: cars have real footprints (oriented boxes). If two touch it is a crash: both lose most of
 *   their speed, become wrecks that block the lane (hazards on), everyone else brakes and steers around
 *   them, and a tow removes them after a while.
 * ~9% of drivers are "aggressive" (random amount): same rules, but smaller gaps, faster,
 *   pushier lane changes, they run late yellows and roll through right-on-red.
 *
 * Trips: every car has a destination car park + a parking spot + an explicit planned route
 * (list of left/straight/right decisions). It drives there, parks for a random time,
 * then picks a new random car park and a new route.
 */
(function (root) {
  'use strict';

  const DX = [1, 0, -1, 0], DY = [0, 1, 0, -1];      // 0=E 1=S 2=W 3=N  (y grows downward)
  const LANES = 3, LW = 3.5, MED = 0.9, H = 16;       // lanes/dir, lane width, median half-width, stop-line distance
  const ROAD_HALF = MED + LANES * LW;
  const CURB = ROAD_HALF + 3;                          // where car parks start (after the sidewalk)
  const LIMIT = 17.9;                                  // 40 mph in m/s
  const MAN_D = [3, 0, 1];                             // heading change for Left, Straight, Right
  const L = 0, S = 1, R = 2;
  const FREE = 1e9;
  const YEL = 3.2, ALLRED = 1.6;
  const AISLE = 6.4, ROW = 5.2, SPOT_W = 2.7, DRIVE_V = 6, PATH_V = 5.5;
  const laneOff = k => MED + (k + 0.5) * LW;

  function mulberry32(a) {
    return function () {
      a |= 0; a = a + 0x6D2B79F5 | 0;
      let t = Math.imul(a ^ a >>> 15, 1 | a);
      t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
      return ((t ^ t >>> 14) >>> 0) / 4294967296;
    };
  }

  /* ---------- geometry ---------- */
  function resample(px, py) {                          // polyline -> uniform arc-length samples with unwrapped angles
    const n = px.length - 1, cum = [0];
    for (let i = 1; i <= n; i++) cum.push(cum[i - 1] + Math.hypot(px[i] - px[i - 1], py[i] - py[i - 1]));
    const len = cum[n], m = Math.max(2, Math.ceil(len / 0.5)), step = len / m;
    const xs = new Float32Array(m + 1), ys = new Float32Array(m + 1), as = new Float32Array(m + 1);
    let j = 0, prev = null;
    for (let k = 0; k <= m; k++) {
      const target = k * step;
      while (j < n - 1 && cum[j + 1] < target) j++;
      const seg = cum[j + 1] - cum[j], f = seg > 0 ? (target - cum[j]) / seg : 0;
      xs[k] = px[j] + (px[j + 1] - px[j]) * f;
      ys[k] = py[j] + (py[j + 1] - py[j]) * f;
      let a = Math.atan2(py[j + 1] - py[j], px[j + 1] - px[j]);
      if (prev !== null) { while (a - prev > Math.PI) a -= 2 * Math.PI; while (a - prev < -Math.PI) a += 2 * Math.PI; }
      as[k] = prev = a;
    }
    return { len, step, xs, ys, as };
  }
  function sampleCurve(P0, Q, P3) {
    const n = 48, px = [], py = [];
    for (let i = 0; i <= n; i++) {
      const t = i / n, u = 1 - t;
      px.push(u * u * P0.x + 2 * u * t * Q.x + t * t * P3.x);
      py.push(u * u * P0.y + 2 * u * t * Q.y + t * t * P3.y);
    }
    return resample(px, py);
  }
  function chaikin(pts, iters) {
    for (let it = 0; it < iters; it++) {
      const out = [pts[0]];
      for (let i = 0; i < pts.length - 1; i++) {
        const a = pts[i], b = pts[i + 1];
        out.push({ x: 0.75 * a.x + 0.25 * b.x, y: 0.75 * a.y + 0.25 * b.y }, { x: 0.25 * a.x + 0.75 * b.x, y: 0.25 * a.y + 0.75 * b.y });
      }
      out.push(pts[pts.length - 1]); pts = out;
    }
    return pts;
  }
  // edge-local (u along the road from the stop line, v to the right of the centre line) -> world
  function edgeP(e, u, v) {
    const d = e.dir;
    return { x: e.a.x + DX[d] * (H + u) - DY[d] * v, y: e.a.y + DY[d] * (H + u) + DX[d] * v };
  }

  /* ---------- vehicle types ---------- */
  const TYPES = [
    { n: 'Sedan', len: 4.6, wid: 1.85, w: 52, v: 1.0, a: 1.0 },
    { n: 'SUV', len: 4.9, wid: 2.0, w: 24, v: 1.0, a: 0.95 },
    { n: 'Pickup', len: 5.6, wid: 2.05, w: 11, v: 1.0, a: 0.9 },
    { n: 'Van', len: 5.3, wid: 2.0, w: 5, v: 0.95, a: 0.85 },
    { n: 'Box truck', len: 8.6, wid: 2.45, w: 4, v: 0.88, a: 0.6 },
    { n: 'Bus', len: 11.5, wid: 2.55, w: 4, v: 0.85, a: 0.55 },
  ];
  const COLORS = ['#f2f2f2', '#f2f2f2', '#f2f2f2', '#c9ced4', '#c9ced4', '#8d949c', '#23262b', '#23262b', '#15171a',
    '#b3262c', '#d0453b', '#2a56a6', '#3f7fd1', '#2c7d4e', '#e1b13a', '#e78a2a', '#6a3a8c', '#7a5230'];
  const PARKED = { type: 4, cars: [], len: 0 };       // sentinel segment for parked cars

  /* ---------- world construction ---------- */
  function createWorld(opts) {
    opts = opts || {};
    const N = opts.n || 10, B = opts.block || 170;
    const rng = mulberry32(opts.seed || 12345);
    const w = { N, B, rng, nodes: [], edges: [], lanes: [], conns: [], drives: [], segs: [], pathSegs: [], blocks: [], lots: [], lotW: [], lotWTotal: 0,
      cars: [], parked: [], time: 0, rtor: opts.rtor !== false, exits: 0, violations: 0, nextId: 1, relocated: 0, trips: 0, crashes: 0, events: [] };

    for (let j = 0; j < N; j++) for (let i = 0; i < N; i++) {
      w.nodes.push({ id: j * N + i, i, j, x: i * B, y: j * B, inE: [null, null, null, null], outE: [null, null, null, null],
        conns: [], phase: (rng() * 4) | 0, sub: 'green', t: rng() * 8, chk: 0 });
    }
    for (const a of w.nodes) for (let d = 0; d < 4; d++) {
      const ni = a.i + DX[d], nj = a.j + DY[d];
      if (ni < 0 || nj < 0 || ni >= N || nj >= N) continue;
      const b = w.nodes[nj * N + ni];
      const e = { id: w.edges.length, a, b, dir: d, len: B - 2 * H, lanes: [] };
      for (let k = 0; k < LANES; k++) {
        const off = laneOff(k);
        const lane = { type: 0, id: w.lanes.length, edge: e, k, len: e.len,
          x0: a.x + DX[d] * H - DY[d] * off, y0: a.y + DY[d] * H + DX[d] * off,
          dx: DX[d], dy: DY[d], ang: d * Math.PI / 2, cars: [], incoming: 0, conns: [[], [], []], vmax: LIMIT };
        e.lanes.push(lane); w.lanes.push(lane);
      }
      w.edges.push(e); a.outE[d] = e; b.inE[d] = e;
    }

    // intersection connectors: lane 0 = left only, lane 1 = straight, lane 2 = straight + right
    for (const n of w.nodes) {
      for (let din = 0; din < 4; din++) {
        const ie = n.inE[din]; if (!ie) continue;
        const has = m => !!n.outE[(din + MAN_D[m]) & 3];
        const laneMans = [[], [], []];
        for (let m = 0; m < 3; m++) if (has(m)) for (const k of (m === L ? [0] : m === S ? [1, 2] : [2])) laneMans[k].push(m);
        for (let k = 0; k < 3; k++) if (!laneMans[k].length) {      // dead lanes at T-junctions / corners get a fallback movement
          const order = k === 0 ? [S, R, L] : k === 1 ? [L, R, S] : [S, L, R];
          for (const m of order) if (has(m)) { laneMans[k].push(m); break; }
        }
        for (let man = 0; man < 3; man++) {
          const dout = (din + MAN_D[man]) & 3, oe = n.outE[dout]; if (!oe) continue;
          const ins = [0, 1, 2].filter(k => laneMans[k].indexOf(man) >= 0);
          for (const ki of ins) {
            const outs = man === L ? [0, 1, 2] : man === S ? [ki] : [2, 1];
            for (const ko of outs) {
              const il = ie.lanes[ki], ol = oe.lanes[ko];
              const P0 = { x: il.x0 + il.dx * il.len, y: il.y0 + il.dy * il.len }, P3 = { x: ol.x0, y: ol.y0 };
              let Q;
              if (man === S) Q = { x: (P0.x + P3.x) / 2, y: (P0.y + P3.y) / 2 };
              else if ((din & 1) === 0) Q = { x: P3.x, y: P0.y };
              else Q = { x: P0.x, y: P3.y };
              const c = sampleCurve(P0, Q, P3);
              const conn = { type: 1, id: w.conns.length, node: n, inLane: il, outLane: ol, man, len: c.len, step: c.step,
                xs: c.xs, ys: c.ys, as: c.as, cars: [], resv: 0, bigResv: 0, conflicts: [], bigC: [],
                vmax: man === L ? 8.5 : man === R ? 6.0 : LIMIT };
              w.conns.push(conn); n.conns.push(conn); il.conns[man].push(conn);
            }
          }
        }
      }
      // which connectors cross / merge with which? (paths closer than 2.4 m)
      const cs = n.conns;
      for (let a = 0; a < cs.length; a++) for (let b = a + 1; b < cs.length; b++) {
        const A = cs[a], Bc = cs[b];
        if (A.inLane === Bc.inLane) continue;           // same lane = just queueing order
        let hit = A.outLane === Bc.outLane, md = hit ? 0 : 1e9;      // merging into the same lane = conflict
        if (!hit) {
          const sa = A.xs.length, sb = Bc.xs.length;
          for (let p = 0; p < sa; p += 2) for (let q = 0; q < sb; q += 2) {
            const dx = A.xs[p] - Bc.xs[q], dy = A.ys[p] - Bc.ys[q], d2 = dx * dx + dy * dy;
            if (d2 < md) md = d2;
          }
          hit = md < 9;
        }
        if (hit) { A.conflicts.push(Bc); Bc.conflicts.push(A); }
        else if (md < 20.25 && !(A.inLane.edge === Bc.inLane.edge && A.man === S && Bc.man === S)) { A.bigC.push(Bc); Bc.bigC.push(A); }   // only matters to buses/trucks (they swing wide)
      }
    }

    // city blocks + car parks. A block's car park opens onto the kerb-side lane of one adjoining road.
    const sideNode = (i, j, d) => d === 0 ? w.nodes[j * N + i] : d === 1 ? w.nodes[j * N + i + 1] : d === 2 ? w.nodes[(j + 1) * N + i + 1] : w.nodes[(j + 1) * N + i];
    for (let j = 0; j < N - 1; j++) for (let i = 0; i < N - 1; i++) {
      const rr = rng(), type = rr < 0.4 ? 0 : rr < 0.6 ? 1 : rr < 0.8 ? 2 : 3;     // 0 homes+shops 1 plaza 2 offices 3 park
      const side = (rng() * 4) | 0, node = sideNode(i, j, side), e = node.outE[side];
      const blk = { i, j, type, side, edge: e, x0: i * B + ROAD_HALF, y0: j * B + ROAD_HALF, s: B - 2 * ROAD_HALF, lot: null,
        tf: [DX[side], DY[side], -DY[side], DX[side], node.x + DX[side] * H, node.y + DY[side] * H] };
      w.blocks.push(blk);
      if (i >= 1 && j >= 1 && i <= N - 3 && j <= N - 3) {
        if (type === 1) makeLot(w, blk, 'ADADADA', 6, e.len - 6, 40, 100, 'Shopping plaza car park', true);
        else if (type === 2) makeLot(w, blk, 'ADADA', 6, e.len - 6, 40, 100, 'Office park car park', true);
        else if (type === 0 && rng() < 0.6) { const u0 = 30 + ((rng() * 36) | 0); makeLot(w, blk, 'AS', u0, u0 + 48, u0 - 2, u0 + 36, 'Strip-mall car park', false); }
        else if (type === 3 && rng() < 0.7) { const u0 = 30 + ((rng() * 36) | 0); makeLot(w, blk, 'AS', u0, u0 + 48, u0 - 2, u0 + 36, 'Park car park', false); }
      }
    }
    for (const l of w.lots) { w.lotWTotal += Math.pow(l.spots.length, 0.7); w.lotW.push(w.lotWTotal); }

    w.segs = w.lanes.concat(w.conns, w.drives);
    return w;
  }

  function makeLot(w, blk, bands, u0, u1, sIn, sOut, label, big) {
    const e = blk.edge, d = e.dir;
    const lot = { id: w.lots.length, blk, edge: e, laneIn: e.lanes[2], u0, u1, v0: CURB, v1: CURB, sIn, sOut, big, label,
      spots: [], aisles: [], lines: [], free: 0, uC: u1 - 3.5, inD: null, outD: null };
    const ang0 = Math.atan2(DX[d], -DY[d]);
    let v = CURB;
    const addRow = (vc, ai, ds) => {
      for (let u = u0 + 3; u + SPOT_W <= lot.uC - 4; u += SPOT_W) {
        const uc = u + SPOT_W / 2, p = edgeP(e, uc, vc);
        lot.spots.push({ u: uc, v: vc, ai, ds, x: p.x, y: p.y, a: ds > 0 ? ang0 : ang0 + Math.PI, taken: false, lot });
        lot.lines.push(u, vc - ROW / 2, u, vc + ROW / 2);
      }
      let ue = u0 + 3; while (ue + SPOT_W <= lot.uC - 4) ue += SPOT_W;
      lot.lines.push(ue, vc - ROW / 2, ue, vc + ROW / 2);
    };
    for (const t of bands) {
      if (t === 'A') { lot.aisles.push(v + AISLE / 2); v += AISLE; }
      else if (t === 'S') { addRow(v + ROW / 2, lot.aisles.length - 1, +1); v += ROW; }
      else { addRow(v + ROW / 2, lot.aisles.length - 1, +1); addRow(v + ROW * 1.5, lot.aisles.length, -1); v += 2 * ROW; }
    }
    lot.v1 = v; lot.free = lot.spots.length;
    // driveways: right-in off the kerb lane, right-out back onto it
    const a0 = lot.aisles[0], lane2 = laneOff(2);
    const dIn = sampleCurve(edgeP(e, sIn, lane2), edgeP(e, sIn + 6, lane2), edgeP(e, sIn + 6, a0));
    const dOut = sampleCurve(edgeP(e, sOut, a0), edgeP(e, sOut, lane2), edgeP(e, sOut + 6, lane2));
    lot.inD = { type: 2, kind: 'in', lot, lane: lot.laneIn, len: dIn.len, step: dIn.step, xs: dIn.xs, ys: dIn.ys, as: dIn.as, cars: [], vmax: DRIVE_V };
    lot.outD = { type: 2, kind: 'out', lot, lane: lot.laneIn, len: dOut.len, step: dOut.step, xs: dOut.xs, ys: dOut.ys, as: dOut.as, cars: [], vmax: DRIVE_V, mergeS: sOut + 6 };
    w.drives.push(lot.inD, lot.outD);
    lot.laneIn.lot = lot;
    w.lots.push(lot); blk.lot = lot; return lot;
  }

  // private path inside a car park between the gate and a spot
  function buildPath(lot, spot, kind) {
    const e = lot.edge, A = lot.aisles, a0 = A[0], pts = [];
    const sp = [];                                              // in (u,v) from the gate to the spot
    sp.push([lot.sIn + 6, a0]);
    if (spot.ai !== 0) { sp.push([lot.uC, a0], [lot.uC, A[spot.ai]]); }
    sp.push([spot.u, spot.ai === 0 ? a0 : A[spot.ai]], [spot.u, spot.v]);
    let seq = sp;
    if (kind === 'out') { seq = sp.slice(1).reverse(); seq = seq.slice(); if (spot.ai !== 0) { /* spot -> aisle -> column -> a0 */ } seq.push([lot.sOut, a0]); }
    for (const q of seq) pts.push(edgeP(e, q[0], q[1]));
    const sm = chaikin(pts, 3), c = resample(sm.map(p => p.x), sm.map(p => p.y));
    return { type: 3, kind, lot, spot, len: c.len, step: c.step, xs: c.xs, ys: c.ys, as: c.as, cars: [], vmax: PATH_V };
  }

  /* ---------- traffic signals (4-phase, protected lefts, actuated) ---------- */
  // phase 0: N/S through+right   1: N/S left   2: E/W through+right   3: E/W left
  function sigState(node, din, man) {          // 0 red, 1 yellow, 2 green
    if (node.sub === 'allred') return 0;
    const ns = (din & 1) === 1;
    const need = ns ? (man === L ? 1 : 0) : (man === L ? 3 : 2);
    if (node.phase !== need) return 0;
    return node.sub === 'green' ? 2 : 1;
  }
  function demand(node, ns, leftOnly, range) {
    const dirs = ns ? [1, 3] : [0, 2];
    for (const d of dirs) {
      const e = node.inE[d]; if (!e) continue;
      for (let k = 0; k < LANES; k++) {
        const lane = e.lanes[k], c = lane.cars[lane.cars.length - 1];
        if (c && lane.len - c.s < range && (c.man === L) === leftOnly) return true;
      }
    }
    return false;
  }
  function anyDemand(node, ns, range) {
    const dirs = ns ? [1, 3] : [0, 2];
    for (const d of dirs) {
      const e = node.inE[d]; if (!e) continue;
      for (let k = 0; k < LANES; k++) {
        const lane = e.lanes[k], c = lane.cars[lane.cars.length - 1];
        if (c && lane.len - c.s < range) return true;
      }
    }
    return false;
  }
  function updateSignal(n, dt) {
    n.t += dt;
    if (n.sub === 'green') {
      const left = (n.phase & 1) === 1, ns = n.phase < 2;
      const maxG = left ? 12 : 17, minG = left ? 4 : 7;
      let end = n.t >= maxG;
      if (!end && n.t >= minG) {
        n.chk -= dt;
        if (n.chk <= 0) {
          n.chk = 0.5;
          if (left) { if (!demand(n, ns, true, 40)) end = true; }
          else if (!demand(n, ns, false, 45) && anyDemand(n, !ns, 60)) end = true;
        }
      }
      if (end) { n.sub = 'yellow'; n.t = 0; }
    } else if (n.sub === 'yellow') {
      if (n.t >= YEL) { n.sub = 'allred'; n.t = 0; }
    } else if (n.t >= ALLRED) {
      let p = n.phase;
      for (let k = 0; k < 4; k++) {
        p = (p + 1) & 3;
        if ((p & 1) === 1) { if (demand(n, p < 2, true, 60)) break; } else break;
      }
      n.phase = p; n.sub = 'green'; n.t = 0;
    }
  }

  /* ---------- trips: choose a car park + spot, plan an explicit route ---------- */
  function isBorder(w, n) { return n.i === 0 || n.j === 0 || n.i === w.N - 1 || n.j === w.N - 1; }
  function bfs(w, startNode, startDin, goal, allowBorder) {
    const total = w.nodes.length * 4, prev = new Int32Array(total).fill(-1), pm = new Int8Array(total), seen = new Uint8Array(total);
    const q = [startNode.id * 4 + startDin]; seen[q[0]] = 1;
    const recon = s => { const p = []; while (prev[s] >= 0) { p.push(pm[s]); s = prev[s]; } return p.reverse(); };
    for (let h = 0; h < q.length; h++) {
      const s = q[h], node = w.nodes[s >> 2], din = s & 3;
      const order = [0, 1, 2]; for (let i = 2; i > 0; i--) { const j = (w.rng() * (i + 1)) | 0, t = order[i]; order[i] = order[j]; order[j] = t; }
      for (const m of order) {
        const dout = (din + MAN_D[m]) & 3, oe = node.outE[dout]; if (!oe) continue;
        if (goal.edge && oe === goal.edge) return { path: recon(s).concat([m]), din: dout };
        const nn = oe.b;
        if (!allowBorder && isBorder(w, nn) && !(goal.node !== undefined && nn.id === goal.node)) continue;
        const ns = nn.id * 4 + dout; if (seen[ns]) continue;
        seen[ns] = 1; prev[ns] = s; pm[ns] = m;
        if (goal.node !== undefined && nn.id === goal.node) return { path: recon(ns), din: dout };
        q.push(ns);
      }
    }
    return null;
  }
  // maneuvers to take after travelling along `edge`, ending by turning onto the lot's own edge
  function planRoute(w, lot, edge, canUse) {
    if (canUse && edge === lot.edge) return [];
    const tgt = lot.edge;
    if (w.rng() < 0.35) {                                           // sometimes detour via a random waypoint -> more varied routes
      const N = w.N, wp = w.nodes[(1 + ((w.rng() * (N - 2)) | 0)) + (1 + ((w.rng() * (N - 2)) | 0)) * N];
      if (wp.id !== edge.b.id) {
        const a = bfs(w, edge.b, edge.dir, { node: wp.id }, false);
        if (a) { const b = bfs(w, wp, a.din, { edge: tgt }, false); if (b) return a.path.concat(b.path); }
      }
    }
    const r = bfs(w, edge.b, edge.dir, { edge: tgt }, false) || bfs(w, edge.b, edge.dir, { edge: tgt }, true);
    return r ? r.path : null;
  }
  function chooseLot(w, exclude) {
    for (let t = 0; t < 12; t++) {
      const x = w.rng() * w.lotWTotal; let lo = 0, hi = w.lots.length - 1;
      while (lo < hi) { const mid = (lo + hi) >> 1; if (w.lotW[mid] < x) lo = mid + 1; else hi = mid; }
      const l = w.lots[lo];
      if (l !== exclude && l.free > 0) return l;
    }
    return null;
  }
  function takeSpot(lot) {
    const n = lot.spots.length;
    for (let t = 0; t < 25; t++) { const s = lot.spots[(Math.random() * n) | 0]; if (!s.taken) { s.taken = true; lot.free--; return s; } }
    for (const s of lot.spots) if (!s.taken) { s.taken = true; lot.free--; return s; }
    return null;
  }
  function releaseSpot(car) { if (car.spot) { car.spot.taken = false; car.spot.lot.free++; car.spot = null; } }
  function assignTrip(w, car, edge, s, exclude) {
    for (let t = 0; t < 10; t++) {
      const lot = chooseLot(w, exclude); if (!lot) return false;
      const route = planRoute(w, lot, edge, edge === lot.edge && s < lot.sIn - 45);
      if (!route) continue;
      const sp = takeSpot(lot); if (!sp) continue;
      car.lot = lot; car.spot = sp; car.route = route; car.ri = 0; car.toLot = false; w.trips++;
      return true;
    }
    return false;
  }
  function replanAfter(w, car, edge) {                              // route changed (missed turn): replan from `edge`
    const lot = car.lot;
    const r = lot ? planRoute(w, lot, edge, true) : null;
    if (r) { car.route = r; car.ri = 0; return; }
    releaseSpot(car); assignTrip(w, car, edge, 0);
  }
  function onLaneEntry(w, car, lane) {                              // take the next planned maneuver (or head for the car park)
    car.conn = null; car.toLot = false;
    if (car.ri >= car.route.length && lane.edge !== car.lot.edge) replanAfter(w, car, lane.edge);
    if (car.ri >= car.route.length) { car.toLot = true; car.man = -1; } else car.man = car.route[car.ri++];
  }
  function missedLot(w, car, lane) {
    car.toLot = false; car.conn = null;
    const r = planRoute(w, car.lot, lane.edge, false);
    if (r && r.length) { car.route = r; car.ri = 0; car.man = car.route[car.ri++]; car.missed = 3; return; }
    releaseSpot(car); assignTrip(w, car, lane.edge, lane.len);
    car.ri = 0; car.man = car.route.length ? car.route[car.ri++] : S; car.missed = 3;
  }
  function fallbackMan(w, car, lane) {                               // lane can't do the planned turn any more
    let best = -1, bs = -1;
    for (let m = 0; m < 3; m++) {
      if (!lane.conns[m].length) continue;
      const sc = (m === S ? 3 : 2) * (0.5 + w.rng());
      if (sc > bs) { bs = sc; best = m; }
    }
    return best;
  }
  function ensureConn(w, car, lane) {
    if (car.conn && car.conn.inLane === lane) return car.conn;
    let list = car.man >= 0 ? lane.conns[car.man] : null;
    if (!list || !list.length) {
      const m = fallbackMan(w, car, lane);
      if (m < 0) { car.conn = null; return null; }
      car.man = m; car.missed = 3; list = lane.conns[m];
      replanAfter(w, car, lane.edge.b.outE[(lane.edge.dir + MAN_D[m]) & 3]);
    }
    const nm = car.ri < car.route.length ? car.route[car.ri] : 3;     // the move after this one (3 = into the car park)
    const pref = nm === L ? 0 : nm === R ? 2 : nm === 3 ? 2 : car.bias;
    let best = list[0], bd = 99;
    for (const c of list) { const d = Math.abs(c.outLane.k - pref); if (d < bd) { bd = d; best = c; } }
    car.conn = best; return best;
  }

  /* ---------- cars ---------- */
  function applyAg(c) {                                              // aggression 0..1 reshapes the driving parameters
    const g = c.ag, b = c.base;
    c.v0 = b.v0 * (1 + 0.22 * g); c.T = Math.max(0.45, b.T * (1 - 0.6 * g)); c.s0 = b.s0 * (1 - 0.35 * g);
    c.a = b.a * (1 + 0.7 * g); c.b = b.b * (1 + 0.6 * g);
  }
  function makeCar(w) {
    let tot = 0; for (const t of TYPES) tot += t.w;
    let r = w.rng() * tot, ty = TYPES[0];
    for (const t of TYPES) { if ((r -= t.w) <= 0) { ty = t; break; } }
    const big = ty.len > 8;
    const c = {
      id: w.nextId++, type: ty.n, len: ty.len, wid: ty.wid,
      color: big ? (w.rng() < 0.5 ? '#e8e2d0' : '#e1b13a') : COLORS[(w.rng() * COLORS.length) | 0],
      base: { v0: LIMIT * (0.86 + 0.22 * w.rng()) * ty.v, T: 1.05 + 0.6 * w.rng(), a: (1.3 + 0.9 * w.rng()) * ty.a, b: 2.0 + w.rng(), s0: 1.8 + 0.7 * w.rng() },
      ag: w.rng() < 0.09 ? 0.12 + 0.88 * w.rng() : 0,                // ~9% of drivers are aggressive, by a random amount
      bias: w.rng() < 0.5 ? 1 : 2, seg: null, s: 0, v: 0, acc: 0, lat: 0, latRate: 0,
      lot: null, spot: null, route: [], ri: 0, toLot: false, man: S, conn: null, resv: null,
      lc: 0, stopT: 0, st: 0, ss: 0, ldGap: FREE, missed: 0, rtor: false,
      parked: false, sig: 0, crashed: false, crashT: 0, yaw: 0, ox: 0, oy: 0, ldWreck: false, mergeGo: false, px: 0, py: 0, pa: 0, parkT: 0, pp: null, hold: false, boostT: 0, ignoreSig: false, leaveNow: false,
    };
    applyAg(c); return c;
  }
  function placeCar(w, c) {
    for (let t = 0; t < 50; t++) {
      const lane = w.lanes[(w.rng() * w.lanes.length) | 0];
      const s = 15 + w.rng() * (lane.len - 30);
      let ok = true, idx = 0;
      for (const o of lane.cars) {
        if (Math.abs(o.s - s) < (o.len + c.len) / 2 + 7) { ok = false; break; }
        if (o.s < s) idx++;
      }
      if (!ok) continue;
      if (!assignTrip(w, c, lane.edge, s)) return false;
      c.seg = lane; c.s = s; c.v = Math.min(c.v0, LIMIT) * 0.12 * w.rng(); c.lat = 0; c.resv = null; c.parked = false;
      c.stopT = 0; c.lc = w.rng() * 2; c.rtor = false; c.missed = 0;
      lane.cars.splice(idx, 0, c);
      onLaneEntry(w, c, lane);
      return true;
    }
    return false;
  }
  function placeParked(w, c) {
    const lot = chooseLot(w, null); if (!lot) return false;
    const sp = takeSpot(lot); if (!sp) return false;
    c.lot = lot; c.spot = sp; c.seg = PARKED; c.parked = true; c.s = 0; c.v = 0; c.pp = { x: sp.x, y: sp.y, a: sp.a };
    c.parkT = 5 + w.rng() * 160; c.route = []; c.ri = 0; c.toLot = false; c.resv = null;
    w.parked.push(c); return true;
  }
  function removeCar(w, c) {
    if (c.parked) { const i = w.parked.indexOf(c); if (i >= 0) w.parked.splice(i, 1); }
    else if (c.seg) {
      const arr = c.seg.cars, i = arr.indexOf(c); if (i >= 0) arr.splice(i, 1);
      if (c.seg.type === 3) { const k = w.pathSegs.indexOf(c.seg); if (k >= 0) w.pathSegs.splice(k, 1); }
    }
    unreserve(c);
    releaseSpot(c); c.seg = null; c.parked = false;
  }
  function setCarCount(w, n) {
    while (w.cars.length < n) {
      const c = makeCar(w);
      const ok = (w.rng() < 0.17 && placeParked(w, c)) || placeCar(w, c);
      if (!ok) break;
      w.cars.push(c);
    }
    while (w.cars.length > n) {
      const k = (w.rng() * w.cars.length) | 0, c = w.cars[k];
      removeCar(w, c); w.cars[k] = w.cars[w.cars.length - 1]; w.cars.pop();
    }
  }

  /* ---------- the per-car decision: stop line / reservation ---------- */
  function reserve(car, conn) { car.resv = conn; conn.resv++; conn.outLane.incoming++; if (car.len > 7) conn.bigResv++; }
  function unreserve(car) { const c = car.resv; if (!c) return; c.resv--; c.outLane.incoming--; if (car.len > 7) c.bigResv--; car.resv = null; }
  function tryReserve(car, conn) {
    const cf = conn.conflicts;
    for (let i = 0; i < cf.length; i++) if (cf[i].resv > 0) { car.ss = 4; return false; }    // someone is using a crossing path
    const bc = conn.bigC;                                                                      // a long vehicle swings wide: keep extra clearance either way
    for (let i = 0; i < bc.length; i++) if (bc[i].resv > 0 && (car.len > 7 || bc[i].bigResv > 0)) { car.ss = 4; return false; }
    if (car.stopT < 60) {                                                                     // don't block the box
      const out = conn.outLane, f = out.cars.length ? out.cars[0] : null;
      const room = f ? f.s - f.len * 0.5 + Math.min(f.v, 14) * 2.0 : out.len;
      if (room < out.incoming * 6 + car.len + 1.5) { car.ss = 5; return false; }
    }
    return true;
  }
  function stopLine(w, car, lane, dist) {
    const conn = ensureConn(w, car, lane);
    const d = dist > 0.1 ? dist : 0.1;
    if (!conn) { car.ss = 5; return d; }
    const v = car.v, ag = car.ag;
    if (car.ignoreSig) {                                            // manual override: blow through (others still see the reservation)
      if (dist > 10 + v * 3.2) return FREE;
      reserve(car, conn); car.rtor = false; return FREE;
    }
    const sig = sigState(lane.edge.b, lane.edge.dir, conn.man);
    let allowed = false;
    if (sig === 2) allowed = true;
    else if (sig === 1) { if (v * v / 7 + v * 0.2 + ag * 1.8 * v > dist - 0.5) allowed = true; else car.ss = 3; }  // can't stop comfortably -> go (aggressive: goes more)
    else {
      car.ss = 2;
      if (conn.man === R && w.rtor && v < 0.6 + ag * 4 && dist < 5 + ag * 8) { allowed = true; car.rtor = true; }  // right on red, after a (rolling, if aggressive) stop
    }
    if (!allowed) return d;
    if (dist > 10 + v * 3.2) return FREE;
    if (tryReserve(car, conn)) { reserve(car, conn); return FREE; }
    return d;
  }
  function mergeOK(car, outD) {
    const lane = outD.lane, ms = outD.mergeS, tc = lane.cars;
    const l1 = lane.edge.lanes[1].cars;                              // a long vehicle swings across the next lane too
    for (let q = 0; q < l1.length; q++) if (Math.abs(l1[q].s - ms) < car.len * 0.5 + l1[q].len * 0.5 + 3) return false;
    let fi = 0; while (fi < tc.length && tc[fi].s <= ms) fi++;
    const f = tc[fi], r = fi > 0 ? tc[fi - 1] : null, k = 1 - 0.5 * car.ag;
    const fg = f ? f.s - f.len * 0.5 - (ms + car.len * 0.5) : FREE;
    const rg = r ? (ms - car.len * 0.5) - (r.s + r.len * 0.5) : FREE;
    return fg > (4 + car.v * 0.8 + (f ? Math.max(0, car.v - f.v) * 1.5 : 0)) * k && rg > (6 + (r ? r.v * 2.0 : 0)) * k;
  }
  function laneLook(w, car, seg, dist) {
    if (car.toLot) {
      if (seg.edge === car.lot.edge && car.s <= car.lot.sIn + 3) return null;   // heading for the driveway
      missedLot(w, car, seg);                                                    // drove past it -> go round the block
    }
    return dist < 220 ? ensureConn(w, car, seg) : null;
  }

  /* ---------- Intelligent Driver Model ---------- */
  function computeAcc(w, seg, i) {
    const arr = seg.cars, car = arr[i], T = seg.type;
    if (car.crashed) { car.acc = -12; car.st = 13; car.sig = 0; car.ldWreck = false; return; }   // wreck: skids to a stop, stays put
    let gap = FREE, vl = 0, st = 0, front = false, dist = 0, stopG = FREE, stSt = 0, lw = false;
    if (i + 1 < arr.length) {
      const l = arr[i + 1];
      gap = (l.s - l.len * 0.5) - (car.s + car.len * 0.5); vl = l.v; st = 1; lw = l.crashed;
    } else {
      front = true;
      dist = seg.len - car.s - car.len * 0.5;
      let ns = null;
      if (T === 0) ns = laneLook(w, car, seg, dist); else if (T === 1) ns = seg.outLane;
      let d = dist, depth = 0;
      while (ns && depth < 2) {
        if (ns.cars.length) { const l = ns.cars[0]; gap = d + l.s - l.len * 0.5; vl = l.v; st = 1; lw = l.crashed; break; }
        d += ns.len; depth++; ns = ns.type === 1 ? ns.outLane : null;
      }
    }
    if (T === 0 && seg.lot && car.s < seg.lot.sIn + 4) {           // a car peeling off into the driveway still occupies the lane for a moment
      const id = seg.lot.inD.cars;
      if (id.length && id[0].s < 8) {
        const ld = id[0], f = Math.min(1, ld.s / 6), lp = seg.lot.sIn + Math.min(ld.s, 6);
        const g = (lp - (ld.len * 0.5 * (1 - f) + ld.wid * 0.5 * f)) - (car.s + car.len * 0.5);
        if (g < gap) { gap = g; vl = ld.v * (1 - f); st = 1; lw = ld.crashed; }
      }
    }
    if (T === 0 && seg.lot) {
      const od = seg.lot.outD, oc = od.cars;
      if (oc.length) {
        const ld = oc[oc.length - 1];
        if (ld.s > od.len - 5 && car.s < od.mergeS) {
          const g = (od.mergeS - ld.len * 0.5) - (car.s + car.len * 0.5);
          if (g > -3 && g < gap) { gap = g < 0.1 ? 0.1 : g; vl = ld.v; st = 1; lw = ld.crashed; }
        }
      }
    }
    car.ldGap = gap; car.ldWreck = lw;
    const bm = car.boostT > 0 ? 1.6 : 1;
    let v0 = Math.min(car.v0, seg.vmax) * bm;
    if (front) {
      if (T === 0) {
        if (!car.resv && !car.toLot) { const sg = stopLine(w, car, seg, dist); if (sg < FREE) { stopG = sg; stSt = car.ss; } }
        if (car.conn && car.conn.vmax < v0 && !car.toLot) {             // slow down for the turn ahead
          const cv = car.conn.vmax, lim = Math.sqrt(cv * cv + 4.4 * (dist > 0 ? dist : 0));
          if (lim < v0) v0 = lim;
        }
      } else if (T === 2 && seg.kind === 'out') {
        if (!car.mergeGo && dist < 20) { if (mergeOK(car, seg)) car.mergeGo = true; else { stopG = dist - 7 > 0.1 ? dist - 7 : 0.1; stSt = 4; } }
      } else if (T === 3 && seg.kind === 'in') { stopG = seg.len - car.s + car.s0; }
    }
    if (T === 0 && car.toLot && seg === car.lot.laneIn && car.s < car.lot.sIn) {   // slow for the driveway; wait if it's occupied
      const dd = car.lot.sIn - car.s, lim = Math.sqrt(25 + 4.4 * dd);
      if (lim < v0) v0 = lim;
      const id = car.lot.inD;
      if (id.cars.length && id.cars[0].s < 9) { const g = Math.max(0.1, dd - car.len * 0.5); if (g < stopG) { stopG = g; stSt = 5; } }
    }
    if (stopG < gap) { gap = stopG; vl = 0; st = stSt; }
    const v = car.v, r = v / v0, r2 = r * r, aE = car.a * (car.boostT > 0 ? 2 : 1);
    let acc = aE * (1 - r2 * r2);
    if (gap < FREE * 0.5) {
      if (gap < 0.1) gap = 0.1;
      let ss = car.s0 + v * car.T + v * (v - vl) / (2 * Math.sqrt(aE * car.b));
      if (ss < car.s0) ss = car.s0;
      const q = ss / gap; acc -= aE * q * q;
    }
    if (acc < -9) acc = -9;
    car.acc = acc;
    if (st === 0) {
      if (T >= 2) st = 12; else if (Math.abs(car.lat) > 0.5) st = 8; else if (T === 1) st = 7;
      else if (car.toLot) st = 10; else if (car.missed > 0) st = 9;
    }
    car.st = st;
    // turn signal: -1 left, +1 right (set early so followers can read intent)
    let sg = 0;
    if (T === 0) {
      if (car.toLot && seg === car.lot.laneIn && car.s > car.lot.sIn - 50) sg = 1;
      else if ((car.man === L || car.man === R) && seg.len - car.s < 75) sg = car.man === L ? -1 : 1;
    } else if (T === 1) sg = seg.man === L ? -1 : seg.man === R ? 1 : 0;
    else if (T === 2) sg = 1;
    car.sig = sg;
    if (lw && gap < 60) car.st = 14;
  }

  /* ---------- segment transitions ---------- */
  function transition(w, c) {
    const seg = c.seg;
    if (seg.type === 0) {
      if (c.toLot && seg === c.lot.laneIn && c.s < seg.len) {          // turn into the driveway
        const lot = c.lot, i = seg.cars.indexOf(c); if (i >= 0) seg.cars.splice(i, 1);
        c.s = Math.max(0, c.s - lot.sIn); c.seg = lot.inD; lot.inD.cars.unshift(c); return;
      }
      const conn = c.resv || ensureConn(w, c, seg);
      if (!conn) { c.s = seg.len - 0.01; c.v = 0; return; }
      if (!c.resv) { reserve(c, conn); w.violations++; }
      const i = seg.cars.indexOf(c); if (i >= 0) seg.cars.splice(i, 1);
      c.s -= seg.len; c.seg = conn; conn.cars.unshift(c);
    } else if (seg.type === 1) {
      const out = seg.outLane, i = seg.cars.indexOf(c); if (i >= 0) seg.cars.splice(i, 1);
      unreserve(c); c.rtor = false; w.exits++;
      c.s -= seg.len; c.seg = out; out.cars.unshift(c);
      c.lc = 1 + w.rng() * 1.5; if (c.missed > 0) c.missed--;
      onLaneEntry(w, c, out);
    } else if (seg.type === 2) {
      const i = seg.cars.indexOf(c);
      if (seg.kind === 'in') {                                          // driveway -> private path to the spot
        if (i >= 0) seg.cars.splice(i, 1);
        const path = buildPath(c.lot, c.spot, 'in'); path.cars = [c]; c.seg = path; c.s = Math.max(0, c.s - seg.len); w.pathSegs.push(path);
      } else {                                                          // driveway -> merge into the kerb lane
        if (!c.mergeGo && !mergeOK(c, seg)) { c.s = seg.len - 0.02; c.v = 0; return; }
        c.mergeGo = false;
        if (i >= 0) seg.cars.splice(i, 1);
        const lane = seg.lane, tc = lane.cars; let fi = 0; while (fi < tc.length && tc[fi].s <= seg.mergeS) fi++;
        tc.splice(fi, 0, c); c.seg = lane; c.s = seg.mergeS; c.lc = 2; c.missed = 0;
        onLaneEntry(w, c, lane);
      }
    } else if (seg.type === 3) {
      const rm = () => { const k = w.pathSegs.indexOf(seg); if (k >= 0) w.pathSegs.splice(k, 1); };
      if (seg.kind === 'in') {                                          // arrived: park
        rm(); c.seg = PARKED; c.parked = true; c.s = 0; c.v = 0; c.pp = { x: c.spot.x, y: c.spot.y, a: c.spot.a };
        c.parkT = 12 + w.rng() * w.rng() * 260; c.leaveNow = false; c.route = []; c.ri = 0; c.toLot = false; w.parked.push(c);
      } else {                                                          // leaving the spot -> exit driveway
        const od = seg.lot.outD;
        if (od.cars.length && od.cars[0].s < 8) { c.s = seg.len - 0.02; c.v = 0; return; }
        rm(); c.seg = od; c.s = Math.max(0, c.s - seg.len); od.cars.unshift(c);
      }
    }
  }
  function tryUnpark(w, c, idx) {
    const lot = c.lot, sp = c.spot;
    if (!assignTrip(w, c, lot.edge, lot.sOut + 6, lot)) { c.parkT = 4; c.leaveNow = false; return; }
    sp.taken = false; lot.free++;                                       // old spot is free again (new one was reserved by assignTrip)
    w.parked.splice(idx, 1);
    const path = buildPath(lot, sp, 'out'); path.cars = [c];
    c.seg = path; c.s = 0; c.v = 0; c.parked = false; c.leaveNow = false; c.stopT = 0; c.missed = 0;
    w.pathSegs.push(path);
  }

  /* ---------- lane changing ---------- */
  function laneChanges(w, lane) {
    const arr = lane.cars, e = lane.edge;
    for (let i = arr.length - 1; i >= 0; i--) {
      const c = arr[i], ag = c.ag;
      if (c.lc > 0 || c.resv || c.crashed) continue;
      c.lc = (0.4 + w.rng() * 0.4) / (1 + ag);
      let dl = lane.len - c.s, targets = null, mand = false, wreck = false;
      if (c.ldWreck && c.ldGap < 70 && c.v < c.v0 * 0.9) {              // crash ahead -> steer around it
        if (dl < 8) continue;
        wreck = mand = true;
        targets = w.rng() < 0.5 ? [lane.k - 1, lane.k + 1] : [lane.k + 1, lane.k - 1];
      } else if (c.toLot && e === c.lot.edge) {                         // need the kerb lane before the driveway
        dl = c.lot.sIn - c.s;
        if (dl < 12 || lane.k === 2) continue;
        mand = true; targets = [lane.k + 1];
      } else {
        if (dl < 20) continue;
        const man = c.man, nm = c.ri < c.route.length ? c.route[c.ri] : 3;
        const wantKerb = man === S && nm === 3;                         // going straight then into a car park -> be in the kerb lane already
        const ok = man >= 0 && lane.conns[man].length > 0 && !(wantKerb && lane.k !== 2);
        let target = -1;
        if (!ok) {
          mand = true;
          if (wantKerb && lane.conns[S].length) target = lane.k + 1;
          else {
            let bd = 99, t = -1;
            for (let k = 0; k < LANES; k++) if (man >= 0 && e.lanes[k].conns[man].length) { const d = Math.abs(k - lane.k); if (d < bd) { bd = d; t = k; } }
            if (t < 0) continue;
            target = lane.k + Math.sign(t - lane.k);
          }
        } else if (man === S && lane.k > 0 && !wantKerb && c.ldGap < 35 + 25 * ag && c.v < c.v0 * (0.7 + 0.25 * ag)) {
          target = lane.k === 1 ? 2 : 1;                                // slower than I'd like -> try the other through lane
        } else continue;
        targets = [target];
      }
      for (const target of targets) {
        if (target < 0 || target >= LANES) continue;
        const tl = e.lanes[target], tc = tl.cars;
        let fi = 0; while (fi < tc.length && tc[fi].s <= c.s) fi++;
        const f = tc[fi], r = fi > 0 ? tc[fi - 1] : null;
        const fg = f ? f.s - f.len * 0.5 - (c.s + c.len * 0.5) : FREE;
        const rg = r ? (c.s - c.len * 0.5) - (r.s + r.len * 0.5) : FREE;
        const urg = wreck ? 1 : mand ? Math.min(1, Math.max(0, (110 - dl) / 90)) : 0, k = (1 - 0.5 * urg) * (1 - 0.55 * ag);
        const needF = (2.5 + 0.45 * c.v + (f ? Math.max(0, c.v - f.v) * 1.2 : 0)) * k;
        const needR = (2.5 + (r ? Math.max(0, r.v - c.v) * 1.4 + r.v * 0.35 : 0)) * k;
        if (fg < needF || rg < needR) continue;
        if (!mand && fg < c.ldGap + 15 * (1 - ag)) continue;
        arr.splice(i, 1); tc.splice(fi, 0, c);
        c.seg = tl; c.lat += laneOff(lane.k) - laneOff(target); c.conn = null; c.lc = (2.5 + w.rng() * 2) / (1 + ag);
        if (!mand && (target === 1 || target === 2)) c.bias = target;
        break;
      }
    }
  }

  /* ---------- physics: footprints, collisions, wrecks ---------- */
  const _P = { x: 0, y: 0, a: 0 };
  function obb(ax, ay, aa, al, aw, bx, by, ba, bl, bw) {          // separating-axis test; returns {d, x, y} (axis from a to b) or null
    const ca = Math.cos(aa), sa = Math.sin(aa), cb = Math.cos(ba), sb = Math.sin(ba), dx = bx - ax, dy = by - ay;
    const AX = [ca, -sa, cb, -sb], AY = [sa, ca, sb, cb];
    let best = 1e9, mx = 0, my = 0;
    for (let k = 0; k < 4; k++) {
      const ux = AX[k], uy = AY[k];
      const ra = al * Math.abs(ca * ux + sa * uy) + aw * Math.abs(-sa * ux + ca * uy);
      const rb = bl * Math.abs(cb * ux + sb * uy) + bw * Math.abs(-sb * ux + cb * uy);
      const proj = dx * ux + dy * uy, ov = ra + rb - Math.abs(proj);
      if (ov <= 0) return null;
      if (ov < best) { best = ov; const g = proj >= 0 ? 1 : -1; mx = ux * g; my = uy * g; }
    }
    return { d: best, x: mx, y: my };
  }
  function collide(w, a, b, m) {
    const rvx = a.v * Math.cos(a.pa) - b.v * Math.cos(b.pa), rvy = a.v * Math.sin(a.pa) - b.v * Math.sin(b.pa), rel = Math.hypot(rvx, rvy);
    if (m.d < 0.55 && rel < 5) return;                              // a gentle scrape, not a crash
    let fresh = false;
    for (const c of [a, b]) if (!c.crashed) {
      fresh = true; c.crashed = true; c.crashT = 14 + Math.min(26, rel * 2.2) + w.rng() * 8;
      c.v *= 0.25; c.hold = false; c.ignoreSig = false; c.boostT = 0;
      c.yaw = (w.rng() - 0.5) * Math.min(0.9, 0.15 + rel * 0.06);
    }
    // push the two apart so they rest touching, not overlapping
    const k = a.crashed && b.crashed ? 0.5 : 0.5;
    a.ox -= m.x * m.d * k; a.oy -= m.y * m.d * k; b.ox += m.x * m.d * k; b.oy += m.y * m.d * k;
    if (fresh && w.onCrash) w.onCrash(a, b, m);
    if (fresh) { w.crashes++; w.events.push({ x: (a.px + b.px) / 2, y: (a.py + b.py) / 2, t: w.time, rel }); }
  }
  function detectCollisions(w) {
    const cell = 10, grid = new Map(), list = [];
    for (const c of w.cars) {
      if (c.parked || !c.seg || c.seg.type > 2) continue;           // car-park internals are not simulated for collisions
      carPose(c, _P); c.px = _P.x; c.py = _P.y; c.pa = _P.a;
      const kx = Math.floor(c.px / cell) + 100, ky = Math.floor(c.py / cell) + 100, key = kx * 10000 + ky;
      let g = grid.get(key); if (!g) grid.set(key, g = []); g.push(c); list.push(c); c._kx = kx; c._ky = ky;
    }
    for (const c of list) {
      for (let ix = c._kx - 1; ix <= c._kx + 1; ix++) for (let iy = c._ky - 1; iy <= c._ky + 1; iy++) {
        const g = grid.get(ix * 10000 + iy); if (!g) continue;
        for (const o of g) {
          if (o.id <= c.id) continue;
          if (c.crashed && o.crashed && c.v < 0.2 && o.v < 0.2) continue;   // two resting wrecks
          const rr = (c.len + o.len) * 0.5 + 0.5, dx = o.px - c.px, dy = o.py - c.py;
          if (dx * dx + dy * dy > rr * rr) continue;
          const m = obb(c.px, c.py, c.pa, c.len * 0.5 - 0.35, c.wid * 0.5 - 0.22, o.px, o.py, o.pa, o.len * 0.5 - 0.35, o.wid * 0.5 - 0.22);
          if (m) collide(w, c, o, m);
        }
      }
    }
  }

  /* ---------- main step ---------- */
  function step(w, dt) {
    w.time += dt;
    const nodes = w.nodes;
    for (let i = 0; i < nodes.length; i++) updateSignal(nodes[i], dt);
    for (let i = w.parked.length - 1; i >= 0; i--) { const c = w.parked[i]; c.parkT -= dt; if (c.parkT <= 0 || c.leaveNow) tryUnpark(w, c, i); }
    const lists = [w.segs, w.pathSegs];
    for (const segs of lists) for (let k = 0; k < segs.length; k++) { const arr = segs[k].cars; for (let i = 0; i < arr.length; i++) computeAcc(w, segs[k], i); }
    const trans = [], stuck = [], tow = [];
    for (const segs of lists) for (let k = 0; k < segs.length; k++) {
      const seg = segs[k], arr = seg.cars;
      for (let i = arr.length - 1; i >= 0; i--) {
        const c = arr[i], vo = c.v; let v = vo + c.acc * dt; if (v < 0) v = 0;
        if (c.hold) v = Math.max(0, vo - 9 * dt);
        c.v = v; c.s += (vo + v) * 0.5 * dt;
        if (c.lat) { const m = (c.latRate || 2.2) * dt; if (Math.abs(c.lat) <= m) { c.lat = 0; c.latRate = 0; } else c.lat -= Math.sign(c.lat) * m; }
        if (c.boostT > 0) c.boostT -= dt;
        if (c.lc > 0) c.lc -= dt;
        if (c.crashed) { c.crashT -= dt; if (c.crashT <= 0) tow.push(c); }
        if (v < 0.25 && !c.hold && !c.crashed) { c.stopT += dt; if (c.stopT > 220 && seg.type < 2) stuck.push(c); } else c.stopT = 0;
        const T = seg.type;
        if (c.s >= seg.len || (T === 0 && c.toLot && seg === c.lot.laneIn && c.s >= c.lot.sIn) ||
            (T === 3 && seg.kind === 'in' && seg.len - c.s < 0.5 && v < 1.5)) trans.push(c);
      }
    }
    for (let i = 0; i < trans.length; i++) transition(w, trans[i]);
    for (let k = 0; k < w.lanes.length; k++) if (w.lanes[k].cars.length) laneChanges(w, w.lanes[k]);
    detectCollisions(w);
    for (const c of tow) {                         // tow truck clears the wreck; the car re-enters the world elsewhere
      removeCar(w, c); c.crashed = false; c.yaw = 0; c.ox = 0; c.oy = 0; c.lat = 0; c.v = 0; c.stopT = 0; c.missed = 0; c.hold = false; c.ignoreSig = false;
      if (!((w.rng() < 0.4 && placeParked(w, c)) || placeCar(w, c))) { const i = w.cars.indexOf(c); if (i >= 0) w.cars.splice(i, 1); }
    }
    for (let i = w.events.length - 1; i >= 0; i--) if (w.time - w.events[i].t > 6) w.events.splice(i, 1);
    for (const c of stuck) {                       // last-resort safety net (should basically never fire)
      removeCar(w, c); c.stopT = 0; c.missed = 0; c.hold = false;
      if (!placeCar(w, c)) { const i = w.cars.indexOf(c); if (i >= 0) w.cars.splice(i, 1); }
      w.relocated++;
    }
  }

  /* ---------- manual control of one car (for experiments) ---------- */
  function cmd(w, car, name, arg) {
    switch (name) {
      case 'lane': {                                // harsh forced lane change, ignores gaps. arg: -1 left, +1 right
        const lane = car.seg; if (!lane || lane.type !== 0 || car.resv || car.crashed) return false;
        const t = lane.k + arg; if (t < 0 || t >= LANES) return false;
        const tl = lane.edge.lanes[t], tc = tl.cars; let fi = 0; while (fi < tc.length && tc[fi].s <= car.s) fi++;
        const i = lane.cars.indexOf(car); if (i >= 0) lane.cars.splice(i, 1);
        tc.splice(fi, 0, car); car.seg = tl; car.lat += laneOff(lane.k) - laneOff(t); car.latRate = 7; car.conn = null; car.lc = 5; return true;
      }
      case 'hold': car.hold = !car.hold; return car.hold;
      case 'boost': car.boostT = 8; return true;
      case 'ignore': car.ignoreSig = !car.ignoreSig; return car.ignoreSig;
      case 'ag': car.ag = Math.max(0, Math.min(1, arg)); applyAg(car); return true;
      case 'leave': if (car.parked) car.leaveNow = true; return true;
      case 'newtrip': {
        if (car.parked) { car.leaveNow = true; return true; }
        const lane = car.seg.type === 0 ? car.seg : null; if (!lane) return false;
        const oldLot = car.lot, oldSp = car.spot, oldRoute = car.route, oldRi = car.ri;
        car.spot = null;
        if (assignTrip(w, car, lane.edge, car.s, oldLot)) { if (oldSp) { oldSp.taken = false; oldSp.lot.free++; } car.conn = null; car.toLot = false; car.ri = 0; car.man = car.route.length ? car.route[car.ri++] : -1; if (!car.route.length) car.toLot = true; }
        else { car.spot = oldSp; car.route = oldRoute; car.ri = oldRi; }
        return true;
      }
    }
    return false;
  }

  /* ---------- queries for renderers / UI ---------- */
  function carPose(car, o) {
    if (car.parked) { o.x = car.pp.x; o.y = car.pp.y; o.a = car.pp.a; return o; }
    const seg = car.seg; let x, y, a;
    if (seg.type === 0) { x = seg.x0 + seg.dx * car.s; y = seg.y0 + seg.dy * car.s; a = seg.ang; }
    else {
      let u = car.s / seg.step, i = u | 0; const m = seg.xs.length - 2;
      if (i > m) i = m; if (i < 0) i = 0;
      let f = u - i; if (f < 0) f = 0; else if (f > 1) f = 1;
      x = seg.xs[i] + (seg.xs[i + 1] - seg.xs[i]) * f; y = seg.ys[i] + (seg.ys[i + 1] - seg.ys[i]) * f;
      a = seg.as[i] + (seg.as[i + 1] - seg.as[i]) * f;
    }
    if (car.lat) {
      x -= Math.sin(a) * car.lat; y += Math.cos(a) * car.lat;
      if (car.v > 1) a += (car.lat > 0 ? -1 : 1) * 0.2 * Math.min(1, Math.abs(car.lat) / 1.2) * Math.min(1, car.v / 8);   // nose points into the lane change
    }
    if (car.crashed) { a += car.yaw; x += car.ox; y += car.oy; }
    o.x = x; o.y = y; o.a = a; return o;
  }
  // the planned route as world points: next intersections, then the car park gate / spot
  function routePoints(w, car) {
    const pts = [];
    if (car.parked || !car.seg || !car.lot) return pts;
    const seg = car.seg;
    if (seg.type >= 2) { pts.push({ x: car.spot.x, y: car.spot.y }); return pts; }
    let edge, ms;
    if (seg.type === 0) { edge = seg.edge; ms = (car.man >= 0 ? [car.man] : []).concat(car.route.slice(car.ri)); }
    else { edge = seg.outLane.edge; ms = car.route.slice(car.ri); }
    if (!car.toLot) {
      let node = edge.b, din = edge.dir; pts.push({ x: node.x, y: node.y });
      for (const m of ms) { const dout = (din + MAN_D[m]) & 3, oe = node.outE[dout]; if (!oe) break; node = oe.b; din = dout; pts.push({ x: node.x, y: node.y }); }
    }
    const g = edgeP(car.lot.edge, car.lot.sIn, laneOff(2)); pts.push(g);
    pts.push({ x: car.spot.x, y: car.spot.y });
    return pts;
  }
  function stats(w) {
    let sv = 0, moving = 0, n = 0, ag = 0, wr = 0;
    for (const c of w.cars) { if (c.ag > 0) ag++; if (c.crashed) wr++; if (c.parked) continue; n++; sv += c.v; if (c.v > 0.5) moving++; }
    return { n: w.cars.length, driving: n, parked: w.parked.length, avg: n ? sv / n : 0, moving: n ? moving / n : 0, aggressive: ag, crashes: w.crashes, wrecks: wr };
  }

  root.Sim = { createWorld, step, setCarCount, carPose, sigState, stats, cmd, routePoints, mulberry32, edgeP,
    K: { DX, DY, LANES, LW, MED, H, ROAD_HALF, CURB, LIMIT, laneOff, L, S, R }, STATE_TEXT: [
      'Cruising — free road', 'Following the car ahead (keeping a safe gap)', 'Stopped / slowing for a red light',
      'Yellow light — stopping', 'Yielding — waiting for a safe gap / crossing traffic', 'Waiting — exit is full, won\'t block the box',
      'Right on red after a full stop', 'Driving through the intersection', 'Changing lanes into a safe gap',
      'Missed a turn — re-routing', 'Heading for the car park entrance', 'Parked', 'Driving inside the car park',
      'Crashed — waiting for a tow', 'Crash ahead — braking / steering around'] };
  if (typeof module !== 'undefined') module.exports = root.Sim;
})(typeof window !== 'undefined' ? window : globalThis);
