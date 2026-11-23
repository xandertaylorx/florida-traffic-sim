/*
 * 3D renderer for the traffic sim (three.js). Reads the same world that sim.js produces:
 * the sim is untouched — this file only turns (x, y, heading) into meshes.
 * Sim (x, y) -> three (x, z); north (-y) is -z; y is up.
 */
(function (root) {
  'use strict';
  const T = root.THREE;
  const V = { ready: false };
  let renderer, scene, camera, W = 1, Hh = 1, world, Sim, K;
  let dyn = [];                                   // everything we must dispose on rebuild
  let bodyM, cabM, wheelM, lampM, shadowM, smokeM, sigLampM, sigBarM, pathsM, ringSel, routeM, beacon, evRings = [];
  let sigList = [], sigLast = null, maxCars = 0, routeArr, routeAttr;
  const cam = { tx: 0, tz: 0, dist: 600, az: 0.5, el: 0.9 };
  V.cam = cam; V.followCar = null; V.chaseDist = 15;
  const P = { x: 0, y: 0, a: 0 }, _v = new T.Vector3();
  const COL = {};
  const col = h => COL[h] || (COL[h] = (() => { const c = new T.Color(h); return [c.r, c.g, c.b]; })());
  const SIGC = [col('#ff2b2b'), col('#ffc020'), col('#27e36a')];

  /* ---------- tiny geometry builder (flat-shaded, vertex colours) ---------- */
  function MB() { this.p = []; this.c = []; }
  MB.prototype.tri = function (a, b, c, k) { this.p.push(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]); for (let i = 0; i < 3; i++) this.c.push(k[0], k[1], k[2]); };
  MB.prototype.quad = function (a, b, c, d, k) { this.tri(a, b, c, k); this.tri(a, c, d, k); };
  // oriented box: centre (cx,cz), half sizes hx along axis (ax,az), hz along its perpendicular, from y0 to y1
  MB.prototype.obox = function (cx, cz, y0, y1, hx, hz, ax, az, k) {
    const bx = -az, bz = ax, S = [[-1, -1], [1, -1], [1, 1], [-1, 1]];
    const pt = (s, y) => [cx + ax * hx * s[0] + bx * hz * s[1], y, cz + az * hx * s[0] + bz * hz * s[1]];
    const top = S.map(s => pt(s, y1)), bot = S.map(s => pt(s, y0));
    this.quad(top[0], top[1], top[2], top[3], k);
    for (let i = 0; i < 4; i++) { const j = (i + 1) & 3; this.quad(bot[i], bot[j], top[j], top[i], shade(k, i & 1 ? 0.82 : 0.92)); }
  };
  MB.prototype.hip = function (cx, cz, y, hx, hz, ax, az, h, k) {
    const bx = -az, bz = ax, S = [[-1, -1], [1, -1], [1, 1], [-1, 1]];
    const pt = s => [cx + ax * hx * s[0] + bx * hz * s[1], y, cz + az * hx * s[0] + bz * hz * s[1]], apex = [cx, y + h, cz];
    for (let i = 0; i < 4; i++) this.tri(pt(S[i]), pt(S[(i + 1) & 3]), apex, shade(k, 0.8 + 0.1 * (i & 1) + 0.05 * i));
  };
  MB.prototype.cone = function (cx, y, cz, r, h, n, k) {
    for (let i = 0; i < n; i++) {
      const a0 = i / n * 6.2832, a1 = (i + 1) / n * 6.2832;
      this.tri([cx + Math.cos(a0) * r, y, cz + Math.sin(a0) * r], [cx + Math.cos(a1) * r, y, cz + Math.sin(a1) * r], [cx, y + h, cz], shade(k, 0.85 + 0.15 * (i & 1)));
    }
  };
  MB.prototype.mesh = function () {
    const g = new T.BufferGeometry();
    g.setAttribute('position', new T.BufferAttribute(new Float32Array(this.p), 3));
    g.setAttribute('color', new T.BufferAttribute(new Float32Array(this.c), 3));
    g.computeVertexNormals();
    return new T.Mesh(g, new T.MeshLambertMaterial({ vertexColors: true, side: T.DoubleSide }));
  };
  function shade(k, f) { return [k[0] * f, k[1] * f, k[2] * f]; }

  /* ---------- init ---------- */
  V.init = function (cv, sim) {
    Sim = sim; K = sim.K;
    renderer = new T.WebGLRenderer({ canvas: cv, antialias: true });
    renderer.setClearColor(0xbcd9f0);
    scene = new T.Scene();
    scene.fog = new T.Fog(0xbcd9f0, 500, 3800);
    camera = new T.PerspectiveCamera(48, 1, 1, 9000);
    scene.add(new T.HemisphereLight(0xe6f0ff, 0x7a8a70, 0.62));
    const sun = new T.DirectionalLight(0xfff1dc, 0.52); sun.position.set(300, 600, 200); scene.add(sun);
    V.ready = true;
  };
  V.resize = function (w, h, dpr) { W = w; Hh = h; renderer.setPixelRatio(dpr); renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); };

  /* ---------- static world ---------- */
  function lw(b, u, v) { const t = b.tf; return [t[4] + t[0] * u + t[2] * v, t[5] + t[1] * u + t[3] * v]; }

  function buildStatic() {
    const mb = new MB(), rng = Sim.mulberry32(99), N = world.N, B = world.B, R = K.ROAD_HALF, H = K.H, MED = K.MED;
    const ROAD = col('#3b3f46'), WHITE = col('#d9dcdf'), YEL = col('#e8b923'), CURB = col('#c9ccc2'), MEDC = col('#9ea39a');
    const ext = (N - 1) * B;
    mb.obox(ext / 2, ext / 2, -0.05, -0.03, ext / 2 + 1500, ext / 2 + 1500, 1, 0, col('#5b9647'));       // ground
    // roads + junction boxes
    for (const n of world.nodes) {
      mb.obox(n.x, n.y, 0, 0.06, R, R, 1, 0, ROAD);
      if (n.i < N - 1) mb.obox(n.x + B / 2, n.y, 0, 0.06, B / 2, R, 1, 0, ROAD);
      if (n.j < N - 1) mb.obox(n.x, n.y + B / 2, 0, 0.06, B / 2, R, 1, 0, ROAD);
      if (n.i < N - 1) mb.obox(n.x + B / 2, n.y, 0, 0.2, B / 2 - H, MED, 1, 0, MEDC);                 // raised medians
      if (n.j < N - 1) mb.obox(n.x, n.y + B / 2, 0, 0.2, B / 2 - H, MED, 0, 1, MEDC);
    }
    const strip = (ax, az, x0, z0, len, wd, y, k) => {                                                   // flat marking quad from (x0,z0) along (ax,az)
      const bx = -az, bz = ax, w2 = wd / 2;
      mb.quad([x0 + bx * w2, y, z0 + bz * w2], [x0 + ax * len + bx * w2, y, z0 + az * len + bz * w2], [x0 + ax * len - bx * w2, y, z0 + az * len - bz * w2], [x0 - bx * w2, y, z0 - bz * w2], k);
    };
    for (const e of world.edges) {                                                                      // lane dividers, edge lines
      const d = e.dir, dx = K.DX[d], dy = K.DY[d], rx = -dy, rz = dx, a = e.a;
      const sx = a.x + dx * H, sz = a.y + dy * H;
      for (let k = 1; k < K.LANES; k++) {
        const off = MED + k * K.LW;
        for (let s = 2; s < e.len - 6; s += 9) strip(dx, dy, sx + rx * off + dx * s, sz + rz * off + dy * s, 3, 0.14, 0.075, WHITE);
      }
      strip(dx, dy, sx + rx * (R - 0.2), sz + rz * (R - 0.2), e.len, 0.16, 0.075, WHITE);
    }
    for (const n of world.nodes) {                                                                       // yellow median edges, stop lines, crosswalks
      for (const s of [-1, 1]) {
        if (n.i < N - 1) strip(1, 0, n.x + H, n.y + s * MED, B - 2 * H, 0.12, 0.075, YEL);
        if (n.j < N - 1) strip(0, 1, n.x + s * MED, n.y + H, B - 2 * H, 0.12, 0.075, YEL);
      }
      for (let d = 0; d < 4; d++) {
        const dx = K.DX[d], dy = K.DY[d], rx = -dy, rz = dx;
        if (n.inE[d]) strip(rx, rz, n.x - dx * H + rx * MED, n.y - dy * H + rz * MED, R - MED, 0.45, 0.08, WHITE);
        if (n.inE[d] || n.outE[d]) {
          const cx = n.x - dx * 13.6, cz = n.y - dy * 13.6;
          for (let t = -R + 0.8; t < R - 0.6; t += 1.5) strip(dx, dy, cx + rx * t - dx * 1.4, cz + rz * t - dy * 1.4, 2.8, 0.7, 0.078, WHITE);
        }
      }
    }
    // signal masts with heads over each lane group
    const HEAD = col('#16181c'), POLE = col('#555a60');
    for (const n of world.nodes) for (let d = 0; d < 4; d++) {
      if (!n.inE[d]) continue;
      const dx = K.DX[d], dy = K.DY[d], rx = -dy, rz = dx, sx = n.x - dx * H, sz = n.y - dy * H, lat = R + 1.4;
      mb.obox(sx + rx * lat, sz + rz * lat, 0, 6.4, 0.16, 0.16, dx, dy, POLE);
      const mid = (lat + MED) / 2;
      mb.obox(sx + rx * mid, sz + rz * mid, 6.1, 6.4, (lat - MED) / 2, 0.12, rx, rz, POLE);
      for (let k = 0; k < K.LANES; k++) { const off = K.laneOff(k); mb.obox(sx + rx * off, sz + rz * off, 5.3, 6.15, 0.2, 0.32, dx, dy, HEAD); }
    }
    // blocks
    const walls = ['#efe6d6', '#e9dcc3', '#f2efe6', '#dfe6e8', '#f0dfd2'];
    for (const b of world.blocks) {
      const cx = b.x0 + b.s / 2, cz = b.y0 + b.s / 2;
      mb.obox(cx, cz, 0, 0.15, b.s / 2, b.s / 2, 1, 0, CURB);
      mb.obox(cx, cz, 0.1, 0.17, b.s / 2 - 3, b.s / 2 - 3, 1, 0, col(b.ground));
      const ax = b.tf[0], az = b.tf[1];
      const L = b.lot || b.fake;
      if (L) {
        const q = [lw(b, L.u0, K.CURB), lw(b, L.u1, K.CURB), lw(b, L.u1, L.v1), lw(b, L.u0, L.v1)].map(p => [p[0], 0.19, p[1]]);
        mb.quad(q[0], q[1], q[2], q[3], col('#4a4e55'));
        if (b.lot) {
          const ln = b.lot.lines, LC = col('#b9bcc2');
          for (let i = 0; i < ln.length; i += 4) {
            const p0 = lw(b, ln[i], ln[i + 1]), p1 = lw(b, ln[i], ln[i + 3]);
            const dxl = p1[0] - p0[0], dzl = p1[1] - p0[1], l = Math.hypot(dxl, dzl) || 1;
            strip(dxl / l, dzl / l, p0[0], p0[1], l, 0.12, 0.2, LC);
          }
        }
      }
      if (b.pond) {
        const [pu, pv, rx, ry] = b.pond, c0 = lw(b, pu + rx, pv + ry), ca = Math.cos(0.3), sa = Math.sin(0.3), pk = col('#4d9bc0'), ring = [];
        for (let i = 0; i < 24; i++) { const t = i / 24 * 6.2832, lu = rx * Math.cos(t), lv = ry * Math.sin(t); ring.push(lw(b, pu + rx + lu * ca - lv * sa, pv + ry + lu * sa + lv * ca)); }
        for (let i = 0; i < 24; i++) { const a = ring[i], c = ring[(i + 1) % 24]; mb.tri([c0[0], 0.2, c0[1]], [a[0], 0.2, a[1]], [c[0], 0.2, c[1]], pk); }
      }
      for (const r of b.rects) {
        const [u, v, w, h, color, kind] = r, c = lw(b, u + w / 2, v + h / 2), k = col(color);
        if (kind === 'drive') { mb.obox(c[0], c[1], 0.17, 0.2, w / 2, h / 2, ax, az, k); continue; }
        if (kind === 'house') {
          mb.obox(c[0], c[1], 0.17, 3.2, w / 2, h / 2, ax, az, col(walls[(rng() * walls.length) | 0]));
          mb.hip(c[0], c[1], 3.2, w / 2 + 0.5, h / 2 + 0.5, ax, az, 1.7, k);
        } else if (kind === 'shop') { mb.obox(c[0], c[1], 0.17, 4.6, w / 2, h / 2, ax, az, k); mb.obox(c[0], c[1], 4.6, 4.95, w / 2 + 0.2, h / 2 + 0.2, ax, az, col('#555a60')); }
        else if (kind === 'store') { mb.obox(c[0], c[1], 0.17, kind === 'store' ? 9 : 9.5, w / 2, h / 2, ax, az, k); mb.obox(c[0], c[1], 9, 9.6, w / 2 + 0.3, h / 2 + 0.3, ax, az, col('#8d8a82')); }
        else if (kind === 'store2') { mb.obox(c[0], c[1], 0.17, 9.8, w / 2, h / 2, ax, az, k); }
        else if (kind === 'office') {
          const hh = 14 + rng() * 18;
          mb.obox(c[0], c[1], 0.17, hh, w / 2, h / 2, ax, az, k);
          for (let y = 3; y < hh - 1; y += 3.4) mb.obox(c[0], c[1], y, y + 1.4, w / 2 + 0.05, h / 2 + 0.05, ax, az, col('#51697f'));      // window bands
          mb.obox(c[0], c[1], hh, hh + 1.6, w / 4, h / 4, ax, az, col('#7a7f86'));
        }
      }
      for (const c of b.circles) {                                                                       // palms
        const p = lw(b, c.x, c.y), h = 5.5 + (c.r - 2.6) * 2 + rng() * 2.5;
        mb.obox(p[0], p[1], 0.17, h, 0.17, 0.17, 1, 0, col('#7b5a3a'));
        mb.cone(p[0], h - 1.1, p[1], c.r * 0.95, 1.5, 7, col('#3e7d36'));
        mb.cone(p[0], h - 0.1, p[1], c.r * 0.6, 1.1, 6, col('#53a046'));
      }
    }
    // driveways over the kerb
    for (const lot of world.lots) for (const dr of [lot.inD, lot.outD]) {
      for (let i = 0; i + 4 < dr.xs.length; i += 4) {
        const x0 = dr.xs[i], z0 = dr.ys[i], x1 = dr.xs[i + 4], z1 = dr.ys[i + 4], l = Math.hypot(x1 - x0, z1 - z0) || 1;
        strip((x1 - x0) / l, (z1 - z0) / l, x0, z0, l + 0.2, 4.6, 0.185, col('#4a4e55'));
      }
    }
    const m = mb.mesh(); scene.add(m); dyn.push(m);
  }

  /* ---------- dynamic things ---------- */
  function inst(geo, mat, cap, withColor) {
    const m = new T.InstancedMesh(geo, mat, cap);
    m.instanceMatrix.setUsage(T.DynamicDrawUsage); m.frustumCulled = false; m.count = 0;
    if (withColor) { m.instanceColor = new T.InstancedBufferAttribute(new Float32Array(cap * 3), 3); m.instanceColor.setUsage(T.DynamicDrawUsage); }
    scene.add(m); dyn.push(m); return m;
  }
  function setM(arr, i, x, y, z, ca, sa, sx, sy, sz) {
    const o = i * 16;
    arr[o] = ca * sx; arr[o + 1] = 0; arr[o + 2] = sa * sx; arr[o + 3] = 0;
    arr[o + 4] = 0; arr[o + 5] = sy; arr[o + 6] = 0; arr[o + 7] = 0;
    arr[o + 8] = -sa * sz; arr[o + 9] = 0; arr[o + 10] = ca * sz; arr[o + 11] = 0;
    arr[o + 12] = x; arr[o + 13] = y; arr[o + 14] = z; arr[o + 15] = 1;
  }

  V.build = function (w) {
    world = w; Sim.world3 = w;
    for (const o of dyn) { scene.remove(o); if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); if (o.dispose) o.dispose(); }
    dyn = [];
    buildStatic();
    const box = new T.BoxGeometry(1, 1, 1), basic = () => new T.MeshBasicMaterial();
    maxCars = w.N * w.N * 22 + 50;
    bodyM = inst(box, new T.MeshLambertMaterial(), maxCars, true);
    cabM = inst(box, new T.MeshLambertMaterial(), maxCars, true);
    const wg = new T.CylinderGeometry(0.32, 0.32, 0.26, 10); wg.rotateX(Math.PI / 2);
    wheelM = inst(wg, new T.MeshLambertMaterial({ color: 0x15171a }), maxCars * 4, false);
    lampM = inst(box, basic(), maxCars * 9, true);
    const sg = new T.PlaneGeometry(1, 1); sg.rotateX(-Math.PI / 2);
    shadowM = inst(sg, new T.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.28, depthWrite: false }), maxCars, false);
    smokeM = inst(new T.SphereGeometry(1, 8, 6), new T.MeshLambertMaterial({ color: 0x777777, transparent: true, opacity: 0.4, depthWrite: false }), 400, false);
    // signals (lamps over lanes + glowing stop bars)
    sigList = []; for (const n of w.nodes) for (let d = 0; d < 4; d++) if (n.inE[d]) sigList.push({ n, d });
    sigLampM = inst(box, basic(), sigList.length * 3, true); sigBarM = inst(box, basic(), sigList.length * 3, true);
    sigLast = new Int8Array(sigList.length * 3).fill(-1);
    sigList.forEach((s, i) => {
      const d = s.d, dx = K.DX[d], dy = K.DY[d], rx = -dy, rz = dx, ca = Math.cos(d * Math.PI / 2), sa = Math.sin(d * Math.PI / 2);
      const sx = s.n.x - dx * K.H, sz = s.n.y - dy * K.H;
      for (let k = 0; k < 3; k++) {
        const off = K.laneOff(k), idx = i * 3 + k;
        setM(sigLampM.instanceMatrix.array, idx, sx + rx * off - dx * 0.24, 5.72, sz + rz * off - dy * 0.24, ca, sa, 0.1, 0.44, 0.44);
        setM(sigBarM.instanceMatrix.array, idx, sx + rx * off, 0.1, sz + rz * off, ca, sa, 0.7, 0.04, K.LW * 0.9);
      }
    });
    sigLampM.count = sigBarM.count = sigList.length * 3; sigLampM.instanceMatrix.needsUpdate = sigBarM.instanceMatrix.needsUpdate = true;
    // junction paths (debug)
    const pm = new MB(), pc = col('#4cc2ff');
    for (const c of w.conns) for (let i = 0; i + 4 < c.xs.length; i += 4) {
      const x0 = c.xs[i], z0 = c.ys[i], x1 = c.xs[i + 4], z1 = c.ys[i + 4], l = Math.hypot(x1 - x0, z1 - z0) || 1, ax = (x1 - x0) / l, az = (z1 - z0) / l, bx = -az * 0.2, bz = ax * 0.2;
      pm.quad([x0 + bx, 0.3, z0 + bz], [x1 + bx, 0.3, z1 + bz], [x1 - bx, 0.3, z1 - bz], [x0 - bx, 0.3, z0 - bz], pc);
    }
    pathsM = pm.mesh(); pathsM.material = new T.MeshBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.6 }); pathsM.visible = false; scene.add(pathsM); dyn.push(pathsM);
    // selection helpers
    const rg = new T.RingGeometry(2.7, 3.2, 28); rg.rotateX(-Math.PI / 2);
    ringSel = new T.Mesh(rg, new T.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.9, depthTest: false })); ringSel.visible = false; ringSel.renderOrder = 5; scene.add(ringSel); dyn.push(ringSel);
    routeArr = new Float32Array(80 * 18);
    const rgeo = new T.BufferGeometry(); routeAttr = new T.BufferAttribute(routeArr, 3); routeAttr.setUsage(T.DynamicDrawUsage); rgeo.setAttribute('position', routeAttr);
    routeM = new T.Mesh(rgeo, new T.MeshBasicMaterial({ color: 0x4cc2ff, transparent: true, opacity: 0.8, side: T.DoubleSide, depthTest: true })); routeM.frustumCulled = false; routeM.visible = false; routeM.renderOrder = 4; scene.add(routeM); dyn.push(routeM);
    beacon = new T.Mesh(new T.CylinderGeometry(1.4, 1.4, 40, 14, 1, true), new T.MeshBasicMaterial({ color: 0xffd24a, transparent: true, opacity: 0.35, side: T.DoubleSide, depthWrite: false })); beacon.visible = false; scene.add(beacon); dyn.push(beacon);
    evRings = [];
    for (let i = 0; i < 6; i++) { const g = new T.RingGeometry(0.85, 1, 32); g.rotateX(-Math.PI / 2); const m = new T.Mesh(g, new T.MeshBasicMaterial({ color: 0xff4630, transparent: true, depthTest: false })); m.visible = false; m.renderOrder = 6; scene.add(m); dyn.push(m); evRings.push(m); }
    V.overview();
  };

  /* ---------- camera ---------- */
  V.overview = function () {
    const span = (world.N - 1) * world.B + 300;
    cam.tx = cam.tz = (world.N - 1) * world.B / 2; cam.dist = span * 0.95; cam.el = 0.95; cam.az = 0.55; V.followCar = null;
  };
  V.orbit = function (dx, dy) { cam.az -= dx * 0.005; cam.el = Math.max(0.06, Math.min(1.52, cam.el + dy * 0.005)); V.followCar = null; };
  V.pan = function (dx, dy) {
    const sc = cam.dist * 0.0017, sa = Math.sin(cam.az), ca = Math.cos(cam.az);
    cam.tx += -ca * dx * sc + -sa * dy * sc; cam.tz += sa * dx * sc + -ca * dy * sc; V.followCar = null;
    const lim = (world.N - 1) * world.B + 300; cam.tx = Math.max(-300, Math.min(lim, cam.tx)); cam.tz = Math.max(-300, Math.min(lim, cam.tz));
  };
  V.zoom = function (f) {
    if (V.followCar) { V.chaseDist = Math.max(6, Math.min(80, V.chaseDist / f)); return; }
    const span = (world.N - 1) * world.B + 300; cam.dist = Math.max(10, Math.min(span * 2.4, cam.dist / f));
  };
  V.follow = function (car) { V.followCar = car || null; if (car) { cam.el = 0.3; V.chaseDist = 14 + car.len; } };
  V.focus = function (car) {                                  // frame a car without chasing it
    Sim.carPose(car, P); cam.tx = P.x; cam.tz = P.y; cam.dist = Math.min(cam.dist, 90);
  };
  V.pick = function (sx, sy) {
    let best = null, bd = 28 * 28; camera.updateMatrixWorld();
    for (const c of world.cars) {
      Sim.carPose(c, P); _v.set(P.x, 1, P.y).project(camera);
      if (_v.z > 1 || _v.z < -1) continue;
      const px = (_v.x * 0.5 + 0.5) * W, py = (-_v.y * 0.5 + 0.5) * Hh, d = (px - sx) * (px - sx) + (py - sy) * (py - sy);
      if (d < bd) { bd = d; best = c; }
    }
    return best;
  };

  /* ---------- per-frame ---------- */
  const DIM = { Sedan: { hb: 0.62, hc: 0.5, cl: 0.5, cs: -0.06 }, SUV: { hb: 0.85, hc: 0.55, cl: 0.58, cs: -0.07 }, Pickup: { hb: 0.8, hc: 0.5, cl: 0.36, cs: 0.1 },
    Van: { hb: 1.5, hc: 0, cl: 0, cs: 0 }, 'Box truck': { hb: 2.5, hc: 0, cl: 0, cs: 0 }, Bus: { hb: 2.3, hc: 0.75, cl: 0.94, cs: 0 } };
  const AMBER = [1, 0.66, 0], RED = [1, 0.12, 0.1], ORG = [1, 0.45, 0.1];

  function updateCars(opts) {
    const bA = bodyM.instanceMatrix.array, cA = cabM.instanceMatrix.array, wA = wheelM.instanceMatrix.array, lA = lampM.instanceMatrix.array, sA = shadowM.instanceMatrix.array;
    const bC = bodyM.instanceColor.array, cC = cabM.instanceColor.array, lC = lampM.instanceColor.array;
    const blink = ((world.time * 2) | 0) % 2 === 0, R2 = Math.pow(Math.max(450, cam.dist * 1.7), 2);
    let n = 0, smoke = 0; const sm = smokeM.instanceMatrix.array;
    for (const c of world.cars) {
      Sim.carPose(c, P);
      const dx = P.x - cam.tx, dz = P.y - cam.tz; if (dx * dx + dz * dz > R2) continue;
      const ca = Math.cos(P.a), sa = Math.sin(P.a), d = DIM[c.type] || DIM.Sedan, len = c.len, wid = c.wid, big = len > 8;
      let rgb = c._rgb || (c._rgb = col(c.color));
      if (c.crashed) rgb = shade(rgb, 0.5);
      const yb = 0.28 + d.hb / 2;
      setM(bA, n, P.x, yb, P.y, ca, sa, len, d.hb, wid); bC[n * 3] = rgb[0]; bC[n * 3 + 1] = rgb[1]; bC[n * 3 + 2] = rgb[2];
      if (d.hc > 0) {
        const lx = d.cs * len, wx = P.x + ca * lx, wz = P.y + sa * lx;
        setM(cA, n, wx, 0.28 + d.hb + d.hc / 2, wz, ca, sa, len * d.cl, d.hc, wid * (c.type === 'Bus' ? 1.02 : 0.86));
        const k = c.type === 'Bus' ? [0.1, 0.14, 0.2] : shade(rgb, 0.32); cC[n * 3] = k[0]; cC[n * 3 + 1] = k[1]; cC[n * 3 + 2] = k[2];
      } else setM(cA, n, 0, -50, 0, 1, 0, 0, 0, 0);
      const ws = big ? 1.3 : 1, wl = len * (big ? 0.3 : 0.32), wz2 = wid / 2 - 0.12;
      for (let k = 0; k < 4; k++) {
        const lx = (k & 1 ? -wl : wl), lz = (k & 2 ? -wz2 : wz2);
        setM(wA, n * 4 + k, P.x + ca * lx - sa * lz, 0.32 * ws, P.y + sa * lx + ca * lz, ca, sa, ws, ws, 1);
      }
      setM(sA, n, P.x, 0.09, P.y, ca, sa, len * 1.08, 1, wid * 1.12);
      // lamps: 0,1 brake  2..5 indicators (FL, FR, RL, RR)  6 spare  7 spare  8 aggressive roof marker
      const li = n * 9, ly = 0.28 + d.hb * 0.62, braking = !c.parked && !c.crashed && (c.acc < -0.8 || c.v < 0.3 || c.hold);
      let sig = c.parked ? 0 : c.sig || 0; if (Math.abs(c.lat) > 0.3) sig = c.lat > 0 ? -1 : 1;
      const haz = c.crashed || (c.hold && !c.parked);
      const put = (j, lx, lz, y, sx, sy, sz, on, k) => {
        if (on) { setM(lA, li + j, P.x + ca * lx - sa * lz, y, P.y + sa * lx + ca * lz, ca, sa, sx, sy, sz); lC[(li + j) * 3] = k[0]; lC[(li + j) * 3 + 1] = k[1]; lC[(li + j) * 3 + 2] = k[2]; }
        else setM(lA, li + j, 0, -50, 0, 1, 0, 0, 0, 0);
      };
      const hw = wid / 2 - 0.28, tw = wid / 2 - 0.1;
      put(0, -len / 2, -hw, ly, 0.08, 0.2, 0.42, braking, RED); put(1, -len / 2, hw, ly, 0.08, 0.2, 0.42, braking, RED);
      const bl = blink && (sig !== 0 || haz);
      put(2, len / 2, -tw, ly + 0.08, 0.1, 0.16, 0.26, bl && (haz || sig < 0), AMBER); put(3, len / 2, tw, ly + 0.08, 0.1, 0.16, 0.26, bl && (haz || sig > 0), AMBER);
      put(4, -len / 2, -tw, ly + 0.08, 0.1, 0.16, 0.26, bl && (haz || sig < 0), AMBER); put(5, -len / 2, tw, ly + 0.08, 0.1, 0.16, 0.26, bl && (haz || sig > 0), AMBER);
      put(6, 0, 0, 0, 0, 0, 0, false, RED); put(7, 0, 0, 0, 0, 0, 0, false, RED);
      put(8, 0, 0, 0.28 + d.hb + d.hc + 0.2, 0.5, 0.22, 0.5, opts.agHi && c.ag > 0, ORG);
      if (c.crashed && smoke < 130) for (let k = 0; k < 3; k++) {
        const t = ((world.time * 0.45 + k / 3) % 1), s = 0.5 + t * 1.8;
        setM(sm, smoke++, P.x + (k - 1) * 0.5, 1.4 + t * 5, P.y + (k % 2 ? 0.4 : -0.4), 1, 0, s, s, s);
      }
      n++;
    }
    for (const m of [bodyM, cabM, lampM, shadowM]) m.count = n; wheelM.count = n * 4; lampM.count = n * 9; smokeM.count = smoke;
    for (const m of [bodyM, cabM, wheelM, lampM, shadowM, smokeM]) m.instanceMatrix.needsUpdate = true;
    bodyM.instanceColor.needsUpdate = cabM.instanceColor.needsUpdate = lampM.instanceColor.needsUpdate = true;
  }

  function updateSignals() {
    const bc = sigLampM.instanceColor.array, bb = sigBarM.instanceColor.array; let ch = false;
    for (let i = 0; i < sigList.length; i++) {
      const s = sigList[i];
      for (let k = 0; k < 3; k++) {
        const st = Sim.sigState(s.n, s.d, k === 0 ? K.L : K.S), idx = i * 3 + k;
        if (sigLast[idx] === st) continue; sigLast[idx] = st; ch = true;
        const c = SIGC[st]; bc[idx * 3] = c[0]; bc[idx * 3 + 1] = c[1]; bc[idx * 3 + 2] = c[2]; bb[idx * 3] = c[0]; bb[idx * 3 + 1] = c[1]; bb[idx * 3 + 2] = c[2];
      }
    }
    if (ch) sigLampM.instanceColor.needsUpdate = sigBarM.instanceColor.needsUpdate = true;
  }

  function updateSelection(sel) {
    if (!sel || !world.cars.includes(sel)) { ringSel.visible = false; routeM.visible = false; beacon.visible = false; return; }
    Sim.carPose(sel, P);
    ringSel.visible = !V.followCar; ringSel.position.set(P.x, 0.3, P.y); ringSel.scale.setScalar(Math.max(1, sel.len / 4.6));
    const pts = Sim.routePoints(world, sel);
    if (sel.spot) { beacon.visible = true; beacon.position.set(sel.spot.x, 20, sel.spot.y); } else beacon.visible = false;
    let seg = 0; const hw = 0.28; let px = P.x, pz = P.y;
    for (const q of pts) {
      if (seg >= 78) break;
      const dx = q.x - px, dz = q.y - pz, l = Math.hypot(dx, dz); if (l < 0.5) continue;
      const nx = -dz / l * hw, nz = dx / l * hw, o = seg * 18, y = 0.55;
      routeArr.set([px + nx, y, pz + nz, q.x + nx, y, q.y + nz, q.x - nx, y, q.y - nz, px + nx, y, pz + nz, q.x - nx, y, q.y - nz, px - nx, y, pz - nz], o);
      px = q.x; pz = q.y; seg++;
    }
    routeM.geometry.setDrawRange(0, seg * 6); routeAttr.needsUpdate = true; routeM.visible = seg > 0;
  }

  V.render = function (sel, opts, dt) {
    if (!world) return;
    if (V.followCar && world.cars.includes(V.followCar) && !V.followCar.parked) {          // chase cam
      Sim.carPose(V.followCar, P);
      const k = Math.min(1, dt * 5), az = Math.atan2(-Math.cos(P.a), -Math.sin(P.a));
      let da = az - cam.az; da = Math.atan2(Math.sin(da), Math.cos(da));
      cam.az += da * Math.min(1, dt * 3); cam.tx += (P.x - cam.tx) * k; cam.tz += (P.y - cam.tz) * k; cam.dist += (V.chaseDist - cam.dist) * k;
    } else if (V.followCar) { Sim.carPose(V.followCar, P); cam.tx += (P.x - cam.tx) * Math.min(1, dt * 4); cam.tz += (P.y - cam.tz) * Math.min(1, dt * 4); }
    const ce = Math.cos(cam.el);
    camera.position.set(cam.tx + cam.dist * ce * Math.sin(cam.az), Math.max(1.2, cam.dist * Math.sin(cam.el)), cam.tz + cam.dist * ce * Math.cos(cam.az));
    camera.lookAt(cam.tx, V.followCar ? 1.4 : 0, cam.tz);
    camera.near = Math.max(0.3, cam.dist * 0.02); camera.far = 9000; camera.updateProjectionMatrix();
    scene.fog.near = Math.max(600, cam.dist * 2.2); scene.fog.far = Math.max(3800, cam.dist * 5);
    pathsM.visible = !!opts.showPaths;
    updateCars(opts); updateSignals(); updateSelection(sel);
    evRings.forEach((m, i) => {
      const ev = world.events[world.events.length - 1 - i];
      if (!ev || world.time - ev.t > 3) { m.visible = false; return; }
      const age = world.time - ev.t, s = 3 + age * 6; m.visible = true; m.position.set(ev.x, 0.4, ev.y); m.scale.setScalar(s); m.material.opacity = 1 - age / 3;
    });
    renderer.render(scene, camera);
  };

  root.View3D = V;
})(typeof window !== 'undefined' ? window : globalThis);
