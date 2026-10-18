(() => {
  'use strict';

  // ---------- Settings you can tweak ----------
  // Water
  const WATER_FRACTION = 0.4;   // water fills 40% of the screen
  const CELLS = 80;             // how many columns of water are simulated
  const GRAVITY = 1400;         // pixels/s^2 at 1 g. Higher = faster, snappier water
  const DRAG = 1.2;             // water friction per second. Lower = sloshes longer
  const VISCOSITY = 800;        // smooths out tiny ripples
  const MAX_SLOPE_CAP = 1.3;    // steepest water surface allowed
  const MAX_SUBSTEPS = 80;

  // Boat
  const BOAT_SCALE = 0.1;       // 1 = the original boat size (0.9 would be 10% smaller)
  const BOAT_BODY_MIN = 0.18;   // smallest physics size (px per unit), keeps a tiny boat stable
  const BUOYANCY_MAX = 4;     // upward push when fully underwater (1 = just enough to float)
  const WATER_DRAG_Y = 26;      // water resistance up/down. Higher = floats up more slowly
  const WATER_DRAG_X = 8;       // how strongly flowing water carries the boat sideways
  const ROLL_DRAG = 20;         // water resistance to rocking
  const AIR_DRAG = 0.15;        // air resistance while flying
  const AIR_ROLL_DRAG = 0.3;    // air resistance to spinning while flying
  const BOAT_DRAFT = 6;         // how deep the hull sits (smaller = harder to sink, bouncier)
  const SLAM = 0.06;            // kick when a wave rises into the hull. Higher = bigger jumps
  const LEAVE_DRAG = 0.2;       // water grip on a boat popping out of the surface (lower = lets go more easily)
  const BOAT_SUBSTEPS = 10;
  const G_EARTH = 9.81;

  const COLORS = {
    backWater: '#b7dcf0',
    frontWater: 'rgba(142, 202, 230, 0.92)',
    hull: '#1f3a5f',
    mast: '#1f3a5f',
    mainSail: '#f4845f',
    jib: '#ffd6a5',
  };

  // ---------- Elements ----------
  const canvas = document.getElementById('scene');
  const ctx = canvas.getContext('2d');
  const overlay = document.getElementById('overlay');
  const startBtn = document.getElementById('start');
  const hint = document.getElementById('hint');

  // ---------- State ----------
  let W = 0, H = 0, dpr = 1, dx = 1, avgDepth = 0;

  // Water: depth h per cell, horizontal flow u on the faces between cells.
  const h = new Float32Array(CELLS);
  const u = new Float32Array(CELLS + 1);
  const F = new Float32Array(CELLS + 1);
  const prevSurf = new Float32Array(CELLS);
  const surf = new Float32Array(CELLS);      // surface height with ripples
  const surfVel = new Float32Array(CELLS);   // how fast the surface moves up/down

  // Gravity as felt inside the screen plane, in m/s^2.
  // x points to the screen right, y points to the screen bottom.
  const sensorG = { x: 0, y: G_EARTH };
  const gSmooth = { x: 0, y: G_EARTH };
  let sensorActive = false;
  let gravitySign = 0;
  let manualTheta = 0;
  let hintTimer = 0;

  // The boat is a free body: position, velocity, angle and spin
  const boat = { x: 0, y: 0, vx: 0, vy: 0, angle: 0, omega: 0, ready: false };

  const clamp = (n, lo, hi) => Math.min(hi, Math.max(lo, n));
  const isIOS = /iP(hone|ad|od)/.test(navigator.userAgent) ||
    (navigator.platform === 'MacIntel' && navigator.maxTouchPoints > 1);

  // ---------- Layout ----------
  function resize() {
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    W = window.innerWidth;
    H = window.innerHeight;
    canvas.width = Math.round(W * dpr);
    canvas.height = Math.round(H * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    dx = W / CELLS;
    avgDepth = H * WATER_FRACTION;
    h.fill(avgDepth);
    u.fill(0);
    boat.x = W / 2;
    boat.ready = false;
  }

  const boatScale = () => (W * 0.3 * BOAT_SCALE) / 100;

  // ---------- Water physics (shallow water equations) ----------
  // Each step: gravity and pressure accelerate the flow, then the flow moves
  // water between neighbouring columns. Water is only ever moved, never created,
  // so the amount stays constant and it truly piles up on the low side.
  function step(dt, gx, gy, uLimit) {
    // 1. Flow between columns (u[0] and u[CELLS] are the walls and stay 0)
    for (let i = 1; i < CELLS; i++) {
      if (Math.max(h[i - 1], h[i]) < 0.5) { u[i] = 0; continue; }
      const pressure = -gy * (h[i] - h[i - 1]) / dx;
      const viscous = VISCOSITY * (u[i - 1] - 2 * u[i] + u[i + 1]) / (dx * dx);
      const next = (u[i] + dt * (gx + pressure + viscous)) / (1 + DRAG * dt);
      u[i] = clamp(next, -uLimit, uLimit);
    }

    // 2. Move water, taking it from the upstream column
    for (let i = 1; i < CELLS; i++) {
      F[i] = u[i] * (u[i] > 0 ? h[i - 1] : h[i]);
    }
    for (let i = 0; i < CELLS; i++) {
      h[i] = Math.max(0, h[i] - (dt * (F[i + 1] - F[i])) / dx);
    }
  }

  // Cancels tiny numerical drift so the water amount stays exactly 40%
  function conserveVolume() {
    let total = 0;
    for (let i = 0; i < CELLS; i++) total += h[i];
    if (total <= 0) return;
    const k = (avgDepth * CELLS) / total;
    for (let i = 0; i < CELLS; i++) h[i] *= k;
  }

  // Surface height of a column, with gentle ripples that fade out where it is dry
  function waveAt(i, t, phase, amp, offset) {
    const wet = clamp(h[i] / 8, 0, 1);
    return H - h[i] + (Math.sin(t * 1.6 + i * 0.35 + phase) * amp + offset) * wet;
  }

  function sampleCells(arr, x) {
    const f = clamp(x / dx - 0.5, 0, CELLS - 1);
    const i0 = Math.floor(f);
    const i1 = Math.min(CELLS - 1, i0 + 1);
    const k = f - i0;
    return arr[i0] * (1 - k) + arr[i1] * k;
  }

  // The boat steps faster than the screen refreshes, so blend between the previous
  // and current surface instead of letting it jump once per frame
  const sampleSurface = (x, blend) =>
    sampleCells(prevSurf, x) * (1 - blend) + sampleCells(surf, x) * blend;
  const sampleSurfaceVel = (x) => sampleCells(surfVel, x);

  function sampleFlow(x) {
    const f = clamp(x / dx, 0, CELLS);
    const i0 = Math.floor(f);
    const i1 = Math.min(CELLS, i0 + 1);
    const k = f - i0;
    return u[i0] * (1 - k) + u[i1] * k;
  }

  // ---------- Boat physics ----------
  // Forces on the boat:
  //  - gravity pulls it down (and sideways when the phone is tilted)
  //  - buoyancy pushes it away from gravity, in proportion to how deep it sits
  //    (measured at five points along the hull, so uneven waves also rock it)
  //  - water drag, relative to the moving water, so flowing water carries it
  //    and a submerged boat rises slowly instead of popping up
  //  - a wave rising into the hull kicks it upward (SLAM), which launches it
  //  - almost no drag in the air, so it can fly off a wave and fall back
  function initBoat() {
    boat.y = sampleSurface(boat.x, 1);
    boat.vx = 0;
    boat.vy = 0;
    boat.angle = 0;
    boat.omega = 0;
    boat.ready = true;
  }

  function updateBoat(dt, gx, gy) {
    const s = boatScale();
    const ps = Math.max(s, BOAT_BODY_MIN);   // physics scale
    const draft = BOAT_DRAFT * ps;                   // how deep the hull sits at rest
    const reach = 40 * ps;                   // half-width of the hull's sample points
    const inertia = 1.69 * reach * reach;
    const comDrop = 6 * ps;                  // centre of mass hangs below the hull
    const offsets = [-1, -0.5, 0, 0.5, 1];
    const count = offsets.length;

    const gMag = Math.hypot(gx, gy);
    const nx = gx / gMag;
    const ny = gy / gMag;                    // "down" direction (screen gravity)
    const sub = dt / BOAT_SUBSTEPS;
    const sideLimit = 50 * s + 2;

    for (let n = 0; n < BOAT_SUBSTEPS; n++) {
      const blend = (n + 1) / BOAT_SUBSTEPS;
      const cosA = Math.cos(boat.angle);
      const sinA = Math.sin(boat.angle);

      let ax = gx;
      let ay = gy;
      // A low centre of mass gently keeps the boat from capsizing
      let torque = -comDrop * sinA * gy - comDrop * cosA * gx;
      let wet = 0;

      for (let k = 0; k < count; k++) {
        const lx = offsets[k] * reach;
        const rx = lx * cosA - draft * sinA;
        const ry = lx * sinA + draft * cosA;
        const px = clamp(boat.x + rx, 0, W);
        const depth = (boat.y + ry - sampleSurface(px, blend)) * ny;
        const push = (gMag * clamp(depth / draft, 0, BUOYANCY_MAX)) / count;
        const fx = -nx * push;
        const fy = -ny * push;
        ax += fx;
        ay += fy;
        torque += rx * fy - ry * fx;
        wet += clamp(depth / (2 * draft), 0, 1) / count;
      }

      const flowX = sampleFlow(boat.x);
      const flowY = sampleSurfaceVel(boat.x);

      // A wave rising into the hull kicks the boat upward. The kick is strongest
      // when the boat is half in the water, and fades when it is fully under or airborne.
      const risingWater = boat.vy - flowY;
      if (risingWater > 0) {
        const zone = clamp(Math.min(2 * wet, 2 * (1 - wet)), 0, 1);
        ay -= Math.min(SLAM * zone * risingWater * risingWater, 6 * gMag);
      }
      // Water grips a boat that is sinking or deep, but lets go of one that is
      // popping out near the surface
      let dragY = WATER_DRAG_Y * wet;
      if (boat.vy < flowY) {
        const deep = clamp((wet - 0.5) * 2, 0, 1);
        dragY *= LEAVE_DRAG + (1 - LEAVE_DRAG) * deep;
      }
      ax -= WATER_DRAG_X * wet * (boat.vx - flowX) + AIR_DRAG * boat.vx;
      ay -= dragY * (boat.vy - flowY) + AIR_DRAG * boat.vy;
      const alpha = torque / inertia - (ROLL_DRAG * wet + AIR_ROLL_DRAG) * boat.omega;

      boat.vx += ax * sub;
      boat.vy += ay * sub;
      boat.omega += alpha * sub;
      boat.x += boat.vx * sub;
      boat.y += boat.vy * sub;
      boat.angle += boat.omega * sub;

      // Side walls: soft bounce
      if (boat.x < sideLimit) {
        boat.x = sideLimit;
        boat.vx = Math.abs(boat.vx) * 0.3;
      } else if (boat.x > W - sideLimit) {
        boat.x = W - sideLimit;
        boat.vx = -Math.abs(boat.vx) * 0.3;
      }

      // Bottom of the screen: if the water runs dry, the boat rests on the bed
      if (boat.y > H - draft) {
        boat.y = H - draft;
        if (boat.vy > 0) boat.vy *= -0.2;
        boat.vx *= 0.9;
        boat.omega *= 0.85;
      }
    }
  }

  // ---------- Drawing ----------
  function drawWater(color, offset, phase, amp, t) {
    const pts = [];
    for (let i = 0; i < CELLS; i++) {
      pts.push([(i + 0.5) * dx, waveAt(i, t, phase, amp, offset)]);
    }
    ctx.beginPath();
    ctx.moveTo(0, H + 2);
    ctx.lineTo(0, pts[0][1]);
    ctx.lineTo(pts[0][0], pts[0][1]);
    for (let i = 1; i < pts.length - 1; i++) {
      const mx = (pts[i][0] + pts[i + 1][0]) / 2;
      const my = (pts[i][1] + pts[i + 1][1]) / 2;
      ctx.quadraticCurveTo(pts[i][0], pts[i][1], mx, my);
    }
    const last = pts[pts.length - 1];
    ctx.lineTo(last[0], last[1]);
    ctx.lineTo(W, last[1]);
    ctx.lineTo(W, H + 2);
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();
  }

  function drawBoat(cx, cy, angle, s) {
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(angle);
    ctx.scale(s, s);
    ctx.lineJoin = 'round';

    // main sail
    ctx.beginPath();
    ctx.moveTo(4, -86);
    ctx.lineTo(4, -14);
    ctx.lineTo(42, -14);
    ctx.closePath();
    ctx.fillStyle = COLORS.mainSail;
    ctx.fill();

    // jib
    ctx.beginPath();
    ctx.moveTo(-4, -70);
    ctx.lineTo(-4, -14);
    ctx.lineTo(-34, -14);
    ctx.closePath();
    ctx.fillStyle = COLORS.jib;
    ctx.fill();

    // mast
    ctx.fillStyle = COLORS.mast;
    ctx.fillRect(-2, -92, 4, 84);

    // flag
    ctx.beginPath();
    ctx.moveTo(2, -92);
    ctx.lineTo(14, -88);
    ctx.lineTo(2, -84);
    ctx.closePath();
    ctx.fillStyle = COLORS.mainSail;
    ctx.fill();

    // hull
    ctx.beginPath();
    ctx.moveTo(-50, -10);
    ctx.lineTo(50, -10);
    ctx.lineTo(34, 10);
    ctx.lineTo(-34, 10);
    ctx.closePath();
    ctx.fillStyle = COLORS.hull;
    ctx.strokeStyle = COLORS.hull;
    ctx.lineWidth = 3;
    ctx.fill();
    ctx.stroke();

    ctx.restore();
  }

  function draw(t) {
    ctx.clearRect(0, 0, W, H);
    drawWater(COLORS.backWater, -6, 1.7, 2, t);
    drawBoat(boat.x, boat.y, boat.angle, boatScale());
    drawWater(COLORS.frontWater, 0, 0, 1.5, t);
  }

  // ---------- Main loop ----------
  let last = performance.now();

  function frame(now) {
    const dt = Math.min(0.033, (now - last) / 1000);
    last = now;

    // Gravity from the phone, or from dragging when there is no sensor
    const tx = sensorActive ? sensorG.x : G_EARTH * Math.sin(manualTheta);
    const ty = sensorActive ? sensorG.y : G_EARTH * Math.cos(manualTheta);
    gSmooth.x += (tx - gSmooth.x) * 0.35;
    gSmooth.y += (ty - gSmooth.y) * 0.35;

    const k = GRAVITY / G_EARTH;
    const gy = clamp(gSmooth.y, 3, 1.5 * G_EARTH) * k;
    const maxSlope = Math.min(MAX_SLOPE_CAP, (0.85 * avgDepth) / (W / 2));
    const gx = clamp(gSmooth.x * k, -maxSlope * gy, maxSlope * gy);

    // Water: small time steps keep it stable
    let hMax = avgDepth;
    for (let i = 0; i < CELLS; i++) if (h[i] > hMax) hMax = h[i];
    const cMax = Math.sqrt(GRAVITY * 1.5 * hMax);
    const dtMax = (0.4 * dx) / (2 * cMax);
    const n = Math.min(MAX_SUBSTEPS, Math.ceil(dt / dtMax));
    for (let s = 0; s < n; s++) step(dt / n, gx, gy, cMax);
    conserveVolume();

    // Surface height and vertical speed, which the boat reacts to
    const t = now / 1000;
    for (let i = 0; i < CELLS; i++) {
      const next = waveAt(i, t, 0, 1.5, 0);
      const previous = boat.ready ? surf[i] : next;
      prevSurf[i] = previous;
      surf[i] = next;
      surfVel[i] = clamp((next - previous) / Math.max(dt, 0.001), -1500, 1500);
    }

    if (!boat.ready) initBoat();
    updateBoat(dt, gx, gy);
    draw(t);

    requestAnimationFrame(frame);
  }

  // ---------- Tilt sensor ----------
  function onMotion(e) {
    const g = e.accelerationIncludingGravity;
    if (!g || g.x == null || g.y == null) return;
    sensorActive = true;

    // Android and iOS report gravity with opposite signs. Holding the phone
    // upright makes y strongly positive on the "standard" convention, so the
    // first clear reading tells us which one we have.
    if (gravitySign === 0) {
      if (Math.abs(g.y) > 3) gravitySign = Math.sign(g.y);
      else gravitySign = isIOS ? -1 : 1;
    }

    // Gravity inside the screen plane. Quick shakes show up here too,
    // so shaking the phone sloshes the water.
    sensorG.x = -g.x * gravitySign;
    sensorG.y = g.y * gravitySign;
  }

  function showHint(message) {
    hint.textContent = message;
    hint.classList.add('show');
    clearTimeout(hintTimer);
    hintTimer = setTimeout(() => hint.classList.remove('show'), 4500);
  }

  async function start() {
    overlay.classList.add('hidden');
    try {
      // iPhones require a tap and a permission prompt for motion data.
      if (
        typeof DeviceMotionEvent !== 'undefined' &&
        typeof DeviceMotionEvent.requestPermission === 'function'
      ) {
        const result = await DeviceMotionEvent.requestPermission();
        if (result !== 'granted') {
          showHint('Motion access was denied. Drag left or right to tilt instead.');
          return;
        }
      }
      window.addEventListener('devicemotion', onMotion);
      setTimeout(() => {
        showHint(
          sensorActive
            ? 'Tilt your phone'
            : 'No tilt sensor found. Drag left or right to tilt.'
        );
      }, 1200);
    } catch (err) {
      showHint('Could not read the tilt sensor. Drag left or right to tilt.');
    }
  }

  // ---------- Drag fallback (desktop or no sensor) ----------
  let dragging = false;
  function dragTo(clientX) {
    manualTheta = clamp(((clientX - W / 2) / (W / 2)) * 0.8, -0.8, 0.8);
  }
  window.addEventListener('pointerdown', (e) => {
    dragging = true;
    dragTo(e.clientX);
  });
  window.addEventListener('pointermove', (e) => {
    if (dragging) dragTo(e.clientX);
  });
  const endDrag = () => {
    dragging = false;
    manualTheta = 0;
  };
  window.addEventListener('pointerup', endDrag);
  window.addEventListener('pointercancel', endDrag);

  // ---------- Go ----------
  startBtn.addEventListener('click', start);
  window.addEventListener('resize', resize);
  resize();
  requestAnimationFrame(frame);
})();
