// ===================================================================
// PROJECTOR ↔ GLASS ALIGNMENT CORE
// ===================================================================
// Forward optics model + pose solver shared by the interactive
// alignment tool (index.html) and the Node tests.
//
// COORDINATES:
//   Glass frame — origin at the block centre, axes along the block
//                 edges, millimetres.  The etched cloud lives here.
//   Projector   — Blender camera convention: looks down local -Z,
//                 +X right, +Y up.
//   Pixels      — continuous projector coords, v measured UP from the
//                 bottom row (same as the Blender scripts).  Integer
//                 pixel k covers [k, k+1), so its centre is k + 0.5.
//
// The model is identical to projector_alignment_fiducials.py:
//   pinhole (HFOV, principal point) → flat front face → Snell → point.
// pointToPixel() is the exact inverse (Fermat's principle on a plane).
//
// The solver estimates small corrections to the nominal rig — where
// the projector REALLY is relative to the etched cloud — from measured
// pixel ↔ fiducial correspondences, using damped least squares with
// weak priors so it behaves sensibly with only 1–3 measurements.
// ===================================================================
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.AlignmentCore = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // -----------------------------------------------------------------
  // VECTOR / MATRIX HELPERS (3x3 matrices are row-major arrays)
  // -----------------------------------------------------------------
  const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const norm = (a) => Math.sqrt(dot(a, a));
  const normalize = (a) => mul(a, 1 / norm(a));
  const matVec = (M, v) => [dot(M[0], v), dot(M[1], v), dot(M[2], v)];
  const matTVec = (M, v) => [
    M[0][0] * v[0] + M[1][0] * v[1] + M[2][0] * v[2],
    M[0][1] * v[0] + M[1][1] * v[1] + M[2][1] * v[2],
    M[0][2] * v[0] + M[1][2] * v[1] + M[2][2] * v[2],
  ];
  const matMul = (A, B) => A.map((row) =>
    [0, 1, 2].map((j) => row[0] * B[0][j] + row[1] * B[1][j] + row[2] * B[2][j]));
  const DEG = Math.PI / 180;

  function rotX(deg) {
    const c = Math.cos(deg * DEG), s = Math.sin(deg * DEG);
    return [[1, 0, 0], [0, c, -s], [0, s, c]];
  }
  function rotY(deg) {
    const c = Math.cos(deg * DEG), s = Math.sin(deg * DEG);
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]];
  }
  function rotZ(deg) {
    const c = Math.cos(deg * DEG), s = Math.sin(deg * DEG);
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]];
  }

  // -----------------------------------------------------------------
  // PARAMETERS
  // -----------------------------------------------------------------
  // Translations/rotations are in the projector's own frame so the
  // answer reads as "move the projector right", "tilt it up", etc.
  // prior = 1-sigma plausibility used to damp under-determined fits.
  const PARAMS = [
    { key: 'tx', label: 'Projector offset right', unit: 'mm', step: 1e-3, prior: 5, free: true },
    { key: 'ty', label: 'Projector offset up', unit: 'mm', step: 1e-3, prior: 5, free: true },
    { key: 'tz', label: 'Projector offset back (away from glass)', unit: 'mm', step: 1e-3, prior: 8, free: true },
    { key: 'rx', label: 'Pitch (+ = tilted up)', unit: 'deg', step: 1e-4, prior: 2, free: true },
    { key: 'ry', label: 'Yaw (+ = turned left)', unit: 'deg', step: 1e-4, prior: 2, free: true },
    { key: 'rz', label: 'Roll (+ = counter-clockwise, seen from behind projector)', unit: 'deg', step: 1e-4, prior: 2, free: true },
    { key: 'dhfov', label: 'HFOV error', unit: 'deg', step: 1e-4, prior: 0.5, free: false },
    { key: 'dcx', label: 'Principal point offset right', unit: 'px', step: 1e-3, prior: 20, free: false },
    { key: 'dcy', label: 'Principal point offset up', unit: 'px', step: 1e-3, prior: 20, free: false },
    { key: 'dior', label: 'Glass IOR error', unit: '', step: 1e-5, prior: 0.03, free: false },
    { key: 'scale', label: 'Cloud scale error (uniform)', unit: '', step: 1e-5, prior: 0.02, free: false },
    { key: 'dscale', label: 'Cloud depth-axis scale error', unit: '', step: 1e-5, prior: 0.02, free: false },
  ];
  const PARAM_KEYS = PARAMS.map((p) => p.key);
  const zeroParams = () => Object.fromEntries(PARAM_KEYS.map((k) => [k, 0]));

  // -----------------------------------------------------------------
  // MODEL
  // -----------------------------------------------------------------
  // rig (from fiducials JSON):
  //   res: [W, H], hfov_deg, cx, cy (continuous px), ior,
  //   projector: { position: [mm], rotation: 3x3 row-major, columns =
  //                projector local X/Y/Z axes in the glass frame },
  //   glass: { half_mm: [hx, hy, hz], safe_margin, front_axis, front_sign }
  function makeModel(rig, params) {
    const p = Object.assign(zeroParams(), params || {});
    const R0 = rig.projector.rotation;
    const R = matMul(R0, matMul(rotX(p.rx), matMul(rotY(p.ry), rotZ(p.rz))));
    const C = add(rig.projector.position, matVec(R0, [p.tx, p.ty, p.tz]));
    const hfov = rig.hfov_deg + p.dhfov;
    const W = rig.res[0];
    return {
      R, C,
      f: (W / 2) / Math.tan((hfov / 2) * DEG),
      cx: rig.cx + p.dcx,
      cy: rig.cy + p.dcy,
      ior: rig.ior + p.dior,
      scale: 1 + p.scale,
      dscale: 1 + p.dscale,
      half: rig.glass.half_mm,
      axis: rig.glass.front_axis,
      sign: rig.glass.front_sign,
      res: rig.res,
    };
  }

  function lateralAxes(a) {
    return [0, 1, 2].filter((i) => i !== a);
  }

  // Snell refraction; returns null on total internal reflection.
  function refract(I, N, n1, n2) {
    I = normalize(I); N = normalize(N);
    const eta = n1 / n2;
    const cosi = -Math.max(-1, Math.min(1, dot(I, N)));
    const k = 1 - eta * eta * (1 - cosi * cosi);
    if (k < 0) return null;
    return normalize(add(mul(I, eta), mul(N, eta * cosi - Math.sqrt(k))));
  }

  // Ray / AABB slab test; returns [tmin, tmax] or null.
  function intersectAabb(o, d, bmin, bmax) {
    let tmin = -1e9, tmax = 1e9;
    for (let i = 0; i < 3; i++) {
      if (Math.abs(d[i]) < 1e-12) {
        if (o[i] < bmin[i] || o[i] > bmax[i]) return null;
      } else {
        let t1 = (bmin[i] - o[i]) / d[i], t2 = (bmax[i] - o[i]) / d[i];
        if (t1 > t2) [t1, t2] = [t2, t1];
        tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
        if (tmin > tmax) return null;
      }
    }
    if (tmax < 0) return null;
    return [Math.max(tmin, 0), tmax];
  }

  // Projector pixel (continuous coords) → refracted ray inside glass.
  // Returns { entry, dir } in the glass frame or null if the ray does
  // not enter through the front face.
  function pixelToRay(m, u, v) {
    const dLocal = normalize([(u - m.cx) / m.f, (v - m.cy) / m.f, -1]);
    const d = matVec(m.R, dLocal);
    const a = m.axis, h = m.sign * m.half[a];
    if (Math.abs(d[a]) < 1e-12) return null;
    const t = (h - m.C[a]) / d[a];
    if (t <= 0) return null;
    const E = add(m.C, mul(d, t));
    for (const b of lateralAxes(a)) if (Math.abs(E[b]) > m.half[b]) return null;
    const N = [0, 0, 0]; N[a] = m.sign;
    if (dot(N, d) > -0.1) return null; // grazing, same rule as the Blender scripts
    const dir = refract(d, N, 1.0, m.ior);
    if (!dir) return null;
    return { entry: E, dir };
  }

  // Projector pixel → point at depthFrac (0 = front, 1 = back) of the
  // refracted ray's segment through the safe zone.  Mirrors the
  // Blender generator; used for verification and simulations.
  function pixelToPoint(m, u, v, depthFrac, safeMargin) {
    const ray = pixelToRay(m, u, v);
    if (!ray) return null;
    const hi = m.half.map((x) => x * safeMargin);
    const lo = hi.map((x) => -x);
    const seg = intersectAabb(ray.entry, ray.dir, lo, hi);
    if (!seg || seg[1] <= seg[0]) return null;
    return add(ray.entry, mul(ray.dir, seg[0] + (seg[1] - seg[0]) * depthFrac));
  }

  // Glass-frame point → projector pixel (continuous coords) that lights
  // it.  Finds the entry point on the front face satisfying Snell's law
  // (1-D root find in the plane of incidence), then projects it.
  function pointToPixel(m, P) {
    const a = m.axis, sgn = m.sign;
    const Q = mul(P, m.scale);
    Q[a] *= m.dscale;
    const h = sgn * m.half[a];
    const hc = sgn * (m.C[a] - h);   // projector height above the face
    const hp = sgn * (h - Q[a]);     // point depth below the face
    if (hc <= 0 || hp < 0) return null;
    const [b, c] = lateralAxes(a);
    const Db = Q[b] - m.C[b], Dc = Q[c] - m.C[c];
    const D = Math.hypot(Db, Dc);
    const n = m.ior;
    let x = 0;
    if (D > 1e-12 && hp > 0) {
      // g(x) = sin(theta_air) - n sin(theta_glass); monotone in x.
      const g = (t) => t / Math.hypot(t, hc) - n * (D - t) / Math.hypot(D - t, hp);
      const dg = (t) => hc * hc / Math.pow(t * t + hc * hc, 1.5) +
        n * hp * hp / Math.pow((D - t) * (D - t) + hp * hp, 1.5);
      let lo = 0, hi = D;
      x = n * hc * D / (hp + n * hc); // paraxial guess
      x = Math.min(Math.max(x, 0), D);
      for (let i = 0; i < 60; i++) {
        const gx = g(x);
        if (Math.abs(gx) < 1e-14) break;
        if (gx > 0) hi = x; else lo = x;
        let xn = x - gx / dg(x);
        if (!(xn > lo && xn < hi)) xn = 0.5 * (lo + hi);
        if (Math.abs(xn - x) < 1e-12) { x = xn; break; }
        x = xn;
      }
    } else if (hp === 0) {
      x = D;
    }
    const E = [0, 0, 0];
    E[a] = h;
    E[b] = m.C[b] + (D > 1e-12 ? Db / D * x : 0);
    E[c] = m.C[c] + (D > 1e-12 ? Dc / D * x : 0);
    const q = matTVec(m.R, sub(E, m.C));
    if (q[2] >= 0) return null;
    return [m.cx + m.f * q[0] / -q[2], m.cy + m.f * q[1] / -q[2]];
  }

  // -----------------------------------------------------------------
  // LINEAR ALGEBRA
  // -----------------------------------------------------------------
  function solveLinear(A, b) {
    const n = b.length;
    const M = A.map((row, i) => row.concat([b[i]]));
    for (let c = 0; c < n; c++) {
      let piv = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
      if (Math.abs(M[piv][c]) < 1e-300) return null;
      [M[c], M[piv]] = [M[piv], M[c]];
      for (let r = 0; r < n; r++) {
        if (r === c) continue;
        const f = M[r][c] / M[c][c];
        if (f !== 0) for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
      }
    }
    return M.map((row, i) => row[n] / row[i]);
  }

  function invert(A) {
    const n = A.length;
    const cols = [];
    for (let j = 0; j < n; j++) {
      const e = new Array(n).fill(0); e[j] = 1;
      const col = solveLinear(A, e);
      if (!col) return null;
      cols.push(col);
    }
    return A.map((_, i) => cols.map((col) => col[i]));
  }

  // -----------------------------------------------------------------
  // SOLVER
  // -----------------------------------------------------------------
  // observations: [{ point: [mm], uv: [u, v] (continuous px) }]
  // opts.free:    parameter keys to estimate (default: the 6 pose DOF)
  // opts.sigmaPx: expected reading noise (px), default 0.5
  // opts.priors:  false to disable the weak priors
  function solve(rig, observations, opts) {
    opts = opts || {};
    const free = opts.free || PARAMS.filter((p) => p.free).map((p) => p.key);
    const defs = free.map((k) => PARAMS.find((p) => p.key === k));
    const sigmaPx = opts.sigmaPx || 0.5;
    const usePriors = opts.priors !== false;
    const base = Object.assign(zeroParams(), opts.init || {});

    const toParams = (x) => {
      const p = Object.assign({}, base);
      free.forEach((k, i) => { p[k] = x[i]; });
      return p;
    };

    const residuals = (x) => {
      const m = makeModel(rig, toParams(x));
      const r = [];
      for (const ob of observations) {
        const uv = pointToPixel(m, ob.point);
        if (!uv) { r.push(1e3, 1e3); continue; }
        r.push((uv[0] - ob.uv[0]) / sigmaPx, (uv[1] - ob.uv[1]) / sigmaPx);
      }
      if (usePriors) defs.forEach((d, i) => r.push(x[i] / d.prior));
      return r;
    };
    const cost = (r) => r.reduce((s, v) => s + v * v, 0);

    const jacobian = (x, r0) => {
      const J = r0.map(() => new Array(x.length).fill(0));
      for (let j = 0; j < x.length; j++) {
        const h = defs[j].step;
        const xp = x.slice(); xp[j] += h;
        const xm = x.slice(); xm[j] -= h;
        const rp = residuals(xp), rm = residuals(xm);
        for (let i = 0; i < r0.length; i++) J[i][j] = (rp[i] - rm[i]) / (2 * h);
      }
      return J;
    };

    let x = free.map((k) => base[k]);
    let r = residuals(x);
    let c = cost(r);
    let lambda = 1e-3;
    let iterations = 0;
    let JTJ = null;
    for (; iterations < 100 && x.length; iterations++) {
      const J = jacobian(x, r);
      const n = x.length;
      JTJ = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) =>
        J.reduce((s, row) => s + row[i] * row[j], 0)));
      const JTr = Array.from({ length: n }, (_, i) => J.reduce((s, row, k) => s + row[i] * r[k], 0));
      let improved = false;
      for (let tries = 0; tries < 10; tries++) {
        const A = JTJ.map((row, i) => row.map((v, j) => v + (i === j ? lambda * (v + 1e-9) : 0)));
        const step = solveLinear(A, JTr.map((v) => -v));
        if (!step) { lambda *= 10; continue; }
        const xn = x.map((v, i) => v + step[i]);
        const rn = residuals(xn);
        const cn = cost(rn);
        if (cn < c) {
          const rel = (c - cn) / Math.max(c, 1e-12);
          x = xn; r = rn; c = cn;
          lambda = Math.max(lambda / 10, 1e-9);
          improved = true;
          if (rel < 1e-10) iterations = 1e9;
          break;
        }
        lambda *= 10;
      }
      if (!improved) break;
    }

    // Report
    const params = toParams(x);
    const m = makeModel(rig, params);
    const perObs = observations.map((ob) => {
      const uv = pointToPixel(m, ob.point);
      return uv ? [uv[0] - ob.uv[0], uv[1] - ob.uv[1]] : [NaN, NaN];
    });
    const sq = perObs.reduce((s, d) => s + d[0] * d[0] + d[1] * d[1], 0);
    const rms = observations.length ? Math.sqrt(sq / (2 * observations.length)) : 0;
    const dof = 2 * observations.length - free.length;
    let sigma = {};
    if (JTJ && x.length) {
      const J = jacobian(x, r);
      const n = x.length;
      const H = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) =>
        J.reduce((s, row) => s + row[i] * row[j], 0)));
      const cov = invert(H);
      // Inflate by the observed scatter when readings are noisier than assumed
      const infl = dof > 0 ? Math.max(1, (rms / sigmaPx)) : 1;
      if (cov) free.forEach((k, i) => { sigma[k] = Math.sqrt(Math.max(cov[i][i], 0)) * infl; });
    }
    return { params, sigma, free, rms, dof, residuals: perObs, model: m, iterations };
  }

  // -----------------------------------------------------------------
  // REPORTING
  // -----------------------------------------------------------------
  function solvedRig(rig, params) {
    const m = makeModel(rig, params);
    return {
      position_mm: m.C,
      rotation: m.R,
      hfov_deg: rig.hfov_deg + (params.dhfov || 0),
      cx: m.cx, cy: m.cy,
      ior: m.ior,
      cloud_scale: m.scale,
      cloud_depth_scale: m.dscale,
    };
  }

  // Plain-language diagnosis of a solve() result.
  function describe(rig, result) {
    const p = result.params, s = result.sigma;
    const lines = [];
    const fmt = (k, digits) => {
      const v = p[k], e = s[k];
      return v.toFixed(digits) + (e !== undefined ? ' ± ' + e.toFixed(digits) : '');
    };
    const sig = (k, thresh) => Math.abs(p[k]) > Math.max(thresh, 2 * (s[k] || 0));

    lines.push(`Fit RMS ${result.rms.toFixed(2)} px over ${result.residuals.length} fiducials` +
      (result.dof > 0 ? '' : ' (under-determined: priors are doing part of the work, add more fiducials)'));

    const moves = [];
    if (sig('tx', 0.1)) moves.push(`${Math.abs(p.tx).toFixed(2)} mm ${p.tx > 0 ? 'LEFT' : 'RIGHT'}`);
    if (sig('ty', 0.1)) moves.push(`${Math.abs(p.ty).toFixed(2)} mm ${p.ty > 0 ? 'DOWN' : 'UP'}`);
    if (sig('tz', 0.2)) moves.push(`${Math.abs(p.tz).toFixed(2)} mm ${p.tz > 0 ? 'CLOSER to the glass' : 'AWAY from the glass'}`);
    if (sig('rx', 0.02)) moves.push(`tilt ${Math.abs(p.rx).toFixed(3)}° ${p.rx > 0 ? 'DOWN' : 'UP'}`);
    if (sig('ry', 0.02)) moves.push(`turn ${Math.abs(p.ry).toFixed(3)}° ${p.ry > 0 ? 'RIGHT' : 'LEFT'}`);
    if (sig('rz', 0.02)) moves.push(`roll ${Math.abs(p.rz).toFixed(3)}° ${p.rz > 0 ? 'clockwise' : 'counter-clockwise'} (seen from behind)`);

    lines.push('Relative to the etched cloud, the projector is:');
    lines.push(`  ${fmt('tx', 2)} mm right, ${fmt('ty', 2)} mm up, ${fmt('tz', 2)} mm further back`);
    lines.push(`  pitch ${fmt('rx', 3)}°, yaw ${fmt('ry', 3)}°, roll ${fmt('rz', 3)}°`);
    for (const k of ['dhfov', 'dcx', 'dcy', 'dior', 'scale', 'dscale']) {
      if (result.free.includes(k)) {
        const d = PARAMS.find((q) => q.key === k);
        const digits = d.unit === 'px' ? 2 : d.unit === 'deg' ? 3 : 4;
        lines.push(`  ${d.label}: ${fmt(k, digits)} ${d.unit}`);
      }
    }
    if (result.free.includes('dior')) lines.push(`  → fitted IOR = ${(rig.ior + p.dior).toFixed(4)}`);
    if (result.free.includes('dhfov')) lines.push(`  → fitted HFOV = ${(rig.hfov_deg + p.dhfov).toFixed(3)}°`);
    lines.push(moves.length
      ? 'To fix physically, move the projector: ' + moves.join(', ') + '.'
      : 'Projector pose matches the design within measurement precision.');
    lines.push('Or fix in software: export the result and run apply_projector_alignment.py in Blender, then regenerate projector images.');

    // Worst fiducial — a big outlier usually means a misidentified point
    let worst = -1, worstErr = 0;
    result.residuals.forEach((d, i) => {
      const e = Math.hypot(d[0], d[1]);
      if (e > worstErr) { worstErr = e; worst = i; }
    });
    if (worst >= 0 && worstErr > Math.max(2, 3 * result.rms)) {
      lines.push(`Warning: reading #${worst} is off by ${worstErr.toFixed(1)} px — re-check it (wrong point lit?).`);
    }
    return lines;
  }

  return {
    PARAMS, PARAM_KEYS, zeroParams,
    makeModel, pixelToRay, pixelToPoint, pointToPixel, refract, intersectAabb,
    solve, solvedRig, describe,
    _vec: { add, sub, mul, dot, norm, normalize, matVec, matTVec, matMul, rotX, rotY, rotZ },
  };
});
