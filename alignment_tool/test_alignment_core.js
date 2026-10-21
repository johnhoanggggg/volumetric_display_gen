// Self-check for alignment_core.js:  node alignment_tool/test_alignment_core.js
// Builds a synthetic rig (50x50x80 mm block, projector 110 mm from the
// +Z face), simulates a misaligned projector with integer-pixel readings
// and checks the solver recovers it.
const A = require('./alignment_core.js');

const rig = {
  res: [1280, 720], hfov_deg: 37.6, cx: 640, cy: 360, ior: 1.517,
  projector: { position: [0, 0, 150], rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]] },
  glass: { half_mm: [25, 25, 40], safe_margin: 0.9, front_axis: 2, front_sign: 1 },
};
const m0 = A.makeModel(rig, {});
const fiducials = [];
for (let j = 0; j < 3; j++) {
  for (let i = 0; i < 5; i++) {
    const U = 304 + i * 166, V = 40 + j * 316, depth = (i + j) % 2 ? 0.9 : 0.1;
    fiducials.push({ pixel: [U, V], point: A.pixelToPoint(m0, U + 0.5, V + 0.5, depth, 0.9) });
  }
}

let failures = 0;
const check = (ok, msg) => { console.log((ok ? 'PASS ' : 'FAIL ') + msg); if (!ok) failures++; };

const roundTrip = Math.max(...fiducials.map((f) => {
  const uv = A.pointToPixel(m0, f.point);
  return Math.max(Math.abs(uv[0] - f.pixel[0] - 0.5), Math.abs(uv[1] - f.pixel[1] - 0.5));
}));
check(roundTrip < 1e-6, `pixel → point → pixel round trip (${roundTrip.toExponential(1)} px)`);

const truth = { tx: 0.9, ty: -0.6, tz: 2.5, rx: 0.15, ry: -0.25, rz: 0.3 };
const mt = A.makeModel(rig, truth);
const obs = fiducials.map((f) => {
  const uv = A.pointToPixel(mt, f.point);
  return { point: f.point, uv: [Math.floor(uv[0]) + 0.5, Math.floor(uv[1]) + 0.5] };
});
const raw = Math.max(...obs.map((o, i) => Math.hypot(o.uv[0] - fiducials[i].pixel[0], o.uv[1] - fiducials[i].pixel[1])));
const res = A.solve(rig, obs);
for (const k of Object.keys(truth)) {
  const tol = k.startsWith('t') ? 0.3 : 0.1;
  check(Math.abs(res.params[k] - truth[k]) < tol, `${k}: true ${truth[k]}, solved ${res.params[k].toFixed(3)} ± ${res.sigma[k].toFixed(3)}`);
}
let worst = 0;
for (let k = 0; k < 500; k++) {
  const P = [(Math.random() * 2 - 1) * 22, (Math.random() * 2 - 1) * 22, (Math.random() * 2 - 1) * 36];
  const a = A.pointToPixel(mt, P), b = A.pointToPixel(res.model, P);
  if (a && b) worst = Math.max(worst, Math.hypot(a[0] - b[0], a[1] - b[1]));
}
check(worst < 1, `raw misalignment ${raw.toFixed(1)} px → worst error after solve ${worst.toFixed(2)} px`);
console.log(A.describe(rig, res).join('\n'));
process.exit(failures ? 1 : 0);
