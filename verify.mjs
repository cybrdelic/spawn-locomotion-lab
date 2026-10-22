import assert from "node:assert/strict";
import { writeFileSync, mkdirSync } from "node:fs";
import { Experiment, DT, legIK } from "./src/physics.js";
mkdirSync("evidence", { recursive: true });
const results = [];
function run(options, push = 0) {
  const sim = new Experiment(options);
  let maxAnchorError = 0,
    maxSpeed = 0;
  const samples = [];
  for (let i = 0; i < 2400; i++) {
    if (i === 480 && push) sim.push(push);
    sim.step();
    const c = sim.com();
    maxSpeed = Math.max(maxSpeed, Math.hypot(c.vx, c.vy));
    for (const q of sim.joints) {
      const local = [q.j.anchor1(), q.j.anchor2()],
        b = [q.a, q.b],
        p = local.map((a, k) => {
          const r = b[k].rotation(),
            t = b[k].translation();
          return {
            x: t.x + Math.cos(r) * a.x - Math.sin(r) * a.y,
            y: t.y + Math.sin(r) * a.x + Math.cos(r) * a.y,
          };
        });
      maxAnchorError = Math.max(
        maxAnchorError,
        Math.hypot(p[0].x - p[1].x, p[0].y - p[1].y),
      );
    }
    if (i % 120 === 0) samples.push(sim.snapshot());
  }
  const r = {
    ...options,
    push,
    ...sim.snapshot(),
    maxAnchorError,
    maxSpeed,
    samples,
  };
  sim.dispose();
  return r;
}
for (const motion of ["stand", "squat", "reach"])
  for (const length of [0.38, 0.44, 0.5]) {
    const r = run({ mode: "adaptive", motion, length, height: 0.16 }, 8);
    assert(!r.metrics.fall, `${motion}/${length} fell`);
    assert(r.metrics.maxLimitError < 0.08, "joint limit error");
    assert(r.maxAnchorError < 0.025, "dislocated joint");
    results.push(r);
  }
for (const mode of ["adaptive", "fixed", "passive"])
  results.push(run({ mode, motion: "squat", length: 0.44, height: 0.24 }, 8));
const raised = results.filter((r) => r.height === 0.24);
assert(!raised[0].metrics.fall);
assert(raised[1].metrics.fall);
assert(raised[2].metrics.fall);
const control = run({ mode: "adaptive", motion: "stand", height: 0 }, 0),
  disturbed = run({ mode: "adaptive", motion: "stand", height: 0 }, 8);
assert(
  Math.abs(control.samples[4].com.vx - disturbed.samples[4].com.vx) > 0.001,
  "push did not change the physical response",
);
for (const r of results)
  assert(
    Number.isFinite(r.com.x) && Number.isFinite(r.com.y),
    "nonfinite state",
  );
// Reach projection: no limb stretch, even for unreachable requested contacts.
for (const point of [
  { x: 0, y: -2 },
  { x: 2, y: 0 },
  { x: 0, y: -0.01 },
]) {
  const a = legIK({ x: 0, y: 0 }, point);
  assert(a.every(Number.isFinite));
  assert(a[1] <= 0, "knee bends backward");
}
writeFileSync(
  "evidence/behavior-results.json",
  JSON.stringify(
    { dt: DT, results, disturbance: { control, disturbed } },
    null,
    2,
  ),
);
console.log(
  JSON.stringify(
    {
      passed: true,
      cases: results.length + 2,
      durationPerCase: 20,
      adaptiveFalls: results.filter(
        (r) => r.mode === "adaptive" && r.metrics.fall,
      ).length,
      maxAdaptiveAnchorError: Math.max(
        ...results
          .filter((r) => r.mode === "adaptive")
          .map((r) => r.maxAnchorError),
      ),
      summary: results.map(({ mode, motion, length, height, metrics }) => ({
        mode,
        motion,
        length,
        height,
        fallen: metrics.fall,
        maxLimitError: metrics.maxLimitError,
      })),
    },
    null,
    2,
  ),
);
