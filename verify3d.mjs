import assert from "node:assert/strict";
import { writeFileSync, mkdirSync } from "node:fs";
import RAPIER from "@dimforge/rapier3d-compat";
import { Quaternion, Euler, Vector3 } from "three";
import { Experiment3D } from "./src/physics3d.js";
mkdirSync("evidence", { recursive: true });
const results = [];
function run(options, { duration = 30, impulse = 24, push = true } = {}) {
  const s = new Experiment3D(options),
    samples = [];
  let hardError = 0;
  for (let i = 0; i < duration * 120; i++) {
    if (push && i === 600) s.push("side", impulse);
    if (push && i === 1800) s.push("forward", impulse);
    s.step();
    for (const j of s.joints) {
      const q = new Quaternion()
        .copy(j.a.rotation())
        .invert()
        .multiply(new Quaternion().copy(j.b.rotation()));
      if (q.w < 0) q.set(-q.x, -q.y, -q.z, -q.w);
      const a =
        j.kind === "hinge"
          ? [2 * Math.atan2(q.z, q.w)]
          : [q.x, q.y, q.z].map(
              (v) => 2 * Math.asin(Math.max(-1, Math.min(1, v))),
            );
      a.forEach((v, k) => {
        const range = j.kind === "hinge" ? j.limits : j.limits[k];
        hardError = Math.max(hardError, range[0] - v, v - range[1], 0);
      });
    }
    if (i % 120 === 0 || i === 601 || i === 1801) samples.push(s.snapshot());
  }
  const result = {
    options,
    impulse: push ? impulse : 0,
    hardError,
    ...s.snapshot(),
    feet: s.feet.map((f) => ({ ...f.translation() })),
    samples,
  };
  s.dispose();
  return result;
}
for (const motion of ["stand", "squat", "reach"])
  for (const height of [0, 0.12, 0.24])
    for (const length of [0.38, 0.44, 0.5]) {
      const r = run({ motion, height, length });
      assert(!r.metrics.fallen, `fell: ${JSON.stringify(r.options)}`);
      assert(r.metrics.maxAnchorError < 0.01, "joint separation");
      assert(r.hardError < 0.015, "native angular limit violation");
      results.push(r);
    }
const quiet = run({ motion: "stand" }, { push: false, duration: 60 });
const first = quiet.samples[2].root,
  last = quiet.root;
const drift = Math.hypot(last.x - first.x, last.z - first.z);
assert(drift < 0.025, `quiet stance drift ${drift}`);
const overwhelmed = run(
  { motion: "squat", height: 0.12 },
  { impulse: 48, duration: 20 },
);
assert(
  overwhelmed.metrics.fallen,
  "overpowering disturbance must be allowed to topple the body",
);
const passive = run({ mode: "passive" }, { push: false, duration: 10 });
assert(
  passive.root.y < 0.9,
  "unactuated rig should lose its reference stance height",
);
// Actual API calibration: combined spherical axes track quaternion components, not Euler angles.
const world = new RAPIER.World({ x: 0, y: 0, z: 0 }),
  a = world.createRigidBody(RAPIER.RigidBodyDesc.fixed()),
  b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
world.createCollider(RAPIER.ColliderDesc.cuboid(0.1, 0.3, 0.1).setMass(3), b);
const created = world.createImpulseJoint(
    RAPIER.JointData.spherical({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }),
    a,
    b,
    true,
  ),
  joint = new RAPIER.SphericalImpulseJoint(
    world.impulseJoints.raw,
    world.bodies,
    created.handle,
  ),
  target = new Quaternion().setFromEuler(new Euler(0.25, 0.15, 0.5));
for (let k = 0; k < 3; k++) {
  joint.configureMotorPosition(
    k + 3,
    2 * Math.asin([target.x, target.y, target.z][k]),
    2400,
    100,
  );
  joint.setMotorMaxForce(k + 3, 180);
}
for (let i = 0; i < 600; i++) world.step();
const actual = new Quaternion().copy(b.rotation()),
  calibrationError = actual.angleTo(target);
assert(calibrationError < 0.001, "combined-axis motor calibration");
world.free();
const summary = {
  passed: true,
  cases: results.length,
  secondsPerCase: 30,
  quietSeconds: 60,
  quietDrift: drift,
  maxJointSeparation: Math.max(...results.map((r) => r.metrics.maxAnchorError)),
  maxHardLimitError: Math.max(...results.map((r) => r.hardError)),
  calibrationError,
  overpoweredFalls: overwhelmed.metrics.fallen,
  passiveRootHeight: passive.root.y,
};
writeFileSync(
  "evidence/3d-behavior-results.json",
  JSON.stringify({ summary, results, quiet, overwhelmed, passive }, null, 2),
);
console.log(JSON.stringify(summary, null, 2));
