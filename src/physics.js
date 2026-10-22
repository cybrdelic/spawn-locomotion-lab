import RAPIER from "@dimforge/rapier2d-compat";
await RAPIER.init();
export const DT = 1 / 120;
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
export function legIK(hip, ankle, L = 0.44) {
  const dx = ankle.x - hip.x,
    dy = ankle.y - hip.y,
    d = clamp(Math.hypot(dx, dy), 0.12, 2 * L - 0.005);
  const a = Math.atan2(dx, -dy),
    b = Math.acos(d / (2 * L));
  return [a + b, -2 * b, -a + b];
}
export class Experiment {
  constructor({
    mode = "adaptive",
    motion = "squat",
    height = 0,
    length = 0.44,
  } = {}) {
    this.length = length;
    const L = length,
      H = (0.91 * L) / 0.44;
    this.mode = mode;
    this.motion = motion;
    this.height = height;
    this.time = 0;
    this.bodies = [];
    this.joints = [];
    this.contacts = [false, false];
    this.ground = [0, 0];
    this.world = new RAPIER.World({ x: 0, y: -9.81 });
    this.world.timestep = DT;
    this.world.numSolverIterations = 12;
    const ground = this.world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    this.world.createCollider(
      RAPIER.ColliderDesc.cuboid(20, 0.1)
        .setTranslation(0, -0.1)
        .setFriction(1),
      ground,
    );
    if (height > 0)
      this.world.createCollider(
        RAPIER.ColliderDesc.cuboid(0.37, height / 2)
          .setTranslation(0.43, height / 2)
          .setFriction(1),
        ground,
      );
    this.root = this.body("torso", 0, H + 0.28, 0.28, 0.56, 27, 0, "blue");
    const head = this.body("head", 0, H + 0.73, 0.25, 0.29, 5, 0, "ivory");
    this.world.createImpulseJoint(
      RAPIER.JointData.fixed({ x: 0, y: 0.28 }, 0, { x: 0, y: -0.17 }, 0),
      this.root,
      head,
      true,
    );
    for (let s = 0; s < 2; s++) {
      const footx = s ? 0.25 : -0.25,
        h = s ? height : 0,
        hip = { x: 0, y: H },
        ankle = { x: footx, y: h + 0.1 };
      const pose = legIK(hip, ankle, L),
        a = pose[0],
        b = a + pose[1],
        knee = { x: hip.x + L * Math.sin(a), y: hip.y - L * Math.cos(a) };
      const thigh = this.body(
        "thigh" + s,
        (hip.x + knee.x) / 2,
        (hip.y + knee.y) / 2,
        0.13,
        L,
        6,
        a,
        "blue",
        s,
      );
      const shin = this.body(
        "shin" + s,
        knee.x + (L / 2) * Math.sin(b),
        knee.y - (L / 2) * Math.cos(b),
        0.105,
        L,
        3,
        b,
        "ivory",
        s,
      );
      const foot = this.body(
        "foot" + s,
        footx,
        ankle.y - 0.045,
        0.34,
        0.11,
        1.5,
        0,
        "dark",
        s,
      );
      this["foot" + s] = foot;
      this.hinge(
        "hip" + s,
        this.root,
        thigh,
        { x: 0, y: -0.28 },
        { x: 0, y: L / 2 },
        [-1.3, 1.3],
      );
      this.hinge(
        "knee" + s,
        thigh,
        shin,
        { x: 0, y: -L / 2 },
        { x: 0, y: L / 2 },
        [-2.5, 0.03],
      );
      this.hinge(
        "ankle" + s,
        shin,
        foot,
        { x: 0, y: -L / 2 },
        { x: 0, y: 0.045 },
        [-0.9, 0.9],
      );
      const arm = this.body("arm" + s, 0, H + 0.31, 0.1, 0.3, 2, 0, "blue", s);
      const fore = this.body(
        "forearm" + s,
        0,
        H + 0.02,
        0.085,
        0.28,
        1,
        0,
        "ivory",
        s,
      );
      this.hinge(
        "shoulder" + s,
        this.root,
        arm,
        { x: 0, y: 0.18 },
        { x: 0, y: 0.15 },
        [-2.8, 1.3],
      );
      this.hinge(
        "elbow" + s,
        arm,
        fore,
        { x: 0, y: -0.15 },
        { x: 0, y: 0.14 },
        [0, 2.6],
      );
    }
    this.metrics = {
      maxLimitError: 0,
      fall: false,
      contactFrames: 0,
      samples: 0,
      targetChange: 0,
    };
  }
  body(name, x, y, w, h, mass, angle, color, side) {
    const b = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(x, y)
        .setRotation(angle)
        .setCanSleep(false),
    );
    this.world.createCollider(
      RAPIER.ColliderDesc.cuboid(w / 2, h / 2)
        .setMass(mass)
        .setFriction(0.9)
        .setCollisionGroups(0x00020001),
      b,
    );
    // Ground belongs to group 1; mannequin belongs to 2 and sees only 1.
    this.bodies.push({ name, b, w, h, mass, color, side });
    return b;
  }
  hinge(name, a, b, pa, pb, limits) {
    const j = this.world.createImpulseJoint(
      RAPIER.JointData.revolute(pa, pb),
      a,
      b,
      true,
    );
    j.setLimits(...limits);
    j.setContactsEnabled(false);
    this.joints.push({ name, j, a, b, limits, target: 0 });
  }
  com() {
    let x = 0,
      y = 0,
      vx = 0,
      vy = 0,
      m = 0;
    for (const o of this.bodies) {
      const p = o.b.translation(),
        v = o.b.linvel();
      x += p.x * o.mass;
      y += p.y * o.mass;
      vx += v.x * o.mass;
      vy += v.y * o.mass;
      m += o.mass;
    }
    return { x: x / m, y: y / m, vx: vx / m, vy: vy / m };
  }
  reference() {
    const phase = 0.5 - 0.5 * Math.cos(this.time * 1.45),
      bend = this.motion === "squat" ? 0.17 * phase : 0;
    return {
      height: 0.91 - bend,
      shift: this.motion === "squat" ? 0.06 * Math.sin(this.time * 1.45) : 0,
      reach:
        this.motion === "reach"
          ? 0.9 * (0.5 - 0.5 * Math.cos(this.time * 1.2))
          : 0,
    };
  }
  targets() {
    const ref = this.reference(),
      com = this.com();
    const support = [];
    for (let s = 0; s < 2; s++) {
      const foot = this["foot" + s];
      let touching = false;
      this.world.contactPairsWith(foot.collider(0), (other) => {
        if (other.parent()?.isDynamic()) return;
        this.world.contactPair(foot.collider(0), other, (manifold) => {
          for (let i = 0; i < manifold.numSolverContacts(); i++) {
            if (manifold.solverContactDist(i) <= 0.005) touching = true;
          }
        });
      });
      this.contacts[s] = touching;
      if (touching) support.push(foot.translation().x);
    }
    for (let s = 0; s < 2; s++) {
      const hit = this.world.castRay(
        new RAPIER.Ray({ x: s ? 0.25 : -0.25, y: 2 }, { x: 0, y: -1 }),
        3,
        true,
        RAPIER.QueryFilterFlags.EXCLUDE_DYNAMIC,
      );
      if (hit) this.ground[s] = 2 - hit.timeOfImpact;
    }
    const mid = support.length
      ? support.reduce((a, b) => a + b, 0) / support.length
      : com.x;
    // Capture-point feedback changes the feasible hip target, never the rigid-body position.
    const correction =
      this.mode === "adaptive" && support.length
        ? clamp(0.45 * (mid - com.x) - 0.18 * com.vx, -0.12, 0.12)
        : 0;
    const hip = {
      x: ref.shift + correction,
      y:
        ref.height * (this.mode === "adaptive" ? this.length / 0.44 : 1) +
        (this.mode === "adaptive"
          ? (this.ground[0] + this.ground[1]) * 0.5
          : 0),
    };
    const out = {};
    for (let s = 0; s < 2; s++) {
      const ankle = {
        x: s ? 0.25 : -0.25,
        y: 0.1 + (this.mode === "adaptive" ? this.ground[s] : 0),
      };
      const p = legIK(
        hip,
        ankle,
        this.mode === "adaptive" ? this.length : 0.44,
      );
      out["hip" + s] = p[0];
      out["knee" + s] = p[1];
      out["ankle" + s] = p[2];
      out["shoulder" + s] = -ref.reach;
      out["elbow" + s] = 0.13 + ref.reach * 0.35;
    }
    this.targetHip = hip;
    this.referenceHip = { x: ref.shift, y: ref.height };
    return out;
  }
  step() {
    for (const o of this.bodies) o.b.resetTorques(false);
    const targets = this.targets();
    for (const q of this.joints) {
      const desired = clamp(
        targets[q.name],
        q.limits[0] + 0.035,
        q.limits[1] - 0.035,
      );
      q.target = desired;
      // Native implicit spring motors are bounded by anatomy. Passive mode removes actuation.
      q.j.configureMotorModel(RAPIER.MotorModel.ForceBased);
      q.j.configureMotorPosition(
        desired,
        this.mode === "passive" ? 0 : q.name.startsWith("ankle") ? 450 : 650,
        this.mode === "passive" ? 0 : 45,
      );
      q.j.setMotorMaxForce(
        q.name.startsWith("shoulder") || q.name.startsWith("elbow") ? 35 : 180,
      );
      const angle = Math.atan2(
        Math.sin(q.b.rotation() - q.a.rotation()),
        Math.cos(q.b.rotation() - q.a.rotation()),
      );
      const band = 0.18,
        lo = q.limits[0] + band,
        hi = q.limits[1] - band;
      let tissue =
        angle < lo ? 80 * (lo - angle) : angle > hi ? -80 * (angle - hi) : 0;
      if (tissue) {
        tissue -= 2 * (q.b.angvel() - q.a.angvel());
        tissue = clamp(tissue, -30, 30);
        q.b.addTorque(tissue, true);
        q.a.addTorque(-tissue, true);
      }
      this.metrics.maxLimitError = Math.max(
        this.metrics.maxLimitError,
        q.limits[0] - angle,
        angle - q.limits[1],
        0,
      );
    }
    this.world.step();
    this.time += DT;
    const c = this.com();
    this.metrics.samples++;
    this.metrics.contactFrames += Number(this.contacts.some(Boolean));
    this.metrics.fall ||= this.root.translation().y < 0.65;
    this.metrics.targetChange = Math.max(
      this.metrics.targetChange,
      Math.abs(this.targetHip.y - this.referenceHip.y),
    );
    return c;
  }
  push(impulse = 8) {
    this.root.applyImpulse({ x: impulse, y: 0 }, true);
  }
  dispose() {
    this.world.free();
  }
  snapshot() {
    return {
      time: this.time,
      com: this.com(),
      root: this.root.translation(),
      angle: this.root.rotation(),
      contacts: this.contacts,
      metrics: { ...this.metrics },
      joints: this.joints.map((q) => ({
        name: q.name,
        target: q.target,
        angle: q.b.rotation() - q.a.rotation(),
      })),
    };
  }
}
