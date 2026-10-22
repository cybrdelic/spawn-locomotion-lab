import RAPIER from "@dimforge/rapier3d-compat";
import { Vector3, Quaternion, Euler } from "three";
await RAPIER.init();
export const DT = 1 / 120;
const V = (x = 0, y = 0, z = 0) => new Vector3(x, y, z),
  Q = (q) => new Quaternion(q.x, q.y, q.z, q.w);
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const rotation = (x = 0, y = 0, z = 0) =>
  new Quaternion().setFromEuler(new Euler(x, y, z, "XYZ"));
const angles = (q) => {
  const e = new Euler().setFromQuaternion(q, "XYZ");
  return [e.x, e.y, e.z];
};
const axes = [
  RAPIER.JointAxis.AngX,
  RAPIER.JointAxis.AngY,
  RAPIER.JointAxis.AngZ,
];

// The knee bends in a plane selected by the ankle direction. Segment lengths are exact.
export function legTarget(hip, ankle, L, lower = L) {
  const delta = ankle.clone().sub(hip),
    roll = -Math.atan2(delta.z, -delta.y);
  const local = delta.clone().applyQuaternion(rotation(-roll));
  const d = clamp(Math.hypot(local.x, local.y), Math.abs(L-lower)+.005, L+lower-.005);
  const direction = Math.atan2(local.x, -local.y),
    bend = Math.acos(clamp((L*L+d*d-lower*lower)/(2*L*d),-1,1));
  const knee = -Math.acos(clamp((d*d-L*L-lower*lower)/(2*L*lower),-1,1));
  const thigh = rotation(roll).multiply(rotation(0, 0, direction + bend));
  const shin = rotation(roll).multiply(rotation(0, 0, direction + bend + knee));
  return {
    thigh,
    shin,
    knee,
    ankle: shin.clone().invert(),
    kneePoint: V(0, -L, 0).applyQuaternion(thigh).add(hip),
  };
}

export class Experiment3D {
  constructor({
    mode = "adaptive",
    motion = "squat",
    height = 0,
    length = 0.44,
    rig = null,
  } = {}) {
    Object.assign(this, {
      mode,
      motion,
      height,
      length,
      time: 0,
      bodies: [],
      joints: [],
      feet: [],
      contacts: [false, false],
      ground: [0, 0],
      targetLegs: [],
    });
    this.rig = rig;
    this.hipWidth = rig?.hipHalfWidth ?? .23;
    this.ankleHeight = rig ? (rig.joints.LeftFoot[1]+rig.joints.RightFoot[1])*.5 : .1;
    this.hipDrop = rig ? ((rig.joints.neck[1]-(rig.joints.LeftUpLeg[1]+rig.joints.RightUpLeg[1])*.5)*.5) : .28;
    this.legData = [0,1].map(s=>({upper:rig?.upper[s]??length,lower:rig?.lower[s]??length}));
    this.armData = [0,1].map(s=>({upper:rig?.armUpper[s]??.30,lower:rig?.armLower[s]??.28}));
    this.world = new RAPIER.World(V(0, -9.81, 0));
    this.world.timestep = DT;
    this.world.numSolverIterations = 16;
    this.world.integrationParameters.numInternalPgsIterations = 8;
    this.metrics = {
      fallen: false,
      maxAnchorError: 0,
      maxTilt: 0,
      maxLateralSpeed: 0,
      maxSoftLimitExcess: 0,
    };
    const floor = this.world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    this.world.createCollider(
      RAPIER.ColliderDesc.cuboid(rig ? 50 : 10, 0.1, rig ? 50 : 10)
        .setTranslation(0, -0.1, 0)
        .setFriction(0.9),
      floor,
    );
    if (height > 0)
      this.world.createCollider(
        RAPIER.ColliderDesc.cuboid(0.6, height / 2, 0.3)
          .setTranslation(0, height / 2, 0.38)
          .setFriction(0.9),
        floor,
      );
    const H = (rig ? (rig.joints.LeftUpLeg[1]+rig.joints.RightUpLeg[1])*.5-.04 : (0.84 * length) / 0.44) + height * 0.5;
    this.root = this.body(
      "torso",
      V(-0.04, H + this.hipDrop, 0),
      V(0.28, this.hipDrop*2, 0.38),
      27,
      rotation(),
      "blue",
    );
    const head = this.body(
      "head",
      V(-0.04, H + this.hipDrop*2+.17, 0),
      V(0.25, 0.29, 0.25),
      5,
      rotation(),
      "ivory",
    );
    this.world.createImpulseJoint(
      RAPIER.JointData.fixed(
        V(0, this.hipDrop, 0),
        rotation(),
        V(0, -0.17, 0),
        rotation(),
      ),
      this.root,
      head,
      true,
    );
    for (let s = 0; s < 2; s++) {
      const {upper,lower}=this.legData[s],armDimensions=this.armData[s];
      const z = s ? this.hipWidth : -this.hipWidth,
        hip = V(-0.04, H, z),
        ankle = V(0, this.ankleHeight + (s ? height : 0), rig?z*1.1:z),
        p = legTarget(hip, ankle, upper,lower);
      const thigh = this.body(
        "thigh" + s,
        hip.clone().add(p.kneePoint).multiplyScalar(0.5),
        V(0.13, upper, 0.13),
        6,
        p.thigh,
        "blue",
      );
      const shin = this.body(
        "shin" + s,
        p.kneePoint.clone().add(ankle).multiplyScalar(0.5),
        V(0.105, lower, 0.105),
        3,
        p.shin,
        "ivory",
      );
      const foot = this.body(
        "foot" + s,
        ankle.clone().add(V(0.035, .055-this.ankleHeight, 0)),
        V(rig?.28:.36, 0.11, rig?.15:.22),
        1.5,
        rotation(),
        "dark",
      );
      this.feet.push(foot);
      this.ball(
        "hip" + s,
        this.root,
        thigh,
        V(0, -this.hipDrop, z),
        V(0, upper / 2, 0),
        [
          [-0.65, 0.65],
          [-0.4, 0.4],
          rig ? [-1.0, 2.1] : [-1.3, 1.3],
        ],
        rig ? 250 : 180,
      );
      this.hinge(
        "knee" + s,
        thigh,
        shin,
        V(0, -upper / 2, 0),
        V(0, lower / 2, 0),
        [-2.5, 0.03],
        rig ? 250 : 180,
      );
      this.ball(
        "ankle" + s,
        shin,
        foot,
        V(0, -lower / 2, 0),
        V(-0.035, this.ankleHeight-.055, 0),
        [
          [-0.55, 0.55],
          [-0.25, 0.25],
          [-0.9, 0.9],
        ],
        150,
      );
      const az = rig ? Math.abs(rig.joints[s?'RightArm':'LeftArm'][2])*(s?1:-1) : (s ? 0.32 : -0.32);
      const shoulderY = rig ? (rig.joints.LeftArm[1]+rig.joints.RightArm[1])*.5-(rig.joints.LeftUpLeg[1]+rig.joints.RightUpLeg[1])*.5 : .46;
      const arm = this.body(
        "arm" + s,
        V(-0.04, H + shoulderY-armDimensions.upper/2, az),
        V(0.1, armDimensions.upper, 0.1),
        2,
        rotation(),
        "blue",
      );
      const fore = this.body(
        "forearm" + s,
        V(-0.04, H + shoulderY-armDimensions.upper-armDimensions.lower/2, az),
        V(0.085, armDimensions.lower, 0.085),
        1,
        rotation(),
        "ivory",
      );
      this.ball(
        "shoulder" + s,
        this.root,
        arm,
        V(0, shoulderY-this.hipDrop, az),
        V(0, armDimensions.upper/2, 0),
        [
          [-1.2, 1.2],
          [-0.7, 0.7],
          [-2.5, 1.2],
        ],
        35,
      );
      this.hinge(
        "elbow" + s,
        arm,
        fore,
        V(0, -armDimensions.upper/2, 0),
        V(0, armDimensions.lower/2, 0),
        [0, 2.6],
        35,
      );
    }
  }
  body(name, p, size, mass, q, color) {
    const b = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(p.x, p.y, p.z)
        .setRotation(q)
        .setCanSleep(false),
    );
    this.world.createCollider(
      RAPIER.ColliderDesc.cuboid(size.x / 2, size.y / 2, size.z / 2)
        .setMass(mass)
        .setFriction(0.9)
        .setCollisionGroups(0x00020001),
      b,
    );
    this.bodies.push({ name, b, size, mass, color });
    return b;
  }
  ball(name, a, b, pa, pb, limits, maxTorque) {
    const created = this.world.createImpulseJoint(
      RAPIER.JointData.spherical(pa, pb),
      a,
      b,
      true,
    );
    const j =
      typeof created.configureMotorPosition === "function"
        ? created
        : new RAPIER.SphericalImpulseJoint(
            this.world.impulseJoints.raw,
            this.world.bodies,
            created.handle,
          );
    j.setContactsEnabled(false);
    for (let i = 0; i < 3; i++)
      this.world.impulseJoints.raw.jointSetLimits(
        j.handle,
        axes[i],
        limits[i][0],
        limits[i][1],
      );
    this.joints.push({
      name,
      a,
      b,
      j,
      limits,
      maxTorque,
      kind: "ball",
      target: rotation(),
    });
  }
  hinge(name, a, b, pa, pb, limits, maxTorque = 180) {
    const j = this.world.createImpulseJoint(
      RAPIER.JointData.revolute(pa, pb, V(0, 0, 1)),
      a,
      b,
      true,
    );
    j.setLimits(...limits);
    j.setContactsEnabled(false);
    this.joints.push({
      name,
      a,
      b,
      j,
      limits,
      maxTorque,
      kind: "hinge",
      target: 0,
    });
  }
  com() {
    const p = V(),
      v = V();
    let m = 0;
    for (const o of this.bodies) {
      p.addScaledVector(o.b.translation(), o.mass);
      v.addScaledVector(o.b.linvel(), o.mass);
      m += o.mass;
    }
    return { p: p.multiplyScalar(1 / m), v: v.multiplyScalar(1 / m) };
  }
  reference() {
    const cycle = 0.5 - 0.5 * Math.cos(this.time * 1.45);
    return {
      height: 0.84 - (this.motion === "squat" ? 0.14 * cycle : 0),
      reach:
        this.motion === "reach"
          ? 0.85 * (0.5 - 0.5 * Math.cos(this.time * 1.2))
          : 0,
    };
  }
  observe() {
    for (let s = 0; s < 2; s++) {
      const hit = this.world.castRay(
        new RAPIER.Ray(V(0, 2, s ? 0.23 : -0.23), V(0, -1, 0)),
        3,
        true,
        RAPIER.QueryFilterFlags.EXCLUDE_DYNAMIC,
      );
      if (hit) this.ground[s] = 2 - hit.timeOfImpact;
      this.contacts[s] = false;
      const foot = this.feet[s].collider(0);
      this.world.contactPairsWith(foot, (other) => {
        if (other.parent()?.isDynamic()) return;
        this.world.contactPair(foot, other, (m) => {
          for (let k = 0; k < m.numSolverContacts(); k++)
            if (m.solverContactDist(k) <= 0.005) this.contacts[s] = true;
        });
      });
    }
  }
  targets() {
    if(this.controller){this.controller.targets(this);return;}
    this.observe();
    const ref = this.reference(),
      { p, v } = this.com(),
      mid = V();
    let n = 0;
    this.feet.forEach((f, s) => {
      if (this.contacts[s]) {
        mid.add(f.translation());
        n++;
      }
    });
    if (n) mid.multiplyScalar(1 / n);
    else mid.copy(p);
    const adaptive = this.mode === "adaptive",
      feedback = adaptive && n;
    const hip = V(
      -0.04 +
        (feedback ? clamp(0.45 * (mid.x - p.x) - 0.18 * v.x, -0.12, 0.12) : 0),
      ref.height * (adaptive ? this.length / 0.44 : 1) +
        (adaptive ? (this.ground[0] + this.ground[1]) * 0.5 : 0),
      feedback ? clamp(0.45 * (mid.z - p.z) - 0.18 * v.z, -0.12, 0.12) : 0,
    );
    this.targetLegs = [];
    for (let s = 0; s < 2; s++) {
      const z = s ? 0.23 : -0.23,
        point = hip.clone().add(V(0, 0, z)),
        ankle = V(0, 0.1 + (adaptive ? this.ground[s] : 0), z),
        leg = legTarget(point, ankle, adaptive ? this.length : 0.44);
      this.targetLegs.push({ ...leg, hip: point, ankle });
      for (const j of this.joints) {
        if (j.name === "hip" + s) j.target = leg.thigh;
        if (j.name === "knee" + s) j.target = leg.knee;
        if (j.name === "ankle" + s) j.target = leg.ankle;
        if (j.name === "shoulder" + s) j.target = rotation(0, 0, ref.reach);
        if (j.name === "elbow" + s) j.target = 0.13 + ref.reach * 0.35;
      }
    }
  }
  actuate() {
    for (const o of this.bodies) o.b.resetTorques(false);
    const feedforward = this.gravityTorques();
    this.controller?.supportAttitude?.(this,feedforward);
    for (const j of this.joints) {
      let ff = feedforward.get(j) || V();
      const localFF = ff.clone().applyQuaternion(Q(j.a.rotation()).invert());
      if(this.controller){
        if(!j.muscleState)j.muscleState=localFF.clone();
        const delta=localFF.clone().sub(j.muscleState).multiplyScalar(1-Math.exp(-DT/.04)).clampLength(0,1200*DT);
        j.muscleState.add(delta);localFF.copy(j.muscleState);ff=localFF.clone().applyQuaternion(Q(j.a.rotation()));
      }
      const relative = Q(j.a.rotation()).invert().multiply(Q(j.b.rotation())),
        current = angles(relative),
        passive = this.mode === "passive";
      if (j.kind === "hinge") {
        const target=clamp(j.target,j.limits[0]+.035,j.limits[1]-.035);
        j.j.configureMotorModel(RAPIER.MotorModel.ForceBased);
        j.j.setMotorMaxForce(Math.max(0, j.maxTorque - Math.abs(localFF.z)));
        j.j.configureMotorPosition(
          target,
          passive ? 0 : 2400,
          passive ? 0 : 100,
        );
      } else {
        const desired = angles(j.target).map((a, i) =>
          clamp(a, j.limits[i][0] + 0.04, j.limits[i][1] - 0.04),
        );
        const feasible = rotation(...desired);
        // Rapier's independent spherical motor coordinates are 2*asin(q.xyz),
        // NOT XYZ Euler angles. Verified by single- and combined-axis calibration.
        const motor = [feasible.x, feasible.y, feasible.z].map(
          (a) => 2 * Math.asin(clamp(a, -1, 1)),
        );
        const torque = V(),
          speed = V()
            .copy(j.b.angvel())
            .sub(j.a.angvel())
            .applyQuaternion(Q(j.a.rotation()).invert());
        for (let i = 0; i < 3; i++) {
          const [lo, hi] = j.limits[i],
            a = current[i],
            inside = clamp(a, lo + 0.12, hi - 0.12),
            excess = Math.max(lo - a, a - hi, 0);
          this.metrics.maxSoftLimitExcess = Math.max(
            this.metrics.maxSoftLimitExcess,
            excess,
          );
          torque.setComponent(
            i,
            clamp(
              100 * (inside - a) -
                (inside !== a ? 4 * speed.getComponent(i) : 0),
              -30,
              30,
            ),
          );
          j.j.configureMotorModel(axes[i],RAPIER.MotorModel.ForceBased);
          j.j.setMotorMaxForce(
            axes[i],
            Math.max(0, j.maxTorque - Math.abs(localFF.getComponent(i))),
          );
          j.j.configureMotorPosition(
            axes[i],
            motor[i],
            passive ? 0 : 2400,
            passive ? 0 : 100,
          );
        }
        torque.applyQuaternion(Q(j.a.rotation()));
        j.b.addTorque(torque, true);
        j.a.addTorque(torque.clone().negate(), true);
      }
      if (!passive) {
        j.b.addTorque(ff, true);
        j.a.addTorque(ff.clone().negate(), true);
      }
    }
  }
  gravityTorques() {
    const result = new Map();
    if (this.mode !== "adaptive") return result;
    const total = this.bodies.reduce((m, o) => m + o.mass, 0) * 9.81,
      com = this.com().p,
      loads = [0, 0];
    if (this.contacts[0] && this.contacts[1]) {
      const a = this.feet[0].translation(),
        b = this.feet[1].translation();
      const line=V().copy(b).sub(a);
      loads[1] = total * clamp(V().copy(com).sub(a).dot(line)/Math.max(.001,line.lengthSq()), 0, 1);
      loads[0] = total - loads[1];
    } else
      this.contacts.forEach((on, s) => {
        if (on) loads[s] = total;
      });
    const descendants = (b) => [
      b,
      ...this.joints.filter((j) => j.a === b).flatMap((j) => descendants(j.b)),
    ];
    for (const j of this.joints) {
      const subtree = descendants(j.b),
        point = V()
          .copy(j.j.anchor1())
          .applyQuaternion(Q(j.a.rotation()))
          .add(j.a.translation()),
        moment = V();
      for (const o of this.bodies)
        if (subtree.includes(o.b))
          moment.add(
            V()
              .copy(o.b.translation())
              .sub(point)
              .cross(V(0, -9.81 * o.mass, 0).addScaledVector(this.supportAcceleration??V(),-o.mass)),
          );
      this.feet.forEach((foot, s) => {
        if (subtree.includes(foot) && (loads[s]||this.supportForces?.[s]?.lengthSq())) {
          const p = foot.translation(),body=this.bodies.find(o=>o.b===foot),local=V().copy(com).sub(p).applyQuaternion(Q(foot.rotation()).invert());
          const cop=this.supportCoPs?.[s]?.clone()??V(clamp(local.x,-body.size.x*.4,body.size.x*.4),-.055,clamp(local.z,-body.size.z*.4,body.size.z*.4)).applyQuaternion(Q(foot.rotation())).add(p);
          const force=this.supportForces?.[s]??V(0,loads[s],0);
          moment.add(cop.sub(point).cross(force));
        }
      });
      let torque = moment.negate().applyQuaternion(Q(j.a.rotation()).invert());
      if (j.kind === "hinge") torque.set(0, 0, torque.z);
      for (let k = 0; k < 3; k++)
        torque.setComponent(
          k,
          clamp(
            torque.getComponent(k),
            -j.maxTorque * 0.75,
            j.maxTorque * 0.75,
          ),
        );
      result.set(j, torque.applyQuaternion(Q(j.a.rotation())));
    }
    return result;
  }
  step() {
    this.targets();
    this.actuate();
    this.world.step();
    this.time += DT;
    const { v } = this.com(),
      up = V(0, 1, 0).applyQuaternion(Q(this.root.rotation()));
    this.metrics.maxTilt = Math.max(
      this.metrics.maxTilt,
      Math.acos(clamp(up.y, -1, 1)),
    );
    this.metrics.maxLateralSpeed = Math.max(
      this.metrics.maxLateralSpeed,
      Math.abs(v.z),
    );
    this.metrics.fallen ||= this.root.translation().y < 0.65;
    for (const j of this.joints) {
      const a = V()
          .copy(j.j.anchor1())
          .applyQuaternion(Q(j.a.rotation()))
          .add(j.a.translation()),
        b = V()
          .copy(j.j.anchor2())
          .applyQuaternion(Q(j.b.rotation()))
          .add(j.b.translation());
      this.metrics.maxAnchorError = Math.max(
        this.metrics.maxAnchorError,
        a.distanceTo(b),
      );
    }
  }
  push(direction = "side", amount = 24) {
    this.root.applyImpulse(
      direction === "side" ? V(0, 0, amount) : V(amount, 0, 0),
      true,
    );
  }
  snapshot() {
    const c = this.com();
    return {
      time: this.time,
      com: c.p,
      velocity: c.v,
      root: this.root.translation(),
      rotation: this.root.rotation(),
      contacts: [...this.contacts],
      metrics: { ...this.metrics },
    };
  }
  dispose() {
    this.world.free();
  }
}
