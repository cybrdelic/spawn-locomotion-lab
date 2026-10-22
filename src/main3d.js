import { fitViewport } from "./viewport.js";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { RoundedBoxGeometry } from "three/addons/geometries/RoundedBoxGeometry.js";
import { Experiment3D, DT } from "./physics3d.js";
const renderer = new THREE.WebGLRenderer({
  canvas: document.querySelector("canvas"),
  antialias: true,
});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setClearColor("#101923");
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFShadowMap;
const scene = new THREE.Scene();
scene.fog = new THREE.Fog("#101923", 12, 28);
const camera = new THREE.PerspectiveCamera(
  34,
  innerWidth / innerHeight,
  0.1,
  60,
);
camera.position.set(6.5, 2.8, 3.2);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.set(-0.45, 1, 0);
orbit.enableDamping = true;
orbit.maxPolarAngle = 1.48;
scene.add(new THREE.HemisphereLight("#d7edff", "#243737", 2.1));
const light = new THREE.DirectionalLight("#ffedce", 4);
light.position.set(-3, 7, 4);
light.castShadow = true;
light.shadow.mapSize.set(2048, 2048);
Object.assign(light.shadow.camera, { left: -5, right: 5, top: 5, bottom: -5 });
scene.add(light);
const floor = new THREE.Mesh(
  new THREE.BoxGeometry(30, 0.2, 30),
  new THREE.MeshStandardMaterial({ color: "#20333c", roughness: 0.92 }),
);
floor.position.y = -0.1;
floor.receiveShadow = true;
scene.add(floor);
const grid = new THREE.GridHelper(20, 40, "#3e5961", "#2d424b");
grid.position.y = 0.002;
scene.add(grid);
const group = new THREE.Group();
scene.add(group);
let sim,
  meshes = [],
  markers = [],
  lastImpulse = "none";
const colors = { blue: "#56b8e9", ivory: "#d7e4db", dark: "#263e4f" };
const targetGeometry = new THREE.BufferGeometry();
targetGeometry.setAttribute(
  "position",
  new THREE.BufferAttribute(new Float32Array(24), 3),
);
const targetLines = new THREE.LineSegments(
  targetGeometry,
  new THREE.LineBasicMaterial({
    color: "#a6f0b8",
    transparent: true,
    opacity: 0.8,
    depthTest: false,
  }),
);
scene.add(targetLines);
const comDot = new THREE.Mesh(
  new THREE.SphereGeometry(0.035, 16, 12),
  new THREE.MeshBasicMaterial({ color: "#ffedb6", depthTest: false }),
);
scene.add(comDot);
const comLineGeometry = new THREE.BufferGeometry().setFromPoints([
  new THREE.Vector3(),
  new THREE.Vector3(),
]);
const comLine = new THREE.Line(
  comLineGeometry,
  new THREE.LineDashedMaterial({
    color: "#ffedb6",
    dashSize: 0.06,
    gapSize: 0.04,
    transparent: true,
    opacity: 0.6,
  }),
);
scene.add(comLine);
function reset() {
  lastImpulse = "none";
  sim?.dispose();
  group.traverse((o) => {
    o.geometry?.dispose();
    o.material?.dispose();
  });
  group.clear();
  meshes = [];
  markers = [];
  sim = new Experiment3D({
    mode: document.querySelector("#mode").value,
    motion: document.querySelector("#motion").value,
    height: +document.querySelector("#height").value,
    length: +document.querySelector("#length").value,
  });
  for (const o of sim.bodies) {
    const { x, y, z } = o.size,
      geometry =
        o.name === "head"
          ? new THREE.SphereGeometry(0.145, 24, 16)
          : new RoundedBoxGeometry(x, y, z, 3, Math.min(x, z) * 0.25);
    const m = new THREE.Mesh(
      geometry,
      new THREE.MeshStandardMaterial({
        color: colors[o.color],
        roughness: 0.45,
        metalness: 0.12,
      }),
    );
    m.castShadow = true;
    m.receiveShadow = true;
    group.add(m);
    meshes.push(m);
  }
  for (const j of sim.joints) {
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(0.043, 16, 12),
      new THREE.MeshStandardMaterial({ color: "#93e5bf" }),
    );
    group.add(m);
    markers.push(m);
  }
  if (sim.height) {
    const platform = new THREE.Mesh(
      new THREE.BoxGeometry(1.2, sim.height, 0.6),
      new THREE.MeshStandardMaterial({ color: "#728576", roughness: 0.8 }),
    );
    platform.position.set(0, sim.height / 2, 0.38);
    platform.receiveShadow = true;
    platform.castShadow = true;
    group.add(platform);
  }
  window.lab3d = {
    snapshot: () => sim.snapshot(),
    rendererInfo: () => ({
      geometries: renderer.info.memory.geometries,
      calls: renderer.info.render.calls,
    }),
    reset,
  };
}
function draw() {
  sim.bodies.forEach((o, i) => {
    meshes[i].position.copy(o.b.translation());
    meshes[i].quaternion.copy(o.b.rotation());
  });
  sim.joints.forEach((j, i) => {
    markers[i].position
      .copy(j.j.anchor1())
      .applyQuaternion(new THREE.Quaternion().copy(j.a.rotation()))
      .add(j.a.translation());
  });
  const attr = targetGeometry.attributes.position;
  let n = 0;
  for (const leg of sim.targetLegs)
    for (const p of [leg.hip, leg.kneePoint, leg.kneePoint, leg.ankle])
      attr.setXYZ(n++, p.x, p.y, p.z);
  attr.needsUpdate = true;
  targetGeometry.computeBoundingSphere();
  const c = sim.com();
  comDot.position.copy(c.p);
  const line = comLineGeometry.attributes.position;
  line.setXYZ(0, c.p.x, c.p.y, c.p.z);
  line.setXYZ(1, c.p.x, 0.012, c.p.z);
  line.needsUpdate = true;
  comLineGeometry.computeBoundingSphere();
  comLine.computeLineDistances();
  const up = new THREE.Vector3(0, 1, 0).applyQuaternion(
    new THREE.Quaternion().copy(sim.root.rotation()),
  );
  document.querySelector("#stats").innerHTML =
    `TIME &nbsp; ${sim.time.toFixed(1)} s<br>COM &nbsp; ${c.p.y.toFixed(2)} m<br>SIDE VELOCITY &nbsp; ${c.v.z.toFixed(3)} m/s<br>BODY TILT &nbsp; ${((Math.acos(THREE.MathUtils.clamp(up.y, -1, 1)) * 180) / Math.PI).toFixed(1)}°<br>SUPPORT &nbsp; ${sim.contacts.map((x) => (x ? "●" : "○")).join(" ")}<br>ROOT &nbsp; ${sim.metrics.fallen ? "fallen" : "6 DOF / free"}<br>IMPULSE &nbsp; ${lastImpulse}`;
}
for (const id of ["mode", "motion", "height", "length"])
  document.querySelector("#" + id).addEventListener("input", () => {
    document.querySelector("#heightValue").textContent =
      Math.round(+document.querySelector("#height").value * 100) + " cm";
    document.querySelector("#lengthValue").textContent =
      Math.round(+document.querySelector("#length").value * 100) + " cm";
    reset();
  });
function applyImpulse(direction) {
  sim.push(direction);
  lastImpulse = `24 N·s ${direction}`;
}
document.querySelector("#side").onclick = () => applyImpulse("side");
document.querySelector("#forward").onclick = () => applyImpulse("forward");
document.querySelector("#reset").onclick = reset;
addEventListener("keydown", (e) => {
  if (e.code === "Space") {
    e.preventDefault();
    applyImpulse("side");
  }
  if (e.code === "KeyX") applyImpulse("forward");
  if (e.code === "KeyR") reset();
});
let last = performance.now(),
  acc = 0;
reset();
function tick(now) {
  requestAnimationFrame(tick);
  acc += Math.min(0.05, (now - last) / 1000);
  last = now;
  while (acc >= DT) {
    sim.step();
    acc -= DT;
  }
  draw();
  orbit.update();
  renderer.render(scene, camera);
}
requestAnimationFrame(tick);
let mobileLayout;
function resize() {
  const mobile = innerWidth <= 600;
  if (mobile !== mobileLayout) {
    camera.position.set(...(mobile ? [4.3, 2.0, 2.1] : [6.5, 2.8, 3.2]));
    orbit.target.set(mobile ? 0 : -0.45, mobile ? 0.9 : 1, 0);
    mobileLayout = mobile;
  }
  fitViewport(renderer, camera);
}
addEventListener("resize", resize);
window.visualViewport?.addEventListener("resize", resize);
resize();
