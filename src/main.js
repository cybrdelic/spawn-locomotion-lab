import { fitViewport } from "./viewport.js";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { Experiment, DT, legIK } from "./physics.js";
const canvas = document.querySelector("canvas"),
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFShadowMap;
renderer.setClearColor("#101923");
const scene = new THREE.Scene();
scene.fog = new THREE.Fog("#101923", 13, 30);
const camera = new THREE.PerspectiveCamera(
  34,
  innerWidth / innerHeight,
  0.1,
  80,
);
camera.position.set(3.8, 2.8, 6.5);
const orbit = new OrbitControls(camera, canvas);
orbit.target.set(-0.45, 0.95, 0);
orbit.enableDamping = true;
orbit.maxPolarAngle = 1.5;
orbit.minDistance = 3;
orbit.maxDistance = 12;
scene.add(new THREE.HemisphereLight("#d7edff", "#243737", 2.1));
const sun = new THREE.DirectionalLight("#ffedce", 4);
sun.position.set(-3, 7, 4);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.left = -5;
sun.shadow.camera.right = 5;
sun.shadow.camera.top = 5;
sun.shadow.camera.bottom = -5;
scene.add(sun);
const floor = new THREE.Mesh(
  new THREE.BoxGeometry(40, 0.2, 40),
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
  platform;
const colors = { blue: "#56b8e9", ivory: "#d7e4db", dark: "#263e4f" };
function mesh(w, h, depth, mat) {
  const o = new THREE.Mesh(new THREE.BoxGeometry(w, h, depth), mat);
  o.castShadow = true;
  o.receiveShadow = true;
  return o;
}
function reset() {
  sim?.dispose();
  group.traverse((o) => {
    o.geometry?.dispose();
    if (o.material) o.material.dispose();
  });
  group.clear();
  meshes = [];
  markers = [];
  sim = new Experiment({
    mode: document.querySelector("#mode").value,
    motion: document.querySelector("#motion").value,
    height: +document.querySelector("#height").value,
    length: +document.querySelector("#length").value,
  });
  for (const b of sim.bodies) {
    const mat = new THREE.MeshStandardMaterial({
      color: colors[b.color],
      roughness: 0.42,
      metalness: 0.15,
    });
    const m = mesh(b.w, b.h, b.side === undefined ? 0.34 : 0.13, mat);
    group.add(m);
    meshes.push(m);
  }
  for (const j of sim.joints) {
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(0.045, 16, 12),
      new THREE.MeshStandardMaterial({ color: "#93e5bf", emissive: "#163b2b" }),
    );
    group.add(m);
    markers.push(m);
  }
  platform = mesh(
    0.74,
    Math.max(0.005, sim.height),
    0.9,
    new THREE.MeshStandardMaterial({ color: "#728576", roughness: 0.8 }),
  );
  platform.position.set(0.43, sim.height / 2, 0);
  group.add(platform);
  window.lab = {
    sim,
    reset,
    snapshot: () => sim.snapshot(),
    step: (n) => {
      for (let i = 0; i < n; i++) sim.step();
      draw();
      return sim.snapshot();
    },
  };
}
const ghost = new THREE.Group();
scene.add(ghost);
const targets = new THREE.Group();
scene.add(targets);
function skeleton(g, hip, h, shift, color, opacity, L = 0.44) {
  if (!g.children.length) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      "position",
      new THREE.BufferAttribute(new Float32Array(120), 3),
    );
    g.add(
      new THREE.LineSegments(
        geometry,
        new THREE.LineBasicMaterial({
          color,
          transparent: true,
          opacity,
          depthTest: false,
        }),
      ),
    );
  }
  const data = g.children[0].geometry.attributes.position;
  let n = 0;
  const segment = (a, b) => {
    for (const p of [a, b]) {
      data.setXYZ(n++, p[0], p[1], p[2]);
    }
  };
  for (let s = 0; s < 2; s++) {
    const z = s ? 0.19 : -0.19,
      ankle = { x: s ? 0.25 : -0.25, y: 0.1 + (s ? h : 0) },
      p = legIK(hip, ankle, L);
    const knee = {
      x: hip.x + L * Math.sin(p[0]),
      y: hip.y - L * Math.cos(p[0]),
    };
    segment([hip.x, hip.y, z], [knee.x, knee.y, z]);
    segment([knee.x, knee.y, z], [ankle.x, ankle.y, z]);
    segment(
      [ankle.x - 0.17, ankle.y - 0.09, z],
      [ankle.x + 0.17, ankle.y - 0.09, z],
    );
    const reach = sim.reference().reach,
      shoulder = [hip.x, hip.y + 0.46, z],
      elbow = [
        hip.x - 0.3 * Math.sin(reach),
        hip.y + 0.46 - 0.3 * Math.cos(reach),
        z,
      ],
      hand = [
        elbow[0] - 0.28 * Math.sin(reach - 0.13),
        elbow[1] - 0.28 * Math.cos(reach - 0.13),
        z,
      ];
    segment(shoulder, elbow);
    segment(elbow, hand);
  }
  segment([hip.x, hip.y, 0], [hip.x, hip.y + 0.56, 0]);
  segment([hip.x, hip.y + 0.56, 0], [hip.x, hip.y + 0.84, 0]);
  segment([hip.x, hip.y + 0.46, -0.19], [hip.x, hip.y + 0.46, 0.19]);
  data.needsUpdate = true;
  g.children[0].geometry.setDrawRange(0, n);
  g.children[0].geometry.computeBoundingSphere();
  g.position.x = shift;
}
function draw() {
  for (let i = 0; i < sim.bodies.length; i++) {
    const b = sim.bodies[i],
      p = b.b.translation(),
      m = meshes[i];
    m.position.set(p.x, p.y, b.side === undefined ? 0 : b.side ? 0.22 : -0.22);
    m.rotation.z = b.b.rotation();
  }
  sim.joints.forEach((q, i) => {
    const a = q.j.anchor1(),
      angle = q.a.rotation(),
      p = q.a.translation();
    markers[i].position.set(
      p.x + Math.cos(angle) * a.x - Math.sin(angle) * a.y,
      p.y + Math.sin(angle) * a.x + Math.cos(angle) * a.y,
      q.name.endsWith("1") ? 0.23 : -0.23,
    );
    const v = q.b.rotation() - q.a.rotation();
    markers[i].material.color.set(
      Math.min(v - q.limits[0], q.limits[1] - v) < 0.12 ? "#ff9477" : "#93e5bf",
    );
  });
  if (sim.targetHip) {
    skeleton(ghost, sim.referenceHip, 0, -1.65, "#8194a5", 0.7);
    skeleton(
      targets,
      sim.targetHip,
      sim.mode === "adaptive" ? sim.height : 0,
      0,
      "#a6f0b8",
      0.65,
      sim.mode === "adaptive" ? sim.length : 0.44,
    );
  }
  const c = sim.com();
  document.querySelector("#stats").innerHTML =
    `TIME &nbsp; ${sim.time.toFixed(1)} s<br>COM &nbsp; ${c.y.toFixed(2)} m · ${c.vx.toFixed(2)} m/s<br>SUPPORT &nbsp; ${sim.contacts.map((x) => (x ? "●" : "○")).join(" ")}<br>ROOT &nbsp; ${sim.metrics.fall ? "fallen" : "free / unpinned"}<br>LIMIT ERROR &nbsp; ${((sim.metrics.maxLimitError * 180) / Math.PI).toFixed(1)}°`;
}
for (const id of ["mode", "motion", "height", "length"])
  document.querySelector("#" + id).addEventListener("input", () => {
    document.querySelector("#heightValue").textContent =
      Math.round(+document.querySelector("#height").value * 100) + " cm";
    document.querySelector("#lengthValue").textContent =
      Math.round(+document.querySelector("#length").value * 100) + " cm";
    reset();
  });
document.querySelector("#reset").onclick = reset;
document.querySelector("#push").onclick = () => sim.push();
addEventListener("keydown", (e) => {
  if (e.code === "Space") {
    e.preventDefault();
    sim.push();
  }
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
const resize = () => fitViewport(renderer, camera);
addEventListener("resize", resize);
window.visualViewport?.addEventListener("resize", resize);
resize();
