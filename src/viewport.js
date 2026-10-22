// Presentation sizing only: keep the body visible above touch controls.
export function fitViewport(renderer, camera) {
  const mobile = innerWidth <= 600;
  const top = mobile ? Math.ceil(document.querySelector("header").getBoundingClientRect().bottom + 12) : 0;
  const controlsTop = document.querySelector("aside").getBoundingClientRect().top;
  const height = mobile ? Math.max(100, Math.min(Math.round(innerHeight * 0.49), controlsTop - top - 12)) : innerHeight;
  renderer.domElement.style.top = top + "px";
  renderer.domElement.style.bottom = "auto";
  renderer.setSize(innerWidth, height);
  camera.aspect = innerWidth / height;
  camera.updateProjectionMatrix();
}
