// Report browser/runtime failures before leaving controls over an empty canvas.
const status = document.createElement("section");
status.className = "runtime-status";
status.setAttribute("role", "status");
status.setAttribute("aria-live", "polite");
status.textContent = "Loading the physical experiment…";
document.querySelector("main").append(status);
const controls = [...document.querySelectorAll("aside button, aside input, aside select")];
const fail = (message) => {
  document.body.classList.add("runtime-unavailable");
  status.setAttribute("role", "alert");
  status.textContent = message;
  status.hidden = false;
  controls.forEach((control) => { control.disabled = true; });
};
try {
  if (typeof WebAssembly !== "object")
    throw new Error("WebAssembly is unavailable. Enable it in your browser to run the physical experiment.");
  if (typeof WebGL2RenderingContext === "undefined")
    throw new Error("WebGL 2 is unavailable. Use a current browser with hardware acceleration enabled.");
  const demos = { planar: () => import("./main.js"), threeD: () => import("./main3d.js") };
  const start = demos[document.body.dataset.demo];
  if (!start) throw new Error("Unknown experiment entry point.");
  await start();
  status.hidden = true;
  document.body.dataset.runtime = "ready";
  document.querySelector("canvas").addEventListener("webglcontextlost", () => {
    fail("The graphics context was lost. Close other GPU-heavy tabs and reload this experiment.");
  });
} catch (error) {
  console.warn("Contact Lab startup:", error);
  fail(error.message + " The experiment has not started. Check browser support and reload.");
}
