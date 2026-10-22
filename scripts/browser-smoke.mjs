import assert from "node:assert/strict";
import { createServer } from "node:http";
import { readFile, stat, mkdir, writeFile } from "node:fs/promises";
import { extname, resolve, sep } from "node:path";
import { fileURLToPath } from "node:url";
import { chromium } from "playwright";

const root = fileURLToPath(new URL("../dist/", import.meta.url));
const evidence = fileURLToPath(new URL("../evidence/browser/", import.meta.url));
await mkdir(evidence, { recursive: true });
const mime = { ".html": "text/html", ".js": "text/javascript", ".css": "text/css", ".wasm": "application/wasm", ".json": "application/json" };
const server = createServer(async (request, response) => {
  try {
    const pathname = decodeURIComponent(new URL(request.url, "http://localhost").pathname);
    const path = resolve(root, "." + (pathname === "/" ? "/index.html" : pathname));
    if (!path.startsWith(resolve(root) + sep)) throw new Error("Invalid path");
    if (!(await stat(path)).isFile()) throw new Error("Not a file");
    response.writeHead(200, { "Content-Type": mime[extname(path)] || "application/octet-stream" });
    response.end(await readFile(path));
  } catch { response.writeHead(404); response.end("Not found"); }
});
await new Promise((resolve) => server.listen(0, "127.0.0.1", resolve));
const origin = "http://127.0.0.1:" + server.address().port;
let browser;
const checks = [];
try {
  browser = await chromium.launch({
    headless: true,
    ...(process.env.BROWSER_CHANNEL ? { channel: process.env.BROWSER_CHANNEL } : {}),
    args: process.env.CI ? ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"] : [],
  });
  for (const viewport of [{width:1280,height:800}, {width:390,height:844}, {width:844,height:390}, {width:320,height:568}]) {
    const context = await browser.newContext({ viewport, hasTouch: viewport.width <= 600 });
    for (const entry of ["index.html", "3d.html"]) {
      const page = await context.newPage();
      const errors = [], requests = [];
      page.on("pageerror", (error) => errors.push(error.message));
      page.on("response", (response) => { if (response.status() >= 400) requests.push({url:response.url(),status:response.status()}); });
      assert.equal((await page.goto(origin + "/" + entry)).status(), 200);
      await page.waitForFunction(() => document.body.dataset.runtime === "ready" || document.body.classList.contains("runtime-unavailable"), undefined, { timeout: 45000 });
      if (await page.locator("body").evaluate((el) => el.dataset.runtime !== "ready")) {
        const message = await page.locator(".runtime-status").innerText();
        await page.screenshot({path:resolve(evidence,"unexpected-startup-failure.png")});
        await writeFile(resolve(evidence,"unexpected-startup-failure.json"),JSON.stringify({entry,viewport,message,errors,requests},null,2));
        throw new Error("Runtime unavailable: " + message);
      }
      await page.locator("#stats").filter({ hasText: "TIME" }).waitFor();
      const layout = await page.evaluate(() => {
        const canvas = document.querySelector("canvas").getBoundingClientRect();
        const aside = document.querySelector("aside").getBoundingClientRect();
        return { width:innerWidth, height:innerHeight, scrollWidth:document.documentElement.scrollWidth,
          canvas:{x:canvas.x,y:canvas.y,width:canvas.width,height:canvas.height,bottom:canvas.bottom},
          controls:{top:aside.top,bottom:aside.bottom,width:aside.width} };
      });
      assert(layout.scrollWidth <= viewport.width, "horizontal page overflow");
      assert(layout.canvas.width > 0 && layout.canvas.height >= 100, "missing physical viewport");
      assert(layout.canvas.bottom <= viewport.height + 1, "canvas below viewport");
      assert(layout.controls.top >= 0 && layout.controls.bottom <= viewport.height + 1, "controls outside viewport");
      if (viewport.width <= 600) assert(layout.canvas.bottom <= layout.controls.top, "controls cover mobile physical view");
      await page.getByRole("button", { name: /Push (sideways|torso)/ }).click();
      await page.getByRole("button", { name: /Reset/ }).click();
      await page.screenshot({ path: resolve(evidence, entry.replace(".html","") + "-" + viewport.width + "x" + viewport.height + ".png") });
      if (entry === "3d.html") {
        await page.getByRole("link", { name: "Earlier planar comparison" }).click();
        await page.waitForURL("**/index.html");
        await page.waitForFunction(() => document.body.dataset.runtime === "ready", undefined, { timeout: 45000 });
      }
      assert.deepEqual(errors, [], "browser exceptions");
      assert.deepEqual(requests, [], "failed production resources");
      checks.push({entry,viewport,layout,errors,failedRequests:requests,status:"passed"});
      await page.close();
    }
    await context.close();
  }
  for (const capability of ["WebAssembly", "WebGL2RenderingContext"]) {
    const context = await browser.newContext({ viewport: {width:390,height:844} });
    await context.addInitScript((name) => { Object.defineProperty(window, name, {value:undefined,configurable:true}); }, capability);
    const page = await context.newPage();
    await page.goto(origin + "/3d.html");
    await page.getByRole("alert").waitFor();
    assert(await page.locator("body").evaluate((el) => el.classList.contains("runtime-unavailable")));
    assert(await page.getByRole("button",{name:/Reset/}).isDisabled());
    await page.screenshot({path:resolve(evidence,"fallback-"+capability+".png")});
    checks.push({capability,status:"clear fallback shown; controls disabled"});
    await context.close();
  }
  await writeFile(resolve(evidence,"checks.json"),JSON.stringify({browser:browser.version(),platform:process.platform,checks},null,2));
  console.log(JSON.stringify({passed:true,browser:browser.version(),platform:process.platform,checks:checks.length,evidence},null,2));
} finally {
  server.closeAllConnections();
  if (browser) await browser.close();
  await new Promise((resolve) => server.close(resolve));
}
