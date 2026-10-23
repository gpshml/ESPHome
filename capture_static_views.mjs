#!/usr/bin/env node
/**
 * Capture static orthographic / 3D views from the Three.js source model.
 */
import puppeteer from "puppeteer-core";
import { mkdir } from "fs/promises";
import path from "path";

const CHROME = process.env.CHROME_PATH || "/usr/local/bin/google-chrome";
const BASE = process.env.VIEWER_URL || "http://127.0.0.1:8765/cabin-bed-3d-viewer.html";
const OUT = process.env.OUT_DIR || "/workspace";
const VIEWS = ["3d", "front", "top", "side"];
const FILES = {
  "3d": "3d_view_FINAL.png",
  front: "front_elevation_FINAL.png",
  top: "top_plan_FINAL.png",
  side: "side_elevation_FINAL.png",
};

async function main() {
  await mkdir(OUT, { recursive: true });
  console.log("Launching Chrome…", CHROME);
  const browser = await puppeteer.launch({
    executablePath: CHROME,
    headless: true,
    args: [
      "--no-sandbox",
      "--disable-setuid-sandbox",
      "--use-gl=angle",
      "--enable-webgl",
      "--ignore-gpu-blocklist",
    ],
  });

  const page = await browser.newPage();
  await page.setViewport({ width: 1600, height: 1200, deviceScaleFactor: 2 });

  page.on("console", (msg) => console.log("  [browser]", msg.type(), msg.text()));
  page.on("pageerror", (err) => console.log("  [pageerror]", err.message));

  console.log("Loading", BASE);
  await page.goto(BASE, { waitUntil: "domcontentloaded", timeout: 60000 });
  await page.waitForFunction(() => window.__cabinBed && window.__cabinBed.ready, {
    timeout: 90000,
  });
  // Hide UI chrome for clean captures
  await page.addStyleTag({
    content: "#ui, #view-label { display: none !important; }",
  });

  for (const view of VIEWS) {
    console.log("Capturing", view, "…");
    await page.evaluate((v) => {
      window.__cabinBed.setView(v);
      window.__cabinBed.render();
    }, view);
    // Allow a couple frames
    await new Promise((r) => setTimeout(r, 400));
    await page.evaluate(() => window.__cabinBed.render());
    const outPath = path.join(OUT, FILES[view]);
    // Screenshot the WebGL canvas only
    const canvas = await page.$("canvas");
    await canvas.screenshot({ path: outPath, type: "png" });
    console.log("  →", outPath);
  }

  await browser.close();
  console.log("✓ All static views captured from Three.js source model");
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
