// Capture README screenshots and demo videos (gravel walk, water) from the running dev server.
//   npm run dev &   then   node scripts/capture-demo.mjs [outDir]
// Uses system Chrome; set HEADED=1 (with DISPLAY) to render on the GPU instead of SwiftShader.
import { chromium } from '@playwright/test';
import { mkdirSync, renameSync } from 'node:fs';
import { join } from 'node:path';

const out = process.argv[2] ?? 'artifacts/demo';
const url = process.env.URL ?? 'http://127.0.0.1:5173/';
const view = { width: 1280, height: 720 };
mkdirSync(out, { recursive: true });

const browser = await chromium.launch({ channel: 'chrome', headless: process.env.HEADED !== '1', args: ['--ignore-gpu-blocklist'] });
const simTime = (page) => page.evaluate(() => parseFloat(document.querySelector('#sim-time')?.textContent ?? '0'));

async function clip(name, prepare, wallSeconds) {
  const ctx = await browser.newContext({ viewport: view, recordVideo: { dir: out, size: view } });
  const page = await ctx.newPage();
  page.setDefaultTimeout(180000);
  const errors = [];
  page.on('pageerror', (e) => errors.push(e.message));
  await page.goto(url);
  await page.locator('#loading').waitFor({ state: 'hidden' });
  await prepare(page);
  const t0 = await simTime(page);
  const shots = [];
  for (let k = 0; k < 3; k++) {
    await page.waitForTimeout((wallSeconds * 1000) / 3);
    const p = join(out, `${name}-${k}.png`);
    await page.screenshot({ path: p });
    shots.push(p);
  }
  const t1 = await simTime(page);
  const video = page.video();
  await ctx.close();
  const dst = join(out, `${name}.webm`);
  renameSync(await video.path(), dst);
  console.log(JSON.stringify({ name, simSeconds: +(t1 - t0).toFixed(2), wallSeconds, realtimeFactor: +((t1 - t0) / wallSeconds).toFixed(3), video: dst, shots, errors }));
  if (errors.length) process.exitCode = 1;
}

try {
  await clip('gravel-walk', async (page) => {
    await page.locator('#scene').selectOption('gravel');
    await page.locator('#follow').check();
    await page.waitForFunction(() => parseFloat(document.querySelector('#sim-time').textContent) > 1);
    await page.locator('#walk').click();
  }, 40);
  await clip('water', async (page) => {
    await page.locator('#scene').selectOption('water');
    await page.waitForFunction(() => parseFloat(document.querySelector('#sim-time').textContent) > 0.2);
  }, 30);
  await clip('flat-trot', async (page) => {
    await page.locator('#scene').selectOption('flat');
    await page.locator('#follow').check();
    await page.locator('#support').check();
    await page.waitForFunction(() => parseFloat(document.querySelector('#sim-time').textContent) > 1);
    await page.locator('#walk').click();
  }, 30);
} finally {
  await browser.close();
}
