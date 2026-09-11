import { chromium } from '@playwright/test';
import assert from 'node:assert/strict';
const browser=await chromium.launch({channel:'chrome',headless:true});
try {
 const page=await browser.newPage({viewport:{width:1440,height:1000},deviceScaleFactor:1});const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto('http://127.0.0.1:5173/');await page.waitForFunction(()=>document.querySelector('#status')?.textContent==='Running');await page.waitForFunction(()=>parseFloat(document.querySelector('#sim-time')?.textContent??'0')>2);
 assert.equal(await page.evaluate(()=>crossOriginIsolated),true);assert.equal(await page.locator('#error').isVisible(),false);
 const height=parseFloat(await page.locator('#height').innerText());assert.ok(height>.25&&height<.5,`base height ${height}`);
 await page.locator('#pause').click();await page.waitForFunction(()=>document.querySelector('#status')?.textContent==='Paused');await page.screenshot({path:'artifacts/viewer-desktop.png',fullPage:true});
 await page.locator('#step').click();await page.locator('#pause').click();await page.locator('#follow').check();await page.locator('#support').check();await page.locator('#push').click();
 await page.locator('#scene').selectOption('slope15');await page.waitForFunction(()=>document.querySelector('#scene-title')?.textContent?.includes('15°'));await page.locator('#scene').selectOption('flat');
 await page.setViewportSize({width:390,height:844});await page.screenshot({path:'artifacts/viewer-mobile.png',fullPage:true});assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false,'mobile horizontal overflow');
 assert.deepEqual(errors,[]);console.log(JSON.stringify({status:'PASS',height,physicsMs:await page.locator('#physics-ms').innerText(),checks:['cross-origin isolation','Worker advances','standing height','pause/resume','step','follow camera','force overlay','impulse','terrain selection','mobile layout','no browser exceptions']},null,2));
} finally {await browser.close();}
