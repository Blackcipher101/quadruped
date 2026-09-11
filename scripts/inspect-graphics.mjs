import {chromium} from '@playwright/test';
const browser=await chromium.launch({channel:'chrome',headless:true});
const page=await browser.newPage({viewport:{width:1440,height:960}});
page.on('pageerror',e=>console.log('ERROR',e.message));page.on('console',m=>{if(m.type()==='error')console.log('CONSOLE',m.text().slice(0,1200));});
try {await page.goto('http://127.0.0.1:5173/',{waitUntil:'domcontentloaded'});await page.waitForFunction(()=>document.querySelector('#status')?.textContent==='Running'||!document.querySelector('#error')?.hidden,{},{timeout:60000});await page.waitForFunction(()=>parseFloat(document.querySelector('#sim-time')?.textContent??'0')>3,{},{timeout:60000});await page.locator('#loading').waitFor({state:'hidden'});console.log(await page.locator('#status').innerText(),await page.locator('#error').innerText(),await page.locator('#fps').innerText());await page.screenshot({path:'artifacts/graphics-inspect.png',fullPage:true,timeout:60000});}finally{await browser.close();}
