/* Real UI/API regression in the SIM server; never point this at hardware. */
const {chromium}=require('playwright');
const assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true});
 const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[];
 page.on('pageerror',e=>errors.push(e.message));
 try{
  await page.goto('http://127.0.0.1:18784/debug');
  await page.waitForFunction(()=>!document.getElementById('vision-save').disabled);
  assert.equal(await page.evaluate(async()=>(await (await fetch('/api/state')).json()).demo),true);
  await page.locator('[data-tab="camera"]').click();
  await page.locator('#vision-target').selectOption('black');
  await page.locator('#vision-save').click();
  await page.waitForFunction(async()=>(await (await fetch('/api/camera/vision/state')).json()).config.target==='black');
  await page.locator('[data-tab="vision"]').click();
  await page.locator('#tuning-color').selectOption('red');
  await page.locator('#tuning-save').click();
  await page.waitForTimeout(500);
  const saved=await page.evaluate(async()=>(await (await fetch('/api/camera/vision/state')).json()).config);
  assert.equal(saved.target,'red','Saving red tuning must not retain the old black target');
  await page.reload();
  await page.waitForFunction(()=>document.getElementById('vision-target').value==='red');
  assert.equal(await page.locator('#tuning-color').inputValue(),'red');
  await page.locator('[data-tab="camera"]').click();
  await page.locator('#camera-source').selectOption('nano');
  await page.route('**/api/camera/vision/tune',async route=>{
    const payload=route.request().postDataJSON();
    assert.equal(payload.config.target,'red');
    assert.deepEqual(payload.preview_roi_pct,[15,15,70,70]);
    await route.fulfill({json:{ok:true,image:'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+jRZkAAAAASUVORK5CYII=',mask:'',report:{target:'red',found:false,roi:[96,72,448,336],color_settings:payload.config.detector.colors.red}}});
  });
  await page.locator('#vision-roi-preview').click();
  await page.waitForFunction(()=>document.getElementById('vision-roi-state').textContent.includes('HSV'));
  assert.ok((await page.locator('#vision-roi-state').textContent()).includes('目标 red'));
  await page.locator('#vision-roi-live').click();
  await page.locator('#vision-roi-stop').click();
  for(const width of [1440,390]){
    await page.setViewportSize({width,height:1000});
    await page.locator('#camera-color-debug').scrollIntoViewIfNeeded();
    assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));
    await page.screenshot({path:`localtmp/color_debug_${width}.png`});
  }
  assert.deepEqual(errors,[]);
  console.log('PASS target black -> red save/reload; ROI payload, returned parameter display, start/stop, desktop/mobile');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
