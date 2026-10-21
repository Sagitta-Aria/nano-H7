/* Saved OCR settings round-trip through the real UI/API in the isolated SIM harness. */
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
(async () => {
  const browser = await chromium.launch({channel:'msedge',headless:true});
  const page = await browser.newPage({viewport:{width:1440,height:1000}}), errors=[];
  page.on('pageerror', e=>errors.push(e.message));
  try {
    await page.goto(new URL('/debug', process.env.NAV_URL).href);
    await page.getByRole('tab',{name:'视觉调参',exact:true}).click();
    await page.waitForFunction(()=>!document.getElementById('tuning-save').disabled);
    assert.equal(await page.locator('#tuning-confidence').inputValue(),'0.6');
    assert.equal(await page.locator('#tuning-confidence_one').inputValue(),'0.3');
    await page.locator('#tuning-confidence').fill('0.55');
    await page.locator('#tuning-confidence_one').fill('0.25');
    await page.locator('#tuning-save').click();
    await page.waitForFunction(()=>document.getElementById('tuning-state').textContent.includes('已保存：'));
    const saved = await page.evaluate(async()=>(await fetch('/api/camera/vision/state')).json());
    assert.deepEqual(saved.config.detector.ocr,{confidence:.55,confidence_one:.25});
    await page.reload();
    await page.getByRole('tab',{name:'视觉调参',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('tuning-confidence_one').value==='0.25');
    await page.locator('#tuning-confidence_one').fill('0');
    await page.locator('#tuning-save').click();
    await page.waitForFunction(()=>document.getElementById('tuning-state').textContent.includes('0.01～1.00'));
    const unchanged = await page.evaluate(async()=>(await fetch('/api/camera/vision/state')).json());
    assert.deepEqual(unchanged.config.detector.ocr,saved.config.detector.ocr);
    await page.locator('#tuning-reload').click();
    await page.waitForFunction(()=>document.getElementById('tuning-confidence_one').value==='0.25');
    await page.locator('#tuning-confidence_one').scrollIntoViewIfNeeded();
    await page.screenshot({path:'localtmp/ocr_confidence_ui.png'});
    assert.deepEqual(errors,[]);
    console.log('OCR controls: save, reload, validation and unchanged geometry settings passed.');
  } finally { await browser.close(); }
})().catch(error=>{console.error(error);process.exit(1);});
