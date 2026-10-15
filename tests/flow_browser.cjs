/* Run against `python -m tests.flow_fixture_server` with isolated saved settings. */
const {chromium}=require('playwright');
const assert=require('node:assert/strict'),path=require('node:path');
(async()=>{
  const browser=await chromium.launch({channel:'msedge',headless:true});
  const page=await browser.newPage({viewport:{width:1440,height:1000}});
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  try {
    await page.goto('http://127.0.0.1:8786/');
    await page.getByRole('tab',{name:'光流位移'}).click();
    await page.waitForFunction(()=>!document.getElementById('flow-start').disabled);
    await page.locator('#flow-height').fill('30.125');
    await page.locator('#flow-gx').fill('1.002345');
    await page.locator('#flow-angle').fill('90.25');
    await page.locator('#flow-swap').check();
    await page.locator('#flow-save').click();
    await page.waitForFunction(()=>document.getElementById('flow-applied').textContent.includes('高度 30 mm'));
    await page.locator('#flow-apply').click();
    await page.waitForFunction(()=>document.getElementById('flow-applied').textContent.includes('30.125 mm'));
    await page.locator('#flow-start').click();
    await page.waitForFunction(()=>document.getElementById('flow-record-time').textContent.includes('记录中'));
    await page.waitForTimeout(600);
    await page.locator('#flow-stop').click();
    await page.waitForFunction(()=>document.getElementById('flow-record-time').textContent.includes('结果保留'));
    const record=await page.locator('#flow-rx').textContent();
    const total=await page.locator('#flow-x').textContent();
    await page.waitForTimeout(650);
    assert.equal(await page.locator('#flow-rx').textContent(),record);
    assert.notEqual(await page.locator('#flow-x').textContent(),total);
    await page.locator('#flow-mode').selectOption('world');
    await page.reload();
    await page.getByRole('tab',{name:'光流位移'}).click();
    await page.waitForFunction(()=>document.getElementById('flow-x').textContent!=='—');
    assert.equal(await page.locator('#flow-mode').inputValue(),'world');
    assert.equal(await page.locator('#flow-height').inputValue(),'30.125');
    assert.equal(await page.locator('#flow-rx').textContent(),record);
    await page.locator('#flow-height').fill('31');
    await page.waitForTimeout(700);
    assert.equal(await page.locator('#flow-height').inputValue(),'31');
    for (const [width,height] of [[1440,1000],[390,844]]) {
      await page.setViewportSize({width,height});
      const layout=await page.evaluate(()=>({overflow:document.documentElement.scrollWidth>innerWidth,
        clipped:[...document.querySelectorAll('#tab-flow button,#tab-flow select')].some(el=>{const r=el.getBoundingClientRect();return r.left<0||r.right>innerWidth;}),
        pixels:document.getElementById('flow-vector').getContext('2d').getImageData(0,0,700,300).data.filter((v,i)=>i%4===3&&v>0).length}));
      assert.equal(layout.overflow,false);assert.equal(layout.clipped,false);assert.ok(layout.pixels>1000);
      await page.screenshot({path:path.join(__dirname,`../tmp/flow_${width}.png`),fullPage:true});
    }
    assert.deepEqual(errors,[]);
    console.log('Flow browser OK: saved/applied calibration, frozen recording, reload, mode, drafts and mobile layout.');
  } finally {await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
