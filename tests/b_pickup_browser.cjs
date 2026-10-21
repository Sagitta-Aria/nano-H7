/* Run against tests.competition_browser_server, never against live hardware. */
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
(async () => {
  const browser = await chromium.launch({channel:'msedge', headless:true});
  const page = await browser.newPage({viewport:{width:1440,height:1000}});
  const errors = []; page.on('pageerror', error => errors.push(error.message));
  try {
    await page.goto('http://127.0.0.1:18784/debug');
    await page.waitForFunction(() => !!document.getElementById('b-pickup-pick_down')?.value);
    const state = () => page.evaluate(async () => (await (await fetch('/api/state')).json()));
    assert.equal((await state()).demo, true);
    await page.locator('[data-tab="camera"]').click();
    await page.locator('#b-pickup-open-roi').click();
    assert.equal(await page.locator('#b-pickup-fields input').count(), 17);
    assert.equal(await page.locator('#b-pickup-recognition_delay_ms').inputValue(), '2000');
    assert.equal(await page.locator('#b-pickup-fields input').first().getAttribute('id'), 'b-pickup-roi_x_pct');
    await page.locator('[data-b-roi-size="70"]').click();
    assert.equal(await page.locator('#b-pickup-roi_x_pct').inputValue(), '15');
    assert.equal(await page.locator('#b-pickup-roi_width_pct').inputValue(), '70');
    assert.equal((await state()).b_pickup_config.roi_width_pct, 30);
    await page.locator('#b-pickup-roi-save').click();
    await page.waitForFunction(async () => (await (await fetch('/api/state')).json()).b_pickup_config.roi_width_pct === 70);
    await page.locator('#b-pickup-roi_x_pct').fill('40');
    await page.locator('#b-pickup-roi_width_pct').fill('20');
    await page.locator('#b-pickup-pick_down').fill('8765');
    await page.locator('#b-pickup-recognition_delay_ms').fill('3500');
    await page.waitForTimeout(500);
    assert.equal(await page.locator('#b-pickup-pick_down').inputValue(), '8765');
    await page.locator('#b-pickup-save').click();
    await page.waitForFunction(async () => (await (await fetch('/api/state')).json()).b_pickup_config.pick_down === 8765);
    assert.equal((await state()).pick_place_config.pick_down, 9200);
    assert.equal((await state()).b_pickup_config.roi_x_pct, 40);
    assert.equal((await state()).b_pickup_config.roi_width_pct, 20);
    assert.equal((await state()).b_pickup_config.recognition_delay_ms, 3500);
    assert.equal((await state()).pick_place_config.recognition_delay_ms, undefined);
    await page.reload();
    await page.locator('[data-tab="auxiliary"]').click();
    await page.waitForFunction(() => document.getElementById('b-pickup-pick_down')?.value === '8765');
    assert.equal(await page.locator('#b-pickup-recognition_delay_ms').inputValue(), '3500');
    await page.locator('#b-pickup-pick_down').fill('7777');
    await page.locator('#b-pickup-reload').click();
    await page.waitForFunction(() => document.getElementById('b-pickup-pick_down').value === '8765');
    fs.mkdirSync('localtmp/b_pickup_browser', {recursive:true});
    for (const width of [1440,390]) {
      await page.setViewportSize({width,height:1000});
      await page.locator('#b-pickup-card').scrollIntoViewIfNeeded();
      assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth+1));
      await page.screenshot({path:`localtmp/b_pickup_browser/b-${width}.png`});
    }
    const final = await state();
    assert.equal(final.armed, false);
    assert.ok(Object.values(final.axes).every(a => a.active === '0'));
    assert.deepEqual(errors, []);
    console.log('PASS B parameters save independently, survive reload, retain drafts; desktop/mobile; no movement or JS errors');
  } finally {await browser.close();}
})().catch(error => {console.error(error); process.exitCode=1;});
