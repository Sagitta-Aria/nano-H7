/* SIM-only UI/preflight checks. Run tests.competition_browser_server first. */
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
(async () => {
  const browser = await chromium.launch({channel:'msedge', headless:true});
  const page = await browser.newPage({viewport:{width:1440,height:1000}});
  const errors = [], requests = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('request', request => {
    if (request.url().endsWith('/api/competition/start')) requests.push(request.postDataJSON());
  });
  try {
    await page.goto('http://127.0.0.1:18784/debug');
    await page.waitForFunction(async () => (await (await fetch('/api/state')).json()).demo === true);
    await page.locator('[data-tab="navigation"]').click();
    await page.waitForFunction(() => !!document.getElementById('material-distance-point2_steps').value);
    await page.locator('#material-distance-point2_steps').fill('1800');
    await page.locator('#material-distance-point3_steps').fill('3900');
    await page.waitForTimeout(700); // SSE must not erase this draft.
    assert.equal(await page.locator('#material-distance-point2_steps').inputValue(), '1800');
    assert.match(await page.locator('#material-distance-delta').innerText(), /2100.*未保存/);
    await page.locator('#material-distance-save').click();
    await page.waitForFunction(() => document.getElementById('material-distance-state').textContent.includes('已保存'));
    await page.reload();
    await page.locator('[data-tab="navigation"]').click();
    await page.waitForFunction(() => document.getElementById('material-distance-point3_steps').value === '3900');
    await page.locator('#material-distance-point3_steps').fill('1000');
    await page.locator('#material-distance-save').click();
    await page.waitForFunction(() => document.getElementById('material-distance-state').textContent.includes('必须大于'));
    await page.locator('#material-distance-reload').click();
    await page.waitForFunction(() => document.getElementById('material-distance-point3_steps').value === '3900');
    fs.mkdirSync('localtmp/material_flow', {recursive:true});
    for (const width of [1440,390]) {
      await page.setViewportSize({width,height:1000});
      await page.locator('#material-distance-panel').scrollIntoViewIfNeeded();
      assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth+1));
      await page.screenshot({path:`localtmp/material_flow/distances-${width}.png`});
    }
    // The start button saves distance edits before requesting startup.
    // Calibrate the new independent second-D depth before exercising the
    // existing mechanism-preflight failure in this isolated simulator.
    await page.locator('[data-tab="auxiliary"]').click();
    await page.waitForFunction(() => document.getElementById('pick-place-d_second_drop_down').value === '-1');
    await page.locator('#pick-place-d_second_drop_down').fill('6100');
    await page.locator('#pick-place-save').click();
    await page.waitForFunction(async () => (await (await fetch('/api/state')).json()).pick_place_config.d_second_drop_down === 6100);
    await page.locator('[data-tab="navigation"]').click();
    await page.locator('#material-distance-point2_steps').fill('1900');
    await page.locator('#material-start').click();
    await page.waitForFunction(async () => (await (await fetch('/api/state')).json()).competition.state === 'fault');
    await page.waitForFunction(() => document.getElementById('task-detail').textContent.includes('启动准备失败'));
    const failed = await page.evaluate(async () => (await fetch('/api/state')).json());
    assert.equal(failed.armed, false);
    assert.equal(failed.task.active, false);
    assert.deepEqual(failed.material_flow_config, {point2_steps:1900,point3_steps:3900});
    assert.equal(requests[0].full_flow, true);
    fs.mkdirSync('localtmp/material_flow', {recursive:true});
    for (const width of [1440,390]) {
      await page.setViewportSize({width,height:1000});
      await page.locator('#material-start').scrollIntoViewIfNeeded();
      assert.ok(await page.locator('#material-start').isVisible());
      assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth+1));
      await page.screenshot({path:`localtmp/material_flow/start-${width}.png`});
    }
    await page.goto('http://127.0.0.1:18784/competition.html');
    await page.waitForFunction(() => !document.getElementById('start').disabled);
    assert.match(await page.locator('#route').innerText(), /C放置并收回/);
    await page.locator('#start').click();
    await page.waitForFunction(() => document.getElementById('status').textContent.includes('启动准备失败'));
    assert.equal(requests[requests.length-1].full_flow, true);
    assert.deepEqual(errors, []);
    console.log('PASS: distance save/reload/validation/drafts/autosave, both full-flow buttons, preflight rejection before movement, desktop/mobile layout, no JS errors');
  } finally {await browser.close();}
})().catch(error => {console.error(error);process.exitCode=1;});
