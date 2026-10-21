/* Only the isolated SIM fixture on 18784 may receive browser motor requests. */
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
(async () => {
  const browser = await chromium.launch({channel:'msedge', headless:true});
  const page = await browser.newPage({viewport:{width:1100,height:850}});
  const errors = []; page.on('pageerror', error => errors.push(error.message));
  // This older fixture exercises the retained saved-route startup branch.
  // The new full-flow UI/preflight is checked in material_flow_browser.cjs.
  await page.route('**/api/competition/start', route => {
    const payload = route.request().postDataJSON();
    assert.equal(payload.full_flow, true);
    return route.continue({postData:JSON.stringify({...payload, full_flow:false})});
  });
  try {
    await page.goto('http://127.0.0.1:18784/competition.html');
    await page.waitForFunction(() => !document.getElementById('start').disabled);
    assert.equal(await page.locator('button').count(), 1);
    assert.equal(await page.locator('input[type=checkbox]').count(), 0);
    const snapshot = () => page.evaluate(async () => (await fetch('/api/state')).json());
    assert.equal((await snapshot()).demo, true);
    assert.equal((await snapshot()).armed, false);
    await page.locator('#start').click();
    await page.waitForFunction(() => document.getElementById('start').textContent === '执行中');
    await page.reload();
    await page.waitForFunction(() => document.querySelector('h1').textContent === '路线已完成', null, {timeout:45000});
    const done = await snapshot();
    assert.equal(done.navigation.fault, 'none');
    assert.equal(done.navigation.x, '0'); assert.equal(done.navigation.y, '0');
    assert.equal(done.mission.route[0].hold, 1, 'race must preserve saved configuration');
    fs.mkdirSync('localtmp/competition_20261018', {recursive:true});
    await page.screenshot({path:'localtmp/competition_20261018/competition.png',fullPage:true});
    await page.goto('http://127.0.0.1:18784/debug');
    await page.waitForFunction(() => document.getElementById('btn-connect'));
    await page.locator('[data-tab="navigation"]').click();
    await page.locator('#nav-turn-kp').fill('34.56');
    await page.locator('#nav-turn-tolerance').fill('0.35');
    await page.locator('#nav-turn-min-sps').fill('321');
    await page.locator('#nav-turn-configure').click();
    await page.waitForFunction(async () => (await (await fetch('/api/state')).json()).control_settings?.turn?.[0] === 3456);
    await page.evaluate(() => localStorage.clear());
    await page.reload();
    await page.waitForFunction(() => document.getElementById('nav-turn-kp').value === '34.56');
    assert.equal(await page.locator('#nav-turn-tolerance').inputValue(), '0.35');
    assert.equal(await page.locator('#nav-turn-min-sps').inputValue(), '321');
    assert.deepEqual(errors, []);
    console.log('PASS: single button, no checkboxes, automatic origin/ARM/route, reload during travel, server-persisted tuning independent of browser storage');
  } finally { await browser.close(); }
})().catch(error => {console.error(error); process.exitCode=1;});
