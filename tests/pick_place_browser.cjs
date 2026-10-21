/* Test the isolated SIM fixture only. No physical board or camera is used. */
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const path = require('node:path');
const fs = require('node:fs');

(async () => {
  const browser = await chromium.launch({ channel:'msedge', headless:true });
  const page = await browser.newPage({viewport:{width:1440,height:1000}});
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  try {
    await page.goto(new URL('/debug', 'http://127.0.0.1:18783/').href);
    await page.waitForFunction(() => document.getElementById('pick-place-pick_down')?.value === '9200');
    const snapshot = () => page.evaluate(async () => (await fetch('/api/state')).json());
    assert.equal((await snapshot()).demo, true, 'only the isolated simulator is allowed');
    await page.locator('[data-tab="auxiliary"]').click();
    assert.equal(await page.locator('#pick-place-vision').count(), 0);
    assert.equal(await page.locator('#pick-place-pick_rotation').inputValue(), '-1750');
    assert.equal(await page.locator('#pick-place-fields input').count(), 20);
    assert.equal(await page.locator('#pick-place-d_second_drop_down').inputValue(), '-1');
    assert.equal(await page.locator('#b-pickup-d_second_drop_down').count(), 0, 'D stacking depth must not enter the B-only profile');
    // app.js keeps its session private; capture the legitimate token emitted by
    // a harmless parameter save, then reuse the same owner for fixture setup.
    const requestPromise = page.waitForRequest(r => r.url().endsWith('/api/pick-place/config'));
    await page.locator('#pick-place-save').click();
    const token = (await requestPromise).postDataJSON().token;
    const post = (url, body) => page.evaluate(async ({url,body,token}) => {
      const response = await fetch(url, {method:'POST', headers:{'Content-Type':'application/json','X-Session':token}, body:JSON.stringify({...body,token})});
      return response.json();
    }, {url,body,token});
    for (const body of ['LIFT DIR -1','AUX ZERO 6','AUX CONFIG 5 0 9400','AUX CONFIG 6 0 0']) {
      assert.equal((await post('/api/command',{body})).ok, true);
      await page.waitForTimeout(120);
    }
    // Short distances keep browser checks fast; backend tests exercise 9200.
    for (const [key,value] of Object.entries({pick_down:12,pick_rotation:-15,place_take_down:14,place_rotation:-15,place_drop_down:18,d_second_drop_down:8,s1_settle_ms:100,s2_settle_ms:100})) {
      await page.locator(`#pick-place-${key}`).fill(String(value));
    }
    assert.equal(await page.locator('#pick-place-pick').isDisabled(), false, 'dirty, disarmed page must allow one-click testing');
    await page.waitForTimeout(300);
    assert.equal(await page.locator('#pick-place-pick_down').inputValue(), '12', 'SSE retains unsaved edits');
    await page.locator('#pick-place-code').fill('156+231+516+123');
    assert.equal((await snapshot()).armed, false);
    // Startup may already restore saved ranges. Deliberately narrow the live
    // range above so this test still exercises one-click range restoration.
    assert.equal((await snapshot()).auxiliary['6'].min, '0');
    await page.locator('#pick-place-pick').click();
    await page.waitForFunction(() => document.getElementById('pick-place-state').textContent === '本次完成', null, {timeout:15000});
    let state = await snapshot();
    assert.equal(state.armed, true);
    assert.equal(state.pick_place_config.pick_down, 12, 'click saves the edited parameters');
    assert.equal(state.pick_place_config.d_second_drop_down, 8, 'second-D depth is saved independently');
    assert.equal(state.auxiliary['6'].min, '-2950', 'click applies the saved rotation range');
    assert.equal(state.lift.down_sign, '-1');
    assert.equal(state.pick_place.context.color_id, 1);
    assert.equal(state.auxiliary['5'].pos, '0'); assert.equal(state.auxiliary['6'].pos, '0');
    const previousJob = state.pick_place.job_id;
    await page.locator('#pick-place-place').click();
    await page.waitForFunction(async previous => {
      const s = (await (await fetch('/api/state')).json()).pick_place;
      return s.job_id !== previous && s.state === 'done';
    }, previousJob, {timeout:15000});
    state = await snapshot();
    assert.equal(state.pick_place.context.color_id, 6);
    assert.equal(state.pick_place.context.target, 2);
    // Manual tests use calibrated positions; they must never wait for an
    // external vision producer or request chassis alignment.
    assert.equal(await page.locator('#pick-place-vision').count(), 0);
    const directRequest = page.waitForRequest(r => r.url().endsWith('/api/pick-place/action'));
    await page.locator('#pick-place-pick').click();
    assert.equal((await directRequest).postDataJSON().require_vision, false);
    await page.waitForFunction(() => document.getElementById('pick-place-state').textContent === '运行中');
    assert.equal(await page.locator('#pick-place-save').isDisabled(), true);
    await page.locator('#pick-place-stop').click();
    await page.waitForFunction(() => document.getElementById('pick-place-state').textContent === '已停止');
    assert.equal((await snapshot()).pick_place.active, false);
    // The B button must use its independent profile even when the generic
    // parameter form contains an invalid, unsaved draft.
    await page.locator('[data-tab="auxiliary"]').click();
    for (const [key,value] of Object.entries({pick_down:14,pick_rotation:-21,lift_sps:500,rotation_sps:550,s1_closed:82,s1_settle_ms:100,s2_settle_ms:100})) {
      await page.locator(`#b-pickup-${key}`).fill(String(value));
    }
    await page.locator('#b-pickup-save').click();
    await page.waitForFunction(async () => (await (await fetch('/api/state')).json()).b_pickup_config.pick_down === 14);
    await page.locator('#pick-place-pick_down').fill('-1');
    await page.locator('#pick-place-code').fill('321+123+546+231');
    const bRequest = page.waitForRequest(r => r.url().endsWith('/api/pick-place/action'));
    await page.locator('#pick-place-b-pick').click();
    const bPayload = (await bRequest).postDataJSON();
    assert.equal(bPayload.action, 'pick');
    assert.equal(bPayload.profile, 'b_pickup');
    assert.equal(bPayload.require_vision, false);
    await page.waitForFunction(() => document.getElementById('pick-place-state').textContent === '等待摄像头1启动');
    await page.waitForFunction(() => document.getElementById('pick-place-state').textContent === 'B点等待目标颜色', null, {timeout:15000});
    await page.waitForTimeout(3500);
    let bState = await snapshot();
    assert.equal(bState.pick_place.state, 'pickup_wait', 'missing frames keep the lowered gripper waiting');
    assert.equal(bState.pick_place.context.color, 'blue');
    assert.match(await page.locator('#pick-place-detail').textContent(), /blue/);
    assert.match(await page.locator('#pick-place-request').textContent(), /无等待超时/);
    assert.equal(bState.pick_place.context.profile, 'b_pickup');
    assert.equal(bState.b_pickup_config.pick_down, 14);
    assert.equal(bState.pick_place_config.pick_down, 12, 'B test does not save generic edits');
    assert.equal(await page.locator('#pick-place-pick_down').inputValue(), '-1', 'B test leaves the generic draft untouched');
    await page.locator('#pick-place-stop').click();
    await page.waitForFunction(() => document.getElementById('pick-place-state').textContent === '已停止');
    assert.equal((await snapshot()).camera_alignment.active, false);
    // Restore production defaults in this temporary fixture for review images.
    await page.locator('#pick-place-pick_down').fill('12');
    for (const [key,value] of Object.entries({pick_down:9200,pick_rotation:-1750,place_take_down:9200,place_rotation:-1750,place_drop_down:9200,d_second_drop_down:8,s1_settle_ms:600,s2_settle_ms:1000})) {
      await page.locator(`#pick-place-${key}`).fill(String(value));
    }
    await page.locator('#pick-place-save').click();
    await page.waitForFunction(async () => (await (await fetch('/api/state')).json()).pick_place_config.place_drop_down === 9200);
    await page.reload();
    await page.locator('[data-tab="auxiliary"]').click();
    await page.waitForFunction(() => document.getElementById('pick-place-d_second_drop_down').value === '8');
    fs.mkdirSync(path.join(__dirname,'../tmp'),{recursive:true});
    for (const width of [1440,390]) {
      await page.setViewportSize({width,height:1000});
      await page.locator('#pick-place-card').evaluate(node => node.scrollIntoView({block:'start'}));
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false);
      assert.equal(await page.locator('.stage').evaluate(node => node.scrollWidth > node.clientWidth), false);
      await page.screenshot({path:path.join(__dirname,`../tmp/pick_place_${width}.png`)});
    }
    assert.deepEqual(errors, []);
    console.log('Pick/place browser OK: generic cycles; B camera startup, blue target, unbounded frame wait and stop; desktop/mobile.');
  } finally { await browser.close(); }
})().catch(error => {console.error(error);process.exitCode=1;});
