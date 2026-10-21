/* Live browser QA against a demo-only server. Restores saved parameters.
 * NODE_PATH may point to the bundled Playwright packages. No hardware ports. */
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const path = require('node:path');

(async () => {
  const browser = await chromium.launch({ channel: 'msedge', headless: true });
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  const errors = [];
  let token;
  page.on('request', request => {
    if (request.url().includes('/api/events?')) token = new URL(request.url()).searchParams.get('token');
  });
  page.on('pageerror', error => errors.push(error.message));
  const snapshot = () => page.evaluate(async () => (await fetch('/api/state')).json());
  const configure = config => page.evaluate(async ({config,token}) => {
    const response = await fetch('/api/navigation/config',{method:'POST',
      headers:{'Content-Type':'application/json','X-Session':token},body:JSON.stringify({config})});
    const result=await response.json(); if(!result.ok) throw Error(JSON.stringify(result));
  },{config,token});
  let original;
  try {
    await page.goto(new URL('/debug', process.env.NAV_URL || 'http://127.0.0.1:8770/').href);
    await page.getByRole('tab', { name: '坐标巡航' }).click();
    await page.waitForFunction(() => !document.getElementById('nav-origin').disabled);
    const initial = await snapshot();
    assert.equal(initial.demo,true,'browser QA requires SIM');
    original = initial.mission;
    assert.ok(original.stations.every(station => station.arrival_enabled === false));
    assert.equal(await page.locator('#nav-points .nav-arrival-enabled:checked').count(),0);
    // Stations carry their own name and the route is an explicit leg list, so
    // row order in the table is what the route indices refer to.
    await configure({ stations:[{name:'O',x:0,y:0,heading:0},{name:'A',x:0,y:14,heading:90},
        {name:'B',x:14,y:0,heading:-90.25,arrival_enabled:true},{name:'C',x:14,y:28,heading:180},
        {name:'D',x:28,y:14,heading:45}],
      route:[{station:1,hold:0},{station:2,hold:0},{station:3,hold:0},
             {station:4,hold:0},{station:1,hold:0,arrival_enabled:true,heading:0}],
      grid:{x:[0,14,28],y:[0,14,28]}, sps:500, accel:2000, dwell_ms:3000,
      turn_inset_enabled:true,turn_inset_steps:4 });
    await page.waitForFunction(() => document.querySelectorAll('[data-station="1"] input')[2].value==='14');
    assert.equal(await page.locator('#nav-points .nav-arrival-enabled:checked').count(),1);
    assert.equal(await page.locator('#nav-route .nav-arrival-mode').nth(4).inputValue(),'on');
    assert.equal(await page.locator('#nav-route .nav-arrival-angle').nth(4).inputValue(),'0');
    // Save through the actual form to distinguish explicit zero from disabled.
    await page.locator('#nav-save').click();
    await page.waitForFunction(async () => (await (await fetch('/api/state')).json()).mission.route[4].heading===0);
    await page.locator('#nav-origin').click();
    await page.waitForFunction(() => document.getElementById('nav-position').textContent==='(0, 0)');
    await page.locator('#btn-arm').click();
    await page.waitForFunction(() => !document.getElementById('nav-start').disabled);
    await page.locator('#nav-start').click();
    await page.waitForFunction(() => document.getElementById('nav-state').textContent.startsWith('到点停留'));
    const firstArrival = await snapshot();
    assert.equal(firstArrival.navigation.leg,'1');
    assert.equal(firstArrival.navigation.arrival_cd,'0','disabled A retains incoming heading');
    assert.ok(Math.abs(Number(firstArrival.navigation.heading_cd))<=20);
    await page.screenshot({ path: path.join(__dirname, '../tmp/navigation_running.png'), fullPage:true });
    // Telemetry comes from the simulator running the mission; the browser
    // issues no per-segment motion and never synthesizes an arrival.
    await page.waitForFunction(() => document.getElementById('nav-state').textContent==='巡航完成',null,{timeout:60000}).catch(async error => {
      const current = await snapshot();
      throw new Error(`${error.message}; navigation=${JSON.stringify(current.navigation)}; runner=${JSON.stringify(current.navigation_runner)}`);
    });
    const finished = await snapshot();
    assert.deepEqual([finished.navigation.x,finished.navigation.y],['0','14']);
    assert.equal(await page.locator('#nav-state').textContent(), '巡航完成');
    assert.equal(finished.mission.stations[2].heading, -90.25);
    assert.ok(Math.abs(Number(finished.navigation.heading_cd)) <= 20);
    assert.equal(finished.navigation.arrival_cd, '0','second A visit explicitly turns to zero');
    assert.equal(finished.mission.stations[1].arrival_enabled,false);
    assert.equal(finished.mission.route[4].arrival_enabled,true);
    assert.equal(finished.navigation.planner, 'host_grid');
    assert.equal(await page.locator('#nav-grid-x1').inputValue(), '14');
    assert.equal(await page.locator('[data-station="2"] input').nth(3).inputValue(), '-90.25');
    await page.locator('#btn-stop').click();
    await page.waitForFunction(() => document.getElementById('m-armed').textContent==='已关闭');
    await configure(original);
    await page.reload();
    await page.getByRole('tab', { name:'坐标巡航' }).click();
    await page.waitForFunction(() => document.querySelectorAll('[data-station="3"] input')[2].value==='28000');
    assert.equal(await page.locator('[data-station="3"] input').nth(2).inputValue(), '28000');
    await page.screenshot({ path:path.join(__dirname,'../tmp/navigation_desktop.png'), fullPage:true });
    for (const [width,height] of [[390,844],[760,900]]) {
      await page.setViewportSize({width,height});
      await page.screenshot({path:path.join(__dirname,`../tmp/navigation_${width}.png`),fullPage:true});
      const layout = await page.evaluate(() => {
        return { overflow:document.documentElement.scrollWidth > innerWidth,
          maps:document.querySelectorAll('#nav-map').length,
          hiddenControls:[...document.querySelectorAll('#tab-navigation button')].some(button=>{
            const rect=button.getBoundingClientRect(); return rect.left<0||rect.right>innerWidth;
          }) };
      });
      assert.equal(layout.overflow,false,`horizontal overflow at ${width}`);
      assert.equal(layout.hiddenControls,false,`controls clipped at ${width}`);
      assert.equal(layout.maps,0,'coordinate editor has no draft map');
    }
    await page.locator('[data-station="1"] input').nth(2).fill('15000');
    await page.waitForTimeout(700);
    assert.equal(await page.locator('[data-station="1"] input').nth(2).inputValue(),'15000',
      'telemetry snapshots must preserve unsaved edits');
    // Structural edits: a route longer than the station count, with a station
    // visited more than once, is the case the fixed five-leg schema could not
    // express. Adding a leg must not discard the unsaved coordinate above.
    const legsBefore = await page.locator('#nav-route tbody tr').count();
    await page.locator('#nav-leg-add').click();
    assert.equal(await page.locator('#nav-route tbody tr').count(), legsBefore + 1);
    assert.equal(await page.locator('[data-station="1"] input').nth(2).inputValue(), '15000',
      'adding a leg must preserve unsaved station edits');
    await page.locator('#nav-route tbody tr').nth(legsBefore).locator('select').first().selectOption('2');
    await page.locator('#nav-route tbody tr').nth(legsBefore).locator('input[type=checkbox]').check();
    const stationsBefore = await page.locator('#nav-points tbody tr').count();
    await page.locator('#nav-station-add').click();
    assert.equal(await page.locator('#nav-points tbody tr').count(), stationsBefore + 1);
    await page.locator('#nav-station-remove').click();
    assert.equal(await page.locator('#nav-points tbody tr').count(), stationsBefore);
    await page.locator('#nav-leg-remove').click();
    assert.equal(await page.locator('#nav-route tbody tr').count(), legsBefore);
    assert.ok((await page.locator('#nav-route-tag').textContent()).includes('→'),
      'route tag must show the configured order');
    assert.deepEqual(errors,[]);
    console.log('Navigation browser OK: disabled and repeated arrivals, explicit zero, NX inset execution, save/reload, stop, station/route editing and desktop/mobile layout.');
  } finally {
    if (original) {
      await page.evaluate(async token => {
        await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json','X-Session':token},body:JSON.stringify({body:'STOP'})});
      },token).catch(()=>{});
      await configure(original).catch(()=>{});
    }
    await browser.close();
  }
})().catch(error=>{ console.error(error); process.exitCode=1; });
