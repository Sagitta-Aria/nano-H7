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
    await page.goto(process.env.NAV_URL || 'http://127.0.0.1:8770/');
    await page.getByRole('tab', { name: '坐标巡航' }).click();
    await page.waitForFunction(() => !document.getElementById('nav-origin').disabled);
    const initial = await snapshot();
    assert.equal(initial.demo,true,'browser QA requires SIM');
    original = initial.mission;
    await configure({ points: { O:[0,0], A:[0,14], B:[14,0], C:[14,28], D:[28,14] },
      grid:{x:[0,14,28],y:[0,14,28]},
      headings:{O:0,A:90,B:-90.25,C:180,D:45}, sps:500, accel:2000, dwell_ms:3000 });
    await page.waitForFunction(() => document.querySelectorAll('[data-point="A"] input')[1].value==='14');
    await page.locator('#nav-origin').click();
    await page.waitForFunction(() => document.getElementById('nav-position').textContent==='(0, 0)');
    await page.locator('#btn-arm').click();
    await page.waitForFunction(() => !document.getElementById('nav-start').disabled);
    await page.locator('#nav-start').click();
    await page.waitForFunction(() => document.getElementById('nav-state').textContent==='到点停留');
    await page.screenshot({ path: path.join(__dirname, '../tmp/navigation_running.png'), fullPage:true });
    // Telemetry comes from the simulator running the mission; the browser
    // issues no per-segment motion and never synthesizes an arrival.
    await page.waitForFunction(() => document.getElementById('nav-state').textContent==='巡航完成',null,{timeout:60000});
    const finished = await snapshot();
    assert.deepEqual([finished.navigation.x,finished.navigation.y],['0','14']);
    assert.equal(await page.locator('#nav-state').textContent(), '巡航完成');
    assert.equal(finished.mission.headings.B, -90.25);
    assert.ok(Math.abs(Number(finished.navigation.heading_cd) - 9000) <= 20);
    assert.equal(finished.navigation.arrival_cd, '9000');
    assert.equal(finished.navigation.planner, 'host_grid');
    assert.equal(await page.locator('#nav-grid-x1').inputValue(), '14');
    assert.equal(await page.locator('[data-point="B"] input').nth(2).inputValue(), '-90.25');
    await page.locator('#btn-stop').click();
    await page.waitForFunction(() => document.getElementById('m-armed').textContent==='已关闭');
    await configure(original);
    await page.reload();
    await page.getByRole('tab', { name:'坐标巡航' }).click();
    await page.waitForFunction(() => document.querySelectorAll('[data-point="C"] input')[1].value==='28000');
    assert.equal(await page.locator('[data-point="C"] input').nth(1).inputValue(), '28000');
    await page.screenshot({ path:path.join(__dirname,'../tmp/navigation_desktop.png'), fullPage:true });
    for (const [width,height] of [[390,844],[760,900]]) {
      await page.setViewportSize({width,height});
      await page.screenshot({path:path.join(__dirname,`../tmp/navigation_${width}.png`),fullPage:true});
      const layout = await page.evaluate(() => {
        const canvas = document.getElementById('nav-map');
        return { overflow:document.documentElement.scrollWidth > innerWidth,
          pixels:canvas.getContext('2d').getImageData(0,0,620,440).data.filter((value,index)=>index%4===3&&value>0).length,
          hiddenControls:[...document.querySelectorAll('#tab-navigation button')].some(button=>{
            const rect=button.getBoundingClientRect(); return rect.left<0||rect.right>innerWidth;
          }) };
      });
      assert.equal(layout.overflow,false,`horizontal overflow at ${width}`);
      assert.equal(layout.hiddenControls,false,`controls clipped at ${width}`);
      assert.ok(layout.pixels>1000,'route canvas must be nonblank');
    }
    await page.locator('[data-point="A"] input').nth(1).fill('15000');
    await page.waitForTimeout(700);
    assert.equal(await page.locator('[data-point="A"] input').nth(1).inputValue(),'15000',
      'telemetry snapshots must preserve unsaved edits');
    assert.deepEqual(errors,[]);
    console.log('Navigation browser OK: save/reload, origin, arm, full mission, stop, desktop/mobile canvas and layout.');
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
