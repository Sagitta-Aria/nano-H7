/* Coordinate edits and reload through the real Web API, on isolated SIM only. */
const {chromium} = require('playwright');
const assert = require('node:assert/strict');

(async () => {
  const browser = await chromium.launch({channel:'msedge',headless:true});
  const page = await browser.newPage({viewport:{width:1440,height:1000}}), errors=[];
  let token;
  page.on('pageerror', error => errors.push(error.message));
  page.on('request', request => {
    if (request.url().includes('/api/events?')) token = new URL(request.url()).searchParams.get('token');
  });
  const state = () => page.evaluate(async () => (await fetch('/api/state')).json());
  const open = async () => {
    await page.goto(new URL('/debug',process.env.NAV_URL).href);
    await page.getByRole('tab',{name:'坐标巡航',exact:true}).click();
    await page.waitForFunction(() => document.querySelectorAll('#nav-points tbody tr').length===5);
  };
  const save = async expected => {
    await page.locator('#nav-save').click();
    await page.waitForFunction(expected);
  };
  try {
    await open();
    assert.equal(await page.locator('#nav-map, #nav-map-caption, #nav-field-apply').count(),0);
    assert.equal(await page.locator('#nav-field-remap').isChecked(),false);
    const row = page.locator('[data-station="1"] input');
    await row.nth(1).fill('123');
    await row.nth(2).fill('456');
    await page.waitForTimeout(500);
    assert.equal(await row.nth(1).inputValue(),'123');
    await save(async () => (await fetch('/api/state').then(r=>r.json())).mission.stations[1].x===123);
    await open();
    assert.equal(await row.nth(1).inputValue(),'123');
    assert.equal(await row.nth(2).inputValue(),'456');
    // Saving dimensions never requires applying a preview, and preserves XYZ.
    await page.locator('#nav-field-width').fill('30000');
    await save(async () => (await fetch('/api/state').then(r=>r.json())).mission.grid.x[2]===30000);
    assert.equal((await state()).mission.stations[1].x,123);
    assert.equal((await state()).mission.stations[1].y,456);
    // Invalid execution is rejected before RESET, ARM or route upload.
    const result = await page.evaluate(async token => (await fetch('/api/navigation/action',{
      method:'POST',headers:{'Content-Type':'application/json','X-Session':token},body:JSON.stringify({action:'origin'})
    })).json(),token);
    assert.equal(result.ok,false);
    assert.match(result.message || result.error || JSON.stringify(result),/通道/);
    assert.equal((await state()).armed,false);
    // Structural edits preserve off-lane coordinates and consume field remap once.
    await page.locator('#nav-field-remap').check();
    await page.locator('#nav-field-width').fill('32000');
    await page.locator('#nav-leg-add').click();
    await page.locator('#nav-save').click();
    await page.waitForFunction(async () => (await fetch('/api/state').then(r=>r.json())).mission.grid.x[2]===32000);
    assert.equal((await state()).mission.stations[1].x,123);
    await row.nth(1).fill('');
    await page.locator('#nav-save').click();
    await page.waitForFunction(() => document.getElementById('status').textContent.includes('整数'));
    assert.equal((await state()).mission.stations[1].x,123);
    await row.nth(1).fill('0'); await row.nth(2).fill('14000');
    await save(async () => (await fetch('/api/state').then(r=>r.json())).mission.stations[1].x===0);
    await open();
    assert.equal(await row.nth(2).inputValue(),'14000');
    for (const width of [390,760,1440]) {
      await page.setViewportSize({width,height:1000});
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth>innerWidth),false);
    }
    assert.deepEqual(errors,[]);
    console.log('Coordinate editing browser OK: no map/apply gate, off-lane save/reload, direct dimensions, single remap, invalid execution and responsive layout.');
  } finally { await browser.close(); }
})().catch(error => {console.error(error);process.exit(1);});
