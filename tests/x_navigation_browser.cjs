/* Real saved start selection/reference on the local SIM launcher only. */
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
(async () => {
  const browser = await chromium.launch({channel:'msedge',headless:true});
  const page = await browser.newPage({viewport:{width:1440,height:1000}});
  const errors = []; let token;
  page.on('pageerror', e => errors.push(e.message));
  page.on('request', r => {if(r.url().includes('/api/events?')) token = new URL(r.url()).searchParams.get('token');});
  const snapshot = () => page.evaluate(async () => (await fetch('/api/state')).json());
  const post = (url,data) => page.evaluate(async ({url,data,token}) => {
    const result = await (await fetch(url,{method:'POST',headers:{'Content-Type':'application/json','X-Session':token},body:JSON.stringify(data)})).json();
    if(!result.ok) throw Error(JSON.stringify(result)); return result;
  }, {url,data,token});
  const until = async predicate => {
    const deadline=Date.now()+60000;
    while(Date.now()<deadline) {if(await predicate()) return; await new Promise(r=>setTimeout(r,100));}
    const s=await snapshot(); throw Error(JSON.stringify({nav:s.navigation,runner:s.navigation_runner,log:s.log?.slice(-4)}));
  };
  try {
    await page.goto(new URL('/debug',process.env.NAV_URL).href);
    await page.getByRole('tab',{name:'坐标巡航'}).click();
    await page.waitForFunction(()=>!document.getElementById('nav-origin').disabled);
    const before=await snapshot(); assert.equal(before.demo,true);
    const config=before.mission;
    config.stations=config.stations.filter(s=>s.name!=='X');
    config.start_station=0; config.route=[{station:1,hold:0}];
    config.sps=5000; config.accel=20000; config.dwell_ms=0; config.turn_inset_steps=100;
    await post('/api/navigation/config',{config});
    await page.waitForFunction(()=>document.querySelectorAll('#nav-points tbody tr').length===5);
    await page.locator('#nav-add-x').click();
    assert.equal(await page.locator('#nav-points tbody tr').count(),6);
    await page.locator('#nav-start-station').selectOption('5');
    assert.equal(await page.locator('#nav-origin').textContent(),'当前位置设为 X');
    await page.locator('#nav-save').click();
    await until(async () => (await snapshot()).mission.start_station===5);
    const saved=await snapshot();
    assert.deepEqual(saved.mission.stations[5],{name:'X',x:0,y:28800,heading:-180,arrival_enabled:true});
    assert.deepEqual(saved.mission.route,[{station:1,hold:0}]);
    assert.equal(saved.navigation.x,before.navigation.x);
    assert.equal(saved.navigation.y,before.navigation.y);
    await page.reload(); await page.getByRole('tab',{name:'坐标巡航'}).click();
    await page.waitForFunction(()=>document.getElementById('nav-start-station').value==='5');
    await page.locator('#nav-origin').click();
    await until(async () => (await snapshot()).navigation.y==='28800');
    await post('/api/command',{body:'ARM 1'});
    await page.waitForFunction(()=>!document.getElementById('nav-start').disabled);
    await page.locator('#nav-start').click();
    await until(async () => (await snapshot()).navigation_runner.state==='done');
    const end=await snapshot();
    assert.equal(end.navigation.x,'0'); assert.equal(end.navigation.y,'14000');
    assert.equal(end.navigation.target_cd,'0'); assert.equal(end.navigation.fault,'none');
    await post('/api/command',{body:'STOP'});
    for(const width of [760,390]) {
      await page.setViewportSize({width,height:1000});
      assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
    }
    assert.deepEqual(errors,[]);
    console.log('X browser OK: add, saved selection, reload, reference, direct A arrival, narrow layout; no page errors.');
  } finally {await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
