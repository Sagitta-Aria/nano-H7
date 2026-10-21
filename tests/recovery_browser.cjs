/* Real UI + serial worker + SIM protocol. Launched only by recovery_browser.py. */
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

(async () => {
  const browser = await chromium.launch({channel:'msedge',headless:true});
  const context = await browser.newContext({viewport:{width:1440,height:1000}});
  const page = await context.newPage(), errors = [], checks = [];
  let token;
  page.on('request', r => {if(r.url().includes('/api/events?'))token=new URL(r.url()).searchParams.get('token');});
  page.on('pageerror', e => errors.push(e.message));
  const snapshot = () => page.evaluate(async () => (await fetch('/api/state')).json());
  const post = (url,data={}) => page.evaluate(async ({url,data,token}) => {
    const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json','X-Session':token},body:JSON.stringify(data)});
    const value=await r.json();if(!value.ok)throw Error(JSON.stringify(value));return value;
  },{url,data,token});
  const until = async predicate => {
    const end=Date.now()+20000;
    while(Date.now()<end){if(await predicate())return;await new Promise(r=>setTimeout(r,100));}
    throw Error('timeout: '+JSON.stringify(await snapshot()));
  };
  try {
    await page.goto(new URL('/debug', process.env.NAV_URL).href);
    await until(async () => (await snapshot()).motion?.v==='1');
    await page.getByRole('tab',{name:'坐标巡航'}).click();
    const s=await snapshot(), nav=s.mission, task=s.task_config;
    nav.stations.forEach(station=>{station.x/=1000;station.y/=1000;station.heading=0;});
    for(const k of ['x','y'])nav.grid[k]=nav.grid[k].map(v=>v/1000);
    nav.route=[{station:1,hold:1}];nav.dwell_ms=0;
    task.legs={'1':[{op:'wait',ms:30000}]};
    await post('/api/navigation/config',{config:nav});
    await post('/api/mission/config',{config:task});
    await post('/api/navigation/action',{action:'origin'});
    await until(async () => (await snapshot()).navigation.ready==='1');
    await post('/api/command',{body:'ARM 1'});
    await until(async () => (await snapshot()).armed);
    assert.equal(await page.locator('#btn-recover').isDisabled(),true);
    await post('/api/mission/action',{action:'start'});
    await until(async () => (await snapshot()).task.state==='wait');
    await post('/api/test/fault');
    await until(async () => {const s=await snapshot();return s.motion.latched==='1'&&!s.armed&&!s.task.active;});
    await until(async () => (await page.locator('#motion-status').textContent()).includes('步进定时超期'));
    await until(async () => page.locator('#btn-recover').isEnabled());
    checks.push('motor fault cancels task, disarms and shows actionable cause');
    const observer=await context.newPage();
    await observer.goto(new URL('/debug', process.env.NAV_URL).href);
    await observer.waitForFunction(()=>document.querySelector('#motion-status').textContent.includes('步进定时超期'));
    assert.equal(await observer.locator('#btn-recover').isDisabled(),true);
    await observer.close();
    checks.push('readonly and reload snapshots retain fault and permissions');
    await page.locator('#btn-recover').click();
    await until(async () => {const s=await snapshot();return s.motion.latched==='0'&&s.navigation.ready==='0';});
    const recovered=await snapshot();
    assert.equal(recovered.armed,false);assert.equal(recovered.task.active,false);
    assert.equal(recovered.motion.total,'1');
    assert.equal(await page.locator('#task-start').isDisabled(),true);
    checks.push('explicit reset preserves history, invalidates pose and never resumes');
    await post('/api/navigation/action',{action:'origin'});
    await until(async () => (await snapshot()).navigation.ready==='1');
    await post('/api/command',{body:'ARM 1'});
    await until(async () => page.locator('#task-start').isEnabled());
    await page.locator('#task-start').click();
    await until(async () => (await snapshot()).task.state==='wait');
    await page.locator('#btn-stop').click();
    await until(async () => !(await snapshot()).armed);
    checks.push('operator can start a new task after reference and explicit arm');
    await page.getByRole('tab',{name:'底盘运动'}).click();
    await post('/api/command',{body:'ARM 1'});
    await until(async () => (await snapshot()).armed);
    await page.locator('body').click({position:{x:2,y:2}});
    await page.keyboard.down('ArrowUp');
    await until(async () => page.locator('.pad[data-dir="FWD"]').evaluate(n=>n.classList.contains('live')));
    await until(async () => page.locator('.pad[data-dir="FWD"]').evaluate(n=>!n.classList.contains('live')));
    assert.equal((await snapshot()).armed,true,'expired jog keeps normal ARM policy');
    await page.keyboard.up('ArrowUp');
    await page.keyboard.down('ArrowUp');
    await post('/api/test/fault');
    await until(async () => (await snapshot()).motion.latched==='1');
    await until(async () => page.locator('.pad[data-dir="FWD"]').evaluate(n=>!n.classList.contains('live')));
    await page.keyboard.up('ArrowUp');
    await until(async () => page.locator('#btn-recover').isEnabled());
    await page.locator('#btn-recover').click();
    await until(async () => (await snapshot()).motion.latched==='0');
    await post('/api/command',{body:'ARM 1'});
    await until(async () => page.locator('#ch-move').isEnabled());
    await page.locator('#btn-stop').click();
    checks.push('expired and faulted jogs release local busy; manual move becomes available');
    for(const width of [1440,760,390]){
      await page.setViewportSize({width,height:1000});
      assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false,`overflow ${width}`);
    }
    checks.push('recovery controls fit desktop and 760/390px layouts');
    assert.deepEqual(errors,[]);
    const report={ok:true,checks,pageErrors:errors};
    fs.mkdirSync(path.join(__dirname,'../tmp'),{recursive:true});
    await page.screenshot({path:path.join(__dirname,'../tmp/recovery_390.png'),fullPage:true});
    fs.writeFileSync(path.join(__dirname,'../tmp/recovery_browser_validation.json'),JSON.stringify(report,null,2));
    console.log(JSON.stringify(report,null,2));
  } finally {await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
