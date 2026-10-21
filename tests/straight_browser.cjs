/* Actual Web controls against isolated SIM. Never run against real hardware. */
const {chromium}=require('playwright');
const assert=require('node:assert/strict');
const path=require('node:path');
(async()=>{
  const browser=await chromium.launch({channel:'msedge',headless:true});
  const page=await browser.newPage({viewport:{width:1440,height:1000}});
  const errors=[],sent=[];let token;
  page.on('pageerror',error=>errors.push(error.message));
  page.on('request',r=>{
    if(r.url().includes('/api/events?')) token=new URL(r.url()).searchParams.get('token');
    if(r.url().endsWith('/api/command')) sent.push(r.postDataJSON().body);
  });
  const snapshot=()=>page.evaluate(async()=>(await fetch('/api/state')).json());
  const until=async(predicate)=>{
    for(let i=0;i<100;i++) {const value=await snapshot();if(predicate(value))return value;await page.waitForTimeout(100);}
    throw Error('timed out waiting for board state');
  };
  try {
    await page.goto(new URL('/debug', process.env.STRAIGHT_URL||'http://127.0.0.1:18774/').href);
    const initial=await until(s=>s.device.straight==='1'&&s.turn.imu==='1');
    assert.equal(initial.demo,true,'requires isolated SIM fixture');
    await page.getByRole('tab',{name:'底盘运动',exact:true}).click();
    await page.locator('.pad[data-dir="FWD"]').click(); // disarmed selection
    await page.locator('#ch-assist').check();
    assert.equal(await page.locator('#ch-move').isDisabled(),true);
    await page.locator('#btn-arm').click();
    await page.waitForFunction(()=>!document.getElementById('ch-move').disabled);
    await page.locator('#ch-steps').fill('1000');
    await page.locator('#ch-move').click();
    const moving=await until(s=>s.chassis.state==='moving'&&s.chassis.assist==='1');
    assert.ok(sent.includes('CHASSIS STRAIGHT FWD 1000 500 2000'));
    assert.equal(await page.locator('#ch-move').isDisabled(),true,'busy move cannot be queued twice');
    assert.match(await page.locator('#ch-assist-state').textContent(),/锁定航向/);
    const done=await until(s=>s.chassis.move_id===moving.chassis.move_id&&s.chassis.state==='done');
    assert.equal(done.chassis.remaining,'0');
    assert.equal(done.chassis.assist,'1');
    await page.evaluate(()=>document.querySelector('.pad[data-dir="CW"]').click());
    assert.equal(await page.locator('#ch-move').isDisabled(),true,'assisted rotation unsupported');
    await page.locator('#ch-assist').uncheck();
    assert.equal(await page.locator('#ch-move').isDisabled(),false,'ordinary rotation remains available');
    await page.evaluate(()=>document.querySelector('.pad[data-dir="BACK"]').click());
    await page.locator('#ch-steps').fill('7');
    await page.locator('#ch-move').click();
    assert.ok(sent.includes('CHASSIS MOVE BACK 7 500 2000'));
    await until(s=>s.chassis.move_id!==done.chassis.move_id&&s.chassis.state==='done');
    await page.locator('#btn-stop').click();
    await until(s=>!s.armed);
    for(const width of [1440,390]) {
      await page.setViewportSize({width,height:1000});
      await page.screenshot({path:path.join(__dirname,`../tmp/straight_${width}.png`),fullPage:true});
      assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
    }
    await page.getByRole('tab',{name:'坐标巡航',exact:true}).click();
    assert.match(await page.locator('#nav-assist-state').textContent(),/自动进行陀螺仪纠偏/);
    assert.deepEqual(errors,[]);
    console.log('Straight browser OK: actual STRAIGHT command, heading readback, busy guard, rotation guard, ordinary move, STOP, cruise indicator, desktop/mobile.');
  }finally {
    if(token) await page.evaluate(async token=>fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json','X-Session':token},body:JSON.stringify({body:'STOP'})}),token).catch(()=>{});
    await browser.close();
  }
})().catch(e=>{console.error(e);process.exitCode=1;});
