/* Exercise complete UI -> backend -> SIM -> confirmed telemetry chains. */
const {chromium}=require('playwright');
const assert=require('node:assert/strict');
(async()=>{
  const browser=await chromium.launch({channel:'msedge',headless:true});
  const page=await browser.newPage({viewport:{width:1440,height:1000}});
  const errors=[]; page.on('pageerror',e=>errors.push(e.message));
  let token;
  page.on('request',r=>{if(r.url().includes('/api/events?'))token=new URL(r.url()).searchParams.get('token');});
  const snapshot=()=>page.evaluate(async()=>(await fetch('/api/state')).json());
  const stateUntil=async predicate=>{
    for(let attempt=0;attempt<300;attempt++) {
      const s=await snapshot(); if(predicate(s))return s;
      await page.waitForTimeout(100);
    }
    throw Error('Confirmed board state did not arrive before timeout');
  };
  const command=body=>page.evaluate(async({body,token})=>{
    const r=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json','X-Session':token},body:JSON.stringify({body})});
    const result=await r.json(); if(!result.ok)throw Error(JSON.stringify(result));
  },{body,token});
  try {
    await page.goto(new URL('/debug', process.env.CONTROL_URL||'http://127.0.0.1:18774/').href);
    await page.waitForFunction(()=>document.getElementById('m-armed').textContent==='已关闭');
    assert.equal((await snapshot()).demo,true,'only use the SIM fixture');
    await page.getByRole('tab',{name:'舵机',exact:true}).click();
    const s1=page.locator('.servo-card[data-ch="1"]'),s2=page.locator('.servo-card[data-ch="2"]');
    assert.equal(await s1.locator('.angle-input').inputValue(),'120');
    assert.equal(await s2.locator('.angle-input').inputValue(),'0');
    await s2.locator('[data-deg="270"]').click();
    await page.waitForFunction(()=>document.querySelector('.servo-card[data-ch="2"] .angle').textContent==='270.0');
    let s=await stateUntil(s=>/ch=2 angle=180.0 us=2500 on=1/.test(s.servo.raw));
    await s1.locator('[data-deg="180"]').click();
    await page.waitForFunction(()=>document.querySelector('.servo-card[data-ch="1"] .angle').textContent==='180.0');
    s=await stateUntil(s=>/ch=1 angle=270.0 us=2500 on=1/.test(s.servo.raw)); assert.equal(s.device.servo1_travel,'2700');
    await page.getByRole('tab',{name:'底盘运动'}).click();
    await page.locator('#turn-kp').fill('40.25');await page.locator('#turn-tolerance').fill('0.20');await page.locator('#turn-configure').click();
    await page.waitForFunction(()=>document.getElementById('turn-live').textContent.includes('P=40.25'));
    s=await snapshot();assert.equal(s.turn.kp_centi,'4025');assert.equal(s.turn.tolerance_cd,'20');
    await page.locator('#btn-arm').click(); await page.locator('#turn-mode').selectOption('REL');
    await page.locator('#turn-angle').fill('90');await page.locator('#turn-sps').fill('1200');
    const previousMove=(await snapshot()).turn.move_id;
    await page.locator('#turn-start').click();
    const confirmed=(await stateUntil(s=>s.turn.move_id!==previousMove && s.turn.state==='done')).turn;
    assert.notEqual(confirmed.move_id,previousMove); assert.ok(Math.abs(Number(confirmed.error_cd))<=20);
    for(const width of [1440,390]) {
      await page.setViewportSize({width,height:1000});
      await page.screenshot({path:require('node:path').join(__dirname,`../tmp/control_fixes_${width}.png`),fullPage:true});
      assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false,`overflow at ${width}`);
    }
    assert.deepEqual(errors,[]);
    console.log('Control fixes browser OK: S1 PH6/180, S2 PE4/270, applied P40.25 and closed-loop completion <=0.2 degrees.');
  }finally{if(token)await command('STOP').catch(()=>{});await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
