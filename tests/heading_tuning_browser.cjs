/* Real page/API/SIM checks: one gyro tuning set shared across three views. */
const {chromium}=require('playwright');
const assert=require('node:assert/strict');
(async()=>{
  const browser=await chromium.launch({channel:'msedge',headless:true});
  const page=await browser.newPage({viewport:{width:1440,height:1000}});
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  let token;page.on('request',r=>{if(r.url().includes('/api/events?'))token=new URL(r.url()).searchParams.get('token');});
  const snapshot=()=>page.evaluate(async()=>(await fetch('/api/state')).json());
  try {
    await page.goto(new URL('/debug', process.env.NAV_URL || 'http://127.0.0.1:18774/').href);
    await page.waitForFunction(()=>document.getElementById('m-armed').textContent==='已关闭');
    assert.equal((await snapshot()).demo,true,'requires SIM fixture');
    await page.getByRole('tab',{name:'陀螺仪 · H723',exact:true}).click();
    await page.locator('#imu-turn-kp').fill('80');await page.locator('#imu-turn-tolerance').fill('0.50');
    await page.locator('#imu-turn-min-sps').fill('350');
    await page.locator('#imu-turn-configure').click();
    await page.waitForFunction(()=>document.getElementById('imu-turn-live').textContent.includes('±0.50°') && document.getElementById('imu-turn-live').textContent.includes('输入已生效'));
    let s=await snapshot();assert.equal(s.turn.kp_centi,'8000');assert.equal(s.turn.tolerance_cd,'50');
    assert.equal(s.turn.min_sps,'350');
    for(const prefix of ['turn','nav-turn']) {
      assert.equal(await page.locator(`#${prefix}-kp`).inputValue(),'80');
      assert.equal(await page.locator(`#${prefix}-tolerance`).inputValue(),'0.50');
      assert.equal(await page.locator(`#${prefix}-min-sps`).inputValue(),'350');
    }
    await page.locator('#imu-turn-tolerance').fill('0.001');await page.locator('#imu-turn-configure').click();
    assert.equal((await snapshot()).turn.tolerance_cd,'50','invalid tolerance must not reach the board');
    await page.locator('#imu-turn-tolerance').fill('0.10');await page.locator('#imu-turn-configure').click();
    await page.waitForFunction(()=>document.getElementById('imu-turn-live').textContent.includes('±0.10°'));
    await page.getByRole('tab',{name:'坐标巡航',exact:true}).click();
    await page.locator('#nav-turn-tolerance').fill('0.50');await page.locator('#nav-turn-configure').click();
    await page.waitForFunction(()=>document.getElementById('nav-turn-live').textContent.includes('±0.50°') && document.getElementById('nav-turn-live').textContent.includes('输入已生效'));
    // Values above the former cap must reach the board unchanged from all views.
    await page.locator('#nav-turn-kp').fill('1000.50');await page.locator('#nav-turn-configure').click();
    await page.waitForFunction(()=>document.getElementById('nav-turn-live').textContent.includes('P=1000.5') && document.getElementById('nav-turn-live').textContent.includes('输入已生效'));
    assert.equal((await snapshot()).turn.kp_centi,'100050');
    await page.getByRole('tab',{name:'底盘运动',exact:true}).click();
    await page.locator('#turn-kp').fill('500');await page.locator('#turn-configure').click();
    await page.waitForFunction(()=>document.getElementById('turn-live').textContent.includes('P=500'));
    assert.equal((await snapshot()).turn.kp_centi,'50000');
    await page.getByRole('tab',{name:'陀螺仪 · H723',exact:true}).click();
    await page.locator('#imu-turn-kp').fill('0');await page.locator('#imu-turn-configure').click();
    await page.waitForFunction(()=>document.getElementById('imu-turn-live').textContent.includes('P=0') && document.getElementById('imu-turn-live').textContent.includes('输入已生效'));
    assert.equal((await snapshot()).turn.kp_centi,'0');
    await page.locator('#imu-turn-kp').fill('80');await page.locator('#imu-turn-configure').click();
    await page.waitForFunction(()=>document.getElementById('imu-turn-live').textContent.includes('P=80') && document.getElementById('imu-turn-live').textContent.includes('输入已生效'));
    await page.reload();await page.getByRole('tab',{name:'陀螺仪 · H723',exact:true}).click();
    assert.equal(await page.locator('#imu-turn-kp').inputValue(),'80');
    assert.equal(await page.locator('#imu-turn-tolerance').inputValue(),'0.50');
    assert.equal(await page.locator('#imu-turn-min-sps').inputValue(),'350');
    for(const width of [1440,390]) {
      await page.setViewportSize({width,height:1000});
      await page.screenshot({path:require('node:path').join(__dirname,`../tmp/heading_tuning_${width}.png`),fullPage:true});
      assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
    }
    assert.deepEqual(errors,[]);
    console.log('Heading tuning browser OK: gyro page applies P80/tolerance0.5, shared drafts/readback, precision validation, navigation apply, refresh and mobile layout.');
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
