/* Reproduce edits across real SSE polls, then apply without moving any wheels. */
const {chromium}=require('playwright');
const assert=require('node:assert/strict');
(async()=>{
  const browser=await chromium.launch({channel:'msedge',headless:true});
  const page=await browser.newPage();
  const errors=[];let token,original;
  page.on('pageerror',e=>errors.push(e.message));
  page.on('request',r=>{if(r.url().includes('/api/events?'))token=new URL(r.url()).searchParams.get('token');});
  const snapshot=()=>page.evaluate(async()=>(await fetch('/api/state')).json());
  const send=body=>page.evaluate(async({body,token})=>(await fetch('/api/command',{
    method:'POST',headers:{'Content-Type':'application/json','X-Session':token},body:JSON.stringify({body})})).json(),{body,token});
  const mask=()=>page.locator('.inv').evaluateAll(boxes=>boxes.reduce((value,b)=>value|(b.checked?1<<Number(b.value):0),0));
  const setMask=async value=>{for(let i=0;i<4;i++)await page.locator(`.inv[value="${i}"]`).setChecked(!!(value&(1<<i)));};
  try{
    await page.goto(process.env.CHASSIS_URL||'http://127.0.0.1:18771/');
    await page.getByRole('tab',{name:'底盘运动',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('inv-state').textContent.includes('5'));
    const initial=await snapshot();assert.equal(initial.demo,true,'requires SIM');
    original=Number(initial.chassis.invert);
    // The ARM event and immediately following SSE snapshot must agree.
    await page.locator('#btn-arm').click();
    await page.waitForFunction(()=>document.getElementById('m-armed').textContent==='已使能'&&document.getElementById('btn-arm').textContent==='取消使能');
    assert.equal((await snapshot()).armed,true);
    await page.waitForTimeout(1200);
    assert.equal(await page.locator('#m-armed').textContent(),'已使能','snapshot/poll undid ARM confirmation');
    await page.locator('#btn-arm').click();
    await page.waitForFunction(()=>document.getElementById('m-armed').textContent==='已关闭'&&document.getElementById('btn-arm').textContent==='使能输出');
    assert.equal((await snapshot()).armed,false);
    await page.locator('.inv[value="1"]').check();
    await page.waitForTimeout(1200);
    assert.equal(await mask(),original|2,'poll overwrote unsubmitted checkbox');
    await setMask(10);await page.waitForTimeout(1200);
    assert.equal(await mask(),10,'four-wheel edit was overwritten');
    assert.equal(Number((await snapshot()).chassis.invert),original,'editing must not apply automatically');
    await page.locator('#inv-apply').click();
    await page.waitForFunction(()=>document.getElementById('inv-state').textContent.includes('板端掩码 10')&&!document.getElementById('inv-state').textContent.includes('待应用'));
    assert.equal(Number((await snapshot()).chassis.invert),10);
    await setMask(5);await send('ARM 1');await send('NAV RESET 0 0');
    // Force the board to reject CONFIG with a busy chassis, without motion.
    await send('ARM 0');await send('NAV RESET 0 0');
    for(let i=0;i<5;i++)await send(`NAV POINT ${i} 0 0`);
    await send('NAV CONFIG 500 2000 60000');
    await send('NAV BEGIN 1 5 0 0');
    const rows=[];
    for(let i=0;i<5;i++){await send(`NAV WP 1 ${i} 0 0 ${i}`);rows.push(`${i} 0 0 ${i}\n`);}
    let crc=0xffff;for(const byte of Buffer.from(rows.join(''))){crc^=byte<<8;for(let j=0;j<8;j++)crc=((crc<<1)^((crc&0x8000)?0x1021:0))&0xffff;}
    await send(`NAV COMMIT 1 ${crc}`);await send('ARM 1');await send('NAV START 1');
    await send('CHASSIS CONFIG 5');await page.waitForTimeout(800);
    assert.equal(Number((await snapshot()).chassis.invert),10,'rejected CONFIG altered board mask');
    assert.equal(await mask(),5,'rejection/poll discarded local selection');
    await send('NAV STOP');await send('STOP');
    await page.locator('#inv-apply').click();
    await page.waitForFunction(()=>document.getElementById('inv-state').textContent==='板端掩码 5');
    await send('CHASSIS CONFIG 10');
    await page.waitForFunction(()=>document.getElementById('inv-state').textContent==='板端掩码 10');
    assert.equal(await mask(),10,'unedited controls must follow external changes');
    await page.locator('#btn-connect').click();
    await page.waitForFunction(()=>document.getElementById('link-text').textContent==='未连接');
    await page.locator('#btn-connect').click();
    await page.waitForFunction(()=>document.getElementById('inv-state').textContent==='板端掩码 5');
    assert.equal(await mask(),5,'reconnect retained obsolete draft');
    assert.deepEqual(errors,[]);
    console.log('Chassis config browser OK: ARM/disable confirmed, polls preserve edits, apply confirmed, rejection retains draft, external changes/reconnect sync.');
  }finally{if(original!==undefined){await send('STOP').catch(()=>{});await send(`CHASSIS CONFIG ${original}`).catch(()=>{});}await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
