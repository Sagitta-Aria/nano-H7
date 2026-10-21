/* Live browser QA on a SIM-only server; restore settings and disarm afterward. */
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const path = require('node:path');

(async () => {
  const browser = await chromium.launch({channel:'msedge',headless:true});
  const page = await browser.newPage({viewport:{width:1440,height:1100}});
  const errors=[];
  let token, original;
  page.on('pageerror',e=>errors.push(e.message));
  page.on('request',r=>{if(r.url().includes('/api/events?'))token=new URL(r.url()).searchParams.get('token');});
  const snapshot=()=>page.evaluate(async()=>(await fetch('/api/state')).json());
  const api=(route,body)=>page.evaluate(async({route,body,token})=>{
    const r=await fetch(route,{method:'POST',headers:{'Content-Type':'application/json','X-Session':token},body:JSON.stringify(body)});
    const result=await r.json();if(!result.ok)throw Error(JSON.stringify(result));return result;
  },{route,body,token});
  const command=body=>api('/api/command',{body});
  const action=(axis,name)=>page.locator(`.aux-card[data-axis="${axis}"] [data-action="${name}"]`);
  const field=(axis,key)=>page.locator(`.aux-card[data-axis="${axis}"] [data-key="${key}"]`);
  const at=async(axis,position)=>page.waitForFunction(({axis,position})=>{
    const card=document.querySelector(`.aux-card[data-axis="${axis}"]`);
    return card.querySelector('.aux-position-value').textContent===String(position)&&card.querySelector('.aux-live').textContent.includes('静止');
  },{axis,position});
  try{
    await page.goto(new URL('/debug', process.env.AUX_URL||'http://127.0.0.1:18770/').href);
    await page.getByRole('tab',{name:'升降与旋转'}).click();
    await page.waitForFunction(()=>document.querySelector('.aux-card[data-axis="5"] .aux-position-value').textContent==='0');
    const initial=await snapshot();assert.equal(initial.demo,true,'requires a simulated board');
    original=initial.aux_settings;
    assert.equal(await action(5,'positive').isDisabled(),true);
    assert.match(await page.locator('.aux-card[data-axis="5"] .aux-readiness').textContent(),/升降方向未确认/);
    await page.locator('#lift-sign').selectOption('1');
    await page.locator('#lift-dir').click();
    await action(6,'zero').click();
    await page.waitForFunction(()=>!document.querySelector('.aux-card[data-axis="6"] [data-action="apply"]').disabled);
    assert.equal(await action(6,'positive').isDisabled(),true);
    assert.match(await page.locator('.aux-card[data-axis="6"] .aux-readiness').textContent(),/板端范围尚未应用/);
    for(const [axis,values] of [[5,{min:0,max:200,step:10,sps:500,accel:2000,p0:20,p1:60}],
                               [6,{min:-100,max:100,step:30,sps:400,accel:2000,p0:-40,p1:80}]]){
      for(const [key,value]of Object.entries(values))await field(axis,key).fill(String(value));
    }
    await action(5,'save').click();
    await page.waitForFunction(()=>document.getElementById('aux-config-state').textContent.includes('已保存'));
    assert.deepEqual((await snapshot()).aux_settings['6'].presets,[0,0],'saving M5 must not save M6 drafts');
    assert.equal(await field(6,'p0').inputValue(),'-40','M5 confirmation must preserve M6 draft');
    assert.equal((await snapshot()).auxiliary['5'].pos,'0','save must not move');
    await action(6,'save').click();
    await page.waitForFunction(()=>document.querySelector('.aux-card[data-axis="6"] .aux-live').textContent.includes('-100～100'));
    await page.reload();await page.getByRole('tab',{name:'升降与旋转'}).click();
    assert.equal(await field(5,'max').inputValue(),'200');
    assert.equal(await field(6,'p1').inputValue(),'80');
    await field(6,'sps').fill('401');await page.waitForTimeout(900);
    assert.equal(await field(6,'sps').inputValue(),'401','telemetry must preserve unsaved edits');
    await field(6,'sps').fill('400');
    await page.locator('#btn-arm').click();
    await field(5,'max').fill('220'); await action(5,'save').click();
    await page.waitForFunction(()=>document.querySelector('.aux-card[data-axis="5"] .aux-live').textContent.includes('0～220'));
    assert.equal((await snapshot()).aux_settings['5'].max,220,'armed idle save must reach board and persistence');
    await action(5,'positive').click();await at(5,10);
    await action(6,'positive').click();await at(6,30);
    for(const [axis,index,target]of [[5,0,20],[5,1,60],[6,0,-40],[6,1,80]]){
      await action(axis,`goto-${index}`).click();await at(axis,target);
    }
    await action(6,'positive').click();await page.waitForTimeout(350);
    assert.equal((await snapshot()).auxiliary['6'].pos,'80','out-of-range step must not move');
    await action(5,'record-0').click();assert.equal(await field(5,'p0').inputValue(),'60');
    await field(5,'p0').fill('20');
    await command('AUX GOTO 5 0 500 2000');await at(5,0);
    await command('AUX GOTO 6 0 400 2000');await at(6,0);
    await action(5,'save').click();
    await page.waitForFunction(()=>document.getElementById('aux-config-state').textContent.includes('已保存'));
    await command('STOP');
    await page.waitForFunction(()=>document.getElementById('btn-arm').textContent==='使能输出');
    for(const width of [1440,760,390]){
      await page.setViewportSize({width,height:1100});
      await page.screenshot({path:path.join(__dirname,`../tmp/auxiliary_${width}.png`),fullPage:true});
      const layout=await page.evaluate(()=>({overflow:document.documentElement.scrollWidth>innerWidth,
        clipped:[...document.querySelectorAll('#tab-auxiliary button,#tab-auxiliary input')].some(el=>{
          const r=el.getBoundingClientRect();return r.left<0||r.right>innerWidth;})}));
      assert.equal(layout.overflow,false,`overflow at ${width}`);assert.equal(layout.clipped,false,`clipped control at ${width}`);
    }
    assert.deepEqual(errors,[]);
    console.log('Auxiliary browser OK: save/reload, edit retention, M5/M6 steps, four presets, record, bounds, desktop/mobile layout.');
  }finally{
    if(original){await command('STOP').catch(()=>{});await api('/api/auxiliary/config',{config:original}).catch(()=>{});}
    await browser.close();
  }
})().catch(e=>{console.error(e);process.exitCode=1;});
