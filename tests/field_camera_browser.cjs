/* End-to-end field drafts and camera-one recognition with synthetic video. */
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

(async () => {
  const browser = await chromium.launch({channel:'msedge',headless:true});
  const context = await browser.newContext({viewport:{width:1440,height:1000}});
  await context.addInitScript(() => {
    Object.defineProperty(navigator.mediaDevices,'enumerateDevices',{value:async()=>[{kind:'videoinput',deviceId:'synthetic',label:'Synthetic test camera'}]});
    Object.defineProperty(navigator.mediaDevices,'getSupportedConstraints',{value:()=>({})});
    Object.defineProperty(navigator.mediaDevices,'getUserMedia',{value:async()=>{
      const canvas=document.createElement('canvas');canvas.width=640;canvas.height=480;
      const ctx=canvas.getContext('2d');ctx.fillStyle='white';ctx.fillRect(0,0,640,480);
      ctx.fillStyle='red';ctx.beginPath();ctx.arc(250,210,45,0,Math.PI*2);ctx.fill();
      return canvas.captureStream(10);
    }});
  });
  const page = await context.newPage(), errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  const state = ()=>page.evaluate(async()=>(await fetch('/api/state')).json());
  try {
    await page.goto(new URL('/debug', process.env.NAV_URL).href);
    await page.getByRole('tab',{name:'坐标巡航',exact:true}).click();
    await page.waitForFunction(()=>document.querySelectorAll('#nav-points tbody tr').length===5);
    for (const [id,value] of Object.entries({x:-100,y:-200,width:40000,height:30000,'middle-x':15000,'middle-y':10000})) await page.locator('#nav-field-'+id).fill(String(value));
    assert.equal(await page.locator('#nav-map, #nav-map-caption, #nav-field-apply').count(),0);
    await page.locator('#nav-field-remap').check();
    await page.locator('#nav-save').click();
    await page.waitForFunction(()=>document.getElementById('nav-config-state').textContent.includes('参数已保存'));
    assert.equal(await page.locator('[data-station="1"] input').nth(2).inputValue(),'9800');
    assert.deepEqual((await state()).mission.grid,{x:[-100,14900,39900],y:[-200,9800,29800]});
    assert.match(await page.locator('#nav-field-summary').textContent(),/40000 × 30000/);
    await page.reload();await page.getByRole('tab',{name:'坐标巡航',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('nav-field-width').value==='40000');
    console.log('field dimensions, negative bounds, station remapping, direct save and persistence');

    await page.getByRole('tab',{name:'摄像头1',exact:true}).click();
    assert.equal(await page.locator('#tab-navigation #vision-editor').count(),0);
    assert.equal(await page.locator('#vision-m00, #vision-verified').count(),0);
    await page.waitForFunction(()=>!document.getElementById('vision-save').disabled);
    await page.locator('#vision-sps').fill('1500');
    await page.locator('#vision-deadzone').fill('5');
    await page.locator('#vision-editor summary').filter({hasText:'视觉 PID 与启动速度'}).click();
    assert.equal(await page.locator('#vision-camera-yaw').inputValue(),'90');
    await page.locator('#vision-camera-yaw').fill('-90');
    await page.locator('#vision-save').click();
    await page.waitForFunction(()=>document.getElementById('vision-config-state').textContent.includes('已保存'));
    assert.equal((await state()).camera_alignment_config.tracking.sps,1500);
    assert.equal((await state()).camera_alignment_config.tracking.deadzone_px,5);
    assert.equal((await state()).camera_alignment_config.tracking.camera_yaw_deg,-90);
    await page.locator('#vision-camera-yaw').fill('90');
    await page.locator('#vision-save').click();
    await page.waitForFunction(async()=>
      (await fetch('/api/state').then(r=>r.json())).camera_alignment_config.tracking.camera_yaw_deg===90);
    await page.locator('#camera-source').selectOption('local');
    await page.locator('#camera-start').click();
    await page.waitForFunction(()=>!document.getElementById('camera-snapshot').disabled);
    await page.locator('#camera-snapshot').click();
    await page.waitForFunction(()=>document.getElementById('camera-image').complete && document.getElementById('camera-image').naturalWidth>0);
    assert.equal((await state()).camera_alignment.vision,null);
    await page.locator('#vision-start').click();
    await page.waitForFunction(()=>document.getElementById('vision-config-state').textContent.includes('Nano 来源'));
    await page.locator('#camera-source').selectOption('nano');
    await page.waitForFunction(()=>document.getElementById('camera-state').textContent.includes('已打开'));
    await page.locator('#tab-camera [data-open-tab="vision"]').click();
    assert.equal(await page.locator('#tab-vision').isVisible(),true);
    await page.locator('#tuning-preview').click();
    await page.waitForFunction(()=>document.getElementById('tuning-state').textContent.includes('中心 ('));
    assert.ok(await page.locator('#tuning-image').evaluate(n=>n.src.startsWith('data:image/jpeg')));
    await page.locator('#tuning-1-0-0').fill('175');
    await page.locator('#tuning-preview').click();
    await page.waitForFunction(()=>document.getElementById('tuning-state').textContent.includes('未找到目标'));
    assert.equal((await state()).camera_alignment_config.detector.colors.red.ranges[1][0][0],170,'preview must not persist draft');
    await page.locator('#tuning-1-0-0').fill('170');
    await page.locator('#tuning-area').fill('3100');
    await page.locator('#tuning-color').selectOption('black');
    assert.match(await page.locator('#tuning-selection').textContent(),/最大/);
    await page.locator('#tuning-color').selectOption('red');
    assert.equal(await page.locator('#tuning-area').inputValue(),'3100','switching colors preserves draft');
    await page.locator('#tuning-save').click();
    await page.waitForFunction(()=>document.getElementById('tuning-state').textContent.includes('已保存：'));
    assert.equal((await state()).camera_alignment_config.detector.colors.red.min_area,3100);
    await page.locator('#tuning-live').click();
    await page.waitForFunction(()=>!document.getElementById('tuning-stop').disabled);
    await page.waitForTimeout(700);
    await page.locator('#tuning-stop').click();
    assert.equal((await state()).armed,false);
    await page.getByRole('tab',{name:'摄像头1',exact:true}).click();
    await page.waitForFunction(()=>!document.getElementById('vision-recognize').disabled);
    await page.locator('#vision-recognize').click();
    await page.waitForFunction(()=>document.getElementById('vision-state').textContent.includes('识别完成'),{timeout:15000});
    assert.match(await page.locator('#vision-result').textContent(),/目标中心/);
    const after = await state();
    assert.equal(after.armed,false);
    assert.equal(after.camera_alignment.vision.found,true);
    assert.ok(Math.abs(after.camera_alignment.vision.error_dx+70)<=1);
    assert.ok(Math.abs(after.camera_alignment.vision.error_dy)<=1);
    assert.deepEqual(after.camera_alignment.vision.control_point,[250,240]);
    assert.equal((await state()).armed,false);
    await page.locator('#vision-kind').selectOption('ring');
    assert.equal(await page.locator('#vision-target').isDisabled(),true);
    assert.equal(await page.locator('#vision-range-steps').inputValue(),'2000');
    await page.locator('#vision-range-steps').fill('3500');
    await page.locator('#vision-scan-step').fill('150');
    await page.locator('#vision-save').click();
    await page.waitForFunction(async()=>
      (await fetch('/api/state').then(r=>r.json())).camera_alignment_config.ring_sequence.range_steps===3500);
    assert.equal((await state()).camera_alignment_config.ring_sequence.scan_step,150);
    assert.match(await page.locator('#vision-start').textContent(),/点1/);
    await page.locator('#vision-kind').selectOption('block');
    assert.equal(await page.locator('#vision-range-steps').isDisabled(),true);
    fs.mkdirSync(path.join(__dirname,'../tmp'),{recursive:true});
    await page.screenshot({path:path.join(__dirname,'../tmp/camera_alignment_desktop.png'),fullPage:true});
    await page.setViewportSize({width:1440,height:1900});
    await page.getByRole('tab',{name:'视觉调参',exact:true}).click();
    await page.locator('#vision-tuning').scrollIntoViewIfNeeded();
    await page.locator('#vision-tuning').screenshot({path:path.join(__dirname,'../tmp/vision_tuning_desktop.png')});
    await page.setViewportSize({width:1440,height:1000});
    console.log('manual photos do not recognize; NX raw-frame preview recognizes without any photo request or movement');

    const observer=await context.newPage();observer.on('pageerror',e=>errors.push(e.message));
    await observer.goto(new URL('/debug', process.env.NAV_URL).href);
    await observer.getByRole('tab',{name:'摄像头1',exact:true}).click();
    await observer.waitForFunction(()=>document.getElementById('vision-save').disabled);
    for(const id of ['vision-save','vision-recognize','vision-start','vision-stop','tuning-save','tuning-preview','tuning-live']) assert.equal(await observer.locator('#'+id).isDisabled(),true);
    await observer.getByRole('tab',{name:'坐标巡航',exact:true}).click();
    assert.equal(await observer.locator('#nav-field-width').isDisabled(),true);
    await observer.close();
    for(const width of [760,390]) {
      await page.setViewportSize({width,height:900});
      for(const tab of ['摄像头1','视觉调参','坐标巡航']) {
        await page.getByRole('tab',{name:tab,exact:true}).click();
        assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false,`${tab} overflow ${width}`);
      }
    }
    await page.getByRole('tab',{name:'摄像头1',exact:true}).click();
    await page.screenshot({path:path.join(__dirname,'../tmp/camera_alignment_mobile.png'),fullPage:true});
    assert.deepEqual(errors,[]);
    console.log('readonly restrictions, 1440/760/390px layout and no page errors');
  } finally {await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
