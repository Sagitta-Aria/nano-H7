/* Single-frame lifecycle checks: no photo on open/idle, one on shutter,
   no network timer, exact modes, readonly guard and late permission release. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../host/static/camera.js'), 'utf8');

async function fixture(nano = false, prefix = 'camera') {
  const nodes = {}, fetched = [], posted = [], images = [];
  const node = id => nodes[id] ||= {
    value: '', textContent: '', disabled: false, dataset: {}, options: [], listeners: {},
    classList: {add() {}, remove() {}, toggle() {}},
    addEventListener(name, fn) { this.listeners[name] = fn; },
    replaceChildren(...options) { this.options = options; this.value = options[0]?.value || ''; },
    add(option) { this.options.push(option); if (!this.value) this.value = option.value; },
    videoWidth: 640, videoHeight: 480, async play() {},
  };
  node(`${prefix}-source`).value = 'local';
  node(`${prefix}-resolution`).options = ['640x480','1280x720','1920x1080'].map(value => ({value}));
  node(`${prefix}-resolution`).value = '1280x720';
  Object.defineProperty(node(`${prefix}-image`), 'src', {set(value) { images.push(value); }});
  const media = {queue: [], calls: [], addEventListener() {},
    getSupportedConstraints: () => ({resizeMode:true}),
    enumerateDevices: async () => [{kind:'videoinput',deviceId:'usb',label:'USB Camera'}],
    getUserMedia(value) {this.calls.push(value); return Promise.resolve(this.queue.shift());},
  };
  let state = {state:'idle'};
  const fetch = async url => {
    fetched.push(url);
    return {json: async () => url.endsWith('/devices') ? {supported:nano,devices:[{id:'/dev/video0',label:'USB Camera',resolutions:['640x480','1280x720']}]} : state};
  };
  const post = async (url,payload) => {
    posted.push({url,payload});
    if (url.endsWith('/start')) state = {state:'ready',device:payload.device,width:640,height:480};
    if (url.endsWith('/snapshot')) state = {...state,sequence:(state.sequence || 0)+1};
    if (url.endsWith('/stop')) state = {state:'idle'};
    return {ok:true,camera:state};
  };
  const windowEvents = {};
  const context = vm.createContext({post,fetch,setTimeout,Date,
    navigator:{mediaDevices:media}, document:{getElementById:node,createElement:() => ({getContext:()=>({drawImage(){}}),toDataURL:()=> 'data:image/jpeg;base64,PHOTO'})},
    window:{isSecureContext:true,addEventListener(name,fn){windowEvents[name]=fn;}},
    Option:class {constructor(label,value){this.textContent=label;this.value=value;}},
  });
  vm.runInContext(source.replace('export class CameraView','class CameraView')+`\nglobalThis.view=new CameraView(post, '${prefix}', '/api/${prefix}');`,context);
  await context.view.initialisation;
  return {view:context.view,nodes,media,fetched,posted,images,windowEvents};
}

function capture() {
  const track = {stopped:false,refused:false,listeners:{},
    getSettings:()=>({width:640,height:480}),
    stop(){this.stopped=true;},addEventListener(name,fn){this.listeners[name]=fn;},
    async applyConstraints(){if(this.refused)throw Object.assign(new Error(),{name:'OverconstrainedError'});},
  };
  return {track,stream:{getTracks:()=>[track],getVideoTracks:()=>[track]}};
}

(async () => {
  let f = await fixture(true);
  assert.equal(f.nodes['camera-source'].value,'nano');
  assert.equal(f.posted.length,0);
  assert.equal(f.images.length,0);
  assert.deepEqual(f.nodes['camera-resolution'].options.map(o=>o.value),['640x480','1280x720']);
  await f.view.start();
  assert.equal(f.images.length,0,'opening must not transfer a photo');
  const count = f.fetched.length;
  await new Promise(r=>setTimeout(r,30));
  assert.equal(f.fetched.length,count,'idle must not poll metadata');
  await f.view.capture();
  assert.equal(f.images.length,1,'one shutter equals one photo request');
  assert.match(f.images[0],/^\/api\/camera\/frame\?sequence=1/);
  await new Promise(r=>setTimeout(r,30));
  assert.equal(f.images.length,1,'photo must not continuously refresh');
  f.view.setRole('readonly');
  assert.equal(f.nodes['camera-snapshot'].disabled,true);
  await f.view.capture(); await f.view.start(); await f.view.stop();
  assert.equal(f.posted.length,2,'readonly must not mutate shared capture');
  f.view.setRole('controller');
  await f.view.stop();
  assert.equal(f.images.length,1,'closing must retain the displayed photo');
  assert.equal(f.nodes['camera-snapshot'].disabled,true);

  const second = await fixture(true, 'camera2');
  second.view.updateRemote({state:'ready', qr:{status:'valid', code:'012+321+456+054', last_code:'012+321+456+054', last_seen_at:100}});
  assert.equal(second.nodes['camera2-qr-code'].textContent, '012+321+456+054');
  assert.match(second.nodes['camera2-qr-status'].textContent, /当前已识别/);
  second.view.updateRemote({state:'error', qr:{status:'stale', last_code:'012+321+456+054', last_seen_at:100}});
  assert.match(second.nodes['camera2-qr-status'].textContent, /离线/);
  assert.match(second.nodes['camera2-qr-seen'].textContent, /保留结果/);
  assert.equal(second.images.length, 0);
  assert.equal(second.posted.length, 0);
  const lockedPlan = {code:'156+231+516+123', batches:[{batch:1, items:[
    {angle_deg:0,color_name:'红',target:2},{angle_deg:135,color_name:'黑',target:3},{angle_deg:270,color_name:'浅蓝',target:1}
  ]}]};
  second.view.setTask({active:true,uses_task_code:true,task_plan:lockedPlan,qr_status:'摄像头2已关闭，摄像头1已打开'});
  second.view.updateRemote({state:'idle'});
  assert.equal(second.nodes['camera2-qr-code'].textContent, lockedPlan.code);
  assert.match(second.nodes['camera2-task-plan'].textContent, /0° 红 → 位置2；135° 黑 → 位置3；270° 浅蓝 → 位置1/);
  assert.equal(second.nodes['camera2-start'].disabled, true);
  second.view.setTask({active:false});
  await second.view.start();
  await second.view.capture();
  assert.match(second.images[0], /^\/api\/camera2\/frame\?sequence=1/);
  assert.ok(second.posted.every(p => p.url.startsWith('/api/camera2/')));
  assert.equal(f.images.length, 1, 'camera 2 must not change camera 1 photo');
  await second.view.stop();
  await second.view.bind();
  assert.equal(second.posted.at(-1).url, '/api/camera2/bind');
  second.view.setRole('readonly');
  const posted = second.posted.length;
  await second.view.bind();
  assert.equal(second.posted.length, posted);

  f = await fixture(); let c = capture(); f.media.queue.push(c.stream);
  assert.equal(f.media.calls.length,0);
  await f.view.start();
  assert.equal(f.images.length,0);
  assert.equal(f.media.calls[0].audio,false);
  await f.view.capture();
  assert.equal(f.images.length,1);
  c.track.refused = true;
  await f.view.applyResolution();
  assert.equal(c.track.stopped,false);
  assert.match(f.nodes['camera-message'].textContent,/不支持/);
  await f.view.stop();
  assert.equal(c.track.stopped,true);
  assert.equal(f.images.length,1);

  for (const cancel of ['stop','pagehide']) {
    f = await fixture(); c = capture(); let resolve;
    f.media.queue.push(new Promise(done=>{resolve=done;}));
    const pending=f.view.start();
    if(cancel==='stop')await f.view.stop();else f.windowEvents.pagehide();
    resolve(c.stream); await pending;
    assert.equal(c.track.stopped,true,'late permission must release capture');
    assert.equal(f.view.stream,null);
  }
  console.log('Camera frontend OK: manual single frame, no idle traffic, Nano/local source, native modes, readonly and release.');
})().catch(error=>{console.error(error);process.exitCode=1;});
