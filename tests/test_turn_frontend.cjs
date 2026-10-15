/* Exercise real browser validation, mode routing and status-based enablement. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../host/static/app.js'), 'utf8');
const context = vm.createContext({crypto:require('node:crypto').webcrypto});
vm.runInContext(source.slice(0, source.indexOf('applyTheme(document.documentElement')).replace(/^import .*;\r?$/gm, ''), context);
(async () => {
  await vm.runInContext(`(async () => {
    globalThis.sent = []; globalThis.warnings = []; globalThis.nodes = {};
    for (const id of ['turn-kp','turn-tolerance','turn-configure','turn-mode','turn-angle','turn-sps','turn-accel','turn-state','turn-zero-state','turn-live','turn-zero','turn-start','turn-stop']) nodes[id] = {value:'',disabled:false,textContent:''};
    globalThis.document = {getElementById:id=>nodes[id]};
    post = async (url,payload) => {sent.push({url,payload}); return {ok:true};};
    status = message => warnings.push(message);
    nodes['turn-mode'].value='REL'; nodes['turn-angle'].value='-90.25'; nodes['turn-sps'].value='500'; nodes['turn-accel'].value='2000';
    await turnAction('start'); nodes['turn-mode'].value='ABS'; await turnAction('start');
    for (const bad of ['','90.001','NaN','181','1e2']) {nodes['turn-angle'].value=bad; await turnAction('start');}
    await turnAction('zero'); await turnAction('stop');
    nodes['turn-tolerance'].value='0.50'; nodes['turn-kp'].value='40.25'; await turnAction('configure');
    for (const value of ['0','200.01','1000.50','21474836.47']) {nodes['turn-kp'].value=value; await turnAction('configure');}
    for (const bad of ['-1','21474836.48','20.001','1e2']) {nodes['turn-kp'].value=bad; await turnAction('configure');}
    state.link='connected'; state.device={turn:'1'}; state.armed=true; state.role='controller';
    state.turn={state:'idle',zero:'0',imu:'1',current_cd:'0',target_cd:'0',error_cd:'0'};
    renderTurn(); globalThis.noZero=nodes['turn-start'].disabled;
    state.turn.zero='1'; renderTurn(); globalThis.ready=nodes['turn-start'].disabled;
    state.turn.state='turning'; renderTurn(); globalThis.busy=nodes['turn-start'].disabled;
    state.turn.state='done'; state.role='readonly'; renderTurn(); globalThis.readonly=nodes['turn-stop'].disabled;
  })()`, context);
  assert.equal(context.sent.length,9);
  assert.equal(context.sent[0].url,'/api/turn/action');
  assert.equal(context.sent[0].payload.mode,'REL');
  assert.equal(context.sent[0].payload.angle,'-90.25');
  assert.equal(context.sent[1].payload.mode,'ABS');
  assert.equal(context.sent[2].payload.action,'zero');
  assert.equal(context.sent[3].payload.action,'stop');
  assert.equal(context.sent[4].payload.kp,'40.25');
  assert.equal(context.sent[4].payload.tolerance,'0.50');
  assert.deepEqual(Array.from(context.sent.slice(5),item=>item.payload.kp),['0','200.01','1000.50','21474836.47']);
  assert.equal(context.warnings.length,9);
  assert.equal(context.noZero,true); assert.equal(context.ready,false);
  assert.equal(context.busy,true); assert.equal(context.readonly,true);
  console.log('Turn frontend OK: both modes, precision, zero, busy and readonly guards.');
})().catch(error=>{console.error(error);process.exitCode=1;});
