/* Regression from the user's physical check: UI S1 must send wire channel 1,
   even when HELLO's pin labels describe the opposite connector numbering. */
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const source=fs.readFileSync(require('node:path').join(__dirname,'../host/static/app.js'),'utf8');
const context=vm.createContext({crypto:require('node:crypto').webcrypto});
vm.runInContext(source.slice(0,source.indexOf('applyTheme(document.documentElement')).replace(/^import .*;\r?$/gm,''),context);
(async()=>{
  await vm.runInContext(`(async()=>{
    globalThis.sent=[];
    command=async body=>{sent.push(body);return {ok:true};};
    status=()=>{};
    state.device={servo:'2',servo1_pin:'PH9',servo2_pin:'PH6',servo1_travel:'2700',servo2_travel:'1800'};
    await sendAngle(1,'120');
    await sendAngle(2,'0');
  })()`,context);
  assert.deepEqual(Array.from(context.sent),['SERVO 1 180','SERVO STAT','SERVO 2 0','SERVO STAT']);
  console.log('Physical identity regression OK: S1->wire1, S2->wire2; angle scale only.');
})().catch(e=>{console.error(e);process.exitCode=1;});
