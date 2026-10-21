/* User-confirmed connectors: UI S1=PH9/180, UI S2=PE4/270. */
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
    state.device={servo:'2',servo1_pin:'PE4',servo2_pin:'PH9',servo1_travel:'2700',servo2_travel:'1800'};
    await sendAngle(1,'120');
    await sendAngle(2,'0');
  })()`,context);
  assert.deepEqual(Array.from(context.sent),['SERVO 2 120','SERVO STAT','SERVO 1 0','SERVO STAT']);
  console.log('Physical identity regression OK: S1/PH9->wire2, S2/PE4->wire1.');
})().catch(e=>{console.error(e);process.exitCode=1;});
