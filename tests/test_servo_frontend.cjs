/* Exercise real browser command routing: PH9=180 degrees, PE4=270 degrees. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../host/static/app.js'), 'utf8');
const context = vm.createContext({crypto: require('node:crypto').webcrypto});
vm.runInContext(source.slice(0, source.indexOf('applyTheme(document.documentElement')).replace(/^import .*;\r?$/gm, ''), context);
(async () => {
  await vm.runInContext(`(async () => {
    globalThis.sent = []; globalThis.warnings = [];
    command = async body => { sent.push(body); return {ok:true}; };
    status = text => warnings.push(text);
    state.device = {servo1_pin:'PE4',servo2_pin:'PH9',servo1_travel:'2700',servo2_travel:'1800'};
    globalThis.mapping = [servoProtocolChannel(1),servoProtocolChannel(2)];
    globalThis.limits = [servoTravel(1),servoTravel(2)];
    await sendAngle(1,'90'); await sendAngle(1,'120'); await sendAngle(1,'180');
    await sendAngle(2,'0'); await sendAngle(2,'135'); await sendAngle(2,'270');
    await sendAngle(1,'180.5'); await sendAngle(1,'270'); await sendAngle(2,'270.5');
    const frames = parseServoFrames('OK cmd=SERVO_STAT ch=1 angle=135.0 us=1500 on=1 pin=PE4 timer=TIM15_CH1N OK cmd=SERVO_STAT ch=2 angle=120.0 us=1833 on=1 pin=PH9 timer=TIM12_CH2');
    globalThis.readback = [pulseToAngle(1,frames[servoProtocolChannel(1)].us),pulseToAngle(2,frames[servoProtocolChannel(2)].us)];
    state.device={servo1_pin:'PH9',servo2_pin:'PH6'};
    globalThis.legacy = [servoProtocolChannel(1),servoProtocolChannel(2)];
    state.device={servo1_pin:'PH9',servo2_pin:'PE4',servo1_travel:'1800',servo2_travel:'2700'};
    globalThis.alternate=[servoProtocolChannel(1),servoProtocolChannel(2),servoWireAngle(1,120),servoWireAngle(2,270)];
  })()`, context);
  assert.deepEqual(Array.from(context.mapping),[2,1]);
  assert.deepEqual(Array.from(context.limits),[180,270]);
  assert.deepEqual(Array.from(context.sent),[
    'SERVO 2 90','SERVO STAT','SERVO 2 120','SERVO STAT','SERVO 2 180','SERVO STAT',
    'SERVO 1 0','SERVO STAT','SERVO 1 135','SERVO STAT','SERVO 1 270','SERVO STAT']);
  assert.equal(context.warnings.length,3);
  assert.deepEqual(Array.from(context.readback),['120.0','135.0']);
  assert.deepEqual(Array.from(context.legacy),[2,1]);
  assert.deepEqual(Array.from(context.alternate),[1,2,'120','270']);
  const html=fs.readFileSync(path.join(__dirname,'../host/static/index.html'),'utf8');
  const s1=html.split('data-ch="1"')[1].split('data-ch="2"')[0];
  const s2=html.split('data-ch="2"')[1].split('手动脉宽校准')[0];
  assert.match(s1,/PH9.*TIM12_CH2/); assert.match(s1,/data-deg="120"/);
  assert.doesNotMatch(s1,/data-deg="(?:225|270|315|360)"/); assert.match(s1,/value="120"/);
  assert.match(s2,/PE4.*TIM15_CH1N/); assert.match(s2,/data-deg="270"/); assert.match(s2,/value="0"/);
  console.log('Servo pin routing OK: PH9 0-180, PE4 0-270, commands, readback and limits.');
})().catch(error=>{console.error(error);process.exitCode=1;});
