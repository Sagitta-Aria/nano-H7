/* Execute real front-end validation/conversion without a browser or board.
 * Physical channel IDs stay unchanged; only angle units are converted. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../host/static/app.js'), 'utf8');
const boot = source.indexOf('applyTheme(document.documentElement');
assert.ok(boot > 0);
const context = vm.createContext({ crypto: require('node:crypto').webcrypto });
// ES module imports belong to browser integration, outside these pure checks.
vm.runInContext(source.slice(0, boot).replace(/^import .*;\r?$/gm, ''), context);

(async () => {
  await vm.runInContext(`(async () => {
    globalThis.sent = [];
    globalThis.warnings = [];
    command = async body => { sent.push(body); return {ok:true}; };
    status = text => warnings.push(text);
    state.device = {servo1_travel:'2700', servo2_travel:'1800',servo1_pin:'PH9',servo2_pin:'PH6'};
    globalThis.values = [servoTravel(1), servoTravel(2), pulseToAngle(1,1500), pulseToAngle(2,1500)];
    globalThis.mapping = [servoProtocolChannel(1), servoProtocolChannel(2)];
    await sendAngle(1, '180');
    await sendAngle(1, '90');
    await sendAngle(1, '120');
    await sendAngle(1, '360');
    await sendAngle(1, '180.5');
    await sendAngle(1, '270');
    await sendAngle(2, '360');
    await sendAngle(2, '270');
    state.device.servo2_pin='PH7'; await sendAngle(1, '90');
    // Updated firmware angle units need no conversion; pin labels never route.
    state.device = {servo1_travel:'1800',servo2_travel:'2700',servo1_pin:'PH6',servo2_pin:'PH9'};
    globalThis.direct = [servoProtocolChannel(1),servoProtocolChannel(2),servoTravel(1),servoTravel(2)];
    await sendAngle(1,'120');
    await sendAngle(2,'0');
    state.device = {servo_travel:'2700'};
    state.travel = 270;
    globalThis.legacy = [servoTravel(1),servoTravel(2),servoProtocolChannel(1),servoProtocolChannel(2)];
    await sendAngle(2, '360');
    await sendAngle(1, '120');
    await sendAngle(2, '0');
    globalThis.frames = parseServoFrames('OK cmd=SERVO_STAT ch=1 angle=180.0 us=1833 on=1 OK cmd=SERVO_STAT ch=2 angle=0.0 us=500 on=1');
    globalThis.readback = [pulseToAngle(1,frames[1].us),pulseToAngle(2,frames[2].us)];
  })()`, context);
  assert.deepEqual(Array.from(context.values), [180,270,'90.0','135.0']);
  assert.deepEqual(Array.from(context.mapping), [1,2]);
  assert.deepEqual(Array.from(context.direct), [1,2,180,270]);
  assert.deepEqual(Array.from(context.sent), ['SERVO 1 270','SERVO STAT','SERVO 1 135','SERVO STAT','SERVO 1 180','SERVO STAT','SERVO 2 180','SERVO STAT','SERVO 1 135','SERVO STAT','SERVO 1 120','SERVO STAT','SERVO 2 0','SERVO STAT','SERVO 1 180','SERVO STAT','SERVO 2 0','SERVO STAT']);
  assert.equal(context.warnings.length,5);
  assert.deepEqual(Array.from(context.legacy),[180,270,1,2]);
  assert.deepEqual(Array.from(context.readback),['120.0','0.0']);
  const html=fs.readFileSync(path.join(__dirname,'../host/static/index.html'),'utf8');
  const s1=html.split('data-ch="1"')[1].split('data-ch="2"')[0];
  const s2=html.split('data-ch="2"')[1].split('手动脉宽校准')[0];
  assert.match(s1,/PH6.*TIM12_CH1/);
  assert.match(s1,/data-deg="120"/);
  assert.doesNotMatch(s1,/data-deg="(?:225|270|315|360)"/);
  assert.match(s1,/value="120"/);
  assert.match(s2,/PH9.*TIM12_CH2/);
  assert.match(s2,/data-deg="270"/);
  assert.match(s2,/value="0"/);
  console.log('Servo frontend OK: physical channel identity, angle scaling, limits, defaults and pulse telemetry.');
})().catch(error => { console.error(error); process.exitCode=1; });
