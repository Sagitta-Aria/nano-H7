/* Execute actual lift controls and wheel rendering with minimal DOM doubles.
 * Checks command boundaries and motor IDs, not browser layout or hardware. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../host/static/app.js'), 'utf8');
const boot = source.indexOf('applyTheme(document.documentElement');
assert.ok(boot > 0);
const elements = {};
const wheels = [3, 4, 1, 2].map(id => ({dataset: {wheel: String(id)},
  classList: {remove() {}, add() {}}, label: {},
  querySelector() { return this.label; }}));
const context = vm.createContext({crypto: require('node:crypto').webcrypto,
  document: {getElementById(id) { return elements[id] ||= {}; },
    querySelectorAll() { return wheels; }}});
// ES module imports belong to browser integration, outside these pure checks.
vm.runInContext(source.slice(0, boot).replace(/^import .*;\r?$/gm, ''), context);
(async () => {
  await vm.runInContext(`(async () => {
    globalThis.sent = []; globalThis.warnings = [];
    command = async body => { sent.push(body); return {ok:true}; };
    status = text => warnings.push(text);
    state.link = 'connected'; state.armed = true; state.device = {lift:'1'};
    state.lift = {initialized:'1', down_sign:'0', active:'0', pos:'0', max:'8000', up_room:'0', down_room:'8000'};
    $('lift-steps').value = '100'; $('lift-sps').value = '300'; $('lift-accel').value = '1000';
    renderLift(); globalThis.unknownDisabled = $('lift-down').disabled;
    await moveLift('DOWN');
    state.lift.down_sign = '-1';
    await moveLift('UP');
    state.lift.pos = '7900'; state.lift.up_room = '7900'; state.lift.down_room = '100';
    $('lift-steps').value = '101'; await moveLift('DOWN');
    $('lift-steps').value = '100'; await moveLift('DOWN');
    await moveLift('UP', true);
    state.role = 'readonly'; await moveLift('DOWN');
    state.role = 'controller'; state.lift.active = '1'; await moveLift('DOWN');
    state.lift.active = '0'; state.lift.pos = '8000'; state.lift.down_room = '0';
    renderLift(); globalThis.bottomDisabled = $('lift-down').disabled;
    renderWheels('LEFT');
  })()`, context);
  assert.equal(context.unknownDisabled, true);
  assert.equal(context.bottomDisabled, true);
  assert.equal(context.warnings.length, 5);
  assert.deepEqual(Array.from(context.sent), ['LIFT MOVE DOWN 100 300 1000', 'LIFT STAT', 'LIFT MOVE UP 7900 300 1000', 'LIFT STAT']);
  assert.deepEqual(wheels.map(w => w.label.textContent), ['−', '+', '+', '−']);
  const html = fs.readFileSync(path.join(__dirname, '../host/static/index.html'), 'utf8');
  const mapping = [...html.matchAll(/data-wheel="(\d)"[^>]*><b>M\d<\/b><span>([^<]+)/g)];
  assert.deepEqual(mapping.map(m => [m[1], m[2]]), [
    ['3', '左前 · I04 / I07'], ['4', '右前 · C13 / I02'],
    ['1', '左后 · I08 / I05'], ['2', '右后 · I03 / I06']]);
  console.log('Lift/chassis frontend OK: boundaries, role, busy state, return to top, wheel mapping.');
})().catch(error => {console.error(error); process.exitCode = 1;});
