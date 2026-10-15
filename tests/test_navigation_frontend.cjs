/* Verify preview paths and configuration serialization in the real frontend.
 * No server, serial port, synthetic motor command or dependency required. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../host/static/app.js'), 'utf8');
const context = vm.createContext({crypto: require('node:crypto').webcrypto});
vm.runInContext(source.slice(0, source.indexOf('applyTheme(document.documentElement')).replace(/^import .*;\r?$/gm, ''), context);
const evaluate = code => JSON.parse(JSON.stringify(vm.runInContext(code, context)));
const grid = {x: [0,14,28], y: [0,14,28]};
context.grid = grid;
for (const [start, goal, expected] of [
  [[0,0],[14,14],[[0,14],[14,14]]],
  [[0,7],[28,21],[[0,14],[28,14],[28,21]]],
  [[7,0],[21,28],[[14,0],[14,28],[21,28]]],
  [[14,14],[14,14],[]],
  [[1,1],[0,0],null],
  [[0,0],[28,29],null],
]) {
  context.start = start; context.goal = goal;
  assert.deepEqual(evaluate('planGridRoute(start, goal, grid)'), expected);
}
vm.runInContext(`
  globalThis.nodes = {};
  for (const axis of ['x','y']) [0,14,28].forEach((v,i) => nodes['nav-grid-'+axis+i] = {value:String(v)});
  nodes['nav-sps'] = {value:'500'}; nodes['nav-accel'] = {value:'2000'}; nodes['nav-dwell'] = {value:'3'};
  globalThis.rows = Object.entries({O:[0,0],A:[0,14],B:[14,0],C:[14,28],D:[28,14]}).map(([name,point]) => ({dataset:{point:name},querySelectorAll:()=>[...point,0].map(value=>({value:String(value)}))}));
  globalThis.document = {getElementById:id=>nodes[id],querySelectorAll:()=>rows};
`, context);
const mission = evaluate('readMission()');
assert.deepEqual(mission.grid, grid);
assert.equal(mission.dwell_ms, 3000);
vm.runInContext(`nodes['nav-grid-x1'].value = '';`, context);
assert.throws(() => evaluate('readMission()'));
console.log('Navigation frontend OK: cardinal preview, lane interiors rejected, coincident goals and grid form.');
