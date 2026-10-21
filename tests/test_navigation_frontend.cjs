/* Verify field editing and configuration serialization in the real frontend.
 * No server, serial port, synthetic motor command or dependency required. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../host/static/app.js'), 'utf8');
const context = vm.createContext({crypto: require('node:crypto').webcrypto, structuredClone});
vm.runInContext(source.slice(0, source.indexOf('applyTheme(document.documentElement')).replace(/^import .*;\r?$/gm, ''), context);
const evaluate = code => JSON.parse(JSON.stringify(vm.runInContext(code, context)));
// SSE progress events must update the same state used by snapshot rendering.
vm.runInContext(`renderNavigation = () => {}; apply({type:'navigation_runner',fields:{active:true,phase:'inward'}});`,context);
assert.deepEqual(evaluate('state.navigationRunner'),{active:true,phase:'inward'});
const grid = {x: [0,14,28], y: [0,14,28]};
vm.runInContext(`
  globalThis.nodes = {};
  for (const axis of ['x','y']) [0,14,28].forEach((v,i) => nodes['nav-grid-'+axis+i] = {value:String(v)});
  nodes['nav-sps'] = {value:'500'}; nodes['nav-accel'] = {value:'2000'}; nodes['nav-dwell'] = {value:'3'};
  nodes['nav-inset-enabled'] = {checked:true}; nodes['nav-inset-steps'] = {value:'1000'};
  globalThis.rows = Object.entries({O:[0,0],A:[0,14],B:[14,0],C:[14,28],D:[28,14]}).map(([name,point],index) => ({dataset:{station:index},querySelector:()=>({checked:index===2}),querySelectorAll:()=>[name,...point,0].map(value=>({value:String(value)}))}));
  globalThis.legs = [1,2,3,4,2,3,4,1,0].map(station => ({querySelector:selector=>selector==='select'?{value:String(station)}:selector==='input[type=checkbox]'?{checked:true}:null}));
  globalThis.document = {getElementById:id=>nodes[id],querySelectorAll:selector=>selector.includes('nav-points')?rows:legs};
`, context);
const mission = evaluate('readMission()');
assert.deepEqual(mission.grid, grid);
assert.equal(mission.dwell_ms, 3000);
assert.equal(mission.turn_inset_enabled, true);
assert.equal(mission.turn_inset_steps, 1000);
assert.equal(mission.stations.length,5);
assert.equal(mission.stations[1].arrival_enabled,false);
assert.equal(mission.stations[2].arrival_enabled,true);
assert.deepEqual(mission.route.map(leg=>leg.station),[1,2,3,4,2,3,4,1,0]);
assert.ok(mission.route.every(leg=>leg.hold===1));
vm.runInContext(`
  legs[0] = {querySelector: selector => selector === 'select' ? {value:'1'} :
    selector === 'input[type=checkbox]' ? {checked:true} :
    selector === '.nav-subtask' ? {value:'1'} :
    selector.startsWith('[data-parameter=') ? {value: selector.includes('outward_steps') ? '450' : '850'} : null};
`, context);
assert.deepEqual(evaluate('readMission().route[0]'),{station:1,hold:1,task:1,outward_steps:450,right_steps:850});
vm.runInContext(`legs[0].querySelector = selector => selector === 'select' ? {value:'1'} : selector === 'input[type=checkbox]' ? {checked:false} : selector === '.nav-subtask' ? {value:'1'} : null;`, context);
assert.deepEqual(evaluate('readMission().route[0]'),{station:1,hold:0});
vm.runInContext(`legs[0].querySelector = selector => selector === 'select' ? {value:'1'} : selector === 'input[type=checkbox]' ? {checked:false} : selector === '.nav-arrival-mode' ? {value:'on'} : selector === '.nav-arrival-angle' ? {value:'0'} : null;`,context);
assert.deepEqual(evaluate('readMission().route[0]'),{station:1,hold:0,arrival_enabled:true,heading:0});
vm.runInContext(`nodes['nav-grid-x1'].value = '';`, context);
assert.throws(() => evaluate('readMission()'));
console.log('Navigation frontend OK: configuration serialization and grid form.');

// Dimensions preserve absolute negative origins and nonuniform middle lanes.
assert.deepEqual(evaluate('fieldGrid({x:-20,y:-10,width:100,height:80,middleX:30,middleY:60})'), {x:[-20,10,80],y:[-10,50,70]});
for (const params of [
  {x:0,y:0,width:0,height:80,middleX:1,middleY:40},
  {x:0,y:0,width:100,height:80,middleX:100,middleY:40},
  {x:999999,y:0,width:100,height:80,middleX:50,middleY:40},
  {x:0,y:0,width:100,height:80,middleX:1.5,middleY:40},
]) {
  context.params = params;
  assert.throws(() => evaluate('fieldGrid(params)'));
}
context.original = {grid:{x:[0,10,30],y:[0,20,30]}, stations:[
  {name:'O',x:0,y:0,heading:0},{name:'A',x:10,y:10,heading:90},
  {name:'B',x:20,y:20,heading:-90}],route:[{station:1,hold:1},{station:2,hold:0}]};
context.resized = {x:[-20,10,80],y:[-10,50,70]};
const remapped = evaluate('remapField(original,resized,true)');
assert.deepEqual(remapped.stations.map(s=>[s.x,s.y]),[[-20,-10],[10,20],[45,50]]);
assert.deepEqual(remapped.route,context.original.route);
assert.deepEqual(evaluate('remapField(original,resized,false).stations'),context.original.stations);
assert.equal(context.original.stations[1].y,10);
console.log('Field editor OK: negative bounds, asymmetric lanes, piecewise station remapping and invalid dimensions.');

// Partially edited stations need not lie on the old lanes to resize the field.
context.original.stations[1].x=11;
assert.equal(evaluate('remapField(original,resized,true).stations[1].x'),14);
// Pending dimensions participate in save/structural reads without an apply step.
vm.runInContext(`
  nodes['nav-grid-x1'].value='14';
  for (const [key,value] of Object.entries({x:0,y:0,width:100,height:80,'middle-x':30,'middle-y':40})) nodes['nav-field-'+key]={value:String(value)};
  nodes['nav-field-remap']={checked:false};
  state.fieldDirty=true;
`,context);
assert.deepEqual(evaluate('readMission().grid'),{x:[0,30,100],y:[0,40,80]});
assert.equal(evaluate('readMission().stations[1].y'),14);
vm.runInContext(`nodes['nav-field-remap'].checked=true;`,context);
assert.equal(evaluate('readMission().stations[1].y'),40);
assert.equal(evaluate('readMission().stations[1].y'),40,'repeated reads do not remap twice');
console.log('Field save OK: pending dimensions, explicit remapping, off-lane station edits.');
