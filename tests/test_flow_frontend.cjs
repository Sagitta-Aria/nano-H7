/* Exercise vector display and calibration guards using the actual page code. */
const assert=require('node:assert/strict'), fs=require('node:fs'), vm=require('node:vm'), path=require('node:path');
const source=fs.readFileSync(path.join(__dirname,'../host/static/app.js'),'utf8');
const context=vm.createContext({crypto:require('node:crypto').webcrypto});
vm.runInContext(source.slice(0,source.indexOf('applyTheme(document.documentElement')).replace(/^import .*;\r?$/gm,''),context);
vm.runInContext(`
globalThis.nodes={};
for (const id of ['mode','x','y','length','rx','ry','rlength','record-time','live','applied','start','stop','apply','save','height','gx','gy','angle','quality','swap','negx','negy']) nodes['flow-'+id]={value:'',textContent:'',disabled:false,checked:false};
globalThis.document={getElementById:id=>nodes[id],querySelectorAll:()=>[]};
drawFlowVector=()=>{};
state.role='controller'; state.link='connected';state.device={flow:'1'};
adoptFlowSettings({height_mm:30,gain_x:1,gain_y:1,angle_deg:0,map:0,min_quality:1});
for (const node of Object.values(nodes)) node.value=String(node.value);
state.flow={x_um:'3000',y_um:'4000',rx_um:'-3000',ry_um:'-4000',wx_um:'4000',wy_um:'-3000',wrx_um:'0',wry_um:'0',marked:'1',recording:'0',start_ms:'4294967000',end_ms:'704',heading:'1',gaps:'0'};
nodes['flow-mode'].value='body';renderFlow();
globalThis.body=[nodes['flow-x'].textContent,nodes['flow-y'].textContent,nodes['flow-length'].textContent,nodes['flow-rlength'].textContent];
globalThis.time=nodes['flow-record-time'].textContent;
nodes['flow-mode'].value='world';renderFlow();
globalThis.world=[nodes['flow-x'].textContent,nodes['flow-y'].textContent,nodes['flow-rlength'].textContent];
state.flow.locked='1';state.flow.heading='0';state.flow.gaps='20';renderFlow();globalThis.warning=nodes['flow-live'].textContent;
state.role='readonly';renderFlow();globalThis.readonly=nodes['flow-start'].disabled&&nodes['flow-apply'].disabled&&nodes['flow-save'].disabled;
nodes['flow-swap'].checked=true;nodes['flow-negx'].checked=true;globalThis.map=readFlowSettings().map;
globalThis.rejected=0;
for (const [id,bad] of [['height','30.0001'],['gx','0'],['gy','1.0000001'],['angle','90.001'],['quality','1.5']]) {
 const node=nodes['flow-'+id],old=node.value;node.value=bad;
 try {readFlowSettings();} catch {rejected++;} node.value=old;
}
`,context);
assert.deepEqual(Array.from(context.body),['3.000','4.000','5.000','5.000']);
assert.deepEqual(Array.from(context.world),['4.000','-3.000','0.000']);
assert.match(context.time,/1.000 s/);assert.equal(context.readonly,true);assert.equal(context.map,5);
assert.equal(context.rejected,5);assert.match(context.warning,/坐标已锁定/);
console.log('Flow frontend OK: independent vectors, magnitude, clock wrap, warnings, readonly and precision.');
