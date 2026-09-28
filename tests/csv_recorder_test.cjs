const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(process.argv[2]||'_build_recovery/csv_simulator.html','utf8');
const script=html.split('// CSV_RECORDER_BEGIN')[1].split('// CSV_RECORDER_END')[0];
const elements={},handlers={}; let clock=1000,blob,clicked=false;
const $=id=>elements[id]??=( {value:'1',disabled:false,textContent:''} );
const ctx=vm.createContext({$,performance:{now:()=>clock},Date,Blob,Number,console,
  lcdPoseFresh:()=>false,lcdPose:{distance:0,angle:0},
  serialPort:null,latestCanCommand:{left:0,right:500,receivedAt:1000},
  latestCanFeedback:{m1:0,m2:1000,fresh:true,receivedAt:1000},state:{tx:20,ty:30},
  confirm:()=>true,setTimeout:fn=>fn(),
  URL:{createObjectURL:b=>(blob=b,'blob:test'),revokeObjectURL:()=>{}},
  document:{body:{appendChild:()=>{}},createElement:()=>({click:()=>{clicked=true},remove:()=>{}})},
  window:{addEventListener:(name,fn)=>handlers[name]=fn}});
vm.runInContext(script+';globalThis.api={csvRecording,csvCell,csvText,recordCsvSample,csvColumns};',ctx);
const {api}=ctx;
const p={t:1,requested:1,command:.5,speed:.4,remaining:2,m1:100,m2:200,
voltage:null,current:null,brakeApplied:null,driveState:'TEST,"quoted"',canAge:5,txSequence:1,txAccepted:true,aligning:false};
const t={dl:100,dr:200},c={leftCmd:100,rightCmd:200};
$('csvRecordBtn').onclick();api.recordCsvSample(p,t,c);
assert.equal(api.csvRecording.rows.length,1);
assert.equal(api.csvRecording.rows[0].length,api.csvColumns.length);
api.recordCsvSample(p,{...t,distance:999,bearing:1,forward:2,lateral:3},c,{valid:true,distance:42.5,bearing:-39.53});
assert.equal(api.csvRecording.rows[1].at(-4),425);assert.equal(api.csvRecording.rows[1].at(-3),-39.53);
api.csvRecording.rows.pop();
assert.equal(api.csvCell(null),'');assert.equal(api.csvCell(-10),'-10');
assert.equal(api.csvCell('=1+1'),"'=1+1");
assert.equal(api.csvCell('a,"b"\nc'),'"a,""b""\nc"');
assert.equal(api.csvText([]).charCodeAt(0),0xfeff);
assert.ok(api.csvText([]).endsWith('\r\n'));
$('csvRecordBtn').onclick();api.recordCsvSample(p,t,c);
assert.equal(api.csvRecording.rows.length,1);
clock=2000;$('csvRecordBtn').onclick();ctx.serialPort={};
api.recordCsvSample({...p,t:0},t,c); // reset graph clock, retain record clock and prior rows
assert.equal(api.csvRecording.rows[1][1],1);
assert.equal(api.csvRecording.rows[1][12],null); // stale command is blank, not zero
clock=2100;ctx.latestCanCommand.receivedAt=2100;ctx.latestCanFeedback.receivedAt=2100;
api.recordCsvSample(p,t,c);
assert.equal(api.csvRecording.rows[2][12],0); // actual fresh zero preserved
assert.equal(api.csvRecording.rows[2][18],0);
let warned=false;handlers.beforeunload({preventDefault:()=>warned=true});assert.ok(warned);
$('csvDownloadBtn').onclick();assert.ok(clicked);assert.equal(blob.type,'text/csv;charset=utf-8;');
assert.equal(api.csvRecording.downloaded,3);
api.csvRecording.limit=4;api.recordCsvSample(p,t,c);assert.equal(api.csvRecording.active,false);
$('csvClearBtn').onclick();assert.equal(api.csvRecording.rows.length,0);
console.log('CSV recording, pause/resume, reset, freshness, escaping, download and limit checks passed.');
