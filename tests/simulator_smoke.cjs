const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(process.argv[2],'utf8'),nodes=new Map();
const arcs=[];
const context=new Proxy({measureText:()=>({width:20}),arc:(...args)=>arcs.push(args)},{get:(o,k)=>o[k]??(()=>{})});
function node(id){if(!nodes.has(id)){
 const input=html.match(new RegExp('id="'+id+'"[^>]*'))?.[0]||'';
 nodes.set(id,{value:input.match(/value="([^"]+)"/)?.[1]||'0',checked:input.includes('checked'),style:{},classList:{add(){},remove(){}},getContext:()=>context,
 getBoundingClientRect:()=>({width:1000,height:600,left:0,top:0}),events:{},addEventListener(type,fn){this.events[type]=fn;},appendChild(){},remove(){},click(){}});
}return nodes.get(id);}
let frame;
const sandbox={console,Math,Number,Date,performance:{now:()=>0},devicePixelRatio:1,
 document:{getElementById:node,querySelector:node,body:node('body')},window:{addEventListener(){}},
 requestAnimationFrame:fn=>{frame=fn;},setInterval(){},setTimeout(){},ResizeObserver:class{observe(){}},TextEncoder,TextDecoder};
let code=html.split('<script>')[1].split('</script>')[0];
code=code.replace(/\}\)\(\);\s*$/, 'globalThis.testApi={state,control,telemetry,appliedControl,draw,sx,sy,update,get model(){return followModel;}};})();');
vm.runInNewContext(code,sandbox);
for(let i=1;i<=100;i++)frame(i*20);
assert.match(node('estimateText').textContent,/TAG [0-9]/);
node('carWidth').value='120';node('carWidth').oninput();frame(2020);
assert.match(node('estimateText').textContent,/TAG [0-9]/);
console.log('Simulator startup, 100 rendered frames, estimated pose and width changes passed.');
assert.ok(!html.includes('truthMode')&&!html.includes('truthModel'));
{
 const {state,control,telemetry,appliedControl}=sandbox.testApi;
 const results=[];
 for(const fakeAngle of [0,Math.PI]){
  node('resetBtn').onclick();
  // Identical distances must give identical control, regardless of hidden TAG coordinates.
  // With horizontal front anchors this pair is a valid centreline position,
  // rather than the diagonal layout's startup shadow.
  let q;for(let i=0;i<20;i++)q=appliedControl(control({dl:3000,dr:3000,distance:5000,bearing:fakeAngle,forward:Math.cos(fakeAngle)*5000},.1),.1);
  results.push([q.leftCmd,q.rightCmd,q.v]);
 }
 assert.deepEqual(results[0],results[1]);
  assert.equal(results[0][0],results[0][1]);
  console.log('Equal horizontal-anchor ranges produce straight, truth-independent control.');
}
{
 const {state,draw,sx,sy}=sandbox.testApi,events=node('#yard').events;
 node('resetBtn').onclick();
 events.pointermove({clientX:600,clientY:400});
 const first=[state.tx,state.ty];
 arcs.length=0;draw();
 assert.ok(arcs.some(a=>a[2]===10&&a[0]===sx(first[0])&&a[1]===sy(first[1])),'TAG must draw at the mouse input position');
 events.pointerdown({clientX:600,clientY:400});assert.equal(state.pinned,true);
 events.pointermove({clientX:400,clientY:200});assert.deepEqual([state.tx,state.ty],first);
 // Changes in vehicle pose must not move the fixed TAG marker.
 state.x+=100;state.a+=.2;arcs.length=0;draw();
 assert.ok(arcs.some(a=>a[2]===10&&a[0]===sx(first[0])&&a[1]===sy(first[1])));
 events.pointerdown({clientX:600,clientY:400});assert.equal(state.pinned,false);
 events.pointermove({clientX:400,clientY:200});assert.notDeepEqual([state.tx,state.ty],first);
 console.log('Mouse TAG drawing, click fixation, vehicle-independent pinned position and release passed.');
}
{
 const {state,update}=sandbox.testApi;
 node('hemisphereMode').value='reference';node('hemisphereMode').onchange();
 for(const width of [80,120]) for(const [x,y] of [[10000,5000],[14000,9000],[6000,9000]]){
  node('carWidth').value=String(width);node('carWidth').oninput();
  node('resetBtn').onclick();state.tx=x;state.ty=y;state.running=true;
  const initial=Math.hypot(x-state.x,y-state.y);
  for(let i=0;i<3000;i++)update(.02);
  const final=Math.hypot(x-state.x,y-state.y);
  console.log({target:[x,y],initial,final,vehicle:[state.x,state.y],angle:state.a,pose:sandbox.testApi.model.pose,commands:[state.simLeftCmd,state.simRightCmd]});
  assert.ok(final<=+node('stopDist').value*10+100,'Vehicle must settle near the TAG, not pass it and continue away');
  // Horizontal geometry keeps the selected front/rear branch through its
  // direction-transition line. Final heading is checked below.
 }
}
{
 const {state,update,sx,sy}=sandbox.testApi,events=node('#yard').events;
 node('resetBtn').onclick();node('carWidth').value='80';node('carWidth').oninput();
 for(let i=0;i<1500;i++){
  const t=Math.min(i/1000,1),x=10000+3000*t,y=5000+3000*t;
  events.pointermove({clientX:sx(x),clientY:sy(y)});update(.02);
 }
 const distance=Math.hypot(state.tx-state.x,state.ty-state.y);
 console.log({movingTagDistance:distance});
 assert.ok(distance<500,'Moving front TAG must be followed');
}
if(process.env.TEST_REAR_MOUSE==='1'){
 const {state,update,sx,sy}=sandbox.testApi,events=node('#yard').events;
 node('resetBtn').onclick();node('carWidth').value='80';node('carWidth').oninput();
 events.pointermove({clientX:sx(10000),clientY:sy(18000)});
 for(let i=0;i<1500;i++)update(.02);
 console.log({rearMouseDistance:Math.hypot(state.tx-state.x,state.ty-state.y),vehicle:[state.x,state.y]});
}
if(html.includes('id="hemisphereMode"')){
 const {state,update,sx,sy}=sandbox.testApi,events=node('#yard').events;
 node('hemisphereMode').value='reference';node('hemisphereMode').onchange();
 for(const [x,y] of [[10000,18000],[15000,16000],[5000,16000],[15000,14500],[5000,14500]]){
  node('resetBtn').onclick();state.running=true;
  events.pointermove({clientX:sx(x),clientY:sy(y)});
  for(let i=0;i<2000;i++)update(.02);
  const distance=Math.hypot(x-state.x,y-state.y);
  console.log({referenceTarget:[x,y],distance,angle:state.a,pose:sandbox.testApi.model.pose,commands:[state.simLeftCmd,state.simRightCmd]});
  assert.ok(distance<350,'Reference-assisted range geometry must follow side/rear mouse TAG');
  if(html.includes('class FinalAlignment')){
    const angle=Math.atan2(y-state.y,x-state.x)-state.a;
    const error=Math.abs(Math.atan2(Math.sin(angle),Math.cos(angle)));
    const allowed=Math.max(3,sandbox.testApi.model.pose.angleError||3);
    assert.ok(Math.min(error,Math.PI-error)*180/Math.PI<=allowed,'Arrival must finish within the sensor-derived heading band');
  }
 }
}
