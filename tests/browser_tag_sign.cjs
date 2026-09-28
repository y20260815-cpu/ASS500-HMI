const {spawn}=require('node:child_process'),path=require('node:path'),assert=require('node:assert/strict');
const {pathToFileURL}=require('node:url');
const chrome='C:/Program Files/Google/Chrome/Application/chrome.exe';
const profile=path.resolve('_build_recovery/browser-sign-profile');
const child=spawn(chrome,['--headless=new','--disable-gpu','--no-first-run','--remote-debugging-port=19328',`--user-data-dir=${profile}`,'about:blank'],{windowsHide:true,stdio:'ignore'});
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
(async()=>{
  let pages;
  for(let i=0;i<50;i++){try{pages=await(await fetch('http://127.0.0.1:19328/json')).json();break;}catch{await sleep(100);}}
  const ws=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);await new Promise(r=>ws.onopen=r);
  let id=0;const pending=new Map(),errors=[];
  ws.onmessage=e=>{const m=JSON.parse(e.data);if(m.id){pending.get(m.id)?.(m);pending.delete(m.id);}if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails);};
  const call=(method,params={})=>new Promise(r=>{const key=++id;pending.set(key,r);ws.send(JSON.stringify({id:key,method,params}));});
  const evaluate=expression=>call('Runtime.evaluate',{expression,returnByValue:true});
  await call('Runtime.enable');await call('Page.enable');
  await call('Page.navigate',{url:pathToFileURL(path.resolve(process.argv[2]||'../uwb_follow_simulator.html')).href});await sleep(300);
  await evaluate(`(()=>{document.getElementById('resetBtn').click();const c=document.getElementById('yard'),r=c.getBoundingClientRect();c.dispatchEvent(new PointerEvent('pointermove',{clientX:r.left+r.width*.45,clientY:r.top+r.height*.525,bubbles:true}));return true})()`);
  await sleep(700);
  const result=await evaluate(`(()=>({bearing:Number(document.getElementById('bearing').textContent),estimate:document.getElementById('estimateText').textContent,body:document.body.innerText}))()`);
  const value=result.result.result.value;
  assert.equal(errors.length,0);assert.ok(value.bearing<0,`Left-side TAG must display a negative angle: ${value.bearing}`);
  assert.ok(!/[李⑤웾湲곕줉異붿쥌]/.test(value.body),'Visible mojibake remains');
  console.log(`Browser left-side TAG sign passed: ${value.bearing.toFixed(2)} deg`);
  await call('Browser.close');ws.close();
})().catch(e=>{console.error(e);child.kill();process.exitCode=1;});
