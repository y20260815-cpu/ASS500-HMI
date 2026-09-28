const {spawn}=require('node:child_process'),fs=require('node:fs'),path=require('node:path');
const {pathToFileURL}=require('node:url');
const chrome='C:/Program Files/Google/Chrome/Application/chrome.exe';
const profile=path.resolve('_build_recovery/browser-test-profile');
const child=spawn(chrome,['--headless=new','--disable-gpu','--no-first-run','--remote-debugging-port=19327',`--user-data-dir=${profile}`,'about:blank'],{windowsHide:true,stdio:'ignore'});
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
(async()=>{
 let pages;
 for(let i=0;i<50;i++){try{pages=await(await fetch('http://127.0.0.1:19327/json')).json();break;}catch{await sleep(200);}}
 const ws=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);
 await new Promise(r=>ws.onopen=r);
 let id=0;const pending=new Map(),errors=[];
 ws.onmessage=e=>{const m=JSON.parse(e.data);if(m.id){pending.get(m.id)?.(m);pending.delete(m.id);}if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails);};
 const call=(method,params={})=>new Promise(r=>{const key=++id;pending.set(key,r);ws.send(JSON.stringify({id:key,method,params}));});
 await call('Runtime.enable');await call('Page.enable');
 await call('Page.navigate',{url:pathToFileURL(path.resolve(process.argv[2]||'../uwb_follow_simulator.html')).href});
 await sleep(1500);
 const read=()=>call('Runtime.evaluate',{expression:`JSON.stringify({vehicle:document.getElementById('vehicleCenterXY')?.textContent,tag:document.getElementById('estimateText')?.textContent,run:document.getElementById('runLabel')?.textContent,fl:document.getElementById('fl')?.textContent,fr:document.getElementById('fr')?.textContent})`,returnByValue:true});
 console.log('initial',JSON.stringify(await read()));await sleep(3000);console.log('later',JSON.stringify(await read()));
 console.log('errors',JSON.stringify(errors));
 await call('Browser.close');ws.close();if(errors.length)process.exitCode=1;
})().catch(e=>{console.error(e);child.kill();process.exitCode=1;});
