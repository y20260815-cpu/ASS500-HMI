const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(process.argv[2]||'_build_recovery/tag_center_simulator.html','utf8');
const code=html.split('// CENTER_GEOMETRY_BEGIN')[1].split('// CENTER_GEOMETRY_END')[0];
const ctx=vm.createContext({Math});vm.runInContext(code,ctx);
const near=(a,b)=>assert.ok(Math.abs(a-b)<1e-8,`${a} != ${b}`);
for(const heading of [0,Math.PI/2,-Math.PI/2,Math.PI,0.37]){
  for(const [forward,right] of [[1000,0],[0,1000],[0,-1000],[-1000,0],[3000,4000],[-3000,-4000],[0,0]]){
    const x=5000,y=9000;
    const tx=x+forward*Math.cos(heading)-right*Math.sin(heading);
    const ty=y+forward*Math.sin(heading)+right*Math.cos(heading);
    const p=ctx.centerGeometry(x,y,heading,tx,ty);
    near(p.distance,Math.hypot(forward,right));near(p.forward,forward);near(p.lateral,right);
    near(Math.sin(p.bearing),Math.sin(Math.atan2(right,forward)));
    near(Math.cos(p.bearing),Math.cos(Math.atan2(right,forward)));
  }
}
console.log('Center distance, signed bearing, rotated vehicle frames and coincident centers passed.');
