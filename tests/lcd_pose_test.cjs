const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(process.argv[2]||'_build_recovery/lcd_pose_simulator.html','utf8');
const code=html.split('// LCD_POSE_BEGIN')[1].split('// LCD_POSE_END')[0];
let now=1000;const ctx=vm.createContext({Math,Number,serialPort:{},performance:{now:()=>now}});
vm.runInContext(code+';globalThis.pose=()=>lcdPose;',ctx);
ctx.acceptLcdPose(['POSE','1','3000','90','250','358']);assert.ok(ctx.lcdPoseFresh());
assert.equal(ctx.pose().distance,3000);assert.equal(ctx.pose().angle,90);
for(const [angle,x,y] of [[0,160,304],[90,304,160],[-90,16,160],[180,160,16]]){
 const p=ctx.lcdProjection(5000,angle);assert.ok(Math.abs(p.x-x)<1e-8);assert.ok(Math.abs(p.y-y)<1e-8);
}
now=1301;assert.equal(ctx.lcdPoseFresh(),false);
ctx.acceptLcdPose(['POSE','1','3000','181','250','358']);assert.equal(ctx.pose().valid,false);
ctx.acceptLcdPose(['POSE','0','3000','0','250','358']);assert.equal(ctx.lcdPoseFresh(),false);
console.log('LCD pose parsing, common projection, stale and invalid inputs passed.');
