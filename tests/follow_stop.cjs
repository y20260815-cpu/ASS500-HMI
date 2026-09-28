const assert=require('node:assert/strict'),{Model,FinalAlignment}=require('../tools/uwb_follow_model.js');
const a=new FinalAlignment();
assert.equal(a.update(60,20,30),'drive');assert.equal(a.update(19,20,-6),'align');
assert.equal(a.update(19,20,3),'stop');assert.equal(a.update(21,20,4),'stop');assert.equal(a.update(19,20,9),'align');
const c={stop:20,maxDistance:2000,maxSpeed:1,nearSpeed:.3,decel:.5,gain:2,pivot:500};
for(const [angle,expected] of [[20,1],[-20,-1],[160,1],[-160,-1]]){
 const m=new Model();m.pose={valid:true,distance:18,bearing:angle};m.accepted=m.lastComplete=0;
 const q=m.command(0,c);assert.ok(expected*q.left>0);assert.equal(q.left,-q.right);assert.equal(q.requested,0);
 m.pose.bearing=0;
 const stopped=m.command(100,c);assert.equal(stopped.left,0);assert.equal(stopped.right,0);
}
console.log('Final heading band, stop hysteresis, forward/reverse alignment and zero translation passed.');
for(const angle of [80,-80,120,-120]){
 const m=new Model();m.pose={valid:true,distance:500,bearing:angle};m.accepted=m.lastComplete=0;
 const q=m.command(0,c);
 assert.ok(!(q.left<0&&q.right<0),`reverse translation forbidden ${angle}: ${q.left},${q.right}`);
}
console.log('Large-angle targets never produce reverse translation.');
