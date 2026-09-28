const assert=require('node:assert/strict');
const {Model,Motion,Direction,KalmanCV,reconcile}=require('../tools/uwb_follow_model.js');
const c={width:80,sensorError:10,ambiguityMultiplier:3,alpha:5,gate:100,maxDistance:2000,stop:20,maxSpeed:1,nearSpeed:.2,decel:.5,gain:1,pivot:500};
const ranges=(forward,right,width=80)=>[Math.hypot(forward,right+width/2),Math.hypot(forward,right-width/2)];
const updateAt=(m,forward,right,now,extra={})=>{const [a0,a1]=ranges(forward,right,extra.width||c.width);return m.update(a0,a1,3,now,{...c,...extra,bearingReference:Math.atan2(right,forward)*180/Math.PI,behindReference:forward<0});};
{
 const h=new KalmanCV(true);h.update(179*Math.PI/180,0,.1,Math.PI);
 const crossed=h.update(-179*Math.PI/180,100,.1,Math.PI)*180/Math.PI;
 assert.ok(Math.abs(Math.abs(crossed)-180)<5);
}
{
 // A0/A1 difference directly fixes lateral sign for both mirror solutions.
 for(const side of [-1,1]){const m=new Model();let q;for(let i=0;i<15;i++)q=updateAt(m,300,side*150,i*100);assert.equal(Math.sign(q.pose.bearing),side);}
}
{
 // Only the positive-normal/front solution is valid for this installation.
 for(const [f,r] of [[300,-120],[300,120],[200,-120],[200,120]]){
  const m=new Model();let q;for(let i=0;i<20;i++)q=updateAt(m,f,r,i*100);
  assert.ok(Math.abs(q.pose.distance-Math.hypot(f,r))<8,`${f},${r}: ${q.pose.distance}`);
  assert.equal(Math.sign(q.pose.bearing),Math.sign(Math.atan2(r,f))||1);
  assert.ok(Math.abs(q.pose.bearing)<90);
 }
}
{
 // At the rear anchor centre line, the front solution remains continuous.
 const m=new Model();let q;for(let i=0;i<10;i++)q=updateAt(m,120,100,i*100);
 let last=q.pose.bearing,sawShadow=false;
 for(let i=11;i<25;i++){q=updateAt(m,5,140,i*100);if(q.pose.shadow)sawShadow=true;assert.ok(Math.abs(q.pose.bearing-last)<20);last=q.pose.bearing;}
 assert.equal(sawShadow,true);
}
for(const width of [60,80,120]){
 const m=new Model();let q;for(let i=0;i<15;i++)q=updateAt(m,300,0,i*100,{width});
 assert.ok(Math.abs(q.pose.distance-300)<3);assert.ok(Math.abs(q.pose.bearing)<1);
}
{
 // A sampled passage through the anchor line changes branch only once.
 const h=new Motion();[100,60,30,10,0,10,30,60,90].forEach((n,i)=>h.select(n,i*100));
 assert.equal(h.behind,true);const selected=h.behind;
 for(let i=0;i<20;i++)h.select(5+i%3,1000+i*100);
 assert.equal(h.behind,selected);
}
{
 const d=new Direction();assert.equal(d.update(180,300,false,0,0),1);assert.equal(d.error(180),180);assert.equal(d.recovery,true);
}
{
 // Close alignment keeps one rotation direction, exits on the zero crossing,
 // and cannot immediately reverse into an opposite pivot.
 const m=new Model();m.width=80;m.lastComplete=0;
 m.pose={valid:true,distance:50,bearing:30,distanceError:10,angleError:5};
 let q=m.command(0,c);assert.ok(q.left===-q.right&&q.left!==0&&q.requested===0);
 m.pose.bearing=-5;q=m.command(100,c);assert.ok(q.left>=0&&q.right>=0);
 m.pose.bearing=-30;q=m.command(200,c);assert.ok(q.left>=0&&q.right>=0);
 m.pose.distance=20;m.pose.bearing=60;q=m.command(300,c);assert.deepEqual([q.left,q.right],[0,0]);
}
{
 const m=new Model();m.width=80;m.lastComplete=0;
 m.pose={valid:true,distance:50,bearing:30,distanceError:10,angleError:5};
 let q=m.command(0,{...c,turnPwm:20});
 assert.ok(Math.abs(q.left)<=200&&Math.abs(q.right)<=200);
 const z=new Model();z.width=80;z.lastComplete=0;z.pose={...m.pose};
 q=z.command(0,{...c,turnPwm:0});assert.deepEqual([q.left,q.right],[0,0]);
}
{
 assert.equal(reconcile(0,100,80),null);
 const m=new Model();let q=updateAt(m,300,-100,0);const before=q.pose.bearing;
 m.update(0,0,0,600,c);q=updateAt(m,300,-100,700);
 assert.ok(q.pose.bearing<0);assert.ok(Math.abs(q.pose.bearing-before)<30);
}
console.log('Front-only horizontal geometry, signed bearing and non-reversing close alignment passed.');
