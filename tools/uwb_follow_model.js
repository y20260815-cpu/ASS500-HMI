// UWB_FOLLOW_MODEL_BEGIN
const UwbFollowModel = (() => {
  const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
  const wrap=a=>{while(a>180)a-=360;while(a< -180)a+=360;return a;};
  const wrapRad=a=>Math.atan2(Math.sin(a),Math.cos(a));
  const limit=m=>m<=150?30:m<=400?30+(m-150)*15/250:m<1000?45+(m-400)*55/600:100;
  const ambiguityBand=(error,multiplier=3)=>clamp(multiplier,1,3)*error;
  function ratios(cm){
    const x=[100,300,500,1000,2000],s=[100,200,300,400,500],f=[100,80,60,50,40];
    if(cm<=100)return [100,100];
    for(let i=1;i<5;i++)if(cm<=x[i]){const t=(cm-x[i-1])/(x[i]-x[i-1]);return [s[i-1]+t*(s[i]-s[i-1]),f[i-1]+t*(f[i]-f[i-1])];}
    return [500,40];
  }
  function reconcile(l,r,width,mask=3){
    if(mask!==3||![l,r,width].every(x=>Number.isFinite(x)&&x>0&&x<=32767))return null;
    const deficit=width-l-r,excess=Math.abs(l-r)-width,max=limit((l+r)/2);
    if(deficit>max||excess>max)return null;
    if(deficit>0){l+=deficit/2;r+=deficit/2;}
    else if(excess>0){const d=excess/2;if(l>r){l-=d;r+=d;}else{l+=d;r-=d;}}
    return {l,r,corrected:deficit>0||excess>0};
  }
  function uncertainty(l,r,forward,right,width,error=10){
    const baseline=width,selectedDistance=Math.hypot(forward,right),selectedAngle=Math.atan2(right,forward);
    let distanceError=0,angleError=0,corners=0;
    for(const dl of [-error,error])for(const dr of [-error,error]){
      const p=reconcile(l+dl,r+dr,baseline);if(!p)continue;
      const along=(p.l*p.l+baseline*baseline-p.r*p.r)/(2*baseline),normal=Math.sqrt(Math.max(0,p.l*p.l-along*along));
      let best=null;
      for(const side of [-1,1]){const f=60+side*normal,x=-width/2+along,d=(f-forward)**2+(x-right)**2;if(!best||d<best.d)best={f,x,d};}
      distanceError=Math.max(distanceError,Math.abs(Math.hypot(best.f,best.x)-selectedDistance));
      angleError=Math.max(angleError,Math.abs(wrap((Math.atan2(best.x,best.f)-selectedAngle)*180/Math.PI)));corners++;
    }
    return {distance:clamp(distanceError,error,2*error),angle:clamp(angleError,2,45),complete:corners===4};
  }
  function branchBalance(l,r,width){
    const p=reconcile(l,r,width);if(!p)return NaN;
    const along=(p.l*p.l+width*width-p.r*p.r)/(2*width);
    return Math.sqrt(Math.max(0,p.l*p.l-along*along));
  }
  class Motion {
    constructor(){this.ready=false;this.behind=false;this.crossingArmed=false;this.normal=0;this.velocity=0;this.time=0;}
    select(n,now){
      n=Math.abs(n);
      if(!this.ready){this.ready=true;this.normal=this.behind?-n:n;this.time=now;this.crossingArmed=n>=50;return this.normal;}
      const elapsed=now-this.time,continuous=elapsed>0&&elapsed<=300,dt=elapsed/1000;
      if(continuous){
        const predicted=this.normal+this.velocity*dt,front=Math.abs(n-predicted),rear=Math.abs(-n-predicted);
        if(!this.crossingArmed&&n>=50)this.crossingArmed=true;
        if(this.crossingArmed&&Math.abs(this.normal)<=25){const old=this.behind;if(rear+4<front)this.behind=true;else if(front+4<rear)this.behind=false;if(this.behind!==old)this.crossingArmed=false;}
      }else this.velocity=0;
      const next=this.behind?-n:n;
      if(continuous)this.velocity+=.5*(clamp((next-this.normal)/dt,-400,400)-this.velocity);
      this.normal=next;this.time=now;return next;
    }
  }
  class Direction {
    constructor(){this.ready=false;this.probing=false;this.recovery=false;this.changed=false;this.startDistance=0;this.bestDistance=0;this.probeTravel=0;this.last=0;}
    update(angle,distance,shadow,commandedMps,now){
      const a=Math.abs(wrap(angle));this.changed=false;
      if(!this.ready){this.ready=true;this.probing=!!shadow;this.recovery=!shadow&&a>45;this.startDistance=this.bestDistance=distance;this.last=now;return 1;}
      const elapsed=now-this.last;this.last=now;
      if(this.probing&&elapsed>0&&elapsed<=300)this.probeTravel+=Math.abs(commandedMps)*elapsed/10;
      if(distance<this.bestDistance)this.bestDistance=distance;
      if(this.probing){
        const closer=distance<=this.startDistance-10,
              farther=distance>=this.startDistance+20,
              limit=this.probeTravel>=50;
        if(closer){this.probing=false;this.recovery=a>45;}
        else if(farther||(limit&&distance>this.startDistance+10)){this.probing=false;this.changed=true;this.recovery=true;}
        else if(limit){this.probing=false;this.recovery=a>45;}
      }
      if(!this.probing&&a>=90)this.recovery=true;
      if(this.recovery&&a<=45)this.recovery=false;
      return 1;
    }
    error(angle){return wrap(angle);}
  }
  class Divergence {
    constructor(){this.ready=false;this.last=0;this.start=0;this.cooldown=0;this.previous=[0,0];this.growth=0;this.error=[0,0];this.samples=0;}
    update(normal,lateral,behind,left,right,enabled,now){
      const forward=[60+(behind?-normal:normal),60+(behind?normal:-normal)],distance=forward.map(f=>Math.hypot(f,lateral));
      const speed=(left+right)/2,sign=speed>0?1:-1,elapsed=now-this.last;
      const usable=enabled&&Math.abs(speed)>=20&&Math.abs(left-right)<=Math.abs(speed)*.25&&speed*forward[0]>0&&distance[0]>100&&now>=this.cooldown;
      const continuous=this.ready&&this.side===behind&&this.direction===sign&&elapsed>0&&elapsed<=300;
      const reset=()=>{this.start=now;this.growth=0;this.error=[0,0];this.samples=0;};
      if(!usable||!continuous)reset();
      else {const dt=elapsed/1000,delta=distance[0]-this.previous[0];
        if(Math.abs(delta)>300*dt)reset();
        else {this.growth+=delta;for(let i=0;i<2;i++)this.error[i]+=Math.abs(distance[i]-this.previous[i]+speed*forward[i]/distance[i]*dt);this.samples++;}
      }
      const correct=usable&&continuous&&now-this.start>=1000&&this.samples>=10&&this.growth>=30&&this.error[1]+15<this.error[0];
      if(correct){this.cooldown=now+3000;this.ready=false;}
      else {this.ready=usable;if(now-this.start>=2000)reset();}
      this.last=now;this.side=behind;this.direction=sign;this.previous=distance;return correct;
    }
  }
  class FinalAlignment {
    constructor(){this.settled=false;}
    update(distance,stop,error,distanceError=10,angleError=3){
      error=Math.abs(error);
      const enter=Math.max(3,angleError),leave=enter*1.5,leaveDistance=stop+2*distanceError;
      const inStop=distance<=stop||(this.settled&&distance<=leaveDistance);
      if(this.settled&&distance<=leaveDistance&&error<=leave)return 'stop';
      this.settled=false;
      if(inStop&&error<=enter){this.settled=true;return 'stop';}
      if(inStop&&error>enter)return 'align';
      return 'drive';
    }
    rpm(error,pivot){const floor=clamp(pivot,750,2500);return (error<0?-1:1)*clamp(Math.max(floor,Math.round(Math.abs(error)*20)),floor,2500);}
  }
  class KalmanCV {
    constructor(angle=false){this.angle=angle;this.reset();}
    reset(){this.ready=false;this.x=this.v=this.p00=this.p01=this.p10=this.p11=0;this.time=0;}
    dt(now){return this.ready?clamp((now-this.time)/1000,0,.5):0;}
    norm(x){return this.angle?wrapRad(x):x;}
    peek(now){return this.ready?this.norm(this.x+this.v*this.dt(now)):0;}
    advance(now,qStd){const dt=this.dt(now);if(dt<=0)return;this.x+=this.v*dt;const a=this.p00+dt*(this.p10+this.p01)+dt*dt*this.p11,b=this.p01+dt*this.p11,c=this.p10+dt*this.p11,d=this.p11,q=qStd*qStd,d2=dt*dt,d3=d2*dt,d4=d2*d2;this.p00=a+q*d4/4;this.p01=b+q*d3/2;this.p10=c+q*d3/2;this.p11=d+q*d2;this.time=now;this.x=this.norm(this.x);}
    predict(now,qStd){if(!this.ready)return 0;this.advance(now,qStd);return this.x;}
    update(z,now,rStd,qStd){z=this.norm(z);if(!this.ready){this.ready=true;this.x=z;this.v=0;this.p00=rStd*rStd;this.p01=this.p10=0;this.p11=qStd*qStd;this.time=now;return this.x;}const predicted=this.peek(now);if(this.angle)z=predicted+wrapRad(z-predicted);this.advance(now,qStd);const variance=Math.max(.0001,rStd*rStd),innovation=z-this.x,s=this.p00+variance,k0=this.p00/s,k1=this.p10/s,o00=this.p00,o01=this.p01;this.x=this.norm(this.x+k0*innovation);this.v+=k1*innovation;this.p00-=k0*o00;this.p01-=k0*o01;this.p10-=k1*o00;this.p11-=k1*o01;return this.x;}
  }
  class Model {
    constructor(){this.reset();}
    reset(){this.finalAlignment=new FinalAlignment();this.divergence=new Divergence();this.a0Kalman=new KalmanCV();this.a1Kalman=new KalmanCV();this.angleKalman=new KalmanCV(true);this.lastPair=null;this.lastPairAt=-Infinity;this.rangePairError=false;this.width=0;this.accepted=-Infinity;this.lastComplete=-Infinity;this.rawBalance=NaN;this.rawBalanceDelta=0;this.rawRanges=null;this.direction=new Direction();this.bearingReady=false;this.pose={valid:false,distance:0,bearing:0,forward:0,lateral:0};this.aligning=false;this.alignSign=0;this.alignCooldown=0;this.steering=false;}
    update(l,r,mask,now,c){
      if(this.width!==c.width){this.reset();this.width=c.width;}
      if(!this.divergence)this.divergence=new Divergence();
      const baseline=c.width;
      const valid0=!!((mask&1)&&Number.isFinite(l)&&l>0),valid1=!!((mask&2)&&Number.isFinite(r)&&r>0);
      const rangeR=clamp(c.alpha||10,5,100),rangeQ=clamp(c.gate||150,5,300);
      valid0?this.a0Kalman.update(l,now,rangeR,rangeQ):this.a0Kalman.predict(now,rangeQ);
      valid1?this.a1Kalman.update(r,now,rangeR,rangeQ):this.a1Kalman.predict(now,rangeQ);
      const complete=valid0&&valid1;if(complete)this.lastComplete=now;
      const filteredPair=this.a0Kalman.ready&&this.a1Kalman.ready?reconcile(this.a0Kalman.x,this.a1Kalman.x,baseline):null;
      this.rangePairError=complete&&!filteredPair;
      let pair=complete?filteredPair:null;
      let held=false;
      if(pair&&complete){this.lastPair={...pair};this.lastPairAt=now;}
      else if(this.lastPair&&now-this.lastPairAt<=(complete?300:500)){pair={...this.lastPair};held=true;}
      if(!pair){this.pose={...this.pose,valid:false,held:false,pairError:this.rangePairError};this.bearingReady=false;return this.command(now,c);}
      if(held){this.pose={...this.pose,held:true,pairError:this.rangePairError};return this.command(now,c);}

      let crossed=false;
      const balance=branchBalance(pair.l,pair.r,c.width),band=ambiguityBand(c.sensorError||10,c.ambiguityMultiplier);
      if(Number.isFinite(balance)){
        crossed=Math.abs(balance)<=band;
        if(this.rawRanges){
          for(let i=1;i<8&&!crossed;i++){const t=i/8,il=this.rawRanges[0]+(pair.l-this.rawRanges[0])*t,ir=this.rawRanges[1]+(pair.r-this.rawRanges[1])*t;crossed=Math.abs(branchBalance(il,ir,c.width))<=band;}
          crossed ||= Number.isFinite(this.rawBalance)&&this.rawBalance*balance<0&&Math.abs(this.rawBalance)>band&&Math.abs(balance)>band;
          const delta=Math.abs(balance)-Math.abs(this.rawBalance);crossed ||= this.rawBalanceDelta<0&&delta>0&&Math.abs(this.rawBalance)<=band;this.rawBalanceDelta=delta;
        }
        this.rawBalance=balance;this.rawRanges=[pair.l,pair.r];
      }
      if(!this.pose.valid)this.bearingReady=false;
      const used=pair,along=(used.l**2+baseline**2-used.r**2)/(2*baseline),normal=Math.sqrt(Math.max(0,used.l**2-along**2));
      const pf=normal,pr=-c.width/2+along;
      const shadow=crossed||normal<=band;
      let forward=pf,right=pr,distance=Math.hypot(forward,right);
      const rawBearing=Math.atan2(right,forward),error=uncertainty(used.l,used.r,forward,right,c.width,c.sensorError||10);
      const filteredBearing=this.angleKalman.update(rawBearing,now,Math.max(2,error.angle)*Math.PI/180,Math.PI);
      this.pose.bearing=filteredBearing*180/Math.PI;
      forward=distance*Math.cos(filteredBearing);right=distance*Math.sin(filteredBearing);
      this.bearingReady=true;this.accepted=now;
      this.pose={...this.pose,valid:true,held:false,pairError:false,corrected:pair.corrected,shadow,distance,forward,right,lateral:right,distanceError:error.distance,angleError:error.angle,uncertaintyComplete:error.complete};
      return this.command(now,c);
    }
    command(now,c){
      if(!this.finalAlignment)this.finalAlignment=new FinalAlignment();
      const p=this.pose,zero=()=>({left:0,right:0,requested:0,aligning:false,pose:{...p}});
      if(!p.valid||now-this.lastComplete>500||p.distance>c.maxDistance){this.finalAlignment=new FinalAlignment();this.direction=new Direction();this.aligning=this.steering=false;return zero();}
      const prior=((c.leftCommand||0)+(c.rightCommand||0))/2000;
      const sign=this.direction.update(p.bearing,p.distance,false,prior,now);
      if(now<(this.correctionHold||0)){this.aligning=this.steering=false;return zero();}
      if(this.direction.changed){this.finalAlignment=new FinalAlignment();this.aligning=this.steering=false;return zero();}
      const error=this.direction.error(p.bearing),abs=Math.abs(error);
      if(p.distance<=c.stop){this.aligning=this.alignSign=0;this.steering=false;return zero();}
      const x=clamp((p.distance-c.stop)/100/(.5*c.maxSpeed*Math.max(.1,c.decel)),0,1);
      const v=Math.min(c.nearSpeed,c.maxSpeed)*x+(c.maxSpeed-Math.min(c.nearSpeed,c.maxSpeed))*x*x*(3-2*x);
      const requested=sign*v*1000,entry=2*ratios(p.distance)[0]/100,exit=entry*.5;
      if(abs>=entry)this.steering=true;else if(abs<=exit)this.steering=false;
      const terminalZone=c.stop+Math.max(50,2*(p.distanceError||10));
      const allowPreArrivalPivot=p.distance<=terminalZone&&!this.direction.probing;
      if(!allowPreArrivalPivot){this.aligning=false;this.alignSign=0;}
      else if(!this.aligning&&now>=this.alignCooldown&&abs>=25){this.aligning=true;this.alignSign=error>0?-1:1;}
      else if(this.aligning&&(abs<=10||error*(this.alignSign<0?1:-1)<=0)){this.aligning=false;this.alignSign=0;this.alignCooldown=now+1000;}
      let base=Math.round(v*2500);if(this.direction.probing)base=Math.min(base,500);let turn=this.steering?Math.round(-base*((c.width||80)/Math.max(p.distance,20))*Math.sin(clamp(error,-45,45)*Math.PI/180)*c.gain):0,l,r;
      if(this.direction.recovery&&abs>45&&turn){
        let mag=2500;const t=turn<0?-mag:mag;l=-t;r=t;
      }else if(allowPreArrivalPivot&&this.aligning&&turn){
        let mag=2500;if(abs<20){const x=clamp((abs-10)/10,0,1);mag=Math.round(mag*x*x*(3-2*x));}
        const t=this.alignSign*mag;l=-t;r=t;
      }else{
        if(this.direction.probing)turn=0;
        if(abs>10)base=Math.round(base*clamp(1-.35*(abs-10)/35,.65,1));
        turn=clamp(turn,-Math.round(base*.75),Math.round(base*.75));
        turn=Math.round(turn*clamp((c.turnPwm??100)/100,0,1));l=sign*base-turn;r=sign*base+turn;
      }
      const pivotOutput=(l<0&&r>0)||(l>0&&r<0);
      const cap=pivotOutput
        ?2500*clamp((c.turnPwm??100)/100,0,1)
        :c.maxSpeed*2500;
      const output=value=>{const v=Math.round(clamp(value,-cap,cap)/2.5);return v===0?0:v;};
      return {left:output(l),right:output(r),requested:pivotOutput?0:requested,aligning:this.aligning,pose:{...p}};
    }
  }
  return {Model,Motion,Direction,Divergence,FinalAlignment,KalmanCV,ratios,reconcile,uncertainty,branchBalance};
})();
if(typeof module!=='undefined')module.exports=UwbFollowModel;
// UWB_FOLLOW_MODEL_END
