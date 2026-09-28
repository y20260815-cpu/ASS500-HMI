from pathlib import Path
root = Path(__file__).resolve().parent.parent
source = root.parent / 'uwb_follow_simulator.html'
s = source.read_text(encoding='utf-8')
s = s.replace("  'use strict';", "  'use strict';\n" + (root/'tools/uwb_follow_model.js').read_text(encoding='utf-8'))
s = s.replace('const WORLD=20000, CAR_L=1400, CAR_W=900;', 'const WORLD=20000, CAR_L=1200; let CAR_W=800;')
s = s.replace('const UWB_LONG_GAP=1100, UWB_SIDE_GAP=900; // L: left-front, R: right-rear', 'const UWB_FORWARD=600; // Both anchors are ahead of the vehicle center.')
s = s.replace('fx*UWB_LONG_GAP/2', 'fx*UWB_FORWARD').replace('fy*UWB_LONG_GAP/2', 'fy*UWB_FORWARD').replace('UWB_SIDE_GAP', 'CAR_W')
s = s.replace('state.x-fx*UWB_FORWARD', 'state.x+fx*UWB_FORWARD').replace('state.y-fy*UWB_FORWARD', 'state.y+fy*UWB_FORWARD')
start=s.index('  function control(t,dt){'); end=s.index('  function appliedControl',start)
s=s[:start]+'''  const followModel=new UwbFollowModel.Model();
  let modelClock=0,lastModelClock=-Infinity,modelResult=null,uwbAlpha=.25,uwbGate=50,controllerPose=null;
  function control(t,dt){
    modelClock+=dt*1000;
    if(!modelResult || modelClock-lastModelClock>=100){
      lastModelClock=modelClock;
      modelResult=followModel.update(Math.round(t.dl/10),Math.round(t.dr/10),3,modelClock,{
        width:CAR_W/10,alpha:uwbAlpha,gate:uwbGate,maxDistance:+$('maxDistance').value*100,
        stop:+$('stopDist').value,maxSpeed:+$('maxSpeed').value,nearSpeed:+$('nearSpeed').value,
        decel:+$('decelTime').value,gain:+$('turnGain').value,pivot:+$('pivotRpm').value});
    }
    const q=modelResult,vl=q.left,vr=q.right,omega=(vl-vr)/CAR_W;
    state.aligning=q.aligning;
    return {v:(vl+vr)/2,requestedV:q.requested,commandV:(vl+vr)/2,aligning:q.aligning,
      omega,lrpm:vl*2.5,rrpm:vr*2.5,leftCmd:vl,rightCmd:vr,
      slipRatio:clamp(Math.abs(omega)*CAR_L/2/Math.max(Math.abs((vl+vr)/2),250),0,1)};
  }
''' +s[end:]
s=s.replace("const fields=line.split(',');", """const fields=line.split(',');
    if(fields[0]==='GEOM' && fields.length===5){
      const [width,offset,alpha,gate]=fields.slice(1).map(Number);
      if([width,offset,alpha,gate].every(Number.isFinite)&&width>=20&&width<=500&&offset===60){
        if(CAR_W!==width*10){followModel.reset();modelResult=null;}
        CAR_W=width*10;uwbAlpha=alpha;uwbGate=gate;$('carWidth').value=width;
      }
    }
    if(fields[0]==='POSE' && fields.length===6){
      const v=fields.slice(1).map(Number);
      if(v.every(Number.isFinite))controllerPose={valid:v[0]===1,distance:v[1]/10,bearing:v[2],receivedAt:performance.now()};
    }""")
s=s.replace("$('resetBtn').onclick=()=>{", "$('resetBtn').onclick=()=>{followModel.reset();modelResult=null;controllerPose=null;")
at=s.index('  function frame(now)')
s=s[:at]+"""  function drawEstimatedPose(){
    const p=serialPort?(controllerPose&&performance.now()-controllerPose.receivedAt<=300?controllerPose:null):modelResult?.pose;
    $('carWidth').disabled=!!serialPort;
    $('estimateText').textContent=p?.valid?`UWB ${p.distance.toFixed(0)} cm / ${p.bearing.toFixed(1)} deg`:'UWB ---';
    if(!p?.valid)return;
    const a=state.a+p.bearing*Math.PI/180,d=p.distance*10;
    const x=sx(state.x+Math.cos(a)*d),y=sy(state.y+Math.sin(a)*d);
    ctx.save();ctx.strokeStyle='#bc8cff';ctx.lineWidth=2;ctx.beginPath();ctx.arc(x,y,12,0,Math.PI*2);ctx.stroke();ctx.fillStyle='#bc8cff';ctx.fillText('UWB',x+15,y);ctx.restore();
  }
  $('carWidth').oninput=()=>{if(!serialPort){CAR_W=+$('carWidth').value*10;followModel.reset();modelResult=null;}};
"""+s[at:]
s=s.replace('update(dt);draw();drawChart();','update(dt);draw();drawEstimatedPose();drawChart();')
s=s.replace('<div class="control"><div class="row"><span>09', '<div class="control"><label>&#52264;&#54253; (cm) <input id="carWidth" type="number" min="20" max="500" value="80"></label><div id="estimateText">UWB ---</div></div>\n        <div class="control"><div class="row"><span>09')
assert 'id="carWidth"' in s
dest=root/'_build_recovery/front_anchor_simulator.html'
dest.write_text(s,encoding='utf-8')
(root/'_build_recovery/front_anchor_simulator.js').write_text(s.split('<script>')[1].split('</script>')[0],encoding='utf-8')
print(dest)
