from pathlib import Path
r=Path(__file__).resolve().parent.parent
s=(r.parent/'uwb_follow_simulator.html').read_text(encoding='utf-8')
start=s.index('  function drawEstimatedPose(){');end=s.index("  $('truthMode').onchange",start)
s=s[:start]+'''  function displayedTag(){
    if(!serialPort&&$('truthMode').checked){
      const t=telemetry();return {valid:true,x:state.tx,y:state.ty,distance:t.distance/10,bearing:t.bearing*180/Math.PI};
    }
    const p=serialPort?(controllerPose&&performance.now()-controllerPose.receivedAt<=300?controllerPose:null):modelResult?.pose;
    if(!p?.valid)return {valid:false,x:NaN,y:NaN};
    const a=state.a+p.bearing*Math.PI/180,d=p.distance*10;
    return {...p,x:state.x+Math.cos(a)*d,y:state.y+Math.sin(a)*d};
  }
  function drawEstimatedPose(){
    const p=displayedTag();
    $('carWidth').disabled=!!serialPort;
    $('truthMode').disabled=!!serialPort;
    $('poseModeLabel').textContent=serialPort?'TAG: LCD / UART':$('truthMode').checked?'TAG: Simulation':'TAG: A0 / A1';
    $('estimateText').textContent=p.valid?`TAG ${p.distance.toFixed(0)} cm / ${p.bearing.toFixed(1)} deg`:'TAG ---';
  }
''' +s[end:]
start=s.index('  function draw(){');end=s.index('  function displayedTag',start)
part=s[start:end].replace('  function draw(){','  function draw(){\n    const shownTag=displayedTag();')
part=part.replace('state.tx','shownTag.x').replace('state.ty','shownTag.y')
s=s[:start]+part+s[end:]
s=s.replace('>UWB ---</div>','>TAG ---</div>')
(r/'_build_recovery/single_tag_simulator.html').write_text(s,encoding='utf-8')
(r/'_build_recovery/single_tag_simulator.js').write_text(s.split('<script>')[1].split('</script>')[0],encoding='utf-8')
