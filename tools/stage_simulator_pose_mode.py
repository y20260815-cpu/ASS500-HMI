from pathlib import Path
root=Path(__file__).resolve().parent.parent
s=(root.parent/'uwb_follow_simulator.html').read_text(encoding='utf-8')
s=s.replace('<div id="estimateText">UWB ---</div>', '<div id="estimateText">UWB ---</div><label><input id="truthMode" type="checkbox" checked> &#49892;&#51228; TAG &#50948;&#52824;&#47196; &#51452;&#54665; &#49884;&#54744; (&#50724;&#54532;&#46972;&#51064;)</label><div id="poseModeLabel"></div>')
s=s.replace('const followModel=new UwbFollowModel.Model();','const followModel=new UwbFollowModel.Model(),truthModel=new UwbFollowModel.Model();')
s=s.replace('let modelClock=0,','let driveResult=null;\n  let modelClock=0,')
s=s.replace('modelResult=followModel.update(Math.round(t.dl/10),Math.round(t.dr/10),3,modelClock,{','const config={')
s=s.replace("decel:+$('decelTime').value,gain:+$('turnGain').value,pivot:+$('pivotRpm').value});", """decel:+$('decelTime').value,gain:+$('turnGain').value,pivot:+$('pivotRpm').value};
      modelResult=followModel.update(Math.round(t.dl/10),Math.round(t.dr/10),3,modelClock,config);
      truthModel.pose={valid:true,distance:t.distance/10,bearing:t.bearing*180/Math.PI};
      truthModel.accepted=modelClock;
      driveResult=!serialPort&&$('truthMode').checked?truthModel.command(modelClock,config):modelResult;""")
s=s.replace('const q=modelResult,vl=q.left', 'const q=driveResult,vl=q.left')
s=s.replace("$('carWidth').disabled=!!serialPort;", """$('carWidth').disabled=!!serialPort;
    $('truthMode').disabled=!!serialPort;
    $('poseModeLabel').textContent=serialPort?'UART: controller commands':$('truthMode').checked?'Drive: actual TAG | Purple: UWB estimate':'Drive: UWB estimate (front/rear ambiguous)';""")
s=s.replace("$('resetBtn').onclick=()=>{followModel.reset();", "$('resetBtn').onclick=()=>{truthModel.reset();followModel.reset();")
s=s.replace("  $('carWidth').oninput=", "  $('truthMode').onchange=()=>{truthModel.reset();followModel.direction=new UwbFollowModel.Direction();modelResult=null;state.simLeftCmd=state.simRightCmd=0;};\n  $('carWidth').oninput=")
dest=root/'_build_recovery/pose_mode_simulator.html'
dest.write_text(s,encoding='utf-8')
(root/'_build_recovery/pose_mode_simulator.js').write_text(s.split('<script>')[1].split('</script>')[0],encoding='utf-8')
