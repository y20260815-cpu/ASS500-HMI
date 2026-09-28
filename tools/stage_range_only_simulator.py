from pathlib import Path
import re
r=Path(__file__).resolve().parent.parent
s=(r.parent/'uwb_follow_simulator.html').read_text(encoding='utf-8')
s=re.sub(r'<label><input id="truthMode".*?</label>', '', s)
s=s.replace('const followModel=new UwbFollowModel.Model(),truthModel=new UwbFollowModel.Model();','const followModel=new UwbFollowModel.Model();')
s=s.replace('  let driveResult=null;\n','')
s=re.sub(r'      truthModel.pose=.*?\n      truthModel.accepted=.*?\n      driveResult=.*?;\n','',s)
s=s.replace('const q=driveResult,','const q=modelResult,')
s=re.sub(r"    if\(!serialPort&&\$\('truthMode'\).checked\)\{.*?\n    \}", '', s, flags=re.S)
s=s.replace("    $('truthMode').disabled=!!serialPort;\n",'')
s=re.sub(r"    \$\('poseModeLabel'\).textContent=.*?;", "    $('poseModeLabel').textContent=serialPort?'TAG: LCD / UART':'TAG: A0 / A1';",s)
s=re.sub(r"  \$\('truthMode'\).onchange=.*?\n",'',s)
s=s.replace('truthModel.reset();','')
start=s.index('  function renderData(t,c){');end=s.index('  function slipColor',start)
part=s[start:end]
part=part.replace("    const precise=v=>", "    const p=displayedTag();\n    const shown=p.valid?{distance:p.distance*10,bearing:p.bearing*Math.PI/180,forward:p.distance*10*Math.cos(p.bearing*Math.PI/180),lateral:p.distance*10*Math.sin(p.bearing*Math.PI/180)}:{distance:NaN,bearing:NaN,forward:NaN,lateral:NaN};\n    const precise=v=>!Number.isFinite(v)?'--':")
part=part.replace('precise(state.tx)', 'precise(p.x)').replace('precise(state.ty)', 'precise(p.y)')
for key in ['distance','bearing','forward','lateral']:part=part.replace('t.'+key,'shown.'+key)
part=part.replace('fmt(shown.distance)',"(p.valid?fmt(shown.distance):'--')")
s=s[:start]+part+s[end:]
s=s.replace('remaining:Math.max(0,t.distance-', 'remaining:displayedTag().valid?Math.max(0,displayedTag().distance*10-')
s=s.replace('value*10))/1000,m1:', 'value*10))/1000:null,m1:')
assert 'truthMode' not in s and 'truthModel' not in s and 'driveResult' not in s
(r/'_build_recovery/range_only_simulator.html').write_text(s,encoding='utf-8')
(r/'_build_recovery/range_only_simulator.js').write_text(s.split('<script>')[1].split('</script>')[0],encoding='utf-8')
