from pathlib import Path
import re
r=Path(__file__).resolve().parent.parent
m=r/'tools/uwb_follow_model.js'
model=m.read_text(encoding='utf-8').replace('reset(){this.width=0;', 'reset(){this.divergence=new Divergence();this.width=0;')
m.write_text(model,encoding='utf-8')
s=(r.parent/'uwb_follow_simulator.html').read_text(encoding='utf-8')
a=s.index('// UWB_FOLLOW_MODEL_BEGIN');b=s.index('// UWB_FOLLOW_MODEL_END',a)+len('// UWB_FOLLOW_MODEL_END')
s=s[:a]+model.strip()+s[b:]
s=s.replace('width:CAR_W/10,alpha:', 'leftCommand:state.simLeftCmd,rightCommand:state.simRightCmd,width:CAR_W/10,alpha:')
s=re.sub(r'<select id="hemisphereMode">.*?</select>', '<select id="hemisphereMode"><option value="ranges">&#51204;&#48169; &#49884;&#51089; / &#44144;&#47532; &#51060;&#47141; &#48372;&#51221;</option><option value="reference">&#51452;&#54665; &#49884;&#54744;: TAG &#50526;&#46244; &#44592;&#51456;</option></select>',s)
s=s.replace('R0913-4','R0913-5')
(r/'_build_recovery/divergence_simulator.html').write_text(s,encoding='utf-8')
