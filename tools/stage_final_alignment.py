from pathlib import Path
r=Path(__file__).resolve().parent.parent
m=r/'tools/uwb_follow_model.js'
model=m.read_text(encoding='utf-8').replace('reset(){this.divergence=', 'reset(){this.finalAlignment=new FinalAlignment();this.divergence=')
m.write_text(model,encoding='utf-8')
s=(r/'_build_recovery/tag_stop_simulator.html').read_text(encoding='utf-8')
a=s.index('// UWB_FOLLOW_MODEL_BEGIN');b=s.index('// UWB_FOLLOW_MODEL_END',a)+len('// UWB_FOLLOW_MODEL_END')
s=s[:a]+model.strip()+s[b:]
s=s.replace('R0913-5','R0913-6')
(r/'_build_recovery/final_alignment_simulator.html').write_text(s,encoding='utf-8')
