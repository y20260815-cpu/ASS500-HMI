from pathlib import Path
r=Path(__file__).resolve().parent.parent
s=(r.parent/'uwb_follow_simulator.html').read_text(encoding='utf-8')
a=s.index('// UWB_FOLLOW_MODEL_BEGIN');b=s.index('// UWB_FOLLOW_MODEL_END',a)+len('// UWB_FOLLOW_MODEL_END')
s=s[:a]+(r/'tools/uwb_follow_model.js').read_text(encoding='utf-8').strip()+s[b:]
s=s.replace('<div id="poseModeLabel"></div>', '<label>&#50526;&#46244; &#54032;&#51221; <select id="hemisphereMode"><option value="reference">&#51452;&#54665; &#49884;&#54744;: TAG &#50526;&#46244; &#44592;&#51456; &#51228;&#44277;</option><option value="ranges">&#49892;&#52264; &#48708;&#44368;: A0/A1 &#51060;&#47141;&#47564;</option></select></label><div id="poseModeLabel"></div>')
s=s.replace("      modelResult=followModel.update", "      if(!serialPort&&$('hemisphereMode').value==='reference')config.behindReference=t.forward<UWB_FORWARD;\n      modelResult=followModel.update")
s=s.replace("    $('carWidth').disabled=!!serialPort;", "    $('carWidth').disabled=!!serialPort;\n    $('hemisphereMode').disabled=!!serialPort;")
s=s.replace("'A0 / A1 calculation'", "$('hemisphereMode').value==='reference'?'SIM TEST: TAG hemisphere + A0/A1':'A0/A1 history only'")
s=s.replace("  $('carWidth').oninput=", "  $('hemisphereMode').onchange=()=>{followModel.reset();modelResult=null;state.simLeftCmd=state.simRightCmd=0;};\n  $('carWidth').oninput=")
s=s.replace('R0913-3','R0913-4')
(r/'_build_recovery/reference_test_simulator.html').write_text(s,encoding='utf-8')
