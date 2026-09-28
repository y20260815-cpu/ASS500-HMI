from pathlib import Path
r=Path(__file__).resolve().parent.parent
s=(r.parent/'uwb_follow_simulator.html').read_text(encoding='utf-8')
s=s.replace('const shownTag=displayedTag();','const shownTag={x:state.tx,y:state.ty}; // Mouse-controlled physical TAG; control still uses ranges.')
s=s.replace("'TAG: A0 / A1'", "'A0 / A1 calculation'")
(r/'_build_recovery/mouse_tag_simulator.html').write_text(s,encoding='utf-8')
(r/'_build_recovery/mouse_tag_simulator.js').write_text(s.split('<script>')[1].split('</script>')[0],encoding='utf-8')
