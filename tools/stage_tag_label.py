from pathlib import Path
r=Path(__file__).resolve().parent.parent
s=(r.parent/'uwb_follow_simulator.html').read_text(encoding='utf-8')
s=s.replace("const tw=ctx.measureText(text).width, lx=clamp((cx+tx)/2+8,4,Math.max(4,view.w-tw-12)),ly=clamp((cy+ty)/2-8,18,view.h-8);", "const tw=ctx.measureText(text).width, lx=clamp(tx+18,4,Math.max(4,view.w-tw-12)),ly=clamp(ty+24,18,view.h-8);")
assert 'lx=clamp(tx+18' in s
(r/'_build_recovery/tag_stop_simulator.html').write_text(s,encoding='utf-8')
