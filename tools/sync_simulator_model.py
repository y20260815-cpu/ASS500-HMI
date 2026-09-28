from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
model_path = root / "tools" / "uwb_follow_model.js"
sim_path = root.parent / "uwb_follow_simulator.html"
model = model_path.read_text(encoding="utf-8")
html = sim_path.read_text(encoding="utf-8")
body = (root / "tools" / "simulator_body_ko.html").read_text(encoding="utf-8")
html = html[:html.index("<body>")] + body + html[html.index("<script>") + len("<script>"):]
html = re.sub(r"<title>.*?(?:</title>|\n)", "<title>UWB 이동차 추종 시뮬레이터</title>\n", html, count=1)
begin = "// UWB_FOLLOW_MODEL_BEGIN"
end = "// UWB_FOLLOW_MODEL_END"
start = html.index(begin)
finish = html.index(end, start) + len(end)
html = html[:start] + model.rstrip() + html[finish:]
html = re.sub(r"R0913-\d+", "R0913-38", html, count=1)
# Repair a legacy mojibake-damaged template literal so the standalone HTML
# remains executable after model synchronization.
html = re.sub(
    r"\s*\$\('csvStatus'\)\.textContent=message\|\|.*?toLocaleString\('ko-KR'\).*?;",
    "\n    $('csvStatus').textContent=message||`${csvRecording.active?'기록 중':'일시정지'} · ${csvRecording.rows.length.toLocaleString('ko-KR')}행`;",
    html,
    count=1,
)
html = re.sub(
    r"csvRecording\.active=false;refreshCsvStatus\('100,000.*?;",
    "csvRecording.active=false;refreshCsvStatus('100,000 row limit reached. Download or clear the CSV.');",
    html,
    count=1,
)
replacements = {
    "$('csvRecordBtn').textContent=": "    $('csvRecordBtn').textContent=csvRecording.active?'기록 정지':csvRecording.rows.length?'기록 계속':'기록 시작';",
    "    if(!confirm(": "    if(!confirm('CSV 기록을 비울까요? 다운로드하지 않은 파일은 유지되지 않습니다.'))return;",
    "        appendSerialLine(": "        appendSerialLine('설정 동기화 완료');",
    "    slipEl.innerHTML=": "    slipEl.innerHTML=`<i class=\"slip-led\"></i>${slipPct<35?'정상':slipPct<65?'주의':'과다'} · ${slipPct}%`;",
    "  canvas.addEventListener('pointermove'": "  canvas.addEventListener('pointermove',e=>{if(!state.pinned)setTag(e)});canvas.addEventListener('pointerdown',e=>{setTag(e);state.pinned=!state.pinned;document.querySelector('.hint').textContent=state.pinned?'Tag 위치 고정됨 · 다시 클릭하면 해제':'마우스를 움직여 Tag를 이동 · 클릭하면 위치 고정';});",
    "  $('runBtn').onclick=": "  $('runBtn').onclick=()=>{state.running=!state.running;$('runBtn').textContent=state.running?'일시정지':'계속';$('runLabel').textContent=state.running?'SIM RUNNING':'SIM PAUSED';document.querySelector('.dot').style.background=state.running?'var(--lime)':'var(--orange)';};",
}
lines = html.splitlines()
for i, line in enumerate(lines):
    for prefix, replacement in replacements.items():
        if prefix in line:
            lines[i] = replacement
            break
    if "const bands=[" in line:
        lines[i] = "    const bands=[{name:'속도 m/s',max:Math.max(1,+$('maxSpeed').value),min:-Math.max(1,+$('maxSpeed').value)},{name:'남은 거리 m',max:distMax,min:0},{name:'모터 rpm',max:2700,min:-2700},{name:'전압 V',max:Math.max(30,...state.history.map(p=>p.voltage||0))*1.1,min:0},{name:'전류 A',max:Math.max(5,...state.history.map(p=>p.current||0))*1.1,min:0}];"
    if "timeline.addEventListener('pointermove'" in line:
        lines[i] = "  timeline.addEventListener('pointermove',e=>{if(!state.history.length)return;const r=timeline.getBoundingClientRect(),end=Math.max(10,state.elapsed),start=Math.max(0,end-60),t=start+clamp((e.clientX-r.left-72)/(r.width-84),0,1)*(end-start);let p=state.history[0];for(const q of state.history)if(Math.abs(q.t-t)<Math.abs(p.t-t))p=q;const tip=$('chartTip'),actual=p.actualM1===null?'수신 없음':`${Math.round(p.actualM1)} / ${Math.round(p.actualM2)} rpm`;tip.innerHTML=`EMB (driver relay): ${p.brakeApplied==null?'UNKNOWN':p.brakeApplied?'APPLIED':'RELEASED'}<br>CAN ${p.driveState||'--'} | RX age ${p.canAge??'--'} ms | TX #${p.txSequence??'--'} ${p.txAccepted===false?'FAIL':''}<br>시간 ${p.t.toFixed(1)} s · ${p.aligning?'제자리 정렬':'전진 조향'}<br>요청 속도 ${p.requested.toFixed(2)} m/s<br>최종 명령 ${p.command.toFixed(2)} m/s<br>추정 속도 ${p.speed.toFixed(2)} m/s<br>남은 거리 ${p.remaining.toFixed(2)} m<br>M1/M2 명령 ${Math.round(p.m1)} / ${Math.round(p.m2)} rpm<br>M1/M2 드라이버 ${actual}<br>전압 ${p.voltage==null?'--':p.voltage.toFixed(1)+' V'}<br>전류 ${p.current==null?'--':p.current.toFixed(1)+' A'}`;tip.style.display='block';tip.style.left=Math.min(r.width-210,Math.max(8,e.clientX-r.left+12))+'px';tip.style.top=Math.max(34,e.clientY-r.top-75)+'px';});"
    if "$('poseModeLabel').textContent=" in line:
        lines[i] = "    $('poseModeLabel').textContent=serialPort?'TAG: LCD/UART 계산값':'A0/A1 전방 해만 사용';"
html = "\n".join(lines) + ("\n" if html.endswith("\n") else "")
html = html.replace("'UART ?곌껐'", "'UART 연결'").replace("'UART ?댁젣'", "'UART 해제'")
html = html.replace("'Chrome/Edge Web Serial API媛 ?꾩슂?⑸땲??'", "'Chrome/Edge Web Serial API가 필요합니다.'")
html = html.replace("'UART ?곌껐 ?ㅽ뙣: '", "'UART 연결 실패: '")
sim_path.write_text(html, encoding="utf-8")
print(sim_path)
