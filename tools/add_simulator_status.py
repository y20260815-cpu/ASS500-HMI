from pathlib import Path
p=Path(__file__).resolve().parent.parent.parent/'uwb_follow_simulator.html'
s=p.read_text(encoding='utf-8')
s=s.replace('<span id="runLabel">SIM RUNNING</span>', '<span id="runLabel">SIM RUNNING</span><span style="font-size:11px">R0913-3</span>')
s=s.replace('  function frame(now){', '''  function showFollowStatus(){
    const label=$('runLabel');
    if(!state.running){label.textContent='PAUSED';return;}
    if(serialPort){label.textContent=performance.now()-latestCanCommand.receivedAt>300?'UART: WAIT CTRL':'UART: CTRL';return;}
    if(!modelResult?.pose.valid){label.textContent='SIM: RANGE INVALID';return;}
    label.textContent=modelResult.left||modelResult.right?'SIM: FOLLOWING':'SIM: STOP';
  }
  window.addEventListener('error',e=>{$('runLabel').textContent='ERROR: '+e.message;});
  function frame(now){''')
s=s.replace('drawEstimatedPose();drawChart();requestAnimationFrame(frame);','drawEstimatedPose();drawChart();showFollowStatus();requestAnimationFrame(frame);')
p.write_text(s,encoding='utf-8')
