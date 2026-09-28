const assert=require('node:assert/strict'),{Divergence}=require('../tools/uwb_follow_model.js');
function sequence(delta,left=500,right=500,enabled=true){const d=new Divergence();let result=false;for(let i=0;i<15;i++)result=d.update(400+delta*i,0,false,left/10,right/10,enabled,i*100)||result;return result;}
assert.equal(sequence(5),true);
assert.equal(sequence(-5),false);
assert.equal(sequence(5,0,0),false);
assert.equal(sequence(5,500,-500),false);
assert.equal(sequence(5,500,500,false),false);
assert.equal(sequence(50),false);
const d=new Divergence();let hits=0;for(let i=0;i<25;i++)if(d.update(400+5*i,0,false,50,50,true,i*100))hits++;
assert.equal(hits,1);
console.log('Divergence correction, approaching, stationary, pivot, disabled, jump rejection and cooldown passed.');
