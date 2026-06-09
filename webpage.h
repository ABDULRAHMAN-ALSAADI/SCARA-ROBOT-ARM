#ifndef WEBPAGE_H
#define WEBPAGE_H

#include <pgmspace.h>

const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no">
<meta http-equiv="Cache-Control" content="no-store, no-cache, must-revalidate, max-age=0">
<meta http-equiv="Pragma" content="no-cache">
<meta http-equiv="Expires" content="0">
<title>SCARA Robot v10</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:Arial,Segoe UI,sans-serif;background:#101114;color:#eee;min-height:100vh;font-size:16px}
header{padding:14px 18px;background:#17181d;border-bottom:1px solid #30313a;display:flex;align-items:center;gap:12px}
h1{font-size:20px;letter-spacing:.5px}
.dot{width:11px;height:11px;border-radius:50%;background:#777}
.wrap{max-width:1150px;margin:0 auto;padding:16px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}
@media(max-width:780px){.grid{grid-template-columns:1fr}.wrap{padding:10px}}
.card{background:#181a20;border:1px solid #30313a;border-radius:12px;padding:16px}
.card h2{font-size:13px;text-transform:uppercase;letter-spacing:1.5px;color:#8b8e99;margin-bottom:14px}
.tabs{display:flex;gap:8px;margin-bottom:14px;flex-wrap:wrap}
.tab{padding:10px 12px;border:1px solid #333743;background:#111217;color:#aaa;border-radius:8px;font-weight:700;cursor:pointer}
.tab.active{background:#e9e9e9;color:#111}
.panel{display:none}
.panel.active{display:block}
.row{margin-bottom:13px}
.rowHead{display:flex;justify-content:space-between;align-items:center;margin-bottom:5px;color:#bbb;font-size:14px;gap:8px}
.num{width:92px;background:#111217;border:1px solid #3a3d48;color:#fff;border-radius:7px;padding:7px;text-align:right;font-size:15px}
input[type=range]{width:100%;height:6px;background:#30313a;border-radius:3px;appearance:none;-webkit-appearance:none}
input[type=range]::-webkit-slider-thumb{appearance:none;-webkit-appearance:none;width:18px;height:18px;border-radius:50%;background:#fff;cursor:pointer}
.btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
button{border:none;border-radius:8px;padding:11px 13px;font-weight:800;cursor:pointer}
.primary{background:#fff;color:#111}
.dark{background:#30313a;color:#fff}
.red{background:#8d2424;color:#fff}
.yellow{background:#846300;color:#fff}
.green{background:#1f6f3a;color:#fff}
.small{padding:8px 10px;font-size:12px}
.status{margin-top:12px;min-height:22px;color:#8b8e99;font-size:14px}
.ok{color:#55d46f}.err{color:#ff6666}.busy{color:#e4c14f}
.warn{background:#2b2100;border:1px solid #705800;color:#e7c451;padding:10px;border-radius:8px;margin-bottom:12px;font-size:14px}
.actual{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-bottom:14px}
.box{background:#111217;border:1px solid #30313a;border-radius:8px;padding:10px;text-align:center}
.lbl{font-size:12px;color:#777;margin-bottom:2px}
.val{font-size:22px;font-weight:900}
.unit{font-size:11px;color:#666}
canvas{width:100%;height:330px;background:#111217;border:1px solid #30313a;border-radius:10px;display:block}
input.text{width:100%;background:#111217;border:1px solid #3a3d48;color:#fff;border-radius:7px;padding:9px;font-size:15px}
.formgrid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
@media(max-width:520px){.formgrid{grid-template-columns:1fr}}
.help{font-size:13px;line-height:1.45;color:#b0b3bd;background:#111217;border:1px solid #30313a;border-radius:8px;padding:10px;margin-bottom:12px}
.kv{font-family:monospace;color:#ddd;font-size:13px;line-height:1.6;white-space:pre-wrap;background:#111217;border:1px solid #30313a;border-radius:8px;padding:10px;margin-top:10px}
</style>
</head>
<body>
<header>
<div class="dot" id="dot"></div>
<h1>SCARA Robot v10</h1>
</header>

<div class="wrap">
<div class="grid">

<div class="card">
<h2>Control</h2>
<div class="warn" id="homeWarn">Home the robot before moving.</div>

<div class="tabs">
<button class="tab active" id="tabManual" onclick="setMode('manual')">Manual</button>
<button class="tab" id="tabIK" onclick="setMode('ik')">IK</button>
<button class="tab" id="tabSetup" onclick="setMode('setup')">Setup</button>
</div>

<div class="panel active" id="manualPanel">
  <div class="row">
    <div class="rowHead"><span>Arm 1</span><input class="num" id="a1Num" type="number" min="-180" max="180" step="1" value="0" onchange="numChange()"></div>
    <input type="range" id="a1" min="-180" max="180" step="1" value="0" oninput="sliderChange()">
  </div>
  <div class="row">
    <div class="rowHead"><span>Arm 2</span><input class="num" id="a2Num" type="number" min="-160" max="160" step="1" value="0" onchange="numChange()"></div>
    <input type="range" id="a2" min="-160" max="160" step="1" value="0" oninput="sliderChange()">
  </div>
  <div class="row">
    <div class="rowHead"><span>Z Axis</span><input class="num" id="zNum" type="number" min="-135" max="135" step="1" value="0" onchange="numChange()"></div>
    <input type="range" id="z" min="-135" max="135" step="1" value="0" oninput="sliderChange()">
  </div>
  <div class="btns">
    <button class="primary" onclick="sendMove()">Send Move</button>
    <button class="green" onclick="sendHome()">HOME</button>
    <button class="red" onclick="sendStop()">STOP</button>
    <button class="dark" onclick="sendOff()">OFF</button>
  </div>
</div>

<div class="panel" id="ikPanel">
  <div class="help">IK now calculates both valid SCARA postures and uses <b>Closest</b> by default. Use A2 Positive or A2 Negative only when you intentionally want that posture.</div>
  <div class="formgrid">
    <div><div class="rowHead">X mm</div><input class="text" id="ikX" type="number" value="180" step="1" oninput="ikPreview()"></div>
    <div><div class="rowHead">Y mm</div><input class="text" id="ikY" type="number" value="0" step="1" oninput="ikPreview()"></div>
    <div><div class="rowHead">Z mm</div><input class="text" id="ikZ" type="number" value="0" step="1"></div>
    <div><div class="rowHead">Solution</div>
      <select class="text" id="ikMode" onchange="ikPreview()">
        <option value="closest">Closest to current</option>
        <option value="positive">A2 positive</option>
        <option value="negative">A2 negative</option>
      </select>
    </div>
  </div>
  <div class="status" id="ikResult">Enter X/Y.</div>
  <div class="btns">
    <button class="primary" onclick="sendIK()">Send IK</button>
  </div>
</div>

<div class="panel" id="setupPanel">
  <div class="help">
    This version protects the limit switches. The switch angle is outside the usable range: A1 switch default = -195, A2 switch default = -185. Therefore A1=-180 and A2=-160 stop before touching the switch. If +180/+160 stops short, tune steps/degree slightly, Save, then HOME again.
  </div>

  <div class="formgrid">
    <div><div class="rowHead">A1 steps/deg</div><input class="text" id="a1spd" type="number" step="0.001"></div>
    <div><div class="rowHead">A2 steps/deg</div><input class="text" id="a2spd" type="number" step="0.001"></div>
    <div><div class="rowHead">A1 switch angle (default -195)</div><input class="text" id="a1home" type="number" step="0.1"></div>
    <div><div class="rowHead">A2 switch angle (default -185)</div><input class="text" id="a2home" type="number" step="0.1"></div>
    <div><div class="rowHead">Arm speed</div><input class="text" id="armspeed" type="number" step="100"></div>
    <div><div class="rowHead">Arm accel</div><input class="text" id="armaccel" type="number" step="100"></div>
  </div>

  <div class="btns">
    <button class="primary" onclick="applySetup()">Apply</button>
    <button class="green" onclick="saveCal()">Save</button>
    <button class="yellow" onclick="clearCal()">Clear Defaults</button>
  </div>

  <div class="btns">
    <button class="small dark" onclick="nudge('a1spd',1.005)">A1 reach +0.5%</button>
    <button class="small dark" onclick="nudge('a1spd',0.995)">A1 reach -0.5%</button>
    <button class="small dark" onclick="nudge('a2spd',1.005)">A2 reach +0.5%</button>
    <button class="small dark" onclick="nudge('a2spd',0.995)">A2 reach -0.5%</button>
  </div>

  <div class="help" style="margin-top:12px">
    Rule 1: keep switch angles more negative than the usable endpoints. Use A1=-195 and A2=-185 first. More negative means more safety space from the switch. Rule 2: if +180/+160 does not reach full end, press the matching <b>reach +0.5%</b>, Save, then HOME and test again. If it goes too far, use <b>reach -0.5%</b>.
  </div>

  <div class="kv" id="calBox">Loading calibration...</div>
</div>

<div class="status" id="msg">Waiting...</div>
</div>

<div class="card">
<h2>Actual Position</h2>
<div class="actual">
  <div class="box"><div class="lbl">X</div><div class="val" id="fkX">--</div><div class="unit">mm</div></div>
  <div class="box"><div class="lbl">Y</div><div class="val" id="fkY">--</div><div class="unit">mm</div></div>
  <div class="box"><div class="lbl">Z</div><div class="val" id="fkZ">--</div><div class="unit">mm</div></div>
</div>
<div class="actual">
  <div class="box"><div class="lbl">A1</div><div class="val" id="actA1">--</div><div class="unit">deg</div></div>
  <div class="box"><div class="lbl">A2</div><div class="val" id="actA2">--</div><div class="unit">deg</div></div>
  <div class="box"><div class="lbl">State</div><div class="val" id="stateTxt" style="font-size:16px">--</div><div class="unit">&nbsp;</div></div>
</div>
<canvas id="cv"></canvas>
</div>

</div>
</div>

<script>
const L1 = 136.5, L2 = 95.5;
let mode = 'manual';
let homed = false;
let moving = false;
let busy = false;
let userEditing = false;
let editTimer = null;

function setMode(m){
  mode=m;
  ['Manual','IK','Setup'].forEach(n=>{
    document.getElementById('tab'+n).className='tab'+(m===n.toLowerCase()?' active':'');
    document.getElementById(n.toLowerCase()+'Panel').className='panel'+(m===n.toLowerCase()?' active':'');
  });
  if(m==='setup') loadCal();
  if(m==='ik') ikPreview();
}

function setMsg(cls, text){
  const m=document.getElementById('msg');
  m.className='status '+cls;
  m.textContent=text;
}

function markEditing(){
  userEditing=true;
  if(editTimer) clearTimeout(editTimer);
  editTimer=setTimeout(()=>{userEditing=false;},2500);
}

function sliderChange(){
  markEditing();
  document.getElementById('a1Num').value=document.getElementById('a1').value;
  document.getElementById('a2Num').value=document.getElementById('a2').value;
  document.getElementById('zNum').value=document.getElementById('z').value;
  drawArm(+document.getElementById('a1').value,+document.getElementById('a2').value);
}

function numChange(){
  markEditing();
  document.getElementById('a1').value=document.getElementById('a1Num').value;
  document.getElementById('a2').value=document.getElementById('a2Num').value;
  document.getElementById('z').value=document.getElementById('zNum').value;
  drawArm(+document.getElementById('a1').value,+document.getElementById('a2').value);
}

async function sendMove(){
  if(busy)return;
  if(!homed){setMsg('err','Home first.');return;}
  busy=true;
  setMsg('busy','Sending move...');
  try{
    const a1=document.getElementById('a1').value;
    const a2=document.getElementById('a2').value;
    const z=document.getElementById('z').value;
    const r=await fetch('/move?a1='+a1+'&a2='+a2+'&z='+z);
    const t=await r.text();
    if(!r.ok){setMsg('err',t);}
    else{setMsg('ok','Move accepted.');}
  }catch(e){setMsg('err','Connection error.');}
  busy=false;
}

async function sendHome(){
  if(busy)return;
  busy=true;
  setMsg('busy','Homing...');
  try{
    const r=await fetch('/home');
    const t=await r.text();
    if(r.ok){
      homed=true;
      document.getElementById('homeWarn').style.display='none';
      document.getElementById('a1').value=0; document.getElementById('a1Num').value=0;
      document.getElementById('a2').value=0; document.getElementById('a2Num').value=0;
      document.getElementById('z').value=0; document.getElementById('zNum').value=0;
      setMsg('ok','Homed.');
    } else setMsg('err',t);
  }catch(e){setMsg('err','Connection error.');}
  busy=false;
}

async function sendStop(){
  try{
    await fetch('/stop');
    homed=false;
    document.getElementById('homeWarn').style.display='';
    setMsg('err','Stopped. Re-home required.');
  }catch(e){setMsg('err','Connection error.');}
}

async function sendOff(){
  try{
    await fetch('/off');
    homed=false;
    document.getElementById('homeWarn').style.display='';
    setMsg('err','Motors off. Re-home required.');
  }catch(e){setMsg('err','Connection error.');}
}

function ikSolve(x,y,force){
  const d2=x*x+y*y;
  const raw=(d2-L1*L1-L2*L2)/(2*L1*L2);
  if(raw<-1 || raw>1) return [];
  const c=Math.max(-1,Math.min(1,raw));
  const sols=[];
  [1,-1].forEach(sign=>{
    if(force==='positive' && sign<0) return;
    if(force==='negative' && sign>0) return;
    const s=sign*Math.sqrt(Math.max(0,1-c*c));
    const a2r=Math.atan2(s,c);
    const a1r=Math.atan2(y,x)-Math.atan2(L2*s,L1+L2*c);
    let a1=a1r*180/Math.PI;
    let a2=a2r*180/Math.PI;
    while(a1>180)a1-=360;
    while(a1<-180)a1+=360;
    if(a1>=-180 && a1<=180 && a2>=-160 && a2<=160) sols.push({a1,a2});
  });
  return sols;
}

function ikPreview(){
  const x=+document.getElementById('ikX').value;
  const y=+document.getElementById('ikY').value;
  const mode=document.getElementById('ikMode').value;
  const res=document.getElementById('ikResult');
  const sols=ikSolve(x,y,mode);
  if(sols.length===0){
    res.className='status err';
    res.textContent='No valid solution.';
    return;
  }
  let s=sols[0];
  res.className='status ok';
  res.textContent='Preview A1 '+s.a1.toFixed(1)+'°  A2 '+s.a2.toFixed(1)+'°';
  drawArm(s.a1,s.a2);
}

async function sendIK(){
  if(busy)return;
  if(!homed){setMsg('err','Home first.');return;}
  busy=true;
  setMsg('busy','Sending IK...');
  try{
    const x=document.getElementById('ikX').value;
    const y=document.getElementById('ikY').value;
    const z=document.getElementById('ikZ').value;
    const m=document.getElementById('ikMode').value;
    const r=await fetch('/ik?x='+x+'&y='+y+'&z='+z+'&mode='+m);
    const d=await r.json();
    if(d.ok){
      document.getElementById('a1').value=d.a1.toFixed(0);
      document.getElementById('a1Num').value=d.a1.toFixed(0);
      document.getElementById('a2').value=d.a2.toFixed(0);
      document.getElementById('a2Num').value=d.a2.toFixed(0);
      document.getElementById('z').value=z;
      document.getElementById('zNum').value=z;
      setMsg('ok','IK accepted: A1 '+d.a1.toFixed(1)+' A2 '+d.a2.toFixed(1));
    } else setMsg('err',d.err||'IK failed');
  }catch(e){setMsg('err','Connection error.');}
  busy=false;
}

async function loadCal(){
  try{
    const d=await (await fetch('/caljson')).json();
    document.getElementById('a1spd').value=d.a1spd;
    document.getElementById('a2spd').value=d.a2spd;
    document.getElementById('a1home').value=d.a1home;
    document.getElementById('a2home').value=d.a2home;
    document.getElementById('armspeed').value=d.armspeed;
    document.getElementById('armaccel').value=d.armaccel;
    showCal(d);
  }catch(e){document.getElementById('calBox').textContent='Calibration load failed.';}
}

function showCal(d){
  document.getElementById('calBox').textContent=
    'A1 steps/deg: '+d.a1spd+'\n'+
    'A2 steps/deg: '+d.a2spd+'\n'+
    'A1 switch angle: '+d.a1home+'\n'+
    'A2 switch angle: '+d.a2home+'\n'+
    'Arm speed: '+d.armspeed+'\n'+
    'Arm accel: '+d.armaccel;
}

async function setParam(name,value){
  const r=await fetch('/setparam?name='+encodeURIComponent(name)+'&value='+encodeURIComponent(value));
  const d=await r.json();
  showCal(d);
  homed=false;
  document.getElementById('homeWarn').style.display='';
  return d;
}

async function applySetup(){
  try{
    await setParam('a1spd',document.getElementById('a1spd').value);
    await setParam('a2spd',document.getElementById('a2spd').value);
    await setParam('a1home',document.getElementById('a1home').value);
    await setParam('a2home',document.getElementById('a2home').value);
    await setParam('armspeed',document.getElementById('armspeed').value);
    const d=await setParam('armaccel',document.getElementById('armaccel').value);
    setMsg('ok','Applied. Press Save, then HOME before moving.');
    showCal(d);
  }catch(e){setMsg('err','Apply failed.');}
}

async function saveCal(){
  try{
    const d=await (await fetch('/savecal')).json();
    showCal(d);
    setMsg('ok','Saved. HOME required if calibration changed.');
  }catch(e){setMsg('err','Save failed.');}
}

async function clearCal(){
  try{
    const d=await (await fetch('/clearcal')).json();
    showCal(d);
    loadCal();
    homed=false;
    document.getElementById('homeWarn').style.display='';
    setMsg('ok','Defaults restored. HOME required.');
  }catch(e){setMsg('err','Clear failed.');}
}

async function nudge(name,mul){
  try{
    const d=await (await fetch('/nudgeparam?name='+name+'&mul='+mul)).json();
    showCal(d);
    loadCal();
    homed=false;
    document.getElementById('homeWarn').style.display='';
    setMsg('ok','Adjusted '+name+'. Press Save, then HOME.');
  }catch(e){setMsg('err','Nudge failed.');}
}

function drawArm(a1,a2){
  const c=document.getElementById('cv');
  const ctx=c.getContext('2d');
  const rect=c.getBoundingClientRect();
  c.width=rect.width;
  c.height=330;
  const W=c.width,H=c.height;
  const sc=(Math.min(W,H)*0.38)/(L1+L2);
  const cx=W/2, cy=H*0.54;
  ctx.clearRect(0,0,W,H);
  [L1+L2, Math.abs(L1-L2)].forEach(r=>{
    ctx.beginPath();ctx.arc(cx,cy,r*sc,0,Math.PI*2);
    ctx.setLineDash([4,4]);ctx.strokeStyle='#30313a';ctx.lineWidth=1;ctx.stroke();ctx.setLineDash([]);
  });
  const r1=a1*Math.PI/180, r2=a2*Math.PI/180;
  const ex=cx+L1*Math.cos(r1)*sc, ey=cy-L1*Math.sin(r1)*sc;
  const px=ex+L2*Math.cos(r1+r2)*sc, py=ey-L2*Math.sin(r1+r2)*sc;
  ctx.lineCap='round';
  ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(ex,ey);ctx.strokeStyle='#aaa';ctx.lineWidth=5;ctx.stroke();
  ctx.beginPath();ctx.moveTo(ex,ey);ctx.lineTo(px,py);ctx.strokeStyle='#ddd';ctx.lineWidth=4;ctx.stroke();
  [[cx,cy,7],[ex,ey,6],[px,py,6]].forEach(p=>{ctx.beginPath();ctx.arc(p[0],p[1],p[2],0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill();});
}

async function sync(){
  try{
    const d=await (await fetch('/position')).json();
    homed=d.homed;
    moving=d.moving;
    document.getElementById('dot').style.cssText='background:#54d46f;box-shadow:0 0 8px #54d46f';
    document.getElementById('homeWarn').style.display=homed?'none':'';
    document.getElementById('actA1').textContent=d.a1.toFixed(1);
    document.getElementById('actA2').textContent=d.a2.toFixed(1);
    document.getElementById('fkX').textContent=d.x.toFixed(1);
    document.getElementById('fkY').textContent=d.y.toFixed(1);
    document.getElementById('fkZ').textContent=d.z.toFixed(1);
    document.getElementById('stateTxt').textContent=homed?(moving?'MOVING':'READY'):'NOT HOMED';
    if(!userEditing && homed && !moving){
      // Do not force sliders while the user is choosing a new target.
      // Only update the drawing to the actual robot pose.
      drawArm(d.a1,d.a2);
    }
    if(!busy) setMsg('ok',homed?'Connected':'Connected. Home required.');
  }catch(e){
    document.getElementById('dot').style.cssText='background:#b43b3b;box-shadow:0 0 8px #b43b3b';
    setMsg('err','Disconnected.');
  }
}

window.addEventListener('load',()=>{
  drawArm(0,0);
  loadCal();
  sync();
  setInterval(sync,1500);
});
</script>
</body>
</html>
)rawhtml";

#endif
