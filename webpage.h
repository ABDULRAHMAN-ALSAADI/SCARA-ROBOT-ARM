#ifndef WEBPAGE_H
#define WEBPAGE_H

#include <pgmspace.h>

const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>SCARA Robot</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',sans-serif;background:#111;color:#eee;min-height:100vh}
header{background:#181818;border-bottom:1px solid #333;padding:14px 24px;display:flex;align-items:center;gap:12px}
header h1{font-size:1.2rem;color:#fff;letter-spacing:1px}
.dot{width:10px;height:10px;border-radius:50%;background:#555}
.container{display:grid;grid-template-columns:1fr 1fr;gap:16px;padding:20px;max-width:1100px;margin:0 auto}
@media(max-width:700px){.container{grid-template-columns:1fr}}
.card{background:#181818;border:1px solid #2a2a2a;border-radius:10px;padding:18px}
.card h2{font-size:.78rem;text-transform:uppercase;letter-spacing:1.5px;color:#666;margin-bottom:14px}
.mode-tabs{display:flex;gap:0;margin-bottom:16px;border:1px solid #333;border-radius:8px;overflow:hidden}
.mode-tab{flex:1;padding:8px;text-align:center;font-size:.82rem;font-weight:600;cursor:pointer;background:#111;color:#666;border:none;transition:.2s}
.mode-tab.active{background:#fff;color:#111}
.axis-row{margin-bottom:14px}
.axis-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:5px;font-size:.85rem}
.axis-header label{color:#aaa}
.axis-input{width:70px;background:#111;border:1px solid #333;color:#fff;border-radius:5px;padding:4px 6px;font-size:.85rem;text-align:right}
.axis-input:focus{outline:none;border-color:#fff}
input[type=range]{width:100%;height:5px;-webkit-appearance:none;background:#2a2a2a;border-radius:3px;outline:none}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:16px;height:16px;border-radius:50%;background:#fff;cursor:pointer}
.btn-row{display:flex;gap:8px;margin-top:14px}
.btn{padding:10px 18px;border:none;border-radius:8px;font-size:.85rem;cursor:pointer;font-weight:700;transition:.15s}
.btn-send{background:#fff;color:#111;flex:1}
.btn-send:hover{background:#ddd}
.btn-send:disabled{background:#333;color:#666;cursor:not-allowed}
.btn-home{background:#333;color:#fff;min-width:80px}
.btn-home:hover{background:#444}
.btn-home:disabled{background:#222;color:#555;cursor:not-allowed}
.btn-stop{background:#8b0000;color:#fff;min-width:80px}
.btn-stop:hover{background:#aa1111}
.warn{background:#1a1500;border:1px solid #555;border-radius:6px;padding:8px 12px;margin-bottom:12px;font-size:.78rem;color:#aa8800}
.warn.hidden{display:none}
.status{margin-top:10px;font-size:.75rem;color:#555;min-height:18px}
.status.ok{color:#4a4}
.status.err{color:#a44}
.status.busy{color:#aa8800}
.fk-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;margin-bottom:14px}
.fk-box{background:#111;border:1px solid #2a2a2a;border-radius:6px;padding:10px;text-align:center}
.fk-box .lbl{font-size:.7rem;color:#555;margin-bottom:2px}
.fk-box .val{font-size:1.3rem;font-weight:700;color:#fff}
.fk-box .unit{font-size:.65rem;color:#444}
canvas{width:100%;border-radius:8px;background:#111;border:1px solid #2a2a2a;display:block}
.ik-panel{display:none}
.ik-row{display:flex;gap:8px;margin-bottom:10px;align-items:center}
.ik-row label{color:#aaa;font-size:.82rem;width:60px;flex-shrink:0}
.ik-row input{flex:1;background:#111;border:1px solid #333;color:#fff;border-radius:5px;padding:6px 8px;font-size:.9rem}
.ik-row input:focus{outline:none;border-color:#fff}
.ik-result{background:#111;border:1px solid #2a2a2a;border-radius:6px;padding:10px;margin-bottom:10px;font-size:.8rem;color:#666;min-height:36px}
.ik-result.ok{color:#4a4}
.ik-result.err{color:#a44}
.small-note{font-size:.72rem;color:#555;margin-top:8px;line-height:1.35}
</style>
</head>
<body>
<header>
  <div class="dot" id="dot"></div>
  <h1>SCARA Robot</h1>
</header>

<div class="container">
  <div class="card">
    <h2>Joint Control</h2>
    <div class="warn" id="warn">Home the robot before moving.</div>

    <div class="mode-tabs">
      <button class="mode-tab active" id="tabManual" onclick="setMode('manual')">Manual</button>
      <button class="mode-tab" id="tabRT" onclick="setMode('realtime')">Real-Time</button>
      <button class="mode-tab" id="tabIK" onclick="setMode('ik')">IK</button>
    </div>

    <div id="ikPanel" class="ik-panel">
      <div class="ik-row"><label>X (mm)</label><input type="number" id="ikX" value="200" step="1" oninput="ikPreview()"></div>
      <div class="ik-row"><label>Y (mm)</label><input type="number" id="ikY" value="0" step="1" oninput="ikPreview()"></div>
      <div class="ik-row"><label>Z (mm)</label><input type="number" id="ikZ" value="0" step="1" min="-135" max="135"></div>
      <div class="ik-row"><label>Elbow</label>
        <button id="elbowPlusBtn" style="flex:1;padding:6px;background:#fff;color:#111;border:none;border-radius:5px 0 0 5px;font-weight:700;cursor:pointer" onclick="setElbow(true)">+ Up</button>
        <button id="elbowMinusBtn" style="flex:1;padding:6px;background:#333;color:#aaa;border:none;border-radius:0 5px 5px 0;font-weight:700;cursor:pointer" onclick="setElbow(false)">- Down</button>
      </div>
      <div class="ik-result" id="ikResult">Enter X, Y coordinates above</div>
    </div>

    <div class="axis-row">
      <div class="axis-header"><label>Arm 1 (Shoulder)</label><input class="axis-input" id="a1Num" type="number" min="-180" max="180" value="0" onchange="numChange()"></div>
      <input type="range" id="a1" min="-180" max="180" value="0" step="1" oninput="sliderChange()">
    </div>

    <div class="axis-row">
      <div class="axis-header"><label>Arm 2 (Elbow)</label><input class="axis-input" id="a2Num" type="number" min="-160" max="160" value="0" onchange="numChange()"></div>
      <input type="range" id="a2" min="-160" max="160" value="0" step="1" oninput="sliderChange()">
    </div>

    <div class="axis-row">
      <div class="axis-header"><label>Z Axis (mm)</label><input class="axis-input" id="zNum" type="number" min="-135" max="135" value="0" onchange="numChange()"></div>
      <input type="range" id="z" min="-135" max="135" value="0" step="1" oninput="sliderChange()">
    </div>

    <div class="btn-row">
      <button class="btn btn-send" id="sendBtn" onclick="sendMove()">Send to Robot</button>
      <button class="btn btn-home" id="homeBtn" onclick="sendHome()">HOME</button>
      <button class="btn btn-stop" id="stopBtn" onclick="sendStop()">STOP</button>
    </div>

    <div class="status" id="msg">Waiting...</div>
    <div class="small-note">STOP cancels the current target and disables the motors. Press HOME again if position is no longer trusted.</div>
  </div>

  <div class="card">
    <h2>End Effector</h2>
    <div class="fk-grid">
      <div class="fk-box"><div class="lbl">X</div><div class="val" id="fkX">--</div><div class="unit">mm</div></div>
      <div class="fk-box"><div class="lbl">Y</div><div class="val" id="fkY">--</div><div class="unit">mm</div></div>
      <div class="fk-box"><div class="lbl">Z</div><div class="val" id="fkZ">--</div><div class="unit">mm</div></div>
    </div>
    <canvas id="cv" height="320"></canvas>
  </div>
</div>

<script>
// ── Constants. Must match the Arduino sketch. ──
const L1 = 136.5;
const L2 = 95.5;

const A1_MIN = -180;
const A1_MAX = 180;
const A2_MIN = -160;
const A2_MAX = 160;
const Z_MIN  = -135;
const Z_MAX  = 135;

let mode = 'manual';
let homed = false;
let busy = false;
let rtTimer = null;
let elbowUp = true;
let robotMoving = false;

// ── Mode switching ──
function setMode(m) {
  mode = m;

  document.getElementById('tabManual').className = 'mode-tab' + (m === 'manual' ? ' active' : '');
  document.getElementById('tabRT').className     = 'mode-tab' + (m === 'realtime' ? ' active' : '');
  document.getElementById('tabIK').className     = 'mode-tab' + (m === 'ik' ? ' active' : '');

  document.getElementById('ikPanel').style.display = m === 'ik' ? 'block' : 'none';

  const jointRows = document.querySelectorAll('.axis-row');
  jointRows.forEach(r => r.style.display = m === 'ik' ? 'none' : '');

  const btn = document.getElementById('sendBtn');

  if (m === 'ik') {
    btn.style.display = '';
    btn.textContent   = 'Send IK';
    btn.onclick       = sendIK;
  } else {
    btn.style.display = m === 'manual' ? '' : 'none';
    btn.textContent   = 'Send to Robot';
    btn.onclick       = sendMove;
  }
}

// ── Clamp helper ──
function clamp(v, mn, mx) {
  return Math.max(mn, Math.min(mx, v));
}

// ── Sync sliders and number inputs ──
function syncInputs() {
  let a1 = clamp(+document.getElementById('a1').value, A1_MIN, A1_MAX);
  let a2 = clamp(+document.getElementById('a2').value, A2_MIN, A2_MAX);
  let z  = clamp(+document.getElementById('z').value,  Z_MIN,  Z_MAX);

  document.getElementById('a1').value = a1;
  document.getElementById('a2').value = a2;
  document.getElementById('z').value  = z;

  document.getElementById('a1Num').value = a1;
  document.getElementById('a2Num').value = a2;
  document.getElementById('zNum').value  = z;

  updateFK(a1, a2, z);
}

function numChange() {
  document.getElementById('a1').value = document.getElementById('a1Num').value;
  document.getElementById('a2').value = document.getElementById('a2Num').value;
  document.getElementById('z').value  = document.getElementById('zNum').value;

  syncInputs();

  if (mode === 'realtime') {
    scheduleRT();
  }
}

function sliderChange() {
  syncInputs();

  if (mode === 'realtime') {
    scheduleRT();
  }
}

// ── Forward Kinematics display ──
function updateFK(a1, a2, z) {
  const r1 = a1 * Math.PI / 180;
  const r2 = a2 * Math.PI / 180;

  const x = L1 * Math.cos(r1) + L2 * Math.cos(r1 + r2);
  const y = L1 * Math.sin(r1) + L2 * Math.sin(r1 + r2);

  document.getElementById('fkX').textContent = x.toFixed(1);
  document.getElementById('fkY').textContent = y.toFixed(1);
  document.getElementById('fkZ').textContent = z.toFixed(1);

  drawArm(a1, a2);
}

// ── Canvas arm drawing ──
function drawArm(a1, a2) {
  const c = document.getElementById('cv');
  const ctx = c.getContext('2d');

  c.width = c.offsetWidth;
  c.height = 320;

  const W = c.width;
  const H = c.height;

  const sc = (Math.min(W, H) * 0.4) / (L1 + L2);
  const cx = W / 2;
  const cy = H * 0.5;

  ctx.clearRect(0, 0, W, H);

  [L1 + L2, Math.abs(L1 - L2)].forEach(r => {
    ctx.beginPath();
    ctx.arc(cx, cy, r * sc, 0, Math.PI * 2);
    ctx.strokeStyle = '#222';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.stroke();
    ctx.setLineDash([]);
  });

  const r1 = a1 * Math.PI / 180;
  const r2 = a2 * Math.PI / 180;

  const ex = cx + L1 * Math.cos(r1) * sc;
  const ey = cy - L1 * Math.sin(r1) * sc;
  const px = ex + L2 * Math.cos(r1 + r2) * sc;
  const py = ey - L2 * Math.sin(r1 + r2) * sc;

  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(ex, ey);
  ctx.strokeStyle = '#888';
  ctx.lineWidth = 4;
  ctx.lineCap = 'round';
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(px, py);
  ctx.strokeStyle = '#bbb';
  ctx.lineWidth = 3;
  ctx.stroke();

  [[cx, cy, 6, '#fff'], [ex, ey, 4, '#aaa'], [px, py, 5, '#fff']].forEach(([x, y, r, col]) => {
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fillStyle = col;
    ctx.fill();
  });

  ctx.beginPath();
  ctx.arc(px, py, 10, 0, Math.PI * 2);
  ctx.strokeStyle = '#ffffff44';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  ctx.font = '10px monospace';
  ctx.fillStyle = '#555';
  ctx.fillText('Base', cx + 10, cy + 4);
  ctx.fillStyle = '#aaa';
  ctx.fillText('EE', px + 10, py - 4);
}

// ── Realtime movement ──
function scheduleRT() {
  if (rtTimer) {
    clearTimeout(rtTimer);
  }

  rtTimer = setTimeout(() => {
    if (!homed) {
      setMsg('err', 'Home first!');
      return;
    }

    const a1 = document.getElementById('a1').value;
    const a2 = document.getElementById('a2').value;
    const z  = document.getElementById('z').value;

    fetch('/move_rt?a1=' + a1 + '&a2=' + a2 + '&z=' + z)
      .then(r => r.json())
      .then(d => {
        if (!d.ok) {
          setMsg('err', d.err || 'Move failed');
        } else {
          setMsg('busy', 'Moving...');
        }
      })
      .catch(() => setMsg('err', 'Connection error'));
  }, 250);
}

// ── Elbow toggle ──
function setElbow(up) {
  elbowUp = up;

  document.getElementById('elbowPlusBtn').style.background  = up ? '#fff' : '#333';
  document.getElementById('elbowPlusBtn').style.color       = up ? '#111' : '#aaa';
  document.getElementById('elbowMinusBtn').style.background = up ? '#333' : '#fff';
  document.getElementById('elbowMinusBtn').style.color      = up ? '#aaa' : '#111';

  ikPreview();
}

// ── IK preview ──
function ikPreview() {
  const x = +document.getElementById('ikX').value;
  const y = +document.getElementById('ikY').value;
  const res = document.getElementById('ikResult');

  const d2 = x * x + y * y;
  const cosA2Raw = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);

  if (cosA2Raw < -1.01 || cosA2Raw > 1.01) {
    res.className = 'ik-result err';
    res.textContent = 'Out of reach. Max: ' + (L1 + L2).toFixed(0) + 'mm, Min: ' + Math.abs(L1 - L2).toFixed(0) + 'mm';
    drawArm(+document.getElementById('a1').value, +document.getElementById('a2').value);
    return;
  }

  const cosA2 = Math.max(-1, Math.min(1, cosA2Raw));
  const sinA2 = elbowUp ? Math.sqrt(1 - cosA2 * cosA2)
                        : -Math.sqrt(1 - cosA2 * cosA2);

  const a2r = Math.atan2(sinA2, cosA2);
  const a1r = Math.atan2(y, x) - Math.atan2(L2 * sinA2, L1 + L2 * cosA2);

  let a1d = a1r * 180 / Math.PI;
  let a2d = a2r * 180 / Math.PI;

  while (a1d > 180) a1d -= 360;
  while (a1d < -180) a1d += 360;

  if (Math.abs(Math.abs(a1d) - 180) < 1.0) {
    const candidates = [-180, 180];
    let bestErr = 999999;
    let bestA1 = a1d;

    for (const c of candidates) {
      if (c < A1_MIN || c > A1_MAX) continue;

      const r1c = c * Math.PI / 180;
      const r2c = a2d * Math.PI / 180;

      const vx = L1 * Math.cos(r1c) + L2 * Math.cos(r1c + r2c);
      const vy = L1 * Math.sin(r1c) + L2 * Math.sin(r1c + r2c);

      const err = Math.sqrt((vx - x) ** 2 + (vy - y) ** 2);

      if (err < bestErr) {
        bestErr = err;
        bestA1 = c;
      }
    }

    a1d = bestA1;
  }

  if (a1d < A1_MIN || a1d > A1_MAX || a2d < A2_MIN || a2d > A2_MAX) {
    res.className = 'ik-result err';
    res.textContent = 'Reachable by length, but outside joint limits.';
    return;
  }

  res.className = 'ik-result ok';
  res.textContent = 'Arm1: ' + a1d.toFixed(1) + '\u00b0   Arm2: ' + a2d.toFixed(1) + '\u00b0';

  drawArm(a1d, a2d);
}

// ── Send IK ──
async function sendIK() {
  if (busy) return;

  if (!homed) {
    setMsg('err', 'Home first!');
    return;
  }

  busy = true;
  document.getElementById('sendBtn').disabled = true;
  document.getElementById('sendBtn').textContent = 'Sending...';
  setMsg('busy', 'Sending IK...');

  try {
    const x = document.getElementById('ikX').value;
    const y = document.getElementById('ikY').value;
    const z = document.getElementById('ikZ').value;

    const r = await fetch('/ik?x=' + x + '&y=' + y + '&z=' + z + '&elbow=' + (elbowUp ? '1' : '0'));
    const d = await r.json();

    if (d.ok) {
      document.getElementById('a1').value = d.a1;
      document.getElementById('a2').value = d.a2;
      document.getElementById('z').value  = d.z;
      syncInputs();
      setMsg('busy', 'Moving...');
    } else {
      busy = false;
      setMsg('err', d.err || 'IK failed');
    }
  } catch (e) {
    busy = false;
    setMsg('err', 'Connection error');
  }

  document.getElementById('sendBtn').disabled = false;
  document.getElementById('sendBtn').textContent = 'Send IK';
}

// ── Send manual move ──
async function sendMove() {
  if (busy) return;

  if (!homed) {
    setMsg('err', 'Home first!');
    return;
  }

  busy = true;
  document.getElementById('sendBtn').disabled = true;
  document.getElementById('sendBtn').textContent = 'Sending...';
  setMsg('busy', 'Sending...');

  try {
    const r = await fetch('/move?a1=' + document.getElementById('a1').value
                        + '&a2=' + document.getElementById('a2').value
                        + '&z='  + document.getElementById('z').value);
    const d = await r.json();

    if (d.ok) {
      setMsg('busy', 'Moving...');
    } else {
      busy = false;
      setMsg('err', d.err || 'Move failed');
    }
  } catch (e) {
    busy = false;
    setMsg('err', 'Connection error');
  }

  document.getElementById('sendBtn').disabled = false;
  document.getElementById('sendBtn').textContent = mode === 'ik' ? 'Send IK' : 'Send to Robot';
}

// ── Home ──
async function sendHome() {
  if (busy) return;

  busy = true;
  document.getElementById('homeBtn').disabled = true;
  document.getElementById('homeBtn').textContent = '...';
  setMsg('busy', 'Homing...');

  try {
    const r = await fetch('/home');
    const d = await r.json();

    if (d.ok) {
      homed = true;
      document.getElementById('warn').classList.add('hidden');
      document.getElementById('a1').value = 0;
      document.getElementById('a2').value = 0;
      document.getElementById('z').value  = 0;
      syncInputs();
      setMsg('ok', 'Homed');
    } else {
      setMsg('err', d.err || 'Homing failed');
    }
  } catch (e) {
    setMsg('err', 'Connection error');
  }

  document.getElementById('homeBtn').disabled = false;
  document.getElementById('homeBtn').textContent = 'HOME';
  busy = false;
}

// ── Stop ──
async function sendStop() {
  try {
    await fetch('/stop');
    busy = false;
    robotMoving = false;
    setMsg('err', 'Stopped');
  } catch (e) {
    setMsg('err', 'Connection error');
  }
}

// ── UI status ──
function setMsg(t, s) {
  const m = document.getElementById('msg');
  m.className = 'status ' + t;
  m.textContent = s;
}

// ── Poll robot position ──
async function sync() {
  try {
    const d = await (await fetch('/position')).json();

    homed = d.homed;
    robotMoving = d.moving;

    document.getElementById('dot').style.cssText = 'background:#4a4;box-shadow:0 0 6px #4a4';

    if (homed) {
      document.getElementById('warn').classList.add('hidden');
    } else {
      document.getElementById('warn').classList.remove('hidden');
    }

    if (!robotMoving && !d.stopped) {
      busy = false;
    }

    if (d.stopped) {
      busy = false;
      setMsg('err', 'Stopped');
    } else if (robotMoving) {
      setMsg('busy', 'Moving...');
    } else {
      if (!busy) setMsg('ok', 'Connected');
    }

    if (!robotMoving && mode !== 'realtime') {
      document.getElementById('a1').value = d.a1;
      document.getElementById('a2').value = d.a2;
      document.getElementById('z').value  = d.z;
      syncInputs();
    } else {
      document.getElementById('fkX').textContent = d.x.toFixed(1);
      document.getElementById('fkY').textContent = d.y.toFixed(1);
      document.getElementById('fkZ').textContent = d.z.toFixed(1);
      drawArm(d.a1, d.a2);
    }

  } catch (e) {
    document.getElementById('dot').style.cssText = 'background:#a44;box-shadow:0 0 6px #a44';
    setMsg('err', 'Disconnected');
  }
}

window.addEventListener('load', () => {
  syncInputs();
  sync();
  setInterval(sync, 800);
});
</script>
</body>
</html>
)rawhtml";

#endif
