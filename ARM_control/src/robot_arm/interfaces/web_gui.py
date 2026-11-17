"""Browser GUI for the arm: sliders, a live URDF stick model, IK and a saved home.

Run on the Pi, open it through an SSH tunnel:

    ssh -L 8000:localhost:8000 pi@<pi-address>
    python -m robot_arm.interfaces.web_gui
    # then open http://localhost:8000 on the laptop

The server binds to 127.0.0.1, so it is reachable only through the tunnel.
IK comes from ARM_control/ik/arm_ik_v2.py (needs numpy and scipy). Without them
the rest of the GUI still works. The model uses that file's servo zero and signs.
The home pose is saved in configs/home.txt and loaded on the next start.
"""

from __future__ import annotations

import argparse
import dataclasses
import importlib.util
import json
import logging
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from robot_arm.config import RobotConfig
from robot_arm.control.arm_controller import STARTUP_WARNING, ArmController
from robot_arm.hardware.driver import NoOpDriver, PCA9685Driver
from robot_arm.utils.errors import RobotArmError
from robot_arm.vision.esp32_serial import SerialCamera
from robot_arm.vision.pi_camera import PiCamera
from robot_arm.vision.qr_view import QRView

logger = logging.getLogger(__name__)

IK_PATH = Path(__file__).resolve().parents[3] / "ik" / "arm_ik_v2.py"
HOME_FILE = RobotConfig.default_dir() / "home.txt"

# Used only when the IK module cannot be imported. Same values as arm_ik_v2.py
# (J0-J4) plus the gripper servo, so the model still draws.
FALLBACK_ZERO = [90.0, 75.0, 58.0, 66.5, 116.25]  # J1 re-mounted: +36 deg
FALLBACK_SIGN = [1.0, 1.0, -1.0, 1.0, -1.0]
# Gripper (joint 5), measured: 167.5 deg is fully closed, 10 deg is fully open,
# and the fully open jaw width is 3.5 cm. Home is the open position.
GRIPPER_CLOSED_DEG = 167.5
GRIPPER_OPEN_DEG = 10.0
GRIPPER_MAX_WIDTH_M = 0.035
# Allowed servo range per joint (deg).
LIMITS_DEG = [(0.0, 180.0)] * 5 + [(GRIPPER_OPEN_DEG, GRIPPER_CLOSED_DEG)]
FALLBACK_TCP = [0.010023, -0.125002, 0.0015]

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Arm</title><style>
*{box-sizing:border-box}
body{font-family:system-ui,sans-serif;background:#14161b;color:#e6e6e6;margin:0;padding:0 14px 24px}
small,.muted{color:#8d94a3}
#bar{position:sticky;top:0;z-index:10;background:#14161bf2;backdrop-filter:blur(6px);padding:10px 0 8px;border-bottom:1px solid #262b35;margin-bottom:12px}
#bar .line{display:flex;gap:8px 14px;align-items:center;flex-wrap:wrap}
#bar h1{font-size:18px;margin:0 8px 0 0;white-space:nowrap}
#msg{min-height:18px;color:#f0c36a;font-size:13px;margin-top:6px}
.grp{display:flex;gap:6px;align-items:center;flex-wrap:wrap;padding-left:14px;border-left:1px solid #2c323e}
.grp:first-of-type{border-left:0;padding-left:0}
.grp label{font-size:13px;color:#c9ced8;white-space:nowrap}
#grid{display:grid;grid-template-columns:minmax(340px,5fr) minmax(380px,6fr);gap:14px;align-items:start}
@media(max-width:900px){#grid{grid-template-columns:1fr}}
.col{display:flex;flex-direction:column;gap:14px;min-width:0}
.card{background:#1c2028;border:1px solid #262b35;border-radius:10px;padding:0}
.card>summary{list-style:none;cursor:pointer;padding:10px 14px;font-weight:600;font-size:14px;display:flex;align-items:center;gap:8px}
.card>summary::-webkit-details-marker{display:none}
.card>summary::before{content:"\\25BE";color:#8d94a3;font-size:12px}
.card:not([open])>summary::before{content:"\\25B8"}
.card>summary small{font-weight:400}
.body{padding:0 14px 14px}
.joint{padding:8px 0 10px;border-top:1px solid #262b35}
.joint:first-child{border-top:0;padding-top:0}
.jh{display:flex;justify-content:space-between;align-items:baseline;gap:8px;flex-wrap:wrap;font-size:13px}
.jh b{font-size:14px}
.jctl{display:flex;align-items:center;gap:6px}
.jctl input[type=range]{flex:1;min-width:0;height:28px}
button{padding:6px 12px;border:0;border-radius:6px;background:#3a6df0;color:#fff;font-size:13px;cursor:pointer}
button.mini{padding:4px 9px;background:#3a404c}
button:disabled{opacity:.4;cursor:default} button.warn{background:#d9822b} button.gray{background:#3a404c}
.val{font-variant-numeric:tabular-nums;font-weight:600}
input[type=number],select{background:#101318;color:#eee;border:1px solid #3a404c;border-radius:5px;padding:4px 6px;font-size:13px}
input[type=number]{width:74px}
canvas{width:100%;height:440px;background:#0e1014;border-radius:8px;touch-action:none;cursor:grab;display:block}
.legend{font-size:12px;margin-top:6px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin:0 4px 0 10px}
.legend i:first-child{margin-left:0}
.form{display:flex;gap:6px 10px;align-items:center;flex-wrap:wrap;margin-bottom:8px}
pre{background:#101318;border-radius:6px;padding:8px;font-size:12px;white-space:pre-wrap;margin:8px 0 0}
table{border-collapse:collapse} td{padding:3px 8px 3px 0}
</style></head><body>
<div id="bar">
 <div class="line">
  <h1>Robot Arm <small id="mode"></small></h1>
  <div class="grp"><button class="warn" id="start">Snap to home (startup)</button>
   <button id="home">Home</button><button class="gray" id="reset">Sliders = current</button></div>
  <div class="grp"><button class="warn" id="sethome">Set current pose as home</button></div>
  <div class="grp"><label><input type="checkbox" id="live" checked> live drag</label>
   <label>soften <input type="number" id="soften" min="0" value="" placeholder="yaml"> ms/deg</label>
   <button class="gray" id="setsoft">set</button></div>
  <div class="grp"><label><input type="checkbox" id="flooron" checked> Z floor guard</label>
   <label>floor <input type="number" id="floormm" step="1" value="5" style="width:60px"> mm</label>
   <span id="tcpz" class="muted">TCP z -</span></div>
 </div>
 <div id="msg"></div>
</div>
<div id="grid">
 <div class="col">
  <details class="card" open><summary>Joints <small>commanded angles, not measured (no position feedback)</small></summary>
   <div class="body"><div id="joints"></div><div class="muted" style="font-size:12px;margin-top:8px" id="homeinfo"></div></div></details>
 </div>
 <div class="col">
  <details class="card" open><summary>3D model</summary><div class="body">
   <canvas id="cv"></canvas>
   <div class="legend muted"><i style="background:#4da3ff"></i>commanded <i style="background:#ffc850"></i>slider target <i style="background:#50e678"></i>IK solution
    &nbsp;&middot; drag to rotate, wheel to zoom</div>
   <div class="legend muted">q = sign &times; (servo&deg; &minus; zero) from arm_ik_v2.py. Gripper: 167.5&deg; closed, 10&deg; open (3.5 cm).</div>
  </div></details>
  <details class="card" id="ik" open><summary>Inverse kinematics <small id="ikstat"></small></summary><div class="body">
   <div class="form"><select id="ikmode"><option value="abs">absolute TCP (x, y, z mm, base_link)</option><option value="rel">relative to current TCP (dx, dy, dz mm)</option><option value="cyl_rel">relative cylindrical (dr mm, d&theta; deg, dz mm)</option><option value="cyl_abs">absolute cylindrical (r mm, &theta; deg, z mm)</option></select>
    <span id="lx">x</span> <input type="number" step="1" id="ikx" value="0"> <span id="ly">y</span> <input type="number" step="1" id="iky" value="0"> <span id="lz">z</span> <input type="number" step="1" id="ikz" value="0">
    <label><input type="checkbox" id="ikhold"> hold tool direction</label></div>
   <div class="muted" style="font-size:12px;margin:2px 0 6px" id="tcpnow">TCP now: -</div>
   <button class="gray" id="ikfill">Fill from current TCP</button>
   <button id="iksolve">Solve (preview only)</button>
   <button class="warn" id="ikgo" disabled>Move arm to solution</button>
   <pre id="ikout">TCP = estimated closed-fingertip midpoint. Solving never moves the arm.</pre>
  </div></details>
 </div>
</div>
<details class="card" id="camrow" open style="margin-top:14px"><summary>Cameras <small>ESP32-CAM vs Pi camera, same QR detector</small></summary><div class="body">
 <div id="cams" style="display:flex;gap:12px;flex-wrap:wrap"></div>
 <table id="camcmp" style="width:100%;font-size:12px;margin-top:10px"></table>
</div></details>
<script>
const ZERO=__ZERO__,SIGN=__SIGN__;
const SIMULATION_ONLY=__SIMULATION_ONLY__;
let state=null,sliders=[],pending={},timer=null,model=null,ikPrev=null;
const $=id=>document.getElementById(id);
const msg=t=>{$('msg').textContent=t||''};
async function api(path,body){
  const r=await fetch(path,{method:body?'POST':'GET',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined});
  const j=await r.json(); if(j.error)msg(j.error); else if(body)msg(''); return j;}
function build(s){
  const box=$('joints'); box.innerHTML=''; sliders=[];
  s.joints.forEach(j=>{
    const d=document.createElement('div'); d.className='joint';
    const role=['base','shoulder','elbow','wrist pitch','wrist roll','gripper'][j.index];
    d.innerHTML=`<div class="jh"><span><b>J${j.index}</b> <span class="muted">${role} &middot; ch ${j.channel} &middot; home ${j.home_deg.toFixed(1)}&deg;</span></span>
    <span>target <span class="val" id="t${j.index}">${j.commanded_deg.toFixed(1)}</span>&deg; &nbsp;now <span class="val" id="c${j.index}">${j.commanded_deg.toFixed(1)}</span>&deg; &nbsp;${j.index===5?'<span class="val" id="q5">0</span> mm':'q <span class="val" id="q'+j.index+'">0</span>&deg;'}</span></div>
    <div class="jctl"><button class="mini" data-n="${j.index}" data-d="-1">&minus;1</button>
    <input type="range" min="${j.min_deg}" max="${j.max_deg}" step="0.5" value="${j.commanded_deg}" id="s${j.index}">
    <button class="mini" data-n="${j.index}" data-d="1">+1</button><button class="mini" data-h="${j.index}">home</button></div>`;
    box.appendChild(d);
    const sl=d.querySelector('input'); sliders[j.index]=sl;
    sl.addEventListener('input',()=>{$('t'+j.index).textContent=(+sl.value).toFixed(1); if($('live').checked)queue(j.index,+sl.value)});
    sl.addEventListener('change',()=>queue(j.index,+sl.value,true));
  });
  box.querySelectorAll('[data-h]').forEach(b=>b.onclick=()=>{const i=+b.dataset.h;const v=state.joints[i].home_deg;setSlider(i,v);queue(i,v,true)});
  box.querySelectorAll('[data-n]').forEach(b=>b.onclick=()=>{const i=+b.dataset.n;const v=Math.min(state.joints[i].max_deg,Math.max(state.joints[i].min_deg,+sliders[i].value+ +b.dataset.d));setSlider(i,v);queue(i,v,true)});
}
function setSlider(i,v){sliders[i].value=v;$('t'+i).textContent=(+v).toFixed(1)}
function queue(i,v,now){
  pending[i]=v; if(now){flush();return}
  if(!timer)timer=setTimeout(flush,80);
}
async function flush(){
  clearTimeout(timer);timer=null;
  if(!Object.keys(pending).length)return;
  const p=pending;pending={};
  const r=await api('/api/move',{angles:p});
  if(r.error&&/^blocked/.test(r.error)&&state){pending={};state.joints.forEach(j=>setSlider(j.index,j.commanded_deg))}
}
async function refresh(){
  try{
    const s=await api('/api/status'); const first=!state; state=s;
    $('start').style.display=s.ready?'none':'';
    if(first||s.home_version!==homeVersion){homeVersion=s.home_version;build(s)}
    s.joints.forEach(j=>{$('c'+j.index).textContent=j.commanded_deg.toFixed(1);$('q'+j.index).textContent=(j.index===5?openFrac(j.commanded_deg)*model_w()*1000:qOf(j.index,j.commanded_deg)).toFixed(1)});
    $('homeinfo').textContent='home (servo deg): ['+s.joints.map(j=>j.home_deg.toFixed(1)).join(', ')+']  '+s.home_source;
    if(document.activeElement!==$('soften')&&s.soften_ms!==null)$('soften').value=s.soften_ms;
    if(document.activeElement!==$('floormm'))$('floormm').value=s.floor_mm;
    $('flooron').checked=s.floor_enabled;
    const tz=$('tcpz');
    if(!s.guard_available){tz.textContent='z guard unavailable (no IK/numpy)';tz.style.color='#ff6b6b'}
    else if(s.tcp_z_mm!==null){tz.textContent='TCP z '+s.tcp_z_mm.toFixed(1)+' mm';tz.style.color=s.tcp_z_mm<s.floor_mm?'#ff6b6b':''}
  }catch(e){msg('connection lost')}
}
let homeVersion=-1;
$('start').onclick=async()=>{
  if(!confirm(`__WARNING__`))return;
  const r=await api('/api/startup',{confirmed:true});
  if(!r.error){state=null;refresh();}};
$('home').onclick=async()=>{const r=await api('/api/home',{});if(!r.error)state.joints.forEach(j=>setSlider(j.index,j.home_deg))};
$('flooron').onchange=()=>api('/api/floor',{enabled:$('flooron').checked});
$('floormm').onchange=()=>api('/api/floor',{floor_mm:+$('floormm').value});
$('reset').onclick=()=>state.joints.forEach(j=>setSlider(j.index,j.commanded_deg));
$('sethome').onclick=async()=>{
  if(!confirm('Save the current commanded pose as the new home? It is written to configs/home.txt and used on every start.'))return;
  const r=await api('/api/sethome',{});
  if(!r.error)msg('home saved to '+r.file);};
$('setsoft').onclick=()=>api('/api/soften',{ms:$('soften').value===''?null:+$('soften').value});
/* ---- 3D stick model (z up, metres) ---- */
let yaw=0.8,pitch=0.5,zoom=1;
const cv=$('cv'),cx=cv.getContext('2d');
const I=()=>[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]; // column-major
function mul(a,b){const r=new Array(16).fill(0);for(let c=0;c<4;c++)for(let rr=0;rr<4;rr++)for(let k=0;k<4;k++)r[c*4+rr]+=a[k*4+rr]*b[c*4+k];return r}
function T(x,y,z){const m=I();m[12]=x;m[13]=y;m[14]=z;return m}
function R(ax,q){const[c,s]=[Math.cos(q),Math.sin(q)],[x,y,z]=ax,t=1-c,m=I();
 m[0]=t*x*x+c;m[4]=t*x*y-s*z;m[8]=t*x*z+s*y;m[1]=t*x*y+s*z;m[5]=t*y*y+c;m[9]=t*y*z-s*x;m[2]=t*x*z-s*y;m[6]=t*y*z+s*x;m[10]=t*z*z+c;return m}
const pt=(m,p=[0,0,0])=>[m[0]*p[0]+m[4]*p[1]+m[8]*p[2]+m[12],m[1]*p[0]+m[5]*p[1]+m[9]*p[2]+m[13],m[2]*p[0]+m[6]*p[1]+m[10]*p[2]+m[14]];
const GC=__GC__,GO=__GO__,GW=__GW__;
const openFrac=d=>Math.max(0,Math.min(1,(GC-d)/(GC-GO)));
const model_w=()=>GW;
function qOf(i,deg){return SIGN[i]*(deg-ZERO[i])} // model angle in DEGREES
function fk(degs){
  const qs=degs.map((d,i)=>qOf(i,d));
  let m=I();const frames=[m];
  model.joints.forEach((j,i)=>{m=mul(m,mul(T(...j.xyz),R(j.axis,qs[i]*Math.PI/180)));frames.push(m)});
  const g=model.gripper,L=0.045,sep0=g.right.xyz[0]-g.left.xyz[0],
    thC=Math.asin(Math.min(1,sep0/(2*L))),thO=Math.asin((sep0-GW)/(2*L)), // tips meet when closed, GW apart when open
    qf=thC+openFrac(degs[5])*(thO-thC);
  const wrist=frames[5],fingers=[];
  [[g.left,qf],[g.right,-qf]].forEach(([f,q])=>{
    const fm=mul(wrist,mul(T(...f.xyz),R([0,0,1],q)));
    fingers.push([pt(fm),pt(fm,[0.0,-0.045,0])]);});
  return{frames,fingers,tcp:pt(wrist,model.tcp)};
}
function proj(p,sc,w,h,ctr){
  const[x,y,z]=[p[0]-ctr[0],p[1]-ctr[1],p[2]-ctr[2]];
  const cyw=Math.cos(yaw),syw=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
  const x1=x*cyw-y*syw,y1=x*syw+y*cyw;
  const y2=y1*cp-z*sp,z2=y1*sp+z*cp;
  return[w/2+x1*sc,h/2-z2*sc,y2];}
function draw(){
  if(!model||!state||!sliders.length)return;
  const dpr=window.devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
  if(cv.width!==w*dpr){cv.width=w*dpr;cv.height=h*dpr}
  cx.setTransform(dpr,0,0,dpr,0,0);cx.clearRect(0,0,w,h);
  const sc=Math.min(w,h)/0.55*zoom,ctr=[0,-0.1,0.12];
  const P=p=>proj(p,sc,w,h,ctr);
  const line=(a,b,col,wd)=>{const A=P(a),B=P(b);cx.strokeStyle=col;cx.lineWidth=wd;cx.beginPath();cx.moveTo(A[0],A[1]);cx.lineTo(B[0],B[1]);cx.stroke()};
  for(let i=-5;i<=5;i++){const v=i*0.05;line([v,-0.35,0],[v,0.15,0],'#1d222b',1);line([-0.25,v-0.1,0],[0.25,v-0.1,0],'#1d222b',1)}
  line([0,0,0],[0.06,0,0],'#e55',2);line([0,0,0],[0,0.06,0],'#5e5',2);line([0,0,0],[0,0,0.06],'#58f',2);
  const cur=state.joints.map(j=>j.commanded_deg),tgt=sliders.map(s=>+s.value);
  const poses=[[tgt,0],[ikPrev,1],[cur,2]];
  let curTcp=null;
  poses.forEach(([degs,kind])=>{
    if(!degs)return;
    const{frames,fingers,tcp}=fk(degs);
    const pts=frames.map(f=>pt(f));
    pts.push(pt(frames[5],[0.009849,-0.0687135,0]));
    const col=kind===0?'rgba(255,200,80,0.35)':kind===1?'rgba(80,230,120,0.8)':'#4da3ff';
    for(let i=0;i<pts.length-1;i++)line(pts[i],pts[i+1],col,kind===2?7:4);
    fingers.forEach(f=>line(f[0],f[1],kind===2?'#ff6b6b':'rgba(255,120,120,0.4)',kind===2?5:3));
    const A=P(tcp);cx.fillStyle=kind===1?'#6f6':'#ffd84d';cx.beginPath();cx.arc(A[0],A[1],kind===2?5:4,0,7);cx.fill();
    if(kind===2){
      curTcp=tcp;
      frames.forEach((f,i)=>{const B=P(pt(f));cx.fillStyle='#fff';cx.beginPath();cx.arc(B[0],B[1],4,0,7);cx.fill();
        if(i>0){cx.fillStyle='#9ab';cx.font='11px sans-serif';cx.fillText('J'+(i-1),B[0]+7,B[1]-6)}
        if(i>0&&i<6){const ax=model.joints[i-1].axis,o=pt(f);line(o,pt(f,ax.map(v=>v*0.03)),'#7f7',2)}});
    }
  });
  cx.fillStyle='#ccc';cx.font='12px monospace';
  cx.fillText(`TCP (base_link): x ${(curTcp[0]*1000).toFixed(1)} y ${(curTcp[1]*1000).toFixed(1)} z ${(curTcp[2]*1000).toFixed(1)} mm`,10,18);
}
let drag=null;
cv.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];cv.setPointerCapture(e.pointerId)});
cv.addEventListener('pointerup',()=>drag=null);
cv.addEventListener('pointermove',e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*0.01;pitch=Math.max(-1.5,Math.min(1.5,pitch+(e.clientY-drag[1])*0.01));drag=[e.clientX,e.clientY]});
cv.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(0.3,Math.min(5,zoom*(e.deltaY<0?1.1:0.9)))},{passive:false});
(function loop(){draw();requestAnimationFrame(loop)})();
/* ---- IK ---- */
const f3=a=>'['+a.map(v=>v.toFixed(3)).join(', ')+']',f1=a=>'['+a.map(v=>v.toFixed(1)).join(', ')+']';
const IKLAB={abs:['x (mm)','y (mm)','z (mm)'],rel:['dx (mm)','dy (mm)','dz (mm)'],cyl_rel:['dr (mm)','d\u03b8 (deg)','dz (mm)'],cyl_abs:['r (mm)','\u03b8 (deg)','z (mm)']};
function cylOf(t){const a=model.joints[0].xyz,dx=t[0]-a[0],dy=t[1]-a[1];return[Math.hypot(dx,dy)*1000,Math.atan2(dx,-dy)*180/Math.PI,t[2]*1000]} // r mm, theta deg, z mm (0 = base -Y, CCW+)
function setIkLabels(){const m=$('ikmode').value;['x','y','z'].forEach((k,i)=>$('l'+k).textContent=IKLAB[m][i])}
setInterval(()=>{if(!model||!state||!sliders.length)return;
  const t=fk(state.joints.map(j=>j.commanded_deg)).tcp,c=cylOf(t);
  $('tcpnow').textContent=`TCP now: x ${(t[0]*1000).toFixed(1)} y ${(t[1]*1000).toFixed(1)} z ${(t[2]*1000).toFixed(1)} mm  |  r ${c[0].toFixed(1)} mm  \u03b8 ${c[1].toFixed(1)}\u00b0  z ${c[2].toFixed(1)} mm`},400);
$('ikfill').onclick=()=>{
  const m=$('ikmode').value;
  if(m==='rel'||m==='cyl_rel'){['x','y','z'].forEach(k=>$('ik'+k).value=0);return}
  if(m==='cyl_abs'){const c=cylOf(fk(state.joints.map(j=>j.commanded_deg)).tcp);['x','y','z'].forEach((k,i)=>$('ik'+k).value=+c[i].toFixed(1));return}
  const t=fk(state.joints.map(j=>j.commanded_deg)).tcp;['x','y','z'].forEach((k,i)=>$('ik'+k).value=+(t[i]*1000).toFixed(1));};
$('ikmode').onchange=()=>{setIkLabels();$('ikfill').click()};setIkLabels();
$('iksolve').onclick=async()=>{
  $('ikgo').disabled=true;ikPrev=null;$('ikout').textContent='solving...';
  const r=await api('/api/ik/solve',{mode:$('ikmode').value,x:+$('ikx').value,y:+$('iky').value,z:+$('ikz').value,hold:$('ikhold').checked});
  if(r.error){$('ikout').textContent='IK: '+r.error;return}
  ikPrev=r.servo;$('ikgo').disabled=false;
  $('ikout').textContent=`target TCP ${f1(r.target_mm)} mm  (r ${r.target_cyl[0].toFixed(1)} mm, \u03b8 ${r.target_cyl[1].toFixed(1)}\u00b0, z ${r.target_cyl[2].toFixed(1)} mm)\\nservo J0-J4 ${f3(r.servo.slice(0,5))} deg\\nchange      ${f3(r.change)} deg\\npulses us   [${r.pulses.join(', ')}]\\nposition error ${r.err_mm.toFixed(3)} mm`+(r.axis_err_deg===null?'':`\\ndirection error ${r.axis_err_deg.toFixed(2)} deg`)+'\\nPreview only: green model. Press "Move arm to solution" to execute.';
};
$('ikgo').onclick=async()=>{
  const question=SIMULATION_ONLY?'Move the simulated arm to the IK solution?':'Move the real arm to the IK solution? Check for collisions first.';
  if(!ikPrev||!confirm(question))return;
  const a={};for(let i=0;i<5;i++)a[i]=ikPrev[i];
  const r=await api('/api/move',{angles:a});
  if(!r.error){for(let i=0;i<5;i++)setSlider(i,ikPrev[i]);ikPrev=null;$('ikgo').disabled=true}};
api('/api/model').then(m=>{model=m;$('ikstat').textContent=m.ik_ok?'(arm_ik_v2.py loaded)':'(IK unavailable: '+m.ik_error+')';$('iksolve').disabled=!m.ik_ok});
$('mode').textContent=SIMULATION_ONLY?'— SIMULATION ONLY (motors disabled)':'';
if(SIMULATION_ONLY){$('start').style.display='none';$('start').disabled=true}
refresh();setInterval(refresh,300);
const CAMS=[['esp','ESP32-CAM (USB)'],['pi','Pi camera (CSI)']];
const OPTS=[['detect','Detect QR'],['focus','Focus meter'],['denoise','Denoise'],['sharpen','Sharpen'],['clahe','Contrast (CLAHE)'],['show_filtered','Show filtered image'],['wechat','WeChat detector'],['robust','Robust decode']];
function optsHtml(k){
  const boxes=OPTS.map(([o,t])=>`<label style="display:inline-block;margin:2px 10px 2px 0;font-size:12px"><input type="checkbox" data-cam="${k}" data-opt="${o}"> ${t}</label>`).join('');
  const pi=k==='pi'?`<div class="form" style="margin-top:6px;font-size:12px">capture:
    <select id="pi_size"></select> sharp <input type="number" id="pi_sharp" step="0.5" min="0" max="16" style="width:60px">
    contrast <input type="number" id="pi_contrast" step="0.1" min="0" max="16" style="width:60px">
    denoise <select id="pi_denoise"></select> zoom <input type="number" id="pi_zoom" step="0.5" min="1" max="4" style="width:60px">
    <button class="mini" id="pi_apply">Apply (restarts camera)</button></div>`:'';
  const rot=`<div style="margin:4px 0;font-size:12px">rotate (clockwise): <select data-rot="${k}"><option>0</option><option>90</option><option>180</option><option>270</option></select>
    <button class="mini" data-rotstep="${k}">+90&deg;</button></div>`;
  return `<details class="card" style="margin-top:8px"><summary>Image options <small>every feature is a toggle</small></summary><div class="body">${rot}${boxes}<div id="params_${k}" style="margin-top:8px"></div>${pi}</div></details>`;
}
function buildParams(k,c){
  const box=$('params_'+k);if(!box||!c.options||!c.options.param_spec)return;
  let html='<div class="muted" style="font-size:12px;margin-bottom:4px">Tune filter values <button class="mini" data-preset="'+k+'">reset to defaults</button></div>',grp='';
  c.options.param_spec.forEach(s=>{
    if(s.group!==grp){grp=s.group;html+=`<div style="font-size:12px;margin-top:6px;color:#9ab"><b>${grp}</b></div>`}
    html+=`<div style="display:flex;align-items:center;gap:8px;font-size:12px"><span style="width:170px">${s.label}</span>
      <input type="range" style="flex:1;height:20px" data-param="${s.key}" data-pcam="${k}" min="${s.min}" max="${s.max}" step="${s.step}" value="${c.options.params[s.key]}">
      <span class="val" style="width:44px;text-align:right" id="pv_${k}_${s.key}">${c.options.params[s.key]}</span></div>`;
  });
  box.innerHTML=html;
  const timers={};
  box.querySelectorAll('[data-param]').forEach(e=>e.oninput=()=>{
    $('pv_'+k+'_'+e.dataset.param).textContent=e.value;
    clearTimeout(timers[e.dataset.param]);
    timers[e.dataset.param]=setTimeout(()=>api('/api/camera/param',{cam:k,key:e.dataset.param,value:+e.value}),120)});
  box.querySelector('[data-preset]').onclick=()=>api('/api/camera/param',{cam:k,key:'_reset',value:0});
}
function wireOpts(k,c){
  buildParams(k,c);
  const rs=document.querySelector(`[data-rot="${k}"]`);
  rs.onchange=()=>api('/api/camera/rotate',{cam:k,deg:+rs.value});
  document.querySelector(`[data-rotstep="${k}"]`).onclick=()=>api('/api/camera/rotate',{cam:k,deg:(+rs.value+90)%360});
  document.querySelectorAll(`[data-cam="${k}"]`).forEach(e=>e.onchange=()=>api('/api/camera/options',{cam:k,key:e.dataset.opt,value:e.checked}));
  if(k==='pi'&&c.pi){
    $('pi_size').innerHTML=c.pi.sizes.map(v=>`<option>${v}</option>`).join('');
    $('pi_denoise').innerHTML=c.pi.denoise_modes.map(v=>`<option>${v}</option>`).join('');
    $('pi_size').value=c.pi.size;$('pi_sharp').value=c.pi.sharpness;$('pi_contrast').value=c.pi.contrast;
    $('pi_denoise').value=c.pi.denoise;$('pi_zoom').value=c.pi.zoom;
    $('pi_apply').onclick=()=>api('/api/camera/pi',{size:$('pi_size').value,sharpness:+$('pi_sharp').value,contrast:+$('pi_contrast').value,denoise:$('pi_denoise').value,zoom:+$('pi_zoom').value});
  }
}
function syncOpts(k,c){
  if(!c.options)return;
  const rs=document.querySelector(`[data-rot="${k}"]`);if(rs&&document.activeElement!==rs)rs.value=String(c.options.rotate);
  document.querySelectorAll(`[data-pcam="${k}"]`).forEach(e=>{
    const v=c.options.params&&c.options.params[e.dataset.param];
    if(v!==undefined&&document.activeElement!==e&&+e.value!==v){e.value=v;$('pv_'+k+'_'+e.dataset.param).textContent=v}});
  document.querySelectorAll(`[data-cam="${k}"]`).forEach(e=>{
    if(document.activeElement!==e)e.checked=!!c.options[e.dataset.opt];
    if(e.dataset.opt==='wechat'&&!c.options.wechat_available){e.disabled=true;e.title='needs opencv-contrib'}
  });
}
async function pullCam(k){ // ask for a frame newer than the last one shown; never queues
  let seq=-1,url=null;
  for(;;){
    try{
      const r=await fetch('/camera.jpg?src='+k+'&seq='+seq+'&t='+Date.now(),{cache:'no-store'});
      if(r.status===200){
        seq=+r.headers.get('X-Seq');
        const u=URL.createObjectURL(await r.blob());
        const img=$('img_'+k);img.src=u;
        if(url)URL.revokeObjectURL(url);url=u;
      }else if(r.status===404){await new Promise(z=>setTimeout(z,2000))}
      else{await new Promise(z=>setTimeout(z,100))}
    }catch(e){await new Promise(z=>setTimeout(z,1000))}
  }
}
function camTick(){
  const cs=state&&state.cameras,box=$('cams');
  if(!cs||!Object.keys(cs).length){box.textContent='cameras disabled';return}
  CAMS.forEach(([k,title])=>{
    const c=cs[k];if(!c)return;
    let d=$('cam_'+k);
    if(!d){d=document.createElement('div');d.id='cam_'+k;d.style.flex='1 1 220px';
      d.innerHTML=`<b>${title}</b> <small id="cs_${k}"></small>${k==='esp'?' <button class="mini" id="light">Light: off</button> <input type="range" id="lbr" min="0" max="100" step="1" value="20" style="width:110px;vertical-align:middle" title="light brightness"> <span id="lbrv" class="muted">20%</span>':''}<br><img id="img_${k}" style="width:100%;max-height:420px;object-fit:contain;background:#000;border-radius:6px"><div id="ql_${k}" style="font-size:13px;margin-top:4px"></div>${optsHtml(k)}`;
      box.appendChild(d);pullCam(k);wireOpts(k,c);
      if(k==='esp'){$('light').onclick=()=>api('/api/camera/light',{on:!(state.cameras.esp&&state.cameras.esp.light)});
        let lt=null,ld=false;const lb=$('lbr');
        lb.oninput=()=>{ld=true;$('lbrv').textContent=lb.value+'%';if(!lt)lt=setTimeout(()=>{lt=null;api('/api/camera/light',{brightness:+lb.value})},80)};
        lb.onchange=async()=>{clearTimeout(lt);lt=null;await api('/api/camera/light',{brightness:+lb.value});ld=false};
        lb.onpointerdown=()=>{ld=true};lb.onpointerup=()=>{ld=false};window.lbDrag=()=>ld}}
    syncOpts(k,c);
    if(k==='esp'&&$('light')){const on=!!c.light;$('light').textContent='Light: '+(on?'ON':'off');$('light').style.background=on?'#d9a62b':'';
      if(c.brightness!=null&&!(window.lbDrag&&window.lbDrag())){$('lbr').value=c.brightness;$('lbrv').textContent=Math.round(c.brightness)+'%'}}
    $('cs_'+k).textContent=c.connected?`(${c.port||'auto'}, ${c.fps.toFixed(1)} fps)`+(c.bytes===0?' - nothing received':(c.fps===0?' - no valid frames':'')):'(not connected: '+(c.error||'waiting')+')';
  });
  let h='<tr style="color:#999"><td>camera</td><td>size</td><td>fps</td><td>jpeg KB</td><td>proc ms</td><td>detect ms</td><td>focus (peak)</td><td>QR found %</td><td>QR decoded %</td></tr>';
  CAMS.forEach(([k,title])=>{const c=cs[k];if(!c||!c.stats)return;const t=c.stats;
    h+=`<tr><td>${title}</td><td>${t.width}x${t.height}</td><td>${c.fps.toFixed(1)}</td><td>${t.jpeg_kb}</td><td>${t.proc_ms}</td><td>${t.detect_ms}</td><td>${c.options&&c.options.focus?t.focus+' ('+t.focus_peak+')':'-'}</td><td>${t.seen_pct}</td><td>${t.decoded_pct}</td></tr>`});
  $('camcmp').innerHTML=h;
}
function qrTick(){
  const cs=state&&state.cameras;if(!cs)return;
  CAMS.forEach(([k])=>{
    const q=cs[k]&&cs[k].qr,box=$('ql_'+k);if(!box)return;
    box.innerHTML='';
    if(!q){return}
    if(!q.length){box.innerHTML='<small>no QR code in view</small>';return}
    q.forEach(c=>{
      const d=document.createElement('div');d.style.color=c.ok?'#6e6':'#f77';
      if(c.ok&&/^https?:\\/\\//i.test(c.text)){const a=document.createElement('a');a.href=c.text;a.target='_blank';a.rel='noopener';a.textContent=c.text;a.style.color='#8cf';d.appendChild(a)}
      else d.appendChild(document.createTextNode(c.text));
      box.appendChild(d)});
  });
}
setInterval(camTick,1000);setInterval(qrTick,500);
</script></body></html>
"""


def to_cylindrical(xyz_m, axis_xy_m) -> tuple[float, float, float]:
    """(x, y, z) in base_link metres -> (r mm, theta deg, z mm) about the joint_1 axis.

    theta = 0 along base -Y (the way the arm reaches); positive is counter-clockwise
    seen from +z, the same sense as joint 0 in arm_ik_v2.py.
    """
    dx, dy = xyz_m[0] - axis_xy_m[0], xyz_m[1] - axis_xy_m[1]
    return (math.hypot(dx, dy) * 1000.0, math.degrees(math.atan2(dx, -dy)), xyz_m[2] * 1000.0)


def from_cylindrical(r_mm: float, theta_deg: float, z_mm: float, axis_xy_m) -> list[float]:
    """Inverse of to_cylindrical. Returns (x, y, z) in base_link metres."""
    r, theta = r_mm / 1000.0, math.radians(theta_deg)
    return [
        axis_xy_m[0] + r * math.sin(theta),
        axis_xy_m[1] - r * math.cos(theta),
        z_mm / 1000.0,
    ]


class IKBridge:
    """Loads ik/arm_ik_v2.py lazily. Solving never touches the hardware."""

    def __init__(self, path: Path = IK_PATH) -> None:
        self.mod = None
        self.error: str | None = None
        try:
            spec = importlib.util.spec_from_file_location("arm_ik_v2", path)
            if spec is None or spec.loader is None:
                raise ImportError(f"cannot load {path}")
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            self.mod = mod
        except Exception as exc:  # numpy/scipy missing, file missing, ...
            self.error = f"{type(exc).__name__}: {exc}"
            logger.warning("IK unavailable: %s", self.error)

    @property
    def ok(self) -> bool:
        return self.mod is not None

    def zero(self) -> list[float]:
        if not self.ok:
            return FALLBACK_ZERO + [GRIPPER_CLOSED_DEG]
        return [float(v) for v in self.mod.URDF_ZERO_SERVO] + [GRIPPER_CLOSED_DEG]

    def sign(self) -> list[float]:
        if not self.ok:
            return FALLBACK_SIGN + [-1.0]
        return [float(v) for v in self.mod.SERVO_SIGN] + [-1.0]

    def tcp(self) -> list[float]:
        return [float(v) for v in self.mod.TCP_ESTIMATE] if self.ok else FALLBACK_TCP

    def tcp_position(self, servo_deg: list[float]) -> list[float] | None:
        if not self.ok:
            return None
        import numpy as np

        mod = self.mod
        T, _ = mod.fk(mod.to_q(np.array(servo_deg[:5])), np.array(mod.TCP_ESTIMATE))
        return [float(v) for v in T[:3, 3]]

    def path_min_z(self, start: list[float], target: list[float], steps: int = 20) -> tuple[float, float, list[float]] | None:
        """Lowest TCP z (mm) along the straight joint-space path start -> target.

        Returns (start_z_mm, min_z_mm, servo pose of the minimum), or None when FK is unavailable.
        The trajectory planner uses one shared tau, so the path is exactly this line.
        """
        if not self.ok:
            return None
        z0 = None
        best: tuple[float, list[float]] | None = None
        for i in range(steps + 1):
            s = i / steps
            pose = [a + s * (b - a) for a, b in zip(start[:5], target[:5], strict=True)]
            z = self.tcp_position(pose)[2] * 1000.0
            if z0 is None:
                z0 = z
            if best is None or z < best[0]:
                best = (z, pose)
        return z0, best[0], best[1]

    def solve(self, current: list[float], body: dict) -> dict:
        if not self.ok:
            raise ValueError(f"IK unavailable ({self.error}). pip install numpy scipy")
        import numpy as np

        mod = self.mod
        start = np.array(current[:5], dtype=float)
        if np.any(start < mod.SOFT_LIMIT_DEG[:, 0]) or np.any(start > mod.SOFT_LIMIT_DEG[:, 1]):
            raise ValueError(
                "current pose is outside the IK soft limits "
                f"{mod.SOFT_LIMIT_DEG.tolist()}; move the sliders inside them first"
            )
        tcp = np.array(mod.TCP_ESTIMATE)
        a, b, c = float(body["x"]), float(body["y"]), float(body["z"])  # mm / mm / mm or deg
        mode = body.get("mode", "abs")
        axis = [float(v) for v in mod.JOINT_ORIGINS[0][:2]]  # joint_1 axis (parallel to z)
        T0, _ = mod.fk(mod.to_q(start), tcp)
        cur = T0[:3, 3]
        cur_cyl = to_cylindrical(cur, axis)
        if mode == "abs":
            target = np.array([a, b, c]) / 1000.0
        elif mode == "rel":
            target = cur + np.array([a, b, c]) / 1000.0
        elif mode in ("cyl_rel", "cyl_abs"):
            if mode == "cyl_rel":  # dr, dtheta, dz from the current gripper TCP
                r, theta, z = cur_cyl[0] + a, cur_cyl[1] + b, cur_cyl[2] + c
            else:
                r, theta, z = a, b, c
            if r < 0:
                raise ValueError(f"radius would be {r:.1f} mm; r must be >= 0")
            target = np.array(from_cylindrical(r, theta, z, axis))
        else:
            raise ValueError(f"unknown IK mode: {mode}")
        hold = bool(body.get("hold"))
        q, _T, err, axis_err = mod.solve(target, tcp, start, hold)
        servo = mod.to_servo(q)
        pulses = [mod.pwm(i, float(v)) for i, v in enumerate(servo)]
        return {
            "target_mm": [float(v) * 1000.0 for v in target],
            "cur_cyl": list(cur_cyl),
            "target_cyl": list(to_cylindrical(target, axis)),
            "servo": [float(v) for v in servo] + [float(current[5])],
            "change": [float(v) for v in servo - start],
            "pulses": pulses,
            "err_mm": float(err) * 1000.0,
            "axis_err_deg": float(axis_err) if hold else None,
        }


def load_home() -> list[float] | None:
    """Read servo_deg from configs/home.txt, or None when there is no saved home."""
    if not HOME_FILE.is_file():
        return None
    for line in HOME_FILE.read_text(encoding="utf-8").splitlines():
        key, sep, value = line.partition("=")
        if sep and key.strip() == "servo_deg":
            degrees = [float(v) for v in value.split()]
            if len(degrees) == 6 and all(0.0 <= d <= 180.0 for d in degrees):
                return degrees
            raise RobotArmError(f"{HOME_FILE}: servo_deg needs 6 values in 0..180")
    return None


def apply_home(config: RobotConfig, degrees: list[float]) -> RobotConfig:
    """Return a config whose home pulses match the saved servo angles."""
    joints = []
    for joint, deg in zip(config.joints, degrees, strict=True):
        us = joint.min_us + (joint.max_us - joint.min_us) * deg / 180.0
        joints.append(dataclasses.replace(joint, home_us=int(round(us))))
    return dataclasses.replace(config, joints=tuple(joints))


class ArmService:
    """Thread-safe wrapper: the latest slider targets win, moves run one at a time."""

    def __init__(
        self,
        arm: ArmController,
        ik: IKBridge,
        *,
        saved_home: bool,
        home_degrees: list[float] | None = None,
        simulation_only: bool = False,
        cameras: dict | None = None,
    ) -> None:
        self.arm = arm
        # name -> (raw camera, QRView or None); the stream shows the view when present
        self.cameras = cameras or {}
        self.ik = ik
        self.simulation_only = simulation_only
        self.home_deg = list(home_degrees) if home_degrees is not None else [j.home_deg for j in arm.config.joints]
        self.home_source = "(saved in configs/home.txt)" if saved_home else "(from joints.yaml)"
        self.home_version = 0
        self._lock = threading.Lock()
        self._pending: dict[int, float] = {}
        self._wake = threading.Event()
        self._error: str | None = None
        self._busy = False
        self.floor_enabled = True
        self.floor_mm = 5.0
        threading.Thread(target=self._worker, daemon=True).start()

    def source(self, name: str):
        entry = self.cameras.get(name)
        return None if entry is None else (entry[1] or entry[0])

    def commanded(self) -> list[float]:
        return [j.commanded_deg for j in self.arm._joints]

    def status(self) -> dict:
        joints = []
        for joint, home in zip(self.arm._joints, self.home_deg, strict=True):
            cal = joint.servo.calibration
            joints.append(
                {
                    "index": cal.index,
                    "channel": cal.channel,
                    "min_us": cal.min_us,
                    "max_us": cal.max_us,
                    "home_deg": home,
                    "min_deg": LIMITS_DEG[cal.index][0],
                    "max_deg": LIMITS_DEG[cal.index][1],
                    "commanded_deg": joint.commanded_deg,
                }
            )
        error, self._error = self._error, None
        return {
            "ready": self.arm.ready,
            "busy": self._busy,
            "soften_ms": self.arm.soften_ms,
            "floor_enabled": self.floor_enabled,
            "floor_mm": self.floor_mm,
            "guard_available": self.ik.ok,
            "tcp_z_mm": self.tcp_z_mm(),
            "joints": joints,
            "home_source": self.home_source,
            "home_version": self.home_version,
            "simulation_only": self.simulation_only,
            "cameras": {
                name: {
                    "connected": cam.connected,
                    "fps": cam.fps,
                    "port": cam.port,
                    "error": cam.last_error,
                    "bytes": cam.bytes_rx,
                    "bad": cam.bad_frames,
                    "light": getattr(cam, "light", None),
                    "brightness": getattr(cam, "brightness", None),
                    "options": view.options() if view is not None else None,
                    "pi": cam.settings() if hasattr(cam, "settings") else None,
                    "qr": view.results() if view is not None else None,
                    "stats": view.stats() if view is not None else None,
                }
                for name, (cam, view) in self.cameras.items()
            },
            "error": error,
        }

    def startup(self) -> None:
        with self._lock:
            self.arm.startup(confirmed=True)

    def move(self, angles: dict[int, float]) -> None:
        if not self.arm.ready:
            raise RobotArmError("press 'Snap to home (startup)' first")
        for index, deg in angles.items():
            low, high = LIMITS_DEG[index]
            if not low <= deg <= high:
                raise ValueError(f"joint {index} angle {deg:.1f} outside {low:g}..{high:g}")
        with self._lock:
            target = self.commanded()
            for index, deg in {**self._pending, **angles}.items():
                target[index] = deg
            self._check_floor(target)
            self._pending.update(angles)
        self._wake.set()

    def _check_floor(self, target: list[float]) -> None:
        """Raise when the straight joint path to target dips the TCP below the floor."""
        if not self.floor_enabled:
            return
        result = self.ik.path_min_z(self.commanded(), target)
        if result is None:
            return
        start_z, min_z, _pose = result
        # a move that starts below the floor may still climb out; it just may not go lower
        if min_z < self.floor_mm and min_z < start_z - 1e-6:
            raise ValueError(
                f"blocked: TCP would reach z={min_z:.1f} mm (floor {self.floor_mm:g} mm)"
            )

    def tcp_z_mm(self) -> float | None:
        pos = self.ik.tcp_position(self.commanded())
        return None if pos is None else pos[2] * 1000.0

    def home(self) -> None:
        if not self.arm.ready:
            raise RobotArmError("press 'Snap to home (startup)' first")
        with self._lock:
            self._check_floor(list(self.home_deg))
            self._pending.clear()
            self.arm.move_all(list(self.home_deg))

    def set_home(self) -> str:
        """Make the current commanded pose the home and save it to configs/home.txt."""
        if not self.arm.ready:
            raise RobotArmError("press 'Snap to home (startup)' first")
        with self._lock:
            degrees = self.commanded()
            pulses = [
                int(round(j.servo.pulse_us(d)))
                for j, d in zip(self.arm._joints, degrees, strict=True)
            ]
            lines = [
                "# Home pose saved by the web GUI. Read at the next start.",
                "# servo_deg is the commanded angle per joint, 0 = min pulse, 180 = max pulse.",
                f"saved = {time.strftime('%Y-%m-%d %H:%M:%S')}",
                "servo_deg = " + " ".join(f"{d:.3f}" for d in degrees),
                "pulse_us = " + " ".join(str(p) for p in pulses),
            ]
            tcp = self.ik.tcp_position(degrees)
            if tcp is not None:
                lines.append("tcp_base_link_mm = " + " ".join(f"{v * 1000:.3f}" for v in tcp))
            HOME_FILE.write_text("\n".join(lines) + "\n", encoding="utf-8")
            self.home_deg = degrees
            self.home_source = "(saved in configs/home.txt)"
            self.home_version += 1
        return str(HOME_FILE)

    def _worker(self) -> None:
        while True:
            self._wake.wait()
            self._wake.clear()
            with self._lock:
                targets, self._pending = self._pending, {}
                if not targets:
                    continue
                self._busy = True
                try:
                    current = self.commanded()
                    for index, deg in targets.items():
                        current[index] = deg
                    self.arm.move_all(current)
                except (RobotArmError, ValueError) as exc:
                    self._error = str(exc)
                finally:
                    self._busy = False
            if self._pending:
                self._wake.set()


def load_model(ik: IKBridge) -> dict:
    """Joint origins and axes from configs/kinematics.yaml, plus the IK servo mapping."""
    import yaml

    path = RobotConfig.default_dir() / "kinematics.yaml"
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    grip = doc["gripper"]
    return {
        "joints": [
            {"name": r["name"], "xyz": r["xyz_m"], "axis": r["axis"]}
            for r in doc["positioning_joints"]
        ],
        "gripper": {
            "left": {"xyz": grip["left_finger_joint"]["xyz_m"]},
            "right": {"xyz": grip["right_finger_joint"]["xyz_m"]},
        },
        "tcp": ik.tcp(),
        "servo_mapping": {
            "zero_deg": ik.zero()[:5],
            "sign": ik.sign()[:5],
        },
        "grip": {
            "closed_deg": GRIPPER_CLOSED_DEG,
            "open_deg": GRIPPER_OPEN_DEG,
            "max_width_m": GRIPPER_MAX_WIDTH_M,
        },
        "ik_ok": ik.ok,
        "ik_error": ik.error,
    }


def make_handler(service: ArmService) -> type[BaseHTTPRequestHandler]:
    page = (
        PAGE.replace("__ZERO__", json.dumps(service.ik.zero()))
        .replace("__SIGN__", json.dumps(service.ik.sign()))
        .replace("__GC__", json.dumps(GRIPPER_CLOSED_DEG))
        .replace("__GO__", json.dumps(GRIPPER_OPEN_DEG))
        .replace("__GW__", json.dumps(GRIPPER_MAX_WIDTH_M))
        .replace("__SIMULATION_ONLY__", json.dumps(service.simulation_only))
        .replace("__WARNING__", STARTUP_WARNING.replace("`", "'"))
        .encode()
    )

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args) -> None:  # quiet
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _json(self, payload: dict, code: int = 200) -> None:
            self._send(code, json.dumps(payload).encode(), "application/json")

        def _query(self, key: str, default: str = "") -> str:
            if "?" in self.path:
                for part in self.path.split("?", 1)[1].split("&"):
                    if part.startswith(key + "="):
                        return part[len(key) + 1 :]
            return default

        def _src(self):
            return service.source(self._query("src", "esp"))

        def _stream_camera(self) -> None:
            cam = self._src()
            if cam is None:
                return self._json({"error": "camera disabled"}, 404)
            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = -1
            try:
                while True:
                    jpeg, seq = cam.wait_frame(seq, timeout=5.0)
                    if jpeg is None:
                        continue
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(jpeg)).encode()
                        + b"\r\n\r\n"
                        + jpeg
                        + b"\r\n"
                    )
            except (BrokenPipeError, ConnectionResetError):
                return

        def do_GET(self) -> None:
            if self.path == "/":
                self._send(200, page, "text/html; charset=utf-8")
            elif self.path == "/api/model":
                self._json(load_model(service.ik))
            elif self.path == "/api/status":
                self._json(service.status())
            elif self.path.startswith("/camera.jpg"):
                # Pull model: the browser asks for a frame newer than the one it has.
                # Nothing queues up, so a slow link shows old-by-one-frame, never seconds old.
                src = self._src()
                if src is None:
                    return self._json({"error": "camera disabled"}, 404)
                try:
                    last = int(self._query("seq", "-1"))
                except ValueError:
                    last = -1
                jpeg, seq = src.wait_frame(last, timeout=1.0)
                if jpeg is None or seq == last:
                    self._json({"error": "no new frame"}, 503)
                else:
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-Seq", str(seq))
                    self.end_headers()
                    self.wfile.write(jpeg)
            elif self.path.startswith("/camera.mjpg"):
                self._stream_camera()
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self) -> None:
            length = int(self.headers.get("Content-Length") or 0)
            result: dict = {"ok": True}
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
                if self.path == "/api/startup":
                    if not body.get("confirmed"):
                        raise RobotArmError("startup not confirmed")
                    service.startup()
                elif self.path == "/api/move":
                    angles = {int(k): float(v) for k, v in body["angles"].items()}
                    for index, deg in angles.items():
                        if not 0 <= index < 6:
                            raise ValueError(f"bad joint: {index}")
                    service.move(angles)
                elif self.path == "/api/home":
                    service.home()
                elif self.path == "/api/sethome":
                    result["file"] = service.set_home()
                elif self.path == "/api/ik/solve":
                    result = service.ik.solve(service.commanded(), body)
                elif self.path == "/api/camera/options":
                    entry = service.cameras.get(str(body.get("cam")))
                    if entry is None or entry[1] is None:
                        raise RobotArmError("camera has no image processing")
                    entry[1].set_option(str(body["key"]), bool(body["value"]))
                elif self.path == "/api/camera/param":
                    entry = service.cameras.get(str(body.get("cam")))
                    if entry is None or entry[1] is None:
                        raise RobotArmError("camera has no image processing")
                    entry[1].set_param(str(body["key"]), body.get("value", 0))
                elif self.path == "/api/camera/rotate":
                    entry = service.cameras.get(str(body.get("cam")))
                    if entry is None or entry[1] is None:
                        raise RobotArmError("camera has no image processing")
                    entry[1].set_rotate(int(body["deg"]))
                elif self.path == "/api/camera/pi":
                    entry = service.cameras.get("pi")
                    if entry is None or not hasattr(entry[0], "apply"):
                        raise RobotArmError("Pi camera not available")
                    entry[0].apply(**body)
                elif self.path == "/api/camera/light":
                    entry = service.cameras.get("esp")
                    if entry is None or not hasattr(entry[0], "set_light"):
                        raise RobotArmError("ESP32 camera not available")
                    try:
                        if "brightness" in body:
                            entry[0].set_brightness(float(body["brightness"]))
                        if "on" in body:
                            entry[0].set_light(bool(body["on"]))
                    except OSError as exc:
                        raise RobotArmError(str(exc)) from exc
                elif self.path == "/api/floor":
                    if "floor_mm" in body:
                        service.floor_mm = float(body["floor_mm"])
                    if "enabled" in body:
                        service.floor_enabled = bool(body["enabled"])
                elif self.path == "/api/soften":
                    ms = body.get("ms")
                    if ms is not None and float(ms) < 0:
                        raise ValueError("soften must be >= 0")
                    service.arm.soften_ms = None if ms is None else float(ms)
                else:
                    return self._json({"error": "not found"}, 404)
            except (RobotArmError, ValueError, KeyError, TypeError) as exc:
                return self._json({"error": str(exc)})
            self._json(result)

    return Handler


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    parser = argparse.ArgumentParser(description="Slider web GUI for the arm.")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--host", default="127.0.0.1", help="use 0.0.0.0 to skip the SSH tunnel (insecure)")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--camera-port", default="auto", help="ESP32-CAM serial port, or auto")
    parser.add_argument("--camera-baud", type=int, default=460800, help="must match BAUD in the .ino")
    parser.add_argument("--no-camera", action="store_true")
    parser.add_argument("--camera-rotate", type=int, default=90, choices=[0, 90, 180, 270], help="clockwise degrees")
    parser.add_argument("--no-pi-camera", action="store_true")
    parser.add_argument("--pi-rotate", type=int, default=0, choices=[0, 90, 180, 270], help="clockwise degrees")
    parser.add_argument("--pi-width", type=int, default=1280)
    parser.add_argument("--pi-height", type=int, default=720)
    parser.add_argument("--pi-fps", type=int, default=15)
    parser.add_argument("--no-qr", action="store_true", help="rotate only, no QR detection")
    parser.add_argument(
        "--simulation-only",
        action="store_true",
        help="never open PWM hardware; sliders and IK update commanded state only",
    )
    args = parser.parse_args(argv)

    config = RobotConfig.load(args.config)
    saved = load_home()
    if saved is not None:
        config = apply_home(config, saved)
        logger.info("home loaded from %s", HOME_FILE)
    driver = NoOpDriver() if args.simulation_only else PCA9685Driver(config)
    cameras: dict = {}
    if not args.no_camera:
        try:
            esp = SerialCamera(None if args.camera_port == "auto" else args.camera_port, args.camera_baud)
        except Exception:
            esp = None
        pi = None if args.no_pi_camera else PiCamera(args.pi_width, args.pi_height, args.pi_fps)
        for name, cam, rotate in (("esp", esp, args.camera_rotate), ("pi", pi, args.pi_rotate)):
            if cam is None:
                continue
            try:
                cam.start()
            except ImportError:
                logger.warning("%s camera disabled (missing dependency, e.g. pyserial)", name)
                continue
            view = None
            try:
                view = QRView(cam, rotate, detect=not args.no_qr)
                view.start()
            except ImportError:
                logger.warning("opencv/numpy missing: no rotation or QR (pip install numpy opencv-python-headless)")
            cameras[name] = (cam, view)
    service = ArmService(
        ArmController(config, driver),
        IKBridge(),
        saved_home=saved is not None,
        home_degrees=saved,
        simulation_only=args.simulation_only,
        cameras=cameras,
    )
    if args.simulation_only:
        # Authorize the normal trajectory/state machinery against a driver
        # that cannot touch I2C. This keeps hardware mode's confirmation
        # behavior unchanged while making simulation ready immediately.
        service.arm.startup_simulation(service.home_deg)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(service))
    server.daemon_threads = True
    print(f"GUI on http://{args.host}:{args.port}  (tunnel: ssh -L {args.port}:localhost:{args.port} <user>@<pi>)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        if args.simulation_only:
            print("\nstopped. Simulation state discarded; motors were never opened.")
        else:
            print("\nstopped. The last pulses were sent.")
    finally:
        for cam, view in cameras.values():
            if view is not None:
                view.stop()
            cam.stop()
        server.server_close()
        driver.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
