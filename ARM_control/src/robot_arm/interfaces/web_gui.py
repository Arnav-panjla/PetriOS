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
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from robot_arm.config import RobotConfig
from robot_arm.control.arm_controller import STARTUP_WARNING, ArmController
from robot_arm.hardware.driver import PCA9685Driver
from robot_arm.utils.errors import RobotArmError
from robot_arm.vision.esp32_serial import SerialCamera

logger = logging.getLogger(__name__)

IK_PATH = Path(__file__).resolve().parents[3] / "ik" / "arm_ik_v2.py"
HOME_FILE = RobotConfig.default_dir() / "home.txt"

# Used only when the IK module cannot be imported. Same values as arm_ik_v2.py
# (J0-J4) plus the gripper servo, so the model still draws.
FALLBACK_ZERO = [90.0, 39.0, 58.0, 66.5, 116.25]
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
body{font-family:system-ui,sans-serif;background:#16181d;color:#e6e6e6;margin:12px auto;padding:0 12px;max-width:1400px}
#layout{display:flex;gap:16px;align-items:flex-start;flex-wrap:wrap}#left{flex:1 1 520px;min-width:320px}#right{flex:1 1 480px;position:sticky;top:8px}
h1{font-size:20px} .row{background:#20242c;border-radius:8px;padding:10px 14px;margin:10px 0}
.top{display:flex;justify-content:space-between;align-items:center;font-size:14px;flex-wrap:wrap;gap:6px}
input[type=range]{width:100%;height:32px} button{padding:8px 14px;margin:4px 4px 4px 0;border:0;border-radius:6px;background:#3a6df0;color:#fff;font-size:14px;cursor:pointer}
button:disabled{opacity:.4;cursor:default} button.warn{background:#d9822b} button.gray{background:#555} .val{font-variant-numeric:tabular-nums;font-weight:600}
#msg{min-height:20px;color:#f0c36a;font-size:14px} small{color:#999}
input[type=number],select{background:#111;color:#eee;border:1px solid #444;border-radius:4px;padding:4px}
input[type=number]{width:78px}
canvas{width:100%;height:480px;background:#0e1014;border-radius:8px;touch-action:none;cursor:grab}
pre{background:#111;border-radius:6px;padding:8px;font-size:12px;white-space:pre-wrap;margin:6px 0 0}
</style></head><body>
<h1>Robot Arm Control</h1>
<div id="msg"></div>
<div class="row">
<button class="warn" id="start">Snap to home (startup)</button>
<button id="home">Home (smooth)</button>
<button class="gray" id="reset">Sliders = current</button>
<button class="warn" id="sethome">Set current pose as new home</button>
<label><input type="checkbox" id="live" checked> live drag</label>
<label>&nbsp; soften <input type="number" id="soften" min="0" value="" placeholder="yaml"> ms/deg</label>
<button class="gray" id="setsoft">set</button>
<br><small id="homeinfo"></small>
<br><small>Angles are commanded, not measured (no position feedback).</small>
</div>
<div id="layout"><div id="left"><div id="joints"></div></div>
<div id="right"><canvas id="cv"></canvas>
<small>Blue = commanded pose, amber = slider target, green = IK solution. Drag to rotate, wheel to zoom.
Model angle q = sign &times; (servo&deg; &minus; zero), taken from arm_ik_v2.py. Joints 0-4 &rarr; joint_1..5. Gripper (J5): 167.5&deg; closed, 10&deg; open (3.5 cm).</small>
<div class="row" id="camrow"><b>Gripper camera</b> <small id="camstat"></small><br>
<img id="cam" alt="camera" style="width:100%;max-height:360px;object-fit:contain;background:#000;border-radius:6px"></div>
<div class="row" id="ik"><b>Inverse kinematics</b> <small id="ikstat"></small><br>
<select id="ikmode"><option value="abs">absolute TCP (base_link, mm)</option><option value="rel">relative to current TCP (mm)</option></select>
x <input type="number" step="1" id="ikx" value="0"> y <input type="number" step="1" id="iky" value="0"> z <input type="number" step="1" id="ikz" value="0">
<label><input type="checkbox" id="ikhold"> hold tool direction</label><br>
<button class="gray" id="ikfill">Fill from current TCP</button>
<button id="iksolve">Solve (preview only)</button>
<button class="warn" id="ikgo" disabled>Move arm to solution</button>
<pre id="ikout">TCP = estimated closed-fingertip midpoint. Solving never moves the arm.</pre></div>
</div></div>
<script>
const ZERO=__ZERO__,SIGN=__SIGN__;
let state=null,sliders=[],pending={},timer=null,model=null,ikPrev=null;
const $=id=>document.getElementById(id);
const msg=t=>{$('msg').textContent=t||''};
async function api(path,body){
  const r=await fetch(path,{method:body?'POST':'GET',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined});
  const j=await r.json(); if(j.error)msg(j.error); else if(body)msg(''); return j;}
function build(s){
  const box=$('joints'); box.innerHTML=''; sliders=[];
  s.joints.forEach(j=>{
    const d=document.createElement('div'); d.className='row';
    d.innerHTML=`<div class="top"><b>Joint ${j.index}</b><small>ch ${j.channel} | home ${j.home_deg.toFixed(1)}&deg; | ${j.min_us}-${j.max_us} us</small>
    <span>target <span class="val" id="t${j.index}">${j.commanded_deg.toFixed(1)}</span>&deg; &nbsp; now <span class="val" id="c${j.index}">${j.commanded_deg.toFixed(1)}</span>&deg; &nbsp; ${j.index===5?'opening <span class="val" id="q5">0</span> mm':'q <span class="val" id="q'+j.index+'">0</span>&deg;'}</span></div>
    <input type="range" min="${j.min_deg}" max="${j.max_deg}" step="0.5" value="${j.commanded_deg}" id="s${j.index}">
    <div><button class="gray" data-h="${j.index}">home</button>
    <button class="gray" data-n="${j.index}" data-d="-1">-1</button><button class="gray" data-n="${j.index}" data-d="1">+1</button></div>`;
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
  await api('/api/move',{angles:p});
}
async function refresh(){
  try{
    const s=await api('/api/status'); const first=!state; state=s;
    $('start').style.display=s.ready?'none':'';
    if(first||s.home_version!==homeVersion){homeVersion=s.home_version;build(s)}
    s.joints.forEach(j=>{$('c'+j.index).textContent=j.commanded_deg.toFixed(1);$('q'+j.index).textContent=(j.index===5?openFrac(j.commanded_deg)*model_w()*1000:qOf(j.index,j.commanded_deg)).toFixed(1)});
    $('homeinfo').textContent='home (servo deg): ['+s.joints.map(j=>j.home_deg.toFixed(1)).join(', ')+']  '+s.home_source;
    if(document.activeElement!==$('soften')&&s.soften_ms!==null)$('soften').value=s.soften_ms;
  }catch(e){msg('connection lost')}
}
let homeVersion=-1;
$('start').onclick=async()=>{
  if(!confirm(`__WARNING__`))return;
  const r=await api('/api/startup',{confirmed:true});
  if(!r.error){state=null;refresh();}};
$('home').onclick=async()=>{await api('/api/home',{});state.joints.forEach(j=>setSlider(j.index,j.home_deg))};
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
$('ikfill').onclick=()=>{
  if($('ikmode').value==='rel'){['x','y','z'].forEach(k=>$('ik'+k).value=0);return}
  const t=fk(state.joints.map(j=>j.commanded_deg)).tcp;['x','y','z'].forEach((k,i)=>$('ik'+k).value=+(t[i]*1000).toFixed(1));};
$('ikmode').onchange=()=>$('ikfill').click();
$('iksolve').onclick=async()=>{
  $('ikgo').disabled=true;ikPrev=null;$('ikout').textContent='solving...';
  const r=await api('/api/ik/solve',{mode:$('ikmode').value,x:+$('ikx').value,y:+$('iky').value,z:+$('ikz').value,hold:$('ikhold').checked});
  if(r.error){$('ikout').textContent='IK: '+r.error;return}
  ikPrev=r.servo;$('ikgo').disabled=false;
  $('ikout').textContent=`target TCP ${f1(r.target_mm)} mm\\nservo J0-J4 ${f3(r.servo.slice(0,5))} deg\\nchange      ${f3(r.change)} deg\\npulses us   [${r.pulses.join(', ')}]\\nposition error ${r.err_mm.toFixed(3)} mm`+(r.axis_err_deg===null?'':`\\ndirection error ${r.axis_err_deg.toFixed(2)} deg`)+'\\nPreview only: green model. Press "Move arm to solution" to execute.';
};
$('ikgo').onclick=async()=>{
  if(!ikPrev||!confirm('Move the real arm to the IK solution? Check for collisions first.'))return;
  const a={};for(let i=0;i<5;i++)a[i]=ikPrev[i];
  const r=await api('/api/move',{angles:a});
  if(!r.error){for(let i=0;i<5;i++)setSlider(i,ikPrev[i]);ikPrev=null;$('ikgo').disabled=true}};
api('/api/model').then(m=>{model=m;$('ikstat').textContent=m.ik_ok?'(arm_ik_v2.py loaded)':'(IK unavailable: '+m.ik_error+')';$('iksolve').disabled=!m.ik_ok});
refresh();setInterval(refresh,300);
function camTick(){
  const c=state&&state.camera;
  if(!c){$('camstat').textContent='(disabled)';return}
  if(!$('cam').getAttribute('src'))$('cam').src='/camera.mjpg';
  $('camstat').textContent=c.connected?`(${c.port||'auto'}, ${c.fps.toFixed(1)} fps)`:'(not connected: '+(c.error||'waiting')+')';
}
setInterval(camTick,1000);
</script></body></html>
"""


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
        vec = np.array([float(body["x"]), float(body["y"]), float(body["z"])]) / 1000.0  # mm in
        if body.get("mode") == "rel":
            T0, _ = mod.fk(mod.to_q(start), tcp)
            target = T0[:3, 3] + vec
        else:
            target = vec
        hold = bool(body.get("hold"))
        q, _T, err, axis_err = mod.solve(target, tcp, start, hold)
        servo = mod.to_servo(q)
        pulses = [mod.pwm(i, float(v)) for i, v in enumerate(servo)]
        return {
            "target_mm": [float(v) * 1000.0 for v in target],
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
        camera: SerialCamera | None = None,
    ) -> None:
        self.arm = arm
        self.camera = camera
        self.ik = ik
        self.home_deg = [j.home_deg for j in arm.config.joints]
        self.home_source = "(saved in configs/home.txt)" if saved_home else "(from joints.yaml)"
        self.home_version = 0
        self._lock = threading.Lock()
        self._pending: dict[int, float] = {}
        self._wake = threading.Event()
        self._error: str | None = None
        self._busy = False
        threading.Thread(target=self._worker, daemon=True).start()

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
            "joints": joints,
            "home_source": self.home_source,
            "home_version": self.home_version,
            "camera": None
            if self.camera is None
            else {
                "connected": self.camera.connected,
                "fps": self.camera.fps,
                "port": self.camera.port,
                "error": self.camera.last_error,
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
            self._pending.update(angles)
        self._wake.set()

    def home(self) -> None:
        if not self.arm.ready:
            raise RobotArmError("press 'Snap to home (startup)' first")
        with self._lock:
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
            self.wfile.write(body)

        def _json(self, payload: dict, code: int = 200) -> None:
            self._send(code, json.dumps(payload).encode(), "application/json")

        def _stream_camera(self) -> None:
            cam = service.camera
            if cam is None:
                return self._json({"error": "camera disabled"}, 404)
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
                jpeg = service.camera.latest() if service.camera else None
                if jpeg is None:
                    self._json({"error": "no camera frame"}, 503)
                else:
                    self._send(200, jpeg, "image/jpeg")
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
    parser.add_argument("--camera-baud", type=int, default=921600, help="must match BAUD in the .ino")
    parser.add_argument("--no-camera", action="store_true")
    args = parser.parse_args(argv)

    config = RobotConfig.load(args.config)
    saved = load_home()
    if saved is not None:
        config = apply_home(config, saved)
        logger.info("home loaded from %s", HOME_FILE)
    driver = PCA9685Driver(config)
    camera = None
    if not args.no_camera:
        try:
            import serial  # noqa: F401  (pyserial)

            port = None if args.camera_port == "auto" else args.camera_port
            camera = SerialCamera(port, args.camera_baud)
            camera.start()
        except ImportError:
            logger.warning("pyserial missing, camera disabled (pip install pyserial)")
    service = ArmService(
        ArmController(config, driver),
        IKBridge(),
        saved_home=saved is not None,
        camera=camera,
    )
    server = ThreadingHTTPServer((args.host, args.port), make_handler(service))
    server.daemon_threads = True
    print(f"GUI on http://{args.host}:{args.port}  (tunnel: ssh -L {args.port}:localhost:{args.port} <user>@<pi>)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped. The last pulses were sent.")
    finally:
        if camera is not None:
            camera.stop()
        server.server_close()
        driver.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
