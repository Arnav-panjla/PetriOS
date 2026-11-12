"""Browser slider GUI for the arm. Run on the Pi, open it through an SSH tunnel.

    ssh -L 8000:localhost:8000 pi@<pi-address>
    python -m robot_arm.interfaces.web_gui
    # then open http://localhost:8000 on the laptop

Standard library only. Binds to 127.0.0.1 so it is reachable only via the tunnel.
"""

from __future__ import annotations

import argparse
import json
import logging
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from robot_arm.config import RobotConfig
from robot_arm.control.arm_controller import STARTUP_WARNING, ArmController
from robot_arm.hardware.driver import PCA9685Driver
from robot_arm.utils.errors import RobotArmError

logger = logging.getLogger(__name__)

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Arm</title><style>
body{font-family:system-ui,sans-serif;background:#16181d;color:#e6e6e6;margin:12px auto;padding:0 12px;max-width:1400px}
#layout{display:flex;gap:16px;align-items:flex-start;flex-wrap:wrap}#left{flex:1 1 520px;min-width:320px}#right{flex:1 1 480px;position:sticky;top:8px}
canvas{width:100%;height:520px;background:#0e1014;border-radius:8px;touch-action:none;cursor:grab}
.cal{font-size:12px;color:#aaa;margin-top:4px}.cal input{width:56px;background:#111;color:#eee;border:1px solid #444;border-radius:4px;padding:2px}
h1{font-size:20px} .row{background:#20242c;border-radius:8px;padding:10px 14px;margin:10px 0}
.top{display:flex;justify-content:space-between;align-items:center;font-size:14px}
input[type=range]{width:100%;height:32px} button{padding:8px 14px;margin:4px 4px 4px 0;border:0;border-radius:6px;background:#3a6df0;color:#fff;font-size:14px;cursor:pointer}
button.warn{background:#d9822b} button.gray{background:#555} .val{font-variant-numeric:tabular-nums;font-weight:600}
#msg{min-height:20px;color:#f0c36a;font-size:14px} small{color:#999}
input[type=number]{width:70px;background:#111;color:#eee;border:1px solid #444;border-radius:4px;padding:4px}
</style></head><body>
<h1>Robot Arm Control</h1>
<div id="msg"></div>
<div class="row">
<button class="warn" id="start">Snap to home (startup)</button>
<button id="home">Home (smooth)</button>
<button class="gray" id="reset">Sliders = current</button>
<br><label><input type="checkbox" id="hold"> hold: preview only (don't move arm)</label>
<button class="warn" id="send">Set: send sliders to arm</button>
<label><input type="checkbox" id="live" checked> live drag</label>
<label>&nbsp; soften <input type="number" id="soften" min="0" value="" placeholder="yaml"> ms/deg</label>
<button class="gray" id="setsoft">set</button>
<br><small>Angles are commanded, not measured (no position feedback).</small>
</div>
<div id="layout"><div id="left"><div id="joints"></div></div>
<div id="right"><canvas id="cv"></canvas>
<div class="row" id="sim"><b>Sim pose</b> <small>(independent model angles q&deg;, does not move the arm)</small>
<div id="simsl"></div>
<button id="calib">SYNC: arm's current angles = this sim pose</button>
<button class="gray" id="simreal">Sim = real</button>
<button class="gray" id="simzero">Sim = 0</button>
<div id="caltxt" class="cal"></div>
<table id="cmp" style="width:100%;font-size:12px;margin-top:6px;border-collapse:collapse"></table></div>
<small>Colours: blue = real (commanded), green = sim pose, amber = slider target.<br>Approximate stick model from kinematics.yaml / URDF. Solid = commanded, faint = slider target. Drag to rotate, wheel to zoom.
Model angle q = sign &times; (servo&deg; &minus; offset); set offset/sign per joint below the slider (saved in your browser).
Joints 0-4 &rarr; URDF joint_1..5, joint 5 &rarr; gripper.</small></div></div>
<script>
localStorage.removeItem('cal');localStorage.removeItem('cal2'); // stale browser offsets are never used
const ZERO_SERVO_DEG=__ZERO__; // from configs/kinematics.yaml model_zero_servo_deg
let cal=ZERO_SERVO_DEG.map(o=>({o:o,s:1}));
let state=null,sliders=[],pending={},timer=null,dragging=false;
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
    <span>target <span class="val" id="t${j.index}">${j.commanded_deg.toFixed(1)}</span>&deg; &nbsp; now <span class="val" id="c${j.index}">${j.commanded_deg.toFixed(1)}</span>&deg;</span></div>
    <input type="range" min="0" max="180" step="0.5" value="${j.commanded_deg}" id="s${j.index}">
    <div class="cal">model: offset <input type="number" step="1" data-o="${j.index}" value="${cal[j.index].o}">&deg; sign <input type="number" step="2" min="-1" max="1" data-g="${j.index}" value="${cal[j.index].s}"></div>
    <div><button class="gray" data-h="${j.index}">home</button>
    <button class="gray" data-n="${j.index}" data-d="-1">-1</button><button class="gray" data-n="${j.index}" data-d="1">+1</button></div>`;
    box.appendChild(d);
    const sl=d.querySelector('input'); sliders[j.index]=sl;
    sl.addEventListener('pointerdown',()=>dragging=true);
    sl.addEventListener('input',()=>{$('t'+j.index).textContent=(+sl.value).toFixed(1); if($('live').checked)queue(j.index,+sl.value)});
    sl.addEventListener('change',()=>{dragging=false;queue(j.index,+sl.value,true)});
  });
  box.querySelectorAll('[data-o]').forEach(e=>e.onchange=()=>{cal[+e.dataset.o].o=+e.value;saveCal()});
  box.querySelectorAll('[data-g]').forEach(e=>e.onchange=()=>{cal[+e.dataset.g].s=(+e.value<0?-1:1);e.value=cal[+e.dataset.g].s;saveCal()});
  box.querySelectorAll('[data-h]').forEach(b=>b.onclick=()=>{const i=+b.dataset.h;const v=state.joints[i].home_deg;setSlider(i,v);queue(i,v,true)});
  box.querySelectorAll('[data-n]').forEach(b=>b.onclick=()=>{const i=+b.dataset.n;const v=Math.min(180,Math.max(0,+sliders[i].value+ +b.dataset.d));setSlider(i,v);queue(i,v,true)});
}
function setSlider(i,v){sliders[i].value=v;$('t'+i).textContent=(+v).toFixed(1)}
function queue(i,v,now){
  if($('hold').checked){pending={};return}
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
    if(first)build(s);
    s.joints.forEach(j=>{$('c'+j.index).textContent=j.commanded_deg.toFixed(1)});
    if(document.activeElement!==$('soften')&&s.soften_ms!==null)$('soften').value=s.soften_ms;
  }catch(e){msg('connection lost')}
}
$('start').onclick=async()=>{
  if(!confirm(`__WARNING__`))return;
  const r=await api('/api/startup',{confirmed:true});
  if(!r.error){state=null;refresh();}};
$('home').onclick=async()=>{await api('/api/home',{});state.joints.forEach(j=>setSlider(j.index,j.home_deg))};
$('reset').onclick=()=>state.joints.forEach(j=>setSlider(j.index,j.commanded_deg));
$('setsoft').onclick=()=>api('/api/soften',{ms:$('soften').value===''?null:+$('soften').value});

function saveCal(){$('caltxt').textContent='offsets in use: ['+cal.map(c=>c.o).join(', ')+']  signs: ['+cal.map(c=>c.s).join(', ')+']  (session only; edit model_zero_servo_deg in configs/kinematics.yaml to keep)'}
/* ---- tiny 3D stick model (z up, metres) ---- */
let model=null,yaw=0.8,pitch=0.5,zoom=1;
const cv=$('cv'),cx=cv.getContext('2d');
const I=()=>[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]; // column-major
function mul(a,b){const r=new Array(16).fill(0);for(let c=0;c<4;c++)for(let rr=0;rr<4;rr++)for(let k=0;k<4;k++)r[c*4+rr]+=a[k*4+rr]*b[c*4+k];return r}
function T(x,y,z){const m=I();m[12]=x;m[13]=y;m[14]=z;return m}
function R(ax,q){const[c,s]=[Math.cos(q),Math.sin(q)],[x,y,z]=ax,t=1-c,m=I();
 m[0]=t*x*x+c;m[4]=t*x*y-s*z;m[8]=t*x*z+s*y;m[1]=t*x*y+s*z;m[5]=t*y*y+c;m[9]=t*y*z-s*x;m[2]=t*x*z-s*y;m[6]=t*y*z+s*x;m[10]=t*z*z+c;return m}
const pt=(m,p=[0,0,0])=>[m[0]*p[0]+m[4]*p[1]+m[8]*p[2]+m[12],m[1]*p[0]+m[5]*p[1]+m[9]*p[2]+m[13],m[2]*p[0]+m[6]*p[1]+m[10]*p[2]+m[14]];
function qOf(i,deg){return cal[i].s*(deg-cal[i].o)} // model angle in DEGREES
function fk(degs){return fkq(degs.map((d,i)=>qOf(i,d)))}
function fkq(qs){ // returns frames [base, l1..l5] and gripper segments
  let m=I();const frames=[m];
  model.joints.forEach((j,i)=>{m=mul(m,mul(T(...j.xyz),R(j.axis,qs[i]*Math.PI/180)));frames.push(m)});
  const g=model.gripper,qf=Math.max(-0.8,Math.min(0.8,qs[5]*Math.PI/180));
  const wrist=frames[5],fingers=[];
  [[g.left,qf],[g.right,-qf]].forEach(([f,q])=>{
    const fm=mul(wrist,mul(T(...f.xyz),R([0,0,1],q)));
    fingers.push([pt(fm),pt(fm,[0.0,-0.045,0])]);});
  return{frames,fingers};
}
function proj(p,sc,w,h,ctr){
  const[x,y,z]=[p[0]-ctr[0],p[1]-ctr[1],p[2]-ctr[2]];
  const cyw=Math.cos(yaw),syw=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
  const x1=x*cyw-y*syw,y1=x*syw+y*cyw;           // rotate about z
  const y2=y1*cp-z*sp,z2=y1*sp+z*cp;             // tilt
  return[w/2+x1*sc,h/2-z2*sc,y2];}
function draw(){
  if(!model||!state){return}
  const dpr=window.devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
  if(cv.width!==w*dpr){cv.width=w*dpr;cv.height=h*dpr}
  cx.setTransform(dpr,0,0,dpr,0,0);cx.clearRect(0,0,w,h);
  const sc=Math.min(w,h)/0.55*zoom,ctr=[0,-0.1,0.12];
  const P=p=>proj(p,sc,w,h,ctr);
  const line=(a,b,col,wd)=>{const A=P(a),B=P(b);cx.strokeStyle=col;cx.lineWidth=wd;cx.beginPath();cx.moveTo(A[0],A[1]);cx.lineTo(B[0],B[1]);cx.stroke()};
  // ground grid
  for(let i=-5;i<=5;i++){const v=i*0.05;line([v,-0.35,0],[v,0.15,0],'#1d222b',1);line([-0.25,v-0.1,0],[0.25,v-0.1,0],'#1d222b',1)}
  line([0,0,0],[0.06,0,0],'#e55',2);line([0,0,0],[0,0.06,0],'#5e5',2);line([0,0,0],[0,0,0.06],'#58f',2);
  const cur=state.joints.map(j=>j.commanded_deg),tgt=sliders.map(s=>+s.value);
  [[fk(tgt),0.25,0],[fkq(simq),0.6,1],[fk(cur),1,2]].forEach(([fr,a,kind])=>{
    const{frames,fingers}=fr;
    const pts=frames.map(f=>pt(f));
    pts.push(pt(frames[5],[0.009849,-0.0687135,0]));
    const col=kind===0?'rgba(255,200,80,0.35)':kind===1?'rgba(80,230,120,0.75)':'#4da3ff';
    for(let i=0;i<pts.length-1;i++)line(pts[i],pts[i+1],col,kind===2?7:4);
    fingers.forEach(f=>line(f[0],f[1],a<1?'rgba(255,120,120,0.4)':'#ff6b6b',a<1?3:5));
    if(kind===2){
      frames.forEach((f,i)=>{const A=P(pt(f));cx.fillStyle='#fff';cx.beginPath();cx.arc(A[0],A[1],4,0,7);cx.fill();
        if(i>0){cx.fillStyle='#9ab';cx.font='11px sans-serif';cx.fillText('J'+(i-1),A[0]+7,A[1]-6)}
        if(i>0&&i<6){const ax=model.joints[i-1].axis,o=pt(f);const e=pt(f,ax.map(v=>v*0.03));line(o,e,'#7f7',2)}});
    }
  });
  const tip=pt(fk(cur).frames[5]);
  cx.fillStyle='#ccc';cx.font='12px monospace';
  cx.fillText(`wrist (base frame): x ${tip[0].toFixed(3)} y ${tip[1].toFixed(3)} z ${tip[2].toFixed(3)} m`,10,18);
}
const simq=[0,0,0,0,0,0];
(function(){const b=$('simsl');for(let i=0;i<6;i++){const d=document.createElement('div');
 d.innerHTML=`<small>${i<5?'J'+i+' (joint_'+(i+1)+')':'gripper'}</small> <span class="val" id="qv${i}">0.0</span>&deg;
 <input type="range" min="-180" max="180" step="0.5" value="0" id="q${i}" style="height:20px">`;b.appendChild(d);
 d.querySelector('input').oninput=e=>{simq[i]=+e.target.value;$('qv'+i).textContent=simq[i].toFixed(1)}}})();
function setSim(i,v){simq[i]=v;$('q'+i).value=v;$('qv'+i).textContent=v.toFixed(1)}
const realQ=()=>state.joints.map((j,i)=>qOf(i,j.commanded_deg));
$('simreal').onclick=()=>realQ().forEach((q,i)=>setSim(i,Math.max(-180,Math.min(180,q))));
$('simzero').onclick=()=>simq.forEach((_,i)=>setSim(i,0));
$('calib').onclick=()=>{
  state.joints.forEach((j,i)=>{cal[i].o=+(j.commanded_deg-cal[i].s*simq[i]).toFixed(2);
    const e=document.querySelector(`[data-o="${i}"]`);if(e)e.value=cal[i].o});
  saveCal();msg('SYNCED: offsets = '+cal.map(c=>c.o).join(', ')+'. Now moving a slider moves the blue model from this pose.');};
$('send').onclick=()=>{const a={};sliders.forEach((s,i)=>a[i]=+s.value);api('/api/move',{angles:a})};
function compare(){
  if(!state||!model)return;
  const rq=realQ(),rf=fkq(rq),sf=fkq(simq),rp=pt(rf.frames[5]),sp=pt(sf.frames[5]);
  let h='<tr style="color:#999"><td>joint</td><td>servo&deg;</td><td>real q&deg;</td><td>sim q&deg;</td><td>diff&deg;</td></tr>';
  rq.forEach((q,i)=>{const d=q-simq[i];h+=`<tr><td>${i}</td><td>${state.joints[i].commanded_deg.toFixed(1)}</td><td>${q.toFixed(1)}</td><td>${simq[i].toFixed(1)}</td><td style="color:${Math.abs(d)<0.5?'#6e6':'#f96'}">${d.toFixed(1)}</td></tr>`});
  const e=Math.hypot(rp[0]-sp[0],rp[1]-sp[1],rp[2]-sp[2])*1000;
  h+=`<tr><td colspan=5>wrist real (${rp.map(v=>(v*1000).toFixed(0)).join(', ')}) mm &nbsp; sim (${sp.map(v=>(v*1000).toFixed(0)).join(', ')}) mm &nbsp; <b>error ${e.toFixed(1)} mm</b></td></tr>`;
  $('cmp').innerHTML=h;}
setInterval(compare,250);
let drag=null;
cv.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];cv.setPointerCapture(e.pointerId)});
cv.addEventListener('pointerup',()=>drag=null);
cv.addEventListener('pointermove',e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*0.01;pitch=Math.max(-1.5,Math.min(1.5,pitch+(e.clientY-drag[1])*0.01));drag=[e.clientX,e.clientY]});
cv.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(0.3,Math.min(5,zoom*(e.deltaY<0?1.1:0.9)))},{passive:false});
(function loop(){draw();requestAnimationFrame(loop)})();
saveCal();api('/api/model').then(m=>model=m);
refresh();setInterval(refresh,300);
</script></body></html>
"""


class ArmService:
    """Thread-safe wrapper: the latest slider targets win, moves run one at a time."""

    def __init__(self, arm: ArmController) -> None:
        self.arm = arm
        self._lock = threading.Lock()
        self._pending: dict[int, float] = {}
        self._wake = threading.Event()
        self._error: str | None = None
        self._busy = False
        threading.Thread(target=self._worker, daemon=True).start()

    def status(self) -> dict:
        joints = []
        for joint in self.arm._joints:
            cal = joint.servo.calibration
            joints.append(
                {
                    "index": cal.index,
                    "channel": cal.channel,
                    "min_us": cal.min_us,
                    "max_us": cal.max_us,
                    "home_us": cal.home_us,
                    "home_deg": cal.home_deg,
                    "commanded_deg": joint.commanded_deg,
                }
            )
        error, self._error = self._error, None
        return {
            "ready": self.arm.ready,
            "busy": self._busy,
            "soften_ms": self.arm.soften_ms,
            "joints": joints,
            "error": error,
        }

    def startup(self) -> None:
        with self._lock:
            self.arm.startup(confirmed=True)

    def move(self, angles: dict[int, float]) -> None:
        if not self.arm.ready:
            raise RobotArmError("press 'Snap to home (startup)' first")
        with self._lock:
            self._pending.update(angles)
        self._wake.set()

    def home(self) -> None:
        with self._lock:
            self._pending.clear()
            self.arm.home()

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
                    current = [j.commanded_deg for j in self.arm._joints]
                    for index, deg in targets.items():
                        current[index] = deg
                    self.arm.move_all(current)
                except (RobotArmError, ValueError) as exc:
                    self._error = str(exc)
                finally:
                    self._busy = False
            if self._pending:
                self._wake.set()


def load_model() -> dict:
    """Joint origins and axes from configs/kinematics.yaml, for the browser model."""
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
    }


def make_handler(service: ArmService) -> type[BaseHTTPRequestHandler]:
    import yaml

    kin = yaml.safe_load(
        (RobotConfig.default_dir() / "kinematics.yaml").read_text(encoding="utf-8")
    )
    zero = [float(v) for v in kin["model_zero_servo_deg"]]
    page = PAGE.replace("__ZERO__", json.dumps(zero)).replace("__WARNING__", STARTUP_WARNING.replace("`", "'")).encode()

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

        def do_GET(self) -> None:
            if self.path == "/":
                self._send(200, page, "text/html; charset=utf-8")
            elif self.path == "/api/model":
                self._json(load_model())
            elif self.path == "/api/status":
                self._json(service.status())
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self) -> None:
            length = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
                if self.path == "/api/startup":
                    if not body.get("confirmed"):
                        raise RobotArmError("startup not confirmed")
                    service.startup()
                elif self.path == "/api/move":
                    angles = {int(k): float(v) for k, v in body["angles"].items()}
                    for index, deg in angles.items():
                        if not 0 <= index < 6 or not 0.0 <= deg <= 180.0:
                            raise ValueError(f"bad joint/angle: {index} {deg}")
                    service.move(angles)
                elif self.path == "/api/home":
                    service.home()
                elif self.path == "/api/soften":
                    ms = body.get("ms")
                    if ms is not None and float(ms) < 0:
                        raise ValueError("soften must be >= 0")
                    service.arm.soften_ms = None if ms is None else float(ms)
                else:
                    return self._json({"error": "not found"}, 404)
            except (RobotArmError, ValueError, KeyError, TypeError) as exc:
                return self._json({"error": str(exc)})
            self._json({"ok": True})

    return Handler


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    parser = argparse.ArgumentParser(description="Slider web GUI for the arm.")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--host", default="127.0.0.1", help="use 0.0.0.0 to skip the SSH tunnel (insecure)")
    parser.add_argument("--config", type=Path, default=None)
    args = parser.parse_args(argv)

    config = RobotConfig.load(args.config)
    driver = PCA9685Driver(config)
    service = ArmService(ArmController(config, driver))
    server = ThreadingHTTPServer((args.host, args.port), make_handler(service))
    print(f"GUI on http://{args.host}:{args.port}  (tunnel: ssh -L {args.port}:localhost:{args.port} <user>@<pi>)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped. The last pulses were sent.")
    finally:
        server.server_close()
        driver.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
