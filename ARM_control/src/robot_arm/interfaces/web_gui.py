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
from robot_arm.motion.pause import PausableClock
from robot_arm.motion.session import SessionRecorder
from robot_arm.utils.errors import RobotArmError
from robot_arm.vision.esp32_serial import SerialCamera
from robot_arm.vision.qr_view import QRView

logger = logging.getLogger(__name__)

IK_PATH = Path(__file__).resolve().parents[3] / "ik" / "arm_ik_v2.py"
HOME_FILE = RobotConfig.default_dir() / "home.txt"
RECORDINGS_DIR = RobotConfig.default_dir().parent / "recordings"

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

PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="light">
<title>Robot Arm</title><style>
:root{color-scheme:light;
--bench:#eaeee9;--panel:#fafbf8;--well:#e1e7e0;--line:#d0d8cf;--line2:#aab6ab;
--tx:#13201c;--tx2:#34433c;--mut:#566860;
--acc:#0a6b60;--acc-hi:#07554c;--acc-bg:#0a6b60;--acc-tint:#0a6b6018;
--warn:#9a5206;--warn-hi:#7f4204;--bad:#b3261e;--ok:#1a7541;
--stage:#0d1815;--stage-line:#1c2d28;
--r:10px;--rc:6px;--ease:cubic-bezier(.16,1,.3,1)}
*{box-sizing:border-box}
html{scrollbar-color:#a3b0a5 transparent;scrollbar-width:thin}
::-webkit-scrollbar{width:10px;height:10px}::-webkit-scrollbar-thumb{background:#a3b0a5;border-radius:99px;border:2px solid var(--bench)}
body{font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",sans-serif;background:var(--bench);color:var(--tx);margin:0;padding:0 20px 40px;accent-color:var(--acc);caret-color:var(--acc);font-variant-numeric:tabular-nums}
::selection{background:#0a6b6040}
:focus-visible{outline:2px solid var(--acc);outline-offset:2px;border-radius:4px}
small,.muted{color:var(--mut)}
a{color:var(--acc);text-underline-offset:3px}
b{font-weight:650}

/* app bar */
#top{position:sticky;top:0;z-index:10;margin:0 -20px 18px;padding:0 20px;background:#eaeee9f2;backdrop-filter:blur(10px);border-bottom:1px solid var(--line)}
#top .line{display:flex;gap:8px 24px;align-items:center;flex-wrap:wrap;min-height:52px}
#top h1{font-size:17px;font-weight:700;letter-spacing:-.01em;margin:0;white-space:nowrap;display:flex;align-items:center;gap:10px}
#top h1 svg{flex:none;color:var(--acc)}
#tabs{display:flex;gap:4px;align-self:stretch;margin-left:4px}
#tabs button{background:none;color:var(--mut);border:0;border-bottom:2px solid transparent;border-radius:0;padding:0 12px;font-size:14px;font-weight:550;margin-bottom:-1px}
#tabs button:hover{color:var(--tx);background:none}
#tabs button[aria-selected=true]{color:var(--tx);border-bottom-color:var(--acc)}
#status{margin-left:auto;display:flex;align-items:center;gap:8px;flex-wrap:wrap}
.chip{display:inline-flex;align-items:center;gap:7px;font-size:12.5px;font-weight:600;color:var(--tx2);background:var(--panel);border:1px solid var(--line);border-radius:99px;padding:3px 11px 3px 9px;white-space:nowrap}
.chip::before{content:"";width:8px;height:8px;border-radius:50%;background:var(--line2)}
.chip.ok::before{background:var(--ok)}
.chip.wait{color:var(--warn)}
.chip.wait::before{background:var(--warn)}
.chip.sim{color:var(--warn);border-color:#9a520655;background:#9a520612}
.chip.sim::before{background:var(--warn)}
#mode:empty{display:none}
#msg{display:none;margin:0 0 10px;padding:8px 12px;border-radius:var(--rc);font-size:13px;font-weight:500;color:#7a1a14;background:#b3261e14;border:1px solid #b3261e55}
#msg:not(:empty){display:block}

/* command strip */
#tools{display:flex;flex-wrap:wrap;gap:10px 32px;padding:4px 0 12px}
.grp{display:flex;gap:8px 14px;align-items:center;flex-wrap:wrap}
.grp label{font-size:13px;color:var(--tx2);white-space:nowrap;display:inline-flex;align-items:center;gap:6px;cursor:pointer}
.grp .gl{font-size:13px;font-weight:700;color:var(--tx);margin-right:2px}
.grp input[type=checkbox],.form input[type=checkbox]{width:16px;height:16px;margin:0}
#tcpz{font-size:13px;min-width:92px;font-weight:550}
@media(max-width:900px){#top{position:static}}
@media(max-width:700px){#status{margin-left:0;width:100%}}

/* layout */
#grid{display:grid;grid-template-columns:minmax(380px,6fr) minmax(380px,5fr);gap:20px;align-items:start}
@media(max-width:900px){#grid{grid-template-columns:1fr}}
.col{display:flex;flex-direction:column;gap:20px;min-width:0}
.view{position:sticky;top:calc(var(--barh,120px) + 18px)}
@media(max-width:900px){.view{position:static}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:var(--r)}
.card .card{background:var(--well);border-color:transparent}
.card>summary{list-style:none;cursor:pointer;padding:14px 18px 12px;font-weight:700;font-size:15px;letter-spacing:-.005em;display:flex;align-items:baseline;gap:10px;border-radius:var(--r);user-select:none}
.card>summary:hover{background:#0a6b600a}
.card>summary::-webkit-details-marker{display:none}
.card>summary::before{content:"";flex:none;align-self:center;width:6px;height:6px;border-right:2px solid var(--mut);border-bottom:2px solid var(--mut);transform:rotate(45deg) translate(-2px,-2px);transition:transform .25s var(--ease)}
.card:not([open])>summary::before{transform:rotate(-45deg)}
.card>summary small{font-weight:400;font-size:12.5px}
.body{padding:2px 18px 18px}

/* controls */
button{padding:8px 14px;border:1px solid transparent;border-radius:var(--rc);background:var(--acc-bg);color:#fff;font:inherit;font-size:13px;font-weight:600;cursor:pointer;transition:background-color .15s,border-color .15s,color .15s,transform .1s}
button:hover:not(:disabled){background:var(--acc-hi)}
button:active:not(:disabled){transform:translateY(1px)}
button.gray,button.mini{background:var(--panel);color:var(--tx);border-color:var(--line2)}
button.gray:hover:not(:disabled),button.mini:hover:not(:disabled){background:#fff;border-color:var(--acc);color:var(--acc-hi)}
button.mini{padding:4px 9px;min-width:34px;font-weight:550;font-size:12.5px}
button.mini.on{background:var(--warn);border-color:var(--warn);color:#fff}
button.warn{background:var(--warn)}
button.warn:hover:not(:disabled){background:var(--warn-hi)}
button:disabled{opacity:.42;cursor:not-allowed}
.val{font-weight:650}
input[type=number],select{background:#fff;color:var(--tx);border:1px solid var(--line2);border-radius:var(--rc);padding:6px 8px;font:inherit;font-size:13px}
input[type=number]:hover,select:hover{border-color:var(--acc)}
input[type=number]{width:74px}
input:disabled{opacity:.5;cursor:not-allowed}
input[type=range]{accent-color:var(--acc)}
.sec{font-size:13px;font-weight:700;color:var(--tx);margin:20px 0 4px}
.sec:first-child{margin-top:2px}
.jrow{display:grid;grid-template-columns:132px minmax(0,1fr) 112px 54px;gap:10px;align-items:center;padding:8px 0;border-top:1px solid var(--line)}
.jrow.hd{border-top:0;padding:0 0 2px;font-size:12px;color:var(--mut)}
.sec+.jrow{border-top:0}
.jl{display:flex;flex-direction:column;line-height:1.3;min-width:0}
.jn{font-size:12px}
.jrow .sl{width:100%;height:28px;cursor:pointer;margin:0}
.vc{display:flex;align-items:center;gap:4px}
.vc .unit{font-size:12px;color:var(--mut);min-width:22px}
.vin{width:80px!important;text-align:right}
.vin:disabled,.sl:disabled{opacity:.4;cursor:not-allowed}
.sw{display:flex;align-items:center;gap:8px;font-size:13px;color:var(--tx2);padding:6px 0 8px;cursor:pointer}
details.sub{margin-top:14px;border-top:1px solid var(--line)}
details.sub>summary{cursor:pointer;padding:12px 0 4px;font-size:13px;font-weight:600;color:var(--tx2);list-style:none}
details.sub>summary::-webkit-details-marker{display:none}
details.sub>summary::before{content:"+";display:inline-block;width:14px;color:var(--acc);font-weight:700}
details.sub[open]>summary::before{content:"-"}
.form{display:flex;gap:8px 10px;align-items:center;flex-wrap:wrap;margin-bottom:10px}
#ik .form+.muted{font-size:12.5px}
.ikbtns{display:flex;gap:8px;flex-wrap:wrap}
pre{background:var(--well);border:1px solid var(--line);border-radius:var(--rc);padding:12px;font:12.5px/1.55 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;white-space:pre-wrap;margin:12px 0 0;color:var(--tx2)}
table{border-collapse:collapse} td{padding:5px 10px 5px 0;border-bottom:1px solid var(--line)}

/* viewport */
.seg{display:inline-flex;background:var(--well);border-radius:8px;padding:3px;margin:0 0 12px}
.seg button{background:none;color:var(--mut);padding:5px 14px;border-radius:6px;border:0}
.seg button:hover:not(:disabled){background:#fff8;color:var(--tx)}
.seg button[aria-pressed=true]{background:#fff;color:var(--tx);box-shadow:0 1px 2px #13201c26}
canvas{width:100%;height:460px;background:var(--stage);border:1px solid var(--stage-line);border-radius:8px;touch-action:none;cursor:grab;display:block}
canvas:active{cursor:grabbing}
.legend{font-size:12.5px;margin-top:10px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:50%;margin:0 6px 0 14px;vertical-align:-1px}
.legend i:first-child{margin-left:0}
@media(max-width:560px){.jrow.hd{display:none}.jrow{grid-template-columns:minmax(0,1fr) 112px}.jrow .sl{grid-column:1/-1;order:3}.jrow button{display:none}canvas{height:340px}}

/* cameras */
#camline{font-size:12.5px;margin-top:8px}
.camhead{display:flex;align-items:center;gap:8px 12px;flex-wrap:wrap;margin-bottom:10px;font-size:13px}
.camimg{width:100%;object-fit:contain;background:#000;border-radius:8px;display:block}
.meter{height:10px;background:var(--well);border-radius:5px;overflow:hidden;margin-top:8px}
#wc_bar{height:100%;width:0;background:var(--ok);transition:width .15s}
#wc_res td{padding:3px 10px 3px 0}
#sess_tbl td,#sess_tbl th{padding:3px 10px 3px 0;text-align:right;font-weight:400}
#sess_tbl th{color:var(--tx2)}
#sess_tbl tr.cur td{font-weight:700;color:#4da3ff}
.wcrow{display:flex;gap:18px;flex-wrap:wrap}
.wcrow>div{flex:1 1 360px}
.btns{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
@media(prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body>
<header id="top">
 <div class="line">
  <h1><svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><circle cx="5" cy="19" r="2"/><circle cx="10" cy="8" r="2"/><circle cx="19" cy="5" r="2"/><path d="M6 17.2 9 9.8M11.8 7.4 17.2 5.6M19 7.5v4M17 11.5h4"/></svg>Robot Arm</h1>
  <nav id="tabs" role="tablist" aria-label="Sections"><button id="tabbtn_arm" role="tab" aria-selected="true">Arm</button><button id="tabbtn_cam" role="tab" aria-selected="false">Webcam</button></nav>
  <div id="status"><span class="chip" id="rdy">Connecting</span><span id="mode" class="chip sim"></span></div>
 </div>
 <div id="tools">
  <div class="grp"><span class="gl">Pose</span><button class="warn" id="start" title="Drive every joint to the saved home pose">Snap to home (startup)</button>
   <button id="home" title="Move every joint to the saved home pose">Go home</button><button class="gray" id="reset" title="Reset sliders to the commanded pose">Sync sliders</button>
   <button class="gray" id="sethome" title="Save the current pose to configs/home.txt">Save pose as home</button></div>
  <div class="grp"><span class="gl">Motion</span><label><input type="checkbox" id="live" checked> Move while dragging</label>
   <label>Soften <input type="number" id="soften" min="0" value="" placeholder="yaml"> ms/deg</label>
   <button class="gray" id="setsoft">Set</button></div>
  <div class="grp"><span class="gl">Safety</span><label><input type="checkbox" id="flooron" checked> Floor guard</label>
   <label>Floor <input type="number" id="floormm" step="1" value="5" style="width:64px"> mm</label>
   <span id="tcpz" class="muted">TCP z -</span></div>
  <div class="grp"><span class="gl">Wrist</span><label><input type="checkbox" id="levelon"> Keep gripper level</label>
   <label>J4 roll <input type="number" id="levelroll" step="1" min="0" max="180" value="28" style="width:64px">&deg;</label></div>
 </div>
 <div id="msg" role="alert"></div>
</header>
<div id="tab_arm">
<div id="grid">
 <div class="col">
  <details class="card" open><summary>Joints <small>commanded angles, not measured (no position feedback)</small></summary>
   <div class="body"><div id="joints"></div><div class="muted" style="font-size:12.5px;margin-top:12px" id="homeinfo"></div></div></details>
  <details class="card" id="ik" open><summary>Inverse kinematics <small id="ikstat"></small></summary><div class="body">
   <div class="form"><select id="ikmode" aria-label="Target type"><option value="abs">absolute TCP (x, y, z mm, base_link)</option><option value="rel">relative to current TCP (dx, dy, dz mm)</option><option value="cyl_rel">relative cylindrical (dr mm, d&theta; deg, dz mm)</option><option value="cyl_abs">absolute cylindrical (r mm, &theta; deg, z mm)</option></select>
    <span id="lx">x</span> <input type="number" step="1" id="ikx" value="0" aria-labelledby="lx"> <span id="ly">y</span> <input type="number" step="1" id="iky" value="0" aria-labelledby="ly"> <span id="lz">z</span> <input type="number" step="1" id="ikz" value="0" aria-labelledby="lz">
    <label class="muted" style="display:inline-flex;align-items:center;gap:6px"><input type="checkbox" id="ikhold"> hold tool direction</label></div>
   <div class="muted" style="margin:2px 0 10px" id="tcpnow">TCP now: -</div>
   <div class="ikbtns"><button class="gray" id="ikfill">Fill from current TCP</button>
   <button id="iksolve">Preview solution</button>
   <button class="warn" id="ikgo" disabled>Move arm to solution</button></div>
   <pre id="ikout">TCP = estimated closed-fingertip midpoint. Solving never moves the arm.</pre>
  </div></details>
  <details class="card" id="petri" open><summary>Petri transfer <small>shared arm path</small></summary><div class="body">
   <p class="muted" style="font-size:13px">Follow the level carousel-to-camera path. The 3D arm model follows its commanded angles; a running Gazebo sync bridge mirrors the same angles. In hardware mode those commands go to the motors.</p>
   <div class="form"><label>Speed <select id="petri_speed"><option value="1.5">1.5×</option><option value="2.0">2.0×</option><option value="3.0" selected>3.0×</option><option value="4.0">4.0×</option><option value="6.0">6.0×</option></select></label>
   <button class="warn" id="petri_run" disabled>Follow Petri path</button></div>
   <pre id="petri_status">Petri path is idle.</pre>
  </div></details>
  <details class="card" id="sess" open><summary>Session recorder <small>saves joint poses to recordings/</small></summary><div class="body">
   <p class="muted" style="font-size:13px">Move the arm with the sliders, then press Record to save every joint's angle as one point. While a session is open, Hold becomes Record hold: it saves a wait of that many seconds after the last point. Play goes to each point in order using the arm's own interpolation and waits at the points that have a hold.</p>
   <div class="form"><button id="sess_start">Start session</button><button id="sess_rec" disabled>Record</button><button class="gray" id="sess_end" disabled>End session</button></div>
   <div class="form"><select id="sess_file" aria-label="Session to play"></select><button class="warn" id="sess_play" disabled>Play</button><button class="gray" id="sess_stop" disabled>Stop</button></div>
   <div class="form"><label>Hold <input type="number" id="sess_hold_s" min="1" max="600" step="1" value="5" style="width:64px"> s</label>
    <button class="warn" id="sess_hold" title="Freeze the arm where it is for this long, then carry on with whatever it was doing">Hold</button>
    <button class="gray" id="sess_resume" disabled>Resume now</button></div>
   <pre id="sess_msg">No session.</pre>
   <div style="overflow:auto;max-height:260px"><table id="sess_tbl" style="font-size:13px"></table></div>
  </div></details>
 </div>
 <div class="col view">
  <div class="card"><div class="body" style="padding-top:16px">
   <div class="seg" role="group" aria-label="Right panel"><button id="vbtn_model" aria-pressed="true">3D model</button><button id="vbtn_cam" aria-pressed="false">ESP32 camera</button></div>
   <div id="view_model">
    <canvas id="cv" aria-label="3D model of the arm. Drag to rotate, scroll to zoom."></canvas>
    <div class="legend muted"><i style="background:#4da3ff"></i>commanded <i style="background:#ffc850"></i>slider target <i style="background:#50e678"></i>IK solution
     &nbsp;&middot; drag to rotate, wheel to zoom</div>
    <div class="legend muted">q = sign &times; (servo&deg; &minus; zero) from arm_ik_v2.py. Gripper: 167.5&deg; closed, 10&deg; open (3.5 cm).</div>
   </div>
   <div id="view_cam" style="display:none"><div id="cams"></div><div id="camline" class="muted"></div></div>
  </div></div>
 </div>
</div>
</div>
<div id="tab_cam" style="display:none">
 <details class="card" open><summary>Webcam <small id="wc_stat"></small></summary><div class="body">
  <div class="wcrow">
   <div>
    <img id="wc_img" class="camimg" style="max-height:480px" alt="Webcam live view">
    <div class="meter"><div id="wc_bar"></div></div>
    <div class="muted" style="font-size:12.5px;margin:6px 0 12px">Focus <b id="wc_focus">-</b> (peak <b id="wc_peak">-</b>). Turn the focus ring until the bar stops rising.</div>
    <div class="btns"><button id="wc_cap">Capture and detect</button>
    <button class="gray" id="wc_rot">Rotate 90&deg;</button>
    <button class="gray" id="wc_reset">Reset focus peak</button>
    <label style="display:inline-flex;align-items:center;gap:6px;font-size:13px;color:var(--tx2)"><input type="checkbox" id="wc_live"> Live dish outline</label></div>
    <details class="card" open style="margin-top:10px"><summary>Light ring <small id="led_stat"></small></summary><div class="body">
     <div class="btns" style="align-items:center">
      <label style="display:inline-flex;align-items:center;gap:6px;font-size:13px"><input type="checkbox" id="led_on"> On</label>
      <input type="color" id="led_color" value="#ffffff" aria-label="Light colour" style="width:44px;height:30px;padding:0;border:0;background:none">
      <button class="gray" id="led_white">White</button>
     </div>
     <div class="jctl" style="margin:8px 0 0"><span style="width:90px;font-size:12px">Brightness</span><input type="range" id="led_bri" min="0" max="100" step="1" value="50"><span class="val" id="led_bri_v">50</span></div>
    </div></details>
   </div>
   <div>
    <img id="wc_resimg" class="camimg" style="max-height:480px;display:none" alt="Annotated capture">
    <div id="wc_resinfo" class="muted" style="font-size:13px;margin-top:8px">No capture yet. Blue = dish, red = disk, purple = zone.</div>
    <table id="wc_res" style="font-size:13px"></table>
    <details class="card" open style="margin-top:10px"><summary>Detection tuning <small>re-runs on the captured frame</small></summary><div class="body">
     <label><input type="checkbox" id="wc_manual"> manual dish circle</label>
     <div id="wc_mdish" style="display:none"></div>
     <div id="wc_params"></div>
     <button class="gray" id="wc_defaults">Reset to defaults</button>
    </div></details>
    <a id="wc_dl" href="/webcam_result.jpg" download="petri_result.jpg" style="display:none">Download annotated image</a>
   </div>
  </div>
 </div></details>
</div>
<script>
const ZERO=__ZERO__,SIGN=__SIGN__;
const SIMULATION_ONLY=__SIMULATION_ONLY__;
let state=null,sliders=[],pending={},timer=null,model=null,ikPrev=null;
const $=id=>document.getElementById(id);
const msg=t=>{$('msg').textContent=t||''};
async function api(path,body){
  const r=await fetch(path,{method:body?'POST':'GET',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined});
  const j=await r.json(); if(j.error)msg(j.error); else if(body)msg(''); return j;}
async function petriRefresh(){const s=await api('/api/petri/status');if(s.error)return;
  $('petri_run').disabled=!state?.ready||s.active;
  $('petri_status').textContent=s.message|| (s.active?'Petri transfer is running…':'Petri path is idle.');}
$('petri_run').onclick=async()=>{const r=await api('/api/petri/run',{speed:+$('petri_speed').value});if(!r.error)petriRefresh()};
setInterval(petriRefresh,1000);
let sessFiles='';
async function sessRefresh(){const s=await api('/api/session/status');if(s.error)return;
  const rdy=!!(state&&state.ready);
  $('sess_start').disabled=s.playing;
  $('sess_rec').disabled=!rdy||!s.active||s.playing;
  $('sess_end').disabled=!s.active||s.playing;
  $('sess_play').disabled=!rdy||s.playing||!s.files.length;
  $('sess_stop').disabled=!s.playing;
  const rec=s.active&&!s.playing;
  $('sess_hold').disabled=!rdy&&!rec;
  $('sess_hold').textContent=rec?'Record hold':'Hold';
  $('sess_hold').title=rec?'Save this wait after the last recorded point; playback pauses there':'Freeze the arm where it is for this long, then carry on with whatever it was doing';
  $('sess_resume').disabled=!(s.hold_left>0);
  const key=s.files.join('|'),sel=$('sess_file');
  if(key!==sessFiles){const keep=sel.value;sessFiles=key;
    sel.innerHTML=s.files.map(f=>`<option value="${f}">${f}</option>`).join('');
    if(keep&&s.files.includes(keep))sel.value=keep}
  $('sess_msg').textContent=(s.message||'No session.')+(s.active?'\\nRecording to '+s.path:'')+(s.hold_left>0?'\\nHOLDING: '+s.hold_left.toFixed(0)+' s left':'');
  $('sess_tbl').innerHTML='<tr><th>#</th><th>time / stage</th>'+[0,1,2,3,4,5].map(i=>`<th>J${i}</th>`).join('')+'<th>hold</th></tr>'
    +s.points.map(p=>`<tr class="${s.playing&&p.n===s.play_index?'cur':''}"><td>${p.n}</td><td>${p.time}</td>${p.angles.map(a=>`<td>${a.toFixed(1)}</td>`).join('')}<td>${p.hold_s>0?p.hold_s+' s':''}</td></tr>`).join('')}
const sessDo=a=>async()=>{await api('/api/session',{action:a,file:$('sess_file').value});sessRefresh()};
$('sess_start').onclick=sessDo('start');$('sess_rec').onclick=sessDo('record');$('sess_end').onclick=sessDo('end');
$('sess_play').onclick=sessDo('play');$('sess_stop').onclick=sessDo('stop');
$('sess_hold').onclick=async()=>{await api('/api/session',{action:'hold',seconds:+$('sess_hold_s').value});sessRefresh()};
$('sess_resume').onclick=sessDo('resume');
setInterval(sessRefresh,800);
const CYL=[['r','r','mm',1],['h','height z','mm',1],['t','theta','deg',0.5],['p','tool pitch','deg',0.5]];
let cylLast=0,cylTimer=null;
const clamp=(v,a,b)=>Math.min(b,Math.max(a,v));
function rowHtml(o){return `<div class="jrow"><div class="jl"><span><b>${o.name}</b> <span class="muted">${o.sub}</span></span><span class="jn muted">now <span class="val" id="${o.nid}">0</span> ${o.unit}${o.q||''}</span></div>
  <input type="range" class="sl" id="${o.sid}" min="${o.min}" max="${o.max}" step="${o.step}" value="${o.val}" aria-label="${o.name} slider">
  <label class="vc"><input type="number" class="vin" id="${o.vid}" min="${o.min}" max="${o.max}" step="${o.step}" value="${(+o.val).toFixed(1)}" aria-label="${o.name} value"><span class="unit">${o.unit}</span></label>
  ${o.home!==undefined?`<button class="mini" data-h="${o.home}" title="Go to this joint's home angle">home</button>`:'<span></span>'}</div>`}
const HEAD='<div class="jrow hd"><span></span><span>Slider</span><span>Value</span><span></span></div>';
function buildCyl(s){
  const box=$('cyl'); box.innerHTML='';
  const ws=s.workspace; if(!ws||!s.cyl){box.innerHTML='<div class="muted" style="padding:6px 0">Tool-position sliders need IK (numpy + scipy). Use the raw joint sliders below.</div>';return}
  const rng={r:ws.r,h:ws.h,t:ws.theta,p:ws.pitch};
  box.innerHTML=HEAD+CYL.map(([k,name,unit,step])=>rowHtml({name,sub:'',nid:'cn_'+k,sid:'cs_'+k,vid:'cv_'+k,unit,step,min:Math.floor(rng[k][0]),max:Math.ceil(rng[k][1]),val:0})).join('');
  CYL.forEach(([k])=>{
    const sl=$('cs_'+k),vin=$('cv_'+k);
    sl.addEventListener('input',()=>{vin.value=(+sl.value).toFixed(1);cylLast=Date.now();if($('live').checked)sendCyl(false)});
    sl.addEventListener('change',()=>{cylLast=Date.now();sendCyl(true)});
    vin.addEventListener('change',()=>{const v=clamp(+vin.value||0,+sl.min,+sl.max);sl.value=v;vin.value=v.toFixed(1);cylLast=Date.now();sendCyl(true)});
  });
  syncCyl(true);
}
function syncCyl(force){
  if(!state||!state.cyl||!$('cs_r'))return;
  const v={r:state.cyl[0],t:state.cyl[1],h:state.cyl[2],p:state.tool_pitch_deg};
  CYL.forEach(([k])=>{$('cn_'+k).textContent=v[k].toFixed(1);
    if((force||Date.now()-cylLast>1500)&&document.activeElement!==$('cv_'+k)){$('cs_'+k).value=v[k];$('cv_'+k).value=v[k].toFixed(1)}});
  const lv=!!state.level_enabled;
  $('cs_p').disabled=lv;$('cv_p').disabled=lv;
  if(state.joints.length>3&&sliders[3]){sliders[3].disabled=lv;$('t3').disabled=lv}
}
function sendCyl(now){
  clearTimeout(cylTimer);
  const go=async()=>{cylTimer=null;
    const r=await api('/api/cyl/move',{r:+$('cs_r').value,h:+$('cs_h').value,theta:+$('cs_t').value,pitch:+$('cs_p').value});
    if(r.error){cylLast=0;syncCyl(true)}};
  if(now)go(); else cylTimer=setTimeout(go,80);
}
const ROLES=['base','shoulder','elbow','wrist pitch','wrist roll','gripper'];
function build(s){
  const box=$('joints');
  box.innerHTML='<div class="sec">Tool position</div><div id="cyl"></div><div class="sec">Wrist and gripper</div><div id="wg"></div><details class="sub" id="rawd"><summary>Raw joints J0-J3</summary><div id="rawj"></div></details>';
  sliders=[];
  buildCyl(s);
  $('wg').innerHTML=HEAD;$('rawj').innerHTML=HEAD;
  s.joints.forEach(j=>{
    const i=j.index,grip=i===5;
    const tmp=document.createElement('div');
    tmp.innerHTML=rowHtml({name:'J'+i,sub:ROLES[i],nid:'c'+i,sid:'s'+i,vid:'t'+i,unit:grip?'deg':'deg',step:0.5,min:j.min_deg,max:j.max_deg,val:j.commanded_deg,home:i,
      q:grip?' &middot; <span class="val" id="q5">0</span> mm':' &middot; q <span class="val" id="q'+i+'">0</span>&deg;'});
    const row=tmp.firstElementChild;row.title=`channel ${j.channel}, home ${j.home_deg.toFixed(1)} deg`;
    (i<4?$('rawj'):$('wg')).appendChild(row);
    const sl=$('s'+i),vin=$('t'+i); sliders[i]=sl;
    sl.addEventListener('input',()=>{vin.value=(+sl.value).toFixed(1); if($('live').checked)queue(i,+sl.value)});
    sl.addEventListener('change',()=>queue(i,+sl.value,true));
    vin.addEventListener('change',()=>{const v=clamp(+vin.value||0,j.min_deg,j.max_deg);setSlider(i,v);queue(i,v,true)});
    row.querySelector('[data-h]').onclick=()=>{const v=state.joints[i].home_deg;setSlider(i,v);queue(i,v,true)};
  });
  const sw=document.createElement('label');sw.className='sw';
  sw.title='On: the gripper eases to its target with the other joints. Off: it jumps straight to the target while the arm still interpolates.';
  sw.innerHTML='<input type="checkbox" id="gripint" checked> Interpolate gripper motion';
  $('wg').appendChild(sw);
  $('gripint').onchange=()=>api('/api/gripper_interp',{enabled:$('gripint').checked});
}
function setSlider(i,v){sliders[i].value=v;$('t'+i).value=(+v).toFixed(1)}
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
    const rd=$('rdy');rd.textContent=s.busy?'Moving':s.ready?'Ready':'Awaiting startup';rd.className='chip '+(s.ready?'ok':'wait');
    if(first||s.home_version!==homeVersion){homeVersion=s.home_version;build(s)}
    s.joints.forEach(j=>{$('c'+j.index).textContent=j.commanded_deg.toFixed(1);$('q'+j.index).textContent=(j.index===5?openFrac(j.commanded_deg)*model_w()*1000:qOf(j.index,j.commanded_deg)).toFixed(1)});
    $('homeinfo').textContent='home (servo deg): ['+s.joints.map(j=>j.home_deg.toFixed(1)).join(', ')+']  '+s.home_source;
    if(document.activeElement!==$('soften')&&s.soften_ms!==null)$('soften').value=s.soften_ms;
    if(document.activeElement!==$('floormm'))$('floormm').value=s.floor_mm;
    $('flooron').checked=s.floor_enabled;
    $('levelon').checked=!!s.level_enabled;
    if($('gripint')&&document.activeElement!==$('gripint'))$('gripint').checked=s.grip_interp!==false;
    if(document.activeElement!==$('levelroll'))$('levelroll').value=s.level_roll_deg;
    syncCyl(false);
    const tz=$('tcpz');
    if(!s.guard_available){tz.textContent='z guard unavailable (no IK/numpy)';tz.style.color='var(--bad)'}
    else if(s.tcp_z_mm!==null){tz.textContent='TCP z '+s.tcp_z_mm.toFixed(1)+' mm';tz.style.color=s.tcp_z_mm<s.floor_mm?'var(--bad)':''}
  }catch(e){msg('Connection lost. Is the server still running?');$('rdy').textContent='Offline';$('rdy').className='chip'}
}
let homeVersion=-1;
$('start').onclick=async()=>{
  if(!confirm(`__WARNING__`))return;
  const r=await api('/api/startup',{confirmed:true});
  if(!r.error){state=null;refresh();}};
$('home').onclick=async()=>{const r=await api('/api/home',{});if(!r.error){cylLast=0;state.joints.forEach(j=>setSlider(j.index,j.home_deg))}};
$('levelon').onchange=async()=>{const r=await api('/api/level',{enabled:$('levelon').checked,roll_deg:+$('levelroll').value});if(r.error)$('levelon').checked=false;cylLast=0};
$('levelroll').onchange=()=>{if($('levelon').checked)api('/api/level',{enabled:true,roll_deg:+$('levelroll').value})};
$('flooron').onchange=()=>api('/api/floor',{enabled:$('flooron').checked});
$('floormm').onchange=()=>api('/api/floor',{floor_mm:+$('floormm').value});
$('reset').onclick=()=>{cylLast=0;syncCyl(true);state.joints.forEach(j=>setSlider(j.index,j.commanded_deg))};
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
  if(!model||!state||!sliders.length||tabNow!=='arm'||viewNow!=='model')return;
  const dpr=window.devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
  if(cv.width!==w*dpr){cv.width=w*dpr;cv.height=h*dpr}
  cx.setTransform(dpr,0,0,dpr,0,0);cx.clearRect(0,0,w,h);
  const sc=Math.min(w,h)/0.55*zoom,ctr=[0,-0.1,0.12];
  const P=p=>proj(p,sc,w,h,ctr);
  const line=(a,b,col,wd)=>{const A=P(a),B=P(b);cx.strokeStyle=col;cx.lineWidth=wd;cx.beginPath();cx.moveTo(A[0],A[1]);cx.lineTo(B[0],B[1]);cx.stroke()};
  for(let i=-5;i<=5;i++){const v=i*0.05;line([v,-0.35,0],[v,0.15,0],'#1b2c27',1);line([-0.25,v-0.1,0],[0.25,v-0.1,0],'#1b2c27',1)}
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
  cx.fillStyle='#b6c6bf';cx.font='12px ui-monospace,monospace';
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
$('mode').textContent=SIMULATION_ONLY?'Simulation only, motors disabled':'';
if(SIMULATION_ONLY){$('start').style.display='none';$('start').disabled=true}
refresh();setInterval(refresh,300);
const CAMS=[['esp','ESP32-CAM']];
const OPTS=[['detect','Detect QR'],['focus','Focus meter'],['denoise','Denoise'],['sharpen','Sharpen'],['clahe','Contrast (CLAHE)'],['show_filtered','Show filtered image'],['wechat','WeChat detector'],['robust','Robust decode']];
function optsHtml(k){
  const boxes=OPTS.map(([o,t])=>`<label style="display:inline-flex;align-items:center;gap:5px;margin:3px 14px 3px 0;font-size:13px;color:var(--tx2)"><input type="checkbox" data-cam="${k}" data-opt="${o}"> ${t}</label>`).join('');
  const rot=`<div style="margin:2px 0 6px;font-size:13px">Rotate (clockwise) <select data-rot="${k}"><option>0</option><option>90</option><option>180</option><option>270</option></select>
    <button class="mini" data-rotstep="${k}">+90&deg;</button></div>`;
  return `<details class="sub"><summary>Image options</summary><div style="padding-top:6px">${rot}${boxes}<div id="params_${k}" style="margin-top:8px"></div></div></details>`;
}
function buildParams(k,c){
  const box=$('params_'+k);if(!box||!c.options||!c.options.param_spec)return;
  let html='<div class="muted" style="font-size:12px;margin-bottom:4px">Filter values <button class="mini" data-preset="'+k+'">reset to defaults</button></div>',grp='';
  c.options.param_spec.forEach(s=>{
    if(s.group!==grp){grp=s.group;html+=`<div class="sec">${s.group}</div>`}
    html+=`<div style="display:flex;align-items:center;gap:8px;font-size:12px"><span style="width:150px">${s.label}</span>
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
const camVisible=()=>tabNow==='arm'&&viewNow==='cam';
async function pullCam(k){ // ask for a frame newer than the last one shown; never queues; idles while hidden
  let seq=-1,url=null;
  for(;;){
    if(!camVisible()){await new Promise(z=>setTimeout(z,400));continue}
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
  if(!cs||!cs.esp){box.innerHTML='<div class="muted" style="padding:8px 0">ESP32 camera disabled (started with --no-camera).</div>';return}
  const k='esp',c=cs.esp;
  let d=$('cam_esp');
  if(!d){d=document.createElement('div');d.id='cam_esp';
    d.innerHTML=`<div class="camhead"><small id="cs_esp"></small><span style="flex:1"></span><button class="mini" id="light">Light: off</button>
      <input type="range" id="lbr" min="0" max="100" step="1" value="20" style="width:110px" aria-label="Light brightness"><span id="lbrv" class="muted" style="width:32px">20%</span></div>
      <img id="img_esp" alt="ESP32-CAM live view" style="width:100%;max-height:440px;object-fit:contain;background:#000;border-radius:6px;display:block"><div id="ql_esp" style="font-size:13px;margin-top:6px"></div>${optsHtml(k)}`;
    box.innerHTML='';box.appendChild(d);pullCam(k);wireOpts(k,c);
    $('light').onclick=()=>api('/api/camera/light',{on:!(state.cameras.esp&&state.cameras.esp.light)});
    let lt=null,ld=false;const lb=$('lbr');
    lb.oninput=()=>{ld=true;$('lbrv').textContent=lb.value+'%';if(!lt)lt=setTimeout(()=>{lt=null;api('/api/camera/light',{brightness:+lb.value})},80)};
    lb.onchange=async()=>{clearTimeout(lt);lt=null;await api('/api/camera/light',{brightness:+lb.value});ld=false};
    lb.onpointerdown=()=>{ld=true};lb.onpointerup=()=>{ld=false};window.lbDrag=()=>ld}
  if(!camVisible())return;
  syncOpts(k,c);
  const on=!!c.light;$('light').textContent='Light: '+(on?'on':'off');$('light').classList.toggle('on',on);
  if(c.brightness!=null&&!(window.lbDrag&&window.lbDrag())){$('lbr').value=c.brightness;$('lbrv').textContent=Math.round(c.brightness)+'%'}
  $('cs_esp').textContent=c.connected?`${c.port||'auto'}, ${c.fps.toFixed(1)} fps`+(c.bytes===0?' - nothing received':(c.fps===0?' - no valid frames':'')):'Not connected: '+(c.error||'waiting');
  const t=c.stats;
  $('camline').textContent=t?`${t.width}x${t.height} · jpeg ${t.jpeg_kb} KB · process ${t.proc_ms} ms · detect ${t.detect_ms} ms`+(c.options&&c.options.focus?` · focus ${t.focus} (peak ${t.focus_peak})`:'')+` · QR seen ${t.seen_pct}%, decoded ${t.decoded_pct}%`:'';
}
function qrTick(){
  const cs=state&&state.cameras;if(!cs||!camVisible())return;
  CAMS.forEach(([k])=>{
    const q=cs[k]&&cs[k].qr,box=$('ql_'+k);if(!box)return;
    box.innerHTML='';
    if(!q){return}
    if(!q.length){box.innerHTML='<small>no QR code in view</small>';return}
    q.forEach(c=>{
      const d=document.createElement('div');d.style.color=c.ok?'var(--ok)':'var(--bad)';
      if(c.ok&&/^https?:\\/\\//i.test(c.text)){const a=document.createElement('a');a.href=c.text;a.target='_blank';a.rel='noopener';a.textContent=c.text;a.style.color='var(--acc)';d.appendChild(a)}
      else d.appendChild(document.createTextNode(c.text));
      box.appendChild(d)});
  });
}
setInterval(camTick,1000);setInterval(qrTick,500);
/* ---- tabs and webcam ---- */
let tabNow='arm',viewNow='model',wcRun=false,wcSeq=-1;
function showTab(t){tabNow=t;
  $('tab_arm').style.display=t==='arm'?'':'none';$('tab_cam').style.display=t==='cam'?'':'none';$('tools').style.display=t==='arm'?'':'none';
  $('tabbtn_arm').setAttribute('aria-selected',t==='arm');$('tabbtn_cam').setAttribute('aria-selected',t==='cam');
  sizeBar();
  if(t==='cam'&&!wcRun){wcRun=true;wcPull()}}
$('tabbtn_arm').onclick=()=>showTab('arm');$('tabbtn_cam').onclick=()=>showTab('cam');
function showView(v){viewNow=v;
  $('view_model').style.display=v==='model'?'':'none';$('view_cam').style.display=v==='cam'?'':'none';
  $('vbtn_model').setAttribute('aria-pressed',v==='model');$('vbtn_cam').setAttribute('aria-pressed',v==='cam')}
$('vbtn_model').onclick=()=>showView('model');$('vbtn_cam').onclick=()=>showView('cam');
function sizeBar(){document.documentElement.style.setProperty('--barh',$('top').offsetHeight+'px')}
new ResizeObserver(sizeBar).observe($('top'));
async function wcPull(){ // only while the Webcam tab is open
  let url=null;
  while(tabNow==='cam'){
    try{
      const r=await fetch('/webcam.jpg?seq='+wcSeq+'&t='+Date.now(),{cache:'no-store'});
      if(r.status===200){wcSeq=+r.headers.get('X-Seq');const u=URL.createObjectURL(await r.blob());$('wc_img').src=u;if(url)URL.revokeObjectURL(url);url=u}
      else await new Promise(z=>setTimeout(z,r.status===404?2000:150));
    }catch(e){await new Promise(z=>setTimeout(z,1000))}
  }
  wcRun=false;
}
function wcTick(){
  const w=state&&state.webcam;if(tabNow!=='cam'||!w)return;
  $('wc_stat').textContent=w.connected?`(${w.port}, ${w.size[0]}x${w.size[1]}, ${w.fps.toFixed(1)} fps)`:'(not connected: '+(w.error||'waiting')+')';
  $('wc_focus').textContent=w.focus.toFixed(1);$('wc_peak').textContent=w.focus_peak.toFixed(1);
  $('wc_bar').style.width=(w.focus_peak>0?Math.min(100,100*w.focus/w.focus_peak):0)+'%';
  if(document.activeElement!==$('wc_live'))$('wc_live').checked=w.live_dish;
  if(w.size&&w.size[0])wcMan.frame=w.size;
  if(w.spec&&w.spec.length&&!$('wc_params').children.length)wcBuild(w);
  const r=w.result;
  if(r&&(r.n+':'+r.rev)!==wcTick.last){wcTick.last=r.n+':'+r.rev;
    $('wc_resimg').src='/webcam_result.jpg?t='+Date.now();$('wc_resimg').style.display='';$('wc_dl').style.display='';
    $('wc_resinfo').textContent=`${r.time}: dish R=${r.dish_r_mm.toFixed(1)} mm (${r.mm_per_px.toFixed(4)} mm/px), ${r.disks.length} disk(s), focus ${r.focus.toFixed(0)}`+(r.vlm==='reading'?' - reading disk labels on the GPU...':'');
    $('wc_res').innerHTML='<tr class="muted"><td>disk</td><td>R mm</td><td>zone R mm</td><td>label</td></tr>'+r.disks.map((d,i)=>`<tr><td>#${i+1} (${d.x.toFixed(0)},${d.y.toFixed(0)})</td><td>${d.r_mm.toFixed(1)}</td><td>${d.zone_mm==null?'none':d.zone_mm.toFixed(1)+(d.clipped?' (lower bound)':'')}</td><td>${d.label||(r.vlm==='reading'?'...':'')}</td></tr>`).join('')}
}
/* ---- light ring ---- */
let ledT=null;
function ledSend(body){clearTimeout(ledT);ledT=setTimeout(()=>api('/api/led',body),120)}
$('led_on').onchange=()=>ledSend({on:$('led_on').checked});
$('led_color').oninput=()=>ledSend({color:$('led_color').value,on:true});
$('led_bri').oninput=()=>{$('led_bri_v').textContent=$('led_bri').value;ledSend({brightness:$('led_bri').value/100})};
$('led_white').onclick=()=>api('/api/led',{white:true});
function ledTick(){
  const l=state&&state.led;
  $('led_stat').textContent=!l?'(disabled)':(l.available?'':'(SPI not available - no light)');
  if(!l)return;
  if(document.activeElement!==$('led_on'))$('led_on').checked=l.on;
  if(document.activeElement!==$('led_color'))$('led_color').value='#'+l.color.map(c=>c.toString(16).padStart(2,'0')).join('');
  if(document.activeElement!==$('led_bri')){$('led_bri').value=Math.round(l.brightness*100);$('led_bri_v').textContent=$('led_bri').value}
}
setInterval(ledTick,700);
setInterval(wcTick,500);
let wcT=null;const wcMan={frame:[640,480]};
function wcSend(extra){clearTimeout(wcT);wcT=setTimeout(async()=>{
  const params={};document.querySelectorAll('[data-wp]').forEach(e=>params[e.dataset.wp]=+e.value);
  const body=Object.assign({params},extra||{});
  if($('wc_manual').checked)body.manual={x:+$('wm_x').value,y:+$('wm_y').value,r:+$('wm_r').value};else body.manual=null;
  const r=await api('/api/webcam/detect',body);if(r.error)$('wc_resinfo').textContent=r.error},200)}
function wcSlider(box,id,label,min,max,step,val,attr){
  const d=document.createElement('div');d.className='jctl';d.style.margin='4px 0';
  d.innerHTML=`<span style="width:230px;font-size:12px">${label}</span><input type="range" ${attr} id="${id}" min="${min}" max="${max}" step="${step}" value="${val}"><span class="val" id="${id}_v">${val}</span>`;
  box.appendChild(d);const e=d.querySelector('input');
  e.oninput=()=>{$(id+'_v').textContent=(+e.value).toFixed(step<1?2:0);wcSend()};return e}
function wcBuild(w){
  const box=$('wc_params');
  w.spec.forEach(sp=>{const v=(w.params&&w.params[sp.key]!=null)?w.params[sp.key]:sp.default;wcSlider(box,'wp_'+sp.key,sp.label,sp.min,sp.max,sp.step,v,'data-wp="'+sp.key+'" data-def="'+sp.default+'"')});
  const m=$('wc_mdish');const f=wcMan.frame;
  const d=w.result?w.result.dish:[f[0]/2,f[1]/2,Math.min(f[0],f[1])/2];
  wcSlider(m,'wm_x','dish centre x px',0,f[0],1,Math.round(d[0]),'');
  wcSlider(m,'wm_y','dish centre y px',0,f[1],1,Math.round(d[1]),'');
  wcSlider(m,'wm_r','dish radius px',10,Math.max(f[0],f[1]),1,Math.round(d[2]),'');
}
$('wc_manual').onchange=()=>{
  const on=$('wc_manual').checked;$('wc_mdish').style.display=on?'':'none';
  const r=state&&state.webcam&&state.webcam.result;
  if(on&&r){[['wm_x',0],['wm_y',1],['wm_r',2]].forEach(([id,i])=>{$(id).value=Math.round(r.dish[i]);$(id+'_v').textContent=Math.round(r.dish[i])})}
  wcSend()};
$('wc_defaults').onclick=()=>{document.querySelectorAll('[data-wp]').forEach(e=>{e.value=e.dataset.def;$(e.id+'_v').textContent=e.dataset.def});$('wc_manual').checked=false;$('wc_mdish').style.display='none';wcSend()};
$('wc_cap').onclick=async()=>{const r=await api('/api/webcam/capture',{});if(r.error)$('wc_resinfo').textContent=r.error};
$('wc_rot').onclick=()=>api('/api/webcam/options',{rotate_step:90});
$('wc_reset').onclick=()=>api('/api/webcam/options',{reset_peak:true});
$('wc_live').onchange=()=>api('/api/webcam/options',{live_dish:$('wc_live').checked});
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

    def axis_xy(self) -> list[float]:
        return [float(v) for v in self.mod.JOINT_ORIGINS[0][:2]]

    def tool_pitch_deg(self, servo_deg: list[float]) -> float | None:
        """Angle of the tool axis above the horizontal (0 = level, -90 = pointing down)."""
        if not self.ok:
            return None
        import numpy as np

        mod = self.mod
        T, _ = mod.fk(mod.to_q(np.array(servo_deg[:5])), np.array(mod.TCP_ESTIMATE))
        z = float(np.clip((T[:3, :3] @ mod.TOOL_AXIS_LOCAL)[2], -1.0, 1.0))
        return math.degrees(math.asin(z))

    def cylindrical(self, servo_deg: list[float]) -> list[float] | None:
        pos = self.tcp_position(servo_deg)
        return None if pos is None else list(to_cylindrical(pos, self.axis_xy()))

    def workspace(self) -> dict | None:
        """r, theta, z ranges reachable inside the soft limits (sampled once)."""
        if not self.ok:
            return None
        if getattr(self, "_ws", None) is None:
            import numpy as np

            lim = self.mod.SOFT_LIMIT_DEG
            grids = [np.linspace(lim[j][0], lim[j][1], n) for j, n in ((0, 9), (1, 9), (2, 9), (3, 3))]
            pts = []
            for a in grids[0]:
                for b in grids[1]:
                    for c in grids[2]:
                        pts.append(self.cylindrical([a, b, c, 90.0, 90.0]))
            arr = np.array(pts)
            self._ws = {
                "r": [float(max(0.0, arr[:, 0].min())), float(arr[:, 0].max())],
                "theta": [float(arr[:, 1].min()), float(arr[:, 1].max())],
                "h": [float(arr[:, 2].min()), float(arr[:, 2].max())],
                "pitch": [-90.0, 90.0],
            }
        return self._ws

    def level_j3(self, servo: list[float], pitch_deg: float = 0.0) -> float:
        """J3 servo angle that gives the requested tool pitch for the other joints."""
        if not self.ok:
            raise ValueError("IK unavailable")
        from scipy.optimize import brentq

        lo, hi = (float(v) for v in self.mod.SOFT_LIMIT_DEG[3])

        def err(j3: float) -> float:
            pose = list(servo[:5])
            pose[3] = j3
            return self.tool_pitch_deg(pose) - pitch_deg

        grid = [lo + (hi - lo) * i / 36 for i in range(37)]
        vals = [err(g) for g in grid]
        for a, b, fa, fb in zip(grid, grid[1:], vals, vals[1:], strict=False):
            if fa == 0:
                return a
            if fa * fb < 0:
                return float(brentq(err, a, b, xtol=1e-4))
        raise ValueError(f"blocked: cannot keep level, J3 would be outside {lo:g}..{hi:g}")

    def solve_cyl(self, current: list[float], r: float, theta: float, h: float, pitch: float) -> list[float]:
        """Servo angles J0..J3 (J4 kept) for a cylindrical TCP target and tool pitch."""
        if not self.ok:
            raise ValueError(f"IK unavailable ({self.error}). pip install numpy scipy")
        import numpy as np

        mod = self.mod
        if r < 0:
            raise ValueError(f"blocked: radius {r:.1f} mm must be >= 0")
        start = np.array(current[:5], dtype=float)
        target = np.array(from_cylindrical(r, theta, h, self.axis_xy()))
        try:
            q, _T, _err, _perr = mod.solve(target, np.array(mod.TCP_ESTIMATE), start, False, pitch)
        except ValueError as exc:
            raise ValueError(
                f"blocked: unreachable (r={r:.0f}, theta={theta:.0f}, h={h:.0f}, pitch={pitch:.0f} deg); "
                "a level tool needs the wrist further out/higher, try a larger r or h"
            ) from exc
        servo = [float(v) for v in mod.to_servo(q)]
        servo[4] = float(current[4])
        return servo

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

    webcam = None  # robot_arm.vision.webcam.WebCam, set by main()
    led = None  # robot_arm.hardware.led_ring.LedRing, set by main()
    _playing = False  # a recorded session is being played back
    _play_index = 0
    _play_points: list = []
    _play_message = ""
    pause_clock = None  # robot_arm.motion.pause.PausableClock shared with the ArmController, set by main()
    _hold_until = 0.0
    _hold_timer: threading.Timer | None = None

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
        self._petri_lock = threading.Lock()
        self._petri_active = False
        self._petri_stage = "idle"
        self._petri_message = "Petri path is idle."
        self._petri_event_seq = 0
        self._petri_event_kind: str | None = None
        self._petri_event_ack = 0
        self._petri_bridge_seen = 0.0
        self._petri_mirror_gazebo = False
        self._play_event_enabled = False
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
        self.level_enabled = False
        self.grip_interp = True  # False: gripper jumps to its target, other joints still interpolate
        self.level_roll_deg = 28.0
        self._cyl_lock = threading.Lock()
        self.session = SessionRecorder(RECORDINGS_DIR)
        self._session_lock = threading.Lock()
        self._hold_lock = threading.Lock()
        self._play_stop = threading.Event()
        threading.Thread(target=self._worker, daemon=True).start()

    def source(self, name: str):
        entry = self.cameras.get(name)
        return None if entry is None else (entry[1] or entry[0])

    def commanded(self) -> list[float]:
        return [j.commanded_deg for j in self.arm._joints]

    def status(self) -> dict:
        petri = self.petri_status()
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
            "busy": self._busy or petri["active"],
            "soften_ms": self.arm.soften_ms,
            "floor_enabled": self.floor_enabled,
            "floor_mm": self.floor_mm,
            "guard_available": self.ik.ok,
            "tcp_z_mm": self.tcp_z_mm(),
            "cyl": self.ik.cylindrical(self.commanded()),
            "tool_pitch_deg": self.ik.tool_pitch_deg(self.commanded()),
            "level_enabled": self.level_enabled,
            "grip_interp": self.grip_interp,
            "level_roll_deg": self.level_roll_deg,
            "workspace": self.ik.workspace(),
            "joints": joints,
            "home_source": self.home_source,
            "home_version": self.home_version,
            "simulation_only": self.simulation_only,
            "petri": petri,
            "transfer": petri,
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
                    "qr": view.results() if view is not None else None,
                    "stats": view.stats() if view is not None else None,
                }
                for name, (cam, view) in self.cameras.items()
            },
            "webcam": None
            if self.webcam is None
            else {
                "connected": self.webcam.connected,
                "port": self.webcam.port,
                "fps": self.webcam.fps,
                "size": list(self.webcam.size),
                "focus": self.webcam.focus,
                "focus_peak": self.webcam.focus_peak,
                "live_dish": self.webcam.live_dish,
                "rotate": self.webcam.rotate,
                "error": self.webcam.last_error,
                "result": self.webcam.result,
                "params": self.webcam.params,
                "manual_dish": self.webcam.manual_dish,
                "spec": self.webcam.spec(),
            },
            "led": None if self.led is None else self.led.state(),
            "error": error,
        }

    def petri_status(self) -> dict:
        with self._petri_lock:
            return {
                "active": self._petri_active or (self._playing and self._play_event_enabled),
                "stage": self._petri_stage,
                "message": self._petri_message,
                "event_seq": self._petri_event_seq,
                "event_kind": self._petri_event_kind,
            }

    def petri_bridge_heartbeat(self) -> None:
        with self._petri_lock:
            self._petri_bridge_seen = time.monotonic()

    def petri_event_ack(self, event_seq: int) -> None:
        with self._petri_lock:
            if (self._petri_active or (self._playing and self._play_event_enabled)) and event_seq == self._petri_event_seq:
                self._petri_event_ack = event_seq

    def start_petri_transfer(self, speed: float) -> None:
        if self._playing:
            raise RobotArmError("stop the recorded session before following the Petri path")
        max_speed = 6.0 if self.simulation_only else 3.0
        if not 0.2 <= speed <= max_speed:
            raise ValueError(f"speed must be between 0.2 and {max_speed:g} for this mode")
        with self._petri_lock:
            if self._petri_active:
                raise RobotArmError("Petri path is already running")
            if not self.arm.ready:
                raise RobotArmError("start the arm before following the Petri path")
            bridge_connected = time.monotonic() - self._petri_bridge_seen <= 3.0
            if self.simulation_only and not bridge_connected:
                raise RobotArmError("Gazebo sync bridge is not connected; launch gazebo_stage2_gui_sync first")
            with self._lock:
                if self._busy or self._pending:
                    raise RobotArmError("wait for the current arm command to finish")
            self._petri_active = True
            self._petri_mirror_gazebo = bridge_connected
            self._petri_stage = "planning"
            self._petri_message = "Planning level Petri transfer…"
            threading.Thread(target=self._run_petri_transfer, args=(speed,), daemon=True).start()

    def _run_petri_transfer(self, speed: float) -> None:
        """Run every planned waypoint through the same arm command path as sliders."""
        try:
            from robot_arm.motion.petri_transfer import SIMULATION_SCENE, Station, plan_transfer

            mod = self.ik.mod
            if mod is None:
                raise RobotArmError("IK model is unavailable")
            start = self.commanded()
            steps = plan_transfer(
                mod, start, self.home_deg,
                Station.from_dict(SIMULATION_SCENE["source"]),
                Station.from_dict(SIMULATION_SCENE["destination"]),
            )
            logger.info("Petri path planned: %d waypoints at %.2fx", len(steps), speed)
            previous = start
            for index, step in enumerate(steps, start=1):
                with self._petri_lock:
                    self._petri_stage = step.stage
                    self._petri_message = f"{step.stage.replace('_', ' ').title()} ({index}/{len(steps)})"
                target = list(step.servo_deg)
                delta = max(abs(a - b) for a, b in zip(previous, target, strict=True))
                duration = max(0.10 / speed, delta / (0.85 * speed))
                if step.stage in ("grip", "release"):
                    duration = max(duration, 0.8 / speed)
                # This is the existing ArmController interpolation path. Its
                # live commanded angles drive both the browser model and bridge.
                self.arm.move_all(target, duration_s=duration)
                previous = target
                if step.event and self._petri_mirror_gazebo:
                    with self._petri_lock:
                        self._petri_event_seq += 1
                        event_seq = self._petri_event_seq
                        self._petri_event_kind = step.event
                        self._petri_event_ack = 0
                    deadline = time.monotonic() + 15.0
                    while time.monotonic() < deadline:
                        with self._petri_lock:
                            if self._petri_event_ack == event_seq:
                                break
                        time.sleep(0.05)
                    else:
                        raise RobotArmError(f"Gazebo did not acknowledge {step.event}; check Gazebo bridge and joint state")
                    with self._petri_lock:
                        self._petri_event_kind = None
            with self._petri_lock:
                self._petri_stage = "complete"
                self._petri_message = f"Petri transfer complete ({len(steps)} waypoints)."
        except Exception as exc:
            logger.exception("Petri transfer stopped")
            with self._petri_lock:
                self._petri_stage = "stopped"
                self._petri_message = f"Stopped: {exc}"
                self._petri_event_kind = None
        finally:
            with self._petri_lock:
                self._petri_active = False

    def session_status(self) -> dict:
        rec = self.session
        shown = rec.points if rec.active and not self._playing else self._play_points
        return {
            "active": rec.active,
            "file": rec.file.name if rec.file else None,
            "path": str(rec.file) if rec.file else None,
            "points": list(shown),
            "files": rec.files(),
            "playing": self._playing,
            "play_index": self._play_index,
            "message": self._play_message,
            "hold_left": self.hold_left(),
        }

    def session_start(self) -> None:
        if self._playing:
            raise RobotArmError("stop the playback first")
        path = self.session.start()
        self._play_points = []
        self._play_message = f"Session started: {path.name}"

    def session_end(self) -> None:
        if self.session.active:
            self._play_points = list(self.session.points)
            self._play_message = f"Session saved: {self.session.file.name} ({len(self.session.points)} point(s))"
        self.session.end()

    def session_record(self) -> dict:
        """Snapshot every joint's commanded angle into the session file."""
        if self._playing:
            raise RobotArmError("cannot record while playing")
        if not self.arm.ready:
            raise RobotArmError("press 'Snap to home (startup)' first")
        if not self.session.active:
            raise RobotArmError("start a session first")
        deadline = time.monotonic() + 5.0
        while True:  # let a slider move finish so the snapshot is where the arm ended up
            with self._lock:
                if not self._busy and not self._pending:
                    break
            if time.monotonic() > deadline:
                raise RobotArmError("arm is still moving; try again")
            time.sleep(0.05)
        point = self.session.record(self.commanded())
        self._play_message = f"Recorded point {point['n']}"
        return point

    def session_play(self, name: str | None = None) -> None:
        """Go to each recorded pose in turn, using the arm's own interpolation."""
        with self._session_lock:
            if self._playing:
                raise RobotArmError("already playing")
            with self._petri_lock:
                if self._petri_active:
                    raise RobotArmError("the Petri path is running")
            if not self.arm.ready:
                raise RobotArmError("press 'Snap to home (startup)' first")
            files = self.session.files()
            name = name or (self.session.file.name if self.session.file else (files[0] if files else None))
            if name is None:
                raise RobotArmError("no recorded session to play")
            points = self.session.load(name)
            if not points:
                raise RobotArmError(f"{name} has no recorded points")
            for p in points:
                for index, deg in enumerate(p["angles"]):
                    low, high = LIMITS_DEG[index]
                    if not low <= deg <= high:
                        raise ValueError(f"{name} point {p['n']}: joint {index} angle {deg:.1f} outside {low:g}..{high:g}")
            has_events = any(p.get("event") for p in points)
            bridge_connected = time.monotonic() - self._petri_bridge_seen <= 3.0
            if has_events and self.simulation_only and not bridge_connected:
                raise RobotArmError("Gazebo sync bridge is not connected; launch gazebo_stage2_gui_sync first")
            with self._lock:
                if self._busy or self._pending:
                    raise RobotArmError("wait for the current arm command to finish")
            self._play_event_enabled = has_events and bridge_connected
            self._playing = True
            self._play_stop.clear()
            self._play_index = 0
            self._play_points = points
            self._play_message = f"Playing {name} ({len(points)} point(s))"
            threading.Thread(target=self._run_playback, args=(name, points), daemon=True).start()

    def session_stop(self) -> None:
        """Stop after the move in progress finishes."""
        self._play_stop.set()
        self.resume()  # a hold would otherwise keep that move from finishing

    def hold_or_record(self, seconds: float) -> None:
        """While recording, save a hold after the last recorded pose; otherwise freeze the arm now."""
        if self.session.active and not self._playing:
            point = self.session.set_hold(seconds)
            self._play_message = f"Hold of {seconds:g} s saved after point {point['n']}"
        else:
            self.hold(seconds)

    def hold(self, seconds: float) -> None:
        """Freeze the arm where it is for `seconds`, then let the move carry on."""
        if self.pause_clock is None:
            raise RobotArmError("hold is not available")
        if not 0 < seconds <= 600:
            raise ValueError("hold time must be more than 0 and at most 600 seconds")
        with self._hold_lock:
            if self._hold_timer is not None:
                self._hold_timer.cancel()
            self.pause_clock.pause()
            self._hold_until = time.monotonic() + seconds
            self._hold_timer = threading.Timer(seconds, self.resume)
            self._hold_timer.daemon = True
            self._hold_timer.start()

    def resume(self) -> None:
        with self._hold_lock:
            if self._hold_timer is not None:
                self._hold_timer.cancel()
                self._hold_timer = None
            if self.pause_clock is not None:
                self.pause_clock.resume()
            self._hold_until = 0.0

    def hold_left(self) -> float:
        return max(0.0, self._hold_until - time.monotonic()) if self._hold_until else 0.0

    def _run_playback(self, name: str, points: list[dict]) -> None:
        try:
            for i, p in enumerate(points, start=1):
                if self._play_stop.is_set():
                    self._play_message = f"Stopped at point {i - 1} of {len(points)}"
                    break
                self._play_index = i
                self._play_message = f"Moving to point {p['n']} of {len(points)}"
                target = list(p["angles"])
                with self._lock:
                    self._check_floor(target)
                    self._pending.clear()
                    if self.grip_interp or abs(target[5] - self.commanded()[5]) < 1e-9:
                        self.arm.move_all(target)
                    else:
                        self.arm.move_all(target, snap=(5,))
                if p.get("hold_s", 0) > 0 and not self._play_stop.is_set():
                    self._play_message = f"Holding at point {p['n']} for {p['hold_s']:g} seconds"
                    self._hold_until = time.monotonic() + p["hold_s"]
                    # "Resume now" (or Stop) clears _hold_until / sets the stop flag and ends the wait early
                    while (
                        self._hold_until
                        and time.monotonic() < self._hold_until
                        and not self._play_stop.is_set()
                    ):
                        time.sleep(0.05)
                    if self.pause_clock is None or not self.pause_clock.paused:
                        self._hold_until = 0.0
                event_kind = p.get("event")
                if event_kind and self._play_event_enabled:
                    with self._petri_lock:
                        self._petri_event_seq += 1
                        event_seq = self._petri_event_seq
                        self._petri_event_kind = event_kind
                        self._petri_event_ack = 0
                    self._play_message = f"Waiting for Gazebo to {event_kind} dish at point {i}"
                    deadline = time.monotonic() + 15.0
                    while time.monotonic() < deadline:
                        with self._petri_lock:
                            if self._petri_event_ack == event_seq:
                                self._petri_event_kind = None
                                break
                        if self._play_stop.wait(0.05):
                            self._play_message = f"Stopped at point {i} of {len(points)}"
                            break
                    else:
                        raise RobotArmError(f"Gazebo did not acknowledge {event_kind}; check Gazebo bridge and joint state")
                    if self._play_stop.is_set():
                        break
            else:
                self._play_message = f"Played {name}: {len(points)} point(s) done"
        except Exception as exc:  # keep the GUI alive, report why playback stopped
            logger.exception("session playback stopped")
            self._play_message = f"Stopped at point {self._play_index} of {len(points)}: {exc}"
        finally:
            with self._petri_lock:
                self._petri_event_kind = None
            self._playing = False
            self._play_event_enabled = False

    def startup(self) -> None:
        if self._playing:
            raise RobotArmError("startup is locked while a recorded session is playing")
        with self._petri_lock:
            if self._petri_active:
                raise RobotArmError("startup is locked while the Petri path is running")
        with self._lock:
            self.arm.startup(confirmed=True)

    def move(self, angles: dict[int, float]) -> None:
        if self._playing:
            raise RobotArmError("manual joint moves are locked while a recorded session is playing")
        with self._petri_lock:
            if self._petri_active:
                raise RobotArmError("manual joint moves are locked while the Petri path is running")
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
            angles = dict(angles)
            if self.level_enabled and any(i in angles for i in (1, 2, 3)):
                target[3] = angles[3] = self.ik.level_j3(target, 0.0)  # keep the tool horizontal
            self._check_floor(target)
            self._pending.update(angles)
        self._wake.set()

    def move_cyl(self, r: float, theta: float, h: float, pitch: float, extra: dict[int, float] | None = None) -> None:
        """Move J0..J3 so the TCP is at (r mm, theta deg, h mm) with the given tool pitch."""
        if not self.arm.ready:
            raise RobotArmError("press 'Snap to home (startup)' first")
        if self.level_enabled:
            pitch = 0.0
        with self._cyl_lock:
            with self._lock:
                base = self.commanded()
                for index, deg in self._pending.items():
                    base[index] = deg
            servo = self.ik.solve_cyl(base, r, theta, h, pitch)
            angles = {i: servo[i] for i in range(4)}
            angles.update(extra or {})
            self.move(angles)

    def set_level(self, enabled: bool, roll_deg: float | None = None) -> None:
        if roll_deg is not None:
            low, high = LIMITS_DEG[4]
            if not low <= roll_deg <= high:
                raise ValueError(f"J4 roll {roll_deg:g} outside {low:g}..{high:g}")
            self.level_roll_deg = float(roll_deg)
        if not enabled:
            self.level_enabled = False
            return
        if not self.arm.ready:
            raise RobotArmError("press 'Snap to home (startup)' first")
        cyl = self.ik.cylindrical(self.commanded())
        if cyl is None:
            raise ValueError("IK unavailable; cannot keep level")
        self.level_enabled = True
        try:
            self.move_cyl(cyl[0], cyl[1], cyl[2], 0.0, {4: self.level_roll_deg})
        except Exception:
            self.level_enabled = False
            raise

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
        if self._playing:
            raise RobotArmError("home is locked while a recorded session is playing")
        with self._petri_lock:
            if self._petri_active:
                raise RobotArmError("home is locked while the Petri path is running")
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
                    if self.grip_interp or 5 not in targets:
                        self.arm.move_all(current)
                    else:
                        self.arm.move_all(current, snap=(5,))
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
            elif self.path == "/api/petri/status":
                self._json(service.petri_status())
            elif self.path == "/api/session/status":
                self._json(service.session_status())
            elif self.path == "/api/petri/bridge":
                service.petri_bridge_heartbeat()
                self._json({"ok": True})
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
            elif self.path.startswith("/webcam.jpg"):
                wc = service.webcam
                if wc is None:
                    return self._json({"error": "webcam disabled"}, 404)
                try:
                    last = int(self._query("seq", "-1"))
                except ValueError:
                    last = -1
                jpeg, seq = wc.wait_frame(last, timeout=1.0)
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
            elif self.path.startswith("/webcam_result.jpg"):
                jpeg = service.webcam.result_jpeg() if service.webcam else None
                if jpeg is None:
                    return self._json({"error": "no result yet"}, 404)
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
                if self.path == "/api/petri/run":
                    service.start_petri_transfer(float(body.get("speed", 1.5)))
                elif self.path == "/api/petri/bridge":
                    service.petri_bridge_heartbeat()
                elif self.path == "/api/petri/event_ack":
                    service.petri_event_ack(int(body["event_seq"]))
                elif self.path == "/api/session":
                    action = body.get("action")
                    if action == "start":
                        service.session_start()
                    elif action == "end":
                        service.session_end()
                    elif action == "record":
                        service.session_record()
                    elif action == "play":
                        service.session_play(body.get("file") or None)
                    elif action == "stop":
                        service.session_stop()
                    elif action == "hold":
                        service.hold_or_record(float(body["seconds"]))
                    elif action == "resume":
                        service.resume()
                    else:
                        raise ValueError(f"unknown session action: {action}")
                    result = {"ok": True, "session": service.session_status()}
                elif self.path == "/api/startup":
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
                elif self.path == "/api/cyl/move":
                    service.move_cyl(
                        float(body["r"]), float(body["theta"]), float(body["h"]), float(body.get("pitch", 0.0))
                    )
                elif self.path == "/api/level":
                    service.set_level(
                        bool(body.get("enabled")),
                        float(body["roll_deg"]) if body.get("roll_deg") is not None else None,
                    )
                elif self.path.startswith("/api/webcam/"):
                    wc = service.webcam
                    if wc is None:
                        raise RobotArmError("webcam disabled")
                    if self.path == "/api/webcam/capture":
                        result = {"ok": True, "result": wc.capture()}
                    elif self.path == "/api/webcam/options":
                        if "rotate_step" in body:
                            wc.set_rotate((wc.rotate + int(body["rotate_step"])) % 360)
                        if "live_dish" in body:
                            wc.live_dish = bool(body["live_dish"])
                        if body.get("reset_peak"):
                            wc.reset_peak()
                    elif self.path == "/api/webcam/detect":
                        # tuning: set detector params / manual dish, then re-run on the captured frame
                        if "params" in body:
                            wc.params = {str(k): float(v) for k, v in body["params"].items()}
                        if "manual" in body:
                            m = body["manual"]
                            wc.manual_dish = None if m is None else [float(m["x"]), float(m["y"]), float(m["r"])]
                        result = {"ok": True, "result": wc.redetect()}
                    else:
                        return self._json({"error": "not found"}, 404)
                elif self.path == "/api/floor":
                    if "floor_mm" in body:
                        service.floor_mm = float(body["floor_mm"])
                    if "enabled" in body:
                        service.floor_enabled = bool(body["enabled"])
                elif self.path == "/api/led":
                    if service.led is None:
                        raise RobotArmError("LED ring disabled")
                    if body.get("white"):
                        service.led.white()
                    service.led.set(body.get("on"), body.get("color"), body.get("brightness"))
                    result = {"ok": True, "led": service.led.state()}
                elif self.path == "/api/gripper_interp":
                    service.grip_interp = bool(body.get("enabled"))
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
    parser.add_argument("--no-qr", action="store_true", help="rotate only, no QR detection")
    parser.add_argument("--no-webcam", action="store_true", help="disable the USB webcam tab source")
    parser.add_argument("--no-led", action="store_true", help="disable the 16-LED WS2812 ring (SPI, GPIO10)")
    parser.add_argument("--led-count", type=int, default=16)
    parser.add_argument("--webcam-device", default=None, help="e.g. /dev/video0 (default: auto)")
    parser.add_argument("--webcam-rotate", type=int, default=0, choices=[0, 90, 180, 270], help="clockwise degrees")
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
        for name, cam, rotate in (("esp", esp, args.camera_rotate),):
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
    pause_clock = PausableClock()  # lets the GUI freeze a move in progress (Hold button)
    service = ArmService(
        ArmController(config, driver, clock=pause_clock),
        IKBridge(),
        saved_home=saved is not None,
        home_degrees=saved,
        simulation_only=args.simulation_only,
        cameras=cameras,
    )
    service.pause_clock = pause_clock
    if not args.no_webcam:
        from robot_arm.vision.webcam import WebCam

        service.webcam = WebCam(args.webcam_device, args.webcam_rotate)
        service.webcam.start()
    if not args.no_led:
        from robot_arm.hardware.led_ring import LedRing

        service.led = LedRing(args.led_count)
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
        if service.led is not None:
            service.led.close()
        driver.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
