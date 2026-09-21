#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手机遥控小车（板子当服务器，手机浏览器直接开网页）—— 零 App、零额外依赖

原理：本机起一个 HTTP 服务，网页里的虚拟摇杆把速度发回来，本节点转成 /cmd_vel。
      地图画面用 /map 渲染成 MJPEG 推给浏览器，手机上能边遥控边看建图效果。
      页面右上角还有「保存地图」按钮：一键把当前 SLAM 图存到
      ~/RobotCode/04_map/（文件名自动带时间戳，含 .pgm/.yaml + .posegraph/.data）。
      地图上**点一下**即设置导航目标点（N4）：画面像素 → 世界坐标 → 发 /goal_pose。
      「回原点」按钮：一键把目标设为地图原点 (0,0)（= 建图起点，到点后车头朝建图开局方向）。
      地图支持**缩放/平移**（捏合缩放、单指拖动、双击复位）——缩放是**服务端重渲染**，
      所以放大后依然清晰，点选坐标也自动跟着缩放走。

只依赖：Python 标准库（http.server）+ rclpy + OpenCV（本机已有）。

订阅 /map（transient_local）、/tf（画车位置）、/odom（显示实际速度）
发布 /cmd_vel、/goal_pose

用法：
    python3 phone_teleop.py                 # 端口 8080，视野 8 米
    python3 phone_teleop.py -p 9000 -r 12   # 换端口 / 初始视野
    python3 phone_teleop.py -s /tmp/maps    # 换地图保存目录（默认 ~/RobotCode/04_map）
    python3 phone_teleop.py -t /test_cmd_vel  # 换速度话题（自测用，不动真车）
    python3 phone_teleop.py -g /goal_pose     # 换目标话题（默认 /goal_pose）
手机：与小车连**同一个 WiFi**，浏览器打开  http://<小车IP>:8080/
      （本机 IP 用别名 myip 查看；启动时终端也会打印）
桌面：方向键（↑↓ 前后，←→ 转向，空格 停）；地图上滚轮/按钮也能缩放

页面手势（地图区域）：
    点一下      → 设导航目标（发 /goal_pose；需另开终端跑 nav --listen）
    拖动        → 平移视野（相对车心偏移，车仍会跟着走）
    捏合/双指   → 缩放（1~20 m 视野）；桌面用右下角 ＋ － ⟳ 按钮
    双击        → 复位（回车上、初始视野）
    HUD 左上角  → 「≡」可折叠/展开，折叠后不遮挡地图

安全（三层）：
    1. 手指离开摇杆 / 页面断开 → 网页立刻发零速度
    2. 本节点看门狗：0.6 s 没收到新指令 → 自动发零速度并停车
    3. 底盘节点自带看门狗：0.5 s 没收到 /cmd_vel → 自动停车
    * 速度上限服务端强制裁剪（-v/-w 可调）；**不发横移速度 vy**（麦轮横移时里程计不可信）
    * 「保存地图」会先自动停车再存（存图时车不动，地图才是"那一刻"的）
    * **同一时刻只能有一个"司机"**：摇杆 / 导航（nav）别同时用；用点选目标时请手离开摇杆
    * 服务只监听局域网、无鉴权——用完记得 Ctrl-C 关掉
"""
import argparse
import json
import math
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

try:
    import cv2
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
except ImportError as e:
    raise SystemExit(
        '导入模块失败：%s\n提示：先 source 环境（或用 ~/.bash_aliases 里的别名 phone）\n'
        '  source /opt/ros2-foxy/install/setup.bash\n'
        '  source ~/RobotCode/ros2_ws/install/setup.bash' % e)

from map_viewer import MapViewer      # 复用地图订阅 + 渲染（同目录）
from save_map import DEFAULT_SAVE_DIR, save_pgm_yaml, serialize_posegraph

CMD_TIMEOUT = 0.6                     # 看门狗：多久没收到指令就发零速度
STREAM_FPS = 8                        # 推给手机的帧率
RANGE_MIN, RANGE_MAX = 1.0, 20.0      # 缩放范围（米）
PAN_MAX = 10.0                        # 视窗中心相对车的最大偏移（米）

PAGE = """<!DOCTYPE html>
<html lang="zh"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>小车遥控</title>
<style>
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  html,body { margin:0; height:100%; background:#111; color:#eee;
              font-family:-apple-system,"Noto Sans CJK SC",sans-serif; overflow:hidden; }
  #wrap { display:flex; flex-direction:column; height:100%; }
  #mapbox { position:relative; flex:1 1 auto; min-height:30%; overflow:hidden;
            touch-action:none; }                 /* 自己处理手势，禁掉浏览器缩放 */
  #map { width:100%; height:100%; object-fit:contain; display:block; cursor:crosshair;
         -webkit-user-drag:none; user-select:none; }
  #hud { position:absolute; left:8px; top:8px; font-size:12px; line-height:1.5;
         background:rgba(0,0,0,.45); padding:6px 8px; border-radius:6px; }
  #hud b { color:#7fd1ff; font-weight:600; }
  #hud.min .line { display:none; }               /* 折叠后只剩连接状态 */
  #hudT { margin-left:6px; padding:0 6px; font-size:12px; color:#ddd;
          background:#333; border:0; border-radius:4px; }
  #hudT:active { background:#555; }
  .dot { display:inline-block; width:8px; height:8px; border-radius:50%; background:#666; }
  .ok { background:#3fbf5f; } .bad { background:#e33; }
  #btns { position:absolute; right:8px; top:8px; display:flex; flex-direction:column; gap:8px; }
  #save { padding:10px 14px; font-size:15px; font-weight:700; color:#fff;
          background:#2d6cdf; border:0; border-radius:10px; }
  #save:active { background:#3f86f5; }
  #save[disabled] { background:#555; }
  #goalclr { padding:8px 12px; font-size:13px; font-weight:600; color:#eee;
            background:#444; border:0; border-radius:10px; }
  #goalclr:active { background:#666; }
  #home { padding:10px 14px; font-size:15px; font-weight:700; color:#fff;
          background:#c47f17; border:0; border-radius:10px; }
  #home:active { background:#e09a2b; }
  #home[disabled] { background:#555; }
  #zbtns { position:absolute; right:8px; bottom:8px; display:flex; flex-direction:column; gap:6px; }
  #zbtns button { width:44px; height:44px; font-size:20px; font-weight:700; color:#eee;
                  background:rgba(0,0,0,.55); border:1px solid #555; border-radius:10px; }
  #zbtns button:active { background:rgba(255,255,255,.25); }
  #toast { position:absolute; left:50%; transform:translateX(-50%); bottom:12px;
           max-width:92%; padding:8px 12px; font-size:13px; line-height:1.4;
           background:rgba(0,0,0,.78); border-radius:8px; display:none; }
  #padbox { flex:0 0 auto; padding:10px 12px 14px; background:#181818; }
  #pad { position:relative; width:100%; height:34vh; min-height:180px; max-height:300px;
         background:#222; border:1px solid #333; border-radius:14px; touch-action:none; }
  #knob { position:absolute; width:84px; height:84px; margin:-42px 0 0 -42px;
          left:50%; top:50%; border-radius:50%; background:#3a6ea5;
          border:2px solid #7fb2e5; transition:background .1s; }
  #knob.on { background:#4f8fd0; }
  #row { display:flex; gap:10px; align-items:center; margin-top:10px; }
  #stop { flex:0 0 34%; height:64px; font-size:22px; font-weight:700; color:#fff;
          background:#c0392b; border:0; border-radius:12px; }
  #stop:active { background:#e74c3c; }
  #sliders { flex:1 1 auto; font-size:12px; color:#bbb; }
  #sliders input { width:100%; }
  #hint { font-size:11px; color:#777; margin-top:6px; text-align:center; }
</style></head>
<body><div id="wrap">
  <div id="mapbox">
    <img id="map" src="/stream.mjpg" alt="map" draggable="false">
    <div id="hud">
      <div><span id="dot" class="dot"></span> <span id="conn">连接中…</span>
        <button id="hudT" title="折叠/展开">≡</button></div>
      <div class="line">位姿 <b id="pose">-</b></div>
      <div class="line">速度 <b id="vel">-</b></div>
      <div class="line">目标 <b id="goal">-</b></div>
      <div class="line">视野 <b id="zoom">-</b></div>
    </div>
    <div id="btns">
      <button id="save">保存地图</button>
      <button id="home">回原点</button>
      <button id="goalclr">清除目标·停车</button>
    </div>
    <div id="zbtns">
      <button id="zin" title="放大">＋</button>
      <button id="zout" title="缩小">－</button>
      <button id="zrst" title="复位（回车上/初始视野）">⟳</button>
    </div>
    <div id="toast"></div>
  </div>
  <div id="padbox">
    <div id="pad"><div id="knob"></div></div>
    <div id="row">
      <button id="stop">停 止</button>
      <div id="sliders">
        前进上限 <span id="vl">0.15</span> m/s
        <input id="vs" type="range" min="5" max="30" value="15">
        转向上限 <span id="wl">0.60</span> rad/s
        <input id="ws" type="range" min="20" max="150" value="60">
      </div>
    </div>
    <div id="hint">圆盘：上=前进 左右=转向 ｜ 地图：点一下=设目标，拖=平移，捏合=缩放，双击=复位 ｜ 桌面可用方向键</div>
  </div>
</div>
<script>
const pad=document.getElementById('pad'), knob=document.getElementById('knob');
const stopBtn=document.getElementById('stop');
const vs=document.getElementById('vs'), ws=document.getElementById('ws');
const saveBtn=document.getElementById('save'), toast=document.getElementById('toast');
const mbox=document.getElementById('mapbox'), mimg=document.getElementById('map');
const goalClr=document.getElementById('goalclr'), hud=document.getElementById('hud');
const homeBtn=document.getElementById('home');
let vmax=0.15, wmax=0.60, timer=null, vx=0, wz=0, active=false;

function cfg(){ vmax=vs.value/100; wmax=ws.value/100;
  document.getElementById('vl').textContent=vmax.toFixed(2);
  document.getElementById('wl').textContent=wmax.toFixed(2); }
vs.oninput=cfg; ws.oninput=cfg; cfg();

function send(){ fetch('/cmd?vx='+vx.toFixed(3)+'&wz='+wz.toFixed(3)).catch(()=>{}); }
function loop(){ if(!timer) timer=setInterval(send,100); }
function halt(){ vx=0; wz=0; send(); if(timer){clearInterval(timer); timer=null;} }
function setKnob(dx,dy,use){ const r=pad.clientWidth/2-52, R=Math.min(r,pad.clientHeight/2-52);
  const x=Math.max(-R,Math.min(R,dx)), y=Math.max(-R,Math.min(R,dy));
  knob.style.transform='translate('+x+'px,'+y+'px)';
  vx=(-y/R)*vmax; wz=(-x/R)*wmax;
  if(use){ loop(); send(); } knob.classList.toggle('on', !!use); }

function onMove(ev){ const r=pad.getBoundingClientRect();
  setKnob(ev.clientX-(r.left+r.width/2), ev.clientY-(r.top+r.height/2), true); }
pad.addEventListener('pointerdown', e=>{ active=true; pad.setPointerCapture(e.pointerId); onMove(e); });
pad.addEventListener('pointermove', e=>{ if(active) onMove(e); });
['pointerup','pointercancel'].forEach(t=>pad.addEventListener(t, e=>{
  active=false; setKnob(0,0,false); halt(); }));
stopBtn.onclick=()=>{ active=false; setKnob(0,0,false); halt(); };

function showToast(msg, ms){ toast.textContent=msg; toast.style.display='block';
  clearTimeout(showToast.t); showToast.t=setTimeout(()=>toast.style.display='none', ms||5000); }

// ---------------- 地图视图：缩放（服务端重渲染）/ 平移 / 复位 ----------------
let view={range:null, dx:0, dy:0};      // range=null 表示"还没从 /state 拿到"
const RANGE_MIN=1.0, RANGE_MAX=20.0, PAN_MAX=10.0;
let vTimer=null, vLast=0;
function viewReady(){ if(view.range===null) view.range=8.0; return view; }
function pushView(){
  const v=viewReady();
  fetch('/view?range='+v.range.toFixed(2)+'&dx='+v.dx.toFixed(2)+'&dy='+v.dy.toFixed(2))
    .then(r=>r.json()).then(s=>{ if(s && s.range){ view.range=s.range; view.dx=s.dx; view.dy=s.dy; }})
    .catch(()=>{});
}
function queueView(){                       // 拖动/捏合中节流，松手后必发最后一次
  const dt=Date.now()-vLast;
  clearTimeout(vTimer);
  if(dt>120){ vLast=Date.now(); pushView(); }
  else vTimer=setTimeout(()=>{ vLast=Date.now(); pushView(); }, 120-dt);
}
function clampView(){
  const v=viewReady();
  v.range=Math.max(RANGE_MIN, Math.min(RANGE_MAX, v.range));
  v.dx=Math.max(-PAN_MAX, Math.min(PAN_MAX, v.dx));
  v.dy=Math.max(-PAN_MAX, Math.min(PAN_MAX, v.dy));
}
function resetView(){ view.range=null; view.dx=0; view.dy=0;
  fetch('/view?reset=1').then(r=>r.json()).then(s=>{
    view.range=s.range; view.dx=s.dx; view.dy=s.dy;
    showToast('视图已复位（回车上）', 2000); }).catch(()=>{}); }

// 屏幕像素 → 米：图片按 object-fit:contain 显示，先算缩放，再用 view.range/边长
function mapGeom(){
  const r=mimg.getBoundingClientRect(), nw=mimg.naturalWidth||480, nh=mimg.naturalHeight||480;
  const sc=Math.min(r.width/nw, r.height/nh) || 1;
  return {r:r, nw:nw, nh:nh, sc:sc, ox:r.left+(r.width-nw*sc)/2, oy:r.top+(r.height-nh*sc)/2};
}
function panBy(dxScreen, dyScreen){
  const g=mapGeom(), v=viewReady();
  const mPerScreen=(v.range/g.nw)/g.sc;
  v.dx -= dxScreen*mPerScreen;
  v.dy += dyScreen*mPerScreen;            // 屏幕 y 向下、世界 y 向上
  clampView(); queueView();
}

const gp=new Map();                       // 正在按下的指针
let startPt=null, moved=false, twoFinger=false, pinch0=null, lastTap=0;

function setGoalAt(cx, cy){
  const now=Date.now();
  if(now-lastTap < 300){ lastTap=0; resetView(); return; }   // 双击 = 复位
  lastTap=now;
  const g=mapGeom();
  const u=(cx-g.ox)/g.sc, v=(cy-g.oy)/g.sc;                  // 画面像素
  if(u<0||v<0||u>g.nw||v>g.nh){ showToast('点到画面外的黑边了，请点地图上', 3000); return; }
  active=false; setKnob(0,0,false); halt();                  // 先停车，避免和导航抢控制
  fetch('/goal?u='+u.toFixed(1)+'&v='+v.toFixed(1)).then(r=>r.json()).then(s=>{
    showToast((s.ok?'✓ ':'✗ ')+(s.msg||''), s.ok?6000:8000);
  }).catch(()=>showToast('✗ 设置目标失败：连不上服务', 5000));
}

mbox.addEventListener('pointerdown', e=>{
  gp.set(e.pointerId, {x:e.clientX, y:e.clientY});
  try{ mbox.setPointerCapture(e.pointerId); }catch(_){}
  if(gp.size===1){ startPt={x:e.clientX, y:e.clientY}; moved=false; twoFinger=false; }
  if(gp.size===2){
    twoFinger=true;
    const p=[...gp.values()];
    pinch0={dist:Math.max(1, Math.hypot(p[0].x-p[1].x, p[0].y-p[1].y)), range:viewReady().range};
  }
});
mbox.addEventListener('pointermove', e=>{
  if(!gp.has(e.pointerId)) return;
  gp.set(e.pointerId, {x:e.clientX, y:e.clientY});
  if(gp.size>=2 && pinch0){
    const p=[...gp.values()];
    const d=Math.hypot(p[0].x-p[1].x, p[0].y-p[1].y);
    if(d>2){ const v=viewReady(); v.range=pinch0.range*pinch0.dist/d; clampView(); queueView(); }
    return;
  }
  if(gp.size===1 && startPt){
    const dx=e.clientX-startPt.x, dy=e.clientY-startPt.y;
    if(Math.hypot(dx,dy) > 8){ moved=true; panBy(dx,dy); startPt={x:e.clientX, y:e.clientY}; }
  }
});
function endPtr(e){
  if(!gp.has(e.pointerId)) return;
  const wasTap=(gp.size===1 && !moved && !twoFinger && startPt);
  gp.delete(e.pointerId);
  if(gp.size<2) pinch0=null;
  if(wasTap) setGoalAt(e.clientX, e.clientY);
  if(gp.size===0){ startPt=null; clearTimeout(vTimer); vLast=Date.now(); pushView(); }
}
mbox.addEventListener('pointerup', endPtr);
mbox.addEventListener('pointercancel', endPtr);

document.getElementById('zin').onclick=()=>{ const v=viewReady(); v.range/=1.5; clampView(); clearTimeout(vTimer); vLast=Date.now(); pushView(); };
document.getElementById('zout').onclick=()=>{ const v=viewReady(); v.range*=1.5; clampView(); clearTimeout(vTimer); vLast=Date.now(); pushView(); };
document.getElementById('zrst').onclick=resetView;
document.getElementById('hudT').onclick=()=>hud.classList.toggle('min');

// 保存地图：先停车，再让板子把 .pgm/.yaml + .posegraph/.data 存到 04_map（名字带时间戳）
saveBtn.onclick=()=>{
  active=false; setKnob(0,0,false); halt();
  saveBtn.disabled=true; saveBtn.textContent='保存中…';
  fetch('/savemap').then(r=>r.json()).then(s=>{
    showToast((s.ok?'✓ 已保存：':'✗ 保存失败：')+(s.msg||''), s.ok?7000:9000);
  }).catch(()=>showToast('✗ 保存失败：连不上服务（板子上的 phone 还在跑吗？）',8000))
    .finally(()=>{ saveBtn.disabled=false; saveBtn.textContent='保存地图'; });
};
goalClr.onclick=()=>{
  fetch('/goal/clear').then(r=>r.json()).then(s=>showToast(s.msg||'已清除', 3000)).catch(()=>{});
};
// 一键回原点：目标 = 地图原点 (0,0) = 建图起点（需另开终端跑 nav --listen 才会动）
homeBtn.onclick=()=>{
  active=false; setKnob(0,0,false); halt();          // 先停车，避免和导航抢控制
  homeBtn.disabled=true; homeBtn.textContent='回原点…';
  fetch('/goal/home').then(r=>r.json()).then(s=>{
    showToast((s.ok?'✓ ':'✗ ')+(s.msg||''), s.ok?7000:8000);
  }).catch(()=>showToast('✗ 回原点失败：连不上服务（phone 还在跑吗？）',8000))
    .finally(()=>{ homeBtn.disabled=false; homeBtn.textContent='回原点'; });
};

// 桌面调试用方向键
const keys={};
document.addEventListener('keydown', e=>{ keys[e.key]=true; applyKeys(); if(e.key===' ') halt(); });
document.addEventListener('keyup',   e=>{ keys[e.key]=false; applyKeys(); });
function applyKeys(){ if(active) return;
  const dv=(keys.ArrowUp?1:0)-(keys.ArrowDown?1:0), dw=(keys.ArrowLeft?1:0)-(keys.ArrowRight?1:0);
  if(dv||dw){ vx=dv*vmax*0.8; wz=dw*wmax*0.8; loop(); send(); }
  else { vx=0; wz=0; if(timer){clearInterval(timer); timer=null;} send(); } }

setInterval(()=>{ fetch('/state').then(r=>r.json()).then(s=>{
  const d=document.getElementById('dot'), c=document.getElementById('conn');
  d.className='dot '+(s.cmd_age<1?'ok':'bad'); c.textContent=s.cmd_age<1?'已连接':'看门狗待机';
  document.getElementById('pose').textContent=s.pose||'-';
  document.getElementById('vel').textContent=s.vel||'-';
  document.getElementById('goal').textContent=s.goal||'-';
  if(s.view){ document.getElementById('zoom').textContent=s.view;
              if(view.range===null){ view.range=s.view_range||8.0; view.dx=0; view.dy=0; } }
}).catch(()=>{ document.getElementById('dot').className='dot bad';
               document.getElementById('conn').textContent='断开'; }); }, 400);
</script></body></html>
"""


class PhoneTeleop(MapViewer):
    """在地图查看器基础上：发 /cmd_vel、设/清导航目标（/goal_pose）、保存地图、网页服务状态"""

    def __init__(self, view_range, vmax, wmax, topic='/cmd_vel',
                 save_dir=DEFAULT_SAVE_DIR, goal_topic='/goal_pose'):
        super().__init__(view_range)
        self.topic = topic
        self.vmax = vmax
        self.wmax = wmax
        self.save_dir = save_dir      # 保存地图的目录（默认 ~/RobotCode/04_map）
        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0)         # 最近一次下发的 (vx, wz)
        self.last_cmd_t = 0.0         # 最近一次收到网页指令的时刻
        self.n_cmd = 0
        self.last_save = None         # 最近一次保存的文件名前缀
        self.odom_vel = None          # (vx, wz) 底盘回报的实际速度
        self.goal = None              # 最近一次设置的目标 (x, y, 时刻)，用于画面标注
        self.n_goal = 0
        self.view_range_cur = float(view_range)   # 当前视野（网页缩放会改它）
        self.view_off = (0.0, 0.0)                # 视窗中心相对车的偏移（米）
        self.pub = self.create_publisher(Twist, self.topic, 10)
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, msg):
        self.odom_vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def set_cmd(self, vx, wz):
        """裁剪并发布速度；返回实际下发值"""
        vx = max(-self.vmax, min(self.vmax, vx))
        wz = max(-self.wmax, min(self.wmax, wz))
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        with self.lock:
            self.pub.publish(t)
            self.cmd = (vx, wz)
            self.last_cmd_t = time.time()
            self.n_cmd += 1
        return vx, wz

    # ---------- 地图视图（缩放/平移）----------
    def set_view(self, range_m=None, dx=None, dy=None, reset=False):
        """网页缩放/平移：range 1~20 m、偏移 ±10 m；reset=True 复位（回车上 + 初始视野）

        缩放是**服务端重渲染**（不是放大图片）：放大后依然清晰，点选坐标也自动跟着缩放走。
        """
        with self.lock:
            if reset:
                self.view_range_cur = float(self.view_range)
                self.view_off = (0.0, 0.0)
            if range_m is not None and range_m == range_m:
                self.view_range_cur = max(RANGE_MIN, min(RANGE_MAX, float(range_m)))
            ox, oy = self.view_off
            if dx is not None and dx == dx:
                ox = max(-PAN_MAX, min(PAN_MAX, float(dx)))
            if dy is not None and dy == dy:
                oy = max(-PAN_MAX, min(PAN_MAX, float(dy)))
            self.view_off = (ox, oy)
            cur = (self.view_range_cur, ox, oy)
        return {'ok': True, 'range': round(cur[0], 3),
                'dx': round(cur[1], 3), 'dy': round(cur[2], 3)}

    def view_offset(self):
        """当前平移偏移 (dx, dy)：render 会把它叠加到"车位置（无 TF 时地图中心）"上"""
        with self.lock:
            return self.view_off

    # ---------- 目标点（N4：手机网页点选导航目标）----------
    def _cell_of(self, x, y):
        """世界坐标 → 地图格值；'out'=出图，None=还没收到 /map"""
        m = self.map_msg
        if m is None:
            return None
        res = m.info.resolution
        w, h = m.info.width, m.info.height
        gx = int((x - m.info.origin.position.x) / res)
        gy = int((y - m.info.origin.position.y) / res)
        if not (0 <= gx < w and 0 <= gy < h):
            return 'out'
        return m.data[gy * w + gx]

    def set_goal(self, u=None, v=None, x=None, y=None):
        """设置导航目标并发 /goal_pose

        u,v  = 网页画面像素（由板子用渲染时的视窗参数反算世界坐标，缩放/平移后同样准确）
        x,y  = 也可直接给世界坐标（便于命令行/脚本自测）
        返回网页用的 dict：{'ok', 'msg', 'x', 'y', 'sub'}
        """
        if x is None or y is None:
            lv = self.last_view
            if lv is None:
                return {'ok': False, 'msg': '还没有地图画面（/map 或 TF 没到），稍等再点'}
            x0, y1, s, size = lv
            if u is None or v is None or not (0 <= u < size and 0 <= v < size):
                return {'ok': False, 'msg': '点到了画面外'}
            x, y = x0 + u / s, y1 - v / s
        self.set_cmd(0.0, 0.0)                      # 先停一帧，避免与摇杆指令打架
        t = PoseStamped()
        t.header.frame_id = 'map'
        t.header.stamp = self.get_clock().now().to_msg()
        t.pose.position.x, t.pose.position.y = float(x), float(y)
        t.pose.orientation.w = 1.0                  # 朝向留给 goal_nav（可用 --goal x y yaw 再转）
        self.goal_pub.publish(t)
        self.goal = (float(x), float(y), time.time())
        with self.lock:
            self.n_goal += 1
        cell = self._cell_of(x, y)
        sub = self.goal_pub.get_subscription_count()
        if cell is None:
            note = '（还没收到 /map）'
        elif cell == 'out':
            note = '（在地图范围外，规划会失败）'
        elif cell != 0:
            note = '（注意：该点不是空闲区，规划可能失败）'
        else:
            note = ''
        if sub == 0:
            note += '｜⚠ /goal_pose 当前无订阅者：先另开终端跑 nav --listen'
        msg = '目标已设：x=%.2f y=%.2f%s' % (x, y, note)
        self.get_logger().info(msg)
        return {'ok': True, 'msg': msg, 'x': round(float(x), 2), 'y': round(float(y), 2),
                'sub': sub}

    def goto_home(self):
        """一键回原点：目标 = 地图原点 (0, 0)，朝向 0°（= 建图开局时车的位置与车头方向）

        定位模式下"原点"就是**建图那次开局车的实际位置/朝向**（见 08 方案 N1），
        所以到点后车头也会转到 0°（地图 +x），与建图开局一致。
        若定位漂移导致原点对不上实际地面标记，先按 08 方案 N1 用 /initialpose 修正。
        """
        info = self.set_goal(x=0.0, y=0.0)
        if info.get('ok'):
            info['msg'] = '回原点 → ' + info['msg']
        return info

    def clear_goal(self):
        """清除目标，并让导航**就地停车**

        做法：把"车当前位姿"当成一个新目标发出去 —— 导航侧会立刻判定"已到达"、
        回到等待状态（它下一拍就不会再发速度指令了）。这比在手机上按「停 止」可靠：
        那个按钮只是把速度置零，导航进程下一轮照样会继续发指令。
        """
        pose = self.robot_pose()
        stopped = False
        if pose is not None and self.goal_pub.get_subscription_count() > 0:
            t = PoseStamped()
            t.header.frame_id = 'map'
            t.header.stamp = self.get_clock().now().to_msg()
            t.pose.position.x, t.pose.position.y = pose[0], pose[1]
            t.pose.orientation.w = 1.0
            self.goal_pub.publish(t)
            stopped = True
        self.set_cmd(0.0, 0.0)
        self.goal = None
        msg = '目标已清除' + ('（已让导航就地停车、回到等待状态）' if stopped else '')
        self.get_logger().info(msg)
        return {'ok': True, 'msg': msg, 'stopped': stopped}

    def _world_px(self, x, y, size):
        """世界坐标 → 当前画面的像素位置（不在画面内返回 None）"""
        if self.last_view is None:
            return None
        x0, y1, s, sz = self.last_view
        px = int(round((x - x0) * s))
        py = int(round((y1 - y) * s))
        if not (0 <= px < sz and 0 <= py < sz):
            return None
        return px, py

    def goal_px(self, size):
        """目标点 → 画面像素（供推流标注）；不在画面内返回 None"""
        if self.goal is None:
            return None
        return self._world_px(self.goal[0], self.goal[1], size)

    def car_px(self, size):
        """车 → 画面像素（缩放/平移后车不在画面中心，标注连线要从这里出发）"""
        pose = self.robot_pose()
        if pose is None:
            return None
        return self._world_px(pose[0], pose[1], size)

    def save_map(self):
        """保存当前 SLAM 图（PGM/YAML + 建图序列），文件名带时间戳，存到 self.save_dir

        返回给网页的 dict：{'ok': bool, 'msg': str, 'name': str, 'dir': str}
        """
        m = self.map_msg
        if m is None:
            return {'ok': False, 'msg': '还没收到 /map —— SLM 没在运行？'}

        try:
            os.makedirs(self.save_dir, exist_ok=True)
        except OSError as e:
            return {'ok': False, 'msg': '无法创建目录 %s（%s）' % (self.save_dir, e)}

        name = time.strftime('map_%Y%m%d_%H%M%S')
        prefix = os.path.join(self.save_dir, name)
        written, notes = [], []

        try:
            written += save_pgm_yaml(m, prefix)
            notes.append('pgm+yaml(%dx%d)' % (m.info.width, m.info.height))
        except Exception as e:
            notes.append('pgm 失败：%s' % e)

        ok_pg, info = serialize_posegraph(self, prefix, timeout=30.0, spin=False)
        if ok_pg:
            written += [prefix + '.posegraph', prefix + '.data']
            notes.append('posegraph')
        else:
            notes.append('序列失败：%s' % info)

        sizes = ' ｜ '.join('%s %.0fKB' % (os.path.basename(p), os.path.getsize(p) / 1024.0)
                            for p in written if os.path.exists(p))
        self.get_logger().info('保存地图 → %s（%s）' % (prefix, ' + '.join(notes)))
        self.last_save = name
        return {'ok': bool(written), 'msg': '%s ｜ %s' % (name, sizes or '未写出文件'),
                'name': name, 'dir': self.save_dir}

    def watchdog(self):
        """超时未收到网页指令 → 主动发零速度（第 2 层保险）"""
        while rclpy.ok():
            time.sleep(0.1)
            with self.lock:
                stale = self.last_cmd_t > 0 and time.time() - self.last_cmd_t > CMD_TIMEOUT
                moving = abs(self.cmd[0]) > 1e-6 or abs(self.cmd[1]) > 1e-6
            if stale and moving:
                self.set_cmd(0.0, 0.0)
                self.get_logger().warn('网页指令超时 %.1fs，已自动停车' % CMD_TIMEOUT)

    def state(self):
        pose = self.robot_pose()
        with self.lock:
            cmd_age = time.time() - self.last_cmd_t if self.last_cmd_t else 1e9
            vx, wz = self.cmd
            n = self.n_cmd
            rng, ox, oy = self.view_range_cur, self.view_off[0], self.view_off[1]
        m = self.map_msg
        return {
            'pose': ('x=%.2f y=%.2f yaw=%.0f°' % (pose[0], pose[1], math.degrees(pose[2])))
                    if pose else None,
            'vel': ('cmd=%.2f m/s %.2f rad/s ｜ 实际=%.2f m/s %.2f rad/s'
                    % (vx, wz, self.odom_vel[0], self.odom_vel[1])) if self.odom_vel
                   else ('cmd=%.2f m/s %.2f rad/s' % (vx, wz)),
            'cmd_age': round(cmd_age, 2),
            'n_cmd': n,
            'map': ('%dx%d @%.3fm' % (m.info.width, m.info.height, m.info.resolution))
                   if m else None,
            'goal': ('x=%.2f y=%.2f%s' % (self.goal[0], self.goal[1],
                                          '' if self.goal_pub.get_subscription_count()
                                          else ' ⚠无监听')) if self.goal else None,
            'view': ('%.1f m%s' % (rng, ('，偏移 %.1f/%.1f' % (ox, oy))
                                   if abs(ox) > 0.05 or abs(oy) > 0.05 else '')),
            'view_range': round(rng, 3),        # 给网页初始化用（数值）
            'last_save': self.last_save,
        }


class Handler(BaseHTTPRequestHandler):
    node = None          # 由 main() 注入
    frame = b''          # 最近一帧 JPEG（渲染线程更新）
    frame_lock = threading.Lock()
    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):
        pass             # 静音访问日志（1 秒几十条会刷屏）

    def _send(self, code, ctype, body, extra=None):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, info):
        body = json.dumps(info, ensure_ascii=False).encode('utf-8')
        self._send(200, 'application/json; charset=utf-8', body,
                   {'Cache-Control': 'no-store'})

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path in ('/', '/index.html'):
            self._send(200, 'text/html; charset=utf-8', PAGE.encode('utf-8'))
        elif u.path == '/cmd':
            try:
                vx = float(q.get('vx', ['0'])[0])
                wz = float(q.get('wz', ['0'])[0])
                vx, wz = self.node.set_cmd(vx, wz)
            except (TypeError, ValueError):
                self._send(400, 'text/plain', b'bad args')
                return
            self._send(200, 'text/plain', ('%+.2f %+.2f' % (vx, wz)).encode())
        elif u.path == '/stop':
            self.node.set_cmd(0.0, 0.0)
            self._send(200, 'text/plain', b'stopped')
        elif u.path == '/view':
            # 缩放/平移：range=米 &&/|| dx= dy=（米，相对车心）；reset=1 复位
            try:
                info = self.node.set_view(
                    range_m=float(q['range'][0]) if 'range' in q else None,
                    dx=float(q['dx'][0]) if 'dx' in q else None,
                    dy=float(q['dy'][0]) if 'dy' in q else None,
                    reset=('reset' in q))
            except (TypeError, ValueError):
                info = {'ok': False, 'msg': '参数错误（需要 range / dx / dy / reset）'}
            self._send_json(info)
        elif u.path == '/goal':
            # 点选目标：u,v = 画面像素（也支持直接给世界坐标 x,y，便于命令行自测）
            try:
                if 'x' in q and 'y' in q:
                    info = self.node.set_goal(x=float(q['x'][0]), y=float(q['y'][0]))
                else:
                    info = self.node.set_goal(u=float(q.get('u', ['-1'])[0]),
                                              v=float(q.get('v', ['-1'])[0]))
            except (TypeError, ValueError):
                info = {'ok': False, 'msg': '参数错误（需要 u,v 或 x,y）'}
            self._send_json(info)
        elif u.path == '/goal/home':
            # 一键回原点：目标 = 地图原点 (0,0) = 建图起点
            self._send_json(self.node.goto_home())
        elif u.path == '/goal/clear':
            self._send_json(self.node.clear_goal())
        elif u.path == '/savemap':
            # 保存地图（会阻塞几秒：等 serialize_map 服务写完文件）
            try:
                info = self.node.save_map()
            except Exception as e:
                info = {'ok': False, 'msg': '保存异常：%s' % e}
            self._send_json(info)
        elif u.path == '/state':
            self._send_json(self.node.state())
        elif u.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                while True:
                    with Handler.frame_lock:
                        data = Handler.frame
                    if data:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                         b'Content-Length: ' + str(len(data)).encode()
                                         + b'\r\n\r\n' + data + b'\r\n')
                    time.sleep(1.0 / STREAM_FPS)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self._send(404, 'text/plain', b'not found')


def render_loop(node, size):
    """后台线程：把地图渲染成 JPEG（手机端直接从缓存取，避免并发渲染）

    顺带把"已设置的目标点"画在画面上（红圈 + 十字 + 车→目标连线）。
    缩放/平移由 node 的视图状态决定（服务端重渲染，放大不糊）。
    """
    while rclpy.ok():
        try:
            img = node.render(size=size, view_range=node.view_range_cur,
                              offset=node.view_offset())
            o = node._world_px(0.0, 0.0, size)  # 地图原点 = 建图起点（「回原点」的目标）
            if o is not None:
                cv2.circle(img, o, 6, (0, 170, 255), 2)
                cv2.drawMarker(img, o, (0, 170, 255), cv2.MARKER_CROSS, 12, 2)
                cv2.putText(img, 'origin', (o[0] + 9, o[1] + 17),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 170, 255), 1)
            g = node.goal_px(size)
            if g is not None:
                c = node.car_px(size)           # 平移后车不在画面中心 → 从车的位置连线
                if c is not None:
                    cv2.line(img, c, g, (0, 0, 255), 1)
                cv2.circle(img, g, 14, (0, 0, 255), 2)
                cv2.drawMarker(img, g, (0, 0, 255), cv2.MARKER_TILTED_CROSS, 18, 2)
                cv2.putText(img, 'goal', (g[0] + 12, g[1] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                with Handler.frame_lock:
                    Handler.frame = buf.tobytes()
        except Exception as e:
            node.get_logger().error('渲染失败：%s' % e, throttle_duration_sec=5.0)
        time.sleep(1.0 / STREAM_FPS)


def local_ip():
    """取本机在局域网里的 IP（用于打印手机要打开的地址）"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser(description='手机遥控小车（网页版，板子当服务器）')
    ap.add_argument('-p', '--port', type=int, default=8080, help='HTTP 端口（默认 8080）')
    ap.add_argument('-r', '--range', type=float, default=8.0,
                    help='初始视野米数（默认 8；手机上可捏合缩放 1~20）')
    ap.add_argument('--size', type=int, default=480, help='推流画面像素边长（默认 480）')
    ap.add_argument('-v', '--vmax', type=float, default=0.20, help='前进速度上限 m/s（默认 0.20）')
    ap.add_argument('-w', '--wmax', type=float, default=1.00, help='转向速度上限 rad/s（默认 1.00）')
    ap.add_argument('-t', '--topic', default='/cmd_vel',
                    help='速度话题（默认 /cmd_vel；自测时可换成测试话题，不动真车）')
    ap.add_argument('-g', '--goal-topic', default='/goal_pose',
                    help='目标点话题（默认 /goal_pose；自测可换成测试话题）')
    ap.add_argument('-s', '--save-dir', default=DEFAULT_SAVE_DIR,
                    help='保存地图的目录（默认 ~/RobotCode/04_map）')
    args = ap.parse_args()

    rclpy.init()
    node = PhoneTeleop(args.range, args.vmax, args.wmax, args.topic,
                       os.path.expanduser(args.save_dir), args.goal_topic)
    Handler.node = node

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=node.watchdog, daemon=True).start()
    threading.Thread(target=render_loop, args=(node, args.size), daemon=True).start()

    srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    srv.daemon_threads = True
    ip = local_ip()
    print('=' * 56)
    print('手机遥控已启动（手机需与小车的 WiFi 在同一网络）')
    print('  手机浏览器打开： http://%s:%d/' % (ip, args.port))
    print('  本机自测：       http://127.0.0.1:%d/' % args.port)
    print('  速度上限：前进 %.2f m/s ｜ 转向 %.2f rad/s（网页上还能再调小）'
          % (args.vmax, args.wmax))
    print('  地图手势：点一下=设目标 ｜ 拖动=平移 ｜ 捏合=缩放(1~20m) ｜ 双击=复位')
    print('  保存地图：网页右上角「保存地图」按钮 → %s' % node.save_dir)
    print('  点选目标：地图上点一下 → 发 %s（要真的开过去需另开终端跑  nav --listen）'
          % args.goal_topic)
    print('  Ctrl-C 退出（退出会自动发零速度）')
    print('=' * 56)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n收到 Ctrl-C，退出中 ...')
    finally:
        node.set_cmd(0.0, 0.0)
        srv.shutdown()
        srv.server_close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
