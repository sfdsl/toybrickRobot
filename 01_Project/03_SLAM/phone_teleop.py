#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手机遥控小车（板子当服务器，手机浏览器直接开网页）—— 零 App、零额外依赖

原理：本机起一个 HTTP 服务，网页里的虚拟摇杆把速度发回来，本节点转成 /cmd_vel。
      地图画面用 /map 渲染成 MJPEG 推给浏览器，手机上能边遥控边看建图效果。
      页面右上角还有「保存地图」按钮：一键把当前 SLAM 图存到
      ~/RobotCode/04_map/（文件名自动带时间戳，含 .pgm/.yaml + .posegraph/.data）。
      地图上**点一下**即设置导航目标点（N4）：画面像素 → 世界坐标 → 发 /goal_pose。
      「回原点」按钮：一键把目标设为地图原点 (0,0)（= 建图起点，到点后车头朝建图开局方向）。
      **点选/回原点前会本地预检**（与 `nav` 同一份规划代码试算：能否规划、路径多长、离障碍多远）——
      不可达就**不下发目标**，直接告诉你要换点；预检按默认参数，`--no-precheck` 可关。
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
    from nav2_msgs.action import FollowWaypoints
except ImportError as e:
    raise SystemExit(
        '导入模块失败：%s\n提示：先 source 环境（或用 ~/.bash_aliases 里的别名 phone）\n'
        '  source /opt/ros/humble/setup.bash\n'
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
      <button id="wpm">多点</button>
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
const wpmBtn=document.getElementById('wpm');
let vmax=0.15, wmax=0.60, timer=null, vx=0, wz=0, active=false;
// P34 多点导航：wpMode=排队中（点地图入队）；再按「多点」按钮 = 出发
let wpMode=false, wpN=0;

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
  if(wpMode){                                                // 多点模式：入队，不停车不下发
    fetch('/wp/add?u='+u.toFixed(1)+'&v='+v.toFixed(1)).then(r=>r.json()).then(s=>{
      if(s.ok){ wpN=s.n; wpmBtn.textContent='出发('+wpN+')'; }
      showToast((s.ok?'':'✗ ')+(s.msg||''), s.ok?3000:6000);
    }).catch(()=>showToast('✗ 入队失败：连不上服务', 4000));
    return;
  }
  active=false; setKnob(0,0,false); halt();                  // 先停车，避免和导航抢控制
  fetch('/goal?u='+u.toFixed(1)+'&v='+v.toFixed(1)).then(r=>r.json()).then(s=>{
    showToast((s.ok?'✓ ':'✗ ')+(s.msg||''), s.ok?6000:8000);
  }).catch(()=>showToast('✗ 设置目标失败：连不上服务', 5000));
}

mbox.addEventListener('pointerdown', e=>{
  // ⚠ 修复：按钮(#btns/#zbtns/HUD)都在 mapbox 内——不排除的话 setPointerCapture 会把
  //   click 的 target 捕获到 mapbox，按钮 onclick 永远收不到（实测"回原点"点了没反应，
  //   反而弹出"点到画面外的黑边"——按钮坐标被当成地图点选换算了）。只在地图图片上响应。
  if(e.target !== mimg) return;
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
  if(wpMode || wpN>0){                       // 多点模式：清队列/取消执行
    fetch('/wp/clear').then(r=>r.json()).then(s=>{
      showToast(s.msg||'队列已清空', 3000);
      wpMode=false; wpN=0; wpmBtn.textContent='多点'; wpmBtn.style.background='';
    }).catch(()=>{});
    return;
  }
  fetch('/goal/clear').then(r=>r.json()).then(s=>showToast(s.msg||'已清除', 3000)).catch(()=>{});
};

// P34 多点导航：三态按钮——「多点」进入排队 → 点地图入队 → 「出发(N)」一键连发
wpmBtn.onclick=()=>{
  if(!wpMode){
    fetch('/wp/clear').then(r=>r.json()).then(s=>{      // 进排队前先清残留
      wpMode=true; wpN=0; wpmBtn.textContent='出发(0)';
      wpmBtn.style.background='#2e7d32';
      showToast('多点模式：点地图依次排队，再按此按钮出发', 4500);
    }).catch(()=>showToast('✗ 连不上服务', 4000));
  } else {
    fetch('/wp/go').then(r=>r.json()).then(s=>{
      showToast((s.ok?'✓ ':'✗ ')+(s.msg||''), s.ok?6000:8000);
      if(s.ok){ wpMode=false; wpN=0; wpmBtn.textContent='多点'; wpmBtn.style.background=''; }
    }).catch(()=>showToast('✗ 连不上服务', 4000));
  }
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
  document.getElementById('goal').textContent=(s.wp?('多点 '+s.wp+' ｜ '):'')+(s.goal||'-');
  if(s.view){ document.getElementById('zoom').textContent=s.view;
              if(view.range===null){ view.range=s.view_range||8.0; view.dx=0; view.dy=0; } }
}).catch(()=>{ document.getElementById('dot').className='dot bad';
               document.getElementById('conn').textContent='断开'; }); }, 400);
</script></body></html>
"""


import goal_nav as gn          # noqa: E402（与本文件同目录：预检直接复用 nav 的规划代码）


class GoalPrecheck:
    """点选目标前的"预检"：用与 `nav` **同一份规划代码**在本地试算一遍（不发任何指令）

    复用 `goal_nav` 的 `load_map` / `build_traversable` / `blocked_mask` / `clearance_cost` /
    `GoalNav.plan()` / `follow_px_path()` / `path_clearance_stats()`，起点取手机侧当前位姿，
    所以"预检通过"≈"nav 也能规划出同一条路径"。

    地图/代价图只加载一次；任何异常都降级为"不预检"，**绝不阻塞点选**。
    注意：预检用的是**默认参数**（inflation 0.15 / prefer-open 3 / min-obstacle-cells 0）；
    如果 `nav` 起了非默认参数，结论可能略有出入。
    """

    def __init__(self, map_prefix, inflation=0.15, prefer_open=3.0, open_dist=0.8,
                 min_obstacle_cells=0, smooth_iter=2):
        self.m = gn.load_map(map_prefix)
        self.trav = gn.build_traversable(self.m, inflation, False, min_obstacle_cells)
        self.blocked, _ = gn.blocked_mask(self.m, False, min_obstacle_cells)
        self.cost = gn.clearance_cost(self.m, self.blocked, open_dist, prefer_open)
        self.d_obs = gn.dist_to_obstacles(self.m, self.blocked)
        self.smooth_iter = smooth_iter
        self.inflation = inflation

    class _Shim:
        """GoalNav.plan() 只用到这几个属性 / get_logger()"""
        def __init__(self, m, trav, blocked, cost, pose):
            self.m, self.trav, self.blocked, self.cost, self.pose = m, trav, blocked, cost, pose

        def get_logger(self):
            class _L:
                def warn(self, msg):
                    pass

                def info(self, msg):
                    pass
            return _L()

    def check(self, start_xy, goal_xy):
        """返回 dict(ok, msg, dist, min_clear, narrow)；失败时 msg = 规划失败原因"""
        shim = self._Shim(self.m, self.trav, self.blocked, self.cost,
                          (float(start_xy[0]), float(start_xy[1]), 0.0))
        path, why = gn.GoalNav.plan(shim, (float(goal_xy[0]), float(goal_xy[1])))
        if path is None:
            return {'ok': False, 'msg': why}
        px, _ = gn.follow_px_path(self.m, self.trav, path, 0.1, self.smooth_iter)
        mn, avg, narrow = gn.path_clearance_stats(self.m, self.d_obs, px)
        return {'ok': True, 'msg': 'ok', 'dist': gn.path_length_m(self.m, px),
                'min_clear': mn, 'avg': avg, 'narrow': narrow}


class PhoneTeleop(MapViewer):
    """在地图查看器基础上：发 /cmd_vel、设/清导航目标（/goal_pose）、保存地图、网页服务状态"""

    def __init__(self, view_range, vmax, wmax, topic='/cmd_vel',
                 save_dir=DEFAULT_SAVE_DIR, goal_topic='/goal_pose',
                 precheck=True, map_prefix=None):
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
        # P34 多点导航（10 方案 A2）：航点队列 + waypoint_follower 的 FollowWaypoints
        self.wp_queue = []            # [(x, y), ...] 世界坐标，排队模式点地图追加
        self.wp_running = False       # FollowWaypoints 正在执行
        self.wp_cur = -1              # 当前航点序号（feedback 回报，0 起）
        self.wp_gh = None             # goal handle（取消用）
        self.wp_res_fut = None        # 结果 future（watch 线程轮询）
        self.wp_client = None         # ActionClient（惰性创建）
        self.wp_n = 0                 # 出发时的航点总数（进度显示用）
        self.view_range_cur = float(view_range)   # 当前视野（网页缩放会改它）
        self.view_off = (0.0, 0.0)                # 视窗中心相对车的偏移（米）
        self.pub = self.create_publisher(Twist, self.topic, 10)
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        # 点选预检：加载地图 + 代价图（只做一次）；失败只降级、不影响其它功能
        self.precheck = None
        if precheck:
            try:
                self.precheck = GoalPrecheck(os.path.expanduser(map_prefix or gn.MAP_DEFAULT))
                self.get_logger().info('预检就绪：%s（与 nav 同一份规划代码）' % (map_prefix or gn.MAP_DEFAULT))
            except Exception as e:
                self.get_logger().warn('预检不可用（点选将跳过预检）：%s' % e)
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
        # ── 预检：用与 nav 同一份规划代码本地试算（不发任何指令）──
        pre, pre_note = None, ''
        if self.precheck is not None:
            pose = self.robot_pose()
            if pose is None:
                pre_note = '｜预检跳过（还没有位姿）'
            else:
                try:
                    pre = self.precheck.check((pose[0], pose[1]), (x, y))
                except Exception as e:
                    pre = None
                    self.get_logger().warn('预检异常（跳过）：%s' % e)
        if pre is not None and not pre['ok']:
            m = '✗ 预检未通过：%s —— 请换个点（确要强行下发：启动 phone 时加 --no-precheck）' % pre['msg']
            self.get_logger().warn('目标 (%.2f, %.2f) 预检失败：%s' % (x, y, pre['msg']))
            return {'ok': False, 'msg': m, 'x': round(float(x), 2), 'y': round(float(y), 2),
                    'sub': 0}
        if pre is not None:
            pre_note = ('｜预检 ✓ 路径 %.2f m，离障碍最近 %.2f m（平均 %.2f）%s'
                        % (pre['dist'], pre['min_clear'], pre['avg'],
                           ' ⚠ 偏紧（<0.12 = 车半宽）' if pre['min_clear'] < 0.12 else ''))
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
        if pre is not None:
            note = pre_note                     # 预检结论比"单格判据"更可信
        elif cell is None:
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

    # ---------------- 多点导航（10 方案 A2）：队列 + FollowWaypoints ----------------
    def _wp_client(self):
        if self.wp_client is None:
            from rclpy.action import ActionClient
            self.wp_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        return self.wp_client

    def wp_add(self, u=None, v=None, x=None, y=None):
        """多点模式：把一个点加入队列（不停车、不下发；出发时交给 waypoint_follower）"""
        if x is None or y is None:
            lv = self.last_view
            if lv is None:
                return {'ok': False, 'msg': '还没有地图画面，稍等再点'}
            x0, y1, s, size = lv
            if u is None or v is None or not (0 <= u < size and 0 <= v < size):
                return {'ok': False, 'msg': '点到了画面外'}
            x, y = x0 + u / s, y1 - v / s
        x, y = float(x), float(y)
        # 逐段预检：起点 = 队列里上一个点（首点用车的当前位姿）——与单目标同一份规划代码
        if self.wp_queue:
            start = self.wp_queue[-1]
        else:
            pose = self.robot_pose()
            start = (pose[0], pose[1]) if pose is not None else None
        note = ''
        if self.precheck is not None and start is not None:
            try:
                pre = self.precheck.check(start, (x, y))
                if pre is not None and not pre['ok']:
                    m = '✗ 第 %d 点预检未通过：%s —— 请换个点' % (len(self.wp_queue) + 1, pre['msg'])
                    self.get_logger().warn(m)
                    return {'ok': False, 'msg': m, 'n': len(self.wp_queue)}
                if pre is not None:
                    note = '｜路径 %.2f m 离障 %.2f' % (pre['dist'], pre['min_clear'])
            except Exception as e:
                self.get_logger().warn('多点预检异常（跳过）：%s' % e)
        self.wp_queue.append((x, y))
        msg = '第 %d 点已排：x=%.2f y=%.2f%s' % (len(self.wp_queue), x, y, note)
        self.get_logger().info(msg)
        return {'ok': True, 'msg': msg, 'n': len(self.wp_queue),
                'x': round(x, 2), 'y': round(y, 2)}

    def wp_go(self):
        """出发：把整个队列交给 waypoint_follower（内部逐段调 navigate_to_pose）"""
        if self.wp_running:
            return {'ok': False, 'msg': '多点导航正在执行（先「清除目标」取消再重排）'}
        if not self.wp_queue:
            return {'ok': False, 'msg': '队列为空：先在多点模式下点地图排队'}
        ac = self._wp_client()
        if not ac.wait_for_server(timeout_sec=3.0):
            return {'ok': False,
                    'msg': 'follow_waypoints 服务不在（robotnav --nav2 未起 nav2 栈？）'}
        g = FollowWaypoints.Goal()
        for (x, y) in self.wp_queue:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            g.poses.append(ps)
        self.set_cmd(0.0, 0.0)
        self.goal = None
        fut = ac.send_goal_async(g, feedback_callback=self._wp_feedback)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 8.0:
            time.sleep(0.05)
        if not fut.done():
            return {'ok': False, 'msg': 'follow_waypoints 响应超时'}
        gh = fut.result()
        if not gh.accepted:
            return {'ok': False, 'msg': '航点任务被 nav2 拒绝'}
        self.wp_gh = gh
        self.wp_running = True
        self.wp_cur = -1
        self.wp_res_fut = gh.get_result_async()
        n = len(self.wp_queue)
        self.wp_n = n
        self.wp_t0 = time.time()
        self.get_logger().info('多点导航出发：%d 个航点' % n)
        threading.Thread(target=self._wp_watch, args=(gh, list(self.wp_queue)),
                         daemon=True).start()
        return {'ok': True, 'msg': '多点出发：%d 个航点（进度见地图与终端）' % n, 'n': n}

    def _wp_feedback(self, fb):
        idx = int(fb.feedback.current_pose_idx)
        if idx != self.wp_cur:
            self.wp_cur = idx
            self.get_logger().info('多点导航：正在前往第 %d/%d 点' % (idx + 1, len(self.wp_queue)))

    def _wp_watch(self, gh, pts):
        """后台线程：轮询结果 future（读布尔不碰 executor），打印每点误差与总耗时"""
        t0 = time.time()
        while rclpy.ok() and not self.wp_res_fut.done() and time.time() - t0 < 1800:
            time.sleep(0.5)
        self.wp_running = False
        if not self.wp_res_fut.done():
            self.get_logger().warn('多点导航 30 分钟超时，取消')
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
            return
        try:
            res = self.wp_res_fut.result().result
            missed = list(res.missed_waypoints)
        except Exception as e:
            self.get_logger().error('多点导航结果读取失败：%s' % e)
            return
        if missed:
            self.get_logger().warn('多点导航完成：未到达的航点序号 %s' % missed)
        else:
            self.get_logger().info('✓ 多点导航全部 %d 点到达，总耗时 %.0f s'
                                   % (len(pts), time.time() - self.wp_t0))

    def wp_clear(self):
        """清空队列；若多点导航在执行则取消（车会在当前航点附近停下）"""
        q = len(self.wp_queue)
        self.wp_queue = []
        cancelled = False
        if self.wp_running and self.wp_gh is not None:
            try:
                self.wp_gh.cancel_goal_async()
                cancelled = True
            except Exception:
                pass
        self.wp_running = False
        self.set_cmd(0.0, 0.0)
        msg = '队列已清空（%d 点）' % q + ('｜多点导航已取消' if cancelled else '')
        self.get_logger().info(msg)
        return {'ok': True, 'msg': msg, 'cancelled': cancelled}

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
            # P34 多点导航状态（HUD「目标」行显示队列/进度）
            'wp': ('排队 %d 点' % len(self.wp_queue)) if self.wp_queue
                  else ('执行中 %d/%d' % (self.wp_cur + 1, self.wp_n)
                        if self.wp_running else None),
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
        elif u.path == '/wp/add':
            # 多点导航：把一个点加入队列（u,v 画面像素 或 x,y 世界坐标）
            try:
                if 'x' in q and 'y' in q:
                    info = self.node.wp_add(x=float(q['x'][0]), y=float(q['y'][0]))
                else:
                    info = self.node.wp_add(u=float(q.get('u', ['-1'])[0]),
                                            v=float(q.get('v', ['-1'])[0]))
            except (TypeError, ValueError):
                info = {'ok': False, 'msg': '参数错误（需要 u,v 或 x,y）'}
            self._send_json(info)
        elif u.path == '/wp/go':
            self._send_json(self.node.wp_go())
        elif u.path == '/wp/clear':
            self._send_json(self.node.wp_clear())
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
            # P34 多点导航：画航点队列（青色点 + 编号 + 顺序连线）
            q = list(node.wp_queue)
            if q:
                prev = node._world_px(q[0][0], q[0][1], size)
                for i, (wx, wy) in enumerate(q):
                    p = node._world_px(wx, wy, size)
                    if p is None:
                        prev = None
                        continue
                    cv2.circle(img, p, 5, (255, 160, 0), 2)
                    cv2.putText(img, str(i + 1), (p[0] + 7, p[1] - 7),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 160, 0), 1)
                    if prev is not None and i > 0:
                        cv2.line(img, prev, p, (255, 160, 0), 1)
                    prev = p
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
    ap.add_argument('--no-precheck', action='store_true',
                    help='关闭点选预检（默认开启：点选/回原点前用 nav 同源代码本地试算，'
                         '不可达就不下发目标；预检按默认参数）')
    ap.add_argument('--map', default=gn.MAP_DEFAULT,
                    help='预检用的地图前缀（默认 %s）' % gn.MAP_DEFAULT)
    ap.add_argument('-s', '--save-dir', default=DEFAULT_SAVE_DIR,
                    help='保存地图的目录（默认 ~/RobotCode/04_map）')
    args = ap.parse_args()

    rclpy.init()
    node = PhoneTeleop(args.range, args.vmax, args.wmax, args.topic,
                       os.path.expanduser(args.save_dir), args.goal_topic,
                       precheck=not args.no_precheck, map_prefix=args.map)
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
