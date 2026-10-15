// 屏1·遥控页逻辑（原 PAGE <script> 平移；状态推送已改走 /events SSE）
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

function send(){ fetch(withK('/cmd?vx='+vx.toFixed(3)+'&wz='+wz.toFixed(3))).catch(()=>{}); }
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

// ---------------- 地图手势/缩放/点选：已组件化到 map.js ----------------
// 点选回调 = 原 setGoalAt 双击复位之后的逻辑（双击复位/黑边判断在组件内）：
function onTapAtMap(u, v){
  if(wpMode){                                                // 多点模式：入队，不停车不下发
    fetch(withK('/wp/add?u='+u.toFixed(1)+'&v='+v.toFixed(1))).then(r=>r.json()).then(s=>{
      if(s.ok){ wpN=s.n; wpmBtn.textContent='出发('+wpN+')'; }
      showToast((s.ok?'':'✗ ')+(s.msg||''), s.ok?3000:6000);
    }).catch(()=>showToast('✗ 入队失败：连不上服务', 4000));
    return;
  }
  active=false; setKnob(0,0,false); halt();                  // 先停车，避免和导航抢控制
  fetch(withK('/goal?u='+u.toFixed(1)+'&v='+v.toFixed(1))).then(r=>r.json()).then(s=>{
    showToast((s.ok?'✓ ':'✗ ')+(s.msg||''), s.ok?6000:8000);
  }).catch(()=>showToast('✗ 设置目标失败：连不上服务', 5000));
}
const mapView = initMap({ box: mbox, img: mimg,
  onTapAt: onTapAtMap,
  zoomBtns: { in: document.getElementById('zin'),
              out: document.getElementById('zout'),
              reset: document.getElementById('zrst') } });
document.getElementById('hudT').onclick=()=>hud.classList.toggle('min');

// 保存地图：先停车，再让板子把 .pgm/.yaml + .posegraph/.data 存到 04_map（名字带时间戳）
saveBtn.onclick=()=>{
  active=false; setKnob(0,0,false); halt();
  saveBtn.disabled=true; saveBtn.textContent='保存中…';
  fetch(withK('/savemap')).then(r=>r.json()).then(s=>{
    showToast((s.ok?'✓ 已保存：':'✗ 保存失败：')+(s.msg||''), s.ok?7000:9000);
  }).catch(()=>showToast('✗ 保存失败：连不上服务（板子上的 phone 还在跑吗？）',8000))
    .finally(()=>{ saveBtn.disabled=false; saveBtn.textContent='保存地图'; });
};
goalClr.onclick=()=>{
  if(wpMode || wpN>0){                       // 多点模式：清队列/取消执行
    fetch(withK('/wp/clear')).then(r=>r.json()).then(s=>{
      showToast(s.msg||'队列已清空', 3000);
      wpMode=false; wpN=0; wpmBtn.textContent='多点'; wpmBtn.style.background='';
    }).catch(()=>{});
    return;
  }
  fetch(withK('/goal/clear')).then(r=>r.json()).then(s=>showToast(s.msg||'已清除', 3000)).catch(()=>{});
};

// P34 多点导航：三态按钮——「多点」进入排队 → 点地图入队 → 「出发(N)」一键连发
wpmBtn.onclick=()=>{
  if(!wpMode){
    fetch(withK('/wp/clear')).then(r=>r.json()).then(s=>{      // 进排队前先清残留
      wpMode=true; wpN=0; wpmBtn.textContent='出发(0)';
      wpmBtn.style.background='#2e7d32';
      showToast('多点模式：点地图依次排队，再按此按钮出发', 4500);
    }).catch(()=>showToast('✗ 连不上服务', 4000));
  } else {
    fetch(withK('/wp/go')).then(r=>r.json()).then(s=>{
      showToast((s.ok?'✓ ':'✗ ')+(s.msg||''), s.ok?6000:8000);
      if(s.ok){ wpMode=false; wpN=0; wpmBtn.textContent='多点'; wpmBtn.style.background=''; }
    }).catch(()=>showToast('✗ 连不上服务', 4000));
  }
};
// 一键回原点：目标 = 地图原点 (0,0) = 建图起点（需另开终端跑 nav --listen 才会动）
homeBtn.onclick=()=>{
  active=false; setKnob(0,0,false); halt();          // 先停车，避免和导航抢控制
  homeBtn.disabled=true; homeBtn.textContent='回原点…';
  fetch(withK('/goal/home')).then(r=>r.json()).then(s=>{
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

// 状态推送（SSE）：断线由 EventSource 自动重连；onerror 时显示「断开」
function renderState(s) {
  const d=document.getElementById('dot'), c=document.getElementById('conn');
  d.className='dot '+(s.cmd_age<1?'ok':'bad'); c.textContent=s.cmd_age<1?'已连接':'看门狗待机';
  document.getElementById('pose').textContent=s.pose||'-';
  document.getElementById('vel').textContent=s.vel||'-';
  document.getElementById('goal').textContent=(s.wp?('多点 '+s.wp+' ｜ '):'')+(s.goal||'-');
  if(s.view){ document.getElementById('zoom').textContent=s.view;
              if(mapView.view.range===null){ mapView.view.range=s.view_range||8.0; mapView.view.dx=0; mapView.view.dy=0; } }
  // 模式互斥：巡逻执行中锁定摇杆（s.mode 由 ModeMgr 提供）
  const patrolling = s.mode && s.mode.mode === 'PATROL';
  pad.style.opacity = patrolling ? '.4' : '';
  pad.style.pointerEvents = patrolling ? 'none' : '';
  document.getElementById('hint').textContent = patrolling
    ? '⚠ 巡逻执行中，摇杆已锁定（终止巡逻后可遥控）'
    : '圆盘：上=前进 左右=转向 ｜ 地图：点一下=设目标，拖=平移，捏合=缩放，双击=复位 ｜ 桌面可用方向键';
}
const esState = onState(renderState);
esState.onerror = () => {
  document.getElementById('dot').className = 'dot bad';
  document.getElementById('conn').textContent = '断开';
};
