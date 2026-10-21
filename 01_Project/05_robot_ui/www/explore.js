// 探索屏（P2-B1）：启停 + 前沿可视化 + 存档/启用
'use strict';
const cv = document.getElementById('cv'), ctx = cv.getContext('2d');
const toastEl = document.getElementById('toast');
const connEl = document.getElementById('conn'), connTxt = document.getElementById('connTxt');

let geom = null;                       // {w,h,resolution,origin:[x,y]}
let img = new Image(), imgReady = false;
let scale = 1, ox = 0, oy = 0;
let mode = 'continue';
let frontiers = [], goal = null, robot = null;
let lastSaved = null;

function flash(t, bad) {
  toastEl.textContent = t;
  toastEl.className = 'e-toast on' + (bad ? ' bad' : ' ok');
  clearTimeout(flash._t);
  flash._t = setTimeout(() => toastEl.className = 'e-toast', 3400);
}
function setConn(ok, txt) {
  connEl.classList.toggle('on', !!ok);
  connTxt.textContent = txt;
}

// ---------------- 画布 ----------------
function fit() {
  if (!imgReady || !geom) return;
  const r = cv.getBoundingClientRect();
  scale = Math.min(r.width / geom.w, r.height / geom.h);
  ox = (r.width - geom.w * scale) / 2;
  oy = (r.height - geom.h * scale) / 2;
  redraw();
}
function resize() {
  const r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  cv.width = r.width * dpr; cv.height = r.height * dpr;
  fit();
}
function worldToPx(x, y) {
  if (!geom) return null;
  // 栅格行从下到上，图片行从上到下 → y 要翻
  const col = (x - geom.origin[0]) / geom.resolution;
  const row = (y - geom.origin[1]) / geom.resolution;
  return [ox + col * scale, oy + (geom.h - row) * scale];
}
function redraw() {
  const r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, r.width, r.height);
  // 每次绘制都按【当前】矩形重算缩放：布局稳定后高度会变，
  // 只在 fit() 里算一次会留下"地图只占左上角、右边一大片空白"（实测踩到）。
  if (imgReady && geom) {
    scale = Math.min(r.width / geom.w, r.height / geom.h);
    ox = (r.width - geom.w * scale) / 2;
    oy = (r.height - geom.h * scale) / 2;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(img, ox, oy, geom.w * scale, geom.h * scale);
  }

  // 前沿点（青）
  frontiers.forEach(p => {
    const q = worldToPx(p[0], p[1]); if (!q) return;
    ctx.beginPath(); ctx.arc(q[0], q[1], 4, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(0,220,255,.9)'; ctx.fill();
    ctx.lineWidth = 1.2; ctx.strokeStyle = 'rgba(0,220,255,.35)'; ctx.stroke();
  });
  // 当前目标（琥珀）
  if (goal) {
    const q = worldToPx(goal.x, goal.y); if (q) {
      ctx.beginPath(); ctx.arc(q[0], q[1], 7, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(255,193,77,.9)'; ctx.fill();
      ctx.lineWidth = 2; ctx.strokeStyle = '#fff'; ctx.stroke();
    }
  }
  // 车（红）
  if (robot) {
    const q = worldToPx(robot.x, robot.y); if (q) {
      ctx.save(); ctx.translate(q[0], q[1]);
      ctx.rotate(Math.PI / 2 - robot.yaw);
      ctx.beginPath(); ctx.moveTo(0, -9);
      ctx.lineTo(7, 7); ctx.lineTo(-7, 7);
      ctx.closePath(); ctx.fillStyle = '#ff5b5b'; ctx.fill();
      ctx.lineWidth = 1.5; ctx.strokeStyle = '#fff'; ctx.stroke();
      ctx.restore();
    }
  }
}

// ---------------- 数据 ----------------
function loadInfo() {
  return fetch('/explore/info').then(r => r.json()).then(s => {
    if (!s.ok) throw new Error(s.msg);
    geom = { w: s.w, h: s.h, resolution: s.resolution, origin: s.origin };
  });
}
function loadImage() {
  return new Promise((res, rej) => {
    const im = new Image();
    im.onload = () => { img = im; imgReady = true; redraw(); res(); };
    im.onerror = () => rej(new Error('底图加载失败'));
    im.src = '/explore/image.png?t=' + Date.now();
  });
}
function refresh() {
  return Promise.all([loadInfo(), loadImage()]).then(fit)
    .catch(e => flash('✗ ' + e.message, true));
}

// ---------------- 状态渲染 ----------------
function render(s) {
  setConn(true, '已连接');
  const e = s.explore;
  if (!e) return;
  frontiers = e.frontier_pts || [];
  goal = e.goal || null;
  // pose 形如 "x=1.23 y=-0.45 yaw=12°"
  if (s.pose) {
    const m = /x=([-\d.]+)\s+y=([-\d.]+)\s+yaw=([-\d.]+)/.exec(s.pose);
    robot = m ? { x: +m[1], y: +m[2], yaw: +m[3] * Math.PI / 180 } : null;
  }
  document.getElementById('frontiers').textContent = e.frontiers;
  document.getElementById('visited').textContent = e.visited;
  document.getElementById('failed').textContent = e.failed;
  document.getElementById('goal').textContent = e.goal
    ? e.goal.x.toFixed(2) + ', ' + e.goal.y.toFixed(2) : '-';
  const sv = document.getElementById('saved');
  sv.textContent = e.saved_as ? e.saved_as.split('/').pop() : '（未保存）';
  sv.classList.toggle('e-saved', !!e.saved_as);
  document.getElementById('msg').textContent = e.msg || '-';

  const badge = document.getElementById('modeBadge');
  const selectedMode = e.running ? e.mode : mode;
  badge.textContent = selectedMode === 'fresh' ? '探索当前图' : '续建已有图';
  badge.classList.toggle('fresh', selectedMode === 'fresh');
  document.querySelectorAll('.e-chip[data-mode]').forEach(b => {
    b.disabled = !!e.running;
    b.classList.toggle('on', b.dataset.mode === selectedMode);
  });

  const run = document.getElementById('runBadge');
  run.className = 'e-run' + (e.running ? ' on' : '');
  run.textContent = e.running ? '探索中' : '空闲';
  document.getElementById('btnGo').disabled = !!e.running;

  if (e.saved_as) lastSaved = e.saved_as;
  redraw();
}

// ---------------- 控制 ----------------
document.querySelectorAll('.e-chip[data-mode]').forEach(b => b.onclick = () => {
  document.querySelectorAll('.e-chip[data-mode]').forEach(x => x.classList.remove('on'));
  b.classList.add('on'); mode = b.dataset.mode;
  document.getElementById('modeBadge').textContent =
    mode === 'fresh' ? '探索当前图' : '续建已有图';
  document.getElementById('modeBadge').classList.toggle('fresh', mode === 'fresh');
});

document.getElementById('btnGo').onclick = e => {
  const button = e.currentTarget;
  button.disabled = true;
  fetch(withK('/explore/go?mode=' + mode)).then(r => r.json()).then(s => {
    flash(s.ok ? '✓ ' + s.msg : '✗ ' + s.msg, !s.ok);
  }).catch(() => flash('✗ 启动失败：连不上服务', true))
    .finally(() => { button.disabled = false; });
};
document.getElementById('btnStop').onclick = () =>
  fetch(withK('/explore/cancel')).then(r => r.json()).then(s =>
    flash('✓ ' + s.msg, false)).catch(() => flash('✗ 停止失败', true));

document.getElementById('btnSave').onclick = e => {
  const button = e.currentTarget;
  button.disabled = true;
  fetch(withK('/explore/save')).then(r => r.json()).then(s => {
    flash(s.ok ? '✓ ' + s.msg : '✗ ' + s.msg, !s.ok);
    if (s.ok && s.saved_as) lastSaved = s.saved_as;
  }).catch(() => flash('✗ 存档失败', true))
    .finally(() => { button.disabled = false; });
};

document.getElementById('btnActivate').onclick = () => {
  const name = lastSaved ? lastSaved.split('/').pop() : '';
  if (!name) { flash('✗ 没有可启用的存档（先「存档」）', true); return; }
  if (!confirm('确定把 ' + name + ' 切为当前地图？\n原图会自动备份，但地点/航点会因 map_id 变化被拒。'))
    return;
  fetch(withK('/explore/activate')).then(r => r.json()).then(s =>
    flash(s.ok ? '✓ ' + s.msg : '✗ ' + s.msg, !s.ok))
    .catch(() => flash('✗ 启用失败', true));
};

// ---------------- 启动 ----------------
onState(render);
window.addEventListener('resize', resize);
// 布局稳定后高度会变（字体/滚动条），只靠 window.resize 会用到错误尺寸 → 画布顶端对齐
if (window.ResizeObserver) new ResizeObserver(resize).observe(cv);
resize();
refresh();
setInterval(() => { if (imgReady) refresh(); }, 4000);
