// 地图编辑屏（P2-A4）：画布笔刷 + 坐标换算 + 另存不覆盖
'use strict';
const cv = document.getElementById('cv'), ctx = cv.getContext('2d');
const ghost = document.getElementById('ghost'), gctx = ghost.getContext('2d');
const toastEl = document.getElementById('toast');
const connEl = document.getElementById('conn'), connTxt = document.getElementById('connTxt');

// 编辑器状态
let geom = null;                       // {w,h,resolution,origin:[x,y]}
let img = new Image(), imgReady = false;
// 视图：scale = 每格多少像素；ox/oy = 图像左上角在 canvas CSS 像素里的位置
let scale = 1, ox = 0, oy = 0;
// 画笔
let mode = 'erase', radius = 3, panMode = false;
let stroke = [], drawing = false;
let mutations = Promise.resolve(), imageSequence = 0;
function mutate(action) {
  mutations = mutations.catch(() => {}).then(action);
  return mutations;
}

function flash(t, bad) {
  toastEl.textContent = t;
  toastEl.className = 'e-toast on' + (bad ? ' bad' : ' ok');
  clearTimeout(flash._t);
  flash._t = setTimeout(() => toastEl.className = 'e-toast', 3200);
}
function setConn(ok, txt) {
  connEl.classList.toggle('on', !!ok);
  connTxt.textContent = txt;
}

// ---------------- 画布 / 视图 ----------------
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
  const oldW = cv.width / dpr, oldH = cv.height / dpr;
  if (oldW === r.width && oldH === r.height) return;
  [cv, ghost].forEach(c => { c.width = r.width * dpr; c.height = r.height * dpr; });
  ox += (r.width - oldW) / 2; oy += (r.height - oldH) / 2;
  redraw();
}
function redraw() {
  const r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, r.width, r.height);
  if (imgReady && geom) {
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(img, ox, oy, geom.w * scale, geom.h * scale);
  }
}
function drawGhost(x, y) {
  const r = ghost.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  gctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  gctx.clearRect(0, 0, r.width, r.height);
  if (panMode || !geom) return;
  gctx.beginPath();
  gctx.arc(x, y, radius * scale, 0, Math.PI * 2);
  gctx.fillStyle = mode === 'erase' ? 'rgba(0,220,255,.32)' : 'rgba(255,80,80,.34)';
  gctx.fill();
  gctx.lineWidth = 1.5;
  gctx.strokeStyle = mode === 'erase' ? '#00dcff' : '#ff5050';
  gctx.stroke();
}

// canvas CSS 像素 → 世界坐标（米）
function pxToWorld(px, py) {
  if (!geom) return null;
  const col = Math.floor((px - ox) / scale);
  const row = Math.floor((py - oy) / scale);
  if (col < 0 || col >= geom.w || row < 0 || row >= geom.h) return null;
  // 图像是 flipud 的（原点在左下）
  const x = geom.origin[0] + (col + 0.5) * geom.resolution;
  const y = geom.origin[1] + ((geom.h - 1 - row) + 0.5) * geom.resolution;
  return { x, y };
}

// ---------------- 数据加载 ----------------
function loadInfo() {
  return fetch('/edit/info').then(r => r.json()).then(s => {
    if (!s.ok) throw new Error(s.msg);
    geom = { w: s.w, h: s.h, resolution: s.resolution, origin: s.origin };
    document.getElementById('dim').textContent = s.w + '×' + s.h;
    return geom;
  });
}
function loadImage() {
  const seq = ++imageSequence;
  return new Promise((res, rej) => {
    const im = new Image();
    im.onload = () => { if (seq === imageSequence) { img = im; imgReady = true; redraw(); } res(); };
    im.onerror = () => rej(new Error('底图加载失败'));
    im.src = '/edit/image.png?t=' + Date.now();
  });
}
function refresh() {
  return Promise.all([loadInfo(), loadImage()]).then(fit).catch(e => flash('✗ ' + e.message, true));
}

// ---------------- 笔刷提交 ----------------
function sendStroke() {
  if (!stroke.length) return;
  const pts = stroke.map(p => p.x.toFixed(3) + ',' + p.y.toFixed(3)).join(';');
  const url = '/edit/stroke?mode=' + mode + '&radius=' + radius + '&pts=' + encodeURIComponent(pts);
  mutate(() => fetch(withK(url))
    .then(r => r.json())
    .then(s => {
      if (!s.ok) flash('✗ ' + s.msg, true);
      updateStatus(s);
      return loadImage();
    })
    .catch(() => flash('✗ 笔刷提交失败', true)));
}
function updateStatus(s) {
  if (!s) return;
  document.getElementById('strokes').textContent = s.strokes ?? 0;
  document.getElementById('dirty').textContent = s.edited ? '是' : '否';
  document.getElementById('dirty').style.color = s.edited ? 'var(--e-amber)' : 'var(--e-accent)';
  document.getElementById('ver').textContent = s.saved_as
    ? s.saved_as.split('/').pop() : '（未保存）';
  if (s.loaded) document.getElementById('hint').textContent =
    '地图已载入，可缩放/平移。清空编辑将恢复抓取时的底图。';
}

// ---------------- 指针交互 ----------------
function local(e) {
  const r = cv.getBoundingClientRect();
  return { x: e.clientX - r.left, y: e.clientY - r.top };
}
let panStart = null;
cv.addEventListener('pointerdown', e => {
  cv.setPointerCapture(e.pointerId);
  const p = local(e);
  if (panMode) { panStart = { x: p.x, y: p.y, ox, oy }; return; }
  if (!geom) { flash('先点「抓取当前地图」', true); return; }
  drawing = true; stroke = [];
  const w = pxToWorld(p.x, p.y);
  if (w) stroke.push(w);
  drawGhost(p.x, p.y);
});
cv.addEventListener('pointermove', e => {
  const p = local(e);
  if (panStart) {
    ox = panStart.ox + (p.x - panStart.x);
    oy = panStart.oy + (p.y - panStart.y);
    redraw(); return;
  }
  if (!drawing) { drawGhost(p.x, p.y); return; }
  const w = pxToWorld(p.x, p.y);
  if (w) {
    const last = stroke[stroke.length - 1];
    if (!last || Math.hypot(w.x - last.x, w.y - last.y) > geom.resolution * 0.6) stroke.push(w);
  }
  drawGhost(p.x, p.y);
});
['pointerup', 'pointercancel'].forEach(t => cv.addEventListener(t, e => {
  if (panStart) { panStart = null; return; }
  if (!drawing) return;
  drawing = false;
  sendStroke();
  const p = local(e); drawGhost(p.x, p.y);
}));

// ---------------- 工具条 ----------------
document.querySelectorAll('.e-chip[data-mode]').forEach(b => b.onclick = () => {
  document.querySelectorAll('.e-chip[data-mode]').forEach(x => x.classList.remove('on'));
  b.classList.add('on'); mode = b.dataset.mode; panMode = false;
  document.getElementById('panToggle').classList.remove('on');
});
document.querySelectorAll('.e-chip[data-r]').forEach(b => b.onclick = () => {
  document.querySelectorAll('.e-chip[data-r]').forEach(x => x.classList.remove('on'));
  b.classList.add('on'); radius = parseInt(b.dataset.r, 10);
});
document.getElementById('panToggle').onclick = e => {
  panMode = !panMode;
  e.currentTarget.classList.toggle('on', panMode);
};
function zoom(factor) {
  if (!geom || !imgReady) return;
  const r = cv.getBoundingClientRect();
  const base = Math.min(r.width / geom.w, r.height / geom.h);
  const next = Math.max(base * 0.25, Math.min(base * 16, scale * factor));
  ox = r.width / 2 - (r.width / 2 - ox) * next / scale;
  oy = r.height / 2 - (r.height / 2 - oy) * next / scale;
  scale = next; redraw();
}
document.getElementById('zIn').onclick = () => zoom(1.25);
document.getElementById('zOut').onclick = () => zoom(1 / 1.25);
document.getElementById('zFit').onclick = fit;

// ---------------- 动作 ----------------
document.getElementById('btnLoad').onclick = e => {
  const button = e.currentTarget;
  button.disabled = true;
  mutate(() => fetch(withK('/edit/load')).then(r => r.json()).then(s => {
    flash(s.ok ? '✓ ' + s.msg : '✗ ' + s.msg, !s.ok);
    updateStatus(s);
    return refresh();
  }).catch(() => flash('✗ 载入失败：连不上服务', true))
    .finally(() => { button.disabled = false; }));
};
document.getElementById('btnUndo').onclick = () =>
  mutate(() => fetch(withK('/edit/undo')).then(r => r.json()).then(s => {
    flash(s.ok ? '✓ ' + s.msg : '✗ ' + s.msg, !s.ok);
    updateStatus(s); return loadImage();
  }).catch(() => flash('✗ 撤销失败', true)));
document.getElementById('btnReset').onclick = () =>
  mutate(() => fetch(withK('/edit/reset')).then(r => r.json()).then(s => {
    flash(s.ok ? '✓ ' + s.msg : '✗ ' + s.msg, !s.ok);
    updateStatus(s); return loadImage();
  }).catch(() => flash('✗ 清空失败', true)));
document.getElementById('btnSave').onclick = e => {
  const button = e.currentTarget;
  button.disabled = true;
  mutate(() => fetch(withK('/edit/save')).then(r => r.json()).then(s => {
    flash(s.ok ? '✓ ' + s.msg : '✗ ' + s.msg, !s.ok);
    updateStatus(s);
  }).catch(() => flash('✗ 导出失败：连不上服务', true))
    .finally(() => { button.disabled = false; }));
};

// ---------------- 状态推送 ----------------
onState(s => {
  setConn(true, '已连接');
  if (s.edit) updateStatus(s.edit);
  const hint = document.getElementById('hint');
  if (hint && s.mode && s.mode.mode === 'PATROL')
    hint.textContent = '⚠ 巡逻执行中（编辑不影响车，可继续）';
});
window.addEventListener('resize', resize);
// 布局稳定后高度会变（字体/滚动条），只靠 window.resize 会用错尺寸 → 地图不居中
if (window.ResizeObserver) new ResizeObserver(resize).observe(cv);
resize();
refresh();
