// 共享工具：SSE 订阅 + JSON fetch + toast（/events 已生效，各页面通过 onState 订阅状态）
'use strict';

// token 鉴权（Task 8）：服务端下发 .js 时把占位符替换为真实 token（--no-auth 时为空串）
const TOKEN = '__K__';                     // 服务端下发时替换为真实 token
// URL 已带 k= 时原样返回（幂等）；否则按有无 ? 追加 &k= / ?k=
function withK(u) { return u.indexOf('k=') >= 0 ? u : u + (u.indexOf('?') >= 0 ? '&' : '?') + 'k=' + TOKEN; }

// SSE 订阅：一条连接接收 /state 推送，断线自动重连（EventSource 自带）
function onState(cb) {
  const es = new EventSource('/events');
  es.onmessage = e => { try { cb(JSON.parse(e.data)); } catch (_) {} };
  return es;
}

// fetch JSON 封装（自动带 k=<token>）
function api(url) { return fetch(withK(url)).then(r => r.json()); }

// toast 提示（从原 index 页 <script> 里原样抽出）
function showToast(msg, ms) {
  const toast = document.getElementById('toast');
  if (!toast) return;
  toast.textContent = msg; toast.style.display = 'block';
  clearTimeout(showToast.t); showToast.t = setTimeout(() => toast.style.display = 'none', ms || 5000);
}
