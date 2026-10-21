// 语音屏（P3）：webkitSpeechRecognition 识别 → /voice/intent 意图执行 → speechSynthesis 播报
// 所有写操作走 withK()；误识别兜底 = 全宽红色「停 止」直调 /stop（最高优先，不走意图层）。
// secure-context：识别 API 需要 HTTPS 或 localhost；http://<板子>:8080 在现代 Chrome
// 会被拒麦克风权限 → 页面给降级提示（chrome://flags 白名单或 --https-cert 自签）。
'use strict';

const $ = (id) => document.getElementById(id);

// ---------- toast（复用 e-toast） ----------
let toastTimer = null;
function flash(msg, bad) {
  const t = $('toast') || document.querySelector('.e-toast');
  if (!t) return;
  t.textContent = msg;
  t.className = 'e-toast on' + (bad ? ' bad' : ' ok');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.className = 'e-toast'; }, 3200);
}

// ---------- 波形（按下说话时律动的青色条带） ----------
const waveCv = $('wave');
const wctx = waveCv.getContext('2d');
let waveOn = false, waveT = 0;
function sizeWave() {
  const r = waveCv.parentElement.getBoundingClientRect();
  waveCv.width = r.width * (window.devicePixelRatio || 1);
  waveCv.height = r.height * (window.devicePixelRatio || 1);
}
window.addEventListener('resize', sizeWave);
function drawWave() {
  if (!waveCv.width) sizeWave();
  const W = waveCv.width, H = waveCv.height, dpr = window.devicePixelRatio || 1;
  wctx.clearRect(0, 0, W, H);
  const n = 44, gap = W / n;
  for (let i = 0; i < n; i++) {
    const x = i * gap + gap * 0.25;
    const w = gap * 0.5;
    const centerF = Math.abs(i - n / 2) / (n / 2);           // 中间高两边低
    const base = waveOn ? (1 - centerF) * 0.72 + 0.06 : 0.045;
    const amp = waveOn
      ? base * (0.55 + 0.45 * Math.sin(waveT * 2.4 + i * 0.55))
      : base;
    const h = Math.max(2 * dpr, amp * H * 0.62);
    const y = (H - h) / 2;
    wctx.fillStyle = waveOn
      ? 'rgba(127,209,255,' + (0.16 + amp * 0.42) + ')'
      : 'rgba(127,209,255,0.07)';
    wctx.beginPath();
    wctx.roundRect ? wctx.roundRect(x, y, w, h, w / 2)
                   : wctx.rect(x, y, w, h);
    wctx.fill();
  }
  waveT += 0.05;
  requestAnimationFrame(drawWave);
}
drawWave();

// ---------- 历史 ----------
const HIST_MAX = 12;
function pushHist(name, msg, ok) {
  const ul = $('hist');
  if (!ul) return;
  const li = document.createElement('li');
  const now = new Date();
  const hh = String(now.getHours()).padStart(2, '0');
  const mm = String(now.getMinutes()).padStart(2, '0');
  const ss = String(now.getSeconds()).padStart(2, '0');
  li.innerHTML = '<span class="t">' + hh + ':' + mm + ':' + ss + '</span>' +
    '<span class="n"></span><span class="m"></span>';
  li.querySelector('.n').textContent = name;
  const m = li.querySelector('.m');
  m.textContent = msg || '';
  m.classList.add(ok ? 'r-ok' : 'r-bad');
  ul.insertBefore(li, ul.firstChild);
  while (ul.children.length > HIST_MAX) ul.removeChild(ul.lastChild);
}

// ---------- TTS 播报 ----------
function speak(text, bad) {
  if (!('speechSynthesis' in window) || !text) return;
  try {
    const u = new SpeechSynthesisUtterance(text);
    u.lang = 'zh-CN';
    u.rate = 1.05;
    if (bad) u.rate = 1.15;
    window.speechSynthesis.cancel();      // 新播报打断旧的
    window.speechSynthesis.speak(u);
  } catch (e) { /* 播报失败不影响控制 */ }
}

// ---------- 提交意图 ----------
const INTENT_CN = {
  stop: '停止', forward: '前进', back: '后退', left: '左转', right: '右转',
  home: '回原点', goto_place: '去地点', patrol_start: '开始巡逻',
  patrol_stop: '停止巡逻',
};
function setHeard(text) { $('heard').textContent = text || '—'; }
function setIntent(it, args) {
  const el = $('intent');
  if (!it) { el.textContent = '—'; el.className = 'v-intent'; return; }
  const extra = args && Object.keys(args).length
    ? ' ' + JSON.stringify(args) : '';
  el.textContent = (INTENT_CN[it] || it) + extra;
  el.className = 'v-intent';
}
function setResult(msg, ok) {
  const el = $('result');
  el.textContent = msg || '—';
  el.className = ok ? 'ok' : (msg ? 'bad' : '');
}

function submitText(text) {
  if (!text || !text.trim()) return;
  setHeard(text);
  setIntent(null);
  setResult('执行中…', true);
  fetch(withK('/voice/intent?source=voice&text=' + encodeURIComponent(text)))
    .then(r => r.json())
    .then(s => {
      if (s.intent) setIntent(s.intent, null);
      if (s.ok) {
        setResult(s.msg || '已执行', true);
        if (s.duplicate) flash('重复指令已忽略', false);
        else flash(s.msg || '已执行', false);
      } else {
        setResult(s.msg || '已拒绝', false);
        flash(s.msg || '已拒绝', true);
      }
      speak(s.msg || (s.ok ? '好的' : '无法执行'), !s.ok);
      pushHist(INTENT_CN[s.intent] || text.trim().slice(0, 8),
               s.msg || '', !!s.ok);
    })
    .catch(() => {
      setResult('连不上服务', false);
      flash('✗ 执行失败：连不上服务', true);
      pushHist(text.trim().slice(0, 8), '连不上服务', false);
    });
}

// ---------- 停止兜底（不走意图层，直调 /stop，最高优先） ----------
function doStop() {
  fetch(withK('/stop')).then(r => r.text()).then(() => {
    setResult('已停止', true);
    flash('已停止（巡逻 / 导航 / 手动动作已取消）', false);
    speak('已停止', false);
    pushHist('停止', '本地停止链', true);
  }).catch(() => {
    flash('✗ 停止失败：连不上服务', true);
    pushHist('停止', '失败', false);
  });
}

// ---------- 识别（webkitSpeechRecognition） ----------
const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
let recog = null, recogActive = false;

function checkDegrade() {
  const d = $('degrade');
  const insecure = !window.isSecureContext &&
    location.hostname !== 'localhost' && location.hostname !== '127.0.0.1';
  if (!SR) {
    d.hidden = false;
    d.textContent = '⚠ 当前浏览器不支持语音识别（iOS Safari 部分支持）。' +
      '可用 Chrome/Edge；或用下方「停 止」与文字接口。';
    $('btnMic').style.opacity = '.4';
    return true;
  }
  if (insecure) {
    d.hidden = false;
    d.textContent = '⚠ 非安全上下文（http://IP 访问）时现代浏览器不给麦克风权限。' +
      '处理：chrome://flags → Insecure origins treated as secure 填 ' +
      location.origin + ' ；或启动 app.py 时加 --https-cert 自签证书用 https:// 访问。';
  }
  return false;
}

function startRecog() {
  if (!SR) return;
  recog = new SR();
  recog.lang = 'zh-CN';
  recog.continuous = false;
  recog.interimResults = true;
  recog.onresult = (ev) => {
    let txt = '';
    for (let i = ev.resultIndex; i < ev.results.length; i++) {
      txt += ev.results[i][0].transcript;
    }
    setHeard(txt);
    if (ev.results[ev.results.length - 1].isFinal) {
      submitText(txt);
    }
  };
  recog.onerror = (ev) => {
    recogActive = false;
    waveOn = false;
    $('btnMic').classList.remove('rec');
    $('micLabel').textContent = '按下说话';
    if (ev.error === 'not-allowed') {
      flash('✗ 麦克风权限被拒（secure-context 限制？见下方提示）', true);
    } else if (ev.error !== 'aborted') {
      flash('识别失败：' + ev.error, true);
    }
  };
  recog.onend = () => {
    recogActive = false;
    waveOn = false;
    $('btnMic').classList.remove('rec');
    $('micLabel').textContent = '按下说话';
  };
  try {
    recog.start();
    recogActive = true;
    waveOn = true;
    $('btnMic').classList.add('rec');
    $('micLabel').textContent = '识别中… 松开或再说一次';
  } catch (e) {
    flash('✗ 识别启动失败（可能已在运行）', true);
  }
}

function stopRecog() {
  if (recog && recogActive) {
    try { recog.stop(); } catch (e) { /* noop */ }
  }
}

// 按下开始、松开结束（点按 = 切换开始/结束，方便戴手套操作）
function onMicDown(e) {
  e.preventDefault();
  if (recogActive) { stopRecog(); return; }
  startRecog();
}
$('btnMic').addEventListener('mousedown', onMicDown);
$('btnMic').addEventListener('touchstart', onMicDown, { passive: false });

$('btnStop').addEventListener('click', () => {
  stopRecog();                 // 先停识别，防识别中的指令插队
  doStop();
});

// ---------- SSE 状态：三路灯 + 控制权 + 历史补充 ----------
let lastN = -1;
function render(s) {
  if (!s) return;
  // 连接灯（SSE 在推送 = 服务在线）
  $('conn').classList.add('on');
  $('connTxt').textContent = '已连接';
  // 三路链路灯：识别能力 / MQTT 桥 / 串口
  const lgV = $('lgVoice');
  lgV.className = 'v-light ' + (SR ? 'on' : 'off');
  lgV.title = SR ? '浏览器语音识别可用' : '浏览器不支持识别';
  const lgM = $('lgMqtt');
  const mqttOn = s.mqtt && s.mqtt.connected;
  lgM.className = 'v-light ' + (mqttOn ? 'on' : (s.mqtt && s.mqtt.broker ? 'off' : ''));
  lgM.title = mqttOn ? ('已连 ' + (s.mqtt.broker || '')) : (s.mqtt && s.mqtt.broker ? '已配置未连接' : '未启用');
  const lgS = $('lgSerial');
  const serOn = s.serial && s.serial.enabled;
  lgS.className = 'v-light ' + (serOn ? 'on' : '');
  lgS.title = serOn ? ('串口 ' + (s.serial.port || '')) : '未启用';
  // 控制权
  if (s.mode) {
    $('modeBadge').textContent = s.mode.mode + (s.mode.holder ? ' · ' + s.mode.holder : '');
  }
  // 后端意图统计（其它入口 MQTT/串口 的指令也进历史视野）
  const v = s.voice;
  if (v && v.last && v.n_exec + v.n_reject !== lastN) {
    lastN = v.n_exec + v.n_reject;
    if (v.last.intent && v.last.source !== 'voice') {
      pushHist((INTENT_CN[v.last.intent] || v.last.intent) +
               '·' + (v.last.source || '?'), v.last.msg || '', !!v.last.ok);
    }
    if (v.moving) $('micLabel').textContent = '手动动作执行中…';
  }
}

// ---------- init ----------
checkDegrade();
try { sizeWave(); } catch (e) { /* noop */ }
onState(render);
// 让语音屏自己的 toast 存在（api.js 的 showToast 走 #toast）
if (!document.getElementById('toast')) {
  const t = document.createElement('div');
  t.id = 'toast';
  document.body.appendChild(t);
}
