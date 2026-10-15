// 屏3·巡逻执行：onState(s.patrol) 驱动进度卡/状态列表/药丸；旧后端无 s.patrol 时降级不报错
'use strict';
const pill=document.getElementById('pill'), curEl=document.getElementById('cur'),
      totalEl=document.getElementById('total'), bar=document.getElementById('bar'),
      elapsedEl=document.getElementById('elapsed'), etaEl=document.getElementById('eta'),
      listEl=document.getElementById('list'), ppBtn=document.getElementById('pp');
const TX={done:'已完成', active:'进行中', skipped:'已跳过', failed:'失败', pending:'待走'};
const TXC={done:'s-tx-done', active:'s-tx-active', skipped:'s-tx-skipped',
           failed:'s-tx-failed', pending:'s-tx-pending'};
const PILL={run:['s-green','运行中'], pause:['s-amber','已暂停'], idle:['s-pending','待命']};

// 秒 → 分:秒
function mmss(t){
  t=Math.max(0, Math.round(t));
  return Math.floor(t/60)+':'+String(t%60).padStart(2,'0');
}

// 巡逻点（名字/dwell）：进页从服务端拿一次，与 onState 的 per 按下标配对
let pts=[];
fetch('/patrol/points').then(r=>r.json()).then(s=>{
  if(s && s.ok && Array.isArray(s.pts)) pts=s.pts;
  if(pts.length) renderList(null);
}).catch(()=>{ listEl.innerHTML='<div class="s-empty">⚠ 巡逻点加载失败（连不上服务）</div>'; });

function renderList(per){
  listEl.innerHTML='';
  if(!pts.length){
    listEl.innerHTML='<div class="s-empty">未装载巡逻点（回屏2 加点后开始巡逻）</div>';
    return;
  }
  pts.forEach((p,i)=>{
    const st=(per && per[i]) || 'pending';
    const row=document.createElement('div'); row.className='s-row';
    const badge=document.createElement('span');
    badge.className='s-badge s-'+st; badge.textContent=i+1;   // 徽标圆随状态换色
    const name=document.createElement('span'); name.className='s-name';
    name.textContent=p.name || ('点'+(i+1));
    const info=document.createElement('span'); info.className='s-xy';
    info.textContent='停留 '+p.dwell+'s';
    const stTx=document.createElement('span'); stTx.className=TXC[st]||'s-tx-pending';
    stTx.style.fontSize='12px'; stTx.textContent=TX[st]||st;
    row.append(badge, name, info, stTx);
    listEl.appendChild(row);
  });
}

// 按钮动作：ok:false 时 toast 原文（withK：/patrol/* 控制类路由需鉴权；points 免鉴权不包）
function act(url){
  fetch(withK(url)).then(r=>r.json()).then(s=>{
    if(!s.ok) showToast('✗ '+(s.msg||'操作失败'), 6000);
    else if(url==='/patrol/cancel') showToast('已取消', 3000);  // 留在页上看状态
  }).catch(()=>showToast('✗ 连不上服务', 4000));
}
ppBtn.onclick=()=>act(ppBtn.dataset.paused==='1'?'/patrol/resume':'/patrol/pause');
document.getElementById('skip').onclick=()=>act('/patrol/skip');
document.getElementById('cancel').onclick=()=>act('/patrol/cancel');

// SSE 状态：旧后端没有 s.patrol 直接忽略
onState(s=>{
  if(!s.patrol) return;
  const p=s.patrol;
  const running=!!p.running, paused=!!p.paused;
  // 药丸：运行中(绿)/已暂停(琥珀)/待命(灰)
  const key=running?(paused?'pause':'run'):'idle';
  const failed=!running && /失败|异常/.test(p.msg||'');
  pill.className='s-pill '+(failed?'s-amber':PILL[key][0]);
  pill.textContent=running?PILL[key][1]:(p.msg||PILL[key][1]);
  // 进度卡
  const total=p.total||0, cur=Math.min(p.cur||0, total);
  curEl.textContent=running||paused ? (cur+1>total?total:cur+1) : cur;
  totalEl.textContent=total;
  bar.style.width=(total?Math.round((cur/total)*100):0)+'%';
  elapsedEl.textContent=mmss(p.elapsed||0);
  etaEl.textContent=(p.eta==null || p.eta<0) ? '--:--' : mmss(p.eta);
  // 暂停/恢复二态按钮
  ppBtn.dataset.paused=paused?'1':'0';
  ppBtn.textContent=paused?'恢复':'暂停';
  // 每点状态列表
  renderList(p.per);
});
