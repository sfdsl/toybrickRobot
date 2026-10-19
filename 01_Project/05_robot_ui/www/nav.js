// 屏2·多点导航：本地列表（localStorage 'wp_list_v1'）为准；地图 tap → /wp/add 预检加点
'use strict';
const listEl=document.getElementById('list'), summaryEl=document.getElementById('summary');
const loopChk=document.getElementById('loop'), goBtn=document.getElementById('go');
const mbox=document.getElementById('mapbox'), mimg=document.getElementById('map');

// 每点元素：{x, y, dwell, name}（世界坐标，米）
const KEY='wp_list_v1';
let list=[];
try{ list=JSON.parse(localStorage.getItem(KEY))||[]; }catch(_){ list=[]; }
if(!Array.isArray(list)) list=[];
let currentMapId=null;
const placeEl=document.getElementById('places');
// 循环开关状态持久化
loopChk.checked = localStorage.getItem(KEY+'_loop')==='1';
loopChk.onchange=()=>localStorage.setItem(KEY+'_loop', loopChk.checked?'1':'0');

function save(){ localStorage.setItem(KEY, JSON.stringify(list)); }
function fmt(n){ return (Math.round(n*100)/100).toFixed(2); }

// 预计 = Σ(每点停留 dwell + 30s 兜底导航估时)
function summary(){
  const sec=list.reduce((a,p)=>a+(+p.dwell||0)+30, 0);
  summaryEl.textContent='共 '+list.length+' 点 · 预计 '+Math.round(sec)+'s';
}

function render(){
  if(!list.length){
    listEl.innerHTML='<div class="s-empty">航点为空：点地图依次加点，顺序即巡逻顺序</div>';
    summary(); return;
  }
  listEl.innerHTML='';
  list.forEach((p,i)=>{
    const row=document.createElement('div'); row.className='s-row';
    const badge=document.createElement('span'); badge.className='s-badge'; badge.textContent=i+1;
    const name=document.createElement('span'); name.className='s-name'; name.textContent=p.name;
    name.title='点击改名';
    name.onclick=()=>{                                     // 改名（prompt）
      const s=prompt('修改点名称', p.name);
      if(s!==null && s.trim()){ p.name=s.trim(); save(); render(); }
    };
    const xy=document.createElement('span'); xy.className='s-xy';
    xy.textContent='x '+fmt(p.x)+' · y '+fmt(p.y);
    const dwell=document.createElement('input');
    dwell.className='s-dwell'; dwell.type='number'; dwell.min='0'; dwell.max='60';
    dwell.value=p.dwell; dwell.title='停留秒（0-60）';
    dwell.onchange=()=>{                                   // 停留秒（0-60，非法回退 3）
      let d=parseFloat(dwell.value);
      if(isNaN(d)) d=3;
      p.dwell=Math.max(0, Math.min(60, d));
      dwell.value=p.dwell; save(); summary();
    };
    const del=document.createElement('button'); del.className='s-del'; del.textContent='×';
    del.title='删除此点';
    del.onclick=()=>{ list.splice(i,1); save(); render(); };
    const named=document.createElement('button'); named.className='s-mv'; named.textContent='存';
    named.title='将该航点保存为地图地点';
    named.onclick=async()=>{
      const label=prompt('地点名称',p.name);
      if(!label || !label.trim()) return;
      try{
        const s=await fetch(withK('/places/add?name='+encodeURIComponent(label.trim())+'&x='+p.x+'&y='+p.y)).then(r=>r.json());
        showToast(s.ok?'地点已保存':'✗ '+s.msg,4000); if(s.ok) loadPlaces();
      }catch(_){ showToast('✗ 保存地点失败',4000); }
    };
    const up=document.createElement('button'); up.className='s-mv'; up.textContent='↑';
    up.title='上移'; up.disabled=(i===0);
    up.onclick=()=>{ [list[i-1],list[i]]=[list[i],list[i-1]]; save(); render(); };
    const dn=document.createElement('button'); dn.className='s-mv'; dn.textContent='↓';
    dn.title='下移'; dn.disabled=(i===list.length-1);
    dn.onclick=()=>{ [list[i+1],list[i]]=[list[i],list[i+1]]; save(); render(); };
    row.append(badge, name, xy, dwell, named, del, up, dn);
    listEl.appendChild(row);
  });
  summary();
}

// 地图手势：tap = 加点（黑边判断/双击复位在组件内）
initMap({ box: mbox, img: mimg,
  onTapAt: (u, v)=>{
    fetch(withK('/wp/add?u='+u.toFixed(1)+'&v='+v.toFixed(1))).then(r=>r.json()).then(s=>{
      if(s.ok){                                            // 以服务端响应里的世界坐标入列
        list.push({ x:s.x, y:s.y, dwell:3, name:'点'+(list.length+1) });
        save(); render();
      }
      showToast((s.ok?'':'✗ ')+(s.msg||''), s.ok?3000:6000);
    }).catch(()=>showToast('✗ 加点失败：连不上服务', 4000));
  }
});

// 清空：服务端清队列 + 本地列表清空
document.getElementById('clr').onclick=()=>{
  fetch(withK('/wp/clear')).then(r=>r.json()).then(s=>{
    if(!s.ok) showToast('✗ '+(s.msg||'清空失败'), 6000);   // 服务端拒绝（如巡逻中）时如实提示
  }).catch(()=>showToast('✗ 清空失败：连不上服务', 4000));
  list=[]; save(); render();
  showToast('航点已清空', 3000);
};

// 开始巡逻：装载（服务端逐点预检，整批拒绝）→ 出发 → 跳屏3
goBtn.onclick=()=>{
  if(!list.length){ showToast('航点为空：先点地图加点', 3000); return; }
  const pts=list.map(p=>[p.x, p.y, p.dwell, p.name]);
  const url='/patrol/load?pts='+encodeURIComponent(JSON.stringify(pts))
           +'&loop='+(loopChk.checked?1:0);
  goBtn.disabled=true; goBtn.textContent='装载中…';
  fetch(withK(url)).then(r=>r.json()).then(s=>{
    if(!s.ok){ showToast('✗ '+(s.msg||'装载失败'), 6000); return; }
    return fetch(withK('/patrol/go')).then(r=>r.json()).then(s2=>{
      if(s2.ok){ location.href='/patrol.html'; }
      else showToast('✗ '+(s2.msg||'出发失败'), 6000);
    });
  }).catch(()=>showToast('✗ 连不上服务', 4000))
    .finally(()=>{ goBtn.disabled=false; goBtn.textContent='开始巡逻'; });
};

render();

async function loadPlaces(){
  try{
    const s=await fetch('/places/list').then(r=>r.json());
    placeEl.replaceChildren();
    if(!s.ok){ placeEl.textContent=s.msg; return; }
    currentMapId=s.map_id;
    const saved=localStorage.getItem(KEY+'_map_id');
    if(saved && saved!==currentMapId && list.length){
      list=[]; save(); render(); showToast('地图已更换，旧航点已清空',6000);
    }
    localStorage.setItem(KEY+'_map_id',currentMapId);
    for(const [name,p] of Object.entries(s.places)){
      const row=document.createElement('div'); row.className='s-row';
      const label=document.createElement('span'); label.className='s-name'; label.textContent=name;
      const xy=document.createElement('span'); xy.className='s-xy'; xy.textContent='x '+fmt(p.x)+' · y '+fmt(p.y);
      const go=document.createElement('button'); go.className='s-mv'; go.textContent='前往';
      go.onclick=async()=>{
        const v=await fetch(withK('/places/go?name='+encodeURIComponent(name)+'&map_id='+currentMapId)).then(r=>r.json());
        showToast(v.ok?'已下发目标':'✗ '+v.msg,5000);
      };
      const del=document.createElement('button'); del.className='s-del'; del.textContent='×';
      del.onclick=async()=>{
        const v=await fetch(withK('/places/delete?name='+encodeURIComponent(name))).then(r=>r.json());
        showToast(v.ok?'已删除':'✗ '+v.msg,4000); if(v.ok) loadPlaces();
      };
      row.append(label,xy,go,del); placeEl.appendChild(row);
    }
    if(!Object.keys(s.places).length) placeEl.textContent='尚无地点：航点行点击「存」';
  }catch(_){ placeEl.parentElement.hidden=true; }
}
loadPlaces();
