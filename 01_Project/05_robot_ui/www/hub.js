// 屏4·功能中心：默认 15 张卡为页内常量（后端 /features 失败时的兜底）；
// 进页向服务端功能注册表拉取（Task 8），拿到后覆盖渲染；可跳转的卡整卡为 <a>
'use strict';
const grid=document.getElementById('grid');
// state: ok=已具备(绿) wip=进行中(琥珀) todo=待做(灰)
const CARDS=[
  { id:'A1', name:'手机点图导航', state:'ok',  href:'/' },
  { id:'A2', name:'多点巡逻',     state:'ok',  href:'/nav.html' },
  { id:'A3', name:'语义标注地图', state:'todo' },
  { id:'A4', name:'地图编辑器',   state:'todo' },
  { id:'A5', name:'RKNN视觉',     state:'wip' },
  { id:'A6', name:'语音指令',     state:'wip',  href:'/voice.html' },
  { id:'A7', name:'rosbag',       state:'ok' },
  { id:'A8', name:'状态灯屏按键', state:'wip' },
  { id:'A9', name:'电量温度监控', state:'ok' },
  { id:'B1', name:'自主探索建图', state:'wip' },
  { id:'B2', name:'动态避障',     state:'todo' },
  { id:'B3', name:'目标跟随',     state:'todo' },
  { id:'B4', name:'ArUco停靠',    state:'todo' },
  { id:'B5', name:'自动回充',     state:'todo' },
  { id:'B6', name:'语义地图',     state:'todo' },
];
const SUB={ ok:'已具备', wip:'进行中', todo:'待做' };
const DOT={ ok:'s-done', wip:'s-skipped', todo:'s-pending' };   // 状态点颜色
// 服务端注册表的 state 用中文（已具备/进行中/待做），归一化成上面的英文键
const STKEY={ '已具备':'ok', '进行中':'wip', '待做':'todo' };
function stKey(s){ return STKEY[s] || s; }

function renderCards(cards){
  grid.innerHTML='';
  cards.forEach(c=>{
    const el=document.createElement(c.href ? 'a' : 'div');
    el.className='s-card2';
    if(c.href) el.href=c.href;
    const dot=document.createElement('span'); dot.className='s-dot '+DOT[stKey(c.state)];
    const txt=document.createElement('span');
    txt.textContent=(c.id ? c.id+' ' : '')+c.name;   // 服务端注册表无 id，只显示名称
    const sub=document.createElement('span'); sub.className='s-sub';
    sub.textContent=SUB[stKey(c.state)];
    txt.appendChild(sub);
    el.append(dot, txt);
    grid.appendChild(el);
  });
  // 第 16 格占位
  const ph=document.createElement('div'); ph.className='s-ph'; ph.textContent='·';
  grid.appendChild(ph);
}

renderCards(CARDS);                                    // 先用页内常量渲染（兜底）
api('/features').then(s=>{                             // 服务端注册表：成功则覆盖
  if(s && s.ok && Array.isArray(s.features) && s.features.length) renderCards(s.features);
}).catch(()=>{});                                      // 失败保留页内默认，不报错

// 相机画面来源提示：det=yolo 画框 / raw=原始画面（--ui 没跑 yolo）/ none=没画面
const CAMTXT={ det:'实时检测画面（YOLO11 画框）',
               raw:'原始相机画面（未启用 YOLO11 识别）',
               none:'无画面：cam_pub 未运行（要看画面用 robotnav --nav2 --ui --cam 或 --vision）' };
onState(s=>{
  const n=document.getElementById('camnote');
  if(n && s && s.cam) n.textContent=CAMTXT[s.cam.src] || CAMTXT.none;
});
