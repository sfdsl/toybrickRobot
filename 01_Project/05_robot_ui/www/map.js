// 地图组件：推流 <img> 的缩放(服务端重渲染)/平移/点选/双击复位
// 从屏1 index.js 整体抽出，逻辑逐字保留，只改挂接方式。
// 用法：const map = initMap({ box, img, onTapAt, zoomBtns })
//   box      地图容器（手势绑在这，按钮事件不受影响的规则见下方注释）
//   img      <img id=... src="/stream.mjpg">
//   onTapAt(u, v)  单指点选回调（画面像素坐标）；双击已被组件消费（复位），不会进回调
//   zoomBtns 可选 {in, out, reset}：桌面缩放按钮 id 对应元素
'use strict';
function initMap(cfg) {
  const box = cfg.box, img = cfg.img;

  // ---------------- 视图状态：缩放（服务端重渲染）/ 平移 / 复位 ----------------
  let view = { range:null, dx:0, dy:0 };   // range=null 表示"还没从 /state 拿到"
  const RANGE_MIN=1.0, RANGE_MAX=20.0, PAN_MAX=10.0;
  let vTimer=null, vLast=0;
  function viewReady(){ if(view.range===null) view.range=8.0; return view; }
  function pushView(){
    const v=viewReady();
    fetch(withK('/view?range='+v.range.toFixed(2)+'&dx='+v.dx.toFixed(2)+'&dy='+v.dy.toFixed(2)))
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
    fetch(withK('/view?reset=1')).then(r=>r.json()).then(s=>{
      view.range=s.range; view.dx=s.dx; view.dy=s.dy;
      showToast('视图已复位（回车上）', 2000); }).catch(()=>{}); }

  // 屏幕像素 → 米：图片按 object-fit:contain 显示，先算缩放，再用 view.range/边长
  function mapGeom(){
    const r=img.getBoundingClientRect(), nw=img.naturalWidth||480, nh=img.naturalHeight||480;
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

  // 单指轻点：双击=复位（组件内消费），其余交给页面 onTapAt(u,v)
  function tapAt(cx, cy){
    const now=Date.now();
    if(now-lastTap < 300){ lastTap=0; resetView(); return; }   // 双击 = 复位
    lastTap=now;
    const g=mapGeom();
    const u=(cx-g.ox)/g.sc, v=(cy-g.oy)/g.sc;                  // 画面像素
    if(u<0||v<0||u>g.nw||v>g.nh){ showToast('点到画面外的黑边了，请点地图上', 3000); return; }
    if(cfg.onTapAt) cfg.onTapAt(u, v);
  }

  box.addEventListener('pointerdown', e=>{
    // ⚠ 修复：按钮(#btns/#zbtns/HUD)都在 mapbox 内——不排除的话 setPointerCapture 会把
    //   click 的 target 捕获到 mapbox，按钮 onclick 永远收不到（实测"回原点"点了没反应，
    //   反而弹出"点到画面外的黑边"——按钮坐标被当成地图点选换算了）。只在地图图片上响应。
    if(e.target !== img) return;
    gp.set(e.pointerId, {x:e.clientX, y:e.clientY});
    try{ box.setPointerCapture(e.pointerId); }catch(_){}
    if(gp.size===1){ startPt={x:e.clientX, y:e.clientY}; moved=false; twoFinger=false; }
    if(gp.size===2){
      twoFinger=true;
      const p=[...gp.values()];
      pinch0={dist:Math.max(1, Math.hypot(p[0].x-p[1].x, p[0].y-p[1].y)), range:viewReady().range};
    }
  });
  box.addEventListener('pointermove', e=>{
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
    if(wasTap) tapAt(e.clientX, e.clientY);
    if(gp.size===0){ startPt=null; clearTimeout(vTimer); vLast=Date.now(); pushView(); }
  }
  box.addEventListener('pointerup', endPtr);
  box.addEventListener('pointercancel', endPtr);

  // 桌面缩放按钮（可选）
  if(cfg.zoomBtns){
    if(cfg.zoomBtns.in)   cfg.zoomBtns.in.onclick  =()=>{ const v=viewReady(); v.range/=1.5; clampView(); clearTimeout(vTimer); vLast=Date.now(); pushView(); };
    if(cfg.zoomBtns.out)  cfg.zoomBtns.out.onclick =()=>{ const v=viewReady(); v.range*=1.5; clampView(); clearTimeout(vTimer); vLast=Date.now(); pushView(); };
    if(cfg.zoomBtns.reset)cfg.zoomBtns.reset.onclick=resetView;
  }

  return { resetView: resetView, view: view };
}
