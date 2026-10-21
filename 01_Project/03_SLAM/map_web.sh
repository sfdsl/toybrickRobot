#!/usr/bin/env bash
# ============================================================
# map_web.sh —— 在浏览器里实时看地图 + nav2 膨胀层（SSH / 无显示器环境用）
#
# 背景：小车本机没有图形会话时 `mapv` 开不了窗（cv2.imshow 报 GTK 后端失败）。
#       本脚本让小车持续渲染 PNG，再用 http.server 提供网页，PC 用浏览器看。
#       只走 HTTP（PC 主动发起 TCP），不依赖 DDS/组播，不受 Windows 防火墙影响。
#
# 用法：
#   ./map_web.sh              启动（端口 8090 ｜ 完整全局代价图 ｜ 1.5 s 刷新）
#   ./map_web.sh -r 4         视野 4 m（看箱子 / 窄缝细节）
#   ./map_web.sh -p 8091      换端口（8080 被四屏 UI 占用）
#   ./map_web.sh -i 1.0       刷新间隔 1 秒
#   ./map_web.sh --status     看是否在跑、图像更新时间
#   ./map_web.sh --stop       停止
#
# 颜色：黄=低代价(1~49) 橙=高代价(50~99) 红=致命(100，车中心到不了)
#       红区连成片 = 这条缝过不去
# ============================================================
set -u

TOOL="$HOME/RobotCode/01_Project/03_SLAM/map_viewer.py"
DIR=/tmp/cm
PIDF="$DIR/pids"
PORT=8090
RANGE=8
FULL_MAP=1
PERIOD=1.5
SETUP=/opt/ros/humble/setup.bash

say()  { printf '%s\n' "$*"; }
fail() { printf '✗ %s\n' "$*" >&2; }

local_ip() {
    ip route get 1.1.1.1 2>/dev/null \
        | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}'
}

stop_all() {
    if [ -f "$PIDF" ]; then
        while read -r pid; do
            [ -n "$pid" ] && kill "$pid" 2>/dev/null
        done < "$PIDF"
        rm -f "$PIDF"
    fi
    pkill -f 'map_viewer.py --costmap --stream' 2>/dev/null
    pkill -f "http.server $PORT --directory $DIR" 2>/dev/null
    sleep 0.5
}

status() {
    say "=== map_web 状态 ==="
    local n_r n_h
    n_r=$(pgrep -fc 'map_viewer.py --costmap --stream' 2>/dev/null || echo 0)
    n_h=$(pgrep -fc "http.server $PORT --directory $DIR" 2>/dev/null || echo 0)
    say "  渲染进程 : $([ "$n_r" -gt 0 ] && echo '运行中' || echo '未运行')"
    say "  HTTP 服务: $([ "$n_h" -gt 0 ] && echo '运行中' || echo '未运行')"
    if [ -f "$DIR/cm.png" ]; then
        say "  图像更新 : $(stat -c '%y' "$DIR/cm.png" | cut -d. -f1)"
        say "  图像大小 : $(stat -c '%s' "$DIR/cm.png") 字节"
    else
        say "  图像     : 还没有"
    fi
    local code
    code=$(curl -s -o /dev/null -w '%{http_code}' "http://127.0.0.1:$PORT/" 2>/dev/null || echo 000)
    say "  本机 HTTP: $code"
    local ip; ip=$(local_ip)
    [ -n "$ip" ] && say "  浏览器   : http://$ip:$PORT/"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --stop)   stop_all; say "已停止"; exit 0 ;;
        --status) status; exit 0 ;;
        -p)       PORT="${2:?缺端口}"; shift 2 ;;
        -r)       RANGE="${2:?缺视野}"; FULL_MAP=0; shift 2 ;;
        -i)       PERIOD="${2:?缺间隔}"; shift 2 ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) fail "未知参数：$1（-p 端口 / -r 视野 / -i 间隔 / --status / --stop）"; exit 2 ;;
    esac
done

[ -f "$TOOL" ] || { fail "找不到 $TOOL"; exit 1; }
[ -f "$SETUP" ] || { fail "找不到 $SETUP（ROS 2 没装？）"; exit 1; }

mkdir -p "$DIR"
stop_all        # 先清旧的，保证幂等
: > "$PIDF"

# 每次更新页面，避免旧 HTML 残留。
    cat > "$DIR/index.html" <<'HTML'
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Nav2 全局膨胀代价地图</title>
<style>
 html,body{margin:0;height:100%;background:#111;color:#ddd;font:13px/1.6 monospace}
 #bar{padding:6px 10px;background:#1d1d1d;border-bottom:1px solid #333}
 #wrap{height:calc(100% - 34px);display:flex;align-items:center;justify-content:center}
 img{max-width:100%;max-height:100%}
 .k{display:inline-block;width:10px;height:10px;margin:0 4px 0 12px;vertical-align:middle}
</style></head><body>
<div id="bar"><b>Nav2 全局代价图</b> ｜ <b id="st">加载中…</b>
<span class="k" style="background:#ffdc00"></span>低代价
<span class="k" style="background:#ff8c00"></span>高代价
<span class="k" style="background:#ff0000"></span>致命障碍
</div>
<div id="wrap"><img id="im" src="cm.png"></div>
<script>
const im=document.getElementById('im'), st=document.getElementById('st');
let busy=false, lastRender=null, lastChange=performance.now();
async function tick(){
 if(busy)return; busy=true;
 try{
  const response=await fetch('cm.png.json?t='+Date.now(),{cache:'no-store'});
  if(!response.ok)throw Error('status');
  const data=await response.json();
  if(data.rendered_at!==lastRender){lastRender=data.rendered_at;lastChange=performance.now();}
  const age=data.age_seconds===null?null:data.age_seconds+(performance.now()-lastChange)/1000;
  const fresh=age!==null && age<5;
  st.textContent=!data.received?'等待 Nav2 全局代价图（不是 SLAM 图）':
   (fresh?'实时':'数据已过期，请检查 Nav2')+' ｜ '+data.width+'×'+data.height+
   ' ｜ '+(data.full_map?'完整全图':'车辆周围视野')+' ｜ 数据距今 '+age.toFixed(1)+' 秒';
  st.style.color=fresh?'#8ed99a':'#ffb35b';
  im.src='cm.png?t='+Date.now();
 }catch(e){st.textContent='等待渲染服务 / 状态读取失败';st.style.color='#ffb35b';}
 finally{busy=false;}
}
im.onerror=()=>{st.textContent='等待图像（渲染进程启动中）…';};
setInterval(tick, __INTERVAL_MS__); tick();
</script></body></html>
HTML
python3 - "$DIR/index.html" "$PERIOD" <<'PY'
import pathlib, sys
p = pathlib.Path(sys.argv[1])
p.write_text(p.read_text().replace("__INTERVAL_MS__", str(max(200, int(float(sys.argv[2]) * 1000)))))
PY

# 注意 set +u：ROS 的 setup.bash 在 set -u 下会报 unbound variable
set +u
# shellcheck disable=SC1090
source "$SETUP" 2>/dev/null
if [ -f "$HOME/RobotCode/ros2_ws/install/setup.bash" ]; then
    source "$HOME/RobotCode/ros2_ws/install/setup.bash"
fi
set -u

VIEW_ARGS=(--costmap-only)
[ "$FULL_MAP" -eq 1 ] && VIEW_ARGS+=(--full-map)
rm -f "$DIR/cm.png" "$DIR/cm.png.json"
setsid nohup python3 "$TOOL" --costmap --stream "$DIR/cm.png" \
    --period "$PERIOD" -r "$RANGE" "${VIEW_ARGS[@]}" > "$DIR/render.log" 2>&1 < /dev/null &
echo $! >> "$PIDF"

setsid nohup python3 -m http.server "$PORT" --directory "$DIR" --bind 0.0.0.0 \
    > "$DIR/http.log" 2>&1 < /dev/null &
echo $! >> "$PIDF"

say "启动中 ...（等 8 秒收数据）"
sleep 8

code=$(curl -s -o /dev/null -w '%{http_code}' "http://127.0.0.1:$PORT/" 2>/dev/null || echo 000)
if [ "$code" != "200" ]; then
    fail "HTTP 自检失败（code=$code），看日志："
    say "  $DIR/render.log"; say "  $DIR/http.log"
    tail -5 "$DIR/render.log" 2>/dev/null
    exit 1
fi
grep -q '打不开窗口' "$DIR/render.log" 2>/dev/null && { fail "渲染进程异常"; exit 1; }

ip=$(local_ip)
say ""
say "✓ 就绪。浏览器打开："
say "    http://${ip:-<小车IP>}:$PORT/"
say ""
say "  全局代价图（-r 可切换局部视野） ｜ 刷新 ${PERIOD} s ｜ 停止： $(basename "$0") --stop"
