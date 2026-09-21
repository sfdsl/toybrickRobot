#!/usr/bin/env bash
# ============================================================
# 一键启动：底盘 + 雷达 + SLAM 建图 + 手机遥控（网页，含「保存地图」按钮）
#
#   ~/RobotCode/start_robot.sh             启动全套；Ctrl-C 一键停止
#   ~/RobotCode/start_robot.sh --dry-run   只做预检查，不启动
#   ~/RobotCode/start_robot.sh --stop      停掉全套（先发零速度，再按序退出）
#
# 启动完成后终端会打印手机要打开的地址：http://<小车IP>:8080/
# 地图保存到 ~/RobotCode/04_map/（文件名自动带时间戳，手机页面点「保存地图」即可）
#
# 别名：robot（启动） / robotstop（停止）——见 ~/.bash_aliases
# ============================================================
set -u

WS="$HOME/RobotCode/ros2_ws"
SLAM_DIR="$HOME/RobotCode/01_Project/03_SLAM"
LOG_DIR="$HOME/RobotCode/logs/run_$(date +%m%d_%H%M%S)"
SETUP="/opt/ros2-foxy/install/setup.bash"
PORT=8080

NAMES=(底盘 雷达 SLAM 手机遥控)
PIDS=()

say()  { printf '%s\n' "$*"; }
fail() { printf '✗ %s\n' "$*" >&2; }
ok()   { printf '✓ %s\n' "$*"; }

# ---------------- 预检查 ----------------
precheck() {
    local bad=0
    for d in /dev/ttyCH341USB0 /dev/LD14P; do
        [ -e "$d" ] || { fail "缺少设备 $d（底盘/雷达没插好？）"; bad=1; }
    done
    [ -f "$SETUP" ] || { fail "找不到 $SETUP"; bad=1; }
    [ -d "$WS/install" ] || { fail "找不到工作空间 $WS/install"; bad=1; }
    [ -f "$SLAM_DIR/phone_teleop.py" ] || { fail "找不到 $SLAM_DIR/phone_teleop.py"; bad=1; }

    local running=""
    pgrep -f 'chassis_node'            >/dev/null && running="$running 底盘节点"
    pgrep -f 'ldlidar --ros-args'      >/dev/null && running="$running 雷达"
    pgrep -f 'async_slam_toolbox_node' >/dev/null && running="$running SLAM"
    pgrep -f 'phone_teleop.py'         >/dev/null && running="$running 手机遥控"
    if [ -n "$running" ]; then
        fail "检测到已有实例在运行：$running"
        say  "        先执行： $0 --stop   （或直接 Ctrl-C 掉旧终端）"
        bad=1
    fi
    return $bad
}

# ---------------- 启动 ----------------
start_all() {
    mkdir -p "$LOG_DIR" "$HOME/RobotCode/04_map"

    # shellcheck disable=SC1090
    source "$SETUP"
    source "$WS/install/setup.bash"

    say "日志目录：$LOG_DIR"
    say "启动中 ...（底盘 → 雷达 → SLAM → 手机遥控，各等 2 秒）"

    ros2 run base_driver chassis_node                    >"$LOG_DIR/chassis.log" 2>&1 &
    PIDS+=($!); sleep 2
    ros2 launch ldlidar ld14p.launch.py                  >"$LOG_DIR/lidar.log"   2>&1 &
    PIDS+=($!); sleep 2
    ros2 launch slam_toolbox online_async_launch.py      >"$LOG_DIR/slam.log"    2>&1 &
    PIDS+=($!); sleep 2
    python3 "$SLAM_DIR/phone_teleop.py" -p "$PORT"       >"$LOG_DIR/phone.log"   2>&1 &
    PIDS+=($!); sleep 2

    # ---- 状态检查 ----
    local ip; ip=$(hostname -I 2>/dev/null | awk '{print $1}')
    say ""
    say "========================================================"
    local i alive=0
    for i in "${!PIDS[@]}"; do
        if kill -0 "${PIDS[$i]}" 2>/dev/null; then
            ok "${NAMES[$i]} 已启动（pid ${PIDS[$i]}）"
            alive=$((alive + 1))
        else
            fail "${NAMES[$i]} 启动后立即退出 —— 日志尾部："
            tail -n 6 "$LOG_DIR/"*.log 2>/dev/null | tail -n 8 >&2
        fi
    done
    say "--------------------------------------------------------"
    say " 手机浏览器打开： http://${ip:-<小车IP>}:${PORT}/"
    say " 手机页面：虚拟摇杆遥控 + 实时地图 + 「保存地图」按钮"
    say " 地图保存目录： ~/RobotCode/04_map/  （文件名自动带时间戳）"
    say " 日志：$LOG_DIR"
    say " Ctrl-C 停止全套（先停车再按序退出）"
    say "========================================================"
    [ "$alive" -eq 0 ] && { fail "没有任何进程存活，检查日志后重试"; return 1; }
    return 0
}

# ---------------- 停止 ----------------
stop_all() {
    # shellcheck disable=SC1090
    [ -f "$SETUP" ] && source "$SETUP" 2>/dev/null
    say "先发零速度（保险）..."
    timeout 1 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1

    # 用 SIGINT（等同 Ctrl-C）：各节点会走自己的退出流程（底盘发 AT+MT_STOP、手机发零速度）
    say "按序退出：手机遥控 → SLAM → 雷达 → 底盘 ..."
    pkill -INT -f 'phone_teleop.py'                2>/dev/null; sleep 0.5
    pkill -INT -f 'async_slam_toolbox_node'        2>/dev/null
    pkill -INT -f 'ros2 launch slam_toolbox'       2>/dev/null; sleep 0.5
    pkill -INT -f 'ldlidar'                        2>/dev/null
    pkill -INT -f 'ros2 launch ldlidar'            2>/dev/null; sleep 0.5
    pkill -INT -f 'chassis_node'                   2>/dev/null; sleep 1.5

    # 兜底：还活着的强制结束
    for pat in 'phone_teleop.py' 'async_slam_toolbox_node' 'ldlidar' 'chassis_node'; do
        if pgrep -f "$pat" >/dev/null; then
            say "强制结束残留：$pat"
            pkill -9 -f "$pat" 2>/dev/null
        fi
    done
    say "已全部停止。"
}

# ---------------- 看门狗：子进程掉线时提醒 ----------------
monitor() {
    while true; do
        sleep 5
        local i alive=0
        for i in "${!PIDS[@]}"; do
            if kill -0 "${PIDS[$i]}" 2>/dev/null; then
                alive=$((alive + 1))
            elif [ -n "${PIDS[$i]}" ]; then
                fail "${NAMES[$i]} 进程退出了（pid ${PIDS[$i]}）——日志：$LOG_DIR"
                tail -n 5 "$LOG_DIR"/*.log 2>/dev/null | tail -n 6 >&2
                PIDS[$i]=""
            fi
        done
        [ "$alive" -eq 0 ] && { say "所有子进程已退出，脚本结束。"; exit 0; }
    done
}

# ---------------- main ----------------
case "${1:-}" in
    --dry-run)
        say "预检查（--dry-run，不启动任何进程）..."
        if precheck; then ok "预检查通过，可以启动： $0"; else fail "预检查未通过"; exit 1; fi
        exit 0
        ;;
    --stop)
        stop_all
        exit 0
        ;;
    "" ) ;;
    * ) fail "未知参数：$1（可用：--dry-run / --stop）"; exit 2 ;;
esac

say "=== 预检查 ==="
precheck || { fail "预检查未通过，未启动。"; exit 1; }
ok "预检查通过"

cleanup() {
    say ""
    say "收到退出信号，正在停止全套 ..."
    stop_all
    exit 0
}
trap cleanup INT TERM

start_all || exit 1
monitor
