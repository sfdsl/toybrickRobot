#!/usr/bin/env bash
# ============================================================
# 一键启动（launch 版）：**预检查 + 急停** 归脚本，真正的启动/停止交给 ROS 2 launch
#
#   ~/RobotCode/start_robot.sh                     建图模式启动（默认）
#   ~/RobotCode/start_robot.sh --mode localization 定位模式启动（导航用）
#   ~/RobotCode/start_robot.sh --nav               同时起导航监听（手机点地图即出发；可连续点）
#   ~/RobotCode/start_robot.sh --dry-run           只做预检查，不启动
#   ~/RobotCode/start_robot.sh --stop              急停（先发零速度，再收掉所有节点）
#
# 启动内容与参数见 bringup launch：
#   ros2 launch robot_bringup bringup.launch.py --show-args
# 停止：在启动终端按 Ctrl-C —— launch 会把所有子进程一起优雅收掉
#       （底盘节点发 AT+MT_STOP、手机遥控发零速度），比逐个 pkill 干净
#
# 别名：robot / robotnav / robotcheck / robotstop（见 ~/.bash_aliases）
# ============================================================
set -u

SETUP="/opt/ros2-foxy/install/setup.bash"
WS="$HOME/RobotCode/ros2_ws"
MODE="mapping"

say()  { printf '%s\n' "$*"; }
fail() { printf '✗ %s\n' "$*" >&2; }
ok()   { printf '✓ %s\n' "$*"; }

# ---------------- 急停（launch 之外的兜底手段）----------------
stop_all() {
    if [ -f "$SETUP" ]; then
        set +u                       # 同上：ROS setup.bash 在 set -u 下会报 unbound variable
        source "$SETUP" 2>/dev/null
        set -u
    fi
    say "先发零速度（保险）..."
    timeout 1 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
    say "按序退出：手机遥控 → SLAM → 雷达 → 底盘 ..."
    pkill -INT -f 'phone_teleop.py'          2>/dev/null; sleep 0.5
    pkill -INT -f 'slam_toolbox'             2>/dev/null; sleep 0.5
    pkill -INT -f 'ldlidar'                  2>/dev/null; sleep 0.5
    pkill -INT -f 'chassis_node'             2>/dev/null; sleep 1.5
    for pat in 'phone_teleop.py' 'slam_toolbox' 'ldlidar' 'chassis_node'; do
        if pgrep -f "$pat" >/dev/null; then
            say "强制结束残留：$pat"
            pkill -9 -f "$pat" 2>/dev/null
        fi
    done
    say "已全部停止。"
}

# ---------------- 预检查 ----------------
precheck() {
    local bad=0
    for d in /dev/ttyCH341USB0 /dev/LD14P; do
        [ -e "$d" ] || { fail "缺少设备 $d（底盘/雷达没插好？）"; bad=1; }
    done
    [ -f "$SETUP" ] || { fail "找不到 $SETUP"; bad=1; }
    [ -d "$WS/install" ] || { fail "找不到工作空间 $WS/install"; bad=1; }

    local running=""
    pgrep -f 'chassis_node'     >/dev/null && running="$running 底盘"
    pgrep -f 'ldlidar'          >/dev/null && running="$running 雷达"
    pgrep -f 'slam_toolbox'     >/dev/null && running="$running SLAM"
    pgrep -f 'phone_teleop.py'  >/dev/null && running="$running 手机遥控"
    if [ -n "$running" ]; then
        fail "检测到已有实例在运行：$running"
        say  "        先执行： $0 --stop   （或到启动终端按 Ctrl-C）"
        bad=1
    fi
    return $bad
}

# ---------------- main ----------------
MODE="mapping"
DRY=0
NAV=0
while [ $# -gt 0 ]; do
    case "$1" in
        --stop)     stop_all; exit 0 ;;
        --mode)     MODE="${2:-mapping}"; shift 2 ;;
        --nav)      NAV=1; shift ;;
        --dry-run)  DRY=1; shift ;;
        *)          fail "未知参数：$1（可用：--mode mapping|localization / --nav / --dry-run / --stop）"; exit 2 ;;
    esac
done
case "$MODE" in
    mapping|localization) ;;
    *) fail "mode 只能是 mapping 或 localization（收到：$MODE）"; exit 2 ;;
esac

say "=== 预检查（mode=$MODE）==="
precheck || { fail "预检查未通过，未启动。"; exit 1; }
ok "预检查通过"

if [ "$DRY" = "1" ]; then
    say "（--dry-run：只做预检查，不启动；真正启动去掉 --dry-run 即可）"
    exit 0
fi

# ---------------- 交给 launch ----------------
# shellcheck disable=SC1090
# ROS 的 setup.bash 模板里有 `${COLCON_TRACE}` 的裸引用（未定义就报错），
# 在 set -u（nounset）下会报 "unbound variable" 并让脚本直接退出 → source 期间临时关掉
set +u
source "$SETUP"
source "$WS/install/setup.bash"
set -u

if ! ros2 pkg prefix robot_bringup >/dev/null 2>&1; then
    fail "找不到 robot_bringup 包 —— 先构建："
    say  "        cd $WS && colcon build --packages-select robot_bringup && source install/setup.bash"
    exit 1
fi

if [ "$NAV" = "1" ]; then
    [ "$MODE" = "localization" ] || say "提示：--nav 通常与 --mode localization 一起用（当前 mode=$MODE）"
    say "启动中 ...（含导航监听：手机点地图即出发、可连续点；Ctrl-C 停止全部）"
    exec ros2 launch robot_bringup bringup.launch.py mode:="$MODE" nav:=true
fi
say "启动中 ...（Ctrl-C 停止全部）"
exec ros2 launch robot_bringup bringup.launch.py mode:="$MODE"
