#!/usr/bin/env bash
# ============================================================
# 一键启动（launch 版）：**预检查 + 急停** 归脚本，真正的启动/停止交给 ROS 2 launch
#
#   ~/RobotCode/start_robot.sh                     建图模式启动（默认）
#   ~/RobotCode/start_robot.sh --mode localization 定位模式启动（导航用）
#   ~/RobotCode/start_robot.sh --nav               同时起导航监听（手机点地图即出发；可连续点）
#   ~/RobotCode/start_robot.sh --nav2              nav2 全栈（手机点地图 → /goal_pose → nav2）
#   ── 网页服务总是启动 robot_ui.app（六屏：遥控/航点/巡逻/探索/编辑/功能），无需参数 ──
#      旧 phone_teleop.py（单屏，无探索/编辑）已归档到 01_Project/bak_phone_teleop_20261020/
#   ~/RobotCode/start_robot.sh --cam               + 相机（cam_pub）：hub 屏看原始画面
#                                                  （仍不跑 yolo 识别）
#   ~/RobotCode/start_robot.sh --vision            视觉全链路：cam_pub + rknn_infer + 六屏UI
#                                                  （= --cam + yolo；手机 hub 屏 → /cam.mjpg
#                                                   实时检测画面，《13》P3）
#   （--ui 仍可写但已无作用：网页总是启动）
#   ~/RobotCode/start_robot.sh --ekf               启用 EKF 融合（卡-02：轮速+IMU）
#                                                  → odom→base_link 改由 EKF 发布，底盘自动停发 TF
#                                                  （可与 --nav2 组合：robotnav --nav2 --ekf）
#   ~/RobotCode/start_robot.sh --serial /dev/ttyUSB0
#                                                  ESP32 串口语音（P3）：进程内监听意图 JSON 行，
#                                                  协议见 robot_ui/serial_cmd.py（禁用 /dev/ttyACM0）
#   ~/RobotCode/start_robot.sh --mqtt [--mqtt-host H]
#                                                  MQTT 遥测/任务桥（P3 卡-04）：云端 Broker，
#                                                  TLS/账号/证书走环境变量 MQTT_HOST/MQTT_PORT/
#                                                  MQTT_USER/MQTT_PASS 或 --mqtt-args 透传
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

SETUP="/opt/ros/humble/setup.bash"
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
    # ⚠ P25：nav2 也在 robotnav --nav2 的 launch 里——不清会把旧 nav2 留成孤儿
    #   （实测：robotstop 后 bringup 死了、nav2 半边还挂着，新 launch 起来后两套打架）
    pkill -INT -f 'nav2.launch.py'           2>/dev/null; sleep 0.5
    pkill -INT -f 'lib/nav2_'                2>/dev/null; sleep 0.5
    pkill -INT -f 'nav2_goal_bridge'         2>/dev/null; sleep 0.5
    pkill -INT -f '/control_mux.py'          2>/dev/null; sleep 0.2
    # 视觉链路（--vision 拉起的三件套 + 可能残留的旧版手机遥控）
    pkill -INT -f 'phone_teleop.py'          2>/dev/null; sleep 0.5
    pkill -INT -f 'robot_ui.app'             2>/dev/null; sleep 0.3
    pkill -INT -f 'mqtt_bridge.py'           2>/dev/null; sleep 0.2
    pkill -f   'rknn_infer/rknn_infer'       2>/dev/null; sleep 0.3
    pkill -f   'cam_pub/cam_pub'             2>/dev/null; sleep 0.3
    for pat in 'phone_teleop.py' 'slam_toolbox' 'ldlidar' 'chassis_node' 'odom_rec.py' \
               'nav2.launch.py' 'lib/nav2_' 'nav2_goal_bridge' \
               'robot_ui.app' 'mqtt_bridge.py' 'rknn_infer/rknn_infer' 'cam_pub/cam_pub'; do
        if pgrep -f "$pat" >/dev/null; then
            say "强制结束残留：$pat"
            pkill -9 -f "$pat" 2>/dev/null
        fi
    done
    say "已全部停止。"
    # ⚠ Fast DDS 共享内存段残留（实测累积 134 个）会让节点间"看得见话题、收不到数据"
    #   ——典型症状：网页设目标车不动、/goal_pose 有订阅者但回调不触发。每次停机清一次。
    rm -f /dev/shm/fastrtps_* 2>/dev/null
    # ~/.ros/log 是各节点的底层 stdout 日志（每次 launch 生成上千个小文件），
    # 排查看 /tmp/bringup.log、/tmp/nav2.log 就够——停机时一并清掉，防越积越多。
    rm -rf /home/toybrick/.ros/log/* 2>/dev/null
    say "已清理 fastrtps 共享内存与 ~/.ros 日志。"
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
    pgrep -f 'robot_ui.app'     >/dev/null && running="$running 网页UI"
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
NAV2=0
UI=1    # 默认就起 robot_ui.app（六屏）；--ui 仍可显式写，行为一致
CAM=0
VISION=0
EKF=0
BAG=0
SERIAL=""        # P3：ESP32 串口设备（空=不起）
MQTT=0           # P3：MQTT 遥测/任务桥
MQTT_ARGS=""     # 透传给 mqtt_bridge.py 的参数（--host/--port/--tls-ca 等）
while [ $# -gt 0 ]; do
    case "$1" in
        --stop)     stop_all; exit 0 ;;
        --mode)     MODE="${2:-mapping}"; shift 2 ;;
        --nav)      NAV=1; shift ;;
        --nav2)     NAV2=1; shift ;;
        --ui)       UI=1; shift ;;
        --cam)      CAM=1; shift ;;            # 只开相机（cam_pub），网页 UI 本来就在跑
        --vision)   UI=1; CAM=1; VISION=1; shift ;;   # = --cam + yolo 检测
        --ekf)      EKF=1; shift ;;            # 启用 EKF 融合（卡-02）：odom→base_link 改由 EKF 发布
        --bag)      BAG=1; shift ;;            # 录制控制权、速度链路和定位话题
        --serial)   SERIAL="${2:-}"; shift 2 ;;   # P3：ESP32 串口语音（如 /dev/ttyUSB0）
        --mqtt)     MQTT=1; shift ;;           # P3：MQTT 遥测/任务桥（卡-04）
        --mqtt-args) MQTT_ARGS="${2:-}"; shift 2 ;;  # 透传 mqtt_bridge.py 参数（引号包整串）
        --dry-run)  DRY=1; shift ;;
        *)          fail "未知参数：$1（可用：--mode mapping|localization / --nav / --nav2 / --ui / --cam / --vision / --ekf / --bag / --serial DEV / --mqtt / --dry-run / --stop）"; exit 2 ;;
    esac
done
# EKF 开关转成 launch 参数（bringup 里 ekf:=true 会同时让底盘 publish_tf=False，避免双发布）
EKF_ARG=false
[ "$EKF" = "1" ] && EKF_ARG=true
if [ "$CAM" = "1" ] && [ "$UI" = "0" ] && [ "$VISION" = "0" ]; then
    say "提示：--cam 只开相机（cam_pub），页面画面要等 cam_pub 起来；继续启动。"
fi
case "$MODE" in
    mapping|localization) ;;
    *) fail "mode 只能是 mapping 或 localization（收到：$MODE）"; exit 2 ;;
esac

say "=== 预检查（mode=$MODE）==="
precheck || { fail "预检查未通过，未启动。"; exit 1; }
ok "预检查通过"

# 启动前也清一次 shm 残留（防止上次异常退出留下的损坏段把 DDS 搞坏）
rm -f /dev/shm/fastrtps_* 2>/dev/null

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

# 旧版 phone_teleop.py 是**单屏** UI（没有探索/编辑/功能中心），现已停用：
#   P2 起网页服务一律用 robot_ui.app（05_robot_ui 的六屏），所以 launch 里恒不拉起旧版，
#   否则 robot --mode mapping --nav2 会退回旧界面，看不到探索/编辑屏。
PHONE_ARG="false"

if ! ros2 pkg prefix robot_bringup >/dev/null 2>&1; then
    fail "找不到 robot_bringup 包 —— 先构建："
    say  "        cd $WS && colcon build --packages-select robot_bringup && source install/setup.bash"
    exit 1
fi

# 实时位姿/速度记录 → /tmp/rec_live.log（每秒一条；errexp 与诊断都读它）
# ⚠ 必须在 source 之后拉起（rec 依赖 rclpy）——P25 教训：放在 source 前会 import 失败静默退出
if pgrep -f 'odom_rec.py' >/dev/null; then pkill -9 -f 'odom_rec.py'; sleep 0.3; fi
nohup python3 "$HOME/RobotCode/01_Project/03_SLAM/odom_rec.py" >/dev/null 2>&1 &
disown 2>/dev/null || true
say "位姿记录已开：tail -f /tmp/rec_live.log"

# ---------------- 六屏 UI / 相机 / 视觉链路（独立后台，robotstop 统一收） ----------------
# （无参数）    ：只 robot_ui（六屏页）；不开相机、不跑 yolo —— 最省
# --cam         ：+ cam_pub → robot_ui（/cam.mjpg 原始相机画面），仍不跑 yolo
# --vision      ：+ cam_pub + rknn_infer（/detections + /image_det）→ /cam.mjpg 画框画面
if [ "$UI" = "1" ] || [ "$CAM" = "1" ]; then
    # 清旧实例（含旧版 phone_teleop——四屏 UI 接管 8080 端口，避免两套 UI 抢）
    pkill -9 -f 'cam_pub/cam_pub'      2>/dev/null
    pkill -9 -f 'rknn_infer/rknn_infer' 2>/dev/null
    pkill -9 -f 'robot_ui.app'         2>/dev/null
    pkill -INT -f 'phone_teleop.py'    2>/dev/null; sleep 0.5
    if [ "$CAM" = "1" ]; then
        nohup ros2 run cam_pub cam_pub > /tmp/cam_pub.log 2>&1 &
    fi
    if [ "$VISION" = "1" ]; then
        nohup ros2 run rknn_infer rknn_infer > /tmp/rknn_infer.log 2>&1 &
    fi
    if [ "$UI" = "1" ]; then
        UI_TOPIC="/cmd_vel"
        [ "$NAV2" = "1" ] && UI_TOPIC="/cmd_vel_manual"
        UI_SERIAL=""
        [ -n "$SERIAL" ] && UI_SERIAL="--serial $SERIAL"
        ( cd "$HOME/RobotCode/01_Project/03_SLAM" && \
          nohup python3 -m robot_ui.app -t "$UI_TOPIC" $UI_SERIAL > /tmp/robot_ui.log 2>&1 & )
    fi
    # P3：MQTT 遥测/任务桥（卡-04）——凭证走环境变量/密钥文件，参数经 --mqtt-args 透传
    if [ "$MQTT" = "1" ]; then
        pkill -INT -f 'mqtt_bridge.py' 2>/dev/null; sleep 0.3
        ( cd "$HOME/RobotCode/01_Project/03_SLAM" && \
          nohup python3 mqtt_bridge.py $MQTT_ARGS > /tmp/mqtt_bridge.log 2>&1 & )
        sleep 1
        pgrep -f 'mqtt_bridge.py' >/dev/null \
            && ok "MQTT 桥已启动（日志 /tmp/mqtt_bridge.log；参数：${MQTT_ARGS:-环境变量}）" \
            || fail "MQTT 桥未启动：看 /tmp/mqtt_bridge.log（--host/MQTT_HOST 必填）"
    fi
    sleep 2
    ok_ui=1
    [ "$UI" = "1" ]     && { pgrep -f 'robot_ui.app'          >/dev/null || ok_ui=0; }
    [ "$CAM" = "1" ]    && { pgrep -f 'cam_pub/cam_pub'       >/dev/null || ok_ui=0; }
    [ "$VISION" = "1" ] && { pgrep -f 'rknn_infer/rknn_infer' >/dev/null || ok_ui=0; }
    if [ "$ok_ui" = "1" ]; then
        if [ "$VISION" = "1" ]; then
            ok "视觉链路已开（含 yolo）：手机开四屏 UI「功能中心」看检测画面（地址见 /tmp/robot_ui.log）"
        elif [ "$CAM" = "1" ]; then
            ok "四屏 UI + 相机已开（无 yolo 识别）：「功能中心」显示原始相机画面（地址见 /tmp/robot_ui.log）"
        elif [ "$UI" = "1" ]; then
            ok "四屏 UI 已开（无相机、无 yolo）：遥控/地图/航点/巡逻可用，「功能中心」无画面（地址见 /tmp/robot_ui.log）"
        else
            ok "相机已开（cam_pub → /image_raw）"
        fi
    else
        fail "UI/相机/视觉链路有进程未存活，查看 /tmp/cam_pub.log /tmp/rknn_infer.log /tmp/robot_ui.log"
    fi
fi

# P33：launch 输出实时落盘（tee → /tmp/nav2_live.log）——打转主体/recovery 判定
#   需要 nav2 日志，不能只靠用户终端滚动缓冲。SIGINT 发给前台进程组，
#   ros2 launch 与 tee 同时收到，Ctrl-C 行为不变（shell 留守等管道结束，无副作用）。
if [ "$NAV" = "1" ]; then
    [ "$MODE" = "localization" ] || say "提示：--nav 通常与 --mode localization 一起用（当前 mode=$MODE）"
    say "启动中 ...（含导航监听：手机点地图即出发、可连续点；Ctrl-C 停止全部）"
    say "日志落盘：tail -f /tmp/nav2_live.log"
    ros2 launch robot_bringup bringup.launch.py mode:="$MODE" nav:=true phone:="$PHONE_ARG" ekf:="$EKF_ARG" bag:="$BAG" 2>&1 | tee /tmp/nav2_live.log
    exit 0
fi

# nav2 全栈（--nav2）：手机点地图 → /goal_pose → nav2_goal_bridge → navigate_to_pose
if [ "$NAV2" = "1" ]; then
    [ "$MODE" = "localization" ] || say "提示：--nav2 通常与 --mode localization 一起用（当前 mode=$MODE）"
    say "启动中 ...（nav2 全栈 + 手机点地图即出发；Ctrl-C 停止全部）"
    say "日志落盘：tail -f /tmp/nav2_live.log"
    ros2 launch robot_bringup bringup.launch.py mode:="$MODE" nav2:=true phone:="$PHONE_ARG" ekf:="$EKF_ARG" bag:="$BAG" 2>&1 | tee /tmp/nav2_live.log
    exit 0
fi
say "启动中 ...（Ctrl-C 停止全部）"
say "日志落盘：tail -f /tmp/nav2_live.log"
ros2 launch robot_bringup bringup.launch.py mode:="$MODE" phone:="$PHONE_ARG" ekf:="$EKF_ARG" bag:="$BAG" 2>&1 | tee /tmp/nav2_live.log
