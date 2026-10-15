#!/bin/bash
# 桌面开机自启包装脚本：等摄像头设备就绪后再启动 YOLOv5 检测窗口
# 由 GNOME 会话(autostart)调用，自动继承 DISPLAY 等环境变量 -> 走 GUI 窗口模式

SCRIPT_DIR="/userdata/aidemo/rk3588-atk-yolov5_2026/yolov5/cpp"
RUN_SH="$SCRIPT_DIR/run.sh"

# 等 /dev/video0 出现（最多 60s），避免登录瞬间摄像头驱动还没就绪
for i in $(seq 1 60); do
    [ -c /dev/video0 ] && break
    sleep 1
done

# 启动检测（摄像头索引 0 = 当前 USB 摄像头）。GUI 窗口由 run.sh 在 DISPLAY 存在时自动开启
# 自启时输出记入日志，便于开机失败时排查（journal 看不到 .desktop 的 stdout）
LOG="/tmp/yolov5_autostart.log"
echo "[autostart] $(date '+%F %T') DISPLAY=$DISPLAY 开始启动..." > "$LOG"
exec /bin/bash "$RUN_SH" 0 >> "$LOG" 2>&1
