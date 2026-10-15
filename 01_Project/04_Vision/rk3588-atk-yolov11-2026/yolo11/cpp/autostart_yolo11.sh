#!/bin/bash
# 桌面开机自启包装脚本：等摄像头设备就绪后再启动 YOLOv11 检测窗口
# 由 GNOME 会话(autostart)调用，自动继承 DISPLAY 等环境变量 -> 走 GUI 窗口模式

SCRIPT_DIR="/userdata/aidemo/rk3588-atk-yolov11-2026/yolo11/cpp"
RUN_SH="$SCRIPT_DIR/run.sh"

# 等 /dev/video0 出现（最多 60s），避免登录瞬间摄像头驱动还没就绪
for i in $(seq 1 60); do
    [ -c /dev/video0 ] && break
    sleep 1
done

# 自启时输出记入日志，便于开机失败时排查（journal 看不到 .desktop 的 stdout）
LOG="/tmp/yolo11_autostart.log"
echo "[autostart] $(date '+%F %T') DISPLAY=$DISPLAY 开始启动..." > "$LOG"
exec /bin/bash "$RUN_SH" 0 >> "$LOG" 2>&1
