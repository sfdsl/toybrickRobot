#!/bin/bash
# 一键启动 rknn_yolov5_cam 摄像头目标检测 demo
# 用法: ./run.sh [camera_index]
#   camera_index 缺省为 44（正点原子/瑞芯微 MIPI 摄像头常见序号），可改成 0/1 等
set -e
cd "$(dirname "$0")"
SCRIPT_DIR="$(pwd)"

BIN_NAME=rknn_yolov5_cam
MODEL_NAME=best-bug.rknn

# 候选安装目录（兼容 build_native.sh 与 build-linux.sh 两种命名/位置）
CANDIDATES=(
    "install/rk3588_linux_aarch64/rknn_yolov5_cam"
    "install/rk3588_linux_aarch64/atk_rknn_yolov5_cam"
    "build/build_linux_aarch64/install/rk3588_linux_aarch64/rknn_yolov5_cam"
    "build/build_linux_aarch64/install/rk3588_linux_aarch64/atk_rknn_yolov5_cam"
)

BIN=""
for d in "${CANDIDATES[@]}"; do
    if [ -x "$SCRIPT_DIR/$d/$BIN_NAME" ]; then
        BIN="$(realpath "$SCRIPT_DIR/$d/$BIN_NAME")"
        break
    fi
done

# 二进制不存在则先编译
if [ -z "$BIN" ]; then
    echo "[run] 未找到二进制，先编译..."
    ./build_native.sh
    for d in "${CANDIDATES[@]}"; do
        if [ -x "$SCRIPT_DIR/$d/$BIN_NAME" ]; then
            BIN="$(realpath "$SCRIPT_DIR/$d/$BIN_NAME")"
            break
        fi
    done
fi

if [ -z "$BIN" ]; then
    echo "[run] 仍然找不到二进制，请检查编译是否成功。"
    exit 1
fi

INSTALL_DIR="$(dirname "$BIN")"
MODEL="$INSTALL_DIR/model/$MODEL_NAME"

cd "$INSTALL_DIR"
export LD_LIBRARY_PATH="$(pwd)/lib:${LD_LIBRARY_PATH}"

CAM_INDEX="${1:-44}"

# 单实例保护：避免“GNOME 登录自启”与“手动点图标/脚本”重复启动，
# 导致屏幕上同时出现两个窗口（关掉一个、另一个过会儿又冒出来）。
LOCK="/tmp/rknn_yolov5_cam.lock"
exec 9>"$LOCK" || { echo "[run] 无法创建锁文件，退出"; exit 1; }
if ! flock -n 9; then
    echo "[run] 已有实例在运行，本次启动直接退出（避免重复窗口）。"
    exit 0
fi

# 显示处理：有 X 显示直接跑；否则用 xvfb 虚拟显示；都没有则以命令行模式运行
if [ -n "$DISPLAY" ]; then
    echo "[run] 使用当前 X 显示 ($DISPLAY) 启动摄像头检测（带画面窗口）..."
    "$BIN" "$MODEL" "$CAM_INDEX"
elif command -v xvfb-run >/dev/null 2>&1; then
    echo "[run] 无 DISPLAY，使用 xvfb 虚拟显示启动（带画面窗口）..."
    xvfb-run -a "$BIN" "$MODEL" "$CAM_INDEX"
else
    echo "[run] 无 DISPLAY，以命令行模式运行（检测结果打印到终端，Ctrl+C 退出）..."
    "$BIN" "$MODEL" "$CAM_INDEX"
fi
