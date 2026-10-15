#!/bin/bash
# 一键启动 atk_mobilenet_v1 摄像头推理 demo
# 用法: ./run.sh
set -e
cd "$(dirname "$0")"

BIN=./atk_mobilenet_v1
MODEL=./model/mobilenet_v1_1.0_224_3588.rknn

# 1) 二进制不存在则先编译
if [ ! -x "$BIN" ]; then
    echo "[run] 未找到二进制，先编译..."
    ./build_native.sh
fi

# 2) 确保 rknn 运行时库可被找到（已系统安装则此项无副作用）
export LD_LIBRARY_PATH="$(pwd)/librknn_api/aarch64:${LD_LIBRARY_PATH}"

# 3) 显示处理
if [ -n "$DISPLAY" ]; then
    echo "[run] 使用当前 X 显示 ($DISPLAY) 启动（带画面窗口）..."
    "$BIN" "$MODEL"
elif command -v xvfb-run >/dev/null 2>&1; then
    echo "[run] 无 DISPLAY，使用 xvfb 虚拟显示启动（带画面窗口）..."
    xvfb-run -a "$BIN" "$MODEL"
else
    echo "[run] 无 DISPLAY，以命令行模式运行（Top5 打印到终端，Ctrl+C 退出）..."
    "$BIN" "$MODEL"
fi
