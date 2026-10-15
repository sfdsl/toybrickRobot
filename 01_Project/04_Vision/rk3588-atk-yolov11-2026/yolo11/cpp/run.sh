#!/bin/bash
# 一键启动 rknn_yolo11_cam 摄像头目标检测 demo
# 用法: ./run.sh [camera_index]
#   camera_index 缺省为 44（正点原子/瑞芯微 MIPI 摄像头常见序号），可改成 0/1 等
set -e
cd "$(dirname "$0")"
SCRIPT_DIR="$(pwd)"

BIN_NAME=rknn_yolo11_cam
MODEL_NAME=yolo11.rknn

# 候选安装目录（兼容 build-linux.sh 与 build_native 两种命名/位置）
CANDIDATES=(
    "install/rk3588_linux_aarch64/atk_rknn_yolo11_cam"
    "build/install/rk3588_linux_aarch64/atk_rknn_yolo11_cam"
)

BIN=""
for d in "${CANDIDATES[@]}"; do
    if [ -x "$SCRIPT_DIR/$d/$BIN_NAME" ]; then
        BIN="$(realpath "$SCRIPT_DIR/$d/$BIN_NAME")"
        break
    fi
done

# 二进制不存在则先编译（优先本机原生编译，避免交叉 glibc 不匹配）
if [ -z "$BIN" ]; then
    echo "[run] 未找到二进制，先编译..."
    if [ -f "$SCRIPT_DIR/build_native.sh" ]; then
        bash "$SCRIPT_DIR/build_native.sh"
    else
        rm -rf "$SCRIPT_DIR/build" && mkdir -p "$SCRIPT_DIR/build"
        ( cd "$SCRIPT_DIR/build" && cmake .. -DTARGET_SOC=rk3588 && make -j8 && make install )
    fi
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

# 依赖自检：若二进制链接的 OpenCV 找不到（如残留的交叉编译产物 .so.409），
# 自动触发原生重编译，避免 "error while loading shared libraries: libopencv_*.so.409"。
if ! ldd "$BIN" 2>/dev/null | grep -q "libopencv"; then
    : # 二进制可能没动态依赖 opencv（异常），继续尝试运行
elif ldd "$BIN" 2>/dev/null | grep -qE "libopencv.*=> not found"; then
    echo "[run] 检测到二进制依赖的 OpenCV 库缺失（可能是交叉编译残留产物），自动原生重编译..."
    bash "$SCRIPT_DIR/build_native.sh" || true
    # 重新定位
    for d in "${CANDIDATES[@]}"; do
        if [ -x "$SCRIPT_DIR/$d/$BIN_NAME" ]; then
            BIN="$(realpath "$SCRIPT_DIR/$d/$BIN_NAME")"
            break
        fi
    done
    [ -z "$BIN" ] && { echo "[run] 重编译后仍找不到二进制。"; exit 1; }
    INSTALL_DIR="$(dirname "$BIN")"
    MODEL="$INSTALL_DIR/model/$MODEL_NAME"
fi

cd "$INSTALL_DIR"
export LD_LIBRARY_PATH="$(pwd)/lib:${LD_LIBRARY_PATH}"

CAM_INDEX="${1:-44}"

# 单实例保护：避免“GNOME 登录自启”与“手动点脚本/图标”重复启动，
# 导致屏幕上同时出现两个窗口（关掉一个、另一个过会儿又冒出来）。
LOCK="/tmp/rknn_yolo11_cam.lock"
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
