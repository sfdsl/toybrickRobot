#!/bin/bash
# 在开发板(Toybrick RK3588, Debian 11)上原生编译 yolo11，无需交叉工具链。
# 依赖: sudo apt install -y build-essential cmake libopencv-dev pkg-config
# 注意: 不要使用 build-linux.sh（它要求 /opt/atk-dlrk3588-toolchain 交叉编译器，
#       产物会链接 buildroot 的 OpenCV3 .so.409，在本机 Debian 上无法运行）。
# CMakeLists.txt 自动探测：工具链不存在 -> 走 native 分支(opencv4 pkg-config)。
set -e

cd "$(dirname "$0")"
SCRIPT_DIR="$(pwd)"

BUILD_DIR=build/build_linux_aarch64
INSTALL_DIR=${SCRIPT_DIR}/install/rk3588_linux_aarch64/atk_rknn_yolo11_cam

if [[ ! -d "${BUILD_DIR}" ]]; then
  mkdir -p ${BUILD_DIR}
fi

# 清掉旧的(可能是交叉编译的)安装目录，避免残留 .so.409 链接的二进制
if [[ -d "${INSTALL_DIR}" ]]; then
  rm -rf ${INSTALL_DIR}
fi

cd ${BUILD_DIR}
cmake ../.. \
    -DTARGET_SOC=rk3588 \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
make -j$(nproc)
make install

echo "build ok -> ${INSTALL_DIR}"
echo "运行: cd ${SCRIPT_DIR} && ./run.sh [camera_index]"
