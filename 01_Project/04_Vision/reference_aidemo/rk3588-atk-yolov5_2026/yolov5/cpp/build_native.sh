#!/bin/bash
# 在开发板(Toybrick, Debian 11)上原生编译，无需交叉工具链。
# 依赖: sudo apt install -y build-essential cmake libopencv-dev
# 注意: 不要在板子上使用 build-linux.sh（它会导出不存在的交叉编译器路径）。
set -e

cd "$(dirname "$0")"
SCRIPT_DIR="$(pwd)"

BUILD_DIR=build/build_linux_aarch64
INSTALL_DIR=${SCRIPT_DIR}/install/rk3588_linux_aarch64/rknn_yolov5_cam

if [[ ! -d "${BUILD_DIR}" ]]; then
  mkdir -p ${BUILD_DIR}
fi

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
echo "运行: cd ${INSTALL_DIR} && ./rknn_yolov5_cam 模型路径"
