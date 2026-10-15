#!/bin/bash
# 在开发板(Toybrick)上原生编译，无需交叉工具链。
# 依赖: sudo apt install -y libopencv-dev
set -e
cd "$(dirname "$0")"
g++ atk_mobilenet_v1.cpp -o atk_mobilenet_v1 \
    $(pkg-config --cflags --libs opencv4) \
    -I./librknn_api/include -L./librknn_api/aarch64 -lrknnrt -ldl -lpthread
echo "build ok -> ./atk_mobilenet_v1"
echo "运行: LD_LIBRARY_PATH=./librknn_api/aarch64 ./atk_mobilenet_v1 ./model/mobilenet_v1_1.0_224_3588.rknn"
