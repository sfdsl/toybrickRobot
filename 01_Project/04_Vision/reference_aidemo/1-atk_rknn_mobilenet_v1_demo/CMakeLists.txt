#QQ GROUP:850578317(售后)
#QQ GROUP:286877688(交流)
#http://www.openedv.com/forum.php
project(atk_mobilenet_v1_demo)
cmake_minimum_required(VERSION 3.8)
message(STATUS "cmake version ${CMAKE_VERSION}")

# 自动探测：若存在正点原子 Buildroot 交叉工具链则用交叉编译，
# 否则（如在 Toybrick 开发板上）使用本机原生编译器，避免 glibc 版本不匹配。
set(ATK_TOOLCHAIN /opt/atk-dlrk3588-toolchain)
if(EXISTS "${ATK_TOOLCHAIN}/bin/aarch64-buildroot-linux-gnu-g++")
    message(STATUS "Using ATK Buildroot cross toolchain")
    set(CMAKE_CXX_COMPILER ${ATK_TOOLCHAIN}/bin/aarch64-buildroot-linux-gnu-g++)
    set(CMAKE_C_COMPILER ${ATK_TOOLCHAIN}/bin/aarch64-buildroot-linux-gnu-gcc)
    set(SYSROOT ${ATK_TOOLCHAIN}/aarch64-buildroot-linux-gnu/sysroot/usr/include)
    set(CMAKE_SYSROOT ${ATK_TOOLCHAIN}/aarch64-buildroot-linux-gnu/sysroot)
    set(OPENCV_LIBS opencv_core opencv_flann opencv_videoio opencv_video opencv_highgui opencv_imgcodecs opencv_imgproc)
    include_directories(${SYSROOT})
    include_directories(${SYSROOT}/opencv4)
    set(RKNN_RT_LIB ${CMAKE_SOURCE_DIR}/librknn_api/aarch64/librknnrt.so)
else()
    message(STATUS "Cross toolchain not found -> native build (target board)")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(OPENCV REQUIRED opencv4)
    set(OPENCV_LIBS ${OPENCV_LIBRARIES})
    include_directories(${OPENCV_INCLUDE_DIRS})
    set(RKNN_RT_LIB ${CMAKE_SOURCE_DIR}/librknn_api/aarch64/librknnrt.so)
endif()

set(CMAKE_CXX_STANDARD 11)
add_definitions(-g -O0 -ggdb -gdwarf -funwind-tables -rdynamic)
add_definitions(-Wno-write-strings -Wno-return-type)

# rknn api
set(RKNN_API_PATH ${CMAKE_SOURCE_DIR}/librknn_api)
set(RKNN_RT_LIB ${RKNN_API_PATH}/aarch64/librknnrt.so)
include_directories(${RKNN_API_PATH}/include)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install/atk_mobilenet_v1_demo)

add_executable(atk_mobilenet_v1 atk_mobilenet_v1.cpp)
target_link_libraries(atk_mobilenet_v1 ${OPENCV_LIBS} ${RKNN_RT_LIB})
install(TARGETS atk_mobilenet_v1 DESTINATION ./)
install(DIRECTORY model DESTINATION ./)
