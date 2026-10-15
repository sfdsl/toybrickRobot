cmake_minimum_required(VERSION 3.10)
project(rknn_infer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ── yolo11 demo 源码目录（只引用不复制，单一来源） ─────────────────
set(DEMO_DIR "/home/toybrick/RobotCode/01_Project/04_Vision/rk3588-atk-yolov11-2026/yolo11/cpp"
    CACHE PATH "yolo11 demo 源码目录")
if(NOT EXISTS "${DEMO_DIR}/postprocess.cc")
  message(FATAL_ERROR "DEMO_DIR 不存在或缺 postprocess.cc: ${DEMO_DIR}")
endif()

# 3rdparty/ 与 utils/ 需要先于编译引用（LIBRKNNRT、LIBJPEG、LIBRGA、STB_INCLUDES 等变量）
set(TARGET_SOC rk3588)   # 3rdparty/CMakeLists.txt 据此选 librknnrt（2.3.2）
add_subdirectory(${DEMO_DIR}/3rdparty ${CMAKE_CURRENT_BINARY_DIR}/3rdparty.out)
add_subdirectory(${DEMO_DIR}/utils    ${CMAKE_CURRENT_BINARY_DIR}/utils.out)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME}
  src/rknn_infer_node.cpp
  ${DEMO_DIR}/postprocess.cc
  ${DEMO_DIR}/rknpu2/yolo11.cc
)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs std_msgs cv_bridge)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${DEMO_DIR}
  ${LIBRKNNRT_INCLUDES}
)
target_link_libraries(${PROJECT_NAME}
  imageutils fileutils imagedrawing   # demo 工具库（letterbox 走 RGA 加速）
  ${LIBRKNNRT}                        # librknnrt.so 2.3.2（自带，优先于系统 1.6.0）
  ${OpenCV_LIBS}
  Threads::Threads
)
# 关键：imageutils 内嵌的 RK 版 libturbojpeg.a 带 91 个全局 jpeg_* 符号，
# 会把 OpenCV imgcodecs 对系统 libjpeg.so.8 的调用劫持到旧版静态实现（ABI 不符 → 段错误）。
# --exclude-libs 把静态库符号局部化，imencode 正常走系统 libjpeg。
target_link_options(${PROJECT_NAME} PRIVATE "LINKER:--exclude-libs,ALL")

# 运行时工作目录（postprocess.cc 以相对路径 ./model/coco_80_labels_list.txt 读标签）
target_compile_definitions(${PROJECT_NAME} PRIVATE
  RKNN_SHARE_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

# 加载顺序：install/lib/librknnrt.so(2.3.2) 优先于系统 /lib/librknnrt.so(1.6.0)
# 二进制位于 <prefix>/lib/rknn_infer/，$ORIGIN/../../lib = <prefix>/lib
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../lib"
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(FILES ${DEMO_DIR}/../model/coco_80_labels_list.txt
        DESTINATION share/${PROJECT_NAME}/model)
install(FILES ${DEMO_DIR}/../model/yolo11.rknn
        DESTINATION share/${PROJECT_NAME}/model)

ament_package()
