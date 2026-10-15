cmake_minimum_required(VERSION 3.10)
message(STATUS "cmake version ${CMAKE_VERSION}")
project(rknn_yolo11_cam)

# 自动探测：存在正点原子 Buildroot 交叉工具链则交叉编译，
# 否则（如在 Toybrick 开发板上 Debian 11）使用本机原生编译器，避免 glibc 版本不匹配。
set(ATK_TOOLCHAIN /opt/atk-dlrk3588-toolchain)
if(EXISTS "${ATK_TOOLCHAIN}/bin/aarch64-buildroot-linux-gnu-g++")
    message(STATUS "Using ATK Buildroot cross toolchain")
    set(CMAKE_CXX_COMPILER ${ATK_TOOLCHAIN}/bin/aarch64-buildroot-linux-gnu-g++)
    set(CMAKE_C_COMPILER ${ATK_TOOLCHAIN}/bin/aarch64-buildroot-linux-gnu-gcc)
    set(SYSROOT ${ATK_TOOLCHAIN}/aarch64-buildroot-linux-gnu/sysroot/usr/include)
    set(CMAKE_SYSROOT ${ATK_TOOLCHAIN}/aarch64-buildroot-linux-gnu/sysroot)
    include_directories(${SYSROOT})
    include_directories(${SYSROOT}/opencv4)
    set(OPENCV_LIBS opencv_core opencv_flann opencv_videoio opencv_video opencv_highgui opencv_imgcodecs opencv_imgproc)
else()
    message(STATUS "Cross toolchain not found -> native build (target board)")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(OPENCV REQUIRED opencv4)
    set(OPENCV_LIBS ${OPENCV_LIBRARIES})
    include_directories(${OPENCV_INCLUDE_DIRS})
endif()

set(CMAKE_CXX_STANDARD 11)
add_definitions(-g -O2 -funwind-tables -rdynamic)
add_definitions(-Wno-write-strings -Wno-return-type)

set(rknpu_yolo11_file rknpu2/yolo11.cc)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ 3rdparty.out)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/utils/ utils.out)

set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

add_executable(${PROJECT_NAME}
    main.cc
    postprocess.cc
    ${rknpu_yolo11_file}
)

target_link_libraries(${PROJECT_NAME}
    imageutils
    fileutils
    imagedrawing
    ${OPENCV_LIBS}
    ${LIBRKNNRT}
    
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} Threads::Threads)


target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LIBRKNNRT_INCLUDES}
)

install(TARGETS ${PROJECT_NAME} DESTINATION .)
install(FILES ${LIBRKNNRT} DESTINATION lib)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../model/coco_80_labels_list.txt DESTINATION model)
file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../model/*.rknn")
install(FILES ${RKNN_FILES} DESTINATION model)
