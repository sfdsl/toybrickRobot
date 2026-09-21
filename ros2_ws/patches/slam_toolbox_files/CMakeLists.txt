cmake_minimum_required(VERSION 3.5)
project(slam_toolbox)

set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
cmake_policy(SET CMP0077 NEW)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMake/")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/karto_sdk/cmake)

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(interactive_markers REQUIRED)

# ---- 可选依赖（RK3588/Toybrick 上的 ROS2 Foxy 为源码安装，以下包默认不具备）----
# 1) nav2_map_server：本仓库只在 src/map_saver.cpp 里用 system("ros2 run nav2_map_server
#    map_saver_cli ...") 做运行时调用，编译期没有任何头文件/库依赖，故不强制查找。
#    （本机保存地图用自带的 map_saver.py，见 04_SLAM建图实验方案.md S6）
# 2) rviz_* + Qt5：仅 RViz 插件 SlamToolboxPlugin 需要。本机 rviz2 因 Mali GPU 无 GLX
#    无法运行，且未安装 Qt5 开发包，故改为可选：找到才编译插件，否则跳过。
#    核心 SLAM 节点（async/sync/localization/lifelong + merge_maps_kinematic）不受影响。
find_package(nav2_map_server QUIET)
find_package(Qt5 QUIET COMPONENTS Core Gui Widgets Test Concurrent)

set(SLAM_TOOLBOX_BUILD_RVIZ_PLUGIN OFF)
if(Qt5_FOUND)
  find_package(rviz_common QUIET)
  find_package(rviz_default_plugins QUIET)
  find_package(rviz_ogre_vendor QUIET)
  find_package(rviz_rendering QUIET)
  if(rviz_common_FOUND AND rviz_default_plugins_FOUND AND rviz_ogre_vendor_FOUND AND rviz_rendering_FOUND)
    set(SLAM_TOOLBOX_BUILD_RVIZ_PLUGIN ON)
  endif()
endif()
if(SLAM_TOOLBOX_BUILD_RVIZ_PLUGIN)
  message(STATUS "slam_toolbox: RViz 插件 启用 (Qt5 + rviz 均已找到)")
else()
  message(STATUS "slam_toolbox: RViz 插件 跳过 (未找到 Qt5/rviz) - 核心 SLAM 节点不受影响")
endif()

#karto_sdk lib
set(BUILD_SHARED_LIBS ON)
add_subdirectory(lib/karto_sdk)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 注意：nav2_map_server / rviz_* / Qt5 不能放进本列表——它们在 ament_target_dependencies()
# 中一旦缺失会直接 FATAL_ERROR（见 CMake 报错 "package name xxx was not found before"），
# 而它们只在可选场景下用到，故统一在各自的可选块里单独指定。
set(dependencies
  rclcpp
  message_filters
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
  pluginlib
  tf2_geometry_msgs
  tf2_sensor_msgs
  std_msgs
  std_srvs
  builtin_interfaces
  interactive_markers
)

set(libraries
    toolbox_common
    ceres_solver_plugin
    async_slam_toolbox
    sync_slam_toolbox
    localization_slam_toolbox
    lifelong_slam_toolbox
)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CSparse REQUIRED)
find_package(G2O QUIET)  # upstream uses REQUIRED, but g2o is unused in this repo
find_package(Cholmod REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
find_package(Boost REQUIRED system serialization filesystem thread)

include_directories(include ${EIGEN3_INCLUDE_DIRS}
                            ${CHOLMOD_INCLUDE_DIR}
                            ${Boost_INCLUDE_DIRS}
                            ${TBB_INCLUDE_DIRS}
                            ${CERES_INCLUDES}
)

add_definitions(${EIGEN3_DEFINITIONS})

rosidl_generate_interfaces(${PROJECT_NAME}
  srvs/Pause.srv
  srvs/ClearQueue.srv
  srvs/ToggleInteractive.srv
  srvs/Clear.srv
  srvs/SaveMap.srv
  srvs/LoopClosure.srv
  srvs/MergeMaps.srv
  srvs/AddSubmap.srv
  srvs/DeserializePoseGraph.srv
  srvs/SerializePoseGraph.srv
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs visualization_msgs
)

#### rviz Plugin（可选：需 Qt5 + rviz_*，缺失则整块跳过）
if(SLAM_TOOLBOX_BUILD_RVIZ_PLUGIN)
list(APPEND libraries SlamToolboxPlugin)
qt5_wrap_cpp(MOC_FILES rviz_plugin/slam_toolbox_rviz_plugin.hpp)
add_library(SlamToolboxPlugin SHARED
  rviz_plugin/slam_toolbox_rviz_plugin.cpp
  ${MOC_FILES})
ament_target_dependencies(SlamToolboxPlugin
 ${dependencies}
 rviz_common
 rviz_default_plugins
 rviz_ogre_vendor
 rviz_rendering
 Qt5
)
target_include_directories(SlamToolboxPlugin PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)
target_link_libraries(SlamToolboxPlugin ${QT_LIBRARIES} rviz_common::rviz_common)
rosidl_target_interfaces(SlamToolboxPlugin ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_definitions(SlamToolboxPlugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(SlamToolboxPlugin PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml) 
endif()

#### Ceres Plugin
add_library(ceres_solver_plugin solvers/ceres_solver.cpp)
ament_target_dependencies(ceres_solver_plugin ${dependencies})
target_link_libraries(ceres_solver_plugin ${CERES_LIBRARIES}
                                          ${Boost_LIBRARIES}
                                          ${TBB_LIBRARIES}
                                          kartoSlamToolbox
)
rosidl_target_interfaces(ceres_solver_plugin ${PROJECT_NAME} "rosidl_typesupport_cpp")
pluginlib_export_plugin_description_file(slam_toolbox solver_plugins.xml)

#### Tool lib for mapping
add_library(toolbox_common src/slam_toolbox_common.cpp src/map_saver.cpp src/loop_closure_assistant.cpp src/laser_utils.cpp src/slam_mapper.cpp)
ament_target_dependencies(toolbox_common
  ${dependencies}
)
target_link_libraries(toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
rosidl_target_interfaces(toolbox_common ${PROJECT_NAME} "rosidl_typesupport_cpp")

#### Mapping executibles
add_library(async_slam_toolbox src/slam_toolbox_async.cpp)
target_link_libraries(async_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(async_slam_toolbox_node src/slam_toolbox_async_node.cpp)
target_link_libraries(async_slam_toolbox_node async_slam_toolbox)

add_library(sync_slam_toolbox src/slam_toolbox_sync.cpp)
target_link_libraries(sync_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(sync_slam_toolbox_node src/slam_toolbox_sync_node.cpp)
target_link_libraries(sync_slam_toolbox_node sync_slam_toolbox)

add_library(localization_slam_toolbox src/slam_toolbox_localization.cpp)
target_link_libraries(localization_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(localization_slam_toolbox_node src/slam_toolbox_localization_node.cpp)
target_link_libraries(localization_slam_toolbox_node localization_slam_toolbox)

add_library(lifelong_slam_toolbox src/experimental/slam_toolbox_lifelong.cpp)
target_link_libraries(lifelong_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(lifelong_slam_toolbox_node src/experimental/slam_toolbox_lifelong_node.cpp)
target_link_libraries(lifelong_slam_toolbox_node lifelong_slam_toolbox)

#### Merging maps tool
add_executable(merge_maps_kinematic src/merge_maps_kinematic.cpp)
target_link_libraries(merge_maps_kinematic kartoSlamToolbox toolbox_common)

#### testing
#if(BUILD_TESTING)
#  find_package(ament_cmake_gtest REQUIRED)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#  include_directories(test)
#  ament_add_gtest(lifelong_metrics_test test/lifelong_metrics_test.cpp)
#  target_link_libraries(lifelong_metrics_test lifelong_slam_toolbox)
#endif()

#### Install
install(TARGETS async_slam_toolbox_node
                sync_slam_toolbox_node
                localization_slam_toolbox_node
                lifelong_slam_toolbox_node
                merge_maps_kinematic
                ${libraries}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(SLAM_TOOLBOX_BUILD_RVIZ_PLUGIN)
install(TARGETS SlamToolboxPlugin
  EXPORT SlamToolboxPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES solver_plugins.xml rviz_plugins.xml
  DESTINATION share
)

ament_export_include_directories(include)
ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies})
if(SLAM_TOOLBOX_BUILD_RVIZ_PLUGIN)
ament_export_targets(SlamToolboxPlugin HAS_LIBRARY_TARGET)
endif()
ament_package()
