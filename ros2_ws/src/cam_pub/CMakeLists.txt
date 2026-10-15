cmake_minimum_required(VERSION 3.5)
project(cam_pub)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS videoio imgproc core)

add_executable(${PROJECT_NAME} src/cam_pub_node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs cv_bridge)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
