cmake_minimum_required(VERSION 3.5)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

# 只有 launch 文件（纯编排包，不需要编译任何东西）
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
