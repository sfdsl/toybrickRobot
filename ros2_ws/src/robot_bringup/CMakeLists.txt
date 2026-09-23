cmake_minimum_required(VERSION 3.5)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

# 只有 launch / params / config（纯编排包，不需要编译任何东西）
install(DIRECTORY launch params config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
