cmake_minimum_required(VERSION 3.8)
project(zhishu_robot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
