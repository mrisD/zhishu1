cmake_minimum_required(VERSION 3.8)
project(zhishu_robot_handeye_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(zhishu_robot_commander_cpp REQUIRED)

add_executable(handeye_pose_sequence src/handeye_pose_sequence.cpp)
ament_target_dependencies(handeye_pose_sequence
  rclcpp
  zhishu_robot_commander_cpp
)

install(TARGETS handeye_pose_sequence DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/head_lock_guard.py
  scripts/check_camera_target.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch models rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
