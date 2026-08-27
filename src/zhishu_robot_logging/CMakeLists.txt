cmake_minimum_required(VERSION 3.8)
project(zhishu_robot_logging)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_executable(zhishu_robot_log_recorder
  src/log_recorder_node.cpp
)
target_compile_features(zhishu_robot_log_recorder PUBLIC cxx_std_17)
ament_target_dependencies(zhishu_robot_log_recorder
  rclcpp
  rcl_interfaces
)

install(TARGETS zhishu_robot_log_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/kernel_ethercat_logger
  scripts/setup_kernel_ethercat_sudo.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
