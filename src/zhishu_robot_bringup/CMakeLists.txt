cmake_minimum_required(VERSION 3.8)
project(zhishu_robot_bringup)

find_package(ament_cmake REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(
  DIRECTORY scripts launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/rviz_with_fixes.sh
    scripts/reset_left_arm_fault.sh
    scripts/reset_right_arm_fault.sh
    scripts/reset_dual_arms_fault.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
