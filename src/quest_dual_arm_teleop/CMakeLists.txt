cmake_minimum_required(VERSION 3.8)
project(quest_dual_arm_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ROS 2 Jazzy's MoveIt dependency chain currently exports the deprecated
# tl_expected compatibility package. Hide that dependency-only CMake notice;
# our targets also opt into the supported header-level suppression below.
set(CMAKE_WARN_DEPRECATED OFF)
find_package(moveit_ros_planning_interface REQUIRED)
unset(CMAKE_WARN_DEPRECATED)

add_executable(quest_dual_arm_teleop_node src/quest_dual_arm_teleop_node.cpp)
ament_target_dependencies(
  quest_dual_arm_teleop_node
  rclcpp
  geometry_msgs
  std_msgs
  trajectory_msgs
)
target_link_libraries(quest_dual_arm_teleop_node Eigen3::Eigen)

add_executable(move_to_servo_ready src/move_to_servo_ready.cpp)
ament_target_dependencies(
  move_to_servo_ready
  rclcpp
  moveit_ros_planning_interface
)
target_compile_definitions(move_to_servo_ready PRIVATE SILENCE_DEPRECATION_WARNINGS)

install(TARGETS
  quest_dual_arm_teleop_node
  move_to_servo_ready
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/servo_smoke_test.py
  scripts/servo_joint_smoke_test.py
  scripts/mock_servo_integration_test.py
  scripts/quest_link_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  unity
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_pytest_test(
    test_jazzy_contracts
    test/test_jazzy_contracts.py
  )
  add_launch_test(
    test/test_mock_servo_launch.py
    TIMEOUT 90
  )
endif()

ament_package()
