cmake_minimum_required(VERSION 3.8)
project(zhishu_robot_sdk_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(zhishu_robot_interfaces REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_core REQUIRED)

set(CMAKE_WARN_DEPRECATED OFF)
find_package(moveit_ros_planning_interface REQUIRED)
unset(CMAKE_WARN_DEPRECATED)

add_library(zhishu_robot_sdk SHARED
  src/commander.cpp
  src/coordinate_frames.cpp
  src/pose_math.cpp
  src/motion_handle.cpp
  src/result.cpp
  src/robot_sdk.cpp
)
target_include_directories(zhishu_robot_sdk
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(zhishu_robot_sdk
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2 tf2_ros moveit_core
)
target_link_libraries(zhishu_robot_sdk Threads::Threads Eigen3::Eigen yaml-cpp)
target_compile_definitions(zhishu_robot_sdk PRIVATE SILENCE_DEPRECATION_WARNINGS)

add_executable(commander_node src/commander_node.cpp)
target_link_libraries(commander_node zhishu_robot_sdk)
ament_target_dependencies(commander_node
  rclcpp
  geometry_msgs
  std_msgs
  zhishu_robot_interfaces
)

add_executable(sdk_example src/sdk_example.cpp)
target_link_libraries(sdk_example zhishu_robot_sdk)
ament_target_dependencies(sdk_example rclcpp)

add_executable(sdk_torso_head_example src/sdk_torso_head_example.cpp)
target_link_libraries(sdk_torso_head_example zhishu_robot_sdk)
ament_target_dependencies(sdk_torso_head_example rclcpp)

add_executable(sdk_named_pose_loop src/sdk_named_pose_loop.cpp)
target_link_libraries(sdk_named_pose_loop zhishu_robot_sdk)
ament_target_dependencies(sdk_named_pose_loop rclcpp std_srvs)
target_sources(sdk_named_pose_loop PRIVATE src/stability_pdo_recording.cpp)

add_executable(sdk_joint_ethercat_loop src/sdk_joint_loop.cpp)
target_link_libraries(sdk_joint_ethercat_loop zhishu_robot_sdk)
ament_target_dependencies(sdk_joint_ethercat_loop rclcpp std_srvs)
target_sources(sdk_joint_ethercat_loop PRIVATE src/stability_pdo_recording.cpp)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS zhishu_robot_sdk
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  TARGETS commander_node sdk_example sdk_torso_head_example sdk_named_pose_loop sdk_joint_ethercat_loop
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md CHANGELOG.rst SDK_FRAMES.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coordinate_frames test/test_coordinate_frames.cpp)
  if(TARGET test_coordinate_frames)
    target_link_libraries(test_coordinate_frames zhishu_robot_sdk Eigen3::Eigen)
  endif()
  ament_add_gtest(test_pose_math test/test_pose_math.cpp)
  if(TARGET test_pose_math)
    target_link_libraries(test_pose_math zhishu_robot_sdk Eigen3::Eigen)
  endif()

  # Compile-time contract for the relocated public umbrella header.
  ament_add_gtest(test_migration_public_api test/test_migration_public_api.cpp)
  if(TARGET test_migration_public_api)
    target_link_libraries(test_migration_public_api zhishu_robot_sdk)
  endif()

  ament_add_gtest(test_sdk_types test/test_sdk_types.cpp)
  if(TARGET test_sdk_types)
    target_link_libraries(test_sdk_types zhishu_robot_sdk)
    ament_target_dependencies(test_sdk_types zhishu_robot_interfaces)
  endif()

  ament_add_gtest(
    test_stability_pdo_recording
    test/test_stability_pdo_recording.cpp
    src/stability_pdo_recording.cpp
  )
  if(TARGET test_stability_pdo_recording)
    target_include_directories(test_stability_pdo_recording PRIVATE src)
    ament_target_dependencies(test_stability_pdo_recording rclcpp std_srvs)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(zhishu_robot_sdk)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  Eigen3 yaml-cpp tf2 tf2_ros moveit_core
)

ament_package()
