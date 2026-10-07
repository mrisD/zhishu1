cmake_minimum_required(VERSION 3.8)
project(zhishu_robot_mtc_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

# ROS 2 Jazzy's MoveIt dependency chain currently exports the deprecated
# tl_expected compatibility package. Suppress only the dependency's CMake
# deprecation notice while resolving MoveIt/MTC packages.
set(CMAKE_WARN_DEPRECATED OFF)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
unset(CMAKE_WARN_DEPRECATED)

add_executable(alternative_path_costs src/alternative_path_costs.cpp)
target_include_directories(alternative_path_costs BEFORE PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/compat_include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(alternative_path_costs
  rclcpp
  rcpputils
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  moveit_msgs
  geometry_msgs
  shape_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  visualization_msgs
)

add_executable(mtc_task_node src/mtc_task_node.cpp)
target_include_directories(mtc_task_node BEFORE PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/compat_include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtc_task_node
  rclcpp
  rcpputils
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  moveit_msgs
  geometry_msgs
  shape_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  visualization_msgs
)
add_executable(dual_arm_crate_carry src/dual_arm_crate_carry.cpp)
target_include_directories(dual_arm_crate_carry BEFORE PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/compat_include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dual_arm_crate_carry
  rclcpp
  rcpputils
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  moveit_msgs
  geometry_msgs
  shape_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  visualization_msgs
)


add_executable(pallet_bin_15kg_transfer src/pallet_bin_15kg_transfer.cpp)
target_include_directories(pallet_bin_15kg_transfer BEFORE PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/compat_include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pallet_bin_15kg_transfer
  sensor_msgs
  rclcpp
  rcpputils
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  moveit_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  shape_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  visualization_msgs
)

add_executable(moving_bottle_grasp src/moving_bottle_grasp.cpp)
target_include_directories(moving_bottle_grasp BEFORE PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/compat_include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(moving_bottle_grasp
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  std_msgs
  std_srvs
  tf2
)

install(TARGETS alternative_path_costs mtc_task_node dual_arm_crate_carry pallet_bin_15kg_transfer moving_bottle_grasp
  DESTINATION lib/${PROJECT_NAME}
)
install (
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
