cmake_minimum_required(VERSION 3.8)
project(zhishu_robot_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

set(CMAKE_WARN_DEPRECATED OFF)
find_package(moveit_ros_planning_interface REQUIRED)
unset(CMAKE_WARN_DEPRECATED)

add_library(${PROJECT_NAME} SHARED
  src/commander.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
)
target_compile_definitions(${PROJECT_NAME} PRIVATE SILENCE_DEPRECATION_WARNINGS)

add_executable(commander_node src/commander_node.cpp)
target_link_libraries(commander_node ${PROJECT_NAME})
ament_target_dependencies(commander_node
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(sdk_example src/sdk_example.cpp)
target_link_libraries(sdk_example ${PROJECT_NAME})
ament_target_dependencies(sdk_example rclcpp)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  TARGETS commander_node sdk_example
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

ament_package()
