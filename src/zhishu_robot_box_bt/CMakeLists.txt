cmake_minimum_required(VERSION 3.8)

project(zhishu_robot_box_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp CONFIG REQUIRED)

add_executable(
  box_transport_bt
  src/box_transport_bt.cpp
)

# ament_target_dependencies() internally uses the plain
# target_link_libraries() signature on ROS 2 Jazzy.
ament_target_dependencies(
  box_transport_bt
  ament_index_cpp
)

# Keep this call plain too. Do not add PRIVATE/PUBLIC/INTERFACE here.
target_link_libraries(
  box_transport_bt
  behaviortree_cpp::behaviortree_cpp
)

install(
  TARGETS box_transport_bt
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
