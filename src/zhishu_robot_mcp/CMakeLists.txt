cmake_minimum_required(VERSION 3.16)
project(zhishu_robot_mcp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(zhishu_robot_commander_cpp REQUIRED)

# Build the pinned SDK source stored with this package. Keeping the complete
# release source in-tree makes clean builds deterministic and independent of
# GitHub availability.
set(_cxxmcp_expected_version "1.2.8")
set(_cxxmcp_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/third_party/cxxmcp")
set(_cxxmcp_version_file "${_cxxmcp_source_dir}/VERSION")
if(NOT EXISTS "${_cxxmcp_version_file}")
  message(FATAL_ERROR "Vendored cxxmcp VERSION file is missing")
endif()
file(STRINGS "${_cxxmcp_version_file}" _cxxmcp_vendor_version)
if(NOT _cxxmcp_vendor_version STREQUAL _cxxmcp_expected_version)
  message(FATAL_ERROR
    "Vendored cxxmcp version '${_cxxmcp_vendor_version}' does not match "
    "'${_cxxmcp_expected_version}'")
endif()

set(CXXMCP_BUILD_SDK ON CACHE BOOL "" FORCE)
set(CXXMCP_BUILD_CLIENT OFF CACHE BOOL "" FORCE)
set(CXXMCP_BUILD_SERVER ON CACHE BOOL "" FORCE)
set(CXXMCP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CXXMCP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(CXXMCP_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(CXXMCP_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(CXXMCP_ENABLE_HTTP OFF CACHE BOOL "" FORCE)
set(CXXMCP_ENABLE_AUTH OFF CACHE BOOL "" FORCE)
set(CXXMCP_ENABLE_WEBSOCKET OFF CACHE BOOL "" FORCE)
set(CXXMCP_SDK_CXX_STANDARD 17 CACHE STRING "" FORCE)
add_subdirectory("${_cxxmcp_source_dir}" "${CMAKE_CURRENT_BINARY_DIR}/cxxmcp")

add_library(${PROJECT_NAME}_core STATIC
  src/control_policy.cpp
  src/log_access.cpp
  src/robot_bridge.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  controller_manager_msgs
  rclcpp
  sensor_msgs
  std_srvs
  zhishu_robot_commander_cpp
)
target_link_libraries(${PROJECT_NAME}_core cxxmcp::protocol)

add_executable(zhishu_robot_mcp_server src/mcp_server.cpp)
target_link_libraries(
  zhishu_robot_mcp_server
  ${PROJECT_NAME}_core
  cxxmcp::peer
)
ament_target_dependencies(zhishu_robot_mcp_server ament_index_cpp rclcpp)

install(
  TARGETS zhishu_robot_mcp_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_policy test/test_control_policy.cpp)
  target_link_libraries(test_control_policy ${PROJECT_NAME}_core)

  ament_add_gtest(test_log_access test/test_log_access.cpp)
  target_link_libraries(test_log_access ${PROJECT_NAME}_core)
endif()

ament_package()
