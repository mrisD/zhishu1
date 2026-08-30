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

# Prefer a pre-installed cxxmcp package.  When it is not installed, consume the
# deterministic v1.2.8 SDK source archive and verify its published SHA-256.
find_package(cxxmcp 1.2.8 EXACT CONFIG QUIET)
if(NOT TARGET cxxmcp::server)
  include(FetchContent)
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
  FetchContent_Declare(
    cxxmcp
    URL
      https://github.com/caomengxuan666/cxxmcp/releases/download/v1.2.8/cxxmcp-sdk-source-v1.2.8.tar.gz
    URL_HASH
      SHA256=ac4be3cd7ba6938960ff48cf25f7ea3293909df60bf7d79bb44d464b69499c29
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  # The upstream v1.2.8 SDK archive (including the published SHA-256 above)
  # omits the VERSION file that its top-level CMakeLists.txt requires. Populate
  # first and restore only that deterministic release metadata before adding
  # the dependency. Fail if a future archive contains a different version.
  FetchContent_GetProperties(cxxmcp)
  if(NOT cxxmcp_POPULATED)
    FetchContent_Populate(cxxmcp)
  endif()
  set(_cxxmcp_expected_version "1.2.8")
  set(_cxxmcp_version_file "${cxxmcp_SOURCE_DIR}/VERSION")
  if(EXISTS "${_cxxmcp_version_file}")
    file(STRINGS "${_cxxmcp_version_file}" _cxxmcp_archive_version)
    if(NOT _cxxmcp_archive_version STREQUAL _cxxmcp_expected_version)
      message(FATAL_ERROR
        "cxxmcp archive version '${_cxxmcp_archive_version}' does not match "
        "'${_cxxmcp_expected_version}'")
    endif()
  else()
    file(WRITE "${_cxxmcp_version_file}" "${_cxxmcp_expected_version}\n")
  endif()
  add_subdirectory("${cxxmcp_SOURCE_DIR}" "${cxxmcp_BINARY_DIR}")
endif()

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
