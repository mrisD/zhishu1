cmake_minimum_required(VERSION 3.16)
project(zhishu_robot_rest_api)

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

include(FetchContent)

find_package(nlohmann_json 3.12.0 EXACT CONFIG QUIET)
if(NOT TARGET nlohmann_json::nlohmann_json)
  set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
  set(JSON_Install OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/archive/refs/tags/v3.12.0.tar.gz
    URL_HASH SHA256=4b92eb0c06d10683f7447ce9406cb97cd4b453be18d7279320f7b2f025c10187
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  FetchContent_MakeAvailable(nlohmann_json)
endif()

find_package(httplib 0.54.0 EXACT CONFIG QUIET)
if(NOT TARGET httplib::httplib)
  set(HTTPLIB_COMPILE OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_TEST OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_INSTALL OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_USE_OPENSSL_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_USE_ZLIB_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_USE_BROTLI_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
  set(HTTPLIB_USE_ZSTD_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    httplib
    URL
      https://github.com/yhirose/cpp-httplib/archive/9185fdb6ae4f7aaa3a78a3ec7289c606c7c5d951.tar.gz
    URL_HASH
      SHA256=583ee4190ae32d7b114f8ad40f04c3dfbfd46cf73ab3fcaf5ab2be478f309aad
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  FetchContent_MakeAvailable(httplib)
endif()

add_library(${PROJECT_NAME}_core STATIC
  src/access_policy.cpp
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
target_link_libraries(${PROJECT_NAME}_core nlohmann_json::nlohmann_json)

add_library(${PROJECT_NAME}_http STATIC src/rest_server.cpp)
target_include_directories(${PROJECT_NAME}_http
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  ${PROJECT_NAME}_http
  ${PROJECT_NAME}_core
  httplib::httplib
)
ament_target_dependencies(${PROJECT_NAME}_http rclcpp)

add_executable(zhishu_robot_rest_api_server src/main.cpp)
target_link_libraries(
  zhishu_robot_rest_api_server
  ${PROJECT_NAME}_http
)
ament_target_dependencies(zhishu_robot_rest_api_server ament_index_cpp rclcpp)

install(
  TARGETS zhishu_robot_rest_api_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch openapi DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_access_policy test/test_access_policy.cpp)
  target_link_libraries(test_access_policy ${PROJECT_NAME}_core)

  ament_add_gtest(test_log_access test/test_log_access.cpp)
  target_link_libraries(test_log_access ${PROJECT_NAME}_core)
endif()

ament_package()
