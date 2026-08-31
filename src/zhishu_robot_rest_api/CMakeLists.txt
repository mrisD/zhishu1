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

set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

# Keep these header-only dependencies in the source tree so clean and offline
# builds never depend on GitHub availability.
add_library(nlohmann_json_vendor INTERFACE)
target_include_directories(
  nlohmann_json_vendor SYSTEM INTERFACE
  "${THIRD_PARTY_DIR}/nlohmann_json/include"
)
add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json_vendor)

add_library(cpp_httplib_vendor INTERFACE)
target_include_directories(
  cpp_httplib_vendor SYSTEM INTERFACE
  "${THIRD_PARTY_DIR}/cpp_httplib/include"
)
add_library(httplib::httplib ALIAS cpp_httplib_vendor)

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
