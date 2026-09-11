cmake_minimum_required(VERSION 3.8)
project(zhishu_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_pdo_diagnostics
    test/test_pdo_diagnostics.py
  )
endif()

# IgH EtherCAT master user-space development files
set(ECRT_INCLUDE_DIR "/usr/local/etherlab/include" CACHE PATH "Path containing ecrt.h")
set(ECRT_LIBRARY "/usr/local/etherlab/lib/libethercat.so" CACHE FILEPATH "Path to libethercat.so or libethercat.a")

if(NOT EXISTS "${ECRT_INCLUDE_DIR}/ecrt.h")
  find_path(ECRT_INCLUDE_DIR
    NAMES ecrt.h
    PATHS
      $ENV{ECRT_INCLUDE_DIR}
      $ENV{IGH_INCLUDE_DIR}
      /usr/local/etherlab/include
      /opt/etherlab/include
      /usr/local/include
      /usr/include
  )
endif()

if(NOT EXISTS "${ECRT_LIBRARY}")
  find_library(ECRT_LIBRARY
    NAMES ethercat libethercat.so libethercat.a
    PATHS
      $ENV{ECRT_LIBRARY_DIR}
      $ENV{IGH_LIBRARY_DIR}
      /usr/local/etherlab/lib
      /usr/local/etherlab/lib64
      /opt/etherlab/lib
      /opt/etherlab/lib64
      /usr/local/lib
      /usr/lib
      /usr/lib/x86_64-linux-gnu
  )
endif()

if(NOT ECRT_INCLUDE_DIR OR NOT EXISTS "${ECRT_INCLUDE_DIR}/ecrt.h" OR NOT ECRT_LIBRARY OR NOT EXISTS "${ECRT_LIBRARY}")
  message(STATUS "ECRT_INCLUDE_DIR='${ECRT_INCLUDE_DIR}'")
  message(STATUS "ECRT_LIBRARY='${ECRT_LIBRARY}'")
  message(FATAL_ERROR
    "IgH EtherCAT master development files not found. Need ecrt.h and libethercat.so/libethercat.a.\n"
    "Build with:\n"
    "  colcon build --packages-select zhishu_robot_hardware --cmake-args "
    "-DECRT_INCLUDE_DIR=/usr/local/etherlab/include "
    "-DECRT_LIBRARY=/usr/local/etherlab/lib/libethercat.so\n"
  )
endif()

message(STATUS "Found IgH ecrt.h: ${ECRT_INCLUDE_DIR}/ecrt.h")
message(STATUS "Found IgH library: ${ECRT_LIBRARY}")

# Main ros2_control hardware plugin library
add_library(${PROJECT_NAME} SHARED
  src/es_motor_hardware_interface.cpp
  src/left_es_motor_hardware_interface.cpp
  src/ethercat_driver.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${ECRT_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_srvs
)

target_link_libraries(${PROJECT_NAME}
  ${ECRT_LIBRARY}
)

# Important:
# libzhishu_robot_hardware.so itself depends on libethercat.so.1,
# so the shared library also needs RPATH.
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "/usr/local/etherlab/lib;/opt/etherlab/lib"
  INSTALL_RPATH "/usr/local/etherlab/lib;/opt/etherlab/lib"
)

pluginlib_export_plugin_description_file(
  hardware_interface
  zhishu_robot_hardware.xml
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/record_pdo_diagnostics.py
    scripts/analyze_pdo_diagnostics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_srvs
)

ament_package()
