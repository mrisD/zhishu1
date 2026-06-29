cmake_minimum_required(VERSION 3.8)
project(zhishu_robot_bring)

find_package(ament_cmake REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
