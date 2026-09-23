cmake_minimum_required(VERSION 3.8)
project(zhishu_robot_handeye_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco imgcodecs imgproc)
find_package(moveit_calibration_plugins REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(zhishu_robot_commander_cpp REQUIRED)

add_executable(handeye_pose_sequence src/handeye_pose_sequence.cpp)
ament_target_dependencies(handeye_pose_sequence
  rclcpp
  zhishu_robot_commander_cpp
)

add_executable(handeye_auto_calibration src/handeye_auto_calibration.cpp)
ament_target_dependencies(handeye_auto_calibration
  cv_bridge
  moveit_calibration_plugins
  rclcpp
  sensor_msgs
  std_msgs
  tf2_eigen
  tf2_ros
  visualization_msgs
  zhishu_robot_commander_cpp
)
target_include_directories(handeye_auto_calibration PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(handeye_auto_calibration Eigen3::Eigen ${OpenCV_LIBRARIES})

install(TARGETS
  handeye_pose_sequence
  handeye_auto_calibration
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/head_lock_guard.py
  scripts/check_camera_target.py
  scripts/check_camera_info.py
  scripts/camera_info_relay.py
  scripts/calibrate_charuco_intrinsics.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config docs launch models rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
