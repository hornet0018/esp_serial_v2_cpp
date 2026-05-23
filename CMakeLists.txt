cmake_minimum_required(VERSION 3.8)
project(esp_serial_v2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(serial REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_executable(esp_serial_ros2 src/esp_serial_ros2.cpp)
ament_target_dependencies(esp_serial_ros2
  rclcpp
  std_msgs
  geometry_msgs
  nlohmann_json
  serial
  std_srvs
  diagnostic_updater
)

add_executable(odometry_publisher src/odometry_publisher.cpp)
ament_target_dependencies(odometry_publisher
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(TARGETS
  esp_serial_ros2
  odometry_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
