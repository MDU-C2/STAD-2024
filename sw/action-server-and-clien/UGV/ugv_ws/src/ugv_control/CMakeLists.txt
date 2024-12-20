cmake_minimum_required(VERSION 3.5)
project(ugv_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(uav_control_interfaces REQUIRED)

add_executable(flight_action_client src/flight_action_client.cpp)
ament_target_dependencies(flight_action_client  rclcpp  rclcpp_action  uav_control_interfaces
)

install(TARGETS
  flight_action_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()

