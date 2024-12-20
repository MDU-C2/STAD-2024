cmake_minimum_required(VERSION 3.5)
project(image_publisher_subscriber)

# Hitta nödvändiga ROS2-paket för C++-noden
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Bygg C++-publishernoden
add_executable(pub_node src/pub_node.cpp)
ament_target_dependencies(pub_node rclcpp sensor_msgs cv_bridge OpenCV)

# Installera C++-noden
install(TARGETS
  pub_node
  DESTINATION lib/${PROJECT_NAME})



ament_package()

