cmake_minimum_required(VERSION 3.5)
project(uav_control_interfaces)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Flight.action"
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()

