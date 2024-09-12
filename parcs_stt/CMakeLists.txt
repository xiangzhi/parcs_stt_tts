cmake_minimum_required(VERSION 3.5)
project(parcs_stt)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces from your action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Listen.action"
  DEPENDENCIES builtin_interfaces action_msgs
)

# Export package
ament_package()
