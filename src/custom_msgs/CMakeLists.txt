cmake_minimum_required(VERSION 3.5)
project(custom_msgs)

# Find necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)  # Ensure this is included
find_package(std_msgs REQUIRED)

# Define the message files
set(MSG_FILES
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
)

# Generate the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs
)

# Export dependencies for other packages
ament_export_dependencies(rosidl_default_runtime)

# Package ament configuration
ament_package()
