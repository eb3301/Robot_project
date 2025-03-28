cmake_minimum_required(VERSION 3.8)
project(detect_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # ← Add this line!

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectObjects.srv"
  DEPENDENCIES std_msgs geometry_msgs  # ← Add geometry_msgs here too!
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

