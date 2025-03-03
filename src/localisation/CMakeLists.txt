find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(localisation
  "srv/EstimatePose.srv"
  DEPENDENCIES builtin_interfaces
)
ament_package()

