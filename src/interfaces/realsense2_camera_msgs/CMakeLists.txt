cmake_minimum_required(VERSION 3.16)
project(realsense2_camera_msgs)

# The official 4.58.3 wire contracts consumed by stream validation and camera
# identity certification. Driver binaries remain in the isolated cameras env.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Metadata.msg"
  "srv/DeviceInfo.srv"
  DEPENDENCIES std_msgs
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
