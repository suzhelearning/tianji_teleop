cmake_minimum_required(VERSION 3.16)
project(tianji_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeviceFeedback.msg"
  "msg/ExecutorState.msg"
  "msg/CollectionStatus.msg"
  "srv/RecordingCommand.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

# `tianji_runtime` holds the pure-Python device/resource contracts that both
# tianji_controller and wuji_controller import. It lives in this package
# because it must never import a business package (that would create a cycle)
# and must stay importable without rclpy.
ament_python_install_package(tianji_runtime)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
