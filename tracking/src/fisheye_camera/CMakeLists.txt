cmake_minimum_required(VERSION 3.14)
project(fisheye_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Embed library paths so the binary works when launched by ros2 launch
# (subprocess may not inherit LD_LIBRARY_PATH in pixi/robostack envs)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(fisheye_camera_node
  src/v4l2_camera.cpp
  src/fisheye_camera_node.cpp
)
target_include_directories(fisheye_camera_node PRIVATE include)
ament_target_dependencies(fisheye_camera_node rclcpp sensor_msgs std_msgs)

install(TARGETS fisheye_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
