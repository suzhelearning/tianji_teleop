cmake_minimum_required(VERSION 3.16)
project(tianji_interfaces)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# This package now ships only the pure-Python resource-location contracts that
# the PICO bare-hand -> SPD route shares. The hardware/collection ROS messages
# and services it used to generate are gone with those routes; the SPD joint
# command lives in `tianji_spd_interfaces`.
#
# `tianji_runtime` must stay importable without rclpy, and must never import a
# business package (that would create a cycle), so it is installed here rather
# than from the packages that consume it. Its checks are plain pytest files
# run from the workspace, not a colcon test target.
ament_python_install_package(tianji_runtime)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
