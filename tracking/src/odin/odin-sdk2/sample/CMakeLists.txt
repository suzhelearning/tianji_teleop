cmake_minimum_required(VERSION 3.10)
project(odin_lidar LANGUAGES CXX)

add_subdirectory(hotplug)
add_subdirectory(upgrade)
add_subdirectory(getCalibration)
add_subdirectory(sendCalibration)
